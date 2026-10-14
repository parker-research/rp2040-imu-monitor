//! Static measurements: collect a window of healthy samples taken after the
//! antenna has settled, reject outliers, and (for azimuth) keep averaging
//! until the standard error is small enough.

use crate::config::{Limits, Thresholds, Timing};
use crate::error::{Fail, R};
use crate::imu::ImuStream;
use crate::math::{circ_mean_std_deg, mean_std, median, robust_mean_v3, wrap180, V3};
use crate::model::{AzEval, Model};
use crate::rotctl::Rotctl;
use crate::sample::Sample;
use log::{debug, warn};
use std::collections::BTreeMap;
use std::thread;
use std::time::Instant;

pub struct Ctx {
    pub imu: ImuStream,
    pub rot: Rotctl,
    pub th: Thresholds,
    pub tm: Timing,
    pub lim: Limits,
    /// What the controller reports at true zero (best current estimate).
    /// Maps reported positions to true ones for the [limits] check.
    pub zero_rep: (f64, f64),
}

impl Ctx {
    pub fn to_true(&self, reported: (f64, f64)) -> (f64, f64) {
        (reported.0 - self.zero_rep.0, reported.1 - self.zero_rep.1)
    }
}

#[derive(Clone, Debug)]
pub struct Window {
    pub acc: V3,
    pub acc_std: f64,
    pub mag: V3,
    pub mag_std: f64,
    pub mags: Vec<V3>,
    pub n: usize,
    pub temp: Option<f64>,
    pub fw_roll: Option<f64>,
    pub fw_pitch: Option<f64>,
    pub fw_heading: Option<f64>,
}

fn gather(ctx: &Ctx, n: usize) -> R<Vec<Sample>> {
    let to = ctx.tm.d(ctx.tm.sample_timeout_s);
    let mut after = Instant::now();
    let mut out: Vec<Sample> = Vec::with_capacity(n);
    let mut rejected = 0usize;
    let mut reasons: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut epoch = None;
    let mut restarts = 0;
    while out.len() < n {
        let Some(rx) = ctx.imu.next_after(after, to) else {
            return Err(Fail::Imu(format!(
                "no samples from {} for {:.1}s (stream stalled or unplugged)",
                ctx.imu.source,
                to.as_secs_f64()
            )));
        };
        after = rx.at;
        if let Some(r) = rx.s.reject_reason() {
            rejected += 1;
            *reasons.entry(r).or_default() += 1;
            if rejected > 2 * n + 8 {
                return Err(Fail::Imu(format!("too many unhealthy samples: {reasons:?}")));
            }
            continue;
        }
        let ep = rx.s.config_epoch();
        match epoch {
            None => epoch = Some(ep),
            Some(e) if e != ep => {
                warn!("IMU re-initialised mid-measurement (reinit/cfg_restore {e:?} -> {ep:?}); restarting window");
                out.clear();
                epoch = Some(ep);
                restarts += 1;
                if restarts > 3 {
                    return Err(Fail::Imu("IMU keeps re-initialising".into()));
                }
                continue;
            }
            _ => {}
        }
        out.push(rx.s);
    }
    if rejected > 0 {
        debug!("rejected {rejected} samples: {reasons:?}");
    }
    Ok(out)
}

fn summarize(s: &[Sample]) -> Window {
    let accs: Vec<V3> = s.iter().map(|x| x.acc()).collect();
    let mags: Vec<V3> = s.iter().map(|x| x.mag()).collect();
    let (acc, acc_std, _) = robust_mean_v3(&accs);
    let (mag, mag_std, _) = robust_mean_v3(&mags);
    let circ = |f: &dyn Fn(&Sample) -> Option<f64>| {
        let v: Vec<f64> = s.iter().filter_map(f).collect();
        (!v.is_empty()).then(|| circ_mean_std_deg(&v).0)
    };
    let lin = |f: &dyn Fn(&Sample) -> Option<f64>| {
        let v: Vec<f64> = s.iter().filter_map(f).collect();
        (!v.is_empty()).then(|| mean_std(&v).0)
    };
    Window {
        acc,
        acc_std,
        mag,
        mag_std,
        mags,
        n: s.len(),
        temp: lin(&|x| x.mcu_temp_c),
        fw_roll: circ(&|x| x.roll_deg),
        fw_pitch: lin(&|x| x.pitch_deg),
        fw_heading: circ(&|x| x.heading_deg),
    }
}

/// Wait `settle_s`, then collect `n` healthy, stationary samples.
pub fn window(ctx: &Ctx, n: usize, settle_s: f64) -> R<Window> {
    thread::sleep(ctx.tm.d(settle_s));
    let mut still_fail = 0;
    loop {
        let s = gather(ctx, n)?;
        let w = summarize(&s);
        if w.acc_std > ctx.th.acc_still_std_g {
            still_fail += 1;
            if still_fail >= 3 {
                return Err(Fail::Imu(format!(
                    "antenna not stationary: accel std {:.4} g > {:.4} g (wind? vibration? rotator still moving?)",
                    w.acc_std, ctx.th.acc_still_std_g
                )));
            }
            warn!("accel not steady (std {:.4} g); re-measuring", w.acc_std);
            thread::sleep(ctx.tm.d(ctx.tm.settle_acc_s));
            continue;
        }
        let g = w.acc.norm();
        if g < ctx.th.acc_mag_min_g || g > ctx.th.acc_mag_max_g {
            return Err(Fail::Imu(format!("accelerometer magnitude {g:.3} g is implausible")));
        }
        return Ok(w);
    }
}

#[derive(Clone, Debug)]
pub struct ElMeas {
    pub el: f64,
    pub off_axis: f64,
    pub w: Window,
}

pub fn measure_el(ctx: &Ctx, model: &Model) -> R<ElMeas> {
    let w = window(ctx, ctx.tm.samples_acc, ctx.tm.settle_acc_s)?;
    Ok(ElMeas { el: model.el(w.acc), off_axis: model.off_axis(w.acc), w })
}

#[derive(Clone, Debug)]
pub struct AzMeas {
    pub az: f64,
    /// Standard error of the mean azimuth.
    pub se: f64,
    /// Single-sample scatter.
    pub sd: f64,
    pub n: usize,
    pub ev: AzEval,
    pub el: f64,
    pub off_axis: f64,
    #[allow(dead_code)]
    pub w: Window,
}

/// Robust circular statistics of per-sample azimuths: (mean, sd, kept).
pub fn az_stats(model: &Model, acc: V3, mags: &[V3]) -> (f64, f64, usize) {
    let th: Vec<f64> = mags.iter().map(|m| model.az_eval(acc, *m).az).collect();
    let (m0, _) = circ_mean_std_deg(&th);
    let dev: Vec<f64> = th.iter().map(|t| wrap180(t - m0)).collect();
    let mad = median(&dev.iter().map(|d| d.abs()).collect::<Vec<_>>()) * 1.4826;
    let lim = (3.5 * mad).max(0.5);
    let kept: Vec<f64> = th.iter().zip(&dev).filter(|(_, d)| d.abs() <= lim).map(|(t, _)| *t).collect();
    let (mean, sd) = circ_mean_std_deg(&kept);
    (mean, sd, kept.len())
}

/// Azimuth with adaptive averaging and magnetic-consistency checks.
pub fn measure_az(ctx: &Ctx, model: &Model) -> R<AzMeas> {
    let mut retried = false;
    loop {
        let mut w = window(ctx, ctx.tm.samples_mag_min, ctx.tm.settle_mag_s)?;
        let (mut az, mut sd, mut kept) = az_stats(model, w.acc, &w.mags);
        let mut se = sd / (kept as f64).sqrt();
        while se > ctx.th.az_se_target_deg && w.n < ctx.tm.samples_mag_max {
            let extra = (ctx.tm.samples_mag_max - w.n).min(w.n);
            let w2 = window(ctx, extra, 0.0)?;
            let tot = (w.n + w2.n) as f64;
            let acc = (w.acc * w.n as f64 + w2.acc * w2.n as f64) / tot;
            let mut mags = w.mags.clone();
            mags.extend(w2.mags);
            let (mag, mag_std, _) = robust_mean_v3(&mags);
            w = Window { acc, mag, mag_std, n: mags.len(), mags, ..w };
            (az, sd, kept) = az_stats(model, w.acc, &w.mags);
            se = sd / (kept as f64).sqrt();
        }
        let ev = model.az_eval(w.acc, w.mag);
        let h_bad = (ev.h_ratio - 1.0).abs() > ctx.th.mag_h_rel_tol;
        let v_bad = ev.v_err.abs() > ctx.th.mag_v_abs_tol_ut;
        if h_bad || v_bad || se > ctx.th.az_se_max_deg {
            let why = format!(
                "horizontal field ratio {:.2} (tol ±{:.2}), vertical error {:+.1} uT (tol ±{:.1}), az std-error {:.2}° (max {:.2}°)",
                ev.h_ratio, ctx.th.mag_h_rel_tol, ev.v_err, ctx.th.mag_v_abs_tol_ut, se, ctx.th.az_se_max_deg
            );
            if !retried {
                warn!("magnetic reading inconsistent ({why}); waiting and re-measuring");
                retried = true;
                thread::sleep(ctx.tm.d(ctx.tm.settle_mag_s * 2.0));
                continue;
            }
            return Err(Fail::Magnetic(format!(
                "field does not match calibration: {why}. Something magnetic near the IMU, motor/brake current, \
                 or the IMU moved on its mount."
            )));
        }
        return Ok(AzMeas {
            az,
            se,
            sd,
            n: kept,
            ev,
            el: model.el(w.acc),
            off_axis: model.off_axis(w.acc),
            w,
        });
    }
}

/// Static "is the IMU still where calibration left it" check.
pub fn check_off_axis(ctx: &Ctx, off_axis: f64, az_tilt: f64) -> R<()> {
    let lim = ctx.th.off_axis_deg + az_tilt;
    if off_axis > lim {
        return Err(Fail::Glue(format!(
            "gravity is {off_axis:.2}° off the elevation-axis cone (limit {lim:.2}°): the IMU has tilted relative to \
             the elevation axis since calibration — it may have shifted on its mount"
        )));
    }
    Ok(())
}
