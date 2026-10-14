//! `calibrate-rotator-from-imu zero`: closed-loop drive to IMU-true az=0 / el=0, with link
//! ("glue") checks on every move.

use crate::config::Config;
use crate::error::{Fail, R};
use crate::math::wrap180;
use crate::measure::{check_off_axis, measure_az, measure_el, AzMeas, Ctx, ElMeas};
use crate::model::Model;
use crate::motion::{move_to, Axis, MoveOut, Watch};
use clap::ValueEnum;
use log::{info, warn};
use serde::Serialize;
use std::path::PathBuf;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Mode {
    /// Elevation only.
    #[value(alias = "el")]
    Elevation,
    /// Azimuth only.
    #[value(alias = "az")]
    Azimuth,
    /// Elevation first, then azimuth, then verify both.
    #[value(alias = "both", alias = "el-az")]
    ElevationThenAzimuth,
}

pub struct ZeroOpts {
    pub mode: Mode,
    pub dry_run: bool,
    pub allow_long_way: bool,
    pub report: Option<PathBuf>,
    pub on_zero_cmd: Option<String>,
    /// Command that makes the controller adopt the current position as 0/0.
    pub sync_cmd: Option<String>,
}

#[derive(Serialize, Debug, Clone, Default)]
pub struct SyncResult {
    pub command: String,
    pub performed: bool,
    pub skipped_reason: Option<String>,
    pub replies: Vec<String>,
    pub rotator_before: Option<(f64, f64)>,
    pub rotator_after: Option<(f64, f64)>,
    pub imu_el_after_deg: Option<f64>,
    pub imu_az_after_deg: Option<f64>,
}

#[derive(Serialize, Debug, Clone)]
pub struct Step {
    pub axis: &'static str,
    pub iter: u32,
    pub imu_before_deg: f64,
    pub rotator_before: (f64, f64),
    pub command: (f64, f64),
    pub rotator_after: (f64, f64),
    pub imu_after_deg: f64,
    pub imu_delta_deg: f64,
    pub expected_delta_deg: f64,
    pub az_std_err_deg: Option<f64>,
}

#[derive(Serialize, Debug, Default)]
pub struct Report {
    pub mode: Option<Mode>,
    pub result: String,
    pub started: String,
    pub finished: String,
    pub dry_run: bool,
    pub initial_el_deg: Option<f64>,
    pub initial_az_deg: Option<f64>,
    pub final_el_deg: Option<f64>,
    pub final_az_deg: Option<f64>,
    pub final_az_std_err_deg: Option<f64>,
    pub rotator_final: Option<(f64, f64)>,
    /// What the rotator reports when the antenna is at true zero.
    /// Subtract these from reported positions to get true positions.
    pub rotator_reports_at_true_zero_az: Option<f64>,
    pub rotator_reports_at_true_zero_el: Option<f64>,
    /// Offset found before the controller restart (what the rotator used to read at true zero).
    pub offset_before_sync_az: Option<f64>,
    pub offset_before_sync_el: Option<f64>,
    pub sync: Option<SyncResult>,
    pub steps: Vec<Step>,
    pub warnings: Vec<String>,
    pub error: Option<String>,
}

pub fn warn_rep(rep: &mut Report, s: String) {
    warn!("{s}");
    rep.warnings.push(s);
}

pub fn temperature_check(cfg: &Config, t: Option<f64>, rep: &mut Report) {
    if let (Some(t0), Some(t)) = (cfg.reference.mcu_temp_c, t) {
        if (t - t0).abs() > cfg.thresholds.temp_warn_delta_c {
            warn_rep(
                rep,
                format!("temperature {t:.1}°C vs {t0:.1}°C at calibration: magnetometer offsets may have drifted"),
            );
        }
    }
}

fn with_hint(e: Fail) -> Fail {
    match e {
        Fail::Rejected(m) => Fail::Rejected(format!(
            "{m}. Reaching true zero needs this reported position (the controller's frame is offset from true, \
             e.g. it boots reporting 0/0 wherever the antenna is). Widen [limits] in the config if those are refusing; \
             with [motion] method = \"hamlib\", rotctld's own min/max apply too (method = \"gcode\" bypasses them)"
        )),
        e => e,
    }
}

fn el_glue(ctx: &Ctx, d_rot: f64, d_imu: f64, gain: f64) -> R<()> {
    let exp = gain * d_rot;
    let tol = ctx.th.glue_el_abs_deg + ctx.th.glue_el_rel * d_rot.abs();
    if (d_imu - exp).abs() > tol {
        return Err(Fail::Glue(format!(
            "elevation: rotator reports {d_rot:+.2}° of motion but the IMU measured {d_imu:+.2}° \
             (expected {exp:+.2}° ± {tol:.2}°). The antenna/IMU is not moving with the rotator."
        )));
    }
    Ok(())
}

pub fn zero_el(ctx: &mut Ctx, cfg: &Config, model: &Model, mut cur: ElMeas, rep: &mut Report) -> R<ElMeas> {
    let gain = cfg.elevation.gain;
    let az_tilt = cfg.mount.az_axis_tilt_deg;
    for iter in 1..=ctx.th.max_iter {
        info!("[el {iter}] IMU elevation {:+.3}°", cur.el);
        if cur.el.abs() <= ctx.th.el_tol_deg {
            return Ok(cur);
        }
        let pos = ctx.rot.get_pos()?;
        ctx.zero_rep.1 = pos.1 - cur.el / gain;
        let target = (pos.0, pos.1 - cur.el / gain);
        let watch = Watch::Follow { axis: Axis::El, model, imu0: cur.el, gain };
        let mv: MoveOut = move_to(ctx, target, watch).map_err(with_hint)?;
        let new = measure_el(ctx, model)?;
        check_off_axis(ctx, new.off_axis, az_tilt)?;
        let after = ctx.rot.get_pos()?;
        let d_rot = after.1 - pos.1;
        let d_imu = new.el - cur.el;
        el_glue(ctx, d_rot, d_imu, gain)?;
        rep.steps.push(Step {
            axis: "el",
            iter,
            imu_before_deg: cur.el,
            rotator_before: pos,
            command: target,
            rotator_after: after,
            imu_after_deg: new.el,
            imu_delta_deg: d_imu,
            expected_delta_deg: gain * d_rot,
            az_std_err_deg: None,
        });
        if !mv.moved {
            if cur.el.abs() <= ctx.th.min_step_deg {
                warn_rep(rep, format!("elevation residual {:+.2}° is below the rotator's resolution", new.el));
                return Ok(new);
            }
            return Err(Fail::Rotator(format!("rotator did not move for a {:+.2}° elevation correction", -cur.el)));
        }
        cur = new;
    }
    if cur.el.abs() <= ctx.th.el_tol_deg {
        Ok(cur)
    } else {
        Err(Fail::Converge(format!("elevation still {:+.2}° after {} iterations", cur.el, ctx.th.max_iter)))
    }
}

/// Short-way target first; the same direction one turn away is offered only
/// with --allow-long-way, if it stays within the (true) azimuth limits.
fn az_targets(ctx: &Ctx, pos_az: f64, corr: f64, allow_long_way: bool) -> Vec<f64> {
    let t = pos_az + corr;
    let mut v = vec![t];
    if allow_long_way {
        let l = &ctx.lim;
        for alt in [t + 360.0, t - 360.0] {
            let ta = alt - ctx.zero_rep.0;
            if ta >= l.az_min - ctx.th.pos_tol_deg && ta <= l.az_max + ctx.th.pos_tol_deg {
                v.push(alt);
            }
        }
    }
    v
}

pub fn zero_az(ctx: &mut Ctx, cfg: &Config, model: &Model, mut cur: AzMeas, o: &ZeroOpts, rep: &mut Report) -> R<AzMeas> {
    let gain = cfg.azimuth.gain;
    let az_tilt = cfg.mount.az_axis_tilt_deg;
    for iter in 1..=ctx.th.max_iter {
        info!("[az {iter}] IMU azimuth {:+.2}° (±{:.2}, n={})", cur.az, cur.se, cur.n);
        if cur.az.abs() <= ctx.th.az_tol_deg {
            return Ok(cur);
        }
        let pos = ctx.rot.get_pos()?;
        ctx.zero_rep.0 = pos.0 - cur.az / gain;
        let corr = -cur.az / gain;
        let mut target = (pos.0 + corr, pos.1);
        let mut res = Err(Fail::Rejected("no azimuth target".into()));
        for (i, t) in az_targets(ctx, pos.0, corr, o.allow_long_way).into_iter().enumerate() {
            if i > 0 {
                warn_rep(rep, format!("azimuth {:.1}° was rejected; going the long way round to {t:.1}°", target.0));
            }
            target = (t, pos.1);
            let watch = Watch::Follow { axis: Axis::Az, model, imu0: cur.az, gain };
            res = move_to(ctx, target, watch);
            if !matches!(res, Err(Fail::Rejected(_))) {
                break;
            }
        }
        let mv = res.map_err(|e| match e {
            Fail::Rejected(m) if !o.allow_long_way => Fail::Rejected(format!(
                "{m}. The same direction one turn away may be reachable: re-run with --allow-long-way"
            )),
            e => with_hint(e),
        })?;
        let new = measure_az(ctx, model)?;
        check_off_axis(ctx, new.off_axis, az_tilt)?;
        let after = ctx.rot.get_pos()?;
        let d_rot = after.0 - pos.0;
        let d_imu = wrap180(new.az - cur.az);
        let exp = wrap180(gain * d_rot);
        let tol = ctx.th.glue_az_abs_deg + ctx.th.glue_az_rel * d_rot.abs() + 3.0 * cur.se.hypot(new.se);
        if wrap180(d_imu - exp).abs() > tol {
            return Err(Fail::Glue(format!(
                "azimuth: rotator reports {d_rot:+.2}° of motion but the magnetometer measured {d_imu:+.2}° \
                 (expected {exp:+.2}° ± {tol:.2}°). Mast slipping in the clamp, or IMU loose."
            )));
        }
        let d_el = new.el - cur.el;
        let el_lim = ctx.th.cross_el_deg + 2.0 * az_tilt;
        if d_el.abs() > el_lim {
            return Err(Fail::Glue(format!(
                "elevation changed by {d_el:+.2}° during an azimuth-only move (limit {el_lim:.2}°): antenna or IMU loose"
            )));
        }
        rep.steps.push(Step {
            axis: "az",
            iter,
            imu_before_deg: cur.az,
            rotator_before: pos,
            command: target,
            rotator_after: after,
            imu_after_deg: new.az,
            imu_delta_deg: d_imu,
            expected_delta_deg: exp,
            az_std_err_deg: Some(new.se),
        });
        if !mv.moved {
            if cur.az.abs() <= ctx.th.min_step_deg.max(ctx.th.az_tol_deg) {
                warn_rep(rep, format!("azimuth residual {:+.2}° is below the rotator's resolution", new.az));
                return Ok(new);
            }
            return Err(Fail::Rotator(format!("rotator did not move for a {:+.2}° azimuth correction", -cur.az)));
        }
        cur = new;
    }
    if cur.az.abs() <= ctx.th.az_tol_deg {
        Ok(cur)
    } else {
        Err(Fail::Converge(format!("azimuth still {:+.2}° after {} iterations", cur.az, ctx.th.max_iter)))
    }
}

pub fn run(ctx: &mut Ctx, cfg: &Config, o: &ZeroOpts, rep: &mut Report) -> R<()> {
    let model = Model::from_config(cfg);
    rep.mode = Some(o.mode);
    rep.dry_run = o.dry_run;
    ctx.imu.wait_alive(ctx.tm.d(5.0))?;
    let do_el = o.mode != Mode::Azimuth;
    let do_az = o.mode != Mode::Elevation;

    // ---- preflight: static checks before anything moves ----
    let el0 = measure_el(ctx, &model)?;
    temperature_check(cfg, el0.w.temp, rep);
    check_off_axis(ctx, el0.off_axis, cfg.mount.az_axis_tilt_deg)?;
    rep.initial_el_deg = Some(el0.el);
    // Elevation needs only gravity. Azimuth is most trustworthy near el=0,
    // so when elevation will be zeroed first, a failed magnetic reading here
    // is only a warning: azimuth is measured again once el≈0.
    let az0 = if !do_az {
        None
    } else {
        match measure_az(ctx, &model) {
            Ok(a) => Some(a),
            Err(Fail::Magnetic(m)) if do_el && el0.el.abs() > ctx.th.el_tol_deg => {
                warn_rep(rep, format!("azimuth not measured at el {:+.1}° ({m}); will measure it after elevation is zeroed", el0.el));
                None
            }
            Err(e) => return Err(e),
        }
    };
    rep.initial_az_deg = az0.as_ref().map(|a| a.az);
    let pos = ctx.rot.get_pos()?;
    ctx.zero_rep = (
        pos.0 - az0.as_ref().map_or(0.0, |a| a.az / cfg.azimuth.gain),
        pos.1 - el0.el / cfg.elevation.gain,
    );
    info!(
        "preflight: rotator az={:.2} el={:.2}; IMU el={:+.2}°{}",
        pos.0,
        pos.1,
        el0.el,
        az0.as_ref().map(|a| format!(" az={:+.2}° (±{:.2})", a.az, a.se)).unwrap_or_default()
    );
    if do_el && el0.el.abs() > ctx.lim.max_correction_el_deg {
        return Err(Fail::Glue(format!(
            "IMU says elevation is off by {:+.1}°, more than max_correction_el_deg={}: refusing to move \
             (a correction this large usually means the IMU or antenna has come loose)",
            el0.el, ctx.lim.max_correction_el_deg
        )));
    }
    if let Some(a) = &az0 {
        if a.az.abs() > ctx.lim.max_correction_az_deg {
            return Err(Fail::Glue(format!(
                "IMU says azimuth is off by {:+.1}°, more than max_correction_az_deg={}: refusing to move",
                a.az, ctx.lim.max_correction_az_deg
            )));
        }
    }
    if o.dry_run {
        rep.final_el_deg = Some(el0.el);
        rep.final_az_deg = rep.initial_az_deg;
        rep.rotator_final = Some(pos);
        fill_offsets(cfg, rep, pos, el0.el, rep.initial_az_deg);
        rep.result = "dry-run".into();
        return Ok(());
    }

    let mut el_m = el0;
    let mut az_m = az0;
    if do_el {
        el_m = zero_el(ctx, cfg, &model, el_m, rep)?;
    }
    if do_az {
        let start = match az_m.take() {
            Some(a) if !do_el => a,
            _ => measure_az(ctx, &model)?,
        };
        az_m = Some(zero_az(ctx, cfg, &model, start, o, rep)?);
        if do_el {
            // Re-verify: azimuth moves on a non-level rotator can disturb elevation.
            for round in 0..2 {
                let e = measure_el(ctx, &model)?;
                if e.el.abs() <= ctx.th.el_tol_deg {
                    el_m = e;
                    break;
                }
                info!("verify round {}: elevation drifted to {:+.2}°, re-zeroing", round + 1, e.el);
                el_m = zero_el(ctx, cfg, &model, e, rep)?;
                let a = measure_az(ctx, &model)?;
                az_m = Some(zero_az(ctx, cfg, &model, a, o, rep)?);
            }
        }
    }
    let fin_el = match (&az_m, do_el) {
        (Some(a), false) => a.el,
        _ => el_m.el,
    };
    let pos = ctx.rot.get_pos()?;
    rep.final_el_deg = Some(fin_el);
    rep.final_az_deg = az_m.as_ref().map(|a| a.az);
    rep.final_az_std_err_deg = az_m.as_ref().map(|a| a.se);
    rep.rotator_final = Some(pos);
    fill_offsets(cfg, rep, pos, fin_el, rep.final_az_deg);
    rep.offset_before_sync_az = rep.rotator_reports_at_true_zero_az;
    rep.offset_before_sync_el = rep.rotator_reports_at_true_zero_el;

    if let Some(cmd) = &o.sync_cmd {
        // A restart zeroes BOTH axes, so both must be at true zero first.
        let el_now = fin_el;
        let az_now = match &az_m {
            Some(a) => a.az,
            None => {
                info!("checking azimuth before restart (restart zeroes both axes)");
                measure_az(ctx, &model)?.az
            }
        };
        let el_ok = el_now.abs() <= ctx.th.el_tol_deg.max(ctx.th.min_step_deg);
        let az_ok = az_now.abs() <= ctx.th.az_tol_deg.max(ctx.th.min_step_deg);
        if el_ok && az_ok {
            sync(ctx, cfg, &model, cmd, el_now, az_now, rep)?;
        } else {
            let why = format!(
                "not sending `{cmd}`: it zeroes both axes but the antenna is at el {el_now:+.2}° az {az_now:+.2}° \
                 (run --mode elevation-then-azimuth)"
            );
            warn_rep(rep, why.clone());
            rep.sync = Some(SyncResult { command: cmd.clone(), skipped_reason: Some(why), ..Default::default() });
        }
    }
    rep.result = "ok".into();

    if let Some(cmd) = &o.on_zero_cmd {
        run_hook(cmd, rep)?;
    }
    Ok(())
}

/// Send the controller-restart command, wait for it to come back, and verify
/// it now reports 0/0 while the IMU confirms the antenna didn't move.
fn sync(ctx: &mut Ctx, cfg: &Config, model: &Model, cmd: &str, el0: f64, az0: f64, rep: &mut Report) -> R<()> {
    let before = ctx.rot.get_pos()?;
    let mut res = SyncResult { command: cmd.into(), rotator_before: Some(before), ..Default::default() };
    info!("sending `{cmd}` to rotctld: controller restarts and adopts the current position as az=el=0");
    res.replies = ctx.rot.send_raw(cmd, ctx.tm.d(3.0))?;
    std::thread::sleep(ctx.tm.d(cfg.sync.boot_wait_s));

    let deadline = std::time::Instant::now() + ctx.tm.d(cfg.sync.reconnect_timeout_s);
    let mut last: Option<(f64, f64)> = None;
    let mut stable = 0;
    let after = loop {
        match ctx.rot.get_pos() {
            Ok(p) => {
                if last.is_some_and(|l| (l.0 - p.0).abs() < 0.05 && (l.1 - p.1).abs() < 0.05) {
                    stable += 1;
                } else {
                    stable = 0;
                }
                last = Some(p);
                if stable >= 2 {
                    break p;
                }
            }
            Err(e) => log::debug!("waiting for controller: {e}"),
        }
        if std::time::Instant::now() > deadline {
            rep.sync = Some(res);
            return Err(Fail::Rotator(format!(
                "controller did not report a steady position within {:.0}s after `{cmd}`",
                cfg.sync.reconnect_timeout_s
            )));
        }
        std::thread::sleep(ctx.tm.d(1.0));
    };
    res.rotator_after = Some(after);
    res.performed = true;
    info!("after restart rotator reports az={:.2} el={:.2}", after.0, after.1);
    let tol = cfg.sync.expect_zero_tol_deg;
    if wrap180(after.0).abs() > tol || after.1.abs() > tol {
        rep.sync = Some(res);
        return Err(Fail::Rotator(format!(
            "after `{cmd}` the rotator reports az={:.2} el={:.2}, expected 0/0 ± {tol}: \
             the controller did not adopt the current position as zero",
            after.0, after.1
        )));
    }
    // The antenna must not have moved while the controller rebooted.
    let e = measure_el(ctx, model)?;
    let a = measure_az(ctx, model)?;
    res.imu_el_after_deg = Some(e.el);
    res.imu_az_after_deg = Some(a.az);
    rep.sync = Some(res);
    let (de, da) = (e.el - el0, wrap180(a.az - az0));
    let az_lim = ctx.th.glue_az_abs_deg + 3.0 * a.se;
    if de.abs() > ctx.th.glue_el_abs_deg || da.abs() > az_lim {
        return Err(Fail::Glue(format!(
            "antenna moved during the controller restart (el {de:+.2}°, az {da:+.2}°): \
             the new zero is not at true zero — re-run `calibrate-rotator-from-imu zero`"
        )));
    }
    rep.rotator_final = Some(after);
    rep.final_el_deg = Some(e.el);
    rep.final_az_deg = Some(a.az);
    rep.final_az_std_err_deg = Some(a.se);
    fill_offsets(cfg, rep, after, e.el, Some(a.az));
    Ok(())
}

pub fn fill_offsets(cfg: &Config, rep: &mut Report, pos: (f64, f64), el: f64, az: Option<f64>) {
    rep.rotator_reports_at_true_zero_el = Some(pos.1 - el / cfg.elevation.gain);
    rep.rotator_reports_at_true_zero_az = az.map(|a| pos.0 - a / cfg.azimuth.gain);
}

fn run_hook(cmd: &str, rep: &Report) -> R<()> {
    info!("running --on-zero-cmd");
    let f = |v: Option<f64>| v.map(|x| format!("{x:.3}")).unwrap_or_default();
    let st = std::process::Command::new("sh")
        .arg("-c")
        .arg(cmd)
        .env("ROTCAL_REPORTED_AZ", f(rep.rotator_final.map(|p| p.0)))
        .env("ROTCAL_REPORTED_EL", f(rep.rotator_final.map(|p| p.1)))
        .env("ROTCAL_RESIDUAL_AZ", f(rep.final_az_deg))
        .env("ROTCAL_RESIDUAL_EL", f(rep.final_el_deg))
        .env("ROTCAL_ZERO_AZ", f(rep.rotator_reports_at_true_zero_az))
        .env("ROTCAL_ZERO_EL", f(rep.rotator_reports_at_true_zero_el))
        .status()
        .map_err(|e| Fail::Config(format!("--on-zero-cmd failed to start: {e}")))?;
    if !st.success() {
        return Err(Fail::Config(format!("--on-zero-cmd exited with {st}")));
    }
    Ok(())
}
