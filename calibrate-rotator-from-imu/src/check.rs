//! `calibrate-rotator-from-imu check`: verify the calibration and the mechanical link without
//! zeroing. By default nothing moves; `--wiggle` adds small ±moves that
//! confirm the stored signs/gains still hold.

use crate::config::Config;
use crate::error::{Fail, R};
use crate::math::wrap180;
use crate::measure::{measure_az, measure_el, Ctx};
use crate::model::Model;
use crate::motion::{check_limits, move_to, Axis, Watch};
use crate::zero::{temperature_check, Report};
use serde::Serialize;

pub struct CheckOpts {
    pub wiggle: bool,
    pub step: f64,
    pub skip_az: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct Item {
    pub name: String,
    pub ok: bool,
    /// glue | magnetic | offset | imu | info
    pub kind: &'static str,
    pub detail: String,
}

#[derive(Serialize, Debug, Default)]
pub struct CheckReport {
    pub rotator: (f64, f64),
    pub imu_el_deg: f64,
    pub imu_az_deg: Option<f64>,
    pub imu_az_std_err_deg: Option<f64>,
    /// Rotator-reported minus IMU-true position (0 = rotator is calibrated).
    pub rotator_error_el_deg: f64,
    pub rotator_error_az_deg: Option<f64>,
    pub items: Vec<Item>,
    pub warnings: Vec<String>,
}

impl CheckReport {
    fn add(&mut self, name: &str, ok: bool, kind: &'static str, detail: String) {
        self.items.push(Item { name: name.into(), ok, kind, detail });
    }
    /// 0 all good, 5 link failure, 7 magnetic problem, 8 rotator not zeroed, 3 IMU health.
    pub fn exit_code(&self) -> i32 {
        for (kind, code) in [("glue", 5), ("imu", 3), ("magnetic", 7), ("offset", 8)] {
            if self.items.iter().any(|i| !i.ok && i.kind == kind) {
                return code;
            }
        }
        0
    }
}

pub fn run(ctx: &mut Ctx, cfg: &Config, o: &CheckOpts) -> R<CheckReport> {
    let model = Model::from_config(cfg);
    let mut r = CheckReport::default();
    ctx.imu.wait_alive(ctx.tm.d(5.0))?;

    let el = measure_el(ctx, &model)?;
    let pos = ctx.rot.get_pos()?;
    r.rotator = pos;
    r.imu_el_deg = el.el;
    let mut tmp = Report::default();
    temperature_check(cfg, el.w.temp, &mut tmp);
    r.warnings.extend(tmp.warnings);

    let (perr, gaps, _) = ctx.imu.stats();
    r.add("imu stream", true, "info", format!("{perr} parse errors, {gaps} missing seq numbers"));
    let g = el.w.acc.norm();
    r.add("accelerometer magnitude", true, "imu", format!("{g:.4} g, noise {:.4} g", el.w.acc_std));

    let lim = ctx.th.off_axis_deg + cfg.mount.az_axis_tilt_deg;
    r.add(
        "IMU attitude vs elevation axis",
        el.off_axis <= lim,
        "glue",
        format!("{:.2}° off the calibrated axis cone (limit {lim:.2}°)", el.off_axis),
    );

    let el_err = pos.1 - el.el / cfg.elevation.gain;
    r.rotator_error_el_deg = el_err;
    ctx.zero_rep = (pos.0, el_err);
    r.add(
        "rotator elevation zero",
        el_err.abs() <= ctx.th.el_tol_deg.max(ctx.th.pos_tol_deg * 0.5),
        "offset",
        format!("reported {:.2}°, IMU true {:+.2}° → rotator error {el_err:+.2}°", pos.1, el.el),
    );

    if !o.skip_az {
        match measure_az(ctx, &model) {
            Ok(a) => {
                r.imu_az_deg = Some(a.az);
                r.imu_az_std_err_deg = Some(a.se);
                r.add(
                    "magnetic field consistency",
                    true,
                    "magnetic",
                    format!(
                        "horizontal ratio {:.3}, vertical error {:+.2} uT, single-sample σ {:.2}°, n={}",
                        a.ev.h_ratio, a.ev.v_err, a.sd, a.n
                    ),
                );
                let az_err = wrap180(pos.0 - a.az / cfg.azimuth.gain);
                r.rotator_error_az_deg = Some(az_err);
                ctx.zero_rep.0 = pos.0 - a.az / cfg.azimuth.gain;
                r.add(
                    "rotator azimuth zero",
                    az_err.abs() <= ctx.th.az_tol_deg + 2.0 * a.se,
                    "offset",
                    format!("reported {:.2}°, IMU true {:+.2}° (±{:.2}) → rotator error {az_err:+.2}°", pos.0, a.az, a.se),
                );
            }
            Err(Fail::Magnetic(m)) => r.add("magnetic field consistency", false, "magnetic", m),
            Err(e) => return Err(e),
        }
    }

    if o.wiggle {
        wiggle_el(ctx, cfg, &model, o.step, el.el, pos, &mut r)?;
        if !o.skip_az && r.imu_az_deg.is_some() {
            wiggle_az(ctx, cfg, &model, o.step, r.imu_az_deg.unwrap(), pos, &mut r)?;
        }
    }
    Ok(r)
}

fn pick(ctx: &Ctx, base: (f64, f64), d: (f64, f64)) -> Option<(f64, f64)> {
    let a = (base.0 + d.0, base.1 + d.1);
    let b = (base.0 - d.0, base.1 - d.1);
    if check_limits(ctx, a, Some(base)).is_ok() {
        Some(a)
    } else if check_limits(ctx, b, Some(base)).is_ok() {
        Some(b)
    } else {
        None
    }
}

fn record(r: &mut CheckReport, name: &str, res: R<String>) -> R<()> {
    match res {
        Ok(d) => r.add(name, true, "glue", d),
        Err(Fail::Glue(m)) => r.add(name, false, "glue", m),
        Err(Fail::Magnetic(m)) => r.add(name, false, "magnetic", m),
        Err(e) => return Err(e),
    }
    Ok(())
}

fn wiggle_el(ctx: &mut Ctx, cfg: &Config, model: &Model, step: f64, el0: f64, pos: (f64, f64), r: &mut CheckReport) -> R<()> {
    let Some(t) = pick(ctx, pos, (0.0, step)) else {
        r.add("elevation wiggle", false, "glue", "no reachable test position".into());
        return Ok(());
    };
    let gain = cfg.elevation.gain;
    let res = (|| -> R<String> {
        move_to(ctx, t, Watch::Follow { axis: Axis::El, model, imu0: el0, gain })?;
        let p1 = ctx.rot.get_pos()?;
        let e1 = measure_el(ctx, model)?;
        let d_rot = p1.1 - pos.1;
        let d_imu = e1.el - el0;
        let tol = ctx.th.glue_el_abs_deg + ctx.th.glue_el_rel * d_rot.abs();
        move_to(ctx, pos, Watch::Follow { axis: Axis::El, model, imu0: e1.el, gain })?;
        let e2 = measure_el(ctx, model)?;
        let back = e2.el - el0;
        if (d_imu - gain * d_rot).abs() > tol || d_imu * d_rot < 0.0 {
            return Err(Fail::Glue(format!(
                "rotator {d_rot:+.2}° → IMU {d_imu:+.2}° (expected {:+.2} ± {tol:.2})",
                gain * d_rot
            )));
        }
        if back.abs() > ctx.th.glue_el_abs_deg {
            return Err(Fail::Glue(format!("did not return: IMU elevation differs by {back:+.2}° after returning")));
        }
        Ok(format!("rotator {d_rot:+.2}° → IMU {d_imu:+.2}° (sign and gain OK), return error {back:+.2}°"))
    })();
    record(r, "elevation wiggle", res)
}

fn wiggle_az(ctx: &mut Ctx, cfg: &Config, model: &Model, step: f64, az0: f64, pos: (f64, f64), r: &mut CheckReport) -> R<()> {
    let Some(t) = pick(ctx, pos, (step, 0.0)) else {
        r.add("azimuth wiggle", false, "glue", "no reachable test position".into());
        return Ok(());
    };
    let gain = cfg.azimuth.gain;
    let res = (|| -> R<String> {
        let a0 = measure_az(ctx, model)?;
        move_to(ctx, t, Watch::Follow { axis: Axis::Az, model, imu0: a0.az, gain })?;
        let p1 = ctx.rot.get_pos()?;
        let a1 = measure_az(ctx, model)?;
        let d_rot = p1.0 - pos.0;
        let d_imu = wrap180(a1.az - a0.az);
        let tol = ctx.th.glue_az_abs_deg + ctx.th.glue_az_rel * d_rot.abs() + 3.0 * a0.se.hypot(a1.se);
        move_to(ctx, pos, Watch::Follow { axis: Axis::Az, model, imu0: a1.az, gain })?;
        let a2 = measure_az(ctx, model)?;
        let back = wrap180(a2.az - a0.az);
        let _ = az0;
        if (d_imu - gain * d_rot).abs() > tol || d_imu * d_rot < 0.0 {
            return Err(Fail::Glue(format!(
                "rotator {d_rot:+.2}° → magnetometer {d_imu:+.2}° (expected {:+.2} ± {tol:.2})",
                gain * d_rot
            )));
        }
        if back.abs() > ctx.th.glue_az_abs_deg {
            return Err(Fail::Glue(format!("did not return: IMU azimuth differs by {back:+.2}° after returning")));
        }
        Ok(format!("rotator {d_rot:+.2}° → IMU {d_imu:+.2}° (sign and gain OK), return error {back:+.2}°"))
    })();
    record(r, "azimuth wiggle", res)
}
