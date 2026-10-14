//! Commanded moves with live supervision: stall detection, timeouts, and a
//! "does the IMU follow the rotator while it moves" watchdog that stops the
//! rotator as soon as the mechanical link looks broken.

use crate::error::{Fail, R};
use crate::math::{circ_mean_std_deg, wrap180};
use crate::measure::Ctx;
use crate::model::Model;
use log::{debug, info, warn};
use std::thread;
use std::time::Instant;

#[derive(Clone, Copy)]
pub enum Axis {
    Az,
    El,
}

#[derive(Clone, Copy)]
pub enum Watch<'a> {
    None,
    /// Expect IMU angle on `axis` to change by `gain` × rotator travel,
    /// starting from `imu0` (a settled measurement taken before the move).
    Follow { axis: Axis, model: &'a Model, imu0: f64, gain: f64 },
}

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)] // start/end/reached are kept for debugging output
pub struct MoveOut {
    pub start: (f64, f64),
    pub end: (f64, f64),
    pub reached: bool,
    pub moved: bool,
}

/// [limits] are physical travel limits in true coordinates, known only to
/// within the IMU's measurement and the rotator's positioning (pos_tol_deg).
/// A target is allowed if each axis is inside them, or (given the current
/// position) no further outside than it already is.
pub fn check_limits(ctx: &Ctx, target: (f64, f64), from: Option<(f64, f64)>) -> R<()> {
    let eps = ctx.th.pos_tol_deg;
    let l = &ctx.lim;
    let t = ctx.to_true(target);
    let f = from.map(|p| ctx.to_true(p));
    let out = |v: f64, lo: f64, hi: f64| (lo - v).max(v - hi).max(0.0);
    let ok = |v: f64, cur: Option<f64>, lo: f64, hi: f64| {
        out(v, lo, hi) <= eps || cur.is_some_and(|c| out(v, lo, hi) <= out(c, lo, hi) + eps)
    };
    if !ok(t.0, f.map(|p| p.0), l.az_min, l.az_max) || !ok(t.1, f.map(|p| p.1), l.el_min, l.el_max) {
        return Err(Fail::Rejected(format!(
            "target az={:.2} el={:.2} (true az={:.2} el={:.2}) outside configured limits az[{}, {}] el[{}, {}]",
            target.0, target.1, t.0, t.1, l.az_min, l.az_max, l.el_min, l.el_max
        )));
    }
    Ok(())
}

fn stop_and(ctx: &mut Ctx, e: Fail) -> Fail {
    if let Err(se) = ctx.rot.stop() {
        warn!("stop command failed: {se}");
    }
    e
}

pub fn move_to(ctx: &mut Ctx, target: (f64, f64), watch: Watch) -> R<MoveOut> {
    let start = ctx.rot.get_pos()?;
    check_limits(ctx, target, Some(start))?;
    let req = (target.0 - start.0).abs().max((target.1 - start.1).abs());
    let timeout = ctx.tm.d(ctx.tm.move_base_timeout_s + req / ctx.tm.min_speed_dps);
    info!(
        "move: az {:.2} -> {:.2}, el {:.2} -> {:.2}",
        start.0, target.0, start.1, target.1
    );
    ctx.rot.set_pos(target.0, target.1)?;
    let t0 = Instant::now();
    let mut last_change = t0;
    let mut prev = start;
    let mut any_motion = false;
    let mut strikes = 0u32;
    loop {
        thread::sleep(ctx.tm.poll());
        let pos = match ctx.rot.get_pos() {
            Ok(p) => p,
            Err(e) => return Err(stop_and(ctx, e)),
        };
        if (pos.0 - prev.0).abs() > 0.05 || (pos.1 - prev.1).abs() > 0.05 {
            last_change = Instant::now();
        }
        prev = pos;
        if (pos.0 - start.0).abs().max((pos.1 - start.1).abs()) > 0.3 {
            any_motion = true;
        }
        if let Err(e) = follow_check(ctx, &watch, t0, start, pos, &mut strikes) {
            return Err(stop_and(ctx, e));
        }
        let err = (pos.0 - target.0).abs().max((pos.1 - target.1).abs());
        let quiet = last_change.elapsed() >= ctx.tm.d(ctx.tm.stop_detect_s);
        if quiet && err <= ctx.th.pos_tol_deg {
            return Ok(MoveOut { start, end: pos, reached: true, moved: any_motion });
        }
        if quiet && !any_motion && req <= ctx.th.min_step_deg {
            debug!("no motion for a {req:.2}° request (inside rotator deadband)");
            return Ok(MoveOut { start, end: pos, reached: false, moved: false });
        }
        if last_change.elapsed() >= ctx.tm.d(ctx.tm.stall_s) {
            if err <= (3.0 * ctx.th.pos_tol_deg).max(ctx.th.min_step_deg) {
                warn!("rotator stopped {err:.2}° short of target");
                return Ok(MoveOut { start, end: pos, reached: false, moved: any_motion });
            }
            let e = Fail::Rotator(format!(
                "stalled at az={:.2} el={:.2}, target az={:.2} el={:.2} (motor, brake, limit switch, or controller fault)",
                pos.0, pos.1, target.0, target.1
            ));
            return Err(stop_and(ctx, e));
        }
        if t0.elapsed() > timeout {
            let e = Fail::Rotator(format!("move timed out after {:.0}s at az={:.2} el={:.2}", timeout.as_secs_f64(), pos.0, pos.1));
            return Err(stop_and(ctx, e));
        }
    }
}

fn follow_check(ctx: &Ctx, watch: &Watch, t_cmd: Instant, start: (f64, f64), pos: (f64, f64), strikes: &mut u32) -> R<()> {
    let Watch::Follow { axis, model, imu0, gain } = *watch else { return Ok(()) };
    let (travel, min_travel, noise, name) = match axis {
        Axis::El => (pos.1 - start.1, ctx.th.follow_min_travel_el_deg, 0.5, "elevation"),
        Axis::Az => (pos.0 - start.0, ctx.th.follow_min_travel_az_deg, 3.0, "azimuth"),
    };
    if travel.abs() < min_travel {
        return Ok(());
    }
    let recent = ctx.imu.recent_since(t_cmd, 3);
    let recent: Vec<_> = recent.into_iter().filter(|r| r.s.reject_reason().is_none()).collect();
    if recent.is_empty() {
        return Ok(());
    }
    let now = match axis {
        Axis::El => recent.iter().map(|r| model.el(r.s.acc())).sum::<f64>() / recent.len() as f64,
        Axis::Az => {
            let v: Vec<f64> = recent.iter().map(|r| model.az_eval(r.s.acc(), r.s.mag()).az).collect();
            circ_mean_std_deg(&v).0
        }
    };
    let expected = gain * travel;
    let d = match axis {
        Axis::El => now - imu0,
        Axis::Az => {
            // Beyond ~150° the wrapped comparison becomes ambiguous; rely on
            // the post-move check instead.
            if expected.abs() > 150.0 {
                return Ok(());
            }
            expected + wrap180(now - imu0 - expected)
        }
    };
    let progress = d * expected.signum();
    if progress < ctx.th.follow_min_frac * expected.abs() - noise {
        *strikes += 1;
        debug!("follow strike {strikes}: rotator {travel:.1}°, IMU {d:.1}°");
    } else {
        *strikes = 0;
    }
    if *strikes >= 3 {
        return Err(Fail::Glue(format!(
            "rotator moved {travel:+.1}° in {name} but the IMU only saw {d:+.1}° (expected ≈{expected:+.1}°). \
             The antenna/IMU is not following the rotator: slipping mast clamp, broken coupling, or IMU detached. \
             Rotator stopped."
        )));
    }
    Ok(())
}
