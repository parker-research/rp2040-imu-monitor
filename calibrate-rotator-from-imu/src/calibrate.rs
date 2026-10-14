//! `calibrate-rotator-from-imu calibrate`: with the rotator physically at true az=0 el=0, capture
//! a reference, make small test moves on each axis, then survey the whole
//! operating envelope (azimuth rings at several elevations, elevation
//! columns between them, and a held-out satellite-pass-like arc for
//! validation). From all of that derive the IMU mounting transform, sign
//! conventions, and magnetic model.

use crate::config::*;
use crate::error::{Fail, R};
use crate::math::{plane_normal, slope_through_origin, wrap180, M3, V3};
use crate::measure::{az_stats, window, Ctx, Window};
use crate::model::{fit_field, FitTerms, MagPoint, Model, ALL_TERMS, HARD_IRON_ONLY};
use crate::motion::{move_to, Axis, MoveOut, Watch};
use log::{info, warn};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufRead, Write};

pub struct CalOpts {
    pub step: f64,
    pub yes: bool,
    pub goto_zero: bool,
    /// Small test moves only; no envelope survey.
    pub quick: bool,
    pub az_step: f64,
    pub el_step: f64,
    pub az_span: f64,
}

#[derive(Clone)]
struct Pt {
    /// "zero" | "el" | "az" (small test moves) | "ring" | "column" | "pass" (survey)
    kind: &'static str,
    /// True position relative to the zero pose (rotator reported minus reported-at-zero).
    daz: f64,
    del: f64,
    w: Window,
}

/// One measured calibration point, written by `calibrate --report`.
#[derive(Serialize)]
pub struct SurveyRecord {
    pub kind: &'static str,
    pub rotator_az: f64,
    pub rotator_el: f64,
    pub true_az: f64,
    pub true_el: f64,
    pub acc_g: V3,
    pub mag_ut: V3,
    pub samples: usize,
    pub imu_el: f64,
    pub imu_az: f64,
    pub az_err: f64,
    pub h_ratio: f64,
    pub v_err_ut: f64,
}

pub struct CalOut {
    pub cfg: Config,
    pub survey: Vec<SurveyRecord>,
}

/// Test-move offsets: ±step, or +step and +2·step when the negative side is
/// outside the limits, so each axis still gets two distinct non-zero poses.
fn test_offsets(ctx: &Ctx, step: f64, axis_el: bool) -> Vec<f64> {
    let neg = if axis_el { (0.0, -step) } else { (-step, 0.0) };
    let blocked = crate::motion::check_limits(ctx, (ctx.zero_rep.0 + neg.0, ctx.zero_rep.1 + neg.1), None).is_err();
    if blocked {
        vec![step, 2.0 * step]
    } else {
        vec![step, -step]
    }
}

/// Move, treating "rotctld rejected the target" as a skip rather than an error.
fn try_move(ctx: &mut Ctx, t: (f64, f64), watch: Watch) -> R<Option<MoveOut>> {
    match move_to(ctx, t, watch) {
        Ok(o) => Ok(Some(o)),
        Err(Fail::Rejected(m)) => {
            warn!("skipping test point az={:.1} el={:.1}: {m}", t.0, t.1);
            Ok(None)
        }
        Err(e) => Err(e),
    }
}

fn confirm(p0: (f64, f64)) -> R<()> {
    print!(
        "\nRotator reports az={:.2} el={:.2}.\nIs the antenna PHYSICALLY at true az=0, el=0 right now? [y/N] ",
        p0.0, p0.1
    );
    std::io::stdout().flush().ok();
    let mut line = String::new();
    std::io::stdin().lock().read_line(&mut line).ok();
    if line.trim().eq_ignore_ascii_case("y") || line.trim().eq_ignore_ascii_case("yes") {
        Ok(())
    } else {
        Err(Fail::Config("aborted: set the rotator to true zero first (or pass --yes)".into()))
    }
}

fn sweep_offsets(p0az: f64, span: f64, step: f64, lim: &Limits) -> Vec<f64> {
    let half = span / 2.0;
    let (lo, hi) = if p0az - half >= lim.az_min && p0az + half <= lim.az_max {
        (-half, half)
    } else if p0az + span <= lim.az_max {
        (0.0, span)
    } else if p0az - span >= lim.az_min {
        (-span, 0.0)
    } else {
        ((lim.az_min - p0az).max(-span), (lim.az_max - p0az).min(span))
    };
    let n = ((hi - lo) / step).ceil().max(1.0) as usize;
    let mut v: Vec<f64> = (0..=n).map(|i| (lo + i as f64 * step).min(hi)).collect();
    v.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    if hi - lo >= 359.9 && v.len() > 1 {
        v.pop(); // same orientation as the first point
    }
    v
}

fn get_field(w: &Window, f: &str) -> Option<f64> {
    Some(match f {
        "fw_roll_deg" => w.fw_roll?,
        "fw_pitch_deg" => w.fw_pitch?,
        "fw_heading_deg" => w.fw_heading?,
        "acc_x_g" => w.acc.x(),
        "acc_y_g" => w.acc.y(),
        "acc_z_g" => w.acc.z(),
        "mag_x_uT" => w.mag.x(),
        "mag_y_uT" => w.mag.y(),
        "mag_z_uT" => w.mag.z(),
        _ => return None,
    })
}

const FIELDS: [&str; 9] = [
    "fw_roll_deg",
    "fw_pitch_deg",
    "fw_heading_deg",
    "acc_x_g",
    "acc_y_g",
    "acc_z_g",
    "mag_x_uT",
    "mag_y_uT",
    "mag_z_uT",
];

/// d(field)/d(rotator degree) for each field, from test points vs reference.
fn sensitivities(pts: &[(f64, &Window)], z: &Window) -> BTreeMap<String, f64> {
    let mut out = BTreeMap::new();
    for f in FIELDS {
        let Some(z0) = get_field(z, f) else { continue };
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        for (x, w) in pts {
            if let Some(v) = get_field(w, f) {
                let d = if f.ends_with("_deg") { wrap180(v - z0) } else { v - z0 };
                xs.push(*x);
                ys.push(d);
            }
        }
        if !xs.is_empty() {
            out.insert(f.to_string(), (slope_through_origin(&xs, &ys) * 1e5).round() / 1e5);
        }
    }
    out
}

fn primary(s: &BTreeMap<String, f64>) -> (String, i32) {
    s.iter()
        .filter(|(k, _)| k.starts_with("fw_"))
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map(|(k, v)| (k.clone(), if *v >= 0.0 { 1 } else { -1 }))
        .unwrap_or(("none".into(), 0))
}

fn axis_name(v: V3) -> String {
    let names = ["x", "y", "z"];
    let i = (0..3).max_by(|&a, &b| v.0[a].abs().total_cmp(&v.0[b].abs())).unwrap();
    format!("{}IMU-{}", if v.0[i] >= 0.0 { "+" } else { "-" }, names[i])
}

struct Plan {
    kind: &'static str,
    az: f64,
    el: f64,
}

/// Serpentine path in true coordinates: an azimuth ring at each elevation
/// level (alternating direction), a mid-level point on each climb, a column
/// back down, then a pass-like arc between the ring azimuths whose points are
/// held out to validate the fit.
fn plan_survey(o: &CalOpts, lim: &Limits) -> (Vec<Plan>, Vec<f64>) {
    let ring = sweep_offsets(0.0, o.az_span, o.az_step, lim);
    let lo = lim.el_min.max(0.0);
    let mut levels = Vec::new();
    let mut l = lo;
    while l <= lim.el_max + 1e-9 {
        levels.push(l);
        l += o.el_step;
    }
    let mut v = Vec::new();
    let mut cur_az = ring[0];
    for (i, &lv) in levels.iter().enumerate() {
        let azs: Vec<f64> = if i % 2 == 0 { ring.clone() } else { ring.iter().rev().copied().collect() };
        if i > 0 {
            v.push(Plan { kind: "column", az: cur_az, el: (levels[i - 1] + lv) / 2.0 });
        }
        v.extend(azs.iter().map(|&az| Plan { kind: "ring", az, el: lv }));
        cur_az = *azs.last().unwrap();
    }
    for i in (1..levels.len()).rev() {
        v.push(Plan { kind: "column", az: cur_az, el: (levels[i - 1] + levels[i]) / 2.0 });
    }
    if levels.len() > 1 && ring.len() > 1 {
        let top = *levels.last().unwrap();
        let mut mids: Vec<f64> = ring.windows(2).map(|w| (w[0] + w[1]) / 2.0).collect();
        if (cur_az - mids[0]).abs() > (cur_az - mids[mids.len() - 1]).abs() {
            mids.reverse();
        }
        let n = mids.len() as f64;
        for (j, &az) in mids.iter().enumerate() {
            let el = lo + (top - lo) * (std::f64::consts::PI * (j as f64 + 0.5) / n).sin();
            v.push(Plan { kind: "pass", az, el: (el * 10.0).round() / 10.0 });
        }
    }
    (v, levels)
}

fn mag_points<'a>(pts: impl IntoIterator<Item = &'a Pt>, model: &Model) -> Vec<MagPoint> {
    pts.into_iter().map(|p| MagPoint { mag_imu: p.w.mag, el: model.el(p.w.acc), daz: p.daz }).collect()
}

/// Soft iron and the azimuth-stage field only become observable with
/// elevation spread, and soft iron additionally needs azimuth spread up there.
fn terms_for(pts: &[&Pt]) -> FitTerms {
    let high: Vec<&&Pt> = pts.iter().filter(|p| p.del >= 15.0).collect();
    let azs: BTreeSet<i64> = high.iter().map(|p| (wrap180(p.daz) / 10.0).round() as i64).collect();
    match (high.len() >= 3, azs.len() >= 4) {
        (true, true) => ALL_TERMS,
        (true, false) => FitTerms { soft_iron: false, stage_field: true },
        _ => HARD_IRON_ONLY,
    }
}

/// Fit with both azimuth sign conventions; keep the better one.
/// Returns (model, sign, how much better it fits than the other sign).
fn fit_best(m: &M3, k: V3, mp: &[MagPoint], terms: FitTerms) -> R<(FieldModel, f64, f64)> {
    match (fit_field(m, k, mp, 1.0, terms), fit_field(m, k, mp, -1.0, terms)) {
        (Some(a), Some(b)) => {
            let (ra, rb) = (a.fit_rms_ut.max(1e-6), b.fit_rms_ut.max(1e-6));
            let ratio = (ra / rb).max(rb / ra);
            Ok(if ra <= rb { (a, 1.0, ratio) } else { (b, -1.0, ratio) })
        }
        (Some(a), None) => Ok((a, 1.0, f64::INFINITY)),
        (None, Some(b)) => Ok((b, -1.0, f64::INFINITY)),
        _ => Err(Fail::Magnetic("magnetic model fit is singular".into())),
    }
}

/// Refit on everything so far. Returns the model, its rms, and whether the
/// azimuth sign is clearly determined (only then is it used to supervise
/// azimuth moves).
fn refit(pts: &[Pt], el_model: &Model) -> R<(Model, f64, bool)> {
    let all: Vec<&Pt> = pts.iter().collect();
    let (f, sign, ratio) = fit_best(&el_model.m, el_model.k, &mag_points(all.iter().copied(), el_model), terms_for(&all))?;
    Ok((el_model.clone().with_field(&f, sign, 1.0, None), f.fit_rms_ut, ratio >= 2.0))
}

fn mount_from(k_imu: V3, u0: V3) -> (M3, V3) {
    let b0 = k_imu.cross(u0).unit();
    (M3::from_rows(b0, u0.cross(b0), u0), b0)
}

/// Per-point link checks during the survey: the IMU must agree with the
/// rotator on elevation, stay on the elevation-axis cone, and roughly agree
/// on azimuth (loosely above el=0, where the model is still being learned).
fn check_survey_point(
    ctx: &Ctx,
    model: &Model,
    trusted: bool,
    w: &Window,
    daz: f64,
    del: f64,
    az_tilt: f64,
) -> R<(f64, f64, f64)> {
    let e = model.el(w.acc);
    // a non-level base makes gravity-measured elevation wobble with azimuth
    let tol = ctx.th.glue_el_abs_deg + ctx.th.glue_el_rel * del.abs() + 2.0 * az_tilt + 0.5;
    if (e - model.el_gain * del).abs() > tol {
        return Err(Fail::Glue(format!(
            "at rotator az {daz:+.1}° el {del:+.1}° the IMU elevation is {e:+.1}° (± {tol:.1}°): antenna/IMU not following elevation"
        )));
    }
    let off = model.off_axis(w.acc);
    let off_lim = ctx.th.off_axis_deg + 2.0 * az_tilt + 0.5;
    if off > off_lim {
        return Err(Fail::Glue(format!(
            "at rotator az {daz:+.1}° el {del:+.1}° gravity is {off:.2}° off the elevation-axis cone (limit {off_lim:.2}°): IMU loose or tilted"
        )));
    }
    let ev = model.az_eval(w.acc, w.mag);
    let disc = wrap180(ev.az - daz);
    let az_tol = if del.abs() < 5.0 { 25.0 } else { 45.0 };
    if trusted && disc.abs() > az_tol {
        return Err(Fail::Glue(format!(
            "at rotator az {daz:+.1}° el {del:+.1}° the magnetometer indicates {:+.1}°: antenna not following the rotator, \
             or a very strong local magnetic disturbance",
            ev.az
        )));
    }
    Ok((e, ev.az, ev.h_ratio))
}

/// Walk the planned survey, measuring at every stop. Refits the magnetic
/// model after every stop so later (higher) rings are supervised with a
/// model that already knows the lower ones.
fn survey(ctx: &mut Ctx, o: &CalOpts, p0: (f64, f64), pts: &mut Vec<Pt>, el_model: &Model, az_tilt: f64, n_mag: usize) -> R<Vec<f64>> {
    let (plan, levels) = plan_survey(o, &ctx.lim);
    let est_s = plan.len() as f64 * (ctx.tm.settle_mag_s + ctx.tm.stop_detect_s + n_mag as f64 * 0.25 + 3.0);
    info!(
        "envelope survey: {} stops — azimuth rings at el {:?} every {}°, columns between them, and a {}-point validation pass (roughly {:.0} min plus travel)",
        plan.len(),
        levels,
        o.az_step,
        plan.iter().filter(|p| p.kind == "pass").count(),
        est_s / 60.0 / ctx.tm.speed
    );
    let (mut model, _, mut trusted) = refit(pts, el_model)?;
    let last = &pts.last().unwrap().w;
    let (mut last_el, mut last_az) = (model.el(last.acc), model.az_eval(last.acc, last.mag).az);
    for (i, pl) in plan.iter().enumerate() {
        let t = (p0.0 + pl.az, p0.1 + pl.el);
        let cur = ctx.rot.get_pos()?;
        let watch = if (t.1 - cur.1).abs() > (t.0 - cur.0).abs() {
            Watch::Follow { axis: Axis::El, model: &model, imu0: last_el, gain: el_model.el_gain }
        } else if trusted {
            Watch::Follow { axis: Axis::Az, model: &model, imu0: last_az, gain: 1.0 }
        } else {
            Watch::None
        };
        let Some(_) = try_move(ctx, t, watch)? else { continue };
        let pos = ctx.rot.get_pos()?;
        let w = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        let (daz, del) = (pos.0 - p0.0, pos.1 - p0.1);
        let (e, a, hr) = check_survey_point(ctx, &model, trusted, &w, daz, del, az_tilt)?;
        info!(
            "survey {:>3}/{} {:<6} az {daz:+7.1}° el {del:5.1}°: IMU el {e:5.1}°, az {a:+7.1}° (diff {:+5.1}°), |B_h| ratio {hr:.2}",
            i + 1,
            plan.len(),
            pl.kind,
            wrap180(a - daz)
        );
        (last_el, last_az) = (e, a);
        pts.push(Pt { kind: pl.kind, daz, del, w });
        let rms;
        (model, rms, trusted) = refit(pts, el_model)?;
        let ring_done = pl.kind == "ring" && plan.get(i + 1).is_none_or(|n| n.kind != "ring" || n.el != pl.el);
        if ring_done {
            info!("ring at el {:.0}° done: magnetic fit rms {rms:.2} uT", pl.el);
        }
    }
    move_to(ctx, p0, Watch::None)?;
    let pr = ctx.rot.get_pos()?;
    let wr = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
    pts.push(Pt { kind: "zero", daz: pr.0 - p0.0, del: pr.1 - p0.1, w: wr });
    Ok(levels)
}

fn rms_max(v: &[f64]) -> (f64, f64) {
    if v.is_empty() {
        return (0.0, 0.0);
    }
    let rms = (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
    (rms, v.iter().fold(0.0f64, |a, x| a.max(x.abs())))
}

pub fn run(ctx: &mut Ctx, o: &CalOpts) -> R<CalOut> {
    if !(1.0..=15.0).contains(&o.step) {
        return Err(Fail::Config("--step must be between 1 and 15 degrees".into()));
    }
    if !(5.0..=90.0).contains(&o.az_step) || !(5.0..=45.0).contains(&o.el_step) {
        return Err(Fail::Config("--az-step must be 5–90° and --el-step 5–45°".into()));
    }
    ctx.imu.wait_alive(ctx.tm.d(5.0))?;
    if o.goto_zero {
        move_to(ctx, (0.0, 0.0), Watch::None)?;
    }
    let p0 = ctx.rot.get_pos()?;
    if !o.yes {
        confirm(p0)?;
    }
    // By definition the antenna is at true zero now.
    ctx.zero_rep = p0;
    info!("capturing reference at true zero ({} samples)...", ctx.tm.samples_ref);
    let z = window(ctx, ctx.tm.samples_ref, ctx.tm.settle_mag_s)?;
    let u_ref = z.acc.unit();
    let n_ref = z.n as f64;
    let mut pts: Vec<Pt> = vec![Pt { kind: "zero", daz: 0.0, del: 0.0, w: z.clone() }];
    let mut rtz_err: f64 = 0.0;
    let n_mag = ctx.tm.samples_mag_min.max(24);

    // ---------------- elevation test moves ----------------
    for d in test_offsets(ctx, o.step, true) {
        let Some(_) = try_move(ctx, (p0.0, p0.1 + d), Watch::None)? else { continue };
        let pos = ctx.rot.get_pos()?;
        let w = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        let rep = pos.1 - p0.1;
        let ang = u_ref.angle_deg(w.acc.unit());
        info!("el test: rotator {rep:+.2}°, IMU tilt change {ang:.2}°");
        if ang < 0.5 * rep.abs() - 0.3 || ang > 1.5 * rep.abs() + 0.5 {
            return Err(Fail::Glue(format!(
                "elevation move of {rep:+.2}° changed the IMU tilt by {ang:.2}° — IMU not rigidly following elevation"
            )));
        }
        pts.push(Pt { kind: "el", daz: pos.0 - p0.0, del: rep, w });
        move_to(ctx, p0, Watch::None)?;
        let pr = ctx.rot.get_pos()?;
        let wr = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        let back = u_ref.angle_deg(wr.acc.unit());
        let rep_back = (pr.1 - p0.1).abs();
        rtz_err = rtz_err.max((back - rep_back).max(0.0));
        if back > rep_back + 0.6 {
            return Err(Fail::Glue(format!(
                "after returning to zero the IMU tilt differs by {back:.2}° (rotator reports {rep_back:.2}°): \
                 slop or slip between the rotator and the IMU"
            )));
        }
        pts.push(Pt { kind: "zero", daz: pr.0 - p0.0, del: pr.1 - p0.1, w: wr });
    }
    if !pts.iter().any(|p| p.kind == "el") {
        return Err(Fail::Rejected("no elevation test move was possible; check rotctld limits".into()));
    }

    // ---------------- azimuth test moves ----------------
    let hz_min = (4.0 * z.mag_std / n_ref.sqrt() * 2f64.sqrt()).max(0.5);
    for d in test_offsets(ctx, o.step, false) {
        let Some(_) = try_move(ctx, (p0.0 + d, p0.1), Watch::None)? else { continue };
        let pos = ctx.rot.get_pos()?;
        let w = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        let rep = pos.0 - p0.0;
        let ang = u_ref.angle_deg(w.acc.unit());
        let hz = (w.mag - z.mag).perp(u_ref).norm();
        info!("az test: rotator {rep:+.2}°, IMU tilt change {ang:.2}°, horizontal field change {hz:.2} uT");
        if ang > 1.0 + 0.1 * rep.abs() {
            return Err(Fail::Glue(format!(
                "azimuth-only move of {rep:+.2}° tilted the IMU by {ang:.2}°: antenna or IMU loose, or rotator far out of level"
            )));
        }
        if hz < hz_min {
            return Err(Fail::Glue(format!(
                "azimuth move of {rep:+.2}° changed the horizontal magnetic field by only {hz:.2} uT (< {hz_min:.2}): \
                 antenna not following azimuth (mast slip?) or magnetometer saturated"
            )));
        }
        pts.push(Pt { kind: "az", daz: rep, del: pos.1 - p0.1, w });
        move_to(ctx, p0, Watch::None)?;
        let pr = ctx.rot.get_pos()?;
        let wr = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        pts.push(Pt { kind: "zero", daz: pr.0 - p0.0, del: pr.1 - p0.1, w: wr });
    }
    if !pts.iter().any(|p| p.kind == "az") {
        return Err(Fail::Rejected("no azimuth test move was possible; check rotctld limits".into()));
    }

    // ---------------- preliminary mounting transform (small moves) ----------------
    let zeros: Vec<&Pt> = pts.iter().filter(|p| p.kind == "zero").collect();
    let u0 = zeros.iter().fold(V3::default(), |a, p| a + p.w.acc.unit()).unit();
    // furthest test point on each side
    let up_of = |s: f64| -> Option<V3> {
        pts.iter()
            .filter(|p| p.kind == "el" && p.del * s > 0.0)
            .max_by(|a, b| (a.del * s).total_cmp(&(b.del * s)))
            .map(|p| p.w.acc.unit())
    };
    let (up_p, up_m) = (up_of(1.0), up_of(-1.0));
    let kraw = match (up_m, up_p) {
        (Some(m), Some(p)) => m.cross(p),
        (None, Some(p)) => u0.cross(p),
        (Some(m), None) => m.cross(u0),
        _ => unreachable!(),
    };
    if kraw.norm() < 1e-4 {
        return Err(Fail::Glue("gravity did not rotate during elevation moves".into()));
    }
    let mut k_imu = kraw.unit();
    let (mut m, _) = mount_from(k_imu, u0);

    let el_model0 = Model::el_only(m, k_imu, 1.0);
    let mut el_moves = Vec::new();
    let (mut xs, mut ys) = (Vec::new(), Vec::new());
    for p in pts.iter().filter(|p| p.kind == "el") {
        let e = el_model0.el(p.w.acc);
        if e * p.del <= 0.0 {
            return Err(Fail::Glue(format!(
                "elevation responses inconsistent: rotator {:+.2}° but IMU {e:+.2}°",
                p.del
            )));
        }
        xs.push(p.del);
        ys.push(e);
        el_moves.push(MoveRecord { axis: "el".into(), commanded_deg: p.del.signum() * o.step, reported_deg: p.del, imu_deg: e });
    }
    let check_el_gain = |g: f64| -> R<()> {
        if !(0.75..=1.25).contains(&g) {
            return Err(Fail::Glue(format!(
                "IMU registers {g:.2}× the rotator's reported elevation change (expected ≈1): \
                 loose IMU, or rotator elevation scaling is badly wrong"
            )));
        }
        Ok(())
    };
    let mut el_gain = slope_through_origin(&xs, &ys);
    check_el_gain(el_gain)?;

    // azimuth axis tilt from accelerometer during az moves
    let ups: Vec<(f64, V3)> = pts.iter().filter(|p| p.kind == "az").map(|p| (p.daz, m.mul_v(p.w.acc.unit()))).collect();
    let up_z = m.mul_v(u0);
    let az_tilt = {
        let (a, b, span) = if ups.len() >= 2 {
            let (lo, hi) = (ups.iter().min_by(|x, y| x.0.total_cmp(&y.0)).unwrap(), ups.iter().max_by(|x, y| x.0.total_cmp(&y.0)).unwrap());
            (lo.1, hi.1, hi.0 - lo.0)
        } else {
            (ups[0].1, up_z, ups[0].0.abs())
        };
        let chord = (a - b).norm();
        let den = 2.0 * (span.to_radians() / 2.0).sin();
        (chord / den).clamp(0.0, 1.0).asin().to_degrees()
    };

    // ---------------- envelope survey ----------------
    let mut stats = SurveyStats::default();
    if !o.quick {
        let el_model = Model::el_only(m, k_imu, el_gain);
        stats.el_levels_deg = survey(ctx, o, p0, &mut pts, &el_model, az_tilt, n_mag)?;
        stats.performed = true;
        stats.az_step_deg = o.az_step;

        // Refine the elevation axis and gain over the whole range: gravity
        // sweeps a circle about the elevation axis.
        let el_span = pts.iter().map(|p| p.del).fold(f64::NEG_INFINITY, f64::max)
            - pts.iter().map(|p| p.del).fold(f64::INFINITY, f64::min);
        if el_span >= 20.0 {
            let ups: Vec<V3> = pts.iter().map(|p| p.w.acc.unit()).collect();
            if let Some(k) = plane_normal(&ups, k_imu) {
                info!("elevation axis refined over {el_span:.0}° of travel: moved {:.2}°", k.angle_deg(k_imu));
                k_imu = k;
                m = mount_from(k_imu, u0).0;
            }
            let em = Model::el_only(m, k_imu, 1.0);
            let (xs, ys): (Vec<f64>, Vec<f64>) =
                pts.iter().filter(|p| p.del.abs() > 1.0).map(|p| (p.del, em.el(p.w.acc))).unzip();
            el_gain = slope_through_origin(&xs, &ys);
            check_el_gain(el_gain)?;
        }
    }
    let el_tilt = 90.0 - k_imu.angle_deg(u0);
    let el_model = Model::el_only(m, k_imu, el_gain);
    stats.el_residual_max_deg =
        pts.iter().map(|p| (el_model.el(p.w.acc) - el_gain * p.del).abs()).fold(0.0, f64::max);
    if stats.el_residual_max_deg > 1.0 + 2.0 * az_tilt {
        warn!(
            "IMU elevation departs from the rotator's by up to {:.2}° over the survey, more than the {az_tilt:.2}° base tilt \
             explains (backlash, sag, or non-linear rotator scale)",
            stats.el_residual_max_deg
        );
    }

    // ---------------- magnetic model ----------------
    let all: Vec<&Pt> = pts.iter().collect();
    let fit_set: Vec<&Pt> = pts.iter().filter(|p| p.kind != "pass").collect();
    let held: Vec<&Pt> = pts.iter().filter(|p| p.kind == "pass").collect();
    if !held.is_empty() {
        let (f, s, _) = fit_best(&m, el_model.k, &mag_points(fit_set.iter().copied(), &el_model), terms_for(&fit_set))?;
        let mv = el_model.clone().with_field(&f, s, 1.0, None);
        let errs: Vec<f64> = held.iter().map(|p| wrap180(mv.az_eval(p.w.acc, p.w.mag).az - p.daz)).collect();
        (stats.validation_az_rms_deg, stats.validation_az_max_deg) = rms_max(&errs);
        stats.validation_points = held.len();
        info!(
            "validation on {} held-out pass points: azimuth error rms {:.2}°, max {:.2}°",
            held.len(),
            stats.validation_az_rms_deg,
            stats.validation_az_max_deg
        );
    }
    let terms = terms_for(&all);
    let (field, az_sign, sign_ratio) = fit_best(&m, el_model.k, &mag_points(all.iter().copied(), &el_model), terms)?;
    info!(
        "magnetic fit ({} points, {}): sign {:+}, |B_h| {:.1} uT, hard-iron {:.1} uT, stage field {:.1} uT, rms {:.2} uT",
        all.len(),
        match (terms.soft_iron, terms.stage_field) {
            (true, true) => "hard + soft iron + azimuth-stage field",
            (false, true) => "hard iron + azimuth-stage field",
            _ => "hard iron only",
        },
        az_sign,
        field.earth_rot_ut.x().hypot(field.earth_rot_ut.y()),
        field.hard_iron_imu_ut.norm(),
        field.stage_field_ut.norm(),
        field.fit_rms_ut
    );

    let bh = field.earth_rot_ut.x().hypot(field.earth_rot_ut.y());
    if bh < 2.0 {
        return Err(Fail::Magnetic(format!(
            "horizontal Earth field resolves to only {bh:.1} uT — the magnetometer cannot sense azimuth here"
        )));
    }
    if field.fit_rms_ut > 0.4 * bh {
        return Err(Fail::Magnetic(format!(
            "magnetic model does not fit (rms {:.2} uT vs horizontal field {bh:.1} uT): magnetometer not tracking azimuth",
            field.fit_rms_ut
        )));
    }
    if sign_ratio < 1.5 {
        warn!("azimuth sign only weakly determined (fit ratio {sign_ratio:.2}); consider a larger --step or a full survey");
    }
    let dip = (-field.earth_rot_ut.z()).atan2(bh).to_degrees();

    // gain + deviation table, both from el≈0 where zeroing finishes
    let model_nodev = el_model.clone().with_field(&field, az_sign, 1.0, None);
    let level: Vec<&Pt> = pts.iter().filter(|p| p.del.abs() < 1.0).collect();
    let mut az_moves = Vec::new();
    let (mut xa, mut ya) = (Vec::new(), Vec::new());
    for p in level.iter().filter(|p| matches!(p.kind, "az" | "ring")) {
        let a = model_nodev.az_eval(p.w.acc, p.w.mag).az;
        let d = wrap180(p.daz);
        if d.abs() <= 30.0 {
            xa.push(d);
            ya.push(a);
        }
        if p.kind == "az" {
            az_moves.push(MoveRecord { axis: "az".into(), commanded_deg: p.daz.signum() * o.step, reported_deg: p.daz, imu_deg: a });
        }
    }
    let az_gain = slope_through_origin(&xa, &ya);
    if !(0.6..=1.4).contains(&az_gain) {
        return Err(Fail::Glue(format!(
            "magnetometer registers {az_gain:.2}× the rotator's azimuth change: antenna not following azimuth, or severe magnetic distortion"
        )));
    }
    let ring0: Vec<&&Pt> = level.iter().filter(|p| p.kind == "ring").collect();
    let swept = ring0.len() >= 3;
    let mut deviation: Vec<[f64; 2]> = Vec::new();
    let mut periodic = false;
    if swept {
        for p in level.iter().filter(|p| matches!(p.kind, "ring" | "zero")) {
            let a = model_nodev.az_eval(p.w.acc, p.w.mag).raw;
            deviation.push([(a * 100.0).round() / 100.0, (wrap180(a - p.daz) * 1000.0).round() / 1000.0]);
        }
        deviation.sort_by(|a, b| a[0].total_cmp(&b[0]));
        deviation.dedup_by(|a, b| (a[0] - b[0]).abs() < 0.5);
        let span = ring0.iter().map(|p| p.daz).fold(f64::NEG_INFINITY, f64::max)
            - ring0.iter().map(|p| p.daz).fold(f64::INFINITY, f64::min);
        periodic = span + o.az_step >= 359.0;
    }
    let dev_tbl = crate::math::DevTable::new(deviation.iter().map(|p| (p[0], p[1])).collect(), periodic);
    let model = el_model.clone().with_field(&field, az_sign, az_gain, dev_tbl);

    // residuals of the final model over every point
    let evs: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| {
            let ev = model.az_eval(p.w.acc, p.w.mag);
            (wrap180(ev.az - p.daz), ev.h_ratio)
        })
        .collect();
    stats.points = pts.len();
    (stats.az_residual_rms_deg, stats.az_residual_max_deg) = rms_max(&evs.iter().map(|e| e.0).collect::<Vec<_>>());
    stats.h_ratio_min = evs.iter().map(|e| e.1).fold(f64::INFINITY, f64::min);
    stats.h_ratio_max = evs.iter().map(|e| e.1).fold(f64::NEG_INFINITY, f64::max);
    if stats.performed {
        info!(
            "survey fit: azimuth error rms {:.2}°, max {:.2}°; |B_h| ratio {:.2}–{:.2}; elevation error max {:.2}°",
            stats.az_residual_rms_deg, stats.az_residual_max_deg, stats.h_ratio_min, stats.h_ratio_max, stats.el_residual_max_deg
        );
        if stats.validation_az_max_deg > 5.0 || stats.h_ratio_max - stats.h_ratio_min > 0.2 {
            warn!(
                "the magnetic model is only good to ±{:.1}° away from el=0 (|B_h| ratio {:.2}–{:.2}); zeroing still \
                 converges at el≈0, but expect `monitor` azimuth to be rough at high elevation",
                stats.validation_az_max_deg.max(stats.az_residual_max_deg),
                stats.h_ratio_min,
                stats.h_ratio_max
            );
        }
    }

    // closure check: first vs last zero
    let first = &pts[0].w;
    let last = &pts.iter().rev().find(|p| p.kind == "zero").unwrap().w;
    let d_el = model.el(last.acc) - model.el(first.acc);
    let d_az = wrap180(model.az_eval(last.acc, last.mag).az - model.az_eval(first.acc, first.mag).az);
    info!("closure: zero reference drift el {d_el:+.2}°, az {d_az:+.2}°");
    if d_el.abs() > 0.7 || d_az.abs() > 3.0 {
        return Err(Fail::Glue(format!(
            "returning to zero gave a different IMU reading (el {d_el:+.2}°, az {d_az:+.2}°): something slipped during calibration"
        )));
    }
    let (_, az_sd, _) = az_stats(&model, z.acc, &z.mags);

    // ---------------- documentation of signs ----------------
    let el_sens = sensitivities(&pts.iter().filter(|p| p.kind == "el").map(|p| (p.del, &p.w)).collect::<Vec<_>>(), &z);
    let az_sens = sensitivities(
        &pts.iter().filter(|p| p.kind == "az").map(|p| (p.daz, &p.w)).collect::<Vec<_>>(),
        &z,
    );
    let (elp, els) = primary(&el_sens);
    let (azp, azs) = primary(&az_sens);
    let ypr = m.euler_zyx_deg();
    let b0 = mount_from(k_imu, u0).1;
    let summary = vec![
        format!("+elevation: firmware {elp} changes {:+.3}°/° (sign {els:+})", el_sens.get(&elp).copied().unwrap_or(0.0)),
        format!("+azimuth:   firmware {azp} changes {:+.3}°/° (sign {azs:+})", az_sens.get(&azp).copied().unwrap_or(0.0)),
        format!("elevation axis is {} (IMU frame {:?})", axis_name(k_imu), round3(k_imu)),
        format!("gravity 'up' at zero is {} (IMU frame {:?})", axis_name(u0), round3(u0)),
        format!("boresight at zero is {} (IMU frame {:?})", axis_name(b0), round3(b0)),
        format!(
            "magnetometer azimuth sign {:+} vs hamlib clockwise azimuth (fit confidence ratio {:.1})",
            az_sign, sign_ratio
        ),
        format!("elevation gain {el_gain:.3}, azimuth gain {az_gain:.3} (IMU deg per rotator deg)"),
    ];
    let mount = Mount {
        description: vec![
            format!("IMU yaw/pitch/roll in rotator frame: {:.2} / {:.2} / {:.2} deg", ypr[0], ypr[1], ypr[2]),
            format!("elevation axle tilt {el_tilt:.2} deg, azimuth axis tilt {az_tilt:.2} deg"),
        ],
        imu_to_rotator: m.0,
        imu_yaw_pitch_roll_deg: ypr,
        up_imu: u0,
        el_axis_imu: k_imu,
        boresight_imu: b0,
        el_axis_tilt_deg: el_tilt,
        az_axis_tilt_deg: az_tilt,
    };
    let survey_out = pts
        .iter()
        .zip(&evs)
        .map(|(p, (err, hr))| {
            let ev = model.az_eval(p.w.acc, p.w.mag);
            SurveyRecord {
                kind: p.kind,
                rotator_az: p0.0 + p.daz,
                rotator_el: p0.1 + p.del,
                true_az: p.daz,
                true_el: p.del,
                acc_g: p.w.acc,
                mag_ut: p.w.mag,
                samples: p.w.n,
                imu_el: model.el(p.w.acc),
                imu_az: ev.az,
                az_err: *err,
                h_ratio: *hr,
                v_err_ut: ev.v_err,
            }
        })
        .collect();
    let cfg = Config {
        version: VERSION,
        created: chrono::Local::now().to_rfc3339(),
        imu_source: ctx.imu.source.clone(),
        reference: Reference {
            rotator_reported_az: p0.0,
            rotator_reported_el: p0.1,
            acc_g: z.acc,
            mag_ut: z.mag,
            acc_noise_g: z.acc_std,
            mag_noise_ut: z.mag_std,
            samples: z.n,
            mcu_temp_c: z.temp,
            fw_roll_deg: z.fw_roll,
            fw_pitch_deg: z.fw_pitch,
            fw_heading_deg: z.fw_heading,
        },
        mount,
        elevation: ElCal {
            gain: el_gain,
            step_deg: o.step,
            cone_angle_deg: el_model.cone0,
            return_to_zero_err_deg: rtz_err,
            moves: el_moves,
        },
        azimuth: AzCal {
            sign: az_sign,
            gain: az_gain,
            step_deg: o.step,
            swept,
            deviation,
            deviation_periodic: periodic,
            horizontal_field_ut: bh,
            dip_deg: dip,
            single_sample_noise_deg: az_sd,
            field_model: field,
            moves: az_moves,
        },
        signs: Signs {
            summary,
            elevation_primary_field: elp,
            elevation_primary_sign: els,
            azimuth_primary_field: azp,
            azimuth_primary_sign: azs,
            elevation: el_sens,
            azimuth: az_sens,
        },
        survey: stats,
        limits: ctx.lim.clone(),
        thresholds: ctx.th.clone(),
        timing: ctx.tm.clone(),
        sync: Sync::default(),
        motion: ctx.rot.motion.clone(),
    };
    Ok(CalOut { cfg, survey: survey_out })
}

fn round3(v: V3) -> [f64; 3] {
    v.0.map(|x| (x * 1000.0).round() / 1000.0)
}
