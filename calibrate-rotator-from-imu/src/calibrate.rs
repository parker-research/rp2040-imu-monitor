//! `calibrate-rotator-from-imu calibrate`: with the rotator physically at true az=0 el=0, capture
//! a reference, make small test moves on each axis, and derive the IMU
//! mounting transform, sign conventions, and magnetic model.

use crate::config::*;
use crate::error::{Fail, R};
use crate::math::{slope_through_origin, wrap180, M3, V3};
use crate::measure::{az_stats, window, Ctx, Window};
use crate::model::{fit_field, MagPoint, Model, UP};
use crate::motion::{move_to, Axis, MoveOut, Watch};
use log::{info, warn};
use std::collections::BTreeMap;
use std::io::{BufRead, Write};
use std::time::Duration;

pub struct CalOpts {
    pub step: f64,
    pub yes: bool,
    pub goto_zero: bool,
    pub sweep: bool,
    pub sweep_step: f64,
    pub sweep_span: f64,
}

#[derive(Clone)]
struct Pt {
    kind: &'static str, // "zero" | "el" | "az" | "sweep"
    daz: f64,
    del: f64,
    w: Window,
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
    if hi - lo >= 359.9 {
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

pub fn run(ctx: &mut Ctx, o: &CalOpts) -> R<Config> {
    if !(1.0..=15.0).contains(&o.step) {
        return Err(Fail::Config("--step must be between 1 and 15 degrees".into()));
    }
    ctx.imu.wait_alive(ctx.tm.d(5.0))?;
    if o.goto_zero {
        move_to(ctx, (0.0, 0.0), Watch::None)?;
    }
    let p0 = ctx.rot.get_pos()?;
    if !o.yes {
        confirm(p0)?;
    }
    info!("capturing reference at true zero ({} samples)...", ctx.tm.samples_ref);
    let z = window(ctx, ctx.tm.samples_ref, ctx.tm.settle_mag_s)?;
    let u_ref = z.acc.unit();
    let n_ref = z.n as f64;
    let mut pts: Vec<Pt> = vec![Pt { kind: "zero", daz: 0.0, del: 0.0, w: z.clone() }];
    let mut rtz_err: f64 = 0.0;
    let n_mag = ctx.tm.samples_mag_min.max(24);

    // ---------------- elevation test moves ----------------
    for d in [o.step, -o.step] {
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
    for d in [o.step, -o.step] {
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

    // ---------------- mounting transform ----------------
    let zeros: Vec<&Pt> = pts.iter().filter(|p| p.kind == "zero").collect();
    let u0 = zeros.iter().fold(V3::default(), |a, p| a + p.w.acc.unit()).unit();
    let up_of = |s: f64| -> Option<V3> {
        let v: Vec<&Pt> = pts.iter().filter(|p| p.kind == "el" && p.del * s > 0.0).collect();
        (!v.is_empty()).then(|| v.iter().fold(V3::default(), |a, p| a + p.w.acc.unit()).unit())
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
    let k_imu = kraw.unit();
    let b0 = k_imu.cross(u0).unit();
    let y = u0.cross(b0);
    let m = M3::from_rows(b0, y, u0);
    let el_tilt = 90.0 - k_imu.angle_deg(u0);

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
    let el_gain = slope_through_origin(&xs, &ys);
    if !(0.75..=1.25).contains(&el_gain) {
        return Err(Fail::Glue(format!(
            "IMU registers {el_gain:.2}× the rotator's reported elevation change (expected ≈1): \
             loose IMU, or rotator elevation scaling is badly wrong"
        )));
    }
    let el_model = Model::el_only(m, k_imu, el_gain);

    // azimuth axis tilt from accelerometer during az moves
    let ups: Vec<(f64, V3)> = pts.iter().filter(|p| p.kind == "az").map(|p| (p.daz, m.mul_v(p.w.acc.unit()))).collect();
    let up_z = m.mul_v(u0);
    let az_tilt = {
        let (a, b, span) = if ups.len() >= 2 {
            (ups[0].1, ups[1].1, (ups[0].0 - ups[1].0).abs())
        } else {
            (ups[0].1, up_z, ups[0].0.abs())
        };
        let chord = (a - b).norm();
        let den = 2.0 * (span.to_radians() / 2.0).sin();
        (chord / den).clamp(0.0, 1.0).asin().to_degrees()
    };

    // ---------------- magnetic model ----------------
    let magpts = |pts: &[Pt], model: &Model| -> Vec<MagPoint> {
        pts.iter().map(|p| MagPoint { mag_imu: p.w.mag, el: model.el(p.w.acc), daz: p.daz }).collect()
    };
    let fit_best = |mp: &[MagPoint]| -> R<(FieldModel, f64, f64)> {
        let fp = fit_field(&m, el_model.k, mp, 1.0);
        let fm = fit_field(&m, el_model.k, mp, -1.0);
        match (fp, fm) {
            (Some(a), Some(b)) => {
                let ratio = (a.fit_rms_ut.max(1e-6) / b.fit_rms_ut.max(1e-6)).max(b.fit_rms_ut.max(1e-6) / a.fit_rms_ut.max(1e-6));
                Ok(if a.fit_rms_ut <= b.fit_rms_ut { (a, 1.0, ratio) } else { (b, -1.0, ratio) })
            }
            (Some(a), None) => Ok((a, 1.0, f64::INFINITY)),
            (None, Some(b)) => Ok((b, -1.0, f64::INFINITY)),
            _ => Err(Fail::Magnetic("magnetic model fit is singular".into())),
        }
    };
    let (mut field, mut az_sign, mut sign_ratio) = fit_best(&magpts(&pts, &el_model))?;
    info!(
        "magnetic fit (small moves): sign {:+}, |B_h| {:.1} uT, hard-iron {:.1} uT, rms {:.2} uT",
        az_sign,
        field.earth_rot_ut.x().hypot(field.earth_rot_ut.y()),
        field.hard_iron_imu_ut.norm(),
        field.fit_rms_ut
    );

    let mut swept = false;
    if o.sweep {
        let prelim = el_model.clone().with_field(&field, az_sign, 1.0, None);
        let offs = sweep_offsets(p0.0, o.sweep_span, o.sweep_step, &ctx.lim);
        info!("magnetometer sweep over {} points: {:?}", offs.len(), offs);
        let mut last_az = 0.0;
        for d in offs {
            let watch = Watch::Follow { axis: Axis::Az, model: &prelim, imu0: last_az, gain: 1.0 };
            let Some(_) = try_move(ctx, (p0.0 + d, p0.1), watch)? else { continue };
            let pos = ctx.rot.get_pos()?;
            let w = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
            let daz = pos.0 - p0.0;
            let ang = u_ref.angle_deg(w.acc.unit());
            if ang > 1.0 + 2.0 * az_tilt + 0.5 {
                return Err(Fail::Glue(format!("IMU tilted {ang:.2}° during azimuth sweep at {daz:+.1}° (az tilt est. {az_tilt:.2}°)")));
            }
            let a = prelim.az_eval(w.acc, w.mag).az;
            let disc = wrap180(a - daz);
            info!("sweep {daz:+7.1}°: model az {a:+7.1}° (diff {disc:+.1}°)");
            if disc.abs() > 25.0 {
                return Err(Fail::Glue(format!(
                    "at rotator az {daz:+.1}° the magnetometer indicates {a:+.1}°: antenna not following the rotator, \
                     or a very strong local magnetic disturbance"
                )));
            }
            last_az = a;
            pts.push(Pt { kind: "sweep", daz, del: pos.1 - p0.1, w });
        }
        let back = Watch::Follow { axis: Axis::Az, model: &prelim, imu0: last_az, gain: 1.0 };
        move_to(ctx, p0, back)?;
        let pr = ctx.rot.get_pos()?;
        let wr = window(ctx, n_mag, ctx.tm.settle_mag_s)?;
        pts.push(Pt { kind: "zero", daz: pr.0 - p0.0, del: pr.1 - p0.1, w: wr });
        (field, az_sign, sign_ratio) = fit_best(&magpts(&pts, &el_model))?;
        swept = true;
        info!(
            "magnetic fit (with sweep): |B_h| {:.1} uT, hard-iron {:.1} uT, rms {:.2} uT",
            field.earth_rot_ut.x().hypot(field.earth_rot_ut.y()),
            field.hard_iron_imu_ut.norm(),
            field.fit_rms_ut
        );
    }

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
        warn!("azimuth sign only weakly determined (fit ratio {sign_ratio:.2}); consider a larger --step or --sweep");
    }
    let dip = (-field.earth_rot_ut.z()).atan2(bh).to_degrees();

    // gain + deviation table
    let model_nodev = el_model.clone().with_field(&field, az_sign, 1.0, None);
    let mut az_moves = Vec::new();
    let (mut xa, mut ya) = (Vec::new(), Vec::new());
    for p in pts.iter().filter(|p| p.kind == "az" || p.kind == "sweep") {
        let a = model_nodev.az_eval(p.w.acc, p.w.mag).az;
        if p.daz.abs() <= 30.0 {
            xa.push(p.daz);
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
    let mut deviation: Vec<[f64; 2]> = Vec::new();
    let mut periodic = false;
    if swept {
        let sw: Vec<&Pt> = pts.iter().filter(|p| p.kind == "sweep" || p.kind == "zero").collect();
        for p in &sw {
            let a = model_nodev.az_eval(p.w.acc, p.w.mag).raw;
            deviation.push([(a * 100.0).round() / 100.0, (wrap180(a - p.daz) * 1000.0).round() / 1000.0]);
        }
        deviation.sort_by(|a, b| a[0].total_cmp(&b[0]));
        deviation.dedup_by(|a, b| (a[0] - b[0]).abs() < 0.5);
        let span = pts.iter().filter(|p| p.kind == "sweep").map(|p| p.daz).fold(f64::NEG_INFINITY, f64::max)
            - pts.iter().filter(|p| p.kind == "sweep").map(|p| p.daz).fold(f64::INFINITY, f64::min);
        periodic = span + o.sweep_step >= 359.0;
    }
    let dev_tbl = crate::math::DevTable::new(deviation.iter().map(|p| (p[0], p[1])).collect(), periodic);
    let model = el_model.clone().with_field(&field, az_sign, az_gain, dev_tbl);

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
    let _ = UP;
    let cfg = Config {
        version: 1,
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
        limits: ctx.lim.clone(),
        thresholds: ctx.th.clone(),
        timing: ctx.tm.clone(),
        sync: Sync::default(),
    };
    std::thread::sleep(Duration::from_millis(1));
    Ok(cfg)
}

fn round3(v: V3) -> [f64; 3] {
    v.0.map(|x| (x * 1000.0).round() / 1000.0)
}
