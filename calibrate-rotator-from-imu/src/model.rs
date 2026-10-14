//! Converts raw IMU vectors into "true" elevation / azimuth relative to the
//! calibration reference, using the measured mounting transform.
//!
//! Elevation: gravity (as seen by the IMU) rotates about the elevation axis
//! when the rotator elevates. True el = signed angle of the current up vector
//! from the reference up vector about that axis. Insensitive to azimuth and
//! to how the IMU is oriented on the boom.
//!
//! Azimuth: the magnetometer reading is modelled as
//!   m_imu = A · Mᵀ · Rot(k, el) · (Rz(sign·az) · e + c)  +  b
//! with e = Earth field at the zero pose (rotator frame), b = hard-iron
//! offset (fixed to the IMU: boom, cables), A = soft-iron / sensor-gain
//! matrix (fixed to the IMU), and c = field of iron and motors on the azimuth
//! stage, which turns with azimuth but not with elevation. A and c only show
//! up away from el=0, so they are fitted from the envelope survey. Given a
//! reading we undo b and A, undo elevation using the accelerometer, remove c,
//! and read az off the horizontal component. Note: the *zero* is where the
//! field matches the reference, so residual model error only distorts the
//! scale away from zero — closed-loop zeroing still converges to the true zero.

use crate::config::{Config, FieldModel};
use crate::math::{lstsq, signed_angle_deg, DevTable, M3, V3};

pub const UP: V3 = V3::new(0.0, 0.0, 1.0);

#[derive(Clone, Debug)]
pub struct Model {
    /// IMU -> rotator frame.
    pub m: M3,
    /// Elevation axis, rotator frame.
    pub k: V3,
    pub cone0: f64,
    pub el_gain: f64,
    pub b: V3,
    /// Inverse of the soft-iron matrix.
    pub a_inv: M3,
    pub e: V3,
    /// Azimuth-stage field (rotator frame at el=0).
    pub c: V3,
    pub az_sign: f64,
    pub az_gain: f64,
    pub dev: Option<DevTable>,
}

#[derive(Clone, Copy, Debug)]
pub struct AzEval {
    /// Model azimuth before deviation-table correction.
    pub raw: f64,
    /// Best estimate of true azimuth (deg, wrapped).
    pub az: f64,
    /// |horizontal field| / reference — should be ~1.
    pub h_ratio: f64,
    /// Vertical field minus reference (uT) — should be ~0.
    pub v_err: f64,
}

impl Model {
    pub fn el_only(m: M3, k_imu: V3, el_gain: f64) -> Model {
        let k = m.mul_v(k_imu).unit();
        Model {
            m,
            k,
            cone0: UP.angle_deg(k),
            el_gain,
            b: V3::default(),
            a_inv: M3::I,
            e: V3::new(1.0, 0.0, 0.0),
            c: V3::default(),
            az_sign: 1.0,
            az_gain: 1.0,
            dev: None,
        }
    }

    pub fn with_field(mut self, f: &FieldModel, sign: f64, gain: f64, dev: Option<DevTable>) -> Model {
        self.b = f.hard_iron_imu_ut;
        self.a_inv = M3(f.soft_iron_imu).inv().unwrap_or(M3::I);
        self.e = f.earth_rot_ut;
        self.c = f.stage_field_ut;
        self.az_sign = sign;
        self.az_gain = gain;
        self.dev = dev;
        self
    }

    pub fn from_config(c: &Config) -> Model {
        let m = M3(c.mount.imu_to_rotator);
        let dev = DevTable::new(
            c.azimuth.deviation.iter().map(|p| (p[0], p[1])).collect(),
            c.azimuth.deviation_periodic,
        );
        Model::el_only(m, c.mount.el_axis_imu, c.elevation.gain).with_field(
            &c.azimuth.field_model,
            c.azimuth.sign,
            c.azimuth.gain,
            dev,
        )
    }

    pub fn up(&self, acc_imu: V3) -> V3 {
        self.m.mul_v(acc_imu.unit())
    }

    /// True elevation (deg) from an accelerometer reading.
    pub fn el(&self, acc_imu: V3) -> f64 {
        signed_angle_deg(UP, self.up(acc_imu), self.k)
    }

    /// How far gravity has left the cone around the elevation axis (deg).
    /// Pure elevation motion keeps this ~0; a shifted IMU does not.
    pub fn off_axis(&self, acc_imu: V3) -> f64 {
        (self.up(acc_imu).angle_deg(self.k) - self.cone0).abs()
    }

    pub fn az_eval(&self, acc_imu: V3, mag_imu: V3) -> AzEval {
        let el = self.el(acc_imu);
        let c = M3::rot(self.k, -el).mul_v(self.m.mul_v(self.a_inv.mul_v(mag_imu - self.b))) - self.c;
        let (ex, ey) = (self.e.x(), self.e.y());
        let (cx, cy) = (c.x(), c.y());
        let raw = self.az_sign * (ex * cy - ey * cx).atan2(ex * cx + ey * cy).to_degrees();
        let corr = self.dev.as_ref().map(|d| d.eval(raw)).unwrap_or(0.0);
        let eh = ex.hypot(ey).max(1e-9);
        AzEval {
            raw,
            az: crate::math::wrap180(raw - corr),
            h_ratio: cx.hypot(cy) / eh,
            v_err: c.z() - self.e.z(),
        }
    }
}

/// One calibration observation for the magnetic fit.
#[derive(Clone, Copy, Debug)]
pub struct MagPoint {
    pub mag_imu: V3,
    /// Accelerometer-measured elevation (deg).
    pub el: f64,
    /// Rotator-reported azimuth relative to the zero pose (deg).
    pub daz: f64,
}

/// Which terms of the magnetic model to fit. Hard iron and the Earth field
/// are always fitted; the rest need data well away from el=0.
#[derive(Clone, Copy, Debug)]
pub struct FitTerms {
    pub soft_iron: bool,
    pub stage_field: bool,
}

pub const HARD_IRON_ONLY: FitTerms = FitTerms { soft_iron: false, stage_field: false };
pub const ALL_TERMS: FitTerms = FitTerms { soft_iron: true, stage_field: true };

/// Weak priors (A ≈ I, c ≈ 0) so poorly-constrained terms stay put instead
/// of making the fit singular, plus a firm trace(A) = 3 to fix the common
/// scale of A versus the fields.
const RIDGE_A: f64 = 5.0;
const RIDGE_C: f64 = 0.1;
const TRACE_W: f64 = 100.0;

/// Parameter vector: b(0..3) e(3..6) c(6..9) a00 a11 a22 a01 a02 a12 (9..15).
fn unpack(x: &[f64]) -> (V3, V3, V3, M3) {
    let a = M3([[x[9], x[12], x[13]], [x[12], x[10], x[14]], [x[13], x[14], x[11]]]);
    (V3([x[0], x[1], x[2]]), V3([x[3], x[4], x[5]]), V3([x[6], x[7], x[8]]), a)
}

/// Residuals and Jacobian of the full model (data rows, then prior rows).
fn eval(x: &[f64], pts: &[MagPoint], rq: &[(M3, M3)], terms: FitTerms) -> (Vec<f64>, Vec<Vec<f64>>) {
    let (b, e, c, a) = unpack(x);
    let (mut r, mut j) = (Vec::new(), Vec::new());
    for (p, (rm, q)) in pts.iter().zip(rq) {
        let f = q.mul_v(e) + rm.mul_v(c);
        let pred = a.mul_v(f) + b;
        let (aq, ar) = (a.mul(q), a.mul(rm));
        let [f0, f1, f2] = f.0;
        let da: [[f64; 6]; 3] = [
            [f0, 0.0, 0.0, f1, f2, 0.0],
            [0.0, f1, 0.0, f0, 0.0, f2],
            [0.0, 0.0, f2, 0.0, f0, f1],
        ];
        for i in 0..3 {
            let mut row = vec![0.0; 15];
            row[i] = 1.0;
            row[3..6].copy_from_slice(&aq.0[i]);
            if terms.stage_field {
                row[6..9].copy_from_slice(&ar.0[i]);
            }
            if terms.soft_iron {
                row[9..15].copy_from_slice(&da[i]);
            }
            r.push(pred.0[i] - p.mag_imu.0[i]);
            j.push(row);
        }
    }
    for i in 0..3 {
        let mut row = vec![0.0; 15];
        row[6 + i] = RIDGE_C;
        r.push(RIDGE_C * x[6 + i]);
        j.push(row);
    }
    for i in 0..6 {
        let mut row = vec![0.0; 15];
        row[9 + i] = RIDGE_A;
        r.push(RIDGE_A * (x[9 + i] - if i < 3 { 1.0 } else { 0.0 }));
        j.push(row);
    }
    let mut row = vec![0.0; 15];
    row[9..12].copy_from_slice(&[TRACE_W; 3]);
    r.push(TRACE_W * (x[9] + x[10] + x[11] - 3.0));
    j.push(row);
    (r, j)
}

/// Least-squares fit of the magnetic model. With hard iron only the model is
/// linear and solved exactly; the extra terms are refined from that start by
/// Levenberg–Marquardt, which only ever accepts cost-reducing steps.
pub fn fit_field(m: &M3, k: V3, pts: &[MagPoint], sign: f64, terms: FitTerms) -> Option<FieldModel> {
    let mt = m.t();
    // r = Mᵀ·Rot(k, el) (stage frame -> IMU), q = r·Rz(sign·az) (zero pose -> IMU)
    let rq: Vec<(M3, M3)> = pts
        .iter()
        .map(|p| {
            let r = mt.mul(&M3::rot(k, p.el));
            (r, r.mul(&M3::rz(sign * p.daz)))
        })
        .collect();
    let mut x = vec![0.0; 15];
    x[9..12].copy_from_slice(&[1.0; 3]);
    // start: hard iron + Earth field (+ stage field) with A = I, linear
    let free: Vec<usize> = (0..6).chain(if terms.stage_field { 6..9 } else { 0..0 }).collect();
    let solve_linear = |x: &mut Vec<f64>| -> Option<()> {
        let (r, j) = eval(x, pts, &rq, FitTerms { soft_iron: false, ..terms });
        let rows: Vec<Vec<f64>> = j.iter().map(|row| free.iter().map(|&c| row[c]).collect()).collect();
        let y: Vec<f64> = r.iter().map(|v| -v).collect();
        let (d, _) = lstsq(&rows, &y)?;
        for (&c, dv) in free.iter().zip(d) {
            x[c] += dv;
        }
        Some(())
    };
    solve_linear(&mut x)?;
    if terms.soft_iron {
        let cost = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>();
        let (mut r, mut j) = eval(&x, pts, &rq, terms);
        let mut c0 = cost(&r);
        let mut lambda = 1e-3;
        for _ in 0..200 {
            let mut diag = [0.0; 15];
            for row in &j {
                for (d, v) in diag.iter_mut().zip(row) {
                    *d += v * v;
                }
            }
            let mut rows = j.clone();
            let mut y: Vec<f64> = r.iter().map(|v| -v).collect();
            for (i, d) in diag.iter().enumerate() {
                let mut row = vec![0.0; 15];
                row[i] = (lambda * d.max(1e-9)).sqrt();
                rows.push(row);
                y.push(0.0);
            }
            let Some((delta, _)) = lstsq(&rows, &y) else { break };
            let xn: Vec<f64> = x.iter().zip(&delta).map(|(a, b)| a + b).collect();
            let (rn, jn) = eval(&xn, pts, &rq, terms);
            let c1 = cost(&rn);
            if c1 < c0 {
                let done = c0 - c1 < 1e-12 * c0.max(1e-30);
                (x, r, j, c0) = (xn, rn, jn, c1);
                lambda = (lambda / 3.0).max(1e-12);
                if done {
                    break;
                }
            } else {
                lambda *= 4.0;
                if lambda > 1e8 {
                    break;
                }
            }
        }
    }
    let (b, e, c, a) = unpack(&x);
    // a mirror-like or collapsed soft-iron matrix is not physical
    if (0..3).any(|i| !(0.5..=1.5).contains(&a.0[i][i])) || a.det() < 0.3 {
        return None;
    }
    let ss: f64 = pts
        .iter()
        .zip(&rq)
        .map(|(p, (r, q))| {
            let d = a.mul_v(q.mul_v(e) + r.mul_v(c)) + b - p.mag_imu;
            d.dot(d)
        })
        .sum();
    Some(FieldModel {
        hard_iron_imu_ut: b,
        soft_iron_imu: a.0,
        earth_rot_ut: e,
        stage_field_ut: c,
        fit_rms_ut: (ss / (3 * pts.len()).max(1) as f64).sqrt(),
        points: pts.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthetic IMU mounted upside-down and yawed, with hard iron.
    #[test]
    fn recovers_az_el_through_mount_and_hard_iron() {
        // IMU orientation in the rotator frame == imu->rotator transform.
        let m = M3::rz(37.0).mul(&M3::ry(4.0)).mul(&M3::rx(178.0));
        let b = V3::new(12.0, -30.0, 8.0);
        let e = V3::new(15.5, 3.7, -53.7);
        let k_rot = V3::new(0.0, 1.0, 0.0);
        let meas = |az: f64, el: f64| {
            let r = M3::rot(k_rot, el).mul(&M3::rz(az));
            let acc = m.t().mul_v(r.mul_v(UP));
            let mag = m.t().mul_v(r.mul_v(e)) + b;
            (acc, mag)
        };
        let k_imu = m.t().mul_v(k_rot);
        let pts: Vec<MagPoint> = [(0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, 10.0), (0.0, -10.0)]
            .iter()
            .map(|&(az, el)| MagPoint { mag_imu: meas(az, el).1, el, daz: az })
            .collect();
        let f = fit_field(&m, k_rot, &pts, 1.0, HARD_IRON_ONLY).unwrap();
        assert!((f.hard_iron_imu_ut - b).norm() < 1e-6, "{:?}", f);
        let model = Model::el_only(m, k_imu, 1.0).with_field(&f, 1.0, 1.0, None);
        for &(az, el) in &[(25.0, 3.0), (-140.0, 7.5), (90.0, -2.0)] {
            let (acc, mag) = meas(az, el);
            assert!((model.el(acc) - el).abs() < 1e-6);
            let a = model.az_eval(acc, mag);
            assert!((a.az - az).abs() < 1e-6, "az {az} got {:?}", a);
            assert!((a.h_ratio - 1.0).abs() < 1e-9 && a.v_err.abs() < 1e-9);
            assert!(model.off_axis(acc) < 1e-6);
        }
    }

    /// Soft iron and an azimuth-stage field only show up away from el=0; a
    /// survey over the envelope must recover them.
    #[test]
    fn full_fit_recovers_soft_iron_and_stage_field() {
        let m = M3::rz(-63.0).mul(&M3::ry(-2.0)).mul(&M3::rx(181.0));
        let b = V3::new(-20.0, 25.0, 11.0);
        let a = M3([[1.06, 0.03, -0.04], [0.03, 0.97, 0.05], [-0.04, 0.05, 0.97]]);
        let c = V3::new(6.0, -7.0, 4.0);
        let e = V3::new(14.0, -6.0, -53.7);
        let k_rot = V3::new(0.0, 1.0, 0.0);
        let meas = |az: f64, el: f64| {
            let r = M3::rot(k_rot, el);
            let acc = m.t().mul_v(r.mul(&M3::rz(az)).mul_v(UP));
            let mag = a.mul_v(m.t().mul_v(r.mul_v(M3::rz(az).mul_v(e) + c))) + b;
            (acc, mag)
        };
        let mut pts = Vec::new();
        for el in [0.0, 20.0, 40.0, 60.0, 80.0] {
            for i in 0..12 {
                let az = 30.0 * i as f64;
                pts.push(MagPoint { mag_imu: meas(az, el).1, el, daz: az });
            }
        }
        let f = fit_field(&m, k_rot, &pts, 1.0, ALL_TERMS).unwrap();
        assert!(f.fit_rms_ut < 0.01, "{f:?}");
        let model = Model::el_only(m, m.t().mul_v(k_rot), 1.0).with_field(&f, 1.0, 1.0, None);
        for &(az, el) in &[(40.0, 40.0), (-150.0, 75.0), (10.0, 5.0)] {
            let (acc, mag) = meas(az, el);
            let r = model.az_eval(acc, mag);
            assert!(crate::math::wrap180(r.az - az).abs() < 0.05, "az {az} el {el} got {r:?}");
            assert!((r.h_ratio - 1.0).abs() < 1e-3, "{r:?}");
        }
        // the hard-iron-only model is what failed at el=40 on real hardware
        let mut near_zero = pts[..12].to_vec();
        near_zero.push(MagPoint { mag_imu: meas(0.0, 10.0).1, el: 10.0, daz: 0.0 });
        let f0 = fit_field(&m, k_rot, &near_zero, 1.0, HARD_IRON_ONLY).unwrap();
        let m0 = Model::el_only(m, m.t().mul_v(k_rot), 1.0).with_field(&f0, 1.0, 1.0, None);
        let (acc, mag) = meas(40.0, 40.0);
        assert!((m0.az_eval(acc, mag).h_ratio - 1.0).abs() > 0.1);
    }
}
