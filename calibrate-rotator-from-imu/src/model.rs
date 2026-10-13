//! Converts raw IMU vectors into "true" elevation / azimuth relative to the
//! calibration reference, using the measured mounting transform.
//!
//! Elevation: gravity (as seen by the IMU) rotates about the elevation axis
//! when the rotator elevates. True el = signed angle of the current up vector
//! from the reference up vector about that axis. Insensitive to azimuth and
//! to how the IMU is oriented on the boom.
//!
//! Azimuth: the magnetometer reading is modelled as
//!   m_imu = Mᵀ · Rot(k, el) · Rz(sign·az) · e  +  b
//! with e = Earth field at the zero pose (rotator frame) and b = hard-iron
//! offset (fixed to the IMU: boom, motors, cables). Given a reading we remove
//! b, undo elevation using the accelerometer, and read az off the horizontal
//! component. Note: the *zero* is where the field matches the reference, so
//! residual model error only distorts the scale away from zero — closed-loop
//! zeroing still converges to the true zero.

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
    #[allow(dead_code)]
    pub el_gain: f64,
    pub b: V3,
    pub e: V3,
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
            e: V3::new(1.0, 0.0, 0.0),
            az_sign: 1.0,
            az_gain: 1.0,
            dev: None,
        }
    }

    pub fn with_field(mut self, f: &FieldModel, sign: f64, gain: f64, dev: Option<DevTable>) -> Model {
        self.b = f.hard_iron_imu_ut;
        self.e = f.earth_rot_ut;
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
        let c = M3::rot(self.k, -el).mul_v(self.m.mul_v(mag_imu - self.b));
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

/// Linear least-squares fit of hard-iron offset (3) and Earth field (3).
pub fn fit_field(m: &M3, k: V3, pts: &[MagPoint], sign: f64) -> Option<FieldModel> {
    let mt = m.t();
    let mut rows = Vec::new();
    let mut y = Vec::new();
    for p in pts {
        let q = mt.mul(&M3::rot(k, p.el)).mul(&M3::rz(sign * p.daz));
        for j in 0..3 {
            let mut r = vec![0.0; 6];
            r[j] = 1.0;
            r[3..6].copy_from_slice(&q.0[j]);
            rows.push(r);
            y.push(p.mag_imu.0[j]);
        }
    }
    let (x, rms) = lstsq(&rows, &y)?;
    Some(FieldModel {
        hard_iron_imu_ut: V3([x[0], x[1], x[2]]),
        earth_rot_ut: V3([x[3], x[4], x[5]]),
        fit_rms_ut: rms,
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
        let f = fit_field(&m, k_rot, &pts, 1.0).unwrap();
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
}
