//! Small, dependency-free 3-D linear algebra and robust statistics.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct V3(pub [f64; 3]);

impl V3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        V3([x, y, z])
    }
    pub fn x(self) -> f64 {
        self.0[0]
    }
    pub fn y(self) -> f64 {
        self.0[1]
    }
    pub fn z(self) -> f64 {
        self.0[2]
    }
    pub fn dot(self, o: V3) -> f64 {
        self.0[0] * o.0[0] + self.0[1] * o.0[1] + self.0[2] * o.0[2]
    }
    pub fn cross(self, o: V3) -> V3 {
        let [a, b, c] = self.0;
        let [d, e, f] = o.0;
        V3([b * f - c * e, c * d - a * f, a * e - b * d])
    }
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn unit(self) -> V3 {
        let n = self.norm();
        if n < 1e-15 {
            self
        } else {
            self / n
        }
    }
    /// Unsigned angle between two vectors, degrees.
    pub fn angle_deg(self, o: V3) -> f64 {
        self.cross(o).norm().atan2(self.dot(o)).to_degrees()
    }
    /// Component perpendicular to unit vector `n`.
    pub fn perp(self, n: V3) -> V3 {
        self - n * self.dot(n)
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        V3([self.0[0] + o.0[0], self.0[1] + o.0[1], self.0[2] + o.0[2]])
    }
}
impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        V3([self.0[0] - o.0[0], self.0[1] - o.0[1], self.0[2] - o.0[2]])
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, s: f64) -> V3 {
        V3([self.0[0] * s, self.0[1] * s, self.0[2] * s])
    }
}
impl Div<f64> for V3 {
    type Output = V3;
    fn div(self, s: f64) -> V3 {
        V3([self.0[0] / s, self.0[1] / s, self.0[2] / s])
    }
}
impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        V3([-self.0[0], -self.0[1], -self.0[2]])
    }
}

/// Row-major 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct M3(pub [[f64; 3]; 3]);

impl M3 {
    pub const I: M3 = M3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    pub fn from_rows(a: V3, b: V3, c: V3) -> M3 {
        M3([a.0, b.0, c.0])
    }
    pub fn mul_v(&self, v: V3) -> V3 {
        let m = &self.0;
        V3([
            m[0][0] * v.0[0] + m[0][1] * v.0[1] + m[0][2] * v.0[2],
            m[1][0] * v.0[0] + m[1][1] * v.0[1] + m[1][2] * v.0[2],
            m[2][0] * v.0[0] + m[2][1] * v.0[1] + m[2][2] * v.0[2],
        ])
    }
    pub fn t(&self) -> M3 {
        let m = &self.0;
        M3([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }
    pub fn mul(&self, o: &M3) -> M3 {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| self.0[i][k] * o.0[k][j]).sum();
            }
        }
        M3(r)
    }
    pub fn det(&self) -> f64 {
        let m = &self.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }
    pub fn inv(&self) -> Option<M3> {
        let d = self.det();
        if d.abs() < 1e-12 {
            return None;
        }
        let m = &self.0;
        let c = |r0: usize, r1: usize, c0: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
        Some(M3([
            [c(1, 2, 1, 2) / d, -c(0, 2, 1, 2) / d, c(0, 1, 1, 2) / d],
            [-c(1, 2, 0, 2) / d, c(0, 2, 0, 2) / d, -c(0, 1, 0, 2) / d],
            [c(1, 2, 0, 1) / d, -c(0, 2, 0, 1) / d, c(0, 1, 0, 1) / d],
        ]))
    }
    /// Rotation of `deg` degrees about unit `axis` (right-hand rule).
    pub fn rot(axis: V3, deg: f64) -> M3 {
        let a = axis.unit();
        let (s, c) = deg.to_radians().sin_cos();
        let t = 1.0 - c;
        let [x, y, z] = a.0;
        M3([
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ])
    }
    pub fn rx(deg: f64) -> M3 {
        M3::rot(V3::new(1.0, 0.0, 0.0), deg)
    }
    pub fn ry(deg: f64) -> M3 {
        M3::rot(V3::new(0.0, 1.0, 0.0), deg)
    }
    pub fn rz(deg: f64) -> M3 {
        M3::rot(V3::new(0.0, 0.0, 1.0), deg)
    }
    /// Decompose as Rz(yaw)·Ry(pitch)·Rx(roll); returns [yaw, pitch, roll] in degrees.
    pub fn euler_zyx_deg(&self) -> [f64; 3] {
        let m = &self.0;
        let pitch = (-m[2][0]).clamp(-1.0, 1.0).asin();
        let roll = m[2][1].atan2(m[2][2]);
        let yaw = m[1][0].atan2(m[0][0]);
        [yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees()]
    }
}

/// Signed angle (degrees) rotating `a` onto `b` about unit axis `n`,
/// after projecting both onto the plane perpendicular to `n`.
pub fn signed_angle_deg(a: V3, b: V3, n: V3) -> f64 {
    let ap = a.perp(n);
    let bp = b.perp(n);
    n.dot(ap.cross(bp)).atan2(ap.dot(bp)).to_degrees()
}

/// Wrap an angle to [-180, 180).
pub fn wrap180(a: f64) -> f64 {
    (a + 180.0).rem_euclid(360.0) - 180.0
}

/// Linear least squares via normal equations with partial pivoting.
/// Returns (solution, rms residual), or None if the system is singular.
pub fn lstsq(rows: &[Vec<f64>], y: &[f64]) -> Option<(Vec<f64>, f64)> {
    let n = rows.first()?.len();
    if rows.len() < n {
        return None;
    }
    let mut a = vec![vec![0.0; n + 1]; n];
    for (r, &yi) in rows.iter().zip(y) {
        for i in 0..n {
            for j in 0..n {
                a[i][j] += r[i] * r[j];
            }
            a[i][n] += r[i] * yi;
        }
    }
    let maxd = (0..n).map(|i| a[i][i].abs()).fold(0.0, f64::max).max(1e-300);
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-10 * maxd {
            return None;
        }
        a.swap(c, p);
        for r in 0..n {
            if r != c {
                let f = a[r][c] / a[c][c];
                let pivot_row = a[c].clone();
                for (x, p) in a[r][c..=n].iter_mut().zip(&pivot_row[c..=n]) {
                    *x -= f * p;
                }
            }
        }
    }
    let x: Vec<f64> = (0..n).map(|i| a[i][n] / a[i][i]).collect();
    let ss: f64 = rows
        .iter()
        .zip(y)
        .map(|(r, &yi)| {
            let p: f64 = r.iter().zip(&x).map(|(a, b)| a * b).sum();
            (p - yi).powi(2)
        })
        .sum();
    Some((x, (ss / rows.len() as f64).sqrt()))
}

pub fn median(v: &[f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    let n = s.len();
    if n % 2 == 1 {
        s[n / 2]
    } else {
        0.5 * (s[n / 2 - 1] + s[n / 2])
    }
}

pub fn mean_std(v: &[f64]) -> (f64, f64) {
    let n = v.len() as f64;
    if v.is_empty() {
        return (f64::NAN, f64::NAN);
    }
    let m = v.iter().sum::<f64>() / n;
    let var = if v.len() > 1 {
        v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (n - 1.0)
    } else {
        0.0
    };
    (m, var.sqrt())
}

/// Circular mean and circular standard deviation, degrees.
pub fn circ_mean_std_deg(v: &[f64]) -> (f64, f64) {
    let (mut s, mut c) = (0.0, 0.0);
    for a in v {
        let r = a.to_radians();
        s += r.sin();
        c += r.cos();
    }
    let n = v.len().max(1) as f64;
    let r = ((s / n).powi(2) + (c / n).powi(2)).sqrt().clamp(1e-12, 1.0);
    (s.atan2(c).to_degrees(), (-2.0 * r.ln()).sqrt().to_degrees())
}

/// Least-squares slope of y = k·x through the origin.
pub fn slope_through_origin(x: &[f64], y: &[f64]) -> f64 {
    let sxy: f64 = x.iter().zip(y).map(|(a, b)| a * b).sum();
    let sxx: f64 = x.iter().map(|a| a * a).sum();
    if sxx < 1e-12 {
        0.0
    } else {
        sxy / sxx
    }
}

/// Outlier-resistant mean of vectors: drop samples far from the
/// component-wise median (MAD-based), then average.
/// Returns (mean, max per-axis std of kept samples, kept count).
pub fn robust_mean_v3(v: &[V3]) -> (V3, f64, usize) {
    if v.is_empty() {
        return (V3::default(), f64::NAN, 0);
    }
    let med = V3([
        median(&v.iter().map(|p| p.0[0]).collect::<Vec<_>>()),
        median(&v.iter().map(|p| p.0[1]).collect::<Vec<_>>()),
        median(&v.iter().map(|p| p.0[2]).collect::<Vec<_>>()),
    ]);
    let d: Vec<f64> = v.iter().map(|p| (*p - med).norm()).collect();
    let mad = median(&d) * 1.4826;
    let lim = (3.5 * mad).max(1e-9) + 1e-12;
    let kept: Vec<V3> = v
        .iter()
        .zip(&d)
        .filter(|(_, &di)| di <= lim)
        .map(|(p, _)| *p)
        .collect();
    let kept = if kept.len() < 3 { v.to_vec() } else { kept };
    let n = kept.len() as f64;
    let mean = kept.iter().fold(V3::default(), |a, b| a + *b) / n;
    let mut sd: f64 = 0.0;
    for i in 0..3 {
        let comp: Vec<f64> = kept.iter().map(|p| p.0[i]).collect();
        sd = sd.max(mean_std(&comp).1);
    }
    (mean, sd, kept.len())
}

/// Unit normal of the best-fit plane through `pts` (the direction of least
/// spread), oriented to agree with `hint`. Points on a circle (gravity
/// rotating about the elevation axis) give that axis.
pub fn plane_normal(pts: &[V3], hint: V3) -> Option<V3> {
    if pts.len() < 3 {
        return None;
    }
    let mean = pts.iter().fold(V3::default(), |a, p| a + *p) / pts.len() as f64;
    let mut c = [[0.0; 3]; 3];
    for p in pts {
        let d = *p - mean;
        for (i, row) in c.iter_mut().enumerate() {
            for (j, x) in row.iter_mut().enumerate() {
                *x += d.0[i] * d.0[j];
            }
        }
    }
    let tr = c[0][0] + c[1][1] + c[2][2];
    for (i, row) in c.iter_mut().enumerate() {
        row[i] += 1e-9 * tr.max(1e-300);
    }
    // inverse power iteration -> eigenvector of the smallest eigenvalue
    let inv = M3(c).inv()?;
    let mut v = hint.unit();
    for _ in 0..100 {
        v = inv.mul_v(v).unit();
    }
    Some(if v.dot(hint) < 0.0 { -v } else { v })
}

/// Piecewise-linear correction table keyed by measured azimuth.
#[derive(Clone, Debug)]
pub struct DevTable {
    pts: Vec<(f64, f64)>,
    periodic: bool,
}

impl DevTable {
    pub fn new(mut pts: Vec<(f64, f64)>, periodic: bool) -> Option<Self> {
        if pts.len() < 2 {
            return None;
        }
        pts.sort_by(|a, b| a.0.total_cmp(&b.0));
        Some(DevTable { pts, periodic })
    }
    pub fn eval(&self, x: f64) -> f64 {
        let p = &self.pts;
        let n = p.len();
        let x = wrap180(x);
        if x <= p[0].0 || x >= p[n - 1].0 {
            if !self.periodic {
                return if x <= p[0].0 { p[0].1 } else { p[n - 1].1 };
            }
            // interpolate across the wrap between last and first point
            let (x0, y0) = p[n - 1];
            let (x1, y1) = (p[0].0 + 360.0, p[0].1);
            let xx = if x < x0 { x + 360.0 } else { x };
            let t = if x1 - x0 < 1e-9 { 0.0 } else { (xx - x0) / (x1 - x0) };
            return y0 + t * (y1 - y0);
        }
        for w in p.windows(2) {
            if x >= w[0].0 && x <= w[1].0 {
                let t = (x - w[0].0) / (w[1].0 - w[0].0).max(1e-9);
                return w[0].1 + t * (w[1].1 - w[0].1);
            }
        }
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_angle_about_axis() {
        let z = V3::new(0.0, 0.0, 1.0);
        let y = V3::new(0.0, 1.0, 0.0);
        let r = M3::rot(y, 10.0).mul_v(z);
        assert!((signed_angle_deg(z, r, y) - 10.0).abs() < 1e-9);
        assert!((signed_angle_deg(r, z, y) + 10.0).abs() < 1e-9);
    }

    #[test]
    fn euler_roundtrip() {
        let m = M3::rz(30.0).mul(&M3::ry(-12.0)).mul(&M3::rx(170.0));
        let e = m.euler_zyx_deg();
        assert!((e[0] - 30.0).abs() < 1e-9);
        assert!((e[1] + 12.0).abs() < 1e-9);
        assert!((e[2] - 170.0).abs() < 1e-9);
    }

    #[test]
    fn inverse_and_plane_normal() {
        let m = M3([[1.1, 0.05, -0.02], [0.05, 0.93, 0.04], [-0.02, 0.04, 1.02]]);
        let p = m.mul(&m.inv().unwrap());
        for i in 0..3 {
            for j in 0..3 {
                assert!((p.0[i][j] - M3::I.0[i][j]).abs() < 1e-12);
            }
        }
        let k = V3::new(0.1, 0.98, -0.05).unit();
        let u0 = k.cross(V3::new(1.0, 0.0, 0.0)).unit();
        let pts: Vec<V3> = (0..8).map(|i| M3::rot(k, 10.0 * i as f64).mul_v(u0)).collect();
        let n = plane_normal(&pts, V3::new(0.0, 1.0, 0.0)).unwrap();
        assert!((n - k).norm() < 1e-9, "{n:?}");
    }

    #[test]
    fn lstsq_line() {
        let rows: Vec<Vec<f64>> = (0..10).map(|i| vec![1.0, i as f64]).collect();
        let y: Vec<f64> = (0..10).map(|i| 3.0 + 2.0 * i as f64).collect();
        let (x, rms) = lstsq(&rows, &y).unwrap();
        assert!((x[0] - 3.0).abs() < 1e-9 && (x[1] - 2.0).abs() < 1e-9 && rms < 1e-9);
    }

    #[test]
    fn wrap() {
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-180.0), -180.0);
        assert!((wrap180(359.5) + 0.5).abs() < 1e-12);
    }

    #[test]
    fn devtable_periodic() {
        let t = DevTable::new(vec![(-170.0, 1.0), (0.0, 0.0), (170.0, 3.0)], true).unwrap();
        assert!((t.eval(180.0) - 2.0).abs() < 1e-9);
        assert!((t.eval(85.0) - 1.5).abs() < 1e-9);
    }
}
