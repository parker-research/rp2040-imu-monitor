//! In-process simulator: a fake `rotctld` on a local TCP port plus an IMU
//! emitting the same JSONL as the real firmware. The IMU is mounted with a
//! random (seeded) orientation, hard-iron offset, non-level rotator base,
//! Calgary-like Earth field and motor magnetic interference while moving.
//! Faults can be injected to exercise the mechanical-link checks.

use crate::math::{M3, V3};
use crate::sample::Sample;
use clap::ValueEnum;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::mpsc::{channel, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum Fault {
    None,
    /// IMU falls off and stops moving with the antenna.
    ImuDetached,
    /// Mast slips in the azimuth clamp: rotator turns, antenna doesn't.
    MastSlip,
    /// Elevation gear stripped: rotator reports motion, antenna doesn't tilt.
    ElStuck,
    /// IMU knocked 4° askew on its mount before the run (static fault).
    ImuShifted,
}

#[derive(Clone, Debug)]
pub struct SimOpts {
    pub seed: u64,
    pub speed: f64,
    pub offset_az: f64,
    pub offset_el: f64,
    pub start_az: Option<f64>,
    pub start_el: Option<f64>,
    pub fault: Fault,
    pub fault_after: u32,
}

struct World {
    az: f64,
    el: f64,
    taz: f64,
    tel: f64,
    sets: u32,
    off_az: f64,
    off_el: f64,
    /// Controller is rebooting until this instant.
    rebooting_until: Option<std::time::Instant>,
    ant_az_frozen: Option<f64>,
    ant_el_frozen: Option<f64>,
    imu_frozen: Option<M3>,
}

struct Fixed {
    /// IMU orientation in the antenna frame (imu -> antenna).
    mount: M3,
    base: M3,
    hard_iron: V3,
    field: V3,
    motor: V3,
    deadband: f64,
    az_dps: f64,
    el_dps: f64,
}

pub struct Sim {
    pub rotctld_addr: String,
    pub lines: Option<Receiver<String>>,
}

const AZ_LIM: (f64, f64) = (-180.0, 540.0);
const EL_LIM: (f64, f64) = (-20.0, 180.0);

impl Fixed {
    fn new(seed: u64) -> Fixed {
        let mut r = StdRng::seed_from_u64(seed);
        let n = Normal::new(0.0, 1.0).unwrap();
        let mount = M3::rz(r.gen_range(-180.0..180.0))
            .mul(&M3::ry(3.0 * n.sample(&mut r)))
            .mul(&M3::rx(180.0 + 3.0 * n.sample(&mut r)));
        let base = M3::rx(0.4 * n.sample(&mut r)).mul(&M3::ry(0.4 * n.sample(&mut r)));
        let hard_iron = V3::new(n.sample(&mut r), n.sample(&mut r), n.sample(&mut r)).unit() * 35.0;
        // Calgary-ish: total 56.5 uT, inclination 73.5°, declination 13.5° E.
        // World frame: x = north, y = west, z = up.
        let (f, inc, dec) = (56.5f64, 73.5f64.to_radians(), 13.5f64.to_radians());
        let h = f * inc.cos();
        let field = V3::new(h * dec.cos(), -h * dec.sin(), -f * inc.sin());
        let motor = V3::new(n.sample(&mut r), n.sample(&mut r), n.sample(&mut r)) * 0.8;
        Fixed { mount, base, hard_iron, field, motor, deadband: 0.4, az_dps: 6.0, el_dps: 3.0 }
    }

    /// World -> IMU rotation for an antenna at (az, el).
    fn world_to_imu(&self, az: f64, el: f64) -> M3 {
        // Hamlib azimuth is clockwise from above = negative rotation about up.
        let a = self.base.mul(&M3::rz(-az)).mul(&M3::ry(-el));
        self.mount.t().mul(&a.t())
    }
}

fn fw_angles(a: V3, m: V3) -> (f64, f64, f64) {
    let roll = a.y().atan2(a.z());
    let pitch = (-a.x()).atan2(a.y().hypot(a.z()));
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let xh = m.x() * cp + m.y() * sp * sr + m.z() * sp * cr;
    let yh = m.y() * cr - m.z() * sr;
    let hdg = (-yh).atan2(xh).to_degrees().rem_euclid(360.0);
    (roll.to_degrees(), pitch.to_degrees(), hdg)
}

pub fn start(o: &SimOpts) -> std::io::Result<Sim> {
    let mut fixed = Fixed::new(o.seed);
    if o.fault == Fault::ImuShifted {
        fixed.mount = fixed.mount.mul(&M3::rx(4.0));
    }
    let fx = Arc::new(fixed);
    let (az0, el0) = (o.start_az.unwrap_or(o.offset_az) - o.offset_az, o.start_el.unwrap_or(o.offset_el) - o.offset_el);
    let w = Arc::new(Mutex::new(World {
        az: az0,
        el: el0,
        taz: az0,
        tel: el0,
        sets: 0,
        ant_az_frozen: None,
        ant_el_frozen: None,
        imu_frozen: None,
        off_az: o.offset_az,
        off_el: o.offset_el,
        rebooting_until: None,
    }));
    let speed = o.speed.max(0.1);

    // physics
    {
        let (w, fx) = (w.clone(), fx.clone());
        thread::spawn(move || loop {
            thread::sleep(Duration::from_millis(5));
            let dt = 0.005 * speed;
            let mut s = w.lock().unwrap();
            let step = |p: f64, t: f64, v: f64| if (t - p).abs() <= v * dt { t } else { p + (t - p).signum() * v * dt };
            s.az = step(s.az, s.taz, fx.az_dps);
            s.el = step(s.el, s.tel, fx.el_dps);
        });
    }

    // IMU emitter
    let (tx, rx) = channel::<String>();
    {
        let (w, fx) = (w.clone(), fx.clone());
        let mut r = StdRng::seed_from_u64(o.seed.wrapping_add(0x5eed));
        thread::spawn(move || {
            let na = Normal::new(0.0, 0.003).unwrap();
            let nm = Normal::new(0.0, 0.35).unwrap();
            let mut seq = 0u64;
            loop {
                thread::sleep(Duration::from_secs_f64(0.25 / speed));
                let (wi, moving) = {
                    let s = w.lock().unwrap();
                    let az = s.ant_az_frozen.unwrap_or(s.az);
                    let el = s.ant_el_frozen.unwrap_or(s.el);
                    let wi = s.imu_frozen.unwrap_or_else(|| fx.world_to_imu(az, el));
                    (wi, (s.az - s.taz).abs() > 1e-6 || (s.el - s.tel).abs() > 1e-6)
                };
                let acc = wi.mul_v(V3::new(0.0, 0.0, 1.0))
                    + V3::new(na.sample(&mut r), na.sample(&mut r), na.sample(&mut r));
                let mut mag = wi.mul_v(fx.field)
                    + fx.hard_iron
                    + V3::new(nm.sample(&mut r), nm.sample(&mut r), nm.sample(&mut r));
                if moving {
                    mag = mag + fx.motor;
                }
                let (roll, pitch, hdg) = fw_angles(acc, mag);
                seq += 1;
                let rnd = |x: f64, d: f64| (x * d).round() / d;
                let s = Sample {
                    kind: "sample".into(),
                    t_ms: seq * 250,
                    seq,
                    health: "ok".into(),
                    sensor: Some("LSM303DLH-sim".into()),
                    read_ok: true,
                    acc_raw: Some(acc.0.map(|x| (x * 1000.0).round() as i64)),
                    acc_g: Some(acc.0.map(|x| rnd(x, 1e4))),
                    acc_mag_g: Some(rnd(acc.norm(), 1e4)),
                    mag_raw: Some(mag.0.map(|x| (x / 0.0947).round() as i64)),
                    mag_ut: Some(mag.0.map(|x| rnd(x, 100.0))),
                    mag_mag_ut: Some(rnd(mag.norm(), 100.0)),
                    roll_deg: Some(rnd(roll, 100.0)),
                    pitch_deg: Some(rnd(pitch, 100.0)),
                    heading_deg: Some(rnd(hdg, 100.0)),
                    imu_temp_c: None,
                    imu_temp_raw: None,
                    mcu_temp_c: Some(26.5 + r.gen_range(-0.2..0.2)),
                    data_frozen: false,
                    plausible: true,
                    i2c_err: 0,
                    fail_streak: 0,
                    reinit: 0,
                    cfg_restore: 0,
                    wdt_reboot: false,
                    interval_ms: Some(250),
                };
                if tx.send(serde_json::to_string(&s).unwrap()).is_err() {
                    return;
                }
            }
        });
    }

    // fake rotctld
    let listener = TcpListener::bind("127.0.0.1:0")?;
    let addr = listener.local_addr()?.to_string();
    let (fault, after) = (o.fault, o.fault_after);
    thread::spawn(move || {
        for conn in listener.incoming().flatten() {
            let (w, fx) = (w.clone(), fx.clone());
            thread::spawn(move || {
                let mut out = conn.try_clone().unwrap();
                for line in BufReader::new(conn).lines().map_while(Result::ok) {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    let booting = {
                        let mut s = w.lock().unwrap();
                        match s.rebooting_until {
                            Some(t) if std::time::Instant::now() < t => true,
                            Some(_) => {
                                s.rebooting_until = None;
                                false
                            }
                            None => false,
                        }
                    };
                    if booting && parts.first() != Some(&"q") {
                        // rotctld can't reach the controller while it boots
                        if out.write_all(b"RPRT -6\n").is_err() {
                            return;
                        }
                        continue;
                    }
                    let reply = match parts.first().copied() {
                        Some("w") | Some("\\send_cmd") if parts.get(1) == Some(&"[ESP444]RESTART") => {
                            // ESP32 reboots; on boot it takes the current position as 0/0.
                            let mut s = w.lock().unwrap();
                            s.taz = s.az;
                            s.tel = s.el;
                            s.off_az = -s.az;
                            s.off_el = -s.el;
                            s.rebooting_until =
                                Some(std::time::Instant::now() + Duration::from_secs_f64(2.0 / speed));
                            // no reply from a rebooting controller -> rotctld times out
                            "RPRT -5\n".to_string()
                        }
                        Some("w") | Some("\\send_cmd") => "RPRT -11\n".to_string(),
                        Some("p") | Some("\\get_pos") => {
                            let s = w.lock().unwrap();
                            format!("{:.6}\n{:.6}\n", s.az + s.off_az, s.el + s.off_el)
                        }
                        Some("P") | Some("\\set_pos") if parts.len() == 3 => {
                            match (parts[1].parse::<f64>(), parts[2].parse::<f64>()) {
                                (Ok(a), Ok(e))
                                    if (AZ_LIM.0..=AZ_LIM.1).contains(&a) && (EL_LIM.0..=EL_LIM.1).contains(&e) =>
                                {
                                    let mut s = w.lock().unwrap();
                                    let (ta, te) = (a - s.off_az, e - s.off_el);
                                    if (ta - s.az).abs() >= fx.deadband {
                                        s.taz = ta;
                                    }
                                    if (te - s.el).abs() >= fx.deadband {
                                        s.tel = te;
                                    }
                                    s.sets += 1;
                                    if s.sets > after {
                                        match fault {
                                            Fault::ImuDetached if s.imu_frozen.is_none() => {
                                                let m = fx.world_to_imu(s.az, s.el);
                                                s.imu_frozen = Some(m);
                                            }
                                            Fault::MastSlip if s.ant_az_frozen.is_none() => s.ant_az_frozen = Some(s.az),
                                            Fault::ElStuck if s.ant_el_frozen.is_none() => s.ant_el_frozen = Some(s.el),
                                            _ => {}
                                        }
                                    }
                                    "RPRT 0\n".to_string()
                                }
                                _ => "RPRT -1\n".to_string(),
                            }
                        }
                        Some("S") | Some("\\stop") => {
                            let mut s = w.lock().unwrap();
                            s.taz = s.az;
                            s.tel = s.el;
                            "RPRT 0\n".to_string()
                        }
                        Some("_") => "calibrate-rotator-from-imu simulator\n".to_string(),
                        Some("q") => return,
                        _ => "RPRT -4\n".to_string(),
                    };
                    if out.write_all(reply.as_bytes()).is_err() {
                        return;
                    }
                }
            });
        }
    });
    Ok(Sim { rotctld_addr: addr, lines: Some(rx) })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rotctl::Rotctl;

    #[test]
    fn rotctld_protocol_roundtrip() {
        let o = SimOpts {
            seed: 1,
            speed: 50.0,
            offset_az: 3.0,
            offset_el: -1.0,
            start_az: None,
            start_el: None,
            fault: Fault::None,
            fault_after: 0,
        };
        let sim = start(&o).unwrap();
        let mut r = Rotctl::connect(&sim.rotctld_addr).unwrap();
        let (az, el) = r.get_pos().unwrap();
        assert!((az - 3.0).abs() < 1e-6 && (el + 1.0).abs() < 1e-6);
        r.set_pos(13.0, 4.0).unwrap();
        std::thread::sleep(Duration::from_millis(300));
        let (az, el) = r.get_pos().unwrap();
        assert!((az - 13.0).abs() < 1e-6 && (el - 4.0).abs() < 1e-6, "{az} {el}");
        assert!(matches!(r.set_pos(1000.0, 0.0), Err(crate::error::Fail::Rejected(_))));
        r.stop().unwrap();
    }
}
