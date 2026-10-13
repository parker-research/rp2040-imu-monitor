//! `rotcal monitor`: live view of the raw stream and (with a config) the
//! IMU-derived true az/el next to what the rotator reports.

use crate::config::Config;
use crate::error::R;
use crate::imu::ImuStream;
use crate::model::Model;
use crate::rotctl::Rotctl;
use std::time::{Duration, Instant};

pub fn run(imu: &ImuStream, mut rot: Option<Rotctl>, cfg: Option<&Config>, count: Option<u64>, speed: f64) -> R<()> {
    let model = cfg.map(Model::from_config);
    let mut after = Instant::now();
    let mut n = 0u64;
    let to = Duration::from_secs_f64(3.0 / speed);
    println!(
        "{:>7} {:>6} {:>7} {:>8} {:>8} {:>8} | {:>8} {:>8} {:>6} {:>6} | {:>8} {:>8}",
        "seq", "health", "|a| g", "fw_roll", "fw_ptch", "fw_hdg", "true_el", "true_az", "offax", "h_rat", "rot_az", "rot_el"
    );
    loop {
        let Some(rx) = imu.next_after(after, to) else {
            println!("(no data for {:.1}s)", to.as_secs_f64());
            after = Instant::now();
            continue;
        };
        after = rx.at;
        let s = &rx.s;
        let f = |v: Option<f64>| v.map(|x| format!("{x:8.2}")).unwrap_or_else(|| format!("{:>8}", "-"));
        let health = if let Some(r) = s.reject_reason() { r.to_string() } else { "ok".into() };
        let (el, az, off, hr) = match (&model, s.reject_reason()) {
            (Some(m), None) => {
                let a = m.az_eval(s.acc(), s.mag());
                (Some(m.el(s.acc())), Some(a.az), Some(m.off_axis(s.acc())), Some(a.h_ratio))
            }
            _ => (None, None, None, None),
        };
        let rp = rot.as_mut().and_then(|r| r.get_pos().ok());
        println!(
            "{:>7} {:>6} {:>7.4} {} {} {} | {} {} {:>6} {:>6} | {} {}",
            s.seq,
            &health[..health.len().min(6)],
            s.acc().norm(),
            f(s.roll_deg),
            f(s.pitch_deg),
            f(s.heading_deg),
            f(el),
            f(az),
            off.map(|x| format!("{x:.2}")).unwrap_or("-".into()),
            hr.map(|x| format!("{x:.2}")).unwrap_or("-".into()),
            f(rp.map(|p| p.0)),
            f(rp.map(|p| p.1)),
        );
        n += 1;
        if count.is_some_and(|c| n >= c) {
            return Ok(());
        }
    }
}
