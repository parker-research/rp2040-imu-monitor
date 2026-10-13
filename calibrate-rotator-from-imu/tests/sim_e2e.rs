//! End-to-end runs of the real binary against the built-in simulator.

use std::path::Path;
use std::process::Command;

fn rotcal(cfg: &Path, args: &[&str]) -> (i32, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_rotcal"))
        .arg("--sim")
        .arg("--sim-speed")
        .arg("25")
        .arg("-c")
        .arg(cfg)
        .args(args)
        .output()
        .expect("run rotcal");
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    (out.status.code().unwrap_or(-1), text)
}

fn calibrated(dir: &Path, sweep: bool) -> std::path::PathBuf {
    let cfg = dir.join("cal.toml");
    let mut a = vec!["calibrate", "--yes"];
    if sweep {
        a.push("--sweep");
    }
    let (code, out) = rotcal(&cfg, &a);
    assert_eq!(code, 0, "{out}");
    cfg
}

fn report(p: &Path) -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
}

#[test]
fn calibrate_then_zero_all_modes() {
    let d = tempfile::tempdir().unwrap();
    let cfg = calibrated(d.path(), true);
    let toml = std::fs::read_to_string(&cfg).unwrap();
    assert!(toml.contains("[mount]") && toml.contains("[signs]") && toml.contains("elevation_primary_field"));

    let rep = d.path().join("r.json");
    let rs = rep.to_str().unwrap();
    let off = ["--sim-offset-az", "17.0", "--sim-offset-el", "-2.5", "--sim-start-az", "40", "--sim-start-el", "6"];

    let mut a = off.to_vec();
    a.extend(["zero", "--report", rs]);
    let (code, out) = rotcal(&cfg, &a);
    assert_eq!(code, 0, "{out}");
    let r = report(&rep);
    assert!((r["offset_before_sync_az"].as_f64().unwrap() - 17.0).abs() < 1.5, "{r}");
    assert!((r["offset_before_sync_el"].as_f64().unwrap() + 2.5).abs() < 0.5, "{r}");
    // controller was restarted at true zero and now reads 0/0
    assert_eq!(r["sync"]["performed"], true, "{r}");
    assert!(r["rotator_final"][0].as_f64().unwrap().abs() < 0.5 && r["rotator_final"][1].as_f64().unwrap().abs() < 0.5);

    for (mode, key) in [("el", "final_el_deg"), ("az", "final_az_deg")] {
        let mut a = off.to_vec();
        a.extend(["zero", "--mode", mode, "--report", rs]);
        let (code, out) = rotcal(&cfg, &a);
        assert_eq!(code, 0, "{mode}: {out}");
        let r = report(&rep);
        assert!(r[key].as_f64().unwrap().abs() < 1.2, "{mode}: {r}");
        // the other axis is still off, so the controller must NOT be restarted
        assert_eq!(r["sync"]["performed"], false, "{mode}: {r}");
    }

    // a restart command the controller doesn't honour must fail loudly
    let mut a = off.to_vec();
    a.extend(["zero", "--sync-cmd", "w BOGUS"]);
    let (code, out) = rotcal(&cfg, &a);
    assert_eq!(code, 4, "{out}");
    assert!(out.contains("did not adopt the current position as zero"), "{out}");

    // dry run must not move
    let mut a = off.to_vec();
    a.extend(["zero", "--dry-run", "--report", rs]);
    let (code, _) = rotcal(&cfg, &a);
    assert_eq!(code, 0);
    let r = report(&rep);
    assert_eq!(r["rotator_final"][0].as_f64().unwrap(), 40.0);
    assert!(r["steps"].as_array().unwrap().is_empty());
}

#[test]
fn link_failures_are_detected() {
    let d = tempfile::tempdir().unwrap();
    let cfg = calibrated(d.path(), false);
    for fault in ["mast-slip", "el-stuck", "imu-detached", "imu-shifted"] {
        let (code, out) = rotcal(
            &cfg,
            &["--sim-offset-az", "20", "--sim-start-az", "50", "--sim-start-el", "8", "--sim-fault", fault, "zero"],
        );
        assert_eq!(code, 5, "{fault} should fail the link check:\n{out}");
        assert!(out.contains("MECHANICAL LINK CHECK FAILED"), "{out}");
    }
}

#[test]
fn check_mode() {
    let d = tempfile::tempdir().unwrap();
    let cfg = calibrated(d.path(), false);
    let (code, out) = rotcal(&cfg, &["check", "--wiggle"]);
    assert_eq!(code, 0, "{out}");
    let (code, _) = rotcal(&cfg, &["--sim-offset-az", "6", "check"]);
    assert_eq!(code, 8, "offset rotator should report not-zeroed");
    let (code, _) = rotcal(&cfg, &["--sim-fault", "mast-slip", "--sim-fault-after", "2", "check", "--wiggle"]);
    assert_eq!(code, 5);
}
