//! End-to-end runs of the real binary against the built-in simulator.

use std::path::Path;
use std::process::Command;

fn run_cli(cfg: &Path, args: &[&str]) -> (i32, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_calibrate-rotator-from-imu"))
        .arg("--sim")
        .arg("--sim-speed")
        .arg("25")
        .arg("-c")
        .arg(cfg)
        .args(args)
        .output()
        .expect("run calibrate-rotator-from-imu");
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    (out.status.code().unwrap_or(-1), text)
}

/// A faster, coarser envelope survey for tests that don't examine the fit.
const COARSE: [&str; 4] = ["--az-step", "45", "--el-step", "40"];

fn calibrated(dir: &Path, extra: &[&str]) -> std::path::PathBuf {
    let cfg = dir.join("cal.toml");
    let mut a = vec!["calibrate", "--yes"];
    a.extend(extra);
    let (code, out) = run_cli(&cfg, &a);
    assert_eq!(code, 0, "{out}");
    cfg
}

fn report(p: &Path) -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
}

#[test]
fn calibrate_then_zero_all_modes() {
    let d = tempfile::tempdir().unwrap();
    let cfg = calibrated(d.path(), &[]);
    let toml = std::fs::read_to_string(&cfg).unwrap();
    assert!(toml.contains("[mount]") && toml.contains("[signs]") && toml.contains("elevation_primary_field"));
    // the envelope survey must model the field well everywhere, including
    // on the held-out pass it was not fitted to
    let c: toml::Value = toml::from_str(&toml).unwrap();
    let sv = &c["survey"];
    assert_eq!(sv["performed"].as_bool(), Some(true), "{toml}");
    assert!(sv["validation_az_max_deg"].as_float().unwrap() < 2.0, "{sv}");
    assert!(sv["h_ratio_min"].as_float().unwrap() > 0.95 && sv["h_ratio_max"].as_float().unwrap() < 1.05, "{sv}");
    let survey: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(cfg.with_extension("survey.json")).unwrap()).unwrap();
    assert!(survey.as_array().unwrap().len() > 60);

    let rep = d.path().join("r.json");
    let rs = rep.to_str().unwrap();
    let off = ["--sim-offset-az", "17.0", "--sim-offset-el", "-2.5", "--sim-start-az", "40", "--sim-start-el", "6"];

    let mut a = off.to_vec();
    a.extend(["zero", "--report", rs]);
    let (code, out) = run_cli(&cfg, &a);
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
        let (code, out) = run_cli(&cfg, &a);
        assert_eq!(code, 0, "{mode}: {out}");
        let r = report(&rep);
        assert!(r[key].as_f64().unwrap().abs() < 1.2, "{mode}: {r}");
        // the other axis is still off, so the controller must NOT be restarted
        assert_eq!(r["sync"]["performed"], false, "{mode}: {r}");
    }

    // a restart command the controller doesn't honour must fail loudly
    let mut a = off.to_vec();
    a.extend(["zero", "--sync-cmd", "w BOGUS"]);
    let (code, out) = run_cli(&cfg, &a);
    assert_eq!(code, 4, "{out}");
    assert!(out.contains("did not adopt the current position as zero"), "{out}");

    // Powered on away from zero: the controller boots reporting 0/0 while the
    // antenna is really at az 40 / el 40.
    let boot = ["--sim-offset-az", "-40", "--sim-offset-el", "-40", "--sim-start-az", "0", "--sim-start-el", "0"];
    let mut a = boot.to_vec();
    a.extend(["zero", "--report", rs]);
    let (code, out) = run_cli(&cfg, &a);
    assert_eq!(code, 0, "{out}");
    let r = report(&rep);
    assert!((r["offset_before_sync_az"].as_f64().unwrap() + 40.0).abs() < 1.5, "{r}");
    assert!((r["offset_before_sync_el"].as_f64().unwrap() + 40.0).abs() < 0.5, "{r}");
    assert_eq!(r["sync"]["performed"], true, "{r}");
    // ...which needs the raw G-code moves: through hamlib's `P` the negative
    // targets are refused by rotctld's limits
    let toml = std::fs::read_to_string(&cfg).unwrap();
    assert!(toml.contains("method = \"gcode\""), "{toml}");
    let hamlib = d.path().join("hamlib.toml");
    std::fs::write(&hamlib, toml.replace("method = \"gcode\"", "method = \"hamlib\"")).unwrap();
    let mut a = boot.to_vec();
    a.push("zero");
    let (code, out) = run_cli(&hamlib, &a);
    assert_eq!(code, 4, "{out}");
    assert!(out.contains("RPRT -1"), "{out}");

    // dry run must not move
    let mut a = off.to_vec();
    a.extend(["zero", "--dry-run", "--report", rs]);
    let (code, _) = run_cli(&cfg, &a);
    assert_eq!(code, 0);
    let r = report(&rep);
    assert_eq!(r["rotator_final"][0].as_f64().unwrap(), 40.0);
    assert!(r["steps"].as_array().unwrap().is_empty());
}

#[test]
fn link_failures_are_detected() {
    let d = tempfile::tempdir().unwrap();
    let cfg = calibrated(d.path(), &COARSE);
    for fault in ["mast-slip", "el-stuck", "imu-detached", "imu-shifted"] {
        let (code, out) = run_cli(
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
    let cfg = calibrated(d.path(), &COARSE);
    let (code, out) = run_cli(&cfg, &["check", "--wiggle"]);
    assert_eq!(code, 0, "{out}");
    let (code, _) = run_cli(&cfg, &["--sim-offset-az", "6", "check"]);
    assert_eq!(code, 8, "offset rotator should report not-zeroed");
    let (code, _) = run_cli(&cfg, &["--sim-fault", "mast-slip", "--sim-fault-after", "2", "check", "--wiggle"]);
    assert_eq!(code, 5);
}
