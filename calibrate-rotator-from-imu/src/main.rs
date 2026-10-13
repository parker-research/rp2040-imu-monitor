mod calibrate;
mod check;
mod config;
mod error;
mod imu;
mod math;
mod measure;
mod model;
mod monitor;
mod motion;
mod rotctl;
mod sample;
mod sim;
mod zero;

use clap::{ArgAction, Args, Parser, Subcommand};
use config::{Config, Limits, Thresholds, Timing};
use error::{Fail, R};
use imu::ImuStream;
use measure::Ctx;
use rotctl::Rotctl;
use std::path::{Path, PathBuf};

/// Calibrate an antenna rotator to true az=0 / el=0 using an IMU riding on the antenna.
#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// IMU serial device, or "auto" to probe /dev/ttyACM* and /dev/ttyUSB*.
    #[arg(long, global = true, default_value = "auto")]
    imu: String,
    /// rotctld address.
    #[arg(long, global = true, default_value = "127.0.0.1:4533")]
    rotctld: String,
    /// Calibration file.
    #[arg(long, short, global = true, default_value = "calibrate-rotator-from-imu.toml")]
    config: PathBuf,
    /// -v debug, -vv trace.
    #[arg(long, short, global = true, action = ArgAction::Count)]
    verbose: u8,
    #[command(flatten)]
    sim: SimArgs,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Args)]
struct SimArgs {
    /// Use the built-in simulator instead of real hardware.
    #[arg(long, global = true)]
    sim: bool,
    #[arg(long, global = true, default_value_t = 7)]
    sim_seed: u64,
    /// Time compression factor for the simulator.
    #[arg(long, global = true, default_value_t = 20.0)]
    sim_speed: f64,
    /// Rotator azimuth error: reported minus true (deg).
    #[arg(long, global = true, default_value_t = 0.0, allow_hyphen_values = true)]
    sim_offset_az: f64,
    #[arg(long, global = true, default_value_t = 0.0, allow_hyphen_values = true)]
    sim_offset_el: f64,
    /// Starting reported position (default: true zero).
    #[arg(long, global = true, allow_hyphen_values = true)]
    sim_start_az: Option<f64>,
    #[arg(long, global = true, allow_hyphen_values = true)]
    sim_start_el: Option<f64>,
    #[arg(long, global = true, value_enum, default_value_t = sim::Fault::None)]
    sim_fault: sim::Fault,
    /// Inject the fault after this many position commands.
    #[arg(long, global = true, default_value_t = 0)]
    sim_fault_after: u32,
}

#[derive(Subcommand)]
enum Cmd {
    /// Characterise the IMU mounting with the rotator at TRUE zero; writes the config.
    Calibrate {
        /// Test move size (deg, 1–15).
        #[arg(long, default_value_t = 10.0)]
        step: f64,
        /// Don't ask for confirmation that the rotator is at true zero.
        #[arg(long, short)]
        yes: bool,
        /// Command the rotator to reported (0,0) first.
        #[arg(long)]
        goto_zero: bool,
        /// Also sweep azimuth to model magnetometer distortion (recommended).
        #[arg(long)]
        sweep: bool,
        #[arg(long, default_value_t = 30.0)]
        sweep_step: f64,
        #[arg(long, default_value_t = 360.0)]
        sweep_span: f64,
        #[command(flatten)]
        lim: LimitArgs,
    },
    /// Drive the rotator to IMU-true zero.
    Zero {
        /// elevation | azimuth | elevation-then-azimuth (aliases: el, az, both)
        #[arg(long, value_enum, default_value_t = zero::Mode::ElevationThenAzimuth)]
        mode: zero::Mode,
        /// Measure and report the needed correction without moving.
        #[arg(long)]
        dry_run: bool,
        /// Permit going the long way round when the short way crosses a limit.
        #[arg(long)]
        allow_long_way: bool,
        /// Write a JSON report here.
        #[arg(long)]
        report: Option<PathBuf>,
        /// Shell command run on success; offsets are passed as ROTCAL_* env vars.
        #[arg(long)]
        on_zero_cmd: Option<String>,
        /// Don't restart the controller at zero; only report the offset.
        #[arg(long)]
        no_sync: bool,
        /// Override the controller-restart command from the config.
        #[arg(long, conflicts_with = "no_sync")]
        sync_cmd: Option<String>,
    },
    /// Check calibration and mechanical link without zeroing (no motion unless --wiggle).
    Check {
        /// Make small ±moves to confirm signs/gains and the mechanical link.
        #[arg(long)]
        wiggle: bool,
        #[arg(long, default_value_t = 5.0)]
        step: f64,
        /// Skip magnetometer/azimuth checks.
        #[arg(long)]
        no_az: bool,
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Live view of IMU data (and true az/el if a config exists).
    Monitor {
        #[arg(long)]
        count: Option<u64>,
        /// Don't query rotctld.
        #[arg(long)]
        no_rotator: bool,
    },
}

#[derive(Args, Clone)]
struct LimitArgs {
    #[arg(long, allow_hyphen_values = true)]
    az_min: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    az_max: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    el_min: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    el_max: Option<f64>,
}

struct Io {
    imu: ImuStream,
    rotctld: String,
    speed: f64,
}

fn open_io(cli: &Cli) -> R<Io> {
    if cli.sim.sim {
        let o = sim::SimOpts {
            seed: cli.sim.sim_seed,
            speed: cli.sim.sim_speed,
            offset_az: cli.sim.sim_offset_az,
            offset_el: cli.sim.sim_offset_el,
            start_az: cli.sim.sim_start_az,
            start_el: cli.sim.sim_start_el,
            fault: cli.sim.sim_fault,
            fault_after: cli.sim.sim_fault_after,
        };
        let mut s = sim::start(&o).map_err(|e| Fail::Config(format!("simulator: {e}")))?;
        let imu = ImuStream::from_lines(s.lines.take().unwrap(), "simulator");
        Ok(Io { imu, rotctld: s.rotctld_addr, speed: cli.sim.sim_speed })
    } else {
        Ok(Io { imu: ImuStream::open_serial(Some(&cli.imu))?, rotctld: cli.rotctld.clone(), speed: 1.0 })
    }
}

fn install_ctrlc(addr: &str) {
    let a = addr.to_string();
    let _ = ctrlc::set_handler(move || {
        eprintln!("\ninterrupted — sending stop to rotctld");
        rotctl::emergency_stop(&a);
        std::process::exit(130);
    });
}

fn make_ctx(io: Io, cfg: Option<&Config>) -> R<Ctx> {
    install_ctrlc(&io.rotctld);
    let rot = Rotctl::connect(&io.rotctld)?;
    let (th, mut tm, lim) = match cfg {
        Some(c) => (c.thresholds.clone(), c.timing.clone(), c.limits.clone()),
        None => (Thresholds::default(), Timing::default(), Limits::default()),
    };
    tm.speed = io.speed;
    Ok(Ctx { imu: io.imu, rot, th, tm, lim })
}

fn write_json<T: serde::Serialize>(path: &Option<PathBuf>, v: &T) {
    if let Some(p) = path {
        match serde_json::to_string_pretty(v).map(|s| std::fs::write(p, s)) {
            Ok(Ok(())) => log::info!("report written to {}", p.display()),
            _ => log::warn!("could not write report {}", p.display()),
        }
    }
}

fn print_cal_summary(cfg: &Config, path: &Path) {
    println!("\nCalibration written to {}\n", path.display());
    for l in cfg.mount.description.iter().chain(&cfg.signs.summary) {
        println!("  {l}");
    }
    println!(
        "  magnetometer: |B_h| {:.1} uT, dip {:.1}°, hard-iron {:.1} uT, fit rms {:.2} uT, single-sample az σ {:.2}°{}",
        cfg.azimuth.horizontal_field_ut,
        cfg.azimuth.dip_deg,
        cfg.azimuth.field_model.hard_iron_imu_ut.norm(),
        cfg.azimuth.field_model.fit_rms_ut,
        cfg.azimuth.single_sample_noise_deg,
        if cfg.azimuth.swept { ", deviation table from sweep" } else { " (no sweep)" }
    );
}

fn real_main(cli: Cli) -> R<i32> {
    match &cli.cmd {
        Cmd::Calibrate { step, yes, goto_zero, sweep, sweep_step, sweep_span, lim } => {
            let io = open_io(&cli)?;
            let mut ctx = make_ctx(io, None)?;
            if let Some(v) = lim.az_min {
                ctx.lim.az_min = v;
            }
            if let Some(v) = lim.az_max {
                ctx.lim.az_max = v;
            }
            if let Some(v) = lim.el_min {
                ctx.lim.el_min = v;
            }
            if let Some(v) = lim.el_max {
                ctx.lim.el_max = v;
            }
            let o = calibrate::CalOpts {
                step: *step,
                yes: *yes,
                goto_zero: *goto_zero,
                sweep: *sweep,
                sweep_step: *sweep_step,
                sweep_span: *sweep_span,
            };
            let cfg = calibrate::run(&mut ctx, &o)?;
            cfg.save(&cli.config)?;
            print_cal_summary(&cfg, &cli.config);
            Ok(0)
        }
        Cmd::Zero { mode, dry_run, allow_long_way, report, on_zero_cmd, no_sync, sync_cmd } => {
            let cfg = Config::load(&cli.config)?;
            let io = open_io(&cli)?;
            let mut ctx = make_ctx(io, Some(&cfg))?;
            let o = zero::ZeroOpts {
                mode: *mode,
                dry_run: *dry_run,
                allow_long_way: *allow_long_way,
                report: report.clone(),
                on_zero_cmd: on_zero_cmd.clone(),
                sync_cmd: if *no_sync { None } else { sync_cmd.clone().or(cfg.sync.command.clone()) },
            };
            let mut rep = zero::Report { started: chrono::Local::now().to_rfc3339(), ..Default::default() };
            let res = zero::run(&mut ctx, &cfg, &o, &mut rep);
            rep.finished = chrono::Local::now().to_rfc3339();
            if let Err(e) = &res {
                rep.result = "failed".into();
                rep.error = Some(e.to_string());
                let _ = ctx.rot.stop();
            }
            write_json(&o.report, &rep);
            res?;
            let f = |v: Option<f64>| v.map(|x| format!("{x:+.2}°")).unwrap_or("-".into());
            println!("\n{} ({:?})", if *dry_run { "DRY RUN" } else { "ZERO COMPLETE" }, mode);
            println!("  IMU true elevation : {} → {}", f(rep.initial_el_deg), f(rep.final_el_deg));
            println!("  IMU true azimuth   : {} → {} (±{})", f(rep.initial_az_deg), f(rep.final_az_deg), rep.final_az_std_err_deg.map(|x| format!("{x:.2}°")).unwrap_or("-".into()));
            if let Some(p) = rep.rotator_final {
                println!("  rotator reports    : az {:.2}°  el {:.2}°", p.0, p.1);
            }
            println!(
                "  rotator offset found       : az {}  el {}  (what it read at true zero)",
                f(rep.offset_before_sync_az),
                f(rep.offset_before_sync_el)
            );
            match &rep.sync {
                Some(s) if s.performed => println!(
                    "  controller re-zeroed       : `{}` → now reports az {}  el {}",
                    s.command,
                    f(s.rotator_after.map(|p| p.0)),
                    f(s.rotator_after.map(|p| p.1))
                ),
                Some(s) => println!("  controller NOT re-zeroed   : {}", s.skipped_reason.clone().unwrap_or_default()),
                None => println!("  controller NOT re-zeroed   : sync disabled; apply the offset yourself"),
            }
            for w in &rep.warnings {
                println!("  warning: {w}");
            }
            Ok(0)
        }
        Cmd::Check { wiggle, step, no_az, report } => {
            let cfg = Config::load(&cli.config)?;
            let io = open_io(&cli)?;
            let mut ctx = make_ctx(io, Some(&cfg))?;
            let o = check::CheckOpts { wiggle: *wiggle, step: *step, skip_az: *no_az };
            let r = check::run(&mut ctx, &cfg, &o)?;
            write_json(report, &r);
            println!("\nCALIBRATION CHECK");
            for i in &r.items {
                println!("  [{}] {:<32} {}", if i.ok { " ok " } else { "FAIL" }, i.name, i.detail);
            }
            for w in &r.warnings {
                println!("  warning: {w}");
            }
            let code = r.exit_code();
            println!("  result: {}", if code == 0 { "PASS".to_string() } else { format!("FAIL (exit {code})") });
            Ok(code)
        }
        Cmd::Monitor { count, no_rotator } => {
            let cfg = Config::load(&cli.config).ok();
            let io = open_io(&cli)?;
            let rot = if *no_rotator { None } else { Rotctl::connect(&io.rotctld).ok() };
            monitor::run(&io.imu, rot, cfg.as_ref(), *count, io.speed)?;
            Ok(0)
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let level = match cli.verbose {
        0 => "info",
        1 => "debug",
        _ => "trace",
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level))
        .format_timestamp_millis()
        .format_target(false)
        .init();
    let code = match real_main(cli) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("\nERROR: {e}");
            e.exit_code()
        }
    };
    std::process::exit(code);
}
