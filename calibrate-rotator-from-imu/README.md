# rotcal

Calibrate a hamlib-controlled antenna rotator to **true az=0 / el=0** using an
IMU (accelerometer + magnetometer) riding on the antenna, with continuous
checks that the rotator, the antenna and the IMU are still mechanically
coupled.

```
cargo build --release
./target/release/rotcal --help
```

Defaults: IMU auto-detected on `/dev/ttyACM*` (or `--imu /dev/ttyACM0`),
rotctld at `127.0.0.1:4533`, config file `rotcal.toml` (`-c` to change).

## Workflow

### 1. `calibrate`: characterise the mounting (once, or after re-mounting)

Put the rotator at **true** zero yourself, then:

```
rotcal calibrate --sweep
```

The command:

1. Captures a 40-sample reference with the antenna at true zero.
2. Makes ±10° test moves (`--step`, 1–15°) on elevation, then azimuth. It returns to zero after each move and re-measures.
3. Derives the **mounting transform** from the data alone:
   - the elevation axis (the axis gravity rotates about during el moves),
   - "up",
   - the boresight,
   - the IMU's yaw/pitch/roll relative to the rotator,
   - elevation-axle and azimuth-axis tilt.

   No assumption is made about how the IMU is mounted. Your sample line, with roll ≈ −180°, means it is upside-down, and that is fine.
4. Documents **signs and gains**: how every firmware field (`pitch_deg`, `roll_deg`, `heading_deg`) and every raw axis responds to +1° of rotator motion. These are written to the `[signs]` table, with a human-readable `summary`.
5. Fits a **magnetometer model** (see below). `--sweep` additionally steps azimuth through 360° (or as much as `[limits]` allow) and builds a deviation table. A sweep is recommended.
6. Checks closure: the IMU must read the same at zero at the end as at the start.

Pass `--yes` to skip the "is it really at true zero?" prompt.
Pass `--goto-zero` to command reported (0,0) first.

### 2. `zero`: auto-zero

```
rotcal zero                       # elevation, then azimuth, then re-verify both
rotcal zero --mode el             # elevation only   (alias of --mode elevation)
rotcal zero --mode az             # azimuth only     (alias of --mode azimuth)
rotcal zero --dry-run             # measure and report, don't move
rotcal zero --report out.json --on-zero-cmd 'echo $ROTCAL_ZERO_AZ $ROTCAL_ZERO_EL'
```

This is a closed loop: measure the true angle from the IMU, command a relative correction, wait to settle, re-measure, and repeat. It stops when the angle is within `el_tol_deg` / `az_tol_deg`, or when the residual is below the rotator's resolution.

**Re-zeroing the controller.** hamlib has no generic "set current position" command. Once the antenna is at IMU-true zero, rotcal sends `w [ESP444]RESTART` through rotctld. The ESP32 controller reboots and takes its current position as az=el=0. rotcal then:

1. waits for the controller to come back (`boot_wait_s`, up to `reconnect_timeout_s`);
2. requires the rotator to report 0/0 (± `expect_zero_tol_deg`), otherwise exit 4;
3. re-measures with the IMU to confirm the antenna didn't move during the reboot, otherwise exit 5.

A restart zeroes **both** axes. So `--mode el` / `--mode az` measure the other axis first and skip the restart, with a warning, unless that axis is also at true zero.

- The command lives in `[sync]` in the config.
- `--sync-cmd '...'` overrides it for one run.
- `--no-sync` only reports the offset found.
- The report records the offset before the restart (`offset_before_sync_az/el`) and the restart result (`sync`).
- `--on-zero-cmd` runs a shell command afterwards, with the values in `ROTCAL_*` environment variables.

### 3. `check`: verify calibration without calibrating

```
rotcal check            # no motion
rotcal check --wiggle   # plus ±5° moves on each axis to re-confirm signs/gains
```

This reports the IMU-true az/el, the rotator's error versus truth, and every static link check.

| Exit code | Meaning |
|---|---|
| 0 | Pass |
| 5 | Mechanical link problem |
| 7 | Magnetic problem |
| 8 | Link healthy but rotator not zeroed |

### 4. `monitor`

A live table of the stream, the IMU-derived true az/el, and the rotator position.

## Mechanical-link ("glue") checks

Each check below, if it fails, **stops the rotator** and exits with code 5.

- **Static, before anything moves.** Gravity must lie on the cone around the calibrated elevation axis. Pure elevation motion keeps it there. If the IMU has shifted or tilted on its mount, it is off the cone.
- **During every move.** Once the rotator has travelled 4° in el (or 15° in az), the IMU must show at least 30% of the expected motion in the right direction. This must hold for 3 consecutive polls, otherwise the rotator is stopped immediately. This catches a mast slipping in its clamp, a stripped gear, or a detached IMU before the antenna is driven further.
- **After every move.** The IMU change must match the rotator-reported change within `abs + rel·|move|`. For azimuth this tolerance also includes the measurement's standard error.
- **Cross-axis.** An azimuth-only move must not change elevation, beyond the measured base tilt.
- **Sanity limits.**
  - Refuse to "correct" more than `max_correction_*` degrees, since a huge correction usually means something came loose.
  - Stall detection, move timeouts, and rotctld limit rejections.
- **During calibration.**
  - Test moves must produce the expected tilt or field change.
  - Elevation and azimuth gains must be ≈1.
  - Returning to zero must reproduce the reference.

Other failure classes have their own exit codes:

| Exit code | Meaning |
|---|---|
| 2 | Config |
| 3 | IMU stream / health |
| 4 | Rotator / rotctld |
| 6 | Did not converge |
| 7 | Magnetic inconsistency |
| 130 | Ctrl-C. A stop (`S`) is sent to rotctld first |

## IMU data handling

A sample is used only if all of the following hold:

- `health == "ok"`, `read_ok`, `plausible`
- not `data_frozen`, not `wdt_reboot`
- `fail_streak == 0`

If `reinit` or `cfg_restore` changes mid-window, the window is discarded, because the sensor configuration may have changed. The reader also counts sequence gaps, tolerates the ACM device re-enumerating, and requires the accelerometer to be still (std < 0.02 g) with |a| ≈ 1 g.

## Why the magnetometer works (and where it won't)

At Calgary the field dips about 73°, so only about 16 µT of the field is horizontal. Your sample shows |B| ≈ 97 µT against Earth's ≈ 56 µT, which means a large **hard-iron** offset from the boom, motors or cables. The firmware `heading_deg` is therefore close to useless: in the simulator it moves only about 0.1° per degree of azimuth. rotcal ignores it and instead:

- **Fits a physical model** with linear least squares:
  `m_imu = Mᵀ·Rot(el_axis, el)·Rz(az)·B_earth + b_hard_iron`.
  The fit uses all calibration poses and their *known* rotator angles, so even the ±10° moves alone recover the hard-iron offset. A sweep makes it more robust and adds a deviation table for residual soft-iron and local distortion.
- **De-rotates elevation using the accelerometer** before reading azimuth. With 54 µT vertical, a 0.1° tilt error alone would be about 0.4° of azimuth error.
- **Averages adaptively.** It uses robust (MAD-rejected) per-sample azimuths, collecting 16–64 samples until the standard error is below 0.25°.
- **Waits longer after moves** (2.5 s) because motor and brake currents disturb the field.
- **Checks consistency.** Every reading's horizontal magnitude and vertical component must match the model. If not, something magnetic is nearby or the IMU moved, and the run fails with exit code 7 rather than trusting the reading.
- **Converges at the zero point regardless of model error.** The zero is the point where the field *matches the reference*. Model error only affects the scale away from zero, so closed-loop zeroing still converges to true zero.

A field that changes after calibration (new metalwork, a car parked nearby) breaks the azimuth reference, so re-run `calibrate`. A temperature change of more than 15 °C from calibration produces a warning.

## Tuning

`[limits]`, `[thresholds]` and `[timing]` in `rotcal.toml` are hand-editable.

- **Rotator rejects targets near zero.** If rotctld refuses positions slightly below 0° (`RPRT -1`), widen its limits (for example `--set-conf=min_el=-10`) or `[limits]`. Azimuth corrections across a hard stop require `--allow-long-way`.
- **Deadband.** Rotators with a 1–2° deadband will stop with "residual below the rotator's resolution". Raise `min_step_deg` / `pos_tol_deg` to match.

## Simulator

Every command runs against a built-in simulator. It provides:

- a fake rotctld,
- an IMU emitting the same JSONL,
- a random upside-down mount,
- 35 µT of hard iron,
- a 0.4° base tilt,
- motor interference,
- time compression.

```
rotcal --sim -c sim.toml calibrate --yes --sweep
rotcal --sim -c sim.toml --sim-offset-az 23.5 --sim-offset-el -3 --sim-start-az 40 --sim-start-el 7 zero
rotcal --sim -c sim.toml --sim-fault mast-slip --sim-start-az 50 zero      # -> exit 5
rotcal --sim -c sim.toml --sim-fault imu-shifted check                       # -> exit 5
```

Available faults: `mast-slip`, `el-stuck`, `imu-detached`, `imu-shifted`.
Run `cargo test` to run the unit tests plus end-to-end runs of each mode and fault.
