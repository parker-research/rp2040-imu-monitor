# calibrate-rotator-from-imu

Calibrate a hamlib-controlled antenna rotator to **true az=0 / el=0** using an
IMU (accelerometer + magnetometer) riding on the antenna, with continuous
checks that the rotator, the antenna and the IMU are still mechanically
coupled.

```
cargo build --release
./target/release/calibrate-rotator-from-imu --help
```

Defaults: IMU auto-detected on `/dev/ttyACM*` (or `--imu /dev/ttyACM0`),
rotctld at `127.0.0.1:4533`, config file `calibrate-rotator-from-imu.toml` (`-c` to change).

## Workflow

### 1. `calibrate`: characterise the mounting (once, or after re-mounting)

Put the rotator at **true** zero yourself, then:

```
calibrate-rotator-from-imu calibrate
```

The command:

1. Captures a 40-sample reference with the antenna at true zero.
2. Makes ±10° test moves (`--step`, 1–15°) on elevation, then azimuth. It returns to zero after each move and re-measures. If `[limits]` block the negative side, it moves +10° and +20° instead.
3. **Surveys the operating envelope.** It stops and measures at about 80 poses along the paths the antenna normally travels:
   - an azimuth ring (every `--az-step`, default 30°) at each elevation level (every `--el-step`, default 20°, up to `el_max`), driven in alternating directions;
   - a stop halfway up each climb between rings, and a column back down;
   - a satellite-pass-like arc (azimuth sweeping while elevation rises to the top ring and falls again). These points are **held out** to validate the fit before being added to it.

   The model is refitted after every stop, and later rings are supervised with it. This takes roughly 15–40 minutes, depending on rotator speed. Pass `--quick` to skip it, but azimuth is then only trustworthy near el=0.
4. Derives the **mounting transform** from the data alone:
   - the elevation axis (the axis gravity rotates about during el moves; refined over the survey's full elevation range),
   - "up",
   - the boresight,
   - the IMU's yaw/pitch/roll relative to the rotator,
   - elevation-axle and azimuth-axis tilt.

   No assumption is made about how the IMU is mounted. Your sample line, with roll ≈ −180°, means it is upside-down, and that is fine.
5. Documents **signs and gains**: how every firmware field (`pitch_deg`, `roll_deg`, `heading_deg`) and every raw axis responds to +1° of rotator motion. These are written to the `[signs]` table, with a human-readable `summary`.
6. Fits a **magnetometer model** (see below) to all points, and builds a deviation table from the el=0 ring.
7. Checks closure: the IMU must read the same at zero at the end as at the start.

The summary and the `[survey]` table report how well the model fits: azimuth error over the survey and on the held-out pass, the horizontal-field ratio range (ideal 1.00), and the worst elevation error. Pass `--report <PATH>` to also write every measured point (rotator position, raw accelerometer and magnetometer, model error) as JSON for inspection.

Configs from before the survey existed (version 1) are rejected; re-run `calibrate`.

Pass `--yes` to skip the "is it really at true zero?" prompt.
Pass `--goto-zero` to command reported (0,0) first.

### 2. `zero`: auto-zero

```
calibrate-rotator-from-imu zero                       # elevation, then azimuth, then re-verify both
calibrate-rotator-from-imu zero --mode el             # elevation only   (alias of --mode elevation)
calibrate-rotator-from-imu zero --mode az             # azimuth only     (alias of --mode azimuth)
calibrate-rotator-from-imu zero --dry-run             # measure and report, don't move
calibrate-rotator-from-imu zero --report out.json --on-zero-cmd 'echo $ROTCAL_ZERO_AZ $ROTCAL_ZERO_EL'
```

This is a closed loop: measure the true angle from the IMU, command a relative correction, wait to settle, re-measure, and repeat. It stops when the angle is within `el_tol_deg` / `az_tol_deg`, or when the residual is below the rotator's resolution.

**Starting from anywhere.** `zero` works from any pose, including straight after a power cycle. The controller then reports 0/0 wherever the antenna actually is. Elevation is zeroed first, using gravity alone. Azimuth is measured once el≈0, which is where the magnetometer is most trustworthy. If the field can't be read at the starting elevation, that is only a warning. Because the controller's frame can be far from true, it may need targets outside its usual range, for example reported el −40° to bring a boom at true 40° down. hamlib's `P` would refuse those, so moves are sent as raw G-code instead (see [Moves](#moves-raw-g-code)). If a target is refused anyway, `--allow-long-way` takes the same direction one turn away, provided it is within `[limits]`.

**Re-zeroing the controller.** hamlib has no generic "set current position" command. Once the antenna is at IMU-true zero, calibrate-rotator-from-imu sends `w [ESP444]RESTART` through rotctld. The ESP32 controller reboots and takes its current position as az=el=0. calibrate-rotator-from-imu then:

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
calibrate-rotator-from-imu check            # no motion
calibrate-rotator-from-imu check --wiggle   # plus ±5° moves on each axis to re-confirm signs/gains
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

Each check below, if it fails, aborts the run with exit code 5. The tool never sends a stop (`S`): a move already under way runs on to its commanded target, so keep moves within safe travel via `[limits]`.

- **Static, before anything moves.** Gravity must lie on the cone around the calibrated elevation axis. Pure elevation motion keeps it there. If the IMU has shifted or tilted on its mount, it is off the cone.
- **During every move.** Once the rotator has travelled 4° in el (or 15° in az), the IMU must show at least 30% of the expected motion in the right direction. This must hold for 3 consecutive polls, otherwise the run aborts and no further moves are sent. This catches a mast slipping in its clamp, a stripped gear, or a detached IMU before the antenna is driven further.
- **After every move.** The IMU change must match the rotator-reported change within `abs + rel·|move|`. For azimuth this tolerance also includes the measurement's standard error.
- **Cross-axis.** An azimuth-only move must not change elevation, beyond the measured base tilt.
- **Sanity limits.**
  - `max_correction_*` refuses to "correct" more than that many degrees. It is off by default (90° / 180°), since after a power cycle the antenna can legitimately be anywhere.
  - Stall detection, move timeouts, and rotctld limit rejections.
- **During calibration.**
  - Test moves must produce the expected tilt or field change.
  - At every survey stop the IMU must agree with the rotator on elevation (allowing for base tilt), stay on the elevation-axis cone, and roughly agree on azimuth once the fit is confident.
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
| 130 | Ctrl-C. No stop is sent; a move in progress finishes |

## IMU data handling

A sample is used only if all of the following hold:

- `health == "ok"`, `read_ok`, `plausible`
- not `data_frozen`, not `wdt_reboot`
- `fail_streak == 0`

If `reinit` or `cfg_restore` changes mid-window, the window is discarded, because the sensor configuration may have changed. The reader also counts sequence gaps, tolerates the ACM device re-enumerating, and requires the accelerometer to be still (std < 0.02 g) with |a| ≈ 1 g.

## Why the magnetometer works (and where it won't)

At Calgary the field dips about 73°, so only about 16 µT of the field is horizontal. Your sample shows |B| ≈ 97 µT against Earth's ≈ 56 µT, which means a large **hard-iron** offset from the boom, motors or cables. The firmware `heading_deg` is therefore close to useless: in the simulator it moves only about 0.1° per degree of azimuth. calibrate-rotator-from-imu ignores it and instead:

- **Fits a physical model:**
  `m_imu = A·Mᵀ·Rot(el_axis, el)·(Rz(az)·B_earth + B_stage) + b_hard_iron`.
  - `b_hard_iron`: magnetised parts that ride with the IMU (boom, cables).
  - `A`: soft iron and sensor gain errors, also fixed to the IMU. The LSM303DLH's Z axis, for example, has a different gain from X/Y.
  - `B_stage`: iron and motors on the azimuth stage. They turn with azimuth but not elevation, so at el=0 they look exactly like hard iron. They only show up once the boom tilts.

  `A` and `B_stage` are invisible near el=0. A calibration from small moves around zero absorbs them into the hard iron, and the model then falls apart at higher elevation (horizontal-field ratio far from 1). The envelope survey fits them using every point's *known* rotator angles. The fit starts from the exact hard-iron-only solution and refines the rest with Levenberg–Marquardt. A non-physical (mirror-like) `A` is rejected, which also settles the azimuth sign.
- **De-rotates elevation using the accelerometer** before reading azimuth. With 54 µT vertical, a 0.1° tilt error alone would be about 0.4° of azimuth error.
- **Averages adaptively.** It uses robust (MAD-rejected) per-sample azimuths, collecting 16–64 samples until the standard error is below 0.25°.
- **Waits longer after moves** (2.5 s) because motor and brake currents disturb the field.
- **Checks consistency.** Every reading's horizontal magnitude and vertical component must match the model. If not, something magnetic is nearby or the IMU moved, and the run fails with exit code 7 rather than trusting the reading.
- **Converges at the zero point regardless of model error.** The zero is the point where the field *matches the reference*. Model error only affects the scale away from zero, so closed-loop zeroing still converges to true zero.

A field that changes after calibration (new metalwork, a car parked nearby) breaks the azimuth reference, so re-run `calibrate`. A temperature change of more than 15 °C from calibration produces a warning.

## Tuning

`[limits]`, `[thresholds]` and `[timing]` in `calibrate-rotator-from-imu.toml` are hand-editable.

- **`[limits]` are physical travel limits in true coordinates** (default az 0–360°, el 0–85°), not in the controller's reported frame. Targets are converted using the IMU's measurement of where true zero is, with `pos_tol_deg` of slack. They also bound the calibration survey. Calibration's `--az-min/--az-max/--el-min/--el-max` override them.
- **Rotator rejects a target** (`RPRT -1`): with `method = "hamlib"`, rotctld's own limits apply in the controller's frame, which can be offset from true. Switch to `gcode` or widen them (for example `--set-conf=min_az=-360,min_el=-90`). Azimuth corrections across a hard stop require `--allow-long-way`.
- **Deadband.** Rotators with a 1–2° deadband will stop with "residual below the rotator's resolution". Raise `min_step_deg` / `pos_tol_deg` to match.

### Moves (raw G-code)

Position commands go to the controller as raw G-code through rotctld, bypassing hamlib's min/max:

```
w G0 X<az / deg_per_gcode_unit> Y<el / deg_per_gcode_unit>
```

The values are absolute targets in the controller's reported frame. With the default 9° per unit, reported az −40° / el 22.5° is sent as `w G0 X-4.4444 Y2.5000`. Positions are still read with hamlib's `p`. The settings live in `[motion]`:

```toml
[motion]
method = "gcode"                  # or "hamlib" to use rotctld's `P az el`
gcode_template = "G0 X{az} Y{el}" # {az}/{el} in G-code units
deg_per_gcode_unit = 9.0
```

`calibrate` keeps an existing `[motion]` from the config file, so set it before calibrating if your controller differs. Because hamlib no longer checks targets, `[limits]` are the only software guard on travel.

## Simulator

Every command runs against a built-in simulator. It provides:

- a fake rotctld (`P` limited to 0–360° / 0–90° like a typical hamlib setup; `w G0 …` at 9°/unit with no limits),
- an IMU emitting the same JSONL,
- a random upside-down mount,
- 35 µT of hard iron,
- ±4% soft iron / sensor gain error,
- a 10 µT field from the azimuth stage,
- a 0.4° base tilt,
- motor interference,
- time compression.

```
calibrate-rotator-from-imu --sim -c sim.toml calibrate --yes
calibrate-rotator-from-imu --sim -c sim.toml --sim-offset-az 23.5 --sim-offset-el -3 --sim-start-az 40 --sim-start-el 7 zero
# powered on at true az 40 / el 40, controller boots reporting 0/0:
calibrate-rotator-from-imu --sim -c sim.toml --sim-offset-az -40 --sim-offset-el -40 --sim-start-az 0 --sim-start-el 0 zero
calibrate-rotator-from-imu --sim -c sim.toml --sim-fault mast-slip --sim-start-az 50 zero      # -> exit 5
calibrate-rotator-from-imu --sim -c sim.toml --sim-fault imu-shifted check                       # -> exit 5
```

Available faults: `mast-slip`, `el-stuck`, `imu-detached`, `imu-shifted`.
Run `cargo test` to run the unit tests plus end-to-end runs of each mode and fault.
