# RP2040 IMU Monitor Firmware

Runs on an RP2040-Zero wired to an LSM303-family accelerometer/magnetometer. Streams every reading as
JSONL over USB serial (4 Hz by default) and shows health on the board's RGB LED.

Supported sensors (auto-detected): LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D, LSM303AGR.

## Build Dependencies

### Arch Linux

```bash
sudo pacman -S --needed cmake make git python arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-binutils
paru -S --needed picotool
```

### Ubuntu/Debian

```bash
sudo apt-get install -y \
  git build-essential cmake \
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib
```

## Building

```bash
cd firmware
mkdir build && cd build
cmake ..     # Fetches the Pico SDK automatically unless PICO_SDK_PATH is set.
make -j
```

Build options (pass to `cmake` as `-D<NAME>=<value>`):

| Option                  | Default | Meaning                                                                                    |
|-------------------------|---------|--------------------------------------------------------------------------------------------|
| `REPORT_INTERVAL_MS`    | `250`   | Default report interval. Also changeable at runtime with `interval <ms>`.                           |
| `LSM303_LEGACY_VARIANT` | `AUTO`  | `AUTO`, `DLH`, or `DLHC`. The DLH and DLHC look nearly identical on the bus; force one here if auto-detection guesses wrong. |
| `STATUS_LED_GRB_ORDER`  | `ON`    | WS2812 colour order is GRB. Set `OFF` if red and green on the LED are swapped (RGB-order LED). |

## Flashing

1. Hold **BOOTSEL** on the RP2040-Zero while plugging it in. Or, hold **BOOTSEL** and press the reset button.
   Or, if the firmware is already running, send the `bootsel` command over serial.
2. See that it mounts as a USB drive.
3. Drag `build/rp2040_imu_monitor.uf2` onto it.
4. It'll reboot and run.

## Serial Output (JSONL)

Open the USB serial port (any baud rate works; it's USB-CDC):

```bash
screen /dev/ttyACM0 115200
# or, to log:
cat /dev/ttyACM0 | tee imu.jsonl
```

Each line is one JSON object with a `type` field.

### `sample` (every report interval)

```json
{"type":"sample","t_ms":12250,"seq":48,"health":"ok","sensor":"LSM303DLHC","read_ok":true,
 "acc_raw":[-12,30,1015],"acc_g":[-0.0120,0.0300,1.0150],"acc_mag_g":1.0155,
 "mag_raw":[210,-85,-402],"mag_uT":[19.09,-7.73,-41.02],"mag_mag_uT":45.90,
 "roll_deg":1.69,"pitch_deg":0.68,"heading_deg":337.95,
 "imu_temp_c":24.50,"imu_temp_raw":36,"mcu_temp_c":27.81,
 "data_frozen":false,"plausible":true,"i2c_err":0,"fail_streak":0,"reinit":0,"cfg_restore":0,
 "wdt_reboot":false,"interval_ms":250}
```

(Shown wrapped here; on the wire it's one line.)

| Field                          | Meaning                                                                                       |
|--------------------------------|-----------------------------------------------------------------------------------------------|
| `t_ms`                         | Milliseconds since boot.                                                                      |
| `seq`                          | Sample counter; gaps mean dropped lines.                                                      |
| `health`                       | `ok`, `degraded`, or `no_sensor` (same as the LED).                                           |
| `sensor`                       | Detected chip, or `null`.                                                                     |
| `read_ok`                      | `false` if this report's sensor read failed; values below are then the last good reading.    |
| `acc_raw`, `mag_raw`           | Raw sensor counts, X/Y/Z.                                                                     |
| `acc_g`, `acc_mag_g`           | Acceleration in g, and its magnitude (≈1.0 at rest).                                          |
| `mag_uT`, `mag_mag_uT`         | Magnetic field in µT, and its magnitude (Earth's field is ~25-65 µT).                         |
| `roll_deg`, `pitch_deg`        | Tilt from the accelerometer.                                                                  |
| `heading_deg`                  | Tilt-compensated heading of the board's +X axis, 0-360. **Uncalibrated** (no hard/soft-iron correction); calibrate on the host from `mag_raw` for accuracy. |
| `imu_temp_c`, `imu_temp_raw`   | Sensor die temperature (DLHC, D, AGR only; `null` on DLH/DLM). ST doesn't specify the absolute offset, so treat it as relative. |
| `mcu_temp_c`                   | RP2040 internal temperature sensor (±a few °C).                                               |
| `data_frozen`                  | Readings have been bit-for-bit identical for ≥2 s (stuck sensor).                             |
| `plausible`                    | Accel magnitude is within 0.3-2.0 g.                                                          |
| `i2c_err`                      | Total I2C errors since boot.                                                                  |
| `fail_streak`                  | Consecutive failed reads (sensor is dropped and re-detected at 3).                            |
| `reinit`                       | Bus recoveries + re-detection attempts.                                                       |
| `cfg_restore`                  | Times the sensor was found to have lost its configuration (e.g. brown-out) and was rewritten. |
| `wdt_reboot`                   | The last reset was a watchdog timeout.                                                        |
| `interval_ms`                  | Current report interval.                                                                      |

When no sensor is present, all sensor fields are `null` but the line keeps the same shape.

### Other record types

- `boot`: firmware/build info, watchdog reset info (`wdt_last_stage` says which part of the loop hung), sensor, interval.
  Printed whenever a host opens the serial port.
- `event`: `sensor_detected`, `sensor_lost`, `sensor_config_restored`, `interval_changed`, etc.
- `help`: one line per command.
- `error`: bad command input.

## Serial Commands

Type a command and press Enter.

| Command          | Action                                             |
|------------------|----------------------------------------------------|
| `help`           | List commands.                                     |
| `info`           | Re-print the `boot` record.                        |
| `interval <ms>`  | Set the report interval (50-60000 ms).             |
| `reinit`         | Recover the I2C bus and re-detect the sensor.      |
| `clearwdt`       | Clear the watchdog LED marker.                     |
| `reboot`         | Reboot.                                            |
| `bootsel`        | Reboot into the UF2 bootloader for flashing.       |

## Status LED

| Pattern                          | Meaning                                                            |
|----------------------------------|--------------------------------------------------------------------|
| Solid blue                       | Booting.                                                           |
| Green blip, once per second      | Healthy.                                                           |
| Yellow double-blip               | Degraded: I2C error in the last 5 s, stuck data, or implausible accel. |
| Fast red blink                   | No sensor detected / sensor lost (retrying every second).          |
| Extra magenta blip mid-cycle     | Last reset was a watchdog timeout (clear with `clearwdt`).         |

## Safety Features

- **Watchdog** (2 s). Fed from the main loop and before each USB write. After a watchdog reset, the `boot`
  record reports it, along with the loop stage it hung in and a count of watchdog resets since power-on.
- **Bounded I2C**: every transaction has a 5 ms timeout.
- **I2C bus recovery**: clocks out a stuck slave (9 SCL pulses + STOP) at boot and whenever the sensor is lost.
- **Automatic re-detection** of the sensor once per second while it's missing (hot-plug tolerant).
- **Config read-back** every 5 s, which catches the sensor silently resetting to power-down after a supply glitch.
- **Stuck-data and plausibility checks** on the readings.

## IDE Support

After building, enable IDE support by creating a symbolic link to `compile_commands.json` in the build directory:

```bash
cd firmware
ln -s build/compile_commands.json compile_commands.json
```
