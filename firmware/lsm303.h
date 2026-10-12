/*
 * Minimal driver for the ST LSM303 family of accelerometer + magnetometer combo chips.
 *
 * Supported variants (auto-detected):
 *   - LSM303DLH   (accel 0x18/0x19, mag 0x1E)          no temperature sensor
 *   - LSM303DLM   (accel 0x18/0x19, mag 0x1E)          no temperature sensor
 *   - LSM303DLHC  (accel 0x19,      mag 0x1E)          temperature via magnetometer die
 *   - LSM303D     (single die at 0x1D or 0x1E)         temperature sensor
 *   - LSM303AGR   (accel 0x19,      mag 0x1E)          temperature via accelerometer die
 *
 * The DLH and DLHC are not reliably distinguishable over I2C (neither has a documented
 * ID register on the accelerometer). See LSM303_LEGACY_VARIANT in CMakeLists.txt.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

typedef enum {
    LSM303_NONE = 0,
    LSM303_DLH,
    LSM303_DLM,
    LSM303_DLHC,
    LSM303_D,
    LSM303_AGR,
} lsm303_variant_t;

typedef struct {
    i2c_inst_t      *i2c;
    lsm303_variant_t variant;
    uint8_t          acc_addr;
    uint8_t          mag_addr;
} lsm303_t;

typedef struct {
    int16_t acc_raw[3];    // Raw register values (after justification shift), X/Y/Z.
    int16_t mag_raw[3];    // Raw register values, X/Y/Z (re-ordered to X/Y/Z for DLM/DLHC).
    float   acc_g[3];
    float   mag_gauss[3];
    bool    has_temp;
    int16_t temp_raw;
    float   temp_c;        // Only meaningful if has_temp. Uncalibrated offset; see lsm303.c.
} lsm303_sample_t;

/// Probe the bus for any supported LSM303 variant. Fills `dev` on success.
bool lsm303_detect(lsm303_t *dev, i2c_inst_t *i2c);

/// Write the measurement configuration (and verify it by read-back).
bool lsm303_configure(const lsm303_t *dev);

/// Read back the configuration registers and check they still hold what we wrote.
/// A false return with `*io_ok == true` means the chip reset itself (e.g. a brown-out).
bool lsm303_verify_config(const lsm303_t *dev, bool *io_ok);

/// Read accel, mag (and temperature where supported).
bool lsm303_read(const lsm303_t *dev, lsm303_sample_t *out);

const char *lsm303_variant_name(lsm303_variant_t v);
