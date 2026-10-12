#include "lsm303.h"

#include <stddef.h>

// Per-transaction I2C timeout. Bounds every bus operation so a stuck bus can never hang the
// main loop long enough to trip the watchdog.
static const uint32_t I2C_TIMEOUT_US = 5000;

// Setting the MSB of the sub-address enables register auto-increment on the accelerometer
// dies (and on the LSM303D). The DLH/DLM/DLHC/AGR magnetometer dies auto-increment always.
#define AUTO_INC 0x80u

// ---------------------------------------------------------------------------
// I2C addresses + ID registers
// ---------------------------------------------------------------------------
#define ADDR_ACC_SA0_LOW   0x18
#define ADDR_ACC_SA0_HIGH  0x19
#define ADDR_MAG           0x1E
#define ADDR_D_SA0_HIGH    0x1D
#define ADDR_D_SA0_LOW     0x1E

#define REG_WHO_AM_I       0x0F  // LSM303D (0x49), DLM mag (0x3C), DLHC/AGR accel (0x33)
#define REG_WHO_AM_I_M_AGR 0x4F  // LSM303AGR mag (0x40)
#define REG_IRA_REG_M      0x0A  // DLH/DLM/DLHC mag: 'H' (0x48)

#define ID_D       0x49
#define ID_DLM_MAG 0x3C
#define ID_ACC_33  0x33
#define ID_AGR_MAG 0x40
#define ID_IRA     0x48

// ---------------------------------------------------------------------------
// Configuration tables: (die, register, value). Written in order and verified by read-back.
// ---------------------------------------------------------------------------
typedef enum { DIE_ACC, DIE_MAG } die_t;

typedef struct {
    die_t   die;
    uint8_t reg;
    uint8_t val;
} reg_write_t;

// DLH / DLM: CTRL_REG1_A = normal power mode, 50 Hz, XYZ on. CTRL_REG4_A = BDU, +/-2 g.
// Mag: CRA = 15 Hz, CRB = +/-1.3 gauss, MR = continuous.
static const reg_write_t CFG_DLH_DLM[] = {
    { DIE_ACC, 0x20, 0x27 },
    { DIE_ACC, 0x23, 0x80 },
    { DIE_MAG, 0x00, 0x10 },
    { DIE_MAG, 0x01, 0x20 },
    { DIE_MAG, 0x02, 0x00 },
};

// DLHC: CTRL_REG1_A = 50 Hz, XYZ on. CTRL_REG4_A = BDU, high-res, +/-2 g.
// Mag: CRA = TEMP_EN + 15 Hz, CRB = +/-1.3 gauss, MR = continuous.
static const reg_write_t CFG_DLHC[] = {
    { DIE_ACC, 0x20, 0x47 },
    { DIE_ACC, 0x23, 0x88 },
    { DIE_MAG, 0x00, 0x90 },
    { DIE_MAG, 0x01, 0x20 },
    { DIE_MAG, 0x02, 0x00 },
};

// LSM303D (single die; DIE_ACC == DIE_MAG address):
// CTRL1 = 50 Hz accel, BDU, XYZ on. CTRL2 = +/-2 g. CTRL5 = TEMP_EN, high-res mag, 50 Hz.
// CTRL6 = +/-2 gauss. CTRL7 = continuous mag.
static const reg_write_t CFG_D[] = {
    { DIE_ACC, 0x20, 0x5F },
    { DIE_ACC, 0x21, 0x00 },
    { DIE_ACC, 0x24, 0xF0 },
    { DIE_ACC, 0x25, 0x00 },
    { DIE_ACC, 0x26, 0x00 },
};

// LSM303AGR: TEMP_CFG_REG_A = temp on. CTRL_REG1_A = 50 Hz, XYZ on. CTRL_REG4_A = BDU, HR, +/-2 g.
// Mag: CFG_REG_A_M = temp compensation, 20 Hz, continuous. CFG_REG_C_M = BDU.
static const reg_write_t CFG_AGR[] = {
    { DIE_ACC, 0x1F, 0xC0 },
    { DIE_ACC, 0x20, 0x47 },
    { DIE_ACC, 0x23, 0x88 },
    { DIE_MAG, 0x60, 0x84 },
    { DIE_MAG, 0x62, 0x10 },
};

// Offsets for the on-die temperature sensors. ST only specifies the slope (8 LSB/degC for
// DLHC/D) and not the absolute zero, so these are approximate; treat imu_temp_c as relative.
static const float DLHC_TEMP_OFFSET_C = 20.0f;
static const float D_TEMP_OFFSET_C    = 25.0f;
static const float AGR_TEMP_OFFSET_C  = 25.0f;

// ---------------------------------------------------------------------------
// Bus helpers
// ---------------------------------------------------------------------------
static bool read_regs(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_timeout_us(i2c, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) return false;
    return i2c_read_timeout_us(i2c, addr, buf, len, false, I2C_TIMEOUT_US) == (int)len;
}

static bool write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = { reg, val };
    return i2c_write_timeout_us(i2c, addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

/// Read a single ID-style register. Returns -1 if the device didn't ACK.
static int probe_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg) {
    uint8_t v;
    return read_regs(i2c, addr, reg, &v, 1) ? (int)v : -1;
}

static void config_table(lsm303_variant_t v, const reg_write_t **tbl, size_t *n) {
    switch (v) {
        case LSM303_DLH:
        case LSM303_DLM:  *tbl = CFG_DLH_DLM; *n = sizeof(CFG_DLH_DLM) / sizeof(CFG_DLH_DLM[0]); return;
        case LSM303_DLHC: *tbl = CFG_DLHC;    *n = sizeof(CFG_DLHC) / sizeof(CFG_DLHC[0]);       return;
        case LSM303_D:    *tbl = CFG_D;       *n = sizeof(CFG_D) / sizeof(CFG_D[0]);             return;
        case LSM303_AGR:  *tbl = CFG_AGR;     *n = sizeof(CFG_AGR) / sizeof(CFG_AGR[0]);         return;
        default:          *tbl = NULL;        *n = 0;                                            return;
    }
}

static uint8_t die_addr(const lsm303_t *dev, die_t die) {
    return (die == DIE_ACC) ? dev->acc_addr : dev->mag_addr;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool lsm303_detect(lsm303_t *dev, i2c_inst_t *i2c) {
    dev->i2c = i2c;
    dev->variant = LSM303_NONE;
    dev->acc_addr = 0;
    dev->mag_addr = 0;

    // LSM303D: single die with a proper WHO_AM_I.
    const uint8_t d_addrs[] = { ADDR_D_SA0_HIGH, ADDR_D_SA0_LOW };
    for (size_t i = 0; i < sizeof(d_addrs); i++) {
        if (probe_reg(i2c, d_addrs[i], REG_WHO_AM_I) == ID_D) {
            dev->variant = LSM303_D;
            dev->acc_addr = d_addrs[i];
            dev->mag_addr = d_addrs[i];
            return true;
        }
    }

    // Everything else has a separate magnetometer at 0x1E and accelerometer at 0x18/0x19.
    uint8_t acc_addr = 0;
    if (probe_reg(i2c, ADDR_ACC_SA0_HIGH, 0x20) >= 0) {        // CTRL_REG1_A exists on all.
        acc_addr = ADDR_ACC_SA0_HIGH;
    } else if (probe_reg(i2c, ADDR_ACC_SA0_LOW, 0x20) >= 0) {
        acc_addr = ADDR_ACC_SA0_LOW;
    } else {
        return false;
    }
    const int acc_id = probe_reg(i2c, acc_addr, REG_WHO_AM_I);

    lsm303_variant_t v = LSM303_NONE;
    if (probe_reg(i2c, ADDR_MAG, REG_WHO_AM_I_M_AGR) == ID_AGR_MAG && acc_id == ID_ACC_33) {
        v = LSM303_AGR;
    } else if (probe_reg(i2c, ADDR_MAG, REG_WHO_AM_I) == ID_DLM_MAG) {
        v = LSM303_DLM;
    } else if (probe_reg(i2c, ADDR_MAG, REG_IRA_REG_M) == ID_IRA) {
#if defined(LSM303_LEGACY_VARIANT_DLH)
        v = LSM303_DLH;
#elif defined(LSM303_LEGACY_VARIANT_DLHC)
        v = LSM303_DLHC;
#else
        // DLHC is fixed at 0x19 and its accel die answers 0x33 at 0x0F (undocumented, but
        // consistent). The DLH doesn't define that register.
        v = (acc_addr == ADDR_ACC_SA0_HIGH && acc_id == ID_ACC_33) ? LSM303_DLHC : LSM303_DLH;
#endif
    } else {
        return false;
    }

    dev->variant = v;
    dev->acc_addr = acc_addr;
    dev->mag_addr = ADDR_MAG;
    return true;
}

bool lsm303_configure(const lsm303_t *dev) {
    const reg_write_t *tbl;
    size_t n;
    config_table(dev->variant, &tbl, &n);
    if (tbl == NULL) return false;

    for (size_t i = 0; i < n; i++) {
        if (!write_reg(dev->i2c, die_addr(dev, tbl[i].die), tbl[i].reg, tbl[i].val)) return false;
    }
    bool io_ok;
    return lsm303_verify_config(dev, &io_ok);
}

bool lsm303_verify_config(const lsm303_t *dev, bool *io_ok) {
    const reg_write_t *tbl;
    size_t n;
    config_table(dev->variant, &tbl, &n);
    *io_ok = (tbl != NULL);
    if (tbl == NULL) return false;

    for (size_t i = 0; i < n; i++) {
        const int v = probe_reg(dev->i2c, die_addr(dev, tbl[i].die), tbl[i].reg);
        if (v < 0) {
            *io_ok = false;
            return false;
        }
        if ((uint8_t)v != tbl[i].val) return false;
    }
    return true;
}

static inline int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static inline int16_t be16(const uint8_t *p) { return (int16_t)((uint16_t)p[1] | ((uint16_t)p[0] << 8)); }

bool lsm303_read(const lsm303_t *dev, lsm303_sample_t *out) {
    uint8_t a[6], m[6], t[2];
    i2c_inst_t *i2c = dev->i2c;

    out->has_temp = false;
    out->temp_raw = 0;
    out->temp_c = 0.0f;

    // Accelerometer: little-endian X/Y/Z at OUT_X_L_A (0x28) on every variant.
    if (!read_regs(i2c, dev->acc_addr, 0x28 | AUTO_INC, a, 6)) return false;

    float acc_g_per_lsb;
    float mag_gauss_per_lsb_xy, mag_gauss_per_lsb_z;

    switch (dev->variant) {
        case LSM303_DLH:
        case LSM303_DLM:
        case LSM303_DLHC:
        case LSM303_AGR:
            // 12-bit left-justified.
            for (int i = 0; i < 3; i++) out->acc_raw[i] = (int16_t)(le16(&a[2 * i]) >> 4);
            acc_g_per_lsb = (dev->variant == LSM303_AGR) ? 0.00098f : 0.001f;
            break;
        case LSM303_D:
            for (int i = 0; i < 3; i++) out->acc_raw[i] = le16(&a[2 * i]);
            acc_g_per_lsb = 0.000061f;
            break;
        default:
            return false;
    }

    switch (dev->variant) {
        case LSM303_DLH:
            // Big-endian, X/Y/Z order. +/-1.3 gauss: 1055 LSB/G (XY), 950 LSB/G (Z).
            if (!read_regs(i2c, dev->mag_addr, 0x03, m, 6)) return false;
            out->mag_raw[0] = be16(&m[0]);
            out->mag_raw[1] = be16(&m[2]);
            out->mag_raw[2] = be16(&m[4]);
            mag_gauss_per_lsb_xy = 1.0f / 1055.0f;
            mag_gauss_per_lsb_z  = 1.0f / 950.0f;
            break;
        case LSM303_DLM:
        case LSM303_DLHC:
            // Big-endian, X/Z/Y order (!). +/-1.3 gauss: 1100 LSB/G (XY), 980 LSB/G (Z).
            if (!read_regs(i2c, dev->mag_addr, 0x03, m, 6)) return false;
            out->mag_raw[0] = be16(&m[0]);
            out->mag_raw[2] = be16(&m[2]);
            out->mag_raw[1] = be16(&m[4]);
            mag_gauss_per_lsb_xy = 1.0f / 1100.0f;
            mag_gauss_per_lsb_z  = 1.0f / 980.0f;
            break;
        case LSM303_D:
            // Little-endian X/Y/Z at OUT_X_L_M (0x08). +/-2 gauss: 0.080 mgauss/LSB.
            if (!read_regs(i2c, dev->mag_addr, 0x08 | AUTO_INC, m, 6)) return false;
            for (int i = 0; i < 3; i++) out->mag_raw[i] = le16(&m[2 * i]);
            mag_gauss_per_lsb_xy = mag_gauss_per_lsb_z = 0.000080f;
            break;
        case LSM303_AGR:
            // Little-endian X/Y/Z at OUTX_L_REG_M (0x68). 1.5 mgauss/LSB.
            if (!read_regs(i2c, dev->mag_addr, 0x68, m, 6)) return false;
            for (int i = 0; i < 3; i++) out->mag_raw[i] = le16(&m[2 * i]);
            mag_gauss_per_lsb_xy = mag_gauss_per_lsb_z = 0.0015f;
            break;
        default:
            return false;
    }

    for (int i = 0; i < 3; i++) out->acc_g[i] = (float)out->acc_raw[i] * acc_g_per_lsb;
    out->mag_gauss[0] = (float)out->mag_raw[0] * mag_gauss_per_lsb_xy;
    out->mag_gauss[1] = (float)out->mag_raw[1] * mag_gauss_per_lsb_xy;
    out->mag_gauss[2] = (float)out->mag_raw[2] * mag_gauss_per_lsb_z;

    // Temperature.
    switch (dev->variant) {
        case LSM303_DLHC:
            // TEMP_OUT_H_M (0x31), TEMP_OUT_L_M (0x32): 12-bit left-justified, 8 LSB/degC.
            if (!read_regs(i2c, dev->mag_addr, 0x31, t, 2)) return false;
            out->temp_raw = (int16_t)(be16(t) >> 4);
            out->temp_c = (float)out->temp_raw / 8.0f + DLHC_TEMP_OFFSET_C;
            out->has_temp = true;
            break;
        case LSM303_D:
            // TEMP_OUT_L (0x05), TEMP_OUT_H (0x06): 12-bit right-justified, 8 LSB/degC.
            if (!read_regs(i2c, dev->acc_addr, 0x05 | AUTO_INC, t, 2)) return false;
            out->temp_raw = (int16_t)((int16_t)(le16(t) << 4) >> 4);
            out->temp_c = (float)out->temp_raw / 8.0f + D_TEMP_OFFSET_C;
            out->has_temp = true;
            break;
        case LSM303_AGR:
            // OUT_TEMP_L_A (0x0C), OUT_TEMP_H_A (0x0D): left-justified, 10-bit in HR mode, 4 LSB/degC.
            if (!read_regs(i2c, dev->acc_addr, 0x0C | AUTO_INC, t, 2)) return false;
            out->temp_raw = (int16_t)(le16(t) >> 6);
            out->temp_c = (float)out->temp_raw / 4.0f + AGR_TEMP_OFFSET_C;
            out->has_temp = true;
            break;
        default:
            break;
    }

    return true;
}

const char *lsm303_variant_name(lsm303_variant_t v) {
    switch (v) {
        case LSM303_DLH:  return "LSM303DLH";
        case LSM303_DLM:  return "LSM303DLM";
        case LSM303_DLHC: return "LSM303DLHC";
        case LSM303_D:    return "LSM303D";
        case LSM303_AGR:  return "LSM303AGR";
        default:          return "none";
    }
}
