//! The JSONL record emitted by the IMU firmware.

use crate::math::V3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sample {
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub t_ms: u64,
    #[serde(default)]
    pub seq: u64,
    #[serde(default)]
    pub health: String,
    #[serde(default)]
    pub sensor: Option<String>,
    #[serde(default)]
    pub read_ok: bool,
    #[serde(default)]
    pub acc_raw: Option<[i64; 3]>,
    #[serde(default)]
    pub acc_g: Option<[f64; 3]>,
    #[serde(default)]
    pub acc_mag_g: Option<f64>,
    #[serde(default)]
    pub mag_raw: Option<[i64; 3]>,
    #[serde(default, rename = "mag_uT")]
    pub mag_ut: Option<[f64; 3]>,
    #[serde(default, rename = "mag_mag_uT")]
    pub mag_mag_ut: Option<f64>,
    #[serde(default)]
    pub roll_deg: Option<f64>,
    #[serde(default)]
    pub pitch_deg: Option<f64>,
    #[serde(default)]
    pub heading_deg: Option<f64>,
    #[serde(default)]
    pub imu_temp_c: Option<f64>,
    #[serde(default)]
    pub imu_temp_raw: Option<i64>,
    #[serde(default)]
    pub mcu_temp_c: Option<f64>,
    #[serde(default)]
    pub data_frozen: bool,
    #[serde(default)]
    pub plausible: bool,
    #[serde(default)]
    pub i2c_err: u64,
    #[serde(default)]
    pub fail_streak: u64,
    #[serde(default)]
    pub reinit: u64,
    #[serde(default)]
    pub cfg_restore: u64,
    #[serde(default)]
    pub wdt_reboot: bool,
    #[serde(default)]
    pub interval_ms: Option<u64>,
}

impl Sample {
    /// Why this sample must not be used, if anything.
    pub fn reject_reason(&self) -> Option<&'static str> {
        if self.health != "ok" {
            return Some("health!=ok");
        }
        if !self.read_ok {
            return Some("read_ok=false");
        }
        if !self.plausible {
            return Some("plausible=false");
        }
        if self.data_frozen {
            return Some("data_frozen");
        }
        if self.wdt_reboot {
            return Some("wdt_reboot");
        }
        if self.fail_streak > 0 {
            return Some("fail_streak>0");
        }
        let finite = |a: &Option<[f64; 3]>| a.map(|v| v.iter().all(|x| x.is_finite())).unwrap_or(false);
        if !finite(&self.acc_g) {
            return Some("acc missing");
        }
        if !finite(&self.mag_ut) {
            return Some("mag missing");
        }
        None
    }
    pub fn acc(&self) -> V3 {
        V3(self.acc_g.unwrap_or_default())
    }
    pub fn mag(&self) -> V3 {
        V3(self.mag_ut.unwrap_or_default())
    }
    /// Counters that, if they change, mean the sensor was reconfigured mid-window.
    pub fn config_epoch(&self) -> (u64, u64) {
        (self.reinit, self.cfg_restore)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const LINE: &str = r#"{"type":"sample","t_ms":348009,"seq":1392,"health":"ok","sensor":"LSM303DLH","read_ok":true,"acc_raw":[45,-8,-1005],"acc_g":[0.0450,-0.0080,-1.0050],"acc_mag_g":1.0060,"mag_raw":[-243,507,-775],"mag_uT":[-23.03,48.06,-81.58],"mag_mag_uT":97.44,"roll_deg":-179.54,"pitch_deg":-2.56,"heading_deg":241.32,"imu_temp_c":null,"imu_temp_raw":null,"mcu_temp_c":26.67,"data_frozen":false,"plausible":true,"i2c_err":0,"fail_streak":0,"reinit":0,"cfg_restore":0,"wdt_reboot":false,"interval_ms":250}"#;

    #[test]
    fn parses_firmware_line() {
        let s: Sample = serde_json::from_str(LINE).unwrap();
        assert_eq!(s.seq, 1392);
        assert_eq!(s.mag().y(), 48.06);
        assert!(s.reject_reason().is_none());
        assert!(s.imu_temp_c.is_none());
    }

    #[test]
    fn rejects_unhealthy() {
        let s: Sample = serde_json::from_str(&LINE.replace("\"data_frozen\":false", "\"data_frozen\":true")).unwrap();
        assert_eq!(s.reject_reason(), Some("data_frozen"));
    }
}
