//! Calibration file (TOML). Produced by `calibrate-rotator-from-imu calibrate`, consumed by
//! `calibrate-rotator-from-imu zero` / `calibrate-rotator-from-imu check`. The [limits], [thresholds] and [timing]
//! tables are hand-editable; everything else is measured.

use crate::error::{Fail, R};
use crate::math::V3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;

pub const VERSION: u32 = 2;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Config {
    pub version: u32,
    pub created: String,
    pub imu_source: String,
    pub reference: Reference,
    pub mount: Mount,
    pub elevation: ElCal,
    pub azimuth: AzCal,
    pub signs: Signs,
    #[serde(default)]
    pub survey: SurveyStats,
    #[serde(default)]
    pub limits: Limits,
    #[serde(default)]
    pub thresholds: Thresholds,
    #[serde(default)]
    pub timing: Timing,
    #[serde(default)]
    pub sync: Sync,
    #[serde(default)]
    pub motion: Motion,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MoveMethod {
    /// Raw G-code through rotctld (`w <gcode_template>`). Bypasses hamlib's
    /// min/max, so targets outside the controller's frame (e.g. negative
    /// after a power cycle) are reachable. [limits] still apply.
    #[default]
    Gcode,
    /// rotctld `P az el`, subject to rotctld's own min/max.
    Hamlib,
}

/// How position commands reach the controller.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Motion {
    pub method: MoveMethod,
    /// `{az}` / `{el}` are replaced by the target in G-code units.
    pub gcode_template: String,
    /// Degrees of rotator motion per G-code unit (both axes).
    pub deg_per_gcode_unit: f64,
}

impl Default for Motion {
    fn default() -> Self {
        Motion { method: MoveMethod::Gcode, gcode_template: "G0 X{az} Y{el}".into(), deg_per_gcode_unit: 9.0 }
    }
}

impl Motion {
    /// `[motion]` from an existing config file, even one too old to load
    /// fully, so hand edits survive re-running `calibrate`.
    pub fn from_file(path: &Path) -> R<Option<Motion>> {
        let Ok(s) = std::fs::read_to_string(path) else { return Ok(None) };
        let v: toml::Value = toml::from_str(&s).map_err(|e| Fail::Config(format!("{}: {e}", path.display())))?;
        let Some(m) = v.get("motion") else { return Ok(None) };
        let m: Motion = m.clone().try_into().map_err(|e| Fail::Config(format!("{} [motion]: {e}", path.display())))?;
        m.validate()?;
        Ok(Some(m))
    }

    pub fn validate(&self) -> R<()> {
        if !(self.deg_per_gcode_unit.is_finite() && self.deg_per_gcode_unit > 0.0) {
            return Err(Fail::Config("[motion] deg_per_gcode_unit must be > 0".into()));
        }
        if self.method == MoveMethod::Gcode
            && !(self.gcode_template.contains("{az}") && self.gcode_template.contains("{el}"))
        {
            return Err(Fail::Config("[motion] gcode_template must contain {az} and {el}".into()));
        }
        Ok(())
    }
}

/// How to make the controller adopt the current position as az=el=0.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Sync {
    /// rotctld command sent once the antenna is at IMU-true zero.
    /// Remove (or use `zero --no-sync`) to only report the offset.
    pub command: Option<String>,
    /// Wait this long before polling the controller again.
    pub boot_wait_s: f64,
    /// Give up if rotctld can't read a position within this.
    pub reconnect_timeout_s: f64,
    /// After restart the rotator must report within this of 0/0.
    pub expect_zero_tol_deg: f64,
}

impl Default for Sync {
    fn default() -> Self {
        Sync {
            command: Some("w [ESP444]RESTART".into()),
            boot_wait_s: 3.0,
            reconnect_timeout_s: 45.0,
            expect_zero_tol_deg: 0.5,
        }
    }
}

/// IMU readings captured with the rotator physically at true az=0 el=0.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Reference {
    /// Rotator-reported position when the reference was captured.
    pub rotator_reported_az: f64,
    pub rotator_reported_el: f64,
    pub acc_g: V3,
    pub mag_ut: V3,
    pub acc_noise_g: f64,
    pub mag_noise_ut: f64,
    pub samples: usize,
    pub mcu_temp_c: Option<f64>,
    pub fw_roll_deg: Option<f64>,
    pub fw_pitch_deg: Option<f64>,
    pub fw_heading_deg: Option<f64>,
}

/// The "mounting transform": how the IMU sits relative to the rotator.
/// Rotator frame: x = boresight (horizontal at el=0), y = elevation axis
/// (left), z = up. Positive elevation tilts +x toward +z.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Mount {
    pub description: Vec<String>,
    /// Rows are the rotator x/y/z axes expressed in IMU coordinates.
    pub imu_to_rotator: [[f64; 3]; 3],
    /// Orientation of the IMU in the rotator frame, Rz(yaw)·Ry(pitch)·Rx(roll).
    pub imu_yaw_pitch_roll_deg: [f64; 3],
    pub up_imu: V3,
    pub el_axis_imu: V3,
    pub boresight_imu: V3,
    /// Elevation axis tilt from horizontal (non-level el axle).
    pub el_axis_tilt_deg: f64,
    /// Azimuth axis tilt from vertical (rotator/mast not plumb).
    pub az_axis_tilt_deg: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MoveRecord {
    pub axis: String,
    pub commanded_deg: f64,
    pub reported_deg: f64,
    pub imu_deg: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ElCal {
    /// IMU-measured degrees per rotator-reported degree.
    pub gain: f64,
    pub step_deg: f64,
    /// Angle between gravity and elevation axis at zero (ideal 90).
    pub cone_angle_deg: f64,
    /// Worst IMU disagreement on returning to zero.
    pub return_to_zero_err_deg: f64,
    pub moves: Vec<MoveRecord>,
}

/// Magnetic model:
///   m_imu = soft_iron_imu · Mᵀ·Rot(el_axis, el)·(Rz(sign·az)·earth_rot + stage_field) + hard_iron_imu
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FieldModel {
    pub hard_iron_imu_ut: V3,
    /// Symmetric soft-iron / sensor-gain matrix (IMU frame, trace 3). Identity
    /// unless the envelope survey ran.
    #[serde(default = "identity")]
    pub soft_iron_imu: [[f64; 3]; 3],
    /// Earth field in the rotator frame at true zero (x boresight, y left, z up).
    pub earth_rot_ut: V3,
    /// Field from iron/motors on the azimuth stage: turns with azimuth but
    /// not with elevation (rotator frame at el=0). Zero unless surveyed.
    #[serde(default)]
    pub stage_field_ut: V3,
    pub fit_rms_ut: f64,
    pub points: usize,
}

fn identity() -> [[f64; 3]; 3] {
    crate::math::M3::I.0
}

/// Summary of the envelope survey done by `calibrate` (`calibrate --report`
/// writes every point).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SurveyStats {
    pub performed: bool,
    pub points: usize,
    pub az_step_deg: f64,
    pub el_levels_deg: Vec<f64>,
    /// Worst |IMU elevation − rotator elevation| over the survey.
    pub el_residual_max_deg: f64,
    /// Azimuth model error over the points used for the fit.
    pub az_residual_rms_deg: f64,
    pub az_residual_max_deg: f64,
    /// Azimuth model error on the held-out "pass" path, before it was added to the fit.
    pub validation_points: usize,
    pub validation_az_rms_deg: f64,
    pub validation_az_max_deg: f64,
    /// Range of |horizontal field| / model over all points (ideal 1).
    pub h_ratio_min: f64,
    pub h_ratio_max: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AzCal {
    /// +1: magnetometer frame rotation agrees with hamlib (clockwise) azimuth.
    pub sign: f64,
    /// Model degrees per rotator-reported degree (after field model).
    pub gain: f64,
    pub step_deg: f64,
    pub swept: bool,
    /// Residual deviation table: [model_az_deg, model_minus_true_deg].
    pub deviation: Vec<[f64; 2]>,
    pub deviation_periodic: bool,
    pub horizontal_field_ut: f64,
    pub dip_deg: f64,
    pub single_sample_noise_deg: f64,
    pub field_model: FieldModel,
    pub moves: Vec<MoveRecord>,
}

/// Human-readable record of how each IMU quantity responds to +1° of
/// commanded rotator motion (units per rotator degree).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Signs {
    pub summary: Vec<String>,
    pub elevation_primary_field: String,
    pub elevation_primary_sign: i32,
    pub azimuth_primary_field: String,
    pub azimuth_primary_sign: i32,
    pub elevation: BTreeMap<String, f64>,
    pub azimuth: BTreeMap<String, f64>,
}

/// Physical travel limits in TRUE (IMU-referenced) coordinates, not in the
/// controller's reported frame, which is arbitrary after a power cycle.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Limits {
    pub az_min: f64,
    pub az_max: f64,
    pub el_min: f64,
    pub el_max: f64,
    /// Refuse to correct more than this in one run. Off by default: after a
    /// power cycle the antenna can legitimately be anywhere.
    pub max_correction_el_deg: f64,
    pub max_correction_az_deg: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            az_min: 0.0,
            az_max: 360.0,
            el_min: 0.0,
            el_max: 85.0,
            max_correction_el_deg: 90.0,
            max_correction_az_deg: 180.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Thresholds {
    pub el_tol_deg: f64,
    pub az_tol_deg: f64,
    /// Rotator considered "at target" within this.
    pub pos_tol_deg: f64,
    /// Corrections below this that don't move the rotator = rotator resolution.
    pub min_step_deg: f64,
    pub max_iter: u32,
    /// Allowed |IMU delta − expected delta| = abs + rel·|move|.
    pub glue_el_abs_deg: f64,
    pub glue_el_rel: f64,
    pub glue_az_abs_deg: f64,
    pub glue_az_rel: f64,
    /// Gravity must stay on the cone around the elevation axis within this.
    pub off_axis_deg: f64,
    /// Elevation change allowed during an azimuth-only move (plus az tilt).
    pub cross_el_deg: f64,
    /// During motion: IMU must show at least this fraction of rotator travel...
    pub follow_min_frac: f64,
    /// ...once the rotator has travelled this far.
    pub follow_min_travel_el_deg: f64,
    pub follow_min_travel_az_deg: f64,
    pub acc_mag_min_g: f64,
    pub acc_mag_max_g: f64,
    pub acc_still_std_g: f64,
    /// Horizontal field magnitude must match the model within this fraction.
    pub mag_h_rel_tol: f64,
    /// Vertical field must match the model within this.
    pub mag_v_abs_tol_ut: f64,
    pub az_se_target_deg: f64,
    pub az_se_max_deg: f64,
    pub temp_warn_delta_c: f64,
}

impl Default for Thresholds {
    fn default() -> Self {
        Thresholds {
            el_tol_deg: 0.3,
            az_tol_deg: 1.0,
            pos_tol_deg: 1.0,
            min_step_deg: 1.0,
            max_iter: 8,
            glue_el_abs_deg: 1.0,
            glue_el_rel: 0.15,
            glue_az_abs_deg: 2.5,
            glue_az_rel: 0.15,
            off_axis_deg: 2.0,
            cross_el_deg: 1.0,
            follow_min_frac: 0.3,
            follow_min_travel_el_deg: 4.0,
            follow_min_travel_az_deg: 15.0,
            acc_mag_min_g: 0.85,
            acc_mag_max_g: 1.15,
            acc_still_std_g: 0.02,
            mag_h_rel_tol: 0.20,
            mag_v_abs_tol_ut: 8.0,
            az_se_target_deg: 0.25,
            az_se_max_deg: 1.0,
            temp_warn_delta_c: 15.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Timing {
    pub poll_ms: u64,
    pub settle_acc_s: f64,
    /// Longer: rotator motor/brake currents disturb the magnetometer.
    pub settle_mag_s: f64,
    pub stop_detect_s: f64,
    pub stall_s: f64,
    pub move_base_timeout_s: f64,
    pub min_speed_dps: f64,
    pub sample_timeout_s: f64,
    pub samples_acc: usize,
    pub samples_mag_min: usize,
    pub samples_mag_max: usize,
    pub samples_ref: usize,
    /// Time-compression factor (simulator only).
    #[serde(skip, default = "one")]
    pub speed: f64,
}

fn one() -> f64 {
    1.0
}

impl Default for Timing {
    fn default() -> Self {
        Timing {
            poll_ms: 250,
            settle_acc_s: 1.0,
            settle_mag_s: 2.5,
            stop_detect_s: 1.5,
            stall_s: 6.0,
            move_base_timeout_s: 20.0,
            min_speed_dps: 0.5,
            sample_timeout_s: 3.0,
            samples_acc: 12,
            samples_mag_min: 16,
            samples_mag_max: 64,
            samples_ref: 40,
            speed: 1.0,
        }
    }
}

impl Timing {
    pub fn d(&self, secs: f64) -> Duration {
        Duration::from_secs_f64((secs / self.speed).max(0.0005))
    }
    pub fn poll(&self) -> Duration {
        self.d(self.poll_ms as f64 / 1000.0)
    }
}

impl Config {
    pub fn load(path: &Path) -> R<Config> {
        let s = std::fs::read_to_string(path).map_err(|e| {
            Fail::Config(format!("cannot read {}: {e} (run `calibrate-rotator-from-imu calibrate` first)", path.display()))
        })?;
        let c: Config = toml::from_str(&s).map_err(|e| Fail::Config(format!("{}: {e}", path.display())))?;
        c.motion.validate()?;
        if c.version != VERSION {
            return Err(Fail::Config(format!(
                "{} is config version {}, this build needs version {VERSION}: re-run `calibrate-rotator-from-imu calibrate`",
                path.display(),
                c.version
            )));
        }
        Ok(c)
    }

    pub fn save(&self, path: &Path) -> R<()> {
        let mut v = toml::Value::try_from(self).map_err(|e| Fail::Config(e.to_string()))?;
        round_floats(&mut v);
        let body = toml::to_string(&v).map_err(|e| Fail::Config(e.to_string()))?;
        let header = "# calibrate-rotator-from-imu calibration file — generated by `calibrate-rotator-from-imu calibrate`.\n\
# Measured sections describe the IMU mounting transform and the sign/gain of\n\
# IMU response to rotator motion. [limits], [thresholds] and [timing] may be\n\
# edited by hand. Re-run `calibrate-rotator-from-imu calibrate` if the IMU or antenna is re-mounted.\n\n";
        let tmp = path.with_extension("toml.tmp");
        std::fs::write(&tmp, format!("{header}{body}"))
            .and_then(|_| std::fs::rename(&tmp, path))
            .map_err(|e| Fail::Config(format!("cannot write {}: {e}", path.display())))
    }
}

/// Keep the file readable: 6 decimal places is far below sensor resolution.
fn round_floats(v: &mut toml::Value) {
    match v {
        toml::Value::Float(f) => *f = (*f * 1e6).round() / 1e6,
        toml::Value::Array(a) => a.iter_mut().for_each(round_floats),
        toml::Value::Table(t) => t.iter_mut().for_each(|(_, x)| round_floats(x)),
        _ => {}
    }
}
