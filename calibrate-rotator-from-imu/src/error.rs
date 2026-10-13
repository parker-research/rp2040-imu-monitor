//! Failure classes. Each maps to a distinct process exit code so scripts can
//! tell "the glue broke" apart from "the serial cable fell out".

#[derive(thiserror::Error, Debug)]
pub enum Fail {
    #[error("config: {0}")]
    Config(String),
    #[error("IMU: {0}")]
    Imu(String),
    #[error("rotator: {0}")]
    Rotator(String),
    /// The rotator was rejected by rotctld (limits / invalid parameter).
    #[error("rotator rejected command: {0}")]
    Rejected(String),
    #[error("MECHANICAL LINK CHECK FAILED: {0}")]
    Glue(String),
    #[error("magnetometer: {0}")]
    Magnetic(String),
    #[error("did not converge: {0}")]
    Converge(String),
}

impl Fail {
    pub fn exit_code(&self) -> i32 {
        match self {
            Fail::Config(_) => 2,
            Fail::Imu(_) => 3,
            Fail::Rotator(_) | Fail::Rejected(_) => 4,
            Fail::Glue(_) => 5,
            Fail::Converge(_) => 6,
            Fail::Magnetic(_) => 7,
        }
    }
}

pub type R<T> = Result<T, Fail>;
