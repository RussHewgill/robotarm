use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorId(pub u8);

impl MotorId {
    pub const UNSET: Self = Self(255);
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorConfigValue {
    DirClockwise(bool),
    MotorParams {
        pole_pairs: u8,
        phase_resistance: f32,
        motor_kv: f32,
    },
    ZeroElectricalAngle(Option<f32>),
    DebugLogging(bool),
    VoltageLimit(f32),
    EncoderCalibrationEnabled(bool),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotionControlType {
    Torque,
    Velocity,
    Angle,
    VelocityOpenLoop,
    AngleOpenLoop,
}

impl MotionControlType {
    pub fn is_open_loop(&self) -> bool {
        matches!(
            self,
            MotionControlType::VelocityOpenLoop | MotionControlType::AngleOpenLoop
        )
    }
}

// #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize, Default)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// pub struct PIDSettings {
//     pub p: Option<f32>,
//     pub i: Option<f32>,
//     pub d: Option<f32>,
//     // ramp: Option<f32>,
//     pub limit: Option<f32>,
//     pub i_band: Option<f32>,
//     pub d_lpf: Option<f32>,
//     pub feed_forward: Option<f32>,
// }

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ADRCParam {
    B0(Option<f32>),
    SpeedFactor(Option<f32>),
    ObserverBandwidth(Option<f32>),
    ControllerBandwidth(Option<f32>),
}
