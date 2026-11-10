use serde::{Deserialize, Serialize};

use crate::ADRCSettings;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SavedConfig {
    id: u8,

    motor_direction_cw: bool,
    motor_pole_pairs: u8,
    motor_phase_resistance: f32,
    motor_kv: f32,
    motor_zero_electrical_angle: Option<f32>,
    motor_voltage_limit: f32,

    encoder_calibration: Option<EncoderCalibration>,

    adrc: ADRCSettings,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EncoderCalibration {
    #[serde(with = "serde_arrays")]
    pub calibration: [f32; 256],
}

// impl SavedConfig {
//     pub fn to_configs(&self) -> Vec<SerialCommand> {
//         let mut out = vec![];
//         out.push(SerialCommand::MotorDirectionCw(self.motor_direction_cw));
//         out.push(SerialCommand::MotorPolePairs(self.motor_pole_pairs));
//         out.push(SerialCommand::MotorPhaseResistance(
//             self.motor_phase_resistance,
//         ));
//         out.push(SerialCommand::MotorKv(self.motor_kv));
//         out.push(SerialCommand::MotorZeroElectricalAngle(
//             self.motor_zero_electrical_angle,
//         ));
//         out.push(SerialCommand::MotorVoltageLimit(self.motor_voltage_limit));
//         out
//     }
// }
