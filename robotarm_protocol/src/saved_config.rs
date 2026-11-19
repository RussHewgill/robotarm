use serde::{Deserialize, Serialize};

use crate::{ADRCParam, MotorId};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SavedConfig {
    pub id: MotorId,

    pub motor_direction_cw: Option<bool>,
    pub motor_pole_pairs: Option<u8>,
    pub motor_phase_resistance: Option<f32>,
    pub motor_kv: Option<f32>,
    pub motor_zero_electrical_angle: Option<f32>,
    pub motor_voltage_limit: f32,

    pub encoder_calibration: Option<EncoderCalibration>,

    pub adrc: Option<heapless::Vec<ADRCParam, 4>>,
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
