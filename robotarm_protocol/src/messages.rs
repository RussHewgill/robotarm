use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::ENCODER_CALIBRATION_CHUNK_SIZE;
use crate::types::*;

// /// Data from MCU to Controller
// #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// pub enum SerialLogMessage {
//     HandshakeResponse {
//         hardware_id: u64,
//     },
//     PingResponse {
//         id: MotorId,
//     },
//     MotorData {
//         id: MotorId,
//         timestamp: u64,
//         // motion_control: MotionControlType,
//         position: f32,
//         angle: f32,
//         velocity: f32,
//         target_position: f32,
//         target_velocity: f32,
//         motor_current: f32,
//         sensor_currents: Option<(f32, f32)>,
//         motor_voltage: (f32, f32),
//         feed_forward: f32,
//         aux_velocity: f32,
//     },
//     // MotorDataExtended {
//     //     id: MotorId,
//     //     timestamp: u64,
//     //     motor_current: f32,
//     //     sensor_currents: Option<(f32, f32)>,
//     //     motor_voltage: (f32, f32),
//     //     feed_forward: f32,
//     //     aux_velocity: f32,
//     // },
//     LogData {
//         id: MotorId,
//         // timestamp: u64,
//         start_new: bool,
//         data: [f32; 8],
//     },
//     ReportConfig {
//         id: MotorId,
//         config: MotorConfigValue,
//     },
//     // PIDDebugData {
//     //     id: MotorId,
//     //     timestamp: u64,
//     //     // (error, p_term, i_term, d_term)
//     //     pid_internals_vel: (f32, f32, f32, f32),
//     //     pid_internals_pos: (f32, f32, f32, f32),
//     // },
//     DebugData {
//         id: MotorId,
//         timestamp: u64,
//         zero_electrical_angle: f32,
//         encoder_calibration_enabled: bool,
//     },
//     FocLoopRate {
//         id: MotorId,
//         timestamp: u64,
//         loop_rate_hz: f32,
//     },
//     EncoderData {
//         id: MotorId,
//         timestamp: u64,
//         position: f32,
//         velocity: Option<f32>,
//     },
//     // MotorPID {
//     //     id: MotorId,
//     //     vel_p: f32,
//     //     vel_i: f32,
//     //     vel_d: f32,
//     //     // vel_ramp: f32,
//     //     vel_limit: f32,
//     //     angle_p: f32,
//     //     angle_i: f32,
//     //     angle_d: f32,
//     //     // angle_ramp: f32,
//     //     angle_limit: f32,
//     //     lpf_vel: f32,
//     //     lpf_angle: f32,
//     //     vel_feed_forward: f32,
//     //     vel_i_band: f32,
//     //     vel_d_lpf: f32,
//     //     // pos_feed_forward: f32,
//     //     pos_i_band: f32,
//     //     pos_d_lpf: f32,
//     // },
//     MotorADRC {
//         id: MotorId,
//         b0: f32,
//         speed_factor: f32,
//         observer_bandwidth: f32,
//         controller_bandwidth: f32,
//     },
//     ADRCDebugData {
//         id: MotorId,
//         timestamp: u64,
//         vs: [f32; 2],
//         state: [f32; 3],
//         u: f32,
//         u0: f32,
//     },
//     EncoderCalibrationData {
//         id: MotorId,
//         index: usize,
//         chunk: [f32; ENCODER_CALIBRATION_CHUNK_SIZE],
//     },
// }

// /// Data from MCU to Controller
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialLogMessage {
    HandshakeResponse {
        hardware_id: u64,
    },
    PingResponse {
        id: MotorId,
    },
    MotorsConfigs {
        id: MotorId,
        // configs: heapless::Vec<MotorConfigValue, 16>,
        // configs: [MotorConfigValue; 16],
        configs: MotorConfigs,
    },
    MotorStatus {
        id: MotorId,
        timestamp: u64,
        position: f32,
        velocity: Option<f32>,
    },
    ADRCStatus {
        id: MotorId,
        b0: f32,
        speed_factor: f32,
        observer_bandwidth: f32,
        controller_bandwidth: f32,
    },
    ADRCDebugData {
        id: MotorId,
        timestamp: u64,
        vs: [f32; 2],
        state: [f32; 3],
        u: f32,
        u0: f32,
    },
    EncoderCalibrationData {
        id: MotorId,
        index: usize,
        chunk: [f32; ENCODER_CALIBRATION_CHUNK_SIZE],
    },
}

impl SerialLogMessage {
    pub fn id(&self) -> MotorId {
        match self {
            SerialLogMessage::HandshakeResponse { .. } => MotorId(0),
            SerialLogMessage::PingResponse { id } => *id,
            SerialLogMessage::MotorsConfigs { id, .. } => *id,
            SerialLogMessage::MotorStatus { id, .. } => *id,
            // SerialLogMessage::MotorDataExtended { id, .. } => *id,
            // SerialLogMessage::DebugData { id, .. } => *id,
            // SerialLogMessage::EncoderData { id, .. } => *id,
            // SerialLogMessage::MotorPID { id, .. } => *id,
            SerialLogMessage::ADRCStatus { id, .. } => *id,
            SerialLogMessage::ADRCDebugData { id, .. } => *id,
            // SerialLogMessage::FocLoopRate { id, .. } => *id,
            // SerialLogMessage::PIDDebugData { id, .. } => *id,
            // SerialLogMessage::LogData { id, .. } => *id,
            SerialLogMessage::EncoderCalibrationData { id, .. } => *id,
            // _ => MotorId::UNSET,
        }
    }
}
