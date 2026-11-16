#![no_std]
// pub mod packet;
pub mod saved_config;
pub mod types;

use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

pub use crate::types::*;

const ENCODER_CALIBRATION_CHUNK_SIZE: usize = 8;

/// Data from MCU to Controller
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialLogMessage {
    HandshakeResponse {
        hardware_id: u64,
    },
    PingResponse {
        id: MotorId,
    },
    MotorData {
        id: MotorId,
        timestamp: u64,
        motion_control: MotionControlType,
        position: f32,
        angle: f32,
        velocity: f32,
        target_position: f32,
        target_velocity: f32,
        motor_current: f32,
        sensor_currents: Option<(f32, f32)>,
        motor_voltage: (f32, f32),
        feed_forward: f32,
        aux_velocity: f32,
    },
    // MotorDataExtended {
    //     id: MotorId,
    //     timestamp: u64,
    //     motor_current: f32,
    //     sensor_currents: Option<(f32, f32)>,
    //     motor_voltage: (f32, f32),
    //     feed_forward: f32,
    //     aux_velocity: f32,
    // },
    LogData {
        id: MotorId,
        // timestamp: u64,
        start_new: bool,
        data: [f32; 8],
    },
    ReportConfig {
        id: MotorId,
        config: MotorConfigValue,
    },
    // PIDDebugData {
    //     id: MotorId,
    //     timestamp: u64,
    //     // (error, p_term, i_term, d_term)
    //     pid_internals_vel: (f32, f32, f32, f32),
    //     pid_internals_pos: (f32, f32, f32, f32),
    // },
    DebugData {
        id: MotorId,
        timestamp: u64,
        zero_electrical_angle: f32,
        encoder_calibration_enabled: bool,
    },
    FocLoopRate {
        id: MotorId,
        timestamp: u64,
        loop_rate_hz: f32,
    },
    EncoderData {
        id: MotorId,
        timestamp: u64,
        position: f32,
        velocity: Option<f32>,
    },
    // MotorPID {
    //     id: MotorId,
    //     vel_p: f32,
    //     vel_i: f32,
    //     vel_d: f32,
    //     // vel_ramp: f32,
    //     vel_limit: f32,
    //     angle_p: f32,
    //     angle_i: f32,
    //     angle_d: f32,
    //     // angle_ramp: f32,
    //     angle_limit: f32,
    //     lpf_vel: f32,
    //     lpf_angle: f32,

    //     vel_feed_forward: f32,
    //     vel_i_band: f32,
    //     vel_d_lpf: f32,

    //     // pos_feed_forward: f32,
    //     pos_i_band: f32,
    //     pos_d_lpf: f32,
    // },
    MotorADRC {
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
            SerialLogMessage::MotorData { id, .. } => *id,
            // SerialLogMessage::MotorDataExtended { id, .. } => *id,
            SerialLogMessage::ReportConfig { id, .. } => *id,
            SerialLogMessage::DebugData { id, .. } => *id,
            SerialLogMessage::EncoderData { id, .. } => *id,
            // SerialLogMessage::MotorPID { id, .. } => *id,
            SerialLogMessage::MotorADRC { id, .. } => *id,
            SerialLogMessage::FocLoopRate { id, .. } => *id,
            // SerialLogMessage::PIDDebugData { id, .. } => *id,
            SerialLogMessage::LogData { id, .. } => *id,
            SerialLogMessage::ADRCDebugData { id, .. } => *id,
            SerialLogMessage::EncoderCalibrationData { id, .. } => *id,
        }
    }
}

/// Handshake:
/// 1. control app sends handshake request command
/// 2. MCU responds with handshake acknowledgment, containing unique hardware ID
/// 3. control app assigns a 1 byte ID to each MCU

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SerialCommandPacket {
    pub command: heapless::Vec<MotorId, 64>,
    pub crc: u32,
    pub msg_id: u64,
}

/// Data from Controller to MCU
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialCommand {
    BroadcastPositions {
        positions: [(MotorId, f32); 6],
    },
    HandshakeRequest {
        //
    },
    HandshakeAssignment {
        hardware_id: u64,
        new_id: MotorId,
    },
    Init {
        id: MotorId,
    },
    PingRequest {
        id: MotorId,
        hardware_id: u64,
    },
    SaveConfigToFlash {
        id: MotorId,
    },
    SetMotorConfig {
        id: MotorId,
        config: MotorConfigValue,
    },
    RequestSettings {
        id: MotorId,
    },
    RequestDebugData {
        id: MotorId,
    },
    SetEnabled {
        id: MotorId,
        enabled: bool,
    },
    SetDebugRate {
        id: MotorId,
        rate_hz: u16,
    },
    SetEncoderCalibration {
        id: MotorId,
        enable: bool,
    },
    SetMotionControl {
        id: MotorId,
        control_type: MotionControlType,
    },
    SetADRCParam {
        id: MotorId,
        adrc_settings: ADRCSettings,
    },
    SetMotorTarget {
        id: MotorId,
        target: f32,
    },
    ZeroPosition {
        id: MotorId,
    },
    SetThermalConfig {
        id: MotorId,
        // TODO
    },
    ResetADRC {
        id: MotorId,
    },
    SendEncoderCalibration {
        id: MotorId,
    },
    SetEncoderCalibrationData {
        id: MotorId,
        index: usize,
        chunk: [f32; ENCODER_CALIBRATION_CHUNK_SIZE],
    },
}

impl SerialCommand {
    pub fn is_handshake(&self) -> bool {
        match self {
            SerialCommand::HandshakeRequest { .. } => true,
            SerialCommand::HandshakeAssignment { .. } => true,
            _ => false,
        }
    }

    pub fn is_broadcast(&self) -> bool {
        match self {
            // SerialCommand::HandshakeRequest { .. } => true,
            // SerialCommand::HandshakeAssignment { .. } => true,
            _ => false,
        }
    }

    pub fn id(&self) -> MotorId {
        match self {
            SerialCommand::BroadcastPositions { .. } => MotorId(0),
            SerialCommand::Init { id } => *id,
            SerialCommand::HandshakeRequest { .. } => MotorId(0),
            SerialCommand::HandshakeAssignment { .. } => MotorId(0),
            SerialCommand::PingRequest { id, .. } => *id,
            SerialCommand::SaveConfigToFlash { id } => *id,
            SerialCommand::RequestSettings { id } => *id,
            SerialCommand::RequestDebugData { id } => *id,
            SerialCommand::SetEnabled { id, .. } => *id,
            SerialCommand::SetDebugRate { id, .. } => *id,
            SerialCommand::SetEncoderCalibration { id, .. } => *id,
            SerialCommand::SetMotionControl { id, .. } => *id,
            // SerialCommand::SetModeTorque { id } => *id,
            // SerialCommand::SetModeVelocityOpenLoop { id } => *id,
            // SerialCommand::SetModeVelocity { id } => *id,
            // SerialCommand::SetModeAngle { id } => *id,
            // SerialCommand::SetModeAngleOpenLoop { id } => *id,
            // SerialCommand::SetVelocityPID { id, .. } => *id,
            // SerialCommand::SetAnglePID { id, .. } => *id,
            // SerialCommand::SetLPF { id, .. } => *id,
            // SerialCommand::SetFeedForward { id, .. } => *id,
            SerialCommand::SetMotorTarget { id, .. } => *id,
            SerialCommand::ZeroPosition { id } => *id,
            // SerialCommand::SetVoltageLimit { id, .. } => *id,
            // SerialCommand::SetZeroElectricalAngle { id, .. } => *id,
            SerialCommand::SetADRCParam { id, .. } => *id,
            SerialCommand::SetMotorConfig { id, .. } => *id,
            // SerialCommand::ToggleDebugLogging { id } => *id,
            SerialCommand::SetThermalConfig { id } => *id,
            SerialCommand::ResetADRC { id } => *id,
            SerialCommand::SendEncoderCalibration { id, .. } => *id,
            SerialCommand::SetEncoderCalibrationData { id, .. } => *id,
        }
    }
}
