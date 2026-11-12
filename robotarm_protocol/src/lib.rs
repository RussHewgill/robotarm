#![no_std]
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
        id: u8,
    },
    MotorData {
        id: u8,
        timestamp: u64,
        motion_control: MotionControlType,
        position: f32,
        angle: f32,
        velocity: f32,
        target_position: f32,
        target_velocity: f32,
        motor_current: f32,
        sensor_currents: Option<(f32, f32)>,
        // raw_currents: Option<(f32, f32)>,
        motor_voltage: (f32, f32),
        feed_forward: f32,
        aux_velocity: f32,
        // pid_outputs: (f32, f32),
        // pid_internals_vel: (f32, f32, f32, f32),
        // pid_internals_pos: (f32, f32, f32, f32),
    },
    LogData {
        id: u8,
        // timestamp: u64,
        start_new: bool,
        data: [f32; 8],
    },
    ReportConfig {
        id: u8,
        config: MotorConfigValue,
    },
    // PIDDebugData {
    //     id: u8,
    //     timestamp: u64,
    //     // (error, p_term, i_term, d_term)
    //     pid_internals_vel: (f32, f32, f32, f32),
    //     pid_internals_pos: (f32, f32, f32, f32),
    // },
    DebugData {
        id: u8,
        timestamp: u64,
        zero_electrical_angle: f32,
        encoder_calibration_enabled: bool,
    },
    FocLoopRate {
        id: u8,
        timestamp: u64,
        loop_rate_hz: f32,
    },
    EncoderData {
        id: u8,
        timestamp: u64,
        position: f32,
        velocity: Option<f32>,
    },
    // MotorPID {
    //     id: u8,
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
        id: u8,
        b0: f32,
        speed_factor: f32,
        observer_bandwidth: f32,
        controller_bandwidth: f32,
    },
    ADRCDebugData {
        id: u8,
        timestamp: u64,
        vs: [f32; 2],
        state: [f32; 3],
        u: f32,
        u0: f32,
    },
    EncoderCalibrationData {
        id: u8,
        index: usize,
        chunk: [f32; ENCODER_CALIBRATION_CHUNK_SIZE],
    },
}

impl SerialLogMessage {
    pub fn id(&self) -> u8 {
        match self {
            SerialLogMessage::HandshakeResponse { .. } => 0,
            SerialLogMessage::PingResponse { id } => *id,
            SerialLogMessage::MotorData { id, .. } => *id,
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

/// Data from Controller to MCU
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialCommand {
    HandshakeRequest {
        //
    },
    HandshakeAssignment {
        hardware_id: u64,
        new_id: u8,
    },
    Init {
        id: u8,
    },
    PingRequest {
        id: u8,
        hardware_id: u64,
    },
    SaveConfigToFlash {
        id: u8,
    },
    SetMotorConfig {
        id: u8,
        config: MotorConfigValue,
    },
    RequestSettings {
        id: u8,
    },
    RequestDebugData {
        id: u8,
    },
    SetEnabled {
        id: u8,
        enabled: bool,
    },
    SetDebugRate {
        id: u8,
        rate_hz: u16,
    },
    SetEncoderCalibration {
        id: u8,
        enable: bool,
    },
    SetMotionControl {
        id: u8,
        control_type: MotionControlType,
    },
    SetADRCParam {
        id: u8,
        adrc_settings: ADRCSettings,
    },
    SetMotorTarget {
        id: u8,
        target: f32,
    },
    ZeroPosition {
        id: u8,
    },
    SetThermalConfig {
        id: u8,
        // TODO
    },
    ResetADRC {
        id: u8,
    },
    SendEncoderCalibration {
        id: u8,
    },
    SetEncoderCalibrationData {
        id: u8,
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

    pub fn id(&self) -> u8 {
        match self {
            SerialCommand::Init { id } => *id,
            SerialCommand::HandshakeRequest { .. } => 0,
            SerialCommand::HandshakeAssignment { .. } => 0,
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
