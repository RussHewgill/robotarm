use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::ENCODER_CALIBRATION_CHUNK_SIZE;
use crate::types::*;

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
