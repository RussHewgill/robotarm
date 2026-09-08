#![no_std]
pub mod types;

use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

pub use crate::types::*;

/// Data from MCU to Controller
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialLogMessage {
    // Ping,
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
        motor_voltage: (f32, f32),
        feed_forward: f32,
        pid_outputs: (f32, f32),
        // pid_internals_vel: (f32, f32, f32, f32),
        // pid_internals_pos: (f32, f32, f32, f32),
    },
    LogData {
        id: u8,
        // timestamp: u64,
        start_new: bool,
        data: [f32; 8],
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
}

impl SerialLogMessage {
    pub fn id(&self) -> u8 {
        match self {
            SerialLogMessage::MotorData { id, .. } => *id,
            SerialLogMessage::DebugData { id, .. } => *id,
            SerialLogMessage::EncoderData { id, .. } => *id,
            // SerialLogMessage::MotorPID { id, .. } => *id,
            SerialLogMessage::MotorADRC { id, .. } => *id,
            SerialLogMessage::FocLoopRate { id, .. } => *id,
            // SerialLogMessage::PIDDebugData { id, .. } => *id,
            SerialLogMessage::LogData { id, .. } => *id,
            SerialLogMessage::ADRCDebugData { id, .. } => *id,
        }
    }
}

/// Data from Controller to MCU
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialCommand {
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
    // SetSensorOffset {
    //     id: u8,
    //     offset: Option<f32>,
    // },
    SetDebugRate {
        id: u8,
        rate_hz: u16,
    },
    SetEncoderCalibration {
        id: u8,
        enable: bool,
    },
    SetModeTorque {
        id: u8,
    },
    SetModeVelocityOpenLoop {
        id: u8,
    },
    SetModeVelocity {
        id: u8,
    },
    SetModeAngle {
        id: u8,
    },
    SetModeAngleOpenLoop {
        id: u8,
    },
    SetVelocityPID {
        id: u8,
        pid_settings: PIDSettings,
    },
    SetAnglePID {
        id: u8,
        pid_settings: PIDSettings,
    },
    SetADRCParam {
        id: u8,
        adrc_settings: ADRCSettings,
    },
    SetLPF {
        id: u8,
        lpf_vel: Option<f32>,
        lpf_angle: Option<f32>,
    },
    SetFeedForward {
        id: u8,
        ff: f32,
    },
    SetMotorTarget {
        id: u8,
        target: f32,
    },
    ZeroPosition {
        id: u8,
    },
    SetVoltageLimit {
        id: u8,
        voltage_limit: f32,
    },
    SetZeroElectricalAngle {
        id: u8,
        angle: f32,
    },
}

impl SerialCommand {
    pub fn id(&self) -> u8 {
        match self {
            SerialCommand::RequestSettings { id } => *id,
            SerialCommand::RequestDebugData { id } => *id,
            SerialCommand::SetEnabled { id, .. } => *id,
            SerialCommand::SetDebugRate { id, .. } => *id,
            SerialCommand::SetEncoderCalibration { id, .. } => *id,
            SerialCommand::SetModeTorque { id } => *id,
            SerialCommand::SetModeVelocityOpenLoop { id } => *id,
            SerialCommand::SetModeVelocity { id } => *id,
            SerialCommand::SetModeAngle { id } => *id,
            SerialCommand::SetModeAngleOpenLoop { id } => *id,
            SerialCommand::SetVelocityPID { id, .. } => *id,
            SerialCommand::SetAnglePID { id, .. } => *id,
            SerialCommand::SetLPF { id, .. } => *id,
            SerialCommand::SetFeedForward { id, .. } => *id,
            SerialCommand::SetMotorTarget { id, .. } => *id,
            SerialCommand::ZeroPosition { id } => *id,
            SerialCommand::SetVoltageLimit { id, .. } => *id,
            SerialCommand::SetZeroElectricalAngle { id, .. } => *id,
            SerialCommand::SetADRCParam { id, .. } => *id,
        }
    }
}
