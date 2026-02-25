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
        motor_voltage: (f32, f32),
    },
    MotorPID {
        id: u8,
        vel_p: f32,
        vel_i: f32,
        vel_d: f32,
        vel_ramp: f32,
        vel_limit: f32,
        angle_p: f32,
        angle_i: f32,
        angle_d: f32,
        angle_ramp: f32,
        angle_limit: f32,
        lpf_vel: f32,
        lpf_angle: f32,
    },
}

/// Data from Controller to MCU
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialCommand {
    RequestSettings {
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
    SetVelocityPID {
        id: u8,
        p: Option<f32>,
        i: Option<f32>,
        d: Option<f32>,
        ramp: Option<f32>,
        limit: Option<f32>,
    },
    SetAnglePID {
        id: u8,
        p: Option<f32>,
        i: Option<f32>,
        d: Option<f32>,
        ramp: Option<f32>,
        limit: Option<f32>,
    },
    SetLPF {
        id: u8,
        lpf_vel: Option<f32>,
        lpf_angle: Option<f32>,
    },
    SetMotorTarget {
        id: u8,
        target: f32,
    },
    ZeroPosition {
        id: u8,
    },
}
