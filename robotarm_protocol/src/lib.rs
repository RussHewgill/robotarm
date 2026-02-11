#![no_std] // Important so the MCU can use it
use serde::{Deserialize, Serialize};

/// Data from MCU to Controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SerialLogMessage {
    MotorData {
        id: u8,
        timestamp: u64,
        target: f32,
        position: f32,
        velocity: f32,
    },
}

/// Data from Controller to MCU
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SerialCommand {
    SetMotorTarget { id: u8, target: f32 },
}
