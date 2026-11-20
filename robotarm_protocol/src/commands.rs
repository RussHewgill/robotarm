use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::BAUD_RATE;
use crate::ENCODER_CALIBRATION_CHUNK_SIZE;
use crate::types::*;

/// Handshake:
/// 1. control app sends handshake request command
/// 2. MCU responds with handshake acknowledgment, containing unique hardware ID
/// 3. control app assigns a 1 byte ID to each MCU

// /// Data from Controller to MCU
// // #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
// // #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// pub enum SerialCommand {
//     BroadcastPositions {
//         positions: [(MotorId, f32); 6],
//     },
//     HandshakeRequest {
//         //
//     },
//     HandshakeAssignment {
//         hardware_id: u64,
//         new_id: MotorId,
//     },
//     Init {
//         id: MotorId,
//     },
//     PingRequest {
//         id: MotorId,
//         hardware_id: u64,
//     },
//     SaveConfigToFlash {
//         id: MotorId,
//     },
//     SetMotorConfig {
//         id: MotorId,
//         config: MotorConfigValue,
//     },
//     RequestSettings {
//         id: MotorId,
//     },
//     RequestDebugData {
//         id: MotorId,
//     },
//     SetEnabled {
//         id: MotorId,
//         enabled: bool,
//     },
//     SetDebugRate {
//         id: MotorId,
//         rate_hz: u16,
//     },
//     SetEncoderCalibration {
//         id: MotorId,
//         enable: bool,
//     },
//     SetMotionControl {
//         id: MotorId,
//         control_type: MotionControlType,
//     },
//     SetADRCParam {
//         id: MotorId,
//         adrc_settings: ADRCSettings,
//     },
//     SetMotorTarget {
//         id: MotorId,
//         target: f32,
//     },
//     ZeroPosition {
//         id: MotorId,
//     },
//     SetThermalConfig {
//         id: MotorId,
//         // TODO
//     },
//     ResetADRC {
//         id: MotorId,
//     },
//     SendEncoderCalibration {
//         id: MotorId,
//     },
//     SetEncoderCalibrationData {
//         id: MotorId,
//         index: usize,
//         chunk: [f32; ENCODER_CALIBRATION_CHUNK_SIZE],
//     },
// }

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
    PingRequest {
        id: MotorId,
        hardware_id: u64,
    },
    Init {
        id: MotorId,
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
    /// expects a MotorData response
    RequestMotorStatus {
        id: MotorId,
    },
    // expects an ADRCData response
    RequestADRCStatus {
        id: MotorId,
    },
    SetADRCParam {
        id: MotorId,
        adrc_param: ADRCParam,
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

    pub const fn is_broadcast(&self) -> bool {
        match self {
            SerialCommand::HandshakeRequest { .. } => true,
            SerialCommand::HandshakeAssignment { .. } => true,
            SerialCommand::BroadcastPositions { .. } => true,
            _ => false,
        }
    }

    pub const fn id(&self) -> MotorId {
        match self {
            SerialCommand::BroadcastPositions { .. } => MotorId::UNSET,
            SerialCommand::HandshakeRequest { .. } => MotorId::UNSET,
            SerialCommand::HandshakeAssignment { .. } => MotorId::UNSET,
            SerialCommand::PingRequest { id, .. } => *id,
            SerialCommand::Init { id } => *id,
            SerialCommand::SaveConfigToFlash { id } => *id,
            SerialCommand::SetMotorConfig { id, .. } => *id,
            SerialCommand::RequestSettings { id } => *id,
            SerialCommand::RequestMotorStatus { id } => *id,
            SerialCommand::SetADRCParam { id, .. } => *id,
            SerialCommand::RequestADRCStatus { id } => *id,
            SerialCommand::SetEncoderCalibrationData { id, .. } => *id,
            // _ => MotorId::UNSET,
        }
    }

    /// baud rate = 2 Mbps
    /// time to respond = packet size in bits / baud rate + margin
    /// on server, add time to transmit request
    ///
    /// processing time:
    /// FOC loop at 10-20 kHz
    /// assume one loop receive command, one to process, and one to send
    /// 3 periods, 100 - 200 us
    pub const fn response_expected(&self) -> Option<(MotorId, u64)> {
        // use crate::messages::SerialLogMessage;

        // let size = core::mem::size_of_val(self);
        // let timeout = (size as u64 * 8) / BAUD_RATE as u64;

        // let processing_time = 200;
        // let processing_time = 600;
        let processing_time = 100_000;

        match self {
            // SerialCommand::RequestSettings { id, .. } => {
            //     let size = MotorConfigs::POSTCARD_MAX_SIZE + 4 + core::mem::size_of_val(self);
            //     let timeout = (size as u64 * 8) / BAUD_RATE as u64 + processing_time;
            //     // Some((*id, timeout))
            //     unimplemented!()
            // }
            // SerialCommand::RequestMotorStatus { id } => {
            //     let response =
            //         crate::SerialPacket::LogMessage(crate::SerialLogMessage::MotorStatus {
            //             id: MotorId(0),
            //             timestamp: 0,
            //             position: 0.,
            //             velocity: Some(0.),
            //         });
            //     let size = core::mem::size_of_val(self) + core::mem::size_of_val(&response);
            //     let timeout = (size as u64 * 8) / BAUD_RATE as u64 + processing_time;
            //     // Some((*id, timeout))
            //     unimplemented!()
            // }
            // SerialCommand::RequestADRCStatus { id, .. } => {
            //     // Some((*id, timeout))
            //     unimplemented!()
            // }
            SerialCommand::PingRequest { id, .. } => {
                let size = core::mem::size_of_val(self)
                    + core::mem::size_of_val(&crate::SerialPacket::LogMessage(
                        crate::SerialLogMessage::PingResponse { id: MotorId(0) },
                    ));
                let baud_rate = BAUD_RATE as f32 / 1_000_000.0;
                let time = (size as f32 * 8.0) / baud_rate;

                // let timeout = 1 / (BAUD_RATE as u64 / (size as u64 * 8)) + processing_time;
                let timeout = time as u64 + processing_time;

                Some((*id, timeout))
            }
            _ => None,
        }
    }
}

// impl SerializablePacket for SerialCommand {
//     fn is_command(&self) -> bool {
//         true
//     }

//     fn serialize(&self, buf: &mut [u8]) -> Option<usize> {
//         // crate::serialize::serialize(self, buf)
//         unimplemented!()
//     }

//     #[cfg(feature = "std")]
//     fn serialize_std(&self) -> Option<std::vec::Vec<u8>> {
//         // crate::serialize::serialize_std(self)
//         unimplemented!()
//     }
// }
