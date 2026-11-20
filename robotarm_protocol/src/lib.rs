#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod commands;
pub mod messages;
pub mod packet;
pub mod saved_config;
pub mod serialize;
pub mod types;

pub use crate::commands::*;
pub use crate::messages::*;
pub use crate::packet::*;
pub use crate::serialize::*;
pub use crate::types::*;

const ENCODER_CALIBRATION_CHUNK_SIZE: usize = 8;

pub const BAUD_RATE: u32 = 115_200;
// pub const BAUD_RATE: u64 = 921_600 * 2;
