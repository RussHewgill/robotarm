use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::{SerialCommand, SerialLogMessage};

// #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
// pub struct SerialLogMessagePacket {
//     pub msg: SerialLogMessage,
//     pub crc32: u32,
// }

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SerialPacket {
    LogMessage(SerialLogMessage),
    Command(SerialCommand),
}
