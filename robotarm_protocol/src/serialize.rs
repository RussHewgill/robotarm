#[cfg(feature = "std")]
use std::vec::Vec;

// use crate::SerialCommand;

// pub const CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);
// pub const CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_);
pub const CRC: crc::Crc<u32, crc::Table<1>> =
    crc::Crc::<u32, crc::Table<1>>::new(&crc::CRC_32_ISO_HDLC);
pub const CRC_SIZE: usize = CRC.algorithm.width as usize / 8;

/// 1. serialize with postcard
/// 2. calculate crc32 and append
/// 3. serialize with COBS
pub fn serialize<S: serde::Serialize>(val: S, buf: &mut [u8]) -> Option<usize> {
    let mut buf2 = [0u8; 256];
    let len = buf.len();
    let serialized = postcard::to_slice(&val, buf).unwrap();
    let crc = CRC.checksum(serialized);
    if serialized.len() + CRC_SIZE > len {
        return None;
    }
    buf2[..serialized.len()].copy_from_slice(serialized);
    buf2[serialized.len()..serialized.len() + CRC_SIZE].copy_from_slice(&crc.to_le_bytes());

    match postcard::to_slice(&buf2[..serialized.len() + 4], buf) {
        Ok(out) => Some(out.len()),
        Err(_) => None,
    }
}

#[cfg(feature = "std")]
pub fn serialize_std<S: serde::Serialize>(val: S) -> Option<Vec<u8>> {
    let Ok(mut buf) = postcard::to_stdvec(&val) else {
        return None;
    };
    let crc = CRC.checksum(&buf);
    buf.extend_from_slice(&crc.to_le_bytes());

    let Ok(out) = postcard::to_stdvec(&buf) else {
        return None;
    };
    Some(out)
}

// impl SerialCommand {
//     pub fn deserialize(buf: &[u8]) -> Option<Self> {
//         unimplemented!()
//     }

//     pub fn deserialize_std(buf: &[u8]) -> Option<Self> {
//         unimplemented!()
//     }
// }
