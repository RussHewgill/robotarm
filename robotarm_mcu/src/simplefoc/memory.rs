use defmt::{debug, error, info, warn};
use robotarm_protocol::saved_config::SavedConfig;
use serde::{Deserialize, Serialize};

pub const EEPROM_DATA_LEN: usize = 32 * 10; // 320

pub struct MemoryController {
    address_a: usize,
    address_b: usize,
    //
}

impl MemoryController {
    pub fn new() -> Self {
        Self {
            address_a: 0x000,
            address_b: 0x400,
        }
    }
}

impl MemoryController {
    pub async fn save<I2C: embedded_hal_async::i2c::I2c>(
        &self,
        eeprom: &mut crate::hardware::m24c64::M24C64<I2C>,
        config: &SavedConfig,
        sequence: u32,
    ) {
        let mut buf = [0u8; EEPROM_DATA_LEN];
        let data = match postcard::to_slice(config, &mut buf) {
            Ok(slice) => slice,
            Err(_) => {
                error!("Failed to serialize config");
                return;
            }
        };
        let saved_values = SavedValues {
            version: 1,
            sequence,
            len: data.len(),
            data: heapless::Vec::from_slice(&data).unwrap(),
            crc: crc32fast::hash(&data),
        };

        let serialized = postcard::to_slice(&saved_values, &mut buf);
        if let Err(_) = serialized {
            error!("Failed to serialize saved values");
            return;
        }
        let serialized = serialized.unwrap();

        if let Err(e) = eeprom.write(self.address_a, serialized).await {
            error!("Failed to write to EEPROM: {:?}", e);
        }
    }

    pub async fn load<I2C: embedded_hal_async::i2c::I2c>(
        &self,
        eeprom: &mut crate::hardware::m24c64::M24C64<I2C>,
    ) -> Option<SavedConfig> {
        let mut buf = [0u8; EEPROM_DATA_LEN];

        eeprom.read(self.address_a, &mut buf).await.ok()?;

        let data_a: Result<SavedValues, _> = postcard::from_bytes(&buf);

        eeprom.read(self.address_b, &mut buf).await.ok()?;

        let data_b: Result<SavedValues, _> = postcard::from_bytes(&buf);

        let data: SavedValues = match (data_a, data_b) {
            (Ok(a), Ok(b)) => {
                if a.sequence >= b.sequence {
                    if a.check_crc() {
                        a
                    } else if b.check_crc() {
                        b
                    } else {
                        return None;
                    }
                } else {
                    if b.check_crc() {
                        b
                    } else if a.check_crc() {
                        a
                    } else {
                        return None;
                    }
                }
            }
            (Err(_), Ok(b)) => {
                if b.check_crc() {
                    b
                } else {
                    return None;
                }
            }
            (Ok(a), Err(_)) => {
                if a.check_crc() {
                    a
                } else {
                    return None;
                }
            }
            (Err(_), Err(_)) => return None,
        };

        match postcard::from_bytes(&data.data) {
            Ok(config) => return Some(config),
            Err(_) => return None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct SavedValues {
    version: u32,
    sequence: u32,
    len: usize,
    data: heapless::Vec<u8, 288>,
    crc: u32,
}

impl SavedValues {
    fn check_crc(&self) -> bool {
        if self.data.len() != self.len {
            return false;
        }
        let checksum = crc32fast::hash(&self.data);

        checksum == self.crc
    }
}
