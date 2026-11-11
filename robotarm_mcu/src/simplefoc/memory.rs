use defmt::{debug, error, info, warn};
use robotarm_protocol::saved_config::SavedConfig;
use serde::{Deserialize, Serialize};

use crate::hardware::m24c64::M24C64;

const VERSION: u32 = 1;
const MAGIC_BYTES: [u8; 4] = [0xd2, 0xc9, 0x99, 0xb3];
pub const EEPROM_DATA_LEN: usize = 32 * 10; // 320

pub struct MemoryController<I2C> {
    eeprom: M24C64<I2C>,
    address_id: usize,
    address_a: usize,
    address_b: usize,
    counter: u32,
}

impl<I2C> MemoryController<I2C> {
    pub fn new(eeprom: M24C64<I2C>) -> Self {
        Self {
            eeprom,
            address_id: 0x000,
            address_a: 0x008,
            address_b: 0x408,
            counter: 0,
        }
    }
}

impl<I2C: embedded_hal_async::i2c::I2c> MemoryController<I2C> {
    pub async fn save_id(&mut self, id: u8) -> Result<(), ()> {
        if let Err(e) = self.eeprom.write(self.address_id, &[id]).await {
            error!("Failed to write ID to EEPROM: {:?}", e);
        }

        let id_read = self.load_id().await?;
        if id_read != id {
            error!("ID mismatch: expected {}, got {}", id, id_read);
            return Err(());
        }

        Ok(())
    }

    pub async fn load_id(&mut self) -> Result<u8, ()> {
        let mut buf = [0u8; 1];
        if let Err(e) = self.eeprom.read(self.address_id, &mut buf).await {
            error!("Failed to read ID from EEPROM: {:?}", e);
            return Err(());
        }
        Ok(buf[0])
    }

    pub async fn save(&mut self, config: &SavedConfig, sequence: u32) {
        let mut buf = [0u8; EEPROM_DATA_LEN];
        let data = match postcard::to_slice(config, &mut buf) {
            Ok(slice) => slice,
            Err(_) => {
                error!("Failed to serialize config");
                return;
            }
        };

        // let mut buf = heapless::Vec::<u8, EEPROM_DATA_LEN>::new();

        // buf.extend_from_slice(&MAGIC_BYTES).unwrap();
        // buf.extend_from_slice(&VERSION.to_le_bytes()).unwrap();
        // buf.extend_from_slice(&self.counter.to_le_bytes()).unwrap();
        // buf.extend_from_slice(&(data.len() as u32).to_le_bytes())
        //     .unwrap();
        // buf.extend_from_slice(&data).unwrap();

        // self.counter += 1;

        // let crc = crc32fast::hash(&buf);
        // buf.extend_from_slice(&crc.to_le_bytes()).unwrap();

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

        let address = match self.choose_write_slot().await {
            Ok(Slot::SlotA) => self.address_a,
            Ok(Slot::SlotB) => self.address_b,
            Err(_) => self.address_a,
        };

        if let Err(e) = self.eeprom.write(address, serialized).await {
            error!("Failed to write to EEPROM: {:?}", e);
        }
    }

    /// True = slot A, False = slot B
    async fn choose_write_slot(&mut self) -> Result<Slot, ()> {
        let mut buf = [0u8; EEPROM_DATA_LEN];

        self.eeprom
            .read(self.address_a, &mut buf)
            .await
            .map_err(|_| ())?;
        let data_a: Result<SavedValues, _> = postcard::from_bytes(&buf);

        self.eeprom
            .read(self.address_b, &mut buf)
            .await
            .map_err(|_| ())?;
        let data_b: Result<SavedValues, _> = postcard::from_bytes(&buf);

        match (data_a, data_b) {
            (Ok(a), Ok(b)) => {
                let check_a = a.check_crc();
                let check_b = b.check_crc();
                if check_a && check_b {
                    if a.sequence >= b.sequence {
                        Ok(Slot::SlotB)
                    } else {
                        Ok(Slot::SlotA)
                    }
                } else if check_a {
                    Ok(Slot::SlotB)
                } else if check_b {
                    Ok(Slot::SlotA)
                } else {
                    Ok(Slot::SlotA)
                }
            }
            (Ok(a), Err(_)) => {
                let check_a = a.check_crc();
                if check_a {
                    Ok(Slot::SlotB)
                } else {
                    Ok(Slot::SlotA)
                }
            }
            (Err(_), Ok(b)) => {
                let check_b = b.check_crc();
                if check_b {
                    Ok(Slot::SlotA)
                } else {
                    Ok(Slot::SlotB)
                }
            }
            (Err(_), Err(_)) => Ok(Slot::SlotA),
        }
    }

    pub async fn load(&mut self) -> Option<SavedConfig> {
        let mut buf = [0u8; EEPROM_DATA_LEN];

        self.eeprom.read(self.address_a, &mut buf).await.ok()?;

        let data_a: Result<SavedValues, _> = postcard::from_bytes(&buf);

        self.eeprom.read(self.address_b, &mut buf).await.ok()?;

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

enum Slot {
    SlotA,
    SlotB,
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
