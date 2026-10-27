use defmt::{debug, error, info, warn};

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

// https://github.com/Christopher-06/m24c64-driver/blob/master/src/

/// M24C64 EEPROM Driver
pub struct M24C64<I2C> {
    /// I2C Interface
    i2c: I2C,
    /// Address set by the E pins
    e_addr: u8,
    /// Command Buffer
    cmd_buf: [u8; 34],
}

impl<I2C> M24C64<I2C> {
    /// Create a new instance of the M24C64 Driver
    /// # Arguments
    /// * `i2c` - I2C Interface (from the embedded-hal(-async) crate)
    /// * `e_addr` - The address set on the E pins
    pub fn new(i2c: I2C, e_addr: u8) -> Self {
        Self {
            i2c,
            e_addr,
            cmd_buf: [0u8; 34],
        }
    }
}

pub enum M24C64Error {
    I2CReadError,
    I2CWriteError,
}

impl<I2C: embedded_hal_async::i2c::I2c> M24C64<I2C> {
    async fn write_raw(&mut self, address: usize, bytes: &[u8]) -> Result<(), M24C64Error> {
        self.cmd_buf[0] = (address >> 8) as u8;
        self.cmd_buf[1] = (address & 0xFF) as u8;
        self.cmd_buf[2..(bytes.len() + 2)].copy_from_slice(bytes);

        // Wait until the device is connected to the bus
        // After a write, the EEPROM disconnects itself from the bus until it can perform the write internally,
        // thus we have to continually poll the i2c bus for the device to be ready to receive new bytes.
        // while self.i2c.write(self.e_addr | 0x50, &[]).is_err() { }

        // Slight modification - keep track of the retries, since if we're over t_w from the datasheet, we want
        // to report an error instead of infinitely looping.
        let mut i = 0;
        loop {
            // Write the data to the EEPROM
            let result = self
                .i2c
                .write(self.e_addr | 0x50, &self.cmd_buf[0..bytes.len() + 2])
                .await;

            // Handle the result
            match result {
                Ok(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!(
                        "[EEPROM M24C64 Unit {:b}] Writing to address {:#x} took {} tries",
                        self.e_addr,
                        address,
                        i + 1
                    );

                    return Ok(());
                }
                Err(_) if i < 10 => (),
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    defmt::error!(
                        "[EEPROM M24C64 Unit {:b}] Writing to address {:#x} failed: {}",
                        self.e_addr,
                        address,
                        e.kind()
                    );

                    // return Err(e);
                    return Err(M24C64Error::I2CWriteError);
                }
            }

            i += 1;
            Timer::after_millis(1).await;
        }
    }
}

#[cfg(feature = "nope")]
impl<I2C> M24C64<I2C>
where
// I2C: embassy_rp::i2c::I2c<Error = E>,
// E: embassy_rp::i2c::Error,
{
    async fn write_raw_blocking(&mut self, address: usize, bytes: &[u8]) -> Result<(), E> {
        self.cmd_buf[0] = (address >> 8) as u8;
        self.cmd_buf[1] = (address & 0xFF) as u8;
        self.cmd_buf[2..(bytes.len() + 2)].copy_from_slice(bytes);

        // Wait until the device is connected to the bus
        // After a write, the EEPROM disconnects itself from the bus until it can perform the write internally,
        // thus we have to continually poll the i2c bus for the device to be ready to receive new bytes.
        // while self.i2c.write(self.e_addr | 0x50, &[]).is_err() { }

        // Slight modification - keep track of the retries, since if we're over t_w from the datasheet, we want
        // to report an error instead of infinitely looping.
        let mut i = 0;
        loop {
            let result = self
                .i2c
                .write(self.e_addr | 0x50, &self.cmd_buf[0..bytes.len() + 2]);

            match result {
                Ok(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!(
                        "[EEPROM M24C64 Unit {:b}] Writing to address {:#x} took {} tries",
                        self.e_addr,
                        address,
                        i + 1
                    );

                    return Ok(());
                }
                Err(_) if i < 10 => (),
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    defmt::error!(
                        "[EEPROM M24C64 Unit {:b}] Writing to address {:#x} failed: {}",
                        self.e_addr,
                        address,
                        e.kind()
                    );

                    return Err(e);
                }
            }

            i += 1;
            delay.delay_ms(1)
        }
    }

    fn read_raw_blocking(&mut self, address: usize, bytes: &mut [u8]) -> Result<(), E> {
        self.cmd_buf[0] = (address >> 8) as u8;
        self.cmd_buf[1] = (address & 0xFF) as u8;

        self.i2c
            .write_read(self.e_addr | 0x50, &self.cmd_buf[0..2], bytes)
    }

    /// Create a new blocking / synchronous instance of the M24C64 EEPROM driver
    /// # Arguments
    /// * `i2c` - I2C Interface (from the embedded-hal crate)
    /// * `e_addr` - The address set on the E pins
    pub fn new_blocking(i2c: I2C, e_addr: u8) -> Self {
        Self::new(i2c, e_addr)
    }

    /// Write an arbitrary number of bytes into the EEPROM, starting at `address`.
    /// This function will automatically paginate.
    pub fn write_blocking(
        &mut self,
        address: usize,
        data: &[u8],
        delay: &mut dyn DelayNs,
    ) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        defmt::info!(
            "[EEPROM M24C64 Unit {:b}] Writing to address {:#x} with {} bytes",
            self.e_addr,
            address,
            data.len()
        );

        // Chunk the write into pages
        let mut i = address;
        while i < (address + data.len()) {
            let page_offset = i % 32;
            self.write_raw_blocking(
                i,
                &data[(i - address)..(i - address + (32 - page_offset)).min(data.len())],
                delay,
            )?;
            i += 32 - page_offset;
        }
        Ok(())
    }

    /// Read an arbitrary number of bytes from the EEPROM, starting at `address`.
    /// This function will automatically paginate.
    pub fn read_blocking(&mut self, address: usize, data: &mut [u8]) -> Result<(), E> {
        // No need to do this per-page
        // self.read_raw(address, data)

        #[cfg(feature = "defmt")]
        defmt::info!(
            "[EEPROM M24C64 Unit {:b}] Reading from address {:#x} with {} bytes",
            self.e_addr,
            address,
            data.len()
        );

        // Chunk the read into pages
        let len = data.len();
        let mut i = address;
        while i < (address + data.len()) {
            let page_offset = i % 32;
            self.read_raw_blocking(
                i,
                &mut data[(i - address)..(i - address + (32 - page_offset)).min(len)],
            )?;
            i += 32 - page_offset;
        }
        Ok(())
    }
}
