use defmt::{debug, error, info};

use embedded_io_async::{Read, Write};

pub struct Sp3485 {
    buf: [u8; 1024],
    serial: embassy_rp::uart::BufferedUart,
    rede_pin: embassy_rp::gpio::Output<'static>,
}

#[derive(defmt::Format)]
pub enum Sp3485Error {
    UartError,
}

impl Sp3485 {
    pub fn new(
        serial: embassy_rp::uart::BufferedUart,
        mut rede_pin: embassy_rp::gpio::Output<'static>,
    ) -> Self {
        rede_pin.set_low();
        Self {
            buf: [0; 1024],
            serial,
            rede_pin,
        }
    }

    pub async fn send(&mut self, data: &[u8]) -> Result<(), Sp3485Error> {
        // Set RE/DE high to enable transmission
        self.rede_pin.set_high();
        self.serial
            // .blocking_write(data)
            .write_all(data)
            .await
            .map_err(|_| Sp3485Error::UartError)?;
        // self.serial.flush().await.map_err(|_| Max485Error::Serial)?;
        self.serial
            .blocking_flush()
            // .flush()
            // .await
            .map_err(|_| Sp3485Error::UartError)?;
        // embassy_time::Timer::after(embassy_time::Duration::from_micros(1000)).await;
        self.rede_pin.set_low();
        Ok(())
    }

    pub async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, Sp3485Error> {
        // // Set RE/DE low to enable reception
        // self.rede_pin.set_low();
        let bytes_read = self
            .serial
            // .blocking_read(buffer)
            .read(buffer)
            .await
            .map_err(|_| Sp3485Error::UartError)?;
        // let bytes_read = self.serial.blocking_read(buffer).unwrap();
        Ok(bytes_read)
        // Ok(())
    }
}
