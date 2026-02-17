use cortex_m::prelude::_embedded_hal_serial_Write as _;
use defmt::{debug, error, info};
use embedded_io_async::{Read, Write};

pub struct Max485 {
    // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Async>,
    // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Blocking>,
    serial: embassy_rp::uart::BufferedUart,
    rede_pin: embassy_rp::gpio::Output<'static>,
    // static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
}

#[derive(Debug)]
pub enum Max485Error {
    Serial,
    Pin,
}

impl Max485 {
    pub fn new(
        // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Async>,
        // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Blocking>,
        serial: embassy_rp::uart::BufferedUart,
        rede_pin: embassy_rp::gpio::Output<'static>,
    ) -> Self {
        Self { serial, rede_pin }
    }

    async fn _send(&mut self, data: &[u8]) -> Result<(), Max485Error> {
        // Set RE/DE high to enable transmission
        self.rede_pin.set_high();
        self.serial
            .write_all(data)
            .await
            .map_err(|_| Max485Error::Serial)?;
        self.serial.flush().await.map_err(|_| Max485Error::Serial)?;
        embassy_time::Timer::after(embassy_time::Duration::from_micros(100)).await;
        self.rede_pin.set_low();
        Ok(())
    }

    pub async fn receive(&mut self, buffer: &mut [u8]) -> usize {
        // Set RE/DE low to enable reception
        self.rede_pin.set_low();
        let bytes_read = self.serial.read(buffer).await.unwrap();
        // let bytes_read = self.serial.blocking_read(buffer).unwrap();
        bytes_read
    }
}
