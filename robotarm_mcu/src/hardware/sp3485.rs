use defmt::{debug, error, info};

use embassy_time::Timer;
use embedded_io_async::{Read, Write};

pub struct Sp3485 {
    buf: [u8; 512],
    // serial_rx: embassy_rp::uart::BufferedUartRx,
    serial_rx: embassy_rp::uart::UartRx<'static, embassy_rp::uart::Async>,
    serial_tx: embassy_rp::uart::UartTx<'static, embassy_rp::uart::Async>,
    rede_pin: embassy_rp::gpio::Output<'static>,
}

#[derive(defmt::Format)]
pub enum Sp3485Error {
    UartError,
    BufferError,
}

impl Sp3485 {
    pub fn new(
        // serial: embassy_rp::uart::BufferedUart,
        // serial_rx: embassy_rp::uart::BufferedUartRx,
        serial_rx: embassy_rp::uart::UartRx<'static, embassy_rp::uart::Async>,
        serial_tx: embassy_rp::uart::UartTx<'static, embassy_rp::uart::Async>,
        mut rede_pin: embassy_rp::gpio::Output<'static>,
    ) -> Self {
        rede_pin.set_low();
        Self {
            buf: [0; 512],
            // serial,
            serial_rx,
            serial_tx,
            rede_pin,
        }
    }

    pub async fn send(&mut self, data: &[u8]) -> Result<(), Sp3485Error> {
        // Set RE/DE high to enable transmission
        self.rede_pin.set_high();
        self.serial_tx
            .blocking_write(data)
            // .write_all(data)
            // .await
            .map_err(|_| Sp3485Error::UartError)?;
        // self.serial.flush().await.map_err(|_| Max485Error::Serial)?;
        self.serial_tx
            .blocking_flush()
            // .flush()
            // .await
            .map_err(|_| Sp3485Error::UartError)?;
        // embassy_time::Timer::after(embassy_time::Duration::from_micros(1000)).await;

        while self.serial_tx.busy() {
            // Timer::after_micros(1).await;
            // n += 1;
        }
        self.rede_pin.set_low();
        Ok(())
    }

    /// Read a single byte
    pub async fn receive<'a>(&mut self) -> Result<u8, Sp3485Error> {
        let mut byte = [0u8; 1];
        self.serial_rx
            .read(&mut byte)
            .await
            .map_err(|_| Sp3485Error::UartError)?;
        Ok(byte[0])
    }

    /// Check for COBS termination (0x00) in the received data
    #[cfg(feature = "nope")]
    pub async fn receive<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a [u8], Sp3485Error> {
        let mut n = 0;

        loop {
            if let Err(e) = self.serial_rx.read(&mut buf[n..n + 1]).await {
                error!("UART read error: {:?}", e);
                return Err(Sp3485Error::UartError);
            }

            if buf[n] == 0x00 {
                return Ok(&buf[..n + 1]);
            } else {
                n += 1;
            }
        }
    }
}

#[cfg(feature = "nope")]
impl Sp3485 {
    pub fn new(
        // serial: embassy_rp::uart::BufferedUart,
        serial_rx: embassy_rp::uart::BufferedUartRx,
        serial_tx: embassy_rp::uart::Uart<'static, embassy_rp::uart::Async>,
        mut rede_pin: embassy_rp::gpio::Output<'static>,
    ) -> Self {
        rede_pin.set_low();
        Self {
            buf: [0; 1024],
            // serial,
            serial_rx,
            serial_tx,
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
