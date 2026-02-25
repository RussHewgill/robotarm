use cortex_m::prelude::_embedded_hal_serial_Write as _;
use defmt::{debug, error, info};
use embedded_io_async::{Read, Write};
use postcard::accumulator::FeedResult;
use robotarm_protocol::SerialLogMessage;

pub struct Max485 {
    buf: [u8; 1024],
    // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Async>,
    // serial: embassy_rp::uart::Uart<'static, embassy_rp::uart::Blocking>,
    serial: embassy_rp::uart::BufferedUart,
    rede_pin: embassy_rp::gpio::Output<'static>,
    // static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
}

#[derive(Debug, defmt::Format)]
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
        Self {
            buf: [0; 1024],
            serial,
            rede_pin,
        }
    }

    pub async fn _send(&mut self, data: &[u8]) -> Result<(), Max485Error> {
        // Set RE/DE high to enable transmission
        self.rede_pin.set_high();
        self.serial
            .write_all(data)
            .await
            .map_err(|_| Max485Error::Serial)?;
        self.serial.flush().await.map_err(|_| Max485Error::Serial)?;
        embassy_time::Timer::after(embassy_time::Duration::from_micros(1000)).await;
        self.rede_pin.set_low();
        Ok(())
    }

    pub async fn send(&mut self, msg: SerialLogMessage) -> Result<(), Max485Error> {
        if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut self.buf) {
            // if encoded.len() <= 64 {
            //     let _ = self.tx.write_packet(encoded).await;
            // } else {
            //     error!("Encoded message too long for USB packet");
            // }
            // let _ = self.class.write_packet(encoded).await;

            // Set RE/DE high to enable transmission
            self.rede_pin.set_high();
            self.serial
                .write_all(encoded)
                .await
                .map_err(|_| Max485Error::Serial)?;
            self.serial.flush().await.map_err(|_| Max485Error::Serial)?;
            embassy_time::Timer::after(embassy_time::Duration::from_micros(100)).await;
            self.rede_pin.set_low();
        } else {
            error!("Failed to encode message");
        }
        Ok(())
    }

    pub async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, Max485Error> {
        // Set RE/DE low to enable reception
        self.rede_pin.set_low();
        let bytes_read = self
            .serial
            .read(buffer)
            .await
            .map_err(|_| Max485Error::Serial)?;
        // let bytes_read = self.serial.blocking_read(buffer).unwrap();
        Ok(bytes_read)
    }
}

pub fn init_rs485_logger(spawner: &embassy_executor::Spawner, max485: Max485) {
    let log_rx = super::usb::LOG_CHAN.receiver();
    let cmd_tx = super::usb::CMD_CHAN.sender();

    spawner
        .spawn(rs485_logger_task(max485, cmd_tx, log_rx))
        .unwrap();
}

#[embassy_executor::task]
async fn rs485_logger_task(
    mut max485: Max485,
    cmd_tx: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        robotarm_protocol::SerialCommand,
        1,
    >,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        robotarm_protocol::SerialLogMessage,
        1,
    >,
) {
    let mut buf: [u8; 4096];
    let mut accum = postcard::accumulator::CobsAccumulator::<4096>::new();

    loop {
        let msg = log_rx.receive().await;

        if let Err(e) = max485.send(msg).await {
            error!("Failed to send log message over RS485: {:?}", e);
        }
    }

    #[cfg(feature = "nope")]
    loop {
        buf = [0; 4096];
        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            // usb_monitor.rx.read_packet(&mut buf),
            max485.receive(&mut buf),
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                // if prev_msg == Some(msg) {
                //     // skip sending duplicate message
                //     debug!("Skipping duplicate log message");
                //     continue;
                // } else {
                //     debug!("Sending log message: {:?}", msg);
                //     prev_msg = Some(msg);
                //     usb_monitor.send(msg).await;
                // }
                // debug!("Sending log message: {:?}", msg);
                // usb_monitor.send(msg).await;
                if let Err(e) = max485.send(msg).await {
                    error!("Failed to send log message over RS485: {:?}", e);
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                error!("USB read error");
            }
            embassy_futures::select::Either::Second(Ok(n)) => {
                // debug!("Received {} bytes from USB", n);
                let mut window = &buf[..n];
                'cobs: while !window.is_empty() {
                    window = match accum.feed(&buf[..n]) {
                        FeedResult::Success { data, remaining } => {
                            // debug!("Received complete message from USB: {:?}", data);
                            loop {
                                match cmd_tx.try_send(data) {
                                    Ok(()) => break,
                                    Err(e) => {
                                        // error!("Failed to send command to main task, retrying...");
                                        embassy_futures::yield_now().await;
                                    }
                                }
                            }

                            remaining
                        }
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(w) => {
                            unimplemented!()
                        }
                        FeedResult::DeserError(w) => {
                            error!("Failed to decode message");
                            w
                        }
                    }
                }
            }
        }
        // let msg = log_rx.receive().await;
        // usb_monitor.send(msg).await;
    }
}
