pub mod codec;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use futures::{SinkExt as _, StreamExt};
use tokio_serial::{SerialPort as _, SerialStream};
use tokio_util::codec::Decoder as _;

use crate::serial::codec::SerialCodec;
use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct SerialHandler {
    port: Option<SerialStream>,
    rate: u32,

    serial_log_tx: tokio::sync::mpsc::Sender<SerialLogMessage>,
    serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
}

impl SerialHandler {
    pub fn new(
        // port: SerialCodec,
        port: SerialStream,
        serial_log_tx: tokio::sync::mpsc::Sender<SerialLogMessage>,
        serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
        rate: u32,
    ) -> Self {
        Self {
            port: Some(port),
            rate,
            serial_log_tx,
            serial_cmd_rx,
        }
    }

    pub async fn reconnect(&mut self) -> Result<()> {
        loop {
            match tokio_serial::SerialPortBuilderExt::open_native_async(tokio_serial::new(
                "COM8", self.rate,
            )) {
                Ok(port) => {
                    self.port = Some(port);
                    break;
                }
                Err(e) => {
                    debug!("Failed to connect to serial port: {e}");
                    tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                }
            }
        }
        Ok(())
    }

    pub async fn run(&mut self) -> Result<()> {
        let mut port = self.port.take().context("Serial port not initialized")?;

        debug!("Waiting for serial connection...");
        loop {
            if let Ok(_) = port.set_baud_rate(self.rate) {
                break;
            }
        }
        debug!("Serial connection established");

        let rate = port.baud_rate().context("Failed to get baud rate")?;
        debug!("Serial port baud rate: {}", rate);

        port.write_data_terminal_ready(true)?;

        let mut framed = crate::serial::codec::SerialCodec::default().framed(port);
        let (mut writer, mut reader) = framed.split();

        let _ = writer.send(SerialCommand::RequestSettings { id: 0 }).await;

        loop {
            // debug!("looping");
            tokio::select! {
                n = reader.next() => {
                    match n {
                        Some(Ok(msg)) => {
                            // debug!("Received message: {:?}", msg);
                            // if let Some(tx) = &self.serial_log_tx {
                            //     tx.send(msg).await.context("Failed to send log message")?;
                            // }
                            self.serial_log_tx.send(msg).await.context("Failed to send log message")?;
                        }
                        Some(Err(e)) => {
                            // debug!("Error reading from serial port: {}", e);
                        }
                        None => {
                            // debug!("Serial port closed");
                        }
                    }
                }
                cmd = self.serial_cmd_rx.recv() => {
                    if let Some(cmd) = cmd {
                        writer.send(cmd).await.context("Failed to send command")?;
                    } else {
                        // debug!("Command channel closed");
                    }
                }
            }
        }

        // Ok(())
    }
}
