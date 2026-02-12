pub mod codec;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use futures::{SinkExt as _, StreamExt};
use tokio_serial::SerialStream;
use tokio_util::codec::Decoder as _;

use crate::serial::codec::SerialCodec;
use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct SerialHandler {
    port: Option<SerialStream>,

    serial_log_tx: tokio::sync::mpsc::Sender<SerialLogMessage>,
    serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
}

impl SerialHandler {
    pub fn new(
        // port: SerialCodec,
        port: SerialStream,
        serial_log_tx: tokio::sync::mpsc::Sender<SerialLogMessage>,
        serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
    ) -> Self {
        Self {
            port: Some(port),
            serial_log_tx,
            serial_cmd_rx,
        }
    }

    pub async fn run(&mut self) -> Result<()> {
        let port = self.port.take().context("Serial port not initialized")?;
        let mut framed = crate::serial::codec::SerialCodec::default().framed(port);

        let (mut writer, mut reader) = framed.split();

        debug!("Waiting for serial connection...");
        loop {
            if let Ok(_) = writer.send(SerialCommand::RequestSettings { id: 0 }).await {
                break;
            }
        }
        debug!("Serial connection established");

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
