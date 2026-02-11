pub mod codec;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use futures::StreamExt;
use tokio_serial::SerialStream;
use tokio_util::codec::Decoder as _;

use crate::serial::codec::SerialCodec;
use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct SerialHandler {
    port: Option<SerialStream>,

    serial_log_tx: Option<tokio::sync::mpsc::Sender<SerialLogMessage>>,
    serial_cmd_rx: Option<tokio::sync::mpsc::Receiver<SerialCommand>>,
}

impl SerialHandler {
    pub fn new(
        // port: SerialCodec,
        port: SerialStream,
        serial_log_tx: Option<tokio::sync::mpsc::Sender<SerialLogMessage>>,
        serial_cmd_rx: Option<tokio::sync::mpsc::Receiver<SerialCommand>>,
    ) -> Self {
        Self {
            port: Some(port),
            serial_log_tx,
            serial_cmd_rx,
        }
    }

    pub async fn run(&mut self) -> Result<()> {
        let port = self.port.take().context("Serial port not initialized")?;
        let mut reader = crate::serial::codec::SerialCodec.framed(port);

        loop {
            tokio::select! {
                n = reader.next() => {
                    match n {
                        Some(Ok(msg)) => {
                            // debug!("Received message: {:?}", msg);
                            if let Some(tx) = &self.serial_log_tx {
                                tx.send(msg).await.context("Failed to send log message")?;
                            }
                        }
                        Some(Err(e)) => {
                            trace!("Error reading from serial port: {}", e);
                        }
                        None => {
                            trace!("Serial port closed");
                        }
                    }
                }
                cmd = async {
                    if let Some(rx) = &mut self.serial_cmd_rx {
                        rx.recv().await
                    } else {
                        None
                    }
                } => {
                    unimplemented!()
                }
            }
        }

        // Ok(())
    }
}
