use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};
use tokio_serial::SerialPortBuilderExt;

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct Rs485Listener {
    pub rx_command: UnboundedReceiver<SerialCommand>,
    pub tx_msg: UnboundedSender<SerialLogMessage>,

    port: String,
    baud_rate: u32,
}

impl Rs485Listener {
    pub fn new(
        rx_command: UnboundedReceiver<SerialCommand>,
        tx_msg: UnboundedSender<SerialLogMessage>,
        port: String,
        baud_rate: u32,
    ) -> Self {
        Self {
            rx_command,
            tx_msg,
            port,
            baud_rate,
        }
    }

    pub fn listen(&mut self) -> Result<()> {
        let mut port = tokio_serial::new(&self.port, self.baud_rate).open_native_async()?;
        port.set_exclusive(false)
            .expect("Unable to set serial port non-exclusive");

        debug!("Opened serial port: {}", &self.port);

        Ok(())
    }
}
