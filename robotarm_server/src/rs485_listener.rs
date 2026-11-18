use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use bytes::{Buf, BytesMut};
use postcard::accumulator::FeedResult;
use tokio::{
    io::AsyncReadExt,
    sync::mpsc::{UnboundedReceiver, UnboundedSender},
};
use tokio_serial::SerialPortBuilderExt;

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct Rs485Listener {
    /// get commands from websocket
    pub rx_command: UnboundedReceiver<SerialCommand>,
    /// send messages to websocket
    // pub tx_msg: UnboundedSender<SerialLogMessage>,
    pub tx_msg: tokio::sync::broadcast::Sender<SerialLogMessage>,

    port: String,
    baud_rate: u32,

    bytes: BytesMut,
    raw_buf: [u8; 1024],
    accum: postcard::accumulator::CobsAccumulator<1024>,
}

impl Rs485Listener {
    pub fn new(
        rx_command: UnboundedReceiver<SerialCommand>,
        tx_msg: tokio::sync::broadcast::Sender<SerialLogMessage>,
        port: String,
        baud_rate: u32,
    ) -> Self {
        Self {
            rx_command,
            tx_msg,
            port,
            baud_rate,
            bytes: BytesMut::new(),
            raw_buf: [0; 1024],
            accum: postcard::accumulator::CobsAccumulator::new(),
        }
    }

    pub async fn listen(&mut self) -> Result<()> {
        let mut port = tokio_serial::new(&self.port, self.baud_rate).open_native_async()?;
        port.set_exclusive(false)
            .expect("Unable to set serial port non-exclusive");

        debug!("Opened serial port: {}", &self.port);

        loop {
            self._listen(&mut port).await?;
        }

        // Ok(())
    }

    async fn _listen(&mut self, port: &mut tokio_serial::SerialStream) -> Result<()> {
        tokio::select! {
            Some(cmd) = self.rx_command.recv() => {
                let cmd = self.rx_command.recv().await.unwrap();

                if let Some(packet) = robotarm_protocol::serialize_std(cmd) {
                    match port.try_write(&packet) {
                        Ok(_) => {}
                        Err(e) => {
                            debug!("Error writing to serial port: {:?}", e);
                        }
                    }
                }
            }
            res = port.readable() => {
                match port.try_read(&mut self.raw_buf) {
                    Ok(n) => {
                        // debug!("Read {} bytes", n);
                        debug!("Read {} bytes: {:?}", n, &self.raw_buf[..n]);
                        self.bytes.extend_from_slice(&self.raw_buf[..n]);
                        match self.run_accum(n) {
                            Ok(_) => {}
                            Err(e) => {
                                // debug!("Error processing serial data: {e}");
                            }
                        }
                    }
                    Err(e) => {
                        // debug!("Error reading from serial port: {:?}", e);
                        // return Err(anyhow!("Error reading from serial port: {}", e));
                        if e.kind() == std::io::ErrorKind::TimedOut {
                            debug!("Serial read timed out");
                        } else if e.kind() == std::io::ErrorKind::WouldBlock {
                            // debug!("Serial read would block");
                        } else {
                            debug!("Error reading from serial port: {:?}", e);
                            // return Err(anyhow!("Error reading from serial port: {}", e));
                        }
                    }
                }
            }
        }

        #[cfg(feature = "nope")]
        match port.try_read(&mut self.raw_buf) {
            Ok(n) => {
                self.bytes.extend_from_slice(&self.raw_buf[..n]);
                match self.run_accum(n) {
                    Ok(_) => {}
                    Err(e) => {
                        // debug!("Error processing serial data: {e}");
                    }
                }
            }
            Err(e) => {
                // debug!("Error reading from serial port: {:?}", e);
                // return Err(anyhow!("Error reading from serial port: {}", e));
                if e.kind() == std::io::ErrorKind::TimedOut {
                    // debug!("Serial read timed out");
                } else {
                    debug!("Error reading from serial port: {:?}", e);
                    return Err(anyhow!("Error reading from serial port: {}", e));
                }
            }
        }

        Ok(())
    }

    // pub fn debug_accum(&mut self, buf: &[u8]) {
    //     self.bytes.extend_from_slice(buf);
    //     self.run_accum(buf.len()).unwrap();
    // }

    // pub fn debug_send(&mut self, buf: &[u8]) -> Result<()> {
    //     // self.port
    // }

    fn run_accum(&mut self, n: usize) -> Result<()> {
        debug!("Running accumulator with {} bytes", n);
        if n == 0 {
            return Ok(());
        }

        loop {
            let len = self.bytes.len();

            match self
                .accum
                .feed::<heapless::Vec<u8, 64>>(&mut self.bytes[..])
            {
                FeedResult::Success { data, remaining } => {
                    // advance src by the number of bytes consumed and return the deserialized message
                    let consumed = len - remaining.len();
                    self.bytes.advance(consumed);
                    // debug!("Received message: {:?}", data);

                    let data: heapless::Vec<u8, 64> = data;
                    let crc0 = u32::from_le_bytes(
                        data[data.len() - 4..]
                            .try_into()
                            .context("Failed to parse CRC")?,
                    );

                    let crc1 = crc32fast::hash(&data[..data.len() - 4]);

                    if crc0 != crc1 {
                        error!("CRC mismatch: expected {:08x}, got {:08x}", crc0, crc1);
                        // return Err(anyhow!("CRC mismatch"));
                    }

                    let msg: SerialLogMessage = postcard::from_bytes(&data[..data.len() - 4])
                        .context("Failed to deserialize message")?;

                    debug!("Deserialized message: {:?}", msg);

                    // self.tx_msg.send(data)?;
                }
                FeedResult::Consumed => {
                    self.bytes.clear();
                    return Ok(());
                }
                FeedResult::OverFull(w) => {
                    error!("Accumulator overflow");
                    panic!("Accumulator overflow");
                }
                FeedResult::DeserError(w) => {
                    error!("Deserialization error: {len}, {}", w.len());
                    let new_len = w.len();
                    self.bytes.advance(len - new_len);
                    // skip the current message by advancing src until the next 0x00 byte
                    // break;
                }
            }
        }
    }
}
