use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use bytes::{Buf, BytesMut};
use postcard::accumulator::FeedResult;
use std::time::Duration;
use tokio::{
    io::AsyncReadExt,
    sync::mpsc::{UnboundedReceiver, UnboundedSender},
    time::Instant,
};
use tokio_serial::SerialPortBuilderExt;

use robotarm_protocol::{MotorId, SerialCommand, SerialLogMessage, SerialPacket};

enum Rs485State {
    Idle,
    WaitingForResponse(MotorId, Instant),
}

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

    state: Rs485State,
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

            state: Rs485State::Idle,
        }
    }

    pub async fn listen(&mut self) -> Result<()> {
        let mut port = tokio_serial::new(&self.port, self.baud_rate).open_native_async()?;
        port.set_exclusive(false)
            .expect("Unable to set serial port non-exclusive");

        debug!("Opened serial port: {}", &self.port);

        loop {
            match self.state {
                Rs485State::Idle => {
                    // debug!("Listening");
                    self._listen(&mut port).await?;
                }
                Rs485State::WaitingForResponse(id, t0) => {
                    if Instant::now() > t0 {
                        warn!("Response timeout for motor {:?}", id);
                        self.state = Rs485State::Idle;
                    } else {
                        // debug!("Waiting for response");
                        self.receive_message(&mut port).await?;
                    }
                }
            }
            // self._listen(&mut port).await?;
        }

        // Ok(())
    }

    async fn _listen(&mut self, port: &mut tokio_serial::SerialStream) -> Result<()> {
        tokio::select! {
            Some(cmd) = self.rx_command.recv() => {
                self.send_command(port, cmd).await?;
            }
            res = port.readable() => {
                self.receive_message(port).await?;
            }
        }

        Ok(())
    }

    async fn send_command(
        &mut self,
        port: &mut tokio_serial::SerialStream,
        cmd: SerialCommand,
    ) -> Result<()> {
        if let Some(packet) = robotarm_protocol::serialize_std(SerialPacket::Command(cmd)) {
            match port.try_write(&packet) {
                Ok(_) => {
                    if let Some((id, timeout)) = cmd.response_expected() {
                        self.state = Rs485State::WaitingForResponse(
                            id,
                            Instant::now() + Duration::from_micros(timeout),
                        );
                    }
                }
                Err(e) => {
                    debug!("Error writing to serial port: {:?}", e);
                }
            }
            Ok(())
        } else {
            bail!("Failed to serialize command");
        }
    }

    async fn receive_message(&mut self, port: &mut tokio_serial::SerialStream) -> Result<()> {
        match port.try_read(&mut self.raw_buf) {
            Ok(n) => {
                // debug!("Read {} bytes", n);
                debug!("Read {} bytes: {:?}", n, &self.raw_buf[..n]);
                self.bytes.extend_from_slice(&self.raw_buf[..n]);
                match self.run_accum(n) {
                    Ok(Some(msg)) => {
                        // if the message ID matches the expected response ID, update the state to Idle
                        match self.state {
                            Rs485State::WaitingForResponse(id, _) => {
                                if msg.id() == id {
                                    self.state = Rs485State::Idle;
                                }
                            }
                            Rs485State::Idle => {
                                unimplemented!()
                            }
                        }

                        self.tx_msg.send(msg)?;

                        Ok(())
                    }
                    Ok(None) => Ok(()),
                    Err(e) => {
                        // bail!("Error processing serial data: {e}");
                        warn!("Error processing serial data: {:?}", e);
                        Ok(())
                    }
                }
            }
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    bail!("Serial read timed out");
                } else if e.kind() == std::io::ErrorKind::WouldBlock {
                    // debug!("Serial read would block");
                    Ok(())
                } else {
                    bail!("Error reading from serial port: {:?}", e);
                }
            }
        }
    }

    fn run_accum(&mut self, n: usize) -> Result<Option<SerialLogMessage>> {
        debug!("Running accumulator with {} bytes", n);
        if n == 0 {
            return Ok(None);
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
                    let crc0 = robotarm_protocol::CRCType::from_le_bytes(
                        data[data.len() - robotarm_protocol::CRC_SIZE..]
                            .try_into()
                            .context("Failed to parse CRC")?,
                    );

                    let crc1 = robotarm_protocol::CRC
                        .checksum(&data[..data.len() - robotarm_protocol::CRC_SIZE]);

                    if crc0 != crc1 {
                        error!("CRC mismatch: expected {:08x}, got {:08x}", crc0, crc1);
                        // return Err(anyhow!("CRC mismatch"));
                    }

                    let msg: SerialPacket =
                        postcard::from_bytes(&data[..data.len() - robotarm_protocol::CRC_SIZE])
                            .context("Failed to deserialize message")?;

                    match msg {
                        SerialPacket::Command(_) => {
                            error!("server received command packet, this should never happen");
                            return Ok(None);
                        }
                        SerialPacket::LogMessage(msg) => {
                            debug!("Deserialized message: {:?}", msg);
                            return Ok(Some(msg));
                        }
                    }

                    // self.tx_msg.send(data)?;
                    // return Ok(Some(msg));
                }
                FeedResult::Consumed => {
                    self.bytes.clear();
                    return Ok(None);
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
