// pub mod codec;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use bytes::{Buf as _, BufMut, BytesMut};
use postcard::accumulator::FeedResult;
use std::time::Duration;

// use futures::{SinkExt as _, StreamExt};
// use tokio_serial::{SerialPort as _, SerialStream};
// use tokio_util::codec::Decoder as _;

// use crate::serial::codec::SerialCodec;
use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct SerialHandler {
    // port: Option<SerialStream>,
    port: Option<Box<dyn serialport::SerialPort>>,
    address: String,
    rate: u32,

    // serial_log_tx: tokio::sync::mpsc::Sender<SerialLogMessage>,
    // serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
    serial_log_tx: crossbeam_channel::Sender<SerialLogMessage>,
    serial_cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
    ui_cmd_tx: crossbeam_channel::Sender<crate::ui::UiCommand>,

    cobs_buf: postcard::accumulator::CobsAccumulator<4096>,
    raw_buf: [u8; 1024],
    bytes: BytesMut,
}

impl SerialHandler {
    pub fn new(
        // port: SerialCodec,
        // port: SerialStream,
        // port: Box<dyn serialport::SerialPort>,
        address: &str,
        serial_log_tx: crossbeam_channel::Sender<SerialLogMessage>,
        serial_cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
        ui_cmd_tx: crossbeam_channel::Sender<crate::ui::UiCommand>,
        rate: u32,
    ) -> Self {
        let port = match serialport::new(address, rate).open() {
            Ok(port) => Some(port),
            Err(e) => {
                debug!("Failed to open serial port: {e}");
                None
            }
        };

        Self {
            port,
            address: address.to_string(),
            rate,
            serial_log_tx,
            serial_cmd_rx,
            ui_cmd_tx,

            cobs_buf: postcard::accumulator::CobsAccumulator::new(),
            raw_buf: [0; 1024],
            bytes: BytesMut::with_capacity(1024),
        }
    }

    pub fn reconnect(&mut self) -> Result<()> {
        self.cobs_buf = postcard::accumulator::CobsAccumulator::new();
        self.raw_buf = [0; 1024];
        self.bytes.clear();

        self.ui_cmd_tx.send(crate::ui::UiCommand::ClearPlot)?;

        loop {
            match serialport::new(&self.address, self.rate).open()
            // match tokio_serial::SerialPortBuilderExt::open_native_async(tokio_serial::new(
            //     "COM8", self.rate,
            // )) 
            {
                Ok(port) => {
                    self.port = Some(port);
                    break;
                }
                Err(e) => {
                    debug!("Failed to connect to serial port: {e}");
                    // tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    // return Err(anyhow!("Failed to connect to serial port: {e}"));
                    // panic!()
                }
            }
        }
        Ok(())
    }

    #[cfg(feature = "nope")]
    fn run_accum(&mut self, n: usize) -> Result<Option<SerialLogMessage>> {
        // Finished reading input
        if n == 0 {
            return Ok(None);
        }

        let len = self.bytes.len();

        match self.cobs_buf.feed::<SerialLogMessage>(&self.bytes[..n]) {
            FeedResult::Consumed => {
                return Ok(None);
            }
            FeedResult::OverFull(w) => {
                // self.raw_buf.advance(n - new_wind.len());
                // return Ok(None);
                error!("Accumulator overflow");
                panic!("Accumulator overflow");
            }
            FeedResult::DeserError(w) => {
                error!("Deserialization error: {len}, {}", w.len());
                let new_len = w.len();
                // self.raw_buf.advance(len - new_len);
                self.bytes.advance(len - new_len);
                // skip the current message by advancing src until the next 0x00 byte
                return Ok(None);
            }
            FeedResult::Success { data, remaining } => {
                // advance src by the number of bytes consumed and return the deserialized message
                let consumed = len - remaining.len();
                self.bytes.advance(consumed);
                return Ok(Some(data));
            }
        }

        #[cfg(feature = "nope")]
        'cobs: while !window.is_empty() {
            window = match self.cobs_buf.feed::<SerialLogMessage>(&window) {
                FeedResult::Consumed => break 'cobs,
                FeedResult::OverFull(new_wind) => new_wind,
                FeedResult::DeserError(new_wind) => new_wind,
                FeedResult::Success { data, remaining } => {
                    debug!("Received message: {:?}", data);

                    let new_len = w.len();
                    src.advance(len - new_len);

                    remaining
                }
            };
        }
    }

    #[cfg(feature = "nope")]
    fn run_accum(&mut self, n: usize) -> Result<Option<SerialLogMessage>> {
        // Finished reading input
        if n == 0 {
            self.bytes.clear();
            return Ok(None);
        }

        let len = self.bytes.len();

        loop {
            match self.cobs_buf.feed::<SerialLogMessage>(&mut self.bytes[..]) {
                FeedResult::Success { data, remaining } => {
                    // advance src by the number of bytes consumed and return the deserialized message
                    let consumed = len - remaining.len();
                    self.bytes.advance(consumed);
                    return Ok(Some(data));
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
                    // error!("Deserialization error: {len}, {}", w.len());
                    let new_len = w.len();
                    self.bytes.advance(len - new_len);
                    // skip the current message by advancing src until the next 0x00 byte
                    return Ok(None);
                }
            }
        }
    }

    fn run_accum(&mut self, n: usize) -> Result<()> {
        if n == 0 {
            return Ok(());
        }

        loop {
            let len = self.bytes.len();

            match self.cobs_buf.feed::<SerialLogMessage>(&mut self.bytes[..]) {
                FeedResult::Success { data, remaining } => {
                    // advance src by the number of bytes consumed and return the deserialized message
                    let consumed = len - remaining.len();
                    self.bytes.advance(consumed);
                    // debug!("Received message: {:?}", data);
                    self.serial_log_tx.send(data)?;
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

    pub fn run(&mut self) -> Result<()> {
        self.cobs_buf = postcard::accumulator::CobsAccumulator::new();
        self.raw_buf = [0; 1024];
        self.bytes.clear();

        let mut port = match self.port.take() {
            Some(port) => port,
            None => {
                debug!("Serial port not initialized, attempting to reconnect...");
                self.reconnect()?;
                self.port
                    .take()
                    .context("Serial port not initialized after reconnecting")?
            }
        };

        debug!("Waiting for serial connection...");
        loop {
            if let Ok(_) = port.set_baud_rate(self.rate) {
                break;
            }
        }
        debug!("Serial connection established");

        port.write_data_terminal_ready(true)?;

        port.write(&postcard::to_stdvec_cobs(
            &SerialCommand::RequestSettings { id: 0 },
        )?)?;

        // let mut codec = SerialCodec::default();

        // let mut cobs_buf = postcard::accumulator::CobsAccumulator::<4096>::new();
        // let mut raw_buf: [u8; 1024] = [0; 1024];

        loop {
            match port.read(&mut self.raw_buf) {
                Ok(n) => {
                    // self.bytes.put(&self.raw_buf[..n]);
                    self.bytes.extend_from_slice(&self.raw_buf[..n]);
                    // self.run_accum(n)?;
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

            match self.serial_cmd_rx.try_recv() {
                Ok(cmd) => {
                    let buf = postcard::to_stdvec_cobs(&cmd)?;
                    port.write(&buf).context("Failed to send command")?;
                }
                Err(crossbeam_channel::TryRecvError::Disconnected) => {
                    // debug!("Command channel disconnected");
                    // return Err(anyhow!("Command channel disconnected"));
                }
                Err(crossbeam_channel::TryRecvError::Empty) => {}
            }

            //
        }
    }

    #[cfg(feature = "nope")]
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

        // let mut framed = crate::serial::codec::SerialCodec::default().framed(port);
        // let (mut writer, mut reader) = framed.split();

        // let _ = writer.send(SerialCommand::RequestSettings { id: 0 }).await;

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
