use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use bytes::{Buf, BytesMut};
use futures::FutureExt;
use nusb::{
    MaybeFuture,
    io::{EndpointRead, EndpointWrite},
    transfer::{Bulk, In, Out},
};
use postcard::accumulator::FeedResult;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct UsbRawHandler {
    writer: EndpointWrite<Bulk>,
    reader: EndpointRead<Bulk>,

    // serial_log_tx: crossbeam_channel::Sender<SerialLogMessage>,
    // // serial_cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
    // serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
    // ui_cmd_tx: crossbeam_channel::Sender<crate::ui::UiCommand>,
    cobs_buf: postcard::accumulator::CobsAccumulator<4096>,
    raw_buf: [u8; 1024],
    bytes: BytesMut,
}

impl UsbRawHandler {
    pub async fn init(// serial_log_tx: crossbeam_channel::Sender<SerialLogMessage>,
        // // serial_cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
        // serial_cmd_rx: tokio::sync::mpsc::Receiver<SerialCommand>,
        // ui_cmd_tx: crossbeam_channel::Sender<crate::ui::UiCommand>,
    ) -> Result<Self> {
        let di = nusb::list_devices()
            .await?
            .find(|d| d.vendor_id() == 0xc0d0 && d.product_id() == 0xcaf0)
            .context("no device found")?;
        let device = di.open().await.context("error opening device")?;
        let interface = device.claim_interface(0).await?;
        // let di = nusb::list_devices()
        //     .wait()
        //     .unwrap()
        //     .find(|d| d.vendor_id() == 0xc0d0 && d.product_id() == 0xcaf0)
        //     .expect("no device found");
        // let device = di.open().wait().expect("error opening device");
        // let interface = device
        //     .claim_interface(0)
        //     .wait()
        //     .expect("error claiming interface");

        debug!("Interface claimed");

        const BULK_OUT_EP: u8 = 0x01;
        const BULK_IN_EP: u8 = 0x81;

        let mut writer = interface
            .endpoint::<Bulk, Out>(0x01)
            .context("error opening bulk out endpoint")?
            .writer(128)
            .with_num_transfers(8);

        let mut reader = interface
            .endpoint::<Bulk, In>(0x81)
            .context("error opening bulk in endpoint")?;

        // debug!("max packet size = {}", reader.max_packet_size());

        let reader = reader.reader(128).with_num_transfers(8);

        Ok(Self {
            writer,
            reader,

            // serial_log_tx,
            // serial_cmd_rx,
            // ui_cmd_tx,
            cobs_buf: postcard::accumulator::CobsAccumulator::new(),
            raw_buf: [0; 1024],
            bytes: BytesMut::with_capacity(1024),
        })
    }

    // #[cfg(feature = "nope")]
    pub async fn reconnect(&mut self) -> Result<()> {
        let di = nusb::list_devices()
            .await?
            .find(|d| d.vendor_id() == 0xc0d0 && d.product_id() == 0xcaf0)
            .context("no device found")?;
        let device = di.open().await.context("error opening device")?;
        let interface = device.claim_interface(0).await?;

        debug!("Interface claimed");

        const BULK_OUT_EP: u8 = 0x01;
        const BULK_IN_EP: u8 = 0x81;

        self.writer = interface
            .endpoint::<Bulk, Out>(0x01)
            .context("error opening bulk out endpoint")?
            .writer(128)
            .with_num_transfers(8);

        self.reader = interface
            .endpoint::<Bulk, In>(0x81)
            .context("error opening bulk in endpoint")?
            .reader(128)
            .with_num_transfers(8);

        Ok(())
    }

    #[cfg(feature = "nope")]
    fn run_accum(
        &mut self,
        n: usize,
        serial_log_tx: &mut crossbeam_channel::Sender<SerialLogMessage>,
    ) -> Result<()> {
        if n == 0 {
            return Ok(());
        }

        // debug!("Received {} bytes: {:?}", n, &self.bytes[..n]);

        match postcard::from_bytes(&self.bytes[..n]) {
            Ok(msg) => {
                serial_log_tx.send(msg)?;
                self.bytes.advance(n);
                Ok(())
            }
            Err(e) => {
                error!("Error deserializing message: {:?}", e);
                self.bytes.advance(n);
                Err(anyhow!("Error deserializing message: {:?}", e))
            }
        }
    }

    // #[cfg(feature = "nope")]
    fn run_accum(
        &mut self,
        n: usize,
        serial_log_tx: &mut crossbeam_channel::Sender<SerialLogMessage>,
    ) -> Result<()> {
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
                    serial_log_tx.send(data)?;
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

    pub async fn run(
        &mut self,
        serial_log_tx: &mut crossbeam_channel::Sender<SerialLogMessage>,
        // serial_cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
        serial_cmd_rx: &mut tokio::sync::mpsc::Receiver<SerialCommand>,
        ui_cmd_tx: &mut crossbeam_channel::Sender<crate::ui::UiCommand>,
    ) -> Result<()> {
        self.cobs_buf = postcard::accumulator::CobsAccumulator::new();
        self.raw_buf = [0; 1024];
        self.bytes.clear();

        // debug!("Waiting for serial connection...");
        // loop {
        //     if let Ok(_) = port.set_baud_rate(self.rate) {
        //         break;
        //     }
        // }
        // debug!("Serial connection established");

        // port.write_data_terminal_ready(true)?;

        // port.write(&postcard::to_stdvec_cobs(
        //     &SerialCommand::RequestSettings { id: 0 },
        // )?)?;

        // port.write(&postcard::to_stdvec_cobs(
        //     &SerialCommand::RequestSettings { id: 1 },
        // )?)?;

        // self.writer.write_all(b"test").await.unwrap();
        // self.writer.flush().await.unwrap();

        // let n = self.reader.read(&mut self.raw_buf).await.unwrap();

        // debug!("Read {} bytes: {:?}", n, &self.raw_buf[..n]);
        // debug!(
        //     "Read {} bytes: {:?}",
        //     n,
        //     std::str::from_utf8(&self.raw_buf[..n]).unwrap()
        // );

        self.reader
            .set_read_timeout(std::time::Duration::from_micros(100));

        self.writer
            .write(&postcard::to_stdvec_cobs(
                &SerialCommand::RequestSettings { id: 0 },
            )?)
            .await?;

        self.writer
            .write(&postcard::to_stdvec_cobs(
                &SerialCommand::RequestSettings { id: 1 },
            )?)
            .await?;
        self.writer.flush().await?;

        debug!("Looping");
        loop {
            // #[cfg(feature = "nope")]
            futures::select! {
                n = self.reader.read(&mut self.raw_buf).fuse() => {
                    match n {
                        Ok(n) => {
                            self.bytes.extend_from_slice(&self.raw_buf[..n]);
                            match self.run_accum(n, serial_log_tx) {
                                Ok(_) => {}
                                Err(e) => {
                                    // debug!("Error processing serial data: {e}");
                                }
                            }
                        }
                        Err(e) => {
                            debug!("Error reading from usb port: {:?}", e);
                            bail!("Error reading from usb port: {:?}", e);
                        }
                    }
                }
                cmd = serial_cmd_rx.recv().fuse() => {
                    match cmd {
                        Some(cmd) => {
                            debug!("Sending command: {:?}", cmd);
                            let buf = postcard::to_stdvec_cobs(&cmd)?;
                            self.writer.write_all(&buf).await.context("Failed to send command")?;
                            self.writer.flush().await.context("Failed to flush command")?;
                        }
                        None => {
                            debug!("Command channel disconnected");
                        }
                    }
                }
            }

            // debug!("Reading");
            #[cfg(feature = "nope")]
            match self.reader.read(&mut self.raw_buf).await {
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
                    debug!("Error reading from usb port: {:?}", e);
                }
            }

            // debug!("Checking for commands");
            #[cfg(feature = "nope")]
            match self.serial_cmd_rx.try_recv() {
                Ok(cmd) => {
                    debug!("Sending command: {:?}", cmd);
                    let buf = postcard::to_stdvec_cobs(&cmd)?;
                    // self.writer.write(&buf).context("Failed to send command")?;
                    self.writer
                        .write(&buf)
                        .await
                        .context("Failed to send command")?;
                    self.writer.flush().await.unwrap();
                }
                // Err(crossbeam_channel::TryRecvError::Disconnected) => {
                //     // debug!("Command channel disconnected");
                //     // return Err(anyhow!("Command channel disconnected"));
                // }
                Err(tokio::sync::mpsc::error::TryRecvError::Empty) => {}
                Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => {
                    debug!("Command channel closed");
                    return Err(anyhow!("Command channel closed"));
                }
            }

            // let n = self.reader.read(&mut self.raw_buf).await;
            // debug!("Read {:?} bytes from USB", n);
        }
    }
}
