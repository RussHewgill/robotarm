use defmt::{debug, error};
use embassy_futures::join::join;
use embassy_sync::{channel::TryReceiveError, pipe::Pipe};
use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
    driver::Driver,
};
use postcard::accumulator::FeedResult;
use static_cell::StaticCell;

static LOG_CHAN: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    robotarm_protocol::SerialLogMessage,
    1,
> = embassy_sync::channel::Channel::new();
static CMD_CHAN: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    robotarm_protocol::SerialCommand,
    1,
> = embassy_sync::channel::Channel::new();

pub struct UsbLogger {
    /// MCU recieves command from USB task
    rx: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        robotarm_protocol::SerialCommand,
        1,
    >,
    /// MCU sends log to USB task
    tx: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        // embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        robotarm_protocol::SerialLogMessage,
        1,
    >,
}

impl UsbLogger {
    pub fn new() -> Self {
        let rx = CMD_CHAN.receiver();
        let tx = LOG_CHAN.sender();
        Self { rx, tx }
    }

    pub async fn recv(&mut self) -> Result<robotarm_protocol::SerialCommand, TryReceiveError> {
        self.rx.try_receive()
    }

    pub fn send_log_msg(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        if let Err(e) = self.tx.try_send(msg) {
            // error!("Failed to send log message to USB task");
        }
    }
}

pub struct UsbMonitor {
    // class: CdcAcmClass<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
    rx: Receiver<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
    tx: Sender<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
    buf: [u8; 256],
}

impl UsbMonitor {
    pub fn init(
        spawner: &embassy_executor::Spawner,
        driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>,
    ) {
        // let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        // Create embassy-usb Config
        let config = {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("Embassy Serial Logger");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let mut class = {
            static STATE: StaticCell<embassy_usb::class::cdc_acm::State<'static>> =
                StaticCell::new();
            let state = STATE.init(embassy_usb::class::cdc_acm::State::new());
            // embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 256)
            embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64)
        };

        let (tx, rx) = class.split();

        let mut out = Self {
            // class,
            rx,
            tx,
            buf: [0; 256],
        };

        let usb = builder.build();

        let log_rx = LOG_CHAN.receiver();
        // let log_tx = LOG_CHAN.sender();
        // let cmd_rx = CMD_CHAN.receiver();
        let cmd_tx = CMD_CHAN.sender();

        spawner.spawn(usb_task(usb)).unwrap();
        spawner.spawn(usb_logger_task(out, cmd_tx, log_rx)).unwrap();

        // UsbLogger {
        //     rx: cmd_rx,
        //     tx: log_tx,
        // }
    }

    // async fn wait_connection(&mut self) {
    //     // self.class.wait_connection().await;
    // }

    // pub async fn wait_read(&mut self, buf: &mut [u8]) -> usize {
    //     self.class.read_packet(buf).await.unwrap_or(0)
    // }

    #[cfg(feature = "nope")]
    async fn send(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        match msg {
            robotarm_protocol::SerialLogMessage::MotorData {
                id,
                timestamp,
                target,
                position,
                velocity,
            } => {
                let start = b"\0";
                let end = b"\0\r\n";
                let sep = b"\t";

                self.class
                    .write_packet(b"target,velocity,position\r\n")
                    .await
                    .unwrap();
                self.class.write_packet(start).await.unwrap();
                self.class.write_packet(b"0.0").await.unwrap();
                self.class.write_packet(sep).await.unwrap();
                self.class.write_packet(b"0.0").await.unwrap();
                self.class.write_packet(sep).await.unwrap();
                self.class.write_packet(b"0.0").await.unwrap();
                self.class.write_packet(end).await.unwrap();
            }
        }
    }

    // #[cfg(feature = "nope")]
    async fn send(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        if !self.tx.dtr() {
            // error!("USB not connected, cannot send log message");
            return;
        }
        if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut self.buf) {
            if encoded.len() <= 64 {
                let _ = self.tx.write_packet(encoded).await;
            } else {
                error!("Encoded message too long for USB packet");
            }
            // let _ = self.class.write_packet(encoded).await;
        } else {
            error!("Failed to encode message");
        }
    }

    // async fn _send(&mut self, data: &[u8]) {
    //     let _ = self.class.write_packet(data).await;
    //     // let _ = self.class.write_packet(b"\n").await;
    // }
}

#[embassy_executor::task]
async fn usb_logger_task(
    mut usb_monitor: UsbMonitor,
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
) -> ! {
    let mut buf: [u8; 4096];
    let mut accum = postcard::accumulator::CobsAccumulator::<4096>::new();
    // let mut prev_msg = None;

    loop {
        buf = [0; 4096];
        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            usb_monitor.rx.read_packet(&mut buf),
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
                usb_monitor.send(msg).await;
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

#[embassy_executor::task]
async fn usb_task(
    mut usb: UsbDevice<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
) -> ! {
    usb.run().await
}
