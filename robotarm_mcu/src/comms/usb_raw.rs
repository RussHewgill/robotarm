use defmt::{debug, error, info};

use embassy_rp::usb::Driver;
use embassy_sync::channel::TryReceiveError;
use embassy_usb::{
    UsbDevice,
    driver::{Endpoint, EndpointIn, EndpointOut},
};
use postcard::accumulator::FeedResult;
use robotarm_protocol::SerialCommand;
use static_cell::StaticCell;

use crate::{MOTOR_ID_A, MOTOR_ID_B};

pub type UsbMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
// pub type UsbMutex = embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

pub type LogChannel =
    embassy_sync::channel::Channel<UsbMutex, robotarm_protocol::SerialLogMessage, 5>;
pub type CmdChannel = embassy_sync::channel::Channel<UsbMutex, robotarm_protocol::SerialCommand, 5>;

pub static LOG_CHAN: LogChannel = LogChannel::new();
pub static CMD_CHAN0: CmdChannel = CmdChannel::new();
pub static CMD_CHAN1: CmdChannel = CmdChannel::new();

// #[embassy_executor::task]
// pub async fn usb_test_task() {
//     debug!("Starting USB test task");
//     let tx = LOG_CHAN.sender();
//     loop {
//         embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
//         tx.send(robotarm_protocol::SerialLogMessage::DebugData {
//             id: 0,
//             timestamp: 0,
//             zero_electrical_angle: 0.0,
//         })
//         .await;
//     }
// }

/// owned by FOC loop task
#[derive(Clone)]
pub struct UsbLogger {
    /// MCU recieves command from USB task
    rx0: embassy_sync::channel::Receiver<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    rx1: embassy_sync::channel::Receiver<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    /// MCU sends log to USB task
    tx: embassy_sync::channel::Sender<'static, UsbMutex, robotarm_protocol::SerialLogMessage, 5>,
}

impl UsbLogger {
    pub fn new() -> Self {
        // let mut queue = heapless::Vec::new();
        // for _ in 0..4 {
        //     queue.push(heapless::Deque::new()).unwrap();
        // }
        let rx0 = CMD_CHAN0.receiver();
        let rx1 = CMD_CHAN1.receiver();
        let tx = LOG_CHAN.sender();
        Self { rx0, rx1, tx }
    }

    pub fn recv(&mut self, id: u8) -> Result<robotarm_protocol::SerialCommand, TryReceiveError> {
        match id {
            MOTOR_ID_A => self.rx0.try_receive(),
            MOTOR_ID_B => self.rx1.try_receive(),
            _ => Err(TryReceiveError::Empty),
        }
    }

    pub fn send_log_msg(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        if let Err(e) = self.tx.try_send(msg) {
            // error!("Failed to send log message to USB task");
        }
    }
}

pub struct UsbMonitor {
    buf: [u8; 512],
    read_ep: embassy_rp::usb::Endpoint<'static, embassy_rp::peripherals::USB, embassy_rp::usb::Out>,
    write_ep: embassy_rp::usb::Endpoint<'static, embassy_rp::peripherals::USB, embassy_rp::usb::In>,
}

// #[embassy_executor::task]
pub fn usb_init(
    spawner: &embassy_executor::Spawner,
    driver: Driver<'static, embassy_rp::peripherals::USB>,
) {
    debug!("Starting USB init");

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0d0, 0xcaf0);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB raw example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    // let mut config_descriptor = [0; 256];

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875322}"];

    // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // which causes it to use the built-in WinUSB driver automatically, which in turn
    // can be used by libusb/rusb software without needing a custom driver or INF file.
    // In principle you might want to call msos_feature() just on a specific function,
    // if your device also has other functions that still use standard class drivers.
    builder.msos_descriptor(embassy_usb::msos::windows_version::WIN8_1, 0);
    builder.msos_feature(embassy_usb::msos::CompatibleIdFeatureDescriptor::new(
        "WINUSB", "",
    ));
    builder.msos_feature(embassy_usb::msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        embassy_usb::msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // that uses our custom handler.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let mut read_ep = alt.endpoint_bulk_out(None, 64);
    // let mut write_ep = alt.endpoint_bulk_in(None, 64);
    let mut write_ep = alt.endpoint_bulk_in(None, 64);
    drop(function);

    // debug!("bulk out address: {}", read_ep.info());
    // debug!("bulk in address: {}", write_ep.info());

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    spawner.spawn(usb_run_task(usb).unwrap());

    let usb_monitor = UsbMonitor {
        buf: [0; 512],
        read_ep,
        write_ep,
    };
    let log_rx = LOG_CHAN.receiver();
    let cmd_tx0 = CMD_CHAN0.sender();
    let cmd_tx1 = CMD_CHAN1.sender();

    spawner.spawn(usb_logger_task(usb_monitor, cmd_tx0, cmd_tx1, log_rx).unwrap());

    // // Do stuff with the class!
    // let echo_fut = async {
    //     loop {
    //         read_ep.wait_enabled().await;
    //         info!("Connected");
    //         loop {
    //             let mut data = [0; 64];
    //             match read_ep.read(&mut data).await {
    //                 Ok(n) => {
    //                     info!("Got bulk: {:a}", data[..n]);
    //                     // Echo back to the host:
    //                     write_ep.write(&data[..n]).await.ok();
    //                 }
    //                 Err(_) => break,
    //             }
    //         }
    //         info!("Disconnected");
    //     }
    // };

    // // Run everything concurrently.
    // // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    // embassy_futures::join::join(usb_fut, echo_fut).await;
}

#[embassy_executor::task]
async fn usb_logger_task(
    mut usb_monitor: UsbMonitor,
    cmd_tx0: embassy_sync::channel::Sender<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    cmd_tx1: embassy_sync::channel::Sender<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        UsbMutex,
        robotarm_protocol::SerialLogMessage,
        5,
    >,
) -> ! {
    debug!("Starting USB logger task");

    let mut buf: [u8; 4096];
    let mut accum = postcard::accumulator::CobsAccumulator::<4096>::new();

    // let read_ep =

    // buf = [0; 4096];

    loop {
        buf = [0; 4096];

        // let msg = log_rx.receive().await;
        // debug!("Sending log message: {:?}", msg);

        // if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut usb_monitor.buf) {
        //     if encoded.len() <= 64 {
        //         // let _ = self.tx.write_packet(encoded).await;
        //         // unimplemented!()
        //     } else {
        //         // for i in encoded.chunks(64) {
        //         //     let _ = self.tx.write_packet(i).await;
        //         // }
        //         error!("Encoded message too long for USB packet");
        //         // let mut _ = self.tx.write_packet(&encoded[..64]).await;
        //     }
        //     // let _ = self.class.write_packet(encoded).await;
        //     if let Err(e) = usb_monitor.write_ep.write(&encoded).await {
        //         error!("Failed to write USB packet: {:?}", e);
        //     }
        //     // usb_monitor.write_ep.
        // } else {
        //     error!("Failed to encode message");
        // }

        // match usb_monitor.read_ep.read(&mut buf).await {
        //     Ok(n) => {
        //         debug!("Received {} bytes from USB", n);
        //     }
        //     Err(e) => {
        //         error!("USB read error");
        //     }
        // }

        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            // usb_monitor.rx.read_packet(&mut buf),
            usb_monitor.read_ep.read(&mut buf),
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                // debug!("Sending log message: {:?}", msg);
                // #[cfg(feature = "nope")]

                // if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut usb_monitor.buf) {
                //     // if encoded.len() <= 64 {
                //     //     // let _ = self.tx.write_packet(encoded).await;
                //     //     // unimplemented!()
                //     //     if let Err(e) = usb_monitor.write_ep.write(encoded).await {
                //     //         error!("Failed to write USB packet: {:?}", e);
                //     //     }
                //     // } else {
                //     //     // for i in encoded.chunks(64) {
                //     //     //     let _ = self.tx.write_packet(i).await;
                //     //     // }
                //     //     // error!("Encoded message too long for USB packet");
                //     //     // let mut _ = self.tx.write_packet(&encoded[..64]).await;
                //     //     for c in encoded.chunks(64) {
                //     //         if let Err(e) = usb_monitor.write_ep.write(c).await {
                //     //             error!("Failed to write USB packet: {:?}", e);
                //     //         }
                //     //     }
                //     // }
                //     // // let _ = self.class.write_packet(encoded).await;
                //     if let Err(e) = usb_monitor.write_ep.write(&encoded).await {
                //         error!("Failed to write USB packet: {:?}", e);
                //     }
                //     // usb_monitor.write_ep.
                // } else {
                //     error!("Failed to encode message");
                // }

                if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut usb_monitor.buf) {
                    // debug!("Encoded len = {}", encoded.len());
                    if let Err(e) = usb_monitor.write_ep.write(&encoded).await {
                        error!("Failed to write USB packet: {:?}", e);
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                error!("USB read error");
                // accum = postcard::accumulator::CobsAccumulator::<4096>::new();
                debug!("Waiting for USB connection...");
                // usb_monitor.rx.wait_connection().await;
                usb_monitor.read_ep.wait_enabled().await;
                debug!("USB connected");
            }
            embassy_futures::select::Either::Second(Ok(n)) => {
                // debug!("Received {} bytes from USB", n);
                let mut window = &buf[..n];
                'cobs: while !window.is_empty() {
                    // window = match accum.feed::<SerialCommand>(&buf[..n]) {
                    window = match accum.feed::<SerialCommand>(window) {
                        FeedResult::Success { data, remaining } => {
                            // debug!("Received complete message from USB: {:?}", data);

                            let mut retries = 0;
                            loop {
                                let tx = match data.id() {
                                    MOTOR_ID_A => &cmd_tx0,
                                    MOTOR_ID_B => &cmd_tx1,
                                    _ => {
                                        error!(
                                            "Received command with invalid id: {}, dropping command",
                                            data.id()
                                        );
                                        break;
                                    }
                                };

                                match tx.try_send(data) {
                                    Ok(()) => break,
                                    Err(e) => {
                                        error!("Failed to send command to main task, retrying...");
                                        if retries >= 5 {
                                            error!(
                                                "Failed to send command after {} retries, dropping command",
                                                retries
                                            );
                                            break;
                                        } else {
                                            retries += 1;
                                            embassy_futures::yield_now().await;
                                        }
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

        #[cfg(feature = "nope")]
        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            // usb_monitor.rx.read_packet(&mut buf),
            usb_monitor.read_ep.read(&mut buf),
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                // debug!("Sending log message: {:?}", msg);
                // #[cfg(feature = "nope")]
                if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut usb_monitor.buf) {
                    if encoded.len() <= 64 {
                        // let _ = self.tx.write_packet(encoded).await;
                        // unimplemented!()
                    } else {
                        // for i in encoded.chunks(64) {
                        //     let _ = self.tx.write_packet(i).await;
                        // }
                        error!("Encoded message too long for USB packet");
                        // let mut _ = self.tx.write_packet(&encoded[..64]).await;
                    }
                    // let _ = self.class.write_packet(encoded).await;
                    if let Err(e) = usb_monitor.write_ep.write(&encoded).await {
                        error!("Failed to write USB packet: {:?}", e);
                    }
                    // usb_monitor.write_ep.
                } else {
                    error!("Failed to encode message");
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                error!("USB read error");
                // accum = postcard::accumulator::CobsAccumulator::<4096>::new();
                // debug!("Waiting for USB connection...");
                // usb_monitor.rx.wait_connection().await;
                // debug!("USB connected");
            }
            embassy_futures::select::Either::Second(Ok(n)) => {
                debug!("Received {} bytes from USB", n);
                // unimplemented!()
            }
        }

        //
    }

    #[cfg(feature = "nope")]
    loop {
        // yield_now().await;
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
                accum = postcard::accumulator::CobsAccumulator::<4096>::new();
                debug!("Waiting for USB connection...");
                usb_monitor.rx.wait_connection().await;
                debug!("USB connected");
            }
            embassy_futures::select::Either::Second(Ok(n)) => {
                // debug!("Received {} bytes from USB", n);
                let mut window = &buf[..n];
                'cobs: while !window.is_empty() {
                    // window = match accum.feed::<SerialCommand>(&buf[..n]) {
                    window = match accum.feed::<SerialCommand>(window) {
                        FeedResult::Success { data, remaining } => {
                            // debug!("Received complete message from USB: {:?}", data);

                            let mut retries = 0;
                            loop {
                                let tx = match data.id() {
                                    MOTOR_ID_A => &cmd_tx0,
                                    MOTOR_ID_B => &cmd_tx1,
                                    _ => {
                                        error!(
                                            "Received command with invalid id: {}, dropping command",
                                            data.id()
                                        );
                                        break;
                                    }
                                };

                                match tx.try_send(data) {
                                    Ok(()) => break,
                                    Err(e) => {
                                        error!("Failed to send command to main task, retrying...");
                                        if retries >= 5 {
                                            error!(
                                                "Failed to send command after {} retries, dropping command",
                                                retries
                                            );
                                            break;
                                        } else {
                                            retries += 1;
                                            embassy_futures::yield_now().await;
                                        }
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
async fn usb_run_task(
    mut usb: UsbDevice<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
) -> ! {
    debug!("Starting USB task");
    usb.run().await
}
