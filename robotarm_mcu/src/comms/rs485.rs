use defmt::{debug, error, info, warn};

use embassy_time::Instant;
use postcard::accumulator::FeedResult;

use crate::comms::{CommsMutex, logger::CommsLogger};
use robotarm_protocol::{MotorId, SerialCommand, SerialLogMessage, SerialPacket};

enum CommsState {
    RxOnly,
    Responding(Instant),
}

pub struct Rs485Comms {
    state: CommsState,
    rs485: crate::hardware::sp3485::Sp3485,
    cmd_tx: embassy_sync::channel::Sender<'static, CommsMutex, robotarm_protocol::SerialCommand, 5>,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        CommsMutex,
        robotarm_protocol::SerialLogMessage,
        5,
    >,
    motor_id: MotorId,
    accum: postcard::accumulator::CobsAccumulator<512>,
    send_buf: [u8; 256],
}

impl Rs485Comms {
    async fn run(&mut self) {
        // debug!("Running");
        match self.state {
            CommsState::RxOnly => {
                self.listen().await;
            }
            CommsState::Responding(instant) => {
                // debug!("Responding");
                self.respond(instant).await;
            }
        }
    }

    async fn respond(&mut self, instant: Instant) {
        loop {
            if Instant::now() >= instant {
                warn!("Response time elapsed, switching to RxOnly");
                self.state = CommsState::RxOnly;
                break;
            }

            match self.log_rx.try_receive() {
                Ok(msg) => {
                    match robotarm_protocol::serialize(
                        SerialPacket::LogMessage(msg),
                        &mut self.send_buf,
                    ) {
                        Some(len) => {
                            if let Err(e) = self.rs485.send(&self.send_buf[..len]).await {
                                error!("Failed to send log message: {:?}", e);
                            } else {
                                debug!("Log message sent successfully, setting to RxOnly");
                                self.state = CommsState::RxOnly;
                                return;
                            }
                        }
                        None => {
                            error!("Failed to serialize log message");
                        }
                    }
                    //
                }
                Err(embassy_sync::channel::TryReceiveError::Empty) => {
                    // debug!("Empty");
                }
            }
        }
    }

    /// listen for one incoming rs485 byte
    async fn listen(&mut self) {
        match embassy_futures::select::select(
            self.log_rx.receive(),
            self.rs485.receive(),
            //
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                error!("FOC tried to send message without being requested or out of time");
            }
            embassy_futures::select::Either::Second(Ok(byte)) => {
                // debug!("Feeding accumulator with byte: {}", byte);
                self.feed_accum(byte).await;
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                error!("Failed to receive RS485 byte");
            }
        }
    }

    /// feed accumulator, if message is found
    async fn feed_accum(&mut self, byte: u8) -> Option<()> {
        // #[cfg(feature = "nope")]
        match self.accum.feed::<heapless::Vec<u8, 128>>(&[byte]) {
            FeedResult::Consumed => {
                // debug!("Byte consumed by accumulator: {:02x}", byte);
                None
            }
            FeedResult::Success { data, remaining } => {
                // debug!("Success");

                let crc0 = robotarm_protocol::CRCType::from_le_bytes(
                    data[data.len() - robotarm_protocol::CRC_SIZE..]
                        .try_into()
                        .unwrap(),
                );
                let data = &data[..data.len() - robotarm_protocol::CRC_SIZE];
                let crc = robotarm_protocol::CRC.checksum(data);
                if crc != crc0 {
                    error!("CRC mismatch: expected {:08x}, got {:08x}", crc0, crc);
                    error!("TODO: request retransmit");
                    return None;
                    // continue;
                }

                let data = match postcard::from_bytes::<SerialPacket>(&data) {
                    Ok(data) => data,
                    Err(e) => {
                        error!("Failed to deserialize incoming data");
                        // continue;
                        return None;
                    }
                };

                let cmd = match data {
                    SerialPacket::Command(cmd) => cmd,
                    _ => return None,
                };

                debug!("Got command: {:?}", cmd);

                if !cmd.is_broadcast() && !cmd.is_handshake() && cmd.id() != self.motor_id {
                    error!(
                        "Received command with invalid id: {}, dropping command",
                        cmd.id()
                    );
                    return None;
                }

                if let Some((id, timeout)) = cmd.response_expected() {
                    // debug!("timeout for server: {}", timeout_server);
                    // debug!("timeout for motor:  {}", timeout_motor);
                    self.state = CommsState::Responding(
                        Instant::now() + embassy_time::Duration::from_micros(timeout),
                    );
                }

                match cmd {
                    SerialCommand::PingRequest { id, hardware_id } => {
                        crate::comms::LOG_CHAN
                            .send(robotarm_protocol::SerialLogMessage::PingResponse { id })
                            .await;
                        return Some(());
                    }
                    _ => {}
                }

                let mut retries = 0;

                loop {
                    match self.cmd_tx.try_send(cmd) {
                        Ok(()) => return Some(()),
                        Err(e) => {
                            error!("Failed to send command to main task, retrying...");
                            if retries >= 5 {
                                error!(
                                    "Failed to send command after {} retries, dropping command",
                                    retries
                                );
                                return None;
                            } else {
                                retries += 1;
                                embassy_futures::yield_now().await;
                            }
                        }
                    }
                }
            }
            FeedResult::OverFull(_) => {
                error!("CobsAccumulator overflow");
                None
            }
            FeedResult::DeserError(e) => {
                error!("CobsAccumulator deserialization error: {:?}", e);
                None
            }
        }
    }
}

#[embassy_executor::task]
pub async fn rs485_test_response_task(logger: CommsLogger) -> ! {
    loop {
        //
    }
}

#[embassy_executor::task]
pub async fn rs485_logger_task(
    // mut usb_monitor: UsbMonitor,
    rs485: crate::hardware::sp3485::Sp3485,
    cmd_tx: embassy_sync::channel::Sender<'static, CommsMutex, robotarm_protocol::SerialCommand, 5>,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        CommsMutex,
        robotarm_protocol::SerialLogMessage,
        5,
    >,
    motor_id: MotorId,
) -> ! {
    let mut comms = Rs485Comms {
        state: CommsState::RxOnly,
        rs485,
        cmd_tx,
        log_rx,
        motor_id,
        accum: postcard::accumulator::CobsAccumulator::<512>::new(),
        send_buf: [0u8; 256],
    };

    // let mut packet = [0x04u8, 0x06, 0x01, 0x03, 0x01, 0x03, 0x01, 0x0f, 0x00];

    // let res: Result<heapless::Vec<u8, 128>, _> = postcard::from_bytes_cobs(&mut packet[..]);

    // for byte in packet {
    //     comms.feed_accum(byte).await;
    // }

    loop {
        comms.run().await;
    }
}

// #[embassy_executor::task]
#[cfg(feature = "nope")]
pub async fn rs485_logger_task(
    // mut usb_monitor: UsbMonitor,
    mut rs485: crate::hardware::sp3485::Sp3485,
    cmd_tx0: embassy_sync::channel::Sender<
        'static,
        CommsMutex,
        robotarm_protocol::SerialCommand,
        5,
    >,
    // cmd_tx1: embassy_sync::channel::Sender<
    //     'static,
    //     CommsMutex,
    //     robotarm_protocol::SerialCommand,
    //     5,
    // >,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        CommsMutex,
        robotarm_protocol::SerialLogMessage,
        5,
    >,
    motor_id: MotorId,
) -> ! {
    // let mut buf: [u8; 1024];
    // let mut tx_buf: [u8; 256] = [0; 256];
    // let mut tx_buf = heapless::Vec::<u8, 256>::new();
    let mut tx_buf = [0u8; 256];
    let mut tx_buf1 = heapless::Vec::<u8, 256>::new();
    let mut tx_buf2 = heapless::Vec::<u8, 256>::new();
    let mut accum = postcard::accumulator::CobsAccumulator::<1024>::new();

    loop {
        // buf = [0; 1024];

        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            // usb_monitor.rx.read_packet(&mut buf),
            // usb_monitor.read_ep.read(&mut buf),
            // rs485.receive(&mut buf),
            rs485.receive(),
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                if let Ok(encoded) = postcard::to_slice(&msg, &mut tx_buf) {
                    let crc = crc32fast::hash(&encoded);
                    tx_buf1.clear();
                    tx_buf1.extend_from_slice(&tx_buf).unwrap();
                    tx_buf1.extend_from_slice(&crc.to_le_bytes()).unwrap();
                    if let Ok(encoded) = postcard::to_slice_cobs(&tx_buf1, &mut tx_buf2) {
                        if let Err(e) = rs485.send(encoded).await {
                            error!("Failed to write RS485 packet: {:?}", e);
                        }
                    }
                }

                // if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut tx_buf) {
                //     // debug!("Encoded len = {}", encoded.len());
                //     if let Err(e) = max485.send(&encoded).await {
                //         error!("Failed to write RS485 packet: {:?}", e);
                //     }
                // }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                error!("Failed to receive RS485 packet: {:?}", e);
            }

            embassy_futures::select::Either::Second(Ok(byte)) => {
                match accum.feed::<heapless::Vec<u8, 64>>(&[byte]) {
                    FeedResult::Consumed => {}
                    FeedResult::Success { data, remaining } => {
                        if remaining.len() > 0 {
                            error!("Remaining unprocessed bytes: {:?}", remaining);
                        }

                        let crc0 = u32::from_le_bytes(data[data.len() - 4..].try_into().unwrap());
                        let data = &data[..data.len() - 4];
                        let crc = crc32fast::hash(data);
                        if crc != crc0 {
                            error!("CRC mismatch: expected {:08x}, got {:08x}", crc0, crc);
                            error!("TODO: request retransmit");
                            continue;
                        }

                        let data = match postcard::from_bytes::<SerialCommand>(&data) {
                            Ok(data) => data,
                            Err(e) => {
                                error!("Failed to deserialize incoming data");
                                continue;
                            }
                        };

                        match data {
                            // SerialCommand::PingRequest { id, hardware_id } => {
                            //     unimplemented!()
                            // }
                            _ => {
                                let mut retries = 0;

                                'sendloop: loop {
                                    if !data.is_broadcast()
                                        && !data.is_handshake()
                                        && data.id() != motor_id
                                    {
                                        error!(
                                            "Received command with invalid id: {}, dropping command",
                                            data.id()
                                        );
                                        break 'sendloop;
                                    }

                                    match cmd_tx0.try_send(data) {
                                        Ok(()) => break 'sendloop,
                                        Err(e) => {
                                            error!(
                                                "Failed to send command to main task, retrying..."
                                            );
                                            if retries >= 5 {
                                                error!(
                                                    "Failed to send command after {} retries, dropping command",
                                                    retries
                                                );
                                                break 'sendloop;
                                            } else {
                                                retries += 1;
                                                embassy_futures::yield_now().await;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    FeedResult::DeserError(e) => {
                        error!("Failed to parse incoming byte");
                    }
                    FeedResult::OverFull(e) => {
                        error!("Incoming byte buffer overflowed");
                    }
                }
            }
        }
    }
}
