use defmt::{debug, error, info};

use postcard::accumulator::FeedResult;

use crate::comms::CommsMutex;
use robotarm_protocol::{MotorId, SerialCommand};

#[embassy_executor::task]
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
