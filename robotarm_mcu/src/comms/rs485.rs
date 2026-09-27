use defmt::{debug, error, info};

use postcard::accumulator::FeedResult;

use crate::{MOTOR_ID_A, MOTOR_ID_B, comms::usb_raw::UsbMutex};
use robotarm_protocol::SerialCommand;

pub async fn modbus_task(mut max485: crate::hardware::max485::Max485) {
    info!("Starting rs485 task");

    const UNIT: u8 = 0;

    loop {

        //
    }
}

#[embassy_executor::task]
async fn rx485_logger_task(
    // mut usb_monitor: UsbMonitor,
    mut max485: crate::hardware::max485::Max485,
    cmd_tx0: embassy_sync::channel::Sender<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    cmd_tx1: embassy_sync::channel::Sender<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    log_rx: embassy_sync::channel::Receiver<
        'static,
        UsbMutex,
        robotarm_protocol::SerialLogMessage,
        5,
    >,
) -> ! {
    let mut buf: [u8; 1024];
    let mut tx_buf: [u8; 256] = [0; 256];
    let mut accum = postcard::accumulator::CobsAccumulator::<1024>::new();

    loop {
        buf = [0; 1024];

        match embassy_futures::select::select(
            log_rx.receive(),
            // usb_monitor.class.read_packet(&mut buf),
            // usb_monitor.rx.read_packet(&mut buf),
            // usb_monitor.read_ep.read(&mut buf),
            max485.receive(&mut buf),
        )
        .await
        {
            embassy_futures::select::Either::First(msg) => {
                if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut tx_buf) {
                    // debug!("Encoded len = {}", encoded.len());
                    if let Err(e) = max485.send(&encoded).await {
                        error!("Failed to write RS485 packet: {:?}", e);
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                unimplemented!()
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
    }
}
