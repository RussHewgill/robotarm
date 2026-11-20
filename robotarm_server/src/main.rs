#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

pub mod comms_controller;
pub mod logging;
pub mod rs485_listener;
pub mod websocket_listener;
// pub mod websocket_server;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serialport::SerialPortType;
use tokio::net::TcpListener;
use tokio_serial::SerialPortBuilderExt;

use robotarm_protocol::{SerialCommand, SerialLogMessage, SerialPacket};

pub const SERVER_ADDR: &str = "0.0.0.0:8080";

/// RS485 test
#[tokio::main]
// #[cfg(feature = "nope")]
async fn main() -> Result<()> {
    logging::init_logs();

    debug!("Starting RS485 test");

    for port in serialport::available_ports().unwrap() {
        debug!("Found port: {:?}", port);
    }

    // let port = "/dev/ttyS0";
    let port = "/dev/ttyAMA0";
    // let baud_rate = 9600;
    // let baud_rate = 115_200;
    // let baud_rate = 921_600;
    // let baud_rate = 921_600 * 2;
    let baud_rate = robotarm_protocol::BAUD_RATE;

    let (tx_command, rx_command) = tokio::sync::mpsc::unbounded_channel();
    let (tx_msg, rx_msg) = tokio::sync::broadcast::channel(32);

    // let mut rs485 =
    //     crate::rs485_listener::Rs485Listener::new(rx_command, tx_msg, port.to_string(), baud_rate);

    // let mut port = tokio_serial::new(port, baud_rate).open_native_async()?;
    // port.set_exclusive(false)
    //     .expect("Unable to set serial port non-exclusive");

    tokio::spawn(async move {
        let mut rs485 = crate::rs485_listener::Rs485Listener::new(
            rx_command,
            tx_msg,
            port.to_string(),
            baud_rate,
        );
        loop {
            if let Err(e) = rs485.listen().await {
                error!("RS485 listener error: {:?}", e);
            }
        }
    });

    // rs485.listen().await.unwrap();

    // #[cfg(feature = "nope")]
    loop {
        // debug!("Looping");

        let cmd = SerialCommand::PingRequest {
            id: robotarm_protocol::MotorId(0),
            hardware_id: 0,
        };

        // if let Some(packet) = robotarm_protocol::serialize_std(SerialPacket::Command(cmd)) {
        //     // debug!("packet: {:02x?}", packet);
        //     if let Err(e) = port.try_write(&packet) {
        //         error!("Error writing to serial port: {:?}", e);
        //     }
        // }

        debug!("Sending ping");
        tx_command.send(cmd).unwrap();

        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }

    // let mut x = 0;
    #[cfg(feature = "nope")]
    loop {
        let msg = SerialCommand::PingRequest {
            id: 0,
            hardware_id: x,
        };
        x += 1;
        let mut buf = postcard::to_stdvec(&msg).context("Failed to serialize buffer")?;
        let crc = crc32fast::hash(&buf);
        buf.extend_from_slice(&crc.to_le_bytes());
        let buf = postcard::to_stdvec_cobs(&buf).context("Failed to serialize buffer")?;

        match port.try_write(&buf) {
            Ok(n) => {
                debug!("Wrote {} bytes", n);
            }
            Err(e) => {
                error!("Error writing to serial port: {:?}", e);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    // debug!("Starting RS485 listener");
    // rs485.listen().await?;

    // use postcard::experimental::max_size::MaxSize;
    // let size = robotarm_protocol::SerialLogMessage::POSTCARD_MAX_SIZE;

    // debug!("SerialLogMessage max size: {}", size);

    // let size = std::mem::size_of::<heapless::index_map::FnvIndexMap<u8, f32, 4>>();
    // let map = std::collections::HashMap::<u8, f32>::with_capacity(8);
    // let size = std::mem::size_of_val(&map);

    // debug!("Size: {}", size);

    // Ok(())
}

/// Main
// #[tokio::main]
#[cfg(feature = "nope")]
async fn main() -> Result<()> {
    logging::init_logs();

    debug!("Starting");

    tokio::spawn(async move {
        loop {
            //
        }
    });

    let state =
        websocket_server::PeerMap::new(std::sync::Mutex::new(std::collections::HashMap::new()));

    let try_socket = TcpListener::bind(&SERVER_ADDR).await;
    let listener = try_socket.expect("Failed to bind");
    info!("Listening on: {}", SERVER_ADDR);

    while let Ok((stream, _)) = listener.accept().await {
        tokio::spawn(websocket_server::accept_connection(stream));
    }

    // rs485 test
    {
        // yellow:  GPIO 14, TX
        // orange:  GPIO 15, RX
        // red:     GPIO 18, EN

        // let port = "/dev/ttyS0";

        // let mut port = tokio_serial::new(port, 9600).open_native_async()?;

        // port.set_exclusive(false)
        //     .expect("Unable to set serial port exclusive to false");

        //
    }

    Ok(())
}
