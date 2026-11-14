#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

pub mod logging;
pub mod rs485;
pub mod rs485_listener;
pub mod websocket_listener;
pub mod websocket_server;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serialport::SerialPortType;
use tokio::net::TcpListener;
use tokio_serial::SerialPortBuilderExt;

use robotarm_protocol::{SerialCommand, SerialLogMessage};

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
    let baud_rate = 921_600;
    // let baud_rate = 921_600 * 2;

    let (tx_command, rx_command) = tokio::sync::mpsc::unbounded_channel();
    let (tx_msg, rx_msg) = tokio::sync::mpsc::unbounded_channel();

    let mut rs485 = crate::rs485_listener::Rs485Listener::new(rx_command, tx_msg, port, baud_rate);

    debug!("Starting RS485 listener");
    rs485.listen().await?;

    // let buf = [3, 6, 1, 5, 190, 35, 194, 88, 0];
    // // [03, 06, 01, 05, be, 23, c2, 58, 00]
    // debug!("Buffer: {:02x?}", &buf);

    // rs485.debug_accum(&buf);

    // rs485.debug_accum(&[3]);
    // rs485.debug_accum(&[6, 1, 5, 190, 35, 194, 88, 0]);

    // let mut port = serial2_tokio::SerialPort::open(port, baud_rate)?;

    // let mut rs485_config = serial2::rs4xx::Rs485Config::new();
    // rs485_config.set_bus_termination(false);
    // rs485_config.set_full_duplex(false);
    // port.set_rs4xx_mode(rs485_config)?;

    // let mut buf = [0u8; 1024];
    // loop {
    //     match port.read(&mut buf).await {
    //         Ok(n) => {
    //             if n > 0 {
    //                 debug!("Read {} bytes: {:?}", n, &buf[..n]);
    //             }
    //         }
    //         Err(e) => {
    //             debug!("Error reading from serial port: {:?}", e);
    //         }
    //     }

    //     //
    // }

    // let mut port = tokio_serial::new(port, 9600).open_native_async()?;
    // debug!("Opened port");

    // port.set_exclusive(false)
    //     .expect("Unable to set serial port non-exclusive");
    // debug!("Set port to non-exclusive");

    // use tokio::io::AsyncReadExt;

    // let mut buf = [0u8; 1024];
    // loop {
    //     if let Ok(n) = port.try_read(&mut buf) {
    //         if n > 0 {
    //             debug!("Read {} bytes: {:?}", n, &buf[..n]);
    //         }
    //     }
    // }

    Ok(())
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
