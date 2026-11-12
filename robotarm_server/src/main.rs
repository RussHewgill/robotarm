#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

pub mod logging;
pub mod rs485;
pub mod websocket_server;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serialport::SerialPortType;
use tokio::net::TcpListener;
use tokio_serial::SerialPortBuilderExt;

pub const SERVER_ADDR: &str = "0.0.0.0:8080";

#[tokio::main]
async fn main() -> Result<()> {
    logging::init_logs();

    debug!("Starting");

    let state =
        websocket_server::PeerMap::new(std::sync::Mutex::new(std::collections::HashMap::new()));

    let try_socket = TcpListener::bind(&SERVER_ADDR).await;
    let listener = try_socket.expect("Failed to bind");
    info!("Listening on: {}", SERVER_ADDR);

    // Let's spawn the handling of each connection in a separate task.
    while let Ok((stream, addr)) = listener.accept().await {
        tokio::spawn(websocket_server::handle_connection(
            state.clone(),
            stream,
            addr,
        ));
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
