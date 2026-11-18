use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct CommsController {
    pub rx_command: UnboundedReceiver<SerialCommand>,
    pub tx_command: UnboundedSender<SerialCommand>,

    pub tx_msg: tokio::sync::broadcast::Sender<SerialLogMessage>,
    // pub rx_msg: tokio::sync::broadcast::S<SerialLogMessage>,
}

impl CommsController {
    pub async fn init(
        port: &str,
        baud_rate: u32,
        websocket_addr: &str,
        //
    ) {
        let (tx_command, rx_command) = tokio::sync::mpsc::unbounded_channel();
        // let (tx_msg, rx_msg) = tokio::sync::mpsc::unbounded_channel();
        let (tx_msg, rx_msg) = tokio::sync::broadcast::channel(32);

        let port = port.to_string();
        let tx_msg2 = tx_msg.clone();
        let rs485_handle = tokio::spawn(async move {
            let mut rs485 =
                crate::rs485_listener::Rs485Listener::new(rx_command, tx_msg2, port, baud_rate);
            loop {
                if let Err(e) = rs485.listen().await {
                    error!("Error in RS485 listener: {:?}", e);
                }
            }
        });

        let server_addr = websocket_addr.to_string();
        let websocket_handle = tokio::spawn(async move {
            let mut websocket_listener = crate::websocket_listener::WebsocketListener {
                server_addr,
                tx_command,
                tx_msg,
            };
            if let Err(e) = websocket_listener.run().await {
                error!("Error in WebSocket listener: {:?}", e);
            }
        });

        let _ = tokio::join!(rs485_handle, websocket_handle);
    }
}
