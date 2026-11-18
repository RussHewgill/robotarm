use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use futures_util::{SinkExt, StreamExt, future, pin_mut, stream::TryStreamExt};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc::{UnboundedReceiver, UnboundedSender},
};
use tokio_tungstenite::tungstenite::Message;

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct WebsocketListener {
    pub server_addr: String,
    /// send commands to rs485
    pub tx_command: UnboundedSender<SerialCommand>,
    /// get messages from rs485
    pub tx_msg: tokio::sync::broadcast::Sender<SerialLogMessage>,
}

impl WebsocketListener {
    pub async fn run(&mut self) -> Result<()> {
        let Ok(listener) = TcpListener::bind(&self.server_addr).await else {
            error!("Failed to bind to server address: {}", &self.server_addr);
            bail!("Failed to bind to server address: {}", &self.server_addr);
        };
        info!("Listening on: {}", self.server_addr);

        while let Ok((stream, _)) = listener.accept().await {
            let tx_command = self.tx_command.clone();
            let rx_msg = self.tx_msg.subscribe();
            tokio::spawn(async move { Self::accept_connection(tx_command, rx_msg, stream) });
        }

        unimplemented!()
    }

    async fn accept_connection(
        mut tx_command: UnboundedSender<SerialCommand>,
        mut rx_msg: tokio::sync::broadcast::Receiver<SerialLogMessage>,
        stream: TcpStream,
    ) {
        let addr = stream
            .peer_addr()
            .expect("connected streams should have a peer address");
        let ws_stream = tokio_tungstenite::accept_async(stream)
            .await
            .expect("Error during the websocket handshake occurred");
        info!("New WebSocket connection: {}", addr);

        let (mut write, mut read) = ws_stream.split();

        loop {
            tokio::select! {
                Ok(msg) = rx_msg.recv() => {
                    let Some(buf) = robotarm_protocol::serialize_std(msg) else {
                        error!("Failed to serialize message");
                        continue;
                    };
                    if let Err(e) = write
                        .send(Message::Binary(bytes::Bytes::from_owner(buf)))
                        .await
                    {
                        error!("WebSocket write error: {:?}", e);
                    }
                },
                cmd = read.next() => match cmd {
                    Some(Ok(Message::Binary(bytes))) => {
                        let crc0 = u32::from_le_bytes(
                            bytes[bytes.len() - robotarm_protocol::CRC_SIZE..]
                                .try_into()
                                .unwrap(),
                        );

                        let crc1 = robotarm_protocol::CRC
                            .checksum(&bytes[..bytes.len() - robotarm_protocol::CRC_SIZE]);

                        if crc0 != crc1 {
                            error!("CRC mismatch: expected {:08x}, got {:08x}", crc1, crc0);
                            continue;
                        }

                        let mut buf = bytes[..bytes.len() - robotarm_protocol::CRC_SIZE].to_vec();
                        let msg: Result<SerialCommand, _> = postcard::from_bytes_cobs(&mut buf);

                        match msg {
                            Ok(cmd) => {
                                if let Err(e) = tx_command.send(cmd) {
                                    error!("Failed to send command: {:?}", e);
                                }
                            }
                            Err(e) => {
                                error!("Failed to deserialize command: {:?}", e);
                            }
                        }
                    },
                    Some(Ok(_)) => {
                        error!("Unexpected WebSocket message");
                    }
                    Some(Err(e)) => {
                        error!("WebSocket read error: {:?}", e);
                    },
                    None => {
                        info!("WebSocket connection closed");
                        break;
                    }
                },
            }

            if let Ok(msg) = rx_msg.recv().await {}

            if let Some(Ok(Message::Binary(bytes))) = read.next().await {}
        }
    }
}
