use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use std::{
    collections::HashMap,
    net::SocketAddr,
    sync::{Arc, Mutex},
};

use futures_util::{SinkExt, StreamExt, future, pin_mut, stream::TryStreamExt};
use tokio::{net::TcpStream, sync::mpsc::UnboundedSender};
use tokio_tungstenite::tungstenite::Message;

pub type Tx = UnboundedSender<Message>;
pub type PeerMap = Arc<Mutex<HashMap<SocketAddr, Tx>>>;

/// example
pub async fn accept_connection(stream: TcpStream) {
    let addr = stream
        .peer_addr()
        .expect("connected streams should have a peer address");
    info!("Peer address: {}", addr);

    let ws_stream = tokio_tungstenite::accept_async(stream)
        .await
        .expect("Error during the websocket handshake occurred");

    info!("New WebSocket connection: {}", addr);

    let (mut write, mut read) = ws_stream.split();

    let mut id = 0;
    loop {
        debug!("Looping");
        let msg = robotarm_protocol::SerialLogMessage::PingResponse { id };
        id += 1;

        let mut msg = postcard::to_stdvec(&msg).unwrap();
        let crc = crc32fast::hash(&msg);
        msg.extend_from_slice(&crc.to_le_bytes());

        // debug!("crc: {:08x}", crc);

        // debug!("msg 0: {:02x?}", &msg);

        let mut msg = postcard::to_stdvec_cobs(&msg).unwrap();

        // debug!("msg 1: {:02x?}", &msg);

        write
            .send(Message::Binary(
                tokio_tungstenite::tungstenite::Bytes::from_owner(msg),
            ))
            .await
            .unwrap();

        // match read.next().await {
        //     Some(Ok(msg)) => {
        //         info!("Received a message: {:?}", msg);
        //     }
        //     Some(Err(e)) => {
        //         error!("Error receiving message: {:?}", e);
        //         break;
        //     }
        //     None => {
        //         info!("Connection closed");
        //         break;
        //     }
        // }

        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}
