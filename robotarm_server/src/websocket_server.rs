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
        // let msg = robotarm_protocol::SerialLogMessage::PingResponse { id };
        // id += 1;

        // let mut msg = postcard::to_stdvec(&msg).unwrap();
        // let crc = crc32fast::hash(&msg);
        // msg.extend_from_slice(&crc.to_le_bytes());

        // write
        //     .send(Message::Binary(
        //         tokio_tungstenite::tungstenite::Bytes::from_owner(msg),
        //     ))
        //     .await
        //     .unwrap();

        match read.next().await {
            Some(Ok(msg)) => {
                info!("Received a message: {:?}", msg);
            }
            Some(Err(e)) => {
                error!("Error receiving message: {:?}", e);
                break;
            }
            None => {
                info!("Connection closed");
                break;
            }
        }

        // tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }

    // // We should not forward messages other than text or binary.
    // read.try_filter(|msg| future::ready(msg.is_text() || msg.is_binary()))
    //     .forward(write)
    //     .await
    //     .expect("Failed to forward messages")
}

/// example
pub async fn handle_connection(peer_map: PeerMap, raw_stream: TcpStream, addr: SocketAddr) {
    println!("Incoming TCP connection from: {}", addr);

    let ws_stream = tokio_tungstenite::accept_async(raw_stream)
        .await
        .expect("Error during the websocket handshake occurred");
    println!("WebSocket connection established: {}", addr);

    // Insert the write part of this peer to the peer map.
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    peer_map.lock().unwrap().insert(addr, tx);

    let (outgoing, incoming) = ws_stream.split();

    unimplemented!()
}

async fn _handle_connection(peer_map: PeerMap, raw_stream: TcpStream, addr: SocketAddr) {
    println!("Incoming TCP connection from: {}", addr);

    let ws_stream = tokio_tungstenite::accept_async(raw_stream)
        .await
        .expect("Error during the websocket handshake occurred");
    println!("WebSocket connection established: {}", addr);

    // Insert the write part of this peer to the peer map.
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    peer_map.lock().unwrap().insert(addr, tx);

    let (outgoing, incoming) = ws_stream.split();

    let broadcast_incoming = incoming.try_for_each(|msg| {
        println!(
            "Received a message from {}: {}",
            addr,
            msg.to_text().unwrap()
        );
        let peers = peer_map.lock().unwrap();

        // We want to broadcast the message to everyone except ourselves.
        let broadcast_recipients = peers
            .iter()
            .filter(|(peer_addr, _)| peer_addr != &&addr)
            .map(|(_, ws_sink)| ws_sink);

        for recp in broadcast_recipients {
            // recp.unbounded_send(msg.clone()).unwrap();
            unimplemented!()
        }

        future::ok(())
    });

    // let receive_from_others = rx.map(Ok).forward(outgoing);

    // pin_mut!(broadcast_incoming, receive_from_others);
    // future::select(broadcast_incoming, receive_from_others).await;

    // println!("{} disconnected", &addr);
    // peer_map.lock().unwrap().remove(&addr);

    //
}
