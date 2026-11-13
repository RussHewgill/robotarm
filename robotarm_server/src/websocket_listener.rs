use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct WebsocketListener {
    pub tx_command: UnboundedReceiver<SerialCommand>,
    pub rx_msg: UnboundedSender<SerialLogMessage>,
}
