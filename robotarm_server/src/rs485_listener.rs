use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

pub struct Rs485Listener {
    pub rx_command: UnboundedReceiver<SerialCommand>,
    pub tx_msg: UnboundedSender<SerialLogMessage>,
}
