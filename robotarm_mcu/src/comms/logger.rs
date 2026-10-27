use defmt::{debug, error, info};

use embassy_sync::channel::TryReceiveError;

use crate::{
    MOTOR_ID_A,
    comms::{CMD_CHAN0, CMD_CHAN1, CommsMutex, LOG_CHAN},
};

/// owned by FOC loop task
#[derive(Clone)]
pub struct CommsLogger {
    /// MCU recieves command from USB task
    rx0: embassy_sync::channel::Receiver<'static, CommsMutex, robotarm_protocol::SerialCommand, 5>,
    // rx1: embassy_sync::channel::Receiver<'static, UsbMutex, robotarm_protocol::SerialCommand, 5>,
    /// MCU sends log to USB task
    tx: embassy_sync::channel::Sender<'static, CommsMutex, robotarm_protocol::SerialLogMessage, 5>,
}

impl CommsLogger {
    pub fn new() -> Self {
        // let mut queue = heapless::Vec::new();
        // for _ in 0..4 {
        //     queue.push(heapless::Deque::new()).unwrap();
        // }
        let rx0 = CMD_CHAN0.receiver();
        // let rx1 = CMD_CHAN1.receiver();
        let tx = LOG_CHAN.sender();
        // Self { rx0, rx1, tx }
        Self { rx0, tx }
    }

    pub fn recv(&mut self, id: u8) -> Result<robotarm_protocol::SerialCommand, TryReceiveError> {
        // match id {
        //     MOTOR_ID_A => self.rx0.try_receive(),
        //     MOTOR_ID_B => self.rx1.try_receive(),
        //     _ => Err(TryReceiveError::Empty),
        // }
        self.rx0.try_receive()
    }

    pub fn send_log_msg(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        if let Err(e) = self.tx.try_send(msg) {
            // error!("Failed to send log message to USB task");
        }
    }
}
