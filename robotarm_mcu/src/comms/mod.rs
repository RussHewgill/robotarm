// pub mod usb;
pub mod logger;
pub mod rs485;
// pub mod usb_handler;
// pub mod usb_raw;

// pub use usb_raw as usb;

pub type CommsMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
// pub type UsbMutex = embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

pub type LogChannel =
    embassy_sync::channel::Channel<CommsMutex, robotarm_protocol::SerialLogMessage, 5>;
pub type CmdChannel =
    embassy_sync::channel::Channel<CommsMutex, robotarm_protocol::SerialCommand, 5>;
// pub type StatusChannel = embassy_sync::signal::Signal<CommsMutex, crate::hardware::led::LEDState>;

pub static LOG_CHAN: LogChannel = LogChannel::new();
pub static CMD_CHAN0: CmdChannel = CmdChannel::new();
// pub static CMD_CHAN1: CmdChannel = CmdChannel::new();
// pub static STATUS_CHAN: StatusChannel = StatusChannel::new();
