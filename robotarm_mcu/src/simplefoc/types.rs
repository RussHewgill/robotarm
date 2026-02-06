#[derive(defmt::Format)]
pub struct DQVoltages {
    pub d: f32,
    pub q: f32,
}

#[derive(defmt::Format)]
pub struct DQCurrents {
    pub d: f32,
    pub q: f32,
}

pub enum FOCMotorStatus {
    Uninitialized = 0x00, // Motor is not yet initialized
    Initializing = 0x01,  // Motor intiialization is in progress
    Uncalibrated = 0x02,  // Motor is initialized, but not calibrated (open loop possible)
    Calibrating = 0x03,   // Motor calibration in progress
    Ready = 0x04,         // Motor is initialized and calibrated (closed loop possible)
    Error = 0x08, // Motor is in error state (recoverable, e.g. overcurrent protection active)
    CalibFailed = 0x0E, // Motor calibration failed (possibly recoverable)
    InitFailed = 0x0F, // Motor initialization failed (not recoverable)
}
