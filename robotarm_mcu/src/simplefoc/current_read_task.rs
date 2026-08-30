use embassy_futures::yield_now;

use crate::{hardware::current_sensor::CurrentSensor, simplefoc::types::DQCurrents};

pub type CurrentChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    DQCurrents,
    2,
>;

pub type CurrentReadRx = embassy_sync::channel::Receiver<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    DQCurrents,
    2,
>;

pub type CurrentReadTx = embassy_sync::channel::Sender<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    DQCurrents,
    2,
>;

pub type ElecAngleChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    f32,
    2,
>;

pub type ElecAngleReadRx = embassy_sync::channel::Receiver<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    f32,
    2,
>;

pub type ElecAngleReadTx = embassy_sync::channel::Sender<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    f32,
    2,
>;

pub static CURRENT_CHANNEL: CurrentChannel = CurrentChannel::new();
pub static ELEC_ANGLE_CHANNEL: ElecAngleChannel = ElecAngleChannel::new();

#[embassy_executor::task]
pub async fn core1_task_current_sens(mut current_sensor: crate::hardware::acs712::ACS712) {
    // crate::comms::usb::UsbMonitor::init(&spawner, driver);
    loop {
        yield_now().await;

        if let Ok(electrical_angle) = ELEC_ANGLE_CHANNEL.receiver().try_receive() {
            if let Ok(currents) = current_sensor.get_foc_currents(electrical_angle).await {
                let _ = CURRENT_CHANNEL.sender().try_send(currents);
            }
        }
    }
}
