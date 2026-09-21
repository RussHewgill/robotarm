// pub mod acs712;
pub mod ads1256;
pub mod as5048a;
pub mod as5600;
pub mod current_sensor;
pub mod encoder_sensor;
// pub mod ina226;
pub mod ina240;
pub mod mt_6701;
// pub mod mt_6701_adc;
pub mod mt6816;
pub mod mt_6701_ssi;
pub mod pio_ssi;
pub mod smooth_sensor;

// type SpiBus<T: embassy_rp::spi::Instance> = embassy_sync::mutex::Mutex<
// pub type SpiBus<T> = embassy_sync::mutex::Mutex<
//     embassy_sync::blocking_mutex::raw::NoopRawMutex,
//     embassy_rp::spi::Spi<'static, T, embassy_rp::spi::Async>,
// >;
