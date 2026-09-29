// pub mod acs712;
// pub mod ads1256;
// pub mod as5048a;
// pub mod as5600;
pub mod current_sensor;
pub mod encoder_sensor;
// pub mod ina226;
pub mod ina240;
pub mod mt_6701;
// pub mod mt_6701_adc;
// pub mod mcp3202;
pub mod mt6816;
pub mod mt_6701_ssi;
pub mod pio_ssi;
// pub mod smooth_sensor;
pub mod max485;
// pub mod pwm_adc;

pub type Spi0Bus = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::NoopRawMutex,
    embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
>;

pub type Spi1Bus = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::NoopRawMutex,
    embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
>;

pub static SPI_BUS0: static_cell::StaticCell<Spi0Bus> = static_cell::StaticCell::new();
// pub static SPI_BUS1: static_cell::StaticCell<Spi1Bus> = static_cell::StaticCell::new();

pub struct NoopEncoder;

impl encoder_sensor::EncoderSensor for NoopEncoder {
    type Error = ();

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        todo!()
    }

    fn get_mechanical_angle(&self) -> f32 {
        todo!()
    }

    fn get_angle(&self) -> f32 {
        todo!()
    }

    fn get_velocity(&self) -> f32 {
        todo!()
    }

    fn reset_position(&mut self) {
        todo!()
    }
}
