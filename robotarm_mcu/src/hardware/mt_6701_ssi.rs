use defmt::{debug, error, info};
use embassy_rp::{Peri, gpio::Output};

use crate::{hardware::encoder_sensor::EncoderSensor, simplefoc::types::_2PI};

#[derive(defmt::Format)]
pub struct MT6701<SPI> {
    spi: SPI,
    cs: Output<'static>,

    buf: [u8; 4],

    min_elapsed_time: f32, // minimum elapsed time between velocity updates in seconds

    // angle: f32,
    velocity: f32, // velocity in radians per second

    angle_prev: f32, // result of last call to getSensorAngle(), used for full rotations and velocity
    angle_prev_ts: u64, // timestamp of last call to getAngle, used for velocity
    vel_angle_prev: f32, // angle at last call to getVelocity, used for velocity
    vel_angle_prev_ts: u64, // last velocity calculation timestamp
    full_rotations: i32, // full rotation tracking
    vel_full_rotations: i32, // previous full rotation value for velocity calculation
}

#[derive(defmt::Format, Debug)]
pub enum MT6701Error {
    // I2CWriteError,
    // I2CReadError,
    SPIError,
}

impl<SPI: embedded_hal_async::spi::SpiBus> EncoderSensor for MT6701<SPI> {
    type Error = MT6701Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        self._update(ts_us).await.map_err(|_| MT6701Error::SPIError)
    }

    fn get_mechanical_angle(&self) -> f32 {
        self._get_mechanical_angle()
    }

    fn get_angle(&self) -> f32 {
        self._get_angle()
    }

    fn get_velocity(&mut self) -> f32 {
        self._get_velocity()
    }

    fn reset_position(&mut self) {
        self.full_rotations = 0;
    }
}

// impl<'d, T> Spi<'d, T, Async>
// pub fn new_rxonly(
//     inner: Peri<'d, T>,
//     clk: Peri<'d, impl ClkPin<T> + 'd>,
//     miso: Peri<'d, impl MisoPin<T> + 'd>,
//     tx_dma: Peri<'d, impl Channel>,
//     rx_dma: Peri<'d, impl Channel>,
//     config: Config

impl<SPI: embedded_hal_async::spi::SpiBus> MT6701<SPI> {
    #[cfg(feature = "nope")]
    pub fn new<T: embassy_rp::spi::Instance>(
        spi: Peri<'static, T>,
        sck: Peri<'static, impl embassy_rp::spi::ClkPin<T>>,
        miso: Peri<'static, impl embassy_rp::spi::MisoPin<T>>,
        tx_dma: Peri<'static, impl embassy_rp::dma::Channel>,
        rx_dma: Peri<'static, impl embassy_rp::dma::Channel>,
        cs: Output<'static>,
    ) -> Self {
        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;

        let mut spi = embassy_rp::spi::Spi::new_rxonly(spi, sck, miso, tx_dma, rx_dma, config);

        Self {
            spi,
            cs,

            buf: [0; 4],

            min_elapsed_time: 0.0001, // 100 microseconds

            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        }
    }

    pub fn new(mut spi: SPI, cs: Output<'static>) -> Self {
        Self {
            spi,
            cs,

            buf: [0; 4],

            min_elapsed_time: 0.0001, // 100 microseconds

            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        }
    }

    fn calc_velocity(&mut self) -> f32 {
        // calculate sample time
        let ts = (self.angle_prev_ts - self.vel_angle_prev_ts) as f32 * 1e-6;

        if ts < 0.0 {
            // handle micros() overflow - we need to reset vel_angle_prev_ts
            self.vel_angle_prev = self.angle_prev;
            self.vel_full_rotations = self.full_rotations;
            self.vel_angle_prev_ts = self.angle_prev_ts;
            return self.velocity;
        }

        if ts < self.min_elapsed_time {
            return self.velocity; // don't update velocity if deltaT is too small
        }

        self.velocity = ((self.full_rotations - self.vel_full_rotations) as f32 * _2PI
            + (self.angle_prev - self.vel_angle_prev))
            / ts;

        self.vel_angle_prev = self.angle_prev;
        self.vel_full_rotations = self.full_rotations;
        self.vel_angle_prev_ts = self.angle_prev_ts;

        self.velocity
    }

    pub async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
        self.cs.set_low();
        self.spi
            .read(&mut self.buf[..2])
            .await
            .map_err(|_| MT6701Error::SPIError)?;
        self.cs.set_high();

        let xs = [self.buf[0], self.buf[1]];
        let angle = (u16::from_be_bytes(xs) >> 1) & 0x3FFF;

        Ok(angle)
    }

    #[cfg(feature = "nope")]
    pub async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
        use embedded_hal_async::i2c::I2c;

        let mut buf: [u8; 2] = [0; 2];

        if let Err(e) = self
            .i2c
            // .write_read(self.address, &[0x03], &mut buf[..1])
            .write_read(self.address, &[0x03], &mut buf)
            .await
        {
            // error!("I2C Error: {:?}", e);
            // error!("I2C Error");
            return Err(MT6701Error::I2CReadError);
        }

        Ok(((buf[0] as u16) << 6) | (buf[1] as u16 & 0b00111111))
    }

    pub async fn _update(&mut self, ts_us: u64) -> Result<(), MT6701Error> {
        let raw_angle = self.read_raw_angle().await?;
        // debug!("Raw angle: {}", raw_angle);
        let angle = (raw_angle as f32 / 16384_f32) * _2PI;
        // debug!("Angle: {}", angle);

        let move_angle = angle - self.angle_prev;

        // handle full rotations - if the angle jumps more than 0.8 * 2PI, we assume it wrapped around
        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            if move_angle > 0.0 {
                self.full_rotations -= 1;
            } else {
                self.full_rotations += 1;
            }
        }

        self.angle_prev = angle;
        self.angle_prev_ts = ts_us;

        self.calc_velocity();

        Ok(())
    }

    fn _get_mechanical_angle(&self) -> f32 {
        self.angle_prev
    }

    fn _get_angle(&self) -> f32 {
        self.full_rotations as f32 * _2PI + self.angle_prev
    }

    // pub fn get_position(&self) -> f64 {
    //     unimplemented!()
    // }

    fn _get_velocity(&mut self) -> f32 {
        self.velocity
    }
}
