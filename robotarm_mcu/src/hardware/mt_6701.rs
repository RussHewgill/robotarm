// use embedded_hal::delay::DelayNs;
// use embedded_hal::i2c::I2c as BlockingI2c;

use defmt::{debug, error, info};

use crate::{hardware::encoder_sensor::EncoderSensor, simplefoc::types::_2PI};

#[derive(defmt::Format)]
pub struct MT6701<I2C> {
    i2c: I2C,
    address: u8,

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
    I2CWriteError,
    I2CReadError,
    // Unknown,
}

impl<I2C: embedded_hal_async::i2c::I2c> EncoderSensor for MT6701<I2C> {
    type Error = MT6701Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        self._update(ts_us)
            .await
            .map_err(|_| MT6701Error::I2CReadError)
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

impl<I2C: embedded_hal_async::i2c::I2c> MT6701<I2C> {
    pub async fn new(i2c: I2C) -> Self {
        let address = 0x06; // default I2C address for MT6701

        let mut out = Self {
            i2c,
            address,

            // min_elapsed_time: 0.00001, // 10 microseconds
            min_elapsed_time: 0.0001, // 100 microseconds
            // min_elapsed_time: 0.005, // 1 millisecond

            // angle: 0.0,
            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        };

        // TODO: init

        out
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
