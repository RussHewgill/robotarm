use defmt::{debug, warn};
use embassy_executor::raw;
use embassy_time::{Instant, Timer};

use crate::{hardware::encoder_sensor::EncoderSensor, simplefoc::types::_2PI};

#[derive(defmt::Format)]
pub struct AS5600<I2C> {
    // i2c: I2C,
    // address: u8,
    sensor: as5600::asynch::As5600<I2C>,

    min_elapsed_time: f32, // minimum elapsed time between velocity updates in seconds

    angle: f32,     // raw sensor angle in radians (0 to 2PI)
    raw_angle: u16, // raw sensor angle (0 to 4095)

    velocity: f32,           // velocity in radians per second
    angle_prev: f32, // result of last call to getSensorAngle(), used for full rotations and velocity
    angle_prev_ts: u64, // timestamp of last call to getAngle, used for velocity
    vel_angle_prev: f32, // angle at last call to getVelocity, used for velocity
    vel_angle_prev_ts: u64, // last velocity calculation timestamp
    full_rotations: i32, // full rotation tracking
    vel_full_rotations: i32, // previous full rotation value for velocity calculation
}

#[derive(defmt::Format, Debug)]
pub enum AS5600Error {
    I2CWriteError,
    I2CReadError,
    Unknown,
}

impl<I2C: embedded_hal_async::i2c::I2c> EncoderSensor for AS5600<I2C> {
    type Error = AS5600Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        self._update(ts_us).await.map_err(|_| AS5600Error::Unknown)
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

impl<I2C> AS5600<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub async fn new(i2c: I2C) -> Self {
        let mut sensor = as5600::asynch::As5600::new(i2c);

        let mut cfg = sensor.config().await.unwrap();
        debug!("AS5600 initial config: {:#}", cfg);

        cfg.slow_filter = as5600::configuration::SlowFilterMode::X16;
        // cfg.slow_filter = as5600::configuration::SlowFilterMode::X2;
        cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::SlowFilterOnly;
        // cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::Lsb10;

        sensor.set_config(cfg).await.unwrap();

        let mut out = Self {
            sensor,

            min_elapsed_time: 0.0001, // 100 microseconds

            raw_angle: 0,
            angle: 0.0,
            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        };

        out.update(Instant::now().as_micros()).await.unwrap();
        Timer::after_micros(1).await;
        out.update(Instant::now().as_micros()).await.unwrap();
        Timer::after_micros(1).await;
        out.update(Instant::now().as_micros()).await.unwrap();

        out
    }

    #[cfg(feature = "nope")]
    fn cal_velocity(&mut self, ts_us: u64) -> Result<(), AS5600Error> {
        // overflow handling or first run
        if self.vel_angle_prev_us == 0 || ts_us < self.vel_angle_prev_us {
            warn!("Timestamp overflow or first run detected, resetting velocity calculation");
            self.vel_angle_prev = self.angle;
            self.vel_angle_prev_us = ts_us;
            self.turns_prev_vel = self.turns;
            return Ok(());
        }
        let d_ts = ts_us - self.vel_angle_prev_us;

        // min_elapsed_time check
        if d_ts < 5000 {
            // warn!(
            //     "Elapsed time {} is less than minimum threshold, skipping velocity calculation",
            //     d_ts
            // );
            return Ok(());
        }

        // convert to seconds
        let ts = d_ts as f32 * 1e-6;

        self.velocity = ((self.turns - self.turns_prev_vel) as f32 * _2PI
            + (self.angle - self.vel_angle_prev))
            / ts;

        self.vel_angle_prev = self.angle;
        self.turns_prev_vel = self.turns;
        self.vel_angle_prev_us = ts_us;

        Ok(())
    }

    pub async fn sample_raw(&mut self) -> Result<u16, AS5600Error> {
        let raw_angle = self
            .sensor
            .angle()
            .await
            .map_err(|_| AS5600Error::I2CReadError)?;

        Ok(raw_angle)
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

    async fn _update(&mut self, ts_us: u64) -> Result<(), AS5600Error> {
        let raw_angle = self.sample_raw().await?;
        self.angle = (raw_angle as f32 / 4096.) * _2PI;

        let move_angle = self.angle - self.angle_prev;

        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            if move_angle > 0.0 {
                self.full_rotations -= 1;
            } else {
                self.full_rotations += 1;
            }
        }

        self.angle_prev = self.angle;
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

    fn _get_velocity(&self) -> f32 {
        self.velocity
    }

    pub fn get_raw_angle(&self) -> u16 {
        self.raw_angle
    }
}
