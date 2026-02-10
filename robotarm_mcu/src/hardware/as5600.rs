use defmt::{debug, warn};
use embassy_executor::raw;
use embassy_time::{Instant, Timer};

use crate::simplefoc::types::_2PI;

#[derive(defmt::Format)]
pub struct AS5600<I2C> {
    // i2c: I2C,
    // address: u8,
    sensor: as5600::asynch::As5600<I2C>,
    // sensor: I2C,
    angle: f32,     // raw sensor angle in radians (0 to 2PI)
    raw_angle: u16, // raw sensor angle as read from the sensor (0 to 4095)
    position: f64,  // absolute position in radians, taking into account turns
    velocity: f32,  // velocity in radians per second

    prev_us: u64, // timestamp of previous update in microseconds
    // prev_us_vel: u64,   // timestamp of previous update in microseconds
    // angle_prev: f32,    // angle at previous update
    // position_prev: f64, // position at previous update
    // turns_prev: i64,   // turns at previous update
    angle_prev: f32,        // angle at previous update
    vel_angle_prev: f32,    // angle used for velocity calculation
    vel_angle_prev_us: u64, // timestamp of previous velocity angle update in microseconds
    turns: i64,             // number of full turns, can be negative
    turns_prev_vel: i64,    // turns at previous update, vel_full_rotations
}

#[derive(defmt::Format, Debug)]
pub enum AS5600Error {
    I2CWriteError,
    I2CReadError,
    Unknown,
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
            // i2c,
            // address,
            sensor,
            turns: 0,
            angle: 0.,
            raw_angle: 0,
            position: 0.,
            // position_prev: 0.,
            velocity: 0.,
            prev_us: 0,
            // prev_us_vel: 0,
            angle_prev: 0.,
            vel_angle_prev: 0.,
            vel_angle_prev_us: 0,
            turns_prev_vel: 0,
        };

        out.update(Instant::now().as_micros()).await.unwrap();
        Timer::after_micros(1).await;
        out.update(Instant::now().as_micros()).await.unwrap();
        Timer::after_micros(1).await;
        out.update(Instant::now().as_micros()).await.unwrap();

        out
    }

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

    pub async fn update(&mut self, ts_us: u64) -> Result<(), AS5600Error> {
        self.raw_angle = self
            .sensor
            .angle()
            .await
            .map_err(|_| AS5600Error::I2CReadError)?;

        self.angle = (self.raw_angle as f32 / 4096.) * _2PI;

        let move_angle = self.angle - self.angle_prev;

        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            if move_angle > 0.0 {
                self.turns -= 1;
            } else {
                self.turns += 1;
            }
        }

        self.position = (self.turns as f32 * _2PI + self.angle) as f64;

        self.cal_velocity(ts_us)?;

        self.angle_prev = self.angle;
        self.prev_us = ts_us;

        Ok(())
    }

    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    pub fn get_turns(&self) -> i64 {
        self.turns
    }

    pub fn get_position(&self) -> f64 {
        self.position
    }

    pub fn get_velocity(&self) -> f32 {
        self.velocity
    }

    // pub fn get_velocity(&mut self) -> f32 {
    //     let _ = self.cal_velocity(Instant::now().as_micros());
    //     self.velocity
    // }

    pub fn get_raw_angle(&self) -> u16 {
        self.raw_angle
    }
}
