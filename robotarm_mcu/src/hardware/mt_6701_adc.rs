use defmt::debug;

use embassy_rp::{
    Peri,
    adc::{Adc, Channel, Config, InterruptHandler},
};

use crate::{hardware::encoder_sensor::EncoderSensor, simplefoc::types::_2PI};

pub struct MT6701<'a, DMA: embassy_rp::dma::Channel> {
    adc: Adc<'a, embassy_rp::adc::Async>,
    dma: Peri<'a, DMA>,
    pin: Channel<'a>,

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
    ADCReadError,
    // Unknown,
}

impl<DMA: embassy_rp::dma::Channel> EncoderSensor for MT6701<'_, DMA> {
    type Error = MT6701Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        self.update(ts_us)
            .await
            .map_err(|_| MT6701Error::ADCReadError)
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

impl<'a, DMA> MT6701<'a, DMA>
where
    DMA: embassy_rp::dma::Channel,
{
    pub async fn new(
        adc: Adc<'a, embassy_rp::adc::Async>,
        dma: Peri<'a, DMA>,
        pin: Channel<'a>,
    ) -> Self {
        let mut out = Self {
            adc,
            dma,
            pin,

            // min_elapsed_time: 0.0001, // 100 microseconds
            min_elapsed_time: 0.001, // 1 ms

            // angle: 0.0,
            velocity: 0.0,

            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        };

        // out.test().await.unwrap();

        out
    }

    pub async fn sample_raw(&mut self) -> Result<u16, MT6701Error> {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};

        let mut buf = [0_u16; 1];
        // let div = 49; // 1MHz sample rate (48Mhz / 1MHz - 1)
        // let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        let div = 47999; // 1kHz sample rate (48Mhz / 1kHz - 1)

        self.adc
            .read_many(&mut self.pin, &mut buf, div, self.dma.reborrow())
            .await
            .map_err(|_| MT6701Error::ADCReadError)?;

        Ok(buf[0])
    }

    pub async fn sample(&mut self) -> Result<f32, MT6701Error> {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};

        let mut buf = [0_u16; 4];
        let div = 49; // 1MHz sample rate (48Mhz / 1MHz - 1)
        // let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        // let div = 47999; // 1kHz sample rate (48Mhz / 1kHz - 1)

        self.adc
            .read_many(&mut self.pin, &mut buf, div, self.dma.reborrow())
            .await
            .map_err(|_| MT6701Error::ADCReadError)?;

        // let avg = buf.iter().map(|&x| x as u32).sum::<u32>() as f32 / (buf.len() as f32);
        let avg = buf[0] as f32;

        let out = (avg - 8.0) / 4095.0;

        // let out = buf[0] as f32;

        // debug!("Raw ADC value: {}", out);

        Ok(out)
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

    pub async fn update(&mut self, ts_us: u64) -> Result<(), MT6701Error> {
        let raw_angle = self.sample().await?;
        // debug!("Raw angle: {}", raw_angle);
        // let angle = (raw_angle as f32 / 16384_f32) * _2PI;
        // debug!("Angle: {}", angle);

        let angle = raw_angle * _2PI;

        let move_angle = angle - self.angle_prev;

        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            if move_angle > 0.0 {
                self.full_rotations -= 1;
            } else {
                self.full_rotations += 1;
            }
        }

        // = (self.full_rotations as f32 * _2PI + angle) as f64;

        self.calc_velocity();

        self.angle_prev = angle;
        self.angle_prev_ts = ts_us;

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
