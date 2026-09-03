use defmt::{debug, error, info};
use embassy_rp::{Peri, gpio::Output};

use crate::{
    hardware::encoder_sensor::{EncoderSensor, N_LUT},
    simplefoc::types::_2PI,
};

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

    lut: [f32; N_LUT],        // lookup table for magnet correction
    enable_calibration: bool, // flag to enable/disable calibration
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

    async fn read_raw_debug(&mut self) -> Result<u16, Self::Error> {
        // self.read_raw_angle_debug()
        //     .await
        //     .map_err(|_| MT6701Error::SPIError)?;
        let raw_angle = self.read_raw_angle().await?;

        Ok(raw_angle)
    }

    fn set_calibration_lut(&mut self, calibration: [f32; super::encoder_sensor::N_LUT]) {
        self.lut = calibration;
    }

    fn enable_calibration(&mut self, enable: bool) {
        self.enable_calibration = enable;
    }
    fn get_encoder_calibration_enabled(&self) -> bool {
        self.enable_calibration
    }
    fn get_encoder_calibration_lut(&self) -> Option<[f32; N_LUT]> {
        Some(self.lut)
    }
}

impl<SPI: embedded_hal_async::spi::SpiBus> MT6701<SPI> {
    pub fn new(mut spi: SPI, cs: Output<'static>) -> Self {
        Self {
            spi,
            cs,

            buf: [0; 4],

            // min_elapsed_time: 0.0001, // 100 microseconds
            // min_elapsed_time: 0.00005, // 50 microseconds
            min_elapsed_time: 0.005, // 5 milliseconds

            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,

            lut: [0.0; N_LUT],
            enable_calibration: false,
        }
    }

    fn calc_velocity(&mut self) -> f32 {
        let dt_us = self.angle_prev_ts.wrapping_sub(self.vel_angle_prev_ts);

        // first run
        if self.vel_angle_prev_ts == 0 {
            self.vel_angle_prev = self.angle_prev;
            self.vel_full_rotations = self.full_rotations;
            self.vel_angle_prev_ts = self.angle_prev_ts;
            return self.velocity;
        }

        if dt_us < (self.min_elapsed_time * 1e6) as u64 {
            return self.velocity;
        }

        if dt_us > 300_000 {
            self.vel_angle_prev = self.angle_prev;
            self.vel_full_rotations = self.full_rotations;
            self.vel_angle_prev_ts = self.angle_prev_ts;
            return self.velocity;
        }

        let ts = dt_us as f32 * 1e-6;
        self.velocity = ((self.full_rotations - self.vel_full_rotations) as f32 * _2PI
            + (self.angle_prev - self.vel_angle_prev))
            / ts;

        self.vel_angle_prev = self.angle_prev;
        self.vel_full_rotations = self.full_rotations;
        self.vel_angle_prev_ts = self.angle_prev_ts;
        self.velocity
    }

    #[cfg(feature = "nope")]
    fn calc_velocity(&mut self) -> f32 {
        // calculate sample time
        // let ts = (self.angle_prev_ts - self.vel_angle_prev_ts) as f32 * 1e-6;
        let ts = self.angle_prev_ts.wrapping_sub(self.vel_angle_prev_ts) as f32 * 1e-6;

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

    pub async fn read_raw_angle_debug(&mut self) -> Result<u16, MT6701Error> {
        use bitvec::prelude as bv;
        use bitvec::prelude::*;

        self.cs.set_low();
        self.spi
            .read(&mut self.buf[..4])
            .await
            .map_err(|_| MT6701Error::SPIError)?;
        self.cs.set_high();

        // Bit 0-13: 14-bit Angle Data D[13:0]
        // Bit 14-17: 4-bit Magnetic Field Status Mg[3:0]
        // Bit 18-23: 6-bit CRC Code CRC[5:0]

        let xs = [self.buf[0], self.buf[1]];
        // let angle0 = (u16::from_be_bytes(xs) >> 1) & 0x3FFF;
        let raw_angle = (u16::from_be_bytes(xs) >> 1) & 0x3FFF;

        let xs = [self.buf[1], self.buf[2]];
        let status = (u16::from_be_bytes(xs) >> 6) & 0b1111;

        // status:
        // [1:0] = 0: Normal, 1: Too Strong, 2: Too Weak, unknown
        // [2] = push
        // [3] = 0: Normal, 1: loss of track
        debug!(
            "field strength: {:02b}, push: {:01b}, track loss: {:01b}",
            status & 0b11,
            (status >> 2) & 0b1,
            (status >> 3) & 0b1
        );

        // let bits = [self.buf[0], self.buf[1], self.buf[2]];
        // let bits = bits.view_bits::<Msb0>();
        // // let bits = bv::BitSlice::<bv::Msb0, _>::from_slice(&bits).unwrap();

        // // let raw_angle: u16 = bits[0..14].view_bits::<Msb0>().load_be();
        // // let status: u8 = bits[14..18].view_bits::<Msb0>().load_be();
        // // let crc: u8 = bits[18..24].view_bits::<Msb0>().load_be();
        // let raw_angle: u16 = bits[0..14].load_be();
        // // let status: u8 = bits[14..18].load_be();
        // // let crc: u8 = bits[18..24].load_be();

        // let mut raw_angle = 0;
        // raw_angle |= (self.buf[1] as u16) >> 2;
        // raw_angle |= (self.buf[0] as u16) << 6;

        // let raw_angle2 = xs.view_bits::<Msb0>()[0..14].load_be::<u16>();

        // let raw24 = u32::from_be_bytes([0, self.buf[0], self.buf[1], self.buf[2]]);
        // let angle3 = ((raw24 >> 10) & 0x3FFF) as u16;

        // debug!("angle0:    {}", angle0);
        // debug!("Raw angle: {}", raw_angle);
        // debug!(
        //     "raw_data: {:08b}_{:08b}_{:08b}_{:08b}, angle0: {:016b}, raw_angle: {:016b}",
        //     self.buf[0], self.buf[1], self.buf[2], self.buf[3], angle0, raw_angle
        // );

        // debug!(
        //     "angle0: {:016b}, raw_angle: {:016b}, raw_angle2: {:016b}, angle3: {:016b}",
        //     angle0, raw_angle, raw_angle2, angle3
        // );

        // let angle0 = (angle0 as f32 / 16384_f32) * _2PI;
        // let angle1 = raw_angle as f32 / 16384_f32 * _2PI;
        // let angle2 = raw_angle2 as f32 / 16384_f32 * _2PI;
        // let angle3 = angle3 as f32 / 16384_f32 * _2PI;

        // debug!(
        //     "\nangle0: {}\nangle1: {}\nangle2: {}\nangle3: {}",
        //     angle0, angle1, angle2, angle3
        // );

        // let raw_angle: u16 = bits[0..14].view_bits::<Msb0>();
        // let status: u8 = bits[14..18].view_bits::<Msb0>();
        // let crc: u8 = bits[18..24].view_bits::<Msb0>();

        // let xs = [self.buf[0], self.buf[1], self.buf[2]];
        // let raw_angle = (u16::from_be_bytes(xs) >> 1) & 0x3FFF;

        // let status = (u16::from_be_bytes(xs) >> 14) & 0b1111;

        // status[1:0] = 0: Normal, 1: Too Strong, 2: Too Weak
        // status[2] = unused
        // status[3] = 0: Normal, 1: loss of track

        // // debug!("Status: {:04b}", status);
        // match status & 0b11 {
        //     0 => debug!("Magnetic field: Normal"),
        //     1 => debug!("Magnetic field: Too Strong"),
        //     2 => debug!("Magnetic field: Too Weak"),
        //     _ => debug!("Magnetic field: Unknown: {:04b}", status),
        // }

        // if (status & 0b1000) != 0 {
        //     debug!("Loss of track detected!");
        // }

        // let crc = ((u16::from_be_bytes(xs) >> 18) & 0x3F) as u8;

        // CRC Data Range: D[13:0] and Mg[3:0] total 18-bit, D[13] is the MSB, Mg[0] is the LSB
        // CRC polynomial: X6+X+1, MSB steam in first.

        // check CRC

        // if crc as u32 != self.calc_crc([]) {
        //     // error!(
        //     //     "CRC mismatch: data: {:04X} calculated = {:06b}, received = {:06b}",
        //     //     u16::from_be_bytes(xs),
        //     //     self.calc_crc(raw_angle, status as u16),
        //     //     crc
        //     // );
        // }

        Ok(raw_angle)
    }

    fn calc_crc(&self, data: [u8; 3]) -> u32 {
        let mut crc: u32 = 0;

        crc
    }

    // pub async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
    //     self.read_raw_angle_debug().await
    // }

    // #[cfg(feature = "nope")]
    pub async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
        self.cs.set_low();
        self.spi
            .read(&mut self.buf[..2])
            .await
            .map_err(|_| MT6701Error::SPIError)?;
        self.cs.set_high();

        let xs = [self.buf[0], self.buf[1]];
        let angle = (u16::from_be_bytes(xs) >> 1) & 0x3FFF;

        // use bitvec::prelude as bv;
        // use bitvec::prelude::*;
        // let bits = [self.buf[0], self.buf[1], self.buf[2]];
        // let bits = bits.view_bits::<Msb0>();
        // let angle: u16 = bits[0..14].load_be();

        // debug!("Angle: {}", angle);

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
        // let angle = (raw_angle as f32 / 16384_f32) * _2PI;

        // let angle = if self.enable_calibration {
        //     let raw_angle = crate::simplefoc::encoder_calibration::apply_lut(raw_angle, &self.lut);
        //     (raw_angle as f32 / 16384_f32) * _2PI
        // } else {
        //     (raw_angle as f32 / 16384_f32) * _2PI
        // };

        // let angle = (raw_angle as f32 / 16384_f32) * _2PI;
        // let angle = if self.enable_calibration {
        //     let index = raw_angle >> 7;
        //     angle - self.lut[index as usize]
        // } else {
        //     angle
        // };

        // let raw_angle = (raw_angle as f32 / 16384_f32) * _2PI;
        // let angle = if self.enable_calibration {
        //     let lut_resolution = _2PI / N_LUT as f32;
        //     let lut_index = (raw_angle / lut_resolution) as usize;

        //     let y0 = self.lut[lut_index];
        //     let y1 = self.lut[(lut_index + 1) % N_LUT];

        //     // Linearly interpolate between the y0 and y1 values
        //     // Calculate the relative distance from the y0 (raw_angle has to be between y0 and y1)
        //     // If distance = 0, interpolated offset = y0
        //     // If distance = 1, interpolated offset = y1
        //     // let distance = (raw_angle - lut_index as f32 * lut_resolution) / lut_resolution;
        //     let base_angle = lut_index as f32 * lut_resolution;
        //     let distance = (raw_angle - base_angle) / lut_resolution;
        //     let offset = (1. - distance) * y0 + distance * y1;

        //     raw_angle - offset
        // } else {
        //     raw_angle
        // };

        let raw_angle = (raw_angle as f32 / 16384_f32) * _2PI;

        let angle = if self.enable_calibration {
            crate::simplefoc::encoder_calibration::apply_calibration_lut(raw_angle, &self.lut)
        } else {
            raw_angle
        };

        // let angle = raw_angle;

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
