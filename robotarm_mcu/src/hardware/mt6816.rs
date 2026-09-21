use defmt::{debug, error, info, warn};

use embassy_rp::{gpio::Output, spi::Instance};
use embedded_hal::spi::Operation;

use crate::{
    hardware::encoder_sensor::{EncoderSensor, N_LUT},
    simplefoc::types::_2PI,
};

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum MT6816Error {
    /// SPI bus error
    Spi,
    /// Parity mismatch
    Parity,
    /// No magnet detected
    Magnet,
}

/// MT6816 Driver
///
/// Requires MODE_3 (CPOL=1, CPHA=1) SPI to exchange data.
// #[cfg(feature = "nope")]
// pub struct Mt6816<SPI: embassy_rp::spi::Instance + 'static> {
// spi: SPI,
// spi: SpiBus<SPI>,
// pub struct Mt6816 {
pub struct Mt6816 {
    spi: embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<
        'static,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        embassy_rp::peripherals::SPI0,
        Output<'static>,
    >,
    // cs: Output<'static>,
    angles_prev: heapless::Deque<Sample, 10>,

    angle_prev: f32, // result of last call to getSensorAngle(), used for full rotations and velocity
    full_rotations: i32, // full rotation tracking

    lut: [f32; N_LUT],        // lookup table for magnet correction
    enable_calibration: bool, // flag to enable/disable calibration
}

#[derive(Copy, Clone)]
struct Sample {
    sample_angle: f32,
    sample_ts: u64,
    sample_full_rotations: i32,
}

#[cfg(feature = "nope")]
impl<SPI: Instance + 'static> EncoderSensor for Mt6816<SPI> {
    type Error = MT6816Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        self._update(ts_us).await.map_err(|_| MT6816Error::Spi)
    }

    fn get_mechanical_angle(&self) -> f32 {
        self.angle_prev
    }

    fn get_angle(&self) -> f32 {
        self.full_rotations as f32 * _2PI + self.angle_prev
    }

    fn get_velocity(&self) -> f32 {
        // self.velocity
        unimplemented!()
    }

    fn reset_position(&mut self) {
        self.full_rotations = 0;
    }

    async fn read_raw_debug(&mut self) -> Result<u16, Self::Error> {
        unimplemented!()
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

#[cfg(feature = "nope")]
impl EncoderSensor for Mt6816 {
    type Error = MT6816Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        // self._update(ts_us).await.map_err(|_| MT6816Error::Spi)
        unimplemented!()
    }

    fn get_mechanical_angle(&self) -> f32 {
        self.angle_prev
    }

    fn get_angle(&self) -> f32 {
        self.full_rotations as f32 * _2PI + self.angle_prev
    }

    fn get_velocity(&self) -> f32 {
        // self.velocity
        unimplemented!()
    }

    fn reset_position(&mut self) {
        self.full_rotations = 0;
    }

    async fn read_raw_debug(&mut self) -> Result<u16, Self::Error> {
        unimplemented!()
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

#[cfg(feature = "nope")]
impl Mt6816 {
    pub fn new(cs: Output<'static>) -> Self {
        Self {
            cs,
            angles_prev: heapless::Deque::new(),
            angle_prev: 0.0,
            full_rotations: 0,
            lut: [0.0; N_LUT],
            enable_calibration: false,
        }
    }

    pub fn read_raw_angle<SPI>(&mut self, spi: &mut SPI) -> Result<u16, MT6816Error> {
        unimplemented!()
    }
}

#[cfg(feature = "nope")]
impl<SPI: embassy_rp::spi::Instance + 'static> Mt6816<SPI> {
    pub fn new(spi: SpiBus<SPI>, cs: Output<'static>) -> Self {
        Self {
            spi,
            cs,
            angles_prev: heapless::Deque::new(),
            angle_prev: 0.0,
            full_rotations: 0,
            lut: [0.0; N_LUT],
            enable_calibration: false,
        }
    }

    pub async fn read_raw_angle(&mut self) -> Result<u16, MT6816Error> {
        unimplemented!()
    }

    pub async fn _update(&mut self, ts_us: u64) -> Result<(), MT6816Error> {
        let mut buf = [0u8; 2];

        // self.spi
        //     .transaction(&mut [Operation::Write(&[0x83]), Operation::Read(&mut buf)])
        //     .map_err(|_| MT6816Error::Spi)?;

        let raw = u16::from_be_bytes(buf);

        // Parity: bit 0, Magnet: bit 1, Angle: bits 2..=15
        let parity_bit = (raw & (1 << 0)) != 0;
        let magnet_bit = (raw & (1 << 1)) != 0;
        let raw_angle = raw >> 2;

        // Calculate even parity over bits 1..15 (angle bits + magnet bit)
        let calculated_parity = ((raw >> 1).count_ones() % 2) != 0;

        if calculated_parity != parity_bit {
            warn!(
                "Parity mismatch: calculated {}, received {}",
                calculated_parity, parity_bit
            );
            // return Err(MT6816Error::Parity);
        }

        if magnet_bit {
            warn!("Magnet bit set, indicating no magnet detected");
            // return Err(MT6816Error::Magnet);
        }

        let angle = (raw_angle as f32 / 16384_f32) * _2PI;

        let angle = if self.enable_calibration {
            crate::simplefoc::encoder_calibration::apply_calibration_lut(angle, &self.lut)
        } else {
            angle
        };

        let move_angle = angle - self.angle_prev;
        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            if move_angle > 0.0 {
                self.full_rotations -= 1;
            } else {
                self.full_rotations += 1;
            }
        }

        self.angle_prev = angle;

        Ok(())
    }
}

// impl<SPI> Mt6816<SPI>
// where
//     SPI: SpiDevice,
// {
//     /// Create a new MT6816 driver.
//     pub fn new(spi: SPI) -> Self {
//         Self { spi }
//     }

//     /// Read the angle value from the sensor.
//     /// Returns the angle in the range 0..=16383, or an error if parity or magnet issues are detected.
//     pub fn read_angle(&mut self) -> Result<u16, Error<SPI::Error>> {
//         let mut buf = [0u8; 2];

//         self.spi
//             .transaction(&mut [Operation::Write(&[0x83]), Operation::Read(&mut buf)])
//             .map_err(Error::Spi)?;

//         let raw = u16::from_be_bytes(buf);

//         // Parity: bit 0, Magnet: bit 1, Angle: bits 2..=15
//         let parity_bit = (raw & (1 << 0)) != 0;
//         let magnet_bit = (raw & (1 << 1)) != 0;
//         let angle = raw >> 2;

//         // Calculate even parity over bits 1..15 (angle bits + magnet bit)
//         let calculated_parity = ((raw >> 1).count_ones() % 2) != 0;

//         if calculated_parity != parity_bit {
//             return Err(Error::Parity);
//         }

//         if magnet_bit {
//             return Err(Error::Magnet);
//         }

//         Ok(angle)
//     }
// }
