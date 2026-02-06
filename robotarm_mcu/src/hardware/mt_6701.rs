// use embedded_hal::delay::DelayNs;
// use embedded_hal::i2c::I2c as BlockingI2c;

use crate::simplefoc::types::_2PI;

#[derive(defmt::Format)]
pub struct MT6701<I2C> {
    i2c: I2C,
    address: u8,
    turns: i64,
    angle: f32,
    angle_prev: f32,
    position: f64,
    position_prev: f64,
    velocity: f32,
    prev_ns: u64,
}

#[derive(defmt::Format)]
pub enum MT6701Error {
    I2CWriteError,
    I2CReadError,
    // Unknown,
}

// impl<I2C> MT6701<I2C>
// where
//     I2C: embedded_hal_async::i2c::I2c,
// {
//     pub fn new(i2c: I2C, address: u8) -> Self {
//         Self { i2c, address }
//     }

//     /// get angle in radians, range [0, 2PI)
//     pub async fn get_angle(&mut self) -> Result<f32, MT6701Error> {
//         let raw_angle = self.read_raw_angle().await?;

//         Ok((raw_angle as f32 / 16384_f32) * _2PI)
//     }

//     async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
//         let mut buffer: [u8; 2] = [0; 2];

//         self.i2c
//             .write(self.address, &[0x03])
//             .await
//             .map_err(|_| MT6701Error::I2CWriteError)?;

//         self.i2c
//             .read(self.address, &mut buffer[..1])
//             .await
//             .map_err(|_| MT6701Error::I2CReadError)?;

//         // if let Err(e) = self.i2c.write(self.address, &[0x04]) {
//         //     defmt::error!("Failed to write to MT6701");
//         //     return 0;
//         // };
//         // if let Err(e) = self.i2c.read(self.address, &mut buffer[1..]) {
//         //     defmt::error!("Failed to read from MT6701");
//         //     return 0;
//         // };

//         self.i2c
//             .write(self.address, &[0x04])
//             .await
//             .map_err(|_| MT6701Error::I2CWriteError)?;

//         self.i2c
//             .read(self.address, &mut buffer[1..])
//             .await
//             .map_err(|_| MT6701Error::I2CReadError)?;

//         defmt::warn!("TODO: check if this is correct");
//         // Ok((buffer[0] >> 1) & 0x3FFF)
//         Ok((u16::from_be_bytes(buffer) >> 1) & 0x3FFF)
//     }
// }

impl<I2C> MT6701<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            turns: 0,
            angle: 0.,
            angle_prev: 0.,
            position: 0.,
            position_prev: 0.,
            velocity: 0.,
            prev_ns: 0,
        }
    }

    fn cal_velocity(&mut self, ts_us: u64) {
        if ts_us == 0 {
            self.velocity = 0.0;
            return;
        }

        let mut ts = (ts_us - self.prev_ns) as f32 * 1e-6;
        if ts < 0.0 {
            ts = 1e-3;
        }

        self.velocity = (self.position - self.position_prev) as f32 / ts;

        self.position_prev = self.position;
    }

    async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c
            .write(self.address, &[0x03])
            .await
            .map_err(|_| MT6701Error::I2CWriteError)?;

        self.i2c
            .read(self.address, &mut buffer[..1])
            .await
            .map_err(|_| MT6701Error::I2CReadError)?;

        self.i2c
            .write(self.address, &[0x04])
            .await
            .map_err(|_| MT6701Error::I2CWriteError)?;

        self.i2c
            .read(self.address, &mut buffer[1..])
            .await
            .map_err(|_| MT6701Error::I2CReadError)?;

        defmt::warn!("TODO: check if this is correct");
        // Ok((buffer[0] >> 1) & 0x3FFF)
        Ok((u16::from_be_bytes(buffer) >> 1) & 0x3FFF)
    }

    pub async fn update(&mut self, ts_us: u64) -> Result<(), MT6701Error> {
        let raw_angle = self.read_raw_angle().await?;

        self.angle = (raw_angle as f32 / 16384_f32) * _2PI;
        let move_angle = self.angle - self.angle_prev;

        if libm::fabsf(move_angle) > (0.8 * _2PI) {
            self.turns += if move_angle > 0.0 { -1 } else { 1 };
        }

        self.position = (self.turns as f32 * _2PI + self.angle) as f64;

        self.cal_velocity(ts_us);

        self.angle_prev = self.angle;
        self.prev_ns = ts_us;

        Ok(())
    }

    pub fn get_angle(&mut self) -> f32 {
        self.angle
    }

    pub fn get_turns(&mut self) -> i64 {
        self.turns
    }

    pub fn get_position(&mut self) -> f64 {
        self.position
    }

    pub fn get_velocity(&mut self) -> f32 {
        self.velocity
    }
}
