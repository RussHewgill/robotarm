use crate::simplefoc::types::_2PI;

#[derive(defmt::Format)]
pub struct AS5600<I2C> {
    // i2c: I2C,
    // address: u8,
    sensor: as5600::asynch::As5600<I2C>,
    turns: i64,
    angle: f32,
    angle_prev: f32,
    position: f64,
    position_prev: f64,
    velocity: f32,
    prev_ns: u64,
}

#[derive(defmt::Format)]
pub enum AS5600Error {
    I2CWriteError,
    I2CReadError,
    // Unknown,
}

impl<I2C> AS5600<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            // i2c,
            // address,
            sensor: as5600::asynch::As5600::new(i2c),
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

    pub async fn update(&mut self, ts_us: u64) -> Result<(), AS5600Error> {
        let raw_angle = self
            .sensor
            .angle()
            .await
            .map_err(|_| AS5600Error::I2CReadError)?;

        self.angle = (raw_angle as f32 / 4096.) * _2PI;
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
}
