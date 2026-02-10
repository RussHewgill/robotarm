use embassy_time::Instant;

pub struct LowPassFilter {
    /// Time constant
    pub tf: f32,
    /// Previous output value
    y_prev: f32,
    /// Last execution timestamp in microseconds
    timestamp_prev: u64,
}

impl LowPassFilter {
    pub fn new(time_constant: f32) -> Self {
        Self {
            tf: time_constant,
            y_prev: 0.0,
            timestamp_prev: Instant::now().as_micros(),
        }
    }

    pub fn filter_with_timestamp(&mut self, x: f32, timestamp: u64) -> f32 {
        let mut dt = (timestamp.wrapping_sub(self.timestamp_prev)) as f32 * 1e-6;

        if dt < 0.0 {
            dt = 1e-3;
        } else if dt > 0.3 {
            self.y_prev = x;
            self.timestamp_prev = timestamp;
            return x;
        }

        let alpha = self.tf / (self.tf + dt);
        let y = alpha * self.y_prev + (1.0 - alpha) * x;
        self.y_prev = y;
        self.timestamp_prev = timestamp;
        y
    }

    pub fn filter(&mut self, x: f32) -> f32 {
        let timestamp = Instant::now().as_micros();
        let mut dt = (timestamp.wrapping_sub(self.timestamp_prev)) as f32 * 1e-6;

        if dt < 0.0 {
            dt = 1e-3;
        } else if dt > 0.3 {
            self.y_prev = x;
            self.timestamp_prev = timestamp;
            return x;
        }

        let alpha = self.tf / (self.tf + dt);
        let y = alpha * self.y_prev + (1.0 - alpha) * x;
        self.y_prev = y;
        self.timestamp_prev = timestamp;
        y
    }
}
