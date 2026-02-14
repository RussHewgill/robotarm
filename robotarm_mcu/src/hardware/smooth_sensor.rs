use defmt::{debug, error, info, trace, warn};

use crate::hardware::encoder_sensor::EncoderSensor;

pub struct SmoothSensor<SENSOR: crate::hardware::encoder_sensor::EncoderSensor> {
    pub sensor: SENSOR,
    phase_correction: u32,
    sensor_downsample: u32,
    sensor_count: u32,
}

impl<SENSOR: crate::hardware::encoder_sensor::EncoderSensor> SmoothSensor<SENSOR> {
    pub fn new(sensor: SENSOR) -> Self {
        Self {
            sensor,
            phase_correction: 0,
            sensor_downsample: 10,
            sensor_count: 0,
        }
    }
}

impl<SENSOR: crate::hardware::encoder_sensor::EncoderSensor> EncoderSensor
    for SmoothSensor<SENSOR>
{
    type Error = SENSOR::Error;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        // self.sensor.update(ts_us).await
        unimplemented!()
    }

    fn get_mechanical_angle(&self) -> f32 {
        // self.sensor.get_mechanical_angle()
        unimplemented!()
    }

    fn get_angle(&self) -> f32 {
        // self.sensor.get_angle()
        unimplemented!()
    }

    fn get_velocity(&mut self) -> f32 {
        // EncoderSen
        unimplemented!()
    }

    fn reset_position(&mut self) {
        // self.sensor.reset_position();
        unimplemented!()
    }
}
