use defmt::{debug, error, info, trace, warn};

// pub const N_LUT: usize = 128;
pub const N_LUT: usize = 256;
// pub const N_LUT_SAMPLES: usize = 256;
pub const N_LUT_SAMPLES: usize = 1024;
// pub const N_LUT: usize = 4096;
// pub const N_LUT_SAMPLES: usize = 4096;

pub trait EncoderSensor {
    type Error: core::fmt::Debug;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error>;

    fn get_mechanical_angle(&self) -> f32;

    fn get_angle(&self) -> f32;

    fn get_velocity(&mut self) -> f32;

    fn reset_position(&mut self);

    async fn read_raw_debug(&mut self) -> Result<u16, Self::Error> {
        unimplemented!()
    }

    fn debug_force_set_angle_velocity(&mut self, angle: f32, velocity: f32) {
        unimplemented!()
    }

    fn set_calibration_lut(&mut self, calibration: [f32; N_LUT]) {}
    fn enable_calibration(&mut self, enable: bool) {}
    fn get_encoder_calibration_enabled(&self) -> bool {
        false
    }
    fn get_encoder_calibration_lut(&self) -> Option<[f32; N_LUT]> {
        None
    }
}

// impl EncoderSensor for () {
//     type Error = ();

//     async fn update(&mut self, _ts_us: u64) -> Result<(), Self::Error> {
//         Ok(())
//     }

//     fn get_mechanical_angle(&self) -> f32 {
//         unimplemented!()
//     }

//     fn get_angle(&self) -> f32 {
//         unimplemented!()
//     }

//     fn get_velocity(&mut self) -> f32 {
//         unimplemented!()
//     }

//     fn reset_position(&mut self) {}
// }
