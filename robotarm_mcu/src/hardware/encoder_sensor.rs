use defmt::{debug, error, info, trace, warn};

pub trait EncoderSensor {
    type Error: core::fmt::Debug;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error>;

    fn get_mechanical_angle(&self) -> f32;

    fn get_angle(&self) -> f32;

    fn get_velocity(&mut self) -> f32;

    fn reset_position(&mut self);
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
