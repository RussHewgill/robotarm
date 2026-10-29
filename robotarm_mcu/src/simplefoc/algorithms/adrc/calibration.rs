use defmt::debug;

use crate::{
    hardware::{current_sensor::CurrentSensor, encoder_sensor::EncoderSensor},
    simplefoc::foc_types::SimpleFOC,
};

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    /// apply a known step in u and measure the slope of x2 over the next few samples to estimate the rotor inertia
    pub async fn find_rotor_inertia(&mut self) {
        const N_SAMPLES: usize = 2;

        // for n in 0..N_SAMPLES {
        // }

        unimplemented!()
    }
}
