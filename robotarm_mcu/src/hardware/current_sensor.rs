use defmt::{debug, error, info, trace, warn};

pub trait CurrentSensor {
    type Error: core::fmt::Debug;

    // async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error>;
}

pub struct NoCurrentSensor;

impl CurrentSensor for NoCurrentSensor {
    type Error = ();

    // async fn update(&mut self, _ts_us: u64) -> Result<(), Self::Error> {
    //     Ok(())
    // }
}
