use defmt::{debug, error, info, trace, warn};

use crate::simplefoc::types::{_1_SQRT3, _2_SQRT3, ABCurrents, DQCurrents, PhaseCurrents};

pub trait CurrentSensor {
    type Error: core::fmt::Debug;

    async fn driver_align(
        &mut self,
        voltage: f32,
        modulation_centered: bool,
    ) -> Result<(), Self::Error>;

    async fn init(&mut self) -> Result<(), Self::Error>;

    fn prev_phase_currents(&self) -> Option<PhaseCurrents>;
    fn prev_foc_currents(&self) -> Option<DQCurrents>;
    fn set_prev_foc_currents(&mut self, currents: DQCurrents);

    async fn get_foc_currents(&mut self, electrical_angle: f32) -> Result<DQCurrents, Self::Error> {
        let currents = self.get_phase_currents().await?;
        let ab_currents = self.get_ab_currents(currents).await;
        let dq_currents = self.get_dq_currents(ab_currents, electrical_angle).await;
        self.set_prev_foc_currents(dq_currents);
        Ok(dq_currents)
    }

    async fn get_phase_currents(&mut self) -> Result<PhaseCurrents, Self::Error>;

    async fn get_ab_currents(&mut self, current: PhaseCurrents) -> ABCurrents {
        // calculate clarke transform

        let alpha;
        let beta;

        match current {
            PhaseCurrents::Three { a, b, c } => {
                unimplemented!()
            }
            PhaseCurrents::TwoAB { a, b } => {
                alpha = a;
                beta = _1_SQRT3 * a + _2_SQRT3 * b;
            }
            _ => unimplemented!(),
        }

        //         if(!current.c){
        //     // if only two measured currents
        //     i_alpha = current.a;
        //     i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
        // }else if(!current.a){
        //     // if only two measured currents
        //     float a = -current.c - current.b;
        //     i_alpha = a;
        //     i_beta = _1_SQRT3 * a + _2_SQRT3 * current.b;
        // }else if(!current.b){
        //     // if only two measured currents
        //     float b = -current.a - current.c;
        //     i_alpha = current.a;
        //     i_beta = _1_SQRT3 * current.a + _2_SQRT3 * b;
        // } else {
        //     // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
        //     float mid = (1.f/3) * (current.a + current.b + current.c);
        //     float a = current.a - mid;
        //     float b = current.b - mid;
        //     i_alpha = a;
        //     i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
        // }

        ABCurrents { alpha, beta }
    }

    async fn get_dq_currents(
        &mut self,
        ab_current: ABCurrents,
        electrical_angle: f32,
    ) -> DQCurrents {
        // calculate park transform
        let st = libm::sinf(electrical_angle);
        let ct = libm::cosf(electrical_angle);
        DQCurrents {
            d: ab_current.alpha * ct + ab_current.beta * st,
            q: ab_current.beta * ct - ab_current.alpha * st,
        }
    }

    // async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error>;
}

impl CurrentSensor for () {
    type Error = ();

    async fn init(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn prev_foc_currents(&self) -> Option<DQCurrents> {
        unimplemented!()
    }
    fn set_prev_foc_currents(&mut self, currents: DQCurrents) {
        unimplemented!()
    }
    fn prev_phase_currents(&self) -> Option<PhaseCurrents> {
        unimplemented!()
    }

    async fn driver_align(
        &mut self,
        voltage: f32,
        modulation_centered: bool,
    ) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn get_phase_currents(&mut self) -> Result<PhaseCurrents, Self::Error> {
        unimplemented!()
    }
}
