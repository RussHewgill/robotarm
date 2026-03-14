use defmt::{Format, debug, error, info, trace, warn};
use embassy_rp::{
    Peri, PeripheralType,
    adc::{Adc, Channel},
};

use crate::hardware::current_sensor::CurrentSensor;

const BLOCK_SIZE: usize = 100;

pub struct INA240<CHANNEL: embassy_rp::dma::Channel + 'static> {
    bus_voltage: f32,

    buffer: [u8; BLOCK_SIZE],

    pin0: Channel<'static>,
    pin1: Channel<'static>,

    adc: Adc<'static, embassy_rp::adc::Async>,
    dma: Peri<'static, CHANNEL>,

    prev_phase_currents: Option<crate::simplefoc::types::PhaseCurrents>,
    prev_foc_currents: Option<crate::simplefoc::types::DQCurrents>,
}

impl<CHANNEL: embassy_rp::dma::Channel + 'static> INA240<CHANNEL> {
    pub fn new(
        pin0: Channel<'static>,
        pin1: Channel<'static>,
        // pin_1: Channel<'static>,
        adc: Adc<'static, embassy_rp::adc::Async>,
        dma: Peri<'static, CHANNEL>,
    ) -> Self {
        Self {
            bus_voltage: 0.0,
            buffer: [0; BLOCK_SIZE],

            pin0,
            pin1,

            adc,
            dma,

            prev_phase_currents: None,
            prev_foc_currents: None,
        }
    }

    pub async fn read_voltage(&mut self) -> f32 {
        let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)

        self.adc
            .read_many(&mut self.pin0, &mut self.buffer, div, self.dma.reborrow())
            .await
            .unwrap();

        unimplemented!()
    }
}

#[cfg(feature = "nope")]
impl<DMA> CurrentSensor for INA240<DMA> {
    type Error = ();

    async fn driver_align(
        &mut self,
        _voltage: f32,
        _modulation_centered: bool,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn init(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn prev_phase_currents(&self) -> Option<crate::simplefoc::types::PhaseCurrents> {
        self.prev_phase_currents
    }

    fn prev_foc_currents(&self) -> Option<crate::simplefoc::types::DQCurrents> {
        self.prev_foc_currents
    }

    fn set_prev_foc_currents(&mut self, currents: crate::simplefoc::types::DQCurrents) {
        self.prev_foc_currents = Some(currents);
    }

    async fn get_phase_currents(
        &mut self,
    ) -> Result<crate::simplefoc::types::PhaseCurrents, Self::Error> {
        let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)

        // adc.read_many(&mut pin, &mut buf, div, dma.reborrow()).await.unwrap();

        unimplemented!()
    }
}
