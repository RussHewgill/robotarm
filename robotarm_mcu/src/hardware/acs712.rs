use defmt::{Format, debug, error, info, trace, warn};
use embassy_rp::{
    Peri, PeripheralType,
    adc::{Adc, Channel},
};

use crate::hardware::current_sensor::CurrentSensor;

// const BLOCK_SIZE: usize = 64;
const BLOCK_SIZE: usize = 128;

pub struct ACS712<CHANNEL: embassy_rp::dma::Channel + 'static> {
    bus_voltage: f32,

    buffer: [u16; BLOCK_SIZE],

    // pin0: Channel<'static>,
    // pin1: Channel<'static>,
    pins: [Channel<'static>; 2],

    adc: Adc<'static, embassy_rp::adc::Async>,
    dma: Peri<'static, CHANNEL>,

    prev_phase_currents: Option<crate::simplefoc::types::PhaseCurrents>,
    prev_foc_currents: Option<crate::simplefoc::types::DQCurrents>,
}

impl<CHANNEL: embassy_rp::dma::Channel + 'static> ACS712<CHANNEL> {
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

            pins: [pin0, pin1],

            adc,
            dma,

            prev_phase_currents: None,
            prev_foc_currents: None,
        }
    }

    pub async fn read_voltage(&mut self) -> (f32, f32) {
        // max current with 3.3V = 4.32 A
        let sensitivity = 185.; // mV/A

        let vref = 3200.;
        // let offset = vref / 2.;
        let offset0 = vref / 2. - 55.;
        let offset1 = vref / 2. - 60.;

        let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        // let div = 95; // 500kHz sample rate (48Mhz / 500kHz - 1)

        self.adc
            .read_many_multichannel(&mut self.pins, &mut self.buffer, div, self.dma.reborrow())
            .await
            .unwrap();

        // get average of each channel
        let (sum0, sum1) = self
            .buffer
            .chunks_exact(2)
            .fold((0u32, 0u32), |(sum0, sum1), chunk| {
                (sum0 + chunk[0] as u32, sum1 + chunk[1] as u32)
            });

        let avg0 = sum0 as f32 / (BLOCK_SIZE as f32 / 2.0);
        let avg1 = sum1 as f32 / (BLOCK_SIZE as f32 / 2.0);

        // raw value is 0-4095

        let voltage0 = avg0 * vref / 4095.;
        let voltage1 = avg1 * vref / 4095.;

        let current0 = (voltage0 - offset0) / sensitivity;
        let current1 = (voltage1 - offset1) / sensitivity;

        (current0, current1)
    }
}

impl<CHANNEL: embassy_rp::dma::Channel + 'static> CurrentSensor for ACS712<CHANNEL> {
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
        // debug!("Reading currents from INA240");
        let (a, b) = self.read_voltage().await;

        let currents = crate::simplefoc::types::PhaseCurrents::Two { a, b };

        self.prev_phase_currents = Some(currents);

        Ok(currents)
    }
}
