use defmt::{Format, debug, error, info, trace, warn};
use embassy_rp::{
    Peri, PeripheralType,
    adc::{Adc, Channel},
};

use crate::hardware::current_sensor::CurrentSensor;

// const BLOCK_SIZE: usize = 64;
const BLOCK_SIZE: usize = 128;

pub struct INA240<CHANNEL: embassy_rp::dma::Channel + 'static> {
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

            pins: [pin0, pin1],

            adc,
            dma,

            prev_phase_currents: None,
            prev_foc_currents: None,
        }
    }

    pub async fn read_voltage(&mut self) -> (f32, f32) {
        let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        // let div = 95; // 500kHz sample rate (48Mhz / 500kHz - 1)

        // read interleaved samples
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

        // bidirectional current sensing with INA240, zero point is at VREF / 2
        // only positive currents are expected, aside from some constant error since the VREF is slightly less than 3.3V
        // convert to voltage
        let vref = 3.3;
        let adc_max = 4095.0;

        debug!("avg0: {}", avg0);
        debug!("avg1: {}", avg1);

        let offset = -56.;

        let voltage0 = (avg0 - offset) * (vref / adc_max);
        let voltage1 = (avg1 - offset) * (vref / adc_max);

        // ina240 with 0.1 ohm shunt and 100 V/V

        let gain = 100.0;
        let shunt = 0.1;

        let c0 = -(voltage0 - vref / 2.) / (gain * shunt);
        let c1 = -(voltage1 - vref / 2.) / (gain * shunt);

        // let currents = crate::simplefoc::types::PhaseCurrents::Two {
        //     a: voltage0 as f32,
        //     b: voltage1 as f32,
        // };
        // self.prev_phase_currents = Some(currents);
        // currents

        debug!("Voltage 0: {} V", voltage0);
        debug!("Voltage 1: {} V", voltage1);

        debug!("Current 0: {} A", c0);
        debug!("Current 1: {} A", c1);

        // (voltage0, voltage1)
        (c0, c1)
    }
}

impl<CHANNEL: embassy_rp::dma::Channel + 'static> CurrentSensor for INA240<CHANNEL> {
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
