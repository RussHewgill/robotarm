use defmt::{Format, debug, error, info, trace, warn};
use embassy_rp::{
    Peri, PeripheralType,
    adc::{Adc, Channel},
};
use embassy_time::Timer;

use crate::hardware::current_sensor::CurrentSensor;

// const BLOCK_SIZE: usize = 64;
// const BLOCK_SIZE: usize = 128;
const BLOCK_SIZE: usize = 16;

pub struct INA240 {
    buffer: [u16; BLOCK_SIZE],

    // pin0: Channel<'static>,
    // pin1: Channel<'static>,
    pins: [Channel<'static>; 2],

    adc: Adc<'static, embassy_rp::adc::Async>,
    // dma: Peri<'static, CHANNEL>,
    dma: embassy_rp::dma::Channel<'static>,

    prev_raw: (f32, f32),
    prev_phase_currents: Option<crate::simplefoc::types::PhaseCurrents>,
    prev_foc_currents: Option<crate::simplefoc::types::DQCurrents>,

    vref: f32,
    offset: (f32, f32),
}

impl INA240 {
    pub fn new(
        pin0: Channel<'static>,
        pin1: Channel<'static>,
        // pin_1: Channel<'static>,
        adc: Adc<'static, embassy_rp::adc::Async>,
        // dma: Peri<'static, CHANNEL>,
        dma: embassy_rp::dma::Channel<'static>,
    ) -> Self {
        Self {
            buffer: [0; BLOCK_SIZE],

            pins: [pin0, pin1],

            adc,
            dma,

            prev_raw: (0.0, 0.0),
            prev_phase_currents: None,
            prev_foc_currents: None,

            vref: 3.3 / 2.,
            offset: (0.0, 0.0),
        }
    }

    // doesn't work?
    #[cfg(feature = "nope")]
    pub async fn calibrate(&mut self) {
        const N: usize = 1024;

        let mut averages0 = [0f32; N];
        let mut averages1 = [0f32; N];

        for i in 0..N {
            let _ = self.read_voltage().await;
            let (v0, v1) = self.prev_raw;
            averages0[i] = v0;
            averages1[i] = v1;

            Timer::after_millis(1).await;
        }

        let sum0 = averages0.iter().sum::<f32>();
        let sum1 = averages1.iter().sum::<f32>();

        let avg0 = sum0 / (N as f32);
        let avg1 = sum1 / (N as f32);

        debug!("avg0: {}", avg0);
        debug!("avg1: {}", avg1);

        let (c0, c1) = self.read_voltage().await;
        debug!("Current 0: {} A", c0);
        debug!("Current 1: {} A", c1);

        self.offset = (avg0, avg1);

        let (c0, c1) = self.read_voltage().await;
        debug!("Current 0: {} A", c0);
        debug!("Current 1: {} A", c1);
    }

    pub async fn calibrate(&mut self) {}

    pub async fn read_voltage(&mut self) -> (f32, f32) {
        // let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        // let div = 95; // 500kHz sample rate (48Mhz / 500kHz - 1)

        // self.adc
        //     .read_many(&mut self.pins[0], &mut self.buffer, div, &mut self.dma)
        //     .await
        //     .unwrap();

        let sample0 = match self.adc.blocking_read(&mut self.pins[0]) {
            Ok(sample) => sample as f32,
            Err(e) => {
                // error!("ADC read error: {:?}", e);
                0.0
            }
        };

        let sample1 = match self.adc.blocking_read(&mut self.pins[1]) {
            Ok(sample) => sample as f32,
            Err(e) => {
                // error!("ADC read error: {:?}", e);
                0.0
            }
        };

        // let sample0 = 0.0;
        // let sample1 = 0.0;

        let current0 = self.raw_to_amps(sample0, self.offset.0);
        let current1 = self.raw_to_amps(sample1, self.offset.1);

        self.prev_raw = (sample0, sample1);
        // self.prev_raw = (current0, current1);

        // (sample0, sample1)
        // self.prev_raw
        (current0, current1)
        // (-0.5, 0.5)
    }

    fn raw_to_amps(&self, raw: f32, offset: f32) -> f32 {
        let adc_max = 4095.0;

        // let offset = -56.;

        let gain = 100.0;
        let shunt = 0.1;

        let voltage0 = (raw - offset) * (self.vref / adc_max);
        let current = -(voltage0 - self.vref / 2.) / (gain * shunt);

        current
    }

    // fn raw_to_amps(raw: f32) -> f32 {
    //     let voltage = (raw / ADC_MAX) * V_REF;
    //     let voltage_centered = voltage - (V_REF / 2.0); // Remove 1.65V offset
    //     // I = V_shunt / R_shunt -> V_shunt = V_out / Gain
    //     (voltage_centered / INA240_GAIN) / SHUNT_RESISTOR
    // }

    #[cfg(feature = "nope")]
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

impl CurrentSensor for INA240 {
    type Error = ();

    async fn driver_align(
        &mut self,
        _voltage: f32,
        _modulation_centered: bool,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.calibrate().await;
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

    fn prev_raw_currents(&self) -> (f32, f32) {
        self.prev_raw
    }

    async fn get_phase_currents(
        &mut self,
    ) -> Result<crate::simplefoc::types::PhaseCurrents, Self::Error> {
        // debug!("Reading currents from INA240");
        let (a, b) = self.read_voltage().await;

        // let currents = crate::simplefoc::types::PhaseCurrents::Two { a, b };
        let currents = crate::simplefoc::types::PhaseCurrents { a, b };

        self.prev_phase_currents = Some(currents);

        Ok(currents)
    }
}
