use defmt::{Format, debug, error, info, trace, warn};
use embassy_rp::{
    Peri, PeripheralType,
    adc::{Adc, Channel},
};
use embassy_time::Timer;

use crate::hardware::current_sensor::CurrentSensor;

// const BLOCK_SIZE: usize = 8;
const BLOCK_SIZE: usize = 16;
// const BLOCK_SIZE: usize = 64;
// const BLOCK_SIZE: usize = 128;
// const BLOCK_SIZE: usize = 256;
// const BLOCK_SIZE: usize = 1024;
// const BLOCK_SIZE: usize = 1;

// pub struct ACS712<CHANNEL: embassy_rp::dma::Channel + 'static>
pub struct ACS712 {
    bus_voltage: f32,

    buffer: [u16; BLOCK_SIZE],

    // pin0: Channel<'static>,
    // pin1: Channel<'static>,
    pins: [Channel<'static>; 2],

    adc: Adc<'static, embassy_rp::adc::Async>,
    // dma: Peri<'static, embassy_rp::dma::Channel<'static>>,
    dma: embassy_rp::dma::Channel<'static>,

    prev_phase_currents: Option<crate::simplefoc::types::PhaseCurrents>,
    prev_foc_currents: Option<crate::simplefoc::types::DQCurrents>,

    lowpass0: crate::simplefoc::lowpass::LowPassFixed,
    lowpass1: crate::simplefoc::lowpass::LowPassFixed,

    vref: f32,
    // div: u16,
    pub div: u16,
    sensitivity: f32,
    offset0: f32,
    offset1: f32,
}

// #[cfg(feature = "nope")]
impl ACS712 {
    pub fn new(
        pin0: Channel<'static>,
        pin1: Channel<'static>,
        // pin_1: Channel<'static>,
        adc: Adc<'static, embassy_rp::adc::Async>,
        // dma: Peri<'static, embassy_rp::dma::Channel>,
        dma: embassy_rp::dma::Channel<'static>,
    ) -> Self {
        let vref = 3200.; // mV

        // let div = 4799; // 10kHz
        // let div = 479; // 100kHz sample rate (48Mhz / 100kHz - 1)
        let div = 191; // 250 kHz
        // let div = 95; // 500kHz sample rate (48Mhz / 500kHz - 1)

        // max current with 3.3V = 4.32 A
        let sensitivity = 185.; // mV/A

        let lowpass_coefficient = 0.1;

        Self {
            bus_voltage: 0.0,
            buffer: [0; BLOCK_SIZE],

            pins: [pin0, pin1],

            adc,
            dma,

            prev_phase_currents: None,
            prev_foc_currents: None,

            vref,
            div,
            sensitivity,
            offset0: vref / 2.,
            offset1: vref / 2.,

            lowpass0: crate::simplefoc::lowpass::LowPassFixed::new(lowpass_coefficient),
            lowpass1: crate::simplefoc::lowpass::LowPassFixed::new(lowpass_coefficient),
        }
    }

    pub async fn calibrate(&mut self) {
        const N: usize = 16;

        let mut averages0 = [0f32; N];
        let mut averages1 = [0f32; N];

        for i in 0..N {
            // self.adc
            //     .read_many_multichannel(&mut self.pins, &mut self.buffer, self.div, &mut self.dma)
            //     .await
            //     .unwrap();

            self.adc
                .read_many(&mut self.pins[0], &mut self.buffer, self.div, &mut self.dma)
                .await
                .unwrap();

            let sum0 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg0 = sum0 as f32 / (BLOCK_SIZE as f32 / 1.0);

            self.adc
                .read_many(&mut self.pins[1], &mut self.buffer, self.div, &mut self.dma)
                .await
                .unwrap();

            let sum1 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg1 = sum1 as f32 / (BLOCK_SIZE as f32 / 1.0);

            // let (sum0, sum1) = self
            //     .buffer
            //     .chunks_exact(2)
            //     .fold((0u32, 0u32), |(sum0, sum1), chunk| {
            //         (sum0 + chunk[0] as u32, sum1 + chunk[1] as u32)
            //     });

            let v0 = avg0 * self.vref / 4095.;
            let v1 = avg1 * self.vref / 4095.;

            averages0[i] = v0;
            averages1[i] = v1;

            Timer::after_millis(10).await;
        }

        let sum0 = averages0.iter().sum::<f32>();
        let sum1 = averages1.iter().sum::<f32>();

        let avg0 = sum0 / (N as f32);
        let avg1 = sum1 / (N as f32);

        // debug!("Calibration: avg0 = {}", avg0);
        // debug!("Calibration: avg1 = {}", avg1);

        self.offset0 = avg0;
        self.offset1 = avg1;

        //
    }

    pub async fn test_noise(&mut self) -> (f32, f32) {
        let mut buf0 = [0u16; BLOCK_SIZE / 2];
        let mut buf1 = [0u16; BLOCK_SIZE / 2];

        // self.adc
        //     // .read_many_multichannel(&mut self.pins, &mut self.buffer, div, self.dma.reborrow())
        //     .read_many_multichannel(&mut self.pins, &mut self.buffer, self.div, &mut self.dma)
        //     .await
        //     .unwrap();

        self.adc
            .read_many(&mut self.pins[0], &mut buf0, self.div, &mut self.dma)
            .await
            .unwrap();
        self.adc
            .read_many(&mut self.pins[1], &mut buf1, self.div, &mut self.dma)
            .await
            .unwrap();

        let sum0 = buf0.iter().map(|&x| x as u64).sum::<u64>();
        let sum1 = buf1.iter().map(|&x| x as u64).sum::<u64>();

        let avg0 = sum0 as f32 / (BLOCK_SIZE as f32 / 2.0);
        let avg1 = sum1 as f32 / (BLOCK_SIZE as f32 / 2.0);

        // Sum the squared differences
        // let (sum_sq_diff0, sum_sq_diff1) = self.buffer.chunks_exact(2).fold(
        //     (0f32, 0f32),
        //     |(sum_sq_diff0, sum_sq_diff1), chunk| {
        //         // (sum0 + chunk[0] as u64, sum1 + chunk[1] as u64)
        //         let diff0 = chunk[0] as f32 - avg0;
        //         let diff1 = chunk[1] as f32 - avg1;
        //         (sum_sq_diff0 + diff0 * diff0, sum_sq_diff1 + diff1 * diff1)
        //     },
        // );

        let sum_sq_diff0 = buf0.iter().fold(0f32, |sum_sq_diff0, &x| {
            let diff0 = x as f32 - avg0;
            sum_sq_diff0 + diff0 * diff0
        });

        let sum_sq_diff1 = buf1.iter().fold(0f32, |sum_sq_diff1, &x| {
            let diff1 = x as f32 - avg1;
            sum_sq_diff1 + diff1 * diff1
        });

        // Calculate Variance and Standard Deviation
        let variance0 = sum_sq_diff0 / (BLOCK_SIZE as f32 / 2.0);
        let variance1 = sum_sq_diff1 / (BLOCK_SIZE as f32 / 2.0);

        let std_dev0 = libm::sqrtf(variance0);
        let std_dev1 = libm::sqrtf(variance1);

        let r = 10.;
        // debug!(
        //     "Variance0: {:03}, Variance1: {:03}",
        //     libm::roundf(variance0 * r) / r,
        //     libm::roundf(variance1 * r) / r
        // );
        // debug!(
        //     "Std Dev0: {:03}, Std Dev1: {:03}",
        //     libm::roundf(std_dev0 * r) / r,
        //     libm::roundf(std_dev1 * r) / r
        // );

        let noise_volts0 = std_dev0 * self.vref / 4095.;
        let noise_volts1 = std_dev1 * self.vref / 4095.;

        let noise_current0 = noise_volts0 / self.sensitivity;
        let noise_current1 = noise_volts1 / self.sensitivity;

        // debug!("Current noise0: {:03}", libm::roundf(std_dev0 * r) / r);
        // debug!("Current noise1: {:03}", libm::roundf(std_dev1 * r) / r);
        // debug!("Current noise0: {}", noise_current0);
        // debug!("Current noise1: {}", noise_current1);

        (noise_current0, noise_current1)
    }

    pub async fn test_noise_interleaved(&mut self) -> (f32, f32) {
        self.adc
            // .read_many_multichannel(&mut self.pins, &mut self.buffer, div, self.dma.reborrow())
            .read_many_multichannel(&mut self.pins, &mut self.buffer, self.div, &mut self.dma)
            .await
            .unwrap();

        // get average of each channel
        let (sum0, sum1) = self
            .buffer
            .chunks_exact(2)
            .fold((0u64, 0u64), |(sum0, sum1), chunk| {
                (sum0 + chunk[0] as u64, sum1 + chunk[1] as u64)
            });

        let avg0 = sum0 as f32 / (BLOCK_SIZE as f32 / 2.0);
        let avg1 = sum1 as f32 / (BLOCK_SIZE as f32 / 2.0);

        // Sum the squared differences
        let (sum_sq_diff0, sum_sq_diff1) = self.buffer.chunks_exact(2).fold(
            (0f32, 0f32),
            |(sum_sq_diff0, sum_sq_diff1), chunk| {
                // (sum0 + chunk[0] as u64, sum1 + chunk[1] as u64)

                let diff0 = chunk[0] as f32 - avg0;

                let diff1 = chunk[1] as f32 - avg1;

                (sum_sq_diff0 + diff0 * diff0, sum_sq_diff1 + diff1 * diff1)
            },
        );

        // Calculate Variance and Standard Deviation
        let variance0 = sum_sq_diff0 / (BLOCK_SIZE as f32 / 2.0);
        let variance1 = sum_sq_diff1 / (BLOCK_SIZE as f32 / 2.0);

        let std_dev0 = libm::sqrtf(variance0);
        let std_dev1 = libm::sqrtf(variance1);

        // debug!("Variance0: {:03}, Variance1: {:03}", variance0, variance1);
        // debug!("Std Dev0: {:03}, Std Dev1: {:03}", std_dev0, std_dev1);

        let noise_volts0 = std_dev0 * self.vref / 4095.;
        let noise_volts1 = std_dev1 * self.vref / 4095.;

        let noise_current0 = noise_volts0 / self.sensitivity;
        let noise_current1 = noise_volts1 / self.sensitivity;

        (noise_current0, noise_current1)
    }

    pub async fn read_current(&mut self) -> (f32, f32) {
        #[cfg(feature = "nope")]
        {
            for v in self.buffer.iter_mut() {
                *v = self.adc.read(&mut self.pins[0]).await.unwrap();
            }

            let sum0 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg0 = sum0 as f32 / (BLOCK_SIZE as f32 / 1.0);

            debug!(
                "min, max = {}, {}",
                self.buffer.iter().min().unwrap(),
                self.buffer.iter().max().unwrap()
            );
            debug!("avg0 = {}", avg0);

            let voltage0 = avg0 * vref / 4095.;
            let current0 = (voltage0 - offset0) / sensitivity;

            for v in self.buffer.iter_mut() {
                *v = self.adc.read(&mut self.pins[1]).await.unwrap();
            }

            let sum1 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg1 = sum1 as f32 / (BLOCK_SIZE as f32 / 1.0);

            let voltage1 = avg1 * vref / 4095.;
            let current1 = (voltage1 - offset1) / sensitivity;
            return (current0, current1);
        }

        // interleaved
        #[cfg(feature = "nope")]
        {
            self.adc
                // .read_many_multichannel(&mut self.pins, &mut self.buffer, div, self.dma.reborrow())
                .read_many_multichannel(&mut self.pins, &mut self.buffer, self.div, &mut self.dma)
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

            let voltage0 = avg0 * self.vref / 4095.;
            let voltage1 = avg1 * self.vref / 4095.;

            let current0 = (voltage0 - self.offset0) / self.sensitivity;
            let current1 = (voltage1 - self.offset1) / self.sensitivity;

            // let current0 = self.lowpass0.filter(current0);
            // let current1 = self.lowpass1.filter(current1);

            return (current0, current1);
        }

        // non-interleaved
        // #[cfg(feature = "nope")]
        {
            self.adc
                .read_many(&mut self.pins[0], &mut self.buffer, self.div, &mut self.dma)
                .await
                .unwrap();

            let sum0 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg0 = sum0 as f32 / BLOCK_SIZE as f32;

            let voltage0 = avg0 * self.vref / 4095.;
            let current0 = (voltage0 - self.offset0) / self.sensitivity;

            self.adc
                .read_many(&mut self.pins[1], &mut self.buffer, self.div, &mut self.dma)
                .await
                .unwrap();

            let sum1 = self.buffer.iter().map(|&x| x as u32).sum::<u32>();
            let avg1 = sum1 as f32 / BLOCK_SIZE as f32;

            let voltage1 = avg1 * self.vref / 4095.;
            let current1 = (voltage1 - self.offset1) / self.sensitivity;

            let current0 = self.lowpass0.filter(current0);
            let current1 = self.lowpass1.filter(current1);

            return (current0, current1);
        }
    }
}

impl CurrentSensor for ACS712 {
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

    async fn get_phase_currents(
        &mut self,
    ) -> Result<crate::simplefoc::types::PhaseCurrents, Self::Error> {
        // debug!("Reading currents from INA240");
        let (a, b) = self.read_current().await;

        let currents = crate::simplefoc::types::PhaseCurrents::Two { a, b };

        self.prev_phase_currents = Some(currents);

        Ok(currents)
    }
}
