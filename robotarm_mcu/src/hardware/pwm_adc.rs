use defmt::{Format, debug, error, info, trace, warn};
use static_cell::StaticCell;

use core::cell::{Cell, RefCell};
use embassy_rp::{Peri, adc, interrupt, pac, pwm::Pwm};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use portable_atomic::{AtomicU32, Ordering};

// static COUNTER: AtomicU32 = AtomicU32::new(0);
static ADC: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<(adc::Adc<embassy_rp::adc::Blocking>, adc::Channel)>>,
> = Mutex::new(RefCell::new(None));

static ADC_VALUES01: embassy_sync::channel::Channel<CriticalSectionRawMutex, (u16, u16), 16> =
    embassy_sync::channel::Channel::new();
static ADC_VALUES34: embassy_sync::channel::Channel<CriticalSectionRawMutex, (u16, u16), 16> =
    embassy_sync::channel::Channel::new();

pub type PwmMutex = Mutex<CriticalSectionRawMutex, RefCell<Option<Pwm<'static>>>>;
static PWM0: PwmMutex = PwmMutex::new(RefCell::new(None));
static PWM12: PwmMutex = PwmMutex::new(RefCell::new(None));
static PWM3: PwmMutex = PwmMutex::new(RefCell::new(None));
static PWM45: PwmMutex = PwmMutex::new(RefCell::new(None));

pub fn setup_pwm_adc(
    spawner: &embassy_executor::Spawner,
    adc_p: Peri<'static, embassy_rp::peripherals::ADC>,
    adc_pin: Peri<'static, embassy_rp::peripherals::PIN_27>,
    // pwm_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE4>,
    // pwm_pin: Peri<'static, embassy_rp::peripherals::PIN_25>,
    pwm0: Pwm<'static>,
    pwm12: Pwm<'static>,
) -> (&'static PwmMutex, &'static PwmMutex) {
    //
    // TODO: Configure ADC sample rate before setting up DMA

    let adc = embassy_rp::adc::Adc::new_blocking(adc_p, Default::default());
    let adc_pin = embassy_rp::adc::Channel::new_pin(adc_pin, embassy_rp::gpio::Pull::None);
    ADC.lock(|a| a.borrow_mut().replace((adc, adc_pin)));

    // // let pwm = Pwm::new_output_b(p.PWM_SLICE4, p.PIN_25, Default::default());
    // let pwm = Pwm::new_output_b(pwm_slice, pwm_pin, Default::default());
    // PWM.lock(|p| p.borrow_mut().replace(pwm));

    PWM0.lock(|p| p.borrow_mut().replace(pwm0));
    PWM12.lock(|p| p.borrow_mut().replace(pwm12));

    // // Enable the interrupt for pwm slice 4
    // embassy_rp::pac::PWM.irq0_inte().modify(|w| w.set_ch4(true));
    // unsafe {
    //     cortex_m::peripheral::NVIC::unmask(interrupt::PWM_IRQ_WRAP_0);
    // }

    {
        // Enable ADC and Round Robin for Channels 1 and 2
        pac::ADC.cs().modify(|w| {
            w.set_en(true);
            w.set_rrobin(0b0000_0110);
        });

        // Setup ADC FIFO: Enable, assert DREQ on 1 sample, clear it
        pac::ADC.fcs().modify(|w| {
            w.set_en(true);
            w.set_dreq_en(true);
            w.set_thresh(1);
        });

        // const START_CMD: u32 =
        let dma_trigger = pac::DMA.ch(0);
        // dma_trigger
        //     .read_addr()
        //     .write_value(&START_CMD as *const _ as u32);

        // dma_trigger.ctrl_trig().write(|w| {
        //     w.set_treq_sel(pac::dma::vals::TreqSel::ADC);
        // });

        //
    }

    // spawner.spawn(processing(avg).unwrap());

    (&PWM0, &PWM12)
}

// #[embassy_executor::task]
// async fn processing(avg: &'static core::cell::Cell<u32>) {
//     let mut buffer: heapless::HistoryBuf<u16, 100> = Default::default();
//     loop {
//         let val = ADC_VALUES.receive().await;
//         buffer.write(val);
//         let sum: u32 = buffer.iter().map(|x| *x as u32).sum();
//         avg.set(sum / buffer.len() as u32);
//     }
// }

// #[interrupt]
// fn PWM_IRQ_WRAP_0() {
//     // critical_section::with(|cs| {
//     //     let mut adc = ADC.borrow(cs).borrow_mut();
//     //     let (adc, p26) = adc.as_mut().unwrap();
//     //     let val = adc.blocking_read(p26).unwrap();
//     //     ADC_VALUES.try_send(val).ok();

//     //     // Clear the interrupt, so we don't immediately re-enter this irq handler
//     //     PWM.borrow(cs)
//     //         .borrow_mut()
//     //         .as_mut()
//     //         .unwrap()
//     //         .clear_wrapped();
//     // });
//     // COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
// }
