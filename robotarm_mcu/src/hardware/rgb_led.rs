use defmt::{debug, error, info};
use embassy_rp::{Peri, pio::PioPin};

use crate::hardware::rgb_led;

pub enum LEDState {
    Off,
    Nominal,
    Error,
}

pub struct StatusLEDs {
    /// pin 25
    status_led: embassy_rp::gpio::Output<'static>,
    // // pin 22
    // status_led2: embassy_rp::gpio::Output<'static>,
    rgb_led: embassy_rp::pio_programs::ws2812::PioWs2812<
        'static,
        embassy_rp::peripherals::PIO0,
        0,
        1,
        embassy_rp::pio_programs::ws2812::Rgb,
    >,
}

impl StatusLEDs {
    pub fn new(
        status_led: embassy_rp::gpio::Output<'static>,
        mut common: embassy_rp::pio::Common<'static, embassy_rp::peripherals::PIO0>,
        sm0: embassy_rp::pio::StateMachine<'static, embassy_rp::peripherals::PIO0, 0>,
        dma: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
        led_pin: Peri<'static, embassy_rp::peripherals::PIN_22>,
    ) -> Self {
        let program = embassy_rp::pio_programs::ws2812::PioWs2812Program::new(&mut common);
        let mut rgb_led = embassy_rp::pio_programs::ws2812::PioWs2812::with_color_order(
            &mut common,
            sm0,
            // p.DMA_CH0,
            dma,
            crate::Irqs,
            // p.PIN_16,
            led_pin,
            &program,
        );

        Self {
            status_led,
            rgb_led,
        }
    }

    pub async fn set_color(&mut self, color: smart_leds::RGB8) {
        self.rgb_led.write(&[color]).await;
    }
}
