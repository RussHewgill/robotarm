#![no_std]
#![no_main]
#![allow(unused_imports, dead_code, unused_variables, unused_mut)]

mod hardware;
mod simplefoc;

use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio};
use embassy_time::Timer;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};

// use crate::simplefoc::SimpleFOC;

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico on board LED, connected to gpio 25"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
    // I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // let sda = p.PIN_14; // orange
    // let scl = p.PIN_15; // yellow

    // info!("set up i2c ");
    // let mut i2c =
    //     embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    // let mut encoder = as5600::asynch::As5600::new(i2c);

    // let foc = SimpleFOC::new(encoder);

    let mut c = embassy_rp::pwm::Config::default();
    let mut pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    let mut pwm1 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c.clone());
    let mut pwm2 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, c.clone());

    let mut pwm_driver = crate::hardware::pwm_driver::PWMDriver::new(pwm0, pwm1, pwm2);

    // let config = encoder.config().unwrap();
    // info!("{:?}", config);

    // Timer::after_millis(2000).await;

    // let status = encoder.magnet_status().unwrap();
    // let agc = encoder.automatic_gain_control().unwrap();
    // let mag = encoder.magnitude().unwrap();
    // let zmco = encoder.zmco().unwrap();

    // info!("{:?}", status);
    // info!("{:?}", agc);
    // info!("{:?}", mag);
    // info!("{:?}", zmco);

    info!("Hello, World!");
    warn!("test");
    loop {
        // led.set_high();
        Timer::after_millis(250).await;

        // let value = encoder.angle().unwrap();
        // info!("Angle: {}", value);
        info!("Tick 1");

        // led.set_low();
        // Timer::after_millis(250).await;
    }
}
