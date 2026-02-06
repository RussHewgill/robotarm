#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_mut)]

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

    let sda = p.PIN_14; // orange
    let scl = p.PIN_15; // yellow

    info!("set up i2c ");
    let i2c =
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    // let mut encoder = as5600::asynch::As5600::new(i2c);
    let encoder = crate::hardware::mt_6701::MT6701::new(i2c, 0x06);

    let c = embassy_rp::pwm::Config::default();
    let pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    // let mut pwm1 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c.clone());
    // let mut pwm2 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, c.clone());
    let pwm12 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, c.clone());
    let pwm_driver = crate::simplefoc::pwm_driver::PWMDriver::new(pwm0, pwm12, 2., 12.);

    let motor_config = crate::simplefoc::bldc::BLDCMotor::new(
        7,    // pole pairs
        11.2, // phase resistance (TODO: measure this)
        90.,  // motor kv
        // Some(0.00035), // phase inductance
        None,
    );

    let mut foc = crate::simplefoc::foc::SimpleFOC::new(encoder, pwm_driver, motor_config);

    foc.sensor_direction = crate::simplefoc::types::SensorDirection::Normal;

    info!("Starting init");

    foc.init();

    info!("Starting FOC init");

    foc.init_foc().await;

    info!("Starting main loop");
    loop {
        // // led.set_high();
        // Timer::after_millis(250).await;

        foc.update_foc().await;
    }
}
