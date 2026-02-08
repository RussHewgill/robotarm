#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unexpected_cfgs)]

mod hardware;
mod simplefoc;

use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio};
use embassy_time::{Instant, Ticker, Timer};
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

// #[embassy_executor::main]
#[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14; // purple
    let scl = p.PIN_15; // blue

    let i2c =
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    let mut encoder = crate::hardware::as5600::AS5600::new(i2c);

    info!("looping");
    loop {
        // info!("Tick");
        Timer::after(embassy_time::Duration::from_millis(50)).await;
        // let angle = encoder.angle().await.unwrap_or(0.);
        if let Err(e) = encoder.update(Instant::now().as_micros()).await {
            error!("Failed to read encoder");
        } else {
            // info!(
            //     "Angle: {}",
            //     libm::roundf(encoder.get_angle() * 1000.) / 1000. * (180. / core::f32::consts::PI)
            // );
            info!(
                "Angle: {}, position: {}, turns: {}, Velocity: {}",
                libm::roundf(encoder.get_angle() * (180. / core::f32::consts::PI) * 100.) / 100.,
                libm::round(encoder.get_position() * 100.) / 100.,
                encoder.get_turns(),
                libm::roundf(encoder.get_velocity() * 100.) / 100.
            );
        }
    }
}

#[embassy_executor::main]
// #[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14; // purple
    let scl = p.PIN_15; // blue

    info!("set up i2c ");
    let i2c =
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    // let mut encoder = as5600::asynch::As5600::new(i2c);
    let encoder = crate::hardware::as5600::AS5600::new(i2c);
    // let encoder = crate::hardware::mt_6701::MT6701::new(i2c, 0x06);

    let mut c = embassy_rp::pwm::Config::default();
    let desired_freq_hz = 20_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;
    c.top = period;
    c.divider = divider.into();

    let voltage_limit = 1.0;

    let pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    // let mut pwm1 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c.clone());
    // let mut pwm2 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, c.clone());
    let pwm12 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, c.clone());
    let pwm_driver = crate::simplefoc::pwm_driver::PWMDriver::new(pwm0, pwm12, voltage_limit, 12.);

    let enable_pin = Output::new(p.PIN_6, Level::Low);

    // let motor_config = crate::simplefoc::bldc::BLDCMotor::new(
    //     7,    // pole pairs
    //     11.2, // phase resistance (TODO: measure this)
    //     90.,  // motor kv
    //     // Some(0.00035), // phase inductance
    //     None,
    // );

    let motor_config = crate::simplefoc::bldc::BLDCMotor::new(
        7, // pole pairs
        // 11.2, // phase resistance (TODO: measure this)
        5.2,  // phase resistance (TODO: measure this)
        260., // motor kv
        // Some(0.00035), // phase inductance
        None,
    );

    let mut foc =
        crate::simplefoc::foc::SimpleFOC::new(encoder, pwm_driver, enable_pin, motor_config);

    foc.sensor_direction = crate::simplefoc::types::SensorDirection::Normal;

    // foc.motion_control = crate::simplefoc::types::MotionControlType::Angle;
    foc.motion_control = crate::simplefoc::types::MotionControlType::Torque;

    info!("Starting init");

    foc.init();

    info!("Starting FOC init");

    foc.init_foc().await;

    // let time_limit = Timer::after(embassy_time::Duration::from_secs(3));

    info!("Starting main loop");

    spawner.spawn(loop_foc(foc)).unwrap();

    // loop {
    // // led.set_high();
    // Timer::after_millis(250).await;

    // foc.update_foc().await;

    // if time_limit.is_elapsed() {
    //     foc.disable();
    //     break;
    // }
    // }

    // info!("Done");
}

#[embassy_executor::task]
async fn loop_foc(
    mut foc: crate::simplefoc::foc::SimpleFOC<'static, embassy_rp::peripherals::I2C1>,
) {
    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(500));
    // let mut ticker = Ticker::every(embassy_time::Duration::from_millis(1000));
    let mut max_time = Instant::now() + embassy_time::Duration::from_secs(2);
    let mut n = 0;

    foc.enable();

    // // let tgt = 130.;
    // let tgt = 170.;
    // foc.set_position_target(tgt);

    loop {
        ticker.next().await;
        foc.update_foc().await;

        if n >= 500 {
            info!("Tick");
            n = 0;
            let pos = foc.get_position_actual();
            info!(
                "Position: {}",
                libm::roundf(pos * 180. / core::f32::consts::PI * 100.) / 100.
            );
        } else {
            n += 1;
        }

        if Instant::now() > max_time {
            foc.disable();
            break;
        }
    }
}

#[embassy_executor::task]
async fn toggle_led() {
    unimplemented!()
}
