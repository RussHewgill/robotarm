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
use embassy_rp::{bind_interrupts, gpio, pwm::SetDutyCycle};
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
    let mut i2c_config = embassy_rp::i2c::Config::default();
    i2c_config.frequency = 400_000; // 400 kHz
    let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

    // let mut encoder = as5600::asynch::As5600::new(i2c);
    let encoder = crate::hardware::as5600::AS5600::new(i2c);
    // let encoder = crate::hardware::mt_6701::MT6701::new(i2c, 0x06);

    let mut c = embassy_rp::pwm::Config::default();
    let desired_freq_hz = 24_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    c.top = 3124;
    c.divider = 1.into();
    c.phase_correct = true;

    // let voltage_limit = 1.0;
    // let voltage_limit = 1.5;
    let voltage_limit = 2.0;
    // let voltage_limit = 2.5;

    let pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    // let mut pwm1 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, c.clone());
    // let mut pwm2 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, c.clone());
    let pwm12 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, c.clone());

    // let m = pwm0.max_duty_cycle();
    // info!("PWM max duty cycle: {}", m);

    let pwm_driver =
        crate::simplefoc::pwm_driver::PWMDriver::new(pwm0, pwm12, c, voltage_limit, 12.);

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

    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Normal);
    foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

    foc.set_motion_control(crate::simplefoc::types::MotionControlType::Torque);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Velocity);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Angle);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::VelocityOpenLoop);

    info!("Starting init");
    foc.init();

    info!("Starting FOC init");
    foc.init_foc().await;

    spawner.spawn(loop_foc(foc)).unwrap();
    // spawner.spawn(test_foc(foc)).unwrap();

    // foc.disable();
    // info!("Done");
}

#[embassy_executor::task]
async fn test_foc(
    mut foc: crate::simplefoc::foc::SimpleFOC<'static, embassy_rp::peripherals::I2C1>,
) {
    info!("Starting FOC test");
    let update_rate_hz = 20000;
    let print_rate_hz = 20;
    let time_limit = 3;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let n_max = update_rate_hz / print_rate_hz;
    let max_time = Instant::now() + embassy_time::Duration::from_secs(time_limit);
    let mut n = 0;

    foc.disable();

    let mut lpf_velocity = simplefoc::lowpass::LowPassFilter::new(0.005);

    loop {
        ticker.next().await;
        foc.update_sensor().await;

        if n >= n_max {
            n = 0;

            let encoder = foc.debug_encoder();

            // let angle = encoder.get_angle();
            let position = encoder.get_position();
            let velocity = encoder.get_velocity();

            // info!(
            //     // "Angle: {}, position: {}, turns: {}, Velocity: {}",
            //     "position: {:03}, Velocity: {}",
            //     // libm::roundf(angle * (180. / core::f32::consts::PI) * 100.) / 100.,
            //     libm::round(position * 100.),
            //     // libm::roundf(velocity * 100.) / 100.
            //     velocity,
            // );

            // #[cfg(feature = "nope")]
            if velocity.abs() > 0. {
                // let velocity = lpf_velocity.filter(velocity);
                info!(
                    // "Angle: {}, position: {}, turns: {}, Velocity: {}",
                    "position: {:03}, Velocity: {}",
                    // libm::roundf(angle * (180. / core::f32::consts::PI) * 100.) / 100.,
                    libm::round(position * 100.),
                    // libm::roundf(velocity * 100.) / 100.
                    velocity,
                );
            }

            //
        } else {
            n += 1;
        }

        if Instant::now() > max_time {
            info!("Halting FOC test");
            foc.disable();
            break;
        }
    }
}

#[embassy_executor::task]
async fn loop_foc(
    mut foc: crate::simplefoc::foc::SimpleFOC<'static, embassy_rp::peripherals::I2C1>,
) {
    info!("Starting main loop");

    let update_rate_hz = 20000;
    let print_rate_hz = 20;
    let time_limit = 1.5;
    // let time_limit = 3.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let n_max = update_rate_hz / print_rate_hz;
    let max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
    let mut n = 0;

    foc.enable();

    // // let tgt = 1.64;
    // let tgt = 1.0;
    // foc.set_target_position(tgt);

    // foc.set_target_velocity(0.0);
    foc.set_target_velocity(0.5);
    // foc.set_target_velocity(1.0);

    // foc.set_target_torque(0.);
    foc.set_target_torque(0.1);

    loop {
        ticker.next().await;
        foc.update_foc().await;

        if n >= n_max {
            n = 0;

            let position = foc.get_position_actual();

            info!("Position: {:03}", libm::roundf(position * 100.) / 100.)

            // let pos = foc.get_position_actual();
            // let v = foc.get_phase_voltages();
            // info!(
            //     "(*100) Position: {:03}, Phase Voltages: A: {:04}, B: {:04}, C: {:04}",
            //     libm::roundf(pos * 180. / core::f32::consts::PI * 100.) as i32,
            //     libm::roundf(v.a * 1000.) as i32,
            //     libm::roundf(v.b * 1000.) as i32,
            //     libm::roundf(v.c * 1000.) as i32,
            // );
            // foc.print_phase_voltages();
        } else {
            n += 1;
        }

        if Instant::now() > max_time {
            info!("Halting FOC loop");
            foc.disable();
            break;
        }
    }
}
