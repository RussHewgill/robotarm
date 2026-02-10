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
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    // DMA_IRQ_0 => InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
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

#[cfg(feature = "nope")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // PWM
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        // i2c_config.frequency = 400_000; // 400 kHz
        i2c_config.frequency = 1_000_000; // 1 MHz
        let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

        let mut sensor = as5600::asynch::As5600::new(i2c);

        let mut cfg = sensor.config().await.unwrap();
        debug!("AS5600 initial config: {:#}", cfg);

        // cfg.slow_filter = as5600::configuration::SlowFilterMode::X16;
        // // cfg.slow_filter = as5600::configuration::SlowFilterMode::X2;
        // cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::SlowFilterOnly;
        // // cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::Lsb10;

        cfg.output_stage = as5600::configuration::OutputStage::DigitalPwm;
        cfg.pwm_frequency = as5600::configuration::PwmFreq::PwmF4;

        sensor.set_config(cfg).await.unwrap();

        let mut cfg = sensor.config().await.unwrap();
        debug!("AS5600 initial config: {:#}", cfg);

        // let cfg: embassy_rp::pwm::Config = Default::default();
        // let pwm = embassy_rp::pwm::Pwm::new_input(
        //     p.PWM_SLICE2,
        //     p.PIN_5,
        //     gpio::Pull::None,
        //     embassy_rp::pwm::InputMode::RisingEdge,
        //     cfg,
        // );

        // let mut ticker = Ticker::every(embassy_time::Duration::from_millis(50));
        // for _ in 0..5 {
        //     ticker.next().await;
        //     info!("Input frequency: {} Hz", pwm.counter());
        // }

        //
    }

    // ADC
    #[cfg(feature = "nope")]
    {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};

        let mut adc = Adc::new(p.ADC, Irqs, Config::default());
        let mut dma = p.DMA_CH0;
        let mut pin = Channel::new_pin(p.PIN_26, gpio::Pull::Up);

        let mut sensor = crate::hardware::mt_6701_adc::MT6701::new(adc, dma, pin).await;

        let mut min = 10_000.0f32; // 8
        let mut max = 0.0f32; // 4095

        loop {
            Timer::after_millis(5).await;

            let sample = sensor.sample().await;

            debug!("Sample: {}", sample);

            // adc.read_many(&mut pin, &mut buf, div, dma.reborrow())
            //     .await
            //     .unwrap();

            // let avg = buf.iter().map(|&x| x as u32).sum::<u32>() as f32 / (buf.len() as f32);

            // min = min.min(avg);
            // max = max.max(avg);

            // debug!("ADC: {}, min: {}, max: {}", avg, min, max);
        }
    }

    #[cfg(feature = "nope")]
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        // let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);
        let mut i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C1, scl, sda, i2c_config);
        // let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;
        let address: u8 = 0b0000110;
        for _ in 0..1 {
            info!("Tick");
            Timer::after_millis(500).await;

            // use embedded_hal_async::i2c::I2c;
            use embedded_hal::i2c::I2c;

            let mut buf: [u8; 2] = [0; 2];

            // i2c.blocking_write_read(address, &[0x03], &mut buf).unwrap();

            // if let Err(e) = i2c.blocking_write_read(address, &[0x03], &mut buf) {
            //     error!("I2C Error: {:?}", e);
            // } else {
            //     info!("Read bytes: 0x{:02X} 0x{:02X}", buf[0], buf[1]);
            // }

            // i2c.blocking_write(address, &[0x03]).unwrap();
            //
            // info!("Wrote byte 0x03 to address 0x{:02X}", address);
            //
            // i2c.blocking_read(address, &mut buf[..1]).unwrap();
            //
            // info!("Read byte: 0x{:02X}", buf[0]);

            // if let Err(e) = i2c.write(address, &[0x03]) {
            //     debug!("I2C write error: {}", e);
            // }

            // Timer::after_millis(1).await;

            // let mut b: [u8; 1] = [0; 1];

            // i2c.read(address, &mut b).unwrap();

            // debug!("Read byte: {:?}", buf);
        }
    }

    // info!("Done");
}

#[embassy_executor::main]
// #[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14; // purple
    let scl = p.PIN_15; // blue

    // info!("set up i2c ");
    let mut i2c_config = embassy_rp::i2c::Config::default();
    // i2c_config.frequency = 400_000; // 400 kHz
    i2c_config.frequency = 1_000_000; // 1 MHz
    let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

    // info!("set up encoder");
    // let mut encoder = as5600::asynch::As5600::new(i2c);
    let encoder = crate::hardware::as5600::AS5600::new(i2c).await;
    // let encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

    // let mut adc = embassy_rp::adc::Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    // let mut dma = p.DMA_CH0;
    // let mut pin = embassy_rp::adc::Channel::new_pin(p.PIN_26, gpio::Pull::Up);

    // let mut encoder = crate::hardware::mt_6701_adc::MT6701::new(adc, dma, pin).await;

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

    // info!("set up PWM driver");
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
        5.35, // phase resistance (TODO: measure this)
        260., // motor kv
        // Some(0.00035), // phase inductance
        None,
    );

    // info!("set up FOC");
    let mut foc =
        crate::simplefoc::foc_types::SimpleFOC::new(encoder, pwm_driver, enable_pin, motor_config);

    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Normal);
    foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Torque);
    foc.set_motion_control(crate::simplefoc::types::MotionControlType::Velocity);
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
    // mut foc: crate::simplefoc::foc_types::SimpleFOC<
    //     'static,
    //     crate::hardware::mt_6701_adc::MT6701<'static, embassy_rp::peripherals::DMA_CH0>,
    // >,
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        hardware::as5600::AS5600<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    info!("Starting FOC test");
    // let update_rate_hz = 9000;
    let update_rate_hz = 1000;
    let print_rate_hz = 20;
    let time_limit = 5;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let n_max = update_rate_hz / print_rate_hz;
    let max_time = Instant::now() + embassy_time::Duration::from_secs(time_limit);
    let mut n = 0;

    // foc.disable();

    // let angle_target = 1.0;
    // let angle_current = 0.0;

    // let mut pid = crate::simplefoc::pid::PIDController::new(1.0, 0.0, 0.0, 0.0, 12.0);

    // let mut vn = 0;
    // let mut vs = 0.;

    let mut len = 0;
    let mut sum = 0;
    let mut min = u64::MAX;
    let mut max = u64::MIN;

    loop {
        ticker.next().await;
        // foc.debug_update_sensor().await;

        // if v.abs() > 0.01 {
        //     // debug!("Angle: {}", foc.debug_encoder().get_angle());
        //     // debug!("Velocity: {}", v);
        //     debug!("Raw angle: {}", foc.debug_encoder().get_raw_angle());
        // }

        let angle = foc.encoder.sample_raw().await.unwrap();
        len += 1;
        sum += angle as u64;

        min = min.min(angle as u64);
        max = max.max(angle as u64);

        // vn += 1;
        // vs += v;
        if n >= n_max {
            n = 0;

            // let v = foc.get_shaft_velocity();
            // // let v = foc.debug_encoder().get_velocity();

            // let angle = foc.encoder.get_angle();

            debug!(
                "Angle: {}",
                // libm::roundf(angle * 1000.) / 1000. * (180. / core::f32::consts::PI),
                angle
            );

            // debug!(
            //     "Angle: {}, velocity: {}",
            //     foc.debug_encoder().get_angle(),
            //     // v
            //     0.0,
            // );
            // let v = vs / (vn as f32);
            // vn = 0;
            // vs = 0.;
            // debug!("Velocity: {}", v);
            //
        } else {
            n += 1;
        }

        if Instant::now() > max_time {
            let avg = sum as u64 / len as u64;
            debug!(
                "Average raw angle: {}, n: {}, min: {}, max: {}, range: {}",
                avg,
                len,
                min,
                max,
                max - min
            );

            info!("Halting FOC test");
            foc.disable();
            break;
        }
    }
}

#[embassy_executor::task]
async fn loop_foc(
    // mut foc: crate::simplefoc::foc_types::SimpleFOC<'static, embassy_rp::peripherals::I2C1>,
    // mut foc: crate::simplefoc::foc_types::SimpleFOC<
    //     'static,
    //     crate::hardware::mt_6701_adc::MT6701<'static, embassy_rp::peripherals::DMA_CH0>,
    // >,
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        hardware::as5600::AS5600<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
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

    // let tgt = 60.;

    // foc.set_target_velocity(tgt * (60. / (2. * core::f32::consts::PI)));

    foc.set_target_velocity(3.14 * 1.);

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    foc.debug = true;
    loop {
        // ticker.next().await;
        foc.update_foc().await;

        if n >= n_max {
            n = 0;

            foc.debug = true;

            // let position = foc.get_position_actual();

            // info!(
            //     "Position: {:03}, Angle: {:03}, Vel: {}",
            //     libm::roundf(position * 100.) / 100.,
            //     libm::roundf(position * (180. / core::f32::consts::PI) * 100.) / 100.,
            //     foc.debug_encoder().get_velocity(),
            // );

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
