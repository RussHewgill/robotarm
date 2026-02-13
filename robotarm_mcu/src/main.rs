#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unexpected_cfgs)]

mod comms;
mod hardware;
mod simplefoc;

use crate::hardware::encoder_sensor::EncoderSensor;
use defmt::{debug, error, info, trace, warn};

// use rtt_target::{rprintln, rtt_init};
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, pwm::SetDutyCycle};
use embassy_time::{Instant, Ticker, Timer};

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
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

/// rtt tests
// #[embassy_executor::main]
#[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // USB 2
    // #[cfg(feature = "nope")]
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

        // info!("set up encoder");
        // let encoder = crate::hardware::as5600::AS5600::new(i2c).await;
        let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

        let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        let mut usb = crate::comms::usb::UsbMonitor::init(&spawner, driver);

        // let (mut sender, mut receiver) = class.split();

        // let mut rx: [u8; 64] = [0; 64];
        // sender.wait_connection().await;

        // usb.wait_connection().await;

        // let mut n: f32 = 0.0;
        let mut n = 0;
        let mut x = 0;

        let mut data: [u8; 64] = [0; 64];

        loop {
            let now = Instant::now().as_micros();
            encoder._update(now).await.unwrap();

            let angle = encoder.get_angle();
            let velocity = encoder.get_velocity();

            debug!(
                "now: {} us, angle: {} rad, velocity: {} rad/s",
                now, angle, velocity
            );

            let msg = robotarm_protocol::SerialLogMessage::MotorData {
                id: x,
                timestamp: now,
                position: angle,
                angle: 0.,
                velocity,
                target_position: x as f32,
                target_velocity: 0.,
                motor_current: 1.,
                motor_voltage: (2., 3.),
            };

            x += 1;

            // usb.send(msg).await;
            usb.send_log_msg(msg);

            #[cfg(feature = "nope")]
            if n > 1_0 {
                n = 0;
                //
            } else {
                n += 1;
            }

            // let data = b"0.0\r\n";

            // usb.send(data).await;

            Timer::after(embassy_time::Duration::from_millis(1000)).await;
        }
    }

    // USB
    #[cfg(feature = "nope")]
    {
        // rprintln!("Hello, world!");

        // let mut config = embassy_rp::uart::Config::default();
        // config.baudrate = 115_200;
        // let mut uart = embassy_rp::uart::Uart::new_blocking(p.UART1, p.PIN_8, p.PIN_9, config);

        use static_cell::StaticCell;

        let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        // Create embassy-usb Config
        let config = {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("Embassy Serial Logger");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let mut class = {
            static STATE: StaticCell<embassy_usb::class::cdc_acm::State> = StaticCell::new();
            let state = STATE.init(embassy_usb::class::cdc_acm::State::new());
            embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64)
        };

        // Build the builder.
        let usb = builder.build();

        // Run the USB device.
        spawner.spawn(usb_task(usb)).unwrap();

        let (mut sender, mut receiver) = class.split();

        let mut rx: [u8; 64] = [0; 64];
        sender.wait_connection().await;

        loop {
            let data = b"Hello, world!";
            let _ = sender.write_packet(data).await;
            Timer::after(embassy_time::Duration::from_millis(500)).await;
        }

        #[cfg(feature = "nope")]
        loop {
            // uart.blocking_write("Hello World!\r\n".as_bytes()).unwrap();
            // rprintln!("Test 0");
            // rprintln!("Test 0");
            // defmt::println!("Test 0");
            // defmt::debug!("Test 1");
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
            loop {
                // let _ = echo(&mut class).await;
                // let data = b"Hello, world!";
                // let _ = class.write_packet(data).await;
                // info!("Wrote packet");
                Timer::after(embassy_time::Duration::from_millis(500)).await;
            }
        }
    }
}

// #[embassy_executor::main]
#[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14; // purple
    let scl = p.PIN_15; // blue

    let mut i2c =
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

    // let mut encoder = crate::hardware::as5600::AS5600::new(i2c);

    info!("looping");
    // #[cfg(feature = "nope")]
    loop {
        // info!("Tick");
        Timer::after(embassy_time::Duration::from_millis(50)).await;

        if let Ok(angle) = encoder.read_raw_angle().await {
            let angle = (angle as f32 / 16384_f32) * 2.0 * core::f32::consts::PI;
            // debug!("Raw angle: {}", angle);
            debug!("Angle: {}", angle);
        }

        // let angle = encoder.angle().await.unwrap_or(0.);
        // if let Err(e) = encoder.update(Instant::now().as_micros()).await {
        //     error!("Failed to read encoder");
        // } else {
        // }
    }
}

#[cfg(feature = "nope")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // PWM
    #[cfg(feature = "nope")]
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
            let mut buf = [0u8; 1];
            if let Err(e) = i2c.write_read(address, &[0x0E], &mut buf) {
                error!("I2C error: {:?}", e);
            } else {
                // debug!("Raw angle: {}", u16::from_be_bytes([0, buf[0]]));
            }
            // Timer::after(embassy_time::Duration::from_millis(100)).await;

            // 0x03 = angle[13:6]
            // 0x04 = angle[5:0]

            let angle = ((buf[0] as u16) << 6) & (buf[1] as u16 & 0b00111111);
        }
    }

    // info!("Done");
}

/// MARK: Main
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
    // let encoder = crate::hardware::as5600::AS5600::new(i2c).await;
    let encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

    let mut c = embassy_rp::pwm::Config::default();
    let desired_freq_hz = 24_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    c.top = 3124;
    c.divider = 1.into();
    c.phase_correct = true;

    // let voltage_limit = 2.0;
    let voltage_limit = 3.;

    let pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    let pwm12 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, c.clone());

    // info!("set up PWM driver");
    let pwm_driver =
        crate::simplefoc::pwm_driver::PWMDriver::new(pwm0, pwm12, c, voltage_limit, 12.);

    let enable_pin = embassy_rp::gpio::Output::new(p.PIN_6, embassy_rp::gpio::Level::Low);

    let motor_config = crate::simplefoc::bldc::BLDCMotor::new(
        7, // pole pairs
        // 11.2, // phase resistance (TODO: measure this)
        Some(5.35), // phase resistance (TODO: measure this)
        Some(260.), // motor kv
        None,
    );

    // let motor_config = crate::simplefoc::bldc::BLDCMotor::new(
    //     7,    // pole pairs
    //     None, // phase resistance (TODO: measure this)
    //     None, // motor kv
    //     None,
    // );

    let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

    let usb = crate::comms::usb::UsbMonitor::init(&spawner, driver);

    // info!("set up FOC");
    let mut foc = crate::simplefoc::foc_types::SimpleFOC::new(
        encoder,
        pwm_driver,
        enable_pin,
        motor_config,
        Some(usb),
        // None,
    );

    foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Normal);
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

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
}

#[embassy_executor::task]
async fn test_foc(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        // hardware::as5600::AS5600<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        // >,
        hardware::mt_6701::MT6701<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    info!("Starting FOC test");
    // let update_rate_hz = 9000;
    let update_rate_hz = 1_000;
    let print_rate_hz = 20;
    let time_limit = 10;

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

    debug!("starting test loop");
    loop {
        ticker.next().await;
        // foc.debug_update_sensor().await;

        // if v.abs() > 0.01 {
        //     // debug!("Angle: {}", foc.debug_encoder().get_angle());
        //     // debug!("Velocity: {}", v);
        //     debug!("Raw angle: {}", foc.debug_encoder().get_raw_angle());
        // }

        // let angle = foc.encoder.sample_raw().await.unwrap();
        // len += 1;
        // sum += angle as u64;

        // min = min.min(angle as u64);
        // max = max.max(angle as u64);

        let t_us = Instant::now().as_micros();
        foc.encoder.update(t_us).await.unwrap();
        foc.run_commands().await;

        // vn += 1;
        // vs += v;
        if n >= n_max {
            n = 0;

            // let v = foc.get_shaft_velocity();
            // // let v = foc.debug_encoder().get_velocity();

            let position = foc.encoder.get_mechanical_angle();
            let v = foc.encoder.get_velocity();

            debug!(
                "Angle: {}, Velocity: {}",
                // libm::roundf(angle * 1000.) / 1000. * (180. / core::f32::consts::PI),
                position,
                v,
            );

            // foc.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
            //     id: 0,
            //     timestamp: t_us,
            //     target: 0.0,
            //     position: foc.encoder.get_angle(),
            //     angle: foc.encoder.get_mechanical_angle(),
            //     velocity: v,
            // })
            // .await;

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

        #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            // let avg = sum as u64 / len as u64;
            // debug!(
            //     "Average raw angle: {}, n: {}, min: {}, max: {}, range: {}",
            //     avg,
            //     len,
            //     min,
            //     max,
            //     max - min
            // );

            info!("Halting FOC test");
            foc.disable();
            break;
        }
    }
}

#[embassy_executor::task]
async fn loop_foc(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        // hardware::as5600::AS5600<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        // >,
        hardware::mt_6701::MT6701<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    info!("Starting main loop");

    let update_rate_hz = 20_000;
    let print_rate_hz = 100;
    // let time_limit = 1.5;
    let time_limit = 2.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let n_max = update_rate_hz / print_rate_hz;
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    foc.enable();

    // let tgt = 1.64;
    let mut tgt = 1.0;
    foc.set_target_position(tgt);

    // let tgt = 60.;

    let mut x = 0;
    foc.set_target_velocity(3.14 * 1.);

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    // foc.debug = true;
    loop {
        ticker.next().await;
        foc.run_commands().await;
        foc.update_foc().await;

        // #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            match x {
                0 => {
                    info!("Setting velocity to -1");
                    x = 1;
                    // foc.set_target_position(1.0);
                    foc.set_target_velocity(3.14 * -1.);
                }
                1 => {
                    info!("Setting velocity to 0");
                    x = 2;
                    // foc.set_target_position(3.);
                    foc.set_target_velocity(3.14 * 0.);
                }
                _ => {
                    info!("Setting velocity to 1");
                    x = 0;
                    // foc.set_target_position(6.);
                    foc.set_target_velocity(3.14 * 1.);
                }
            }
            max_time = max_time + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
        }

        // if Instant::now() > max_time {
        //     tgt += 3.14 / 2.;
        //     foc.set_target_position(tgt);
        //     max_time = max_time + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
        // }

        // if Instant::now() > max_time {
        //     info!("Halting FOC loop");
        //     foc.disable();
        //     break;
        // }
    }
}
