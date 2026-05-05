use defmt::{debug, error, info, trace, warn};

use embassy_futures::yield_now;
use embassy_time::{Instant, Ticker};
use robotarm_protocol::{SerialCommand, types::MotionControlType};
use static_cell::StaticCell;

use crate::{
    Irqs,
    comms::usb::UsbLogger,
    hardware::{
        current_sensor::CurrentSensor, encoder_sensor::EncoderSensor, ina226::INA226,
        ina240::INA240,
    },
};

pub static mut CORE1_STACK: embassy_rp::multicore::Stack<4096> =
    embassy_rp::multicore::Stack::new();
pub static EXECUTOR0: StaticCell<embassy_executor::Executor> = StaticCell::new();
pub static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();

#[embassy_executor::task]
pub async fn core0_task0(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        crate::hardware::mt_6701_ssi::MT6701<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
        >,
        // INA226<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
        // >,
    >,
    // mut output_encoder: crate::hardware::mt_6701::MT6701<
    //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
    // >,
) {
    // core0_task(foc, output_encoder).await;
    foc_task(foc).await;
}

#[embassy_executor::task]
pub async fn core0_task1(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        crate::hardware::mt_6701_ssi::MT6701<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
        >,
        // INA226<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
        // >,
        // INA240<embassy_rp::peripherals::DMA_CH0>,
    >,
    // mut output_encoder: crate::hardware::mt_6701::MT6701<
    //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
    // >,
) {
    // core0_task(foc, output_encoder).await;
    foc_task(foc).await;
}

// #[embassy_executor::task]
pub async fn foc_task<SENSOR: EncoderSensor, CURRENT: CurrentSensor>(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<'static, SENSOR, CURRENT>,
    // mut output_encoder: crate::hardware::mt_6701::MT6701<
    //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
    // >,
) {
    // // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::CW);
    // // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::CCW);
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

    match foc.id {
        0 => foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::CW),
        1 => foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::CW),
        _ => foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown),
    };

    // foc.set_motion_control(MotionControlType::Torque);
    // foc.set_motion_control(MotionControlType::Velocity);
    // foc.set_motion_control(MotionControlType::Angle);
    foc.set_motion_control(MotionControlType::VelocityOpenLoop);

    info!("Starting init");
    foc.init();
    info!("Starting FOC init");
    foc.init_foc().await;
    // spawner.spawn(loop_foc(foc)).unwrap();
    foc.enable();

    // spawner.spawn(test_foc(foc)).unwrap();

    let update_rate_hz = 20_000;
    // let update_rate_hz = 1_000;
    // let time_limit = 2.;
    let time_limit = 1.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    // foc.set_debug_freq(2);
    // foc.set_debug_freq(10);
    foc.set_debug_freq(100);
    // foc.set_debug_freq(500);
    // foc.set_debug_freq(0);

    // foc.set_vel_pid_debug(0.);

    // foc.set_target_torque(10.0);

    // // let tgt = 1.64;
    // let mut tgt = 1.0;
    // foc.set_target_position(tgt);

    let mut x = 0;

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    let v = 3.14;
    // foc.set_target_velocity(v);

    info!("Starting main loop");

    #[cfg(feature = "nope")]
    loop {
        ticker.next().await;

        let t_us = Instant::now().as_micros();

        foc.encoder.update(t_us).await.unwrap();
        foc.run_commands().await;

        let position = foc.encoder.get_mechanical_angle();
        let v = foc.encoder.get_velocity();

        debug!(
            "Angle: {}, Velocity: {}",
            // libm::roundf(angle * 1000.) / 1000. * (180. / core::f32::consts::PI),
            position,
            v,
        );
    }

    // basic sine wave test
    #[cfg(feature = "nope")]
    {
        let mut t0 = Instant::now();
        let mut c = 0;

        let rate_hz = 5_000;
        let mut ticker = Ticker::every(embassy_time::Duration::from_micros(1_000_000 / rate_hz));

        let mut a = 0.;
        loop {
            ticker.next().await;

            a += 0.01;

            foc._set_phase_pwm(
                libm::sinf(a + 0.5),
                libm::sinf(a + 0.5 + core::f32::consts::PI * 2. / 3.),
                libm::sinf(a + 0.5 + core::f32::consts::PI * 4. / 3.),
            );
        }
    }

    let mut t0 = Instant::now();
    let mut c = 0;

    // foc.angle_sensor_downsample = 0;
    // foc.angle_sensor_downsample = 1;
    // foc.angle_sensor_downsample = 2;
    // foc.angle_sensor_downsample = 5;
    // foc.angle_sensor_downsample = 8;

    foc.current_sensor_downsample = 10;

    // foc.angle_sensor_downsample = 1;
    // foc.current_sensor_downsample = 1;

    // foc.torque_controller = crate::simplefoc::types::TorqueControlType::FOCCurrent;

    // let output_encoder_downsample = 10;
    // let mut output_encoder_counter = 0;

    let output_encoder_update_rate_hz = 20;
    let output_encoder_update_interval_us =
        embassy_time::Duration::from_micros(1_000_000 / output_encoder_update_rate_hz);
    let mut output_encoder_next_update =
        (Instant::now() + output_encoder_update_interval_us).as_micros();

    // #[cfg(feature = "nope")]
    loop {
        yield_now().await;
        foc.run_commands().await;

        let t_us = Instant::now().as_micros();
        foc.loop_foc(t_us).await;
        foc.update_foc(t_us).await;

        #[cfg(feature = "nope")]
        if t_us >= output_encoder_next_update {
            output_encoder.update(t_us).await.unwrap();
            let output_position = output_encoder.get_mechanical_angle();
            let output_velocity = output_encoder.get_velocity();

            debug!(
                "Output encoder - Angle: {}, Velocity: {}",
                output_position, output_velocity,
            );

            foc.send_debug_message(robotarm_protocol::SerialLogMessage::EncoderData {
                id: foc.id,
                timestamp: t_us,
                position: output_position,
                velocity: None,
            })
            .await;

            output_encoder_next_update =
                (Instant::now() + output_encoder_update_interval_us).as_micros();
        }

        let t1 = Instant::now();
        #[cfg(feature = "nope")]
        if t1 > max_time {
            let elapsed = t1 - t0;
            let freq = c as f32 / (elapsed.as_micros() as f32 * 1e-6);
            info!(
                "ID: {}, Elapsed: {}s, Cycles: {}, Freq: {}Hz",
                foc.id,
                elapsed.as_millis() as f32 * 1e-3,
                c,
                freq
            );
            t0 = t1;
            c = 0;
            max_time = t1 + embassy_time::Duration::from_millis((time_limit * 5000.) as u64);
        } else {
            c += 1;
        }

        #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            match x {
                0 => {
                    info!("Setting velocity to -1");
                    x = 1;
                    foc.set_target_velocity(v * -1.);
                }
                _ => {
                    info!("Setting velocity to 1");
                    x = 0;
                    foc.set_target_velocity(v * 1.);
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

    //
}

#[embassy_executor::task]
pub async fn core1_task(driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>) {
    // crate::comms::usb::UsbMonitor::init(&spawner, driver);
}
