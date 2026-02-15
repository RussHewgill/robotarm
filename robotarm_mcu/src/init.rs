use defmt::{debug, error, info, trace, warn};

use embassy_time::{Instant, Ticker};
use robotarm_protocol::SerialCommand;
use static_cell::StaticCell;

use crate::{Irqs, comms::usb::UsbLogger, hardware::encoder_sensor::EncoderSensor};

pub static mut CORE1_STACK: embassy_rp::multicore::Stack<4096> =
    embassy_rp::multicore::Stack::new();
pub static EXECUTOR0: StaticCell<embassy_executor::Executor> = StaticCell::new();
pub static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();

#[embassy_executor::task]
pub async fn core0_task0(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        crate::hardware::mt_6701_ssi::MT6701<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
        >,
    >,
) {
    core0_task(foc).await;
}

#[embassy_executor::task]
pub async fn core0_task1(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        crate::hardware::mt_6701_ssi::MT6701<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
        >,
    >,
) {
    core0_task(foc).await;
}

// #[embassy_executor::task]
pub async fn core0_task<SENSOR: EncoderSensor>(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<'static, SENSOR>,
) {
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Normal);
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Inverted);
    foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Torque);
    foc.set_motion_control(crate::simplefoc::types::MotionControlType::Velocity);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Angle);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::VelocityOpenLoop);

    info!("Starting init");
    foc.init();
    info!("Starting FOC init");
    foc.init_foc().await;
    // spawner.spawn(loop_foc(foc)).unwrap();
    foc.enable();

    // spawner.spawn(test_foc(foc)).unwrap();

    let update_rate_hz = 20_000;
    // let update_rate_hz = 1_000;
    let time_limit = 2.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    // foc.set_debug_freq(1);
    // foc.set_debug_freq(50);
    // foc.set_debug_freq(75);
    // foc.set_debug_freq(100);
    foc.set_debug_freq(500);
    // foc.set_debug_freq(0);

    // foc.set_vel_pid_debug(3.);

    // foc.set_target_torque(10.0);

    // // let tgt = 1.64;
    // let mut tgt = 1.0;
    // foc.set_target_position(tgt);

    let mut x = 0;

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    let v = 3.14;
    foc.set_target_velocity(v);

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

    foc.sensor_downsample = 0;
    // foc.sensor_downsample = 5;
    // foc.sensor_downsample = 8;

    // #[cfg(feature = "nope")]
    loop {
        embassy_futures::yield_now().await;

        // ticker.next().await;
        foc.run_commands().await;
        foc.update_foc().await;

        let t1 = Instant::now();
        #[cfg(feature = "nope")]
        if t1 > max_time {
            let elapsed = t1 - t0;
            let freq = c as f32 / (elapsed.as_micros() as f32 * 1e-6);
            info!(
                "Elapsed: {}s, Cycles: {}, Freq: {}Hz",
                elapsed.as_millis() as f32 * 1e-3,
                c,
                freq
            );
            t0 = t1;
            c = 0;
            max_time = t1 + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
        } else {
            c += 1;
        }

        // #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            match x {
                0 => {
                    info!("Setting velocity to -1");
                    x = 1;
                    // foc.set_target_position(1.0);
                    foc.set_target_velocity(v * -1.);
                }
                _ => {
                    info!("Setting velocity to 1");
                    x = 0;
                    // foc.set_target_position(6.);
                    foc.set_target_velocity(v * 1.);
                } // 1 => {
                  //     // info!("Setting velocity to 0");
                  //     x = 2;
                  //     // foc.set_target_position(3.);
                  //     foc.set_target_velocity(3.14 * 0.);
                  // }
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
