use defmt::{debug, error, info, trace, warn};

use embassy_time::{Instant, Ticker};
use static_cell::StaticCell;

use crate::{Irqs, comms::usb::UsbLogger, hardware::encoder_sensor::EncoderSensor as _};

pub static mut CORE1_STACK: embassy_rp::multicore::Stack<4096> =
    embassy_rp::multicore::Stack::new();
pub static EXECUTOR0: StaticCell<embassy_executor::Executor> = StaticCell::new();
pub static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();

#[embassy_executor::task]
pub async fn core0_task(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        crate::hardware::mt_6701::MT6701<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Normal);
    foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Inverted);
    // foc.set_encoder_direction(crate::simplefoc::types::SensorDirection::Unknown);

    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Torque);
    // foc.set_motion_control(crate::simplefoc::types::MotionControlType::Velocity);
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
    let time_limit = 2.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    // foc.set_debug_freq(10);
    // foc.set_debug_freq(50);
    // foc.set_debug_freq(75);
    foc.set_debug_freq(100);
    // foc.set_debug_freq(1_000);

    foc.set_vel_pid_debug(3.);

    // foc.pid_velocity.set_p(0.23195632);
    // foc.pid_velocity.set_i(15.113432);
    // foc.pid_velocity.set_d(0.000891323);

    // foc.set_target_torque(10.0);

    // // let tgt = 1.64;
    // let mut tgt = 1.0;
    // foc.set_target_position(tgt);

    // let tgt = 60.;

    let mut x = 0;
    foc.set_target_velocity(3.14 * 1.);

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    let v = 3.14;

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

    // #[cfg(feature = "nope")]
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
