use defmt::{debug, error, info, trace, warn};
use embassy_time::{Duration, Instant, Timer};

use embassy_rp::gpio::Output;
use robotarm_protocol::{SerialCommand, SerialLogMessage, types::MotionControlType};

use crate::{
    hardware::{
        as5600::AS5600,
        current_sensor::{self, CurrentSensor},
        encoder_sensor::EncoderSensor,
        mt_6701::MT6701,
    },
    simplefoc::{
        bldc::BLDCMotor,
        foc_types::{FOCModulation, SimpleFOC},
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

/// debug
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub async fn debug_update_sensor(&mut self) {
        // if let Err(_e) = self.encoder.update(Instant::now().as_micros()).await {
        //     error!("Failed to update encoder");
        //     unimplemented!()
        // }
        let _ = self.encoder.update(Instant::now().as_micros()).await;
    }

    // pub fn debug_encoder(&mut self) -> &mut MT6701<'a, DMA> {
    //     &mut self.encoder
    // }

    pub async fn send_debug_message(&mut self, message: robotarm_protocol::SerialLogMessage) {
        // self.usb_logger.send_log_msg(message);
        // if let Some(logger) = &mut self.usb_logger {
        //     logger.send_log_msg(message);
        // }
        self.usb_logger.send_log_msg(message);
    }
}

/// control, info
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub fn enable(&mut self) {
        self.enabled = true;
        // self.enable_pin.set_high();
        self.pwm_driver.enable();
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        // self.enable_pin.set_low();
        self.set_phase_voltage(0., 0., 0.);
        self.pwm_driver.disable();
    }

    pub fn set_target_torque(&mut self, torque: f32) {
        self.motor.target_current = torque;
    }

    pub fn set_target_velocity(&mut self, velocity: f32) {
        self.motor.target_shaft_velocity = velocity;
    }

    pub fn set_target_position(&mut self, position: f32) {
        // self.motor.target_shaft_angle = position * -self.sensor_direction.multiplier();
        self.motor.target_shaft_angle = position;
    }

    pub fn set_motion_control(&mut self, control_type: MotionControlType) {
        self.motion_control = control_type;
    }

    pub fn set_torque_control(&mut self, control_type: TorqueControlType) {
        self.torque_controller = control_type;
    }

    pub fn set_encoder_direction(&mut self, direction: SensorDirection) {
        self.sensor_direction = direction;
    }

    pub fn set_acceleration(&self, _acceleration: ()) {
        unimplemented!()
    }

    // pub fn get_position_actual(&self) -> f32 {
    //     self.encoder.get_position() as f32
    // }

    pub fn set_voltage_limit(&mut self, voltage_limit: f32) {
        // self.motor.limit_voltage = voltage_limit;
        warn!("TODO: allow setting voltage");
    }

    pub fn get_position_requested(&self) -> f32 {
        self.motor.target_shaft_angle
    }

    pub fn print_phase_voltages(&self) {
        info!(
            "Phase Voltages: A: {} V, B: {} V, C: {} V",
            self.phase_v.a, self.phase_v.b, self.phase_v.c
        );
    }

    pub fn get_phase_voltages(&self) -> &PhaseVoltages {
        &self.phase_v
    }
}

/// internal
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub fn init(&mut self) {
        // check driver initialized

        // self.motor_status = FOCStatus::MotorInitializing;

        if self.motor.limit_voltage > self.pwm_driver.voltage_limit {
            error!(
                "Motor voltage limit {} is higher than driver voltage limit {}, constraining to driver limit",
                self.motor.limit_voltage, self.pwm_driver.voltage_limit
            );
            self.motor.limit_voltage = self.pwm_driver.voltage_limit;
        }

        if self.motor.voltage_sensor_align > self.motor.limit_voltage {
            error!(
                "Motor voltage sensor align {} is higher than motor voltage limit {}, constraining to motor limit",
                self.motor.voltage_sensor_align, self.motor.limit_voltage
            );
            self.motor.voltage_sensor_align = self.motor.limit_voltage;
        }

        // // velocity control loop controls current
        // self.pid_velocity.set_limit(self.motor.limit_current);

        // self.pid_angle.limit = self.motor.limit_velocity;

        // self.motor_status = FOCStatus::MotorReady;
    }

    pub async fn init_foc(&mut self) {
        // self.pid_current_q.limit = self.motor.limit_voltage;
        // self.pid_current_d.limit = self.motor.limit_voltage;

        // // needs phase resistance set
        // self.pid_velocity.set_limit(self.motor.limit_current);
        // // self.pid_angle.limit = self.motor.limit_velocity;

        if let Some(current_sensor) = &mut self.current_sensor {
            current_sensor.init().await.unwrap();
        }

        // self.motor_status = FOCStatus::MotorCalibrating;

        // align motor if necessary
        // alignment necessary for encoders!
        // sensor and motor alignment - can be skipped
        // by setting motor.sensor_direction and motor.zero_electric_angle
        self.align_sensor().await;
        // self.motor.shaft_angle

        //
    }

    async fn align_sensor(&mut self) {
        let _ = self.encoder.update(Instant::now().as_micros()).await;
        Timer::after_millis(1).await;
        // let _ = self.encoder.update(Instant::now().as_micros()).await;
        // Timer::after_millis(1).await;
        self.enable();

        // self.motor.voltage_sensor_align = 0.5;
        // self.motor.voltage_sensor_align = 1.0;
        // self.motor.voltage_sensor_align = 2.0;
        self.motor.voltage_sensor_align = 4.0;
        // self.motor.voltage_sensor_align = 6.0;
        // self.motor.voltage_sensor_align = 8.0;

        // find encoder rotation direction
        if self.sensor_direction == SensorDirection::Unknown {
            let n = 100;

            info!("Sensor direction unknown, starting alignment procedure...");
            // find natural direction
            // move one electrical revolution forward

            info!("Rotating motor forward to find natural direction...");
            for i in 0..n {
                let angle = crate::simplefoc::types::_3PI_2
                    + crate::simplefoc::types::_2PI * (i as f32) / n as f32;
                self.set_phase_voltage(self.motor.voltage_sensor_align, 0., angle);
                let _ = self.encoder.update(Instant::now().as_micros()).await;

                Timer::after_millis(2).await;
            }

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let mid_angle = self.encoder.get_angle();

            info!(
                "Mid angle: {}, now rotating backwards to find natural direction...",
                mid_angle
            );
            // move one electrical revolution backwards
            for i in (0..n).rev() {
                let angle = crate::simplefoc::types::_3PI_2
                    + crate::simplefoc::types::_2PI * (i as f32) / n as f32;
                self.set_phase_voltage(self.motor.voltage_sensor_align, 0., angle);
                let _ = self.encoder.update(Instant::now().as_micros()).await;

                Timer::after_millis(2).await;
            }

            Timer::after_millis(20).await;

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let end_angle = self.encoder.get_angle();

            let moved = (mid_angle - end_angle).abs();

            debug!(
                "Sensor alignment: mid_angle: {}, end_angle: {}, moved: {}",
                mid_angle, end_angle, moved
            );

            if (moved.abs() as f32) < (crate::simplefoc::types::_2PI / 101.0) {
                // no movement
                panic!(
                    "Sensor alignment failed: no movement detected, check motor and encoder wiring"
                );
            } else if mid_angle < end_angle {
                self.sensor_direction = SensorDirection::CW;
                // info!("Sensor direction: Normal");
                info!("Sensor direction: Clockwise");
            } else {
                self.sensor_direction = SensorDirection::CCW;
                // info!("Sensor direction: Reversed");
                info!("Sensor direction: Counter-Clockwise");
            }

            // self.sensor_direction = SensorDirection::Inverted;
            // self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);

            // error!("TODO: implement sensor alignment procedure to determine sensor direction");

            // self.disable();
            // panic!()
            //
        }

        // zero electric angle not known
        // basic simpleFOC aligment
        #[cfg(feature = "nope")]
        if self.zero_electric_angle == NOT_SET {
            // align the electrical phases of the motor and sensor
            // set angle -90(270 = 3PI/2) degrees

            info!("Aligning sensor, rotating motor to known angle...");

            Timer::after_millis(2000).await;

            // not sure why this is needed
            self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
            // Timer::after_millis(2).await;
            Timer::after_millis(50).await;

            self.set_phase_voltage(
                self.motor.voltage_sensor_align,
                0.,
                crate::simplefoc::types::_3PI_2,
            );

            // info!("Waiting for sensor to align...");

            // Timer::after_millis(700).await;
            Timer::after_millis(1000).await;

            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // info!("Sensor aligned, setting zero electric angle...");

            // get the current zero electric angle
            self.zero_electric_angle = 0.;
            let electrical_angle = self.get_electrical_angle();
            self.zero_electric_angle = electrical_angle;

            info!("Zero electric angle set to {}", self.zero_electric_angle);
            // info!("Shaft angle at alignment position: {}", _shaft_angle);

            Timer::after_millis(20).await;

            // // stop everything
            self.set_phase_voltage(0., 0., 0.);
            Timer::after_millis(200).await;
        }

        // #[cfg(feature = "nope")]
        if self.zero_electric_angle == NOT_SET {
            info!("Aligning sensor, rotating motor to known angle...");

            Timer::after_millis(1000).await;

            // // not sure why this is needed
            // self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
            // // Timer::after_millis(2).await;
            // Timer::after_millis(50).await;

            // self.set_phase_voltage(
            //     self.motor.voltage_sensor_align,
            //     0.,
            //     crate::simplefoc::types::_3PI_2,
            // );

            let align_angle = 3.0 * core::f32::consts::PI / 2.0; // SimpleFOC's default D-axis alignment angle

            let mut left_angles = heapless::Vec::<f32, 4>::new();
            let mut right_angles = heapless::Vec::<f32, 4>::new();

            let delay = 500;

            for _ in 0..2 {
                // approach from left
                let mut a = align_angle - 1.5;
                while a <= align_angle {
                    self.set_phase_voltage(self.motor.voltage_sensor_align, 0., a);
                    a += 0.01;
                    Timer::after_millis(3).await;
                    let _ = self.encoder.update(Instant::now().as_micros()).await;
                }
                Timer::after_millis(delay).await;
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                // let angle_left = self.encoder.get_mechanical_angle();

                self.zero_electric_angle = 0.;
                let angle_left = self.get_electrical_angle();
                debug!("Angle left: {}", angle_left);
                left_angles.push(angle_left).unwrap();

                // approach from right
                let mut a = align_angle + 1.5;
                while a >= align_angle {
                    self.set_phase_voltage(self.motor.voltage_sensor_align, 0., a);
                    a -= 0.01;
                    Timer::after_millis(3).await;
                    let _ = self.encoder.update(Instant::now().as_micros()).await;
                }
                Timer::after_millis(delay).await;
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                // let angle_right = self.encoder.get_mechanical_angle();

                self.zero_electric_angle = 0.;
                let angle_right = self.get_electrical_angle();
                debug!("Angle right: {}", angle_right);

                right_angles.push(angle_right).unwrap();
            }

            self.set_phase_voltage(0., 0., 0.);

            let avg_left = left_angles.iter().copied().sum::<f32>() / left_angles.len() as f32;
            let avg_right = right_angles.iter().copied().sum::<f32>() / right_angles.len() as f32;

            debug!("Avg left angle: {}", avg_left);
            debug!("Avg right angle: {}", avg_right);

            let avg_angle = (avg_left + avg_right) / 2.0;
            // let angle =
            //     self.sensor_direction.multiplier() * avg_angle * self.motor.pole_pairs as f32;
            // let angle = angle - align_angle;

            // let angle = Self::normalize_angle(angle);

            self.zero_electric_angle = avg_angle;
            info!("Zero electric angle set to {}", self.zero_electric_angle);
        } else {
            debug!(
                "Zero electric angle already set to {}, skipping alignment",
                self.zero_electric_angle
            );
        }
        // self.disable();
    }

    pub async fn find_angle_limits(&mut self) -> (f32, f32) {
        // start turning slowly in one direction until the encoder stops moving, then record the angle

        let mut encoder_pos_prev = self.encoder.get_angle();

        self.set_motion_control(MotionControlType::Velocity);

        let vel = 2.0; // rad/s

        let angle_update_rate = 20;
        let angle_update_period = Duration::from_micros(1_000_000 / angle_update_rate);
        let mut angle_next_update = (Instant::now() + angle_update_period).as_micros();

        let max_time_secs = 1;
        let n0 = max_time_secs * angle_update_rate;

        let d_angle_expected = vel / angle_update_rate as f32;

        self.enable();

        for i in 0..2 {
            if i == 0 {
                self.set_target_velocity(-vel);
            } else {
                self.set_target_velocity(vel);
            }

            let mut n = n0;
            loop {
                embassy_futures::yield_now().await;
                // self.run_commands().await;
                self.run_commands();

                let t_us = Instant::now().as_micros();
                self.loop_foc(t_us).await;
                self.update_foc(t_us).await;

                if t_us >= angle_next_update {
                    let encoder_pos = self.encoder.get_angle();

                    let d_angle = (encoder_pos - encoder_pos_prev).abs();

                    debug!("d_angle_expected: {}", d_angle_expected);
                    debug!("d_angle: {}", d_angle);

                    encoder_pos_prev = encoder_pos;

                    n -= 1;
                    angle_next_update = (Instant::now() + angle_update_period).as_micros();
                }

                if n <= 0 {
                    break;
                }
            }
        }

        // unimplemented!()
        (0.0, 0.0)
    }

    #[cfg(feature = "nope")]
    fn fit_encoder_offset(expected: &[f32], measured: &[f32]) -> (f32, f32, f32, f32) {
        if expected.len() != measured.len() || expected.len() < 4 {
            return (0.0, 1.0, 0.0, 0.0);
        }

        let mut a = [[0.0f32; 4]; 4];
        let mut b = [0.0f32; 4];

        for i in 0..expected.len() {
            let x = expected[i];
            let y = measured[i];
            let s = libm::sinf(x);
            let c = libm::cosf(x);
            let basis = [1.0f32, x, s, c];

            for row in 0..4 {
                b[row] += basis[row] * y;
                for col in 0..4 {
                    a[row][col] += basis[row] * basis[col];
                }
            }
        }

        // Gaussian elimination with partial pivoting.
        let mut mat = a;
        let mut vec = b;

        for i in 0..4 {
            let mut pivot_row = i;
            let mut pivot_val = mat[i][i].abs();

            for r in (i + 1)..4 {
                let v = mat[r][i].abs();
                if v > pivot_val {
                    pivot_val = v;
                    pivot_row = r;
                }
            }

            if pivot_val < 1e-8 {
                return (0.0, 1.0, 0.0, 0.0);
            }

            if pivot_row != i {
                mat.swap(i, pivot_row);
                vec.swap(i, pivot_row);
            }

            let inv_pivot = 1.0 / mat[i][i];
            for col in i..4 {
                mat[i][col] *= inv_pivot;
            }
            vec[i] *= inv_pivot;

            for row in 0..4 {
                if row == i {
                    continue;
                }

                let factor = mat[row][i];
                if factor == 0.0 {
                    continue;
                }

                for col in i..4 {
                    mat[row][col] -= factor * mat[i][col];
                }
                vec[row] -= factor * vec[i];
            }
        }

        let offset = vec[0];
        let gain = vec[1];
        let sin_coeff = vec[2];
        let cos_coeff = vec[3];
        let phase = libm::atan2f(-cos_coeff, sin_coeff);
        let amplitude = libm::sqrtf(sin_coeff * sin_coeff + cos_coeff * cos_coeff);

        (offset, gain, phase, amplitude)
    }

    #[cfg(feature = "nope")]
    pub async fn calibrate_encoder(&mut self) {
        // self.set_motion_control(MotionControlType::VelocityOpenLoop);

        self.motor.voltage_sensor_align = 4.0;

        // let vel = 1.0; // rad/s

        self.enable();
        // self.set_target_velocity(vel);

        // let (offset, gain, phase, amplitude) = Self::fit_encoder_offset(&expected, &measured);
        // debug!(
        //     "encoder fit: offset={}, gain={}, phase={}, amplitude={}",
        //     offset, gain, phase, amplitude
        // );

        self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
        Timer::after_millis(500).await;

        let (out_cw_a, out_cw_b) = self.run_sweep(1.).await;
        let (out_ccw_a, out_ccw_b) = self.run_sweep(-1.).await;

        let ecc_a = (out_cw_a + out_ccw_a) / 2.0;
        let ecc_b = (out_cw_b + out_ccw_b) / 2.0;

        debug!(
            "encoder fit: CW: a={}, b={}, CCW: a={}, b={}, ECC: a={}, b={}",
            out_cw_a, out_cw_b, out_ccw_a, out_ccw_b, ecc_a, ecc_b
        );

        // self.disable();

        let mut lut = [0.0f32; 128];

        for i in 0..lut.len() {
            let angle = (i as f32 / lut.len() as f32) * crate::simplefoc::types::_2PI;
            lut[i] = ecc_a * libm::cosf(angle) + ecc_b * libm::sinf(angle);
            // lut[i] = ecc_a * libm::sinf(angle) + ecc_b * libm::cosf(angle);
        }

        self.encoder.set_calibration_lut(lut);

        // unimplemented!()
    }

    #[cfg(feature = "nope")]
    async fn run_sweep(&mut self, direction: f32) -> (f32, f32) {
        let mut expected = heapless::Vec::<f32, 2_000>::new();
        let mut measured = heapless::Vec::<f32, 2_000>::new();

        let _ = self.encoder.update(Instant::now().as_micros()).await;

        // self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
        // Timer::after_millis(500).await;

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let angle0 = self.encoder.get_angle();

        let n = 2_000;
        // make motor rotate one full mechanical revolution (2PI rad) forward
        for i in 0..n {
            let shaft_angle = crate::simplefoc::types::_2PI * (i as f32 / n as f32);
            let mut electrical_angle =
                self.sensor_direction.multiplier() * shaft_angle * self.motor.pole_pairs as f32;

            if direction < 0.0 {
                electrical_angle =
                    super::types::_2PI * self.motor.pole_pairs as f32 - electrical_angle;
            }

            self.set_phase_voltage(self.motor.voltage_sensor_align, 0., electrical_angle);
            Timer::after_micros(1000).await;
            let t_us = Instant::now().as_micros();
            let _ = self.encoder.update(t_us).await;

            let measured_angle = self.encoder.get_angle() - angle0;
            expected.push(shaft_angle * direction.signum()).unwrap();
            measured.push(measured_angle).unwrap();
        }

        // Find average angular offset using Circular Mean
        let mut sum_sin = 0.;
        let mut sum_cos = 0.;
        for i in 0..n {
            let diff = measured[i] - expected[i];
            sum_sin += libm::sinf(diff);
            sum_cos += libm::cosf(diff);
        }
        let offset = libm::atan2f(sum_sin, sum_cos);

        // Extract the 1st Harmonic (Fourier Transform)
        let mut sum_a = 0.;
        let mut sum_b = 0.;
        for i in 0..n {
            // let expected = expected[i] + offset;
            // let mut err = measured[i] - expected;
            let reference = expected[i] + offset;
            let mut err = measured[i] - reference;

            // while(err >  PI) err -= 2.0f * PI;
            // while(err < -PI) err += 2.0f * PI;
            while err > core::f32::consts::PI {
                err -= crate::simplefoc::types::_2PI;
            }
            while err < -core::f32::consts::PI {
                err += crate::simplefoc::types::_2PI;
            }

            // sum_a += err * libm::cosf(measured[i]);
            // sum_b += err * libm::sinf(measured[i]);
            sum_a += err * libm::cosf(reference);
            sum_b += err * libm::sinf(reference);
        }

        // Multiply by (2/N) to get Fourier amplitude coefficients
        let out_a = (2. / n as f32) * sum_a;
        let out_b = (2. / n as f32) * sum_b;

        (out_a, out_b)
    }

    /// https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/calibrated/CalibratedSensor.cpp
    // #[cfg(feature = "nope")]
    pub async fn calibrate_encoder(&mut self) {
        use crate::hardware::encoder_sensor::N_LUT;

        let mut avg_elec_angle = 0.0;
        let mut elec_angle = 0.0;

        let align_voltage = self.motor.voltage_sensor_align;

        // Calibration parameters
        // The motor will take a n_pos samples per electrical cycle
        // which amounts to n_ticks (n_pos * motor.pole_pairs) samples per mechanical rotation
        // Additionally, the motor will take n2_ticks steps to reach any of the n_ticks posiitons
        // incrementing the electrical angle by deltaElectricalAngle each time
        let n_pos = 10;
        let n_ticks = n_pos * self.motor.pole_pairs as usize;
        let n2_ticks = 10;
        let delta_electrical_angle = crate::simplefoc::types::_2PI * self.motor.pole_pairs as f32
            / (n_ticks as f32 * n2_ticks as f32);
        let mut error = [0f32; N_LUT];

        self.set_phase_voltage(align_voltage, 0., elec_angle);
        Timer::after_millis(1000).await;
        let _ = self.encoder.update(Instant::now().as_micros()).await;

        let theta_init = self.encoder.get_angle();
        let theta_absolute_init = self.encoder.get_mechanical_angle();

        let settle_time_ms = 10;

        // Start calibration
        // forwards

        let mut zero_angle_prev = 0.0;
        for i in 0..n_ticks {
            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle += delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
            }
            Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            error[i] = 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);

            // calculate the current electrical zero angle
            let zero_angle = (self.sensor_direction.multiplier()
                * self.encoder.get_mechanical_angle()
                * self.motor.pole_pairs as f32)
                - (elec_angle + crate::simplefoc::types::_PI_2);
            let mut zero_angle = Self::normalize_angle(zero_angle);

            // remove the 2PI jumps
            if zero_angle - zero_angle_prev > core::f32::consts::PI {
                zero_angle = zero_angle - crate::simplefoc::types::_2PI;
            } else if zero_angle - zero_angle_prev < -core::f32::consts::PI {
                zero_angle = zero_angle + crate::simplefoc::types::_2PI;
            }
            zero_angle_prev = zero_angle;
            avg_elec_angle += zero_angle / n_ticks as f32;
        }

        // backwards
        let mut zero_angle_prev = 0.0;
        // for (int i = n_ticks - 1; i >= 0; i--)
        for i in (0..n_ticks).rev() {
            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle -= delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
            }
            Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            error[i] += 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);

            // calculate the current electrical zero angle
            let zero_angle = (self.sensor_direction.multiplier()
                * self.encoder.get_mechanical_angle()
                * self.motor.pole_pairs as f32)
                - (elec_angle + crate::simplefoc::types::_PI_2);
            let mut zero_angle = Self::normalize_angle(zero_angle);

            // remove the 2PI jumps
            if zero_angle - zero_angle_prev > core::f32::consts::PI {
                zero_angle = zero_angle - crate::simplefoc::types::_2PI;
            } else if zero_angle - zero_angle_prev < -core::f32::consts::PI {
                zero_angle = zero_angle + crate::simplefoc::types::_2PI;
            }
            zero_angle_prev = zero_angle;
            avg_elec_angle += zero_angle / n_ticks as f32;
        }

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let theta_absolute_post = self.encoder.get_mechanical_angle();

        self.set_phase_voltage(0., 0., 0.);

        // raw offset from initial position in absolute radians between 0-2PI
        let raw_offset = (theta_absolute_init - theta_absolute_post) / 2.;

        // calculating the average zero electrical angle from the forward calibration.
        let zero_electric_angle = Self::normalize_angle(avg_elec_angle / 2.);

        // Perform filtering to linearize position sensor eccentricity
        // FIR n-sample average, where n = number of samples in one electrical cycle
        // This filter has zero gain at electrical frequency and all integer multiples
        // So cogging effects should be completely filtered out
        let error_mean = self.filter_error(&mut error, n_ticks, n_pos);

        // calculate offset index
        let index_offset =
            libm::floorf(N_LUT as f32 * raw_offset / crate::simplefoc::types::_2PI) as usize;
        let dn = n_ticks as f32 / N_LUT as f32;

        let mut calibration_lut: [f32; N_LUT] = [0.0; N_LUT];

        // Build Look Up Table
        for i in 0..N_LUT {
            let mut ind =
                index_offset as i32 + i as i32 * self.sensor_direction.multiplier() as i32;
            if ind > N_LUT as i32 - 1 {
                ind -= N_LUT as i32;
            } else if ind < 0 {
                ind += N_LUT as i32;
            }
            calibration_lut[ind as usize] = error[i * dn as usize] - error_mean;
            calibration_lut[ind as usize] =
                self.sensor_direction.multiplier() as f32 * calibration_lut[ind as usize];
        }

        debug!("Calibration LUT: {:?}", calibration_lut);
        self.encoder.set_calibration_lut(calibration_lut);
    }

    // #[cfg(feature = "nope")]
    fn filter_error(
        &self,
        error: &mut [f32; crate::hardware::encoder_sensor::N_LUT],
        n_ticks: usize,
        window: usize,
    ) -> f32 {
        use crate::hardware::encoder_sensor::N_LUT;
        let mut window_buf = heapless::Vec::<f32, 256>::new();

        let mut window_sum = 0.;
        let mut buffer_index = 0;

        for i in 0..window {
            window_buf.push(0.0).unwrap();
        }

        for i in 0..window {
            let ind = n_ticks + window / 2 - 1 + i;
            window_buf[i] = error[ind % n_ticks];
            window_sum += window_buf[i];
        }

        let mut error_mean = 0.0;
        for i in 0..n_ticks {
            window_sum -= window_buf[buffer_index];
            window_buf[buffer_index] = error[(i + window / 2) % n_ticks];
            window_sum += window_buf[buffer_index];
            buffer_index = (buffer_index + 1) % window;

            error[i] = window_sum / window as f32;
            error_mean += error[i] / n_ticks as f32;
        }

        error_mean
    }

    async fn align_current_sensor(&mut self) {
        unimplemented!()
    }

    pub fn _set_phase_pwm(&mut self, duty_a: f32, duty_b: f32, duty_c: f32) {
        self.pwm_driver.set_duty_cycles_f32(duty_a, duty_b, duty_c);
    }

    // Method using FOC to set Uq and Ud to the motor at the optimal angle
    // Function implementing Space Vector PWM and Sine PWM algorithms
    //
    // Function using sine approximation
    // regular sin + cos ~300us    (no memory usage)
    // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
    pub(super) fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
        // debug!(
        //     "Setting phase voltage: Uq: {}, Ud: {}, Electrical angle: {}",
        //     uq, ud, angle_el
        // );

        // Sinusoidal PWM modulation
        // Inverse Park + Clarke transformation
        let sa = libm::sinf(angle_el);
        let ca = libm::cosf(angle_el);

        // // Inverse park transform
        let u_alpha = ca * ud - sa * uq; // -sin(angle) * Uq;
        let u_beta = sa * ud + ca * uq; //  cos(angle) * Uq;

        // Clarke transform
        self.phase_v.a = u_alpha;
        self.phase_v.b = -0.5 * u_alpha + super::types::_SQRT3_2 * u_beta;
        self.phase_v.c = -0.5 * u_alpha - super::types::_SQRT3_2 * u_beta;

        let mut center = self.pwm_driver.voltage_limit / 2.0;

        // if (foc_modulation == FOCModulationType::SpaceVectorPWM){
        //     // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        //     // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        //     // Midpoint Clamp
        //     float Umin = min(Ua, min(Ub, Uc));
        //     float Umax = max(Ua, max(Ub, Uc));
        //     center -= (Umax+Umin) / 2;
        // }

        if self.modulation == FOCModulation::SpaceVectorPWM {
            // Space Vector PWM modulation
            let umin = self.phase_v.a.min(self.phase_v.b.min(self.phase_v.c));
            let umax = self.phase_v.a.max(self.phase_v.b.max(self.phase_v.c));
            center = center - (umax + umin) / 2.0;
        }

        let modulation_centered = true; // default
        // let modulation_centered = false;

        if !modulation_centered {
            let umin = self.phase_v.a.min(self.phase_v.b.min(self.phase_v.c));
            self.phase_v.a -= umin;
            self.phase_v.b -= umin;
            self.phase_v.c -= umin;
        } else {
            self.phase_v.a += center;
            self.phase_v.b += center;
            self.phase_v.c += center;
        }

        self.pwm_driver
            .set_duty_cycles_f32(self.phase_v.a, self.phase_v.b, self.phase_v.c);
    }

    /// shaft velocity in rad/s
    pub(super) fn get_shaft_velocity(&mut self, t_us: u64) -> f32 {
        self.sensor_direction.multiplier()
            * self
                .lpf_velocity
                .filter_with_timestamp(self.encoder.get_velocity(), t_us)
    }

    /// shaft angle in rad
    pub(super) fn get_shaft_angle(&mut self) -> f32 {
        let angle = self.encoder.get_angle();

        let angle =
            self.sensor_direction.multiplier() * self.lpf_angle.filter(angle) - self.sensor_offset;

        angle
    }

    pub fn set_zero_angle(&mut self) {
        // self.sensor_offset = self.get_shaft_angle();
    }

    // pub fn get_mechanical_angle(&mut self) -> f32 {
    // }

    pub(super) fn get_electrical_angle(&mut self) -> f32 {
        let shaft_angle = self.encoder.get_mechanical_angle();
        let angle = self.sensor_direction.multiplier() * self.motor.pole_pairs as f32 * shaft_angle
            - self.zero_electric_angle;

        // (Self::normalize_angle(angle), shaft_angle)
        Self::normalize_angle(angle)
    }
}

/// helpers
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub fn estimate_back_emf(&self, shaft_velocity: f32) -> f32 {
        match self.motor.motor_kv {
            Some(kv) => shaft_velocity / (kv * super::types::_SQRT3) / super::types::_RPM_TO_RADS,
            None => 0.0,
        }
    }

    /// normalizing radian angle to [0,2PI]
    fn normalize_angle(angle: f32) -> f32 {
        let angle = angle % (2.0 * core::f32::consts::PI);
        if angle >= 0.0 {
            angle
        } else {
            angle + 2.0 * core::f32::consts::PI
        }
    }

    // #[cfg(feature = "nope")]
    pub(super) fn velocity_openloop(
        &mut self,
        target: f32,
        // shaft_angle: f32,
        voltage_bemf: f32,
        // electrical_angle: f32,
    ) -> f32 {
        let now_us = Instant::now().as_micros();

        let t_us = (now_us - self.motor.openloop_ts) as f32 * 1e-6;

        if t_us <= 0.0 || t_us > 0.5 {
            self.motor.openloop_ts = now_us;
            return 0.0;
        }

        // // calculate the necessary angle to achieve target velocity
        // shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
        // // for display purposes
        // shaft_velocity = target_velocity;

        self.openloop_shaft_angle = Self::normalize_angle(
            self.openloop_shaft_angle + self.motor.target_shaft_velocity * t_us,
        );

        let mut uq = self.motor.limit_voltage;
        if let Some(phase_resistance) = self.motor.phase_resistance {
            uq = (self.motor.limit_current * phase_resistance + voltage_bemf.abs())
                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

            // // recalculate the current
            self.motor.current.q = (uq - voltage_bemf.abs()) / phase_resistance;
        }

        // set the maximal allowed voltage (voltage_limit) with the necessary angle
        // setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
        // self.set_phase_voltage(uq, 0., electrical_angle);
        self.set_phase_voltage(
            uq,
            0.,
            self.openloop_shaft_angle * self.motor.pole_pairs as f32,
        );

        self.motor.openloop_ts = now_us;

        uq
    }
}
