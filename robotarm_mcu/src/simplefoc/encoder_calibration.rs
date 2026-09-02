use defmt::{debug, error, info, trace, warn};
use embassy_time::{Duration, Instant, Timer};

use crate::{
    hardware::{
        current_sensor::CurrentSensor,
        encoder_sensor::{EncoderSensor, N_LUT, N_LUT_SAMPLES},
    },
    simplefoc::{foc_types::SimpleFOC, types::_2PI},
};

pub fn apply_calibration_lut(raw_angle: f32, lut: &[f32; N_LUT]) -> f32 {
    // 1. Ensure raw_angle is within [0, 2π) limits efficiently
    let mut norm_angle = raw_angle;
    while norm_angle < 0.0 {
        norm_angle += _2PI;
    }
    while norm_angle >= _2PI {
        norm_angle -= _2PI;
    }

    // 2. Scale the normalized angle to a LUT index
    let lut_idx_f = (norm_angle / _2PI) * (N_LUT as f32);
    let idx_int = lut_idx_f as usize;
    let frac = lut_idx_f - (idx_int as f32);

    // 3. Get adjacent LUT values with integer wrap-around
    let idx0 = idx_int % N_LUT;
    let idx1 = (idx0 + 1) % N_LUT;

    // 4. Linearly interpolate the compensation value
    let err0 = lut[idx0];
    let err1 = lut[idx1];
    let compensation = err0 + frac * (err1 - err0);

    // 5. Apply the error (Assuming error was calculated as: Expected - Measured)
    let mut compensated = norm_angle + compensation;

    // 6. Final safety bounds wrap
    if compensated < 0.0 {
        compensated += _2PI;
    } else if compensated >= _2PI {
        compensated -= _2PI;
    }

    compensated
}

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
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

    pub async fn test_calibration(&mut self) {
        let mut expected = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();
        let mut measured_uncalibrated = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();
        let mut measured_calibrated = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();

        let _ = self.encoder.update(Instant::now().as_micros()).await;

        self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
        Timer::after_millis(500).await;

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        // let angle0 = self.encoder.get_angle();
        let angle0 = self.encoder.read_raw_debug().await.unwrap();

        let lut = self.encoder.get_encoder_calibration_lut().unwrap();

        for i in 0..N_LUT_SAMPLES {
            let shaft_angle = _2PI * (i as f32 / N_LUT_SAMPLES as f32);
            let mut electrical_angle =
                self.sensor_direction.multiplier() * shaft_angle * self.motor.pole_pairs as f32;

            // if direction < 0.0 {
            //     electrical_angle =
            //         super::types::_2PI * self.motor.pole_pairs as f32 - electrical_angle;
            // }

            self.set_phase_voltage(self.motor.voltage_sensor_align, 0., electrical_angle);
            Timer::after_micros(1000).await;
            // let t_us = Instant::now().as_micros();
            // let _ = self.encoder.update(t_us).await;

            let measured_raw = self.encoder.read_raw_debug().await.unwrap();
            let angle = (measured_raw as f32 / 16384_f32) * _2PI;

            let angle_calibrated = apply_calibration_lut(angle, &lut);

            expected.push(shaft_angle).unwrap();
            measured_uncalibrated.push(angle).unwrap();
            measured_calibrated.push(angle_calibrated).unwrap();

            // let measured_angle = self.encoder.get_angle() - angle0;
            // expected.push(shaft_angle * direction.signum()).unwrap();
            // measured.push(measured_angle).unwrap();
        }

        let mut avg_calibrated = 0.0;
        let mut avg_uncalibrated = 0.0;

        for i in 0..measured_uncalibrated.len() {
            avg_uncalibrated += expected[i] - measured_uncalibrated[i];
            avg_calibrated += expected[i] - measured_calibrated[i];

            // self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
            //     id: self.id,
            //     // timestamp: t_us,
            //     start_new: i == 0,
            //     data: [
            //         expected[i],
            //         measured_uncalibrated[i],
            //         measured_calibrated[i],
            //         expected[i] - measured_uncalibrated[i],
            //         expected[i] - measured_calibrated[i],
            //         0.0,
            //         0.0,
            //         0.0,
            //     ],
            // })
            // .await;
        }

        debug!(
            "avg error uncalibrated: {}",
            avg_uncalibrated / measured_uncalibrated.len() as f32
        );
        debug!(
            "avg error calibrated: {}",
            avg_calibrated / measured_calibrated.len() as f32
        );

        self.set_phase_voltage(0., 0., 0.);
    }

    #[cfg(feature = "nope")]
    pub async fn calibrate_encoder(&mut self) {
        // self.set_motion_control(MotionControlType::VelocityOpenLoop);

        self.motor.voltage_sensor_align = 3.0;

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

        let _ = self.encoder.update(Instant::now().as_micros()).await;

        let theta_absolute_init = self.encoder.get_mechanical_angle();

        let ((out_cw_a, out_cw_b), offset_cw, (expected_cw, measured_cw)) =
            self.run_sweep(1.).await;
        let ((out_ccw_a, out_ccw_b), offset_ccw, (expected_ccw, measured_ccw)) =
            self.run_sweep(-1.).await;

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let theta_absolute_post = self.encoder.get_mechanical_angle();

        self.set_phase_voltage(0., 0., 0.);

        let ecc_a = (out_cw_a + out_ccw_a) / 2.0;
        let ecc_b = (out_cw_b + out_ccw_b) / 2.0;

        // debug!(
        //     "encoder fit: CW: a={}, b={}, CCW: a={}, b={}, ECC: a={}, b={}",
        //     out_cw_a, out_cw_b, out_ccw_a, out_ccw_b, ecc_a, ecc_b
        // );

        debug!("offset: CW: {}, CCW: {}", offset_cw, offset_ccw);

        // let mut expected_cw2 = [0f32; N_LUT_SAMPLES];
        // expected_cw2.copy_from_slice(&expected_cw);

        // let mut measured_cw2 = [0f32; N_LUT_SAMPLES];
        // measured_cw2.copy_from_slice(&measured_cw);

        // let mut expected_ccw2 = [0f32; N_LUT_SAMPLES];
        // expected_ccw2.copy_from_slice(&expected_ccw);

        // let mut measured_ccw2 = [0f32; N_LUT_SAMPLES];
        // measured_ccw2.copy_from_slice(&measured_ccw);

        // debug!("expected_cw: \n{:?}", expected_cw2);
        // debug!("measured_cw: \n{:?}", measured_cw2);
        // // debug!("expected_ccw: \n{:?}", expected_ccw2);
        // // debug!("measured_ccw: \n{:?}", measured_ccw2);

        // self.disable();

        // let mut error = [0f32; N_LUT_SAMPLES];

        let raw_offset = (theta_absolute_init - theta_absolute_post) / 2.;
        debug!("raw offset: {}", raw_offset);

        // use crate::hardware::encoder_sensor::N_LUT;

        // let index_offset = libm::floorf(
        //     crate::hardware::encoder_sensor::N_LUT as f32 * raw_offset
        //         / crate::simplefoc::types::_2PI,
        // ) as usize;
        // debug!("index offset: {}", index_offset);

        // let mut window = N_LUT_SAMPLES / N_LUT;

        #[cfg(feature = "nope")]
        for i in 0..lut.len() {
            let mut ind =
                index_offset as i32 + i as i32 * self.sensor_direction.multiplier() as i32;
            if ind > N_LUT as i32 - 1 {
                ind -= N_LUT as i32;
            } else if ind < 0 {
                ind += N_LUT as i32;
            }

            let samples_cw = &measured_cw[i..i + window];

            lut[ind as usize] = self.sensor_direction.multiplier() as f32;
        }

        let mut lut2 = [0.0f32; N_LUT];
        for i in 0..lut2.len() {
            let angle = (i as f32 / lut2.len() as f32) * crate::simplefoc::types::_2PI;
            lut2[i] = ecc_a * libm::cosf(angle) + ecc_b * libm::sinf(angle);
            // lut2[i] = out_ccw_a * libm::cosf(angle) + out_ccw_b * libm::sinf(angle);
        }

        let mut errors_calibrated_ccw = [0f32; N_LUT_SAMPLES];
        let mut errors_calibrated_cw = [0f32; N_LUT_SAMPLES];
        let mut errors_calibrated_ccw2 = [0f32; N_LUT_SAMPLES];
        let mut errors_calibrated_cw2 = [0f32; N_LUT_SAMPLES];
        let mut errors_uncalibrated_ccw = [0f32; N_LUT_SAMPLES];
        let mut errors_uncalibrated_cw = [0f32; N_LUT_SAMPLES];

        #[cfg(feature = "nope")]
        {
            let mut error_ccw = [0f32; crate::hardware::encoder_sensor::N_LUT_SAMPLES];
            let mut error_cw = [0f32; crate::hardware::encoder_sensor::N_LUT_SAMPLES];

            for i in 0..error_ccw.len() {
                error_ccw[i] = measured_ccw[i] - expected_ccw[i];
                error_cw[i] = measured_cw[i] - expected_cw[i];
            }

            let error_mean_ccw = self.filter_error2(&expected_ccw, &measured_ccw, &mut error_ccw);
            let error_mean_cw = self.filter_error2(&expected_cw, &measured_cw, &mut error_cw);

            let mut overall_mean = 0.0;
            let mut error = [0f32; crate::hardware::encoder_sensor::N_LUT_SAMPLES];
            for i in 0..error.len() {
                error[i] = (error_ccw[i] + error_cw[i]) / 2.0;
                overall_mean += error[i] / error.len() as f32;
            }

            debug!("error mean CCW: {}", error_mean_ccw);
            debug!("error mean CW: {}", error_mean_cw);

            let mut lut = [0.0f32; N_LUT];
            for i in 0..N_LUT {
                let index = (i as f32 / N_LUT as f32) * N_LUT_SAMPLES as f32;
                let index_i = index as usize;
                let index_f = index - index_i as f32;

                let idx0 = index_i % N_LUT_SAMPLES;
                let idx1 = (idx0 + 1) % N_LUT_SAMPLES;

                let err0 = (error_cw[idx0] + error_ccw[idx0]) / 2.0;
                let err1 = (error_cw[idx1] + error_ccw[idx1]) / 2.0;

                let interp_error = err0 + index_f * (err1 - err0);

                lut[i] = interp_error - overall_mean;
            }

            self.encoder.set_calibration_lut(lut);

            let mut errors_calibrated_ccw = [0f32; N_LUT_SAMPLES];
            let mut errors_calibrated_cw = [0f32; N_LUT_SAMPLES];
            let mut errors_calibrated_ccw2 = [0f32; N_LUT_SAMPLES];
            let mut errors_calibrated_cw2 = [0f32; N_LUT_SAMPLES];
            let mut errors_uncalibrated_ccw = [0f32; N_LUT_SAMPLES];
            let mut errors_uncalibrated_cw = [0f32; N_LUT_SAMPLES];

            debug!("expected_ccw.len(): {}", expected_ccw.len());

            for i in 0..expected_ccw.len() {
                let calibrated_angle_ccw = apply_calibration_lut(measured_ccw[i], &lut);
                let calibrated_angle_cw = apply_calibration_lut(measured_cw[i], &lut);

                let calibrated_angle_ccw2 = apply_calibration_lut(measured_ccw[i], &lut2);
                let calibrated_angle_cw2 = apply_calibration_lut(measured_cw[i], &lut2);

                errors_calibrated_ccw[i] = calibrated_angle_ccw - expected_ccw[i];
                errors_calibrated_cw[i] = calibrated_angle_cw - expected_cw[i];
                errors_calibrated_ccw2[i] = calibrated_angle_ccw2 - expected_ccw[i];
                errors_calibrated_cw2[i] = calibrated_angle_cw2 - expected_cw[i];
                errors_uncalibrated_ccw[i] = measured_ccw[i] - expected_ccw[i];
                errors_uncalibrated_cw[i] = measured_cw[i] - expected_cw[i];

                #[cfg(feature = "nope")]
                if i == 0 {
                    debug!("expected_ccw[0]: {}", expected_ccw[0]);
                    debug!("measured_ccw[0]: {}", measured_ccw[0]);
                    debug!("calibrated_ccw[0]: {}", calibrated_angle_ccw);
                    debug!("calibrated_ccw2[0]: {}", calibrated_angle_ccw2);

                    debug!("expected_cw[0]: {}", expected_cw[0]);
                    debug!("measured_cw[0]: {}", measured_cw[0]);
                    debug!("calibrated_cw[0]: {}", calibrated_angle_cw);
                    debug!("calibrated_cw2[0]: {}", calibrated_angle_cw2);
                }

                self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
                    id: self.id,
                    // timestamp: t_us,
                    start_new: i == 0,
                    data: [
                        expected_ccw[i],
                        expected_cw[i],
                        measured_ccw[i],
                        measured_cw[i],
                        0.,
                        0.,
                        // errors_uncalibrated_ccw[i],
                        // errors_uncalibrated_cw[i],
                        // errors_calibrated_ccw[i],
                        // errors_calibrated_cw[i],
                        // errors_calibrated_ccw2[i],
                        // errors_calibrated_cw2[i],
                        0.,
                        0.,
                    ],
                })
                .await;

                Timer::after_micros(1000).await;
            }

            let avg_error_ccw_uncalibrated =
                errors_uncalibrated_ccw.iter().sum::<f32>() / errors_uncalibrated_ccw.len() as f32;
            let avg_error_cw_uncalibrated =
                errors_uncalibrated_cw.iter().sum::<f32>() / errors_uncalibrated_cw.len() as f32;
            let avg_error_ccw_calibrated =
                errors_calibrated_ccw.iter().sum::<f32>() / errors_calibrated_ccw.len() as f32;
            let avg_error_cw_calibrated =
                errors_calibrated_cw.iter().sum::<f32>() / errors_calibrated_cw.len() as f32;
            let avg_error_ccw_calibrated2 =
                errors_calibrated_ccw2.iter().sum::<f32>() / errors_calibrated_ccw2.len() as f32;
            let avg_error_cw_calibrated2 =
                errors_calibrated_cw2.iter().sum::<f32>() / errors_calibrated_cw2.len() as f32;

            debug!(
                "Average error CCW (uncalibrated): {}",
                avg_error_ccw_uncalibrated
            );
            debug!(
                "Average error CW (uncalibrated):  {}",
                avg_error_cw_uncalibrated
            );

            debug!(
                "Average error CCW (calibrated):   {}",
                avg_error_ccw_calibrated,
            );
            debug!(
                "Average error CW (calibrated):    {}",
                avg_error_cw_calibrated,
            );

            debug!(
                "Average error CCW (calibrated2):  {}",
                avg_error_ccw_calibrated2
            );
            debug!(
                "Average error CW (calibrated2):   {}",
                avg_error_cw_calibrated2
            );
        }

        // for i in 0..measured_cw.len() {
        //     self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
        //         id: self.id,
        //         // timestamp: t_us,
        //         start_new: i == 0,
        //         data: [
        //             expected_ccw[i],
        //             measured_ccw[i],
        //             expected_cw[i],
        //             measured_cw[i],
        //             0.0,
        //             0.0,
        //             0.0,
        //             0.0,
        //         ],
        //     })
        //     .await;
        // }

        // self.encoder.set_calibration_lut(lut);

        // unimplemented!()
    }

    fn filter_error2(
        &self,
        expected: &heapless::Vec<f32, { crate::hardware::encoder_sensor::N_LUT_SAMPLES }>,
        measured: &heapless::Vec<f32, { crate::hardware::encoder_sensor::N_LUT_SAMPLES }>,
        error: &mut [f32; crate::hardware::encoder_sensor::N_LUT_SAMPLES],
    ) -> f32 {
        use crate::hardware::encoder_sensor::N_LUT_SAMPLES;

        // // Find average angular offset using Circular Mean
        // let mut sum_sin = 0.;
        // let mut sum_cos = 0.;
        // for i in 0..N_LUT_SAMPLES {
        //     let diff = measured[i] - expected[i];
        //     sum_sin += libm::sinf(diff);
        //     sum_cos += libm::cosf(diff);
        // }
        // let offset = libm::atan2f(sum_sin, sum_cos);
        // // debug!("offset: {}", offset);

        // average n samples around the point of interest, and choose
        // n such that the samples being averaged exactly span one electrical cycle

        let n_ticks = N_LUT_SAMPLES;

        let npp = self.motor.pole_pairs;
        let window = n_ticks / npp as usize;

        let mut window_buf = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();

        let mut window_sum = 0.;
        let mut buffer_index = 0;

        for i in 0..window {
            window_buf.push(0.0).unwrap();
        }

        for i in 0..window {
            let ind = n_ticks - window / 2 - 1 + i;
            window_buf[i] = error[ind % n_ticks];
            window_sum += window_buf[i];
        }

        let mut error_mean = 0.0;

        for i in 0..n_ticks {
            // // Update buffer
            window_sum -= window_buf[buffer_index];
            window_buf[buffer_index] = error[(i + window / 2) % n_ticks];
            window_sum += window_buf[buffer_index];
            // update the buffer index
            buffer_index = (buffer_index + 1) % window;

            // Update filtered error
            error[i] = window_sum / window as f32;
            // update the mean value
            error_mean += error[i] / n_ticks as f32;
        }

        // debug!("error mean: {}", error_mean);

        error_mean
    }

    #[cfg(feature = "nope")]
    async fn run_sweep(
        &mut self,
        direction: f32,
    ) -> (
        (f32, f32),
        f32,
        (
            heapless::Vec<f32, { N_LUT_SAMPLES }>,
            heapless::Vec<f32, { N_LUT_SAMPLES }>,
        ),
    ) {
        let mut expected = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();
        let mut measured = heapless::Vec::<f32, { N_LUT_SAMPLES }>::new();

        let _ = self.encoder.update(Instant::now().as_micros()).await;

        self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
        Timer::after_millis(500).await;

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let angle0 = self.encoder.get_angle();

        // let n = 2_000;
        // make motor rotate one full mechanical revolution (2PI rad) forward
        for i in 0..N_LUT_SAMPLES {
            let shaft_angle = crate::simplefoc::types::_2PI * (i as f32 / N_LUT_SAMPLES as f32);
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
        for i in 0..N_LUT_SAMPLES {
            let diff = measured[i] - expected[i];
            sum_sin += libm::sinf(diff);
            sum_cos += libm::cosf(diff);
        }
        let offset = libm::atan2f(sum_sin, sum_cos);

        // Extract the 1st Harmonic (Fourier Transform)
        let mut sum_a = 0.;
        let mut sum_b = 0.;
        for i in 0..N_LUT_SAMPLES {
            // let expected = expected[i] + offset;
            // let mut err = measured[i] - expected;
            let reference = expected[i] + offset;
            let mut err = measured[i] - reference;

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
        let out_a = (2. / N_LUT_SAMPLES as f32) * sum_a;
        let out_b = (2. / N_LUT_SAMPLES as f32) * sum_b;

        // if direction > 0. {
        //     expected.reverse();
        //     measured.reverse();
        // }

        ((out_a, out_b), offset, (expected, measured))
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
        let n_pos = 15;
        let n_ticks = n_pos * self.motor.pole_pairs as usize;
        let n2_ticks = 5;
        let delta_electrical_angle = crate::simplefoc::types::_2PI * self.motor.pole_pairs as f32
            / (n_ticks as f32 * n2_ticks as f32);
        let mut error = [0f32; N_LUT];

        self.set_phase_voltage(align_voltage, 0., elec_angle);
        Timer::after_millis(1000).await;
        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let _ = self.encoder.update(Instant::now().as_micros()).await;

        let theta_init = self.encoder.get_angle();
        let theta_absolute_init = self.encoder.get_mechanical_angle();

        // let settle_time_ms = 50;
        let settle_time_us = 50;

        let mut expected_cw2 = [0f32; N_LUT];
        let mut measured_cw2 = [0f32; N_LUT];
        let mut expected_ccw2 = [0f32; N_LUT];
        let mut measured_ccw2 = [0f32; N_LUT];

        // Start calibration
        // forwards

        let mut zero_angle_prev = 0.0;
        for i in 0..n_ticks {
            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle += delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                // Timer::after_micros(100).await;
                Timer::after_micros(settle_time_us).await;
            }
            // Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            let e = 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);
            error[i] = e;

            expected_ccw2[i] = elec_angle / self.motor.pole_pairs as f32;
            // measured_ccw2[i] = e;
            measured_ccw2[i] = theta_actual;

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

        // let theta_half = self.encoder.get_mechanical_angle();

        // backwards
        let mut zero_angle_prev = 0.0;
        // for (int i = n_ticks - 1; i >= 0; i--)
        for i in (0..n_ticks).rev() {
            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle -= delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                Timer::after_micros(settle_time_us).await;
            }
            // Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            let e = 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);
            error[i] += e;

            expected_cw2[i] = elec_angle / self.motor.pole_pairs as f32;
            measured_cw2[i] = e;

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

        // debug!("expected_cw: \n{:?}", expected_cw2);
        // debug!("measured_cw: \n{:?}", measured_cw2);
        // debug!("expected_ccw: {:?}", expected_ccw2);
        // debug!("measured_ccw: {:?}", measured_ccw2);

        self.set_phase_voltage(0., 0., 0.);

        // raw offset from initial position in absolute radians between 0-2PI
        let raw_offset = (theta_absolute_init - theta_absolute_post) / 2.;

        // let half_offset = theta_absolute_init - theta_half;
        // debug!("half offset: {}", half_offset);
        debug!("raw offset: {}", raw_offset);

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
        #[cfg(feature = "nope")]
        for i in 0..N_LUT {
            let mut ind =
                index_offset as i32 + i as i32 * self.sensor_direction.multiplier() as i32;
            if ind > N_LUT as i32 - 1 {
                ind -= N_LUT as i32;
            } else if ind < 0 {
                ind += N_LUT as i32;
            }
            calibration_lut[ind as usize] = error[(i as f32 * dn) as usize] - error_mean;
            calibration_lut[ind as usize] =
                self.sensor_direction.multiplier() as f32 * calibration_lut[ind as usize];
        }

        for i in 0..N_LUT {
            let mut ind =
                index_offset as i32 + i as i32 * self.sensor_direction.multiplier() as i32;
            if ind > N_LUT as i32 - 1 {
                ind -= N_LUT as i32;
            } else if ind < 0 {
                ind += N_LUT as i32;
            }

            let sample_pos = i as f32 * dn;
            let sample_floor = libm::floorf(sample_pos) as usize;
            let sample_frac = sample_pos - sample_floor as f32;

            let e0 = error[sample_floor % n_ticks];
            let e1 = error[(sample_floor + 1) % n_ticks];
            let sample = (1.0 - sample_frac) * e0 + sample_frac * e1;

            calibration_lut[ind as usize] =
                self.sensor_direction.multiplier() as f32 * (sample - error_mean);
        }

        for i in 0..n_ticks {
            let error_uncalibrated_ccw = measured_ccw2[i] - expected_ccw2[i];
            let error_uncalibrated_cw = measured_cw2[i] - expected_cw2[i];

            let angle_calibrated_ccw = apply_calibration_lut(measured_ccw2[i], &calibration_lut);
            let angle_calibrated_cw = apply_calibration_lut(measured_cw2[i], &calibration_lut);

            let error_calibrated_ccw = angle_calibrated_ccw - expected_ccw2[i];
            let error_calibrated_cw = angle_calibrated_cw - expected_cw2[i];

            self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
                id: self.id,
                // timestamp: t_us,
                start_new: i == 0,
                data: [
                    expected_ccw2[i],
                    measured_ccw2[i],
                    // expected_cw2[i],
                    // measured_cw2[i],
                    angle_calibrated_ccw,
                    error_uncalibrated_ccw,
                    error_calibrated_ccw,
                    0.0,
                    0.0,
                    0.0,
                ],
            })
            .await;
            Timer::after_micros(1000).await;
        }

        // debug!("Calibration LUT: \n{:?}", calibration_lut);
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
            let ind = n_ticks - window / 2 - 1 + i;
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
}

#[cfg(feature = "nope")]
mod prev {
    impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
        pub async fn calibrate_encoder(&mut self) {
            self.motor.voltage_sensor_align = 4.0;

            self.enable();

            let tick_hz = 1000;

            let mut ticker =
                embassy_time::Ticker::every(Duration::from_micros(1_000_000 / tick_hz));

            let mut cal = Calibrator::<128>::new(CalibratorConfig::new(
                self.motor.pole_pairs as u32,
                tick_hz as f32,
            ));

            self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
            Timer::after_millis(500).await;

            self.motor.voltage_sensor_align = 3.0;

            self.modulation = crate::simplefoc::foc_types::FOCModulation::SinePWM;

            let raw_angle = self.encoder.read_raw_debug().await.unwrap();
            match cal.poll(raw_angle) {
                CalStep::Drive(theta_e) => {
                    debug!("theta_e: {}", theta_e);
                    // set_open_loop_angle(theta_e, 4.0), // your existing inverse Park/Clarke
                    // self.set_phase_voltage(self.motor.voltage_sensor_align, 0., theta_e);
                }
                CalStep::Finished => {}
            }

            // loop {
            //     // let _ = self.encoder.update(Instant::now().as_micros()).await;
            //     // let raw_angle = self.encoder.get_angle();
            //     let raw_angle = self.encoder.read_raw_debug().await.unwrap();
            //     match cal.poll(raw_angle) {
            //         CalStep::Drive(theta_e) => {
            //             // set_open_loop_angle(theta_e, 4.0), // your existing inverse Park/Clarke
            //             self.set_phase_voltage(self.motor.voltage_sensor_align, 0., theta_e);
            //         }
            //         CalStep::Finished => break,
            //     }

            //     ticker.next().await;
            // }

            // let (expected, measured) = cal.results();
            // let mut lut = [0i16; 128];
            // build_lut(expected, measured, &mut lut);

            // debug!("LUT:\n{:?}", lut);

            // self.encoder.set_calibration_lut(lut);

            //
        }

        #[cfg(feature = "nope")]
        async fn run_sweep(&mut self, direction: f32) {
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

                // let measured_angle = self.encoder.get_angle() - angle0;
                // expected.push(shaft_angle * direction.signum()).unwrap();
                // measured.push(measured_angle).unwrap();
            }
        }
    }

    const BITS: u32 = 14;
    /// Counts per revolution of the encoder (16384 for a 14-bit encoder).
    const FULL_SCALE: i32 = 1 << BITS;

    /// Max number of calibration sample points supported (stack scratch buffer
    /// size used inside `build_lut`; bump if you calibrate with more points).
    // pub const MAX_CAL_SAMPLES: usize = 1024;
    pub const MAX_CAL_SAMPLES: usize = 16;

    /// Wrap `x` into `[0, FULL_SCALE)`.
    // #[inline]
    pub fn wrap_u(x: i32) -> u16 {
        x.rem_euclid(FULL_SCALE) as u16
    }

    /// Wrap `x` into `[-FULL_SCALE/2, FULL_SCALE/2)` — i.e. give the signed
    /// "shortest path" representation of a difference between two raw counts.
    // #[inline]
    pub fn wrap_signed(x: i32) -> i32 {
        let half = FULL_SCALE / 2;
        ((x + half).rem_euclid(FULL_SCALE)) - half
    }

    /// Unwrap a sequence of raw 14-bit samples, taken at monotonically
    /// increasing *expected* mechanical angle, into a monotonic `i32` sequence
    /// with the 0/16383 rollover resolved between consecutive samples. Each
    /// output sample is the input closest to the previous output sample (mod
    /// FULL_SCALE).
    fn unwrap_into(measured: &[u16], out: &mut [i32]) {
        // debug_assert_eq!(measured.len(), out.len());
        out[0] = measured[0] as i32;
        for i in 1..measured.len() {
            let prev = out[i - 1];
            let raw = measured[i] as i32;
            let delta = wrap_signed(raw - wrap_u(prev) as i32);
            out[i] = prev + delta;
        }
    }

    /// Apply a previously-built correction table to a raw encoder reading.
    /// O(1), integer-only, linearly interpolated between the two nearest table
    /// entries. Safe to call every control-loop tick.
    pub fn apply_lut(raw: u16, lut: &[i16]) -> u16 {
        let m = lut.len() as i32;
        let bin_width = FULL_SCALE / m;
        let pos = raw as i32;
        let j0 = (pos / bin_width) as usize % lut.len();
        let j1 = (j0 + 1) % lut.len();
        let frac = pos - (j0 as i32 * bin_width); // 0..bin_width

        let c0 = lut[j0] as i32;
        let c1 = lut[j1] as i32;
        // Two neighbouring corrections might be expressed on "opposite sides"
        // of the +-FULL_SCALE/2 wrap; take the short way between them.
        let dc = wrap_signed(c1 - c0);
        let correction = c0 + dc * frac / bin_width;

        wrap_u(pos + correction)
    }

    /// Build an `lut.len()`-entry correction table from `N` `(expected,
    /// measured)` sample pairs collected across exactly one mechanical
    /// revolution (see [`calibration::Calibrator`]).
    ///
    /// `expected[i]` must be uniformly spaced and monotonically increasing
    /// over `[0, FULL_SCALE)`, e.g. `expected[i] = i * FULL_SCALE / N`.
    ///
    /// The table is indexed by **raw encoder reading**, not by expected angle:
    /// `lut[j]` is the correction to *add* (mod FULL_SCALE) to a raw reading of
    /// `j * FULL_SCALE / lut.len()` to recover the true mechanical angle.
    /// Values are wrapped to `[-FULL_SCALE/2, FULL_SCALE/2)` and fit in an
    /// `i16`.
    ///
    /// Panics if `expected.len() != measured.len()` or that length exceeds
    /// [`MAX_CAL_SAMPLES`].
    pub fn build_lut(expected: &[u16], measured: &[u16], lut: &mut [i16]) {
        // assert_eq!(expected.len(), measured.len());
        let n = expected.len();
        // assert!(n >= 2 && n <= MAX_CAL_SAMPLES);
        let m = lut.len();
        // assert!(m >= 1);

        let mut measured_u = [0i32; MAX_CAL_SAMPLES];
        let measured_u = &mut measured_u[..n];
        unwrap_into(measured, measured_u);

        for j in 0..m {
            // Target raw reading this table entry corresponds to.
            let target = (j as i64 * FULL_SCALE as i64 / m as i64) as i32;

            // Find the calibration segment [i, i+1] (wrapping i+1 -> 0 at the
            // end) whose *measured* span brackets `target`, extending the last
            // segment's far endpoint by +FULL_SCALE so the wraparound segment
            // is handled the same way as every other segment.
            let mut seg = 0usize;
            for i in 0..n {
                let i1 = (i + 1) % n;
                let m0 = measured_u[i];
                let m1 = if i1 == 0 {
                    measured_u[i1] + FULL_SCALE
                } else {
                    measured_u[i1]
                };
                // bring `target` into the same winding as this segment
                let t = m0 + wrap_signed(target - wrap_u(m0) as i32);
                if t >= m0 && t <= m1 {
                    seg = i;
                    break;
                }
            }

            let i0 = seg;
            let i1 = (seg + 1) % n;
            let m0 = measured_u[i0];
            let m1 = if i1 == 0 {
                measured_u[i1] + FULL_SCALE
            } else {
                measured_u[i1]
            };
            let e0 = expected[i0] as i32;
            let e1 = if i1 == 0 {
                expected[i1] as i32 + FULL_SCALE
            } else {
                expected[i1] as i32
            };

            let t = m0 + wrap_signed(target - wrap_u(m0) as i32);
            let expected_interp = if m1 != m0 {
                e0 + ((e1 - e0) as i64 * (t - m0) as i64 / (m1 - m0) as i64) as i32
            } else {
                e0
            };

            lut[j] = wrap_signed(expected_interp - target) as i16;
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum State {
        Ramping,
        Settling,
        Sampling,
        Done,
    }

    /// What to do this tick.
    #[derive(Clone, Copy, Debug)]
    pub enum CalStep {
        /// Command this electrical angle (radians), at the voltage/current
        /// magnitude you configured, this tick.
        Drive(f32),
        /// All `N` sample points have been collected; call [`Calibrator::results`].
        Finished,
    }

    /// Tuning parameters. Defaults are conservative starting points — see the
    /// field docs for how to adjust them for your motor.
    #[derive(Clone, Copy, Debug)]
    pub struct CalibratorConfig {
        /// Motor pole pairs. One mechanical revolution = `pole_pairs` electrical
        /// revolutions.
        pub pole_pairs: u32,
        /// Largest electrical-angle step (radians) taken per tick while ramping
        /// toward the next sample point. Smaller = slower but less risk of the
        /// rotor skipping a step (losing sync with the applied field) on a
        /// low-friction gimbal motor. Start small (e.g. 0.02-0.05 rad/tick) and
        /// only increase once you've confirmed via the raw-encoder trace that
        /// motion is smooth and monotonic with no discontinuities.
        pub max_step_rad: f32,
        /// Ticks to sit still at each target angle before sampling, letting
        /// mechanical ringing die out. Gimbal motors are lightly damped, so err
        /// high initially (e.g. equivalent to 100-300 ms) and reduce once you
        /// see the encoder reading has stabilized well before this elapses.
        pub settle_ticks: u32,
        /// Consecutive encoder reads averaged together at each sample point, to
        /// reduce quantization/electrical noise.
        pub samples_to_average: u32,
    }

    impl CalibratorConfig {
        /// `ticks_per_sec` is the rate you intend to call `poll()` at — used
        /// only to convert the default settle time (200 ms) into a tick count.
        pub fn new(pole_pairs: u32, ticks_per_sec: f32) -> Self {
            Self {
                pole_pairs,
                max_step_rad: 0.03,
                settle_ticks: (0.1 * ticks_per_sec) as u32,
                samples_to_average: 16,
            }
        }
    }

    /// Open-loop calibration sequencer. `N` is the number of sample points
    /// spread evenly across one mechanical revolution (128 or 256 are good
    /// choices — see module docs).
    pub struct Calibrator<const N: usize> {
        cfg: CalibratorConfig,
        state: State,
        target_index: usize,
        current_elec_angle: f32,
        ticks_in_state: u32,
        ref_sample: i32,
        accum: i32,
        accum_count: u32,
        expected: [u16; N],
        measured: [u16; N],
    }

    impl<const N: usize> Calibrator<N> {
        pub fn new(cfg: CalibratorConfig) -> Self {
            Self {
                cfg,
                state: State::Ramping,
                target_index: 0,
                current_elec_angle: 0.0,
                ticks_in_state: 0,
                ref_sample: 0,
                accum: 0,
                accum_count: 0,
                expected: [0; N],
                measured: [0; N],
            }
        }

        fn target_electrical_angle(&self) -> f32 {
            let target_mech_frac = self.target_index as f32 / N as f32; // 0..1
            target_mech_frac * self.cfg.pole_pairs as f32 * 2.0 * core::f32::consts::PI
        }

        /// Call once per control-loop tick with the current raw encoder
        /// reading. Returns the electrical angle to drive this tick, or
        /// `Finished` once all `N` points are collected.
        pub fn poll(&mut self, raw_encoder: u16) -> CalStep {
            match self.state {
                State::Ramping => {
                    let target = self.target_electrical_angle();
                    let diff = target - self.current_elec_angle;
                    if libm::fabsf(diff) <= self.cfg.max_step_rad {
                        self.current_elec_angle = target;
                        self.state = State::Settling;
                        self.ticks_in_state = 0;
                    } else {
                        self.current_elec_angle += self.cfg.max_step_rad * diff.signum();
                    }
                    CalStep::Drive(self.current_elec_angle)
                }
                State::Settling => {
                    self.ticks_in_state += 1;
                    if self.ticks_in_state >= self.cfg.settle_ticks {
                        self.state = State::Sampling;
                        self.accum = 0;
                        self.accum_count = 0;
                    }
                    CalStep::Drive(self.current_elec_angle)
                }
                State::Sampling => {
                    // Mod-aware averaging: reference the first sample so we
                    // don't corrupt the average if the true angle sits right on
                    // the 0/16383 rollover.
                    let raw = raw_encoder as i32;
                    if self.accum_count == 0 {
                        self.ref_sample = raw;
                    } else {
                        self.accum += wrap_signed(raw - self.ref_sample);
                    }
                    self.accum_count += 1;

                    if self.accum_count >= self.cfg.samples_to_average {
                        let avg = wrap_u(self.ref_sample + self.accum / self.accum_count as i32);
                        let expected =
                            (self.target_index as u32 * FULL_SCALE as u32 / N as u32) as u16;
                        self.expected[self.target_index] = expected;
                        self.measured[self.target_index] = avg;

                        self.target_index += 1;
                        self.state = if self.target_index >= N {
                            State::Done
                        } else {
                            State::Ramping
                        };
                    }
                    CalStep::Drive(self.current_elec_angle)
                }
                State::Done => CalStep::Finished,
            }
        }

        /// The collected `(expected, measured)` sample pairs. Only meaningful
        /// once `poll` has returned `CalStep::Finished`.
        pub fn results(&self) -> (&[u16; N], &[u16; N]) {
            (&self.expected, &self.measured)
        }

        pub fn progress(&self) -> (usize, usize) {
            (self.target_index, N)
        }
    }
}
