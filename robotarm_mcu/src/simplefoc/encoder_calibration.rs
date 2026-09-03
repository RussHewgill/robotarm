use defmt::{debug, error, info, trace, warn};
use embassy_time::{Duration, Instant, Timer};

use crate::{
    hardware::{
        current_sensor::CurrentSensor,
        encoder_sensor::{EncoderSensor, N_LUT, N_LUT_SAMPLES},
    },
    simplefoc::{foc::normalize_angle, foc_types::SimpleFOC, types::_2PI},
};

pub fn apply_calibration_lut(raw_angle: f32, lut: &[f32; N_LUT]) -> f32 {
    const LUT_RESOLUTION: f32 = _2PI / N_LUT as f32;
    const LUT_RESOLUTION_INV: f32 = 1. / LUT_RESOLUTION;

    // if (raw_angle < 0 || raw_angle >= _2PI) raw_angle = _normalizeAngle(raw_angle);
    let raw_angle = if raw_angle < 0.0 || raw_angle >= _2PI {
        let angle = raw_angle % (2.0 * core::f32::consts::PI);
        if angle >= 0.0 {
            angle
        } else {
            angle + 2.0 * core::f32::consts::PI
        }
    } else {
        raw_angle
    };

    // Calculate LUT index
    let mut lut_index = (raw_angle * LUT_RESOLUTION_INV) as usize;
    // let lut_index = (raw_angle / LUT_RESOLUTION) as usize;

    // if lut_index == 256 {
    //     lut_index = 0;
    // }

    // Get calibration values from the LUT
    let lut_entry_lower = lut[lut_index];
    // let lut_entry_higher = calibrationLut[ lut_index >= n_lut-1 ? 0 : (lut_index + 1)];
    let lut_entry_higher = lut[if lut_index >= N_LUT - 1 {
        0
    } else {
        lut_index + 1
    }];

    // Linearly interpolate between the two closest LUT entries (one lower and one higher than the raw angle)
    // Calculate the distance between the raw angle and the lower LUT entry
    // Distance is normalized to [0,1]
    let lut_lower_angle = lut_index as f32 * LUT_RESOLUTION;
    let distance_lower = (raw_angle - lut_lower_angle) * LUT_RESOLUTION_INV;
    // Linearly interpolate between lower and higher LUT entries
    let correction_offset =
        (1.0 - distance_lower) * lut_entry_lower + distance_lower * lut_entry_higher;

    // Calculate the calibrated angle
    return raw_angle - correction_offset;
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

        self.encoder.enable_calibration(false);

        let _ = self.encoder.update(Instant::now().as_micros()).await;

        self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
        Timer::after_millis(500).await;

        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let angle0 = self.encoder.get_mechanical_angle();
        // let angle0 = self.encoder.read_raw_debug().await.unwrap();
        // let angle0 = (angle0 as f32 / 16384_f32) * _2PI;
        debug!("angle0: {}", angle0);

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
            Timer::after_micros(2000).await;
            // let t_us = Instant::now().as_micros();
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // let measured_raw = self.encoder.read_raw_debug().await.unwrap();
            // let angle = (measured_raw as f32 / 16384_f32) * _2PI;
            let angle1 = self.encoder.get_angle();
            // let angle = self.encoder.get_mechanical_angle();

            let angle = normalize_angle(angle1);

            let angle_calibrated = normalize_angle(apply_calibration_lut(angle, &lut) - angle0);

            expected.push(shaft_angle).unwrap();
            measured_uncalibrated
                .push(normalize_angle(angle1 - angle0))
                .unwrap();
            measured_calibrated.push(angle_calibrated).unwrap();

            // let measured_angle = self.encoder.get_angle() - angle0;
            // expected.push(shaft_angle * direction.signum()).unwrap();
            // measured.push(measured_angle).unwrap();
        }

        let mut avg_calibrated = 0.0;
        let mut avg_uncalibrated = 0.0;

        for i in 0..measured_uncalibrated.len() {
            // avg_uncalibrated += expected[i] - measured_uncalibrated[i];
            // avg_calibrated += expected[i] - measured_calibrated[i];

            self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
                id: self.id,
                // timestamp: t_us,
                start_new: i == 0,
                data: [
                    expected[i],
                    measured_uncalibrated[i],
                    measured_calibrated[i],
                    expected[i] - measured_uncalibrated[i],
                    expected[i] - measured_calibrated[i],
                    0.0,
                    0.0,
                    0.0,
                ],
            })
            .await;
            Timer::after_micros(200).await;
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

        let mut errors_ccw = [0f32; N_LUT_SAMPLES];
        let mut errors_cw = [0f32; N_LUT_SAMPLES];

        for i in 0..errors_ccw.len() {
            errors_ccw[i] = measured_ccw[i] - expected_ccw[i];
            // errors_cw[i] = measured_cw[N_LUT_SAMPLES - 1 - i] - expected_cw[N_LUT_SAMPLES - 1 - i];
            errors_cw[i] = measured_cw[i] - expected_cw[i];
        }

        // let mut overall_mean = 0.0;
        // let mut error = [0f32; crate::hardware::encoder_sensor::N_LUT_SAMPLES];
        // for i in 0..error.len() {
        //     error[i] = (errors_ccw[i] + errors_cw[i]) / 2.0;
        //     overall_mean += error[i] / error.len() as f32;
        // }

        let mean_ccw = errors_ccw.iter().sum::<f32>() / errors_ccw.len() as f32;
        let mean_cw = errors_cw.iter().sum::<f32>() / errors_cw.len() as f32;

        let mut lut = [0.0f32; N_LUT];
        // #[cfg(feature = "nope")]
        for i in 0..N_LUT {
            let index = (i as f32 / N_LUT as f32) * N_LUT_SAMPLES as f32;
            let index_i = index as usize;
            let index_f = index - index_i as f32;

            let idx0 = index_i % N_LUT_SAMPLES;
            let idx1 = (idx0 + 1) % N_LUT_SAMPLES;

            // let err0 = (errors_cw[idx0] + errors_ccw[idx0]) / 2.0;
            // let err1 = (errors_cw[idx1] + errors_ccw[idx1]) / 2.0;

            let err0 = errors_ccw[idx0];
            let err1 = errors_ccw[idx1];

            let interp_error = err0 + index_f * (err1 - err0);

            // lut[i] = interp_error - overall_mean;
            lut[i] = interp_error - mean_ccw;

            if i == 50 {
                debug!("expected_ccw[{}]: {}", idx0, expected_ccw[idx0]);
                debug!("expected_ccw[{}]: {}", idx1, expected_ccw[idx1]);
                debug!("measured_ccw[{}]: {}", idx0, measured_ccw[idx0]);
                debug!("measured_ccw[{}]: {}", idx1, measured_ccw[idx1]);
                debug!("err0 = {}", err0);
                debug!("err1 = {}", err1);
                debug!("interp_error = {}", interp_error);
                debug!("mean_ccw = {}", mean_ccw);
            }
        }

        debug!("expected_ccw[20]: {}", expected_ccw[20]);
        debug!("expected_cw[20]: {}", expected_cw[20]);
        debug!("measured_ccw[20]: {}", measured_ccw[20]);
        debug!("measured_cw[20]: {}", measured_cw[20]);

        for i in 0..N_LUT_SAMPLES {
            let error_ccw = measured_ccw[i] - expected_ccw[i];
            let error_cw = measured_cw[i] - expected_cw[i];

            let corrected_ccw = apply_calibration_lut(measured_ccw[i], &lut);
            let corrected_error_ccw = corrected_ccw - expected_ccw[i];

            let corrected_cw = apply_calibration_lut(measured_cw[i], &lut);
            let corrected_error_cw = corrected_cw - expected_cw[i];

            self.send_debug_message(robotarm_protocol::SerialLogMessage::LogData {
                id: self.id,
                // timestamp: t_us,
                start_new: i == 0,
                data: [
                    expected_ccw[i],
                    measured_ccw[i],
                    // errors_ccw[i],
                    // errors_cw[i],
                    // corrected_ccw,
                    error_ccw.abs(),
                    corrected_error_ccw.abs(),
                    expected_cw[i],
                    measured_cw[i],
                    error_cw.abs(),
                    corrected_error_cw.abs(),
                    // 0.0,
                    // 0.0,
                    // 0.0,
                    // 0.0,
                ],
            })
            .await;
            Timer::after_micros(1000).await;
        }

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
        // let raw_angle = (raw_angle as f32 / 16384_f32) * _2PI;

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
            Timer::after_micros(5000).await;
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

        if direction < 0. {
            expected.reverse();
            measured.reverse();
        }

        ((out_a, out_b), offset, (expected, measured))
    }

    /// https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/calibrated/CalibratedSensor.cpp
    // #[cfg(feature = "nope")]
    pub async fn calibrate_encoder(&mut self) {
        use crate::hardware::encoder_sensor::N_LUT;

        let mut avg_elec_angle = 0.0;
        let mut elec_angle = 0.0;

        // let align_voltage = self.motor.voltage_sensor_align;
        let align_voltage = 3.0;

        // Calibration parameters
        // The motor will take a n_pos samples per electrical cycle
        // which amounts to n_ticks (n_pos * motor.pole_pairs) samples per mechanical rotation
        // Additionally, the motor will take n2_ticks steps to reach any of the n_ticks posiitons
        // incrementing the electrical angle by deltaElectricalAngle each time
        let n_pos = 10;
        let n_ticks = n_pos * self.motor.pole_pairs as usize;
        let n2_ticks = 10;
        let delta_electrical_angle =
            _2PI * self.motor.pole_pairs as f32 / (n_ticks as f32 * n2_ticks as f32);
        let mut errors = [0f32; N_LUT];

        self.set_phase_voltage(align_voltage, 0., elec_angle);
        Timer::after_millis(1000).await;
        let _ = self.encoder.update(Instant::now().as_micros()).await;
        let _ = self.encoder.update(Instant::now().as_micros()).await;

        // let theta_init = self.sensor_direction.multiplier() * self.encoder.get_angle();
        let theta_init = self.encoder.get_angle();
        let theta_absolute_init = self.encoder.get_mechanical_angle();

        debug!("theta_init:     {}", theta_init);
        // debug!("theta_absolute_init: {}", theta_absolute_init);

        #[cfg(feature = "nope")]
        {
            let settle_us = 10_000;

            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle += delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                // Timer::after_micros(100).await;
                Timer::after_micros(settle_us).await;
            }

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let theta = self.sensor_direction.multiplier() * self.encoder.get_angle();
            let expected_angle = elec_angle / self.motor.pole_pairs as f32
                - theta_init * self.sensor_direction.multiplier();
            debug!("elec_angle:     {}", elec_angle);
            debug!("theta:          {}", theta);
            debug!("expected_angle: {}", expected_angle);
            debug!("error:          {}", theta - expected_angle);
            debug!("");

            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle -= delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                // Timer::after_micros(100).await;
                Timer::after_micros(settle_us).await;
            }

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let theta = self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            let expected_angle = elec_angle / self.motor.pole_pairs as f32;
            debug!("elec_angle:     {}", elec_angle);
            debug!("theta:          {}", theta);
            debug!("expected_angle: {}", expected_angle);
            debug!("error:          {}", theta - expected_angle);
            debug!("");

            for j in 0..n2_ticks {
                let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle -= delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                // Timer::after_micros(100).await;
                Timer::after_micros(settle_us).await;
            }

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let theta = self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
            let expected_angle = elec_angle / self.motor.pole_pairs as f32;
            debug!("elec_angle:     {}", elec_angle);
            debug!("theta:          {}", theta);
            debug!("expected_angle: {}", expected_angle);
            debug!("error:          {}", theta - expected_angle);

            return;
        }

        // let settle_time_ms = 50;
        let settle_time_us = 10_000;

        let mut expected_cw2 = [0f32; N_LUT];
        let mut measured_cw2 = [0f32; N_LUT];
        let mut expected_ccw2 = [0f32; N_LUT];
        let mut measured_ccw2 = [0f32; N_LUT];

        // Start calibration
        // forwards

        let mut zero_angle_prev = 0.0;
        for i in 0..n_ticks {
            for j in 0..n2_ticks {
                // let _ = self.encoder.update(Instant::now().as_micros()).await;
                elec_angle += delta_electrical_angle;
                self.set_phase_voltage(align_voltage, 0., elec_angle);
                // Timer::after_micros(100).await;
                Timer::after_micros(settle_time_us).await;
            }
            // Timer::after_micros(settle_time_us).await;
            // Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                // self.sensor_direction.multiplier() * self.encoder.get_angle() - theta_init;
                self.sensor_direction.multiplier() * (self.encoder.get_angle() - theta_init);
            let e = 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);
            errors[i] = e;
            // errors[i] = -2. * e;

            expected_ccw2[i] = elec_angle / self.motor.pole_pairs as f32;
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

        Timer::after_millis(200).await;

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
            // Timer::after_micros(settle_time_us).await;
            // Timer::after_millis(settle_time_ms).await;
            let _ = self.encoder.update(Instant::now().as_micros()).await;

            // calculate error
            let theta_actual =
                self.sensor_direction.multiplier() * (self.encoder.get_angle() - theta_init);
            let e = 0.5 * (theta_actual - elec_angle / self.motor.pole_pairs as f32);
            errors[i] += e;
            // errors[i] += -2. * e;

            expected_cw2[i] = elec_angle / self.motor.pole_pairs as f32;
            measured_cw2[i] = theta_actual;

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

        // debug!("delta_electrical_angle: {}", delta_electrical_angle);

        // let i = 50;
        // let i = 10;
        // debug!("expected_ccw[{}]: {}", i, expected_ccw2[i]);
        // debug!("measured_ccw[{}]: {}", i, measured_ccw2[i]);
        // debug!("measured_ccw[{}]: {}", i + 1, measured_ccw2[i + 1]);
        // debug!("expected_cw[{}]: {}", i, expected_cw2[i]);
        // debug!("measured_cw[{}]: {}", i, measured_cw2[i]);
        // debug!("measured_cw[{}]: {}", i - 1, measured_cw2[i - 1]);

        // debug!("expected_cw: \n{:?}", expected_cw2);
        // debug!("measured_cw: \n{:?}", measured_cw2);
        // debug!("expected_ccw: {:?}", expected_ccw2);
        // debug!("measured_ccw: {:?}", measured_ccw2);

        self.set_phase_voltage(0., 0., 0.);

        // raw offset from initial position in absolute radians between 0-2PI
        let raw_offset = (theta_absolute_init + theta_absolute_post) / 2.;

        // let half_offset = theta_absolute_init - theta_half;
        // debug!("half offset: {}", half_offset);
        debug!("raw offset: {}", raw_offset);

        // calculating the average zero electrical angle from the forward calibration.
        let zero_electric_angle = Self::normalize_angle(avg_elec_angle / 2.);

        let errors2 = errors.clone();
        // Perform filtering to linearize position sensor eccentricity
        // FIR n-sample average, where n = number of samples in one electrical cycle
        // This filter has zero gain at electrical frequency and all integer multiples
        // So cogging effects should be completely filtered out
        let error_mean = self.filter_error(&mut errors, n_ticks, n_pos);

        debug!("error mean: {}", error_mean);

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

        #[cfg(feature = "nope")]
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

        for i in 0..N_LUT {
            let mut ind =
                index_offset as i32 + i as i32 * self.sensor_direction.multiplier() as i32;
            if ind > N_LUT as i32 - 1 {
                ind -= N_LUT as i32;
            } else if ind < 0 {
                ind += N_LUT as i32;
            }
            // let mut ind = i;

            // let sample_pos = i as f32 * dn;
            // let sample_floor = libm::floorf(sample_pos) as usize;
            // let sample_frac = sample_pos - sample_floor as f32;

            // debug!("ind = {}", ind);

            // let error0 = expected_ccw2[ind as usize] - measured_ccw2[ind as usize];
            // let error1 = expected_ccw2[(ind as usize + 1) % n_ticks]
            //     - measured_ccw2[(ind as usize + 1) % n_ticks];

            // let error = (1.0 - sample_frac) * error0 + sample_frac * error1;

            // let error = expected_ccw2[ind as usize] - measured_ccw2[ind as usize];
            let sample_index = (i as f32 * dn) as usize;
            // let error = (expected_ccw2[sample_index] - measured_ccw2[sample_index]) - error_mean;
            let error = errors[sample_index] - error_mean;
            // let error = errors[sample_index];
            let error = error * self.sensor_direction.multiplier() as f32;

            calibration_lut[ind as usize] = error;

            // let error = expected_ccw2[(ind as usize + 1) % n_ticks]
            //     - measured_ccw2[(ind as usize + 1) % n_ticks];

            // calibration_lut[ind as usize] = self.sensor_direction.multiplier() as f32 * error;

            // calibration_lut[ind as usize] =
            //     self.sensor_direction.multiplier() as f32 * (error - error_mean);

            // #[cfg(feature = "nope")]
        }

        #[cfg(feature = "nope")]
        {
            let i = 22;
            let error_uncalibrated_ccw = measured_ccw2[i] - expected_ccw2[i];
            let angle_calibrated_ccw = apply_calibration_lut(measured_ccw2[i], &calibration_lut);
            let error_calibrated_ccw = angle_calibrated_ccw - expected_ccw2[i];

            debug!("expected_ccw[{}]:           {}", i, expected_ccw2[i]);
            debug!("measured_ccw[{}]:           {}", i, measured_ccw2[i]);
            debug!("angle_calibrated_ccw[{}]:   {}", i, angle_calibrated_ccw);
            debug!("error_uncalibrated_ccw[{}]: {}", i, error_uncalibrated_ccw);
            debug!("error_calibrated_ccw[{}]:   {}", i, error_calibrated_ccw);
        }

        // let _ = self.filter_error(&mut expected_ccw2, n_ticks, n_pos);
        // let _ = self.filter_error(&mut measured_ccw2, n_ticks, n_pos);
        // let _ = self.filter_error(&mut expected_cw2, n_ticks, n_pos);
        // let _ = self.filter_error(&mut measured_cw2, n_ticks, n_pos);

        #[cfg(feature = "nope")]
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
                    angle_calibrated_ccw,
                    error_uncalibrated_ccw,
                    error_calibrated_ccw,
                    // errors2[i],
                    // errors[i],
                    // expected_cw2[i],
                    // measured_cw2[i],
                    // angle_calibrated_cw,
                    // error_uncalibrated_cw,
                    // error_calibrated_cw,
                    // 0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            })
            .await;
            Timer::after_micros(1000).await;
        }

        debug!("Calibration LUT: \n{:?}", calibration_lut);

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
