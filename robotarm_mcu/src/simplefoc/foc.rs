use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};
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
        if let Some(logger) = &mut self.usb_logger {
            logger.send_log_msg(message);
        }
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
        self.motor.target_shaft_angle = position;
    }

    pub fn set_motion_control(&mut self, control_type: MotionControlType) {
        self.motion_control = control_type;
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
                info!("Sensor direction: Normal");
            } else {
                self.sensor_direction = SensorDirection::CCW;
                info!("Sensor direction: Reversed");
            }

            // self.sensor_direction = SensorDirection::Inverted;
            // self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);

            // error!("TODO: implement sensor alignment procedure to determine sensor direction");

            // self.disable();
            // panic!()
            //
        }

        // zero electric angle not known
        // warn!("Skipping sensor alignment for testing");
        // #[cfg(feature = "nope")]
        if self.zero_electric_angle == NOT_SET {
            // align the electrical phases of the motor and sensor
            // set angle -90(270 = 3PI/2) degrees

            info!("Aligning sensor, rotating motor to known angle...");

            // self.motor.voltage_sensor_align = 0.5;
            // self.motor.voltage_sensor_align = 1.0;
            self.motor.voltage_sensor_align = 2.0;

            // not sure why this is needed
            self.set_phase_voltage(self.motor.voltage_sensor_align, 0., 0.);
            Timer::after_millis(2).await;

            self.set_phase_voltage(
                self.motor.voltage_sensor_align,
                0.,
                crate::simplefoc::types::_3PI_2,
            );

            // info!("Waiting for sensor to align...");

            Timer::after_millis(700).await;

            // info!("Sensor aligned, setting zero electric angle...");

            // get the current zero electric angle
            self.zero_electric_angle = 0.;
            let electrical_angle = self.get_electrical_angle();
            self.zero_electric_angle = electrical_angle;

            // info!("Zero electric angle set to {}", self.zero_electric_angle);
            // info!("Shaft angle at alignment position: {}", _shaft_angle);

            Timer::after_millis(20).await;

            // // stop everything
            self.set_phase_voltage(0., 0., 0.);
            Timer::after_millis(200).await;
        }

        // self.disable();
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
        self.sensor_direction.multiplier() * self.lpf_velocity.filter(self.encoder.get_velocity())
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
