use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};

use crate::{
    hardware::{as5600::AS5600, mt_6701::MT6701},
    simplefoc::{
        bldc::BLDCMotor,
        foc_types::{FOCStatus, SimpleFOC},
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{MotionControlType, NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

/// debug
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    pub async fn update_sensor(&mut self) {
        if let Err(_e) = self.encoder.update(Instant::now().as_micros()).await {
            error!("Failed to update encoder");
            unimplemented!()
        }
    }

    pub fn debug_encoder(
        &mut self,
    ) -> &mut AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>> {
        &mut self.encoder
    }
}

/// control, info
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    pub fn enable(&mut self) {
        self.enabled = true;
        self.enable_pin.set_high();
        self.pwm_driver.enable();
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.enable_pin.set_low();
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

    pub fn get_position_actual(&self) -> f32 {
        self.encoder.get_position() as f32
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

/// update, internal
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    pub fn init(&mut self) {
        // check driver initialized

        self.motor_status = FOCStatus::MotorInitializing;

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

        // velocity control loop controls current
        self.pid_velocity.limit = self.motor.limit_current;

        // self.pid_angle.limit = self.motor.limit_velocity;

        self.motor_status = FOCStatus::MotorReady;
    }

    pub async fn init_foc(&mut self) {
        // self.pid_current_q.limit = self.motor.limit_voltage;
        // self.pid_current_d.limit = self.motor.limit_voltage;

        // needs phase resistance set
        self.pid_velocity.limit = self.motor.limit_current;
        // self.pid_angle.limit = self.motor.limit_velocity;

        self.motor_status = FOCStatus::MotorCalibrating;

        // align motor if necessary
        // alignment necessary for encoders!
        // sensor and motor alignment - can be skipped
        // by setting motor.sensor_direction and motor.zero_electric_angle
        self.align_sensor().await;
        // self.motor.shaft_angle

        //
    }

    async fn align_sensor(&mut self) {
        // let _ = self.encoder.update(Instant::now().as_micros()).await;
        // Timer::after_millis(1).await;

        if self.sensor_direction == SensorDirection::Unknown {
            self.enable();

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
            let mid_angle = self.encoder.get_position();

            info!(
                "Mid angle: {}, now rotating backwards to find natural direction...",
                mid_angle
            );
            // move one electrical revolution backwards
            for i in 0..n {
                let i = n - 1 - i;
                let angle = crate::simplefoc::types::_3PI_2
                    + crate::simplefoc::types::_2PI * (i as f32) / n as f32;
                self.set_phase_voltage(self.motor.voltage_sensor_align, 0., angle);
                let _ = self.encoder.update(Instant::now().as_micros()).await;

                Timer::after_millis(2).await;
            }

            Timer::after_millis(20).await;

            let _ = self.encoder.update(Instant::now().as_micros()).await;
            let end_angle = self.encoder.get_position();

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
            } else if mid_angle > end_angle {
                self.sensor_direction = SensorDirection::Normal;
                info!("Sensor direction: Normal");
            } else {
                self.sensor_direction = SensorDirection::Inverted;
                info!("Sensor direction: Reversed");
            }

            // error!("TODO: implement sensor alignment procedure to determine sensor direction");

            self.disable();
            // panic!()
            //
        }

        #[cfg(feature = "nope")]
        {
            self.enable();

            let n = 50;

            info!("Rotating motor forward to find natural direction...");
            for i in 0..n {
                let angle = crate::simplefoc::types::_3PI_2
                    + crate::simplefoc::types::_2PI * (i as f32) / n as f32;
                self.set_phase_voltage(self.motor.voltage_sensor_align, 0., angle);
                let _ = self.encoder.update(Instant::now().as_micros()).await;

                Timer::after_millis(2).await;
            }
        }

        // zero electric angle not known
        // warn!("Skipping sensor alignment for testing");
        // #[cfg(feature = "nope")]
        if self.zero_electric_angle == NOT_SET {
            // align the electrical phases of the motor and sensor
            // set angle -90(270 = 3PI/2) degrees

            info!("Aligning sensor, rotating motor to known angle...");

            // self.motor.voltage_sensor_align = 0.5;
            self.motor.voltage_sensor_align = 1.0;

            self.enable();

            self.set_phase_voltage(
                self.motor.voltage_sensor_align,
                0.,
                crate::simplefoc::types::_3PI_2,
            );

            info!("Waiting for sensor to align...");

            Timer::after_millis(700).await;

            info!("Sensor aligned, setting zero electric angle...");

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

        self.disable();
    }

    /// main loop
    pub async fn update_foc(&mut self) {
        trace!("Updating FOC control loop");

        // update sensor readings
        if let Err(_e) = self.encoder.update(Instant::now().as_micros()).await {
            error!("Failed to update encoder");
            unimplemented!()
        }

        // let (electrical_angle, shaft_angle) = self.get_electrical_angle().await;
        let electrical_angle = self.get_electrical_angle();
        let shaft_angle = self.get_shaft_angle();
        trace!(
            "Electrical angle: {}, Shaft angle: {}",
            electrical_angle, shaft_angle
        );
        let shaft_velocity = self.get_shaft_velocity();
        trace!("Shaft velocity: {}", shaft_velocity);

        if !self.enabled {
            trace!("FOC is disabled, skipping control update");
            return;
        }

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // nothing to do
            } // _ => { unimplemented!() }
        }

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = shaft_velocity
            / (self.motor.motor_kv * super::types::_SQRT3)
            / super::types::_RPM_TO_RADS;

        trace!("Estimated back-EMF voltage: {}", voltage_bemf);

        // estimate the motor current if phase reistance available and current_sense not available
        self.motor.current.q = (self.motor.voltage.q - voltage_bemf) / self.motor.phase_resistance;

        match self.motion_control {
            MotionControlType::Torque => {
                if self.torque_controller == TorqueControlType::Voltage {
                    // voltage.q =  target*phase_resistance + voltage_bemf;
                    // voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);

                    self.motor.voltage.q =
                        (self.motor.target_current * self.motor.phase_resistance + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

                    match self.motor.phase_inductance {
                        Some(phase_inductance) => {
                            // voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
                            self.motor.voltage.d = (-self.motor.target_current
                                * shaft_velocity
                                * (self.motor.pole_pairs as f32)
                                * phase_inductance)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => {
                            self.motor.voltage.d = 0.0;
                        }
                    }
                }
            }
            MotionControlType::Velocity => {
                self.motor.target_current = self
                    .pid_velocity
                    .update(self.motor.target_shaft_velocity - shaft_velocity);

                if self.torque_controller == TorqueControlType::Voltage {
                    self.motor.voltage.q =
                        (self.motor.target_current * self.motor.phase_resistance + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                    // voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
                    self.motor.voltage.d = 0.;
                }

                if self.debug {
                    debug!(
                        "target V: {}, shaft V: {}, V error: {}",
                        self.motor.target_shaft_velocity,
                        shaft_velocity,
                        self.motor.target_shaft_velocity - shaft_velocity
                    );
                    debug!(
                        "Voltage: Uq: {}, Ud: {}",
                        self.motor.voltage.q, self.motor.voltage.d
                    );
                }
            }
            MotionControlType::Angle => {
                // calculate velocity set point
                self.motor.target_shaft_velocity = self
                    .pid_angle
                    .update(self.motor.target_shaft_angle - shaft_angle);
                // .clamp(-self.motor.limit_velocity, self.motor.limit_velocity);

                // calculate the torque command - sensor precision: this calculation is ok,
                // but based on bad value from previous calculation
                // current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
                self.motor.target_current = self
                    .pid_velocity
                    .update(self.motor.target_shaft_velocity - shaft_velocity);

                if self.debug {
                    debug!(
                        "target angle: {}, shaft angle: {}, angle error: {}, target velocity: {}, shaft velocity: {}, velocity error: {}, target current: {}",
                        self.motor.target_shaft_angle,
                        shaft_angle,
                        self.motor.target_shaft_angle - shaft_angle,
                        self.motor.target_shaft_velocity,
                        shaft_velocity,
                        self.motor.target_shaft_velocity - shaft_velocity,
                        self.motor.target_current,
                    );
                }

                if self.torque_controller == TorqueControlType::Voltage {
                    self.motor.voltage.q =
                        (self.motor.target_current * self.motor.phase_resistance + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

                    match self.motor.phase_inductance {
                        Some(phase_inductance) => {
                            self.motor.voltage.d = (-self.motor.target_current
                                * shaft_velocity
                                * (self.motor.pole_pairs as f32)
                                * phase_inductance)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => {
                            self.motor.voltage.d = 0.0;
                        }
                    }
                }
            }
            MotionControlType::VelocityOpenLoop => {
                self.motor.voltage.q = self.velocity_openloop(
                    self.motor.target_shaft_velocity,
                    // shaft_angle,
                    voltage_bemf,
                    // electrical_angle,
                );
                self.motor.voltage.d = 0.0;

                if self.debug {
                    debug!(
                        "target V: {}, shaft V: {}, V error: {}",
                        // self.motor.target_shaft_velocity,
                        (self.motor.target_shaft_velocity / crate::simplefoc::types::_2PI),
                        (shaft_velocity / crate::simplefoc::types::_2PI),
                        ((self.motor.target_shaft_velocity - shaft_velocity)
                            / crate::simplefoc::types::_2PI)
                    );
                    // debug!(
                    //     "Voltage: Uq: {}, Ud: {}",
                    //     self.motor.voltage.q, self.motor.voltage.d
                    // );
                }
            }
        }

        match self.motion_control {
            MotionControlType::VelocityOpenLoop => {}
            _ => {
                self.set_phase_voltage(
                    self.motor.voltage.q,
                    self.motor.voltage.d,
                    electrical_angle,
                );
            }
        }

        self.debug = false;
    }

    // Method using FOC to set Uq and Ud to the motor at the optimal angle
    // Function implementing Space Vector PWM and Sine PWM algorithms
    //
    // Function using sine approximation
    // regular sin + cos ~300us    (no memory usage)
    // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
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

        let center = self.pwm_driver.voltage_limit / 2.0;

        // if (!modulation_centered)

        self.phase_v.a += center;
        self.phase_v.b += center;
        self.phase_v.c += center;

        self.pwm_driver
            .set_duty_cycles_f32(self.phase_v.a, self.phase_v.b, self.phase_v.c);
    }

    fn get_shaft_velocity(&mut self) -> f32 {
        let dir = if self.sensor_direction == SensorDirection::Normal {
            1.0
        } else if self.sensor_direction == SensorDirection::Inverted {
            -1.0
        } else {
            0.0
        };

        dir * self.lpf_velocity.filter(self.encoder.get_velocity())
    }

    fn get_shaft_angle(&mut self) -> f32 {
        // let angle = self.encoder.get_angle();
        let angle = self.encoder.get_position();

        // convert from 12 bit to float
        let angle = (angle as f32) * 2.0 * core::f32::consts::PI / 4096.0;

        let angle = self.lpf_angle.filter(angle) - self.sensor_offset;

        angle
    }

    /// returns both electrical and shaft angle
    fn get_electrical_angle(&mut self) -> f32 {
        // _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
        // Self::normalize_angle()

        let dir = if self.sensor_direction == SensorDirection::Normal {
            1.0
        } else if self.sensor_direction == SensorDirection::Inverted {
            -1.0
        } else {
            0.0
        };

        // let shaft_angle = self.get_shaft_angle().await;
        let shaft_angle = self.encoder.get_angle();
        let angle = dir * self.motor.pole_pairs as f32 * shaft_angle - self.zero_electric_angle;

        // (Self::normalize_angle(angle), shaft_angle)
        Self::normalize_angle(angle)
    }
}

/// helpers
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    /// normalizing radian angle to [0,2PI]
    fn normalize_angle(angle: f32) -> f32 {
        let angle = angle % (2.0 * core::f32::consts::PI);
        if angle >= 0.0 {
            angle
        } else {
            angle + 2.0 * core::f32::consts::PI
        }
    }

    fn velocity_openloop(
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

        let uq = (self.motor.limit_current * self.motor.phase_resistance + voltage_bemf.abs())
            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

        // // recalculate the current
        self.motor.current.q = (uq - voltage_bemf.abs()) / self.motor.phase_resistance;

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
