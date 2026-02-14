use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};
use robotarm_protocol::{SerialCommand, SerialLogMessage};

use crate::{
    hardware::{as5600::AS5600, encoder_sensor::EncoderSensor, mt_6701::MT6701},
    simplefoc::{
        bldc::BLDCMotor,
        foc_types::{FOCStatus, SimpleFOC},
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{MotionControlType, NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

/// debug
impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
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
impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
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

    // pub fn get_position_actual(&self) -> f32 {
    //     self.encoder.get_position() as f32
    // }

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
impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
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
        self.pid_velocity.set_limit(self.motor.limit_current);

        // self.pid_angle.limit = self.motor.limit_velocity;

        self.motor_status = FOCStatus::MotorReady;
    }

    pub async fn init_foc(&mut self) {
        // self.pid_current_q.limit = self.motor.limit_voltage;
        // self.pid_current_d.limit = self.motor.limit_voltage;

        // needs phase resistance set
        self.pid_velocity.set_limit(self.motor.limit_current);
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
                // let _ = self.encoder.update(Instant::now().as_micros()).await;

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
                // let _ = self.encoder.update(Instant::now().as_micros()).await;

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
                self.sensor_direction = SensorDirection::Normal;
                info!("Sensor direction: Normal");
            } else {
                self.sensor_direction = SensorDirection::Inverted;
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

        self.disable();
    }

    /// main loop
    pub async fn update_foc(&mut self) {
        // trace!("Updating FOC control loop");

        let t_us = Instant::now().as_micros();

        if t_us - self.prev_debug_us >= self.debug_us_interval() {
            self.debug = true;
            self.prev_debug_us = t_us;
        } else {
            self.debug = false;
        }

        // update sensor readings
        if let Err(_e) = self.encoder.update(t_us).await {
            error!("Failed to update encoder");
        }

        if !self.enabled {
            return;
        }

        let electrical_angle = self.get_electrical_angle();
        if !self.motion_control.is_open_loop() {
            self.set_phase_voltage(self.motor.voltage.q, self.motor.voltage.d, electrical_angle);
        }

        // let (electrical_angle, shaft_angle) = self.get_electrical_angle().await;
        let electrical_angle = self.get_electrical_angle();
        let shaft_angle = self.get_shaft_angle();
        // trace!(
        //     "Electrical angle: {}, Shaft angle: {}",
        //     electrical_angle, shaft_angle
        // );
        let shaft_velocity = self.get_shaft_velocity(t_us);
        // trace!("Shaft velocity: {}", shaft_velocity);

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // nothing to do
            } // _ => { unimplemented!() }
        }

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = match self.motor.motor_kv {
            Some(kv) => shaft_velocity / (kv * super::types::_SQRT3) / super::types::_RPM_TO_RADS,

            None => 0.0,
        };

        // trace!("Estimated back-EMF voltage: {}", voltage_bemf);

        if let Some(phase_resistance) = self.motor.phase_resistance {
            // estimate the motor current if phase reistance available and current_sense not available
            self.motor.current.q = (self.motor.voltage.q - voltage_bemf) / phase_resistance;
        }

        match self.motion_control {
            MotionControlType::Torque => {
                if self.torque_controller == TorqueControlType::Voltage {
                    // voltage.q =  target*phase_resistance + voltage_bemf;
                    // voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);

                    if let Some(phase_resistance) = self.motor.phase_resistance {
                        self.motor.voltage.q = (self.motor.target_current * phase_resistance
                            + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                    } else {
                        self.motor.voltage.q = self.motor.target_current;
                    }

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

                if self.debug {
                    let rpm = shaft_velocity * 30.0 / core::f32::consts::PI;

                    let kv = rpm / self.motor.voltage.q;

                    debug!(
                        "KV Calculation: voltage: {}, velocity (rad/s): {}, velocity (RPM), KV: {}",
                        self.motor.voltage.q,
                        shaft_velocity,
                        //
                        kv
                    )
                }
            }
            MotionControlType::Velocity => {
                if let Some(tuner) = &mut self.pid_velocity_tuner {
                    if !tuner.done() {
                        self.motor.target_current = tuner.update(shaft_velocity, t_us);
                    } else {
                        self.pid_velocity_tuner = None;
                    }
                } else {
                    self.motor.target_current = self.pid_velocity.update(
                        self.motor.target_shaft_velocity,
                        shaft_velocity,
                        t_us,
                    );
                }

                if self.torque_controller == TorqueControlType::Voltage {
                    match self.motor.phase_resistance {
                        None => self.motor.voltage.q = 0.0,
                        Some(phase_resistance) => {
                            self.motor.voltage.q = (self.motor.target_current * phase_resistance
                                + voltage_bemf)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                    }
                    // self.motor.voltage.d = 0.;
                }

                #[cfg(feature = "nope")]
                if self.debug {
                    debug!(
                        "target V: {}, shaft V: {}, V error: {}, angle: {}",
                        self.motor.target_shaft_velocity,
                        shaft_velocity,
                        self.motor.target_shaft_velocity - shaft_velocity,
                        // self.motor.target_current,
                        // self.motor.voltage.q,
                        shaft_angle,
                    );
                    // debug!(
                    //     "Voltage: Uq: {}, Ud: {}",
                    //     self.motor.voltage.q, self.motor.voltage.d
                    // );
                }
            }
            MotionControlType::Angle => {
                // calculate velocity set point
                self.motor.target_shaft_velocity =
                    self.pid_angle
                        .update(self.motor.target_shaft_angle, shaft_angle, t_us);
                // .clamp(-self.motor.limit_velocity, self.motor.limit_velocity);

                // calculate the torque command - sensor precision: this calculation is ok,
                // but based on bad value from previous calculation
                // current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
                self.motor.target_current = self.pid_velocity.update(
                    self.motor.target_shaft_velocity,
                    shaft_velocity,
                    t_us,
                );

                #[cfg(feature = "nope")]
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
                    match self.motor.phase_resistance {
                        Some(phase_resistance) => {
                            self.motor.voltage.q = (self.motor.target_current * phase_resistance
                                + voltage_bemf)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => self.motor.voltage.q = self.motor.target_current,
                    }

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

                #[cfg(feature = "nope")]
                if self.debug {
                    debug!(
                        "target V: {}, shaft V: {}, V error: {}, angle: {}",
                        // self.motor.target_shaft_velocity,
                        libm::roundf(
                            (self.motor.target_shaft_velocity / crate::simplefoc::types::_2PI)
                                * 100.
                        ) / 100.,
                        // (shaft_velocity / crate::simplefoc::types::_2PI),
                        libm::roundf((shaft_velocity / crate::simplefoc::types::_2PI) * 100.)
                            / 100.,
                        // ((self.motor.target_shaft_velocity - shaft_velocity) / crate::simplefoc::types::_2PI)
                        libm::roundf(
                            ((self.motor.target_shaft_velocity - shaft_velocity)
                                / crate::simplefoc::types::_2PI)
                                * 100.
                        ) / 100.,
                        // libm::roundf((shaft_angle / crate::simplefoc::types::_2PI) * 100.) / 100.,
                        // shaft_angle,
                        self.encoder.get_mechanical_angle()
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

        if self.debug {
            #[cfg(feature = "nope")]
            self.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
                id: 0,
                timestamp: t_us,
                position: shaft_angle,
                angle: self.encoder.get_mechanical_angle(),
                velocity: shaft_velocity,
                target_position: self.motor.target_shaft_angle,
                target_velocity: self.motor.target_shaft_velocity,
                motor_current: self.motor.current.q,
                motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
            })
            .await;

            self.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
                id: 0,
                timestamp: t_us,
                // position: 3.,
                position: shaft_angle,
                // angle: 4.,
                angle: self.encoder.get_mechanical_angle(),
                // velocity: 5.,
                velocity: shaft_velocity,
                // target_position: 6.,
                target_position: self.motor.target_shaft_angle,
                // target_velocity: 7.,
                target_velocity: self.motor.target_shaft_velocity,
                // motor_current: 8.,
                motor_current: self.motor.current.q,
                // motor_voltage: (9., 10.),
                motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
            })
            .await;
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

        let mut center = self.pwm_driver.voltage_limit / 2.0;

        // if (foc_modulation == FOCModulationType::SpaceVectorPWM){
        //     // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        //     // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        //     // Midpoint Clamp
        //     float Umin = min(Ua, min(Ub, Uc));
        //     float Umax = max(Ua, max(Ub, Uc));
        //     center -= (Umax+Umin) / 2;
        // }

        if false {
            // Space Vector PWM modulation
            let umin = self.phase_v.a.min(self.phase_v.b.min(self.phase_v.c));
            let umax = self.phase_v.a.max(self.phase_v.b.max(self.phase_v.c));
            center = center - (umax + umin) / 2.0;
        }

        // if (!modulation_centered)

        self.phase_v.a += center;
        self.phase_v.b += center;
        self.phase_v.c += center;

        self.pwm_driver
            .set_duty_cycles_f32(self.phase_v.a, self.phase_v.b, self.phase_v.c);
    }

    fn get_shaft_velocity(&mut self, t_us: u64) -> f32 {
        // let dir = if self.sensor_direction == SensorDirection::Normal {
        //     1.0
        // } else if self.sensor_direction == SensorDirection::Inverted {
        //     -1.0
        // } else {
        //     warn!("Sensor direction unknown, cannot calculate velocity");
        //     0.0
        //     // 1.0
        // };

        self.sensor_direction.multiplier()
            * self
                .lpf_velocity
                // .filter_with_timestamp(self.encoder.get_velocity(), t_us)
                .filter(self.encoder.get_velocity())
    }

    fn get_shaft_angle(&mut self) -> f32 {
        let angle = self.encoder.get_angle();

        let angle =
            self.sensor_direction.multiplier() * self.lpf_angle.filter(angle) - self.sensor_offset;

        angle
    }

    fn get_electrical_angle(&mut self) -> f32 {
        let shaft_angle = self.encoder.get_mechanical_angle();
        let angle = self.sensor_direction.multiplier() * self.motor.pole_pairs as f32 * shaft_angle
            - self.zero_electric_angle;

        // (Self::normalize_angle(angle), shaft_angle)
        Self::normalize_angle(angle)
    }
}

/// helpers
impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
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
