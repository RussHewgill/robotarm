use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};
use robotarm_protocol::{SerialCommand, SerialLogMessage, types::MotionControlType};

use crate::{
    hardware::{
        as5600::AS5600, current_sensor::CurrentSensor, encoder_sensor::EncoderSensor,
        mt_6701::MT6701,
    },
    simplefoc::{
        bldc::BLDCMotor,
        foc_types::{FOCModulation, SimpleFOC},
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{
            _2PI, DQCurrents, NOT_SET, PhaseCurrents, PhaseVoltages, SensorDirection,
            TorqueControlType,
        },
    },
};

/// main loop
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    /// Iterative function looping FOC algorithm, setting Uq on the Motor
    /// The faster it can be run the better
    #[inline(always)]
    pub async fn loop_foc(&mut self, t_us: u64) {
        // let mut read_current = false;

        #[cfg(feature = "nope")]
        if self.angle_sensor_downsample > 1 {
            if self.angle_sensor_downsample_counter >= self.angle_sensor_downsample {
                self.angle_sensor_downsample_counter = 0;
                let _ = self.encoder.update(t_us).await;
                // read_current = true;
            } else {
                self.angle_sensor_downsample_counter += 1;
            }
        } else {
            let _ = self.encoder.update(t_us).await;
            // read_current = true;
        }

        // if let Some(next_angle_sample_time) = self.get_angle_sensor_next_sample_time() {
        //     unimplemented!()
        // } else {
        //     let _ = self.encoder.update(t_us).await;
        // }

        #[cfg(feature = "nope")]
        let electrical_angle = if self.enabled {
            let e1 = self
                .update_luenberger_observer(t_us, self.motor.target_current)
                .await;
            let e2 = self.get_electrical_angle();
            if (e2 - e1).abs() > 0.1 {
                debug!(
                    "Electrical angle from observer: {}, from encoder: {}",
                    e1, e2
                );
            }
            if e1.is_nan() || e2.is_nan() {
                self.disable();
                panic!()
            }
            e2
        } else {
            self.get_electrical_angle()
        };

        // state observer mech_angle should be the same as shaft angle (does not wrap)

        // self.update_luenberger_observer(t_us, self.motor.target_current)
        self.update_luenberger_observer(t_us, self.motor.current.q)
            .await;
        // self.update_luenberger_observer(t_us, 0.).await;

        // // let _ = self.encoder.update(t_us).await;
        // let e2 = self.get_electrical_angle();
        // if (e2 - e1).abs() > 0.1 {
        //     debug!(
        //         "Electrical angle from observer: {}, from encoder: {}",
        //         e1, e2
        //     );
        // }
        // if e1.is_nan() || e2.is_nan() {
        //     self.disable();
        //     panic!()
        // }

        // let _ = self.encoder.update(t_us).await;
        let e2 = self.get_electrical_angle();

        let electrical_angle = e2;

        // let a1 = self.state_observer.get_angle_vel().0;
        // let a1 = self.sensor_direction.multiplier() * self.motor.pole_pairs as f32 * a1
        //     - self.zero_electric_angle;
        // let a1 = Self::normalize_angle(a1);

        // let a2 = self.encoder.get_mechanical_angle();
        // let a2 = self.sensor_direction.multiplier() * self.motor.pole_pairs as f32 * a2
        //     - self.zero_electric_angle;
        // let a2 = Self::normalize_angle(a2);

        // if (a2 - a1).abs() > 0.1 {
        //     debug!(
        //         "Mechanical angle from observer: {}, from encoder: {}",
        //         a1, a2
        //     );
        // }

        // let e1 = (e1 * 100.) as i32;
        // let e2 = (e2 * 100.) as i32;
        // debug!(
        //     "Electrical angle from observer: {:04}, from encoder: {:04}",
        //     e1, e2
        // );

        // self.set_phase_voltage(0., 0., 0.);
        // return;

        // let _ = self.encoder.update(t_us).await;
        // let electrical_angle = self.get_electrical_angle();

        // if let Some(tx) = &mut self.current_sensor_elec_angle_tx {
        //     let _ = tx.try_send(electrical_angle);
        // }

        #[cfg(feature = "current_sensing")]
        if let Some(current_sensor) = &mut self.current_sensor {
            let mut read_current = false;
            if self.current_sensor_downsample > 1 {
                if self.current_sensor_downsample_counter >= self.current_sensor_downsample {
                    self.current_sensor_downsample_counter = 0;
                    read_current = true;
                } else {
                    self.current_sensor_downsample_counter += 1;
                }
            } else {
                read_current = true;
            }

            if read_current {
                // match current_sensor.get_phase_currents().await {
                //     Ok(currents) => {
                //         // self.motor.current = currents;
                //         // debug!("Phase currents: {:?}", currents);
                //     }
                //     Err(e) => {
                //         error!("Error reading current");
                //     }
                // }

                match current_sensor.get_foc_currents(electrical_angle).await {
                    Ok(currents) => {
                        // self.motor.current = currents;
                    }
                    Err(e) => {
                        error!("Error reading current");
                    }
                }
            }
        }

        if matches!(self.motion_control, MotionControlType::VelocityOpenLoop) {
            return;
        }
        if matches!(self.motion_control, MotionControlType::AngleOpenLoop) {
            return;
        }
        if !self.enabled {
            return;
        }

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // nothing to do
                // self.motor.voltage.q = self.
            }
            TorqueControlType::EstimatedCurrent => {
                self.motor.target_current = self
                    .motor
                    .target_current
                    .clamp(-self.motor.limit_current, self.motor.limit_current)
                    + self.feed_forward_current.q;

                let voltage_bemf = self.estimate_back_emf(self.shaft_velocity);

                self.motor.current.q = self
                    .lpf_current_q
                    .filter_with_timestamp(self.motor.target_current, t_us);

                self.motor.voltage.q = self.motor.current.q
                    * self.motor.phase_resistance.unwrap_or(0.0)
                    + voltage_bemf;

                self.motor.voltage.q = self
                    .motor
                    .voltage
                    .q
                    .clamp(-self.motor.limit_voltage, self.motor.limit_voltage)
                    + self.feed_forward_voltage.q;
            }
            TorqueControlType::DCCurrent => {
                error!("TODO: implement DC current control");
                unimplemented!()
            }
            TorqueControlType::FOCCurrent => {
                let Some(current_sensor) = &mut self.current_sensor else {
                    error!("FOC current control requires a current sensor");
                    unimplemented!()
                };

                // let Some(current) = current_sensor.prev_foc_currents() else {
                //     error!("FOC current control requires a current sensor reading");
                //     unimplemented!()
                // };

                if let Some(current) = current_sensor.prev_foc_currents() {
                    // filter values
                    self.motor.current.q =
                        self.lpf_current_q.filter_with_timestamp(current.q, t_us);
                    self.motor.current.d =
                        self.lpf_current_d.filter_with_timestamp(current.d, t_us);

                    // calculate the phase voltages
                    self.motor.voltage.q = self.pid_current_q.update(
                        self.motor.target_current,
                        self.motor.current.q,
                        t_us,
                    );
                    self.motor.voltage.d =
                        self.pid_current_d.update(0., self.motor.current.d, t_us);

                    self.motor.voltage.q += self.feed_forward_current.q;
                    self.motor.voltage.d += self.feed_forward_current.d;
                } else {
                    error!("FOC current control requires a current sensor reading");
                }
            }
        }

        #[cfg(feature = "thermal_throttling")]
        if let Some(thermals) = &mut self.motor.thermal_limits {
            if let Some(current_sensor) = &mut self.current_sensor {
                if let Some(current) = current_sensor.prev_foc_currents() {
                    if let Some(fraction) = thermals.update(current.q, t_us) {
                        self.motor.voltage.q *= fraction;
                        self.motor.voltage.d *= fraction;
                    }
                }
            } else {
                error!("Thermal control with no current sensor");
            }
        }

        self.set_phase_voltage(self.motor.voltage.q, self.motor.voltage.d, electrical_angle);
    }

    #[inline(always)]
    pub async fn update_foc(&mut self, t_us: u64) {
        if self.motion_downsample > 0 {
            if self.motion_downsample_counter >= self.motion_downsample {
                self.motion_downsample_counter = 0;
            } else {
                self.motion_downsample_counter += 1;
                return;
            }
        }

        #[cfg(feature = "debug_logging")]
        if self.debug_us_interval() > 0 {
            if t_us - self.prev_debug_us >= self.debug_us_interval() {
                self.debug = true;
                self.prev_debug_us = t_us;
            } else {
                self.debug = false;
            }
        }

        let shaft_angle = self.get_shaft_angle();
        self.shaft_velocity = self.get_shaft_velocity(t_us);

        if !self.enabled {
            return;
        }

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = self.estimate_back_emf(self.shaft_velocity);

        if self.current_sensor.is_none() {
            // #[cfg(feature = "nope")]
            if let Some(phase_resistance) = self.motor.phase_resistance {
                // estimate the motor current if phase reistance available and current_sense not available
                self.motor.current.q = (self.motor.voltage.q - voltage_bemf) / phase_resistance;
            }

            // if let Some(phase_resistance) = self.motor.phase_resistance {
            //     unimplemented!()
            // }
        }

        // MARK: Motion Control
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
                                * self.shaft_velocity
                                * (self.motor.pole_pairs as f32)
                                * phase_inductance)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => {
                            self.motor.voltage.d = 0.0;
                        }
                    }
                }

                #[cfg(feature = "nope")]
                if self.debug {
                    let rpm = self.shaft_velocity * 30.0 / core::f32::consts::PI;

                    let kv = rpm / self.motor.voltage.q;

                    debug!(
                        "KV Calculation: voltage: {}, velocity (rad/s): {}, velocity (RPM), KV: {}",
                        self.motor.voltage.q,
                        self.shaft_velocity,
                        //
                        kv
                    )
                }
            }
            MotionControlType::Velocity => {
                #[cfg(feature = "nope")]
                if let Some(tuner) = &mut self.pid_velocity_tuner {
                    if !tuner.done() {
                        self.motor.target_current = tuner.update(self.shaft_velocity, t_us);
                    } else {
                        debug!("Done tuning velocity PID");
                        self.pid_velocity_tuner = None;
                    }
                } else {
                    // self.motor.target_current = self.feed_forward_torque +
                    self.motor.target_current = self.pid_velocity.update(
                        self.motor.target_shaft_velocity,
                        self.shaft_velocity,
                        t_us,
                    );
                }

                self.motor.target_current = self.pid_velocity.update(
                    self.motor.target_shaft_velocity,
                    self.shaft_velocity,
                    t_us,
                );

                if self.torque_controller == TorqueControlType::Voltage {
                    match self.motor.phase_resistance {
                        None => self.motor.voltage.q = 0.0,
                        Some(phase_resistance) => {
                            self.motor.voltage.q = (self.motor.target_current * phase_resistance
                                + voltage_bemf)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                    }
                    match self.motor.phase_inductance {
                        None => self.motor.voltage.d = 0.,
                        Some(phase_inductance) => {
                            //
                            unimplemented!()
                        }
                    }
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
                self.motor.target_current = self.feed_forward_torque
                    + self.pid_velocity.update(
                        self.motor.target_shaft_velocity,
                        self.shaft_velocity,
                        t_us,
                    );

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
                                * self.shaft_velocity
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
                    t_us,
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
            MotionControlType::AngleOpenLoop => {
                self.motor.voltage.q = self.angle_openloop(
                    self.motor.target_shaft_angle,
                    // shaft_angle,
                    // voltage_bemf,
                    // electrical_angle,
                    t_us,
                );
                self.motor.voltage.d = 0.0;
            }
        }

        #[cfg(feature = "debug_logging")]
        if self.debug {
            let sensor_currents = if let Some(current_sensor) = &mut self.current_sensor {
                // match current_sensor.prev_phase_currents() {
                //     Some(PhaseCurrents::Two { a, b }) => Some((a, b, 0.)),
                //     Some(PhaseCurrents::Three { a, b, c }) => Some((a, b, c)),
                //     None => None,
                // }

                match current_sensor.prev_foc_currents() {
                    Some(DQCurrents { d, q }) => Some((d, q)),
                    None => None,
                }
            } else {
                None
            };

            let v = self.sensor_direction.multiplier()
                * self
                    .lpf_velocity
                    .filter_with_timestamp(self.encoder.get_velocity(), t_us);

            self.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
                id: self.id,
                timestamp: t_us,
                motion_control: self.motion_control,
                // position: shaft_angle,
                // position: self.encoder.get_angle(),
                position: self.encoder.get_mechanical_angle(),
                angle: self.get_mechanical_angle(),
                velocity: self.shaft_velocity,
                target_position: self.motor.target_shaft_angle,
                target_velocity: self.motor.target_shaft_velocity,
                motor_current: self.motor.current.q,
                sensor_currents,
                motor_voltage: (v, 0.),
                // motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
                feed_forward: self.feed_forward_torque,
                pid_outputs: (
                    self.pid_velocity.prev_output(),
                    self.pid_angle.prev_output(),
                ),
                // pid_internals_vel: self.pid_velocity.prev_internals(),
                // pid_internals_vel: (0., 0., 0., 0.),
                // pid_internals_vel: None,
            })
            .await;

            #[cfg(feature = "nope")]
            self.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
                id: self.id,
                timestamp: t_us,
                motion_control: self.motion_control,
                // position: shaft_angle,
                position: {
                    let angle =
                        // self.sensor_direction.multiplier() * self.state_observer.get_angle_vel().0;
                        self.state_observer.get_angle_vel().0;
                    Self::normalize_angle(angle)
                },
                // angle: self.encoder.get_mechanical_angle(),
                // angle: self.get_mechanical_angle(),
                angle: Self::normalize_angle(
                    self.sensor_direction.multiplier() * self.encoder.get_mechanical_angle(),
                ),
                velocity: self.shaft_velocity,

                motor_voltage: (
                    { self.sensor_direction.multiplier() * self.encoder.get_velocity() },
                    0.0,
                ),

                // motor_voltage: (
                //     // { self.sensor_direction.multiplier() * self.state_observer.get_angle_vel().1 },
                //     { self.state_observer.get_angle_vel().1 },
                //     0.0,
                // ),
                target_position: self.motor.target_shaft_angle,
                target_velocity: self.motor.target_shaft_velocity,
                // target_position: self.motor.target_shaft_angle * self.sensor_direction.multiplier(),
                // target_velocity: self.motor.target_shaft_velocity
                //     * self.sensor_direction.multiplier(),
                motor_current: self.motor.current.q,
                sensor_currents,
                // motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
                feed_forward: self.feed_forward_torque,
                pid_outputs: (
                    self.pid_velocity.prev_output(),
                    self.pid_angle.prev_output(),
                ),
                // pid_internals_vel: self.pid_velocity.prev_internals(),
                // pid_internals_vel: (0., 0., 0., 0.),
                // pid_internals_vel: None,
            })
            .await;

            self.send_debug_message(SerialLogMessage::PIDDebugData {
                id: self.id,
                timestamp: t_us,
                pid_internals_vel: self.pid_velocity.prev_internals(),
                pid_internals_pos: self.pid_angle.prev_internals(),
            })
            .await;
        }

        self.debug = false;

        //
    }
}

#[cfg(feature = "nope")]
/// PID tuning
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub async fn pid_tuning_loop(
        &mut self,
        tuner: &mut crate::simplefoc::pid_tuning_vel::VelocityAutoTuner,
    ) -> bool {
        let t_us = Instant::now().as_micros();

        let _ = self.encoder.update(t_us).await;

        let electrical_angle = self.get_electrical_angle();

        let shaft_velocity = self.get_shaft_velocity(t_us);

        let (q, d) = tuner.update(t_us, shaft_velocity);

        match tuner.state {
            crate::simplefoc::pid_tuning_vel::AutoTuneState::Done => {
                self.set_phase_voltage(0., 0., electrical_angle);

                debug!("PID tuning done! Kp: {}, Ki: {}", tuner.kp, tuner.ki);

                return true;
            }
            _ => {}
        }

        self.motor.voltage.q = q;
        self.motor.voltage.d = d;

        self.set_phase_voltage(self.motor.voltage.q, self.motor.voltage.d, electrical_angle);

        false
    }
}
