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
            DQCurrents, NOT_SET, PhaseCurrents, PhaseVoltages, SensorDirection, TorqueControlType,
        },
    },
};

/// main loop
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    /// Iterative function looping FOC algorithm, setting Uq on the Motor
    /// The faster it can be run the better
    pub async fn loop_foc(&mut self) {
        let t_us = Instant::now().as_micros();

        // let mut read_current = false;

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

        // // MARK: DEBUG CURRENT
        // let electrical_angle = self.get_electrical_angle();
        // if let Some(current_sensor) = &mut self.current_sensor {
        //     let _ = current_sensor.get_foc_currents(electrical_angle).await;
        // }

        if matches!(self.motion_control, MotionControlType::VelocityOpenLoop) {
            return;
        }
        if !self.enabled {
            return;
        }

        let electrical_angle = self.get_electrical_angle();

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

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // nothing to do
            }
            TorqueControlType::DCCurrent => {
                error!("TODO: implement DC current control");
                unimplemented!()
            }
            TorqueControlType::FOCCurrent => {
                if let Some(current_sensor) = &mut self.current_sensor {}
                // error!("TODO: implement FOC current control");
            }
        }

        self.set_phase_voltage(self.motor.voltage.q, self.motor.voltage.d, electrical_angle);
    }

    pub async fn update_foc(&mut self) {
        let t_us = Instant::now().as_micros();

        if self.motion_downsample > 0 {
            if self.motion_downsample_counter >= self.motion_downsample {
                self.motion_downsample_counter = 0;
            } else {
                self.motion_downsample_counter += 1;
                return;
            }
        }

        if self.debug_us_interval() > 0 {
            if t_us - self.prev_debug_us >= self.debug_us_interval() {
                self.debug = true;
                self.prev_debug_us = t_us;
            } else {
                self.debug = false;
            }
        }

        let shaft_angle = self.get_shaft_angle();
        let shaft_velocity = self.get_shaft_velocity(t_us);

        if !self.enabled {
            return;
        }

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = match self.motor.motor_kv {
            Some(kv) => shaft_velocity / (kv * super::types::_SQRT3) / super::types::_RPM_TO_RADS,
            None => 0.0,
        };

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
                        debug!("Done tuning velocity PID");
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
                        shaft_velocity,
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

            let measured_iq = None;

            self.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
                id: self.id,
                timestamp: t_us,
                motion_control: self.motion_control,
                position: shaft_angle,
                angle: self.encoder.get_mechanical_angle(),
                velocity: shaft_velocity,
                target_position: self.motor.target_shaft_angle,
                target_velocity: self.motor.target_shaft_velocity,
                motor_current: self.motor.current.q,
                sensor_currents,
                measured_iq,
                motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
                feed_forward: self.feed_forward_torque,
            })
            .await;
        }

        self.debug = false;

        //
    }
}
