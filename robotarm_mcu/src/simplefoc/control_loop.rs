use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};
use robotarm_protocol::{SerialCommand, SerialLogMessage, types::MotionControlType};

use crate::{
    hardware::{as5600::AS5600, encoder_sensor::EncoderSensor, mt_6701::MT6701},
    simplefoc::{
        bldc::BLDCMotor,
        foc_types::{FOCModulation, SimpleFOC},
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

/// main loop
impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
    /// main loop
    pub async fn update_foc(&mut self) {
        // trace!("Updating FOC control loop");

        let t_us = Instant::now().as_micros();

        if self.debug_us_interval() > 0 {
            if t_us - self.prev_debug_us >= self.debug_us_interval() {
                self.debug = true;
                self.prev_debug_us = t_us;
            } else {
                self.debug = false;
            }
        }

        // update sensor readings

        // #[cfg(feature = "nope")]
        if self.sensor_downsample > 1 {
            if self.sensor_us_counter >= self.sensor_downsample {
                self.sensor_us_counter = 0;
                let _ = self.encoder.update(t_us).await;
            } else {
                self.sensor_us_counter += 1;
            }
        } else {
            let _ = self.encoder.update(t_us).await;
        }

        // if let Err(_e) = self.encoder.update(t_us).await {
        //     error!("Failed to update encoder");
        // }

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
                motor_voltage: (self.motor.voltage.q, self.motor.voltage.d),
                feed_forward: self.feed_forward_torque,
            })
            .await;
        }

        self.debug = false;
    }
}
