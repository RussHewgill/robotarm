use defmt::{debug, error};
use robotarm_protocol::{SerialCommand, SerialLogMessage, types::MotionControlType};

use crate::{hardware::encoder_sensor::EncoderSensor, simplefoc::foc_types::SimpleFOC};

impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
    // #[cfg(feature = "nope")]
    pub async fn run_commands(&mut self) {
        let mut cmds = heapless::Vec::<SerialCommand, 4>::new();
        if let Some(logger) = &mut self.usb_logger {
            while let Ok(cmd) = logger.recv().await {
                // let _ = cmds.push(cmd);
                cmds.push(cmd).unwrap_or_else(|_| {
                    error!("Command queue full, dropping command");
                });
            }
        }

        for cmd in cmds {
            self.run_command(cmd);
        }
    }

    #[cfg(feature = "nope")]
    pub async fn run_commands(&mut self, cmd_buf: &mut heapless::Vec<SerialCommand, 4>) {
        if let Some(logger) = &mut self.usb_logger {
            while let Ok(cmd) = logger.recv().await {
                cmd_buf.push(cmd).unwrap_or_else(|_| {
                    error!("Command queue full, dropping command");
                });
            }
        }

        for cmd in cmd_buf.drain(..) {
            self.run_command(cmd);
        }
    }

    fn run_command(&mut self, cmd: SerialCommand) {
        match cmd {
            SerialCommand::ZeroPosition { id } => {
                debug!("Received ZeroPosition command: id: {}", id);
                self.encoder.reset_position();
            }
            SerialCommand::SetLPF {
                id,
                lpf_vel,
                lpf_angle,
            } => {
                if let Some(lpf_vel) = lpf_vel {
                    self.lpf_velocity.tf = lpf_vel;
                }
                if let Some(lpf_angle) = lpf_angle {
                    self.lpf_angle.tf = lpf_angle;
                }
                debug!(
                    "Received SetLPF command: id: {}, lpf_vel: {:?}, lpf_angle: {:?}",
                    id, lpf_vel, lpf_angle
                );
            }
            SerialCommand::SetDebugRate { id, rate_hz } => {
                self.set_debug_freq(rate_hz as u64);
                debug!(
                    "Received SetDebugRate command: id: {}, rate_hz: {}",
                    id, rate_hz
                );
            }
            // SerialCommand::SetSensorOffset { id, offset } => {
            //     if id == self.id {
            //         debug!(
            //             "Received SetSensorOffset command: id: {}, current offset: {}, sensor_offset: {}",
            //             id, self.sensor_offset, offset
            //         );
            //         match offset {
            //             Some(offset) => self.sensor_offset = offset,
            //             None => self.sensor_offset = 0.0,
            //         }
            //         // self.sensor_offset = offset;
            //         // self.set_zero_angle();
            //         // let position = self.encoder.get_mechanical_angle();
            //         // self.zero_electric_angle = position;
            //     }
            // }
            SerialCommand::SetFeedForward { id, ff } => {
                self.feed_forward_torque = ff;
                debug!(
                    "Received SetFeedForward command: id: {}, feed_forward_torque: {}",
                    id, ff
                );
            }
            SerialCommand::RequestSettings { id } => {
                if id == self.id {
                    if let Some(logger) = &mut self.usb_logger {
                        logger.send_log_msg(SerialLogMessage::MotorPID {
                            id: self.id,
                            vel_p: self.pid_velocity.get_p(),
                            vel_i: self.pid_velocity.get_i(),
                            vel_d: self.pid_velocity.get_d(),
                            vel_ramp: self.pid_velocity.get_ramp(),
                            vel_limit: self.pid_velocity.get_limit(),
                            angle_p: self.pid_angle.get_p(),
                            angle_i: self.pid_angle.get_i(),
                            angle_d: self.pid_angle.get_d(),
                            angle_ramp: self.pid_angle.get_ramp(),
                            angle_limit: self.pid_angle.get_limit(),
                            lpf_angle: self.lpf_angle.tf,
                            lpf_vel: self.lpf_velocity.tf,
                        });
                    }
                }
            }
            SerialCommand::SetEnabled { id, enabled } => {
                if enabled {
                    self.enable();
                } else {
                    self.disable();
                }
                debug!(
                    "Received SetEnabled command: id: {}, enabled: {}",
                    id, enabled
                );
            }
            SerialCommand::SetMotorTarget { id, target } => {
                match self.motion_control {
                    MotionControlType::Torque => self.set_target_torque(target),
                    MotionControlType::Velocity => self.set_target_velocity(target),
                    MotionControlType::Angle => self.set_target_position(target),
                    MotionControlType::VelocityOpenLoop => self.set_target_velocity(target),
                }
                // self.set_target_position(target);
                debug!(
                    "Received SetMotorTarget command: id: {}, target: {}",
                    id, target
                );
            }
            SerialCommand::SetModeTorque { id } => {
                self.set_motion_control(MotionControlType::Torque);
                debug!("Received SetModeTorque command: id: {}", id);
            }
            SerialCommand::SetModeAngle { id } => {
                self.set_motion_control(MotionControlType::Angle);
                debug!("Received SetModeAngle command: id: {}", id);
            }
            SerialCommand::SetModeVelocity { id } => {
                self.set_motion_control(MotionControlType::Velocity);
                debug!("Received SetModeVelocity command: id: {}", id);
            }
            SerialCommand::SetModeVelocityOpenLoop { id } => {
                self.set_motion_control(MotionControlType::VelocityOpenLoop);
                debug!("Received SetModeVelocityOpenLoop command: id: {}", id);
            }
            SerialCommand::SetVelocityPID {
                id,
                p,
                i,
                d,
                ramp,
                limit,
            } => {
                if let Some(p) = p {
                    self.pid_velocity.set_p(p);
                }
                if let Some(i) = i {
                    self.pid_velocity.set_i(i);
                }
                if let Some(d) = d {
                    self.pid_velocity.set_d(d);
                }
                if let Some(limit) = limit {
                    self.pid_velocity.set_limit(limit);
                }
                if let Some(ramp) = ramp {
                    self.pid_velocity.set_ramp(ramp);
                }
                debug!(
                    "Received SetPID command: id: {}, p: {:?}, i: {:?}, d: {:?}, limit: {:?}",
                    id, p, i, d, limit
                );
            }
            SerialCommand::SetAnglePID {
                id,
                p,
                i,
                d,
                ramp,
                limit,
            } => {
                if let Some(p) = p {
                    self.pid_angle.set_p(p);
                }
                if let Some(i) = i {
                    self.pid_angle.set_i(i);
                }
                if let Some(d) = d {
                    self.pid_angle.set_d(d);
                }
                if let Some(limit) = limit {
                    self.pid_angle.set_limit(limit);
                }
                if let Some(ramp) = ramp {
                    self.pid_angle.set_ramp(ramp);
                }
                debug!(
                    "Received SetPID command: id: {}, p: {:?}, i: {:?}, d: {:?}, limit: {:?}",
                    id, p, i, d, limit
                );
            }
        }
    }
}
