use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use std::time::Instant;

use robotarm_protocol::SerialLogMessage;

use crate::ui::app::App;

impl App {
    pub fn get_from_channels(&mut self) {
        if let Some(rx) = &mut self.ui_cmd_rx {
            while let Ok(cmd) = rx.try_recv() {
                match cmd {
                    crate::ui::UiCommand::ClearPlot => {
                        debug!("Clearing plot");
                        for plot in self.plots.iter_mut() {
                            plot.reset();
                        }
                        // self.plot.reset();
                        self.t0 = None;
                    }
                }
            }
        }

        if let Some(rx) = &mut self.serial_log_rx {
            while let Ok(msg) = rx.try_recv() {
                // debug!("Got serial log message {:#?}", msg);
                match msg {
                    SerialLogMessage::DebugData {
                        id,
                        timestamp,
                        zero_electrical_angle,
                    } => {
                        debug!("Got debug data from motor {}", id);
                        self.status[id as usize].zero_electrical_angle =
                            zero_electrical_angle as f64;
                    }
                    // SerialLogMessage::Ping => {}
                    SerialLogMessage::MotorPID {
                        id,
                        vel_p,
                        vel_i,
                        vel_d,
                        vel_ramp,
                        vel_limit,
                        angle_p,
                        angle_i,
                        angle_d,
                        angle_ramp,
                        angle_limit,
                        lpf_angle,
                        lpf_vel,
                    } => {
                        // debug!("Got motor PID settings {:#?}", msg);
                        debug!("Got motor PID settings from motor {}", id);

                        self.status[id as usize].vel_pid_p = vel_p;
                        self.status[id as usize].vel_pid_i = vel_i;
                        self.status[id as usize].vel_pid_d = vel_d;
                        self.status[id as usize].vel_pid_ramp = vel_ramp;
                        self.status[id as usize].vel_pid_limit = vel_limit as f64;
                        self.status[id as usize].pos_pid_p = angle_p;
                        self.status[id as usize].pos_pid_i = angle_i;
                        self.status[id as usize].pos_pid_d = angle_d;
                        self.status[id as usize].pos_pid_ramp = angle_ramp;
                        self.status[id as usize].pos_pid_limit = angle_limit;
                        self.status[id as usize].lpf_angle = lpf_angle;
                        self.status[id as usize].lpf_vel = lpf_vel;
                    }
                    SerialLogMessage::EncoderData {
                        id,
                        timestamp,
                        position,
                        velocity,
                    } => {
                        debug!("TODO: Got encoder data {:#?}", id);
                    }
                    SerialLogMessage::MotorData {
                        id,
                        timestamp,
                        motion_control,
                        position,
                        angle,
                        velocity,
                        target_position,
                        target_velocity,
                        motor_current,
                        motor_voltage,
                        sensor_currents,
                        feed_forward,
                        pid_outputs,
                    } => {
                        // debug!("Got motor data {:#?}", msg);
                        // debug!("Got motor data");

                        // debug!("Got motor data from motor {}", id);

                        // debug!("Sensor currents: {:?}", sensor_currents);

                        /// only plot data from motor 0 for now
                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;
                            self.plots[id as usize].add_point_angle(t, angle as f64);
                            self.plots[id as usize].add_point_vel(t, velocity as f64);
                            self.plots[id as usize].add_point_target_vel(t, target_velocity as f64);
                            self.plots[id as usize].add_point_target_pos(t, target_position as f64);
                            self.plots[id as usize].add_point_voltage(t, motor_voltage.0 as f64);
                            self.plots[id as usize]
                                .add_point_pid_output_vel(t, pid_outputs.0 as f64);
                            self.plots[id as usize]
                                .add_point_pid_output_pos(t, pid_outputs.1 as f64);
                            // self.plot.add_point_current(t, motor_current as f64);
                            if let Some((current_d, current_q)) = sensor_currents {
                                self.plots[id as usize].add_point_current(
                                    t,
                                    current_d as f64,
                                    current_q as f64,
                                );
                            }
                            // self.plot.add_point_current(t, );
                        } else {
                            self.t0 = Some(timestamp);
                        }

                        self.status[id as usize].motion_control = Some(motion_control);

                        if let Some(t) = self.last_update[id as usize] {
                            if t.elapsed() >= self.update_interval {
                                self.last_update[id as usize] = Some(Instant::now());

                                let offset = self.status[id as usize].angle_offset;
                                self.status[id as usize].target_pos =
                                    target_position as f64 + offset;
                                self.status[id as usize].target_vel = target_velocity as f64;

                                self.status[id as usize].pos = position as f64;
                                self.status[id as usize].angle = angle as f64;
                                self.status[id as usize].vel = velocity as f64;

                                self.status[id as usize].current = motor_current;

                                if let Some((a, b)) = sensor_currents {
                                    self.status[id as usize].sensor_currents = (a, b);
                                }

                                // {
                                //     let mult =
                                //     let offset =
                                //     self.status.current = (motor_current * mult) + offset;
                                // }

                                self.status[id as usize].voltage = motor_voltage;

                                self.status[id as usize].feed_forward = feed_forward as f64;
                            }
                        }
                    }
                }
            }
        }
    }
}
