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
                        encoder_calibration_enabled,
                    } => {
                        debug!("Got debug data from motor {}", id);
                        self.status[id as usize].zero_electrical_angle =
                            zero_electrical_angle as f64;
                        self.status[id as usize].calibration_enabled = encoder_calibration_enabled;
                    }
                    // SerialLogMessage::Ping => {}
                    SerialLogMessage::LogData {
                        id,
                        // timestamp,
                        start_new,
                        data,
                    } => {
                        // debug!("Got log data from motor {}", id);

                        let file = "motor_data.csv";
                        let mut writer = if start_new {
                            std::fs::remove_file(file).ok();
                            std::fs::OpenOptions::new()
                                .create(true)
                                .append(true)
                                .open(file)
                                .expect("Failed to open CSV file for writing")
                        } else {
                            std::fs::OpenOptions::new()
                                .create(true)
                                .append(true)
                                .open(file)
                                .expect("Failed to open CSV file for writing")
                        };
                        let mut wtr = csv::Writer::from_writer(writer);
                        // wtr.write_field(format!("{:.6}", timestamp as f64 * 1e-6))
                        //     .unwrap();
                        for d in data.iter() {
                            wtr.write_field(format!("{}", d)).unwrap();
                        }
                        // wtr.write_field(format!("{}", data[0])).unwrap();
                        // wtr.write_field(format!("{}", data[1])).unwrap();
                        // wtr.write_field(format!("{}", velocity))
                        //     .expect("Failed to write velocity");
                        wtr.write_record(None::<&[u8]>).unwrap();
                    }
                    SerialLogMessage::ADRCDebugData {
                        id,
                        timestamp,
                        vs,
                        state,
                        u,
                    } => {
                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;
                            // debug!("State: {:?}", state);
                            // debug!("vs: {:?}", vs);
                            self.plots[id as usize].add_points_adrc(t, vs, state, u);
                            self.status[id as usize].adrc_internals = (
                                [vs[0] as f64, vs[1] as f64],
                                [state[0] as f64, state[1] as f64, state[2] as f64],
                                u as f64,
                            );
                            // } else {
                            //     self.t0 = Some(timestamp as f64 * 1e-6);
                        }
                    }
                    SerialLogMessage::MotorADRC {
                        id,
                        b0,
                        speed_factor,
                        observer_bandwidth,
                        controller_bandwidth,
                    } => {
                        debug!("Got motor ADRC settings from motor {}", id);

                        self.status[id as usize].adrc_b0 = b0;
                        self.status[id as usize].adrc_speed_factor = speed_factor;
                        self.status[id as usize].adrc_observer_bandwidth = observer_bandwidth;
                        self.status[id as usize].adrc_controller_bandwidth = controller_bandwidth;
                    }
                    #[cfg(feature = "nope")]
                    SerialLogMessage::MotorPID {
                        id,
                        vel_p,
                        vel_i,
                        vel_d,
                        // vel_ramp,
                        vel_limit,
                        angle_p,
                        angle_i,
                        angle_d,
                        // angle_ramp,
                        angle_limit,
                        lpf_angle,
                        lpf_vel,

                        vel_feed_forward,
                        vel_i_band,
                        vel_d_lpf,

                        pos_i_band,
                        pos_d_lpf,
                    } => {
                        // debug!("Got motor PID settings {:#?}", msg);
                        debug!("Got motor PID settings from motor {}", id);

                        self.status[id as usize].vel_pid_p = vel_p;
                        self.status[id as usize].vel_pid_i = vel_i;
                        self.status[id as usize].vel_pid_d = vel_d;
                        // self.status[id as usize].vel_pid_ramp = vel_ramp;
                        self.status[id as usize].vel_pid_limit = vel_limit as f64;
                        self.status[id as usize].pos_pid_p = angle_p;
                        self.status[id as usize].pos_pid_i = angle_i;
                        self.status[id as usize].pos_pid_d = angle_d;
                        // self.status[id as usize].pos_pid_ramp = angle_ramp;
                        self.status[id as usize].pos_pid_limit = angle_limit;
                        self.status[id as usize].lpf_angle = lpf_angle;
                        self.status[id as usize].lpf_vel = lpf_vel;

                        self.status[id as usize].vel_pid_d_lpf = vel_d_lpf as f64;
                        self.status[id as usize].vel_pid_i_band = vel_i_band as f64;
                        self.status[id as usize].vel_pid_feed_forward = vel_feed_forward as f64;

                        self.status[id as usize].pos_pid_i_band = pos_i_band as f64;
                        self.status[id as usize].pos_pid_d_lpf = pos_d_lpf as f64;
                    }
                    SerialLogMessage::EncoderData {
                        id,
                        timestamp,
                        position,
                        velocity,
                    } => {
                        debug!("TODO: Got encoder data {:#?}", id);
                    }
                    #[cfg(feature = "nope")]
                    SerialLogMessage::PIDDebugData {
                        id,
                        timestamp,
                        pid_internals_vel,
                        pid_internals_pos,
                    } => {
                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;
                            self.plots[id as usize].add_points_pid_vel_internals(
                                t,
                                pid_internals_vel.0 as f64,
                                pid_internals_vel.1 as f64,
                                pid_internals_vel.2 as f64,
                                pid_internals_vel.3 as f64,
                            );
                            self.plots[id as usize].add_points_pid_pos_internals(
                                t,
                                pid_internals_pos.0 as f64,
                                pid_internals_pos.1 as f64,
                                pid_internals_pos.2 as f64,
                                pid_internals_pos.3 as f64,
                            );
                        }
                    }
                    SerialLogMessage::FocLoopRate {
                        id,
                        timestamp,
                        loop_rate_hz,
                    } => {
                        debug!(
                            "Got FOC loop rate data from motor {}: {} Hz",
                            id, loop_rate_hz
                        );
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
                        // pid_internals_vel,
                    } => {
                        // debug!("Got motor data {:#?}", msg);
                        // debug!("Got motor data");

                        // let motor_voltage = (0., 0.);
                        // let sensor_currents = Some((0., 0.));
                        // let feed_forward = 0.;

                        // debug!("Got motor data from motor {}", id);

                        // debug!("Sensor currents: {:?}", sensor_currents);

                        // let file = "motor_data.csv";
                        // let mut writer = std::fs::OpenOptions::new()
                        //     .create(true)
                        //     .append(true)
                        //     .open(file)
                        //     .expect("Failed to open CSV file for writing");
                        // let mut wtr = csv::Writer::from_writer(writer);
                        // wtr.write_field(format!("{:.6}", timestamp as f64 * 1e-6))
                        //     .expect("Failed to write timestamp");
                        // wtr.write_field(format!("{}", target_position))
                        //     .expect("Failed to write position");
                        // wtr.write_field(format!("{}", position))
                        //     .expect("Failed to write position");
                        // // wtr.write_field(format!("{}", velocity))
                        // //     .expect("Failed to write velocity");
                        // wtr.write_record(None::<&[u8]>).unwrap();

                        /// only plot data from motor 0 for now
                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;

                            // debug!("Got motor data, t = {:.6} s", t);

                            self.plots[id as usize].add_point_angle(t, angle as f64);
                            self.plots[id as usize].add_point_pos(t, position as f64);
                            self.plots[id as usize].add_point_vel(t, velocity as f64);
                            self.plots[id as usize].add_point_target_vel(t, target_velocity as f64);
                            self.plots[id as usize].add_point_target_pos(t, target_position as f64);
                            self.plots[id as usize].add_point_voltage(t, motor_voltage.0 as f64);
                            // self.plots[id as usize]
                            //     .add_point_pid_output_vel(t, pid_outputs.0 as f64);
                            // self.plots[id as usize]
                            //     .add_point_pid_output_pos(t, pid_outputs.1 as f64);
                            // self.plot.add_point_current(t, motor_current as f64);
                            if let Some((current_d, current_q)) = sensor_currents {
                                self.plots[id as usize].add_point_current(
                                    t,
                                    current_d as f64,
                                    current_q as f64,
                                );
                            }
                            // self.plot.add_point_current(t, );
                            // self.plots[id as usize].add_points_pid_vel_internals(
                            //     t,
                            //     pid_internals_vel.0 as f64,
                            //     pid_internals_vel.1 as f64,
                            //     pid_internals_vel.2 as f64,
                            //     pid_internals_vel.3 as f64,
                            // );
                        } else {
                            self.t0 = Some(timestamp);
                        }

                        self.status[id as usize].motion_control = Some(motion_control);

                        if let Some(t) = self.last_update[id as usize] {
                            if t.elapsed() >= self.update_interval {
                                self.last_update[id as usize] = Some(Instant::now());

                                let offset = self.status[id as usize].angle_offset;
                                // debug!("target_pos: {}", target_position as f64);
                                self.status[id as usize].target_pos =
                                    target_position as f64 + offset;
                                self.status[id as usize].target_vel = target_velocity as f64;

                                self.status[id as usize].pos = position as f64;
                                self.status[id as usize].angle = angle as f64;
                                self.status[id as usize].vel = velocity as f64;

                                self.status[id as usize].current = motor_current;

                                if let Some((a, b)) = sensor_currents {
                                    self.status[id as usize].sensor_currents = (a, b);

                                    self.status[id as usize].sensor_currents_avg.0.push_back(a);
                                    self.status[id as usize].sensor_currents_avg.1.push_back(b);

                                    if self.status[id as usize].sensor_currents_avg.0.len() > 4 {
                                        self.status[id as usize].sensor_currents_avg.0.pop_front();
                                    }
                                    if self.status[id as usize].sensor_currents_avg.1.len() > 4 {
                                        self.status[id as usize].sensor_currents_avg.1.pop_front();
                                    }
                                }

                                self.status[id as usize].vel_pid_output = pid_outputs.0;
                                self.status[id as usize].pos_pid_output = pid_outputs.1;

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
