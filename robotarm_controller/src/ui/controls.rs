use std::f64::consts::PI;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use egui::{Color32, RichText, Sense, Stroke, Vec2};
use egui_extras::Size;

use crate::ui::{app::App, controls::scrollable::make_scrollable};
use robotarm_protocol::SerialCommand;

mod pid_settings {
    use anyhow::{Context, Result, anyhow, bail, ensure};
    use tracing::{debug, error, info, trace, warn};

    use robotarm_protocol::SerialCommand;

    pub(super) fn pid_control_inc<F>(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut F,
        inc_value: &mut F,
        tx: &tokio::sync::mpsc::Sender<SerialCommand>,
        // tx: &crossbeam_channel::Sender<SerialCommand>,
        id: u8,
        cmd_fn: impl Fn(u8, F) -> SerialCommand,
    ) where
        F: egui::emath::Numeric + Copy,
    {
        ui.label(label);

        let resp = ui.add(egui::DragValue::new(value).fixed_decimals(5));

        let but_inc = ui.button("+");
        let amt = ui.add(egui::DragValue::new(inc_value).fixed_decimals(5));
        let but_dec = ui.button("-");

        // let send_resp = ui.button("Send");
        // let zero_resp = ui.button("Zero");

        // if (resp.lost_focus()
        //     && resp
        //         .ctx
        //         .input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Tab)))
        //     || send_resp.clicked()
        // {
        //     let cmd = cmd_fn(id, *value);
        //     if let Err(e) = tx.try_send(cmd) {
        //         error!("Failed to send command: {}", e);
        //     }
        // }

        // if zero_resp.clicked() {
        //     *value = F::from_f64(0.0);
        //     let cmd = cmd_fn(id, *value);
        //     if let Err(e) = tx.try_send(cmd) {
        //         error!("Failed to send command: {}", e);
        //     }
        // }
    }

    pub(super) fn pid_control_dec<F>(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut F,
        tx: &tokio::sync::mpsc::Sender<SerialCommand>,
        // tx: &crossbeam_channel::Sender<SerialCommand>,
        id: u8,
        decimals: usize,
        cmd_fn: impl Fn(u8, F) -> SerialCommand,
    ) where
        F: egui::emath::Numeric + Copy,
    {
        ui.label(label);

        let resp = ui.add(egui::DragValue::new(value).fixed_decimals(decimals));

        let send_resp = ui.button("Send");
        let zero_resp = ui.button("Zero");

        if (resp.lost_focus()
            && resp
                .ctx
                .input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Tab)))
            || send_resp.clicked()
        {
            let cmd = cmd_fn(id, *value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }

        if zero_resp.clicked() {
            *value = F::from_f64(0.0);
            let cmd = cmd_fn(id, *value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }
    }

    pub(super) fn pid_control<F>(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut F,
        tx: &tokio::sync::mpsc::Sender<SerialCommand>,
        // tx: &crossbeam_channel::Sender<SerialCommand>,
        id: u8,
        cmd_fn: impl Fn(u8, F) -> SerialCommand,
    ) where
        F: egui::emath::Numeric + Copy,
    {
        ui.label(label);

        let resp = ui.add(egui::DragValue::new(value).fixed_decimals(5));

        let send_resp = ui.button("Send");
        let zero_resp = ui.button("Zero");

        if (resp.lost_focus()
            && resp
                .ctx
                .input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Tab)))
            || send_resp.clicked()
        {
            let cmd = cmd_fn(id, *value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }

        if zero_resp.clicked() {
            *value = F::from_f64(0.0);
            let cmd = cmd_fn(id, *value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }
    }

    pub(super) fn set_vel_p(id: u8, p: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                p: Some(p),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_i(id: u8, i: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                i: Some(i),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_d(id: u8, d: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                d: Some(d),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_lpf(id: u8, lpf: f32) -> SerialCommand {
        SerialCommand::SetLPF {
            id,
            lpf_vel: Some(lpf),
            lpf_angle: None,
        }
    }

    pub(super) fn set_vel_limit(id: u8, limit: f64) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                limit: Some(limit as f32),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_i_band(id: u8, i_band: f64) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                i_band: Some(i_band as f32),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_d_lpf(id: u8, d_lpf: f64) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                d_lpf: Some(d_lpf as f32),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_vel_feed_forward(id: u8, ff: f64) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                feed_forward: Some(ff as f32),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_pos_limit(id: u8, limit: f32) -> SerialCommand {
        SerialCommand::SetAnglePID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                limit: Some(limit),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_pos_i_band(id: u8, i_band: f64) -> SerialCommand {
        SerialCommand::SetAnglePID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                i_band: Some(i_band as f32),
                ..Default::default()
            },
        }
    }

    pub(super) fn set_pos_d_lpf(id: u8, d_lpf: f64) -> SerialCommand {
        SerialCommand::SetAnglePID {
            id,
            pid_settings: robotarm_protocol::types::PIDSettings {
                d_lpf: Some(d_lpf as f32),
                ..Default::default()
            },
        }
    }

    // pub(super) fn set_pos_feed_forward(id: u8, ff: f64) -> SerialCommand {
    //     SerialCommand::SetAnglePID {
    //         id,
    //         pid_settings: robotarm_protocol::types::PIDSettings {
    //             feed_forward: Some(ff as f32),
    //             ..Default::default()
    //         },
    //     }
    // }
}

mod scrollable {
    pub fn make_scrollable(
        ui: &mut egui::Ui,
        resp: egui::Response,
        mut inc: f64,
        inc2: (f64, f64),
        curr: &mut f64,
        min: f64,
        max: f64,
    ) -> Option<f64> {
        if resp.hovered() {
            let delta = ui.input(|i| {
                if i.modifiers.shift {
                    inc = inc2.1;
                } else if i.modifiers.ctrl {
                    inc = inc2.0;
                }

                i.events.iter().find_map(|e| match e {
                    egui::Event::MouseWheel {
                        unit: _,
                        delta,
                        modifiers,
                    } => Some(*delta),
                    _ => None,
                })
            });

            if let Some(delta) = delta {
                if delta.y > 0. && *curr < max {
                    *curr += inc;
                    return Some(*curr);
                } else if delta.y < 0. && *curr > min {
                    *curr -= inc;
                    return Some(*curr);
                }
            }
        }

        if ui.button("Reset").clicked() {
            *curr = 0.0;
            return Some(*curr);
        }

        None
    }
}

impl App {
    pub fn send_command(&mut self, cmd: SerialCommand) {
        if let Some(tx) = &self.serial_cmd_tx {
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }
    }
}

impl App {
    pub fn controls(&mut self, ui: &mut egui::Ui, id: u8) {
        // StripBuilder::new(ui)
        //     .size(Size::relative(0.3))

        ui.columns_const(|[col_0, col_1, col_2, col_3]| {
            col_0.vertical(|ui| {
                self.col_0(ui, id);
            });
            col_1.vertical(|ui| {
                self.col_1(ui, id);
            });
            col_2.vertical(|ui| {
                self.col_2(ui, id);
            });
            col_3.vertical(|ui| {
                self.col_3(ui, id);
            });
        });
    }

    // motor data
    fn col_0(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("Motor Data Grid {id}")).show(ui, |ui| {
            ui.label(RichText::new("Current").monospace());
            ui.label(
                RichText::new(format!("{:>+0.3} A", self.status[id as usize].current)).monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Voltage").monospace());
            ui.label(
                RichText::new(format!("{:>+0.3} V", self.status[id as usize].voltage.0))
                    .monospace(),
            );
            ui.label(
                RichText::new(format!("{:>+0.3} V", self.status[id as usize].voltage.1))
                    .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Angle").monospace());
            ui.label(
                RichText::new(format!("{:>+0.3} rad", self.status[id as usize].angle)).monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Position").monospace());
            ui.label(
                RichText::new(format!("{:>+0.3} rad", self.status[id as usize].pos,)).monospace(),
            );
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rev",
                    self.status[id as usize].pos / (2.0 * std::f64::consts::PI)
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Velocity").monospace());
            ui.label(
                RichText::new(format!("{:>+0.3} rad/s", self.status[id as usize].vel)).monospace(),
            );
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rpm",
                    self.status[id as usize].vel * 60.0 / (2.0 * std::f64::consts::PI)
                ))
                .monospace(),
            );
            ui.end_row();

            #[cfg(feature = "nope")]
            ui.collapsing(format!("Phase Currents {id}"), |ui| {
                ui.label(
                    RichText::new(format!(
                        "{:>+0.3} A",
                        self.status[id as usize].sensor_currents.0
                    ))
                    .monospace(),
                );
                ui.label(
                    RichText::new(format!(
                        "{:>+0.3} A",
                        self.status[id as usize].sensor_currents.1
                    ))
                    .monospace(),
                );
            });
            ui.end_row();

            // // TEST
            // self.status.angle_offset = -0.28;

            // calculate current torque
            let kv = 140.;
            let phase_resistance = 9.2; // ohms

            let torque_constant = 8.27 / kv; // Nm/A

            let (a, b) = self.status[id as usize].sensor_currents;

            let current_magnitude = (a.powi(2) + b.powi(2)).sqrt();
            let torque = torque_constant * current_magnitude; // Nm

            // let quadrature_current = iq = -i_alpha *

            ui.label(RichText::new("Sens Current").monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} A",
                    self.status[id as usize].sensor_currents.0
                ))
                .monospace(),
            );
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} A",
                    self.status[id as usize].sensor_currents.1
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Sens Current Avg").monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} A",
                    self.status[id as usize]
                        .sensor_currents_avg
                        .0
                        .iter()
                        .sum::<f32>()
                        / self.status[id as usize].sensor_currents_avg.0.len() as f32
                ))
                .monospace(),
            );
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} A",
                    self.status[id as usize]
                        .sensor_currents_avg
                        .1
                        .iter()
                        .sum::<f32>()
                        / self.status[id as usize].sensor_currents_avg.1.len() as f32
                ))
                .monospace(),
            );

            ui.end_row();

            ui.label(RichText::new("Torque").monospace());
            ui.label(RichText::new(format!("{:>+0.4} Ncm", torque * 100.)).monospace());
            ui.end_row();

            // at -1.57 rad with no load, torque is 0.51 Ncm with a 50mm arm, or 0.102 N
            // let force = (0.51 / 100.) / 0.05; // N
            let force = 0.0051 / 0.05; // N

            // find force at current angle
            let angle = ((self.status[id as usize].pos + self.status[id as usize].angle_offset)
                * self.status[id as usize].gear_ratio) as f32;
            let force = force * angle.sin();

            ui.label(RichText::new("Force").monospace());
            ui.label(RichText::new(format!("{:>+0.4} mN", force * 1000.0)).monospace());

            ui.end_row();

            // ui.label(RichText::new("Vel PID Output").monospace());
            // ui.label(
            //     // RichText::new(format!("{:>+0.5}", self.status[id as usize].vel_pid_output))
            //     RichText::new(format!("{}", self.status[id as usize].vel_pid_output)).monospace(),
            // );
            // ui.end_row();

            ui.label("Gear Ratio");
            // ui.end_row();
            let resp = ui.add(
                egui::Slider::new(&mut self.status[id as usize].gear_ratio, -20.0..=60.0).integer(),
            );

            //
        });
    }

    // motor controls
    fn col_1(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("Motor Controls Grid {id}")).show(ui, |ui| {
            if ui.button("Enable Motor").clicked() {
                self.status[id as usize].enabled = true;
                let cmd = SerialCommand::SetEnabled { id, enabled: true };
                self.send_command(cmd);
            }
            if ui.button("Disable Motor").clicked() {
                self.status[id as usize].enabled = false;
                let cmd = SerialCommand::SetEnabled { id, enabled: false };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Torque");
            let but = if matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Torque)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                let cmd = SerialCommand::SetModeTorque { id };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Velocity");
            let but = if matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Velocity)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                self.send_command(SerialCommand::SetModeVelocity { id });
            }
            // ui.end_row();

            let but = egui::Button::new("Set Velocity Open Loop");
            let but = if matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::VelocityOpenLoop)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                let cmd = SerialCommand::SetModeVelocityOpenLoop { id };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Angle");
            let but = if matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Angle)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                self.send_command(SerialCommand::SetModeAngle { id });
            }
            // ui.end_row();

            let but = egui::Button::new("Set Angle Open Loop");
            let but = if matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::AngleOpenLoop)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                let cmd = SerialCommand::SetModeAngleOpenLoop { id };
                self.send_command(cmd);
            }
            ui.end_row();
        });

        // ui.horizontal(|ui| {
        //     ui.label("Gear Ratio:");
        //     let resp = ui.add(egui::Slider::new(&mut self.gear_ratio, -0.5..=10.0));
        // });

        // target torque
        ui.horizontal(|ui| {
            let (min, max) = (
                // -self.status[id as usize].vel_pid_limit,
                // self.status[id as usize].vel_pid_limit,
                -2., 2.,
            );

            ui.label("Target Torque:");
            let range =
                // -self.status[id as usize].vel_pid_limit..=self.status[id as usize].vel_pid_limit;
                min..=max;
            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].target_voltage,
                range,
            ));

            let mut send_target = None;

            if !matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Torque)
            ) {
            } else if resp.changed() {
                send_target = Some(self.status[id as usize].target_voltage);
            }

            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                0.2,
                (0.01, 1.0),
                &mut self.status[id as usize].target_voltage,
                min,
                max,
            ) {
                send_target = Some(tgt);
            }

            if !matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Torque)
            ) {
                send_target = None;
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id,
                    target: tgt as f32,
                };
                self.send_command(cmd);
            }

            // if ui.button("Reset Target").clicked() {
            //     self.status.target_voltage = 0.0;
            //     let cmd = SerialCommand::SetMotorTarget { id, target: 0.0 };
            //     self.send_command(cmd);
            // }
        });

        // target pos
        ui.horizontal(|ui| {
            ui.label("Target Pos:");
            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].target_pos,
                -100.0..=100.0,
            ));
            // if resp.sc
            let mut send_target = None;

            let mut inc = 0.5;

            if !matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Angle)
                    | Some(robotarm_protocol::MotionControlType::AngleOpenLoop)
            ) {
            } else if resp.changed() {
                send_target = Some(self.status[id as usize].target_pos);
            }

            let (min, max) = (
                -10. * self.status[id as usize].gear_ratio,
                10. * self.status[id as usize].gear_ratio,
            );
            if let Some(tgt) = self::scrollable::make_scrollable(
                ui,
                resp,
                // 0.5,
                // (0.1, 1.0),
                3.14 / 8. * self.status[id as usize].gear_ratio,
                // 3.14 / 4.,
                (
                    3.14 / 16. * self.status[id as usize].gear_ratio,
                    3.14 / 4. * self.status[id as usize].gear_ratio,
                    // 3.14 / 8.,
                    // 3.14 / 2.,
                ),
                &mut self.status[id as usize].target_pos,
                min,
                max,
            ) {
                send_target = Some(tgt);
            }

            if !matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Angle)
                    | Some(robotarm_protocol::MotionControlType::AngleOpenLoop)
            ) {
                send_target = None;
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id,

                    // target: ((self.status.target_pos + self.status.angle_offset)
                    //     * self.status.gear_ratio) as f32,
                    target: (self.status[id as usize].target_pos
                        + self.status[id as usize].angle_offset) as f32,
                };
                self.send_command(cmd);
            }
        });

        // target vel
        ui.horizontal(|ui| {
            ui.label("Target Vel:");
            // let resp = ui.add(egui::Slider::new(&mut self.target_vel, -20.0..=20.0));
            let range =
                // -self.status[id as usize].vel_pid_limit..=self.status[id as usize].vel_pid_limit;
                -15.0..=15.0;
            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].target_vel,
                range,
            ));
            let mut send_target = None;

            if !(matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Velocity)
            ) || matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::VelocityOpenLoop)
            )) {
            } else if resp.changed() {
                send_target = Some(self.status[id as usize].target_vel);
            }

            let (min, max) = (
                -self.status[id as usize].vel_pid_limit,
                self.status[id as usize].vel_pid_limit,
            );
            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                5.,
                (2., 10.),
                // 3.14 / 2.,
                // (3.14 / 4., 3.14),
                &mut self.status[id as usize].target_vel,
                min,
                max,
            ) {
                send_target = Some(tgt);
            }

            if !(matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::Velocity)
            ) || matches!(
                self.status[id as usize].motion_control,
                Some(robotarm_protocol::MotionControlType::VelocityOpenLoop)
            )) {
                send_target = None;
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id,
                    target: tgt as f32,
                };
                self.send_command(cmd);
            }

            // if ui.button("Reset Target").clicked() {
            //     self.status.target_vel = 0.0;
            //     let cmd = SerialCommand::SetMotorTarget { id, target: 0.0 };
            //     self.send_command(cmd);
            // }
        });

        // feedforward
        ui.horizontal(|ui| {
            ui.label("Feedforward:");

            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].feed_forward,
                -10.0..=10.0,
            ));
            let mut send_target = None;

            if resp.changed() {
                send_target = Some(self.status[id as usize].feed_forward);
            }

            let (min, max) = (
                -self.status[id as usize].vel_pid_limit,
                self.status[id as usize].vel_pid_limit,
            );
            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                0.1,
                (0.05, 0.25),
                &mut self.status[id as usize].feed_forward,
                min,
                max,
            ) {
                send_target = Some(tgt);
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetFeedForward {
                    id,
                    ff: self.status[id as usize].feed_forward as f32,
                };
                self.send_command(cmd);
            }
        });

        // arm torque
        #[cfg(feature = "nope")]
        ui.horizontal(|ui| {
            let ball_bearing_mass = 3.528; // g

            ui.label("Arm torque (3.528 g * dm, 90 deg):");

            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].feed_forward,
                -10.0..=10.0,
            ));
            let mut send_target = None;

            if resp.changed() {
                send_target = Some(self.status[id as usize].feed_forward);
            }

            let (min, max) = (
                -self.status[id as usize].vel_pid_limit,
                self.status[id as usize].vel_pid_limit,
            );
            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                0.1,
                (0.05, 0.25),
                &mut self.status[id as usize].feed_forward,
                min,
                max,
            ) {
                send_target = Some(tgt);
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetFeedForward {
                    id,
                    ff: self.status[id as usize].feed_forward as f32,
                };
                self.send_command(cmd);
            }
        });

        // angle offset
        #[cfg(feature = "nope")]
        ui.horizontal(|ui| {
            let size = Vec2::splat(16.0);
            let (response, painter) = ui.allocate_painter(size, Sense::hover());

            let rect = response.rect;
            let c = rect.center();
            let r = rect.width() / 2.0 - 1.0;
            let color = Color32::from_gray(128);
            let stroke = Stroke::new(1.0, color);
            painter.circle_stroke(c, r, stroke);

            // draw line from center to edge based on angle
            // let angle = self.status.pos as f32 + self.status.angle_offset as f32;
            let angle = ((self.status[id as usize].pos + self.status[id as usize].angle_offset)
                * self.status[id as usize].gear_ratio) as f32;
            let end_pos = egui::pos2(c.x + r * angle.sin(), c.y - r * angle.cos());

            painter.line_segment([c, end_pos], stroke);

            let prev_offset = self.status[id as usize].angle_offset;

            ui.label("Angle Offset");
            let resp = ui.add(egui::Slider::new(
                &mut self.status[id as usize].angle_offset,
                -10.0..=10.0,
            ));

            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                3.14 / 2.,
                (3.14 / 16., 3.14),
                &mut self.status[id as usize].angle_offset,
                -10.,
                10.,
            ) {
                self.status[id as usize].target_pos =
                    self.status[id as usize].target_pos + (tgt - prev_offset);
            }
        });

        // electrical angle
        ui.horizontal(|ui| {
            ui.label("Elec Angle:");

            let inc = 0.01;
            if ui.button("+").clicked() {
                self.status[id as usize].zero_electrical_angle += inc;
                let cmd = SerialCommand::SetZeroElectricalAngle {
                    id,
                    angle: self.status[id as usize].zero_electrical_angle as f32,
                };
                self.send_command(cmd);
            }
            if ui.button("-").clicked() {
                self.status[id as usize].zero_electrical_angle -= inc;
                let cmd = SerialCommand::SetZeroElectricalAngle {
                    id,
                    angle: self.status[id as usize].zero_electrical_angle as f32,
                };
                self.send_command(cmd);
            }

            // let resp = ui.add(egui::Slider::new(
            //     &mut self.status[id as usize].feed_forward,
            //     -..=10.0,
            // ));
            let resp = ui.add(
                egui::DragValue::new(&mut self.status[id as usize].zero_electrical_angle)
                    .suffix(" rad")
                    .fixed_decimals(5),
            );
            let mut send_target = None;

            if resp.changed() {
                send_target = Some(self.status[id as usize].feed_forward);
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetZeroElectricalAngle {
                    id,
                    angle: self.status[id as usize].zero_electrical_angle as f32,
                };
                self.send_command(cmd);
            }

            if ui.button("Get").clicked() {
                let cmd = SerialCommand::RequestDebugData { id };
                self.send_command(cmd);
            }
        });

        //
    }

    fn col_2(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("col_2_grid")).show(ui, |ui| {
            self::pid_settings::pid_control_dec(
                ui,
                "b0",
                &mut self.status[id as usize].adrc_b0,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                0,
                |id, p| SerialCommand::SetADRCParam {
                    id,
                    adrc_settings: robotarm_protocol::ADRCSettings {
                        b0: Some(p),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();

            self::pid_settings::pid_control_dec(
                ui,
                "speed",
                &mut self.status[id as usize].adrc_speed_factor,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                0,
                |id, p| SerialCommand::SetADRCParam {
                    id,
                    adrc_settings: robotarm_protocol::ADRCSettings {
                        speed_factor: Some(p),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();

            self::pid_settings::pid_control_dec(
                ui,
                "observer bw",
                &mut self.status[id as usize].adrc_observer_bandwidth,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                1,
                |id, p| SerialCommand::SetADRCParam {
                    id,
                    adrc_settings: robotarm_protocol::ADRCSettings {
                        observer_bandwidth: Some(p),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();

            self::pid_settings::pid_control_dec(
                ui,
                "controller bw",
                &mut self.status[id as usize].adrc_controller_bandwidth,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                1,
                |id, p| SerialCommand::SetADRCParam {
                    id,
                    adrc_settings: robotarm_protocol::ADRCSettings {
                        controller_bandwidth: Some(p),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();
            ui.end_row();

            ui.label(RichText::new("vs").monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rad",
                    self.status[id as usize].adrc_internals.0[0]
                ))
                .monospace(),
            );
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rad",
                    self.status[id as usize].adrc_internals.0[1]
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new(super::plot::LABEL_X1).monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rad",
                    self.status[id as usize].adrc_internals.1[0]
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new(super::plot::LABEL_X2).monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rad/s",
                    self.status[id as usize].adrc_internals.1[1]
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new(super::plot::LABEL_X3).monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3}",
                    self.status[id as usize].adrc_internals.1[2]
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Output").monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.5}",
                    self.status[id as usize].adrc_internals.2,
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Output (raw)").monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.5}",
                    self.status[id as usize].adrc_internals.3,
                ))
                .monospace(),
            );
            ui.end_row();

            //
        });
    }

    fn col_3(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("col_3_grid")).show(ui, |ui| {
            // debug motion
            ui.toggle_value(
                &mut self.status[id as usize].debug_motion_sine,
                "Debug Sine",
            );

            if self.status[id as usize].enabled && self.status[id as usize].debug_motion_sine {
                self.status[id as usize].debug_motion_val += 0.01;

                let scale_x = 0.5;
                let scale_y = 0.25;

                self.status[id as usize].target_pos =
                    PI + (self.status[id as usize].debug_motion_val / scale_x).sin() * PI * scale_y;

                let cmd = SerialCommand::SetMotorTarget {
                    id,

                    // target: ((self.status.target_pos + self.status.angle_offset)
                    //     * self.status.gear_ratio) as f32,
                    target: (self.status[id as usize].target_pos
                        + self.status[id as usize].angle_offset) as f32,
                };
                self.send_command(cmd);
            }

            ui.toggle_value(
                &mut self.status[id as usize].debug_motion_steps,
                "Debug Steps",
            );

            if self.status[id as usize].enabled && self.status[id as usize].debug_motion_steps {
                let dt = ui.input(|i| i.stable_dt);
                self.status[id as usize].debug_motion_val += dt as f64;

                // motion should wrap from 0-2 pi
                // motion should jump by step_change, then wait for step_length

                let n_steps = 2;
                let step_length = 1.;
                // let step_change = PI / n_steps as f64;
                let step_change = PI / 16.0;

                let center = PI / 2.;

                // let time =
                //     // self.status[id as usize].debug_motion_val % (n_steps as f64 * step_length);
                //     self.status[id as usize].debug_motion_val;

                let time = self.status[id as usize]
                    .debug_motion_val
                    .rem_euclid(n_steps as f64 * step_length);

                let rem = self.status[id as usize].debug_motion_val - time;

                // debug!("time: {}, rem: {}", time, rem);

                let step_index = (time / step_length).floor();
                let angle = step_index * step_change + center;

                // let angle = rem *

                // let angle = (angle / step_length).floor() * step_length;
                // let angle = angle * step_change;

                if (angle - self.status[id as usize].target_pos).abs() > 0.01 {
                    self.status[id as usize].target_pos = angle;
                    // self.status[id as usize].target_pos = angle;

                    let cmd = SerialCommand::SetMotorTarget {
                        id,

                        // target: ((self.status.target_pos + self.status.angle_offset)
                        //     * self.status.gear_ratio) as f32,
                        target: (self.status[id as usize].target_pos
                            + self.status[id as usize].angle_offset)
                            as f32,
                    };
                    self.send_command(cmd);
                }
            }

            let calibrated = self.status[id as usize].calibration_enabled;
            if ui
                .add(
                    egui::Button::new("Calibrated")
                        .selected(calibrated)
                        .frame_when_inactive(!calibrated)
                        .frame(true),
                )
                .clicked()
            {
                self.status[id as usize].calibration_enabled = !calibrated;
                let cmd = SerialCommand::SetEncoderCalibration {
                    id,
                    enable: !calibrated,
                };
                self.send_command(cmd);
            }
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity LPF",
                &mut self.status[id as usize].lpf_vel,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_lpf,
            );
            ui.end_row();
        });
    }

    // velocity PID controls
    #[cfg(feature = "nope")]
    fn col_2(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("velocity_pid_grid {id}")).show(ui, |ui| {
            self::pid_settings::pid_control(
                ui,
                "Velocity KP",
                &mut self.status[id as usize].vel_pid_p,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_p,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KI",
                &mut self.status[id as usize].vel_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_i,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KD",
                &mut self.status[id as usize].vel_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_d,
            );
            ui.end_row();
            // ui.separator();
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity LPF",
                &mut self.status[id as usize].lpf_vel,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_lpf,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity PID Limit",
                &mut self.status[id as usize].vel_pid_limit,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_limit,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity PID I Band",
                &mut self.status[id as usize].vel_pid_i_band,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_i_band,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity PID D LPF",
                &mut self.status[id as usize].vel_pid_d_lpf,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_d_lpf,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity PID FF",
                &mut self.status[id as usize].vel_pid_feed_forward,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_vel_feed_forward,
            );
            ui.end_row();

            let calibrated = self.status[id as usize].calibration_enabled;
            if ui
                .add(
                    egui::Button::new("Calibrated")
                        .selected(calibrated)
                        .frame_when_inactive(!calibrated)
                        .frame(true),
                )
                .clicked()
            {
                self.status[id as usize].calibration_enabled = !calibrated;
                let cmd = SerialCommand::SetEncoderCalibration {
                    id,
                    enable: !calibrated,
                };
                self.send_command(cmd);
            }
            ui.end_row();
        });
    }

    // position PID controls
    #[cfg(feature = "nope")]
    fn col_3(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("position_pid_grid {id}")).show(ui, |ui| {
            self::pid_settings::pid_control(
                ui,
                "Position KP",
                &mut self.status[id as usize].pos_pid_p,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, p| SerialCommand::SetAnglePID {
                    id,
                    pid_settings: robotarm_protocol::types::PIDSettings {
                        p: Some(p),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position KI",
                &mut self.status[id as usize].pos_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, i| SerialCommand::SetAnglePID {
                    id,
                    pid_settings: robotarm_protocol::types::PIDSettings {
                        i: Some(i),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position KD",
                &mut self.status[id as usize].pos_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, d| SerialCommand::SetAnglePID {
                    id,
                    pid_settings: robotarm_protocol::types::PIDSettings {
                        d: Some(d),
                        ..Default::default()
                    },
                },
            );
            ui.end_row();
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position LPF",
                &mut self.status[id as usize].lpf_angle,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, lpf| SerialCommand::SetLPF {
                    id,
                    lpf_vel: None,
                    lpf_angle: Some(lpf),
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position PID Limit",
                &mut self.status[id as usize].pos_pid_limit,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_pos_limit,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position PID I Band",
                &mut self.status[id as usize].pos_pid_i_band,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_pos_i_band,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position PID D LPF",
                &mut self.status[id as usize].pos_pid_d_lpf,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                self::pid_settings::set_pos_d_lpf,
            );
            ui.end_row();

            // self::pid_settings::pid_control(
            //     ui,
            //     "Position PID FF",
            //     &mut self.status[id as usize].pos_pid_d_feed_forward,
            //     &self.serial_cmd_tx.as_ref().unwrap(),
            //     id,
            //     self::pid_settings::set_pos_feed_forward,
            // );
            // ui.end_row();
        });
    }
}
