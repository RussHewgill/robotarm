use anyhow::{Context, Result, anyhow, bail, ensure};
use egui_extras::Size;
use tracing::{debug, error, info, trace, warn};

use egui::RichText;

use crate::ui::app::App;
use robotarm_protocol::SerialCommand;

mod pid_settings {
    use anyhow::{Context, Result, anyhow, bail, ensure};
    use tracing::{debug, error, info, trace, warn};

    use robotarm_protocol::SerialCommand;

    pub(super) fn pid_control(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut f32,
        // tx: &tokio::sync::mpsc::Sender<SerialCommand>,
        tx: &crossbeam_channel::Sender<SerialCommand>,
        cmd_fn: impl Fn(f32) -> SerialCommand,
    ) {
        ui.label(label);

        let resp = ui.add(egui::DragValue::new(value).fixed_decimals(4));

        let send_resp = ui.button("Send");
        let zero_resp = ui.button("Zero");

        if (resp.lost_focus()
            && resp
                .ctx
                .input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Tab)))
            || send_resp.clicked()
        {
            let cmd = cmd_fn(*value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }

        if zero_resp.clicked() {
            *value = 0.0;
            let cmd = cmd_fn(*value);
            if let Err(e) = tx.try_send(cmd) {
                error!("Failed to send command: {}", e);
            }
        }
    }

    pub(super) fn set_vel_p(p: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id: 0,
            p: Some(p),
            i: None,
            d: None,
            ramp: None,
            limit: None,
        }
    }

    pub(super) fn set_vel_i(i: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id: 0,
            p: None,
            i: Some(i),
            d: None,
            ramp: None,
            limit: None,
        }
    }

    pub(super) fn set_vel_d(d: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id: 0,
            p: None,
            i: None,
            d: Some(d),
            ramp: None,
            limit: None,
        }
    }

    pub(super) fn set_vel_lpf(lpf: f32) -> SerialCommand {
        SerialCommand::SetLPF {
            id: 0,
            lpf_vel: Some(lpf),
            lpf_angle: None,
        }
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
    pub fn controls(&mut self, ui: &mut egui::Ui) {
        // StripBuilder::new(ui)
        //     .size(Size::relative(0.3))

        ui.columns_const(|[col_0, col_1, col_2, col_3]| {
            col_0.vertical(|ui| {
                self.col_0(ui);
            });
            col_1.vertical(|ui| {
                self.col_1(ui);
            });
            col_2.vertical(|ui| {
                self.col_2(ui);
            });
            col_3.vertical(|ui| {
                self.col_3(ui);
            });
        });
    }

    fn col_0(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("Motor Data Grid").show(ui, |ui| {
            ui.label(RichText::new("Current").monospace());
            ui.label(RichText::new(format!("{:>+0.3} A", self.status.current)).monospace());
            ui.end_row();

            ui.label(RichText::new("Voltage").monospace());
            ui.label(RichText::new(format!("{:>+0.3} V", self.status.voltage.0)).monospace());
            ui.label(RichText::new(format!("{:>+0.3} V", self.status.voltage.1)).monospace());
            ui.end_row();

            ui.label(RichText::new("Angle").monospace());
            ui.label(RichText::new(format!("{:>+0.3} rad", self.status.angle)).monospace());
            ui.end_row();

            ui.label(RichText::new("Position").monospace());
            ui.label(RichText::new(format!("{:>+0.3} rad", self.status.pos,)).monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rev",
                    self.status.pos / (2.0 * std::f64::consts::PI)
                ))
                .monospace(),
            );
            ui.end_row();

            ui.label(RichText::new("Velocity").monospace());
            ui.label(RichText::new(format!("{:>+0.3} rad/s", self.status.vel)).monospace());
            ui.label(
                RichText::new(format!(
                    "{:>+0.3} rpm",
                    self.status.vel * 60.0 / (2.0 * std::f64::consts::PI)
                ))
                .monospace(),
            );
            ui.end_row();
        });
    }

    fn col_1(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("Motor Controls Grid").show(ui, |ui| {
            if ui.button("Enable Motor").clicked() {
                let cmd = SerialCommand::SetEnabled {
                    id: 0,
                    enabled: true,
                };
                self.send_command(cmd);
            }
            if ui.button("Disable Motor").clicked() {
                let cmd = SerialCommand::SetEnabled {
                    id: 0,
                    enabled: false,
                };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Torque");
            let but = if matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Torque)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                let cmd = SerialCommand::SetModeTorque { id: 0 };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Velocity Open Loop");
            let but = if matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::VelocityOpenLoop)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                let cmd = SerialCommand::SetModeVelocityOpenLoop { id: 0 };
                self.send_command(cmd);
            }
            ui.end_row();

            let but = egui::Button::new("Set Velocity");
            let but = if matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Velocity)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                self.send_command(SerialCommand::SetModeVelocity { id: 0 });
            }
            ui.end_row();

            let but = egui::Button::new("Set Angle");
            let but = if matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Angle)
            ) {
                but.fill(egui::Color32::LIGHT_GREEN)
            } else {
                but
            };
            if ui.add(but).clicked() {
                self.send_command(SerialCommand::SetModeAngle { id: 0 });
            }
            ui.end_row();
        });

        // ui.horizontal(|ui| {
        //     ui.label("Gear Ratio:");
        //     let resp = ui.add(egui::Slider::new(&mut self.gear_ratio, -0.5..=10.0));
        // });

        ui.horizontal(|ui| {
            ui.label("Target Torque:");
            let resp = ui.add(egui::Slider::new(
                &mut self.status.target_voltage,
                -self.status.vel_pid_limit..=self.status.vel_pid_limit,
            ));

            let mut send_target = None;

            if !matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Torque)
            ) {
            } else if resp.changed() {
                send_target = Some(self.status.target_voltage);
            } else if resp.hovered() {
                let delta = ui.input(|i| {
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
                    if delta.y > 0. && self.status.target_voltage < self.status.vel_pid_limit {
                        self.status.target_voltage += 0.5;
                        send_target = Some(self.status.target_voltage);
                    } else if delta.y < 0.
                        && self.status.target_voltage > -self.status.vel_pid_limit
                    {
                        self.status.target_voltage -= 0.5;
                        send_target = Some(self.status.target_voltage);
                    }
                }
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id: 0,
                    target: tgt as f32,
                };
                self.send_command(cmd);
            }

            if ui.button("Reset Target").clicked() {
                self.status.target_voltage = 0.0;
                let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
                self.send_command(cmd);
            }
        });

        ui.horizontal(|ui| {
            ui.label("Target Pos:");
            let resp = ui.add(egui::Slider::new(
                &mut self.status.target_pos,
                -100.0..=100.0,
            ));
            // if resp.sc
            let mut send_target = None;

            let mut inc = 0.5;

            if !matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Angle)
            ) {
            } else if resp.changed() {
                send_target = Some(self.status.target_pos);
            } else if resp.hovered() {
                let delta = ui.input(|i| {
                    if i.modifiers.shift {
                        inc = 1.0;
                    } else if i.modifiers.ctrl {
                        inc = 0.1;
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
                    if delta.y > 0. && self.status.target_pos < 10. {
                        self.status.target_pos += inc;
                        send_target = Some(self.status.target_pos);
                    } else if delta.y < 0. && self.status.target_pos > -10. {
                        self.status.target_pos -= inc;
                        send_target = Some(self.status.target_pos);
                    }
                }
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id: 0,
                    target: self.status.target_pos as f32,
                };
                self.send_command(cmd);
            }

            if ui.button("Reset Target").clicked() {
                self.status.target_pos = 0.0;
                let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
                self.send_command(cmd);
            }

            // if ui.button("Zero Position").clicked() {
            //     let cmd = SerialCommand::SetSensorOffset {
            //         id: 0,
            //         offset: Some(self.status.pos as f32),
            //     };
            //     self.send_command(cmd);

            //     let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
            //     self.send_command(cmd);
            // }
        });

        ui.horizontal(|ui| {
            ui.label("Target Vel:");
            // let resp = ui.add(egui::Slider::new(&mut self.target_vel, -20.0..=20.0));
            let resp = ui.add(egui::Slider::new(
                &mut self.status.target_vel,
                -self.status.vel_pid_limit..=self.status.vel_pid_limit,
            ));
            let mut send_target = None;

            if !(matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::Velocity)
            ) || matches!(
                self.status.motion_control,
                Some(robotarm_protocol::MotionControlType::VelocityOpenLoop)
            )) {
            } else if resp.changed() {
                send_target = Some(self.status.target_vel);
            } else if resp.hovered() {
                let delta = ui.input(|i| {
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
                    if delta.y > 0. && self.status.target_vel < self.status.vel_pid_limit {
                        // self.target_vel += 1.0;
                        // self.status.target_vel += 3.14 / 2.;
                        self.status.target_vel += 3.14 * 2.;
                        // send_target = Some(self.target_vel * self.gear_ratio);
                        send_target = Some(self.status.target_vel);
                    } else if delta.y < 0. && self.status.target_vel > -self.status.vel_pid_limit {
                        // self.target_vel -= 1.0;
                        // self.status.target_vel -= 3.14 / 2.;
                        self.status.target_vel -= 3.14 * 2.;
                        // send_target = Some(self.target_vel * self.gear_ratio);
                        send_target = Some(self.status.target_vel);
                    }
                }
            }

            if let Some(tgt) = send_target {
                let cmd = SerialCommand::SetMotorTarget {
                    id: 0,
                    target: tgt as f32,
                };
                self.send_command(cmd);
            }

            if ui.button("Reset Target").clicked() {
                self.status.target_vel = 0.0;
                let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
                self.send_command(cmd);
            }
        });

        ui.horizontal(|ui| {
            ui.label("Feedforward:");

            let resp = ui.add(egui::Slider::new(
                &mut self.status.feed_forward,
                -10.0..=10.0,
            ));

            if resp.changed() {
                let cmd = SerialCommand::SetFeedForward {
                    id: 0,
                    ff: self.status.feed_forward as f32,
                };
                self.send_command(cmd);
            }
        });
    }

    fn col_2(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("velocity_pid_grid").show(ui, |ui| {
            self::pid_settings::pid_control(
                ui,
                "Velocity KP",
                &mut self.status.vel_pid_p,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_p,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KI",
                &mut self.status.vel_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_i,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KD",
                &mut self.status.vel_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_d,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity LPF",
                &mut self.status.lpf_vel,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_lpf,
            );
            ui.end_row();
        });
    }

    fn col_3(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("position_pid_grid").show(ui, |ui| {
            self::pid_settings::pid_control(
                ui,
                "Position KP",
                &mut self.status.pos_pid_p,
                &self.serial_cmd_tx.as_ref().unwrap(),
                |p| SerialCommand::SetAnglePID {
                    id: 0,
                    p: Some(p),
                    i: None,
                    d: None,
                    ramp: None,
                    limit: None,
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position KI",
                &mut self.status.pos_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                |i| SerialCommand::SetAnglePID {
                    id: 0,
                    p: None,
                    i: Some(i),
                    d: None,
                    ramp: None,
                    limit: None,
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position KD",
                &mut self.status.pos_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                |d| SerialCommand::SetAnglePID {
                    id: 0,
                    p: None,
                    i: None,
                    d: Some(d),
                    ramp: None,
                    limit: None,
                },
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Position LPF",
                &mut self.status.lpf_angle,
                &self.serial_cmd_tx.as_ref().unwrap(),
                |lpf| SerialCommand::SetLPF {
                    id: 0,
                    lpf_vel: None,
                    lpf_angle: Some(lpf),
                },
            );
            ui.end_row();
        });

        #[cfg(feature = "nope")]
        egui::Grid::new("position_pid_grid").show(ui, |ui| {
            ui.label("Position KP: ");
            if ui.add(egui::DragValue::new(&mut self.pos_pid_p)).changed() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetAnglePID {
                        id: 0,
                        p: Some(self.pos_pid_p),
                        i: None,
                        d: None,
                        ramp: None,
                        limit: None,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
            ui.end_row();

            ui.label("Position KI: ");
            if ui.add(egui::DragValue::new(&mut self.pos_pid_i)).changed() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetAnglePID {
                        id: 0,
                        p: None,
                        i: Some(self.pos_pid_i),
                        d: None,
                        ramp: None,
                        limit: None,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
            ui.end_row();

            ui.label("Position KD: ");
            if ui.add(egui::DragValue::new(&mut self.pos_pid_d)).changed() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetAnglePID {
                        id: 0,
                        p: None,
                        i: None,
                        d: Some(self.pos_pid_d),
                        ramp: None,
                        limit: None,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
            ui.end_row();

            ui.label("Position LPF: ");
            if ui
                .add(egui::DragValue::new(&mut self.lpf_angle).fixed_decimals(4))
                .changed()
            {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetLPF {
                        id: 0,
                        lpf_vel: None,
                        lpf_angle: Some(self.lpf_angle),
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
            ui.end_row();
        });
    }
}
