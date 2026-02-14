use anyhow::{Context, Result, anyhow, bail, ensure};
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
    pub fn controls(&mut self, ui: &mut egui::Ui) {
        ui.columns_const(|[col_1, col_2, col_3]| {
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

    fn col_1(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Enabled Motor").clicked() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetEnabled {
                        id: 0,
                        enabled: true,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }

            if ui.button("Disable Motor").clicked() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetEnabled {
                        id: 0,
                        enabled: false,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
        });

        if ui.button("Set Torque").clicked() {
            if let Some(tx) = &self.serial_cmd_tx {
                let cmd = SerialCommand::SetModeTorque { id: 0 };
                if let Err(e) = tx.try_send(cmd) {
                    error!("Failed to send command: {}", e);
                }
            }
        }

        if ui.button("Set Velocity Open Loop").clicked() {
            if let Some(tx) = &self.serial_cmd_tx {
                let cmd = SerialCommand::SetModeVelocityOpenLoop { id: 0 };
                if let Err(e) = tx.try_send(cmd) {
                    error!("Failed to send command: {}", e);
                }
            }
        }

        if ui.button("Set Velocity").clicked() {
            if let Some(tx) = &self.serial_cmd_tx {
                let cmd = SerialCommand::SetModeVelocity { id: 0 };
                if let Err(e) = tx.try_send(cmd) {
                    error!("Failed to send command: {}", e);
                }
            }
        }

        if ui.button("Set Angle").clicked() {
            if let Some(tx) = &self.serial_cmd_tx {
                let cmd = SerialCommand::SetModeAngle { id: 0 };
                if let Err(e) = tx.try_send(cmd) {
                    error!("Failed to send command: {}", e);
                }
            }
        }

        ui.horizontal(|ui| {
            ui.label("Target Pos:");
            let resp = ui.add(egui::Slider::new(&mut self.target_pos, -10.0..=10.0));
            // if resp.sc
            let mut send_target = None;

            if resp.changed() {
                send_target = Some(self.target_pos);
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
                    if delta.y > 0. && self.target_pos < 10. {
                        self.target_pos += 0.5;
                        send_target = Some(self.target_pos);
                    } else if delta.y < 0. && self.target_pos > -10. {
                        self.target_pos -= 0.5;
                        send_target = Some(self.target_pos);
                    }
                }
            }

            if let Some(tgt) = send_target {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetMotorTarget {
                        id: 0,
                        target: self.target_pos as f32,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }

            if ui.button("Reset Target").clicked() {
                self.target_pos = 0.0;
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("Target Vel:");
            let resp = ui.add(egui::Slider::new(&mut self.target_vel, -20.0..=20.0));
            let mut send_target = None;

            if resp.changed() {
                send_target = Some(self.target_vel);
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
                    if delta.y > 0. && self.target_vel < 20. {
                        self.target_vel += 0.5;
                        send_target = Some(self.target_vel);
                    } else if delta.y < 0. && self.target_vel > -20. {
                        self.target_vel -= 0.5;
                        send_target = Some(self.target_vel);
                    }
                }
            }

            if let Some(tgt) = send_target {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetMotorTarget {
                        id: 0,
                        target: tgt as f32,
                    };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }

            if ui.button("Reset Target").clicked() {
                self.target_vel = 0.0;
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetMotorTarget { id: 0, target: 0.0 };
                    if let Err(e) = tx.try_send(cmd) {
                        error!("Failed to send command: {}", e);
                    }
                }
            }
            //
        });

        ui.label(RichText::new(format!("Current: {:>+0.3} A", self.current)).monospace());
        ui.label(
            RichText::new(format!(
                "Voltage: {:>+0.3} V, {:>+0.3} V",
                self.voltage.0, self.voltage.1
            ))
            .monospace(),
        );

        ui.label(RichText::new(format!("Angle: {:>+0.3} A", self.pos)).monospace());
        ui.label(RichText::new(format!("Velocity: {:>+0.3} A", self.vel)).monospace());
    }

    fn col_2(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("velocity_pid_grid").show(ui, |ui| {
            self::pid_settings::pid_control(
                ui,
                "Velocity KP",
                &mut self.vel_pid_p,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_p,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KI",
                &mut self.vel_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_i,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity KD",
                &mut self.vel_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                self::pid_settings::set_vel_d,
            );
            ui.end_row();

            self::pid_settings::pid_control(
                ui,
                "Velocity LPF",
                &mut self.lpf_vel,
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
                &mut self.pos_pid_p,
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
                &mut self.pos_pid_i,
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
                &mut self.pos_pid_d,
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
                &mut self.lpf_angle,
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
