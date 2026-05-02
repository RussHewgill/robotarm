use anyhow::{Context, Result, anyhow, bail, ensure};
use egui_extras::Size;
use tracing::{debug, error, info, trace, warn};

use egui::{Color32, RichText, Sense, Stroke, Vec2};

use crate::ui::{app::App, controls::scrollable::make_scrollable};
use robotarm_protocol::SerialCommand;

mod pid_settings {
    use anyhow::{Context, Result, anyhow, bail, ensure};
    use tracing::{debug, error, info, trace, warn};

    use robotarm_protocol::SerialCommand;

    pub(super) fn pid_control<F>(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut F,
        // tx: &tokio::sync::mpsc::Sender<SerialCommand>,
        tx: &crossbeam_channel::Sender<SerialCommand>,
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
            p: Some(p),
            i: None,
            d: None,
            ramp: None,
            limit: None,
        }
    }

    pub(super) fn set_vel_i(id: u8, i: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            p: None,
            i: Some(i),
            d: None,
            ramp: None,
            limit: None,
        }
    }

    pub(super) fn set_vel_d(id: u8, d: f32) -> SerialCommand {
        SerialCommand::SetVelocityPID {
            id,
            p: None,
            i: None,
            d: Some(d),
            ramp: None,
            limit: None,
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
            p: None,
            i: None,
            d: None,
            ramp: None,
            limit: Some(limit as f32),
        }
    }

    pub(super) fn set_pos_limit(id: u8, limit: f32) -> SerialCommand {
        SerialCommand::SetAnglePID {
            id,
            p: None,
            i: None,
            d: None,
            ramp: None,
            limit: Some(limit),
        }
    }
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

            ui.label("Gear Ratio");
            ui.end_row();
            let resp = ui.add(
                egui::Slider::new(&mut self.status[id as usize].gear_ratio, -20.0..=30.0).integer(),
            );

            //
        });
    }

    // motor controls
    fn col_1(&mut self, ui: &mut egui::Ui, id: u8) {
        egui::Grid::new(format!("Motor Controls Grid {id}")).show(ui, |ui| {
            if ui.button("Enable Motor").clicked() {
                let cmd = SerialCommand::SetEnabled { id, enabled: true };
                self.send_command(cmd);
            }
            if ui.button("Disable Motor").clicked() {
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
            ui.end_row();
        });

        // ui.horizontal(|ui| {
        //     ui.label("Gear Ratio:");
        //     let resp = ui.add(egui::Slider::new(&mut self.gear_ratio, -0.5..=10.0));
        // });

        // target torque
        ui.horizontal(|ui| {
            ui.label("Target Torque:");
            let range =
                -self.status[id as usize].vel_pid_limit..=self.status[id as usize].vel_pid_limit;
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

            let (min, max) = (
                -self.status[id as usize].vel_pid_limit,
                self.status[id as usize].vel_pid_limit,
            );
            if let Some(tgt) = make_scrollable(
                ui,
                resp,
                0.5,
                (0.1, 1.0),
                &mut self.status[id as usize].target_voltage,
                min,
                max,
            ) {
                send_target = Some(tgt);
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
                -self.status[id as usize].vel_pid_limit..=self.status[id as usize].vel_pid_limit;
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
                3.14 / 2.,
                (3.14 / 4., 3.14),
                &mut self.status[id as usize].target_vel,
                min,
                max,
            ) {
                send_target = Some(tgt);
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

        //
    }

    // velocity PID controls
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
        });
    }

    // position PID controls
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
                &mut self.status[id as usize].pos_pid_i,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, i| SerialCommand::SetAnglePID {
                    id,
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
                &mut self.status[id as usize].pos_pid_d,
                &self.serial_cmd_tx.as_ref().unwrap(),
                id,
                |id, d| SerialCommand::SetAnglePID {
                    id,
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
        });

        #[cfg(feature = "nope")]
        egui::Grid::new("position_pid_grid").show(ui, |ui| {
            ui.label("Position KP: ");
            if ui.add(egui::DragValue::new(&mut self.pos_pid_p)).changed() {
                if let Some(tx) = &self.serial_cmd_tx {
                    let cmd = SerialCommand::SetAnglePID {
                        id,
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
                        id,
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
                        id,
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
                        i_vel: None,
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
