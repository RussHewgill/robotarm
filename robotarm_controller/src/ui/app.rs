use std::time::Instant;

use anyhow::{Context, Result, anyhow, bail, ensure};
use egui::RichText;
use tracing::{debug, error, info, trace, warn};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

use serde::{Deserialize, Serialize};

#[derive(Default, Deserialize, Serialize)]
#[serde(default)]
pub struct App {
    #[serde(skip)]
    plot: super::plot::DataPlot,

    #[serde(skip)]
    t0: Option<u64>,

    #[serde(skip)]
    serial_log_rx: Option<tokio::sync::mpsc::Receiver<SerialLogMessage>>,
    #[serde(skip)]
    serial_cmd_tx: Option<tokio::sync::mpsc::Sender<SerialCommand>>,

    #[serde(skip)]
    target_pos: f64,
    #[serde(skip)]
    target_vel: f64,

    current: f32,
    voltage: (f32, f32),

    vel_pid_p: f32,
    vel_pid_i: f32,
    vel_pid_d: f32,
    vel_pid_ramp: f32,
    vel_pid_limit: f32,

    pos_pid_p: f32,
    pos_pid_i: f32,
    pos_pid_d: f32,
    pos_pid_ramp: f32,
    pos_pid_limit: f32,

    lpf_angle: f32,
    lpf_vel: f32,
}

impl App {
    pub fn new(
        serial_log_rx: tokio::sync::mpsc::Receiver<SerialLogMessage>,
        serial_cmd_tx: tokio::sync::mpsc::Sender<SerialCommand>,
    ) -> Self {
        let mut plot = super::plot::DataPlot::default();
        // plot.window_time = 10.0;
        Self {
            plot,
            // t0: Some(Instant::now()),
            t0: None,
            serial_log_rx: Some(serial_log_rx),
            serial_cmd_tx: Some(serial_cmd_tx),

            target_pos: 0.,
            target_vel: 0.,

            current: 0.,
            voltage: (0., 0.),

            vel_pid_p: 0.,
            vel_pid_i: 0.,
            vel_pid_d: 0.,
            vel_pid_ramp: 0.,
            vel_pid_limit: 0.,

            pos_pid_p: 0.,
            pos_pid_i: 0.,
            pos_pid_d: 0.,
            pos_pid_ramp: 0.,
            pos_pid_limit: 0.,

            lpf_angle: 0.,
            lpf_vel: 0.,
        }
    }

    fn get_from_channels(&mut self) {
        if let Some(rx) = &mut self.serial_log_rx {
            while let Ok(msg) = rx.try_recv() {
                // let t = msg.timestamp.duration_since(self.t0.unwrap()).as_secs_f64();
                // self.plot.add_point(t, msg.angle, msg.velocity);
                match msg {
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
                        debug!("Got motor PID settings");

                        self.vel_pid_p = vel_p;
                        self.vel_pid_i = vel_i;
                        self.vel_pid_d = vel_d;
                        self.vel_pid_ramp = vel_ramp;
                        self.vel_pid_limit = vel_limit;

                        self.pos_pid_p = angle_p;
                        self.pos_pid_i = angle_i;
                        self.pos_pid_d = angle_d;
                        self.pos_pid_ramp = angle_ramp;
                        self.pos_pid_limit = angle_limit;

                        self.lpf_angle = lpf_angle;
                        self.lpf_vel = lpf_vel;
                    }
                    SerialLogMessage::MotorData {
                        id,
                        timestamp,
                        position,
                        angle,
                        velocity,
                        target_position,
                        target_velocity,
                        motor_current,
                        motor_voltage,
                    } => {
                        // debug!("Got motor data");

                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;
                            self.plot.add_point_angle(t, angle as f64);
                            self.plot.add_point_vel(t, velocity as f64);
                            self.plot.add_point_target_vel(t, target_velocity as f64);
                            self.plot.add_point_target_pos(t, target_position as f64);
                        } else {
                            self.t0 = Some(timestamp);
                        }

                        self.target_pos = target_position as f64;
                        self.target_vel = target_velocity as f64;

                        self.current = motor_current;
                        self.voltage = motor_voltage;
                    }
                }
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(50));

        // if cfg!(debug_assertions) && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        //     ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        // }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        self.get_from_channels();

        egui::TopBottomPanel::bottom("Bottom").show(ctx, |ui| {
            ui.columns_const(|[col_1, col_2, col_3]| {
                col_1.vertical(|ui| {
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
                        ui.label("Target Velocity:");
                        if ui
                            .add(egui::Slider::new(&mut self.target_vel, -20.0..=20.0))
                            .changed()
                        {
                            if let Some(tx) = &self.serial_cmd_tx {
                                let cmd = SerialCommand::SetMotorTarget {
                                    id: 0,
                                    target: self.target_vel as f32,
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

                    ui.label(
                        RichText::new(format!("Current: {:>+0.3} A", self.current)).monospace(),
                    );
                    ui.label(
                        RichText::new(format!(
                            "Voltage: {:>+0.3} V, {:>+0.3} V",
                            self.voltage.0, self.voltage.1
                        ))
                        .monospace(),
                    );
                });

                col_2.vertical(|ui| {
                    egui::Grid::new("velocity_pid_grid").show(ui, |ui| {
                        ui.label("Velocity KP: ");
                        if ui.add(egui::DragValue::new(&mut self.vel_pid_p)).changed() {
                            if let Some(tx) = &self.serial_cmd_tx {
                                let cmd = SerialCommand::SetVelocityPID {
                                    id: 0,
                                    p: Some(self.vel_pid_p),
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

                        ui.label("Velocity KI: ");
                        if ui.add(egui::DragValue::new(&mut self.vel_pid_i)).changed() {
                            if let Some(tx) = &self.serial_cmd_tx {
                                let cmd = SerialCommand::SetVelocityPID {
                                    id: 0,
                                    p: None,
                                    i: Some(self.vel_pid_i),
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

                        ui.label("Velocity KD: ");
                        if ui.add(egui::DragValue::new(&mut self.vel_pid_d)).changed() {
                            if let Some(tx) = &self.serial_cmd_tx {
                                let cmd = SerialCommand::SetVelocityPID {
                                    id: 0,
                                    p: None,
                                    i: None,
                                    d: Some(self.vel_pid_d),
                                    ramp: None,
                                    limit: None,
                                };
                                if let Err(e) = tx.try_send(cmd) {
                                    error!("Failed to send command: {}", e);
                                }
                            }
                        }
                        ui.end_row();

                        ui.label("Velocity LPF: ");
                        if ui
                            .add(egui::DragValue::new(&mut self.lpf_vel).fixed_decimals(4))
                            .changed()
                        {
                            if let Some(tx) = &self.serial_cmd_tx {
                                let cmd = SerialCommand::SetLPF {
                                    id: 0,
                                    lpf_vel: Some(self.lpf_vel),
                                    lpf_angle: None,
                                };
                                if let Err(e) = tx.try_send(cmd) {
                                    error!("Failed to send command: {}", e);
                                }
                            }
                        }
                        ui.end_row();
                    });
                });

                col_3.vertical(|ui| {
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
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // ui.heading("Hello World!");
            self.plot.show_plot(ui);
        });
    }
}
