use std::time::Instant;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use egui::RichText;

use robotarm_protocol::{SerialCommand, SerialLogMessage};

use serde::{Deserialize, Serialize};

#[derive(Default, Deserialize, Serialize)]
#[serde(default)]
pub struct App {
    #[serde(skip)]
    pub plot: super::plot::DataPlot,

    #[serde(skip)]
    pub t0: Option<u64>,

    #[serde(skip)]
    pub serial_log_rx: Option<tokio::sync::mpsc::Receiver<SerialLogMessage>>,
    #[serde(skip)]
    pub serial_cmd_tx: Option<tokio::sync::mpsc::Sender<SerialCommand>>,

    #[serde(skip)]
    pub target_pos: f64,
    #[serde(skip)]
    pub target_vel: f64,

    pub current: f32,
    pub voltage: (f32, f32),

    pub vel_pid_p: f32,
    pub vel_pid_i: f32,
    pub vel_pid_d: f32,
    pub vel_pid_ramp: f32,
    pub vel_pid_limit: f32,

    pub pos_pid_p: f32,
    pub pos_pid_i: f32,
    pub pos_pid_d: f32,
    pub pos_pid_ramp: f32,
    pub pos_pid_limit: f32,

    pub lpf_angle: f32,
    pub lpf_vel: f32,
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
        ctx.request_repaint_after(std::time::Duration::from_millis(1_000 / 120));
        // ctx.request_repaint_after(std::time::Duration::from_millis(1_000 / 60));

        // if cfg!(debug_assertions) && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        //     ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        // }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        // debug!("Fetching serial data");
        self.get_from_channels();
        // debug!("Done");

        egui::SidePanel::left("Left").show(ctx, |ui| {
            ui.heading("Plot Settings");
        });

        egui::TopBottomPanel::bottom("Bottom").show(ctx, |ui| {
            self.controls(ui);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // ui.heading("Hello World!");
            self.plot.show_plot(ui);
        });
    }
}
