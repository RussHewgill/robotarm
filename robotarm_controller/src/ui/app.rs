use std::time::Instant;

use anyhow::{Context, Result, anyhow, bail, ensure};
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
}

impl App {
    pub fn new(
        serial_log_rx: tokio::sync::mpsc::Receiver<SerialLogMessage>,
        serial_cmd_tx: tokio::sync::mpsc::Sender<SerialCommand>,
    ) -> Self {
        Self {
            plot: super::plot::DataPlot::default(),
            // t0: Some(Instant::now()),
            t0: None,
            serial_log_rx: Some(serial_log_rx),
            serial_cmd_tx: Some(serial_cmd_tx),
        }
    }

    fn get_from_channels(&mut self) {
        if let Some(rx) = &mut self.serial_log_rx {
            while let Ok(msg) = rx.try_recv() {
                // let t = msg.timestamp.duration_since(self.t0.unwrap()).as_secs_f64();
                // self.plot.add_point(t, msg.angle, msg.velocity);
                match msg {
                    SerialLogMessage::MotorData {
                        id,
                        timestamp,
                        target,
                        position,
                        velocity,
                    } => {
                        // let t = timestamp.duration_since(self.t0.unwrap()).as_secs_f64();
                        if let Some(t0) = self.t0 {
                            let t = timestamp as f64 * 1e-6 - t0 as f64 * 1e-6;
                            self.plot.add_point(t, position, velocity);
                        } else {
                            self.t0 = Some(timestamp);
                            let t = timestamp as f64 * 1e-6;
                            self.plot.add_point(t, position, velocity);
                        }
                    }
                }
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(50));

        if cfg!(debug_assertions) && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        self.get_from_channels();

        egui::CentralPanel::default().show(ctx, |ui| {
            // ui.heading("Hello World!");
            self.plot.show_plot(ui);
        });
    }
}
