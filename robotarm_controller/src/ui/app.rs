use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use egui::RichText;
use std::time::{Duration, Instant};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

use serde::{Deserialize, Serialize};

use crate::simplefoc::types::FocStatus;

#[derive(Default, Deserialize, Serialize)]
// #[serde(default)]
pub struct App {
    pub plot: super::plot::DataPlot,

    #[serde(skip)]
    pub t0: Option<u64>,

    // #[serde(skip)]
    // pub serial_log_rx: Option<tokio::sync::mpsc::Receiver<SerialLogMessage>>,
    // #[serde(skip)]
    // pub serial_cmd_tx: Option<tokio::sync::mpsc::Sender<SerialCommand>>,
    #[serde(skip)]
    pub serial_log_rx: Option<crossbeam_channel::Receiver<SerialLogMessage>>,
    #[serde(skip)]
    pub serial_cmd_tx: Option<crossbeam_channel::Sender<SerialCommand>>,
    #[serde(skip)]
    pub ui_cmd_rx: Option<crossbeam_channel::Receiver<crate::ui::UiCommand>>,

    pub update_interval: Duration,
    #[serde(skip)]
    pub(super) last_update: Option<Instant>,
    // pub motion_control:
    #[serde(skip)]
    pub status: FocStatus,
}

impl App {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        // serial_log_rx: tokio::sync::mpsc::Receiver<SerialLogMessage>,
        // serial_cmd_tx: tokio::sync::mpsc::Sender<SerialCommand>,
        serial_log_rx: crossbeam_channel::Receiver<SerialLogMessage>,
        serial_cmd_tx: crossbeam_channel::Sender<SerialCommand>,
        ui_cmd_rx: crossbeam_channel::Receiver<crate::ui::UiCommand>,
    ) -> Self {
        let mut out = if let Some(storage) = cc.storage {
            // debug!("Restoring app state");
            let out: App = match eframe::get_value(storage, eframe::APP_KEY) {
                Some(out) => out,
                None => {
                    // warn!("No previous app state found, using default");
                    App::default()
                }
            };
            // debug!("out.plot = {:#?}", out.plot);
            out
        } else {
            // warn!("using default app state");
            Self::default()
        };

        // let mut plot = super::plot::DataPlot::default();
        // plot.window_time = 10.0;

        // let mut status = FocStatus::default();

        out.t0 = None;
        out.serial_log_rx = Some(serial_log_rx);
        out.serial_cmd_tx = Some(serial_cmd_tx);
        out.ui_cmd_rx = Some(ui_cmd_rx);
        out.last_update = Some(Instant::now());

        out
    }
}

impl eframe::App for App {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(1_000 / 120));
        // ctx.request_repaint_after(std::time::Duration::from_millis(1_000 / 60));

        // if cfg!(debug_assertions) && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        //     ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        // }
        // if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        //     ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        // }

        // debug!("Fetching serial data");
        self.get_from_channels();
        // debug!("Done");

        egui::SidePanel::left("Left").show(ctx, |ui| {
            self.plot_settings(ui);
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
