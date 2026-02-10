use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serde::{Deserialize, Serialize};

#[derive(Default, Deserialize, Serialize)]
#[serde(default)]
pub struct App {
    //
}

impl App {
    pub fn new() -> Self {
        Self {}
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Hello World!");
        });
    }
}
