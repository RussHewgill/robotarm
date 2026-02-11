use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use eframe::egui::{self, Response};
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};

pub struct DataPlot {
    window_time: f64,
    angle: Vec<PlotPoint>,
    vel: Vec<PlotPoint>,
}

impl Default for DataPlot {
    fn default() -> Self {
        Self {
            window_time: 10.,
            angle: Vec::new(),
            vel: Vec::new(),
        }
    }
}

impl DataPlot {
    pub fn add_point(&mut self, t: f64, angle: f32, vel: f32) {
        debug!("Adding point: t={}, angle={}, vel={}", t, angle, vel);
        self.angle.push(PlotPoint::new(t as f64, angle as f64));
        self.vel.push(PlotPoint::new(t as f64, vel as f64));
    }

    pub fn show_plot(&self, ui: &mut egui::Ui) -> Response {
        Plot::new("My Plot")
            .legend(Legend::default())
            .auto_bounds(false)
            .allow_zoom(false)
            .allow_scroll(false)
            .allow_boxed_zoom(false)
            .allow_drag(false)
            .include_x(0.)
            .include_x(self.window_time)
            .include_y(0.)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("angle", PlotPoints::Borrowed(&self.angle)).name("angle"));
                plot_ui.line(Line::new("vel", PlotPoints::Borrowed(&self.vel)).name("vel"));
            })
            .response
    }
}
