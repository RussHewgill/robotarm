use std::collections::VecDeque;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use eframe::egui::{self, Response};

// use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use egui_plotter::EguiBackend;
use plotters::{prelude::*, style::full_palette::ORANGE};

use crate::ui::{self, app::App};

pub struct DataPlot {
    window_time: f64,
    prev_time: f64,

    pub draw_angle: bool,
    angle: VecDeque<(f64, f64)>,

    pub draw_vel: bool,
    vel: VecDeque<(f64, f64)>,

    pub draw_target_pos: bool,
    target_pos: VecDeque<(f64, f64)>,

    pub draw_target_vel: bool,
    target_vel: VecDeque<(f64, f64)>,

    pub draw_voltage: bool,
    voltage: VecDeque<(f64, f64)>,

    pub draw_current: bool,
    current: VecDeque<(f64, f64)>,

    // scale_angle: f64,
    scale_vel: f64,
}

impl Default for DataPlot {
    fn default() -> Self {
        Self {
            window_time: 10.,
            prev_time: 0.,
            draw_angle: true,
            angle: VecDeque::new(),
            draw_vel: true,
            vel: VecDeque::new(),
            draw_target_pos: true,
            target_pos: VecDeque::new(),
            draw_target_vel: false,
            target_vel: VecDeque::new(),
            draw_voltage: false,
            voltage: VecDeque::new(),
            draw_current: false,
            current: VecDeque::new(),

            // scale_angle: std::f64::consts::PI * 2.,
            scale_vel: 0.05,
        }
    }
}

impl App {
    pub fn plot_settings(&mut self, ui: &mut egui::Ui) {
        ui.label("Plot settings:");
        ui.checkbox(&mut self.plot.draw_angle, "Angle");
        ui.checkbox(&mut self.plot.draw_vel, "Velocity");
        ui.checkbox(&mut self.plot.draw_target_pos, "Target position");
        ui.checkbox(&mut self.plot.draw_target_vel, "Target velocity");
        ui.checkbox(&mut self.plot.draw_voltage, "Voltage");
        ui.checkbox(&mut self.plot.draw_current, "Current");

        // ui.add(egui::Slider::new(&mut self.plot.scale_angle, 0.1..=10.).text("Angle scale"));
        // ui.add(egui::Slider::new(&mut self.plot.scale_vel, 0.01..=1.).text("Velocity scale"));
    }
}

impl DataPlot {
    pub fn get_angle(&self) -> &VecDeque<(f64, f64)> {
        &self.angle
    }
    pub fn get_vel(&self) -> &VecDeque<(f64, f64)> {
        &self.vel
    }
    pub fn get_target_pos(&self) -> &VecDeque<(f64, f64)> {
        &self.target_pos
    }
    pub fn get_target_vel(&self) -> &VecDeque<(f64, f64)> {
        &self.target_vel
    }

    pub fn add_point_angle(&mut self, t: f64, angle: f64) {
        self.angle.push_back((t, angle));
        self.prev_time = t;
    }

    pub fn add_point_vel(&mut self, t: f64, vel: f64) {
        self.vel.push_back((t, vel));
        self.prev_time = t;
    }

    pub fn add_point_target_vel(&mut self, t: f64, target: f64) {
        self.target_vel.push_back((t, target));
        self.prev_time = t;
    }

    pub fn add_point_target_pos(&mut self, t: f64, target: f64) {
        self.target_pos.push_back((t, target));
        self.prev_time = t;
    }

    pub fn add_point_voltage(&mut self, t: f64, voltage: f64) {
        self.voltage.push_back((t, voltage as f64));
        self.prev_time = t;
    }

    pub fn add_point_current(&mut self, t: f64, current: f64) {
        self.current.push_back((t, current as f64));
        self.prev_time = t;
    }

    fn clear_old_points(&mut self, current_time: f64) {
        while let Some((t2, _)) = self.angle.front() {
            if *t2 < current_time - self.window_time {
                self.angle.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.vel.front() {
            if *t2 < current_time - self.window_time {
                self.vel.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.target_pos.front() {
            if *t2 < current_time - self.window_time {
                self.target_pos.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.target_vel.front() {
            if *t2 < current_time - self.window_time {
                self.target_vel.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.voltage.front() {
            if *t2 < current_time - self.window_time {
                self.voltage.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.current.front() {
            if *t2 < current_time - self.window_time {
                self.current.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn reset(&mut self) {
        self.angle.clear();
        self.vel.clear();
        self.target_pos.clear();
        self.target_vel.clear();
        self.prev_time = 0.;
    }
}

impl DataPlot {
    pub fn show_plot(&mut self, ui: &mut egui::Ui) {
        self.clear_old_points(self.prev_time);

        let root = EguiBackend::new(ui).into_drawing_area();

        root.fill(&plotters::style::WHITE).unwrap();
        let mut chart = plotters::chart::ChartBuilder::on(&root)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(
                self.prev_time - self.window_time..self.prev_time,
                -1f64..1f64,
            )
            .unwrap();

        // chart.set_secondary_coord(0f64..self.window_time, -20f64..20f64);

        chart.configure_mesh().draw().unwrap();

        if self.draw_angle {
            // data in 0-2pi, we want -1 to 1
            chart
                .draw_series(LineSeries::new(
                    self.angle.iter().map(|(t, angle)| {
                        (*t, (*angle - std::f64::consts::PI) / std::f64::consts::PI)
                    }),
                    &GREEN,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN))
                .label("Position");
        }

        if self.draw_vel {
            chart
                .draw_series(LineSeries::new(
                    self.vel.iter().map(|(t, vel)| (*t, *vel * self.scale_vel)),
                    &BLUE,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE))
                .label("Velocity");
        }

        if self.draw_target_pos {
            chart
                .draw_series(LineSeries::new(
                    self.target_pos.iter().map(|(t, angle)| {
                        (*t, -(*angle - std::f64::consts::PI) / std::f64::consts::PI)
                    }),
                    &RED,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED))
                .label("Target Pos");
        }

        if self.draw_target_vel {
            chart
                .draw_series(LineSeries::new(
                    self.target_vel
                        .iter()
                        .map(|(t, target)| (*t, *target * self.scale_vel)),
                    &MAGENTA,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &MAGENTA))
                .label("Target Vel");
        }

        if self.draw_voltage {
            chart
                .draw_series(LineSeries::new(
                    self.voltage.iter().map(|(t, voltage)| (*t, *voltage / 12.)),
                    &ORANGE,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &ORANGE))
                .label("Voltage");
        }

        if self.draw_current {
            chart
                .draw_series(LineSeries::new(
                    self.current.iter().map(|(t, current)| (*t, *current / 2.)),
                    &CYAN,
                ))
                .unwrap()
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN))
                .label("Current");
        }

        chart
            .configure_series_labels()
            .border_style(&BLACK)
            .background_style(&WHITE.mix(0.8))
            .draw()
            .unwrap();

        root.present().unwrap();
    }
}
