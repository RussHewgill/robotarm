use std::collections::VecDeque;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use eframe::egui::{self, Response};

// use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use egui_plotter::EguiBackend;
use plotters::prelude::*;

pub struct DataPlot {
    window_time: f64,
    prev_time: f64,
    angle: VecDeque<(f64, f64)>,
    vel: VecDeque<(f64, f64)>,
    target_pos: VecDeque<(f64, f64)>,
    target_vel: VecDeque<(f64, f64)>,

    // scale_angle: f64,
    scale_vel: f64,
}

impl Default for DataPlot {
    fn default() -> Self {
        Self {
            window_time: 10.,
            prev_time: 0.,
            angle: VecDeque::new(),
            vel: VecDeque::new(),
            target_pos: VecDeque::new(),
            target_vel: VecDeque::new(),

            // scale_angle: std::f64::consts::PI * 2.,
            scale_vel: 0.05,
        }
    }
}

impl DataPlot {
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

        // data in 0-2pi, we want -1 to 1
        chart
            .draw_series(LineSeries::new(
                self.angle
                    .iter()
                    .map(|(t, angle)| (*t, (*angle - std::f64::consts::PI) / std::f64::consts::PI)),
                &GREEN,
            ))
            .unwrap()
            .label("Position");

        chart
            .draw_series(LineSeries::new(
                self.vel.iter().map(|(t, vel)| (*t, *vel * self.scale_vel)),
                &BLUE,
            ))
            .unwrap()
            .label("Velocity");

        chart
            .draw_series(LineSeries::new(
                self.target_pos.iter().map(|(t, target)| (*t, *target)),
                &RED,
            ))
            .unwrap()
            .label("Target Pos");

        // chart
        //     .configure_series_labels()
        //     .background_style(&WHITE.mix(0.8))
        //     .border_style(&BLACK)
        //     .draw()
        //     .unwrap();

        root.present().unwrap();
    }
}
