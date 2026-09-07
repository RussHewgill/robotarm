use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serde::{Deserialize, Serialize};
use std::{collections::VecDeque, f64::consts::PI};

use eframe::egui::{self, Response};

// use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use egui_plotter::EguiBackend;
use plotters::{
    chart::{ChartBuilder, ChartContext, LabelAreaPosition, SeriesLabelPosition},
    coord::{
        CoordTranslate,
        cartesian::Cartesian2d,
        combinators::{
            BindKeyPointMethod, BindKeyPoints, BuildNestedCoord, GroupBy, IntoLinspace,
            IntoLogRange, IntoPartialAxis, Linspace, LogCoord, LogScalable, NestedRange,
            NestedValue, ToGroupByRange, make_partial_axis,
        },
        ranged1d::{DiscreteRanged, IntoSegmentedCoord, Ranged, SegmentValue},
        types::RangedCoordf64,
    },
    drawing::*,
    element::{
        Circle, Cross, Cubiod, DashedPathElement, DynElement, EmptyElement, IntoDynElement,
        MultiLineText, PathElement, Pie, Pixel, Polygon, Rectangle, Text, TriangleMarker,
    },
    prelude::DrawResult,
    series::{DashedLineSeries, DottedLineSeries, LineSeries},
    style::{
        AsRelative, Color, FontDesc, FontFamily, FontStyle, FontTransform, HSLColor, IntoFont,
        IntoTextStyle, Palette, Palette99, Palette100, Palette9999, PaletteColor, RGBAColor,
        RGBColor, ShapeStyle, TextStyle, colors::colormaps::*,
    },
};

use self::colors::*;
use crate::ui::{self, app::App};

// const COLORS: [RGBColor; 10] = [
//     RGBColor(0, 0, 255),
//     RGBColor(255, 0, 0),
//     RGBColor(0, 255, 0),
//     RGBColor(255, 255, 0),
//     RGBColor(0, 255, 255),
//     RGBColor(255, 0, 255),
//     RGBColor(128, 128, 128),
//     RGBColor(128, 0, 0),
//     RGBColor(0, 128, 0),
//     RGBColor(0, 0, 128),
// ];

pub mod colors {
    use plotters::style::RGBColor;

    pub use plotters::prelude::WHITE;

    pub const MAROON: RGBColor = RGBColor(0x80, 0, 0);
    pub const BROWN: RGBColor = RGBColor(0x9a, 0x63, 0x24);
    pub const TEAL: RGBColor = RGBColor(0x46, 0x99, 0x90);
    pub const NAVY: RGBColor = RGBColor(0, 0, 0x75);
    pub const BLACK: RGBColor = RGBColor(0, 0, 0);
    pub const RED: RGBColor = RGBColor(0xe6, 0x19, 0x4b);
    pub const ORANGE: RGBColor = RGBColor(0xf5, 0x82, 0x31);
    pub const YELLOW: RGBColor = RGBColor(0xff, 0xe1, 0x19);
    pub const GREEN: RGBColor = RGBColor(0x3c, 0xb4, 0x4b);
    pub const CYAN: RGBColor = RGBColor(0x42, 0xd4, 0xf4);
    pub const BLUE: RGBColor = RGBColor(0x43, 0x63, 0xd8);
    pub const PURPLE: RGBColor = RGBColor(0x91, 0x1e, 0xb4);
    pub const MAGENTA: RGBColor = RGBColor(0xf0, 0x32, 0xe6);
    pub const GREY: RGBColor = RGBColor(0xa9, 0xa9, 0xa9);
}

// const LABEL_V1: &str = "v1";
// const LABEL_V2: &str = "v2";
// const LABEL_X1: &str = "x1";
// const LABEL_X2: &str = "x2";
// const LABEL_X3: &str = "x3";

pub const LABEL_V1: &str = "Ref Angle";
pub const LABEL_V2: &str = "Ref Vel";
pub const LABEL_X1: &str = "State Angle";
pub const LABEL_X2: &str = "State Vel";
pub const LABEL_X3: &str = "Disturbance";

const COLOR_V1: RGBColor = CYAN;
const COLOR_V2: RGBColor = MAGENTA;
const COLOR_X1: RGBColor = GREEN;
const COLOR_X2: RGBColor = BLUE;
const COLOR_X3: RGBColor = RED;
const COLOR_U: RGBColor = BLACK;

#[derive(Debug, Deserialize, Serialize)]
pub struct DataPlot {
    #[serde(default)]
    window_time: f64,
    #[serde(default)]
    window_scale: f64,
    #[serde(default)]
    window_offset: f64,
    #[serde(skip)]
    prev_time: f64,

    stroke_width: u32,

    // pub vel_min_max: (f64, f64),
    pub draw_angle: bool,
    #[serde(skip)]
    angle: VecDeque<(f64, f64)>,

    angle_scale: f64,

    pub draw_vel: bool,
    #[serde(skip)]
    vel: VecDeque<(f64, f64)>,
    vel_bounds: (f64, f64),

    pub draw_pos: bool,
    #[serde(skip)]
    pos: VecDeque<(f64, f64)>,

    pub draw_target_pos: bool,
    #[serde(skip)]
    target_pos: VecDeque<(f64, f64)>,

    pub draw_target_vel: bool,
    #[serde(skip)]
    target_vel: VecDeque<(f64, f64)>,

    pub draw_voltage: bool,
    #[serde(skip)]
    voltage: VecDeque<(f64, f64)>,

    pub draw_current: bool,
    #[serde(skip)]
    current_d: VecDeque<(f64, f64)>,
    #[serde(skip)]
    current_q: VecDeque<(f64, f64)>,
    #[serde(skip)]
    current_d_lpf: Option<f64>,
    #[serde(skip)]
    current_q_lpf: Option<f64>,
    current_lpf_alpha: f64,

    // pub draw_pid_output_vel: bool,
    // #[serde(skip)]
    // pid_output_vel: VecDeque<(f64, f64)>,

    // pub draw_pid_output_pos: bool,
    // #[serde(skip)]
    // pid_output_pos: VecDeque<(f64, f64)>,

    // pub draw_pid_vel_internals_error: bool,
    // pub draw_pid_vel_internals_p: bool,
    // pub draw_pid_vel_internals_i: bool,
    // pub draw_pid_vel_internals_d: bool,
    // #[serde(skip)]
    // pid_vel_internals_error: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_vel_internals_p: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_vel_internals_i: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_vel_internals_d: VecDeque<(f64, f64)>,

    // pid_vel_lock_scales: bool,
    // pid_vel_internals_scale_output: f64,
    // pid_vel_internals_scale_error: f64,
    // pid_vel_internals_scale_p: f64,
    // pid_vel_internals_scale_i: f64,
    // pid_vel_internals_scale_d: f64,

    // pub draw_pid_pos_internals_error: bool,
    // pub draw_pid_pos_internals_p: bool,
    // pub draw_pid_pos_internals_i: bool,
    // pub draw_pid_pos_internals_d: bool,
    // #[serde(skip)]
    // pid_pos_internals_error: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_pos_internals_p: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_pos_internals_i: VecDeque<(f64, f64)>,
    // #[serde(skip)]
    // pid_pos_internals_d: VecDeque<(f64, f64)>,

    // pid_pos_lock_scales: bool,
    // pid_pos_internals_scale_output: f64,
    // pid_pos_internals_scale_error: f64,
    // pid_pos_internals_scale_p: f64,
    // pid_pos_internals_scale_i: f64,
    // pid_pos_internals_scale_d: f64,
    pub draw_adrc_v1: bool,
    pub draw_adrc_v2: bool,
    pub draw_adrc_x1: bool,
    pub draw_adrc_x2: bool,
    pub draw_adrc_x3: bool,
    pub draw_adrc_u: bool,

    adrc_v1_scale: f64,
    adrc_v2_scale: f64,
    adrc_x1_scale: f64,
    adrc_x2_scale: f64,
    adrc_x3_scale: f64,
    adrc_u_scale: f64,

    #[serde(skip)]
    adrc_vs: VecDeque<(f64, [f64; 2])>,
    #[serde(skip)]
    adrc_state: VecDeque<(f64, [f64; 3])>,
    #[serde(skip)]
    adrc_u: VecDeque<(f64, f64)>,

    // scale_angle: f64,
    scale_vel: f64,
}

impl Default for DataPlot {
    fn default() -> Self {
        Self {
            window_time: 20.,
            window_scale: 16.,
            window_offset: 0.,
            prev_time: 0.,

            stroke_width: 2,

            // vel_min_max: (-15., 15.),
            draw_angle: true,
            angle: VecDeque::new(),
            angle_scale: 1.0,
            draw_pos: true,
            pos: VecDeque::new(),
            draw_vel: true,
            vel: VecDeque::new(),
            vel_bounds: (-6.28, 6.28),
            draw_target_pos: true,
            target_pos: VecDeque::new(),
            draw_target_vel: true,
            target_vel: VecDeque::new(),
            draw_voltage: false,
            voltage: VecDeque::new(),
            draw_current: false,
            current_d: VecDeque::new(),
            current_q: VecDeque::new(),
            current_d_lpf: None,
            current_q_lpf: None,
            current_lpf_alpha: 0.1,

            // draw_pid_output_vel: true,
            // pid_output_vel: VecDeque::new(),
            // draw_pid_output_pos: false,
            // pid_output_pos: VecDeque::new(),
            // draw_pid_vel_internals_error: true,
            // draw_pid_vel_internals_p: true,
            // draw_pid_vel_internals_i: true,
            // draw_pid_vel_internals_d: true,
            // pid_vel_internals_error: VecDeque::new(),
            // pid_vel_internals_p: VecDeque::new(),
            // pid_vel_internals_i: VecDeque::new(),
            // pid_vel_internals_d: VecDeque::new(),

            // pid_vel_lock_scales: false,
            // pid_vel_internals_scale_output: 0.1,
            // pid_vel_internals_scale_error: 0.1,
            // pid_vel_internals_scale_p: 0.1,
            // pid_vel_internals_scale_i: 0.1,
            // pid_vel_internals_scale_d: 0.1,

            // draw_pid_pos_internals_error: false,
            // draw_pid_pos_internals_p: false,
            // draw_pid_pos_internals_i: false,
            // draw_pid_pos_internals_d: false,
            // pid_pos_internals_error: VecDeque::new(),
            // pid_pos_internals_p: VecDeque::new(),
            // pid_pos_internals_i: VecDeque::new(),
            // pid_pos_internals_d: VecDeque::new(),

            // pid_pos_lock_scales: false,
            // pid_pos_internals_scale_output: 0.1,
            // pid_pos_internals_scale_error: 0.1,
            // pid_pos_internals_scale_p: 0.1,
            // pid_pos_internals_scale_i: 0.1,
            // pid_pos_internals_scale_d: 0.1,
            draw_adrc_v1: true,
            draw_adrc_v2: true,
            draw_adrc_x1: true,
            draw_adrc_x2: true,
            draw_adrc_x3: true,
            draw_adrc_u: true,

            adrc_v1_scale: 0.5,
            adrc_v2_scale: 0.5,
            adrc_x1_scale: 0.5,
            adrc_x2_scale: 0.5,
            adrc_x3_scale: 0.0001,
            adrc_u_scale: 3.0,

            adrc_vs: VecDeque::new(),
            adrc_state: VecDeque::new(),
            adrc_u: VecDeque::new(),

            // scale_angle: std::f64::consts::PI * 2.,
            // scale_vel: 0.05,
            scale_vel: 1.0,
        }
    }
}

impl App {
    pub fn plot_settings(&mut self, ui: &mut egui::Ui) {
        let log_decimals = 4;
        let log_min = 0.001;
        let log_max = 10.0;

        egui::Grid::new(format!("Plot Controls Grid")).show(ui, |ui| {
            if ui.button("Clear Plot").clicked() {
                self.plots[self.current_plot].reset();
                self.t0 = None;
            }
            ui.end_row();
            ui.end_row();

            if ui.button("Save to CSV").clicked() {
                self.plots[self.current_plot].save_plot();
            }

            ui.end_row();
            ui.end_row();
            ui.label("Window time (s)");
            ui.add(egui::Slider::new(
                &mut self.plots[self.current_plot].window_time,
                1.0..=60.0,
            ));
            ui.end_row();

            ui.end_row();
            ui.label("Window scale");
            ui.add(
                egui::Slider::new(&mut self.plots[self.current_plot].window_scale, 2.0..=60.0)
                    .step_by(2.0),
            );
            ui.end_row();

            ui.end_row();
            ui.label("Window offset");
            ui.add(
                egui::Slider::new(
                    &mut self.plots[self.current_plot].window_offset,
                    -60.0..=60.0,
                )
                .step_by(1.0),
            );
            ui.end_row();

            ui.end_row();

            for (i, plot) in self.plots.iter().enumerate() {
                if ui
                    .selectable_label(self.current_plot == i, format!("Motor {}", i))
                    .clicked()
                {
                    self.current_plot = i;
                }
                ui.end_row();
            }

            ui.add_space(10.);
            ui.end_row();

            // plot settings
            {
                ui.label("Plot settings:");
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_angle, "Angle");

                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].angle_scale,
                        // log_min..=log_max,
                        0.01..=20.0,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );

                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_pos, "Position");
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_vel, "Velocity");
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_target_pos,
                    "Target position",
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_target_vel,
                    "Target velocity",
                );
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_voltage, "Voltage");
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_current, "Current");
                ui.end_row();
            }

            ui.end_row();

            // ADRC
            {
                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_v1, LABEL_V1);
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_v1_scale,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_v2, LABEL_V2);
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_v2_scale,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();

                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_x1, LABEL_X1);
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_x1_scale,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_x2, LABEL_X2);
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_x2_scale,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_x3, LABEL_X3);
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_x3_scale,
                        log_min / 100.0..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(&mut self.plots[self.current_plot].draw_adrc_u, "ADRC u");
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].adrc_u_scale,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
            }

            // vel PID
            #[cfg(feature = "nope")]
            {
                // ui.checkbox(
                //     &mut self.plots[self.current_plot].pid_pos_lock_scales,
                //     "Lock scales",
                // );
                // ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_output_vel,
                    "PID output vel",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_vel_internals_scale_output,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_vel_internals_error,
                    "PID vel error",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_vel_internals_scale_error,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_vel_internals_p,
                    "PID vel P",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_vel_internals_scale_p,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_vel_internals_i,
                    "PID vel I",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_vel_internals_scale_i,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_vel_internals_d,
                    "PID vel D",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_vel_internals_scale_d,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                // if self.plots[self.current_plot].pid_pos_lock_scales {
                //     self.plots[self.current_plot].pid_pos_internals_scale_error =
                //         self.plots[self.current_plot].pid_vel_internals_scale_output;
                // }
            }

            // pos PID
            #[cfg(feature = "nope")]
            {
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_output_pos,
                    "PID output pos",
                );
                ui.end_row();

                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_pos_internals_error,
                    "PID pos error",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_pos_internals_scale_error,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_pos_internals_p,
                    "PID pos P",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_pos_internals_scale_p,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_pos_internals_i,
                    "PID pos I",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_pos_internals_scale_i,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
                ui.checkbox(
                    &mut self.plots[self.current_plot].draw_pid_pos_internals_d,
                    "PID pos D",
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.plots[self.current_plot].pid_pos_internals_scale_d,
                        log_min..=log_max,
                    )
                    .logarithmic(true)
                    .max_decimals(log_decimals),
                );
                ui.end_row();
            }

            // ui.add(egui::Slider::new(&mut self.plot.scale_angle, 0.1..=10.).text("Angle scale"));
            // ui.add(egui::Slider::new(&mut self.plot.scale_vel, 0.01..=1.).text("Velocity scale"));
        });
    }
}

impl DataPlot {
    pub fn save_plot(&self) {
        let file = format!(
            "motor_data_{}.csv",
            chrono::Local::now().format("%Y-%m-%d_%H-%M-%S")
        );

        let mut writer = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(file)
            .expect("Failed to open CSV file for writing");
        let mut wtr = csv::Writer::from_writer(writer);

        let data = [
            &self.angle,
            &self.vel,
            &self.pos,
            &self.target_pos,
            &self.target_vel,
            &self.voltage,
            &self.current_d,
            &self.current_q,
            // &self.pid_output_vel,
            // &self.pid_vel_internals_error,
            // &self.pid_vel_internals_p,
            // &self.pid_vel_internals_i,
            // &self.pid_pos_internals_d,
            // &self.pid_output_pos,
            // &self.pid_pos_internals_error,
            // &self.pid_pos_internals_p,
            // &self.pid_pos_internals_i,
            // &self.pid_pos_internals_d,
        ];

        let n = data.iter().map(|d| d.len()).max().unwrap_or(0);

        {
            wtr.write_field("Timestamp").unwrap();
            wtr.write_field("Angle").unwrap();
            wtr.write_field("Velocity").unwrap();
            wtr.write_field("Position").unwrap();
            wtr.write_field("Target Position").unwrap();
            wtr.write_field("Target Velocity").unwrap();
            wtr.write_field("Voltage").unwrap();
            wtr.write_field("Current D").unwrap();
            wtr.write_field("Current Q").unwrap();
            // wtr.write_field("PID Output Velocity").unwrap();
            // wtr.write_field("PID Velocity Internals Error").unwrap();
            // wtr.write_field("PID Velocity Internals P").unwrap();
            // wtr.write_field("PID Velocity Internals I").unwrap();
            // wtr.write_field("PID Velocity Internals D").unwrap();
            // wtr.write_field("PID Output Position").unwrap();
            // wtr.write_field("PID Position Internals Error").unwrap();
            // wtr.write_field("PID Position Internals P").unwrap();
            // wtr.write_field("PID Position Internals I").unwrap();
            // wtr.write_field("PID Position Internals D").unwrap();
            wtr.write_record(None::<&[u8]>).unwrap();
        }

        for i in 0..n {
            let timestamps = data
                .iter()
                .flat_map(|d| d.get(i).into_iter().flat_map(|(t, _)| Some(*t)));

            let timestamps: Vec<f64> = timestamps.collect();
            let avg = if !timestamps.is_empty() {
                timestamps.iter().sum::<f64>() / timestamps.len() as f64
            } else {
                0.0
            };

            if timestamps.iter().filter(|&&t| t != avg).count() > 0 {
                warn!(
                    "Timestamps are not aligned at index {}: {:?}",
                    i, timestamps
                );
            }
            wtr.write_field(avg.to_string()).unwrap();

            for d in &data {
                if let Some((_, value)) = d.get(i) {
                    wtr.write_field(value.to_string()).unwrap();
                } else {
                    wtr.write_field("").unwrap();
                }
            }
            wtr.write_record(None::<&[u8]>).unwrap();
        }
    }

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
        // self.prev_time = t;
        self.prev_time = t.max(self.prev_time);
    }

    pub fn add_point_pos(&mut self, t: f64, pos: f64) {
        self.pos.push_back((t, pos));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);
    }

    pub fn add_point_vel(&mut self, t: f64, vel: f64) {
        self.vel.push_back((t, vel));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);

        self.vel_bounds.0 = self.vel_bounds.0.min(vel);
        self.vel_bounds.1 = self.vel_bounds.1.max(vel);
    }

    pub fn add_point_target_vel(&mut self, t: f64, target: f64) {
        self.target_vel.push_back((t, target));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);

        self.vel_bounds.0 = self.vel_bounds.0.min(target);
        self.vel_bounds.1 = self.vel_bounds.1.max(target);
    }

    pub fn add_point_target_pos(&mut self, t: f64, target: f64) {
        self.target_pos.push_back((t, target));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);
    }

    pub fn add_point_voltage(&mut self, t: f64, voltage: f64) {
        self.voltage.push_back((t, voltage as f64));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);
    }

    pub fn add_point_current(&mut self, t: f64, current_d: f64, current_q: f64) {
        let alpha = self.current_lpf_alpha.clamp(0.0, 1.0);

        let filtered_d = match self.current_d_lpf {
            Some(prev) => prev + alpha * (current_d - prev),
            None => current_d,
        };
        let filtered_q = match self.current_q_lpf {
            Some(prev) => prev + alpha * (current_q - prev),
            None => current_q,
        };

        self.current_d_lpf = Some(filtered_d);
        self.current_q_lpf = Some(filtered_q);

        self.current_d.push_back((t, filtered_d));
        self.current_q.push_back((t, filtered_q));
        // self.prev_time = t;
        // self.prev_time = t.max(self.prev_time);
    }

    // pub fn add_point_pid_output_vel(&mut self, t: f64, output: f64) {
    //     self.pid_output_vel.push_back((t, output));
    //     self.prev_time = t;
    // }

    // pub fn add_point_pid_output_pos(&mut self, t: f64, output: f64) {
    //     self.pid_output_pos.push_back((t, output));
    //     self.prev_time = t;
    // }

    // pub fn add_points_pid_vel_internals(&mut self, t: f64, error: f64, p: f64, i: f64, d: f64) {
    //     self.pid_vel_internals_error.push_back((t, error));
    //     self.pid_vel_internals_p.push_back((t, p));
    //     self.pid_vel_internals_i.push_back((t, i));
    //     self.pid_vel_internals_d.push_back((t, d));
    //     self.prev_time = t;
    // }

    // pub fn add_points_pid_pos_internals(&mut self, t: f64, error: f64, p: f64, i: f64, d: f64) {
    //     self.pid_pos_internals_error.push_back((t, error));
    //     self.pid_pos_internals_p.push_back((t, p));
    //     self.pid_pos_internals_i.push_back((t, i));
    //     self.pid_pos_internals_d.push_back((t, d));
    //     self.prev_time = t;
    // }

    pub fn add_points_adrc(&mut self, t: f64, vs: [f32; 2], xs: [f32; 3], u: f32) {
        self.adrc_vs.push_back((t, [vs[0] as f64, vs[1] as f64]));
        self.adrc_state
            .push_back((t, [xs[0] as f64, xs[1] as f64, xs[2] as f64]));
        self.adrc_u.push_back((t, u as f64));
        self.prev_time = t.max(self.prev_time);
    }

    fn clear_old_points(&mut self, current_time: f64) {
        self.angle
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.vel
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.pos
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.target_pos
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.target_vel
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.voltage
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.current_d
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.current_q
            .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_output_vel
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_output_pos
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_vel_internals_error
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_vel_internals_p
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_vel_internals_i
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_vel_internals_d
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_pos_internals_error
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_pos_internals_p
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_pos_internals_i
        //     .retain(|(t, _)| *t >= current_time - self.window_time);
        // self.pid_pos_internals_d
        //     .retain(|(t, _)| *t >= current_time - self.window_time);

        self.adrc_vs
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.adrc_state
            .retain(|(t, _)| *t >= current_time - self.window_time);
        self.adrc_u
            .retain(|(t, _)| *t >= current_time - self.window_time);
    }

    #[cfg(feature = "nope")]
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

        while let Some((t2, _)) = self.pos.front() {
            if *t2 < current_time - self.window_time {
                self.pos.pop_front();
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

        while let Some((t2, _)) = self.current_d.front() {
            if *t2 < current_time - self.window_time {
                self.current_d.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.current_q.front() {
            if *t2 < current_time - self.window_time {
                self.current_q.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.pid_output_vel.front() {
            if *t2 < current_time - self.window_time {
                self.pid_output_vel.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.pid_output_pos.front() {
            if *t2 < current_time - self.window_time {
                self.pid_output_pos.pop_front();
            } else {
                break;
            }
        }

        self.vel_bounds = (-6.28, 6.28);
        for (_, v) in self.vel.iter() {
            self.vel_bounds.0 = self.vel_bounds.0.min(*v);
            self.vel_bounds.1 = self.vel_bounds.1.max(*v);
        }
        for (_, v) in self.target_vel.iter() {
            self.vel_bounds.0 = self.vel_bounds.0.min(*v);
            self.vel_bounds.1 = self.vel_bounds.1.max(*v);
        }

        while let Some((t2, _)) = self.pid_vel_internals_error.front() {
            if *t2 < current_time - self.window_time {
                self.pid_vel_internals_error.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_vel_internals_p.front() {
            if *t2 < current_time - self.window_time {
                self.pid_vel_internals_p.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_vel_internals_i.front() {
            if *t2 < current_time - self.window_time {
                self.pid_vel_internals_i.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_vel_internals_d.front() {
            if *t2 < current_time - self.window_time {
                self.pid_vel_internals_d.pop_front();
            } else {
                break;
            }
        }

        while let Some((t2, _)) = self.pid_pos_internals_error.front() {
            if *t2 < current_time - self.window_time {
                self.pid_pos_internals_error.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_pos_internals_p.front() {
            if *t2 < current_time - self.window_time {
                self.pid_pos_internals_p.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_pos_internals_i.front() {
            if *t2 < current_time - self.window_time {
                self.pid_pos_internals_i.pop_front();
            } else {
                break;
            }
        }
        while let Some((t2, _)) = self.pid_pos_internals_d.front() {
            if *t2 < current_time - self.window_time {
                self.pid_pos_internals_d.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn reset(&mut self) {
        self.angle.clear();
        self.vel.clear();
        self.pos.clear();
        self.target_pos.clear();
        self.target_vel.clear();
        self.voltage.clear();
        self.current_d.clear();
        self.current_q.clear();
        self.current_d_lpf = None;
        self.current_q_lpf = None;

        // self.pid_output_vel.clear();
        // self.pid_output_pos.clear();

        // self.pid_vel_internals_error.clear();
        // self.pid_vel_internals_p.clear();
        // self.pid_vel_internals_i.clear();
        // self.pid_vel_internals_d.clear();

        // self.pid_pos_internals_error.clear();
        // self.pid_pos_internals_p.clear();
        // self.pid_pos_internals_i.clear();
        // self.pid_pos_internals_d.clear();

        self.prev_time = 0.;

        self.vel_bounds = (-6.28, 6.28);

        self.adrc_state.clear();
        self.adrc_vs.clear();
        self.adrc_u.clear();
    }
}

impl DataPlot {
    pub fn show_plot(&mut self, ui: &mut egui::Ui) {
        // self.clear_old_points(self.prev_time);

        let root = EguiBackend::new(ui).into_drawing_area();

        // let (upper, lower) = root.split_vertically(root.dim_in_pixel().1 as f64 * 0.7);
        let (upper, lower) = root.split_vertically(root.dim_in_pixel().1 as f64 * 0.5);

        {
            let root = upper;

            root.fill(&WHITE.mix(0.5)).unwrap();
            let mut chart = plotters::chart::ChartBuilder::on(&root)
                .margin(5)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .right_y_label_area_size(30)
                .build_cartesian_2d(
                    self.prev_time - self.window_time..self.prev_time,
                    -1f64..1f64,
                )
                .unwrap()
                .set_secondary_coord(
                    self.prev_time - self.window_time..self.prev_time,
                    // self.vel_bounds.0..self.vel_bounds.1,
                    // self.vel_min_max.0..self.vel_min_max.1,
                    // self.window_scale * self.window_offset - self.window_scale
                    //     ..self.window_scale * self.window_offset + self.window_scale,
                    -self.window_scale - self.window_offset..self.window_scale - self.window_offset,
                );

            chart.configure_mesh().draw().unwrap();

            chart
                .configure_secondary_axes()
                .y_desc("Velocity (rad/s)")
                .draw()
                .unwrap();

            if self.draw_angle {
                // data in 0-2pi, we want -1 to 1
                chart
                    .draw_series(LineSeries::new(
                        self.angle
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, angle)| {
                                // (*t, self.angle_scale * (*angle - PI) / PI)
                                // (*t, *angle * self.angle_scale)
                                // let a = self.angle_scale * angle / PI;
                                let a = self.angle_scale * (angle - PI) / PI;
                                // (*t, self.angle_scale * (*pos - PI) / PI)
                                (*t, a)
                            }),
                        GREEN.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN))
                    .label("Angle");
            }

            if self.draw_pos {
                chart
                    .draw_series(LineSeries::new(
                        self.pos
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, angle)| {
                                // (*t, (*pos - std::f64::consts::PI) / std::f64::consts::PI)
                                // let a = self.angle_scale * angle / PI;
                                let a = self.angle_scale * (angle - PI) / PI;
                                (*t, a)
                            }),
                        TEAL.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &TEAL))
                    .label("Position");
            }

            if self.draw_vel {
                chart
                    .draw_secondary_series(LineSeries::new(
                        self.vel
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, vel)| (*t, *vel * self.scale_vel)),
                        BLUE.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE))
                    .label("Velocity");
            }

            if self.draw_target_pos {
                chart
                    .draw_series(LineSeries::new(
                        self.target_pos
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, angle)| {
                                // (*t, (std::f64::consts::PI - *angle) / std::f64::consts::PI)
                                // let a = angle % (2.0 * PI);
                                // let a = if a >= 0.0 { a } else { a + 2.0 * PI };
                                // let a = self.angle_scale * angle / PI;
                                let a = self.angle_scale * (angle - PI) / PI;
                                (*t, a)
                                // (*t, *angle * self.angle_scale)
                            }),
                        RED.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED))
                    .label("Target Pos");
            }

            if self.draw_target_vel {
                chart
                    .draw_secondary_series(LineSeries::new(
                        self.target_vel
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, target)| (*t, *target * self.scale_vel)),
                        MAGENTA.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &MAGENTA))
                    .label("Target Vel");
            }

            if self.draw_voltage {
                // chart
                //     .draw_secondary_series(LineSeries::new(
                //         self.voltage
                //             .iter()
                //             .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                //         .map(|(t, voltage)| (*t, *voltage)),
                //         colors::ORANGE.stroke_width(self.stroke_width),
                //     ))
                //     .unwrap()
                //     .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &colors::ORANGE))
                //     .label("Voltage");

                chart
                    .draw_secondary_series(LineSeries::new(
                        self.voltage
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, angle)| {
                                // (*t, (*pos - std::f64::consts::PI) / std::f64::consts::PI)
                                // (*t, self.angle_scale * (*angle - PI) / PI)
                                // let a = self.angle_scale * (angle - PI) / PI;
                                let a = *angle * self.scale_vel;
                                (*t, a)
                            }),
                        // .map(|(t, voltage)| (*t, *voltage)),
                        colors::ORANGE.stroke_width(self.stroke_width),
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &colors::ORANGE))
                    .label("Raw Vel");
            }

            if self.draw_current {
                // chart
                //     .draw_series(LineSeries::new(
                //         self.current.iter().filter(|(t, _)| *t >= self.prev_time - self.window_time).map(|(t, current)| (*t, *current / 2.)),
                //         &CYAN,
                //     ))
                //     .unwrap()
                //     .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN))
                //     .label("Current");

                let current_scale = 10.0;

                chart
                    .draw_series(LineSeries::new(
                        self.current_d
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, current)| (*t, *current * current_scale)),
                        &CYAN,
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN))
                    .label("Current Id");

                chart
                    .draw_series(LineSeries::new(
                        self.current_q
                            .iter()
                            .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                            .map(|(t, current)| (*t, *current * current_scale)),
                        &YELLOW,
                    ))
                    .unwrap()
                    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &YELLOW))
                    .label("Current Iq");
            }

            chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft).margin(20)
            .legend_area_size(25)
            .border_style(BLACK)
            .background_style(&WHITE.mix(0.7))
            .draw()
            .unwrap()
            // .borrow_secondary()
            // .legend_area_size(5)
            // .legend_area_size(5)
            ;

            // chart
            //     .configure_series_labels()
            //     .border_style(&BLACK)
            //     .background_style(&WHITE.mix(0.8))
            //     .draw()
            //     .unwrap();

            root.present().unwrap();
        }

        {
            let root = lower;

            root.fill(&WHITE.mix(0.5)).unwrap();
            let mut chart = plotters::chart::ChartBuilder::on(&root)
                .margin(5)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .right_y_label_area_size(30)
                .build_cartesian_2d(
                    self.prev_time - self.window_time..self.prev_time,
                    -1f64..1f64,
                )
                .unwrap();

            chart.configure_mesh().draw().unwrap();

            // ADRC
            {
                if self.draw_adrc_v1 {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_vs
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, vs)| (*t, vs[0] * self.adrc_v1_scale)),
                            COLOR_V1.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_V1))
                        .label(LABEL_V1);
                }

                if self.draw_adrc_v2 {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_vs
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, vs)| (*t, vs[1] * self.adrc_v2_scale)),
                            COLOR_V2.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_V2))
                        .label(LABEL_V2);
                }

                if self.draw_adrc_x1 {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_state
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, xs)| (*t, xs[0] * self.adrc_x1_scale)),
                            COLOR_X1.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_X1))
                        .label(LABEL_X1);
                }
                if self.draw_adrc_x2 {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_state
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, xs)| (*t, xs[1] * self.adrc_x2_scale)),
                            COLOR_X2.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_X2))
                        .label(LABEL_X2);
                }
                if self.draw_adrc_x3 {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_state
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, xs)| (*t, xs[2] * self.adrc_x3_scale)),
                            COLOR_X3.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_X3))
                        .label(LABEL_X3);
                }
                if self.draw_adrc_u {
                    chart
                        .draw_series(LineSeries::new(
                            self.adrc_u
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, u)| (*t, *u * self.adrc_u_scale)),
                            COLOR_U.stroke_width(self.stroke_width),
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLOR_U))
                        .label("ADRC u");
                }
            }

            // vel
            #[cfg(feature = "nope")]
            {
                if self.draw_pid_output_vel {
                    chart
                        .draw_series(LineSeries::new(
                            self.pid_output_vel
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                // .map(|(t, output)| (*t, *output * self.scale_vel * 10.0)),
                                .map(|(t, output)| {
                                    (*t, *output * self.pid_vel_internals_scale_output)
                                }),
                            &CYAN,
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN))
                        .label("PID Output Vel (x10)");
                }

                if self.draw_pid_vel_internals_error {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_vel_internals_error
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, error)| {
                                    (*t, *error * self.pid_vel_internals_scale_error)
                                }),
                            5,
                            5,
                            ShapeStyle {
                                color: RED.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &RED)
                        })
                        .label("PID Vel Error");
                }

                if self.draw_pid_vel_internals_p {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_vel_internals_p
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, p)| (*t, *p * self.pid_vel_internals_scale_p)),
                            5,
                            5,
                            ShapeStyle {
                                color: GREEN.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &GREEN)
                        })
                        .label("PID Vel P");
                }

                if self.draw_pid_vel_internals_i {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_vel_internals_i
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, i)| (*t, *i * self.pid_vel_internals_scale_i)),
                            5,
                            5,
                            ShapeStyle {
                                color: MAGENTA.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &MAGENTA)
                        })
                        .label("PID Vel I");
                }

                if self.draw_pid_vel_internals_d {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_vel_internals_d
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, d)| (*t, *d * self.pid_vel_internals_scale_d)),
                            5,
                            5,
                            ShapeStyle {
                                color: NAVY.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        // .legend(|(x, y)| DashedPathElement::new(vec![(x, y), (x + 20, y)], &BLUE))
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &NAVY)
                        })
                        .label("PID Vel D");
                }
            }

            // pos
            #[cfg(feature = "nope")]
            {
                if self.draw_pid_output_pos {
                    chart
                        .draw_series(LineSeries::new(
                            self.pid_output_pos
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                // .map(|(t, output)| (*t, *output * self.scale_pos * 10.0)),
                                .map(|(t, output)| (*t, *output * self.scale_vel * 1.0)),
                            &CYAN,
                        ))
                        .unwrap()
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN))
                        .label("PID Output pos (x10)");
                }

                if self.draw_pid_pos_internals_error {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_pos_internals_error
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, error)| {
                                    (*t, *error * self.pid_pos_internals_scale_error)
                                }),
                            5,
                            5,
                            ShapeStyle {
                                color: RED.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &RED)
                        })
                        .label("PID pos Error");
                }

                if self.draw_pid_pos_internals_p {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_pos_internals_p
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, p)| (*t, *p * self.pid_pos_internals_scale_p)),
                            5,
                            5,
                            ShapeStyle {
                                color: GREEN.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &GREEN)
                        })
                        .label("PID pos P");
                }

                if self.draw_pid_pos_internals_i {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_pos_internals_i
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, i)| (*t, *i * self.pid_pos_internals_scale_i)),
                            5,
                            5,
                            ShapeStyle {
                                color: MAGENTA.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &MAGENTA)
                        })
                        .label("PID pos I");
                }

                if self.draw_pid_pos_internals_d {
                    chart
                        .draw_series(DashedLineSeries::new(
                            self.pid_pos_internals_d
                                .iter()
                                .filter(|(t, _)| *t >= self.prev_time - self.window_time)
                                .map(|(t, d)| (*t, *d * self.pid_pos_internals_scale_d)),
                            5,
                            5,
                            ShapeStyle {
                                color: NAVY.to_rgba(),
                                filled: false,
                                stroke_width: self.stroke_width,
                            },
                        ))
                        .unwrap()
                        // .legend(|(x, y)| DashedPathElement::new(vec![(x, y), (x + 20, y)], &BLUE))
                        .legend(|(x, y)| {
                            DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, &NAVY)
                        })
                        .label("PID pos D");
                }
            }

            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperLeft)
                .margin(20)
                .legend_area_size(25)
                .border_style(BLACK)
                .background_style(&WHITE.mix(0.7))
                .draw()
                .unwrap();

            root.present().unwrap();
        }
    }
}
