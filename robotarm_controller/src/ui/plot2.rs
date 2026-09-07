use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use serde::{Deserialize, Serialize};
use std::{collections::VecDeque, f64::consts::PI};

use eframe::egui::{self, Response};

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

use crate::ui::plot::colors::*;
use crate::ui::{self, app::App};

#[derive(Debug, Deserialize, Serialize)]
pub struct Plottable {
    pub name: String,
    draw: bool,
    #[serde(skip)]
    values: VecDeque<(f64, f64)>,
}

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
}
