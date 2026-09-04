pub mod app;
pub mod comms;
pub mod controls;
pub mod plot;
pub mod widgets;

#[derive(Debug, Clone, PartialEq)]
pub enum UiCommand {
    ClearPlot,
}
