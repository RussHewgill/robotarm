pub mod app;
pub mod controls;
pub mod plot;
pub mod comms;

#[derive(Debug, Clone, PartialEq)]
pub enum UiCommand {
    ClearPlot,
}
