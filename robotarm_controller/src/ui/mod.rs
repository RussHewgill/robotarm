pub mod app;
pub mod comms;
pub mod controls;
pub mod plot;
pub mod plot2;
pub mod widgets;

#[derive(Debug, Clone, PartialEq)]
pub enum UiCommand {
    ClearPlot,
}
