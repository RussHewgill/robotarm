pub mod app;
pub mod controls;
pub mod plot;

#[derive(Debug, Clone, PartialEq)]
pub enum UiCommand {
    ClearPlot,
}
