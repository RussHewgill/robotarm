pub const _2_SQRT3: f32 = 1.15470053838;
pub const _SQRT3: f32 = 1.73205080757;
pub const _1_SQRT3: f32 = 0.57735026919;
pub const _SQRT3_2: f32 = 0.86602540378;
pub const _SQRT2: f32 = 1.41421356237;
pub const _120_D2R: f32 = 2.09439510239;
pub const _PI: f32 = 3.14159265359;
pub const _PI_2: f32 = 1.57079632679;
pub const _PI_3: f32 = 1.0471975512;
pub const _2PI: f32 = 6.28318530718;
pub const _3PI_2: f32 = 4.71238898038;
pub const _PI_6: f32 = 0.52359877559;
pub const _RPM_TO_RADS: f32 = 0.10471975512;

pub const NOT_SET: f32 = -12345.0;

#[derive(defmt::Format)]
pub struct DQVoltages {
    pub d: f32,
    pub q: f32,
}

#[derive(defmt::Format)]
pub struct DQCurrents {
    pub d: f32,
    pub q: f32,
}

#[derive(defmt::Format, Default)]
pub struct PhaseVoltages {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[derive(defmt::Format, Clone, Copy, PartialEq, PartialOrd)]
pub enum SensorDirection {
    Unknown,
    Normal,
    Inverted,
}

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub enum TorqueControlType {
    Voltage,
    // DCCurrent,
    // FOCCurrent,
}
