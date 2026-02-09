use as5600::asynch::As5600;
use embassy_rp::i2c::Async;

use crate::simplefoc::{
    lowpass::LowPassFilter,
    pid::PIDController,
    types::{DQCurrents, DQVoltages, PhaseVoltages},
};

#[derive(defmt::Format)]
pub struct BLDCMotor {
    pub pole_pairs: u8,
    pub phase_resistance: f32,
    pub motor_kv: f32,
    pub phase_inductance: Option<f32>,

    // pub(super) target: f32,
    // pub(super) feed_forward_velocity: f32,
    // pub(super) shaft_angle: f32,
    // pub(super) electrical_angle: f32,
    // pub(super) shaft_velocity: f32,
    /// target current (q current)
    pub(super) target_current: f32,
    pub(super) target_shaft_velocity: f32,
    pub(super) target_shaft_angle: f32,

    pub(super) voltage: DQVoltages,
    pub(super) current: DQCurrents,

    pub(super) estimated_back_emf: f32,
    // /// Phase voltages U alpha and U beta used for inverse Park and Clarke transform
    // pub(super) phase_voltages_alpha_beta: (f32, f32),
    // pub(super) phase_v: PhaseVoltages,
    pub(super) voltage_sensor_align: f32,
    pub(super) velocity_index_search: f32,

    pub(super) limit_voltage: f32,
    pub(super) limit_current: f32,
    pub(super) limit_velocity: f32,

    pub(super) openloop_ts: u64,
}

impl BLDCMotor {
    pub fn new(
        pole_pairs: u8,
        phase_resistance: f32,
        motor_kv: f32,
        phase_inductance: Option<f32>,
    ) -> Self {
        Self {
            pole_pairs,
            phase_resistance,
            motor_kv,
            phase_inductance,

            // target: 0.0,
            // feed_forward_velocity: 0.0,
            // shaft_angle: 0.0,
            // electrical_angle: 0.0,
            // shaft_velocity: 0.0,
            target_current: 0.0,
            target_shaft_velocity: 0.0,
            target_shaft_angle: 0.0,

            voltage: DQVoltages { d: 0.0, q: 0.0 },
            current: DQCurrents { d: 0.0, q: 0.0 },

            estimated_back_emf: 0.0,
            // phase_voltages_alpha_beta: (0.0, 0.0),
            // phase_v: PhaseVoltages::default(),
            voltage_sensor_align: 2.5,
            velocity_index_search: 1.5,

            limit_voltage: 12.0,
            limit_current: 1.0,
            limit_velocity: 20.0,

            openloop_ts: 0,
        }
    }
}
