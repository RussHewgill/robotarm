use defmt::{debug, error, info, trace, warn};
use embassy_time::{Instant, Timer};

use serde::{Deserialize, Serialize};

// use super::bldc::BLDCMotor;

#[derive(defmt::Format, Clone, Copy, Serialize, Deserialize)]
pub struct BLDCThermalLimits {
    pub current_continuous: f32,
    pub current_peak: f32,
    pub peak_time: f32,
    pub temperature_limit: f32,
    pub thermal_time_constant: f32,
    current_continuous_sq: f32,
    // derating_start: f32,
    cutoff_energy: f32,
    max_energy: f32,
    // accumlated_energy: (f32, f32, f32),
    accumlated_energy: f32,
}

impl BLDCThermalLimits {
    pub fn new(
        current_continuous: f32,
        current_peak: f32,
        peak_time: f32,
        temperature_limit: f32,
        thermal_time_constant: f32,
    ) -> Self {
        let max_energy =
            (current_peak * current_peak - current_continuous * current_continuous) * peak_time;

        let start_energy = max_energy / 2.0;

        Self {
            current_continuous,
            current_peak,
            peak_time,
            temperature_limit,
            thermal_time_constant,
            current_continuous_sq: current_continuous * current_continuous,
            max_energy,
            cutoff_energy: max_energy / 2.0,
            // accumlated_energy: (start_energy, start_energy, start_energy),
            accumlated_energy: start_energy,
        }
    }

    /// returns a fraction of allowed current
    pub fn update(&mut self, current: f32, dt_us: u64) -> Option<f32> {
        let dt = dt_us as f32 / 1_000_000.0;
        let i_sq = current * current;

        let delta_energy = (i_sq - self.current_continuous_sq) * dt;
        // let delta_energy = (i_sq - self.accumlated_energy / self.thermal_time_constant) * dt;

        self.accumlated_energy += delta_energy;
        // self.accumlated_energy.clamp(0.0, self.max_energy);
        self.accumlated_energy = self.accumlated_energy.max(0.0);

        if self.accumlated_energy > self.max_energy {
            warn!(
                "Thermal limit exceeded: energy = {}, max = {}",
                self.accumlated_energy, self.max_energy
            );
            return Some(0.0);
        } else if self.accumlated_energy > self.cutoff_energy {
            debug!(
                "Thermal energy over cutoff point: energy = {}, cutoff = {}",
                self.accumlated_energy, self.cutoff_energy
            );
            return Some(0.0);
        }

        None
    }
}
