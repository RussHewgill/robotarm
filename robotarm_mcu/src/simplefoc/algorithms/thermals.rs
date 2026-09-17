use defmt::{debug, error, info, trace, warn};
use embassy_time::{Instant, Timer};

use serde::{Deserialize, Serialize};

// use super::bldc::BLDCMotor;

#[derive(defmt::Format, Clone, Copy, Serialize, Deserialize)]
pub struct BLDCThermalLimits {
    config: ThermalConfig,
    estimated_temp: f32,
}

#[derive(defmt::Format, Clone, Copy, Serialize, Deserialize)]
pub struct ThermalConfig {
    /// Thermal resistance from windings to ambient (Kelvin / Watt)
    pub r_th: f32,
    /// Thermal capacitance of the motor (Joules / Kelvin)
    pub c_th: f32,
    /// Ambient temperature (Celsius)
    pub t_ambient: f32,
    /// Temperature at which the motor starts reducing power (Celsius)
    pub t_warning: f32,
    /// Temperature at which the motor is completely disabled (Celsius)
    pub t_cutoff: f32,
}

impl ThermalConfig {
    /// Helper to guess parameters if you have absolutely no datasheet.
    /// `mass_kg`: Total mass of the motor in kilograms.
    /// `continuous_power_watts`: The safe continuous power rating (if known).
    /// If unknown, use 1/5th of peak power.
    pub fn guess_from_minimal_info(
        mass_kg: f32,
        continuous_power_watts: f32,
        t_max: f32,
        t_ambient: f32,
    ) -> Self {
        // Copper is ~385 J/(kg*K), Iron is ~450 J/(kg*K), Aluminum is ~890 J/(kg*K).
        // A generic brushless motor averages out to about 500 J/(kg*K).
        let c_th = mass_kg * 500.0;

        // Estimate thermal resistance: R_th = delta_T / P_continuous
        // Assumes continuous_power_watts results in exactly t_max.
        let r_th = (t_max - t_ambient) / continuous_power_watts;

        Self {
            r_th,
            c_th,
            t_ambient,
            t_warning: t_max * 0.85, // Warn at 85% of max temp
            t_cutoff: t_max,
        }
    }
}

impl BLDCThermalLimits {
    pub fn new(config: ThermalConfig) -> Self {
        Self {
            config,
            estimated_temp: config.t_ambient, // Assume starting at ambient
        }
    }

    /// Helper to estimate resistive heat without current sensors.
    ///
    /// Because you lack current sensors, you must estimate the current using the
    /// motor's voltage equation: I = (V_applied - Back_EMF) / R
    ///
    /// * `v_q`: The Q-axis voltage you are currently commanding (Volts)
    /// * `v_d`: The D-axis voltage you are currently commanding (Volts) - usually 0.
    /// * `omega_rads`: The mechanical speed of the motor (Rads/sec)
    /// * `kv_rpm_v`: The motor Kv rating (RPM / Volt)
    /// * `r_phase`: The phase resistance (Ohms)
    pub fn estimate_heat_power(
        v_q: f32,
        v_d: f32,
        omega_rads: f32,
        kv_rpm_v: f32,
        r_phase: f32,
    ) -> f32 {
        // Convert Kv (RPM/V) to Ke (V / rad/s)
        let k_e = 1.0 / (kv_rpm_v * (core::f32::consts::PI / 30.0));

        // Estimate Back EMF
        let back_emf = omega_rads * k_e;

        // Estimate Q-axis current: Iq = (Vq - Vbemf) / R
        let i_q = (v_q - back_emf) / r_phase;

        // Estimate D-axis current: Id = Vd / R  (ignoring inductance dynamics)
        let i_d = v_d / r_phase;

        // P = I^2 * R
        let power_q = i_q * i_q * r_phase;
        let power_d = i_d * i_d * r_phase;

        power_q + power_d
    }

    /// Step the thermal model forward in time.
    ///
    /// * `dt_seconds`: Time elapsed since the last loop (Seconds)
    /// * `power_dissipated`: Estimated heat power (Watts)
    ///
    /// Returns a scalar from `0.0` to `1.0`.
    /// Multiply your commanded Q-axis voltage / PWM duty cycle by this number!
    pub fn update(&mut self, dt_seconds: f32, power_dissipated: f32) -> f32 {
        // Calculate heat leaving the motor into the air: P_out = (T_motor - T_air) / R_th
        let power_dissipated_to_air =
            (self.estimated_temp - self.config.t_ambient) / self.config.r_th;

        // Net heat changing the motor temperature
        let net_power = power_dissipated - power_dissipated_to_air;

        // Temperature derivative: dT/dt = P_net / C_th
        let temp_derivative = net_power / self.config.c_th;

        // Integrate temperature
        self.estimated_temp += temp_derivative * dt_seconds;

        // Prevent floating point drift below ambient
        if self.estimated_temp < self.config.t_ambient {
            self.estimated_temp = self.config.t_ambient;
        }

        self.derating_factor()
    }

    pub fn current_temp(&self) -> f32 {
        self.estimated_temp
    }

    fn derating_factor(&self) -> f32 {
        if self.estimated_temp <= self.config.t_warning {
            1.0 // 100% power allowed
        } else if self.estimated_temp >= self.config.t_cutoff {
            0.0 // 0% power allowed (Cutoff)
        } else {
            // Linear throttle between warning and cutoff
            let range = self.config.t_cutoff - self.config.t_warning;
            let excess = self.estimated_temp - self.config.t_warning;
            1.0 - (excess / range)
        }
    }
}

#[cfg(feature = "nope")]
mod prev {

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
}
