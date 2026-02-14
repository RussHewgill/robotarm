use defmt::debug;
use embassy_time::Instant;

use crate::simplefoc::pid::PIDController;

/// https://github.com/ahmedosama07/PID-autotune/blob/main/src/pid-autotune.cpp
pub struct PidTuner {
    // pid: PIDController,
    done: bool,

    target_input: f32,
    hysteresis: f32,
    output_low: f32,
    output_high: f32,

    cycles_requested: u32,
    cycle_count: u32,
    // timeout_ms: u64,
    output_state: bool,
    last_switch_us: u64,

    peak_min: f32,
    peak_max: f32,

    kp_sum: f32,
    ki_sum: f32,
    kd_sum: f32,

    n_samples: u32,
}

impl PidTuner {
    pub fn new(pid: &PIDController, target_input: f32) -> Self {
        let output_low = -pid.get_limit();
        let output_high = pid.get_limit();
        Self {
            // pid,
            done: false,
            target_input,
            hysteresis: 1.0,
            output_low,
            output_high,
            cycles_requested: 100,
            cycle_count: 0,
            // timeout_ms: 30_000,
            output_state: true,
            last_switch_us: Instant::now().as_micros(),
            peak_min: f32::MAX,
            peak_max: f32::MIN,
            kp_sum: 0.0,
            ki_sum: 0.0,
            kd_sum: 0.0,
            n_samples: 0,
        }
    }

    pub fn update(&mut self, input: f32, t_us: u64) -> f32 {
        if self.done {
            return 0.;
        }

        self.peak_max = self.peak_max.max(input);
        self.peak_min = self.peak_min.min(input);

        if self.output_state && (input > self.target_input + self.hysteresis) {
            self.output_state = false
        } else if !self.output_state && (input < self.target_input - self.hysteresis) {
            // Switch to HIGH - One full cycle complete
            debug!(
                "Cycle {} complete. Remaining: {} Input: {}",
                self.cycle_count,
                self.cycles_requested - self.cycle_count,
                input
            );

            let elapsed = t_us - self.last_switch_us;

            // Ultimate Period (Tu) in seconds
            let tu = elapsed as f32 * 1e-6;

            self.last_switch_us = t_us;
            self.output_state = true;

            // Calculate amplitudes
            let a = (self.peak_max - self.peak_min) / 2.0;
            let d = (self.output_high - self.output_low) / 2.0;

            // Ultimate gain

            if a > 0. {
                let ku = (4.0 * d) / (core::f32::consts::PI * a);

                let kp_c = 0.6;
                let ti_c = 0.5;
                let td_c = 0.125;

                // // Tuning rules
                // if (_mode == PESSEN_INTEGRAL_RULE)
                // {
                //     kp_c = 0.7;
                //     ti_c = 0.4;
                //     td_c = 0.15;
                // }
                // else if (_mode == SOME_OVERSHOOT)
                // {
                //     kp_c = 0.33;
                //     ti_c = 0.5;
                //     td_c = 0.33;
                // }
                // else if (_mode == NO_OVERSHOOT)
                // {
                //     kp_c = 0.2;
                //     ti_c = 0.5;
                //     td_c = 0.33;
                // }

                // Skip the first cycle as it is often erratic (transient)
                if self.cycle_count > 0 {
                    let current_kp = kp_c * ku;
                    self.kp_sum += current_kp;
                    self.ki_sum += current_kp / (ti_c * tu);
                    self.kd_sum += current_kp * (td_c * tu);
                    self.n_samples += 1;
                }
            } else {
                debug!("Amplitude too small for tuning");
                debug!("Peak Max: {}, Peak Min: {}", self.peak_max, self.peak_min);
                debug!(
                    "Output High: {}, Output Low: {}",
                    self.output_high, self.output_low
                );
            }

            // Reset peaks for next cycle
            self.peak_max = -1e12;
            self.peak_min = 1e12;
            self.cycle_count += 1;

            // Check if finished
            if self.cycle_count >= self.cycles_requested {
                if self.n_samples > 0 {
                    debug!("Done tune");
                    // _pid.setConstants(_kp_sum / _samples, _ki_sum / _samples, _kd_sum / _samples);
                    debug!(
                        "PID Tuning complete: Kp: {}, Ki: {}, Kd: {}",
                        self.kp_sum / self.n_samples as f32,
                        self.ki_sum / self.n_samples as f32,
                        self.kd_sum / self.n_samples as f32
                    );
                    self.done = true;
                    return 0.;
                }
                // _state = DONE;
            }
        }

        // return _output_state ? _output_high : _output_low;
        if self.output_state {
            self.output_high
        } else {
            self.output_low
        }
    }

    pub fn done(&self) -> bool {
        self.done
    }
}
