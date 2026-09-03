use num_traits::float::FloatCore;

/// https://github.com/Gip-Gip/standard-pid

/// Dead simple low pass filter implementation, used internally for
/// optionally preventing derivative kick but can be used by the end user
/// as well.
pub struct LowPassFilter<T: FloatCore> {
    accumulator: T,
    time_s: T,
}

impl<T> LowPassFilter<T>
where
    T: FloatCore,
{
    /// Create a low pass filter that will average inputs over a given duration.
    pub fn new(time_s: T, initial: T) -> Self {
        Self {
            time_s,
            accumulator: initial,
        }
    }

    /// Filter out the input given the time since the last filter call,
    /// and return the current computed low pass value.
    pub fn update(&mut self, input: T, dt_s: T) -> T {
        let time_ratio = dt_s / (dt_s + self.time_s);

        let multiplier_x = time_ratio;
        let multiplier_y = T::one() - time_ratio;

        self.accumulator = multiplier_x * input + multiplier_y * self.accumulator;

        self.accumulator
    }
}

/// Standard PID struct implementation.
///
/// ### Example:
///
/// ```rust
/// let mut temp_c: f32 = 25.0; // Pretend this is the input of a thermocouple
///
/// let mut ssr: bool = false; // And this is the output to a solid state relay
///
/// let mut pid: StandardPID = StandardPID::new(
///     1.4, // Kp
///     10.2, // Ti
///     20.8, // Td
///     100.0, // Setpoint of 100c
///     0.0, // Minimum output value
///     1.0 // Maximum output value
/// )
///     .d_low_pass(1.0) // Set the low pass filter to average over 1 second
///     .i_band(10.0); // Disable the integral when 10c outside the target temperature
///
/// loop {
///     // Insert misc. logic here..
///
///     // Turn on the SSR if the output is greater than 0.5
///     // Delta time is hard coded in this example
///     ssr = pid.update(temp_c, 0.1) > 0.5;
/// }
///
/// ```
pub struct StandardPID<T: FloatCore> {
    kp: T,
    one_over_ti_s: T,
    td_s: T,
    i_band: T,
    p: T,
    i: T,
    d: T,
    error: T,
    sp: T,
    out_range_min: T,
    out_range_max: T,
    d_low_pass: Option<LowPassFilter<T>>,
    feed_forward: T,
}

impl<T> StandardPID<T>
where
    T: FloatCore,
{
    /// Create a new PID controller given the Kp, Ti, Td, Sp, and the out range.
    pub fn new(kp: T, ti_s: T, td_s: T, sp: T, out_range_min: T, out_range_max: T) -> Self {
        let ti_s = ti_s.max(T::min_positive_value());
        let td_s = td_s.max(T::zero());

        Self {
            kp,
            one_over_ti_s: T::one() / ti_s,
            td_s,
            // i_band: T::max_value(),
            i_band: T::from(2f32).unwrap(),
            p: T::zero(),
            i: T::zero(),
            d: T::zero(),
            error: T::zero(),
            sp,
            out_range_min,
            out_range_max,
            // d_low_pass: None,
            d_low_pass: Some(LowPassFilter::new(T::from(0.005).unwrap(), T::zero())),
            feed_forward: T::zero(),
        }
    }

    pub fn update(&mut self, pv: T, dt_s: T) -> (T, (T, T, T, T)) {
        let old_error = self.error;
        self.error = self.sp - pv;
        let delta_error = self.error - old_error;

        let delta_error = if let Some(mut lp_filter) = self.d_low_pass.take() {
            let val = lp_filter.update(delta_error, dt_s);

            self.d_low_pass = Some(lp_filter);

            val
        } else {
            delta_error
        };

        // Calculate tentative integral change (dI)
        let mut delta_i = self.one_over_ti_s * self.error * dt_s;

        // Existing `i_band` logic: reset inner integral if error is out of band bounds
        if self.error.abs() > self.i_band {
            self.i = T::zero();
            delta_i = T::zero();
        }

        // Calculate D term
        // unstable for very small dt_s
        self.d = self.td_s * (delta_error / dt_s);

        // Calculate tentative unconstrained output
        // Standard form: P_total = Kp * (e + I + dI + D)
        let tentative_p = self.kp * (self.error + self.i + delta_i + self.d);

        // no anti-windup
        // #[cfg(feature = "nope")]
        {
            // self.i = T::zero();
            // self.p = self.kp * (self.error + self.d);
            self.i = self.i + delta_i;
            self.p = tentative_p;
        }

        // conditional anti-windup
        #[cfg(feature = "nope")]
        {
            let is_saturated_high = tentative_p > self.out_range_max;
            let is_saturated_low = tentative_p < self.out_range_min;

            // Freeze integration if we are saturated AND the error is trying to
            // push us further into saturation.
            if (is_saturated_high && self.error > T::zero())
                || (is_saturated_low && self.error < T::zero())
            {
                delta_i = T::zero();
            }

            self.i = self.i + delta_i;
            self.p = self.kp * (self.error + self.i + self.d);
        }

        // back calculation anti-windup
        #[cfg(feature = "nope")]
        {
            let tt_s = self.get_ti_s();

            // Normal integration for this timestep
            self.i = self.i + delta_i;
            self.p = self.kp * (self.error + self.i + self.d);

            // Calculate the difference between saturated and unsaturated output
            let out = self.out_range_min.max(self.out_range_max.min(self.p));
            let excess_output = out - self.p;

            // Back-calculate the integral for the *next* time step
            // Standard form back-calculation: dI = (Excess / Kp) * (dt / Tt)
            if self.kp != T::zero() && tt_s > T::zero() {
                self.i = self.i + (excess_output / self.kp) * (dt_s / tt_s);
            }
        }

        let out = self.p + self.feed_forward * self.sp;
        let out = out.clamp(self.out_range_min, self.out_range_max);

        (out, (self.error, self.p, self.i, self.d))
    }

    pub fn reset(&mut self) {
        self.i = T::zero();
        self.d = T::zero();
        self.p = T::zero();
        self.error = T::zero();
    }

    #[inline]
    /// Consuming variant of `set_d_low_pass`.
    pub fn d_low_pass(mut self, time_s: T) -> Self {
        self.set_d_low_pass(time_s);

        self
    }

    #[inline]
    /// Consuming variant of `set_i_band`.
    pub fn i_band(mut self, i_band: T) -> Self {
        self.set_i_band(i_band);

        self
    }

    #[inline]
    /// Reset the integral term to zero.
    pub fn reset_integral_term(&mut self) {
        self.i = T::zero();
    }

    #[inline]
    /// Enable the low pass filter on the derivative term and set the averaging time.
    pub fn set_d_low_pass(&mut self, time_s: T) {
        // self.d_low_pass = Some(LowPassFilter::new(time_s, self.sp));
        self.d_low_pass = Some(LowPassFilter::new(time_s, T::zero()));
    }

    #[inline]
    /// Set the proportional gain.
    pub fn set_kp(&mut self, kp: T) {
        self.kp = kp;
    }

    #[inline]
    /// Set the integral time. **note: cannot less than or equal to zero; inputs are sanitized to
    /// prevent panics and unexpected behaviour**.
    pub fn set_ti_s(&mut self, ti_s: T) {
        let ti_s = ti_s.max(T::min_positive_value());
        self.one_over_ti_s = T::one() / ti_s;
    }

    #[inline]
    /// Set the derivative time. **note: cannot be less than zero; inputs are sanitized to prevent
    /// unexpected behaviour**.
    pub fn set_td_s(&mut self, td_s: T) {
        let td_s = td_s.max(T::zero());
        self.td_s = td_s;
    }

    #[inline]
    /// Enable and set the I band. If the absolute error between the process
    /// variable and the set point is greater than the I band, the integral
    /// term will be set to zero.
    pub fn set_i_band(&mut self, i_band: T) {
        let i_band = i_band.max(T::zero());
        self.i_band = i_band;
    }

    #[inline]
    /// Set the set point.
    pub fn set_sp(&mut self, sp: T) {
        self.sp = sp;
    }

    #[inline]
    pub fn set_feed_forward(&mut self, feed_forward: T) {
        self.feed_forward = feed_forward;
    }

    #[inline]
    /// Get the proportional gain.
    pub fn get_kp(&self) -> T {
        self.kp
    }

    #[inline]
    /// Get the integral time.
    pub fn get_ti_s(&self) -> T {
        T::one() / self.one_over_ti_s
    }

    #[inline]
    /// Get the derivative time.
    pub fn get_td_s(&self) -> T {
        self.td_s
    }

    #[inline]
    /// Get the set point.
    pub fn get_sp(&self) -> T {
        self.sp
    }

    #[inline]
    pub fn set_output_range(&mut self, min: T, max: T) {
        self.out_range_min = min;
        self.out_range_max = max;
    }

    pub fn get_output_range(&self) -> (T, T) {
        (self.out_range_min, self.out_range_max)
    }

    pub fn get_feed_forward(&self) -> T {
        self.feed_forward
    }

    pub fn get_i_band(&self) -> T {
        self.i_band
    }

    pub fn get_d_lpf(&self) -> Option<T> {
        self.d_low_pass.as_ref().map(|lp| lp.time_s)
    }
}
