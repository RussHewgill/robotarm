use embassy_time::Instant;

pub struct PIDController {
    /// Proportional gain
    pub p: f32,
    /// Integral gain
    pub i: f32,
    /// Derivative gain
    pub d: f32,
    /// Maximum speed of change of the output value
    pub output_ramp: f32,
    /// Maximum output value
    pub limit: f32,

    /// Last tracking error value
    error_prev: f32,
    /// Last pid output value
    output_prev: f32,
    /// Last integral component value
    integral_prev: f32,
    /// Last execution timestamp in microseconds
    timestamp_prev: u64,
}

impl PIDController {
    pub fn new(p: f32, i: f32, d: f32, ramp: f32, limit: f32) -> Self {
        Self {
            p,
            i,
            d,
            output_ramp: ramp,
            limit,
            error_prev: 0.0,
            output_prev: 0.0,
            integral_prev: 0.0,
            timestamp_prev: Instant::now().as_micros(),
        }
    }

    pub fn reset(&mut self) {
        self.integral_prev = 0.0;
        self.output_prev = 0.0;
        self.error_prev = 0.0;
    }

    pub fn update(&mut self, error: f32) -> f32 {
        // calculate the time from the last call
        let timestamp_now = Instant::now().as_micros();
        let mut ts = (timestamp_now.wrapping_sub(self.timestamp_prev)) as f32 * 1e-6;

        // quick fix for strange cases (micros overflow)
        if ts <= 0.0 || ts > 0.5 {
            ts = 1e-3;
        }

        // u(s) = (P + I/s + Ds)e(s)
        // Discrete implementations

        // proportional part
        // u_p  = P *e(k)
        let proportional = self.p * error;

        // Tustin transform of the integral part
        // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
        let mut integral = self.integral_prev + self.i * ts * 0.5 * (error + self.error_prev);
        // antiwindup - limit the output
        integral = integral.clamp(-self.limit, self.limit);

        // Discrete derivation
        // u_dk = D(ek - ek_1)/Ts
        let derivative = self.d * (error - self.error_prev) / ts;

        // sum all the components
        let mut output = proportional + integral + derivative;
        // antiwindup - limit the output variable
        output = output.clamp(-self.limit, self.limit);

        // if output ramp defined
        if self.output_ramp > 0.0 {
            // limit the acceleration by ramping the output
            let output_rate = (output - self.output_prev) / ts;
            if output_rate > self.output_ramp {
                output = self.output_prev + self.output_ramp * ts;
            } else if output_rate < -self.output_ramp {
                output = self.output_prev - self.output_ramp * ts;
            }
        }

        // saving for the next pass
        self.integral_prev = integral;
        self.output_prev = output;
        self.error_prev = error;
        self.timestamp_prev = timestamp_now;

        output
    }
}
