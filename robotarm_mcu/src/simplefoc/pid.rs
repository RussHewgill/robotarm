use defmt::debug;
use embassy_time::Instant;

pub struct PIDController {
    pid2: discrete_pid::pid::PidController<discrete_pid::time::Micros, f32>,
    pid: self::prev::PIDController,
    ramp: f32,
}

impl PIDController {
    pub fn new(p: f32, i: f32, d: f32, ramp: f32, limit: f32) -> Self {
        // debug!(
        //     "Creating PIDController with p: {}, i: {}, d: {}, ramp: {}, limit: {}",
        //     p, i, d, ramp, limit
        // );

        let config = discrete_pid::pid::PidConfigBuilder::default()
            .kp(p)
            .ki(i)
            .kd(d)
            .output_limits(-limit, limit)
            .sample_time(core::time::Duration::from_micros(100))
            .build()
            .expect("Invalid PID config");
        let mut pid2 = discrete_pid::pid::PidController::new_uninit(config);
        pid2.activate();
        let _ = pid2.config_mut().set_filter_tc(0.000001);
        // let _ = pid.config_mut().set_use_strict_causal_integrator(true);
        // let _ = pid2.config_mut().set_use_derivative_on_measurement(true);
        let _ = pid2.config_mut().set_use_derivative_on_measurement(false);

        let pid = self::prev::PIDController::new(p, i, d, ramp, limit);

        Self { pid, pid2, ramp }
    }

    #[cfg(feature = "nope")]
    pub fn clone(&self) -> Self {
        let config = self.pid.config().clone();
        let pid = discrete_pid::pid::PidController::new_uninit(config);
        Self {
            // pid: self.pid.clone(),
            pid,
            ramp: self.ramp,
        }
    }

    pub fn reset(&mut self) {
        unimplemented!()
    }

    // #[cfg(feature = "nope")]
    pub fn update(&mut self, setpoint: f32, input: f32, t_us: u64) -> f32 {
        let output = self
            .pid2
            .compute(input, setpoint, discrete_pid::time::Micros(t_us), None);
        output
    }

    #[cfg(feature = "nope")]
    pub fn update(&mut self, setpoint: f32, input: f32, t_us: u64) -> f32 {
        let output = self.pid.update(setpoint - input, t_us);

        // debug!(
        //     "PID update: setpoint: {}, input: {}, output1: {}, output2: {}",
        //     setpoint, input, output1, output2
        // );

        output
        // 0.0
    }
}

#[cfg(feature = "nope")]
impl PIDController {
    pub fn get_p(&self) -> f32 {
        self.pid.p
    }
    pub fn get_i(&self) -> f32 {
        self.pid.i
    }
    pub fn get_d(&self) -> f32 {
        self.pid.d
    }
    pub fn get_ramp(&self) -> f32 {
        self.ramp
    }
    pub fn get_limit(&self) -> f32 {
        self.pid.limit
    }
    pub fn set_p(&mut self, p: f32) {
        self.pid.p = p;
    }
    pub fn set_i(&mut self, i: f32) {
        self.pid.i = i;
    }
    pub fn set_d(&mut self, d: f32) {
        self.pid.d = d;
    }
    pub fn set_ramp(&mut self, ramp: f32) {
        self.ramp = ramp;
    }
    pub fn set_limit(&mut self, limit: f32) {
        self.pid.limit = limit;
    }
}

// #[cfg(feature = "nope")]
impl PIDController {
    pub fn get_p(&self) -> f32 {
        self.pid2.config().kp()
    }
    pub fn get_i(&self) -> f32 {
        self.pid2.config().ki()
    }
    pub fn get_d(&self) -> f32 {
        self.pid2.config().kd()
    }
    pub fn get_ramp(&self) -> f32 {
        self.ramp
    }
    pub fn get_limit(&self) -> f32 {
        let (a, b) = (
            self.pid2.config().output_min(),
            self.pid2.config().output_max(),
        );
        if a == -b {
            b
        } else {
            // asymmetric limits not supported
            0.0
        }
    }
    pub fn set_p(&mut self, p: f32) {
        let _ = self.pid2.config_mut().set_kp(p);
        self.pid.p = p;
    }
    pub fn set_i(&mut self, i: f32) {
        let _ = self.pid2.config_mut().set_ki(i);
        self.pid.i = i;
    }
    pub fn set_d(&mut self, d: f32) {
        let _ = self.pid2.config_mut().set_kd(d);
        self.pid.d = d;
    }
    pub fn set_ramp(&mut self, ramp: f32) {
        self.ramp = ramp;
    }
    pub fn set_limit(&mut self, limit: f32) {
        debug!("Setting PID limit to {}", limit);
        let _ = self.pid2.config_mut().set_output_limits(-limit, limit);
        self.pid.limit = limit;
    }
}

// #[cfg(feature = "nope")]
mod prev {
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

        pub fn update(&mut self, error: f32, timestamp_now: u64) -> f32 {
            // // calculate the time from the last call
            // let timestamp_now = Instant::now().as_micros();
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
}
