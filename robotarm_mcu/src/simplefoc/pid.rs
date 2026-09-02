use defmt::debug;
use embassy_time::Instant;
use num_traits::float::FloatCore;

use crate::simplefoc::pid_standard;

pub struct PIDController<T: FloatCore> {
    // pid2: discrete_pid::pid::PidController<discrete_pid::time::Micros, f32>,
    pid2: discrete_pid::pid::PidController<discrete_pid::time::Micros, f64>,
    pid: self::prev::PIDController,
    // pid3: (pidgeon::ControllerConfig, pidgeon::PidState),
    pid4: pid_standard::StandardPID<T>,
    // pid4: pid_standard::StandardPID<f32>,
    ramp: f32,
    prev_output: f32,
    prev_internals: (f32, f32, f32, f32),
    prev_t_us: u64,
}

impl<T: FloatCore> PIDController<T> {
    pub fn new(p: f32, i: f32, d: f32, ramp: f32, limit: f32) -> Self {
        // debug!(
        //     "Creating PIDController with p: {}, i: {}, d: {}, ramp: {}, limit: {}",
        //     p, i, d, ramp, limit
        // );

        let config = discrete_pid::pid::PidConfigBuilder::default()
            .kp(p as f64)
            .ki(i as f64)
            .kd(d as f64)
            .output_limits(-limit as f64, limit as f64)
            .sample_time(core::time::Duration::from_micros(100))
            .filter_tc(0.000001)
            .use_derivative_on_measurement(false)
            .build()
            .expect("Invalid PID config");
        let mut pid2 = discrete_pid::pid::PidController::new_uninit(config);
        pid2.activate();
        // let _ = pid2.config_mut().set_filter_tc(0.000001);
        // let _ = pid.config_mut().set_use_strict_causal_integrator(true);
        // let _ = pid2.config_mut().set_use_derivative_on_measurement(true);
        // let _ = pid2.config_mut().set_use_derivative_on_measurement(false);

        let pid = self::prev::PIDController::new(p, i, d, ramp, limit);

        // use pidgeon::{ControllerConfig, PidState, pid_compute};
        // let config = ControllerConfig::builder()
        //     .with_kp(p as f64)
        //     // .with_ki(i as f64)
        //     .with_kd(d as f64)
        //     // .with_kp(0.05)
        //     .with_ki(0.0)
        //     // .with_kd(0.0005)
        //     .with_setpoint(0.0)
        //     .with_output_limits(-limit as f64, limit as f64)
        //     // .with_anti_windup_mode(pidgeon::AntiWindupMode::BackCalculation { tracking_time: 0.05 })
        //     .with_anti_windup_mode(pidgeon::AntiWindupMode::Conditional)
        //     .with_derivative_mode(pidgeon::DerivativeMode::OnMeasurement)
        //     .with_derivative_filter_coeff(10.0)
        //     .with_deadband(0.0)
        //     .build()
        //     .unwrap();
        // let pid3 = PidState::default();

        // let pid4 = pid_standard::StandardPID::new(
        //     p as f64,
        //     i as f64,
        //     d as f64,
        //     0.0,
        //     -limit as f64,
        //     limit as f64,
        // );

        // let pid4 = pid_standard::StandardPID::new(
        //     p as f32,
        //     i as f32,
        //     d as f32,
        //     0.0,
        //     -limit as f32,
        //     limit as f32,
        // );

        let pid4 = pid_standard::StandardPID::new(
            T::from(p).unwrap(),
            T::from(i).unwrap(),
            T::from(d).unwrap(),
            T::from(0.0).unwrap(),
            T::from(-limit).unwrap(),
            T::from(limit).unwrap(),
        );

        Self {
            pid,
            pid2,
            // pid3: (config, pid3),
            pid4,
            ramp,
            prev_output: 0.0,
            prev_internals: (0.0, 0.0, 0.0, 0.0),
            prev_t_us: 0,
        }
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
        // self.pid.reset();
        // self.pid2.reset();
        self.pid4.reset();
        self.prev_output = 0.0;
        self.prev_internals = (0.0, 0.0, 0.0, 0.0);
        self.prev_t_us = 0;
    }

    // #[cfg(feature = "nope")]
    pub fn update(&mut self, setpoint: f32, input: f32, t_us: u64) -> f32 {
        // let output = self.pid2.compute(
        //     input as f64,
        //     setpoint as f64,
        //     discrete_pid::time::Micros(t_us),
        //     None,
        // );

        // let config = pidgeon::ControllerConfigBuilder::new()
        //     .with_kp(self.pid3.0.kp())
        //     .with_ki(self.pid3.0.ki())
        //     .with_kd(self.pid3.0.kd())
        //     .with_setpoint(setpoint as f64)
        //     .with_output_limits(self.pid3.0.min_output(), self.pid3.0.max_output())
        //     .with_anti_windup_mode(self.pid3.0.anti_windup_mode())
        //     .with_deadband(self.pid3.0.deadband())
        //     .with_derivative_mode(self.pid3.0.derivative_mode())
        //     .with_derivative_filter_coeff(self.pid3.0.derivative_filter_coeff())
        //     .build()
        //     .unwrap();
        // self.pid3.0 = config;
        // let (output, _state) =
        //     pidgeon::pid_compute(&self.pid3.0, &self.pid3.1, input as f64, t_us as f64 * 1e-6)
        //         .unwrap();
        // self.pid3.1 = _state;

        if self.prev_t_us == 0 {
            self.prev_t_us = t_us;
            return 0.0;
        }

        // self.pid4.set_sp(setpoint as f64);
        // let dt = (t_us - self.prev_t_us) as f64 * 1e-6;
        // let (output, internals) = self.pid4.update(input as f64, dt);

        // self.pid4.set_sp(setpoint as f32);
        // let dt = (t_us - self.prev_t_us) as f32 * 1e-6;
        // let (output, internals) = self.pid4.update(input as f32, dt);

        self.pid4.set_sp(T::from(setpoint).unwrap());
        let dt = T::from(t_us - self.prev_t_us).unwrap() * T::from(1e-6).unwrap();
        let (output, internals) = self.pid4.update(T::from(input).unwrap(), dt);

        // debug!("output: {}", output);

        self.prev_internals = (
            internals.0.to_f32().unwrap(),
            internals.1.to_f32().unwrap(),
            internals.2.to_f32().unwrap(),
            internals.3.to_f32().unwrap(),
        );

        // self.prev_output = output as f32;
        // output as f32
        self.prev_output = output.to_f32().unwrap();
        output.to_f32().unwrap()
    }

    pub fn prev_output(&self) -> f32 {
        self.prev_output
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
impl<T: FloatCore> PIDController<T> {
    pub fn get_p(&self) -> f32 {
        self.pid2.config().kp() as f32
        // self.pid3.0.kp() as f32
    }
    pub fn get_i(&self) -> f32 {
        self.pid2.config().ki() as f32
        // self.pid3.0.ki() as f32
    }
    pub fn get_d(&self) -> f32 {
        self.pid2.config().kd() as f32
        // self.pid3.0.kd() as f32
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
            b as f32
        } else {
            // asymmetric limits not supported
            0.0
        }
    }
    pub fn set_p(&mut self, p: f32) {
        // let _ = self.pid2.config_mut().set_kp(p);
        let mut conf = *self.pid2.config();
        // conf.set_kp(p as f64).expect("Invalid PID config");
        if let Err(e) = conf.set_kp(p as f64) {
            // debug!("Failed to set kp: {}", e);
            return;
        }
        self.pid2.set_config(conf);
        self.pid.p = p;
        // let config = pidgeon::ControllerConfigBuilder::new()
        //     .with_kp(p as f64)
        //     .with_ki(self.pid3.0.ki())
        //     .with_kd(self.pid3.0.kd())
        //     .with_setpoint(self.pid3.0.setpoint())
        //     .with_output_limits(self.pid3.0.min_output(), self.pid3.0.max_output())
        //     .with_anti_windup_mode(self.pid3.0.anti_windup_mode())
        //     .with_deadband(self.pid3.0.deadband())
        //     .with_derivative_mode(self.pid3.0.derivative_mode())
        //     .with_derivative_filter_coeff(self.pid3.0.derivative_filter_coeff())
        //     .build()
        //     .unwrap();
        // self.pid3.0 = config;
        self.pid4.set_kp(T::from(p).unwrap());
    }
    pub fn set_i(&mut self, i: f32) {
        let mut conf = *self.pid2.config();
        conf.set_ki(i as f64).expect("Invalid PID config");
        self.pid2.set_config(conf);
        self.pid.i = i;
        // let config = pidgeon::ControllerConfigBuilder::new()
        //     .with_kp(self.pid3.0.kp())
        //     .with_ki(i as f64)
        //     .with_kd(self.pid3.0.kd())
        //     .with_setpoint(self.pid3.0.setpoint())
        //     .with_output_limits(self.pid3.0.min_output(), self.pid3.0.max_output())
        //     .with_anti_windup_mode(self.pid3.0.anti_windup_mode())
        //     .with_deadband(self.pid3.0.deadband())
        //     .with_derivative_mode(self.pid3.0.derivative_mode())
        //     .with_derivative_filter_coeff(self.pid3.0.derivative_filter_coeff())
        //     .build()
        //     .unwrap();
        // self.pid3.0 = config;
        // self.pid4.set_ki(i as f64);
        // self.pid4 = standard_pid::StandardPID::new(
        //     // self.pid4.get_kp(),
        //     self.pid.p as f64,
        //     i as f64,
        //     self.pid.d as f64,
        //     0.0,
        //     -self.pid.limit as f64,
        //     self.pid.limit as f64,
        // );
        self.pid4.set_ti_s(T::from(i).unwrap());
        self.pid4.reset_integral_term();
    }
    pub fn set_d(&mut self, d: f32) {
        let mut conf = *self.pid2.config();
        conf.set_kd(d as f64).expect("Invalid PID config");
        self.pid2.set_config(conf);
        self.pid.d = d;
        // let config = pidgeon::ControllerConfigBuilder::new()
        //     .with_kp(self.pid3.0.kp())
        //     .with_ki(self.pid3.0.ki())
        //     .with_kd(d as f64)
        //     .with_setpoint(self.pid3.0.setpoint())
        //     .with_output_limits(self.pid3.0.min_output(), self.pid3.0.max_output())
        //     .with_anti_windup_mode(self.pid3.0.anti_windup_mode())
        //     .with_deadband(self.pid3.0.deadband())
        //     .with_derivative_mode(self.pid3.0.derivative_mode())
        //     .with_derivative_filter_coeff(self.pid3.0.derivative_filter_coeff())
        //     .build()
        //     .unwrap();
        // self.pid3.0 = config;
        // self.pid4.set_kd(d as f64);
        // self.pid4 = standard_pid::StandardPID::new(
        //     self.pid.p as f64,
        //     // self.pid4.get_kp(),
        //     self.pid.i as f64,
        //     d as f64,
        //     0.0,
        //     -self.pid.limit as f64,
        //     self.pid.limit as f64,
        // );
        self.pid4.set_td_s(T::from(d).unwrap());
    }
    pub fn set_ramp(&mut self, ramp: f32) {
        self.ramp = ramp;
    }
    pub fn set_limit(&mut self, limit: f32) {
        debug!("Setting PID limit to {}", limit);
        // let _ = self.pid2.config_mut().set_output_limits(-limit, limit);
        let mut conf = *self.pid2.config();
        conf.set_output_limits(-limit as f64, limit as f64)
            .expect("Invalid PID config");
        // self.pid2.set_config(conf);
        // self.pid.limit = limit;
        // let config = pidgeon::ControllerConfigBuilder::new()
        //     .with_kp(self.pid3.0.kp())
        //     .with_ki(self.pid3.0.ki())
        //     .with_kd(self.pid3.0.kd())
        //     .with_setpoint(self.pid3.0.setpoint())
        //     .with_output_limits(-limit as f64, limit as f64)
        //     .with_anti_windup_mode(self.pid3.0.anti_windup_mode())
        //     .with_deadband(self.pid3.0.deadband())
        //     .with_derivative_mode(self.pid3.0.derivative_mode())
        //     .with_derivative_filter_coeff(self.pid3.0.derivative_filter_coeff())
        //     .build()
        //     .unwrap();
        // self.pid3.0 = config;
        self.pid4
            .set_output_range(T::from(-limit).unwrap(), T::from(limit).unwrap());
    }

    pub fn prev_internals(&self) -> (f32, f32, f32, f32) {
        self.prev_internals
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
