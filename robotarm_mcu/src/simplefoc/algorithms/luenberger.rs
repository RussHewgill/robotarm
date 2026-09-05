use defmt::debug;

use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::{RealField, SMatrix, SVector};

use crate::{
    hardware::{current_sensor::CurrentSensor, encoder_sensor::EncoderSensor},
    simplefoc::{foc_types::SimpleFOC, types::_2PI},
};

/// https://github.com/JRL-CARI-CNR-UNIBS/state_observers

pub mod luenberger_optimize {
    use defmt::debug;

    use embassy_time::{Duration, Instant, Ticker, Timer};
    use nalgebra::{SMatrix, SVector};

    use crate::{
        hardware::{current_sensor::CurrentSensor, encoder_sensor::EncoderSensor},
        simplefoc::foc_types::SimpleFOC,
    };

    const TIME_MS: f32 = 30.;
    // const TIME_MS: f32 = 50.;
    const LOOP_HZ: f32 = 10000.;
    const N_SAMPLES: usize = ((TIME_MS / 1000.) * LOOP_HZ) as usize;

    // use a window of samples to calculate velocity to reduce noise
    const WINDOW: usize = 3;

    impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
        pub async fn optimize_state_observer(&mut self) {
            self.enable();

            const N: usize = 5;

            let mut inertia_ccw: heapless::Vec<f32, N> = heapless::Vec::new();
            let mut inertia_cw: heapless::Vec<f32, N> = heapless::Vec::new();

            let voltage = self.motor.voltage_sensor_align;

            for _ in 0..N {
                let ((inertia, _), _) = self.optimize_state_observer_sweep(voltage, 1.0).await;
                inertia_ccw.push(inertia).unwrap();
            }

            for _ in 0..N {
                let ((inertia, _), _) = self.optimize_state_observer_sweep(voltage, -1.0).await;
                inertia_cw.push(inertia).unwrap();
            }

            let avg_ccw = inertia_ccw.iter().sum::<f32>() / inertia_ccw.len() as f32;
            let avg_cw = inertia_cw.iter().sum::<f32>() / inertia_cw.len() as f32;
            let avg_ccw = avg_ccw * 10_000_000.; // convert from kg*m^2 to g*cm^2
            let avg_cw = avg_cw * 10_000_000.; // convert from kg*m^2 to g*cm^2

            let datasheet_inertia = 0.000_035_5 * 10_000_000.; // Rotor Moment of inertia (kg*m^2)
            let error_ccw = avg_ccw - datasheet_inertia;
            let error_cw = avg_cw - datasheet_inertia;

            let avg = (avg_ccw + avg_cw) / 2.0;
            let error_avg = avg - datasheet_inertia;

            debug!("Average Inertia CCW: {}, error: {}", avg_ccw, error_ccw);
            debug!("Average Inertia CW:  {}, error: {}", avg_cw, error_cw);
            debug!("Average Inertia:     {}, error: {}", avg, error_avg);

            //
        }

        async fn optimize_state_observer_sweep(
            &mut self,
            voltage: f32,
            direction: f32,
        ) -> ((f32, f32), heapless::Vec<(u64, f32, f32), N_SAMPLES>) {
            let mut samples: heapless::Vec<(u64, f32, f32), N_SAMPLES> = heapless::Vec::new();

            let mut shaft_angle = 0.0;

            let vel = 5.; // rad/s
            let angle = vel * (N_SAMPLES as f32 / LOOP_HZ);

            // let motor settle
            self.set_phase_voltage(voltage, 0., 0.);
            Timer::after_millis(1000).await;
            self.set_phase_voltage(0., 0., 0.);
            Timer::after_millis(200).await;

            let step = Duration::from_micros(1_000_000 / LOOP_HZ as u64).as_micros();
            let mut next_t = Instant::now().as_micros() + step;

            let t0 = Instant::now();
            let _ = self.encoder.update(t0.as_micros()).await;

            let angle0 = self.encoder.get_angle();

            for i in 0..N_SAMPLES {
                let now = loop {
                    let now = Instant::now().as_micros();
                    if now >= next_t {
                        next_t = now + step;
                        break Instant::now();
                    }
                    Timer::after_micros(1).await;
                };

                let t_us = now.as_micros();

                let shaft_angle = angle * (i as f32 / N_SAMPLES as f32);
                let mut electrical_angle =
                    self.sensor_direction.multiplier() * shaft_angle * self.motor.pole_pairs as f32;

                if direction < 0.0 {
                    electrical_angle = crate::simplefoc::types::_2PI * self.motor.pole_pairs as f32
                        - electrical_angle;
                }

                self.set_phase_voltage(voltage, 0., electrical_angle);

                let _ = self.encoder.update(t_us).await;

                let measured_angle = (self.encoder.get_angle() - angle0) * direction;
                samples
                    .push((t_us - t0.as_micros(), shaft_angle, measured_angle))
                    .unwrap();
            }
            let duration = t0.elapsed();
            self.set_phase_voltage(0., 0., 0.);

            let i0 = self.identify_inertia(voltage, samples.clone()).unwrap();

            debug!(
                // "Estimated Inertia (identify_inertia):   {}",
                "Estimated Inertia: {}",
                i0 * 10_000_000.
            );

            ((i0, 0.), samples)
        }

        /// Equation: Kt * Iq = J * (d_omega/d_t) + B * omega
        pub fn identify_inertia(
            &self,
            voltage: f32,
            samples: heapless::Vec<(u64, f32, f32), N_SAMPLES>,
            // samples: heapless::Vec<(u64, f32), N_SAMPLES>,
        ) -> Option<f32> {
            // if n < 2 {
            //     return None;
            // }

            // We will build a system of equations: Y = X * Theta
            // Y = Kt * Iq (Torque)
            // X = [d_omega/d_t, omega]
            // Theta = [J, B]^T (Inertia and Viscous Friction)

            let mut vel = heapless::Vec::<(u64, f32), N_SAMPLES>::new();

            // // convert position samples to velocity samples
            // for i in WINDOW..N_SAMPLES {
            //     let dt = (samples[i].0 - samples[i - WINDOW].0) as f32 * 1e-6; // Convert microseconds to seconds
            //     let d_angle = samples[i].1 - samples[i - WINDOW].1;
            //     let velocity = d_angle / dt;
            //     vel.push((samples[i].0, velocity)).unwrap();
            // }

            let samples0 = samples;
            let mut samples = heapless::Vec::<(f32, f32), N_SAMPLES>::new();

            for i in WINDOW..N_SAMPLES {
                let dt = (samples0[i].0 - samples0[i - WINDOW].0) as f32 * 1e-6; // Convert microseconds to seconds
                let d_angle = samples0[i].2 - samples0[i - WINDOW].2;
                let velocity = d_angle / dt;
                // samples.push((samples0[i].0, velocity)).unwrap();
                samples.push((dt, velocity)).unwrap();
            }

            let mut y_data = SVector::<f32, { N_SAMPLES - WINDOW - 1 }>::zeros();
            let mut x_data = SMatrix::<f32, { N_SAMPLES - WINDOW - 1 }, 2>::zeros();

            let t0 = samples[WINDOW].0;
            // debug!("t0: {}", t0);

            // Convert T to f64 for nalgebra DMatrix (or use T directly if supported by your nalgebra setup)
            for i in WINDOW..N_SAMPLES - WINDOW {
                // let dt = (samples[i].0 - samples[i - WINDOW].0) as f32 * 1e-6; // Convert microseconds to seconds
                // let d_omega = vel[i].1 - vel[i - WINDOW].1;
                let dt = samples[i].0;
                let d_omega = samples[i].1 - samples[i - WINDOW].1;
                let alpha = d_omega / dt;

                // let omega_avg = (vel[i].1 + vel[i - WINDOW].1) / 2.0;
                let omega_avg = (samples[i].1 + samples[i - WINDOW].1) / 2.0;

                let torque = self.state_observer.torque_constant() * voltage;

                // y_data.push(torque).unwrap();
                // x_data.push((alpha, omega_avg)).unwrap();
                y_data[i - WINDOW] = torque;
                x_data[(i - WINDOW, 0)] = alpha;
                x_data[(i - WINDOW, 1)] = omega_avg;
            }

            // debug!("y_data.shape(): {:?}", y_data.shape());
            // debug!("x_data.shape(): {:?}", x_data.shape());

            let y = y_data;
            let x = x_data;

            // // Solve Ordinary Least Squares

            // Theta = (X^T X)^-1 X^T Y
            let xt = x.transpose();
            let xt_x = &xt * &x;

            // debug!("x = {:?}", x.as_slice());
            // debug!("xt = {:?}", xt.as_slice());

            if let Some(xt_x_inv) = xt_x.try_inverse() {
                let theta = xt_x_inv * xt * y;

                let inertia = theta[0];
                let _friction = theta[1]; // Useful if you want to include B in your observer!

                // self.inertia = Some(inertia);
                // debug!("Inertia: {}", inertia * 10_000_000.);
                // debug!("Friction: {}", _friction);
                return Some(inertia);
            }

            debug!("Failed to invert matrix for inertia identification.");

            None
        }
    }
}

// const J: f32 = 0.000_035_5; // Rotor Moment of inertia (kg*m^2)
// const KT: f32 = 0.45; // Torque constant (Nm/A)
// const KT: f32 = 0.0; // Torque constant (Nm/A)
const W_0: f32 = 100.; // bandwidth of the observer (rad/s)
// const W_0: f32 = 1_000.; // bandwidth of the observer (rad/s)
const L1_CONTINUOUS: f32 = 2. * 1. * W_0;
const L2_CONTINUOUS: f32 = W_0 * W_0;

const L1_ESO: f32 = 3. * W_0;
const L2_ESO: f32 = 3. * W_0 * W_0;
const L3_ESO: f32 = W_0 * W_0 * W_0;

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub async fn update_luenberger_observer(&mut self, t_us: u64, commanded_torque: f32) -> f32 {
        let dir = self.sensor_direction.multiplier();
        // let dir = 1.;

        let _ = self.encoder.update(t_us).await;

        let measured_angle = self.encoder.get_angle() * dir;
        // let measured_angle = self.encoder.get_angle();
        // debug!("Measured angle: {}", measured_angle);

        if self.prev_t_us == 0 {
            self.prev_t_us = t_us;
            return measured_angle; // Return the measured angle on the first call
        }

        // debug!("measured_angle: {}", measured_angle);
        // let dt = (t_us - self.prev_t_us) as f32 * 1e-6; // Convert microseconds to seconds
        let dt_us = t_us.saturating_sub(self.prev_t_us);
        if dt_us == 0 || dt_us > 20_000 {
            // debug!("Invalid dt_us: {}. Resetting prev_t_us.", dt_us);
            self.prev_t_us = t_us;
            // return measured_angle;
            panic!()
        }
        let dt = dt_us as f32 * 1e-6; // Convert microseconds to seconds
        // debug!("dt: {} seconds", dt);

        let dt = dt_us as f32 * 1e-6;
        if !(dt > 0.0 && dt.is_finite()) {
            // debug!("Invalid dt: {}. Resetting prev_t_us.", dt);
            self.prev_t_us = t_us;
            // return measured_angle;
            panic!()
        }

        self.prev_t_us = t_us;

        let j = self.state_observer.rotor_inertia();
        let kt = self.state_observer.torque_constant();

        #[cfg(feature = "nope")]
        {
            // 1. Dynamically calculate the A and B matrices based on actual elapsed time
            let a = SMatrix::<f32, 2, 2>::new(1.0, dt, 0.0, 1.0);
            let b = SMatrix::<f32, 2, 1>::new((kt * dt * dt) / (2.0 * j), (kt * dt) / j);
            // let b = SMatrix::<f32, 2, 1>::zeros();

            // 2. Discretize the L gains. Continuous gain must be multiplied by dt
            let l = SMatrix::<f32, 2, 1>::new(L1_CONTINUOUS * dt, L2_CONTINUOUS * dt);

            // let p = libm::expf(-W_0 * dt);
            // let l1 = 2. * (1. - p);
            // let l2 = ((1. - p) * (1. - p)) / dt;
            // let l = SMatrix::<f32, 2, 1>::new(l1, l2);

            // C and D matrices remain constant
            let c = SMatrix::<f32, 1, 2>::new(1.0, 0.0);
            let d = SMatrix::<f32, 1, 1>::new(0.0);

            // 3. Inject the new parameters into the observer
            self.state_observer
                .set_params(LuenbergerParam::new(a, b, c, d, l));
        }

        {
            // 1. Dynamically calculate the A and B matrices based on actual elapsed time
            #[rustfmt::skip]
            let a = SMatrix::<f32, 3, 3>::new(
                1.0, dt, 0.5 * dt * dt,
                0.0, 1.0, dt,
                0.0, 0.0, 1.0
            );

            let b0 = self.state_observer.torque_constant
                / (self.state_observer.rotor_inertia * self.motor.phase_resistance.unwrap());

            #[rustfmt::skip]
            let b = SMatrix::<f32, 3, 1>::new(
                0.5 * b0 * dt * dt,
                b0 * dt,
                0.,
            );

            // C and D matrices remain constant
            let c = SMatrix::<f32, 1, 3>::new(1.0, 0.0, 0.0);
            let d = SMatrix::<f32, 1, 1>::new(0.0);

            let l = SMatrix::<f32, 3, 1>::new(L1_ESO * dt, L2_ESO * dt, L3_ESO * dt);

            // 3. Inject the new parameters into the observer
            self.state_observer
                .set_params(LuenbergerParam::new(a, b, c, d, l));
        }

        // 4. Run the observer step
        // let input = SVector::<f32, 1>::new(-commanded_torque * dir);
        let input = SVector::<f32, 1>::new(commanded_torque);
        let measurement = SVector::<f32, 1>::new(measured_angle); // (Assume unwrapped)

        // debug!("input: {:?}", input.as_slice());
        // debug!("measurement: {:?}", measurement.as_slice());

        let state = self.state_observer.update(&input, &measurement);

        // let mech_angle = state[0];
        // let mech_velocity = state[1];

        let mech_angle = state[0] * dir;
        let mech_velocity = state[1] * dir;

        // Convert mechanical angle to electrical angle
        let elec_angle = (mech_angle * self.motor.pole_pairs as f32) % _2PI;

        // self.encoder.debug_force_set_angle_velocity(
        //     // mech_angle * self.sensor_direction.multiplier(),
        //     // mech_velocity * self.sensor_direction.multiplier(),
        //     mech_angle,
        //     mech_velocity,
        // );

        elec_angle
    }
}

/// Parameters for the Luenberger Observer.
/// This mirrors `luenberger_param.hpp` and stores the system/gain matrices.
///
/// Generics:
/// - `T`: Floating-point type (e.g., `f32` or `f64`).
/// - `S`: State dimension.
/// - `I`: Input dimension.
/// - `O`: Output dimension.
#[derive(Debug, Clone, PartialEq)]
pub struct LuenbergerParam<T: RealField, const S: usize, const I: usize, const O: usize> {
    /// State transition matrix (A)
    pub a: SMatrix<T, S, S>,
    /// Input matrix (B)
    pub b: SMatrix<T, S, I>,
    /// Output matrix (C)
    pub c: SMatrix<T, O, S>,
    /// Feedforward matrix (D)
    pub d: SMatrix<T, O, I>,
    /// Observer gain matrix (L)
    pub l: SMatrix<T, S, O>,
}

impl<T: RealField, const S: usize, const I: usize, const O: usize> LuenbergerParam<T, S, I, O> {
    /// Creates a new Luenberger observer parameter set.
    pub fn new(
        a: SMatrix<T, S, S>,
        b: SMatrix<T, S, I>,
        c: SMatrix<T, O, S>,
        d: SMatrix<T, O, I>,
        l: SMatrix<T, S, O>,
    ) -> Self {
        Self { a, b, c, d, l }
    }
}

/// Standard Luenberger State Observer.
/// This mirrors `luenberger.hpp` / `luenberger.cpp`.
///
/// Generics:
/// - `T`: Floating-point type (e.g., `f32` or `f64`).
/// - `S`: State dimension.
/// - `I`: Input dimension.
/// - `O`: Output dimension.
#[derive(Debug, Clone)]
pub struct LuenbergerObserver<T: RealField, const S: usize, const I: usize, const O: usize> {
    /// Observer parameters (system matrices and gain)
    params: LuenbergerParam<T, S, I, O>,
    /// Current state estimate (\hat{x})
    state: SVector<T, S>,

    rotor_inertia: T,
    torque_constant: T,
    // pub l1_continuous: T,
    // pub l2_continuous: T,
}

impl<T: RealField, const S: usize, const I: usize, const O: usize> LuenbergerObserver<T, S, I, O> {
    /// Initializes a new Luenberger observer with the given parameters and initial state.
    pub fn new(
        params: LuenbergerParam<T, S, I, O>,
        initial_state: SVector<T, S>,

        rotor_inertia: T,
        torque_constant: T,
        // l1_continuous: T,
        // l2_continuous: T,
    ) -> Self {
        Self {
            params,
            state: initial_state,
            rotor_inertia,
            torque_constant,
            // l1_continuous,
            // l2_continuous,
        }
    }

    /// Performs the predict-and-update step in a single iteration.
    ///
    /// Tracks the state using the standard discrete Luenberger equation:
    /// x_{k+1} = A * x_k + B * u_k + L * (y_k - C * x_k - D * u_k)
    pub fn update(&mut self, input: &SVector<T, I>, measurement: &SVector<T, O>) -> &SVector<T, S> {
        // Estimated output: \hat{y} = C * \hat{x} + D * u
        let y_hat = &self.params.c * &self.state + &self.params.d * input;

        // Output error (innovation): e = y - \hat{y}
        let error = measurement - y_hat;

        // State update: \hat{x}_{new} = A * \hat{x} + B * u + L * e
        self.state = &self.params.a * &self.state + &self.params.b * input + &self.params.l * error;

        &self.state
    }

    pub fn get_angle_vel(&self) -> (T, T) {
        (self.state[0].clone(), self.state[1].clone())
    }

    /// Returns a reference to the current state estimate.
    pub fn state(&self) -> &SVector<T, S> {
        &self.state
    }

    /// Overrides the current state estimate manually.
    pub fn set_state(&mut self, state: SVector<T, S>) {
        self.state = state;
    }

    /// Returns a reference to the observer parameters.
    pub fn params(&self) -> &LuenbergerParam<T, S, I, O> {
        &self.params
    }

    /// Updates the observer parameters dynamically (e.g., for Gain Scheduling).
    pub fn set_params(&mut self, params: LuenbergerParam<T, S, I, O>) {
        self.params = params;
    }

    pub fn rotor_inertia(&self) -> &T {
        &self.rotor_inertia
    }

    pub fn set_rotor_inertia(&mut self, rotor_inertia: T) {
        self.rotor_inertia = rotor_inertia;
    }

    pub fn torque_constant(&self) -> &T {
        &self.torque_constant
    }

    pub fn set_torque_constant(&mut self, torque_constant: T) {
        self.torque_constant = torque_constant;
    }
}
