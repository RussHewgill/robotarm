use defmt::debug;

use nalgebra::{RealField, SMatrix, SVector};

use crate::{
    hardware::{current_sensor::CurrentSensor, encoder_sensor::EncoderSensor},
    simplefoc::{foc_types::SimpleFOC, types::_2PI},
};

/// https://github.com/JRL-CARI-CNR-UNIBS/state_observers

const J: f32 = 0.000_035_5; // Rotor Moment of inertia (kg*m^2)
const KT: f32 = 0.45; // Torque constant (Nm/A)
// const KT: f32 = 0.0; // Torque constant (Nm/A)
const W_0: f32 = 200.; // bandwidth of the observer (rad/s)
const L1_CONTINUOUS: f32 = 2. * 1. * W_0; // Continuous gain for the first state
const L2_CONTINUOUS: f32 = W_0 * W_0; // Continuous gain for the second state

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    #[cfg(feature = "nope")]
    pub async fn update_luenberger_observer(&mut self, t_us: u64, commanded_torque: f32) -> f32 {
        let _ = self.encoder.update(t_us).await;
        let measured_angle = self.encoder.get_angle();

        let dt = (t_us - self.prev_t_us) as f32 * 1e-6; // Convert microseconds to seconds

        let input = SVector::<f32, 1>::new(commanded_torque);

        // Handle encoder wrapping (ensure shortest path for the error calculation)
        // If the encoder rolls over from 2PI to 0, you must unwrap it before passing
        // to the observer, or the observer will think the motor spun backwards instantly.
        let measurement = SVector::<f32, 1>::new(measured_angle); // (Assume unwrapped here)

        let state = self.state_observer.update(&input, &measurement);

        let mech_angle = state[0];
        let mech_velocity = state[1];

        // Convert mechanical angle to electrical angle for the FOC Clarke/Park transforms
        let elec_angle = (mech_angle * self.motor.pole_pairs as f32) % _2PI;

        self.encoder
            .debug_force_set_angle_velocity(mech_angle, mech_velocity);

        // (elec_angle, mech_velocity)
        elec_angle
    }

    // #[cfg(feature = "nope")]
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

        // 1. Dynamically calculate the A and B matrices based on actual elapsed time
        let a = SMatrix::<f32, 2, 2>::new(1.0, dt, 0.0, 1.0);
        let b = SMatrix::<f32, 2, 1>::new((KT * dt * dt) / (2.0 * J), (KT * dt) / J);

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

        // 4. Run the observer step
        let input = SVector::<f32, 1>::new(-commanded_torque * dir);
        // let input = SVector::<f32, 1>::new(commanded_torque);
        let measurement = SVector::<f32, 1>::new(measured_angle); // (Assume unwrapped)

        // debug!("input: {:?}", input.as_slice());
        // debug!("measurement: {:?}", measurement.as_slice());

        let state = self.state_observer.update(&input, &measurement);

        // let mech_angle = state[0];
        // let mech_velocity = state[1];

        let mech_angle = state[0] * dir;
        let mech_velocity = state[1] * dir;

        // if measured_angle.signum() != mech_angle.signum() {
        //     debug!(
        //         "Sign mismatch: measured_angle: {}, mech_angle: {}",
        //         measured_angle, mech_angle
        //     );
        // }

        // debug!(
        //     "dt: {}, mechanical angle: {}, mechanical velocity: {}",
        //     dt, mech_angle, mech_velocity
        // );

        // debug!(
        //     "measured_angle: {}, mech_angle: {}, mech_velocity: {}",
        //     measured_angle, mech_angle, mech_velocity
        // );

        // if mech_velocity.abs() > 100.0 {
        //     debug!(
        //         "Unrealistic mech_velocity: {}. Resetting prev_t_us.",
        //         mech_velocity
        //     );
        //     panic!()
        // }

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
}

impl<T: RealField, const S: usize, const I: usize, const O: usize> LuenbergerObserver<T, S, I, O> {
    /// Initializes a new Luenberger observer with the given parameters and initial state.
    pub fn new(params: LuenbergerParam<T, S, I, O>, initial_state: SVector<T, S>) -> Self {
        Self {
            params,
            state: initial_state,
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
}
