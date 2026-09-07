use defmt::debug;

use nalgebra::{RealField, SMatrix, SVector};

use crate::simplefoc::adrc::nlsef::fal;

/// Third-order Extended State Observer for a second-order plant
///
/// ```text
/// x1' = x2
/// x2' = f(x1, x2, w(t), t) + b0 * u
/// y   = x1
/// ```
///
/// where `f(...)` bundles unmodeled dynamics and `w(t)` is an external
/// disturbance. The ESO tracks `x1` and `x2` while simultaneously estimating
/// an extended state `z3`, an online estimate of the "total disturbance"
/// `f(x1, x2, w(t), t)`. This total-disturbance estimate is what lets ADRC
/// cancel model uncertainty and external disturbances without needing an
/// accurate plant model.
///
/// Set [`ExtendedStateObserver::fal_alpha`] to `None` for a **linear** ESO
/// (LESO), which is simpler to tune (e.g. via bandwidth/pole-placement) and
/// is a very strong default, or to `Some((alpha2, alpha3))` for the original
/// **nonlinear** ESO (NESO), which can give a better noise/speed trade-off
/// once tuned.
///
/// S = number of states (3 for a second-order plant)
/// I = number of inputs (1 for a second-order plant)
/// O = number of outputs (1 for a second-order plant)
#[derive(Clone, Copy, PartialEq)]
// pub struct ExtendedStateObserver<const S: usize = 3, const I: usize = 1, const O: usize = 1> {
pub struct ExtendedStateObserver {
    /// Estimated state `[z1 (~x1), z2 (~x2), z3 (~total disturbance)]`.
    // pub state: SVector<f32, S>,
    pub state: SVector<f32, 3>,
    /// Observer correction gains `[beta1, beta2, beta3]`. Larger gains give
    /// faster (but noisier) convergence. For a LESO with observer bandwidth
    /// `wo`, the standard pole-placement choice is
    /// `beta = [3*wo, 3*wo^2, wo^3]`.
    // pub beta: SVector<f32, S>,
    // pub beta: SMatrix<f32, S, O>,
    pub beta: SMatrix<f32, 3, 1>,
    /// Estimate of the plant's control-input gain (`x2' ~= ... + b0 * u`).
    pub b0: f32,
    /// Exponents `(alpha2, alpha3)` for the nonlinear correction terms on
    /// `z2` and `z3`. `None` selects a linear ESO (the `z1` correction term
    /// is always linear).
    pub fal_alpha: Option<(f32, f32)>,
    /// Linear/nonlinear boundary layer width used by [`fal`] when
    /// `fal_alpha` is `Some`. Ignored for a linear ESO.
    pub fal_delta: f32,
    // params: LuenbergerParam<S, I, O>,
}

// impl<const S: usize, const I: usize, const O: usize> ExtendedStateObserver<S, I, O> {
impl ExtendedStateObserver {
    /// Construct a linear ESO (LESO) with correction gains `beta` and input
    /// gain `b0`, initialized to all-zero state.
    // pub fn new_linear(beta: SMatrix<f32, S, O>, b0: f32) -> Self {
    pub fn new_linear(beta: SMatrix<f32, 3, 1>, b0: f32) -> Self {
        Self {
            state: SVector::zeros(),
            beta,
            b0,
            fal_alpha: None,
            fal_delta: 0.01,
            // params: LuenbergerParam::new(
            //     SMatrix::<f32, S, S>::identity(),
            //     SMatrix::<f32, S, I>::zeros(),
            //     SMatrix::<f32, O, S>::zeros(),
            //     SMatrix::<f32, O, I>::zeros(),
            //     beta,
            // ),
        }
    }

    /// Construct a linear ESO from an observer bandwidth `wo` using the
    /// standard critical-damping pole placement `beta = [3*wo, 3*wo^2,
    /// wo^3]`, a common and effective tuning shortcut.
    pub fn from_bandwidth(wo: f32, b0: f32) -> Self {
        // let beta = SVector::new(3.0 * wo, 3.0 * wo * wo, wo * wo * wo);
        // let beta = SVector::from_row_slice(&[3.0 * wo, 3.0 * wo * wo, wo * wo * wo]);
        let beta = SMatrix::<f32, 3, 1>::from_row_slice(&[3.0 * wo, 3.0 * wo * wo, wo * wo * wo]);
        Self::new_linear(beta, b0)
    }

    /// Reset the internal state estimate.
    pub fn reset(&mut self, z: SVector<f32, 3>) {
        self.state = z;
    }

    /// A: SMatrix<f32, S, S>
    /// B: SMatrix<f32, S, I>
    /// C: SMatrix<f32, O, S>
    /// D: SMatrix<f32, O, I>
    /// L: SMatrix<f32, S, O>
    #[cfg(feature = "nope")]
    pub fn update(&mut self, measurement: f32, input: f32, dt: f32) -> SVector<f32, 3> {
        #[rustfmt::skip]
        let a = SMatrix::<f32, 3, 3>::new(
            1.0, dt, 0.5 * dt * dt,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0
        );

        #[rustfmt::skip]
        let b = SMatrix::<f32, 3, 1>::new(
            0.5 * self.b0 * dt * dt,
            self.b0 * dt,
            0.,
        );

        // C and D matrices remain constant
        let c = SMatrix::<f32, 1, 3>::new(1.0, 0.0, 0.0);
        let d = SMatrix::<f32, 1, 1>::new(0.0);

        // let l = self.beta * dt;
        let l = SMatrix::<f32, 3, 1>::new(self.beta[0] * dt, self.beta[1] * dt, self.beta[2] * dt);

        // debug!("A: {:?}", a.as_slice());
        // debug!("B: {:?}", b.as_slice());
        // // debug!("C: {:?}", c);
        // // debug!("D: {:?}", d);
        // debug!("L: {:?}", l.as_slice());

        let measurement = SVector::<f32, 1>::new(measurement);
        let input = SVector::<f32, 1>::new(input);

        // Estimated output: \hat{y} = C * \hat{x} + D * u
        let y_hat = &c * &self.state + &d * input;

        // Output error (innovation): e = y - \hat{y}
        let error = measurement - y_hat;

        // State update: \hat{x}_{new} = A * \hat{x} + B * u + L * e
        self.state = &a * &self.state + &b * input + &l * error;

        // debug!("Input: {:?}", input.as_slice());
        // debug!("Measurement: {:?}", measurement.as_slice());

        // debug!("Estimated Output (y_hat): {:?}", y_hat.as_slice());
        // debug!("Output Error (e): {:?}", error.as_slice());
        // debug!(
        //     "Updated State Estimate (x_hat): {:?}",
        //     self.state.as_slice()
        // );

        self.state
    }

    /// Advance the observer by one step of `dt` seconds, given the measured
    /// plant output `y` and the control `u` that was actually applied
    /// (during the *previous* step). Returns the updated state estimate.
    // #[cfg(feature = "nope")]
    pub fn update(&mut self, y: f32, u: f32, dt: f32) -> SVector<f32, 3> {
        let e = self.state[0] - y;

        let (g2, g3) = match self.fal_alpha {
            None => (e, e),
            Some((alpha2, alpha3)) => (
                fal(e, alpha2, self.fal_delta),
                fal(e, alpha3, self.fal_delta),
            ),
        };

        let z1_dot = self.state[1] - self.beta[0] * e;
        let z2_dot = self.state[2] + self.b0 * u - self.beta[1] * g2;
        let z3_dot = -self.beta[2] * g3;

        self.state[0] += dt * z1_dot;
        self.state[1] += dt * z2_dot;
        self.state[2] += dt * z3_dot;

        self.state
    }
}
