use defmt::debug;

use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::{RealField, SMatrix, SVector};

use super::extended_state_observer::ExtendedStateObserver;
use super::nlsef::NonlinearStateErrorFeedback;
use super::tracking_differentiator::TrackingDifferentiator;

/// A complete second-order Active Disturbance Rejection Controller.
///
/// ADRC controls a plant of the form `x1' = x2`, `x2' = f(x1, x2, w(t), t) +
/// b0*u`, `y = x1` (e.g. position/velocity, angle/angular rate) without
/// needing an accurate model of `f`: the [`ExtendedStateObserver`] estimates
/// `f` online as a "total disturbance" and the controller cancels it, while
/// the [`TrackingDifferentiator`] provides a smooth reference trajectory to
/// track and the [`NonlinearStateErrorFeedback`] law closes the loop on the
/// resulting tracking error.
///
/// Each control cycle, call [`Adrc::update`] with the current setpoint and
/// plant measurement to get the next control action.
///
/// # Example
///
/// ```
/// use adrc::{Adrc, ExtendedStateObserver, NonlinearStateErrorFeedback, TrackingDifferentiator};
///
/// let td = TrackingDifferentiator::new(20.0, 0.01);
/// let eso = ExtendedStateObserver::from_bandwidth(30.0, 1.0);
/// let nlsef = NonlinearStateErrorFeedback::new(25.0, 10.0, 0.5, 0.25, 0.05);
/// let mut controller = Adrc::new(td, eso, nlsef, 1.0).with_limits(-50.0, 50.0);
///
/// let dt = 0.01;
/// let mut y = 0.0_f32; // plant measurement, updated by your own plant/sensor
/// for _ in 0..100 {
///     let u = controller.update(/* setpoint */ 1.0, y, dt);
///     // Apply `u` to your plant, then update `y` from the new sensor reading.
///     y += dt * u; // toy placeholder plant
/// }
/// ```
#[derive(Clone, Copy, PartialEq)]
pub struct Adrc {
    /// Tracking differentiator generating the smooth reference trajectory.
    pub td: TrackingDifferentiator,
    /// Extended state observer estimating plant state and total disturbance.
    pub eso: ExtendedStateObserver,
    /// Nonlinear error-feedback law producing the baseline control action.
    pub nlsef: NonlinearStateErrorFeedback,
    /// Control-input gain used to scale the disturbance-compensated output
    /// (`u = (u0 - z3) / b0`). Should normally match [`ExtendedStateObserver::b0`].
    pub b0: f32,
    /// Optional `(min, max)` clamp applied to the final control output.
    pub u_limits: Option<(f32, f32)>,
    // last_u: f32,
    last_u: f32,
}

impl Adrc {
    /// Assemble a controller from its three building blocks and a control
    /// input gain `b0`.
    pub fn new(
        td: TrackingDifferentiator,
        eso: ExtendedStateObserver,
        nlsef: NonlinearStateErrorFeedback,
        b0: f32,
    ) -> Self {
        // if b0 == 0.0 {
        //     panic!("ADRC: b0 must be nonzero");
        // }
        Self {
            td,
            eso,
            nlsef,
            b0,
            u_limits: None,
            last_u: 0.0,
        }
    }

    /// Clamp the controller's output to `[min, max]`.
    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.u_limits = Some((min, max));
        self
    }

    /// Run one control step and return the control action `u` to apply to
    /// the plant.
    ///
    /// * `setpoint` - desired reference value for the plant output.
    /// * `measurement` - current plant output (sensor reading), *before*
    ///   this step's control action is applied.
    /// * `dt` - elapsed time since the previous call, in seconds.
    // pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32, debug: bool) -> f32 {
        // 1. Smooth reference trajectory and its derivative.
        let (v1, v2) = self.td.update(setpoint, dt);

        // 2. Update the state/disturbance estimate using the control that
        //    was actually applied last step.
        let state = self.eso.update(measurement, self.last_u, dt);
        // let state = self.eso.update(measurement, setpoint, dt);

        // 3. Track the smooth reference with the estimated state, not the
        //    raw (possibly noisy) measurement.
        let e1 = v1 - state[0];
        let e2 = v2 - state[1];

        // if debug {
        //     debug!(
        //         "e1: {}, e2: {}, v1: {}, v2: {}, state: {:?}",
        //         e1,
        //         e2,
        //         v1,
        //         v2,
        //         state.as_slice()
        //     );
        // }

        // 4. Nonlinear error feedback gives a baseline control action.
        let u0 = self.nlsef.compute(e1, e2);

        // 5. Cancel the estimated total disturbance and scale by the input gain.
        let mut u = (u0 - state[2]) / self.b0;

        // 6. Respect actuator limits, if configured.
        if let Some((min, max)) = self.u_limits {
            u = u.clamp(min, max);
        }

        self.last_u = u;
        u
    }

    /// The ESO's current state estimate `[x1_hat, x2_hat, disturbance_hat]`.
    pub fn state_estimate(&self) -> SVector<f32, 3> {
        self.eso.state
    }

    /// The most recent control action returned by [`Adrc::update`].
    pub fn last_control(&self) -> f32 {
        self.last_u
    }

    /// Reset the TD and ESO internal state (e.g. before starting a new
    /// trajectory), and forget the last applied control.
    pub fn reset(&mut self) {
        self.td.reset(0.0, 0.0);
        self.eso.reset(SVector::zeros());
        self.last_u = 0.0;
    }
}
