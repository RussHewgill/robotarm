use defmt::debug;

/// The `fal` (nonlinear gain) function at the heart of ADRC's nonlinear
/// blocks (ESO correction terms, NLSEF error feedback).
///
/// ```text
///          |e|^alpha * sign(e)      if |e| >  delta
/// fal(e) =
///          e / delta^(1 - alpha)    if |e| <= delta
/// ```
///
/// For `|e| <= delta` the function is linear (with slope `delta^(alpha-1)`),
/// which avoids the infinite gain (and resulting chattering) that a pure
/// power law would have near zero. `alpha` is typically in `(0, 1)`, e.g.
/// `0.5` for the outer term and `0.25` for the inner term of a classic
/// second-order ESO/NLSEF. `delta` is a small positive number, typically a
/// few multiples of the sample time.
#[inline]
pub fn fal(e: f32, alpha: f32, delta: f32) -> f32 {
    // debug_assert!(delta > 0.0, "fal: delta must be > 0");
    let abs_e = libm::fabsf(e);
    if abs_e > delta {
        libm::powf(abs_e, alpha) * e.signum()
    } else {
        e * libm::powf(delta, 1.0 - alpha) / delta
    }
}

/// Combines tracking errors `(e1, e2)` between the desired trajectory and
/// the ESO's state estimate into a baseline control action `u0`, before
/// disturbance-rejection compensation is applied.
///
/// This plays the same role a PD controller would in a classical control
/// loop, but uses the nonlinear [`fal`] gain instead of a fixed linear gain:
/// it applies high gain to small errors (for accuracy and quick settling
/// near the target) and comparatively lower gain to large errors (avoiding
/// large, aggressive control actions), which is generally referred to as
/// "small error, big gain; big error, small gain".
#[derive(Clone, Copy, PartialEq)]
pub struct NonlinearStateErrorFeedback {
    /// Proportional-like gain on the position error `e1`.
    pub beta1: f32,
    /// Derivative-like gain on the velocity error `e2`.
    pub beta2: f32,
    /// Nonlinear exponent applied to `e1` (typically in `(0, 1)`, e.g. `0.5`).
    pub alpha1: f32,
    /// Nonlinear exponent applied to `e2` (typically in `(0, 1)`, e.g. `0.25`,
    /// smaller than `alpha1`).
    pub alpha2: f32,
    /// Linear/nonlinear boundary layer width used by [`fal`].
    pub delta: f32,
}

impl NonlinearStateErrorFeedback {
    /// Construct a nonlinear (fal-based) feedback law.
    pub fn new(beta1: f32, beta2: f32, alpha1: f32, alpha2: f32, delta: f32) -> Self {
        Self {
            beta1,
            beta2,
            alpha1,
            alpha2,
            delta,
        }
    }

    /// Construct a plain linear PD-style feedback law
    /// (`u0 = beta1*e1 + beta2*e2`) by picking exponents of `1.0`. This is a
    /// reasonable, easy-to-tune starting point before moving to a fully
    /// nonlinear law.
    pub fn linear(beta1: f32, beta2: f32) -> Self {
        Self {
            beta1,
            beta2,
            alpha1: 1.0,
            alpha2: 1.0,
            delta: 1.0,
        }
    }

    /// Compute the baseline control action `u0` from the position error `e1`
    /// and velocity error `e2`.
    pub fn compute(&self, e1: f32, e2: f32) -> f32 {
        self.beta1 * fal(e1, self.alpha1, self.delta)
            + self.beta2 * fal(e2, self.alpha2, self.delta)
    }
}
