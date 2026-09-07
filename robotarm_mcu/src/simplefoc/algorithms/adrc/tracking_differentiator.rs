use defmt::debug;

/// A second-order Tracking Differentiator.
///
/// Given a reference input that may be noisy, stepwise, or otherwise
/// non-smooth, the TD produces:
///
/// - `v1`: a smoothed tracking signal that follows the input without the
///   "differential explosion" (huge, noise-amplifying derivatives) that a
///   naive numerical difference would produce, and
/// - `v2`: a well-behaved estimate of `v1`'s derivative.
///
/// This is normally used to turn a (possibly discontinuous) setpoint into a
/// smooth reference trajectory `(v1, v2)` for the rest of the ADRC loop, but
/// it is also useful standalone as a noise-robust differentiator.
#[derive(Clone, Copy, PartialEq)]
pub struct TrackingDifferentiator {
    /// Tracking signal (smoothed reference), i.e. the TD's estimate of the input.
    pub v1: f32,
    /// Derivative of the tracking signal.
    pub v2: f32,
    /// Speed factor: the bound on `v2`'s rate of change. Larger `r` tracks
    /// the input faster but can overshoot and amplify noise.
    pub r: f32,
    /// Filtering step size (`> 0`), typically the loop's sample time `dt` or
    /// a small multiple of it. Larger `h0` gives smoother, more filtered
    /// output at the cost of increased tracking lag.
    pub h0: f32,
}

impl TrackingDifferentiator {
    /// Create a new tracking differentiator, initialized at rest at the origin.
    ///
    /// * `r` - speed factor (bound on acceleration). Larger tracks faster.
    /// * `h0` - filtering step size (`> 0`).
    pub fn new(r: f32, h0: f32) -> Self {
        Self {
            v1: 0.0,
            v2: 0.0,
            r,
            h0,
        }
    }

    /// Reset the internal tracking state.
    pub fn reset(&mut self, v1: f32, v2: f32) {
        self.v1 = v1;
        self.v2 = v2;
    }

    /// Advance the differentiator by one step of `dt` seconds, tracking
    /// towards input `v`. Returns the updated `(v1, v2)`.
    pub fn update(&mut self, v: f32, dt: f32) -> (f32, f32) {
        let a = fhan(self.v1 - v, self.v2, self.r, self.h0);
        self.v1 += dt * self.v2;
        self.v2 += dt * a;
        (self.v1, self.v2)
    }
}

/// Discrete "fastest control synthesis function" from Han Jingqing's ADRC
/// papers.
///
/// For a discrete double integrator
/// `x1[k+1] = x1[k] + h*x2[k]`, `x2[k+1] = x2[k] + h*u[k]`,
/// `fhan(x1, x2, r, h0)` returns (an approximation of) the time-optimal
/// control `u` that drives `(x1, x2)` to `(0, 0)` with `|u| <= r`, using a
/// step size `h0` for the internal discretization (this is normally set
/// equal to, or a small multiple of, the outer loop's sample time and acts
/// as a filtering/smoothing parameter: larger `h0` gives more filtering of
/// noise at the cost of slower, less precise tracking).
///
/// * `x1` - position-like error term the differentiator is driving to zero.
/// * `x2` - velocity-like term (current derivative estimate).
/// * `r`  - "speed factor": bound on the magnitude of the returned control /
///   acceleration. Larger `r` tracks the input faster.
/// * `h0` - filtering step size (`> 0`).
#[allow(clippy::many_single_char_names)]
pub fn fhan(x1: f32, x2: f32, r: f32, h0: f32) -> f32 {
    let d = r * h0 * h0;
    let a0 = h0 * x2;
    let y = x1 + a0;
    let a1 = libm::sqrtf(d * (d + 8.0 * libm::fabsf(y)));
    let a2 = a0 + y.signum() * (a1 - d) / 2.0;
    let sy = ((y + d).signum() - (y - d).signum()) / 2.0;
    let a = (a0 + y - a2) * sy + a2;
    let sa = ((a + d).signum() - (a - d).signum()) / 2.0;

    -r * (a / d) * sa - r * a.signum() * (1.0 - sa)
}
