use defmt::debug;

use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::{RealField, SMatrix, SVector};

use crate::{
    hardware::{current_sensor::CurrentSensor, encoder_sensor::EncoderSensor},
    simplefoc::{
        adrc::{Adrc, TrackingDifferentiator},
        foc_types::SimpleFOC,
        lowpass::LowPassFilter,
        types::_2PI,
    },
};

pub struct MotorADRC {
    adrc: Adrc,

    rotor_inertia: f32,
    torque_constant: f32,
    phase_resistance: f32,
    b0: f32,
    disturbance_lpf: crate::simplefoc::lowpass::LowPassFilter,
    velocity_tracker: TrackingDifferentiator,
    prev_output: f32,
    // test_luenberger: crate::simplefoc::luenberger::LuenbergerObserver<f32, 3, 1, 1>,
    next_debug: u64,
}

impl MotorADRC {
    pub fn test_adrc(&mut self) {
        let mut eso = self.adrc.eso.clone();

        let commanded_torque = 0.0;
        let mut measured_angle = 0.0;

        #[cfg(feature = "nope")]
        for i in 0..10 {
            let setpoint = 0.0;
            let measurement = measured_angle + 0.01 * (i as f32); // Simulate a changing measurement
            let dt = 0.001;

            let _ = eso.update(measurement, commanded_torque, dt);

            debug!(
                "State 0: angle: {}, velocity: {}, disturbance: {}",
                eso.state[0], eso.state[1], eso.state[2]
            );

            // let output = self.adrc.update(setpoint, measurement, dt);
            // debug!(
            //     "State: angle: {}, velocity: {}, disturbance: {}, output: {}",
            //     self.adrc.eso.z[0], self.adrc.eso.z[1], self.adrc.eso.z[2], output
            // );

            let test_state = {
                {
                    // 1. Dynamically calculate the A and B matrices based on actual elapsed time
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

                    // let l = SMatrix::<f32, 3, 1>::new(L1_ESO * dt, L2_ESO * dt, L3_ESO * dt);
                    let l = SMatrix::<f32, 3, 1>::new(
                        self.adrc.eso.beta[0] * dt,
                        self.adrc.eso.beta[1] * dt,
                        self.adrc.eso.beta[2] * dt,
                    );

                    // debug!("test A: {:?}", a.as_slice());
                    // debug!("test B: {:?}", b.as_slice());
                    // // debug!("test C: {:?}", c);
                    // // debug!("test D: {:?}", d);
                    // debug!("test L: {:?}", l.as_slice());

                    // 3. Inject the new parameters into the observer
                    self.test_luenberger.set_params(
                        crate::simplefoc::luenberger::LuenbergerParam::new(a, b, c, d, l),
                    );
                }

                let input = SVector::<f32, 1>::new(commanded_torque);
                let measurement = SVector::<f32, 1>::new(measurement);

                self.test_luenberger.update(&input, &measurement)
            };
            // self.state_observer.adrc.eso.state = *test_state;
            debug!(
                "State 1: angle: {}, velocity: {}, disturbance: {}",
                test_state[0], test_state[1], test_state[2]
            );
        }

        // unimplemented!()
    }
}

impl MotorADRC {
    pub fn new(
        // adrc: Adrc,
        rotor_inertia: f32,
        torque_constant: f32,
        phase_resistance: f32,
        disturbance_lpf_time: f32,
    ) -> Self {
        use crate::simplefoc::algorithms::adrc::{
            ExtendedStateObserver, NonlinearStateErrorFeedback, TrackingDifferentiator,
        };

        let speed_factor = 500.0;
        let step_size = 0.0001;
        let td = TrackingDifferentiator::new(speed_factor, step_size);

        let w0 = 100.0;
        let b0 = torque_constant / (rotor_inertia * phase_resistance);
        debug!("b0: {}", b0);

        // let b0 = b0 * 3.0;

        let eso = ExtendedStateObserver::from_bandwidth(w0, b0);

        // let wc = w0 / 3.;
        // let wc = w0 / 5.;
        // let wc = w0 / 4.;
        // let wc = w0 / 10.;

        let wc = 100.;

        let beta1 = wc * wc;
        let beta2 = 2.0 * wc;
        // let beta2 = beta2 * 1.2;
        let nlsef = NonlinearStateErrorFeedback::linear(beta1, beta2);

        let adrc = Adrc::new(td, eso, nlsef, b0);

        let disturbance_lpf = LowPassFilter::new(disturbance_lpf_time);

        let velocity_tracker = TrackingDifferentiator::new(100.0, 0.0001);

        Self {
            adrc,
            rotor_inertia,
            torque_constant,
            phase_resistance,
            b0,
            disturbance_lpf,
            velocity_tracker,
            prev_output: 0.0,
            // test_luenberger,
            next_debug: 0,
        }
    }

    pub fn get_state_angle(&self) -> f32 {
        self.adrc.eso.state[0]
    }

    pub fn get_state_velocity(&self) -> f32 {
        self.adrc.eso.state[1]
    }

    pub fn get_state_disturbance(&self) -> f32 {
        self.adrc.eso.state[2]
    }

    pub fn set_rotor_inertia(&mut self, rotor_inertia: f32) {
        // self.rotor_inertia = rotor_inertia;
        // self.b0 = self.torque_constant / (self.rotor_inertia * self.b0);
        *self = Self::new(
            rotor_inertia,
            self.torque_constant,
            self.phase_resistance,
            self.disturbance_lpf.tf,
        );
    }

    pub fn get_prev_output(&self) -> f32 {
        self.prev_output
    }

    pub fn reset(&mut self) {
        self.adrc.eso.reset(SVector::<f32, 3>::new(0.0, 0.0, 0.0));
        self.prev_output = 0.0;
    }
}

/// update
impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    // #[cfg(feature = "nope")]
    pub async fn update_adrc(
        &mut self,
        t_us: u64,
        // commanded_torque: f32,
        commanded_torque: f32,
    ) {
        let dir = self.sensor_direction.multiplier();

        let _ = self.encoder.update(t_us).await;
        let measured_angle = self.encoder.get_angle();

        if self.prev_t_us == 0 {
            self.prev_t_us = t_us;
            // return (measured_angle, 0.0); // Return the measured angle on the first call
            return;
        }

        let dt_us = t_us.saturating_sub(self.prev_t_us);
        if dt_us == 0 || dt_us > 20_000 {
            // self.prev_t_us = t_us;
            // return measured_angle;
            panic!()
        }
        let dt = dt_us as f32 * 1e-6; // Convert microseconds to seconds

        let dt = dt_us as f32 * 1e-6;
        if !(dt > 0.0 && dt.is_finite()) {
            // self.prev_t_us = t_us;
            // return measured_angle;
            panic!()
        }

        self.prev_t_us = t_us;

        // let state = self.state_observer.adrc.state_estimate();
        // debug!(
        //     "ADRC state: angle: {}, velocity: {}, disturbance: {}",
        //     state[0], state[1], state[2]
        // );

        let measured_angle = measured_angle * dir;
        let commanded_torque = commanded_torque * dir;
        // let commanded_torque = 0.0;

        // if self.enabled && t_us >= self.state_observer.next_debug {
        //     debug!("pos: {}, vel: {}", pos, vel);
        // }

        // self.state_observer.adrc.eso.state[0] = pos;
        // self.state_observer.adrc.eso.state[1] = vel;

        let dbg = self.enabled && t_us >= self.state_observer.next_debug;

        // self.state_observer.prev_output =
        //     self.state_observer
        //         .adrc
        //         .update(commanded_torque, measured_angle, dt, dbg);

        // self.state_observer
        //     .adrc
        //     .eso
        //     .update(measured_angle, commanded_torque, dt);

        // #[cfg(feature = "nope")]
        match self.motion_control {
            robotarm_protocol::MotionControlType::Torque => {
                let (pos, vel) = self
                    .state_observer
                    .velocity_tracker
                    .update(measured_angle, dt);

                let state = self
                    .state_observer
                    .adrc
                    .eso
                    .update(pos, commanded_torque, dt);
            }
            robotarm_protocol::MotionControlType::Velocity => {
                let (pos, vel) = self
                    .state_observer
                    .velocity_tracker
                    .update(measured_angle, dt);

                self.state_observer.prev_output =
                    self.state_observer
                        .adrc
                        .update(self.motor.target_shaft_velocity, vel, dt, dbg);

                let k = 0.7;
                self.state_observer.prev_output = -k
                    * self
                        .state_observer
                        .disturbance_lpf
                        .filter_with_timestamp(self.state_observer.prev_output, t_us);

                self.motor.target_current += self.state_observer.prev_output;
            }
            robotarm_protocol::MotionControlType::Angle => {
                // let target = self.motor.target_shaft_angle;
                // let wrapped_target = measured_angle
                //     + crate::simplefoc::foc::shortest_angular_delta(target, measured_angle);

                self.state_observer.prev_output = self
                    .state_observer
                    .adrc
                    // .update(wrapped_target, measured_angle, dt, dbg);
                    .update(self.motor.target_shaft_angle, measured_angle, dt, dbg);

                // let k = 1.0;
                // let u = -k
                //     * self
                //         .state_observer
                //         .disturbance_lpf
                //         .filter_with_timestamp(self.state_observer.prev_output, t_us);

                let u = self.state_observer.prev_output;

                self.motor.target_current = u;
            }
            robotarm_protocol::MotionControlType::VelocityOpenLoop => todo!(),
            robotarm_protocol::MotionControlType::AngleOpenLoop => todo!(),
        }

        // let k = 0.7;
        // self.state_observer.prev_output = -k
        //     * self
        //         .state_observer
        //         .disturbance_lpf
        //         .filter_with_timestamp(self.state_observer.adrc.eso.state[2], t_us);

        let state = self.state_observer.adrc.eso.state;
        if state[0].is_nan() || state[1].is_nan() || state[2].is_nan() {
            debug!(
                "ADRC ESO: NaN detected in state estimate: {:?}",
                state.as_slice()
            );
            panic!()
        }

        // #[cfg(feature = "nope")]
        if self.enabled && t_us >= self.state_observer.next_debug {
            // debug!(
            //     "ADRC: commanded_torque: {}, measured_angle: {}, state: {:?}",
            //     commanded_torque,
            //     measured_angle,
            //     state.as_slice(),
            // );
            // debug!("Target angle: {}", self.motor.target_shaft_angle);
            // debug!("Shaft angle:  {}", measured_angle);
            // debug!("prev_output: {}", self.state_observer.prev_output);

            // debug!("");
            self.state_observer.next_debug = t_us + 50_000;
        }
    }
}

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    //
}
