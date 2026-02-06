use as5600::As5600;

use crate::{
    hardware::mt_6701::MT6701,
    simplefoc::{
        bldc::BLDCMotor,
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

#[derive(defmt::Format)]
pub enum FOCStatus {
    // Motor is not yet initialized
    MotorUninitialized = 0x00,
    // Motor intiialization is in progress
    MotorInitializing = 0x01,
    // Motor is initialized, but not calibrated (open loop possible)
    MotorUncalibrated = 0x02,
    // Motor calibration in progress
    MotorCalibrating = 0x03,
    // Motor is initialized and calibrated (closed loop possible)
    MotorReady = 0x04,
    // Motor is in error state (recoverable, e.g. overcurrent protection active)
    MotorError = 0x08,
    // Motor calibration failed (possibly recoverable)
    MotorCalibFailed = 0x0E,
    // Motor initialization failed (not recoverable)
    MotorInitFailed = 0x0F,
}

pub struct SimpleFOC<'a> {
    // encoder:
    //     As5600<embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>>,
    encoder:
        MT6701<embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>>,
    pwm_driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

    pub motor_status: FOCStatus,

    pub motor: BLDCMotor,

    pub phase_v: PhaseVoltages,

    pub sensor_direction: SensorDirection,
    pub sensor_offset: f32,
    pub zero_electric_angle: f32,

    pub torque_controller: TorqueControlType,

    pub pid_current_q: PIDController,
    pub pid_current_d: PIDController,

    pub lpf_current_q: LowPassFilter,
    pub lpf_current_d: LowPassFilter,

    pub pid_velocity: PIDController,
    pub pid_angle: PIDController,

    pub lpf_velocity: LowPassFilter,
    pub lpf_angle: LowPassFilter,
}

/// new, control
impl<'a> SimpleFOC<'a> {
    pub fn new(
        // encoder: As5600<
        //     embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>,
        // >,
        encoder: MT6701<
            embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>,
        >,
        driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

        motor: BLDCMotor,
    ) -> Self {
        const PID_CURRENT_KP: f32 = 3.;
        const PID_CURRENT_KI: f32 = 300.;
        const PID_CURRENT_KD: f32 = 0.;
        const PID_CURRENT_RAMP: f32 = 0.;
        const PID_CURRENT_LIMIT: f32 = 12.0;

        const CURR_LPF_TF: f32 = 0.005;

        const PID_VELOCITY_KP: f32 = 0.5;
        const PID_VELOCITY_KI: f32 = 10.0;
        const PID_VELOCITY_KD: f32 = 0.0;
        const PID_VELOCITY_RAMP: f32 = 1000.0;
        const PID_VELOCITY_LIMIT: f32 = 12.0;

        const PID_ANGLE_KP: f32 = 20.0;
        const PID_ANGLE_LIMIT: f32 = 20.0;

        const VEL_LPF_TF: f32 = 0.005;

        SimpleFOC {
            encoder,
            pwm_driver: driver,
            motor,

            motor_status: FOCStatus::MotorUninitialized,

            sensor_direction: SensorDirection::Unknown,
            sensor_offset: 0.0,
            zero_electric_angle: 0.0,

            torque_controller: TorqueControlType::Voltage,

            phase_v: PhaseVoltages::default(),

            pid_current_q: PIDController::new(
                PID_CURRENT_KP,
                PID_CURRENT_KI,
                PID_CURRENT_KD,
                PID_CURRENT_RAMP,
                PID_CURRENT_LIMIT,
            ),
            pid_current_d: PIDController::new(
                PID_CURRENT_KP,
                PID_CURRENT_KI,
                PID_CURRENT_KD,
                PID_CURRENT_RAMP,
                PID_CURRENT_LIMIT,
            ),

            lpf_current_q: LowPassFilter::new(CURR_LPF_TF),
            lpf_current_d: LowPassFilter::new(CURR_LPF_TF),

            pid_velocity: PIDController::new(
                PID_VELOCITY_KP,
                PID_VELOCITY_KI,
                PID_VELOCITY_KD,
                PID_VELOCITY_RAMP,
                PID_VELOCITY_LIMIT,
            ),

            pid_angle: PIDController::new(PID_ANGLE_KP, 0.0, 0.0, 0.0, PID_ANGLE_LIMIT),

            lpf_velocity: LowPassFilter::new(VEL_LPF_TF),
            lpf_angle: LowPassFilter::new(0.),
        }
    }

    pub fn enable(&self) {
        unimplemented!()
    }

    pub fn set_position_target(&self, position: ()) {
        unimplemented!()
    }

    // pub fn set_torque(&self, torque: ()) {
    //     unimplemented!()
    // }

    pub fn set_acceleration(&self, acceleration: ()) {
        unimplemented!()
    }

    pub fn get_position_actual(&self) -> () {
        unimplemented!()
    }

    pub fn get_position_requested(&self) -> f32 {
        self.motor.target_shaft_angle
    }
}

/// update, internal
impl<'a> SimpleFOC<'a> {
    pub fn init_foc(&mut self) {
        self.pid_current_q.limit = self.motor.limit_voltage;
        self.pid_current_d.limit = self.motor.limit_voltage;

        // needs phase resistance set
        self.pid_velocity.limit = self.motor.limit_current;
        self.pid_angle.limit = self.motor.limit_velocity;

        self.motor_status = FOCStatus::MotorCalibrating;

        // align motor if necessary
        // alignment necessary for encoders!
        // sensor and motor alignment - can be skipped
        // by setting motor.sensor_direction and motor.zero_electric_angle
        self.align_sensor();
        // self.motor.shaft_angle

        //
    }

    fn align_sensor(&mut self) {
        if self.sensor_direction == SensorDirection::Unknown {
            // find natural direction
            // move one electrical revolution forward

            for i in 0..500 {
                let angle = crate::simplefoc::types::_3PI_2
                    + crate::simplefoc::types::_2PI * (i as f32) / 500.0;
                self.set_phase_voltages(self.motor.voltage_sensor_align, 0., angle);
            }

            //
        }
        unimplemented!()
    }

    /// main loop
    pub fn update_foc(&mut self) {
        let electrical_angle = self.get_electrical_angle();

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // unimplemented!()
            }
            _ => {
                unimplemented!()
            }
        }

        let shaft_angle = self.get_shaft_angle();

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = self.motor.shaft_velocity
            / (self.motor.motor_kv * super::types::_SQRT3)
            / super::types::_RPM_TO_RADS;

        // estimate the motor current if phase reistance available and current_sense not available
        self.motor.current.q = (self.motor.voltage.q - voltage_bemf) / self.motor.phase_resistance;

        // set current
        // shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
        // shaft_velocity_sp = _constrain(shaft_velocity_sp,-velocity_limit, velocity_limit);
        self.motor.target_shaft_velocity = self.motor.feed_forward_velocity
            + self
                .pid_angle
                .update(self.motor.target_shaft_angle - shaft_angle);

        self.motor.target_shaft_velocity = self
            .motor
            .target_shaft_velocity
            .clamp(-self.motor.limit_velocity, self.motor.limit_velocity);

        // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
        // current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control

        self.motor.target_current = self
            .pid_velocity
            .update(self.motor.target_shaft_velocity - self.motor.shaft_velocity);

        if self.torque_controller == TorqueControlType::Voltage {
            self.motor.voltage.q = (self.motor.target_current * self.motor.phase_resistance
                + voltage_bemf)
                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

            // voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);

            match self.motor.phase_inductance {
                Some(phase_inductance) => {
                    self.motor.voltage.d = (-self.motor.target_current
                        * self.motor.shaft_velocity
                        * (self.motor.pole_pairs as f32)
                        * phase_inductance)
                        .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                }
                None => {
                    self.motor.voltage.d = 0.0;
                }
            }
        }

        //
    }

    // Method using FOC to set Uq and Ud to the motor at the optimal angle
    // Function implementing Space Vector PWM and Sine PWM algorithms
    //
    // Function using sine approximation
    // regular sin + cos ~300us    (no memory usage)
    // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
    fn set_phase_voltages(&mut self, uq: f32, ud: f32, angle_el: f32) {
        // Sinusoidal PWM modulation
        // Inverse Park + Clarke transformation
        let sa = libm::sinf(angle_el);
        let ca = libm::cosf(angle_el);

        // // Inverse park transform
        let u_alpha = ca * ud - sa * uq; // -sin(angle) * Uq;
        let u_beta = sa * ud + ca * uq; //  cos(angle) * Uq;

        // // Clarke transform
        // Ua = Ualpha;
        // Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
        // Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

        // Clarke transform
        self.phase_v.a = u_alpha;
        self.phase_v.b = -0.5 * u_alpha + super::types::_SQRT3_2 * u_beta;
        self.phase_v.c = -0.5 * u_alpha - super::types::_SQRT3_2 * u_beta;

        let center = self.pwm_driver.voltage_limit / 2.0;

        // if (!modulation_centered)

        self.phase_v.a += center;
        self.phase_v.b += center;
        self.phase_v.c += center;

        self.pwm_driver
            .set_duty_cycles_f32(self.phase_v.a, self.phase_v.b, self.phase_v.c);
    }

    /// ignore full rotations for now
    fn get_shaft_angle(&mut self) -> f32 {
        // let Ok(angle) = self.encoder.get_angle() else {
        //     self.motor_status = FOCStatus::MotorError;
        //     // return 0;
        //     panic!()
        // };
        let angle = self.encoder.get_angle();

        // convert from 12 bit to float
        let angle = (angle as f32) * 2.0 * core::f32::consts::PI / 4096.0;

        let angle = self.lpf_angle.filter(angle);

        // self.motor.shaft_angle = angle;

        angle
    }

    fn get_electrical_angle(&mut self) -> f32 {
        // _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
        // Self::normalize_angle()

        let dir = if self.sensor_direction == SensorDirection::Normal {
            1.0
        } else if self.sensor_direction == SensorDirection::Inverted {
            -1.0
        } else {
            0.0
        };

        let angle =
            dir * self.motor.pole_pairs as f32 * self.get_shaft_angle() - self.zero_electric_angle;

        Self::normalize_angle(angle)
    }
}

/// helpers
impl<'a> SimpleFOC<'a> {
    /// normalizing radian angle to [0,2PI]
    fn normalize_angle(angle: f32) -> f32 {
        let mut angle = angle % (2.0 * core::f32::consts::PI);
        if angle >= 0.0 {
            angle
        } else {
            angle + 2.0 * core::f32::consts::PI
        }
    }
}
