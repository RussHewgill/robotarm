use as5600::As5600;

use crate::simplefoc::{
    bldc::BLDCMotor,
    lowpass::LowPassFilter,
    pid::PIDController,
    types::{SensorDirection, TorqueControlType},
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
    encoder:
        As5600<embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>>,
    driver: crate::hardware::pwm_driver::PWMDriver<'a>,

    pub motor_status: FOCStatus,

    pub motor: BLDCMotor,

    pub sensor_direction: SensorDirection,
    pub sensor_offset: f32,
    pub zero_electric_angle: f32,

    pub torque_controller: TorqueControlType,

    pub phase_voltages: [f32; 3],

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
        encoder: As5600<
            embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>,
        >,
        driver: crate::hardware::pwm_driver::PWMDriver<'a>,

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
            driver,
            motor,

            motor_status: FOCStatus::MotorUninitialized,

            sensor_direction: SensorDirection::Unknown,
            sensor_offset: 0.0,
            zero_electric_angle: 0.0,

            torque_controller: TorqueControlType::Voltage,

            phase_voltages: [0.0; 3],

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

    pub fn set_position(&self, position: ()) {
        unimplemented!()
    }

    pub fn set_torque(&self, torque: ()) {
        unimplemented!()
    }

    pub fn set_acceleration(&self, acceleration: ()) {
        unimplemented!()
    }

    pub fn get_position_actual(&self) -> () {
        unimplemented!()
    }

    pub fn get_position_requested(&self) -> () {
        unimplemented!()
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
        unimplemented!()
    }

    pub fn update_foc(&mut self) {
        let electrical_angle = self.get_electrical_angle();

        unimplemented!()
    }

    /// ignore full rotations for now
    fn get_shaft_angle(&mut self) -> f32 {
        let Ok(angle) = self.encoder.angle() else {
            self.motor_status = FOCStatus::MotorError;
            // return 0;
            panic!()
        };

        // convert from 12 bit to float
        let angle = (angle as f32) * 2.0 * core::f32::consts::PI / 4096.0;

        let angle = self.lpf_angle.filter(angle);

        self.motor.shaft_angle = angle;

        angle
    }

    fn get_electrical_angle(&mut self) -> f32 {
        // _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
        // Self::normalize_angle()
        unimplemented!()
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
