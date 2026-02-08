use defmt::{error, info};
use embassy_rp::gpio::Output;
use embassy_time::{Instant, Timer};

use crate::{
    hardware::{as5600::AS5600, mt_6701::MT6701},
    simplefoc::{
        bldc::BLDCMotor,
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{MotionControlType, NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
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

pub struct SimpleFOC<'a, I2C: embassy_rp::i2c::Instance> {
    encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    // encoder:
    //     MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    pwm_driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

    enable_pin: Output<'a>,

    pub motor_status: FOCStatus,

    pub enabled: bool,

    pub motor: BLDCMotor,

    pub phase_v: PhaseVoltages,

    pub sensor_direction: SensorDirection,
    pub sensor_offset: f32,
    pub zero_electric_angle: f32,

    pub motion_control: MotionControlType,
    pub torque_controller: TorqueControlType,

    pub pid_current_q: PIDController,
    pub pid_current_d: PIDController,

    // not used except with current sensor
    // pub lpf_current_q: LowPassFilter,
    // pub lpf_current_d: LowPassFilter,
    pub pid_velocity: PIDController,
    pub pid_angle: PIDController,

    pub lpf_velocity: LowPassFilter,
    pub lpf_angle: LowPassFilter,
}

/// new, control
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    pub fn new(
        encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
        // encoder: MT6701<
        //     embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>,
        // >,
        driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

        enable_pin: Output<'a>,

        motor: BLDCMotor,
    ) -> Self {
        const PID_CURRENT_KP: f32 = 3.;
        const PID_CURRENT_KI: f32 = 300.;
        const PID_CURRENT_KD: f32 = 0.;
        const PID_CURRENT_RAMP: f32 = 0.;
        const PID_CURRENT_LIMIT: f32 = 12.0;

        // const CURR_LPF_TF: f32 = 0.005;

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

            enable_pin,

            motor_status: FOCStatus::MotorUninitialized,

            enabled: false,

            sensor_direction: SensorDirection::Unknown,
            sensor_offset: 0.0,
            zero_electric_angle: NOT_SET,

            motion_control: MotionControlType::Angle,
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

            // lpf_current_q: LowPassFilter::new(CURR_LPF_TF),
            // lpf_current_d: LowPassFilter::new(CURR_LPF_TF),
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

    pub fn enable(&mut self) {
        self.enabled = true;
        self.enable_pin.set_high();
    }

    pub fn disable(&mut self) {
        self.enabled = false;
        self.enable_pin.set_low();
        self.set_phase_voltage(0., 0., 0.);
    }

    pub fn set_position_target(&mut self, position: f32) {
        self.motor.target_shaft_angle = position;
    }

    // pub fn set_torque(&self, torque: ()) {
    //     unimplemented!()
    // }

    pub fn set_acceleration(&self, _acceleration: ()) {
        unimplemented!()
    }

    pub fn get_position_actual(&self) -> f32 {
        self.encoder.get_position() as f32
    }

    pub fn get_position_requested(&self) -> f32 {
        self.motor.target_shaft_angle
    }
}

/// update, internal
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    pub fn init(&mut self) {
        // check driver initialized

        self.motor_status = FOCStatus::MotorInitializing;

        if self.motor.limit_voltage > self.pwm_driver.voltage_limit {
            error!(
                "Motor voltage limit {} is higher than driver voltage limit {}, constraining to driver limit",
                self.motor.limit_voltage, self.pwm_driver.voltage_limit
            );
            self.motor.limit_voltage = self.pwm_driver.voltage_limit;
        }

        if self.motor.voltage_sensor_align > self.motor.limit_voltage {
            error!(
                "Motor voltage sensor align {} is higher than motor voltage limit {}, constraining to motor limit",
                self.motor.voltage_sensor_align, self.motor.limit_voltage
            );
            self.motor.voltage_sensor_align = self.motor.limit_voltage;
        }

        // velocity control loop controls current
        self.pid_velocity.limit = self.motor.limit_current;

        self.pid_angle.limit = self.motor.limit_velocity;

        self.motor_status = FOCStatus::MotorReady;
    }

    pub async fn init_foc(&mut self) {
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
        self.align_sensor().await;
        // self.motor.shaft_angle

        //
    }

    async fn align_sensor(&mut self) {
        if self.sensor_direction == SensorDirection::Unknown {
            // find natural direction
            // move one electrical revolution forward

            // for i in 0..500 {
            //     let angle = crate::simplefoc::types::_3PI_2
            //         + crate::simplefoc::types::_2PI * (i as f32) / 500.0;
            //     self.set_phase_voltage(self.motor.voltage_sensor_align, 0., angle);
            // }

            error!("TODO: implement sensor alignment procedure to determine sensor direction");

            // panic!()
            //
        }

        // zero electric angle not known
        if self.zero_electric_angle == NOT_SET {
            // align the electrical phases of the motor and sensor
            // set angle -90(270 = 3PI/2) degrees

            info!("Aligning sensor, rotating motor to known angle...");

            self.enable();

            self.set_phase_voltage(
                self.motor.voltage_sensor_align,
                0.,
                crate::simplefoc::types::_3PI_2,
            );

            info!("Waiting for sensor to align...");

            Timer::after_millis(700).await;

            info!("Sensor aligned, setting zero electric angle...");

            // get the current zero electric angle
            self.zero_electric_angle = 0.;
            let (electrical_angle, _shaft_angle) = self.get_electrical_angle().await;
            self.zero_electric_angle = electrical_angle;

            info!("Zero electric angle set to {}", self.zero_electric_angle);
            info!("Shaft angle at alignment position: {}", _shaft_angle);

            Timer::after_millis(20).await;

            // // stop everything
            self.set_phase_voltage(0., 0., 0.);
            self.disable();
            Timer::after_millis(200).await;
        }
    }

    /// main loop
    pub async fn update_foc(&mut self) {
        // update sensor readings
        if let Err(_e) = self.encoder.update(Instant::now().as_micros()).await {
            error!("Failed to update encoder");
            unimplemented!()
        }

        let (electrical_angle, shaft_angle) = self.get_electrical_angle().await;
        let shaft_velocity = self.get_shaft_velocity();

        if !self.enabled {
            return;
        }

        match self.torque_controller {
            TorqueControlType::Voltage => {
                // nothing to do
            } // _ => { unimplemented!() }
        }

        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        let voltage_bemf = shaft_velocity
            / (self.motor.motor_kv * super::types::_SQRT3)
            / super::types::_RPM_TO_RADS;

        // estimate the motor current if phase reistance available and current_sense not available
        self.motor.current.q = (self.motor.voltage.q - voltage_bemf) / self.motor.phase_resistance;

        // angle control

        match self.motion_control {
            MotionControlType::Torque => {
                if self.torque_controller == TorqueControlType::Voltage {
                    // voltage.q =  target*phase_resistance + voltage_bemf;
                    // voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);

                    self.motor.voltage.q =
                        (self.motor.target_current * self.motor.phase_resistance + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

                    match self.motor.phase_inductance {
                        Some(phase_inductance) => {
                            // voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
                            self.motor.voltage.d = (-self.motor.target_current
                                * shaft_velocity
                                * (self.motor.pole_pairs as f32)
                                * phase_inductance)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => {
                            self.motor.voltage.d = 0.0;
                        }
                    }
                }
            }
            MotionControlType::Velocity => {
                // TODO
            }
            MotionControlType::Angle => {
                // calculate velocity set point
                self.motor.target_shaft_velocity = self
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
                    .update(self.motor.target_shaft_velocity - shaft_velocity);

                if self.torque_controller == TorqueControlType::Voltage {
                    self.motor.voltage.q =
                        (self.motor.target_current * self.motor.phase_resistance + voltage_bemf)
                            .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);

                    match self.motor.phase_inductance {
                        Some(phase_inductance) => {
                            self.motor.voltage.d = (-self.motor.target_current
                                * shaft_velocity
                                * (self.motor.pole_pairs as f32)
                                * phase_inductance)
                                .clamp(-self.motor.limit_voltage, self.motor.limit_voltage);
                        }
                        None => {
                            self.motor.voltage.d = 0.0;
                        }
                    }
                }
            }
        }

        self.set_phase_voltage(self.motor.voltage.q, self.motor.voltage.d, electrical_angle);

        //
    }

    // Method using FOC to set Uq and Ud to the motor at the optimal angle
    // Function implementing Space Vector PWM and Sine PWM algorithms
    //
    // Function using sine approximation
    // regular sin + cos ~300us    (no memory usage)
    // approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
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

    fn get_shaft_velocity(&mut self) -> f32 {
        let dir = if self.sensor_direction == SensorDirection::Normal {
            1.0
        } else if self.sensor_direction == SensorDirection::Inverted {
            -1.0
        } else {
            0.0
        };

        dir * self.lpf_velocity.filter(self.encoder.get_velocity())
    }

    /// ignore full rotations for now
    async fn get_shaft_angle(&mut self) -> f32 {
        let angle = self.encoder.get_angle();

        // convert from 12 bit to float
        let angle = (angle as f32) * 2.0 * core::f32::consts::PI / 4096.0;

        let angle = self.lpf_angle.filter(angle) - self.sensor_offset;

        angle
    }

    /// returns both electrical and shaft angle
    async fn get_electrical_angle(&mut self) -> (f32, f32) {
        // _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
        // Self::normalize_angle()

        let dir = if self.sensor_direction == SensorDirection::Normal {
            1.0
        } else if self.sensor_direction == SensorDirection::Inverted {
            -1.0
        } else {
            0.0
        };

        let shaft_angle = self.get_shaft_angle().await;
        let angle = dir * self.motor.pole_pairs as f32 * shaft_angle - self.zero_electric_angle;

        (Self::normalize_angle(angle), shaft_angle)
    }
}

/// helpers
impl<'a, I2C: embassy_rp::i2c::Instance> SimpleFOC<'a, I2C> {
    /// normalizing radian angle to [0,2PI]
    fn normalize_angle(angle: f32) -> f32 {
        let angle = angle % (2.0 * core::f32::consts::PI);
        if angle >= 0.0 {
            angle
        } else {
            angle + 2.0 * core::f32::consts::PI
        }
    }
}
