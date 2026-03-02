use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;
use robotarm_protocol::types::MotionControlType;

use crate::{
    comms::usb::UsbLogger,
    hardware::{
        as5600::AS5600,
        current_sensor::{self, CurrentSensor},
        encoder_sensor::EncoderSensor,
        mt_6701_adc::MT6701,
    },
    simplefoc::{
        bldc::BLDCMotor,
        lowpass::LowPassFilter,
        pid::PIDController,
        types::{NOT_SET, PhaseVoltages, SensorDirection, TorqueControlType},
    },
};

// #[derive(defmt::Format)]
#[cfg(feature = "nope")]
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

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub enum FOCModulation {
    SinePWM,
    SpaceVectorPWM,
}

pub struct SimpleFOC<'a, ENCODER: EncoderSensor, CURRENT = ()> {
    pub id: u8,

    // pub(super) encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    // pub(super) encoder: MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    // pub(super) encoder: MT6701<'a, DMA>,
    // pub encoder: MT6701<'a, DMA>,
    pub encoder: ENCODER,

    pub current_sensor: Option<CURRENT>,

    // encoder:
    //     MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    pub(super) pwm_driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

    // pub(super) enable_pin: Output<'a>,
    pub usb_logger: Option<UsbLogger>,

    pub(super) debug: bool,
    pub(super) prev_debug_us: u64,
    // debug_freq_hz: u64,
    debug_us_interval: u64,

    pub motion_downsample: u32,
    pub(super) motion_downsample_counter: u32,

    // sensor_us_interval: u64,
    // pub(super) prev_sensor_us: u64,
    pub angle_sensor_downsample: u32,
    pub(super) angle_sensor_downsample_counter: u32,

    pub current_sensor_downsample: u32,
    pub(super) current_sensor_downsample_counter: u32,

    // pub(super) motor_status: FOCStatus,
    pub(super) enabled: bool,

    pub(super) motor: BLDCMotor,

    pub(super) phase_v: PhaseVoltages,

    pub(super) modulation: FOCModulation,

    pub(super) sensor_direction: SensorDirection,
    pub(super) sensor_offset: f32,
    pub(super) zero_electric_angle: f32,

    pub(super) motion_control: MotionControlType,
    pub(super) torque_controller: TorqueControlType,

    pub feed_forward_torque: f32,

    // pub(super) pid_current_q: PIDController,
    // pub(super) pid_current_d: PIDController,

    // not used except with current sensor
    // pub(super) lpf_current_q: LowPassFilter,
    // pub(super) lpf_current_d: LowPassFilter,
    pub pid_velocity: PIDController,
    pub pid_angle: PIDController,

    pub(super) pid_velocity_tuner: Option<crate::simplefoc::pid_tuning::PidTuner>,
    pub(super) pid_angle_tuner: Option<crate::simplefoc::pid_tuning::PidTuner>,

    // pub(super) lpf_velocity: LowPassFilter,
    pub(super) lpf_velocity: LowPassFilter,
    pub(super) lpf_angle: LowPassFilter,

    pub(super) openloop_shaft_angle: f32,
}

impl<'a, ENCODER: EncoderSensor, CURRENT: CurrentSensor> SimpleFOC<'a, ENCODER, CURRENT> {
    pub fn new(
        id: u8,

        // encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
        // encoder: MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
        encoder: ENCODER,
        current_sensor: Option<CURRENT>,

        driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

        motor: BLDCMotor,

        usb_logger: Option<UsbLogger>,
    ) -> Self {
        // const PID_CURRENT_KP: f32 = 3.;
        // const PID_CURRENT_KI: f32 = 300.;
        // const PID_CURRENT_KD: f32 = 0.;
        // const PID_CURRENT_RAMP: f32 = 0.;
        // const PID_CURRENT_LIMIT: f32 = 12.0;

        // const CURR_LPF_TF: f32 = 0.005;

        const PID_VELOCITY_KP: f32 = 0.1;
        const PID_VELOCITY_KI: f32 = 0.0;
        // const PID_VELOCITY_KD: f32 = 0.0;

        const PID_VELOCITY_KD: f32 = 0.0005;

        // const PID_VELOCITY_KP: f32 = 0.08;
        // const PID_VELOCITY_KI: f32 = 0.0;
        // const PID_VELOCITY_KD: f32 = 0.00015;

        // const PID_VELOCITY_KP: f32 = 0.5;
        // const PID_VELOCITY_KI: f32 = 10.0;

        // const PID_VELOCITY_KP: f32 = 0.35077736;
        // const PID_VELOCITY_KI: f32 = 23.697653;
        // const PID_VELOCITY_KD: f32 = 0.0013006842;

        const PID_VELOCITY_RAMP: f32 = 1000.0;
        // const PID_VELOCITY_RAMP: f32 = 0.0;
        const PID_VELOCITY_LIMIT: f32 = 100.;

        const PID_ANGLE_KP: f32 = 20.0;
        // const PID_ANGLE_LIMIT: f32 = 20.0;
        const PID_ANGLE_LIMIT: f32 = 100.0;

        // const VEL_LPF_TF: f32 = 0.;
        // const VEL_LPF_TF: f32 = 0.01;
        const VEL_LPF_TF: f32 = 0.05;
        // const VEL_LPF_TF: f32 = 0.2;

        // const ANGLE_LPF_TF: f32 = 0.;
        const ANGLE_LPF_TF: f32 = 0.005;

        SimpleFOC {
            id,

            encoder,
            current_sensor,
            pwm_driver: driver,
            motor,

            usb_logger,

            debug: false,
            prev_debug_us: 0,
            // debug_freq_hz: 10,
            debug_us_interval: 100_000,

            motion_downsample: 0,
            motion_downsample_counter: 0,

            // sensor_us_interval: 00,
            // prev_sensor_us: 0,
            angle_sensor_downsample: 1,
            angle_sensor_downsample_counter: 0,

            current_sensor_downsample: 1,
            current_sensor_downsample_counter: 0,

            // motor_status: FOCStatus::MotorUninitialized,
            enabled: false,

            sensor_direction: SensorDirection::Unknown,
            sensor_offset: 0.0,
            zero_electric_angle: NOT_SET,

            motion_control: MotionControlType::Angle,
            torque_controller: TorqueControlType::Voltage,

            feed_forward_torque: 0.0,

            phase_v: PhaseVoltages::default(),

            // modulation: FOCModulation::SinePWM,
            modulation: FOCModulation::SpaceVectorPWM,

            pid_velocity: PIDController::new(
                PID_VELOCITY_KP,
                PID_VELOCITY_KI,
                PID_VELOCITY_KD,
                PID_VELOCITY_RAMP,
                PID_VELOCITY_LIMIT,
            ),

            pid_angle: PIDController::new(PID_ANGLE_KP, 0.0, 0.0, 0.0, PID_ANGLE_LIMIT),

            pid_angle_tuner: None,
            pid_velocity_tuner: None,

            lpf_velocity: LowPassFilter::new(VEL_LPF_TF),
            lpf_angle: LowPassFilter::new(ANGLE_LPF_TF),

            openloop_shaft_angle: 0.,
        }
    }

    pub fn set_debug_freq(&mut self, freq_hz: u64) {
        if freq_hz == 0 {
            self.debug_us_interval = 0;
        } else {
            self.debug_us_interval = 1_000_000 / freq_hz;
        }
    }

    pub fn debug_us_interval(&self) -> u64 {
        self.debug_us_interval
    }

    // pub fn set_sensor_update_freq(&mut self, freq_hz: u64) {
    //     if freq_hz == 0 {
    //         self.sensor_us_interval = 0;
    //     } else {
    //         self.sensor_us_interval = 1_000_000 / freq_hz;
    //     }
    // }

    // pub fn sensor_update_us_interval(&self) -> u64 {
    //     self.sensor_us_interval
    // }

    pub fn set_vel_pid_debug(&mut self, target_input: f32) {
        let tuner = crate::simplefoc::pid_tuning::PidTuner::new(&self.pid_velocity, target_input);
        self.pid_velocity_tuner = Some(tuner);
    }
}
