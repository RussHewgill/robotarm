use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::Output;

use crate::{
    comms::usb::UsbMonitor,
    hardware::{as5600::AS5600, encoder_sensor::EncoderSensor, mt_6701_adc::MT6701},
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

pub struct SimpleFOC<'a, SENSOR: EncoderSensor> {
    // pub(super) encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    // pub(super) encoder: MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    // pub(super) encoder: MT6701<'a, DMA>,
    // pub encoder: MT6701<'a, DMA>,
    pub encoder: SENSOR,
    // encoder:
    //     MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
    pub(super) pwm_driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

    pub(super) enable_pin: Output<'a>,

    pub debug_port: UsbMonitor,

    pub debug: bool,

    pub(super) motor_status: FOCStatus,

    pub(super) enabled: bool,

    pub(super) motor: BLDCMotor,

    pub(super) phase_v: PhaseVoltages,

    pub(super) sensor_direction: SensorDirection,
    pub(super) sensor_offset: f32,
    pub(super) zero_electric_angle: f32,

    pub(super) motion_control: MotionControlType,
    pub(super) torque_controller: TorqueControlType,

    // pub(super) pid_current_q: PIDController,
    // pub(super) pid_current_d: PIDController,

    // not used except with current sensor
    // pub(super) lpf_current_q: LowPassFilter,
    // pub(super) lpf_current_d: LowPassFilter,
    pub(super) pid_velocity: PIDController,
    pub(super) pid_angle: PIDController,

    // pub(super) lpf_velocity: LowPassFilter,
    pub(super) lpf_velocity: LowPassFilter,
    pub(super) lpf_angle: LowPassFilter,

    pub(super) openloop_shaft_angle: f32,
}

impl<'a, SENSOR: EncoderSensor> SimpleFOC<'a, SENSOR> {
    pub fn new(
        // encoder: AS5600<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
        // encoder: MT6701<embassy_rp::i2c::I2c<'a, I2C, embassy_rp::i2c::Async>>,
        encoder: SENSOR,
        driver: crate::simplefoc::pwm_driver::PWMDriver<'a>,

        enable_pin: Output<'a>,

        motor: BLDCMotor,

        usb_monitor: UsbMonitor,
    ) -> Self {
        // const PID_CURRENT_KP: f32 = 3.;
        // const PID_CURRENT_KI: f32 = 300.;
        // const PID_CURRENT_KD: f32 = 0.;
        // const PID_CURRENT_RAMP: f32 = 0.;
        // const PID_CURRENT_LIMIT: f32 = 12.0;

        // const CURR_LPF_TF: f32 = 0.005;

        const PID_VELOCITY_KP: f32 = 0.1;
        const PID_VELOCITY_KI: f32 = 0.0;

        // const PID_VELOCITY_KP: f32 = 0.5;
        // const PID_VELOCITY_KI: f32 = 10.0;

        const PID_VELOCITY_KD: f32 = 0.0;
        const PID_VELOCITY_RAMP: f32 = 1000.0;
        const PID_VELOCITY_LIMIT: f32 = 20.0;

        const PID_ANGLE_KP: f32 = 20.0;
        const PID_ANGLE_LIMIT: f32 = 20.0;

        // const VEL_LPF_TF: f32 = 0.;
        // const VEL_LPF_TF: f32 = 0.005;
        const VEL_LPF_TF: f32 = 0.05;
        // const VEL_LPF_TF: f32 = 0.001;

        const ANGLE_LPF_TF: f32 = 0.;
        // const ANGLE_LPF_TF: f32 = 0.001;

        SimpleFOC {
            encoder,
            pwm_driver: driver,
            motor,

            enable_pin,

            debug_port: usb_monitor,

            debug: false,

            motor_status: FOCStatus::MotorUninitialized,

            enabled: false,

            sensor_direction: SensorDirection::Unknown,
            sensor_offset: 0.0,
            zero_electric_angle: NOT_SET,

            motion_control: MotionControlType::Angle,
            torque_controller: TorqueControlType::Voltage,

            phase_v: PhaseVoltages::default(),

            pid_velocity: PIDController::new(
                PID_VELOCITY_KP,
                PID_VELOCITY_KI,
                PID_VELOCITY_KD,
                PID_VELOCITY_RAMP,
                PID_VELOCITY_LIMIT,
            ),

            pid_angle: PIDController::new(PID_ANGLE_KP, 0.0, 0.0, 0.0, PID_ANGLE_LIMIT),

            lpf_velocity: LowPassFilter::new(VEL_LPF_TF),
            lpf_angle: LowPassFilter::new(ANGLE_LPF_TF),

            openloop_shaft_angle: 0.,
        }
    }
}
