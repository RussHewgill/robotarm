use as5600::asynch::As5600;
use embassy_rp::i2c::Async;

use crate::simplefoc::{
    pid::PIDController,
    types::{DQCurrents, DQVoltages},
};

pub struct SimpleFOC<'a> {
    encoder: As5600<embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, Async>>,
    driver: crate::hardware::pwm_driver::PWMDriver<'a>,

    pub motor: BLDCMotor,

    pub phase_voltages: [f32; 3],

    pub pid_velocity: PIDController,
    pub pid_angle: PIDController,
}

#[derive(defmt::Format)]
pub struct BLDCMotor {
    pub pole_pairs: u8,
    pub phase_resistance: f32,
    pub motor_kv: f32,
    pub phase_inductance: f32,

    target: f32,
    feed_forward_velocity: f32,
    shaft_angle: f32,
    electrical_angle: f32,
    shaft_velocity: f32,
    /// target current (q current)
    target_current: f32,
    target_shaft_velocity: f32,
    target_shaft_angle: f32,

    voltage: DQVoltages,
    current: DQCurrents,

    estimated_back_emf: f32,
    /// Phase voltages U alpha and U beta used for inverse Park and Clarke transform
    phase_voltages_alpha_beta: (f32, f32),

    voltage_sensor_align: f32,
    velocity_index_search: f32,

    limit_voltage: f32,
    limit_current: f32,
    limit_velocity: f32,
}

/// new, control
impl<'a> SimpleFOC<'a> {
    // pub fn new(
    //     encoder: As5600<embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C1, Async>>,
    //     driver: crate::hardware::pwm_driver::PWMDriver<'a>,

    //     motor: BLDCMotor,
    // ) -> Self {
    //     SimpleFOC {
    //         encoder,
    //         driver,
    //         motor,
    //         // pid_velocity: pid::FocPid::default(),
    //         // pid_angle: pid::FocPid::default(),
    //     }
    // }

    pub fn disable(&self) {
        unimplemented!()
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
        unimplemented!()
    }

    pub fn update_foc(&mut self) {
        unimplemented!()
    }
}

/// helpers
impl<'a> SimpleFOC<'a> {
    fn get_angle(&mut self) -> u16 {
        // let Ok(angle) = self.encoder.angle() else {
        //     panic!("Failed to read angle from encoder");
        // };

        // let angle = self.encoder.angle().unwrap();

        unimplemented!()
    }
}
