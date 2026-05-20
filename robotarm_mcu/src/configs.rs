pub use motor_configs::*;

mod motor_configs {
    use crate::simplefoc::bldc::BLDCMotor;

    // #[cfg(feature = "nope")]
    // let motor_config0 = BLDCMotor::new(
    //     7,          // pole pairs
    //     Some(9.2),  // phase resistance
    //     Some(140.), // motor kv
    //     None,
    //     None,
    // );

    // 4015
    pub const MOTOR_CONFIG_4015: BLDCMotor = BLDCMotor::new(
        11,        // pole pairs
        Some(4.8), // phase resistance
        Some(61.), // motor kv
        None,
        None,
    );

    // GM3506
    pub const MOTOR_CONFIG_GM3506: BLDCMotor = BLDCMotor::new(
        11, // pole pairs
        // Some(5.6), // phase resistance
        Some(2.8),  // phase resistance
        Some(100.), // motor kv
        // None,
        // Some(0.0026), // phase inductance
        None,
        None,
    );

    // GM4108
    pub const MOTOR_CONFIG_GM4108: BLDCMotor = BLDCMotor::new(
        11,        // pole pairs
        Some(5.4), // phase resistance
        // Some(), // motor kv
        None,
        // Some(0.0026), // phase inductance
        None,
        None,
    );

    // GM5208-24
    pub const MOTOR_CONFIG_GM5208_24: BLDCMotor = BLDCMotor::new(
        11,        // pole pairs
        Some(10.), // phase resistance
        // Some(20.), // motor kv
        None,
        // Some(0.0026), // phase inductance
        None,
        None,
    );
}
