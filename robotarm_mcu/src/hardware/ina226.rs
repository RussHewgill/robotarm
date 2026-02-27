use defmt::{debug, error, info, trace, warn};

/// https://github.com/justinlatimer/ina226/blob/master/src/lib.rs

/// INA226 voltage/current/power monitor
pub struct INA226<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Option<self::config::Calibration>,
}

impl<I2C, E> INA226<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        INA226 {
            i2c,
            address,
            calibration: None,
        }
    }
}

pub mod config {
    use bitflags::bitflags;

    #[repr(u8)]
    enum Register {
        Configuration = 0x00,
        ShuntVoltage = 0x01,
        BusVoltage = 0x02,
        Power = 0x03,
        Current = 0x04,
        Calibration = 0x05,
        MaskEnable = 0x06,
        AlertLimit = 0x07,
        ManufacturerID = 0xFE,
        DieID = 0xFF,
    }

    /// Determines the number of samples that are collected and averaged.
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum AVG {
        /// 1 sample averaging
        _1 = 0b000,
        /// 4 sample averaging
        _4 = 0b001,
        /// 16 sample averaging
        _16 = 0b010,
        /// 64 sample averaging
        _64 = 0b011,
        /// 128 sample averaging
        _128 = 0b100,
        /// 256 sample averaging
        _256 = 0b101,
        /// 512 sample averaging
        _512 = 0b110,
        /// 1024 sample averaging
        _1024 = 0b111,
    }

    impl AVG {
        fn parse(value: u8) -> Option<AVG> {
            match value {
                0b000 => Some(AVG::_1),
                0b001 => Some(AVG::_4),
                0b010 => Some(AVG::_16),
                0b011 => Some(AVG::_64),
                0b100 => Some(AVG::_128),
                0b101 => Some(AVG::_256),
                0b110 => Some(AVG::_512),
                0b111 => Some(AVG::_1024),
                _ => None,
            }
        }
    }

    /// Sets the conversion time for the bus voltage measurement.
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum VBUSCT {
        /// 140us conversion time
        _140us = 0b000,
        /// 204us conversion time
        _204us = 0b001,
        /// 332us conversion time
        _332us = 0b010,
        /// 588us conversion time
        _588us = 0b011,
        /// 1100us conversion time
        _1100us = 0b100,
        /// 2116us conversion time
        _2116us = 0b101,
        /// 4156us conversion time
        _4156us = 0b110,
        /// 8244us conversion time
        _8244us = 0b111,
    }

    impl VBUSCT {
        fn parse(value: u8) -> Option<VBUSCT> {
            match value {
                0b000 => Some(VBUSCT::_140us),
                0b001 => Some(VBUSCT::_204us),
                0b010 => Some(VBUSCT::_332us),
                0b011 => Some(VBUSCT::_588us),
                0b100 => Some(VBUSCT::_1100us),
                0b101 => Some(VBUSCT::_2116us),
                0b110 => Some(VBUSCT::_4156us),
                0b111 => Some(VBUSCT::_8244us),
                _ => None,
            }
        }
    }

    /// Sets the conversion time for the shunt voltage measurement.
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum VSHCT {
        /// 140us conversion time
        _140us = 0b000,
        /// 204us conversion time
        _204us = 0b001,
        /// 332us conversion time
        _332us = 0b010,
        /// 588us conversion time
        _588us = 0b011,
        /// 1100us conversion time
        _1100us = 0b100,
        /// 2116us conversion time
        _2116us = 0b101,
        /// 4156us conversion time
        _4156us = 0b110,
        /// 8244us conversion time
        _8244us = 0b111,
    }

    impl VSHCT {
        fn parse(value: u8) -> Option<VSHCT> {
            match value {
                0b000 => Some(VSHCT::_140us),
                0b001 => Some(VSHCT::_204us),
                0b010 => Some(VSHCT::_332us),
                0b011 => Some(VSHCT::_588us),
                0b100 => Some(VSHCT::_1100us),
                0b101 => Some(VSHCT::_2116us),
                0b110 => Some(VSHCT::_4156us),
                0b111 => Some(VSHCT::_8244us),
                _ => None,
            }
        }
    }

    /// Selects continuous, triggered, or power-down mode of operation.
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum MODE {
        /// Power Down (or Shutdown)
        PowerDown = 0b000,
        /// Shunt Voltage, Triggered
        ShuntVoltageTriggered = 0b001,
        /// Bus Voltage, Triggered
        BusVoltageTriggered = 0b010,
        /// Shunt and Bus, Triggered
        ShuntBusVoltageTriggered = 0b011,
        /// Power Down (or Shutdown)
        PowerDown2 = 0b100,
        /// Shunt Voltage, Continuous
        ShuntVoltageContinuous = 0b101,
        /// Bus Voltage, Continuous
        BusVoltageContinuous = 0b110,
        /// Shunt and Bus, Continuous
        ShuntBusVoltageContinuous = 0b111,
    }

    impl MODE {
        fn parse(value: u8) -> Option<MODE> {
            match value {
                0b000 => Some(MODE::PowerDown),
                0b001 => Some(MODE::ShuntVoltageTriggered),
                0b010 => Some(MODE::BusVoltageTriggered),
                0b011 => Some(MODE::ShuntBusVoltageTriggered),
                0b100 => Some(MODE::PowerDown2),
                0b101 => Some(MODE::ShuntVoltageContinuous),
                0b110 => Some(MODE::BusVoltageContinuous),
                0b111 => Some(MODE::ShuntBusVoltageContinuous),
                _ => None,
            }
        }
    }

    /// The state of the configuration register.
    #[derive(Debug, PartialEq)]
    pub struct Config {
        /// Averaging Mode
        pub avg: AVG,
        /// Bus Voltage Conversion Time
        pub vbusct: VBUSCT,
        /// Shunt Voltage Conversion Time
        pub vshct: VSHCT,
        /// Operating Mode
        pub mode: MODE,
    }

    impl Config {
        fn to_value(&self) -> u16 {
            (1 << 14)
                | ((self.avg as u16) << 9)
                | ((self.vbusct as u16) << 6)
                | ((self.vshct as u16) << 3)
                | (self.mode as u16)
        }

        fn from_value(value: u16) -> Option<Config> {
            Some(Config {
                avg: AVG::parse(((value >> 9) & 0b111) as u8)?,
                vbusct: VBUSCT::parse(((value >> 6) & 0b111) as u8)?,
                vshct: VSHCT::parse(((value >> 3) & 0b111) as u8)?,
                mode: MODE::parse((value & 0b111) as u8)?,
            })
        }
    }

    bitflags! {
        /// The Mask/Enable Register selects the function that is enabled to control the
        /// Alert pin as well as how that pin functions. If multiple functions are
        /// enabled, the highest significant bit position Alert Function (D15-D11) takes
        /// priority and responds to the Alert Limit Register.
        #[derive(Debug, PartialEq)]
        pub struct MaskEnableFlags: u16 {
            /// Shunt Voltage Over-Voltage
            /// Bit 15
            /// Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement
            /// following a conversion exceeds the value programmed in the Alert Limit Register.
            const SOL = 1 << 15;

            /// Shunt Voltage Under-Voltage
            /// Bit 14
            /// Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement
            /// following a conversion drops below the value programmed in the Alert Limit Register.
            const SUL = 1 << 14;

            /// Bus Voltage Over-Voltage
            /// Bit 13
            /// Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement
            /// following a conversion exceeds the value programmed in the Alert Limit Register.
            const BOL = 1 << 13;

            /// Bus Voltage Under-Voltage
            /// Bit 12
            /// Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement
            /// following a conversion drops below the value programmed in the Alert Limit Register.
            const BUL = 1 << 12;

            /// Power Over-Limit
            /// Bit 11
            /// Setting this bit high configures the Alert pin to be asserted if the Power calculation made
            /// following a bus voltage measurement exceeds the value programmed in the Alert Limit Register.
            const POL = 1 << 11;

            /// Conversion Ready
            /// Bit 10
            /// Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag, Bit 3, is
            /// asserted indicating that the device is ready for the next conversion.
            const CNVR = 1 << 10;

            /// Alert Function Flag
            /// Bit 4
            /// While only one Alert Function can be monitored at the Alert pin at a time, the Conversion Ready can also
            /// be enabled to assert the Alert pin. Reading the Alert Function Flag following an alert allows the user
            /// to determine if the Alert Function was the source of the Alert.
            ///
            /// When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit clears only when the
            /// Mask/Enable Register is read. When the Alert Latch Enable bit is set to Transparent mode, the Alert
            /// Function Flag bit is cleared following the next conversion that does not result in an Alert condition.
            const AFF = 1 << 4;

            /// Conversion Ready Flag
            /// Bit 3
            /// Although the device can be read at any time, and the data from the last conversion is available, the
            /// Conversion Ready Flag bit is provided to help coordinate one-shot or triggered conversions. The
            /// Conversion Ready Flag bit is set after all conversions, averaging, and multiplications are complete.
            /// Conversion Ready Flag bit clears under the following conditions:
            /// 1.) Writing to the Configuration Register (except for Power-Down selection)
            /// 2.) Reading the Mask/Enable Register
            const CVRF = 1 << 3;

            /// Math Overflow Flag
            /// Bit 2
            /// This bit is set to '1' if an arithmetic operation resulted in an overflow error. It indicates that
            /// current and power data may be invalid.
            const OVF = 1 << 2;

            /// Alert Polarity bit; sets the Alert pin polarity.
            /// Bit 1
            /// 1 = Inverted (active-high open collector)
            /// 0 = Normal (active-low open collector) (default)
            const APOL = 1 << 1;

            /// Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits.
            /// Bit 0
            /// 1 = Latch enabled
            /// 0 = Transparent (default)
            /// When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit
            /// resets to the idle states when the fault has been cleared. When the Alert Latch Enable
            /// bit is set to Latch mode, the Alert pin and Alert Flag bit remains active following a
            /// fault until the Mask/Enable Register has been read.
            const LEN = 1 << 0;
        }
    }

    /// Default address of INA226 devices.
    pub const DEFAULT_ADDRESS: u8 = 0b1000000;

    const SHUNT_VOLTAGE_LSB_UV: f64 = 2.5; // 2.5 μV.
    const BUS_VOLTAGE_LSB_MV: f64 = 1.25; // 1.25 mV.
    const SCALING_VALUE: f64 = 0.00512; // An internal fixed value used to ensure scaling is maintained properly.
    const POWER_LSB_FACTOR: f64 = 25.0; // The Power Register LSB is internally programmed to equal 25 times the programmed value of the Current_LSB.

    #[inline(always)]
    fn calculate_calibration_value(shunt_resistance: f64, current_lsb: f64) -> u16 {
        (SCALING_VALUE / (current_lsb * shunt_resistance)) as u16
    }

    #[inline(always)]
    fn calculate_current_lsb(current_expected_max: f64) -> f64 {
        current_expected_max / ((1 << 15) as f64)
    }

    pub(super) struct Calibration {
        current_lsb: f64,
        power_lsb: f64,
    }
}
