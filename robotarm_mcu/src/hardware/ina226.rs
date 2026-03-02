use byteorder::{BigEndian, ByteOrder as _};
use defmt::{Format, debug, error, info, trace, warn};

use crate::hardware::ina226::config::*;

/// https://github.com/justinlatimer/ina226/blob/master/src/lib.rs

impl<I2C, E> crate::hardware::current_sensor::CurrentSensor for INA226<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug + Format,
{
    type Error = INA226Error;

    async fn init(&mut self) -> Result<(), Self::Error> {
        let current_expected_max = 2.5; // Amps

        self.calibrate0(0.125, current_expected_max).await.unwrap();
        // error!("Skipping second INA226 calibration");
        self.calibrate1(0.125, current_expected_max).await.unwrap();

        Ok(())
    }

    fn prev_phase_currents(&self) -> Option<crate::simplefoc::types::PhaseCurrents> {
        self.prev_phase_currents
    }

    fn prev_foc_currents(&self) -> Option<crate::simplefoc::types::DQCurrents> {
        self.prev_foc_currents
    }

    fn set_prev_foc_currents(&mut self, currents: crate::simplefoc::types::DQCurrents) {
        self.prev_foc_currents = Some(currents);
    }

    async fn driver_align(
        &mut self,
        _voltage: f32,
        _modulation_centered: bool,
    ) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn get_phase_currents(
        &mut self,
    ) -> Result<crate::simplefoc::types::PhaseCurrents, Self::Error> {
        let Some(c0) = self
            .current_amps0()
            .await
            .map_err(|_| INA226Error::I2CReadError)?
        else {
            return Err(INA226Error::I2CReadError);
        };
        let Some(c1) = self
            .current_amps1()
            .await
            .map_err(|_| INA226Error::I2CReadError)?
        else {
            return Err(INA226Error::I2CReadError);
        };

        // debug!("Current amps: {}", c0);

        let currents = crate::simplefoc::types::PhaseCurrents::TwoAB {
            a: c0 as f32,
            b: c1 as f32,
        };

        self.prev_phase_currents = Some(currents);

        Ok(currents)
    }
}

#[derive(Debug, Format)]
pub enum INA226Error {
    I2CReadError,
    I2CWriteError,
}

/// INA226 voltage/current/power monitor
pub struct INA226<I2C> {
    i2c: I2C,
    address0: u8,
    address1: u8,
    calibration0: Option<self::config::Calibration>,
    calibration1: Option<self::config::Calibration>,

    prev_phase_currents: Option<crate::simplefoc::types::PhaseCurrents>,
    prev_foc_currents: Option<crate::simplefoc::types::DQCurrents>,
}

impl<I2C, E> INA226<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug + Format,
{
    pub fn new(i2c: I2C, address0: u8, address1: u8) -> Self {
        Self {
            i2c,
            address0,
            address1,
            calibration0: None,
            calibration1: None,
            prev_phase_currents: None,
            prev_foc_currents: None,
        }
    }
    // /// Gets the raw configuration value.
    // #[inline(always)]
    // pub async fn configuration_raw(&mut self) -> Result<u16, E> {
    //     self.read_u16(Register::Configuration).await
    // }

    /// Gets the configuration.
    #[inline(always)]
    pub async fn configuration0(&mut self) -> Result<Option<Config>, E> {
        self.read_u160(Register::Configuration)
            .await
            .map(Config::from_value)
    }

    /// Gets the configuration.
    #[inline(always)]
    pub async fn configuration1(&mut self) -> Result<Option<Config>, E> {
        self.read_u161(Register::Configuration)
            .await
            .map(Config::from_value)
    }

    /// Set the configuration of the device.
    #[inline(always)]
    pub async fn set_configuration0(&mut self, config: &Config) -> Result<(), E> {
        let value = config.to_value();
        self.write_u160(Register::Configuration, value).await
    }

    /// Set the configuration of the device.
    #[inline(always)]
    pub async fn set_configuration1(&mut self, config: &Config) -> Result<(), E> {
        let value = config.to_value();
        self.write_u161(Register::Configuration, value).await
    }

    #[cfg(feature = "nope")]
    /// Gets the raw shunt voltage measurement.
    #[inline(always)]
    pub async fn shunt_voltage_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::ShuntVoltage).await
    }

    #[cfg(feature = "nope")]
    /// Gets the shunt voltage in microvolts.
    #[inline(always)]
    pub async fn shunt_voltage_microvolts(&mut self) -> Result<f64, E> {
        self.read_i16(Register::ShuntVoltage)
            .await
            .map(|raw| (raw as f64) * SHUNT_VOLTAGE_LSB_UV)
    }

    #[cfg(feature = "nope")]
    /// Gets the raw bus voltage measurement.
    #[inline(always)]
    pub async fn bus_voltage_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::BusVoltage).await
    }

    /// Gets the bus voltage in millivolts.
    #[inline(always)]
    pub async fn bus_voltage_millivolts0(&mut self) -> Result<f64, E> {
        self.read_u160(Register::BusVoltage)
            .await
            .map(|raw| (raw as f64) * BUS_VOLTAGE_LSB_MV)
    }

    #[inline(always)]
    pub async fn bus_voltage_millivolts1(&mut self) -> Result<f64, E> {
        self.read_u161(Register::BusVoltage)
            .await
            .map(|raw| (raw as f64) * BUS_VOLTAGE_LSB_MV)
    }

    #[cfg(feature = "nope")]
    /// Gets the raw calculated power being delivered to the load.
    /// Returns zero if calibration has not been performed.
    #[inline(always)]
    pub async fn power_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::Power).await
    }

    #[cfg(feature = "nope")]
    /// Gets the calculated power (in Watts) being delivered to the load.
    /// Requires calibration.
    #[inline(always)]
    pub async fn power_watts(&mut self) -> Result<Option<f64>, E> {
        if let Some(Calibration { power_lsb, .. }) = self.calibration0 {
            self.read_u16(Register::Power)
                .await
                .map(|raw| Some((raw as f64) * power_lsb))
        } else {
            Ok(None)
        }
    }

    #[cfg(feature = "nope")]
    /// Gets the calculated current flowing through the shunt resistor.
    /// Returns zero if calibration has not been performed.
    #[inline(always)]
    pub async fn current_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::Current).await
    }

    /// Gets the calculated current (in Amps) flowing through the shunt resistor.
    /// Requires calibration.
    #[inline(always)]
    pub async fn current_amps0(&mut self) -> Result<Option<f64>, E> {
        if let Some(Calibration { current_lsb, .. }) = self.calibration0 {
            self.read_i160(Register::Current)
                .await
                .map(|raw| Some((raw as f64) * current_lsb))
        } else {
            Ok(None)
        }
    }

    #[inline(always)]
    pub async fn current_amps1(&mut self) -> Result<Option<f64>, E> {
        if let Some(Calibration { current_lsb, .. }) = self.calibration1 {
            self.read_i161(Register::Current)
                .await
                .map(|raw| Some((raw as f64) * current_lsb))
        } else {
            Ok(None)
        }
    }

    /// Gets the calibration register, which controls full-scale
    /// range of current and power measurements.
    #[inline(always)]
    pub async fn calibration0(&mut self) -> Result<u16, E> {
        self.read_u160(Register::Calibration).await
    }

    /// Gets the calibration register, which controls full-scale
    /// range of current and power measurements.
    #[inline(always)]
    pub async fn calibration1(&mut self) -> Result<u16, E> {
        self.read_u161(Register::Calibration).await
    }

    #[cfg(feature = "nope")]
    /// Set the calibration register directly.
    /// NB: after calling this, only `_raw` methods can be used.
    #[inline(always)]
    pub async fn set_calibration_raw(&mut self, value: u16) -> Result<(), E> {
        self.calibration0 = None;
        self.write_u16(Register::Calibration, value).await
    }

    /// Calibrate the sensitvity of the current and power values.
    #[inline(always)]
    pub async fn calibrate0(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), E> {
        let current_lsb = calculate_current_lsb(current_expected_max);
        let power_lsb = current_lsb * POWER_LSB_FACTOR;
        self.calibration0 = Some(Calibration {
            current_lsb,
            power_lsb,
        });
        let value = calculate_calibration_value(shunt_resistance, current_lsb);
        self.write_u160(Register::Calibration, value).await
    }

    #[inline(always)]
    pub async fn calibrate1(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), E> {
        let current_lsb = calculate_current_lsb(current_expected_max);
        let power_lsb = current_lsb * POWER_LSB_FACTOR;
        self.calibration1 = Some(Calibration {
            current_lsb,
            power_lsb,
        });
        let value = calculate_calibration_value(shunt_resistance, current_lsb);
        self.write_u161(Register::Calibration, value).await
    }

    #[cfg(feature = "nope")]
    /// Get the Alert configuration and Conversion Ready flag.
    #[inline(always)]
    pub async fn mask_enable(&mut self) -> Result<MaskEnableFlags, E> {
        self.read_u16(Register::MaskEnable)
            .await
            .map(MaskEnableFlags::from_bits_truncate)
    }

    #[cfg(feature = "nope")]
    /// Set the Alert configuration and Conversion Ready flags.
    #[inline(always)]
    pub async fn set_mask_enable(&mut self, flags: MaskEnableFlags) -> Result<(), E> {
        let value = flags.bits();
        self.write_u16(Register::MaskEnable, value).await
    }

    #[cfg(feature = "nope")]
    /// Get the limit value to compare to the selected Alert function.
    #[inline(always)]
    pub async fn alert_limit(&mut self) -> Result<u16, E> {
        self.read_u16(Register::AlertLimit).await
    }

    #[cfg(feature = "nope")]
    /// Set the Alert Limit register.
    #[inline(always)]
    pub async fn set_alert_limit(&mut self, value: u16) -> Result<(), E> {
        self.write_u16(Register::AlertLimit, value).await
    }

    #[cfg(feature = "nope")]
    /// Get the unique manufacturer identification number
    #[inline(always)]
    pub async fn manufacturer_id(&mut self) -> Result<u16, E> {
        self.read_u16(Register::ManufacturerID).await
    }

    #[cfg(feature = "nope")]
    /// Get the unique die identification number.
    #[inline(always)]
    pub async fn die_id(&mut self) -> Result<u16, E> {
        self.read_u16(Register::DieID).await
    }

    #[inline(always)]
    async fn read_i160(&mut self, register: Register) -> Result<i16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address0, &[register as u8]).await?;
        self.i2c.read(self.address0, &mut buf).await?;
        Ok(BigEndian::read_i16(&buf))
    }

    #[inline(always)]
    async fn read_u160(&mut self, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address0, &[register as u8]).await?;
        self.i2c.read(self.address0, &mut buf).await?;
        Ok(BigEndian::read_u16(&buf))
    }

    #[inline(always)]
    async fn write_u160(&mut self, register: Register, value: u16) -> Result<(), E> {
        self.i2c
            .write(
                self.address0,
                &[register as u8, (value >> 8) as u8, value as u8],
            )
            .await
    }

    #[inline(always)]
    async fn read_i161(&mut self, register: Register) -> Result<i16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address1, &[register as u8]).await?;
        self.i2c.read(self.address1, &mut buf).await?;
        Ok(BigEndian::read_i16(&buf))
    }

    #[inline(always)]
    async fn read_u161(&mut self, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address1, &[register as u8]).await?;
        self.i2c.read(self.address1, &mut buf).await?;
        Ok(BigEndian::read_u16(&buf))
    }

    #[inline(always)]
    async fn write_u161(&mut self, register: Register, value: u16) -> Result<(), E> {
        self.i2c
            .write(
                self.address1,
                &[register as u8, (value >> 8) as u8, value as u8],
            )
            .await
    }

    /// Destroy the INA226 instance and return the I2C.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

pub mod config {
    use bitflags::bitflags;
    use defmt::Format;

    #[repr(u8)]
    pub(super) enum Register {
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
    #[derive(Format, Copy, Clone, Debug, PartialEq)]
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
    #[derive(Format, Copy, Clone, Debug, PartialEq)]
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
    #[derive(Format, Copy, Clone, Debug, PartialEq)]
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
    #[derive(Format, Copy, Clone, Debug, PartialEq)]
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
    #[derive(Format, Debug, PartialEq)]
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
        pub(super) fn to_value(&self) -> u16 {
            (1 << 14)
                | ((self.avg as u16) << 9)
                | ((self.vbusct as u16) << 6)
                | ((self.vshct as u16) << 3)
                | (self.mode as u16)
        }

        pub(super) fn from_value(value: u16) -> Option<Config> {
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

    pub(super) const SHUNT_VOLTAGE_LSB_UV: f64 = 2.5; // 2.5 μV.
    pub(super) const BUS_VOLTAGE_LSB_MV: f64 = 1.25; // 1.25 mV.
    pub(super) const SCALING_VALUE: f64 = 0.00512; // An internal fixed value used to ensure scaling is maintained properly.
    pub(super) const POWER_LSB_FACTOR: f64 = 25.0; // The Power Register LSB is internally programmed to equal 25 times the programmed value of the Current_LSB.

    #[inline(always)]
    pub(super) fn calculate_calibration_value(shunt_resistance: f64, current_lsb: f64) -> u16 {
        (SCALING_VALUE / (current_lsb * shunt_resistance)) as u16
    }

    #[inline(always)]
    pub(super) fn calculate_current_lsb(current_expected_max: f64) -> f64 {
        current_expected_max / ((1 << 15) as f64)
    }

    pub(super) struct Calibration {
        pub(super) current_lsb: f64,
        pub(super) power_lsb: f64,
    }
}
