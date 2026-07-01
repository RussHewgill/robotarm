use defmt::{debug, error, info};

use embassy_rp::gpio::Output;

use crate::hardware::encoder_sensor::EncoderSensor;

const READ_BIT: u16 = 0x4000;
const PARITY_BIT: u16 = 0x8000;
const ERROR_FLAG: u16 = 0x4000;
const DATA_MASK: u16 = 0x3FFF;
const NOP_COMMAND: u16 = 0x0000;

/// Maximum angle value (14-bit: 0-16383, representing 0-360°)
const ANGLE_MAX: u16 = 0x3FFF + 1;

/// https://docs.rs/as5048a-async/latest/as5048a_async/
#[derive(defmt::Format)]
pub struct As5048a<SPI> {
    spi: SPI,
    cs: Output<'static>,

    buf: [u8; 4],

    min_elapsed_time: f32, // minimum elapsed time between velocity updates in seconds

    // angle: f32,
    velocity: f32, // velocity in radians per second

    angle_prev: f32, // result of last call to getSensorAngle(), used for full rotations and velocity
    angle_prev_ts: u64, // timestamp of last call to getAngle, used for velocity
    vel_angle_prev: f32, // angle at last call to getVelocity, used for velocity
    vel_angle_prev_ts: u64, // last velocity calculation timestamp
    full_rotations: i32, // full rotation tracking
    vel_full_rotations: i32, // previous full rotation value for velocity calculation
}

#[derive(defmt::Format, Debug)]
pub enum As5048aError {
    // I2CWriteError,
    // I2CReadError,
    SPIError,
    Communication,
    ParityError,
    SensorError,
}

impl<SPI: embedded_hal_async::spi::SpiBus> EncoderSensor for As5048a<SPI> {
    type Error = As5048aError;

    async fn update(&mut self, ts_us: u64) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn get_mechanical_angle(&self) -> f32 {
        unimplemented!()
    }

    fn get_angle(&self) -> f32 {
        unimplemented!()
    }

    fn get_velocity(&mut self) -> f32 {
        unimplemented!()
    }

    fn reset_position(&mut self) {
        self.full_rotations = 0;
    }
}

impl<SPI: embedded_hal_async::spi::SpiBus> As5048a<SPI> {
    pub fn new(spi: SPI, cs: Output<'static>) -> Self {
        Self {
            spi,
            cs,
            buf: [0; 4],
            min_elapsed_time: 0.0001, // 100 microseconds
            velocity: 0.0,
            angle_prev: 0.0,
            angle_prev_ts: 0,
            vel_angle_prev: 0.0,
            vel_angle_prev_ts: 0,
            full_rotations: 0,
            vel_full_rotations: 0,
        }
    }

    pub async fn _update(&mut self, ts_us: u64) -> Result<(), As5048aError> {
        let angle = self.angle().await?;

        unimplemented!()
    }
}

impl<SPI: embedded_hal_async::spi::SpiBus> As5048a<SPI> {
    /// Read a register from the AS5048A
    ///
    /// This follows the command-response protocol:
    /// - Transaction 1: Send read command, ignore response
    /// - Transaction 2: Send NOP, receive actual data
    pub async fn read_register(&mut self, register: Register) -> Result<u16, As5048aError> {
        let address = u16::from(register);

        let command = READ_BIT | address;

        let command = if calculate_parity(command) {
            PARITY_BIT | command
        } else {
            command
        };

        #[cfg(feature = "defmt")]
        defmt::trace!(
            "Reading register 0x{:04X}, command: 0x{:04X}",
            address,
            command
        );

        let tx_cmd = command.to_be_bytes();
        let mut rx_cmd = [0u8; 2];
        self.spi
            .transfer(&mut rx_cmd, &tx_cmd)
            .await
            .map_err(|_| As5048aError::Communication)?;

        let tx_nop = NOP_COMMAND.to_be_bytes();
        let mut rx_data = [0u8; 2];
        self.spi
            .transfer(&mut rx_data, &tx_nop)
            .await
            .map_err(|_| As5048aError::Communication)?;

        let response = u16::from_be_bytes(rx_data);

        #[cfg(feature = "defmt")]
        defmt::trace!("Received response: 0x{:04X}", response);

        if !verify_parity(response) {
            #[cfg(feature = "defmt")]
            defmt::warn!("Parity error in response: 0x{:04X}", response);

            defmt::warn!("Parity error in response: 0x{:04X}", response);
            return Err(As5048aError::ParityError);
        }

        if response & ERROR_FLAG != 0 {
            // #[cfg(feature = "defmt")]
            defmt::warn!("Sensor error flag set in response: 0x{:04X}", response);
            return Err(As5048aError::SensorError);
        }

        let data = response & DATA_MASK;
        #[cfg(feature = "defmt")]
        defmt::debug!("Register 0x{:04X} value: 0x{:04X}", address, data);

        Ok(data)
    }

    /// Write a register to the AS5048A
    ///
    /// This follows the write protocol:
    /// - Transaction 1: Send write command
    /// - Transaction 2: Send data frame
    /// - Transaction 3: Send NOP to verify write
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - SPI communication fails
    /// - Parity check fails on the response
    /// - The sensor reports an error
    #[allow(dead_code)]
    async fn write_register(&mut self, register: Register, data: u16) -> Result<(), As5048aError> {
        let address = u16::from(register);

        #[cfg(feature = "defmt")]
        defmt::debug!("Writing 0x{:04X} to register 0x{:04X}", data, address);

        let command = address;

        let command = if calculate_parity(command) {
            PARITY_BIT | command
        } else {
            command
        };

        let tx_cmd = command.to_be_bytes();
        let mut rx_cmd = [0u8; 2];
        self.spi
            .transfer(&mut rx_cmd, &tx_cmd)
            .await
            .map_err(|_| As5048aError::Communication)?;

        let data_frame = data & DATA_MASK;
        let data_frame = if calculate_parity(data_frame) {
            PARITY_BIT | data_frame
        } else {
            data_frame
        };

        let tx_data = data_frame.to_be_bytes();
        let mut rx_old = [0u8; 2];
        self.spi
            .transfer(&mut rx_old, &tx_data)
            .await
            .map_err(|_| As5048aError::Communication)?;

        let tx_nop = NOP_COMMAND.to_be_bytes();
        let mut rx_verify = [0u8; 2];
        self.spi
            .transfer(&mut rx_verify, &tx_nop)
            .await
            .map_err(|_| As5048aError::Communication)?;

        let response = u16::from_be_bytes(rx_verify);

        if !verify_parity(response) {
            #[cfg(feature = "defmt")]
            defmt::warn!("Parity error in write verification: 0x{:04X}", response);
            return Err(As5048aError::ParityError);
        }

        if response & ERROR_FLAG != 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!("Sensor error flag set during write");
            return Err(As5048aError::SensorError);
        }

        #[cfg(feature = "defmt")]
        defmt::trace!("Write to register 0x{:04X} successful", address);

        Ok(())
    }

    /// Get the 14-bit corrected angular position
    ///
    /// Value ranges from 0 to 16383 (0° to 359.978°)
    /// Use [`ANGLE_MAX`] constant for conversion calculations
    ///
    /// For integer degree conversion, use [`Self::angle_degrees`]
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails, parity check fails, or the sensor reports an error
    pub async fn angle(&mut self) -> Result<u16, As5048aError> {
        self.read_register(Register::Angle).await
    }

    /// Get the angular position in degrees (0-359)
    ///
    /// This method converts the raw 14-bit angle value to degrees using
    /// integer arithmetic with saturation. The result is rounded down
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails, parity check fails, or the sensor reports an error
    pub async fn angle_degrees(&mut self) -> Result<u16, As5048aError> {
        let angle = self.angle().await?;
        let degrees = (u32::from(angle).saturating_mul(360)) / u32::from(ANGLE_MAX);
        #[allow(clippy::cast_possible_truncation)]
        Ok(degrees as u16)
    }

    /// Get the 14-bit magnitude value from CORDIC
    ///
    /// Useful for checking magnet presence and strength
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails, parity check fails, or the sensor reports an error
    pub async fn magnitude(&mut self) -> Result<u16, As5048aError> {
        self.read_register(Register::Magnitude).await
    }
}

/// Register addresses for AS5048A
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
#[repr(u16)]
pub enum Register {
    /// NOP command (no operation)
    Nop = 0x0000,
    /// Clear error flag.
    ClearErrorFlag = 0x0001,
    /// Diagnostics and AGC register
    DiagAgc = 0x3FFD,
    /// Magnitude register (14-bit)
    Magnitude = 0x3FFE,
    /// Angle register (14-bit corrected position)
    Angle = 0x3FFF,
}

impl From<Register> for u16 {
    fn from(reg: Register) -> u16 {
        reg as u16
    }
}

/// Calculate even parity bit for the lower 15 bits of a 16-bit value
fn calculate_parity(value: u16) -> bool {
    let bits = value & 0x7FFF;
    bits.count_ones() % 2 == 1
}

/// Verify even parity of a 16-bit frame
fn verify_parity(frame: u16) -> bool {
    frame.count_ones().is_multiple_of(2)
}
