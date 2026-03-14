use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{Input, Output};
use embassy_time::Timer;

// use std::time::Duration;

// use rppal::gpio::{InputPin, OutputPin};

pub struct ADS1256<'a, SPI> {
    spi: SPI,
    cs: Output<'a>,

    reset: Output<'a>,
    data_ready_pin: Input<'a>,
    config: Config,
}

impl<'a, SPI> ADS1256<'a, SPI>
where
    SPI: embedded_hal::spi::SpiBus,
{
    pub fn new(
        spi: SPI,
        cs: Output<'a>,
        reset: Output<'a>,
        data_ready_pin: Input<'a>,
        config: Config,
    ) -> Self {
        Self {
            spi,
            cs,
            reset,
            data_ready_pin,
            config,
        }
    }

    pub async fn set_config(&mut self, config: &Config) -> Result<(), ADS1256Error> {
        self.config = *config;
        self.init().await?;
        Ok(())
    }

    pub async fn test_init(&mut self) -> Result<(), ADS1256Error> {
        debug!("Resetting sensor");
        self.reset().await?;
        debug!("Done reset");

        let adcon = self.read_register(Register::ADCON).await?;
        debug!("ADCON: {:08b}", adcon);

        let new_adcon = (adcon & 0x07) | self.config.gain.bits();
        self.write_register(Register::ADCON, new_adcon).await?;
        debug!("Wrote ADCON: {:08b}", new_adcon);

        let adcon = self.read_register(Register::ADCON).await?;
        debug!("ADCON: {:08b}", adcon);

        Ok(())
    }

    pub async fn test_read(&mut self) -> Result<(), ADS1256Error> {
        let r = self.read_register(Register::STATUS).await?;
        debug!("STATUS: {:08b}", r);

        // let reg = Register::STATUS;
        // self.cs.set_low();
        // //write
        // self.spi
        //     .write(&[(Command::RREG.bits() | reg.addr()), 0x00])
        //     .map_err(|_| ADS1256Error::SpiError)?;
        // Timer::after_micros(100).await;
        // self.cs.set_low();

        Ok(())
    }

    pub async fn init(&mut self) -> Result<(), ADS1256Error> {
        self.reset().await?;

        let adcon = self.read_register(Register::ADCON).await?;
        //disable clkout and set the gain
        let new_adcon = (adcon & 0x07) | self.config.gain.bits();
        self.write_register(Register::ADCON, new_adcon).await?;
        self.write_register(Register::DRATE, self.config.sampling_rate.bits())
            .await?;
        self.send_command(Command::SELFCAL).await?;
        self.wait_for_ready().await; //wait for calibration to complete
        Ok(())
    }

    pub async fn wait_for_ready(&mut self) {
        // self.data_ready_pin.wait_for_low();
        self.data_ready_pin.wait_for_low().await;

        // self.data_ready_pin
        //     .set_interrupt(rppal::gpio::Trigger::FallingEdge, None)
        //     .unwrap();
        // // don't reset
        // self.data_ready_pin.poll_interrupt(false, None).unwrap();
        // unimplemented!()
        // debug!("Skipping wait for ready");
    }

    pub async fn reset(&mut self) -> Result<(), ADS1256Error> {
        self.reset.set_low();
        Timer::after_micros(100).await;
        self.reset.set_high();
        Timer::after_millis(200).await;
        Ok(())
    }

    pub async fn read_register(&mut self, reg: Register) -> Result<u8, ADS1256Error> {
        self.cs.set_low();
        //write
        self.spi
            .write(&[(Command::RREG.bits() | reg.addr()), 0x00])
            .map_err(|_| ADS1256Error::SpiError)?;
        Timer::after_micros(10).await;
        //read
        let mut rx_buf = [0];
        // self.spi.transfer(&mut rx_buf)?;
        self.spi
            .transfer_in_place(&mut rx_buf)
            // .read(&mut rx_buf)
            .map_err(|_| ADS1256Error::SpiError)?;
        Timer::after_micros(5).await; // t11
        self.cs.set_high();
        Ok(rx_buf[0])
    }

    pub async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), ADS1256Error> {
        self.cs.set_low();

        let mut tx_buf = [(Command::WREG.bits() | reg.addr()), 0x00, value];
        self.spi
            .transfer_in_place(&mut tx_buf)
            .map_err(|_| ADS1256Error::SpiError)?;
        Timer::after_micros(5).await; // t11
        self.cs.set_high();
        Ok(())
    }

    pub async fn send_command(&mut self, command: Command) -> Result<(), ADS1256Error> {
        self.cs.set_low();
        self.spi
            .write(&[command.bits()])
            .map_err(|_| ADS1256Error::SpiError)?;
        self.cs.set_high();
        Ok(())
    }

    /// Read 24 bit value from ADS1256. Issue this command after DRDY goes low
    async fn read_raw_data(&mut self) -> Result<i32, ADS1256Error> {
        self.cs.set_low();
        self.spi
            .write(&[Command::RDATA.bits()])
            .map_err(|_| ADS1256Error::SpiError)?;
        Timer::after_micros(10).await;
        //receive 3 bytes from spi
        let mut buf = [0u8; 3];
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(|_| ADS1256Error::SpiError)?;
        self.cs.set_high();

        let mut result: u32 = ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32);
        //sign extension if result is negative
        if (result & 0x800000) != 0 {
            result |= 0xFF000000;
        }
        Ok(result as i32)
    }

    pub async fn read_channel(&mut self, ch1: Channel, ch2: Channel) -> Result<i32, ADS1256Error> {
        //wait form data ready pin to be low
        self.wait_for_ready().await;

        //select channel
        self.write_register(Register::MUX, ch1.bits() << 4 | ch2.bits())
            .await?;

        //start conversion
        self.send_command(Command::SYNC).await?;
        Timer::after_micros(5).await;

        self.send_command(Command::WAKEUP).await?;
        Timer::after_micros(5).await; // t11

        //read channel data
        let adc_code = self.read_raw_data().await?;

        Ok(adc_code)
    }

    pub fn convert_to_voltage(&self, raw_value: i32) -> f64 {
        (raw_value as f64) / (0x7FFFFF as f64) * (2.0 * REF_VOLTS) / (self.config.gain.val() as f64)
    }
}

/// ADC reference voltage in volts
const REF_VOLTS: f64 = 2.5;

//The operation of the ADS1256 is controlled through a set of registers.
//ADS1256 datasheet,  Table 23.
#[derive(Debug, Copy, Clone)]
pub enum Register {
    STATUS = 0x00,
    MUX = 0x01,
    ADCON = 0x02,
    DRATE = 0x03,
    IO = 0x04,
    OFC0 = 0x05,
    OFC1 = 0x06,
    OFC2 = 0x07,
    FSC0 = 0x08,
    FSC1 = 0x09,
    FSC2 = 0x0A,
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

/// The commands control the operation of the ADS1256.
/// CS must stay low during the entire command sequence.
/// See ADS1256 datasheet, Table 24.
pub enum Command {
    WAKEUP = 0x00,   // Completes SYNC and Exits Standby Mode
    RDATA = 0x01,    // Read Data
    RDATAC = 0x03,   // Read Data Continuously
    SDATAC = 0x0F,   // Stop Read Data Continuously
    RREG = 0x10,     // Read from REG
    WREG = 0x50,     // Write to REG
    SELFCAL = 0xF0,  // Offset and Gain Self-Calibration
    SELFOCAL = 0xF1, // Offset Self-Calibration
    SELFGCAL = 0xF2, // Gain Self-Calibration
    SYSOCAL = 0xF3,  // System Offset Calibration
    SYSGCAL = 0xF4,  // System Gain Calibration
    SYNC = 0xFC,     // Synchronize the A/D Conversion
    STANDBY = 0xFD,  // Begin Standby Mode
    RESET = 0xFE,    // Reset to Power-Up Values
}

impl Command {
    fn bits(self) -> u8 {
        self as u8
    }
}

///Programmable Gain Amplifier (pga) ads1256 datasheet, p. 16
#[derive(Debug, Copy, Clone)]
pub enum PGA {
    Gain1 = 0b000,
    Gain2 = 0b001,
    Gain4 = 0b010,
    Gain8 = 0b011,
    Gain16 = 0b100,
    Gain32 = 0b101,
    Gain64 = 0b110,
}

impl Default for PGA {
    fn default() -> Self {
        PGA::Gain1
    }
}

impl PGA {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn val(self) -> u8 {
        1 << self as u8
    }
}

//Sampling rate
#[derive(Debug, Copy, Clone)]
pub enum SamplingRate {
    Sps30000 = 0b1111_0000,
    Sps15000 = 0b1110_0000,
    Sps7500 = 0b1101_0000,
    Sps3750 = 0b1100_0000,
    Sps2000 = 0b1011_0000,
    Sps1000 = 0b1010_0001,
    Sps500 = 0b1001_0010,
    Sps100 = 0b1000_0010,
    Sps60 = 0b0111_0010,
    Sps50 = 0b0110_0011,
    Sps30 = 0b0101_0011,
    Sps25 = 0b0100_0011,
    Sps15 = 0b0011_0011,
    Sps10 = 0b0010_0011,
    Sps5 = 0b0001_0011,
    Sps2_5 = 0b0000_0011,
}

impl SamplingRate {
    fn bits(self) -> u8 {
        self as u8
    }
}

impl Default for SamplingRate {
    fn default() -> Self {
        SamplingRate::Sps1000
    }
}

//Channel
#[derive(Debug, Copy, Clone)]
pub enum Channel {
    AIN0 = 0,
    AIN1 = 1,
    AIN2 = 2,
    AIN3 = 3,
    AIN4 = 4,
    AIN5 = 5,
    AIN6 = 6,
    AIN7 = 7,
    AINCOM = 8,
}

impl Channel {
    fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Config {
    pub sampling_rate: SamplingRate,
    pub gain: PGA,
}

impl Config {
    pub fn new(sampling_rate: SamplingRate, gain: PGA) -> Self {
        Config {
            sampling_rate,
            gain,
        }
    }
}

#[derive(Debug)]
pub enum ADS1256Error {
    SpiError,
}
