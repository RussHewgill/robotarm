use defmt::{debug, error, info, warn};

use embassy_rp::{gpio::Output, spi::Instance};
use embedded_hal::spi::Operation;

use crate::hardware::current_sensor::CurrentSensor;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Mcp3202Error {
    InvalidChannelNumber,
    Peripheral,
    UnsupportedChannel,
    UnsupportedDifferentialCombination,
}

#[derive(Copy, Clone, defmt::Format)]
enum Mode {
    Differential = 0,
    SingleEnded = 1,
}

#[derive(Copy, Clone)]
pub(crate) struct Resolution(pub(crate) u8);

impl Resolution {
    pub(crate) fn new(resolution: u8) -> Result<Self, Mcp3202Error> {
        if resolution == 0 || resolution >= 16 {
            unreachable!();
        }

        Ok(Self(resolution))
    }

    pub fn range(self) -> (u16, u16) {
        (0, 2u16.pow(u32::from(self.0)) - 1)
    }
}

#[derive(Copy, Clone)]
pub(crate) struct Channels(pub(crate) u8);

impl Channels {
    pub fn new(num: u8) -> Result<Self, Mcp3202Error> {
        match num {
            2 | 4 | 8 => Ok(Self(num)),
            _ => unreachable!(),
        }
    }

    pub fn bit_size(self) -> u8 {
        match self.0 {
            2 => 1,
            4 | 8 => 3,
            _ => unreachable!(),
        }
    }
}

pub struct Reading {
    value: u16,
    range: u16,
}

pub struct MT6816<T: Instance + 'static> {
    spi: embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<
        'static,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        // embassy_rp::peripherals::SPI0,
        // embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
        embassy_rp::spi::Spi<'static, T, embassy_rp::spi::Async>,
        Output<'static>,
    >,
    channels: Channels,
    resolution: Resolution,
}

pub fn mcp3202_test(address: u8) {
    let channels = Channels::new(2).unwrap();
    let resolution = Resolution::new(12).unwrap();

    let size = 1 + 1 + channels.bit_size() + 1 + 1 + resolution.0;
    let mode = Mode::SingleEnded;

    // let command: u32 = 1u32 << u32::from(size - 1)
    //     | ((mode as u32) << u32::from(size - 2))
    //     | ((u32::from(address)) << (resolution.0 + 2)) as u32;

    // let c0 = 1u32 << u32::from(size - 1);
    let c0 = 1u32;
    // let c1 = (mode as u32) << u32::from(size - 2);
    let c1 = mode as u32;
    // let c2 = (u32::from(address)) << (resolution.0 + 2);
    let c2 = u32::from(address);

    let c0 = 1u32;
    let c1 = mode as u32;
    // let c2 = address as

    // debug!("c0: 0b{:08b}", c0);
    // debug!("c1: 0b{:08b}", c1);
    // debug!("c2: 0b{:08b}", c2);

    // debug!(
    //     "MCP3202 Test: address={}, command=0x{:#X}, 0b{:b}",
    //     address, command, command
    // );
}

impl<T: Instance + 'static> MT6816<T> {
    pub fn new(
        mut spi: embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<
            'static,
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            // embassy_rp::peripherals::SPI0,
            // embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
            embassy_rp::spi::Spi<'static, T, embassy_rp::spi::Async>,
            Output<'static>,
        >,
        // channels: Channels,
        // resolution: Resolution,
    ) -> Self {
        use embassy_embedded_hal::SetConfig;

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 1_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleLow;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;

        spi.set_config(config);

        let channels = Channels::new(2).unwrap();
        let resolution = Resolution::new(12).unwrap();

        Self {
            spi,
            channels,
            resolution,
        }
    }

    // pub fn single_ended_read(&mut self, channel: Channel) -> Result<Reading> {
    //     self.check_channel_valid(channel)?;

    //     self.read(Mode::SingleEnded, channel.0)
    // }

    fn read(&mut self, mode: Mode, address: u8) -> Result<Reading, Mcp3202Error> {
        let start = 0b0000_0001;

        unimplemented!()
    }

    #[cfg(feature = "nope")]
    fn read(&mut self, mode: Mode, address: u8) -> Result<Reading, Mcp3202Error> {
        // START[1] + MODE[1] + ADDR[1/3] + SAMPLE[1] + NULL[1] + DATA[10-13]
        let size = 1 + 1 + self.channels.bit_size() + 1 + 1 + self.resolution.0;
        let bytes = (f32::from(size) / 8f32).ceil() as u8;

        let command: u32 = 1u32 << u32::from(size - 1)
            | ((mode as u32) << u32::from(size - 2))
            | ((u32::from(address)) << (self.resolution.0 + 2)) as u32;

        let mut tx: Vec<u8> = Vec::with_capacity(bytes as usize);
        for i in (0..bytes).rev() {
            let shift = u32::from(8u8 * i);
            tx.push(((command & (0b_1111_1111u32 << shift)) >> shift) as u8);
        }

        let mut rx: Vec<u8> = Vec::with_capacity(bytes as usize);
        for _ in 0..bytes {
            rx.push(0);
        }

        self.spi.transfer(&mut rx.as_mut_slice(), &tx.as_slice())?;

        let mut result: u32 = 0;
        for (i, byte) in rx.iter().enumerate() {
            result |= (u32::from(*byte) << (u32::from(bytes - 1 - i as u8) * 8)) as u32;
        }

        debug_assert_eq!(result >> u32::from(self.resolution.0), 0);

        Ok(Reading::new(result as u16, self.resolution.range().1))
    }
}

impl<T: Instance + 'static> CurrentSensor for MT6816<T> {
    type Error = ();

    async fn driver_align(
        &mut self,
        voltage: f32,
        modulation_centered: bool,
    ) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn init(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn prev_phase_currents(&self) -> Option<crate::simplefoc::types::PhaseCurrents> {
        unimplemented!()
    }
    fn prev_foc_currents(&self) -> Option<crate::simplefoc::types::DQCurrents> {
        unimplemented!()
    }
    fn prev_raw_currents(&self) -> (f32, f32) {
        unimplemented!()
    }

    fn set_prev_foc_currents(&mut self, currents: crate::simplefoc::types::DQCurrents) {
        unimplemented!()
    }
    async fn get_phase_currents(
        &mut self,
    ) -> Result<crate::simplefoc::types::PhaseCurrents, Self::Error> {
        unimplemented!()
    }
}
