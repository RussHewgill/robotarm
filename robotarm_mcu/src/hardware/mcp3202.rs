use defmt::{debug, error, info, warn};

use embassy_rp::{gpio::Output, spi::Instance};
use embedded_hal::spi::Operation;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Mcp3202Error {
    InvalidChannelNumber,
    Peripheral,
    UnsupportedChannel,
    UnsupportedDifferentialCombination,
}

pub struct Mt6816<T: Instance + 'static> {
    spi: embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<
        'static,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        // embassy_rp::peripherals::SPI0,
        // embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
        embassy_rp::spi::Spi<'static, T, embassy_rp::spi::Async>,
        Output<'static>,
    >,
}
