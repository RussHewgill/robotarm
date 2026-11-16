use defmt::{debug, error, info};

use embassy_rp::{Peri, pio::StateMachine};
// use embedded_io_async::{Read, Write};

pub struct UartPio {
    buf: [u8; 1024],
    // serial: embassy_rp::uart::BufferedUart,
    tx: embassy_rp::pio_programs::uart::PioUartTx<'static, embassy_rp::peripherals::PIO0, 0>,
    rx: embassy_rp::pio_programs::uart::PioUartRx<'static, embassy_rp::peripherals::PIO0, 1>,
    rede_pin: embassy_rp::gpio::Output<'static>,
}

impl UartPio {
    pub fn new(
        mut common: &mut embassy_rp::pio::Common<'static, embassy_rp::peripherals::PIO0>,
        sm0: StateMachine<'static, embassy_rp::peripherals::PIO0, 0>,
        sm1: StateMachine<'static, embassy_rp::peripherals::PIO0, 1>,

        tx_pin: Peri<'static, impl embassy_rp::pio::PioPin>,
        rx_pin: Peri<'static, impl embassy_rp::pio::PioPin>,
        rede_pin: embassy_rp::gpio::Output<'static>,
        baud_rate: u32,
    ) -> Self {
        // let baud_rate = 115_200;

        let tx_program = embassy_rp::pio_programs::uart::PioUartTxProgram::new(common);
        let tx = embassy_rp::pio_programs::uart::PioUartTx::new(
            baud_rate,
            common,
            sm0,
            tx_pin,
            &tx_program,
        );

        let rx_program = embassy_rp::pio_programs::uart::PioUartRxProgram::new(&mut common);
        let mut rx = embassy_rp::pio_programs::uart::PioUartRx::new(
            baud_rate,
            &mut common,
            sm1,
            rx_pin,
            &rx_program,
        );

        Self {
            buf: [0; 1024],
            // serial,
            tx,
            rx,
            rede_pin,
        }
    }
}
