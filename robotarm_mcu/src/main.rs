#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unexpected_cfgs)]

mod comms;
mod configs;
mod hardware;
mod init;
mod simplefoc;

use defmt::{debug, error, info, trace, warn};
use static_cell::StaticCell;

// use rtt_target::{rprintln, rtt_init};
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{Spawner, raw};
use embassy_rp::{bind_interrupts, pwm::SetDutyCycle};
use embassy_time::{Instant, Ticker, Timer};

use crate::configs::*;
use crate::hardware::encoder_sensor::EncoderSensor;

// use crate::simplefoc::SimpleFOC;

#[cfg(feature = "testing")]
pub const MOTOR_ID_A: u8 = 0;
#[cfg(feature = "testing")]
pub const MOTOR_ID_B: u8 = 1;

#[cfg(feature = "picoA")]
pub const MOTOR_ID_A: u8 = 0;
#[cfg(feature = "picoA")]
pub const MOTOR_ID_B: u8 = 1;

#[cfg(feature = "picoB")]
pub const MOTOR_ID_A: u8 = 2;
#[cfg(feature = "picoB")]
pub const MOTOR_ID_B: u8 = 3;

#[cfg(feature = "picoC")]
pub const MOTOR_ID_A: u8 = 4;
#[cfg(feature = "picoC")]
pub const MOTOR_ID_B: u8 = 5;

const CONFIG_DATA: &str = include_str!("../configs/motors.ron");

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico on board LED, connected to gpio 25"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

use embassy_rp::peripherals as rpp;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<rpp::I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<rpp::I2C1>;
    // I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    // DMA_IRQ_0 => InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<rpp::USB>;
    DMA_IRQ_0 =>
        embassy_rp::dma::InterruptHandler<rpp::DMA_CH0>,
        embassy_rp::dma::InterruptHandler<rpp::DMA_CH1>,
        embassy_rp::dma::InterruptHandler<rpp::DMA_CH2>,
        embassy_rp::dma::InterruptHandler<rpp::DMA_CH3>,
        embassy_rp::dma::InterruptHandler<rpp::DMA_CH4>;
    // DMA_IRQ_1 => embassy_rp::dma::InterruptHandler<rpp::DMA_CH2>, embassy_rp::dma::InterruptHandler<rpp::DMA_CH3>;
    // DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<rpp::DMA_CH4>;
    // UART0_IRQ => embassy_rp::uart::InterruptHandler<rpp::UART0>;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<rpp::UART0>;
});

/// rtt tests
// #[embassy_executor::main]
#[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // USB 2
    // #[cfg(feature = "nope")]
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

        // info!("set up encoder");
        // let encoder = crate::hardware::as5600::AS5600::new(i2c).await;
        let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

        let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        let mut usb = crate::comms::usb::UsbMonitor::init(&spawner, driver);

        // let (mut sender, mut receiver) = class.split();

        // let mut rx: [u8; 64] = [0; 64];
        // sender.wait_connection().await;

        // usb.wait_connection().await;

        // let mut n: f32 = 0.0;
        let mut n = 0;
        let mut x = 0;

        let mut data: [u8; 64] = [0; 64];

        loop {
            let now = Instant::now().as_micros();
            encoder._update(now).await.unwrap();

            let angle = encoder.get_angle();
            let velocity = encoder.get_velocity();

            debug!(
                "now: {} us, angle: {} rad, velocity: {} rad/s",
                now, angle, velocity
            );

            let msg = robotarm_protocol::SerialLogMessage::MotorData {
                id: x,
                timestamp: now,
                position: angle,
                angle: 0.,
                velocity,
                target_position: x as f32,
                target_velocity: 0.,
                motor_current: 1.,
                motor_voltage: (2., 3.),
            };

            x += 1;

            // usb.send(msg).await;
            usb.send_log_msg(msg);

            #[cfg(feature = "nope")]
            if n > 1_0 {
                n = 0;
                //
            } else {
                n += 1;
            }

            // let data = b"0.0\r\n";

            // usb.send(data).await;

            Timer::after(embassy_time::Duration::from_millis(1000)).await;
        }
    }

    // USB
    #[cfg(feature = "nope")]
    {
        // rprintln!("Hello, world!");

        // let mut config = embassy_rp::uart::Config::default();
        // config.baudrate = 115_200;
        // let mut uart = embassy_rp::uart::Uart::new_blocking(p.UART1, p.PIN_8, p.PIN_9, config);

        use static_cell::StaticCell;

        let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        // Create embassy-usb Config
        let config = {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("Embassy Serial Logger");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let mut class = {
            static STATE: StaticCell<embassy_usb::class::cdc_acm::State> = StaticCell::new();
            let state = STATE.init(embassy_usb::class::cdc_acm::State::new());
            embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64)
        };

        // Build the builder.
        let usb = builder.build();

        // Run the USB device.
        spawner.spawn(usb_task(usb)).unwrap();

        let (mut sender, mut receiver) = class.split();

        let mut rx: [u8; 64] = [0; 64];
        sender.wait_connection().await;

        loop {
            let data = b"Hello, world!";
            let _ = sender.write_packet(data).await;
            Timer::after(embassy_time::Duration::from_millis(500)).await;
        }

        #[cfg(feature = "nope")]
        loop {
            // uart.blocking_write("Hello World!\r\n".as_bytes()).unwrap();
            // rprintln!("Test 0");
            // rprintln!("Test 0");
            // defmt::println!("Test 0");
            // defmt::debug!("Test 1");
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
            loop {
                // let _ = echo(&mut class).await;
                // let data = b"Hello, world!";
                // let _ = class.write_packet(data).await;
                // info!("Wrote packet");
                Timer::after(embassy_time::Duration::from_millis(500)).await;
            }
        }
    }
}

// #[embassy_executor::main]
#[cfg(feature = "nope")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14; // purple
    let scl = p.PIN_15; // blue

    let mut i2c =
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;

    // let mut encoder = crate::hardware::as5600::AS5600::new(i2c);

    info!("looping");
    // #[cfg(feature = "nope")]
    loop {
        // info!("Tick");
        Timer::after(embassy_time::Duration::from_millis(50)).await;

        if let Ok(angle) = encoder.read_raw_angle().await {
            let angle = (angle as f32 / 16384_f32) * 2.0 * core::f32::consts::PI;
            // debug!("Raw angle: {}", angle);
            debug!("Angle: {}", angle);
        }

        // let angle = encoder.angle().await.unwrap_or(0.);
        // if let Err(e) = encoder.update(Instant::now().as_micros()).await {
        //     error!("Failed to read encoder");
        // } else {
        // }
    }
}

#[cfg(feature = "nope")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // PWM
    #[cfg(feature = "nope")]
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        // i2c_config.frequency = 400_000; // 400 kHz
        i2c_config.frequency = 1_000_000; // 1 MHz
        let i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);

        let mut sensor = as5600::asynch::As5600::new(i2c);

        let mut cfg = sensor.config().await.unwrap();
        debug!("AS5600 initial config: {:#}", cfg);

        // cfg.slow_filter = as5600::configuration::SlowFilterMode::X16;
        // // cfg.slow_filter = as5600::configuration::SlowFilterMode::X2;
        // cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::SlowFilterOnly;
        // // cfg.fast_filter_threshold = as5600::configuration::FastFilterThreshold::Lsb10;

        cfg.output_stage = as5600::configuration::OutputStage::DigitalPwm;
        cfg.pwm_frequency = as5600::configuration::PwmFreq::PwmF4;

        sensor.set_config(cfg).await.unwrap();

        let mut cfg = sensor.config().await.unwrap();
        debug!("AS5600 initial config: {:#}", cfg);

        // let cfg: embassy_rp::pwm::Config = Default::default();
        // let pwm = embassy_rp::pwm::Pwm::new_input(
        //     p.PWM_SLICE2,
        //     p.PIN_5,
        //     gpio::Pull::None,
        //     embassy_rp::pwm::InputMode::RisingEdge,
        //     cfg,
        // );

        // let mut ticker = Ticker::every(embassy_time::Duration::from_millis(50));
        // for _ in 0..5 {
        //     ticker.next().await;
        //     info!("Input frequency: {} Hz", pwm.counter());
        // }

        //
    }

    // ADC
    #[cfg(feature = "nope")]
    {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};

        let mut adc = Adc::new(p.ADC, Irqs, Config::default());
        let mut dma = p.DMA_CH0;
        let mut pin = Channel::new_pin(p.PIN_26, gpio::Pull::Up);

        let mut sensor = crate::hardware::mt_6701_adc::MT6701::new(adc, dma, pin).await;

        let mut min = 10_000.0f32; // 8
        let mut max = 0.0f32; // 4095

        loop {
            Timer::after_millis(5).await;

            let sample = sensor.sample().await;

            debug!("Sample: {}", sample);

            // adc.read_many(&mut pin, &mut buf, div, dma.reborrow())
            //     .await
            //     .unwrap();

            // let avg = buf.iter().map(|&x| x as u32).sum::<u32>() as f32 / (buf.len() as f32);

            // min = min.min(avg);
            // max = max.max(avg);

            // debug!("ADC: {}, min: {}, max: {}", avg, min, max);
        }
    }

    #[cfg(feature = "nope")]
    {
        let sda = p.PIN_14; // purple
        let scl = p.PIN_15; // blue

        // info!("set up i2c ");
        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        // let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);
        let mut i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C1, scl, sda, i2c_config);
        // let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c).await;
        let address: u8 = 0b0000110;
        for _ in 0..1 {
            let mut buf = [0u8; 1];
            if let Err(e) = i2c.write_read(address, &[0x0E], &mut buf) {
                error!("I2C error: {:?}", e);
            } else {
                // debug!("Raw angle: {}", u16::from_be_bytes([0, buf[0]]));
            }
            // Timer::after(embassy_time::Duration::from_millis(100)).await;

            // 0x03 = angle[13:6]
            // 0x04 = angle[5:0]

            let angle = ((buf[0] as u16) << 6) & (buf[1] as u16 & 0b00111111);
        }
    }

    // info!("Done");
}

// RS485 tests
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 16])[..];
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 16])[..];

    let tx_dma = p.DMA_CH0;
    let rx_dma = p.DMA_CH1;

    let mut config = embassy_rp::uart::Config::default();
    config.baudrate = 115_200;
    // config.baudrate = 9600;

    // let uart =
    //     embassy_rp::uart::Uart::new(p.UART0, p.PIN_16, p.PIN_17, Irqs, tx_dma, rx_dma, config);

    // let uart = embassy_rp::uart::Uart::new_blocking(p.UART0, p.PIN_16, p.PIN_17, config);

    let mut uart = embassy_rp::uart::BufferedUart::new(
        p.UART0, p.PIN_16, p.PIN_17, Irqs, tx_buf, rx_buf, config,
    );

    let mut enable = embassy_rp::gpio::Output::new(p.PIN_18, embassy_rp::gpio::Level::Low);
    // let mut enable = embassy_rp::gpio::Output::new(p.PIN_18, embassy_rp::gpio::Level::High);

    // let mut tx = max485_async::Max485::new(uart, enable, embassy_time::Delay);
    // let mut tx = crate::comms::rs485::Max485::new(uart, enable, embassy_time::Delay);

    let mut tx = crate::comms::rs485::Max485::new(uart, enable);

    // use embedded_io_async::Write;

    // tx.flush();

    let mut x = 0;

    // let (mut tx, rx) = uart.split();

    // use embedded_io_async::{Read, Write};

    // let mut buf = [0u8; 16];

    // let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
    let mut data: [u8; 16] = [0; 16];

    // let data = postcard::to_slice_cobs(&[0xDE, 0xAD, 0xBE, 0xEF], &mut data).unwrap();
    // debug!("Encoded message {}: {:?}", data.len(), data);

    loop {
        // debug!("Sending: {}", x);
        // tx.send(&[x]).await.unwrap();

        // let msg = robotarm_protocol::SerialLogMessage::MotorData {
        //     id: x,
        //     timestamp: Instant::now().as_micros(),
        //     position: 0.,
        //     angle: 0.,
        //     velocity: 0.,
        //     target_position: 0.,
        //     target_velocity: 0.,
        //     motor_current: 0.,
        //     motor_voltage: (0., 0.),
        // };

        let msg: [u8; 4] = [x, x + 1, x + 2, x + 3];

        let msg = postcard::to_slice_cobs(&msg, &mut data).unwrap();

        debug!("Sending");
        // tx.send(msg).await.unwrap();
        tx._send(&msg).await.unwrap();

        // let n = tx.receive(&mut buf).await;

        // if n > 0 {
        //     debug!("Received {}: {:?}", n, &buf[..n]);
        // }

        x += 1;
        Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}

/// encoder tests
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    #[cfg(feature = "nope")]
    let mut encoder = {
        let miso = p.PIN_16;
        // let mosi = p.PIN_19;

        let sck = p.PIN_18;
        let cs = p.PIN_17;

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI0, sck, miso, p.DMA_CH0, p.DMA_CH1, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder = crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    #[cfg(feature = "nope")]
    let mut encoder = {
        let miso = p.PIN_12;
        // let mosi = p.PIN_15;

        let sck = p.PIN_14;
        let cs = p.PIN_13;

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 1_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI1, sck, miso, p.DMA_CH2, p.DMA_CH3, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder = crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    #[cfg(feature = "nope")]
    let mut encoder = {
        let miso = p.PIN_20;
        // let mosi = p.PIN_19;

        let sck = p.PIN_18;
        // let cs = p.PIN_17;
        let cs = p.PIN_21;

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI0, sck, miso, p.DMA_CH0, p.DMA_CH1, Irqs, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder = crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    // output encoder
    #[cfg(feature = "nope")]
    let mut encoder = {
        let sda = p.PIN_16;
        let scl = p.PIN_17;

        let mut i2c_config = embassy_rp::i2c::Config::default();
        // i2c_config.frequency = 50_000; // 400 kHz
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);
        // let mut i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, scl, sda, i2c_config);

        let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c);

        encoder
    };

    let mut encoder1 = {
        let cs = p.PIN_21; // Z, yellow, orange
        let miso = p.PIN_20; // SDA, brown, brown
        let sck = p.PIN_18; // SCL, blue, red

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;

        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI0, sck, miso, p.DMA_CH0, p.DMA_CH1, Irqs, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder = crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        // let mut encoder: hardware::mt_6701_ssi::MT6701<embassy_rp::spi::Spi<'static, _, _>> =
        //     crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    let time_limit = 0.2;
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    let mut x = 0.;
    let mut c = 0;
    let mut t0 = Instant::now();
    let mut n = 0;
    let mut angle_prev = 0;
    loop {
        // encoder1.update(Instant::now().as_micros()).await.unwrap();
        // let angle = encoder1.get_angle();
        let angle = encoder1.read_raw_angle().await.unwrap();
        // let angle = encoder.read_raw_angle().await.unwrap();

        let t1 = Instant::now();
        // #[cfg(feature = "nope")]
        if t1 > max_time {
            let elapsed = t1 - t0;
            let freq = c as f32 / (elapsed.as_micros() as f32 * 1e-6);
            info!(
                "Elapsed: {} s, Cycles: {}, Freq: {} Hz",
                elapsed.as_millis() as f32 * 1e-3,
                c,
                freq
            );
            t0 = t1;
            c = 0;
            max_time = t1 + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

            debug!("angle: {}", angle);
        } else {
            c += 1;
        }

        // let angle = encoder.read_raw_angle_debug().await.unwrap();

        // debug!("Angle: {}", angle);

        // Timer::after(embassy_time::Duration::from_millis(100)).await;
    }

    #[cfg(feature = "nope")]
    for _ in 0..10 {
        let update_rate_hz = 1_000_000;
        let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
            1_000_000 / update_rate_hz,
        ));

        let mut c = 0;
        let mut sum = 0u64;

        // i2c read frequency test
        for _ in 0..1_000 {
            // ticker.next().await;

            let t0 = Instant::now();
            // let angle = foc.encoder.read_raw_angle().await.unwrap();
            // let angle = encoder.read_raw_angle().await.unwrap();
            let _ = encoder._update(t0.as_micros()).await;

            let t1 = Instant::now();
            let elapsed = t1 - t0;
            sum += elapsed.as_micros() as u64;

            c += 1;
        }

        let avg = sum as f32 / c as f32;
        info!("Average SSI read time: {} us", avg);
    }

    #[cfg(feature = "nope")]
    {
        let sda = p.PIN_20;
        let scl = p.PIN_21;

        let mut i2c_config = embassy_rp::i2c::Config::default();
        // i2c_config.frequency = 400_000;
        i2c_config.frequency = 1_000_000;

        let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

        let address0 = 0b1000000;
        let address1 = 0b1000001;
        let mut sensor = crate::hardware::ina226::INA226::new(i2c, address0, address1);

        hardware::current_sensor::CurrentSensor::init(&mut sensor)
            .await
            .unwrap();

        let c = sensor.configuration0().await.unwrap();
        debug!("INA226 config: {:#?}", c);

        for _ in 0..5 {
            let mut c = 0;
            let mut sum = 0u64;

            // i2c read frequency test
            for _ in 0..1_000 {
                // ticker.next().await;

                let t0 = Instant::now();
                // let angle = foc.encoder.read_raw_angle().await.unwrap();
                // let angle = encoder.read_raw_angle().await.unwrap();
                // let _ = encoder._update(t0.as_micros()).await;
                hardware::current_sensor::CurrentSensor::get_phase_currents(&mut sensor)
                    .await
                    .unwrap();

                let t1 = Instant::now();
                let elapsed = t1 - t0;
                sum += elapsed.as_micros() as u64;

                c += 1;
            }

            let avg = sum as f32 / c as f32;
            info!("Average I2C read time: {} us", avg);
        }

        #[cfg(feature = "nope")]
        loop {
            let b = sensor.bus_voltage_millivolts0().await.unwrap();
            debug!("Bus 0 voltage: {} mV", b);

            let c = sensor.current_amps0().await.unwrap();
            debug!("Current 0: {} mA", c);

            let b = sensor.bus_voltage_millivolts1().await.unwrap();
            debug!("Bus 1 voltage: {} mV", b);

            let c = sensor.current_amps1().await.unwrap();
            debug!("Current 1: {} mA", c);

            // Timer::after(embassy_time::Duration::from_millis(1000)).await;
        }
    }
}

/// ADS1256 test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let miso = p.PIN_12;
    let mosi = p.PIN_15;

    let sck = p.PIN_14;
    let cs = p.PIN_13;

    let mut config = embassy_rp::spi::Config::default();
    // config.frequency = 1_000_000;
    config.frequency = 400_000;
    config.polarity = embassy_rp::spi::Polarity::IdleLow;
    config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

    let mut spi =
            // embassy_rp::spi::Spi::new_rxonly(p.SPI1, sck, miso, p.DMA_CH2, p.DMA_CH3, config);
            embassy_rp::spi::Spi::new(p.SPI1, sck, mosi, miso, p.DMA_CH2, p.DMA_CH3, config);

    debug!("SPI initialized");

    let cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);
    let mut reset = embassy_rp::gpio::Output::new(p.PIN_10, embassy_rp::gpio::Level::Low);
    let data_ready_pin = embassy_rp::gpio::Input::new(p.PIN_9, embassy_rp::gpio::Pull::Up);

    let config = crate::hardware::ads1256::Config {
        gain: crate::hardware::ads1256::PGA::Gain64,
        sampling_rate: crate::hardware::ads1256::SamplingRate::Sps10,
    };

    let mut sensor = crate::hardware::ads1256::ADS1256::new(spi, cs, reset, data_ready_pin, config);

    // sensor.test_init().await.unwrap();
    sensor.init().await.unwrap();
    debug!("ADS1256 initialized");

    loop {
        let data = sensor
            .read_channel(
                hardware::ads1256::Channel::AIN0,
                hardware::ads1256::Channel::AIN1,
            )
            .await
            .unwrap();
        // sensor.test_read().await.unwrap();

        debug!("Read data: {}", data);

        // Timer::after_millis(1000).await;

        // debug!("Setting low");
        // reset.set_low();
        // Timer::after_millis(1).await;
        // debug!("Setting high");
        // reset.set_high();
        // Timer::after_millis(1).await;
    }

    //
}

/// INA240 test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
    use embassy_rp::gpio::Pull;

    let mut adc = Adc::new(p.ADC, Irqs, Config::default());
    let mut dma = p.DMA_CH0;
    let mut pin0 = Channel::new_pin(p.PIN_27, Pull::Up);
    let mut pin1 = Channel::new_pin(p.PIN_28, Pull::Up);

    // Peri<'_, impl dma::Channel>
    // adc.read_many(&mut pin, &mut buf, div, dma.reborrow()).await.unwrap();

    let mut sensor = crate::hardware::ina240::INA240::new(pin0, pin1, adc, dma);

    // sensor.read_voltage().await.unwrap();

    loop {
        let (v0, v1) = sensor.read_voltage().await;

        // debug!("Voltage 0: {} mV", v0);
        // debug!("Voltage 1: {} mV", v1);

        Timer::after_millis(100).await;
        //
    }

    //
}

/// INA226 test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    use hardware::current_sensor::CurrentSensor;

    // let sda = p.PIN_16;
    // let scl = p.PIN_17;

    // let mut i2c_config = embassy_rp::i2c::Config::default();
    // i2c_config.frequency = 1_000_000;

    // let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

    // let address0 = 0b1000000;
    // let address1 = 0b1000001;

    // #[cfg(feature = "nope")]
    let mut current_sensor = {
        let sda = p.PIN_16;
        let scl = p.PIN_17;

        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 1_000_000;

        let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

        let address0 = 0b1000000;
        let address1 = 0b1000001;
        crate::hardware::ina226::INA226::new(i2c, address0, address1)
    };

    current_sensor.init().await.unwrap();

    // #[cfg(feature = "nope")]
    loop {
        let currents = current_sensor.get_phase_currents().await.unwrap();

        match currents {
            simplefoc::types::PhaseCurrents::Two { a, b } => {
                debug!("Current 0: {} mA", a);
                debug!("Current 1: {} mA", b);
            }
            _ => {
                error!("Failed to read currents");
            }
        }

        Timer::after_millis(100).await;
    }

    //
}

/// AS5048A test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // green    miso    12  yellow
    // white    cs      13  orange
    // blue     clk     14  red
    // yellow   mosi    15  brown

    let miso = p.PIN_12;
    let mosi = p.PIN_15;

    let sck = p.PIN_14;
    let cs = p.PIN_13;

    let cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::High);

    let mut config = embassy_rp::spi::Config::default();
    // config.frequency = 1_000_000;
    // config.frequency = 8_000_000;
    config.frequency = 400_000;
    // config.frequency = 100_000;
    // config.polarity = embassy_rp::spi::Polarity::IdleLow;
    // config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

    config.polarity = embassy_rp::spi::Polarity::IdleLow;
    // config.polarity = embassy_rp::spi::Polarity::IdleHigh;
    config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;

    let mut spi =
            // embassy_rp::spi::Spi::new_rxonly(p.SPI1, sck, miso, p.DMA_CH2, p.DMA_CH3, config);
            embassy_rp::spi::Spi::new(p.SPI1, sck, mosi, miso, p.DMA_CH2, p.DMA_CH3, Irqs, config);

    debug!("SPI initialized");

    // let mut spi =
    //     embassy_sync::mutex::Mutex::<embassy_sync::blocking_mutex::raw::NoopRawMutex, _>::new(spi);
    // let mut spi = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi, cs);
    // // let mut encoder = as5048a_spi::As5048a::new(spi);

    // let mut encoder = crate::hardware::as5048a::As5048a::new(spi);
    let mut encoder = crate::hardware::as5048a::As5048a::new(spi, cs);

    debug!("Waiting");
    Timer::after(embassy_time::Duration::from_millis(1000)).await;
    debug!("Done");

    // if let Err(e) = encoder.clear_error_flag().await {
    //     error!("Failed to clear error flag: {:?}", e);
    // } else {
    //     debug!("Cleared error flag");
    // }

    // match encoder.angle().await {
    //     Ok(angle) => {
    //         debug!("Angle: {}", angle);
    //     }
    //     Err(e) => {
    //         error!("Failed to read angle: {:?}", e);
    //     }
    // }

    // for _ in 0..10 {
    loop {
        // let angle = encoder.read_raw_angle().await.unwrap();

        #[cfg(feature = "nope")]
        match encoder.read_diagnostics_async().await {
            Ok(as5048a_spi::Diagnostics {
                agc,
                compensation_too_high,
                compensation_too_low,
                cordic_overflow,
                offset_compensation_finished,
            }) => {
                debug!(
                    "Diagnostics: agc: {}, compensation_too_high: {}, compensation_too_low: {}, cordic_overflow: {}, offset_comp_finished: {}",
                    agc,
                    compensation_too_high,
                    compensation_too_low,
                    cordic_overflow,
                    offset_compensation_finished
                );
            }
            Err(e) => {
                // error!("Failed to read diagnostics: {:?}", e);
                error!("Failed to read diagnostics");
            }
        }

        #[cfg(feature = "nope")]
        match encoder.diagnostics().await {
            Ok(diag) => {
                // debug!(
                //     "comp_high: {}, comp_low: {}, cordic_overflow: {}, offset_compensation: {}",
                //     diag.comp_high(),
                //     diag.comp_low(),
                //     diag.cordic_overflow(),
                //     diag.offset_comp_finished(),
                // );

                // debug!("agc_value: {}", diag.agc_value());
                // debug!("is_valid: {}", diag.is_valid());
            }
            Err(e) => {
                error!("Failed to read diagnostics: {:?}", e);
            }
        }

        // let _ = encoder.clear_error_flag().await;

        // let angle = encoder.angle().await.unwrap();

        // encoder.send_noop().await.unwrap();

        // 0xF9F3
        // #[cfg(feature = "nope")]
        match encoder.angle().await {
            Ok(angle) => {
                debug!("Angle: {}", angle);
            }
            Err(e) => {
                error!("Failed to read angle: {:?}", e);
            }
        }

        // debug!("Angle: {}", angle);

        Timer::after(embassy_time::Duration::from_millis(5)).await;
        // Timer::after(embassy_time::Duration::from_millis(500)).await;
    }

    //
}

/// ACS712 test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
    use embassy_rp::gpio::Pull;

    let mut adc = Adc::new(p.ADC, Irqs, Config::default());
    // let mut dma = p.DMA_CH4;
    let mut dma = embassy_rp::dma::Channel::new(p.DMA_CH4, Irqs);
    let mut pin0 = Channel::new_pin(p.PIN_26, Pull::Up);
    let mut pin1 = Channel::new_pin(p.PIN_27, Pull::Up);
    // let mut pin0 = Channel::new_pin(p.PIN_26, Pull::None);
    // let mut pin1 = Channel::new_pin(p.PIN_27, Pull::None);

    let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // Peri<'_, impl dma::Channel>
    // adc.read_many(&mut pin, &mut buf, div, dma.reborrow()).await.unwrap();
    // adc.read_many(&mut pin0, &mut buf, div, &mut dma)
    //     .await
    //     .unwrap();

    let mut sensor = crate::hardware::acs712::ACS712::new(pin0, pin1, adc, dma);

    sensor.calibrate().await;

    use crate::hardware::current_sensor::CurrentSensor;

    let mut t0 = Instant::now();
    let mut c = 0;
    let interval = embassy_time::Duration::from_millis(1000);
    let mut max_time = t0 + interval;

    #[cfg(feature = "nope")]
    {
        const N: usize = 32;

        let mut sum0 = 0f32;
        let mut sum1 = 0f32;

        debug!("div = 4799, 10 kHz");
        debug!("");
        sensor.div = 4799;
        for i in 0..N {
            // let (n0, n1) = sensor.test_noise().await;
            let (n0, n1) = sensor.test_noise_interleaved().await;
            sum0 += n0;
            sum1 += n1;
            Timer::after_millis(10).await;
        }

        let avg0 = sum0 / N as f32;
        let avg1 = sum1 / N as f32;

        debug!("avg0 = {}", avg0);
        debug!("avg1 = {}", avg1);

        Timer::after_millis(10).await;

        let mut sum0 = 0f32;
        let mut sum1 = 0f32;

        debug!("");
        debug!("div = 479, 100 kHz");
        debug!("");
        sensor.div = 479;
        for i in 0..N {
            // let (n0, n1) = sensor.test_noise().await;
            let (n0, n1) = sensor.test_noise_interleaved().await;
            sum0 += n0;
            sum1 += n1;
            Timer::after_millis(10).await;
        }

        let avg0 = sum0 / N as f32;
        let avg1 = sum1 / N as f32;

        debug!("avg0 = {}", avg0);
        debug!("avg1 = {}", avg1);

        debug!("");
        // debug!("div = 95, 500 kHz");
        debug!("div = 191, 250 kHz");
        debug!("");
        // sensor.div = 95;
        // sensor.div = 191;
        // sensor.div = 238; // 200 kHz
        sensor.div = 435;
        for i in 0..N {
            // let (n0, n1) = sensor.test_noise().await;
            let (n0, n1) = sensor.test_noise_interleaved().await;
            sum0 += n0;
            sum1 += n1;
            Timer::after_millis(10).await;
        }

        let avg0 = sum0 / N as f32;
        let avg1 = sum1 / N as f32;

        debug!("avg0 = {}", avg0);
        debug!("avg1 = {}", avg1);
    }

    loop {
        let currents = sensor.get_phase_currents().await.unwrap();

        debug!("Phase currents: {:?}", currents);

        let ab_currents = sensor.get_ab_currents(currents).await;

        debug!("AB currents: {:?}", ab_currents);

        let electrical_angle = 0.0;
        let dq_currents = sensor.get_dq_currents(ab_currents, electrical_angle).await;

        debug!("DQ currents: {:?}", dq_currents);

        Timer::after_millis(1000).await;
    }

    #[cfg(feature = "nope")]
    loop {
        let current = sensor.read_current().await;
        // debug!("Current: {} A, {} A", current.0, current.1);

        let t1 = Instant::now();

        if t1 > max_time {
            let elapsed = t1 - t0;
            let freq = c as f32 / (elapsed.as_micros() as f32 * 1e-6);
            info!(
                "Elapsed: {}s, Cycles: {}, Freq: {}Hz",
                elapsed.as_millis() as f32 * 1e-3,
                c,
                freq
            );
            t0 = t1;
            c = 0;
            max_time = t1 + interval;
        } else {
            c += 1;
        }

        // Timer::after_millis(1000).await;
    }

    #[cfg(feature = "nope")]
    loop {
        // let v = sensor.read_voltage().await;
        // debug!("Voltage: {} mV", v);

        adc.read_many(&mut pin0, &mut buf, div, &mut dma)
            .await
            .unwrap();

        // for i in 0..BLOCK_SIZE {
        //     debug!("ADC[{}]: {}", i, buf[i]);
        // }

        let sum0 = buf.iter().map(|&x| x as u32).sum::<u32>();
        let avg0 = sum0 as f32 / (BLOCK_SIZE as f32);

        // let temp = 27.0 - (raw_temp * 3.3 / 4096.0 - 0.706) / 0.001721;

        let sensitivity = 185.; // mV/A

        let vref = 3.2;
        // let offset = vref / 2. - 55.;
        let offset = 0.;

        let voltage = avg0 * 3.3 / 4096.0;
        debug!("Voltage: {} V", voltage);
        let current0 = (voltage - offset) / sensitivity;
        debug!("Current: {} A", current0);

        // let temp = adc.read(&mut ts).await.unwrap() as f32;
        // debug!("Raw temp: {}", temp);
        // info!("Temp: {} degrees", convert_to_celsius(temp));

        Timer::after_millis(1000).await;
    }
}

/// USB raw bulk test
// #[cortex_m_rt::entry]
#[cfg(feature = "nope")]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    debug!("Starting USB raw bulk test");

    // Create the driver, from the HAL.
    let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

    // // Create embassy-usb Config
    // let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    // config.manufacturer = Some("Embassy");
    // config.product = Some("USB raw example");
    // config.serial_number = Some("12345678");
    // config.max_power = 100;
    // config.max_packet_size_0 = 64;

    // // Create embassy-usb DeviceBuilder using the driver and config.
    // // It needs some buffers for building the descriptors.
    // let mut config_descriptor = [0; 256];
    // let mut bos_descriptor = [0; 256];
    // let mut msos_descriptor = [0; 256];
    // let mut control_buf = [0; 64];

    // let mut builder = embassy_usb::Builder::new(
    //     driver,
    //     config,
    //     &mut config_descriptor,zM
    //     &mut bos_descriptor,
    //     &mut msos_descriptor,
    //     &mut control_buf,
    // );

    // const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

    // // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // // which causes it to use the built-in WinUSB driver automatically, which in turn
    // // can be used by libusb/rusb software without needing a custom driver or INF file.
    // // In principle you might want to call msos_feature() just on a specific function,
    // // if your device also has other functions that still use standard class drivers.
    // builder.msos_descriptor(embassy_usb::msos::windows_version::WIN8_1, 0);
    // builder.msos_feature(embassy_usb::msos::CompatibleIdFeatureDescriptor::new(
    //     "WINUSB", "",
    // ));
    // builder.msos_feature(embassy_usb::msos::RegistryPropertyFeatureDescriptor::new(
    //     "DeviceInterfaceGUIDs",
    //     embassy_usb::msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    // ));

    // // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // // that uses our custom handler.
    // let mut function = builder.function(0xFF, 0, 0);
    // let mut interface = function.interface();
    // let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    // let mut read_ep = alt.endpoint_bulk_out(None, 64);
    // // let mut write_ep = alt.endpoint_bulk_in(None, 64);
    // let mut write_ep = alt.endpoint_bulk_in(None, 64);
    // drop(function);

    // // debug!("bulk out address: {}", read_ep.info());
    // // debug!("bulk in address: {}", write_ep.info());

    // // Build the builder.
    // let mut usb = builder.build();

    // // Run the USB device.
    // let usb_fut = usb.run();

    // use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};

    // // Do stuff with the class!
    // let echo_fut = async {
    //     loop {
    //         read_ep.wait_enabled().await;
    //         info!("Connected");
    //         loop {
    //             let mut data = [0; 64];
    //             match read_ep.read(&mut data).await {
    //                 Ok(n) => {
    //                     info!("Got bulk: {:a}", data[..n]);
    //                     // Echo back to the host:
    //                     write_ep.write(&data[..n]).await.ok();
    //                 }
    //                 Err(_) => break,
    //             }
    //         }
    //         info!("Disconnected");
    //     }
    // };

    // // Run everything concurrently.
    // // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    // embassy_futures::join::join(usb_fut, echo_fut).await;

    // #[cfg(feature = "nope")]
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(init::CORE1_STACK) },
        move || {
            let executor1 = init::EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner| {
                // spawner.spawn(crate::comms::usb_raw::usb_test_task().unwrap());
                crate::comms::usb_raw::usb_init(&spawner, driver);
                //
            });
        },
    );

    let executor0 = init::EXECUTOR0.init(embassy_executor::Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(crate::comms::usb_raw::usb_test_task().unwrap());
        // crate::comms::usb_raw::usb_init(&spawner, driver);
        //
    });
}

// ADRC test
#[cfg(feature = "nope")]
// #[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    use approx::assert_relative_eq;

    use crate::simplefoc::algorithms::adrc::*;

    debug!("Starting tests");

    #[cfg(feature = "nope")]
    {
        {
            fn fal_is_continuous_at_delta() {
                let alpha = 0.5;
                let delta = 0.1;
                let just_above = fal(delta + 1e-4, alpha, delta);
                let just_below = fal(delta - 1e-4, alpha, delta);
                assert_relative_eq!(just_above, just_below, epsilon = 1e-3);
            }

            fn fal_is_odd() {
                let alpha = 0.5;
                let delta = 0.1;
                for &e in &[0.02_f32, 0.2, 1.5] {
                    assert_relative_eq!(
                        fal(-e, alpha, delta),
                        -fal(e, alpha, delta),
                        epsilon = 1e-6
                    );
                }
            }

            fn fal_linear_region_matches_slope() {
                // Inside |e| <= delta, fal(e) = e * delta^(alpha - 1)
                let alpha = 0.5;
                let delta = 0.1;
                let e = 0.05;
                let expected = e * libm::powf(delta, alpha - 1.0);
                assert_relative_eq!(fal(e, alpha, delta), expected, epsilon = 1e-6);
            }

            fal_is_continuous_at_delta();
            fal_is_odd();
            fal_linear_region_matches_slope();
        }
        debug!("fal test passed");

        {
            assert_relative_eq!(fhan(0.0, 0.0, 10.0, 0.01), 0.0, epsilon = 1e-6);

            let r = 5.0;
            for &(x1, x2) in &[(10.0, 0.0), (-10.0, 3.0), (0.5, -20.0), (100.0, 100.0)] {
                let u = fhan(x1, x2, r, 0.01);
                assert!(u.abs() <= r + 1e-3, "u={u} exceeds r={r}");
            }

            // Positive error, zero velocity -> should command negative (decelerating/reversing) control.
            assert!(fhan(5.0, 0.0, 1.0, 0.01) < 0.0);
            // Negative error, zero velocity -> should command positive control.
            assert!(fhan(-5.0, 0.0, 1.0, 0.01) > 0.0);

            // linear_mode_matches_pd
            let nlsef = NonlinearStateErrorFeedback::linear(3.0, 0.5);
            let e1 = 0.4;
            let e2 = -0.2;
            assert_relative_eq!(nlsef.compute(e1, e2), 3.0 * e1 + 0.5 * e2, epsilon = 1e-6);

            // zero_error_gives_zero_output
            let nlsef = NonlinearStateErrorFeedback::new(10.0, 2.0, 0.5, 0.25, 0.01);
            assert_relative_eq!(nlsef.compute(0.0, 0.0), 0.0, epsilon = 1e-6);
        }
        debug!("NLESF test passed");

        {
            fn tracks_a_step_input_without_overshoot_blowup() {
                let mut td = TrackingDifferentiator::new(50.0, 0.01);
                let dt = 0.01;
                let mut last_v1 = 0.0;
                for _ in 0..500 {
                    let (v1, _v2) = td.update(1.0, dt);
                    last_v1 = v1;
                }
                assert!(
                    (last_v1 - 1.0).abs() < 0.01,
                    "did not converge: v1={last_v1}"
                );
            }

            fn derivative_of_ramp_converges_to_its_slope() {
                let mut td = TrackingDifferentiator::new(200.0, 0.005);
                let dt = 0.005;
                let slope = 2.0;
                let mut t = 0.0;
                let mut v2 = 0.0;
                for _ in 0..2000 {
                    t += dt;
                    let (_v1, v2_now) = td.update(slope * t, dt);
                    v2 = v2_now;
                }
                assert!((v2 - slope).abs() < 0.05, "v2={v2}, expected ~{slope}");
            }

            tracks_a_step_input_without_overshoot_blowup();
            derivative_of_ramp_converges_to_its_slope();
        }
        debug!("TD test passed");

        {
            fn leso_estimates_state_and_constant_disturbance() {
                let b0 = 1.0;
                let mut eso = ExtendedStateObserver::<3, 1, 1>::from_bandwidth(30.0, b0);

                let dt = 0.001;
                let u = 0.5;
                let w = 2.0; // true constant disturbance
                let mut x1 = 0.0_f32;
                let mut x2 = 0.0_f32;

                for _ in 0..20_000 {
                    // True plant: x1' = x2, x2' = u + w
                    x1 += dt * x2;
                    x2 += dt * (u + w);
                    eso.update(x1, u, dt);
                }

                assert!((eso.z[0] - x1).abs() < 0.05, "z1={}, x1={}", eso.z[0], x1);
                assert!((eso.z[1] - x2).abs() < 0.2, "z2={}, x2={}", eso.z[1], x2);
                assert!((eso.z[2] - w).abs() < 0.2, "z3={}, w={}", eso.z[2], w);
            }

            leso_estimates_state_and_constant_disturbance();
        }
        debug!("ESO test passed");

        {
            /// Closed-loop simulation: a double integrator plant with an unmodeled
            /// constant disturbance and a gain error in `b0`. ADRC should still
            /// track a step setpoint accurately despite both.
            fn rejects_disturbance_and_tracks_step() {
                let td = TrackingDifferentiator::new(20.0, 0.01);
                let eso = ExtendedStateObserver::from_bandwidth(30.0, 1.0);
                let nlsef = NonlinearStateErrorFeedback::new(25.0, 10.0, 0.5, 0.25, 0.05);
                let mut controller = Adrc::new(td, eso, nlsef, 1.0).with_limits(-50.0, 50.0);

                let dt = 0.001;
                // Plant: x1' = x2, x2' = 0.8*u + w  (b0 mismatch: true gain is 0.8, not 1.0)
                let true_b0 = 0.8;
                let w = 3.0; // constant external disturbance
                let mut x1 = 0.0_f32;
                let mut x2 = 0.0_f32;

                for _ in 0..5000 {
                    let u = controller.update(1.0, x1, dt);
                    x1 += dt * x2;
                    x2 += dt * (true_b0 * u + w);
                }

                assert!(
                    (x1 - 1.0).abs() < 0.02,
                    "steady-state error too large: x1={x1}"
                );
            }

            fn reset_clears_state() {
                let td = TrackingDifferentiator::new(50.0, 0.01);
                let eso = ExtendedStateObserver::from_bandwidth(20.0, 1.0);
                let nlsef = NonlinearStateErrorFeedback::linear(5.0, 1.0);
                let mut controller = Adrc::new(td, eso, nlsef, 1.0);

                for _ in 0..100 {
                    controller.update(1.0, 0.0, 0.01);
                }
                assert_ne!(
                    controller.state_estimate(),
                    nalgebra::SVector::<f32, 3>::zeros()
                );

                controller.reset();
                assert_eq!(
                    controller.state_estimate(),
                    nalgebra::SVector::<f32, 3>::zeros()
                );
                assert_eq!(controller.last_control(), 0.0);
            }
        }
        debug!("ADRC tests passed");
    }

    let mut adrc = motor_adrc::MotorADRC::new(100. * 1e-7, 0.45, 5.8, 0.01);

    adrc.test_adrc();

    debug!("Done");

    //
}

/// MARK: Main
// #[cfg(feature = "nope")]
#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // let config = Config::new(ClockConfig::system_freq(200_000_000).unwrap());
    // let p = embassy_rp::init(config);

    debug!(
        "Clock frequency: {} MHz",
        embassy_rp::clocks::clk_sys_freq() / 1_000_000
    );
    let core_voltage = embassy_rp::clocks::core_voltage().unwrap();
    info!("Core voltage: {}", core_voltage);

    debug!(
        "core voltage: {:?}",
        embassy_rp::clocks::ClockConfig::default().core_voltage
    );

    // let voltage_limit = 2.0;
    // let voltage_limit = 4.;
    let voltage_limit = 6.;
    // let voltage_limit = 8.;
    // let voltage_limit = 8.;
    // let voltage_limit = 10.;
    // let voltage_limit = 12.;
    // let voltage_limit = 18.;

    let supply_voltage = 12.0;
    // let supply_voltage = 16.0;
    // let supply_voltage = 20.0;

    // #[cfg(feature = "nope")]
    let encoder0 = {
        let miso = p.PIN_12;
        // let mosi = p.PIN_15;

        let sck = p.PIN_14;
        let cs = p.PIN_13;

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;
        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI1, sck, miso, p.DMA_CH2, p.DMA_CH3, Irqs, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder: hardware::mt_6701_ssi::MT6701<embassy_rp::spi::Spi<'static, _, _>> =
            crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    // #[cfg(feature = "nope")]
    let encoder1 = {
        let cs = p.PIN_21; // Z, yellow, orange
        let miso = p.PIN_20; // SDA, brown, brown
        let sck = p.PIN_18; // SCL, blue, red

        let mut config = embassy_rp::spi::Config::default();
        config.frequency = 4_000_000;

        config.polarity = embassy_rp::spi::Polarity::IdleHigh;
        config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
        // let mut spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, sck, mosi, miso, config);

        let mut spi =
            embassy_rp::spi::Spi::new_rxonly(p.SPI0, sck, miso, p.DMA_CH0, p.DMA_CH1, Irqs, config);

        // Configure CS
        let mut cs = embassy_rp::gpio::Output::new(cs, embassy_rp::gpio::Level::Low);

        // let mut buf: [u8; 4] = [0; 4];

        let mut encoder = crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        // let mut encoder: hardware::mt_6701_ssi::MT6701<embassy_rp::spi::Spi<'static, _, _>> =
        //     crate::hardware::mt_6701_ssi::MT6701::new(spi, cs);

        encoder
    };

    // #[cfg(feature = "nope")]
    let output_encoder0 = {
        let sda = p.PIN_16;
        let scl = p.PIN_17;

        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // 400 kHz
        // i2c_config.frequency = 1_000_000; // 1 MHz
        let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);
        // let mut i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, scl, sda, i2c_config);

        let mut encoder = crate::hardware::mt_6701::MT6701::new(i2c);

        encoder
    };

    #[cfg(feature = "nope")]
    let output_encoder0 = ();

    #[cfg(feature = "nope")]
    let current_sensor = {
        let sda = p.PIN_16;
        let scl = p.PIN_17;

        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 1_000_000;

        let mut i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

        let address0 = 0b1000000;
        let address1 = 0b1000001;
        crate::hardware::ina226::INA226::new(i2c, address0, address1)
    };

    // #[cfg(feature = "nope")]
    let current_sensor = {
        use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
        use embassy_rp::gpio::Pull;

        let mut adc = Adc::new(p.ADC, Irqs, Config::default());
        let mut dma = embassy_rp::dma::Channel::new(p.DMA_CH4, Irqs);
        // let mut pin0 = Channel::new_pin(p.PIN_26, Pull::Up);
        // let mut pin1 = Channel::new_pin(p.PIN_27, Pull::Up);
        let mut pin0 = Channel::new_pin(p.PIN_26, Pull::None);
        let mut pin1 = Channel::new_pin(p.PIN_27, Pull::None);

        // Peri<'_, impl dma::Channel>
        // adc.read_many(&mut pin, &mut buf, div, dma.reborrow()).await.unwrap();

        let mut sensor = crate::hardware::acs712::ACS712::new(pin0, pin1, adc, dma);

        sensor
    };

    // simpleFOCShield
    // PWM pins:
    // pico     shield
    // 7        10       white
    // 8        6      black
    // 9        5       brown
    // en: 10   8       red

    // #[cfg(feature = "nope")]
    let (pwm_driver0, pwm_driver1) = {
        let mut c = embassy_rp::pwm::Config::default();
        // let desired_freq_hz = 24_000 * 1;
        // let desired_freq_hz = 24_000 * 2;
        let desired_freq_hz = 24_000 * 2 * 2;
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

        let div = 1;
        let period = (clock_freq_hz / (desired_freq_hz * div as u32)) as u16 - 1;

        c.top = period;
        c.divider = div.into();
        c.phase_correct = true;

        // c.invert_a = true;
        // c.invert_b = true;

        // debug!("PWM top: {}", c.top);
        // debug!("PWM divider: {}", c.divider);
        // debug!("PWM phase_correct: {}", c.phase_correct);

        let pwm0 = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
        let pwm12 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, c.clone());

        let enable_pin0 = embassy_rp::gpio::Output::new(p.PIN_6, embassy_rp::gpio::Level::Low);
        let driver0: simplefoc::pwm_driver::PWMDriver<'static> =
            crate::simplefoc::pwm_driver::PWMDriver::new(
                pwm0,
                pwm12,
                enable_pin0,
                c.clone(),
                voltage_limit,
                supply_voltage,
            );

        let pwm3 = embassy_rp::pwm::Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, c.clone());
        let pwm45 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, c.clone());

        let enable_pin1 = embassy_rp::gpio::Output::new(p.PIN_10, embassy_rp::gpio::Level::Low);
        let driver1: simplefoc::pwm_driver::PWMDriver<'static> =
            crate::simplefoc::pwm_driver::PWMDriver::new(
                pwm3,
                pwm45,
                enable_pin1,
                c,
                voltage_limit,
                supply_voltage,
            );

        (driver0, driver1)
    };

    #[cfg(feature = "picoA")]
    // let (motor_config0, motor_config1) = (MOTOR_CONFIG_4015, MOTOR_CONFIG_GM5208_24);
    let (motor_config0, motor_config1) = (unimplemented!(), unimplemented!());

    #[cfg(feature = "picoB")]
    let (motor_config0, motor_config1) = (MOTOR_CONFIG_GM4108, MOTOR_CONFIG_GM3506);

    #[cfg(feature = "picoC")]
    let (motor_config0, motor_config1) = (MOTOR_CONFIG_GM3506, MOTOR_CONFIG_GM3506);

    #[cfg(feature = "testing")]
    // let (motor_config0, motor_config1) = (MOTOR_CONFIG_GM4108, MOTOR_CONFIG_GM5208_24);
    let (motor_config0, motor_config1) = (MOTOR_CONFIG_GM5208_24, MOTOR_CONFIG_GM5208_24);
    // let (motor_config0, motor_config1) = (MOTOR_CONFIG_GL60, MOTOR_CONFIG_GL60);

    #[cfg(feature = "picoA")]
    // let (output_encoder0, output_encoder1) = (None, Some(output_encoder0));
    let output_encoder1 = Some(output_encoder0);

    let usb = comms::usb::UsbLogger::new();
    let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

    #[cfg(feature = "nope")]
    let max485 = {
        static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; 16])[..];
        static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
        let rx_buf = &mut RX_BUF.init([0; 16])[..];

        let mut config = embassy_rp::uart::Config::default();
        config.baudrate = 115_200;
        // config.baudrate = 9600;

        let mut uart = embassy_rp::uart::BufferedUart::new(
            p.UART0, p.PIN_16, p.PIN_17, Irqs, tx_buf, rx_buf, config,
        );

        let mut enable = embassy_rp::gpio::Output::new(p.PIN_18, embassy_rp::gpio::Level::Low);

        crate::comms::rs485::Max485::new(uart, enable)
    };

    let foc0 = crate::simplefoc::foc_types::SimpleFOC::new(
        MOTOR_ID_A,
        encoder0,
        None::<()>,
        // Some(current_sensor),
        // None,
        // None,
        pwm_driver0,
        motor_config0,
        usb.clone(),
        // None,
    );

    let foc1 = crate::simplefoc::foc_types::SimpleFOC::new(
        MOTOR_ID_B,
        encoder1,
        None::<()>,
        // Some(current_sensor),
        // None,
        // Some(crate::simplefoc::current_read_task::CURRENT_CHANNEL.receiver()),
        // Some(crate::simplefoc::current_read_task::ELEC_ANGLE_CHANNEL.sender()),
        pwm_driver1,
        motor_config1,
        usb,
        // None,
    );

    #[cfg(feature = "nope")]
    let foc = crate::simplefoc::foc_types::SimpleFOC::new(
        0,
        // encoder0,
        encoder1,
        pwm_driver0,
        enable_pin0,
        motor_config0,
        Some(usb),
        // None,
    );

    // second core runs USB
    // #[cfg(feature = "nope")]
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(init::CORE1_STACK) },
        move || {
            let executor1 = init::EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner| {
                // spawner.spawn(crate::init::core0_task1(foc1, None).unwrap());

                // let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

                // crate::comms::usb::UsbMonitor::init(&spawner, driver);
                crate::comms::usb_raw::usb_init(&spawner, driver);

                // spawner.spawn(
                //     crate::simplefoc::current_read_task::core1_task_current_sens(current_sensor)
                //         .unwrap(),
                // );

                //
            });
        },
    );

    // second core runs rs485
    #[cfg(feature = "nope")]
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(init::CORE1_STACK) },
        move || {
            let executor1 = init::EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner| crate::comms::rs485::init_rs485_logger(&spawner, max485));
        },
    );

    // second core runs motor
    #[cfg(feature = "nope")]
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(init::CORE1_STACK) },
        move || {
            let executor1 = init::EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(crate::init::core0_task1(foc1)).unwrap();
            });
        },
    );

    let executor0 = init::EXECUTOR0.init(embassy_executor::Executor::new());
    executor0.run(|spawner| {
        // spawner.spawn(crate::init::core0_task0(foc0)).unwrap();
        // spawner.spawn(crate::init::core0_task1(foc1)).unwrap();

        // spawner
        //     // .spawn(crate::init::core0_task0(foc0, output_encoder0))
        //     .spawn(crate::init::core0_task0(foc0, None))
        //     .unwrap();

        // crate::comms::usb_raw::usb_init(&spawner, driver);
        spawner.spawn(crate::init::core0_task1(foc1, None).unwrap());

        // crate::comms::usb::UsbMonitor::init(&spawner, driver);
        // crate::comms::usb_raw::usb_init(&spawner, driver);

        // spawner.spawn(crate::init::core0_task1(foc)).unwrap();
    });
}

// #[embassy_executor::task]
#[cfg(feature = "nope")]
async fn test_foc(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        // hardware::as5600::AS5600<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        // >,
        hardware::mt_6701::MT6701<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    info!("Starting FOC test");
    // let update_rate_hz = 9000;
    let update_rate_hz = 1_000;
    let print_rate_hz = 20;
    let time_limit = 10;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    let n_max = update_rate_hz / print_rate_hz;
    let max_time = Instant::now() + embassy_time::Duration::from_secs(time_limit);
    let mut n = 0;

    // foc.disable();

    // let angle_target = 1.0;
    // let angle_current = 0.0;

    // let mut pid = crate::simplefoc::pid::PIDController::new(1.0, 0.0, 0.0, 0.0, 12.0);

    // let mut vn = 0;
    // let mut vs = 0.;

    let mut len = 0;
    let mut sum = 0;
    let mut min = u64::MAX;
    let mut max = u64::MIN;

    debug!("starting test loop");
    loop {
        ticker.next().await;
        // foc.debug_update_sensor().await;

        // if v.abs() > 0.01 {
        //     // debug!("Angle: {}", foc.debug_encoder().get_angle());
        //     // debug!("Velocity: {}", v);
        //     debug!("Raw angle: {}", foc.debug_encoder().get_raw_angle());
        // }

        // let angle = foc.encoder.sample_raw().await.unwrap();
        // len += 1;
        // sum += angle as u64;

        // min = min.min(angle as u64);
        // max = max.max(angle as u64);

        let t_us = Instant::now().as_micros();
        foc.encoder.update(t_us).await.unwrap();
        foc.run_commands().await;

        // vn += 1;
        // vs += v;
        if n >= n_max {
            n = 0;

            // let v = foc.get_shaft_velocity();
            // // let v = foc.debug_encoder().get_velocity();

            let position = foc.encoder.get_mechanical_angle();
            let v = foc.encoder.get_velocity();

            debug!(
                "Angle: {}, Velocity: {}",
                // libm::roundf(angle * 1000.) / 1000. * (180. / core::f32::consts::PI),
                position,
                v,
            );

            // foc.send_debug_message(robotarm_protocol::SerialLogMessage::MotorData {
            //     id: 0,
            //     timestamp: t_us,
            //     target: 0.0,
            //     position: foc.encoder.get_angle(),
            //     angle: foc.encoder.get_mechanical_angle(),
            //     velocity: v,
            // })
            // .await;

            // debug!(
            //     "Angle: {}, velocity: {}",
            //     foc.debug_encoder().get_angle(),
            //     // v
            //     0.0,
            // );
            // let v = vs / (vn as f32);
            // vn = 0;
            // vs = 0.;
            // debug!("Velocity: {}", v);
            //
        } else {
            n += 1;
        }

        #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            // let avg = sum as u64 / len as u64;
            // debug!(
            //     "Average raw angle: {}, n: {}, min: {}, max: {}, range: {}",
            //     avg,
            //     len,
            //     min,
            //     max,
            //     max - min
            // );

            info!("Halting FOC test");
            foc.disable();
            break;
        }
    }
}

// #[embassy_executor::task]
#[cfg(feature = "nope")]
async fn loop_foc(
    mut foc: crate::simplefoc::foc_types::SimpleFOC<
        'static,
        // hardware::as5600::AS5600<
        //     embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        // >,
        hardware::mt_6701::MT6701<
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Async>,
        >,
    >,
) {
    let update_rate_hz = 20_000;
    // let print_rate_hz = 100;
    // let time_limit = 1.5;
    let time_limit = 2.;

    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(
        1_000_000 / update_rate_hz,
    ));
    // let n_max = update_rate_hz / print_rate_hz;
    let mut max_time =
        Instant::now() + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);

    foc.set_debug_freq(10);

    foc.enable();

    // let tgt = 1.64;
    let mut tgt = 1.0;
    foc.set_target_position(tgt);

    // let tgt = 60.;

    let mut x = 0;
    foc.set_target_velocity(3.14 * 1.);

    // // foc.set_target_torque(0.);
    // foc.set_target_torque(0.05);

    let v = 3.14;

    let mut cmd_buf = heapless::Vec::<_, 4>::new();

    info!("Starting main loop");
    loop {
        ticker.next().await;
        foc.run_commands(&mut cmd_buf).await;
        foc.update_foc().await;

        // #[cfg(feature = "nope")]
        if Instant::now() > max_time {
            match x {
                0 => {
                    // info!("Setting velocity to -1");
                    x = 1;
                    // foc.set_target_position(1.0);
                    foc.set_target_velocity(v * -1.);
                }
                _ => {
                    // info!("Setting velocity to 1");
                    x = 0;
                    // foc.set_target_position(6.);
                    foc.set_target_velocity(v * 1.);
                } // 1 => {
                  //     // info!("Setting velocity to 0");
                  //     x = 2;
                  //     // foc.set_target_position(3.);
                  //     foc.set_target_velocity(3.14 * 0.);
                  // }
            }
            max_time = max_time + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
        }

        // if Instant::now() > max_time {
        //     tgt += 3.14 / 2.;
        //     foc.set_target_position(tgt);
        //     max_time = max_time + embassy_time::Duration::from_millis((time_limit * 1000.) as u64);
        // }

        // if Instant::now() > max_time {
        //     info!("Halting FOC loop");
        //     foc.disable();
        //     break;
        // }
    }
}
