#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

pub mod kinematics;
mod logging;
mod serial;
mod simplefoc;
mod ui;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tracing::{debug, error, info, trace, warn};

use bytes::Buf as _;
use postcard::{accumulator::FeedResult, experimental::max_size::MaxSize};

use serialport::SerialPort as _;

use robotarm_protocol::SerialLogMessage;

// use tokio_serial::SerialPort;

// k test
#[cfg(feature = "nope")]
fn main() {
    logging::init_logs();

    // let rate = 9600;
    let rate = 115200;
    let mut port = serialport::new("COM11", rate).open().unwrap();

    let mut raw_buf: [u8; 1024] = [0; 1024];

    let mut x = 0;

    debug!("Looping");
    #[cfg(feature = "nope")]
    loop {
        if let Ok(n) = port.read(&mut raw_buf) {
            debug!("Read {}", n);
            for i in &raw_buf[..n] {
                print!("{:02X} ", i);
            }
            println!();
        }

        // debug!("Sending");
        // port.write(&[x]).unwrap();
        // // std::thread::sleep(std::time::Duration::from_micros(1000));
        // // port.write(&[0xff]).unwrap();

        x += 1;

        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    let mut cobs_buf = postcard::accumulator::CobsAccumulator::<4096>::new();

    // let out = cobs_buf.feed::<[u8; 4]>(&[9, 188, 3, 218, 2, 252, 2, 222, 3, 0]);

    // match out {
    //     FeedResult::Consumed => debug!("Consumed"),
    //     FeedResult::OverFull(new_wind) => debug!("Overfull: {:?}", new_wind),
    //     FeedResult::DeserError(new_wind) => debug!("Deserialization error: {:?}", new_wind),
    //     FeedResult::Success { data, remaining } => {
    //         debug!("Success: {:?}, remaining: {:?}", data, remaining);
    //     }
    // }
    // debug!("Feed result: {:?}", out);

    // #[cfg(feature = "nope")]
    loop {
        // while let Ok(ct) = tokio::io::AsyncReadExt::read(&mut port, &mut raw_buf).await {
        while let Ok(ct) = port.read(&mut raw_buf) {
            debug!("Read {} bytes", ct);
            // Finished reading input
            if ct == 0 {
                break;
            }

            let buf = &raw_buf[..ct];
            let mut window = &buf[..];

            'cobs: while !window.is_empty() {
                window = match cobs_buf.feed::<[u8; 4]>(&window) {
                    FeedResult::Consumed => break 'cobs,
                    FeedResult::OverFull(new_wind) => new_wind,
                    FeedResult::DeserError(new_wind) => {
                        debug!("Deserialization error");
                        new_wind
                    }
                    FeedResult::Success { data, remaining } => {
                        debug!("Received message: {:?}", data);

                        remaining
                    }
                };
            }
        }
    }

    //
}

#[cfg(feature = "nope")]
// #[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    use futures::{SinkExt as _, StreamExt as _};
    use tokio_serial::SerialPortBuilderExt as _;
    use tokio_util::codec::Decoder as _;

    use postcard::accumulator::FeedResult;

    logging::init_logs();

    let rate = 115200;
    // let rate = 921600;

    // let mut port = serialport::new("COM8", rate).open().unwrap();
    let mut port = serialport::new("COM11", rate).open().unwrap();

    // let mut port =
    //     tokio_serial::SerialPortBuilderExt::open_native_async(tokio_serial::new("COM8", rate))
    //         .unwrap();

    let _ = port.write_data_terminal_ready(true);
    // serialport::SerialPort::write_data_terminal_ready(&mut port, true)?;

    debug!("timeout set: {:?}", port.timeout());

    let mut cobs_buf = postcard::accumulator::CobsAccumulator::<4096>::new();

    let mut raw_buf: [u8; 1024] = [0; 1024];

    loop {
        // while let Ok(ct) = tokio::io::AsyncReadExt::read(&mut port, &mut raw_buf).await {
        while let Ok(ct) = port.read(&mut raw_buf) {
            // Finished reading input
            if ct == 0 {
                break;
            }

            let buf = &raw_buf[..ct];
            let mut window = &buf[..];

            'cobs: while !window.is_empty() {
                window = match cobs_buf.feed::<SerialLogMessage>(&window) {
                    FeedResult::Consumed => break 'cobs,
                    FeedResult::OverFull(new_wind) => new_wind,
                    FeedResult::DeserError(new_wind) => new_wind,
                    FeedResult::Success { data, remaining } => {
                        debug!("Received message: {:?}", data);

                        remaining
                    }
                };
            }
        }
    }

    // let mut framed = crate::serial::codec::SerialCodec::default().framed(port);

    // let mut buf: [u8; 1024] = [0; 1024];
    // loop {
    //     if let Ok(n) = port.read(&mut buf) {
    //         debug!("Read {} bytes: {:?}", n, &buf[..n]);
    //     }
    // }

    //
}

#[cfg(feature = "nope")]
// #[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    use futures::{SinkExt as _, StreamExt as _};
    use std::io::Read;
    use tokio_serial::SerialPortBuilderExt as _;
    use tokio_util::codec::Decoder as _;

    logging::init_logs();

    // let (serial_log_tx, serial_log_rx) = tokio::sync::mpsc::channel(100);
    // let (serial_cmd_tx, serial_cmd_rx) = tokio::sync::mpsc::channel(100);

    // let rate = 115200;
    let rate = 921600;

    let mut port =
        tokio_serial::SerialPortBuilderExt::open_native_async(tokio_serial::new("COM8", rate))
            .unwrap();

    info!("waiting for serial connection...");
    loop {
        if let Ok(_) = port.set_baud_rate(rate) {
            break;
        }
    }
    info!("serial connection established");
    port.write_data_terminal_ready(true).unwrap();
    // port.set_baud_rate(rate);

    // let mut buf: [u8; 1024] = [0; 1024];
    // loop {
    //     if let Ok(n) = port.read(&mut buf) {
    //         debug!("Read {} bytes: {:?}", n, &buf[..n]);
    //     }
    // }

    let mut framed = crate::serial::codec::SerialCodec::default().framed(port);

    // #[cfg(feature = "nope")]
    loop {
        if let Some(n) = framed.next().await {
            match n {
                Ok(msg) => {
                    debug!("Received message: {:?}", msg);
                }
                Err(e) => {
                    debug!("Error reading from serial port: {}", e);
                }
            }
        } else {
            debug!("Serial port closed");
        }

        // t += 1.;
    }

    // Ok(())
}

#[cfg(feature = "nope")]
fn main() -> eframe::Result<()> {
    logging::init_logs();

    let (serial_log_tx, serial_log_rx) = crossbeam_channel::unbounded();
    let (serial_cmd_tx, serial_cmd_rx) = crossbeam_channel::unbounded();

    let rate = 115200;
    // let rate = 921600;

    // let mut port = serialport::new("COM8", rate).open().unwrap();

    // port.write_data_terminal_ready(true).unwrap();

    // let mut bs = bytes::BytesMut::with_capacity(1024);
    // let mut raw_buf: [u8; 1024] = [0; 1024];

    // let mut cobs_buf = postcard::accumulator::CobsAccumulator::<4096>::new();

    #[cfg(feature = "nope")]
    loop {
        match port.read(&mut raw_buf) {
            Ok(n) => {
                debug!("Read {} bytes", n);
                bs.extend_from_slice(&raw_buf[..n]);

                let mut len;

                'cobs: loop {
                    len = bs.len();
                    match cobs_buf.feed::<SerialLogMessage>(&mut bs[..]) {
                        FeedResult::Consumed => {
                            debug!("No complete message in buffer, {}", bs.len());
                            break 'cobs;
                        }
                        FeedResult::OverFull(new_wind) => {
                            debug!("Accumulator overflow");
                            // bs = new_wind.into();
                            panic!()
                        }
                        FeedResult::DeserError(new_wind) => {
                            debug!("Deserialization error: {}", new_wind.len());
                            // bs = new_wind.into();
                        }
                        FeedResult::Success { data, remaining } => {
                            debug!("Received message: {:?}", data);
                            // bs = remaining.into();

                            let consumed = len - remaining.len();
                            bs.advance(consumed);
                        }
                    }
                }
            }
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    // debug!("Serial read timed out");
                } else {
                    debug!("Error reading from serial port: {:?}", e);
                }
            }
        }
    }

    // works
    #[cfg(feature = "nope")]
    loop {
        match port.read(&mut raw_buf) {
            Ok(n) => {
                // debug!("Read {} bytes: {:?}", n, &raw_buf[..n]);
                // debug!("Read {} bytes", n);
                // raw_buf.advance(n);

                let buf = &raw_buf[..n];
                let mut window = &buf[..];

                'cobs: while !window.is_empty() {
                    window = match cobs_buf.feed::<SerialLogMessage>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(new_wind) => new_wind,
                        FeedResult::DeserError(new_wind) => new_wind,
                        FeedResult::Success { data, remaining } => {
                            debug!("Received message: {:?}", data);

                            remaining
                        }
                    };
                }
            }
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    // debug!("Serial read timed out");
                } else {
                    debug!("Error reading from serial port: {:?}", e);
                }
            }
        }
    }

    std::thread::spawn(move || {
        loop {
            let _ = serial_log_rx.recv();
        }
    });

    let mut serial_handler = serial::SerialHandler::new("COM8", serial_log_tx, serial_cmd_rx, rate);

    loop {
        if let Err(e) = serial_handler.run() {
            error!("Error in serial handler: {}", e);
        }
    }
    // Ok(())
}

/// MARK: Main
// #[cfg(feature = "nope")]
fn main() -> eframe::Result<()> {
    logging::init_logs();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            // .with_icon(icon)
            .with_inner_size([850.0, 750.0])
            .with_min_inner_size([550.0, 400.0]),
        // renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    // let (serial_log_tx, serial_log_rx) = tokio::sync::mpsc::channel(100);
    // let (serial_cmd_tx, serial_cmd_rx) = tokio::sync::mpsc::channel(100);
    let (serial_log_tx, serial_log_rx) = crossbeam_channel::unbounded();
    let (serial_cmd_tx, serial_cmd_rx) = crossbeam_channel::unbounded();
    let (ui_cmd_tx, ui_cmd_rx) = crossbeam_channel::unbounded();

    debug!("Starting serial thread");
    std::thread::spawn(|| {
        let port = "COM8";
        let rate = 921600;

        // let port = "COM11";
        // let rate = 115200;

        let mut serial_handler =
            serial::SerialHandler::new(port, serial_log_tx, serial_cmd_rx, ui_cmd_tx, rate);

        loop {
            if let Err(e) = serial_handler.run() {
                // error!("Error in serial handler: {}", e);
            }
        }
    });

    eframe::run_native(
        "Robot Control",
        native_options,
        Box::new(move |cc| {
            // egui_extras::install_image_loaders(&cc.egui_ctx);

            // /// repaint
            // let ctx2 = cc.egui_ctx.clone();
            // std::thread::spawn(move || {
            //     loop {
            //         std::thread::sleep(std::time::Duration::from_millis(1000));
            //         ctx2.request_repaint();
            //     }
            // });

            Ok(Box::new(ui::app::App::new(
                cc,
                serial_log_rx,
                serial_cmd_tx,
                ui_cmd_rx,
            )))
        }),
    )

    // println!("Hello, world!");
}
