#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

mod logging;
mod serial;
mod simplefoc;
mod ui;

use anyhow::{Context, Result, anyhow, bail, ensure};
use tokio_serial::SerialPort;
use tracing::{debug, error, info, trace, warn};

// #[cfg(feature = "nope")]
#[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    use futures::{SinkExt as _, StreamExt as _};
    use std::io::Read;
    use tokio_serial::SerialPortBuilderExt as _;
    use tokio_util::codec::Decoder as _;

    logging::init_logs();

    // let (serial_log_tx, serial_log_rx) = tokio::sync::mpsc::channel(100);
    // let (serial_cmd_tx, serial_cmd_rx) = tokio::sync::mpsc::channel(100);

    let rate = 115200;
    // let rate = 921600;

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

/// MARK: Main
#[cfg(feature = "nope")]
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

    let (serial_log_tx, serial_log_rx) = tokio::sync::mpsc::channel(100);
    let (serial_cmd_tx, serial_cmd_rx) = tokio::sync::mpsc::channel(100);

    debug!("Starting serial thread");
    std::thread::spawn(|| {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async move {
            let rate = 921600;
            let mut serial_handler = serial::SerialHandler::new(
                tokio_serial::SerialPortBuilderExt::open_native_async(tokio_serial::new(
                    "COM8", rate,
                ))
                .unwrap(),
                serial_log_tx,
                serial_cmd_rx,
                rate,
            );

            loop {
                if let Err(e) = serial_handler.run().await {
                    // error!("Error in serial handler: {}", e);
                }
            }
        });
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

            Ok(Box::new(ui::app::App::new(serial_log_rx, serial_cmd_tx)))
        }),
    )

    // println!("Hello, world!");
}
