#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_doc_comments)]
#![allow(unexpected_cfgs)]

mod logging;
mod simplefoc;
mod ui;

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

            Ok(Box::new(ui::app::App::new()))
        }),
    )

    // println!("Hello, world!");
}
