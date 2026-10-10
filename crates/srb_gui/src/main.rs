mod cli_args;

fn main() -> eframe::Result<()> {
    if cli_args::startup_action(std::env::args_os()) == cli_args::StartupAction::PrintHelp {
        print!("{}", cli_args::USAGE);
        return Ok(());
    }

    tracing_subscriber::fmt::init();

    let icon = image::load_from_memory_with_format(
        include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/assets/window_icon.png"
        )),
        image::ImageFormat::Png,
    )
    .unwrap()
    .to_rgba8();
    let (icon_width, icon_height) = icon.dimensions();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_min_inner_size([600.0, 480.0])
            .with_icon(egui::IconData {
                rgba: icon.into_raw(),
                width: icon_width,
                height: icon_height,
            }),
        ..Default::default()
    };

    eframe::run_native(
        "Space Robotics Bench",
        native_options,
        Box::new(|cc| Ok(Box::new(srb_gui::App::new(cc)))),
    )
}
