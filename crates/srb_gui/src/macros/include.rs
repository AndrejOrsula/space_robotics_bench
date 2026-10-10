macro_rules! include_assets_font {
    ($file:expr $(,)?) => {
        ::egui::FontData::from_static(include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/assets/fonts/",
            $file
        )))
    };
}

pub(crate) use include_assets_font;
