use egui::{FontFamily, FontId, TextStyle};

pub fn styles() -> std::collections::BTreeMap<TextStyle, FontId> {
    [
        (
            TextStyle::Monospace,
            FontId::new(16.0, FontFamily::Monospace),
        ),
        (
            TextStyle::Small,
            FontId::new(12.0, FontFamily::Proportional),
        ),
        (TextStyle::Body, FontId::new(18.0, FontFamily::Proportional)),
        (
            TextStyle::Button,
            FontId::new(18.0, FontFamily::Proportional),
        ),
        (
            TextStyle::Heading,
            FontId::new(48.0, FontFamily::Proportional),
        ),
    ]
    .into()
}
