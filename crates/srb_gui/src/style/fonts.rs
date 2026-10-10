use egui::Context;

pub fn set(ctx: &Context) {
    let font_definitions = crate::macros::generate_font_definitions! {
        "MonaspaceNeon" as Monospace [
            "MonaspaceNeon-Medium.otf",
        ],

        "Inter" as Proportional [
            "Inter-Regular.otf",
        ],
        "MaterialIcons" as Proportional [
            "MaterialIconsRound-Regular.otf",
        ],
    };

    ctx.set_fonts(font_definitions);
}
