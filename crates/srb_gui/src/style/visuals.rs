use std::sync::OnceLock;

use egui::{
    epaint::Shadow,
    style::{HandleShape, NumericColorSpace, Selection, TextCursorStyle, WidgetVisuals, Widgets},
    Color32, CornerRadius, Stroke, Visuals,
};

pub fn dark() -> &'static Visuals {
    static VISUALS: OnceLock<Visuals> = OnceLock::new();
    VISUALS.get_or_init(dark_theme)
}

fn dark_theme() -> Visuals {
    Visuals {
        dark_mode: true,
        override_text_color: None,
        widgets: Widgets {
            noninteractive: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(30, 30, 46).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(30, 30, 46),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(138, 143, 163)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
            inactive: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(49, 50, 68).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(49, 50, 68),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(138, 143, 163)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
            hovered: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(88, 91, 112).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(88, 91, 112),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
                fg_stroke: Stroke::new(1.5_f32, Color32::from_rgb(205, 214, 244)),
                corner_radius: CornerRadius::same(3),
                expansion: 1.0,
            },
            active: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(69, 71, 90).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(69, 71, 90),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
                fg_stroke: Stroke::new(2.0_f32, Color32::from_rgb(205, 214, 244)),
                corner_radius: CornerRadius::same(2),
                expansion: 1.0,
            },
            open: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(49, 50, 68).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(49, 50, 68),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(205, 214, 244)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
        },
        selection: Selection {
            bg_fill: Color32::from_rgb(137, 180, 250).gamma_multiply(0.25),
            stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
        },
        hyperlink_color: Color32::from_rgb(245, 224, 220),
        faint_bg_color: Color32::from_rgb(49, 50, 68),
        extreme_bg_color: Color32::from_rgb(17, 17, 27),
        code_bg_color: Color32::from_rgb(24, 24, 37),
        warn_fg_color: Color32::from_rgb(250, 179, 135),
        error_fg_color: Color32::from_rgb(235, 160, 172),
        window_corner_radius: CornerRadius::same(6),
        window_shadow: Shadow {
            offset: [10, 20],
            blur: 15,
            spread: 0,
            color: Color32::from_rgb(30, 30, 46),
        },
        window_fill: Color32::from_rgb(30, 30, 46),
        window_stroke: Stroke::new(1.0_f32, Color32::from_rgb(127, 132, 156)),
        window_highlight_topmost: true,
        menu_corner_radius: CornerRadius::same(6),
        panel_fill: Color32::from_rgb(30, 30, 46),
        popup_shadow: Shadow {
            offset: [6, 10],
            blur: 8,
            spread: 0,
            color: Color32::from_rgb(30, 30, 46),
        },
        resize_corner_size: 12.0,
        text_cursor: TextCursorStyle {
            stroke: Stroke::new(2.0_f32, Color32::from_rgb(210, 219, 250)),
            ..Default::default()
        },
        clip_rect_margin: 3.0,
        button_frame: false,
        collapsing_header_frame: false,
        indent_has_left_vline: true,
        striped: false,
        slider_trailing_fill: false,
        handle_shape: HandleShape::Circle,
        interact_cursor: Some(egui::CursorIcon::PointingHand),
        image_loading_spinners: true,
        numeric_color_space: NumericColorSpace::GammaByte,
        ..Default::default()
    }
}

pub fn light() -> &'static Visuals {
    static VISUALS: OnceLock<Visuals> = OnceLock::new();
    VISUALS.get_or_init(light_theme)
}

fn light_theme() -> Visuals {
    Visuals {
        dark_mode: false,
        override_text_color: None,
        widgets: Widgets {
            noninteractive: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(239, 241, 245).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(239, 241, 245),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(76, 79, 105)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
            inactive: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(204, 208, 218).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(204, 208, 218),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(76, 79, 105)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
            hovered: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(172, 176, 190).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(172, 176, 190),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
                fg_stroke: Stroke::new(1.5_f32, Color32::from_rgb(11, 12, 16)),
                corner_radius: CornerRadius::same(3),
                expansion: 1.0,
            },
            active: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(188, 192, 204).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(188, 192, 204),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
                fg_stroke: Stroke::new(2.0_f32, Color32::from_rgb(11, 12, 16)),
                corner_radius: CornerRadius::same(2),
                expansion: 1.0,
            },
            open: WidgetVisuals {
                weak_bg_fill: Color32::from_rgb(204, 208, 218).gamma_multiply(0.75),
                bg_fill: Color32::from_rgb(204, 208, 218),
                bg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
                fg_stroke: Stroke::new(1.0_f32, Color32::from_rgb(11, 12, 16)),
                corner_radius: CornerRadius::same(2),
                expansion: 0.0,
            },
        },
        selection: Selection {
            bg_fill: Color32::from_rgb(30, 102, 245).gamma_multiply(0.25),
            stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
        },
        hyperlink_color: Color32::from_rgb(220, 138, 120),
        faint_bg_color: Color32::from_rgb(204, 208, 218),
        extreme_bg_color: Color32::from_rgb(220, 224, 232),
        code_bg_color: Color32::from_rgb(230, 233, 239),
        warn_fg_color: Color32::from_rgb(254, 100, 11),
        error_fg_color: Color32::from_rgb(230, 69, 83),
        window_corner_radius: CornerRadius::same(6),
        window_highlight_topmost: true,
        menu_corner_radius: CornerRadius::same(6),
        window_shadow: Shadow {
            offset: [10, 20],
            blur: 15,
            spread: 0,
            color: Color32::from_rgb(239, 241, 245),
        },
        window_fill: Color32::from_rgb(239, 241, 245),
        window_stroke: Stroke::new(1.0_f32, Color32::from_rgb(140, 143, 161)),
        panel_fill: Color32::from_rgb(239, 241, 245),
        popup_shadow: Shadow {
            offset: [6, 10],
            blur: 8,
            spread: 0,
            color: Color32::from_rgb(239, 241, 245),
        },
        resize_corner_size: 12.0,
        text_cursor: TextCursorStyle {
            stroke: Stroke::new(2.0_f32, Color32::from_rgb(63, 65, 87)),
            ..Default::default()
        },
        clip_rect_margin: 3.0,
        button_frame: false,
        collapsing_header_frame: false,
        indent_has_left_vline: true,
        striped: false,
        slider_trailing_fill: false,
        handle_shape: HandleShape::Circle,
        interact_cursor: Some(egui::CursorIcon::PointingHand),
        image_loading_spinners: true,
        numeric_color_space: NumericColorSpace::GammaByte,
        ..Default::default()
    }
}
