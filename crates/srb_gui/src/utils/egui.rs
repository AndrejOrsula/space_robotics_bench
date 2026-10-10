#[must_use = "You should call .show()"]
#[derive(Debug, Clone, Copy, PartialEq, typed_builder::TypedBuilder)]
pub struct ScrollableFramedCentralPanel {
    #[builder(default = 1024.0)]
    pub max_content_width: f32,
    #[builder(default = egui::Margin {left: 0, right: 8, top: 0, bottom: 0})]
    pub min_inner_margin: egui::Margin,
}

impl Default for ScrollableFramedCentralPanel {
    fn default() -> Self {
        Self::builder().build()
    }
}

impl ScrollableFramedCentralPanel {
    pub fn show<R>(
        self,
        ctx: &egui::Context,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> egui::InnerResponse<R> {
        egui::CentralPanel::default().show(ctx, |ui| {
            let margin_x = (ctx.content_rect().width() - self.max_content_width).max(0.0) / 2.0;
            let inner_margin = egui::Margin {
                left: (margin_x as i8).max(self.min_inner_margin.left),
                right: (margin_x as i8).max(self.min_inner_margin.right),
                ..self.min_inner_margin
            };
            egui::Frame::default()
                .inner_margin(inner_margin)
                .show(ui, |ui| add_contents(ui))
                .inner
        })
    }
}
