use crate::cache;
use crate::gui_interface_controls::{GuiInterfaceControlEvent, GuiInterfaceControls};
use crate::gui_ros::GuiRosControls;
use crate::gui_runtime::GuiRuntimeError;
use crate::gui_runtime::GuiRuntimeSession;
use crate::gui_runtime_params::RuntimeParams;
use crate::gui_session::GuiSessionState;
use crate::gui_task_browser::GuiTaskBrowser;
use crate::gui_workflow::GuiWorkflow;
use crate::gui_workflow::GuiWorkflowError;
use crate::page::Page;
use egui_commonmark::{CommonMarkCache, CommonMarkViewer};
use strum::IntoEnumIterator;
use tracing::error;

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub enum GuiInitializationState {
    #[default]
    Healthy,
    Unhealthy(GuiWorkflowError),
}

impl GuiInitializationState {
    #[must_use]
    pub const fn is_healthy(&self) -> bool {
        matches!(self, Self::Healthy)
    }

    #[must_use]
    pub fn error(&self) -> Option<&GuiWorkflowError> {
        match self {
            Self::Healthy => None,
            Self::Unhealthy(error) => Some(error),
        }
    }
}

/// Restore the `(theme:...)` value under `eframe::APP_KEY` without initializing `App`.
#[derive(serde::Deserialize, serde::Serialize)]
struct PersistedApp {
    theme: egui::Theme,
}

pub struct App {
    theme: egui::Theme,
    task_browser: GuiTaskBrowser,
    runtime_session: GuiRuntimeSession,
    session: GuiSessionState,
    ros: GuiRosControls,
    interface_controls: GuiInterfaceControls,
    metrics: crate::gui_metrics::GuiMetrics,

    gui_catalog: cache::GuiCatalogCacheResult,

    initialization: GuiInitializationState,
    workflow_errors: Vec<GuiWorkflowError>,

    commonmark_cache: CommonMarkCache,

    runtime_params_delivery: RuntimeParamsDelivery,
}

impl Default for App {
    fn default() -> Self {
        let gui_catalog = cache::load_gui_catalog();
        match Self::try_initialize(gui_catalog.clone()) {
            Ok(app) => app,
            Err(error) => Self::neutral_initialization_fallback(gui_catalog, error),
        }
    }
}

impl App {
    const MAX_WORKFLOW_ERRORS: usize = 32;
    const HEADER_HEIGHT: f32 = 50.0;
    const HEADER_CONTENT_HEIGHT: f32 = 34.0;
    const HEADER_NAV_BUTTON_WIDTH: f32 = 118.0;
    const HEADER_NAV_BUTTON_HEIGHT: f32 = 30.0;
    const HEADER_CONFIG_WIDTH: f32 = 118.0;
    const HEADER_SIDE_MARGIN: f32 = 14.0;

    #[must_use]
    pub fn new(cc: &eframe::CreationContext) -> Self {
        egui_extras::install_image_loaders(&cc.egui_ctx);

        crate::style::load_fonts(&cc.egui_ctx);

        let restored_theme = cc.storage.and_then(restore_theme);
        let mut app = Self::default();
        if let Some(theme) = restored_theme {
            app.theme = theme;
        }

        crate::style::set_theme(&cc.egui_ctx, app.theme);

        app.publish_runtime_params();

        app
    }

    pub fn try_default() -> Result<Self, GuiWorkflowError> {
        Self::try_initialize(cache::load_gui_catalog())
    }

    #[must_use]
    pub fn initialization_state(&self) -> &GuiInitializationState {
        &self.initialization
    }

    #[must_use]
    pub fn workflow_errors(&self) -> &[GuiWorkflowError] {
        &self.workflow_errors
    }

    fn try_initialize(gui_catalog: cache::GuiCatalogCacheResult) -> Result<Self, GuiWorkflowError> {
        let task_browser =
            GuiTaskBrowser::from_catalog_result(&gui_catalog).map_err(|error| match error {
                crate::gui_selection::GuiSelectionInitializationError::CatalogLoad(error) => {
                    GuiWorkflowError::CatalogLoad(error)
                }
                crate::gui_selection::GuiSelectionInitializationError::InvalidCatalogDefaults(
                    error,
                ) => GuiWorkflowError::CatalogDefaults(error),
            })?;
        let session = GuiSessionState::from_selection(&task_browser.selection);
        let runtime_session = GuiRuntimeSession::try_new().map_err(GuiWorkflowError::Runtime)?;

        Ok(Self {
            theme: egui::Theme::Dark,
            task_browser,
            runtime_session,
            session,
            ros: GuiRosControls::default(),
            interface_controls: GuiInterfaceControls::default(),
            metrics: crate::gui_metrics::GuiMetrics::default(),
            gui_catalog,
            initialization: GuiInitializationState::Healthy,
            workflow_errors: Vec::new(),
            commonmark_cache: CommonMarkCache::default(),
            runtime_params_delivery: RuntimeParamsDelivery::default(),
        })
    }

    fn neutral_initialization_fallback(
        gui_catalog: cache::GuiCatalogCacheResult,
        error: GuiWorkflowError,
    ) -> Self {
        let task_browser = GuiTaskBrowser::neutral_initialization_fallback();
        let session = GuiSessionState::from_selection(&task_browser.selection);
        Self {
            theme: egui::Theme::Dark,
            task_browser,
            runtime_session: GuiRuntimeSession::unavailable(
                crate::gui_runtime::GuiRuntimeError::initialization_disabled(),
            ),
            session,
            ros: GuiRosControls::default(),
            interface_controls: GuiInterfaceControls::default(),
            metrics: crate::gui_metrics::GuiMetrics::default(),
            gui_catalog,
            initialization: GuiInitializationState::Unhealthy(error.clone()),
            workflow_errors: vec![error],
            commonmark_cache: CommonMarkCache::default(),
            runtime_params_delivery: RuntimeParamsDelivery::default(),
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // A stop runs on a worker thread; keep polling frames flowing so its
        // terminal result is adopted promptly even without input events.
        if self.runtime_session.is_stopping() {
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F11)) {
            let fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!fullscreen));
        }

        egui::TopBottomPanel::top("top_panel")
            .exact_height(Self::HEADER_HEIGHT)
            .show(ctx, |ui| {
                self.app_header(ui);
            });

        egui::TopBottomPanel::bottom("bottom_panel")
            .exact_height(36.0)
            .show(ctx, |ui| {
                self.status_bar(ui);
            });

        crate::utils::egui::ScrollableFramedCentralPanel::builder()
            .max_content_width(ctx.content_rect().width())
            .build()
            .show(ctx, |ui| {
                if let Some(error) = self.initialization.error() {
                    self.initialization_error_page(ui, error);
                    return;
                }
                let selected_task = self
                    .gui_catalog
                    .as_ref()
                    .ok()
                    .and_then(|catalog| self.task_browser.primary_task(catalog))
                    .cloned();
                let runtime_running = self.runtime_running_for_controls();
                let event = match self.session.current_page() {
                    Page::QuickStart => self.interface_controls.quickstart_page(
                        ui,
                        &mut self.task_browser,
                        &self.gui_catalog,
                    ),
                    Page::Interface => self.interface_controls.configuration_page(
                        ui,
                        &mut self.task_browser,
                        &mut self.session,
                        selected_task,
                        runtime_running,
                    ),
                    Page::Metrics => {
                        self.metrics.show(ui);
                        None
                    }
                };
                self.handle_interface_control_event(event);
                self.about_window(ui);
                self.virtual_keyboard_window(ui);
                let event = self.interface_controls.customization_window(
                    ui,
                    &mut self.task_browser,
                    &mut self.session,
                    self.gui_catalog.as_ref().ok(),
                    runtime_running,
                );
                self.handle_interface_control_event(event);
            });

        if matches!(
            self.session.current_page(),
            Page::QuickStart | Page::Interface
        ) || self.interface_controls.customization_visible()
        {
            self.publish_runtime_params();
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(
            storage,
            eframe::APP_KEY,
            &PersistedApp { theme: self.theme },
        );
    }
}

trait RuntimeParamsPublisher {
    fn publish_runtime_params(
        &mut self,
        params: RuntimeParams,
    ) -> Result<RuntimeParams, crate::gui_ros::GuiRosError>;

    fn resend_all_on_next_publish(&mut self);
}

impl RuntimeParamsPublisher for GuiRosControls {
    fn publish_runtime_params(
        &mut self,
        params: RuntimeParams,
    ) -> Result<RuntimeParams, crate::gui_ros::GuiRosError> {
        GuiRosControls::publish_runtime_params(self, params)
    }

    fn resend_all_on_next_publish(&mut self) {
        GuiRosControls::resend_all_on_next_publish(self);
    }
}

#[derive(Debug, Default)]
struct RuntimeParamsDelivery {
    armed_launch: Option<u64>,
    retry_at: Option<std::time::Instant>,
    retry_delay: std::time::Duration,
}

impl RuntimeParamsDelivery {
    const INITIAL_RETRY_DELAY: std::time::Duration = std::time::Duration::from_millis(250);
    const MAX_RETRY_DELAY: std::time::Duration = std::time::Duration::from_secs(5);
}

struct LaunchRuntimeParamsPublisher<'a, P: RuntimeParamsPublisher> {
    publisher: &'a mut P,
    delivery: &'a mut RuntimeParamsDelivery,
    runtime_session: &'a GuiRuntimeSession,
    now: std::time::Instant,
}

impl<'a, P: RuntimeParamsPublisher> LaunchRuntimeParamsPublisher<'a, P> {
    fn new(
        publisher: &'a mut P,
        delivery: &'a mut RuntimeParamsDelivery,
        runtime_session: &'a GuiRuntimeSession,
        now: std::time::Instant,
    ) -> Self {
        Self {
            publisher,
            delivery,
            runtime_session,
            now,
        }
    }

    fn publish_runtime_params(
        &mut self,
        params: RuntimeParams,
    ) -> Option<Result<RuntimeParams, crate::gui_ros::GuiRosError>> {
        let launch = self.runtime_session.launch_generation();
        if self.delivery.armed_launch != Some(launch) {
            self.publisher.resend_all_on_next_publish();
            self.delivery.armed_launch = Some(launch);
            self.delivery.retry_at = None;
            self.delivery.retry_delay = std::time::Duration::ZERO;
        }
        if self.delivery.retry_at.is_some_and(|at| self.now < at) {
            return None;
        }
        let result = self.publisher.publish_runtime_params(params);
        if result.is_ok() {
            self.delivery.retry_at = None;
            self.delivery.retry_delay = std::time::Duration::ZERO;
        } else {
            let delay = if self.delivery.retry_delay.is_zero() {
                RuntimeParamsDelivery::INITIAL_RETRY_DELAY
            } else {
                (self.delivery.retry_delay * 2).min(RuntimeParamsDelivery::MAX_RETRY_DELAY)
            };
            self.delivery.retry_delay = delay;
            self.delivery.retry_at = Some(self.now + delay);
        }
        Some(result)
    }
}

/// Published runtime parameters for trajectory `DATA` rows (`gravity` in m/s^2,
/// `latency_ms` in milliseconds, `motion_sensitivity` as a factor; non-finite as `null`).
fn runtime_params_record(params: RuntimeParams) -> serde_json::Value {
    serde_json::json!({
        "gravity": params.gravity,
        "latency_ms": params.latency_ms,
        "motion_sensitivity": params.motion_sensitivity,
    })
}

fn restore_theme(storage: &dyn eframe::Storage) -> Option<egui::Theme> {
    eframe::get_value::<PersistedApp>(storage, eframe::APP_KEY).map(|stored| stored.theme)
}

impl App {
    fn runtime_running_for_controls(&mut self) -> bool {
        let query = self.runtime_session.is_running();
        self.runtime_running_from_query(query)
    }

    fn runtime_running_from_query(&mut self, query: Result<bool, GuiRuntimeError>) -> bool {
        match query {
            Ok(running) => running,
            Err(error) => {
                let workflow_error = GuiWorkflowError::Runtime(error);
                error!("Failed to query GUI subprocess state: {workflow_error}");
                self.record_workflow_error(workflow_error);

                // Unknown runtime state must keep launch disabled. A later successful
                // query supplies the fresh state; the bounded error remains evidence.
                true
            }
        }
    }

    fn record_workflow_error(&mut self, workflow_error: GuiWorkflowError) {
        if self.workflow_errors.contains(&workflow_error) {
            return;
        }

        self.workflow_errors.push(workflow_error);
        if self.workflow_errors.len() > Self::MAX_WORKFLOW_ERRORS {
            self.workflow_errors.remove(0);
        }
    }

    fn app_header(&mut self, ui: &mut egui::Ui) {
        ui.set_height(Self::HEADER_HEIGHT);
        let header_rect = egui::Rect::from_min_size(
            ui.max_rect().min,
            egui::vec2(ui.max_rect().width(), Self::HEADER_HEIGHT),
        );
        let stroke = ui.visuals().widgets.noninteractive.bg_stroke;
        ui.painter().line_segment(
            [
                egui::pos2(header_rect.left(), header_rect.bottom() - 0.5),
                egui::pos2(header_rect.right(), header_rect.bottom() - 0.5),
            ],
            stroke,
        );

        let nav_size = egui::vec2(
            Self::navigation_segment_width(),
            Self::HEADER_CONTENT_HEIGHT,
        );
        let configure_size = egui::vec2(Self::HEADER_CONFIG_WIDTH, Self::HEADER_NAV_BUTTON_HEIGHT);
        let configure_rect = egui::Rect::from_center_size(
            egui::pos2(
                header_rect.right() - Self::HEADER_SIDE_MARGIN - configure_size.x / 2.0,
                header_rect.center().y,
            ),
            configure_size,
        );
        let min_nav_center_x = header_rect.left() + Self::HEADER_SIDE_MARGIN + nav_size.x / 2.0;
        let max_nav_center_x = configure_rect.left() - 12.0 - nav_size.x / 2.0;
        let nav_center_x = header_rect
            .center()
            .x
            .min(max_nav_center_x)
            .max(min_nav_center_x);
        let nav_rect = egui::Rect::from_center_size(
            egui::pos2(nav_center_x, header_rect.center().y),
            nav_size,
        );

        ui.scope_builder(
            egui::UiBuilder::new()
                .id_salt("header_navigation")
                .max_rect(nav_rect)
                .layout(egui::Layout::left_to_right(egui::Align::Center)),
            |ui| {
                ui.set_min_size(nav_size);
                self.navigation_buttons(ui);
            },
        );
        ui.scope_builder(
            egui::UiBuilder::new()
                .id_salt("header_configuration")
                .max_rect(configure_rect)
                .layout(egui::Layout::left_to_right(egui::Align::Center)),
            |ui| {
                ui.set_min_size(configure_size);
                self.advanced_opts_button(ui);
            },
        );
    }

    fn navigation_segment_width() -> f32 {
        let page_count = Page::iter().count() as f32;
        let button_gap = 2.0 * (page_count - 1.0).max(0.0);
        let frame_margin = 4.0;
        frame_margin + page_count * Self::HEADER_NAV_BUTTON_WIDTH + button_gap
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.add_space(3.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            self.dark_mode_toggle_button(ui);
            self.warn_if_debug_build(ui);
            if let Some(error) = self.workflow_errors.last() {
                ui.colored_label(
                    egui::Color32::LIGHT_RED,
                    format!("{}: {}", error.category().label(), error),
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                self.show_trajectory_collection_checkbox(ui);
            });
        });
    }

    fn navigation_buttons(&mut self, ui: &mut egui::Ui) {
        egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 2,
                right: 2,
                top: 2,
                bottom: 2,
            })
            .corner_radius(egui::CornerRadius::same(5))
            .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.18))
            .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 2.0;
                    ui.spacing_mut().button_padding = egui::vec2(10.0, 4.0);
                    for page in Page::iter() {
                        let selected = self.session.current_page() == page;
                        let text = if selected {
                            egui::RichText::new(page.title()).strong().size(15.0)
                        } else {
                            egui::RichText::new(page.title()).size(15.0)
                        };
                        let mut button = egui::Button::new(text).frame(true).min_size(egui::vec2(
                            Self::HEADER_NAV_BUTTON_WIDTH,
                            Self::HEADER_NAV_BUTTON_HEIGHT,
                        ));
                        if selected {
                            button =
                                button.fill(ui.visuals().selection.bg_fill.gamma_multiply(0.48));
                        } else {
                            button = button.fill(ui.visuals().faint_bg_color.gamma_multiply(0.28));
                        }

                        let response = ui
                            .add_sized(
                                egui::vec2(
                                    Self::HEADER_NAV_BUTTON_WIDTH,
                                    Self::HEADER_NAV_BUTTON_HEIGHT,
                                ),
                                button,
                            )
                            .on_hover_text(if selected {
                                format!("{} (current page)", page.description())
                            } else {
                                page.description().to_string()
                            });

                        if response.clicked() && !selected {
                            self.session.select_page(page);
                        }
                    }
                });
            });
    }

    fn advanced_opts_button(&mut self, ui: &mut egui::Ui) {
        let visible = self.interface_controls.customization_visible();
        let text = if visible {
            egui::RichText::new("Configure").strong().size(15.0)
        } else {
            egui::RichText::new("Configure").size(15.0)
        };
        let mut button = egui::Button::new(text).frame(true).min_size(egui::vec2(
            Self::HEADER_CONFIG_WIDTH,
            Self::HEADER_NAV_BUTTON_HEIGHT,
        ));
        if visible {
            button = button.fill(ui.visuals().selection.bg_fill.gamma_multiply(0.48));
        } else {
            button = button.fill(ui.visuals().faint_bg_color.gamma_multiply(0.28));
        }
        let button = ui
            .add_sized(
                egui::vec2(Self::HEADER_CONFIG_WIDTH, Self::HEADER_NAV_BUTTON_HEIGHT),
                button,
            )
            .on_hover_text(if visible {
                "Hide task configuration"
            } else {
                "Show task configuration"
            });
        if button.clicked() {
            self.interface_controls.toggle_customization();
        }
    }

    fn about_window(&mut self, ui: &mut egui::Ui) {
        if self.session.show_about() {
            let available_rect = ui.ctx().available_rect();
            let center_point = available_rect.center();

            egui::containers::Window::new(egui::RichText::new("About").size(18.0))
                .interactable(true)
                .open(self.session.show_about_mut())
                .collapsible(false)
                .resizable(false)
                .fixed_size([512.0, 1024.0])
                .default_rect(egui::Rect {
                    min: egui::Pos2::new(
                        center_point.x - 512.0 / 2.0,
                        center_point.y - 1024.0 / 2.0,
                    ),
                    max: egui::Pos2::new(
                        center_point.x + 512.0 / 2.0,
                        center_point.y + 1024.0 / 2.0,
                    ),
                })
                .show(ui.ctx(), |ui| {
                    ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                        // Keep this source-relative path valid in the crate checked by `cargo
                        // publish`.
                        CommonMarkViewer::new().show(
                            ui,
                            &mut self.commonmark_cache,
                            include_str!("../content/about.md"),
                        );
                    });
                });
        }
    }

    fn virtual_keyboard_window(&mut self, ui: &mut egui::Ui) {
        if self.interface_controls.virtual_keyboard_visible()
            && self.session.current_page() == Page::Interface
        {
            let available_rect = ui.ctx().available_rect();

            egui::containers::Window::new(egui::RichText::new("Gripper").size(16.0))
                .interactable(true)
                .collapsible(true)
                .resizable(false)
                .default_rect(egui::Rect {
                    min: egui::Pos2::new(
                        available_rect.max.x - 150.0,
                        available_rect.max.y - 100.0,
                    ),
                    max: egui::Pos2::new(available_rect.max.x - 10.0, available_rect.max.y - 10.0),
                })
                .show(ui.ctx(), |ui| {
                    ui.with_layout(
                        egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                        |ui| {
                            let button = egui::Button::new("🤏 Gripper Toggle");
                            if ui
                                .add(button)
                                .on_hover_text("Click to toggle gripper (Spacebar alternative)")
                                .clicked()
                            {
                                if let Err(error) = self.ros.publish_gripper_toggle() {
                                    error!("Failed to publish gripper toggle: {error}");
                                }
                            }
                        },
                    );
                });
        }
    }

    fn handle_interface_control_event(&mut self, event: Option<GuiInterfaceControlEvent>) {
        let outcome = GuiWorkflow::handle_event(
            event,
            &mut self.task_browser,
            &mut self.session,
            &mut self.runtime_session,
            &self.ros,
        );
        for error in outcome.errors {
            self.record_workflow_error(error);
        }
    }

    fn initialization_error_page(&self, ui: &mut egui::Ui, error: &GuiWorkflowError) {
        ui.vertical_centered(|ui| {
            ui.heading("GUI initialization unavailable");
            ui.label(format!("Category: {}", error.category().label()));
            match error {
                GuiWorkflowError::CatalogLoad(cache_error) => {
                    ui.label(cache_error.summary());
                    ui.label(cache_error.recovery_hint());
                    for command in cache_error.recovery_commands() {
                        ui.label(format!("{}: {}", command.label(), command.command()));
                    }
                }
                _ => {
                    ui.label(error.to_string());
                }
            };
            ui.add_space(8.0);
            ui.label("No catalog selection, session mutation, or runtime process was accepted.");
            ui.label("Fix the error above, then reopen the GUI.");
        });
    }

    pub fn dark_mode_toggle_button(&mut self, ui: &mut egui::Ui) {
        let (icon, tooltip, target_visuals) = match self.theme {
            egui::Theme::Dark => (
                "\u{e51c}",
                "Switch to light mode",
                crate::style::light_visuals(),
            ),
            egui::Theme::Light => (
                "\u{e518}",
                "Switch to dark mode",
                crate::style::dark_visuals(),
            ),
        };

        if ui
            .add(egui::Button::new(icon).min_size(egui::vec2(28.0, 24.0)))
            .on_hover_text(tooltip)
            .clicked()
        {
            ui.ctx().set_visuals(target_visuals.to_owned());
            self.theme = match self.theme {
                egui::Theme::Dark => egui::Theme::Light,
                egui::Theme::Light => egui::Theme::Dark,
            };
        }
    }

    fn publish_runtime_params(&mut self) {
        let published = LaunchRuntimeParamsPublisher::new(
            &mut self.ros,
            &mut self.runtime_params_delivery,
            &self.runtime_session,
            std::time::Instant::now(),
        )
        .publish_runtime_params(self.session.runtime_params());
        if let Some(published) = published {
            self.record_runtime_params_publish(published);
        }
    }

    fn record_runtime_params_publish(
        &mut self,
        published: Result<RuntimeParams, crate::gui_ros::GuiRosError>,
    ) {
        match published {
            Ok(params) => {
                self.session.set_runtime_params(params);
                self.runtime_session
                    .set_published_runtime_params(runtime_params_record(params));
            }
            Err(error) => {
                error!("Failed to publish runtime parameters: {error}");
                self.record_workflow_error(GuiWorkflowError::Runtime(GuiRuntimeError::new(
                    format!("Failed to publish runtime parameters: {error}"),
                )));
            }
        }
    }

    fn show_trajectory_collection_checkbox(&mut self, ui: &mut egui::Ui) {
        ui.add(egui::Checkbox::new(
            self.session.collect_trajectory_mut(),
            egui::RichText::new(format!(
                "Record episode configs  |  {} records",
                self.runtime_session.collected_trajectories()
            ))
            .size(14.0),
        ))
        .on_hover_text("Log the task configuration when restarting a running episode.");
    }

    fn warn_if_debug_build(&mut self, ui: &mut egui::Ui) {
        if cfg!(debug_assertions) {
            ui.separator();
            ui.add(egui::Button::new("⚠ Debug build ⚠"))
                .on_hover_ui(|ui| {
                    ui.label(
                        egui::RichText::new(format!(
                            "Current page: {:?}\n\
                                     Screen size:  {:?}\n\
                                    ",
                            self.session.current_page(),
                            ui.ctx().content_rect().size(),
                        ))
                        .font(egui::FontId::monospace(
                            ui.style().text_styles[&egui::TextStyle::Button].size,
                        )),
                    );
                });
            ui.separator();
        }
    }
}
