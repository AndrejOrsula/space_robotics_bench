use crate::cache;
use crate::command_contract::redact_shell_command;
use crate::env_cfg::{Domain, TaskConfig, Workflow};
use crate::gui_cache_recovery::{self, GuiCacheRecoveryAction};
use crate::gui_catalog::{GuiCatalog, GuiCatalogTask};
use crate::gui_launch::GuiLaunchRequest;
use crate::gui_selection::GuiSelection;
use crate::gui_session::GuiSessionState;
use crate::gui_task_browser::GuiTaskBrowser;
use crate::gui_task_overrides::{humanize_display_value, GuiTaskOverrideChoice};
use egui::ScrollArea;
use strum::IntoEnumIterator;

#[derive(Clone, Debug)]
pub enum GuiInterfaceControlEvent {
    StartSelectedTask,
    StartLaunchRequest(GuiLaunchRequest),
    SelectCatalogTask(GuiCatalogTask),
    StartCatalogTask(GuiCatalogTask),
    SelectTaskById {
        task_id: String,
        catalog_task: Option<GuiCatalogTask>,
    },
    RestartEpisode,
    ExitToTasks,
}

#[derive(Clone, Debug, Default)]
pub struct GuiInterfaceControls {
    show_customization: bool,
    reset_customization_scroll: bool,
    show_virtual_keyboard: bool,
    /// Whether a subprocess was running when the window was drawn; Core controls
    /// then edit the next launch.
    customization_runtime_running: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct CommandCopyControl<'a> {
    label: &'a str,
    command: &'a str,
}

impl<'a> CommandCopyControl<'a> {
    const fn new(label: &'a str, command: &'a str) -> Self {
        Self { label, command }
    }
}

impl GuiInterfaceControls {
    #[must_use]
    pub fn customization_visible(&self) -> bool {
        self.show_customization
    }

    pub fn toggle_customization(&mut self) {
        if !self.show_customization {
            self.reset_customization_scroll = true;
        }
        self.show_customization = !self.show_customization;
    }

    #[must_use]
    pub fn virtual_keyboard_visible(&self) -> bool {
        self.show_virtual_keyboard
    }

    pub fn quickstart_page(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        gui_catalog: &cache::GuiCatalogCacheResult,
    ) -> Option<GuiInterfaceControlEvent> {
        match gui_catalog {
            Ok(catalog) => self.catalog_quickstart_page(ui, task_browser, catalog),
            Err(error) => self.catalog_error_page(ui, task_browser, error),
        }
    }

    pub fn configuration_page(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        selected_task: Option<GuiCatalogTask>,
        runtime_running: bool,
    ) -> Option<GuiInterfaceControlEvent> {
        let mut event = None;
        ScrollArea::vertical().show(ui, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                Self::constrain_page_width(ui, 860.0);
                ui.spacing_mut().item_spacing = egui::vec2(12.0, 12.0);

                self.interface_overview(ui, task_browser, selected_task.as_ref(), runtime_running);
                self.interface_environment_section(ui, session);
                event = self.session_controls(ui, runtime_running);
            });
        });
        event
    }

    pub fn customization_window(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        gui_catalog: Option<&GuiCatalog>,
        runtime_running: bool,
    ) -> Option<GuiInterfaceControlEvent> {
        if !self.show_customization {
            return None;
        }
        self.customization_runtime_running = runtime_running;

        let mut event = None;
        let mut window_open = self.show_customization;
        let available_rect = ui.ctx().available_rect();
        let window_width = available_rect.width().clamp(360.0, 620.0);
        let window_height = available_rect.height().clamp(520.0, 700.0);
        egui::containers::Window::new(egui::RichText::new("Task configuration").size(20.0))
            .interactable(true)
            .open(&mut window_open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-16.0, 16.0))
            .fixed_size([window_width, window_height])
            .show(ui.ctx(), |ui| {
                ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                    let mut scroll_area = ScrollArea::vertical()
                        .id_salt("customization_scroll_area")
                        .max_height(window_height - 88.0);
                    if self.reset_customization_scroll {
                        scroll_area = scroll_area.vertical_scroll_offset(0.0);
                    }
                    scroll_area.show(ui, |ui| {
                        Self::customization_section(ui, "Core", |ui| {
                            if let Some(control_event) = self.core_customization_controls(
                                ui,
                                task_browser,
                                session,
                                gui_catalog,
                            ) {
                                event = Some(control_event);
                            }
                        });
                        Self::customization_section(ui, "Task overrides", |ui| {
                            self.task_specific_override_controls(ui, task_browser, gui_catalog);
                        });
                        Self::customization_section(ui, "Advanced", |ui| {
                            self.advanced_interface_controls(ui, task_browser);
                        });
                        Self::customization_section(ui, "CLI equivalent", |ui| {
                            merge_control_event(
                                &mut event,
                                self.command_preview_controls(ui, task_browser, "Run"),
                            );
                        });
                    });
                    self.reset_customization_scroll = false;
                    ui.separator();

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add_sized(
                                egui::vec2(172.0, 36.0),
                                Self::primary_button(ui, "Run selected task"),
                            )
                            .clicked()
                        {
                            event = Some(GuiInterfaceControlEvent::StartSelectedTask);
                        }
                    });
                });
            });
        self.show_customization = window_open;

        event
    }

    fn catalog_quickstart_page(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        catalog: &GuiCatalog,
    ) -> Option<GuiInterfaceControlEvent> {
        let mut event = None;
        ScrollArea::vertical().show(ui, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                Self::constrain_page_width(ui, 1024.0);
                ui.spacing_mut().item_spacing = egui::vec2(12.0, 12.0);

                Self::catalog_overview_header(ui, catalog);
                event = self.selected_task_summary(ui, task_browser, catalog);
                if event.is_none() {
                    self.catalog_filter_bar(ui, task_browser, catalog);
                    event = self.catalog_task_picker(ui, task_browser, catalog);
                }
            });
        });
        event
    }

    fn selected_task_summary(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        catalog: &GuiCatalog,
    ) -> Option<GuiInterfaceControlEvent> {
        let selected_task = task_browser.primary_task(catalog).cloned();

        let mut event = None;
        Self::interface_panel_frame(ui).show(ui, |ui| {
            Self::fill_width(ui);
            let Some(task) = selected_task else {
                ui.label(
                    "No GUI tasks are available in the catalog cache. Run `srb cache update`.",
                );
                return;
            };
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Selected").weak());
                ui.add(
                    egui::Label::new(egui::RichText::new(&task.display_name).strong().size(20.0))
                        .wrap(),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .add_sized(
                            egui::vec2(172.0, 36.0),
                            Self::primary_button(ui, "Run selected task"),
                        )
                        .clicked()
                    {
                        event = Some(GuiInterfaceControlEvent::StartCatalogTask(task.clone()));
                    }
                });
            });
        });
        event
    }

    fn catalog_overview_header(ui: &mut egui::Ui, catalog: &GuiCatalog) {
        let counts = [
            (catalog.tasks.len(), "task", "tasks"),
            (catalog.categories().len(), "category", "categories"),
            (catalog.domain_options().len(), "domain", "domains"),
            (catalog.robot_options().len(), "robot", "robots"),
        ]
        .map(|(count, one, many)| format!("{count} {}", if count == 1 { one } else { many }))
        .join(" · ");
        ui.horizontal_wrapped(|ui| {
            ui.label(
                egui::RichText::new("Space Robotics Bench")
                    .strong()
                    .size(24.0),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(egui::RichText::new(counts).weak());
            });
        });
    }

    fn catalog_filter_bar(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        catalog: &GuiCatalog,
    ) {
        let filter_choices = task_browser.filter_choices(catalog);
        let gap = 10.0;
        let clear_width = 120.0;
        let narrow = ui.available_width() < 520.0;
        let field_width = if narrow {
            ui.available_width()
        } else {
            ((ui.available_width() - clear_width - gap * 2.0) / 2.0).max(160.0)
        };
        let add_filters = |ui: &mut egui::Ui| {
            ui.spacing_mut().item_spacing = egui::vec2(gap, 8.0);
            ui.add_sized(
                egui::vec2(field_width, 28.0),
                egui::TextEdit::singleline(&mut task_browser.filters.query)
                    .hint_text("Search tasks"),
            );
            egui::ComboBox::new("catalog_category_filter", "")
                .width(field_width)
                .wrap()
                .selected_text(
                    task_browser
                        .filters
                        .category
                        .as_deref()
                        .unwrap_or("All categories"),
                )
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut task_browser.filters.category, None, "All categories");
                    for category in &filter_choices.categories {
                        ui.selectable_value(
                            &mut task_browser.filters.category,
                            Some(category.clone()),
                            category,
                        );
                    }
                });
            if ui
                .add_enabled(
                    task_browser.filter_active(),
                    egui::Button::new("Clear").min_size(egui::vec2(clear_width, 28.0)),
                )
                .clicked()
            {
                task_browser.clear_filters();
            }
        };
        if narrow {
            ui.vertical(add_filters);
        } else {
            ui.horizontal(add_filters);
        }
    }

    fn catalog_task_picker(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        catalog: &GuiCatalog,
    ) -> Option<GuiInterfaceControlEvent> {
        let tasks = task_browser.filtered_tasks(catalog);
        let task_count = tasks.len();

        let mut event = None;
        Self::interface_panel_frame(ui).show(ui, |ui| {
            Self::fill_width(ui);
            ui.vertical(|ui| {
                Self::section_header(ui, "Tasks", Some(&format!("{task_count} available")));
                ui.add_space(6.0);

                ScrollArea::vertical()
                    .max_height(460.0)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if tasks.is_empty() {
                            ui.label("No tasks match the active filters.");
                            return;
                        }

                        for task in &tasks {
                            if event.is_none() {
                                event = self.task_picker_row(ui, task_browser, task);
                            } else {
                                self.task_picker_row(ui, task_browser, task);
                            }
                            ui.add_space(6.0);
                        }
                    });
            });
        });
        event
    }

    fn task_picker_row(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        task: &GuiCatalogTask,
    ) -> Option<GuiInterfaceControlEvent> {
        let selected = task_browser.selection.task_config().task == task.id;
        let fill = if selected {
            ui.visuals().selection.bg_fill.gamma_multiply(0.32)
        } else {
            ui.visuals().faint_bg_color.gamma_multiply(0.20)
        };
        let stroke = if selected {
            egui::Stroke::new(1.0_f32, ui.visuals().selection.bg_fill.gamma_multiply(0.85))
        } else {
            ui.visuals().widgets.noninteractive.bg_stroke
        };

        let row = egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 10,
                right: 10,
                top: 8,
                bottom: 8,
            })
            .corner_radius(egui::CornerRadius::same(5))
            .fill(fill)
            .stroke(stroke)
            .show(ui, |ui| {
                Self::fill_width(ui);
                if ui.available_width() < 560.0 {
                    ui.vertical(|ui| {
                        Self::task_thumbnail(
                            ui,
                            task,
                            egui::vec2(ui.available_width().min(240.0), 128.0),
                        );
                        ui.add_space(6.0);
                        Self::task_picker_details(ui, task);
                        ui.add_space(6.0);
                        Self::task_picker_selection_indicator(ui, selected);
                    });
                } else {
                    ui.horizontal(|ui| {
                        Self::task_thumbnail(ui, task, egui::vec2(104.0, 72.0));
                        ui.add_space(8.0);
                        ui.vertical(|ui| {
                            Self::task_picker_details(ui, task);
                        });
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            Self::task_picker_selection_indicator(ui, selected);
                        });
                    });
                }
            });

        let response = row
            .response
            .interact(egui::Sense::click())
            .on_hover_text("Select this task");
        if response.clicked() && !selected {
            Some(GuiInterfaceControlEvent::SelectCatalogTask(task.clone()))
        } else {
            None
        }
    }

    fn task_picker_details(ui: &mut egui::Ui, task: &GuiCatalogTask) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(&task.display_name).strong());
            if task.featured {
                ui.label(egui::RichText::new("Featured").weak().size(11.0));
            }
        });
        Self::task_tagline(ui, task);
        Self::task_metadata(ui, task);
    }

    fn task_tagline(ui: &mut egui::Ui, task: &GuiCatalogTask) {
        let tagline = task.tagline.trim();
        if !tagline.is_empty() {
            ui.add(
                egui::Label::new(egui::RichText::new(tagline).weak().size(12.0))
                    .wrap()
                    .selectable(false),
            );
        }
    }

    fn task_picker_selection_indicator(ui: &mut egui::Ui, selected: bool) {
        if selected {
            ui.label(egui::RichText::new("Selected").weak().strong().size(12.0));
        }
    }

    fn task_metadata(ui: &mut egui::Ui, task: &GuiCatalogTask) {
        let mut metadata = vec![
            ("Domain", task.defaults.domain.as_str()),
            ("Category", task.category.as_str()),
        ];
        if let Some(robot) = task
            .defaults
            .robot
            .as_deref()
            .filter(|robot| !robot.is_empty())
        {
            metadata.push(("Robot", robot));
        }

        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            for (label, value) in metadata {
                ui.label(egui::RichText::new(label).weak().size(12.0));
                ui.label(
                    egui::RichText::new(if label == "Domain" {
                        humanize_display_value(&value.to_ascii_lowercase())
                    } else {
                        humanize_display_value(value)
                    })
                    .size(12.0),
                );
                ui.add_space(10.0);
            }
        });
    }

    fn task_thumbnail(
        ui: &mut egui::Ui,
        task: &GuiCatalogTask,
        size: egui::Vec2,
    ) -> egui::Response {
        let thumbnail = task.resolved_thumbnail();
        let image = egui::Image::from_uri(thumbnail.file_uri()).fit_to_exact_size(size);

        egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 4,
                right: 4,
                top: 4,
                bottom: 4,
            })
            .corner_radius(egui::CornerRadius::same(5))
            .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.28))
            .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
            .show(ui, |ui| ui.add(image))
            .inner
            .on_hover_text(task.display_name.as_str())
    }

    fn section_header(ui: &mut egui::Ui, title: &str, aside: Option<&str>) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(title).strong().size(18.0));
            if let Some(aside) = aside {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(egui::RichText::new(aside).weak().size(13.0));
                });
            }
        });
    }

    fn catalog_error_page(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        error: &cache::GuiCatalogCacheError,
    ) -> Option<GuiInterfaceControlEvent> {
        ui.heading("GUI catalog cache missing or invalid");
        ui.label(error.summary());
        ui.add_space(8.0);
        let recovery_actions = gui_cache_recovery::actions(error.recovery_commands());
        Self::recovery_command_controls(ui, "Cache recovery", &recovery_actions);
        ui.separator();
        self.command_preview_controls(ui, task_browser, "Run Raw Command")
    }

    fn recovery_command_controls(
        ui: &mut egui::Ui,
        title: &str,
        actions: &[GuiCacheRecoveryAction<'_>],
    ) {
        Self::section_header(ui, title, None);
        ui.add_space(4.0);
        for action in actions {
            Self::copyable_command_action(ui, *action);
        }
    }

    fn copyable_command_action(ui: &mut egui::Ui, action: GuiCacheRecoveryAction<'_>) {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new(action.label).strong().size(12.0));
            ui.label(egui::RichText::new(action.description).weak().size(12.0));
            let copy_control = Self::recovery_copy_control(action);
            if ui.available_width() < 420.0 {
                Self::readonly_command_field(ui, action.command, f32::INFINITY);
                Self::command_copy_button(ui, copy_control);
            } else {
                ui.horizontal(|ui| {
                    let copy_width = 72.0;
                    let command_width =
                        (ui.available_width() - copy_width - ui.spacing().item_spacing.x)
                            .max(180.0);
                    Self::readonly_command_field(ui, action.command, command_width);
                    Self::sized_command_copy_button(ui, copy_control, copy_width);
                });
            }
        });
    }

    fn recovery_copy_control(action: GuiCacheRecoveryAction<'_>) -> CommandCopyControl<'_> {
        CommandCopyControl::new("Copy", action.command)
    }

    fn preview_copy_control(command: &str) -> CommandCopyControl<'_> {
        CommandCopyControl::new("Copy Command", command)
    }

    fn command_copy_button(ui: &mut egui::Ui, control: CommandCopyControl<'_>) -> bool {
        let clicked = ui.button(control.label).clicked();
        if clicked {
            ui.ctx().copy_text(control.command.to_owned());
        }
        clicked
    }

    fn sized_command_copy_button(
        ui: &mut egui::Ui,
        control: CommandCopyControl<'_>,
        width: f32,
    ) -> bool {
        let clicked = ui
            .add_sized(egui::vec2(width, 28.0), egui::Button::new(control.label))
            .clicked();
        if clicked {
            ui.ctx().copy_text(control.command.to_owned());
        }
        clicked
    }

    fn readonly_command_field(ui: &mut egui::Ui, command: &str, width: f32) {
        let mut command_text = command.to_owned();
        ui.add_sized(
            egui::vec2(width, 28.0),
            egui::TextEdit::singleline(&mut command_text)
                .code_editor()
                .interactive(false),
        );
    }

    fn interface_panel_frame(ui: &egui::Ui) -> egui::Frame {
        egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 16,
                right: 16,
                top: 14,
                bottom: 14,
            })
            .corner_radius(egui::CornerRadius::same(6))
            .fill(ui.visuals().faint_bg_color.gamma_multiply(0.45))
            .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
    }

    fn primary_button(ui: &egui::Ui, label: &str) -> egui::Button<'static> {
        egui::Button::new(egui::RichText::new(label.to_string()).strong().size(16.0))
            .frame(true)
            .fill(ui.visuals().selection.bg_fill.gamma_multiply(0.46))
    }

    fn fixed_width_scope<R>(
        ui: &mut egui::Ui,
        width: f32,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> R {
        ui.scope(|ui| {
            ui.set_min_width(width);
            ui.set_max_width(width);
            ui.set_width(width);
            add_contents(ui)
        })
        .inner
    }

    fn constrain_page_width(ui: &mut egui::Ui, max_width: f32) {
        let width = ui.available_width().min(max_width);
        ui.set_min_width(width);
        ui.set_max_width(width);
        ui.set_width(width);
    }

    fn fill_width(ui: &mut egui::Ui) {
        let width = ui.available_width();
        ui.set_min_width(width);
        ui.set_max_width(width);
        ui.set_width(width);
    }

    fn interface_overview(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        selected_task: Option<&GuiCatalogTask>,
        runtime_running: bool,
    ) {
        let task = selected_task
            .map(|task| task.display_name.clone())
            .unwrap_or_else(|| task_browser.selection.task_config().task.clone());
        let domain = task_browser.selection.task_config().domain.to_string();
        let robot = task_browser.selection.task_config().robot.clone();
        let state = if runtime_running {
            "Running"
        } else {
            "Stopped"
        };

        Self::interface_panel_frame(ui).show(ui, |ui| {
            Self::fill_width(ui);
            if ui.available_width() < 650.0 {
                ui.vertical(|ui| {
                    Self::section_header(ui, "Session", Some(state));
                    if let Some(task) = selected_task {
                        ui.add_space(6.0);
                        Self::task_thumbnail(
                            ui,
                            task,
                            egui::vec2(ui.available_width().min(280.0), 140.0),
                        );
                    }
                    ui.add_space(6.0);
                    Self::interface_status_tiles(ui, &task, &domain, &robot);
                });
            } else {
                ui.horizontal(|ui| {
                    if let Some(task) = selected_task {
                        Self::task_thumbnail(ui, task, egui::vec2(164.0, 104.0));
                        ui.add_space(14.0);
                    }
                    ui.vertical(|ui| {
                        Self::section_header(ui, "Session", Some(state));
                        ui.add_space(6.0);
                        Self::interface_status_tiles(ui, &task, &domain, &robot);
                    });
                });
            }
        });
    }

    fn interface_status_tiles(ui: &mut egui::Ui, task: &str, domain: &str, robot: &str) {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            for (label, value) in [("Task", task), ("Domain", domain), ("Robot", robot)] {
                ui.label(egui::RichText::new(label).weak());
                ui.label(egui::RichText::new(humanize_display_value(value)).strong());
                ui.add_space(12.0);
            }
        });
    }

    fn interface_environment_section(&mut self, ui: &mut egui::Ui, session: &mut GuiSessionState) {
        Self::interface_panel_frame(ui).show(ui, |ui| {
            Self::fill_width(ui);
            ui.vertical(|ui| {
                Self::section_header(ui, "Environment", Some("Live tuning"));
                ui.add_space(6.0);

                if ui.available_width() < 640.0 {
                    Self::environment_control_card(ui, |ui| {
                        self.gravity_control_panel(ui, session);
                    });
                    ui.add_space(12.0);
                    Self::environment_control_card(ui, |ui| {
                        self.latency_control_panel(ui, session);
                    });
                } else {
                    let gap = 14.0;
                    let card_width = ((ui.available_width() - gap) / 2.0).max(300.0);
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = gap;
                        Self::fixed_width_scope(ui, card_width, |ui| {
                            Self::environment_control_card(ui, |ui| {
                                self.gravity_control_panel(ui, session);
                            });
                        });
                        Self::fixed_width_scope(ui, card_width, |ui| {
                            Self::environment_control_card(ui, |ui| {
                                self.latency_control_panel(ui, session);
                            });
                        });
                    });
                }
            });
        });
    }

    fn environment_control_card<R>(
        ui: &mut egui::Ui,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> R {
        egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 12,
                right: 12,
                top: 10,
                bottom: 10,
            })
            .corner_radius(egui::CornerRadius::same(5))
            .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.22))
            .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
            .show(ui, |ui| {
                let width = ui.available_width();
                ui.set_min_width(width);
                ui.set_max_width(width);
                add_contents(ui)
            })
            .inner
    }

    fn gravity_control_panel(&mut self, ui: &mut egui::Ui, session: &mut GuiSessionState) {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new("Gravity").strong());
            let value = &mut session.runtime_params_mut().gravity;
            let value_text = format!("{:.2} m/s²", *value);
            Self::slider_with_value(
                ui,
                egui::Slider::new(value, 0.0..=25.0)
                    .trailing_fill(true)
                    .show_value(false)
                    .custom_formatter(|x, _| format!("{x:.2}"))
                    .custom_parser(|x| x.parse::<f64>().ok().map(|x| x.max(0.0_f64))),
                value_text,
            )
            .on_hover_text("Acceleration due to gravity");
            self.gravity_preset_buttons(ui, session);
        });
    }

    fn latency_control_panel(&mut self, ui: &mut egui::Ui, session: &mut GuiSessionState) {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new("Latency").strong());
            let value = &mut session.runtime_params_mut().latency_ms;
            let value_text = format!("{:.1} ms", *value);
            Self::slider_with_value(
                ui,
                egui::Slider::new(value, 0.0..=2000.0)
                    .trailing_fill(true)
                    .show_value(false)
                    .custom_formatter(|x, _| format!("{x:.1}")),
                value_text,
            )
            .on_hover_text("One-way communication delay between the operator and the robot");
            self.latency_preset_buttons(ui, session);
        });
    }

    fn slider_with_value(
        ui: &mut egui::Ui,
        slider: egui::Slider<'_>,
        value_text: String,
    ) -> egui::Response {
        ui.horizontal(|ui| {
            let value_width = 78.0;
            let slider_width = (ui.available_width() - value_width - 10.0).clamp(140.0, 300.0);
            ui.style_mut().spacing.slider_width = slider_width;
            let response = ui.add(slider);
            ui.add_space(8.0);
            ui.label(egui::RichText::new(value_text).monospace());
            response
        })
        .inner
    }

    fn gravity_preset_buttons(&mut self, ui: &mut egui::Ui, session: &mut GuiSessionState) {
        ui.horizontal_wrapped(|ui| {
            for (label, value) in [
                ("Zero", Domain::Orbit.gravity_magnitude()),
                ("Moon", Domain::Moon.gravity_magnitude()),
                ("Mars", Domain::Mars.gravity_magnitude()),
                ("Earth", 9.81),
                ("Jupiter", 24.79),
            ] {
                if Self::preset_button(
                    ui,
                    label,
                    (session.runtime_params().gravity - value).abs() < 0.01,
                )
                .clicked()
                {
                    session.runtime_params_mut().gravity = value;
                }
            }
        });
    }

    fn latency_preset_buttons(&mut self, ui: &mut egui::Ui, session: &mut GuiSessionState) {
        ui.horizontal_wrapped(|ui| {
            for (label, value) in [
                ("None", 0.0),
                ("LEO", 50.0),
                ("MEO", 225.0),
                ("GEO", 280.0),
                ("Moon", 1250.0),
            ] {
                if Self::preset_button(
                    ui,
                    label,
                    (session.runtime_params().latency_ms - value).abs() < 0.01,
                )
                .clicked()
                {
                    session.runtime_params_mut().latency_ms = value;
                }
            }
        });
    }

    fn preset_button(ui: &mut egui::Ui, label: &str, selected: bool) -> egui::Response {
        let text = if selected {
            egui::RichText::new(label).strong().size(14.0)
        } else {
            egui::RichText::new(label).size(14.0)
        };
        let mut button = egui::Button::new(text).frame(true);
        if selected {
            button = button.fill(ui.visuals().selection.bg_fill.gamma_multiply(0.42));
        }
        ui.add_sized(egui::vec2(62.0, 26.0), button)
    }

    fn session_controls(
        &mut self,
        ui: &mut egui::Ui,
        runtime_running: bool,
    ) -> Option<GuiInterfaceControlEvent> {
        let mut event = None;
        Self::interface_panel_frame(ui).show(ui, |ui| {
            Self::fill_width(ui);
            if ui.available_width() < 610.0 {
                ui.vertical(|ui| {
                    ui.spacing_mut().item_spacing.y = 9.0;
                    ui.label(egui::RichText::new("Session controls").strong().size(18.0));
                    let button_size = egui::vec2(ui.available_width(), 38.0);
                    event = Self::session_buttons(ui, button_size, runtime_running);
                });
            } else {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Session controls").strong().size(18.0));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        event = Self::session_buttons(ui, egui::vec2(176.0, 38.0), runtime_running);
                    });
                });
            }
        });
        event
    }

    fn session_buttons(
        ui: &mut egui::Ui,
        button_size: egui::Vec2,
        runtime_running: bool,
    ) -> Option<GuiInterfaceControlEvent> {
        let mut event = None;
        if !runtime_running
            && ui
                .add_sized(button_size, Self::primary_button(ui, "Run selected task"))
                .clicked()
        {
            event = Some(GuiInterfaceControlEvent::StartSelectedTask);
        }

        if ui
            .add_sized(
                button_size,
                egui::Button::new(egui::RichText::new("End session").size(16.0)).frame(true),
            )
            .on_hover_text("Stop the running task, reset session settings, and return to Tasks")
            .clicked()
        {
            event = Some(GuiInterfaceControlEvent::ExitToTasks);
        }

        if ui
            .add_enabled(
                runtime_running,
                egui::Button::new(egui::RichText::new("Restart episode").size(16.0))
                    .min_size(button_size)
                    .frame(true),
            )
            .on_disabled_hover_text("Start a task before restarting the episode")
            .clicked()
        {
            event = Some(GuiInterfaceControlEvent::RestartEpisode);
        }
        event
    }

    fn customization_section<R>(
        ui: &mut egui::Ui,
        title: &str,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> R {
        let response = egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 12,
                right: 12,
                top: 10,
                bottom: 10,
            })
            .corner_radius(egui::CornerRadius::same(6))
            .fill(ui.visuals().faint_bg_color.gamma_multiply(0.35))
            .show(ui, |ui| {
                Self::fill_width(ui);
                ui.label(egui::RichText::new(title).strong().size(17.0));
                ui.add_space(6.0);
                add_contents(ui)
            });
        ui.add_space(8.0);
        response.inner
    }

    fn core_customization_controls(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        gui_catalog: Option<&GuiCatalog>,
    ) -> Option<GuiInterfaceControlEvent> {
        ui.spacing_mut().item_spacing = egui::vec2(10.0, 10.0);
        if ui.available_width() < 560.0 {
            self.core_customization_controls_one_column(ui, task_browser, session, gui_catalog)
        } else {
            let mut event = None;
            let gap = 12.0;
            let field_width = ((ui.available_width() - gap) / 2.0).max(240.0);

            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = gap;
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.workflow_control(ui, task_browser);
                });
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.domain_control(ui, task_browser, session);
                });
            });
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = gap;
                Self::fixed_width_scope(ui, field_width, |ui| {
                    event = self.task_control(ui, task_browser, gui_catalog);
                });
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.robot_control(ui, task_browser);
                });
            });
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = gap;
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.tool_control(ui, task_browser);
                });
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.scenery_control(ui, task_browser);
                });
            });
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = gap;
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.rendering_mode_control(ui, task_browser);
                });
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.seed_control(ui, task_browser);
                });
            });
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = gap;
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.num_envs_control(ui, task_browser);
                });
                Self::fixed_width_scope(ui, field_width, |ui| {
                    self.hide_ui_control(ui, task_browser);
                });
            });
            event
        }
    }

    fn core_customization_controls_one_column(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        gui_catalog: Option<&GuiCatalog>,
    ) -> Option<GuiInterfaceControlEvent> {
        self.workflow_control(ui, task_browser);
        self.domain_control(ui, task_browser, session);
        let event = self.task_control(ui, task_browser, gui_catalog);
        self.robot_control(ui, task_browser);
        self.tool_control(ui, task_browser);
        self.scenery_control(ui, task_browser);
        self.rendering_mode_control(ui, task_browser);
        self.seed_control(ui, task_browser);
        self.num_envs_control(ui, task_browser);
        self.hide_ui_control(ui, task_browser);
        event
    }

    fn control_field<R>(
        ui: &mut egui::Ui,
        title: &str,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> R {
        Self::control_field_with_hint(ui, title, None, add_contents)
    }

    fn control_field_with_hint<R>(
        ui: &mut egui::Ui,
        title: &str,
        hint: Option<&str>,
        add_contents: impl FnOnce(&mut egui::Ui) -> R,
    ) -> R {
        egui::Frame::default()
            .inner_margin(egui::Margin {
                left: 4,
                right: 4,
                top: 6,
                bottom: 8,
            })
            .show(ui, |ui| {
                ui.vertical(|ui| {
                    let width = ui.available_width();
                    ui.set_min_width(width);
                    ui.set_max_width(width);
                    ui.set_min_height(56.0);
                    let label =
                        ui.add(egui::Label::new(egui::RichText::new(title).size(13.0)).wrap());
                    if let Some(hint) = hint {
                        label.on_hover_text(hint);
                    }
                    ui.add_space(4.0);
                    add_contents(ui)
                })
                .inner
            })
            .inner
    }

    fn control_width(ui: &egui::Ui) -> f32 {
        ui.available_width().clamp(180.0, 420.0)
    }

    fn workflow_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Workflow", |ui| {
            egui::ComboBox::new("dev_workflow_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(task_browser.selection.task_config().workflow.to_string())
                .show_ui(ui, |ui| {
                    for workflow in Workflow::iter() {
                        let selected = task_browser.selection.task_config().workflow == workflow;
                        if ui
                            .selectable_label(selected, workflow.to_string())
                            .clicked()
                        {
                            task_browser.selection.set_workflow(workflow);
                        }
                    }
                });
        });
    }

    fn domain_control(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
    ) {
        Self::control_field(ui, "Domain", |ui| {
            egui::ComboBox::new("dev_domain_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(task_browser.selection.task_config().domain.to_string())
                .show_ui(ui, |ui| {
                    for domain in Domain::iter() {
                        let selected = task_browser.selection.task_config().domain == domain;
                        if ui.selectable_label(selected, domain.to_string()).clicked() {
                            let gravity = task_browser.selection.set_domain(domain);
                            if let Some(gravity) =
                                domain_choice_gravity(self.customization_runtime_running, gravity)
                            {
                                session.runtime_params_mut().gravity = gravity;
                            }
                        }
                    }
                });
        });
    }

    fn task_control(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        gui_catalog: Option<&GuiCatalog>,
    ) -> Option<GuiInterfaceControlEvent> {
        let mut event = None;
        Self::control_field(ui, "Task", |ui| {
            egui::ComboBox::new("dev_task_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(task_browser.selection.task_config().task.clone())
                .show_ui(ui, |ui| {
                    for task in task_browser.selection.options.tasks.clone() {
                        let selected = task_browser.selection.task_config().task == task;
                        if ui.selectable_label(selected, &task).clicked() {
                            let catalog_task = gui_catalog
                                .and_then(|catalog| task_browser.task_by_id(catalog, &task))
                                .cloned();
                            event = Some(GuiInterfaceControlEvent::SelectTaskById {
                                task_id: task.clone(),
                                catalog_task,
                            });
                        }
                    }
                });
        });
        event
    }

    fn robot_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Robot", |ui| {
            if !robot_control_enabled(task_browser.selection.task_config()) {
                ui.add_enabled(
                    false,
                    egui::Button::new(task_browser.selection.task_config().robot.clone()),
                )
                .on_disabled_hover_text(
                    "Composite robot: this task launches env.robot.mobile_base and \
                     env.robot.manipulator, so the Robot control does not apply.",
                );
                return;
            }
            egui::ComboBox::new("dev_robot_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(task_browser.selection.task_config().robot.clone())
                .show_ui(ui, |ui| {
                    for robot in task_browser.selection.options.robots.clone() {
                        let selected = task_browser.selection.task_config().robot == robot;
                        if ui.selectable_label(selected, &robot).clicked() {
                            apply_robot_choice(&mut task_browser.selection, robot);
                        }
                    }
                });
        });
    }

    fn tool_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Tool or payload", |ui| {
            let selected_text = task_browser
                .selection
                .task_config()
                .end_effector
                .as_deref()
                .map(|tool| format!("Tool: {tool}"))
                .or_else(|| {
                    task_browser
                        .selection
                        .task_config()
                        .payload
                        .as_deref()
                        .map(|payload| format!("Payload: {payload}"))
                })
                .unwrap_or_else(|| "None".to_string());

            egui::ComboBox::new("dev_tool_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(selected_text)
                .show_ui(ui, |ui| {
                    let nothing_selected =
                        task_browser.selection.task_config().end_effector.is_none()
                            && task_browser.selection.task_config().payload.is_none();
                    if ui.selectable_label(nothing_selected, "None").clicked() {
                        task_browser.selection.clear_tool_payload();
                    }

                    for tool in task_browser.selection.options.tools.clone() {
                        let selected = task_browser.selection.task_config().end_effector.as_deref()
                            == Some(tool.as_str());
                        if ui
                            .selectable_label(selected, format!("Tool: {tool}"))
                            .clicked()
                        {
                            task_browser.selection.select_tool(tool);
                        }
                    }

                    for payload in task_browser.selection.options.payloads.clone() {
                        let selected = task_browser.selection.task_config().payload.as_deref()
                            == Some(payload.as_str());
                        if ui
                            .selectable_label(selected, format!("Payload: {payload}"))
                            .clicked()
                        {
                            task_browser.selection.select_payload(payload);
                        }
                    }
                });
        });
    }

    fn scenery_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Scenery", |ui| {
            egui::ComboBox::new("dev_scenery_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(
                    task_browser
                        .selection
                        .task_config()
                        .scenery
                        .as_deref()
                        .unwrap_or("None"),
                )
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(
                            task_browser.selection.task_config().scenery.is_none(),
                            "None",
                        )
                        .clicked()
                    {
                        task_browser.selection.set_scenery(None);
                    }
                    for scenery in task_browser.selection.options.sceneries.clone() {
                        let selected = task_browser.selection.task_config().scenery.as_deref()
                            == Some(scenery.as_str());
                        if ui.selectable_label(selected, &scenery).clicked() {
                            task_browser.selection.set_scenery(Some(scenery));
                        }
                    }
                });
        });
    }

    fn rendering_mode_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Rendering quality", |ui| {
            egui::ComboBox::new("dev_rendering_mode_combo_box", "")
                .width(Self::control_width(ui))
                .wrap()
                .selected_text(
                    task_browser
                        .selection
                        .task_config()
                        .rendering_mode
                        .to_string(),
                )
                .show_ui(ui, |ui| {
                    for rendering_mode in task_browser.selection.options.rendering_modes.clone() {
                        let selected =
                            task_browser.selection.task_config().rendering_mode == rendering_mode;
                        if ui
                            .selectable_label(selected, rendering_mode.to_string())
                            .clicked()
                        {
                            task_browser.selection.set_rendering_mode(rendering_mode);
                        }
                    }
                });
        });
    }

    fn seed_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Random seed", |ui| {
            let mut seed = task_browser.selection.task_config().seed;
            let response = ui
                .add(egui::DragValue::new(&mut seed).range(0..=u64::MAX))
                .on_hover_text("Drag, or click to type a seed");
            if response.changed() {
                task_browser.selection.set_seed(seed);
            }
        });
    }

    fn num_envs_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Parallel environments", |ui| {
            let mut num_envs = task_browser.selection.task_config().num_envs;
            let num_envs_text = num_envs.to_string();
            let response = Self::slider_with_value(
                ui,
                egui::Slider::new(&mut num_envs, 1..=16)
                    .clamping(egui::SliderClamping::Never)
                    .trailing_fill(true)
                    .integer()
                    .show_value(false)
                    .custom_formatter(|x, _| format!("{x}")),
                num_envs_text,
            );
            if response.changed() {
                task_browser.selection.set_num_envs(num_envs);
            }
        });
    }

    fn hide_ui_control(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        Self::control_field(ui, "Simulation UI", |ui| {
            let mut show_ui = !task_browser.selection.task_config().hide_ui;
            if ui.checkbox(&mut show_ui, "Show simulation UI").changed() {
                task_browser.selection.set_hide_ui(!show_ui);
            }
        });
    }

    fn task_specific_override_controls(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
        gui_catalog: Option<&GuiCatalog>,
    ) {
        let selected_task = gui_catalog
            .and_then(|catalog| task_browser.selected_task(catalog))
            .cloned();

        if let Some(task) = selected_task {
            for override_choice in GuiTaskOverrideChoice::from_task(&task, &task_browser.selection)
            {
                Self::control_field_with_hint(
                    ui,
                    override_choice.label(),
                    Some(override_choice.key()),
                    |ui| {
                        egui::ComboBox::new(override_choice.widget_id(), "")
                            .width(Self::control_width(ui))
                            .wrap()
                            .selected_text(override_choice.selected_text())
                            .show_ui(ui, |ui| {
                                if ui
                                    .selectable_label(
                                        override_choice.default_selected(),
                                        override_choice.default_label(),
                                    )
                                    .clicked()
                                {
                                    override_choice.apply_default(&mut task_browser.selection);
                                }
                                for value in override_choice.values() {
                                    if ui
                                        .selectable_label(
                                            override_choice.value_selected(value),
                                            value,
                                        )
                                        .clicked()
                                    {
                                        override_choice
                                            .apply_value(&mut task_browser.selection, value);
                                    }
                                }
                            });
                    },
                );
            }
        }

        Self::control_field(ui, "Raw Hydra overrides", |ui| {
            self.raw_override_editor(ui, task_browser);
        });
    }

    fn raw_override_editor(&mut self, ui: &mut egui::Ui, task_browser: &mut GuiTaskBrowser) {
        let mut raw_overrides = task_browser.selection.task_config().extras.join("\n");
        if ui
            .add(
                egui::TextEdit::multiline(&mut raw_overrides)
                    .desired_rows(4)
                    .code_editor(),
            )
            .changed()
        {
            task_browser.selection.replace_raw_overrides(&raw_overrides);
        }
    }

    fn advanced_interface_controls(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &mut GuiTaskBrowser,
    ) {
        // Every cached device, haptic included, gets a generic checkbox.
        let visible_teleop_devices = task_browser.selection.options.teleop_devices.clone();

        Self::control_field(ui, "Input devices", |ui| {
            ui.horizontal_wrapped(|ui| {
                for device in &visible_teleop_devices {
                    let mut enabled = task_browser.selection.teleop_device_enabled(*device);
                    if ui.checkbox(&mut enabled, device.to_string()).changed() {
                        task_browser
                            .selection
                            .set_teleop_device_enabled(*device, enabled);
                    }
                }

                let mut all_devices = !visible_teleop_devices.is_empty()
                    && visible_teleop_devices
                        .iter()
                        .all(|device| task_browser.selection.teleop_device_enabled(*device));
                if ui.checkbox(&mut all_devices, "All input devices").changed() {
                    for device in &visible_teleop_devices {
                        task_browser
                            .selection
                            .set_teleop_device_enabled(*device, all_devices);
                    }
                }
            });
        });

        Self::control_field(ui, "Interfaces", |ui| {
            ui.horizontal_wrapped(|ui| {
                let mut all_interfaces = task_browser
                    .selection
                    .options
                    .interfaces
                    .iter()
                    .all(|interface| task_browser.selection.interface_enabled(*interface));
                if ui.checkbox(&mut all_interfaces, "All interfaces").changed() {
                    for interface in task_browser.selection.options.interfaces.clone() {
                        task_browser
                            .selection
                            .set_interface_enabled(interface, all_interfaces);
                    }
                }

                for interface in task_browser.selection.options.interfaces.clone() {
                    let mut enabled = task_browser.selection.interface_enabled(interface);
                    if ui.checkbox(&mut enabled, interface.to_string()).changed() {
                        task_browser
                            .selection
                            .set_interface_enabled(interface, enabled);
                    }
                }
            });
        });

        Self::control_field(ui, "On-screen tools", |ui| {
            ui.checkbox(&mut self.show_virtual_keyboard, "Virtual gripper");
        });
    }

    fn command_preview_controls(
        &mut self,
        ui: &mut egui::Ui,
        task_browser: &GuiTaskBrowser,
        run_label: &str,
    ) -> Option<GuiInterfaceControlEvent> {
        let launch_request = task_browser.selection.launch_request();
        // Preview and clipboard text are display surfaces: credential tokens
        // are redacted here while the executed argv stays exact.
        let command = redact_shell_command(launch_request.preview_command());
        let mut command_text = command.clone();
        let mut event = None;
        ui.add(
            egui::TextEdit::multiline(&mut command_text)
                .desired_width(f32::INFINITY)
                .desired_rows(2)
                .code_editor()
                .interactive(false),
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .add_sized(egui::vec2(96.0, 30.0), Self::primary_button(ui, run_label))
                .clicked()
            {
                event = Some(GuiInterfaceControlEvent::StartLaunchRequest(launch_request));
            }
            Self::command_copy_button(ui, Self::preview_copy_control(&command));
        });
        event
    }
}

fn merge_control_event(
    event: &mut Option<GuiInterfaceControlEvent>,
    section_event: Option<GuiInterfaceControlEvent>,
) {
    if section_event.is_some() {
        *event = section_event;
    }
}

/// Whether the Robot control applies; composite tasks use base and manipulator keys instead of
/// `env.robot`.
fn robot_control_enabled(task_config: &TaskConfig) -> bool {
    !task_config.uses_composite_robot()
}

/// Apply a Robot choice and clear tool/payload overrides tied to the old robot.
fn apply_robot_choice(selection: &mut GuiSelection, robot: String) {
    if selection.task_config().robot == robot {
        return;
    }
    selection.set_robot(robot);
    selection.clear_tool_payload();
}

/// Gravity to publish after a Domain choice, unless a subprocess is running.
fn domain_choice_gravity(runtime_running: bool, gravity: f64) -> Option<f64> {
    (!runtime_running).then_some(gravity)
}
