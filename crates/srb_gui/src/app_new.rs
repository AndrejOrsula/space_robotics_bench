use eframe::epaint::Color32;
use egui_commonmark::{commonmark_str, CommonMarkCache};
use r2r::{
    std_msgs::msg::{Bool as BoolMsg, Empty as EmptyMsg, Float64 as Float64Msg},
    QosProfile,
};
use std::{
    io::{Read, Write},
    path::PathBuf,
}; // Added PathBuf
   // Updated sysinfo imports
use tracing::{error, info, trace, warn}; // Added ProcessesToUpdate
                                         // Import cache loading functions and structs
use crate::cache::{
    self,
    find_srb_root,
    EnvCache,
    ObjectCache,
    RobotCache,
    SceneryCache, // Import find_srb_root
};
use crate::env_cfg::{Domain, TaskConfig}; // Keep Domain for gravity, TaskConfig is updated
use crate::page::Page;
use strum::IntoEnumIterator;

const LOGFILE_PATH: &str = "srb_gui.log";

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct App {
    theme: egui::Theme,
    #[serde(skip)]
    current_page: Page,
    #[serde(skip)]
    task_config: TaskConfig, // Updated struct
    #[serde(skip)]
    subprocess: Option<subprocess::Popen>,

    // --- UI State ---
    #[serde(skip)]
    show_about: bool,
    #[serde(skip)]
    show_virtual_keyboard_window: bool,
    #[serde(skip)]
    show_developer_options: bool,

    // --- Data Collection ---
    #[serde(skip)]
    collect_trajectory: bool,
    #[serde(skip)]
    n_collected_trajectories: usize,

    // --- Real-time Params (Keep as is for now) ---
    #[serde(skip)]
    gravity: f64,
    #[serde(skip)]
    latency: f64,
    #[serde(skip)]
    motion_sensitivity: f64,
    #[serde(skip)]
    force_feedback_sensitivity: f64,
    #[serde(skip)]
    max_feedback_force: f64,

    #[serde(skip)]
    prev_gravity: f64,
    #[serde(skip)]
    prev_latency: f64,
    #[serde(skip)]
    prev_motion_sensitivity: f64,
    #[serde(skip)]
    prev_force_feedback_sensitivity: f64,
    #[serde(skip)]
    prev_max_feedback_force: f64,

    // --- ROS ---
    #[serde(skip)]
    node: r2r::Node,
    #[serde(skip)]
    pub_gripper_toggle: r2r::Publisher<BoolMsg>,
    #[serde(skip)]
    pub_reset_discard_dataset: r2r::Publisher<EmptyMsg>,
    #[serde(skip)]
    pub_gracefully_shutdown_process: r2r::Publisher<EmptyMsg>,
    #[serde(skip)]
    pub_gravity: r2r::Publisher<Float64Msg>,
    #[serde(skip)]
    pub_latency: r2r::Publisher<Float64Msg>,
    #[serde(skip)]
    pub_motion_sensitivity: r2r::Publisher<Float64Msg>,
    #[serde(skip)]
    pub_force_feedback_sensitivity: r2r::Publisher<Float64Msg>,
    #[serde(skip)]
    pub_max_feedback_force: r2r::Publisher<Float64Msg>,

    #[serde(skip)]
    last_message_pub: std::time::Instant,

    // --- Logging ---
    #[serde(skip)]
    logfile: std::fs::File,

    // --- Cache Data ---
    #[serde(skip)]
    env_cache: EnvCache,
    #[serde(skip)]
    robot_cache: RobotCache,
    #[serde(skip)]
    object_cache: ObjectCache,
    #[serde(skip)]
    scenery_cache: SceneryCache,

    // --- UI HelperTopBs ---
    #[serde(skip)]
    all_robot_names: Vec<String>,
    #[serde(skip)]
    all_object_names: Vec<String>,
    #[serde(skip)]
    all_scenery_names: Vec<String>, // Added
    #[serde(skip)]
    all_tool_names: Vec<String>, // For end effectors
    #[serde(skip)]
    all_payload_names: Vec<String>, // For payloads
    #[serde(skip)]
    all_domain_names: Vec<String>,

    #[serde(skip)]
    commonmark_cache: CommonMarkCache,
    // Remove hovered_task if quickstart page changes significantly
    // #[serde(skip)]
    // hovered_task: Option<usize>,
}

impl Default for App {
    fn default() -> Self {
        // --- Load Cache Data ---
        let env_cache = cache::load_env_cache();
        let robot_cache = cache::load_robot_cache();
        let object_cache = cache::load_object_cache();
        let scenery_cache = cache::load_scenery_cache();

        // --- Prepare UI Helper Lists ---
        let all_robot_names = cache::get_all_robot_names(&robot_cache);
        let all_object_names = cache::get_all_asset_names(&object_cache);
        let mut all_scenery_names = cache::get_all_asset_names(&scenery_cache); // Make mutable
                                                                                // Add "none" option for scenery
        all_scenery_names.insert(0, "none".to_string()); // Add "none" at the beginning
        all_scenery_names.sort(); // Re-sort after adding "none"
        all_scenery_names.dedup();

        // Use helpers which already include "none" and ""
        let all_tool_names = cache::get_object_names_by_subtype(&object_cache, "tool");
        let all_payload_names = cache::get_object_names_by_subtype(&object_cache, "payload");

        let all_domain_names = Domain::iter().map(|d| d.to_string()).collect();

        // --- Initialize TaskConfig with defaults based on cache if possible ---
        let mut task_config = TaskConfig::builder().build();
        if let Some(first_env) = env_cache.first() {
            task_config.task = first_env.clone();
        }
        if let Some(first_robot) = all_robot_names.first() {
            // Ensure default robot exists in cache, otherwise keep hardcoded default
            if all_robot_names.contains(&task_config.robot) {
                // Keep default if it's valid
            } else {
                task_config.robot = first_robot.clone();
            }
        }
        // Set default gravity based on default domain
        let initial_gravity = task_config.domain.gravity_magnitude();

        // --- ROS Initialization (Keep as is) ---
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "srb_gui_gui", "").unwrap();
        let pub_gripper_toggle = node
            .create_publisher::<BoolMsg>("/touch/event", QosProfile::default())
            .unwrap();
        let pub_reset_discard_dataset = node
            .create_publisher::<EmptyMsg>("/gui/reset_discard_dataset", QosProfile::default())
            .unwrap();
        let pub_gracefully_shutdown_process = node
            .create_publisher::<EmptyMsg>("/gui/shutdown_process", QosProfile::default())
            .unwrap();
        let pub_gravity = node
            .create_publisher::<Float64Msg>("/gui/gravity", QosProfile::default())
            .unwrap();
        let pub_latency = node
            .create_publisher::<Float64Msg>("/gui/latency", QosProfile::default())
            .unwrap();
        let pub_motion_sensitivity = node
            .create_publisher::<Float64Msg>("/gui/motion_sensitivity", QosProfile::default())
            .unwrap();
        let pub_force_feedback_sensitivity = node
            .create_publisher::<Float64Msg>(
                "/gui/force_feedback_sensitivity",
                QosProfile::default(),
            )
            .unwrap();
        let pub_max_feedback_force = node
            .create_publisher::<Float64Msg>("/gui/max_feedback_force", QosProfile::default())
            .unwrap();
        let last_message_pub = std::time::Instant::now();

        // --- Log File Initialization (Keep as is) ---
        let mut logfile = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .append(true)
            .open(LOGFILE_PATH)
            .unwrap();
        let n_collected_trajectories = {
            let mut file_str = String::new();
            logfile.read_to_string(&mut file_str).unwrap();
            file_str
                .clone()
                .lines()
                .filter(|x| x.starts_with("DATA"))
                .count()
        };
        writeln!(
            logfile,
            "START, {}, {}",
            chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
        )
        .unwrap();

        // --- Construct App State ---
        Self {
            theme: egui::Theme::Dark,
            current_page: Page::default(),
            task_config, // Use updated default
            subprocess: None,

            show_about: false,
            show_virtual_keyboard_window: false,
            show_developer_options: false, // Maybe default to true now?

            collect_trajectory: false,
            n_collected_trajectories,

            gravity: initial_gravity, // Use calculated initial gravity
            latency: 0.0,
            motion_sensitivity: 1.0,
            force_feedback_sensitivity: 1.0,
            max_feedback_force: 2.0,

            // Initialize prev values to force initial publish
            prev_gravity: initial_gravity + 1.0, // Ensure initial publish
            prev_latency: -1.0,                  // Ensure initial publish
            prev_motion_sensitivity: -1.0,       // Ensure initial publish
            prev_force_feedback_sensitivity: -1.0, // Ensure initial publish
            prev_max_feedback_force: -1.0,       // Ensure initial publish

            node,
            pub_gripper_toggle,
            pub_reset_discard_dataset,
            pub_gracefully_shutdown_process,
            pub_gravity,
            pub_latency,
            pub_motion_sensitivity,
            pub_force_feedback_sensitivity,
            pub_max_feedback_force,

            last_message_pub,
            logfile,

            // Store loaded cache data
            env_cache,
            robot_cache,
            object_cache,
            scenery_cache,

            // Store UI helper lists
            all_robot_names,
            all_object_names,
            all_scenery_names,
            all_tool_names,
            all_payload_names,
            all_domain_names,

            commonmark_cache: CommonMarkCache::default(),
            // hovered_task: None, // Remove if quickstart changes
        }
    }
}

impl App {
    #[must_use]
    pub fn new(cc: &eframe::CreationContext) -> Self {
        // Enable image loading
        egui_extras::install_image_loaders(&cc.egui_ctx);

        // Load the fonts
        crate::style::load_fonts(&cc.egui_ctx);

        // Enable screen web reader support
        #[cfg(target_arch = "wasm32")]
        cc.egui_ctx.options_mut(|o| o.screen_reader = true);

        // Construct the app state
        let mut app = if let Some(storage) = cc.storage {
            // Try to restore previous state (TaskConfig structure changed, might need migration or reset)
            // For now, let's just default if loading fails or structure mismatches.
            eframe::get_value(storage, eframe::APP_KEY).unwrap_or_else(|| {
                warn!("Failed to load previous state or state structure changed, using default.");
                Self::default()
            })
        } else {
            // Otherwise, use default state
            Self::default()
        };

        // Ensure loaded state has up-to-date cache info (in case cache changed since last save)
        // This might overwrite user selections if cache is missing items, handle carefully.
        // A simpler approach is to just reload cache every time like in Default::default().
        // Let's assume Default::default() handles the loading correctly for now.

        // Set the theme
        crate::style::set_theme(&cc.egui_ctx, app.theme);

        // Publish initial messages if values differ from prev_ values
        app.publish_messages();

        app
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        // --- URL Sync (Keep as is) ---
        #[cfg(target_arch = "wasm32")]
        if let Some(page) = frame.info().web_info.location.hash.strip_prefix('#') {
            // ... (keep existing logic)
            if let Some(page) = crate::ENABLED_PAGES
                .into_iter()
                .find(|x| x.to_string().eq_ignore_ascii_case(page))
            {
                // If a known page was requested, update the current page
                self.current_page = page;
            } else {
                // If an unknown page was requested, update the URL to open the default page
                crate::utils::egui::open_url_on_page(ctx, Page::default(), true);
            }
        } else {
            // Otherwise, update the URL to match the current page
            crate::utils::egui::open_url_on_page(ctx, self.current_page, true);
        }

        // --- Fullscreen Toggle (Keep as is) ---
        #[cfg(not(target_arch = "wasm32"))]
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F11)) {
            // ... (keep existing logic)
            let fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!fullscreen));
        }

        // --- Top Panel (Keep structure, content might change) ---
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                ui.spacing_mut().item_spacing.x = 16.0;
                self.navigation_buttons(ui); // Keep navigation

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Maybe rename "Advanced Options" to "Customize" or similar
                    self.advanced_opts_button(ui);
                    self.show_top_center_bar(ui); // Keep title bar
                });
            });
        });

        // --- Bottom Panel (Keep structure, content might change) ---
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    self.about_button(ui); // Keep about

                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        self.dark_mode_toggle_button(ui); // Keep theme toggle
                        ui.separator();
                        self.show_trajectory_collection_checkbox(ui); // Keep data collection
                        self.warn_if_debug_build(ui); // Keep debug warning
                    });
                });
            });
        });

        // --- Central Panel (Content depends on current_page) ---
        crate::utils::egui::ScrollableFramedCentralPanel::builder()
            .max_content_width(ctx.screen_rect().width())
            .build()
            .show(ctx, |ui| {
                match self.current_page {
                    Page::QuickStart => {
                        // Renamed QuickStart to Home
                        self.configuration_page(ui); // Show config page directly for now
                    }
                    Page::Interface => {
                        self.configuration_page(ui); // Main configuration UI
                    }
                }
                // Keep window popups
                self.about_window(ui);
                self.virtual_keyboard_window(ui);
                self.developer_options_window(ui); // This might merge with config page
            });

        // --- Publish ROS messages if values changed ---
        // Only publish if needed, e.g., on the config page or if dev options are open
        if self.current_page == Page::Interface || self.show_developer_options {
            self.publish_messages();
        }

        // --- Update Gravity based on Domain Selection ---
        // This should happen whenever task_config.domain changes in the UI
        let new_gravity = self.task_config.domain.gravity_magnitude();
        if (self.gravity - new_gravity).abs() > f64::EPSILON {
            self.gravity = new_gravity;
            // The publish_messages function will handle sending the update
        }
    }

    // ... (keep save function)
    #[cfg(target_arch = "wasm32")]
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(&mut *self)
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }
}

impl App {
    // --- UI Rendering Functions ---

    // Keep navigation_buttons, about_button, advanced_opts_button (maybe rename)

    fn navigation_buttons(&mut self, ui: &mut egui::Ui) {
        // ... (Keep existing logic) ...
        for page in Page::iter() {
            if self.current_page == page {
                ui.add(egui::Button::new(page.title()))
                    .highlight()
                    .on_hover_text(format!("{} (current page)", page.description()));
            } else {
                let button = ui
                    .add(egui::Button::new(page.title()))
                    .on_hover_text(page.description());
                // If the button is clicked, change the current page
                if button.clicked() {
                    // Change URL to the new page in the same tab
                    #[cfg(target_arch = "wasm32")]
                    crate::utils::egui::open_url_on_page(ui.ctx(), page, true);

                    // Manually update the current page for non-web platforms
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.current_page = page;
                    }

                    if page == Page::QuickStart {
                        self.show_about = false;
                    }
                } else {
                    // Open URL in a new page if the middle mouse button is clicked
                    #[cfg(target_arch = "wasm32")]
                    if button.middle_clicked() {
                        crate::utils::egui::open_url_on_page(ui.ctx(), page, false);
                    }
                }
            }
        }
    }

    fn about_button(&mut self, ui: &mut egui::Ui) {
        // ... (Keep existing logic) ...
        let button = ui
            .add(egui::Button::new({
                let text = egui::RichText::new("About");
                if self.show_about {
                    text.strong()
                } else {
                    text
                }
            }))
            .on_hover_text(if self.show_about {
                "Close the associated window"
            } else {
                "Learn more about this demo"
            });
        // If the button is clicked, change the current page
        if button.clicked() {
            self.show_about = !self.show_about;
        }
    }

    fn advanced_opts_button(&mut self, ui: &mut egui::Ui) {
        // Renamed to "Customize"
        let button = ui
            .add(egui::Button::new({
                let text = egui::RichText::new("Customize"); // Renamed
                if self.show_developer_options {
                    // Use the same flag for now
                    text.strong()
                } else {
                    text
                }
            }))
            .on_hover_text(if self.show_developer_options {
                "Hide customization options"
            } else {
                "Show customization options"
            });
        // If the button is clicked, toggle the customization window/panel
        if button.clicked() {
            self.show_developer_options = !self.show_developer_options;
        }
    }

    // Remove or completely redesign quickstart_page
    // fn quickstart_page(&mut self, ui: &mut egui::Ui) { ... }

    // Main configuration UI
    fn configuration_page(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            // Center the content area
            let max_width = 768.0;
            let margin_x = (ui.ctx().screen_rect().width() - max_width).max(0.0) / 2.0;
            let inner_margin = egui::Margin::symmetric(margin_x as i8, 10); // Cast f32 to i8 and use integer 10

            egui::Frame::default()
                .inner_margin(inner_margin)
                .show(ui, |ui| {
                    ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                        ui.heading("Simulation Configuration");
                        ui.add_space(10.0);

                        egui::Grid::new("main_config_grid")
                            .num_columns(2)
                            .spacing([40.0, 4.0])
                            .striped(true)
                            .show(ui, |ui| {
                                // --- Task Selection ---
                                ui.label("Task:");
                                egui::ComboBox::new("task_combo", "") // Renamed from_id_source to id_salt
                                    .selected_text(self.task_config.task.as_str())
                                    .show_ui(ui, |ui| {
                                        for task_name in &self.env_cache {
                                            ui.selectable_value(
                                                &mut self.task_config.task,
                                                task_name.clone(),
                                                task_name.as_str(),
                                            );
                                        }
                                    });
                                ui.end_row();

                                // --- Domain Selection ---
                                ui.label("Domain:");
                                egui::ComboBox::new("domain_combo","") // Renamed from_id_source to id_salt
                                    .selected_text(self.task_config.domain.to_string())
                                    .show_ui(ui, |ui| {
                                        for domain in Domain::iter() {
                                            ui.selectable_value(
                                                &mut self.task_config.domain,
                                                domain,
                                                domain.to_string(),
                                            );
                                        }
                                    });
                                // Update gravity display based on selected domain
                                let current_gravity = self.task_config.domain.gravity_magnitude();
                                ui.label(format!("(Gravity: {current_gravity:.2} m/s²)"))
                                    .on_hover_text(
                                        "Gravitational acceleration for the selected domain",
                                    );
                                ui.end_row();

                                // --- Scenery Selection --- Added
                                ui.label("Scenery:");
                                let mut selected_scenery = self
                                    .task_config
                                    .scenery
                                    .clone()
                                    .unwrap_or("none".to_string());
                                egui::ComboBox::new("scenery_combo","") // Renamed from_id_source to id_salt
                                    .selected_text(&selected_scenery)
                                    .show_ui(ui, |ui| {
                                        for scenery_name in &self.all_scenery_names {
                                            ui.selectable_value(
                                                &mut selected_scenery,
                                                scenery_name.clone(),
                                                scenery_name.as_str(),
                                            );
                                        }
                                    });
                                // Update TaskConfig scenery
                                let new_scenery_opt = if selected_scenery == "none" {
                                    None
                                } else {
                                    Some(selected_scenery)
                                };
                                if self.task_config.scenery != new_scenery_opt {
                                    self.task_config.scenery = new_scenery_opt;
                                }
                                ui.end_row();

                                // --- Robot Selection ---
                                ui.label("Robot Base:");
                                egui::ComboBox::new("robot_base_combo","") // Renamed from_id_source to id_salt
                                    .selected_text(self.task_config.robot.as_str())
                                    .show_ui(ui, |ui| {
                                        // Add an option for no base (to only specify attachment)
                                        ui.selectable_value(
                                            &mut self.task_config.robot,
                                            String::new(), // Represent "no base" with empty string
                                            "<Default/None>",
                                        );
                                        for robot_name in &self.all_robot_names {
                                            ui.selectable_value(
                                                &mut self.task_config.robot,
                                                robot_name.clone(),
                                                robot_name.as_str(),
                                            );
                                        }
                                    });
                                ui.end_row();

                                // --- End Effector Selection ---
                                let mut selected_ee = self
                                    .task_config
                                    .end_effector
                                    .clone()
                                    .unwrap_or("none".to_string());
                                egui::ComboBox::new("robot_ee_combo","") // Renamed from_id_source to id_salt
                                    .selected_text(match selected_ee.as_str() {
                                        "" => "Keep Default",
                                        "none" => "None",
                                        _ => &selected_ee,
                                    })
                                    .show_ui(ui, |ui| {
                                        for tool_name in &self.all_tool_names {
                                            let display_name = match tool_name.as_str() {
                                                "" => "Keep Default",
                                                "none" => "None",
                                                _ => tool_name.as_str(),
                                            };
                                            ui.selectable_value(
                                                &mut selected_ee,
                                                tool_name.clone(),
                                                display_name,
                                            );
                                        }
                                    });
                                // Only update if the selection is different from current state to avoid unnecessary changes
                                let new_ee_opt = if selected_ee == "none" {
                                    None
                                } else {
                                    Some(selected_ee)
                                };
                                if self.task_config.end_effector != new_ee_opt {
                                    self.task_config.end_effector = new_ee_opt;
                                    // If EE is selected, ensure payload is None
                                    if self.task_config.end_effector.is_some() {
                                        self.task_config.payload = None;
                                    }
                                }
                                ui.end_row();

                                // --- Payload Selection ---
                                ui.label("Payload:");
                                let mut selected_payload = self
                                    .task_config
                                    .payload
                                    .clone()
                                    .unwrap_or("none".to_string());
                                // Disable payload selection if an end effector is actively selected (not "None" or "Keep Default")
                                let ee_is_selected = self
                                    .task_config
                                    .end_effector
                                    .as_deref()
                                    .is_some_and(|s| !s.is_empty() && s != "none");
                                ui.add_enabled_ui(
                                    !ee_is_selected,
                                    // Pass the ComboBox widget directly
                                    |ui| {
                                        egui::ComboBox::new("robot_payload_combo","") // Renamed from_id_source to id_salt
                                            .selected_text(match selected_payload.as_str() {
                                                "" => "Keep Default",
                                                "none" => "None",
                                                _ => &selected_payload,
                                            })
                                            .show_ui(ui, |ui| {
                                                for payload_name in &self.all_payload_names {
                                                    let display_name = match payload_name.as_str() {
                                                        "" => "Keep Default",
                                                        "none" => "None",
                                                        _ => payload_name.as_str(),
                                                    };
                                                    ui.selectable_value(
                                                        &mut selected_payload,
                                                        payload_name.clone(),
                                                        display_name,
                                                    );
                                                }
                                            })
                                    },
                                );
                                // Only update if the selection is different and payload is enabled
                                let new_payload_opt = if selected_payload == "none" {
                                    None
                                } else {
                                    Some(selected_payload)
                                };
                                if !ee_is_selected && self.task_config.payload != new_payload_opt {
                                    self.task_config.payload = new_payload_opt;
                                } else if ee_is_selected {
                                    // Ensure payload is None if EE is selected
                                    self.task_config.payload = None;
                                }
                                ui.end_row();

                                // --- Simulation Parameters ---
                                ui.label("Number of Envs:");
                                ui.add(
                                    egui::Slider::new(&mut self.task_config.num_envs, 1..=64)
                                        .logarithmic(true),
                                );
                                ui.end_row();

                                ui.label("Random Seed:");
                                ui.add(egui::DragValue::new(&mut self.task_config.seed));
                                ui.end_row();

                                ui.label("Stack Envs:");
                                ui.add(egui::Checkbox::new(&mut self.task_config.stack_envs, ""));
                                ui.end_row();

                                ui.label("Enable Graphics UI:");
                                ui.add(egui::Checkbox::new(&mut self.task_config.hide_ui, ""));
                                ui.end_row();

                                // Add Scenery/Object selection if desired
                                // ui.label("Scenery:");
                                // ... ComboBox using self.all_scenery_names ...
                                // ui.end_row();
                                // ui.label("Objects:");
                                // ... Multi-select or list for self.task_config.objects ...
                                // ui.end_row();
                            });

                        ui.add_space(20.0);

                        // --- Real-time Adjustments (Optional - could be in Customize window) ---
                        ui.separator();
                        ui.heading("Real-time Adjustments");
                        ui.add_space(10.0);

                        egui::Grid::new("real_time_env_config")
                            .num_columns(2)
                            .spacing([40.0, 4.0])
                            .striped(true)
                            .show(ui, |ui| {
                                ui.style_mut().spacing.slider_width = ui.available_width() * 0.6; // Adjust slider width

                                // Gravity (Display only, controlled by Domain)
                                ui.label("Gravity (m/s²):")
                                    .on_hover_text("Set via Domain selection above");
                                // Use a DragValue for display consistency, but disable it
                                ui.add_enabled(
                                    false,
                                    egui::DragValue::new(&mut self.gravity)
                                        .speed(0.01)
                                        .fixed_decimals(3),
                                );
                                ui.end_row();

                                // Latency
                                ui.label("Latency (ms):")
                                    .on_hover_text("Simulated network latency");
                                ui.add(egui::Slider::new(&mut self.latency, 0.0..=500.0));
                                ui.end_row();

                                // Motion Sensitivity
                                ui.label("Motion Sensitivity:")
                                    .on_hover_text("Teleoperation movement scaling");
                                ui.add(
                                    egui::Slider::new(&mut self.motion_sensitivity, 0.1..=5.0)
                                        .logarithmic(true),
                                );
                                ui.end_row();

                                // Force Feedback Sensitivity
                                ui.label("FFB Sensitivity:")
                                    .on_hover_text("Force feedback scaling for haptics");
                                ui.add(
                                    egui::Slider::new(
                                        &mut self.force_feedback_sensitivity,
                                        0.1..=5.0,
                                    )
                                    .logarithmic(true),
                                );
                                ui.end_row();

                                // Max Feedback Force
                                ui.label("Max FFB (N):")
                                    .on_hover_text("Maximum force feedback applied");
                                ui.add(egui::Slider::new(&mut self.max_feedback_force, 0.0..=10.0));
                                ui.end_row();
                            });

                        ui.add_space(20.0);

                        // --- Action Buttons ---
                        ui.separator();
                        ui.add_space(10.0);
                        ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                            let button_size = egui::vec2(max_width * 0.4, 40.0); // Make buttons larger

                            if self.subprocess.is_none() {
                                // Start Button
                                let start_button = ui.add_sized(
                                    button_size,
                                    egui::Button::new(
                                        egui::RichText::new("🚀 Start Simulation").size(18.0),
                                    ),
                                );
                                if start_button.clicked() {
                                    self.start_subprocess();
                                }
                            } else {
                                // Stop and Restart Buttons
                                ui.horizontal(|ui| {
                                    let stop_button = ui.add_sized(
                                        button_size,
                                        egui::Button::new(
                                            egui::RichText::new("⏹ Stop Simulation")
                                                .size(18.0)
                                                .color(Color32::LIGHT_RED),
                                        ),
                                    );
                                    if stop_button.clicked() {
                                        self.stop_subprocess();
                                    }

                                    let restart_button = ui.add_sized(
                                        button_size,
                                        egui::Button::new(
                                            egui::RichText::new("🔄 Restart Episode").size(18.0),
                                        ),
                                    );
                                    if restart_button.clicked() {
                                        self.restart_episode();
                                    }
                                });
                            }
                        });
                    });
                });
        });
    }

    // Keep restart_episode, about_window, virtual_keyboard_window

    fn restart_episode(&mut self) {
        if self.subprocess.is_some() {
            info!("Restarting episode");
            // Log data collection event
            if self.collect_trajectory {
                info!("Logging collected trajectory data point (on restart)");
                self.n_collected_trajectories += 1; // Increment here or based on actual save confirmation if added

                // Log the configuration used for this trajectory
                // Use serde_json for easy serialization of TaskConfig
                let config_json = serde_json::to_string(&self.task_config)
                    .unwrap_or_else(|e| format!("Error serializing config: {e}"));

                writeln!(
                    self.logfile,
                    "DATA, {}, {}, {}",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_secs(),
                    chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                    config_json // Log the JSON config
                )
                .unwrap_or_else(|e| error!("Failed to write to logfile: {}", e));
            }
            // Send reset signal (discard current trajectory in sim)
            self.pub_reset_discard_dataset
                .publish(&EmptyMsg {})
                .unwrap_or_else(|e| error!("Failed to publish reset message: {}", e));
        } else {
            error!("Cannot restart episode: subprocess is not running");
        }
    }

    fn about_window(&mut self, ui: &mut egui::Ui) {
        // ... (Keep existing logic, maybe update content path) ...
        if self.show_about {
            let available_rect = ui.ctx().available_rect();
            let center_point = available_rect.center();

            egui::containers::Window::new(egui::RichText::new("About").size(18.0))
                .interactable(true)
                .open(&mut self.show_about)
                .collapsible(false)
                .resizable(false)
                .fixed_size([512.0, 1024.0]) // Consider making resizable or adjusting size
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
                        commonmark_str!(
                            ui,
                            &mut self.commonmark_cache,
                            "crates/srb_gui/content/about.md"
                        );
                    });
                });
        }
    }

    fn virtual_keyboard_window(&mut self, ui: &mut egui::Ui) {
        // ... (Keep existing logic) ...
        if self.show_virtual_keyboard_window && self.current_page == Page::Interface {
            let available_rect = ui.ctx().available_rect();
            // let available_size = available_rect.size(); // Not used

            egui::containers::Window::new(egui::RichText::new("Gripper").size(16.0))
                .interactable(true)
                .collapsible(true)
                .resizable(false)
                // .max_size([0.61 * available_size.x, 0.61 * available_size.y]) // Remove max size for simplicity
                .default_rect(egui::Rect {
                    // Position near bottom right?
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
                            // Use a toggle button style
                            let button = egui::Button::new("🤏 Gripper Toggle");
                            if ui
                                .add(button)
                                .on_hover_text("Click to toggle gripper (Spacebar alternative)")
                                .clicked()
                            {
                                // Publish True, assuming the sim interprets any BoolMsg as a toggle trigger
                                self.pub_gripper_toggle
                                    .publish(&BoolMsg { data: true })
                                    .unwrap_or_else(|e| {
                                        error!("Failed to publish gripper toggle: {}", e);
                                    });
                            }
                        },
                    );
                });
        }
    }

    // Merge developer_options_window into configuration_page or keep separate?
    // Let's keep it separate for now, but maybe simplify it.
    fn developer_options_window(&mut self, ui: &mut egui::Ui) {
        if self.show_developer_options {
            egui::containers::Window::new(egui::RichText::new("⚙ Customize").size(18.0))
                .open(&mut self.show_developer_options) // Allow closing via 'x'
                .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-5.0, 5.0)) // Anchor to top-right
                .resizable(true)
                .collapsible(true)
                .show(ui.ctx(), |ui| {
                    ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                        ui.heading("Display & Debug");
                        ui.checkbox(
                            &mut self.show_virtual_keyboard_window,
                            "Show Gripper Button",
                        );
                        // Add other debug/display options here if needed

                        ui.separator();
                        ui.heading("Real-time Parameters");
                        // Sliders are already on the main config page, maybe just show values here?
                        ui.label(format!("Gravity: {:.3}", self.gravity));
                        ui.label(format!("Latency: {:.1}", self.latency));
                        ui.label(format!(
                            "Motion Sensitivity: {:.2}",
                            self.motion_sensitivity
                        ));
                        ui.label(format!(
                            "FFB Sensitivity: {:.2}",
                            self.force_feedback_sensitivity
                        ));
                        ui.label(format!("Max FFB: {:.1}", self.max_feedback_force));

                        // Or keep sliders here if preferred
                        // egui::Grid::new("dev_options_grid").show(ui, |ui| { ... });

                        ui.separator();
                        ui.heading("Logging");
                        ui.label(format!(
                            "Collected Trajectories: {}",
                            self.n_collected_trajectories
                        ));
                        ui.checkbox(&mut self.collect_trajectory, "Enable Trajectory Collection");
                    });
                });
        }
    }

    // --- Subprocess Management (Keep as is, ensure python path finding is robust) ---
    fn start_subprocess(&mut self) {
        // Use PathBuf for better path handling
        let srb_root = if let Some(root) = find_srb_root() {
            root
        } else {
            error!("Could not find SRB project root directory. Cannot determine script path.");
            // Optionally show an error dialog to the user
            return;
        };
        let python_script_path = srb_root.join("srb").join("__main__.py");

        if !python_script_path.exists() {
            error!(
                "Python script not found at expected location: {}",
                python_script_path.display()
            );
            return;
        }

        self.stop_subprocess(); // Ensure any previous process is stopped
        if self.subprocess.is_some() {
            warn!("Subprocess already running (should have been stopped).");
            return;
        }

        // Construct the subprocess command
        let python_exe = Self::get_isaacsim_python_exe();
        if python_exe.is_empty() {
            error!("Could not find Isaac Sim Python executable.");
            // Optionally show an error dialog
            return;
        }
        println!("Using Python: {python_exe}"); // Log which python is used

        let exec = subprocess::Exec::cmd(python_exe).arg(python_script_path.as_os_str()); // Use OsStr

        // Apply environment configuration arguments
        let exec = self.task_config.set_exec_env(exec);

        // Log the command being run
        info!("Starting subprocess with command: {:?}", exec);

        // Start the subprocess
        match exec.popen() {
            Ok(mut popen) => {
                // Brief wait to check if it started successfully
                match popen.wait_timeout(std::time::Duration::from_millis(2000)) {
                    Ok(Some(exit_status)) => {
                        error!(
                            "Subprocess failed to start or exited immediately with status: {:?}",
                            exit_status
                        );
                        // Optionally read stderr/stdout if captured
                    }
                    Ok(None) => {
                        info!("Subprocess started successfully.");
                        self.subprocess = Some(popen);
                    }
                    Err(e) => {
                        error!("Error waiting for subprocess: {}", e);
                    }
                }
            }
            Err(e) => {
                error!("Failed to execute subprocess: {}", e);
                // Optionally show error dialog
            }
        }
    }

    fn stop_subprocess(&mut self) {
        // Publish shutdown message first
        if let Err(e) = self.pub_gracefully_shutdown_process.publish(&EmptyMsg {}) {
            warn!("Failed to publish shutdown message: {}", e); // Don't stop here, still try to kill
        }

        // const SUBPROCESS_NAME_PY3: &str = "python3"; // Not used directly anymore
        // const SUBPROCESS_NAME_SH: &str = "python.sh"; // Not used directly anymore
        const SLEEP_DURATION: std::time::Duration = std::time::Duration::from_millis(50); // Shorter sleep
        const GRACEFUL_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(3); // Time for graceful shutdown

        if let Some(mut p) = self.subprocess.take() {
            // Take ownership
            info!("Stopping subprocess (PID: {:?})...", p.pid());

            let start_time = std::time::Instant::now();
            let mut terminated_gracefully = false;

            // Wait for graceful shutdown
            while start_time.elapsed() < GRACEFUL_TIMEOUT {
                match p.poll() {
                    Some(status) => {
                        info!("Subprocess exited with status: {:?}", status);
                        terminated_gracefully = true;
                        break;
                    }
                    None => {
                        // Still running, sleep briefly
                        std::thread::sleep(SLEEP_DURATION);
                    }
                }
            }

            if !terminated_gracefully {
                warn!("Subprocess did not exit gracefully within timeout. Attempting forceful termination...");
                // Try SIGTERM first
                let _ =
                    Self::kill_all_isaac_sim_python_processes(nix::sys::signal::Signal::SIGTERM);
                std::thread::sleep(SLEEP_DURATION * 5); // Give SIGTERM a moment

                if p.poll().is_none() {
                    // Check if still running
                    warn!("Subprocess still running after SIGTERM. Sending SIGKILL...");
                    // Force kill with SIGKILL
                    let _ = Self::kill_all_isaac_sim_python_processes(
                        nix::sys::signal::Signal::SIGKILL,
                    );
                    std::thread::sleep(SLEEP_DURATION); // Brief pause after SIGKILL
                }
            }

            // Final check
            if p.poll().is_none() {
                error!(
                    "Failed to terminate subprocess (PID: {:?}) even with SIGKILL.",
                    p.pid()
                );
                // Put it back if we failed? Or just log the error?
                // self.subprocess = Some(p); // Maybe not, it's likely defunct
            } else {
                info!("Subprocess stopped.");
            }
        } else {
            // No subprocess handle, but maybe a process is running? Attempt cleanup.
            warn!("No subprocess handle found. Attempting to clean up any stray Isaac Sim Python processes...");
            let _ = Self::kill_all_isaac_sim_python_processes(nix::sys::signal::Signal::SIGTERM);
            std::thread::sleep(SLEEP_DURATION * 2);
            let _ = Self::kill_all_isaac_sim_python_processes(nix::sys::signal::Signal::SIGKILL);
        }
    }

    // Refined process killing to target Isaac Sim specifically if possible
    fn kill_all_isaac_sim_python_processes(signal: nix::sys::signal::Signal) -> Result<(), String> {
        let system = sysinfo::System::new_all();
        // system.refresh_processes_specifics(ProcessesToUpdate::All); // Updated refresh_processes call

        let mut killed_count = 0;
        let mut errors = Vec::new();

        let isaac_sim_python_path_str = Self::get_isaacsim_python_exe(); // Get the specific python path/script
        let isaac_sim_python_path = PathBuf::from(&isaac_sim_python_path_str);

        for (pid, process) in system.processes() {
            // Check name and potentially command line/exe path for more specific targeting
            let name_matches = process.name() == "python3" || process.name() == "python.sh";
            // Check if the executable path matches or if srb/__main__.py is in the command arguments
            let cmd_matches = process
                .cmd()
                .iter()
                .any(|arg| arg.to_string_lossy().contains("srb/__main__.py"))
                || process
                    .exe()
                    .is_some_and(|exe_path| exe_path == isaac_sim_python_path);

            if name_matches && cmd_matches {
                trace!(
                    "Attempting to send signal {:?} to process {} ({:?})",
                    signal,
                    pid,
                    process.name()
                );
                match nix::sys::signal::kill(
                    nix::unistd::Pid::from_raw(pid.as_u32() as i32),
                    Some(signal),
                ) {
                    Ok(()) => killed_count += 1,
                    Err(e) => {
                        // Ignore ESRCH (No such process), log others
                        if e != nix::errno::Errno::ESRCH {
                            let err_msg =
                                format!("Failed to send signal {signal:?} to PID {pid}: {e}");
                            warn!("{}", err_msg);
                            errors.push(err_msg);
                        }
                    }
                }
            }
        }

        if killed_count > 0 {
            info!("Sent signal {:?} to {} process(es).", signal, killed_count);
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("\n"))
        }
    }

    fn get_isaacsim_python_exe() -> String {
        // Keep existing logic, maybe add more search paths or error handling
        if let Ok(python_exe) = std::env::var("ISAAC_SIM_PYTHON") {
            trace!("Using ISAAC_SIM_PYTHON env var: {}", python_exe);
            return python_exe.trim().to_owned();
        }

        // Try relative path often used in containers/dev setups
        let possible_paths = [
            "../isaac-sim/python.sh", // Common relative path from target/debug
            "../../isaac-sim/python.sh",
            "../../../isaac-sim/python.sh",
            "isaac-sim/python.sh", // Relative from project root
        ];
        for path_str in possible_paths {
            let path = PathBuf::from(path_str);
            if path.exists() {
                trace!(
                    "Found Isaac Sim Python at relative path: {}",
                    path.display()
                );
                // Return absolute path if possible for clarity
                return path
                    .canonicalize()
                    .map_or_else(|_| path.display().to_string(), |p| p.display().to_string());
            }
        }

        // Check common install locations (adjust paths as needed)
        let home_dir = home::home_dir().unwrap_or_else(|| PathBuf::from("/root")); // Use PathBuf
        let default_install_pattern = home_dir.join(".local/share/ov/pkg/isaac_sim-*/python.sh"); // Example pattern

        // Use glob to find matching paths
        use glob::glob;
        if let Ok(mut entries) = glob(&default_install_pattern.to_string_lossy()) {
            if let Some(Ok(path)) = entries.next() {
                // Take the first match
                trace!("Found Isaac Sim Python via glob: {}", path.display());
                return path.display().to_string();
            }
        }

        // Fallback to checking PATH for 'python3' - least reliable for Isaac Sim
        warn!("Isaac Sim Python not found via env var or common paths. Falling back to 'python3' from PATH.");
        match subprocess::Exec::cmd("which")
            .arg("python3")
            .stdout(subprocess::Redirection::Pipe)
            .capture()
        {
            Ok(data) if data.exit_status.success() => {
                let path = data.stdout_str().trim().to_owned();
                trace!("Using 'python3' from PATH: {}", path);
                path
            }
            _ => {
                error!(
                    "'python3' not found in PATH either. Unable to determine Python executable."
                );
                String::new() // Return empty string to indicate failure
            }
        }
    }

    // Keep dark_mode_toggle_button, publish_messages, show_top_center_bar,
    // show_trajectory_collection_checkbox, warn_if_debug_build

    pub fn dark_mode_toggle_button(&mut self, ui: &mut egui::Ui) {
        // ... (Keep existing logic) ...
        let (icon, tooltip, target_visuals) = match self.theme {
            egui::Theme::Dark => (
                "\u{e51c}", // Light mode icon (sun)
                "Switch to light mode",
                crate::style::light_visuals(),
            ),
            egui::Theme::Light => (
                "\u{e518}", // Dark mode icon (moon)
                "Switch to dark mode",
                crate::style::dark_visuals(),
            ),
        };

        if ui
            .add(egui::Button::new(icon).frame(false)) // Use frame(false) for icon-like button
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

    fn publish_messages(&mut self) {
        // Clamp values
        self.gravity = self.gravity.max(0.0);
        self.latency = self.latency.max(0.0);
        self.motion_sensitivity = self.motion_sensitivity.max(0.0);
        self.force_feedback_sensitivity = self.force_feedback_sensitivity.max(0.0);
        self.max_feedback_force = self.max_feedback_force.clamp(0.0, 10.0); // Increased max slightly

        // Publish only if changed (using f64::EPSILON for float comparison)
        if (self.prev_gravity - self.gravity).abs() > f64::EPSILON {
            self.prev_gravity = self.gravity;
            if let Err(e) = self.pub_gravity.publish(&Float64Msg { data: self.gravity }) {
                error!("Failed to publish gravity: {}", e);
            }
        }

        if (self.prev_latency - self.latency).abs() > f64::EPSILON {
            self.prev_latency = self.latency;
            if let Err(e) = self.pub_latency.publish(&Float64Msg {
                data: self.latency / 1000.0,
            }) {
                // Convert ms to s
                error!("Failed to publish latency: {}", e);
            }
        }

        if (self.prev_motion_sensitivity - self.motion_sensitivity).abs() > f64::EPSILON {
            self.prev_motion_sensitivity = self.motion_sensitivity;
            if let Err(e) = self.pub_motion_sensitivity.publish(&Float64Msg {
                data: self.motion_sensitivity,
            }) {
                error!("Failed to publish motion sensitivity: {}", e);
            }
        }

        if (self.prev_force_feedback_sensitivity - self.force_feedback_sensitivity).abs()
            > f64::EPSILON
        {
            self.prev_force_feedback_sensitivity = self.force_feedback_sensitivity;
            if let Err(e) = self.pub_force_feedback_sensitivity.publish(&Float64Msg {
                data: self.force_feedback_sensitivity,
            }) {
                error!("Failed to publish FFB sensitivity: {}", e);
            }
        }

        if (self.prev_max_feedback_force - self.max_feedback_force).abs() > f64::EPSILON {
            self.prev_max_feedback_force = self.max_feedback_force;
            if let Err(e) = self.pub_max_feedback_force.publish(&Float64Msg {
                data: self.max_feedback_force,
            }) {
                error!("Failed to publish max FFB: {}", e);
            }
        }
    }

    fn show_top_center_bar(&mut self, ui: &mut egui::Ui) {
        // Keep simple title
        ui.with_layout(
            egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
            |ui| {
                ui.heading("🚀 Space Robotics Bench GUI");
            },
        );
    }

    fn show_trajectory_collection_checkbox(&mut self, ui: &mut egui::Ui) {
        // Use a slightly smaller text size
        ui.add(egui::Checkbox::new(
            &mut self.collect_trajectory,
            format!(" Collect Trajectory ( {} )", self.n_collected_trajectories),
        ))
        .on_hover_text("Enable to log trajectory data when restarting episodes.\nYour participation makes our robots more intelligent!");
    }

    fn warn_if_debug_build(&mut self, ui: &mut egui::Ui) {
        // Keep as is
        if cfg!(debug_assertions) {
            ui.separator();
            // Use label instead of button for non-interactive warning
            ui.label(egui::RichText::new("⚠ Debug Build").color(Color32::YELLOW))
                .on_hover_text("Performance will be significantly lower in debug builds.");
            ui.separator();
        }
    }
}
