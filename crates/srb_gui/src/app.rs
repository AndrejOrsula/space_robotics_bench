use std::io::{Read, Write};

use eframe::epaint::Color32;
use egui_commonmark::{commonmark_str, CommonMarkCache};
use itertools::*;
use r2r::{
    std_msgs::msg::{Bool as BoolMsg, Empty as EmptyMsg, Float64 as Float64Msg},
    QosProfile,
};
use tracing::{error, info, trace, warn};

use crate::page::Page;

const LOGFILE_PATH: &str = "srb_gui.log";

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct App {
    theme: egui::Theme,
    #[serde(skip)]
    current_page: Page,
    #[serde(skip)]
    task_config: crate::config::TaskConfig,
    #[serde(skip)]
    subprocess: Option<subprocess::Popen>,

    #[serde(skip)]
    show_about: bool,
    #[serde(skip)]
    show_virtual_keyboard_window: bool,
    #[serde(skip)]
    show_developer_options: bool,

    #[serde(skip)]
    collect_trajectory: bool,
    #[serde(skip)]
    n_collected_trajectories: usize,

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

    #[serde(skip)]
    node: r2r::Node,
    #[serde(skip)]
    pub_gripper_toggle: r2r::Publisher<BoolMsg>,
    // #[serde(skip)]
    // pub_reset_save_dataset: r2r::Publisher<EmptyMsg>,
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

    #[serde(skip)]
    logfile: std::fs::File,

    #[serde(skip)]
    hovered_task: Option<usize>,

    #[serde(skip)]
    commonmark_cache: CommonMarkCache,
}

impl Default for App {
    fn default() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "srb_gui_gui", "").unwrap();

        let pub_gripper_toggle = node
            .create_publisher::<BoolMsg>("touch/event", QosProfile::default())
            .unwrap();
        // let pub_reset_save_dataset = node
        //     .create_publisher::<EmptyMsg>("gui/reset_save_dataset", QosProfile::default())
        //     .unwrap();
        let pub_reset_discard_dataset = node
            .create_publisher::<EmptyMsg>("gui/reset_discard_dataset", QosProfile::default())
            .unwrap();
        let pub_gracefully_shutdown_process = node
            .create_publisher::<EmptyMsg>("gui/shutdown_process", QosProfile::default())
            .unwrap();
        let pub_gravity = node
            .create_publisher::<Float64Msg>("gui/gravity", QosProfile::default())
            .unwrap();
        let pub_latency = node
            .create_publisher::<Float64Msg>("gui/latency", QosProfile::default())
            .unwrap();
        let pub_motion_sensitivity = node
            .create_publisher::<Float64Msg>("gui/motion_sensitivity", QosProfile::default())
            .unwrap();
        let pub_force_feedback_sensitivity = node
            .create_publisher::<Float64Msg>("gui/force_feedback_sensitivity", QosProfile::default())
            .unwrap();
        let pub_max_feedback_force = node
            .create_publisher::<Float64Msg>("gui/max_feedback_force", QosProfile::default())
            .unwrap();

        let last_message_pub = std::time::Instant::now();

        let mut logfile = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .append(true)
            .open(LOGFILE_PATH)
            .unwrap();

        // Iterate over the lines and count all collected trajectories
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

        Self {
            theme: egui::Theme::Dark,
            current_page: Page::default(),
            task_config: crate::config::TaskConfig::default(),
            subprocess: None,

            show_about: false,
            show_virtual_keyboard_window: false,
            show_developer_options: false,

            collect_trajectory: false,
            n_collected_trajectories,

            gravity: crate::config::TaskConfig::default()
                .env_cfg
                .scenario
                .gravity_magnitude(),
            latency: 0.0,
            motion_sensitivity: 1.0,
            force_feedback_sensitivity: 1.0,
            max_feedback_force: 2.0,

            prev_gravity: 0.0,
            prev_latency: 0.0,
            prev_motion_sensitivity: 0.0,
            prev_force_feedback_sensitivity: 0.0,
            prev_max_feedback_force: 0.0,

            node,
            pub_gripper_toggle,
            // pub_reset_save_dataset,
            pub_reset_discard_dataset,
            pub_gracefully_shutdown_process,
            pub_gravity,
            pub_latency,
            pub_motion_sensitivity,
            pub_force_feedback_sensitivity,
            pub_max_feedback_force,

            last_message_pub,

            logfile,

            hovered_task: None,

            commonmark_cache: CommonMarkCache::default(),
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
            // Try to restore previous state
            eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default()
        } else {
            // Otherwise, use default state
            Self::default()
        };

        // Set the theme
        crate::style::set_theme(&cc.egui_ctx, app.theme);

        // Publish messages
        app.publish_messages();

        app
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Synchronize the page URL and content if the URL contains a hash
        #[cfg(target_arch = "wasm32")]
        if let Some(page) = frame.info().web_info.location.hash.strip_prefix('#') {
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

        // Support native fullscreen toggle
        #[cfg(not(target_arch = "wasm32"))]
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F11)) {
            let fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!fullscreen));
        }

        // Navigation panel that allows switching between page
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                // Navigation
                ui.spacing_mut().item_spacing.x = 16.0;
                self.navigation_buttons(ui);

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    self.advanced_opts_button(ui);

                    self.show_top_center_bar(ui);
                });
            });
        });

        // Bottom panel
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    self.about_button(ui);

                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        self.dark_mode_toggle_button(ui);
                        ui.separator();
                        self.show_trajectory_collection_checkbox(ui);
                        self.warn_if_debug_build(ui);
                    });
                });
            });
        });

        // Central panel
        crate::utils::egui::ScrollableFramedCentralPanel::builder()
            .max_content_width(ctx.screen_rect().width())
            .build()
            .show(ctx, |ui| {
                match self.current_page {
                    Page::QuickStart => {
                        self.quickstart_page(ui);
                    }
                    Page::Interface => {
                        self.configuration_page(ui);
                    }
                };
                self.about_window(ui);
                self.virtual_keyboard_window(ui);
                self.developer_options_window(ui);
            });

        // Publish values
        if self.current_page == Page::Interface || self.show_developer_options {
            self.publish_messages();
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(&mut *self)
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }
}

impl App {
    fn navigation_buttons(&mut self, ui: &mut egui::Ui) {
        for page in crate::ENABLED_PAGES {
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
        // About button
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
        // Advanced options button
        let button = ui
            .add(egui::Button::new({
                let text = egui::RichText::new("Advanced Options");
                if self.show_about {
                    text.strong()
                } else {
                    text
                }
            }))
            .on_hover_text(if self.show_about {
                "Close the associated window"
            } else {
                "Show advanced options"
            });
        // If the button is clicked, open the advanced options window
        if button.clicked() {
            self.show_developer_options = !self.show_developer_options;
        }
    }

    fn quickstart_page(&mut self, ui: &mut egui::Ui) {
        let quick_start_options: [(
            egui::Theme,
            &str,
            crate::utils::Difficulty,
            egui::ImageSource,
            crate::config::TaskConfig,
        ); 9] = [
            (
                egui::Theme::Dark,
                "Lunar Sample Collection",
                crate::utils::Difficulty::Easy,
                crate::macros::include_content_image!("_images/envs/sample_collection_moon.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::SampleCollection,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Moon,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 1,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Dark,
                "Mars Sample Tube Collection",
                crate::utils::Difficulty::Easy,
                crate::macros::include_content_image!("_images/envs/sample_collection_mars.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::SampleCollection,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Mars,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 3,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Light,
                "Debris Capture",
                crate::utils::Difficulty::Medium,
                crate::macros::include_content_image!("_images/envs/debris_capture_orbit.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::DebrisCapture,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Orbit,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::None,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 2,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Dark,
                "Lunar Peg-in-Hole Assembly",
                crate::utils::Difficulty::Medium,
                crate::macros::include_content_image!("_images/envs/peg_in_hole_moon.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::PegInHole,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Moon,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 7,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Dark,
                "Mars Perseverance Rover",
                crate::utils::Difficulty::Demo,
                crate::macros::include_content_image!("_images/envs/perseverance.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::Perseverance,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Mars,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 4,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Light,
                "Peg-in-Hole Assembly",
                crate::utils::Difficulty::Challenging,
                crate::macros::include_content_image!("_images/envs/peg_in_hole_orbit.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::PegInHole,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Orbit,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::None,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 9,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Dark,
                "Solar Panel Assembly",
                crate::utils::Difficulty::Challenging,
                crate::macros::include_content_image!("_images/envs/solar_panel_assembly_moon.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::SolarPanelAssembly,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Moon,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 12,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Dark,
                "Ingenuity Mars Helicopter",
                crate::utils::Difficulty::Demo,
                crate::macros::include_content_image!("_images/envs/ingenuity.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::Ingenuity,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Mars,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 7,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
            (
                egui::Theme::Light,
                "Canadarm3",
                crate::utils::Difficulty::Demo,
                crate::macros::include_content_image!("_images/envs/gateway.jpg"),
                crate::config::TaskConfig {
                    task: crate::config::Task::Gateway,
                    num_envs: 1,
                    env_cfg: srb::envs::EnvironmentConfig {
                        scenario: srb::envs::Scenario::Orbit,
                        assets: srb::envs::Assets {
                            robot: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            object: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                            terrain: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Procedural,
                            },
                            vehicle: srb::envs::Asset {
                                variant: srb::envs::AssetVariant::Dataset,
                            },
                        },
                        seed: 8,
                        detail: 1.0,
                    },
                    enable_ui: false,
                },
            ),
        ];

        ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
            egui::Grid::new("extra_nav_buttons")
                .spacing(egui::vec2(8.0, 8.0))
                .show(ui, |ui| {
                    const N_COLS: usize = 3;
                    const N_ROWS: usize = 3;
                    let target_button_width = ui.ctx().available_rect().width() / N_COLS as f32
                        - 1.25 * ui.spacing().item_spacing.x;
                    let target_button_height = (ui.ctx().available_rect().height() - 18.0)
                        / N_ROWS as f32
                        - 1.5 * ui.spacing().item_spacing.y;

                    let mut hovered_task = None;

                    quick_start_options.into_iter().enumerate().for_each(
                        |(i, (theme, task, difficulty, thumbnail, config))| {
                            ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                                difficulty.set_theme(ui, self.theme);

                                let button = ui.add_sized(
                                    egui::Vec2::new(target_button_width, target_button_height),
                                    egui::ImageButton::new(thumbnail).rounding(
                                        0.01 * (target_button_height + target_button_width),
                                    ),
                                );
                                if button.clicked() {
                                    self.task_config = config;
                                    self.gravity =
                                        self.task_config.env_cfg.scenario.gravity_magnitude();
                                    self.start_subprocess();
                                    self.current_page = Page::Interface;
                                }
                                if button.hovered() {
                                    hovered_task = Some(i);
                                }

                                let task_text_size = 0.125 * button.rect.height();
                                ui.allocate_new_ui(
                                    egui::UiBuilder::new().max_rect(egui::Rect {
                                        min: egui::Pos2 {
                                            x: button.rect.min.x,
                                            y: button.rect.max.y - 1.3 * task_text_size,
                                        },
                                        max: egui::Pos2 {
                                            x: button.rect.max.x,
                                            y: button.rect.max.y - 0.3 * task_text_size,
                                        },
                                    }),
                                    |ui| {
                                        ui.with_layout(
                                            egui::Layout::bottom_up(egui::Align::Center),
                                            |ui| {
                                                ui.add(
                                                    egui::Label::new(
                                                        egui::RichText::new(task)
                                                            .color(
                                                                // if button.hovered() {
                                                                match theme {
                                                                    egui::Theme::Light => {
                                                                        Color32::from_rgb(
                                                                            // 11, 12, 16,
                                                                            0, 0, 0,
                                                                        )
                                                                    }
                                                                    egui::Theme::Dark => {
                                                                        Color32::from_rgb(
                                                                            205, 214, 244,
                                                                        )
                                                                    }
                                                                },
                                                            )
                                                            .size(task_text_size),
                                                    )
                                                    .selectable(false),
                                                )
                                            },
                                        )
                                    },
                                );

                                let difficulty_text_size = 0.1125 * button.rect.height();
                                ui.allocate_new_ui(
                                    egui::UiBuilder::new().max_rect(egui::Rect {
                                        min: egui::Pos2 {
                                            x: button.rect.min.x + 0.5 * difficulty_text_size,
                                            y: button.rect.min.y + 0.4 * difficulty_text_size,
                                        },
                                        max: egui::Pos2 {
                                            x: button.rect.max.x,
                                            y: button.rect.min.y + 1.4 * difficulty_text_size,
                                        },
                                    }),
                                    |ui| {
                                        ui.with_layout(
                                            egui::Layout::left_to_right(egui::Align::TOP),
                                            |ui| {
                                                ui.add(
                                                    egui::Label::new(
                                                        egui::RichText::new(difficulty.to_string())
                                                            .color(difficulty.get_text_color(
                                                                if self.theme == egui::Theme::Dark {
                                                                    button.hovered()
                                                                } else {
                                                                    !button.hovered()
                                                                },
                                                            ))
                                                            .size(difficulty_text_size),
                                                    )
                                                    .selectable(false),
                                                )
                                            },
                                        )
                                    },
                                );
                            });

                            if (i + 1) % N_COLS == 0 {
                                ui.end_row();
                            }
                        },
                    );

                    self.hovered_task = hovered_task;
                });
        });
    }

    fn configuration_page(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical()
            .show(ui, |ui| {
                let margin_x = (ui.ctx().screen_rect().width() - 768.0).max(0.0) / 2.0;
                let inner_margin = egui::Margin {
                    left: margin_x.max(0.0),
                    right: margin_x.max(0.0),
                    ..egui::Margin::default()
                };
                egui::Frame::default()
                    .inner_margin(inner_margin)
                    .show(ui, |ui|
                    {
        ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
            ui.with_layout(
                egui::Layout::left_to_right(egui::Align::TOP)
                    , |ui| {
                        ui.spacing_mut().item_spacing.x = 0.0;

                let task_focus=match self.task_config.task {
                            crate::config::Task::SampleCollection
                            | crate::config::Task::DebrisCapture
                            | crate::config::Task::PegInHole
                            | crate::config::Task::SolarPanelAssembly =>
                    {
                        "Objective"
                    }
                    _ => {
                        "Demo"
                    }
                };
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(format!("{task_focus}\n")).strong().size(24.0),
                    )
                    .selectable(false),
                );

                    match self.task_config.task {
                        crate::config::Task::SampleCollection => {
                            let sample_type = match self.task_config.env_cfg.scenario {
                                srb::envs::Scenario::Moon => "Lunar ",
                                srb::envs::Scenario::Mars => "Martian ",
                                _ => "",
                            };

                            ui.add(
                                egui::Label::new(egui::RichText::new(format!("\t1) Grasp the {sample_type}sample.\n\t2) [Optional] Place it in the rover cargo bay.")).monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::DebrisCapture => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t— Detumble and grasp the \"misplaced\" object.").monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::PegInHole => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t1) Grasp the profile.\n\t2) Align the profile with the hole.\n\t3) Insert the profile into the hole.").monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::SolarPanelAssembly => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t1) Grasp, align, and insert all profiles into their holes.\n\t2) Place the solar panel on top of the profiles.").monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::Perseverance => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t— Explore the Martian terrain with the Perseverance rover.").monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::Ingenuity => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t— Explore the Martian terrain with the Ingenuity helicopter.").monospace()).selectable(false)
                            );
                        }
                        crate::config::Task::Gateway => {
                            ui.add(
                                egui::Label::new(egui::RichText::new("\t— Explore the Gateway space station with the Canadarm3 robotic arm.").monospace()).selectable(false)
                            );
                        }
                    }
                });


                ui.add_space(25.0);

                egui::Grid::new("real_time_env_config").show(ui, |ui| {
                    ui.style_mut().spacing.slider_width = 170.0;

                    ui.add(egui::Label::new(egui::RichText::new("\u{e80b} Gravity").size(22.0)).selectable(false)).on_hover_text(
                        "Acceleration due to gravity"
                    );
                    ui.add(egui::Slider::new(&mut self.gravity, 0.0..=25.0)
                    .trailing_fill(true).custom_formatter(|x, _| format!("{x:.2}")).suffix(" m/s²").custom_parser(
                        |x| if let Ok(x) = x.parse::<f64>() {
                            Some(x.max(0.0_f64))
                        } else {
                            None
                        }
                    ));

                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Zero")
                            ).frame(true)).clicked() {
                            self.gravity = srb::envs::Scenario::Orbit.gravity_magnitude();
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Moon")
                            ).frame(true)).clicked() {
                            self.gravity = srb::envs::Scenario::Moon.gravity_magnitude();
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Mars")
                            ).frame(true)).clicked() {
                            self.gravity = srb::envs::Scenario::Mars.gravity_magnitude();
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Earth")
                            ).frame(true)).clicked() {
                            self.gravity = 9.81;
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Jupiter")
                            ).frame(true)).clicked() {
                            self.gravity = 24.79;
                        }
                    });
                    ui.end_row();

                    ui.add(egui::Label::new(egui::RichText::new("\u{e8b5} Latency").size(22.0)).selectable(false)).on_hover_text(
                        "One-way communication delay between the operator and the robot"
                    );
                    ui.add(egui::Slider::new(&mut self.latency, 0.0..=2000.0)
                    .trailing_fill(true).custom_formatter(|x, _| format!("{x:.0}")).suffix(" ms"));
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        if ui.add(
                            egui::Button::new(egui::RichText::new("None")
                            ).frame(true)).clicked() {
                            self.latency = 0.0;
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("LEO")
                            ).frame(true)).clicked() {
                            self.latency = 50.0;
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("MEO")
                            ).frame(true)).clicked() {
                            self.latency = 225.0;
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("GEO")
                            ).frame(true)).clicked() {
                            self.latency = 280.0;
                        }
                        if ui.add(
                            egui::Button::new(egui::RichText::new("Moon")
                            ).frame(true)).clicked() {
                            self.latency = 1250.0;
                        }
                        ui.end_row();
                    });

                    ui.end_row();
                });

                ui.add_space(20.0);

                ui.with_layout(
                    egui::Layout::bottom_up(egui::Align::Center),
                    |ui| {
                        let max_button_width = ui.ctx().available_rect().width().min(768.0);

                        if ui.add(egui::Button::new(egui::RichText::new("\u{f23a} Exit").size(32.0)).min_size(
                            egui::Vec2::new(max_button_width, 75.0),
                        ).frame(true)).clicked() {
                            self.restart_episode();
                            self.current_page = Page::QuickStart;
                            self.show_about = false;
                            self.gravity = self.task_config.env_cfg.scenario.gravity_magnitude();
                            self.latency = 0.0;
                            self.motion_sensitivity = 1.0;
                            self.force_feedback_sensitivity = 1.0;
                            self.max_feedback_force = 2.0;
                            self.collect_trajectory = false;
                            self.task_config = crate::config::TaskConfig::default();
                    self.stop_subprocess();
                }

                ui.add_space(10.0);

                if ui.add(egui::Button::new(egui::RichText::new("\u{e5d5} Restart").size(32.0)).min_size(
                    egui::Vec2::new(max_button_width, 75.0),
                ).frame(true)).clicked() {
                        self.restart_episode();
                    }
                });
                });
            });
        });
    }

    fn restart_episode(&mut self) {
        if self.subprocess.is_some() {
            info!("Restarting episode");
            // if self.collect_trajectory {
            //     info!("Saving trajectory");
            //     self.pub_reset_save_dataset.publish(&EmptyMsg {}).unwrap();
            //     self.n_collected_trajectories += 1;

            writeln!(
                self.logfile,
                "DATA, {}, {}, {}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_secs(),
                chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                self.task_config
            )
            .unwrap();
            // } else {
            self.pub_reset_discard_dataset
                .publish(&EmptyMsg {})
                .unwrap();
            // }
        } else {
            error!("Cannot restart episode: subprocess is not running");
        }
    }

    fn about_window(&mut self, ui: &mut egui::Ui) {
        if self.show_about {
            let available_rect = ui.ctx().available_rect();
            let center_point = available_rect.center();

            egui::containers::Window::new(egui::RichText::new("About").size(18.0))
                .interactable(true)
                .open(&mut self.show_about)
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
        if self.show_virtual_keyboard_window && self.current_page == Page::Interface {
            let available_rect = ui.ctx().available_rect();
            let available_size = available_rect.size();

            egui::containers::Window::new(egui::RichText::new("Gripper").size(16.0))
                .interactable(true)
                .collapsible(true)
                .resizable(false)
                .max_size([0.61 * available_size.x, 0.61 * available_size.y])
                .default_rect(egui::Rect {
                    min: egui::Pos2::new(available_rect.max.x, available_rect.min.y),
                    max: egui::Pos2::new(available_rect.max.x, available_rect.min.y),
                })
                .show(ui.ctx(), |ui| {
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::TOP), |ui| {
                        if ui
                            .add(
                                egui::Button::new(
                                    egui::RichText::new("Toggle\nGripper")
                                        .heading()
                                        .size(32.0),
                                )
                                    .frame(true),
                            )
                            .on_hover_text("\u{e811} The button stopped working \u{e811}\n          (this is a workaround)")
                            .clicked()
                        {
                            self.pub_gripper_toggle
                                .publish(&BoolMsg { data: true })
                                .unwrap();
                        }
                    });
                });
        }
    }

    fn developer_options_window(&mut self, ui: &mut egui::Ui) {
        if self.show_developer_options {
            let available_rect = ui.ctx().available_rect();
            egui::containers::Window::new(egui::RichText::new("Customization").size(22.0))
                .interactable(true)
                .collapsible(false)
                .resizable(false)
                .default_rect(egui::Rect {
                    min: available_rect.max,
                    max: available_rect.max,
                })
                .fixed_size([384.0, 256.0])
                .show(ui.ctx(), |ui| {
                    ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                        egui::Grid::new("extra_developer_options_grid").show(ui, |ui| {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{ef75} Scenario").size(20.0),
                                )
                                .selectable(false),
                            );
                            egui::ComboBox::new("dev_scenario_combo_box", "")
                                .width(235.0)
                                .selected_text(
                                    egui::RichText::new(match self.task_config.env_cfg.scenario {
                                        srb::envs::Scenario::Asteroid => "Asteroid",
                                        srb::envs::Scenario::Earth => "Earth",
                                        srb::envs::Scenario::Mars => "Mars",
                                        srb::envs::Scenario::Moon => "Moon",
                                        srb::envs::Scenario::Orbit => "Orbit",
                                    })
                                    .size(20.0),
                                )
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.scenario,
                                        srb::envs::Scenario::Moon,
                                        "Moon",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.scenario,
                                        srb::envs::Scenario::Mars,
                                        "Mars",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.scenario,
                                        srb::envs::Scenario::Orbit,
                                        "Orbit",
                                    );
                                });

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(format!(
                                        "\u{eb9b} {}",
                                        match self.task_config.task {
                                            crate::config::Task::SampleCollection
                                            | crate::config::Task::DebrisCapture
                                            | crate::config::Task::PegInHole
                                            | crate::config::Task::SolarPanelAssembly => "Task",
                                            crate::config::Task::Perseverance
                                            | crate::config::Task::Ingenuity
                                            | crate::config::Task::Gateway => "Demo",
                                        }
                                    ))
                                    .size(20.0),
                                )
                                .selectable(false),
                            );
                            egui::ComboBox::new("dev_task_combo_box", "")
                                .width(235.0)
                                .selected_text(
                                    egui::RichText::new(match self.task_config.task {
                                        crate::config::Task::SampleCollection => {
                                            "Sample Collection"
                                        }
                                        crate::config::Task::PegInHole => "Peg-in-Hole",
                                        crate::config::Task::SolarPanelAssembly => {
                                            "Solar Panel Assembly"
                                        }
                                        crate::config::Task::DebrisCapture => "Debris Capture",
                                        crate::config::Task::Perseverance => "Perseverance",
                                        crate::config::Task::Ingenuity => "Ingenuity",
                                        crate::config::Task::Gateway => "Gateway",
                                    })
                                    .size(20.0),
                                )
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::SampleCollection,
                                        "Sample Collection",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::PegInHole,
                                        "Peg-in-Hole",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::SolarPanelAssembly,
                                        "Solar Panel Assembly",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::DebrisCapture,
                                        "Debris Capture",
                                    );
                                    ui.separator();
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::Perseverance,
                                        "Perseverance",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::Ingenuity,
                                        "Ingenuity",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.task,
                                        crate::config::Task::Gateway,
                                        "Gateway",
                                    );
                                });

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{ea3c} Object Assets").size(20.0),
                                )
                                .selectable(false),
                            );
                            egui::ComboBox::new("dev_object_asset_combo_box", "")
                                .width(235.0)
                                .selected_text(
                                    egui::RichText::new(
                                        match self.task_config.env_cfg.assets.object.variant {
                                            srb::envs::AssetVariant::None => "None",
                                            srb::envs::AssetVariant::Primitive => "Primitive",
                                            srb::envs::AssetVariant::Dataset => "Dataset",
                                            srb::envs::AssetVariant::Procedural => "Procedural",
                                        },
                                    )
                                    .size(20.0),
                                )
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.assets.object.variant,
                                        srb::envs::AssetVariant::Primitive,
                                        "Primitive",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.assets.object.variant,
                                        srb::envs::AssetVariant::Dataset,
                                        "Dataset",
                                    );
                                    ui.selectable_value(
                                        &mut self.task_config.env_cfg.assets.object.variant,
                                        srb::envs::AssetVariant::Procedural,
                                        "Procedural",
                                    );
                                });

                            ui.end_row();

                            ui.style_mut().spacing.slider_width = 185.0;

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e9e1} Random seed").size(20.0),
                                )
                                .selectable(false),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.task_config.env_cfg.seed, 0..=42)
                                    .trailing_fill(true)
                                    .integer()
                                    .custom_formatter(|x, _| format!("{x}")),
                            );

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e3ea} Parallel envs").size(20.0),
                                )
                                .selectable(false),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.task_config.num_envs, 1..=16)
                                    .clamping(egui::SliderClamping::Always)
                                    .trailing_fill(true)
                                    .integer()
                                    .custom_formatter(|x, _| format!("{x}")),
                            );

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e839} Level of detail").size(20.0),
                                )
                                .selectable(false),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.task_config.env_cfg.detail, 0.01..=2.0)
                                    .clamping(egui::SliderClamping::Always)
                                    .trailing_fill(true)
                                    .custom_formatter(|x, _| format!("{x:.1}")),
                            );

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e1bd} Enable UI").size(20.0),
                                )
                                .selectable(false),
                            );
                            let is_enabled = self.task_config.enable_ui;
                            ui.add(egui::Checkbox::new(
                                &mut self.task_config.enable_ui,
                                egui::RichText::new(if is_enabled {
                                    " Enabled"
                                } else {
                                    " Disabled"
                                })
                                .size(20.0),
                            ));

                            ui.end_row();
                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e766} Feedback sensitivity").size(20.0),
                                )
                                .selectable(false),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.force_feedback_sensitivity, 0.0..=20.0)
                                    .clamping(egui::SliderClamping::Always)
                                    .trailing_fill(true)
                                    .custom_formatter(|x, _| format!("{x:.1}")),
                            );

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{e9e4} Feedback force limit").size(20.0),
                                )
                                .selectable(false),
                            );
                            ui.add(
                                egui::Slider::new(&mut self.max_feedback_force, 0.0..=5.0)
                                    .clamping(egui::SliderClamping::Always)
                                    .trailing_fill(true)
                                    .custom_formatter(|x, _| format!("{x:.1}")),
                            );

                            ui.end_row();

                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new("\u{f049} Virtual gripper").size(20.0),
                                )
                                .selectable(false),
                            );
                            let is_enabled = self.show_virtual_keyboard_window;
                            ui.add(egui::Checkbox::new(
                                &mut self.show_virtual_keyboard_window,
                                egui::RichText::new(if is_enabled {
                                    " Enabled"
                                } else {
                                    " Disabled"
                                })
                                .size(20.0),
                            ));
                        });

                        // ui.add_space(10.0);

                        ui.separator();

                        ui.with_layout(
                            egui::Layout::centered_and_justified(egui::Direction::TopDown),
                            |ui| {
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new("\u{e1c4}").size(40.0),
                                        )
                                        .frame(false),
                                    )
                                    .clicked()
                                {
                                    self.gravity =
                                        self.task_config.env_cfg.scenario.gravity_magnitude();
                                    self.start_subprocess();
                                    self.current_page = Page::Interface;
                                }
                            },
                        );
                    });
                });
        }
    }

    fn start_subprocess(&mut self) {
        const PYTHON_SCRIPT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../scripts/teleop.py");

        self.stop_subprocess();
        if self.subprocess.is_some() {
            warn!("Subprocess already running.");
            return;
        }

        // Construct the subprocess
        println!("python: {}", Self::get_isaacsim_python_exe());
        let exec = subprocess::Exec::cmd(Self::get_isaacsim_python_exe()).arg(PYTHON_SCRIPT);
        let exec = self.task_config.set_exec_env(exec);

        // Start the subprocess
        info!("Starting subprocess...");
        let mut popen = exec.popen().unwrap();

        popen
            .wait_timeout(std::time::Duration::from_millis(2000))
            .unwrap();

        if popen.poll().is_none() {
            info!("Subprocess started.");
            self.subprocess = Some(popen);
        } else {
            error!("Failed to start subprocess.");
        }
    }

    /// Note: The `python.sh` interface of isaac sim launches a new process `python3` but does not propagate the signal to the child process.
    /// Therefore, we need kill the `python3` process directly.
    fn stop_subprocess(&mut self) {
        self.pub_gracefully_shutdown_process
            .publish(&EmptyMsg {})
            .unwrap();

        const SUBPROCESS_NAME: &str = "python3";
        const SLEEP_DURATION: std::time::Duration = std::time::Duration::from_millis(10);

        if let Some(p) = &mut self.subprocess {
            info!("Stopping subprocess...");

            if p.wait_timeout(std::time::Duration::from_millis(1000))
                .unwrap()
                .is_some()
            {
                info!("Subprocess stopped.");
                self.subprocess = None;
                return;
            }

            // Try to terminate the process gracefully
            for signal in [
                nix::sys::signal::Signal::SIGINT,
                nix::sys::signal::Signal::SIGTERM,
                nix::sys::signal::Signal::SIGKILL,
            ] {
                loop {
                    let _ = Self::kill_all_processes_by_name(SUBPROCESS_NAME, signal);

                    if p.wait_timeout(SLEEP_DURATION).unwrap().is_some() {
                        info!("Subprocess stopped.");
                        self.subprocess = None;
                        return;
                    }

                    if signal != nix::sys::signal::Signal::SIGKILL {
                        break;
                    }
                }
            }
        } else {
            warn!("No known subprocess to stop.");
            // Directly kill the process if the subprocess might have been spawned by other means
            let _ = Self::kill_all_processes_by_name(
                SUBPROCESS_NAME,
                nix::sys::signal::Signal::SIGKILL,
            );
        }
    }

    fn get_isaacsim_python_exe() -> String {
        if let Ok(python_exe) = std::env::var("ISAAC_SIM_EXE") {
            trace!("ISAAC_SIM_EXE: {}", python_exe);
            python_exe.trim().to_owned()
        } else {
            let home_dir = home::home_dir().unwrap_or("/root".into());
            let isaac_sim_python_sh = home_dir.join("isaac-sim/python.sh");
            if std::path::Path::new(&isaac_sim_python_sh).exists() {
                trace!("ISAAC_SIM_EXE: {}", isaac_sim_python_sh.display());
                isaac_sim_python_sh.display().to_string()
            } else {
                trace!("ISAAC_SIM_EXE: which python3");
                subprocess::Exec::cmd("which")
                    .arg("python3")
                    .stdout(subprocess::Redirection::Pipe)
                    .stderr(subprocess::Redirection::Merge)
                    .capture()
                    .expect("No Python interpreter was found.")
                    .stdout_str()
                    .trim()
                    .to_owned()
            }
        }
    }

    fn kill_all_processes_by_name(name: &str, signal: nix::sys::signal::Signal) -> nix::Result<()> {
        // Create a System object to gather information
        let mut system = sysinfo::System::new_all();
        // Refresh the process list
        system.refresh_all();

        // Iterate through all processes
        for process in system.processes() {
            // Check if the process name matches the given name
            if process.1.name() == name {
                let pid = process.1.pid();
                // Send a signal to the process
                trace!("Killing process {} with signal {:?}", pid, signal);
                nix::sys::signal::kill(nix::unistd::Pid::from_raw(pid.as_u32() as i32), signal)?;
            }
        }

        Ok(())
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
            .add(egui::Button::new(icon))
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
        self.gravity = self.gravity.max(0.0);
        self.latency = self.latency.max(0.0);
        self.motion_sensitivity = self.motion_sensitivity.max(0.0);
        self.force_feedback_sensitivity = self.force_feedback_sensitivity.max(0.0);
        self.max_feedback_force = self.max_feedback_force.clamp(0.0, 5.0);

        if self.prev_gravity != self.gravity {
            self.prev_gravity = self.gravity;
            self.pub_gravity
                .publish(&Float64Msg { data: self.gravity })
                .unwrap();
        }

        if self.prev_latency != self.latency {
            self.prev_latency = self.latency;
            self.pub_latency
                .publish(&Float64Msg {
                    data: self.latency / 1000.0,
                })
                .unwrap();
        }

        if self.prev_motion_sensitivity != self.motion_sensitivity {
            self.prev_motion_sensitivity = self.motion_sensitivity;
            self.pub_motion_sensitivity
                .publish(&Float64Msg {
                    data: self.motion_sensitivity,
                })
                .unwrap();
        }

        if self.prev_force_feedback_sensitivity != self.force_feedback_sensitivity {
            self.prev_force_feedback_sensitivity = self.force_feedback_sensitivity;
            self.pub_force_feedback_sensitivity
                .publish(&Float64Msg {
                    data: self.force_feedback_sensitivity,
                })
                .unwrap();
        }

        if self.prev_max_feedback_force != self.max_feedback_force {
            self.prev_max_feedback_force = self.max_feedback_force;
            self.pub_max_feedback_force
                .publish(&Float64Msg {
                    data: self.max_feedback_force,
                })
                .unwrap();
        }
    }

    fn show_top_center_bar(&mut self, ui: &mut egui::Ui) {
        ui.with_layout(
            egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
            |ui| match self.current_page {
                Page::QuickStart => {
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new("\u{eb9b} Select your Experience \u{ef75}     ")
                                .family(egui::FontFamily::Proportional)
                                .strong(),
                        )
                        .selectable(false),
                    );
                }
                Page::Interface => {
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new("\u{f049} Complete the Task \u{ea3c}     ")
                                .family(egui::FontFamily::Proportional)
                                .strong(),
                        )
                        .selectable(false),
                    );
                }
            },
        );
    }

    fn show_trajectory_collection_checkbox(&mut self, ui: &mut egui::Ui) {
        ui.add(egui::Checkbox::new(
            &mut self.collect_trajectory,
            egui::RichText::new(format!(
                " Collect Trajectory ( {} samples )",
                self.n_collected_trajectories
            ))
            .heading()
            .size(20.0),
        ))
        .on_hover_text("Your participation makes our robots more intelligent!");
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
                            self.current_page,
                            ui.ctx().screen_rect().size(),
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