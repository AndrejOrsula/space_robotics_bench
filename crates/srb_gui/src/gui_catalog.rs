use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use crate::env_cfg::{Domain, InterfaceType, RenderingMode, TaskConfig, TeleopDevice, Workflow};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiCatalogSemanticField {
    Workflow,
    Domain,
    RenderingMode,
}

#[derive(Clone, Debug, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)] // Explicit field names are part of the typed diagnostic.
pub enum GuiCatalogTaskError {
    InvalidWorkflow {
        task_id: String,
        value: String,
        reason: String,
    },
    InvalidDomain {
        task_id: String,
        value: String,
        reason: String,
    },
    InvalidRenderingMode {
        task_id: String,
        value: String,
        reason: String,
    },
}

impl GuiCatalogTaskError {
    fn invalid(field: GuiCatalogSemanticField, task_id: &str, value: &str, reason: String) -> Self {
        let task_id = task_id.to_string();
        let value = value.to_string();
        match field {
            GuiCatalogSemanticField::Workflow => Self::InvalidWorkflow {
                task_id,
                value,
                reason,
            },
            GuiCatalogSemanticField::Domain => Self::InvalidDomain {
                task_id,
                value,
                reason,
            },
            GuiCatalogSemanticField::RenderingMode => Self::InvalidRenderingMode {
                task_id,
                value,
                reason,
            },
        }
    }
}

impl std::fmt::Display for GuiCatalogTaskError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidWorkflow {
                task_id,
                value,
                reason,
            } => write!(
                formatter,
                "task '{task_id}' has invalid workflow '{value}': {reason}"
            ),
            Self::InvalidDomain {
                task_id,
                value,
                reason,
            } => write!(
                formatter,
                "task '{task_id}' has invalid domain '{value}': {reason}"
            ),
            Self::InvalidRenderingMode {
                task_id,
                value,
                reason,
            } => write!(
                formatter,
                "task '{task_id}' has invalid rendering mode '{value}': {reason}"
            ),
        }
    }
}

impl std::error::Error for GuiCatalogTaskError {}

/// Required semantic version for generated catalogs; readers reject mismatches.
pub const GUI_CATALOG_SCHEMA_VERSION: u64 = 1;

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct GuiCatalog {
    pub schema_version: u64,
    #[serde(default)]
    pub tasks: Vec<GuiCatalogTask>,
    #[serde(default)]
    pub teleop_devices: Vec<String>,
    #[serde(default)]
    pub interfaces: Vec<String>,
    #[serde(default)]
    pub rendering_modes: Vec<String>,
    #[serde(default)]
    pub options: GuiCatalogOptions,
}

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct GuiCatalogOptions {
    #[serde(default)]
    pub tasks: Vec<String>,
    #[serde(default)]
    pub domains: Vec<String>,
    #[serde(default)]
    pub robots: Vec<String>,
    #[serde(default)]
    pub sceneries: Vec<String>,
    #[serde(default)]
    pub tools: Vec<String>,
    #[serde(default)]
    pub payloads: Vec<String>,
}

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct GuiCatalogTask {
    pub id: String,
    pub display_name: String,
    #[serde(default)]
    pub category: String,
    #[serde(default)]
    pub tagline: String,
    #[serde(default)]
    pub featured: bool,
    #[serde(default = "default_rank")]
    pub rank: u64,
    #[serde(default = "default_theme")]
    pub theme: String,
    #[serde(default)]
    pub defaults: GuiTaskDefaults,
    #[serde(default)]
    pub thumbnails: GuiTaskThumbnails,
    #[serde(default)]
    pub overrides: GuiTaskOverrides,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct GuiTaskDefaults {
    #[serde(
        default = "default_workflow",
        deserialize_with = "deserialize_workflow"
    )]
    pub workflow: String,
    #[serde(default = "default_domain", deserialize_with = "deserialize_domain")]
    pub domain: String,
    #[serde(default, rename = "stack")]
    pub stack_envs: bool,
    #[serde(default)]
    pub scenery: Option<String>,
    #[serde(default)]
    pub robot: Option<String>,
    #[serde(default)]
    pub mobile_base: Option<String>,
    #[serde(default)]
    pub manipulator: Option<String>,
    #[serde(default)]
    pub end_effector: Option<String>,
    #[serde(default)]
    pub payload: Option<String>,
    #[serde(default = "default_teleop_devices")]
    pub teleop_devices: Vec<String>,
    #[serde(default = "default_interfaces")]
    pub interfaces: Vec<String>,
    #[serde(
        default = "default_rendering_mode",
        deserialize_with = "deserialize_rendering_mode"
    )]
    pub rendering_mode: String,
}

impl Default for GuiTaskDefaults {
    fn default() -> Self {
        Self {
            workflow: default_workflow(),
            domain: default_domain(),
            stack_envs: false,
            scenery: None,
            robot: None,
            mobile_base: None,
            manipulator: None,
            end_effector: None,
            payload: None,
            teleop_devices: default_teleop_devices(),
            interfaces: default_interfaces(),
            rendering_mode: default_rendering_mode(),
        }
    }
}

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct GuiTaskThumbnails {
    #[serde(default)]
    pub task: String,
    #[serde(default)]
    pub robot: String,
    #[serde(default)]
    pub fallback: String,
    #[serde(default)]
    pub display: String,
    #[serde(default)]
    pub display_source: String,
    #[serde(default)]
    pub missing_thumbnail: bool,
}

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct GuiTaskOverrides {
    #[serde(default, rename = "enum")]
    pub enum_values: BTreeMap<String, Vec<String>>,
    #[serde(default)]
    pub keys: Vec<String>,
    #[serde(default)]
    pub action_modes: Vec<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiThumbnailSource {
    Task,
    Robot,
    Scenery,
    Domain,
    Fallback,
    Placeholder,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiResolvedThumbnail {
    pub path: PathBuf,
    pub source: GuiThumbnailSource,
}

impl GuiResolvedThumbnail {
    pub fn file_uri(&self) -> String {
        format!("file://{}", self.path.display())
    }
}

impl GuiThumbnailSource {
    fn from_cache_value(value: &str) -> Option<Self> {
        match value {
            "task" => Some(Self::Task),
            "robot" => Some(Self::Robot),
            "scenery" => Some(Self::Scenery),
            "domain" => Some(Self::Domain),
            "fallback" => Some(Self::Fallback),
            "placeholder" => Some(Self::Placeholder),
            _ => None,
        }
    }
}

const GUI_PLACEHOLDER_THUMBNAIL: &str = "docs/theme/favicon.png";

fn default_rank() -> u64 {
    1000
}

fn default_theme() -> String {
    String::from("dark")
}

fn default_workflow() -> String {
    String::from("teleop")
}

fn default_domain() -> String {
    String::from("MOON")
}

fn default_teleop_devices() -> Vec<String> {
    vec![String::from("keyboard"), String::from("spacemouse")]
}

fn default_interfaces() -> Vec<String> {
    vec![String::from("gui")]
}

fn default_rendering_mode() -> String {
    String::from("quality")
}

fn deserialize_workflow<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = <String as serde::Deserialize>::deserialize(deserializer)?;
    match value.parse::<Workflow>() {
        Ok(_) => Ok(value),
        Err(error) => Err(serde::de::Error::custom(format!(
            "invalid workflow '{value}': {error}"
        ))),
    }
}

fn deserialize_domain<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = <String as serde::Deserialize>::deserialize(deserializer)?;
    match value.parse::<Domain>() {
        Ok(_) => Ok(value),
        Err(error) => Err(serde::de::Error::custom(format!(
            "invalid domain '{value}': {error}"
        ))),
    }
}

fn deserialize_rendering_mode<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = <String as serde::Deserialize>::deserialize(deserializer)?;
    match value.parse::<RenderingMode>() {
        Ok(_) => Ok(value),
        Err(error) => Err(serde::de::Error::custom(format!(
            "invalid rendering mode '{value}': {error}"
        ))),
    }
}

impl GuiCatalog {
    pub fn recommended_tasks(&self, limit: usize) -> Vec<&GuiCatalogTask> {
        let mut tasks: Vec<&GuiCatalogTask> =
            self.tasks.iter().filter(|task| task.featured).collect();
        tasks.sort_by(|left, right| {
            left.rank
                .cmp(&right.rank)
                .then_with(|| left.display_name.cmp(&right.display_name))
        });
        tasks.truncate(limit);
        tasks
    }

    pub fn categories(&self) -> Vec<String> {
        self.tasks
            .iter()
            .filter_map(|task| {
                let category = task.category.trim();
                if category.is_empty() {
                    None
                } else {
                    Some(category.to_string())
                }
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    pub fn default_task(&self) -> Option<&GuiCatalogTask> {
        self.recommended_tasks(1)
            .into_iter()
            .next()
            .or_else(|| self.tasks.first())
    }

    pub fn validate_task_defaults(&self) -> Result<(), GuiCatalogTaskError> {
        for task in &self.tasks {
            task.try_to_task_config()?;
        }
        Ok(())
    }

    pub fn teleop_device_options(&self) -> Vec<TeleopDevice> {
        parse_options(&self.teleop_devices)
    }

    pub fn interface_options(&self) -> Vec<InterfaceType> {
        parse_options(&self.interfaces)
    }

    pub fn task_options(&self) -> Vec<String> {
        if self.options.tasks.is_empty() {
            unique_sorted(self.tasks.iter().map(|task| task.id.as_str()))
        } else {
            self.options.tasks.clone()
        }
    }

    pub fn domain_options(&self) -> Vec<String> {
        if !self.options.domains.is_empty() {
            return self.options.domains.clone();
        }

        let domains = unique_preserving_order(
            self.tasks
                .iter()
                .map(|task| task.defaults.domain.as_str())
                .filter(|domain| !domain.is_empty()),
        );
        if domains.is_empty() {
            vec![
                String::from("ASTEROID"),
                String::from("EARTH"),
                String::from("MARS"),
                String::from("MOON"),
                String::from("ORBIT"),
            ]
        } else {
            domains
        }
    }

    pub fn robot_options(&self) -> Vec<String> {
        if self.options.robots.is_empty() {
            self.derived_robot_options()
        } else {
            self.options.robots.clone()
        }
    }

    pub fn scenery_options(&self) -> Vec<String> {
        if self.options.sceneries.is_empty() {
            unique_sorted(
                self.tasks
                    .iter()
                    .filter_map(|task| task.defaults.scenery.as_deref()),
            )
        } else {
            self.options.sceneries.clone()
        }
    }

    pub fn tool_options(&self) -> Vec<String> {
        if self.options.tools.is_empty() {
            self.derived_tool_options()
        } else {
            self.options.tools.clone()
        }
    }

    pub fn payload_options(&self) -> Vec<String> {
        if self.options.payloads.is_empty() {
            self.derived_payload_options()
        } else {
            self.options.payloads.clone()
        }
    }

    fn derived_robot_options(&self) -> Vec<String> {
        let mut values = Vec::new();
        for task in &self.tasks {
            values.extend(task.robot_values());
        }
        unique_sorted(values)
    }

    fn derived_tool_options(&self) -> Vec<String> {
        let mut values = Vec::new();
        for task in &self.tasks {
            values.extend(optional_string_refs([task
                .defaults
                .end_effector
                .as_deref()]));
            values.extend(enum_values_for_keys(
                &task.overrides.enum_values,
                &[
                    "env.robot.end_effector",
                    "env.robot.manipulator.end_effector",
                ],
            ));
        }
        unique_sorted(values)
    }

    fn derived_payload_options(&self) -> Vec<String> {
        let mut values = Vec::new();
        for task in &self.tasks {
            values.extend(optional_string_refs([task.defaults.payload.as_deref()]));
            values.extend(enum_values_for_keys(
                &task.overrides.enum_values,
                &["env.robot.payload", "env.robot.mobile_base.payload"],
            ));
        }
        unique_sorted(values)
    }
}

impl GuiCatalogTask {
    pub fn robot_values(&self) -> Vec<&str> {
        let mut values: Vec<&str> = optional_string_refs([
            self.defaults.robot.as_deref(),
            self.defaults.mobile_base.as_deref(),
            self.defaults.manipulator.as_deref(),
        ])
        .collect();
        values.extend(enum_values_for_keys(
            &self.overrides.enum_values,
            &[
                "env.robot",
                "env.robot.mobile_base",
                "env.robot.manipulator",
            ],
        ));
        values
    }

    pub fn resolved_thumbnail(&self) -> GuiResolvedThumbnail {
        self.resolved_thumbnail_for_candidates()
            .unwrap_or_else(|| GuiResolvedThumbnail {
                path: repo_path(GUI_PLACEHOLDER_THUMBNAIL),
                source: GuiThumbnailSource::Placeholder,
            })
    }

    fn resolved_thumbnail_for_candidates(&self) -> Option<GuiResolvedThumbnail> {
        self.display_thumbnail_candidate()
            .into_iter()
            .chain(self.thumbnail_candidates())
            .filter(|(path, _source)| !path.is_empty())
            .map(|(path, source)| GuiResolvedThumbnail {
                path: repo_path(path),
                source,
            })
            .find(|thumbnail| thumbnail.path.exists())
    }

    fn display_thumbnail_candidate(&self) -> Option<(&str, GuiThumbnailSource)> {
        if self.thumbnails.display.is_empty() {
            None
        } else {
            Some((
                self.thumbnails.display.as_str(),
                GuiThumbnailSource::from_cache_value(&self.thumbnails.display_source)
                    .unwrap_or(GuiThumbnailSource::Fallback),
            ))
        }
    }

    fn thumbnail_candidates(&self) -> [(&str, GuiThumbnailSource); 3] {
        [
            (self.thumbnails.task.as_str(), GuiThumbnailSource::Task),
            (self.thumbnails.robot.as_str(), GuiThumbnailSource::Robot),
            (
                self.thumbnails.fallback.as_str(),
                GuiThumbnailSource::Fallback,
            ),
        ]
    }

    pub fn try_to_task_config(&self) -> Result<TaskConfig, GuiCatalogTaskError> {
        let workflow = self
            .defaults
            .workflow
            .parse::<Workflow>()
            .map_err(|error| {
                GuiCatalogTaskError::invalid(
                    GuiCatalogSemanticField::Workflow,
                    &self.id,
                    &self.defaults.workflow,
                    error,
                )
            })?;
        let domain = self.defaults.domain.parse::<Domain>().map_err(|error| {
            GuiCatalogTaskError::invalid(
                GuiCatalogSemanticField::Domain,
                &self.id,
                &self.defaults.domain,
                error,
            )
        })?;
        let rendering_mode = self
            .defaults
            .rendering_mode
            .parse::<RenderingMode>()
            .map_err(|error| {
                GuiCatalogTaskError::invalid(
                    GuiCatalogSemanticField::RenderingMode,
                    &self.id,
                    &self.defaults.rendering_mode,
                    error,
                )
            })?;

        let mut cfg = TaskConfig::builder().build();
        cfg.task = self.id.clone();
        cfg.workflow = workflow;
        cfg.domain = domain;
        cfg.stack_envs = self.defaults.stack_envs;
        cfg.scenery = self.defaults.scenery.clone();
        if let Some(robot) = &self.defaults.robot {
            cfg.robot = robot.clone();
        }
        cfg.mobile_base = self.defaults.mobile_base.clone();
        cfg.manipulator = self.defaults.manipulator.clone();
        cfg.end_effector = self.defaults.end_effector.clone();
        cfg.payload = self.defaults.payload.clone();
        cfg.rendering_mode = rendering_mode;

        let teleop_devices = parse_options(&self.defaults.teleop_devices);
        if !teleop_devices.is_empty() {
            cfg.teleop_devices = teleop_devices;
        }

        let interfaces = parse_options(&self.defaults.interfaces);
        if !interfaces.is_empty() {
            cfg.interfaces = interfaces;
        }

        Ok(cfg)
    }
}

fn repo_path(relative_path: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(relative_path)
}

pub(crate) fn parse_options<T>(values: &[String]) -> Vec<T>
where
    T: std::str::FromStr,
{
    values
        .iter()
        .filter_map(|value| value.parse::<T>().ok())
        .collect()
}

fn optional_string_refs<const N: usize>(values: [Option<&str>; N]) -> impl Iterator<Item = &str> {
    values
        .into_iter()
        .flatten()
        .filter(|value| !value.is_empty())
}

fn enum_values_for_keys<'a>(
    enum_values: &'a BTreeMap<String, Vec<String>>,
    keys: &[&str],
) -> Vec<&'a str> {
    keys.iter()
        .filter_map(|key| enum_values.get(*key))
        .flat_map(|values| values.iter().map(String::as_str))
        .filter(|value| !value.is_empty())
        .collect()
}

fn unique_sorted<'a>(values: impl IntoIterator<Item = &'a str>) -> Vec<String> {
    values
        .into_iter()
        .map(str::to_string)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn unique_preserving_order<'a>(values: impl IntoIterator<Item = &'a str>) -> Vec<String> {
    let mut seen = BTreeSet::new();
    let mut result = Vec::new();
    for value in values {
        if seen.insert(value.to_string()) {
            result.push(value.to_string());
        }
    }
    result
}
