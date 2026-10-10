use crate::env_cfg::{Domain, InterfaceType, RenderingMode, TaskConfig, TeleopDevice, Workflow};
use crate::gui_catalog::{parse_options, GuiCatalog, GuiCatalogTask, GuiCatalogTaskError};
use crate::gui_launch::GuiLaunchRequest;
use std::fmt;
use strum::IntoEnumIterator;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiTaskSelectionOutcome {
    CatalogApplied,
    ManualTaskIdApplied,
    NoChange,
}

impl GuiTaskSelectionOutcome {
    pub const fn is_applied(self) -> bool {
        matches!(self, Self::CatalogApplied | Self::ManualTaskIdApplied)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GuiSelectionInitializationError<E> {
    CatalogLoad(E),
    InvalidCatalogDefaults(GuiCatalogTaskError),
}

impl<E: fmt::Display> fmt::Display for GuiSelectionInitializationError<E> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CatalogLoad(error) => write!(formatter, "GUI catalog could not load: {error}"),
            Self::InvalidCatalogDefaults(error) => {
                write!(formatter, "GUI catalog defaults are invalid: {error}")
            }
        }
    }
}

impl<E> std::error::Error for GuiSelectionInitializationError<E>
where
    E: std::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CatalogLoad(error) => Some(error),
            Self::InvalidCatalogDefaults(error) => Some(error),
        }
    }
}

#[derive(Clone, Debug)]
pub struct GuiSelection {
    task_config: TaskConfig,
    pub options: GuiSelectionOptions,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GuiSelectionOptions {
    pub tasks: Vec<String>,
    pub robots: Vec<String>,
    pub sceneries: Vec<String>,
    pub tools: Vec<String>,
    pub payloads: Vec<String>,
    pub domains: Vec<String>,
    pub rendering_modes: Vec<RenderingMode>,
    pub teleop_devices: Vec<TeleopDevice>,
    pub interfaces: Vec<InterfaceType>,
}

impl Default for GuiSelection {
    fn default() -> Self {
        Self {
            task_config: TaskConfig::builder().build(),
            options: GuiSelectionOptions::fallback(),
        }
    }
}

impl GuiSelection {
    pub fn task_config(&self) -> &TaskConfig {
        &self.task_config
    }

    pub fn launch_request(&self) -> GuiLaunchRequest {
        GuiLaunchRequest::from_command_contract(self.task_config.command_contract())
    }

    pub fn from_catalog_result<E: Clone>(
        gui_catalog: &Result<GuiCatalog, E>,
    ) -> Result<Self, GuiSelectionInitializationError<E>> {
        let catalog = gui_catalog
            .as_ref()
            .map_err(|error| GuiSelectionInitializationError::CatalogLoad(error.clone()))?;
        Self::try_from_catalog(catalog)
            .map_err(GuiSelectionInitializationError::InvalidCatalogDefaults)
    }

    pub fn try_from_catalog(catalog: &GuiCatalog) -> Result<Self, GuiCatalogTaskError> {
        catalog.validate_task_defaults()?;
        let task_config = match catalog.default_task() {
            Some(task) => task.try_to_task_config()?,
            None => TaskConfig::builder().build(),
        };
        Ok(Self {
            task_config,
            options: GuiSelectionOptions::from_catalog(catalog),
        })
    }

    pub(crate) fn neutral_initialization_fallback() -> Self {
        Self::default()
    }

    pub fn select_task_from_catalog(
        &mut self,
        catalog_task: &GuiCatalogTask,
    ) -> Result<GuiTaskSelectionOutcome, GuiCatalogTaskError> {
        let config = catalog_task.try_to_task_config()?;
        self.task_config = config;
        Ok(GuiTaskSelectionOutcome::CatalogApplied)
    }

    pub fn select_task(
        &mut self,
        catalog_task: Option<&GuiCatalogTask>,
        task_id: &str,
    ) -> Result<GuiTaskSelectionOutcome, GuiCatalogTaskError> {
        if let Some(task) = catalog_task {
            self.select_task_from_catalog(task)
        } else if self.task_config.task == task_id
            || !self.options.tasks.iter().any(|task| task == task_id)
        {
            Ok(GuiTaskSelectionOutcome::NoChange)
        } else {
            self.task_config.task = task_id.to_string();
            Ok(GuiTaskSelectionOutcome::ManualTaskIdApplied)
        }
    }

    pub fn reset_task_config(&mut self) {
        self.task_config = TaskConfig::builder().build();
    }

    pub fn set_workflow(&mut self, workflow: Workflow) {
        self.task_config.workflow = workflow;
    }

    pub fn set_domain(&mut self, domain: Domain) -> f64 {
        self.task_config.domain = domain;
        domain.gravity_magnitude()
    }

    pub fn set_robot(&mut self, robot: impl Into<String>) {
        self.task_config.robot = robot.into();
    }

    pub fn clear_tool_payload(&mut self) {
        self.task_config.end_effector = None;
        self.task_config.payload = None;
    }

    pub fn select_tool(&mut self, tool: impl Into<String>) {
        self.task_config.end_effector = Some(tool.into());
        self.task_config.payload = None;
    }

    pub fn select_payload(&mut self, payload: impl Into<String>) {
        self.task_config.payload = Some(payload.into());
        self.task_config.end_effector = None;
    }

    pub fn set_scenery(&mut self, scenery: Option<String>) {
        self.task_config.scenery = scenery;
    }

    pub fn set_rendering_mode(&mut self, rendering_mode: RenderingMode) {
        self.task_config.rendering_mode = rendering_mode;
    }

    pub fn set_seed(&mut self, seed: u64) {
        self.task_config.seed = seed;
    }

    pub fn set_num_envs(&mut self, num_envs: u64) {
        self.task_config.num_envs = num_envs.max(1);
    }

    pub fn set_hide_ui(&mut self, hide_ui: bool) {
        self.task_config.hide_ui = hide_ui;
    }

    pub fn hydra_override_value(&self, key: &str) -> Option<String> {
        let prefix = format!("{key}=");
        self.task_config
            .extras
            .iter()
            .find_map(|extra| extra.strip_prefix(&prefix).map(str::to_string))
    }

    pub fn set_hydra_override(&mut self, key: &str, value: &str) {
        self.remove_hydra_override(key);
        self.task_config.extras.push(format!("{key}={value}"));
    }

    pub fn remove_hydra_override(&mut self, key: &str) {
        let prefix = format!("{key}=");
        self.task_config
            .extras
            .retain(|extra| !extra.starts_with(&prefix));
    }

    pub fn replace_raw_overrides(&mut self, raw_overrides: &str) {
        self.task_config.extras = raw_overrides
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty())
            .map(str::to_string)
            .collect();
    }

    pub fn teleop_device_enabled(&self, device: TeleopDevice) -> bool {
        self.task_config.teleop_devices.contains(&device)
    }

    pub fn interface_enabled(&self, interface: InterfaceType) -> bool {
        self.task_config.interfaces.contains(&interface)
    }

    pub fn set_teleop_device_enabled(&mut self, device: TeleopDevice, enabled: bool) {
        // Keep one device: the CLI reads an omitted --teleop_device as "use the
        // task defaults", so an empty selection would launch unselected devices.
        if !enabled && self.task_config.teleop_devices == [device] {
            return;
        }
        set_checked(&mut self.task_config.teleop_devices, device, enabled);
    }

    pub fn set_interface_enabled(&mut self, interface: InterfaceType, enabled: bool) {
        set_checked(&mut self.task_config.interfaces, interface, enabled);
    }
}

impl GuiSelectionOptions {
    fn fallback() -> Self {
        Self {
            domains: Domain::iter().map(|domain| domain.to_string()).collect(),
            rendering_modes: RenderingMode::iter().collect(),
            teleop_devices: vec![TeleopDevice::Keyboard, TeleopDevice::Spacemouse],
            interfaces: vec![InterfaceType::Gui],
            ..Self::default()
        }
    }

    pub(crate) fn from_catalog(catalog: &GuiCatalog) -> Self {
        let rendering_modes = parse_options(&catalog.rendering_modes);
        let teleop_devices = catalog.teleop_device_options();
        let interfaces = catalog.interface_options();

        Self {
            tasks: catalog.task_options(),
            robots: catalog.robot_options(),
            sceneries: catalog.scenery_options(),
            tools: catalog.tool_options(),
            payloads: catalog.payload_options(),
            domains: catalog.domain_options(),
            rendering_modes: if rendering_modes.is_empty() {
                RenderingMode::iter().collect()
            } else {
                rendering_modes
            },
            teleop_devices: if teleop_devices.is_empty() {
                vec![TeleopDevice::Keyboard, TeleopDevice::Spacemouse]
            } else {
                teleop_devices
            },
            interfaces: if interfaces.is_empty() {
                vec![InterfaceType::Gui]
            } else {
                interfaces
            },
        }
    }
}

fn set_checked<T>(values: &mut Vec<T>, value: T, enabled: bool)
where
    T: Copy + Eq,
{
    if enabled {
        if !values.contains(&value) {
            values.push(value);
        }
    } else {
        values.retain(|existing| *existing != value);
    }
}
