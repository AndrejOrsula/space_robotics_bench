use crate::gui_catalog::GuiCatalogTask;
use crate::gui_selection::GuiSelection;

const TASK_DEFAULT_LABEL: &str = "Task default";

/// Hydra keys owned by dedicated controls; duplicates could override the displayed selection.
const DEDICATED_CONTROL_KEYS: [&str; 14] = [
    "env.domain",
    "env.gravity",
    "env.seed",
    "env.num_envs",
    "env.stack",
    "env.scenery",
    "env.sim.render.rendering_mode",
    "env.robot",
    "env.robot.mobile_base",
    "env.robot.manipulator",
    "env.robot.end_effector",
    "env.robot.payload",
    "env.robot.mobile_base.payload",
    "env.robot.manipulator.end_effector",
];

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiTaskOverrideChoice {
    key: String,
    label: String,
    values: Vec<String>,
    selected_value: Option<String>,
}

impl GuiTaskOverrideChoice {
    pub fn from_task(task: &GuiCatalogTask, selection: &GuiSelection) -> Vec<Self> {
        task.overrides
            .enum_values
            .iter()
            .filter(|(key, values)| {
                !values.is_empty() && !DEDICATED_CONTROL_KEYS.contains(&key.as_str())
            })
            .map(|(key, values)| Self {
                key: key.clone(),
                label: humanize_config_key(key),
                values: values.clone(),
                selected_value: selection.hydra_override_value(key),
            })
            .collect()
    }

    #[must_use]
    pub fn label(&self) -> &str {
        &self.label
    }

    #[must_use]
    pub fn key(&self) -> &str {
        &self.key
    }

    #[must_use]
    pub fn widget_id(&self) -> String {
        format!("task_override_{}", self.key)
    }

    #[must_use]
    pub fn selected_text(&self) -> &str {
        self.selected_value.as_deref().unwrap_or(TASK_DEFAULT_LABEL)
    }

    #[must_use]
    pub fn default_label(&self) -> &'static str {
        TASK_DEFAULT_LABEL
    }

    #[must_use]
    pub fn default_selected(&self) -> bool {
        self.selected_value.is_none()
    }

    #[must_use]
    pub fn values(&self) -> &[String] {
        &self.values
    }

    #[must_use]
    pub fn value_selected(&self, value: &str) -> bool {
        self.selected_value.as_deref() == Some(value)
    }

    pub fn apply_default(&self, selection: &mut GuiSelection) {
        selection.remove_hydra_override(&self.key);
    }

    pub fn apply_value(&self, selection: &mut GuiSelection, value: &str) {
        selection.set_hydra_override(&self.key, value);
    }
}

/// Label an override by its last two key segments; the full key is the hover hint.
fn humanize_config_key(key: &str) -> String {
    let segments = key.split('.').collect::<Vec<_>>();
    segments[segments.len().saturating_sub(2)..]
        .iter()
        .copied()
        .map(humanize_display_value)
        .collect::<Vec<_>>()
        .join(" / ")
}

pub(crate) fn humanize_display_value(value: &str) -> String {
    let cleaned = value.trim().replace(['_', '-'], " ");
    if cleaned.is_empty() {
        return String::from("None");
    }

    cleaned
        .split_whitespace()
        .map(|word| {
            if word.chars().all(|ch| !ch.is_ascii_lowercase()) {
                return word.to_string();
            }

            let mut chars = word.chars();
            match chars.next() {
                Some(first) => format!("{}{}", first.to_ascii_uppercase(), chars.as_str()),
                None => String::new(),
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}
