#[derive(
    Copy,
    Clone,
    Debug,
    Default,
    Eq,
    PartialEq,
    Hash,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
)]
pub enum Page {
    #[default]
    QuickStart,
    Interface,
    Metrics,
}

impl Page {
    pub fn title(&self) -> &str {
        match self {
            Self::QuickStart => "Tasks",
            Self::Interface => "Session",
            Self::Metrics => "Metrics",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            Self::QuickStart => "Browse and select a task",
            Self::Interface => "Configure and run your session",
            Self::Metrics => "Track reward over time",
        }
    }
}
