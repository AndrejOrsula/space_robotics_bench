#[derive(
    Copy,
    Clone,
    Debug,
    Eq,
    PartialEq,
    Hash,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
)]
pub enum Page {
    QuickStart,
    Interface,
}

impl Default for Page {
    fn default() -> Self {
        Self::QuickStart
    }
}

impl Page {
    pub fn title(&self) -> &str {
        match self {
            Self::QuickStart => "Quickstart",
            Self::Interface => "Interface",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            Self::QuickStart => "Select your experience",
            Self::Interface => "Complete the task",
        }
    }
}
