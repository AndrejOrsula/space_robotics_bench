use crate::command_contract::CommandContract;

#[derive(
    Clone,
    Debug,
    Eq,
    PartialEq,
    Hash,
    serde::Deserialize,
    serde::Serialize,
    typed_builder::TypedBuilder,
)]
pub struct TaskConfig {
    #[builder(default = String::from("sample_collection"))]
    pub task: String,
    #[builder(default = Workflow::Teleop)]
    pub workflow: Workflow,
    #[builder(default = Domain::Moon)]
    pub domain: Domain,
    #[builder(default = None)]
    pub scenery: Option<String>,
    #[builder(default = String::from("franka"))]
    pub robot: String,
    #[builder(default = None)]
    pub mobile_base: Option<String>,
    #[builder(default = None)]
    pub manipulator: Option<String>,
    #[builder(default = None)]
    pub end_effector: Option<String>,
    #[builder(default = None)]
    pub payload: Option<String>,
    #[builder(default = 0)]
    pub seed: u64,
    #[builder(default = 1)]
    pub num_envs: u64,
    #[builder(default = false)]
    pub stack_envs: bool,
    #[builder(default = true)]
    pub hide_ui: bool,
    #[builder(default = RenderingMode::Quality)]
    pub rendering_mode: RenderingMode,
    #[builder(default = vec![TeleopDevice::Keyboard, TeleopDevice::Spacemouse])]
    pub teleop_devices: Vec<TeleopDevice>,
    #[builder(default = vec![InterfaceType::Gui])]
    pub interfaces: Vec<InterfaceType>,
    #[builder(default = vec![])]
    pub extras: Vec<String>,
}

impl TaskConfig {
    pub fn command_contract(&self) -> CommandContract {
        let mut argparse_args = vec![
            String::from("agent"),
            self.workflow.to_string().to_lowercase(),
            String::from("--env"),
            self.task.clone(),
        ];

        if self.hide_ui {
            argparse_args.push(String::from("--hide_ui"));
        }

        argparse_args.push(String::from("--rendering_mode"));
        argparse_args.push(self.rendering_mode.to_string().to_lowercase());

        if !self.interfaces.is_empty() {
            argparse_args.push(String::from("--interface"));
            argparse_args.extend(
                self.interfaces
                    .iter()
                    .map(|interface| interface.to_string().to_lowercase()),
            );
        }

        if self.workflow == Workflow::Teleop && !self.teleop_devices.is_empty() {
            argparse_args.push(String::from("--teleop_device"));
            argparse_args.extend(
                self.teleop_devices
                    .iter()
                    .map(|device| device.to_string().to_lowercase()),
            );
        }

        let mut hydra_overrides = vec![format!(
            "env.domain={}",
            self.domain.to_string().to_uppercase()
        )];
        hydra_overrides.push(format!("env.seed={}", self.seed));
        hydra_overrides.push(format!("env.num_envs={}", self.num_envs.max(1)));
        hydra_overrides.push(format!(
            "env.stack={}",
            self.stack_envs.to_string().to_lowercase()
        ));

        if let Some(scenery) = &self.scenery {
            if Self::is_selected(scenery) {
                hydra_overrides.push(format!("env.scenery={scenery}"));
            }
        }

        hydra_overrides.extend(self.robot_overrides());
        hydra_overrides.extend(self.extras.iter().cloned());

        CommandContract::new(argparse_args, hydra_overrides)
    }

    #[must_use]
    pub fn uses_composite_robot(&self) -> bool {
        self.composite_mobile_base().is_some() || self.composite_manipulator().is_some()
    }

    fn composite_mobile_base(&self) -> Option<&str> {
        self.mobile_base
            .as_deref()
            .filter(|value| Self::is_selected(value))
    }

    fn composite_manipulator(&self) -> Option<&str> {
        self.manipulator
            .as_deref()
            .filter(|value| Self::is_selected(value))
    }

    fn robot_overrides(&self) -> Vec<String> {
        let mobile_base = self.composite_mobile_base();
        let manipulator = self.composite_manipulator();

        if mobile_base.is_some() || manipulator.is_some() {
            let mut overrides = Vec::new();
            // Payloads use the base slot; the tool slot accepts end effectors.
            let payload = self
                .payload
                .as_deref()
                .filter(|value| Self::is_selected(value));

            match (mobile_base, payload) {
                (Some(mobile_base), Some(payload)) => {
                    overrides.push(format!("env.robot.mobile_base={mobile_base}+{payload}"));
                }
                (Some(mobile_base), None) => {
                    overrides.push(format!("env.robot.mobile_base={mobile_base}"));
                }
                (None, Some(payload)) => {
                    overrides.push(format!("env.robot.mobile_base.payload={payload}"));
                }
                (None, None) => {}
            }

            if let Some(manipulator) = manipulator {
                let mut manipulator_arg = manipulator.to_string();
                let tool = self
                    .end_effector
                    .as_deref()
                    .filter(|value| Self::is_selected(value));

                if let Some(tool) = tool {
                    manipulator_arg.push('+');
                    manipulator_arg.push_str(tool);
                }

                overrides.push(format!("env.robot.manipulator={manipulator_arg}"));
            }

            return overrides;
        }

        let robot_arg = self.build_robot_arg();
        if robot_arg.is_empty() {
            Vec::new()
        } else {
            vec![format!("env.robot={robot_arg}")]
        }
    }

    fn is_selected(value: &str) -> bool {
        !value.is_empty() && value != "none" && value != "default"
    }

    fn build_robot_arg(&self) -> String {
        let mut final_robot_arg = String::new();
        let has_base = !self.robot.is_empty() && self.robot != "default";

        let ee_opt = self.end_effector.as_deref();
        let payload_opt = self.payload.as_deref();

        let has_ee = ee_opt.is_some_and(Self::is_selected);
        let has_payload = payload_opt.is_some_and(Self::is_selected);

        if has_base {
            final_robot_arg.push_str(&self.robot);

            if has_ee {
                final_robot_arg.push('+');
                if let Some(end_effector) = ee_opt {
                    final_robot_arg.push_str(end_effector);
                }
            } else if has_payload {
                final_robot_arg.push('+');
                if let Some(payload) = payload_opt {
                    final_robot_arg.push_str(payload);
                }
            }
        } else if has_ee {
            final_robot_arg.push('+');
            if let Some(end_effector) = ee_opt {
                final_robot_arg.push_str(end_effector);
            }
        } else if has_payload {
            final_robot_arg.push('+');
            if let Some(payload) = payload_opt {
                final_robot_arg.push_str(payload);
            }
        }

        if final_robot_arg.is_empty() || final_robot_arg == "+" {
            String::new()
        } else {
            final_robot_arg
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum Domain {
    Asteroid,
    Earth,
    Mars,
    Moon,
    Orbit,
}
impl std::str::FromStr for Domain {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "asteroid" => Ok(Self::Asteroid),
            "earth" => Ok(Self::Earth),
            "mars" => Ok(Self::Mars),
            "moon" => Ok(Self::Moon),
            "orbit" => Ok(Self::Orbit),
            _ => Err(format!(
                "Invalid domain '{s}'; choose asteroid, earth, mars, moon, or orbit"
            )),
        }
    }
}
impl Domain {
    #[must_use]
    pub fn gravity_magnitude(self) -> f64 {
        match self {
            Self::Asteroid => 0.14219,
            Self::Earth => 9.80665,
            Self::Mars => 3.72076,
            Self::Moon => 1.62496,
            Self::Orbit => 0.0,
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum Workflow {
    Rand,
    Teleop,
    Zero,
}
impl std::str::FromStr for Workflow {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "rand" => Ok(Self::Rand),
            "teleop" => Ok(Self::Teleop),
            "zero" => Ok(Self::Zero),
            _ => Err(format!(
                "Invalid workflow '{s}'; choose rand, teleop, or zero"
            )),
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum RenderingMode {
    Performance,
    Balanced,
    Quality,
    Xr,
}
impl std::str::FromStr for RenderingMode {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "performance" => Ok(Self::Performance),
            "balanced" => Ok(Self::Balanced),
            "quality" => Ok(Self::Quality),
            "xr" => Ok(Self::Xr),
            _ => Err(format!(
                "Invalid rendering mode '{s}'; choose performance, balanced, quality, or xr"
            )),
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum TeleopDevice {
    Keyboard,
    Ros,
    Gamepad,
    Spacemouse,
    Haptic,
}
impl std::str::FromStr for TeleopDevice {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "keyboard" => Ok(TeleopDevice::Keyboard),
            "ros" => Ok(TeleopDevice::Ros),
            "gamepad" => Ok(TeleopDevice::Gamepad),
            "spacemouse" => Ok(TeleopDevice::Spacemouse),
            "haptic" => Ok(TeleopDevice::Haptic),
            _ => Err(format!(
                "Invalid teleop device '{s}'; choose keyboard, ros, gamepad, spacemouse, or haptic"
            )),
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum InterfaceType {
    Gui,
    Ros,
}
impl std::str::FromStr for InterfaceType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "gui" => Ok(InterfaceType::Gui),
            "ros" => Ok(InterfaceType::Ros),
            _ => Err(format!("Invalid interface '{s}'; choose gui or ros")),
        }
    }
}
