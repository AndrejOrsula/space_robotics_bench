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
    #[builder(default = Domain::Moon)]
    pub domain: Domain,
    #[builder(default = None)]
    pub scenery: Option<String>,
    #[builder(default = String::from("franka"))]
    pub robot: String,
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
    #[builder(default = vec![TeleopDevice::Keyboard, TeleopDevice::Spacemouse, TeleopDevice::Ros, TeleopDevice::Haptic])]
    pub teleop_devices: Vec<TeleopDevice>,
    #[builder(default = vec![InterfaceType::Gui])]
    pub interfaces: Vec<InterfaceType>,
    #[builder(default = vec![])]
    pub extras: Vec<String>,
}

impl TaskConfig {
    pub fn set_exec_env(&self, mut exec: subprocess::Exec) -> subprocess::Exec {
        exec = exec.arg("agent");

        let is_direct_teleop_supported = !self.task.contains("locomotion");
        if is_direct_teleop_supported {
            exec = exec.arg("teleop");
            exec = exec.arg("--teleop_device");
            exec = exec.args(
                &self
                    .teleop_devices
                    .iter()
                    .map(|device| device.to_string().to_lowercase())
                    .collect::<Vec<String>>(),
            );
        } else {
            // TODO[gui]: Support teleop via policy
            exec = exec.arg("rand");
        }
        if self.hide_ui {
            exec = exec.arg("--hide_ui");
        }
        if !self.interfaces.is_empty() {
            exec = exec.arg("--interface");
            exec = exec.args(
                &self
                    .interfaces
                    .iter()
                    .map(|interface| interface.to_string().to_lowercase())
                    .collect::<Vec<String>>(),
            );
        }

        // --- Core Environment Configuration ---
        exec = exec.args(&["--task", &self.task]);
        exec = exec.arg(format!(
            "env.domain={}",
            self.domain.to_string().to_lowercase()
        ));
        exec = exec.arg(format!("env.seed={}", self.seed));
        exec = exec.arg(format!("env.num_envs={}", self.num_envs.max(1)));
        exec = exec.arg(format!(
            "env.stack={}",
            self.stack_envs.to_string().to_lowercase()
        ));

        // --- Extra Environment Configuration ---
        if !self.extras.is_empty() {
            exec = exec.args(&self.extras);
        }

        // --- Scenery Configuration ---
        if let Some(scenery) = &self.scenery {
            exec = exec.arg(format!("env.scenery={scenery}"));
        }

        // --- Robot Configuration ---
        let robot_arg = self.build_robot_arg();
        exec = exec.arg(format!("env.robot={robot_arg}"));

        // --- Pass through other necessary env vars ---
        exec = exec.env(
            "DISPLAY",
            std::env::var("SRB_DISPLAY")
                .unwrap_or(std::env::var("DISPLAY").unwrap_or(":0".to_string())),
        );
        exec = exec.env(
            "ROS_DOMAIN_ID",
            std::env::var("ROS_DOMAIN_ID").unwrap_or("0".to_string()),
        );
        exec = exec.env(
            "RMW_IMPLEMENTATION",
            std::env::var("RMW_IMPLEMENTATION").unwrap_or("rmw_cyclonedds_cpp".to_string()),
        );

        exec
    }

    /// TODO[gui]: Review robot arg building logic
    fn build_robot_arg(&self) -> String {
        let mut final_robot_arg = String::new();
        let has_base = !self.robot.is_empty() && self.robot != "default";

        let ee_opt = self.end_effector.as_deref();
        let payload_opt = self.payload.as_deref();

        let has_ee = ee_opt.is_some_and(|s| !s.is_empty() && s != "none" && s != "default");
        let has_payload =
            payload_opt.is_some_and(|s| !s.is_empty() && s != "none" && s != "default");
        let keep_default_ee = ee_opt == Some("default");
        let keep_default_payload = payload_opt == Some("default");
        let no_ee = ee_opt == Some("none");
        let no_payload = payload_opt == Some("none");

        if has_base {
            final_robot_arg.push_str(&self.robot);

            if has_ee || keep_default_ee {
                final_robot_arg.push('+');
                if has_ee {
                    final_robot_arg.push_str(self.end_effector.as_ref().unwrap());
                }
            } else if has_payload || keep_default_payload {
                final_robot_arg.push('+');
                if has_payload {
                    final_robot_arg.push_str(self.payload.as_ref().unwrap());
                }
            } else if no_ee || no_payload {
            }
        } else if has_ee {
            final_robot_arg.push('+');
            final_robot_arg.push_str(self.end_effector.as_ref().unwrap());
        } else if has_payload {
            final_robot_arg.push('+');
            final_robot_arg.push_str(self.payload.as_ref().unwrap());
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
            _ => Err(format!("Invalid Domain: {s}")),
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
pub enum RobotType {
    Manipulator,
    MobileManipulator,
    MobileRobot,
}
impl std::str::FromStr for RobotType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "manipulator" => Ok(RobotType::Manipulator),
            "mobile_manipulator" => Ok(RobotType::MobileManipulator),
            "mobile_robot" => Ok(RobotType::MobileRobot),
            _ => Err(format!("Invalid RobotType string: {s}")),
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
pub enum SceneryType {
    Extravehicular,
    Structure,
    Subterrane,
    Terrain,
}
impl std::str::FromStr for SceneryType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "extravehicular" => Ok(SceneryType::Extravehicular),
            "structure" => Ok(SceneryType::Structure),
            "subterrane" => Ok(SceneryType::Subterrane),
            "terrain" => Ok(SceneryType::Terrain),
            _ => Err(format!("Invalid SceneryType string: {s}")),
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
pub enum ObjectType {
    Common,
    Light,
    Payload,
    Pedestal,
    Tool,
}
impl std::str::FromStr for ObjectType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "common" => Ok(ObjectType::Common),
            "light" => Ok(ObjectType::Light),
            "payload" => Ok(ObjectType::Payload),
            "pedestal" => Ok(ObjectType::Pedestal),
            "tool" => Ok(ObjectType::Tool),
            _ => Err(format!("Invalid ObjectType: {s}")),
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
            _ => Err(format!("Invalid TeleopDevice: {s}")),
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
            _ => Err(format!("Invalid InterfaceType: {s}")),
        }
    }
}
