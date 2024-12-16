use serde::{Deserialize, Serialize};
use space_robotics_bench::envs::EnvironmentConfig;

const ENVIRON_PREFIX: &str = "SRB_";

#[derive(
    Deserialize,
    Serialize,
    Debug,
    display_json::DisplayAsJson,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    Default,
)]
#[serde(rename_all = "snake_case")]
pub enum Task {
    #[default]
    SampleCollection,
    DebrisCapture,
    PegInHole,
    SolarPanelAssembly,
    Perseverance,
    Ingenuity,
    Gateway,
}

#[derive(
    Deserialize,
    Serialize,
    display_json::DebugAsJson,
    display_json::DisplayAsJson,
    Clone,
    Copy,
    PartialEq,
)]
pub struct TaskConfig {
    pub task: Task,
    pub num_envs: u64,
    pub env_cfg: EnvironmentConfig,
    pub enable_ui: bool,
}

impl Default for TaskConfig {
    fn default() -> Self {
        Self {
            task: Task::SampleCollection,
            num_envs: 1,
            env_cfg: EnvironmentConfig::default(),
            enable_ui: false,
        }
    }
}

impl TaskConfig {
    pub fn set_exec_env(mut self, mut exec: subprocess::Exec) -> subprocess::Exec {
        // Arguments
        if self.enable_ui {
            exec = exec.args(&[
                "--gui_integration",
                "--ros2_integration",
                "--task",
                &format!("{}_visual", self.task.to_string().trim_matches('"')),
            ]);
        } else {
            exec = exec.args(&[
                "--gui_integration",
                "--task",
                self.task.to_string().trim_matches('"'),
            ]);
        }
        self.num_envs = self.num_envs.max(1);
        exec = exec.args(&["--num_envs", self.num_envs.to_string().as_str()]);
        exec = exec.args(&["--teleop_device", "keyboard", "spacemouse", "touch", "ros2"]);
        if !self.enable_ui {
            exec = exec.arg("--disable_ui");
        }

        // Environment variables - Environment
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "SEED"),
            std::env::var(const_format::concatcp!(ENVIRON_PREFIX, "SEED"))
                .unwrap_or(self.env_cfg.seed.to_string().trim_matches('"').to_owned()),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "SCENARIO"),
            std::env::var(const_format::concatcp!(ENVIRON_PREFIX, "SCENARIO")).unwrap_or(
                self.env_cfg
                    .scenario
                    .to_string()
                    .trim_matches('"')
                    .to_owned(),
            ),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "DETAIL"),
            std::env::var(const_format::concatcp!(ENVIRON_PREFIX, "DETAIL"))
                .unwrap_or(self.env_cfg.detail.to_string().trim_matches('"').to_owned()),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "ASSETS_ROBOT_VARIANT"),
            std::env::var(const_format::concatcp!(
                ENVIRON_PREFIX,
                "ASSETS_ROBOT_VARIANT"
            ))
            .unwrap_or(
                self.env_cfg
                    .assets
                    .robot
                    .variant
                    .to_string()
                    .trim_matches('"')
                    .to_owned(),
            ),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "ASSETS_OBJECT_VARIANT"),
            std::env::var(const_format::concatcp!(
                ENVIRON_PREFIX,
                "ASSETS_OBJECT_VARIANT"
            ))
            .unwrap_or(
                self.env_cfg
                    .assets
                    .object
                    .variant
                    .to_string()
                    .trim_matches('"')
                    .to_owned(),
            ),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "ASSETS_TERRAIN_VARIANT"),
            std::env::var(const_format::concatcp!(
                ENVIRON_PREFIX,
                "ASSETS_TERRAIN_VARIANT"
            ))
            .unwrap_or(
                self.env_cfg
                    .assets
                    .terrain
                    .variant
                    .to_string()
                    .trim_matches('"')
                    .to_owned(),
            ),
        );
        exec = exec.env(
            const_format::concatcp!(ENVIRON_PREFIX, "ASSETS_VEHICLE_VARIANT"),
            std::env::var(const_format::concatcp!(
                ENVIRON_PREFIX,
                "ASSETS_VEHICLE_VARIANT"
            ))
            .unwrap_or(
                self.env_cfg
                    .assets
                    .vehicle
                    .variant
                    .to_string()
                    .trim_matches('"')
                    .to_owned(),
            ),
        );

        // Environment variables - GUI
        exec = exec.env(
            "DISPLAY",
            std::env::var(const_format::concatcp!(ENVIRON_PREFIX, "DISPLAY"))
                .unwrap_or(":0".to_string()),
        );

        // Environment variables - ROS
        exec = exec.env(
            "ROS_DOMAIN_ID",
            std::env::var("ROS_DOMAIN_ID").unwrap_or("0".to_string()),
        );
        exec = exec.env(
            "RMW_IMPLEMENTATION",
            std::env::var(const_format::concatcp!(
                ENVIRON_PREFIX,
                "RMW_IMPLEMENTATION"
            ))
            .unwrap_or("rmw_cyclonedds_cpp".to_string()),
        );

        exec
    }
}
