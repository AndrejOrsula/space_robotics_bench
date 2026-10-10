use crate::gui_runtime::{GuiRuntimeSignalError, GuiRuntimeSignals};
use crate::gui_runtime_params::RuntimeParams;
use r2r::std_msgs::msg::{Bool as BoolMsg, Empty as EmptyMsg, Float64 as Float64Msg};
use r2r::QosProfile;
use std::fmt;
use tracing::error;

const NODE_NAME: &str = "srb_gui_gui";
const GRIPPER_TOGGLE_TOPIC: &str = "/touch/event";
const RESET_DISCARD_DATASET_TOPIC: &str = "/gui/reset_discard_dataset";
const SHUTDOWN_PROCESS_TOPIC: &str = "/gui/shutdown_process";
const GRAVITY_TOPIC: &str = "/gui/gravity";
const LATENCY_TOPIC: &str = "/gui/latency";
const MOTION_SENSITIVITY_TOPIC: &str = "/gui/motion_sensitivity";

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GuiRosError {
    Initialization(String),
    Unavailable(String),
    Publish { topic: &'static str, error: String },
}

impl fmt::Display for GuiRosError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Initialization(error) => write!(formatter, "ROS initialization failed: {error}"),
            Self::Unavailable(error) => write!(formatter, "ROS controls unavailable: {error}"),
            Self::Publish { topic, error } => {
                write!(formatter, "ROS publish to {topic} failed: {error}")
            }
        }
    }
}

impl std::error::Error for GuiRosError {}

struct RosPublishers {
    _node: r2r::Node,
    gripper_toggle: r2r::Publisher<BoolMsg>,
    reset_discard_dataset: r2r::Publisher<EmptyMsg>,
    gracefully_shutdown_process: r2r::Publisher<EmptyMsg>,
    gravity: r2r::Publisher<Float64Msg>,
    latency: r2r::Publisher<Float64Msg>,
    motion_sensitivity: r2r::Publisher<Float64Msg>,
}

pub struct GuiRosControls {
    publishers: Option<RosPublishers>,
    initialization_error: Option<String>,
    previous_params: RuntimeParams,
}

impl Default for GuiRosControls {
    fn default() -> Self {
        Self::new()
    }
}

impl GuiRosControls {
    pub fn new() -> Self {
        match Self::try_new() {
            Ok(controls) => controls,
            Err(error) => {
                error!("{error}");
                Self {
                    publishers: None,
                    initialization_error: Some(error.to_string()),
                    previous_params: RuntimeParams::default(),
                }
            }
        }
    }

    pub fn try_new() -> Result<Self, GuiRosError> {
        let ctx = r2r::Context::create()
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let mut node = r2r::Node::create(ctx, NODE_NAME, "")
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let gripper_toggle = node
            .create_publisher::<BoolMsg>(GRIPPER_TOGGLE_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let reset_discard_dataset = node
            .create_publisher::<EmptyMsg>(RESET_DISCARD_DATASET_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let gracefully_shutdown_process = node
            .create_publisher::<EmptyMsg>(SHUTDOWN_PROCESS_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let gravity = node
            .create_publisher::<Float64Msg>(GRAVITY_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let latency = node
            .create_publisher::<Float64Msg>(LATENCY_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;
        let motion_sensitivity = node
            .create_publisher::<Float64Msg>(MOTION_SENSITIVITY_TOPIC, QosProfile::default())
            .map_err(|error| GuiRosError::Initialization(error.to_string()))?;

        Ok(Self {
            publishers: Some(RosPublishers {
                _node: node,
                gripper_toggle,
                reset_discard_dataset,
                gracefully_shutdown_process,
                gravity,
                latency,
                motion_sensitivity,
            }),
            initialization_error: None,
            previous_params: RuntimeParams::default(),
        })
    }

    fn publishers(&self) -> Result<&RosPublishers, GuiRosError> {
        self.publishers.as_ref().ok_or_else(|| {
            GuiRosError::Unavailable(
                self.initialization_error
                    .clone()
                    .unwrap_or_else(|| "no ROS publishers were initialized".to_string()),
            )
        })
    }

    pub fn publish_runtime_params(
        &mut self,
        params: RuntimeParams,
    ) -> Result<RuntimeParams, GuiRosError> {
        let params = params.sanitized();
        let publishers = self.publishers()?;

        if self.previous_params.gravity != params.gravity {
            publishers
                .gravity
                .publish(&Float64Msg {
                    data: params.gravity,
                })
                .map_err(|error| GuiRosError::Publish {
                    topic: GRAVITY_TOPIC,
                    error: error.to_string(),
                })?;
        }

        if self.previous_params.latency_ms != params.latency_ms {
            publishers
                .latency
                .publish(&Float64Msg {
                    data: params.latency_seconds(),
                })
                .map_err(|error| GuiRosError::Publish {
                    topic: LATENCY_TOPIC,
                    error: error.to_string(),
                })?;
        }

        if self.previous_params.motion_sensitivity != params.motion_sensitivity {
            publishers
                .motion_sensitivity
                .publish(&Float64Msg {
                    data: params.motion_sensitivity,
                })
                .map_err(|error| GuiRosError::Publish {
                    topic: MOTION_SENSITIVITY_TOPIC,
                    error: error.to_string(),
                })?;
        }

        self.previous_params = params;
        Ok(params)
    }

    /// Make the next `publish_runtime_params` send every value to a fresh
    /// subprocess, even when unchanged.
    pub fn resend_all_on_next_publish(&mut self) {
        // The unset (NaN) state differs from every value, so all are sent.
        self.previous_params = RuntimeParams::default();
    }

    pub fn publish_reset_discard_dataset(&self) -> Result<(), GuiRosError> {
        self.publishers()?
            .reset_discard_dataset
            .publish(&EmptyMsg {})
            .map_err(|error| GuiRosError::Publish {
                topic: RESET_DISCARD_DATASET_TOPIC,
                error: error.to_string(),
            })
    }

    pub fn publish_shutdown_request(&self) -> Result<(), GuiRosError> {
        self.publishers()?
            .gracefully_shutdown_process
            .publish(&EmptyMsg {})
            .map_err(|error| GuiRosError::Publish {
                topic: SHUTDOWN_PROCESS_TOPIC,
                error: error.to_string(),
            })
    }

    pub fn publish_gripper_toggle(&self) -> Result<(), GuiRosError> {
        self.publishers()?
            .gripper_toggle
            .publish(&BoolMsg { data: true })
            .map_err(|error| GuiRosError::Publish {
                topic: GRIPPER_TOGGLE_TOPIC,
                error: error.to_string(),
            })
    }
}

impl GuiRuntimeSignals for GuiRosControls {
    fn request_shutdown(&self) -> Result<(), GuiRuntimeSignalError> {
        self.publish_shutdown_request()
            .map_err(|error| GuiRuntimeSignalError::new(error.to_string()))
    }

    fn reset_discard_dataset(&self) -> Result<(), GuiRuntimeSignalError> {
        self.publish_reset_discard_dataset()
            .map_err(|error| GuiRuntimeSignalError::new(error.to_string()))
    }
}
