#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RuntimeParams {
    pub gravity: f64,
    pub latency_ms: f64,
    pub motion_sensitivity: f64,
}

impl RuntimeParams {
    #[must_use]
    pub fn sanitized(self) -> Self {
        Self {
            gravity: self.gravity.max(0.0),
            latency_ms: self.latency_ms.max(0.0),
            motion_sensitivity: self.motion_sensitivity.max(0.0),
        }
    }

    #[must_use]
    pub(crate) fn latency_seconds(self) -> f64 {
        self.latency_ms / 1000.0
    }
}

impl Default for RuntimeParams {
    fn default() -> Self {
        Self {
            gravity: f64::NAN,
            latency_ms: f64::NAN,
            motion_sensitivity: f64::NAN,
        }
    }
}
