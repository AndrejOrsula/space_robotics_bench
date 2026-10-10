use egui_plot::{Line, Plot, PlotPoints};
use futures::task::LocalSpawnExt;
use futures::StreamExt;
use r2r::std_msgs::msg::Float64 as Float64Msg;
use r2r::QosProfile;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// Environment 0 reward published by the SRB GUI interface.
const REWARD_TOPIC: &str = "/gui/reward";
/// Maximum number of retained reward samples.
const REWARD_HISTORY_CAPACITY: usize = 1000;
/// Upper bound on a single `spin_once` call so the stop flag is observed promptly.
const SPIN_INTERVAL: Duration = Duration::from_millis(10);

pub struct GuiMetrics {
    reward_history: Arc<Mutex<VecDeque<(f64, f64)>>>,
    stop: Arc<AtomicBool>,
    startup_error: Arc<Mutex<Option<String>>>,
    subscriber: Option<JoinHandle<()>>,
}

impl Default for GuiMetrics {
    fn default() -> Self {
        let history = Arc::new(Mutex::new(VecDeque::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let startup_error = Arc::new(Mutex::new(None));
        let subscriber = std::thread::Builder::new()
            .name("srb-gui-metrics".to_string())
            .spawn({
                let history = history.clone();
                let stop = stop.clone();
                let startup_error = startup_error.clone();
                move || subscribe_rewards(history, stop, startup_error)
            })
            .map_err(|err| {
                *startup_error
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner()) =
                    Some(format!("Could not start reward subscriber: {err}"));
            })
            .ok();

        Self {
            reward_history: history,
            stop,
            startup_error,
            subscriber,
        }
    }
}

/// Collect [`REWARD_TOPIC`] samples and release the owned ROS node when stopped.
fn subscribe_rewards(
    history: Arc<Mutex<VecDeque<(f64, f64)>>>,
    stop: Arc<AtomicBool>,
    startup_error: Arc<Mutex<Option<String>>>,
) {
    let ctx = match r2r::Context::create() {
        Ok(ctx) => ctx,
        Err(err) => {
            *startup_error
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner()) =
                Some(format!("ROS context setup failed: {err}"));
            return;
        }
    };
    let mut node = match r2r::Node::create(ctx, "srb_gui_metrics", "") {
        Ok(node) => node,
        Err(err) => {
            *startup_error
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner()) =
                Some(format!("ROS node setup failed: {err}"));
            return;
        }
    };
    let mut sub = match node.subscribe::<Float64Msg>(REWARD_TOPIC, QosProfile::default()) {
        Ok(sub) => sub,
        Err(err) => {
            *startup_error
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner()) =
                Some(format!("Reward subscription setup failed: {err}"));
            return;
        }
    };
    let start = std::time::Instant::now();
    let mut local_pool = futures::executor::LocalPool::new();
    if let Err(err) = local_pool.spawner().spawn_local(async move {
        while let Some(msg) = sub.next().await {
            let t = start.elapsed().as_secs_f64();
            let mut hist = history
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            hist.push_back((t, msg.data));
            if hist.len() > REWARD_HISTORY_CAPACITY {
                hist.pop_front();
            }
        }
    }) {
        *startup_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) =
            Some(format!("Reward subscriber setup failed: {err}"));
        return;
    }

    while !stop.load(Ordering::Acquire) {
        node.spin_once(SPIN_INTERVAL);
        local_pool.run_until_stalled();
    }
}

impl Drop for GuiMetrics {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(subscriber) = self.subscriber.take() {
            let _ = subscriber.join();
        }
    }
}

impl GuiMetrics {
    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Reward").strong().size(20.0));
            ui.weak(format!("environment 0 · {REWARD_TOPIC}"));
        });
        ui.add_space(6.0);

        if let Some(error) = self
            .startup_error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .as_ref()
        {
            ui.colored_label(ui.visuals().error_fg_color, format!("ROS setup: {error}"));
        }

        let hist = self
            .reward_history
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if hist.is_empty() {
            ui.centered_and_justified(|ui| {
                ui.weak("No reward samples yet");
            });
        } else {
            let points: PlotPoints = hist.iter().map(|&(t, v)| [t, v]).collect();
            let line = Line::new("Reward", points);
            Plot::new("reward_plot")
                .x_axis_label("Time (s)")
                .y_axis_label("Reward")
                .show(ui, |plot_ui| {
                    plot_ui.line(line);
                });
        }

        ui.ctx().request_repaint_after(Duration::from_millis(100));
    }
}
