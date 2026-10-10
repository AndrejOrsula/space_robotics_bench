use crate::env_cfg::TaskConfig;
use crate::gui_runtime_params::RuntimeParams;
use crate::gui_selection::GuiSelection;
use crate::page::Page;

#[derive(Clone, Debug, PartialEq)]
pub struct GuiSessionState {
    current_page: Page,
    show_about: bool,
    collect_trajectory: bool,
    runtime_params: RuntimeParams,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiSessionEpisodeRequest {
    pub collect_trajectory: bool,
    pub task_config: TaskConfig,
}

impl GuiSessionState {
    #[must_use]
    pub fn from_selection(selection: &GuiSelection) -> Self {
        Self {
            current_page: Page::default(),
            show_about: false,
            collect_trajectory: false,
            runtime_params: Self::default_runtime_params(selection.task_config()),
        }
    }

    #[must_use]
    pub fn current_page(&self) -> Page {
        self.current_page
    }

    pub fn select_page(&mut self, page: Page) {
        self.current_page = page;
        if page == Page::QuickStart {
            self.show_about = false;
        }
    }

    #[must_use]
    pub fn show_about(&self) -> bool {
        self.show_about
    }

    pub fn show_about_mut(&mut self) -> &mut bool {
        &mut self.show_about
    }

    pub fn collect_trajectory_mut(&mut self) -> &mut bool {
        &mut self.collect_trajectory
    }

    #[must_use]
    pub fn runtime_params(&self) -> RuntimeParams {
        self.runtime_params
    }

    pub fn runtime_params_mut(&mut self) -> &mut RuntimeParams {
        &mut self.runtime_params
    }

    pub fn set_runtime_params(&mut self, runtime_params: RuntimeParams) {
        self.runtime_params = runtime_params;
    }

    pub fn start_task(&mut self, task_config: &TaskConfig) {
        self.runtime_params.gravity = task_config.domain.gravity_magnitude();
    }

    #[must_use]
    pub fn restart_episode_request(&self, task_config: &TaskConfig) -> GuiSessionEpisodeRequest {
        GuiSessionEpisodeRequest {
            collect_trajectory: self.collect_trajectory,
            task_config: task_config.clone(),
        }
    }

    pub fn exit_to_tasks(&mut self, selection: &mut GuiSelection) -> GuiSessionEpisodeRequest {
        let request = self.restart_episode_request(selection.task_config());
        self.select_page(Page::QuickStart);
        self.collect_trajectory = false;
        selection.reset_task_config();
        self.runtime_params = Self::default_runtime_params(selection.task_config());
        request
    }

    fn default_runtime_params(task_config: &TaskConfig) -> RuntimeParams {
        RuntimeParams {
            gravity: task_config.domain.gravity_magnitude(),
            latency_ms: 0.0,
            motion_sensitivity: 1.0,
        }
    }
}
