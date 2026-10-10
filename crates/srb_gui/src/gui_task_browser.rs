use crate::gui_catalog::{GuiCatalog, GuiCatalogTask, GuiCatalogTaskError};
use crate::gui_selection::{
    GuiSelection, GuiSelectionInitializationError, GuiTaskSelectionOutcome,
};

#[derive(Clone, Debug, Default)]
pub struct GuiTaskBrowser {
    pub selection: GuiSelection,
    pub filters: GuiTaskBrowserFilters,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GuiTaskBrowserFilters {
    pub query: String,
    pub category: Option<String>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GuiTaskFilterChoices {
    pub categories: Vec<String>,
}

impl GuiTaskBrowser {
    pub fn from_catalog_result<E: Clone>(
        gui_catalog: &Result<GuiCatalog, E>,
    ) -> Result<Self, GuiSelectionInitializationError<E>> {
        Ok(Self {
            selection: GuiSelection::from_catalog_result(gui_catalog)?,
            filters: GuiTaskBrowserFilters::default(),
        })
    }

    pub(crate) fn neutral_initialization_fallback() -> Self {
        Self {
            selection: GuiSelection::neutral_initialization_fallback(),
            filters: GuiTaskBrowserFilters::default(),
        }
    }

    pub fn filter_active(&self) -> bool {
        self.filters.filter_active()
    }

    pub fn clear_filters(&mut self) {
        self.filters.clear();
    }

    pub fn filtered_tasks(&self, catalog: &GuiCatalog) -> Vec<GuiCatalogTask> {
        let query = self.filters.query.trim().to_lowercase();
        let mut tasks: Vec<GuiCatalogTask> = catalog
            .tasks
            .iter()
            .filter(|task| self.matches_task_filters(task, &query))
            .cloned()
            .collect();
        tasks.sort_by(|left, right| {
            left.rank
                .cmp(&right.rank)
                .then_with(|| left.display_name.cmp(&right.display_name))
        });
        tasks
    }

    pub fn filter_choices(&self, catalog: &GuiCatalog) -> GuiTaskFilterChoices {
        GuiTaskFilterChoices {
            categories: catalog.categories(),
        }
    }

    pub fn primary_task<'a>(&self, catalog: &'a GuiCatalog) -> Option<&'a GuiCatalogTask> {
        self.selected_task(catalog)
            .or_else(|| catalog.default_task())
    }

    pub fn selected_task<'a>(&self, catalog: &'a GuiCatalog) -> Option<&'a GuiCatalogTask> {
        self.task_by_id(catalog, &self.selection.task_config().task)
    }

    pub fn task_by_id<'a>(
        &self,
        catalog: &'a GuiCatalog,
        task_id: &str,
    ) -> Option<&'a GuiCatalogTask> {
        catalog.tasks.iter().find(|task| task.id == task_id)
    }

    pub fn apply_task(
        &mut self,
        task: &GuiCatalogTask,
    ) -> Result<GuiTaskSelectionOutcome, GuiCatalogTaskError> {
        self.selection.select_task_from_catalog(task)
    }

    pub fn select_task(
        &mut self,
        catalog_task: Option<&GuiCatalogTask>,
        task_id: &str,
    ) -> Result<GuiTaskSelectionOutcome, GuiCatalogTaskError> {
        self.selection.select_task(catalog_task, task_id)
    }

    fn matches_task_filters(&self, task: &GuiCatalogTask, query: &str) -> bool {
        self.matches_query(task, query)
            && self
                .filters
                .category
                .as_ref()
                .is_none_or(|category| task.category.trim() == category.trim())
    }

    fn matches_query(&self, task: &GuiCatalogTask, query: &str) -> bool {
        query.is_empty()
            || task.display_name.to_lowercase().contains(query)
            || task.id.to_lowercase().contains(query)
            || task.tagline.to_lowercase().contains(query)
    }
}

impl GuiTaskBrowserFilters {
    fn filter_active(&self) -> bool {
        !self.query.trim().is_empty() || self.category.is_some()
    }

    fn clear(&mut self) {
        self.query.clear();
        self.category = None;
    }
}
