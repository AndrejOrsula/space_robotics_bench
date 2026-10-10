use crate::cache;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GuiCacheRecoveryAction<'a> {
    pub(crate) label: &'a str,
    pub(crate) description: &'a str,
    pub(crate) command: &'a str,
}

pub(crate) fn actions(
    recovery_commands: [cache::GuiCatalogCacheRecoveryCommand; 2],
) -> [GuiCacheRecoveryAction<'static>; 2] {
    recovery_commands.map(|recovery_command| GuiCacheRecoveryAction {
        label: recovery_command.label(),
        description: recovery_command.description(),
        command: recovery_command.command(),
    })
}
