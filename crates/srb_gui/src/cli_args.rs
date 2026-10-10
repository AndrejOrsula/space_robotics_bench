use std::ffi::OsStr;

#[derive(Debug, PartialEq, Eq)]
pub enum StartupAction {
    RunGui,
    PrintHelp,
}

pub const USAGE: &str = "\
Space Robotics Bench GUI

Usage:
  srb gui
  srb gui -- --help

Options:
  -h, --help    Print this help message and exit.
";

pub fn startup_action<I, S>(args: I) -> StartupAction
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    if args.into_iter().skip(1).any(|arg| {
        let arg = arg.as_ref();
        arg == OsStr::new("--help") || arg == OsStr::new("-h")
    }) {
        StartupAction::PrintHelp
    } else {
        StartupAction::RunGui
    }
}
