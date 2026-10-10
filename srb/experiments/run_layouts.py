"""Compatibility exports for integration run-layout discovery."""

from srb.integrations import run_layouts as _implementation

CheckpointDiscoverer = _implementation.CheckpointDiscoverer
FrameworkConfigArtifact = _implementation.FrameworkConfigArtifact
FrameworkConfigDiscoverer = _implementation.FrameworkConfigDiscoverer
RLZooNormalizationArtifact = _implementation.RLZooNormalizationArtifact
find_dreamer_checkpoint = _implementation.find_dreamer_checkpoint
find_rsl_rl_checkpoint = _implementation.find_rsl_rl_checkpoint
find_sb3_checkpoint = _implementation.find_sb3_checkpoint
find_skrl_checkpoint = _implementation.find_skrl_checkpoint
find_tdmpc2_checkpoint = _implementation.find_tdmpc2_checkpoint
normalize_artifact_relpath = _implementation.normalize_artifact_relpath
resolve_algorithm_adapter = _implementation.resolve_algorithm_adapter
slugify_underscore = _implementation.slugify_underscore
discover_framework_config_artifact = _implementation.discover_framework_config_artifact
discover_rl_zoo_normalization_artifact = (
    _implementation.discover_rl_zoo_normalization_artifact
)
discover_run_checkpoint = _implementation.discover_run_checkpoint
latest_file = _implementation.latest_file

# Keep legacy resolver aliases; patching this facade does not rewrite the
# implementation's globals.
_CHECKPOINT_DISCOVERERS = _implementation._CHECKPOINT_DISCOVERERS
_FRAMEWORK_CONFIG_DISCOVERERS = _implementation._FRAMEWORK_CONFIG_DISCOVERERS
_discover_dreamer_checkpoint = _implementation._discover_dreamer_checkpoint
_discover_plain_framework_config = _implementation._discover_plain_framework_config
_discover_rl_zoo_checkpoint = _implementation._discover_rl_zoo_checkpoint
_discover_rsl_rl_checkpoint = _implementation._discover_rsl_rl_checkpoint
_discover_rsl_rl_framework_config = _implementation._discover_rsl_rl_framework_config
_discover_rl_zoo_framework_config = _implementation._discover_rl_zoo_framework_config
_discover_robomimic_checkpoint = _implementation._discover_robomimic_checkpoint
_discover_skrl_checkpoint = _implementation._discover_skrl_checkpoint
_discover_tdmpc2_checkpoint = _implementation._discover_tdmpc2_checkpoint
_nested_mapping = _implementation._nested_mapping

__all__ = [
    "CheckpointDiscoverer",
    "FrameworkConfigArtifact",
    "FrameworkConfigDiscoverer",
    "RLZooNormalizationArtifact",
    "find_dreamer_checkpoint",
    "find_rsl_rl_checkpoint",
    "find_sb3_checkpoint",
    "find_skrl_checkpoint",
    "find_tdmpc2_checkpoint",
    "normalize_artifact_relpath",
    "resolve_algorithm_adapter",
    "slugify_underscore",
    "discover_framework_config_artifact",
    "discover_rl_zoo_normalization_artifact",
    "discover_run_checkpoint",
    "latest_file",
]
