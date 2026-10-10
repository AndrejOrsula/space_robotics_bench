"""Compatibility exports for integration policy dispatch and loading."""

from srb.integrations import policies as _implementation

AlgorithmAdapter = _implementation.AlgorithmAdapter
ModelArtifactReference = _implementation.ModelArtifactReference
CheckpointInference = _implementation.CheckpointInference
LoadedPolicy = _implementation.LoadedPolicy
PolicyLoader = _implementation.PolicyLoader
discover_run_checkpoint = _implementation.discover_run_checkpoint
find_saved_policy_config = _implementation.find_saved_policy_config
infer_algo_from_model = _implementation.infer_algo_from_model
infer_model_algorithm = _implementation.infer_model_algorithm
load_policy = _implementation.load_policy
load_policy_from_checkpoint = _implementation.load_policy_from_checkpoint
policy_action_to_numpy = _implementation.policy_action_to_numpy
resolve_agent_workflow_algo = _implementation.resolve_agent_workflow_algo
resolve_model_artifact_policy_inputs = (
    _implementation.resolve_model_artifact_policy_inputs
)
resolve_model_artifact_reference = _implementation.resolve_model_artifact_reference
resolve_model_path = _implementation.resolve_model_path
resolve_algorithm_adapter = _implementation.resolve_algorithm_adapter
slugify_underscore = _implementation.slugify_underscore
supports_policy_collection = _implementation.supports_policy_collection
try_model_artifact_reference = _implementation.try_model_artifact_reference

# Keep legacy loader seams as aliases; this facade does not alter implementation
# globals when callers patch them.
_POLICY_LOADERS = _implementation._POLICY_LOADERS
_supports_algorithm_policy_collection = (
    _implementation._supports_algorithm_policy_collection
)
_ambiguous_legacy_inference = _implementation._ambiguous_legacy_inference
_flatten_rl_zoo_observation = _implementation._flatten_rl_zoo_observation
_load_dreamer_policy = _implementation._load_dreamer_policy
_load_rl_zoo_policy = _implementation._load_rl_zoo_policy
_load_sb3_policy = _implementation._load_sb3_policy
_load_rsl_rl_policy = _implementation._load_rsl_rl_policy
_load_robomimic_policy = _implementation._load_robomimic_policy
_load_sbx_policy = _implementation._load_sbx_policy
_load_skrl_policy = _implementation._load_skrl_policy
_load_tdmpc2_policy = _implementation._load_tdmpc2_policy
_require_local_algo = _implementation._require_local_algo
_resolve_legacy_candidates = _implementation._resolve_legacy_candidates

__all__ = [
    "AlgorithmAdapter",
    "CheckpointInference",
    "LoadedPolicy",
    "ModelArtifactReference",
    "PolicyLoader",
    "discover_run_checkpoint",
    "find_saved_policy_config",
    "infer_algo_from_model",
    "infer_model_algorithm",
    "load_policy",
    "load_policy_from_checkpoint",
    "policy_action_to_numpy",
    "resolve_agent_workflow_algo",
    "resolve_algorithm_adapter",
    "resolve_model_artifact_policy_inputs",
    "resolve_model_artifact_reference",
    "resolve_model_path",
    "supports_policy_collection",
    "slugify_underscore",
    "try_model_artifact_reference",
]
