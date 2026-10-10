"""Framework-aware artifact discovery for SRB integration run layouts."""

from __future__ import annotations

import glob
import os
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, cast

from srb.integrations.algorithms import resolve_algorithm_adapter
from srb.utils.artifact_identity import slugify_underscore
from srb.utils.checkpoint import (
    find_dreamer_checkpoint,
    find_rsl_rl_checkpoint,
    find_sb3_checkpoint,
    find_skrl_checkpoint,
    find_tdmpc2_checkpoint,
)
from srb.utils.model_artifacts import (
    RL_ZOO_NORMALIZATION_NONE,
    RL_ZOO_NORMALIZATION_VECNORMALIZE,
    normalize_artifact_relpath,
)
from srb.utils.publication import PublicationError, read_regular_file_bytes

CheckpointDiscoverer = Callable[[Path, str, bool | None], Path | str | None]

SB3_VECNORMALIZE_NAME = "vecnormalize.pkl"
# ``CheckpointCallback`` names models ``<prefix>_<N>_steps.zip`` and, when
# enabled, their statistics ``<prefix>_vecnormalize_<N>_steps.pkl`` and replay
# buffers ``<prefix>_replay_buffer_<N>_steps.pkl``.
SB3_PERIODIC_CHECKPOINT = re.compile(r"(?P<prefix>.+)_(?P<steps>\d+)_steps")

_RL_ZOO_ORDERED_DICT_TAG = (
    "tag:yaml.org,2002:python/object/apply:collections.OrderedDict"
)
_YAML_BINARY_TAG = "tag:yaml.org,2002:binary"
_YAML_BOOL_TAG = "tag:yaml.org,2002:bool"
_YAML_FLOAT_TAG = "tag:yaml.org,2002:float"
_YAML_INT_TAG = "tag:yaml.org,2002:int"
_YAML_MAP_TAG = "tag:yaml.org,2002:map"
_YAML_NULL_TAG = "tag:yaml.org,2002:null"
_YAML_SCALAR_TAG = "tag:yaml.org,2002:str"
_YAML_SEQUENCE_TAG = "tag:yaml.org,2002:seq"
_YAML_TIMESTAMP_TAG = "tag:yaml.org,2002:timestamp"
_YAML_SAFE_SCALAR_TAGS = frozenset(
    {
        _YAML_BINARY_TAG,
        _YAML_BOOL_TAG,
        _YAML_FLOAT_TAG,
        _YAML_INT_TAG,
        _YAML_NULL_TAG,
        _YAML_SCALAR_TAG,
        _YAML_TIMESTAMP_TAG,
    }
)
_RL_ZOO_MAX_INPUT_BYTES = 1 << 20
_RL_ZOO_MAX_NODES = 10_000
_RL_ZOO_MAX_CONTAINER_DEPTH = 32


def _read_rl_zoo_config_bytes(config_path: Path) -> bytes:
    """Capture one bounded regular RL-Zoo config for admission and publication."""

    try:
        payload = read_regular_file_bytes(
            config_path,
            max_bytes=_RL_ZOO_MAX_INPUT_BYTES,
        )
    except (OSError, UnicodeError, ValueError, PublicationError) as exc:
        raise ValueError(f"could not read RL-Zoo config: {config_path}: {exc}") from exc
    if payload is None:
        raise ValueError(f"RL-Zoo config must be a regular file: {config_path}")
    return payload


def _validate_rl_zoo_events(payload: bytes | str) -> None:
    """Admit a bounded, alias-free event stream before composing YAML nodes."""

    import yaml

    nodes = 0
    depth = 0
    try:
        for event in yaml.parse(payload, Loader=yaml.SafeLoader):
            if isinstance(event, yaml.events.AliasEvent):
                raise ValueError("RL-Zoo YAML aliases are unsupported")
            if isinstance(
                event,
                (
                    yaml.events.ScalarEvent,
                    yaml.events.SequenceStartEvent,
                    yaml.events.MappingStartEvent,
                ),
            ):
                nodes += 1
                if nodes > _RL_ZOO_MAX_NODES:
                    raise ValueError(
                        "RL-Zoo YAML node limit exceeded: "
                        f"observed {nodes}, limit {_RL_ZOO_MAX_NODES}"
                    )
            if isinstance(
                event,
                (yaml.events.SequenceStartEvent, yaml.events.MappingStartEvent),
            ):
                depth += 1
                if depth > _RL_ZOO_MAX_CONTAINER_DEPTH:
                    raise ValueError(
                        "RL-Zoo YAML container depth limit exceeded: "
                        f"observed {depth}, limit {_RL_ZOO_MAX_CONTAINER_DEPTH}"
                    )
            elif isinstance(
                event,
                (yaml.events.SequenceEndEvent, yaml.events.MappingEndEvent),
            ):
                depth -= 1
    except RecursionError as exc:
        raise ValueError("RL-Zoo YAML recursion limit exceeded") from exc


def _load_rl_zoo_config(payload: bytes | str) -> Any:
    """Project RL-Zoo YAML nodes without invoking arbitrary constructors."""

    import yaml

    if isinstance(payload, bytes):
        if len(payload) > _RL_ZOO_MAX_INPUT_BYTES:
            raise ValueError(
                "RL-Zoo YAML byte limit exceeded: "
                f"observed {len(payload)}, limit {_RL_ZOO_MAX_INPUT_BYTES}"
            )
    elif isinstance(payload, str):
        encoded_bytes = 0
        for character in payload:
            codepoint = ord(character)
            encoded_bytes += (
                1
                if codepoint < 0x80
                else 2
                if codepoint < 0x800
                else 3
                if codepoint < 0x10000
                else 4
            )
            if encoded_bytes > _RL_ZOO_MAX_INPUT_BYTES:
                raise ValueError(
                    "RL-Zoo YAML byte limit exceeded: "
                    f"observed {encoded_bytes}, limit {_RL_ZOO_MAX_INPUT_BYTES}"
                )
    else:
        raise TypeError("RL-Zoo YAML input must be bytes or str")

    _validate_rl_zoo_events(payload)

    def project_scalar(node: Any) -> Any:
        if node.tag not in _YAML_SAFE_SCALAR_TAGS:
            raise ValueError(f"unsupported RL-Zoo YAML tag: {node.tag}")
        if node.tag == _YAML_SCALAR_TAG:
            return node.value

        loader = yaml.SafeLoader(node.value)
        try:
            constructor = loader.yaml_constructors.get(node.tag)
            if constructor is None:
                raise ValueError(f"unsupported RL-Zoo YAML tag: {node.tag}")
            return constructor(loader, node)
        finally:
            cast(Callable[[], None], loader.dispose)()

    def project(node: Any, active_nodes: set[int]) -> Any:
        node_id = id(node)
        if node_id in active_nodes:
            raise ValueError("recursive RL-Zoo YAML aliases are unsupported")
        active_nodes.add(node_id)
        try:
            if node.tag == _RL_ZOO_ORDERED_DICT_TAG:
                if (
                    not isinstance(node, yaml.nodes.SequenceNode)
                    or len(node.value) != 1
                ):
                    raise ValueError(
                        "RL-Zoo OrderedDict tag must contain one pair sequence"
                    )
                pairs_node = node.value[0]
                if not isinstance(pairs_node, yaml.nodes.SequenceNode) or (
                    pairs_node.tag != _YAML_SEQUENCE_TAG
                ):
                    raise ValueError(
                        "RL-Zoo OrderedDict tag must contain one pair sequence"
                    )
                result: dict[str, Any] = {}
                for pair_node in pairs_node.value:
                    if not isinstance(pair_node, yaml.nodes.SequenceNode) or (
                        pair_node.tag != _YAML_SEQUENCE_TAG or len(pair_node.value) != 2
                    ):
                        raise ValueError(
                            "RL-Zoo OrderedDict entries must be key-value pairs"
                        )
                    key = project(pair_node.value[0], active_nodes)
                    if not isinstance(key, str):
                        raise ValueError(
                            "RL-Zoo OrderedDict keys must be scalar strings"
                        )
                    if key in result:
                        raise ValueError(f"duplicate RL-Zoo YAML mapping key: {key!r}")
                    result[key] = project(pair_node.value[1], active_nodes)
                return result

            if isinstance(node, yaml.nodes.MappingNode):
                if node.tag != _YAML_MAP_TAG:
                    raise ValueError(f"unsupported RL-Zoo YAML tag: {node.tag}")
                result_mapping: dict[Any, Any] = {}
                for key_node, value_node in node.value:
                    key = project(key_node, active_nodes)
                    try:
                        duplicate = key in result_mapping
                    except TypeError as exc:
                        raise ValueError(
                            "RL-Zoo YAML mapping keys must be hashable"
                        ) from exc
                    if duplicate:
                        raise ValueError(f"duplicate RL-Zoo YAML mapping key: {key!r}")
                    value = project(value_node, active_nodes)
                    try:
                        result_mapping[key] = value
                    except TypeError as exc:
                        raise ValueError(
                            "RL-Zoo YAML mapping keys must be hashable"
                        ) from exc
                return result_mapping

            if isinstance(node, yaml.nodes.SequenceNode):
                if node.tag != _YAML_SEQUENCE_TAG:
                    raise ValueError(f"unsupported RL-Zoo YAML tag: {node.tag}")
                return [project(item, active_nodes) for item in node.value]

            if isinstance(node, yaml.nodes.ScalarNode):
                return project_scalar(node)

            raise ValueError(f"unsupported RL-Zoo YAML node: {type(node).__name__}")
        finally:
            active_nodes.remove(node_id)

    try:
        node = yaml.compose(payload, Loader=yaml.SafeLoader)
        if node is None:
            return None
        return project(node, set())
    except RecursionError as exc:
        raise ValueError("RL-Zoo YAML recursion limit exceeded") from exc


@dataclass(frozen=True)
class FrameworkConfigArtifact:
    """Framework config packaged alongside a checkpoint."""

    source_path: Path | None
    artifact_relpath: str
    generated_config: dict[str, Any] | None = None
    admitted_bytes: bytes | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "artifact_relpath",
            normalize_artifact_relpath(
                self.artifact_relpath,
                field_name="framework config artifact_relpath",
            ),
        )
        if self.source_path is None and self.generated_config is None:
            raise ValueError(
                "framework config artifact requires source_path or generated_config"
            )


@dataclass(frozen=True, slots=True)
class Sb3StatsLookup:
    """Result of locating VecNormalize statistics."""

    path: Path | None
    searched: tuple[Path, ...]
    # Run-level statistics used for a periodic checkpoint without its own.
    stale: bool = False
    # A run that saves per-checkpoint statistics lacks this checkpoint's file.
    missing_own: Path | None = None


def sb3_checkpoint_stem(path: Path) -> str:
    return path.name.removesuffix(".zip")


def sb3_checkpoint_paths(checkpoint: str | Path) -> list[Path]:
    """Return the supplied checkpoint path and its SB3 .zip target."""
    literal = Path(checkpoint)
    if not literal.exists() and literal.suffix != ".zip":
        # SB3's loaders append ``.zip`` to a path that does not exist.
        zipped = literal.with_name(f"{literal.name}.zip")
        if zipped.exists():
            literal = zipped
    paths = [literal]
    if (resolved := literal.resolve()) != literal.absolute():
        paths.append(resolved)
    return paths


def pair_sb3_checkpoint_vecnormalize(checkpoint: str | Path) -> Sb3StatsLookup:
    """Find statistics paired with a periodic checkpoint.

    A <prefix>_<N>_steps checkpoint uses matching
    <prefix>_vecnormalize_<N>_steps.pkl. If its own stats are missing while
    earlier stats exist, record missing_own to block stale substitution;
    otherwise run-level stats apply.
    """
    if not checkpoint:
        return Sb3StatsLookup(None, ())
    searched: list[Path] = []
    for model in sb3_checkpoint_paths(checkpoint):
        match = SB3_PERIODIC_CHECKPOINT.fullmatch(sb3_checkpoint_stem(model))
        if match is None:
            continue
        own = model.with_name(
            f"{match['prefix']}_vecnormalize_{match['steps']}_steps.pkl"
        )
        searched.append(own)
        if own.is_file():
            return Sb3StatsLookup(own, tuple(searched))
        # Only an earlier checkpoint with statistics shows that this one was
        # written with them too; a run resumed with per-checkpoint statistics
        # adds them to later checkpoints only.
        siblings = model.parent.glob(
            f"{glob.escape(match['prefix'])}_vecnormalize_*_steps.pkl"
        )
        if any(
            sibling.is_file()
            and (earlier := SB3_PERIODIC_CHECKPOINT.fullmatch(sibling.stem)) is not None
            and int(earlier["steps"]) < int(match["steps"])
            for sibling in siblings
        ):
            return Sb3StatsLookup(None, tuple(searched), missing_own=own)
    return Sb3StatsLookup(None, tuple(searched))


def find_sb3_vecnormalize(
    checkpoint: str | Path, *, env_names: Sequence[str]
) -> Sb3StatsLookup:
    """Find VecNormalize statistics for a checkpoint.

    Periodic checkpoints use paired stats. Otherwise search packaged stats, the
    model directory, run root, then RL-Zoo environment directories. Symlink
    targets are also searched.
    """
    paired = pair_sb3_checkpoint_vecnormalize(checkpoint)
    if not checkpoint or paired.path is not None or paired.missing_own is not None:
        return paired
    searched = list(paired.searched)
    periodic = bool(searched)
    for model in sb3_checkpoint_paths(checkpoint):
        parent = model.parent
        if parent.name == "checkpoints":
            # ``srb model`` artifact: packaged with this checkpoint.
            artifact = parent.parent / "normalization" / SB3_VECNORMALIZE_NAME
            searched.append(artifact)
            if artifact.is_file():
                return Sb3StatsLookup(artifact, tuple(searched))
        candidates = [parent / SB3_VECNORMALIZE_NAME]
        if parent.name == "ckpt":
            candidates.append(parent.parent / SB3_VECNORMALIZE_NAME)
        candidates.extend(parent / name / SB3_VECNORMALIZE_NAME for name in env_names)
        for candidate in candidates:
            searched.append(candidate)
            if candidate.is_file():
                return Sb3StatsLookup(candidate, tuple(searched), stale=periodic)
    return Sb3StatsLookup(None, tuple(searched))


@dataclass(frozen=True, slots=True)
class RLZooNormalizationArtifact:
    """Normalization evidence discovered in an RL-Zoo run directory."""

    metadata: dict[str, str] | None
    stats_source_path: Path | None = None


def discover_rl_zoo_normalization_artifact(
    run_dir: str | Path,
    config_artifact: FrameworkConfigArtifact | None,
    *,
    checkpoint: str | Path | None = None,
    algo_id: str | None = None,
) -> RLZooNormalizationArtifact:
    """Resolve RL-Zoo normalization from config and statistics.

    Explicit normalize: false proves normalization is disabled; enabled config
    requires stats. Missing or ambiguous evidence stays unknown. Periodic
    checkpoints use their own stats, never stale run-level stats when paired
    stats are missing. algo_id, when supplied, must identify SB3/SBX.
    """

    if algo_id is not None and not algo_id.startswith(("sb3_", "sbx_")):
        return RLZooNormalizationArtifact(None)
    if config_artifact is None or config_artifact.source_path is None:
        return RLZooNormalizationArtifact(None)
    config_path = config_artifact.source_path

    import yaml

    try:
        payload = config_artifact.admitted_bytes
        if payload is None:
            payload = _read_rl_zoo_config_bytes(config_path)
        config = _load_rl_zoo_config(payload)
    except (OSError, UnicodeError, ValueError, PublicationError, yaml.YAMLError) as exc:
        raise ValueError(f"could not read RL-Zoo config: {config_path}: {exc}") from exc
    if config is None:
        return RLZooNormalizationArtifact(None)
    if not isinstance(config, dict):
        raise ValueError("RL-Zoo config must be a mapping")
    normalize = config.get("normalize")
    if normalize is None:
        return RLZooNormalizationArtifact(None)
    if type(normalize) is bool:
        normalize_enabled = normalize
    elif isinstance(normalize, (str, dict)):
        # RL-Zoo treats a string expression or mapping as VecNormalize
        # kwargs, then enables normalization unconditionally.
        normalize_enabled = True
    else:
        raise ValueError("RL-Zoo config normalize must be boolean or kwargs")

    stats_path = Path(run_dir) / SB3_VECNORMALIZE_NAME
    # ``lexists`` catches a dangling symlink, which ``exists`` would turn into
    # an indistinguishable missing-statistics case.
    stats_present = os.path.lexists(stats_path)
    paired = (
        pair_sb3_checkpoint_vecnormalize(checkpoint)
        if checkpoint is not None
        else Sb3StatsLookup(None, ())
    )
    if not normalize_enabled:
        if stats_present or paired.path is not None or paired.missing_own is not None:
            raise ValueError(
                "RL-Zoo config declares normalize=false but VecNormalize statistics "
                "exist for this run; remove the statistics or set normalize=true"
            )
        return RLZooNormalizationArtifact(dict(RL_ZOO_NORMALIZATION_NONE))

    if paired.missing_own is not None:
        raise ValueError(
            f"checkpoint {checkpoint} belongs to a run that saves VecNormalize "
            f"statistics per checkpoint, but its own {paired.missing_own} is "
            "missing; the run's other statistics were written at a different "
            "training step and are not substituted. Package the final model or "
            "a periodic checkpoint whose statistics exist"
        )
    if paired.path is not None:
        stats_path = paired.path
    if stats_path.is_symlink() or not stats_path.is_file():
        raise ValueError(
            "RL-Zoo config declares normalize=true but vecnormalize.pkl is "
            f"missing or not a regular file: {stats_path}"
        )
    if paired.path is None and paired.searched:
        from srb.utils import logging

        logging.warning(
            f"{checkpoint} has no VecNormalize statistics of its own (saved "
            f"before per-checkpoint statistics); packaging {stats_path}, which "
            "was written at a different training step"
        )
    return RLZooNormalizationArtifact(
        dict(RL_ZOO_NORMALIZATION_VECNORMALIZE),
        stats_path,
    )


FrameworkConfigDiscoverer = Callable[
    [Path, dict[str, Any]], FrameworkConfigArtifact | None
]


def discover_run_checkpoint(
    run_dir: str | Path,
    algo: str,
    *,
    workflow: str = "eval",
    continue_training: bool | None = False,
    required: bool = True,
) -> Path | None:
    """Discover a checkpoint by SRB algorithm.

    Framework-specific layouts stay behind the AlgorithmAdapter seam.
    """
    run_path = Path(run_dir)
    adapter = resolve_algorithm_adapter(slugify_underscore(algo))
    discoverer = _CHECKPOINT_DISCOVERERS.get(adapter.framework)
    if discoverer is None:
        if required:
            raise ValueError(
                f"unknown SRB algorithm checkpoint layout: {adapter.framework}"
            )
        return None

    try:
        checkpoint = discoverer(run_path, workflow, continue_training)
    except FileNotFoundError:
        if required:
            raise
        return None

    if not checkpoint:
        if required:
            raise FileNotFoundError(
                f"No {adapter.framework} checkpoint found under {run_path}"
            )
        return None
    return Path(checkpoint)


def discover_framework_config_artifact(
    run_dir: str | Path,
    algo: str,
    run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    """Discover a companion framework config for an SRB run."""
    run_path = Path(run_dir)
    adapter = resolve_algorithm_adapter(slugify_underscore(algo))
    discoverer = _FRAMEWORK_CONFIG_DISCOVERERS.get(adapter.framework)
    if discoverer is None:
        return None
    return discoverer(run_path, run_metadata)


def latest_file(
    directory: Path,
    *,
    pattern: str = "*",
    modification_time: bool = False,
) -> Path | None:
    """Return the first root-contained regular file matching pattern."""
    if not directory.is_dir():
        return None
    root = directory.resolve()
    files: list[Path] = []
    for path in directory.glob(pattern):
        if path.is_symlink() or not path.is_file():
            continue
        try:
            if not path.resolve(strict=True).is_relative_to(root):
                continue
        except (OSError, RuntimeError):
            continue
        files.append(path)
    if not files:
        return None
    files.sort(
        key=(lambda path: path.stat().st_mtime) if modification_time else None,
        reverse=True,
    )
    return files[0]


def _discover_rl_zoo_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    return find_sb3_checkpoint(run_path, None, workflow, continue_training)


def _discover_skrl_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    return find_skrl_checkpoint(run_path, None, workflow, continue_training)


def _discover_rsl_rl_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    return find_rsl_rl_checkpoint(run_path, None, workflow, continue_training)


def _discover_dreamer_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    checkpoint = find_dreamer_checkpoint(
        run_path,
        None,
        workflow,
        continue_training,
    )
    if checkpoint or workflow == "eval" or not continue_training:
        return checkpoint
    return latest_file(
        run_path / "ckpt",
        pattern="*.ckpt",
        modification_time=True,
    )


def _discover_tdmpc2_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    return find_tdmpc2_checkpoint(run_path, None, workflow, continue_training)


def _discover_robomimic_checkpoint(
    run_path: Path,
    workflow: str,
    continue_training: bool | None,
) -> Path | str | None:
    if workflow != "eval" and not continue_training:
        return None

    candidates: dict[Path, None] = {}
    for pattern in (
        "models/model_epoch_*.pth",
        "models/*.pth",
        "*.pth",
        "**/model_epoch_*.pth",
        "**/*.pth",
    ):
        for path in run_path.glob(pattern):
            if path.is_file():
                candidates[path] = None
    if not candidates:
        if workflow == "eval":
            raise FileNotFoundError(f"No robomimic checkpoint found under {run_path}")
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _discover_rsl_rl_framework_config(
    run_path: Path,
    run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    """Package the unsanitized RSL-RL agent config, dropping secret values.

    Prefer run-root config.yaml, then agent from .hydra/config.yaml; use
    unsanitized run.agent_cfg metadata only as fallback.
    """
    plain_config = run_path / "config.yaml"
    if plain_config.is_file():
        return _generated_agent_config(_read_yaml_mapping(plain_config), plain_config)
    return _discover_hydra_or_metadata_agent_config(run_path, run_metadata)


def _discover_skrl_framework_config(
    run_path: Path,
    run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    """Package SKRL's Hydra agent section as the runner config.

    Use unsanitized run.agent_cfg metadata only as fallback; secret values are dropped.
    """
    return _discover_hydra_or_metadata_agent_config(run_path, run_metadata)


def _discover_hydra_or_metadata_agent_config(
    run_path: Path,
    run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    hydra_config = run_path / ".hydra" / "config.yaml"
    if hydra_config.is_file():
        agent_section = _read_yaml_mapping(hydra_config).get("agent")
        if isinstance(agent_section, dict) and agent_section:
            return _generated_agent_config(agent_section, hydra_config)

    agent_cfg = _nested_mapping(run_metadata, "run", "agent_cfg")
    if agent_cfg:
        source = f"{run_path / 'metadata.json'} run.agent_cfg"
        require_unsanitized_config(agent_cfg, source=source)
        return _generated_agent_config(agent_cfg, source)
    return None


def _read_yaml_mapping(path: Path) -> dict[str, Any]:
    import yaml

    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"run config must be a mapping: {path}")
    return loaded


def _generated_agent_config(
    agent_cfg: dict[str, Any],
    source: Path | str,
) -> FrameworkConfigArtifact:
    return FrameworkConfigArtifact(
        None, "config.yaml", _drop_secret_values(agent_cfg, source=str(source))
    )


def _drop_secret_values(config: dict[str, Any], *, source: str) -> dict[str, Any]:
    """Copy config while dropping nonempty secret-key values and strings containing
    redactable credentials. Warn with paths only; leave source files unchanged.
    """
    from srb.utils import logging
    from srb.utils.redaction import is_secret_key, redact_secret_text

    def embeds_secret(item: Any) -> bool:
        return isinstance(item, str) and redact_secret_text(item) != item

    dropped: list[str] = []

    def scrub(node: Any, prefix: str) -> Any:
        if isinstance(node, dict):
            kept: dict[Any, Any] = {}
            for key, item in node.items():
                path = f"{prefix}.{key}" if prefix else str(key)
                if (
                    isinstance(key, str)
                    and is_secret_key(key)
                    and item is not None
                    and not isinstance(item, bool)
                ) or embeds_secret(item):
                    dropped.append(path)
                    continue
                kept[key] = scrub(item, path)
            return kept
        if isinstance(node, list):
            items: list[Any] = []
            for index, item in enumerate(node):
                path = f"{prefix}[{index}]"
                if embeds_secret(item):
                    dropped.append(path)
                    continue
                items.append(scrub(item, path))
            return items
        return node

    scrubbed = scrub(config, "")
    if dropped:
        logging.warning(
            f"Model artifact framework config from {source}: dropped secret-looking "
            f"values at {', '.join(dropped)} (the source run files are unchanged)"
        )
    return scrubbed


#: String forms ``srb.utils.metadata._snapshot_config`` writes in place of a
#: value it could not keep: ``repr`` of a non-finite float, and the
#: recursion/depth placeholders. ``REDACTED`` is matched as a substring because
#: ``redact_secret_text`` also rewrites credentials embedded in longer strings.
_SANITIZED_FLOAT_TEXT = frozenset({"inf", "-inf", "nan"})
_SANITIZED_PLACEHOLDER_PREFIXES = ("<recursive ", "<truncated ")


def sanitized_config_paths(value: Any, prefix: str = "") -> list[str]:
    """Return dotted paths whose values run-metadata sanitization rewrote."""
    from srb.utils.redaction import REDACTED

    if isinstance(value, dict):
        paths: list[str] = []
        for key, item in value.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            paths.extend(sanitized_config_paths(item, path))
        return paths
    if isinstance(value, (list, tuple)):
        paths = []
        for index, item in enumerate(value):
            paths.extend(sanitized_config_paths(item, f"{prefix}[{index}]"))
        return paths
    if isinstance(value, str) and (
        REDACTED in value
        or value in _SANITIZED_FLOAT_TEXT
        or value.startswith(_SANITIZED_PLACEHOLDER_PREFIXES)
    ):
        return [prefix or "<root>"]
    return []


def require_unsanitized_config(config: Any, *, source: str) -> None:
    """Reject configs rewritten by run-metadata sanitization.

        Redaction placeholders and stringified non-finite floats cannot restore
    runner config.
    """
    paths = sanitized_config_paths(config)
    if paths:
        raise ValueError(
            f"policy config from {source} holds sanitized values at "
            f"{', '.join(paths)} and cannot restore the training configuration; "
            "use the run's unsanitized `.hydra/config.yaml` instead"
        )


def _discover_plain_framework_config(
    run_path: Path,
    _run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    config_path = run_path / "config.yaml"
    if not config_path.is_file():
        return None
    return FrameworkConfigArtifact(config_path, "config.yaml")


def _discover_rl_zoo_framework_config(
    run_path: Path,
    _run_metadata: dict[str, Any],
) -> FrameworkConfigArtifact | None:
    config_path = run_path / "config.yml"
    if not config_path.is_file():
        return None
    return FrameworkConfigArtifact(
        config_path,
        "config.yml",
        admitted_bytes=_read_rl_zoo_config_bytes(config_path),
    )


def _nested_mapping(
    data: dict[str, Any],
    section: str,
    key: str,
) -> dict[str, Any] | None:
    section_value = data.get(section)
    if not isinstance(section_value, dict):
        return None
    value = section_value.get(key)
    if isinstance(value, dict):
        return dict(value)
    return None


_CHECKPOINT_DISCOVERERS: dict[str, CheckpointDiscoverer] = {
    "dreamer": _discover_dreamer_checkpoint,
    "rsl_rl": _discover_rsl_rl_checkpoint,
    "sb3": _discover_rl_zoo_checkpoint,
    "sbx": _discover_rl_zoo_checkpoint,
    "skrl": _discover_skrl_checkpoint,
    "tdmpc2": _discover_tdmpc2_checkpoint,
    "robomimic": _discover_robomimic_checkpoint,
}

_FRAMEWORK_CONFIG_DISCOVERERS: dict[str, FrameworkConfigDiscoverer] = {
    "rsl_rl": _discover_rsl_rl_framework_config,
    "skrl": _discover_skrl_framework_config,
    "dreamer": _discover_plain_framework_config,
    "tdmpc2": _discover_plain_framework_config,
    "sb3": _discover_rl_zoo_framework_config,
    "sbx": _discover_rl_zoo_framework_config,
}
