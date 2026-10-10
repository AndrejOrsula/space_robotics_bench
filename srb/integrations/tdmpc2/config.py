"""Build consistent TD-MPC2 configs for training and policy loading."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from omegaconf import OmegaConf

from srb.integrations.tdmpc2.preflight import ending_semantics_record, resolve_episodic
from srb.integrations.tdmpc2.upstream import import_upstream, upstream_pin_record

if TYPE_CHECKING:
    from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper

#: Namespace for SRB-owned provenance persisted inside the upstream config.
SRB_CONFIG_KEY = "srb_tdmpc2"

_UPSTREAM_CONFIG_PATH: Path | None = None


def upstream_config_path() -> Path:
    """Path of the upstream ``config.yaml`` (resolved after ``sys.path`` setup)."""
    global _UPSTREAM_CONFIG_PATH
    if _UPSTREAM_CONFIG_PATH is None:
        import tdmpc2 as _tdmpc2_module

        _UPSTREAM_CONFIG_PATH = Path(_tdmpc2_module.__file__).parent / "config.yaml"
    return _UPSTREAM_CONFIG_PATH


def build_tdmpc2_config(
    *,
    env: "SrbTdmpc2Wrapper",
    task_name: str,
    work_dir: Path,
    agent_cfg: dict[str, Any] | None = None,
    saved_config: Path | None = None,
    seed: int | None = None,
    data_dir: Path | None = None,
) -> tuple[Any, Any]:
    """Return the upstream config and merged OmegaConf for a wrapped env.

    Observation shape comes from the wrapper transform. ``seed=None`` keeps
    the saved seed or upstream default.
    """
    cfg_to_dataclass = import_upstream("common.parser", "cfg_to_dataclass")

    upstream_cfg = OmegaConf.load(upstream_config_path())
    OmegaConf.set_struct(upstream_cfg, False)

    if saved_config is not None:
        merged_cfg = OmegaConf.merge(upstream_cfg, OmegaConf.load(saved_config))
    elif agent_cfg:
        merged_cfg = OmegaConf.merge(upstream_cfg, OmegaConf.create(agent_cfg))
    else:
        merged_cfg = upstream_cfg

    obs_shape = env.obs_shape
    action_dim = int(env.action_space.shape[0])
    episode_length = int(env.max_episode_steps)

    merged_cfg.task = task_name
    merged_cfg.action_dim = action_dim
    merged_cfg.episode_length = episode_length
    merged_cfg.seed_steps = max(1000, 5 * episode_length)
    merged_cfg.num_envs = env.num_envs
    if seed is not None:
        merged_cfg.seed = int(seed)
    elif merged_cfg.get("seed") in (None, "???"):
        merged_cfg.seed = 0

    # Resolve fields normally supplied by Hydra's ``parse_cfg``.
    merged_cfg.task_title = task_name.replace("-", " ").replace("_", " ").title()
    merged_cfg.bin_size = (merged_cfg.vmax - merged_cfg.vmin) / (
        merged_cfg.num_bins - 1
    )
    merged_cfg.multitask = False
    merged_cfg.task_dim = 0

    # Resolve ending mode per task; upstream requires episodic mode for terminals.
    episodic = resolve_episodic(task_name, merged_cfg.get("episodic"))
    merged_cfg.episodic = episodic

    # Replace unresolved config markers.
    if merged_cfg.get("wandb_project") == "???":
        merged_cfg.wandb_project = "none"
    if merged_cfg.get("wandb_entity") == "???":
        merged_cfg.wandb_entity = "none"
    if merged_cfg.get("data_dir") == "???":
        merged_cfg.data_dir = str(data_dir or work_dir)
    if merged_cfg.get("checkpoint") == "???":
        merged_cfg.checkpoint = None

    merged_cfg[SRB_CONFIG_KEY] = {
        "upstream": upstream_pin_record(),
        "ending_semantics": ending_semantics_record(task_name, episodic),
        "observation": env.state_transform.record(),
    }

    # Apply upstream model-size presets.
    model_size = merged_cfg.get("model_size")
    if model_size is not None and model_size != "???":
        model_size_presets = import_upstream("common", "MODEL_SIZE")
        if model_size in model_size_presets:
            for key, value in model_size_presets[model_size].items():
                merged_cfg[key] = value

    cfg = cfg_to_dataclass(merged_cfg)

    # Set fields OmegaConf cannot represent.
    cfg.work_dir = Path(work_dir)
    cfg.obs_shape = obs_shape
    cfg.tasks = [task_name]
    cfg.obs_shapes = {task_name: obs_shape}
    cfg.action_dims = {task_name: action_dim}
    cfg.episode_lengths = [episode_length]

    if cfg.obs_shape != env.obs_shape:
        raise ValueError(
            f"TD-MPC2 cfg.obs_shape {cfg.obs_shape} does not match the "
            f"observation transform {env.obs_shape}"
        )
    return cfg, merged_cfg


def saved_observation_record(saved_config: Path | None) -> dict[str, Any] | None:
    """Return saved observation metadata, or ``None`` when absent."""
    if saved_config is None:
        return None
    saved_cfg = OmegaConf.load(saved_config)
    block = saved_cfg.get(SRB_CONFIG_KEY)
    if not block:
        return None
    record = block.get("observation")
    if record is None:
        return None
    return dict(OmegaConf.to_container(record, resolve=True))  # type: ignore[arg-type]


__all__ = [
    "SRB_CONFIG_KEY",
    "build_tdmpc2_config",
    "saved_observation_record",
    "upstream_config_path",
]
