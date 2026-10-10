"""CLI adapter for ``srb agent collect`` workflows."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Sequence

from srb.cli.commands.agent_policy import (
    load_policy as _load_policy,
)
from srb.cli.commands.agent_policy import (
    resolve_model_artifact_policy_inputs as _resolve_model_artifact_policy_inputs,
)
from srb.cli.commands.agent_policy import (
    resolve_model_artifact_reference as _resolve_model_artifact_reference,
)
from srb.cli.commands.agent_policy import (
    resolve_model_path as _resolve_model_path,
)
from srb.experiments.policies import infer_algo_from_model
from srb.interfaces.runtime import find_env_interface_node

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv
    from srb.interfaces import TeleopDeviceType

_DEFAULT_TELEOP_DEVICES = ("keyboard",)


def _resolve_teleop_devices(
    env: "AnyEnv",
    requested: Sequence[str] | None,
) -> list["TeleopDeviceType"]:
    """Resolve CLI teleop devices, falling back to environment defaults."""

    from srb.interfaces import TeleopDeviceType

    names = tuple(requested or ())
    if not names:
        cfg = getattr(getattr(env, "unwrapped", env), "cfg", None)
        configured = getattr(cfg, "teleop_devices", None)
        # Isaac Lab's managed configs hold a ``DevicesCfg`` here, not device names.
        if not isinstance(configured, (list, tuple)):
            configured = None
        names = tuple(configured or _DEFAULT_TELEOP_DEVICES)

    devices: list[TeleopDeviceType] = []
    for name in names:
        device = (
            name
            if isinstance(name, TeleopDeviceType)
            else TeleopDeviceType.from_str(str(name))
        )
        if device is None:
            raise ValueError(f"Unsupported teleop device: {name!r}")
        if device not in devices:
            devices.append(device)
    return devices


def collect_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    headless: bool = False,
    teleop_device: Sequence[str] | None = None,
    pos_sensitivity: float = 1.0,
    rot_sensitivity: float = 3.1415927,
    algo: str | None = None,
    model: "Path | str | None" = None,
    collector: str | None = None,
    num_episodes: int = 100,
    horizon: int = 400,
    success_only: bool = False,
    invert_controls: bool = True,
    logdir: "Path | None" = None,
    env_cfg: "Any" = None,
    dataset_split: str | None = None,
    **kwargs,
) -> None:
    """Collect demonstrations from a policy or teleoperation."""
    model_artifact: Path | str | None = None
    artifact_reference = _resolve_model_artifact_reference(algo=algo, model=model)
    if artifact_reference is not None:
        model_artifact = model
        algo = artifact_reference.algo
        model = artifact_reference.checkpoint_path
    else:
        algo, model = _resolve_model_artifact_policy_inputs(algo=algo, model=model)
    collect_source = _resolve_collect_source(collector, algo, model)
    policy_kwargs = dict(kwargs)
    if model_artifact is not None:
        policy_kwargs.setdefault("model_artifact", str(model_artifact))
    dataset_split_kwargs = (
        {"dataset_split": dataset_split} if dataset_split is not None else {}
    )
    if collect_source == "policy" and algo:
        if _supports_policy_collection(algo):
            _collect_agent_policy(
                env=env,
                sim_app=sim_app,
                algo=algo,
                model=model,
                num_episodes=num_episodes,
                horizon=horizon,
                success_only=success_only,
                logdir=logdir,
                env_cfg=env_cfg,
                **dataset_split_kwargs,
                **policy_kwargs,
            )
        else:
            raise ValueError(
                f"Unsupported algorithm for collection: {algo}. "
                "Supported: dreamer, tdmpc2, rsl_rl_*, sb3_*, sbx_*, "
                "skrl_*, robomimic_*"
            )
    elif collect_source == "policy" and model:
        _collect_agent_policy(
            env=env,
            sim_app=sim_app,
            algo=infer_algo_from_model(model),
            model=model,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            logdir=logdir,
            env_cfg=env_cfg,
            **dataset_split_kwargs,
            **policy_kwargs,
        )
    elif collect_source in ("random", "zero"):
        _collect_agent_builtin(
            env=env,
            sim_app=sim_app,
            collector=collect_source,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            logdir=logdir,
            env_cfg=env_cfg,
            **dataset_split_kwargs,
        )
    else:
        _collect_agent_teleop(
            env=env,
            sim_app=sim_app,
            headless=headless,
            teleop_device=teleop_device,
            pos_sensitivity=pos_sensitivity,
            rot_sensitivity=rot_sensitivity,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            invert_controls=invert_controls,
            logdir=logdir,
            env_cfg=env_cfg,
            **dataset_split_kwargs,
            **kwargs,
        )


def _supports_policy_collection(algo: str) -> bool:
    from srb.experiments.policies import supports_policy_collection

    return supports_policy_collection(algo)


def _resolve_collect_source(
    collector: str | None,
    algo: str | None,
    model: "Path | str | None",
) -> Literal["human", "policy", "random", "zero"]:
    if collector is None:
        return "policy" if algo or model else "human"

    if collector not in ("human", "policy", "random", "zero"):
        raise ValueError(f"Unsupported collector: {collector}")
    if collector == "policy" and not (algo or model):
        raise ValueError("policy collection requires --algo or --model")
    if collector == "human" and (algo or model):
        raise ValueError("human collection cannot be combined with --algo or --model")
    return collector  # type: ignore[return-value]


def _dataset_env_kwargs(
    env: "AnyEnv",
    env_cfg: "Any",
    *,
    split: str,
    controller: str,
    algo: str | None = None,
    model: "Path | str | None" = None,
    model_artifact: "Path | str | None" = None,
) -> dict[str, object]:
    from srb.datasets.provenance import dataset_provenance_from_env

    env_kwargs = dataset_provenance_from_env(
        env=env,
        env_cfg=env_cfg,
        split=split,
        controller=controller,
        algo=algo,
        model=model,
        model_artifact=model_artifact,
    ).to_env_kwargs()
    recording_reference = _recording_artifact_reference(env)
    if recording_reference is not None:
        env_kwargs["recording_artifacts"] = recording_reference
    return env_kwargs


def _recording_artifact_reference(env: "AnyEnv") -> dict[str, object] | None:
    """Return run-level visual/event provenance when the env owns a recorder."""
    unwrapped = getattr(env, "unwrapped", env)
    for attribute in ("_episode_recorder", "_video_recorder"):
        recorder = getattr(unwrapped, attribute, None)
        getter = getattr(recorder, "recording_artifact_reference", None)
        if callable(getter):
            if getattr(recorder, "persists_artifacts", True) is False:
                continue
            reference = getter()
            if isinstance(reference, dict) and reference:
                return dict(reference)
    return None


def _robomimic_action_space(wrapped_env: object, env: "AnyEnv") -> object | None:
    get_action_space = getattr(wrapped_env, "get_action_space", None)
    if callable(get_action_space):
        return get_action_space()
    return getattr(getattr(env, "unwrapped", env), "single_action_space", None)


def _collect_agent_policy(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    algo: str,
    model: "Path | str | None",
    num_episodes: int,
    horizon: int,
    success_only: bool,
    logdir: "Path",
    env_cfg: "Any" = None,
    dataset_split: str | None = None,
    **kwargs,
) -> None:
    """Collect demonstrations by rolling out a trained policy."""
    from datetime import datetime
    from pathlib import Path

    from srb.datasets.collection import PolicyActionSource, run_robomimic_collection
    from srb.utils import logging

    model_path = _resolve_model_path(algo, model, logdir)
    logging.info(f"Loading policy from: {model_path}")

    policy_fn, start_episode_fn, deterministic = _load_policy(
        algo,
        model_path,
        env,
        agent_cfg=kwargs.get("agent_cfg"),
        env_cfg=env_cfg,
        logdir=logdir,
    )

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_path = Path(logdir).joinpath("datasets", f"demos_{algo}_{timestamp}.hdf5")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    seed = env_cfg.seed if env_cfg and hasattr(env_cfg, "seed") else 42

    print("\n" + "=" * 60)
    print("POLICY-BASED DATA COLLECTION")
    print("=" * 60)
    print(f"  Algorithm: {algo}")
    print(f"  Model: {model_path}")
    print(f"  Output: {output_path}")
    print(f"  Target episodes: {num_episodes}")
    print(f"  Max steps/episode: {horizon}")
    print(f"  Success only: {success_only}")
    print(f"  Deterministic: {deterministic}")
    print("=" * 60 + "\n")

    run_robomimic_collection(
        env=env,
        sim_app=sim_app,
        source=PolicyActionSource(
            algo=algo,
            model_path=Path(model_path),
            policy_fn=policy_fn,
            start_episode_fn=start_episode_fn,
            deterministic=deterministic,
        ),
        output_path=output_path,
        num_episodes=num_episodes,
        horizon=horizon,
        success_only=success_only,
        env_cfg=env_cfg,
        seed=seed,
        max_demos=num_episodes * 2,
        env_kwargs=_dataset_env_kwargs(
            env,
            env_cfg,
            split=dataset_split or "policy",
            controller="policy",
            algo=algo,
            model=model_path,
            model_artifact=kwargs.get("model_artifact"),
        ),
    )


def _collect_agent_builtin(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    collector: Literal["random", "zero"],
    num_episodes: int,
    horizon: int,
    success_only: bool,
    logdir: "Path",
    env_cfg: "Any" = None,
    dataset_split: str | None = None,
) -> None:
    """Collect demonstrations using built-in random or zero action sources."""
    from datetime import datetime
    from pathlib import Path

    from srb.datasets.collection import BuiltinActionSource, run_robomimic_collection

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_path = Path(logdir).joinpath(
        "datasets",
        f"demos_{collector}_{timestamp}.hdf5",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 60)
    print(f"{collector.upper()} DATA COLLECTION")
    print("=" * 60)
    print(f"  Output: {output_path}")
    print(f"  Target episodes: {num_episodes}")
    print(f"  Max steps/episode: {horizon}")
    print(f"  Success only: {success_only}")
    print("=" * 60 + "\n")

    run_robomimic_collection(
        env=env,
        sim_app=sim_app,
        source=BuiltinActionSource(collector=collector),
        output_path=output_path,
        num_episodes=num_episodes,
        horizon=horizon,
        success_only=success_only,
        env_cfg=env_cfg,
        # Seed random actions from the env for reproducible baseline data.
        seed=getattr(env_cfg, "seed", None),
        max_demos=num_episodes * 2,
        env_kwargs=_dataset_env_kwargs(
            env,
            env_cfg,
            split=dataset_split or collector,
            controller=collector,
        ),
    )


def _collect_agent_teleop(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    headless: bool,
    teleop_device: Sequence[str] | None,
    pos_sensitivity: float,
    rot_sensitivity: float,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    invert_controls: bool,
    logdir: "Path",
    env_cfg: "Any" = None,
    dataset_split: str | None = None,
    **kwargs,
) -> None:
    """Collect demonstrations via teleoperation."""
    if _is_real_env(env):
        _collect_agent_real_teleop(
            env=env,
            sim_app=sim_app,
            teleop_device=teleop_device,
            pos_sensitivity=pos_sensitivity,
            rot_sensitivity=rot_sensitivity,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            invert_controls=invert_controls,
            logdir=logdir,
            env_cfg=env_cfg,
            dataset_split=dataset_split,
            real_agent_session=kwargs.get("real_agent_session"),
        )
        return

    from datetime import datetime
    from pathlib import Path

    from srb.core.action.command import env_supports_direct_action_command
    from srb.core.env import ManipulationEnv
    from srb.datasets.collection import run_robomimic_teleop_collection
    from srb.interfaces import TeleopDeviceType
    from srb.interfaces.teleop import CombinedTeleopInterface

    teleop_device = _resolve_teleop_devices(env, teleop_device)
    if (
        headless
        and len(teleop_device) == 1
        and TeleopDeviceType.KEYBOARD in teleop_device
    ):
        raise ValueError(
            "Data collection with keyboard teleop is only supported in GUI mode. "
            'Rerun without "--headless" or choose a non-keyboard "--teleop_device".'
        )

    if hasattr(env.unwrapped.cfg, "truncate_episodes"):  # type: ignore[misc]
        env.unwrapped.cfg.truncate_episodes = False  # type: ignore[attr-defined]
    ros_node = find_env_interface_node(env)

    teleop_interface = CombinedTeleopInterface(
        devices=teleop_device,  # type: ignore[misc]
        node=ros_node,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
        actions=env.unwrapped.cfg.actions,  # type: ignore[attr-defined]
    )

    if not env_supports_direct_action_command(env):
        raise ValueError(
            "Data collection via teleop requires an environment that supports "
            "direct teleoperation. Consider using an RL policy for collection instead."
        )

    if invert_controls:
        invert_controls = isinstance(env.unwrapped, ManipulationEnv)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_path = Path(logdir).joinpath("datasets", f"demos_{timestamp}.hdf5")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    teleop_interface.reset()
    print(teleop_interface)
    print("\n" + "=" * 60)
    print("DATA COLLECTION MODE")
    print("=" * 60)
    print(f"  Output: {output_path}")
    print(f"  Target episodes: {num_episodes}")
    print(f"  Max steps/episode: {horizon}")
    print(f"  Success only: {success_only}")
    print("-" * 60)
    print("Controls:")
    print("  L        - Reset episode (discard current)")
    print("  SPACE    - Save current episode")
    print("  BACKSPACE- Discard current episode")
    print("=" * 60 + "\n")

    run_robomimic_teleop_collection(
        env=env,
        sim_app=sim_app,
        teleop_interface=teleop_interface,
        output_path=output_path,
        num_episodes=num_episodes,
        horizon=horizon,
        success_only=success_only,
        invert_controls=invert_controls,
        env_cfg=env_cfg,
        env_kwargs=_dataset_env_kwargs(
            env,
            env_cfg,
            split=dataset_split or "human",
            controller="human",
        ),
    )


def _is_real_env(env: Any) -> bool:
    """Recognize an already constructed RealEnv without importing Isaac."""
    import sys

    module = sys.modules.get("srb.interfaces.sim_to_real.core.env")
    real_env_type = getattr(module, "RealEnv", None)
    return isinstance(real_env_type, type) and isinstance(
        getattr(env, "unwrapped", env), real_env_type
    )


def _collect_agent_real_teleop(
    *,
    env: Any,
    sim_app: Any,
    teleop_device: Sequence[str] | None,
    pos_sensitivity: float,
    rot_sensitivity: float,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    invert_controls: bool,
    logdir: Path,
    env_cfg: Any,
    dataset_split: str | None,
    real_agent_session: Any,
) -> None:
    """Collect scalar RealEnv demonstrations from a native input source."""
    if real_agent_session is None:
        raise RuntimeError(
            "Human collection with RealEnv requires a RealAgentSession owner"
        )

    from datetime import datetime

    from srb.cli.commands.real_agent_teleop import prepare_real_teleop
    from srb.datasets.collection import run_robomimic_teleop_collection

    real_env = getattr(env, "unwrapped", env)
    teleop_interface = prepare_real_teleop(
        real_env,
        session=real_agent_session,
        teleop_device=teleop_device,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
    )
    mapping = real_env.command_mapping
    if mapping is None:
        raise ValueError("RealEnv does not declare direct command mapping metadata")

    output_path = Path(logdir).joinpath(
        "datasets", f"demos_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.hdf5"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    teleop_interface.reset()
    print(teleop_interface)
    print("\n" + "=" * 60)
    print("DATA COLLECTION MODE")
    print("=" * 60)
    print(f"  Output: {output_path}")
    print(f"  Target episodes: {num_episodes}")
    print(f"  Max steps/episode: {horizon}")
    print(f"  Success only: {success_only}")
    print("-" * 60)
    print("Controls:")
    controls = teleop_interface.callback_keys
    if controls:
        print("  SpaceMouse L/R/LR buttons (L resets; R toggles gripper)")
    else:
        print("  No manual device buttons available (ROS/haptic input)")
    if "SPACE" not in controls and "BACKSPACE" not in controls:
        print(
            "  Manual save/discard unavailable; episodes end at the environment or horizon."
        )
    print("=" * 60 + "\n")

    run_robomimic_teleop_collection(
        env=env,
        sim_app=sim_app,
        teleop_interface=teleop_interface,
        output_path=output_path,
        num_episodes=num_episodes,
        horizon=horizon,
        success_only=success_only,
        invert_controls=invert_controls and mapping.task_family == "manipulation",
        env_cfg=env_cfg,
        env_kwargs=_dataset_env_kwargs(
            env,
            env_cfg,
            split=dataset_split or "human",
            controller="human",
        ),
    )


__all__ = (
    "collect_agent",
    "_collect_agent_builtin",
    "_collect_agent_policy",
    "_collect_agent_teleop",
    "_dataset_env_kwargs",
    "_resolve_collect_source",
    "_resolve_teleop_devices",
    "_robomimic_action_space",
)
