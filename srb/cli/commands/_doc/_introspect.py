"""Introspection engine -- extract metadata from robots, tasks, objects, and scenery."""

from __future__ import annotations

import dataclasses
import inspect
import re
from dataclasses import dataclass, field
from typing import Any

ActuatorValue = float | tuple[float, float] | None
"""A scalar actuator value, or the ``(min, max)`` span of a per-joint mapping."""


@dataclass(frozen=True)
class ActuatorInfo:
    group_name: str
    joint_names_expr: list[str]
    effort_limit: ActuatorValue = None
    velocity_limit: ActuatorValue = None
    stiffness: ActuatorValue = None
    damping: ActuatorValue = None


@dataclass(frozen=True)
class SensorInfo:
    name: str
    sensor_type: str
    location: str


@dataclass(frozen=True)
class ActionModeInfo:
    key: str
    is_default: bool
    group_class_name: str
    controller_type: str
    dims: int
    scale: str


@dataclass
class RobotInfo:
    name: str
    display_name: str
    robot_type: str
    category: str
    dof: int
    end_effector: str
    frame_base: str
    frame_flange: str
    sensor_count: int
    action_mode_count: int
    default_action_mode: str
    source_path: str
    actuators: list[ActuatorInfo] = field(default_factory=list)
    sensors: list[SensorInfo] = field(default_factory=list)
    action_modes: list[ActionModeInfo] = field(default_factory=list)
    init_joint_pos: dict[str, float] = field(default_factory=dict)
    mass_kg: float | None = None
    page_path: str = ""
    anchor: str = ""
    _cls: type | None = field(default=None, repr=False)


_PAGE_MAP: dict[str, str] = {
    "manipulator": "assets/robots/manipulators.md",
    "mobile_robot": "assets/robots/mobile_robots.md",
    "mobile_manipulator": "assets/robots/mobile_manipulators.md",
    "tool": "assets/robots/tools.md",
}


def _snake_to_title(name: str) -> str:
    """Convert ``foo_bar`` to ``Foo Bar``."""
    return name.strip("_").replace("_", " ").title()


def _extract_controller_type(action_cfg: Any) -> str:
    """Derive a human-readable controller type from an action term config."""
    cls_name = type(action_cfg).__name__
    label = cls_name
    for suffix in ("ActionCfg", "Cfg"):
        if label.endswith(suffix):
            label = label[: -len(suffix)]
            break
    label = re.sub(r"(?<=[a-z])(?=[A-Z])", " ", label)
    return label


def _extract_action_dims(action_group: Any) -> int:
    """Count declared widths; IK/OSC pose is 6-DOF, impedance adds channels, joint terms
    use patterns; unknown terms make group unknown.
    """
    total_dims = 0
    for f in dataclasses.fields(action_group):
        term_cfg = getattr(action_group, f.name, None)
        if term_cfg is None:
            continue
        dims = _extract_action_term_dims(term_cfg)
        if dims < 0:
            return -1
        total_dims += dims

    return total_dims if total_dims > 0 else -1


def _osc_impedance_extra_dims(controller_cfg: Any) -> tuple[int, int]:
    """Return OSC impedance channels as (stiffness, damping) dimensions."""
    impedance = getattr(controller_cfg, "impedance_mode", "fixed")
    if impedance == "variable_kp":
        return 6, 0
    if impedance == "variable":
        return 6, 6
    return 0, 0


def _static_action_dim(term_cfg: Any) -> int | None:
    """Return a term class's declared ACTION_DIM, if any."""
    dim = getattr(getattr(term_cfg, "class_type", None), "ACTION_DIM", None)
    if isinstance(dim, int) and not isinstance(dim, bool) and dim > 0:
        return dim
    return None


def _is_joint_space_term(term_cfg: Any) -> bool:
    """True for a joint-space term whose width is the matched joint count."""
    return (
        _static_action_dim(term_cfg) is None
        and getattr(term_cfg, "controller_cfg", None) is None
        and getattr(term_cfg, "controller", None) is None
        and getattr(term_cfg, "thrusters", None) is None
        and isinstance(getattr(term_cfg, "joint_names", None), (list, tuple, str))
    )


def _is_joint_space_group(action_group: Any) -> bool:
    terms = [
        getattr(action_group, f.name, None) for f in dataclasses.fields(action_group)
    ]
    terms = [term for term in terms if term is not None]
    return bool(terms) and all(_is_joint_space_term(term) for term in terms)


def _extract_action_term_dims(term_cfg: Any) -> int:
    static_dim = _static_action_dim(term_cfg)
    if static_dim is not None:
        return static_dim

    controller_cfg = getattr(term_cfg, "controller_cfg", None)
    if controller_cfg is not None:
        kp_dims, kd_dims = _osc_impedance_extra_dims(controller_cfg)
        return 6 + kp_dims + kd_dims  # 6 = pose_rel

    controller = getattr(term_cfg, "controller", None)
    if controller is not None:
        cmd_type = getattr(controller, "command_type", "pose")
        return 6 if cmd_type == "pose" else 3

    thrusters = getattr(term_cfg, "thrusters", None)
    if thrusters is not None:
        dims = 0
        for thruster in thrusters:
            dims += 1
            gimbal_limits = getattr(thruster, "gimbal_limits", None)
            if gimbal_limits not in (None, (0.0, 0.0)):
                dims += 2
        return dims if dims > 0 else -1

    joint_names = getattr(term_cfg, "joint_names", None)
    if isinstance(joint_names, (list, tuple)):
        if len(joint_names) == 1 and joint_names[0] == ".*":
            return -1
        return len(joint_names)
    return -1


def _extract_scale(action_group: Any) -> str:
    """Extract scale values from action terms in the group."""
    scales: list[tuple[str, str]] = []
    for f in dataclasses.fields(action_group):
        term_cfg = getattr(action_group, f.name, None)
        if term_cfg is None:
            continue
        if scale := _extract_action_term_scale(term_cfg):
            scales.append((f.name, scale))
    if not scales:
        return ""
    if len(scales) == 1:
        return scales[0][1]
    return "; ".join(
        f"`{name}`: {scale}" if "," in scale else f"`{name}`={scale}"
        for name, scale in scales
    )


def _extract_action_term_scale(term_cfg: Any) -> str:
    scale = getattr(term_cfg, "scale", None)
    if scale is not None:
        return str(scale)
    pos_scale = getattr(term_cfg, "position_scale", None)
    ori_scale = getattr(term_cfg, "orientation_scale", None)
    if pos_scale is not None or ori_scale is not None:
        parts = []
        if pos_scale is not None:
            parts.append(f"pos={pos_scale}")
        if ori_scale is not None:
            parts.append(f"ori={ori_scale}")
        # Keep advertised impedance scales aligned with the counted channels.
        controller_cfg = getattr(term_cfg, "controller_cfg", None)
        kp_dims, kd_dims = _osc_impedance_extra_dims(controller_cfg)
        if kp_dims:
            stiff_scale = getattr(term_cfg, "stiffness_scale", None)
            if stiff_scale is not None:
                parts.append(f"stiffness={stiff_scale}")
        if kd_dims:
            damp_scale = getattr(term_cfg, "damping_ratio_scale", None)
            if damp_scale is not None:
                parts.append(f"damping={damp_scale}")
        return ", ".join(parts)
    # Filtered drives scale normalized actions by their velocity limits instead.
    lin_scale = getattr(term_cfg, "v_lin_max", getattr(term_cfg, "scale_linear", None))
    ang_scale = getattr(term_cfg, "v_ang_max", getattr(term_cfg, "scale_angular", None))
    if lin_scale is not None or ang_scale is not None:
        parts = []
        if lin_scale is not None:
            parts.append(f"lin={lin_scale}")
        if ang_scale is not None:
            parts.append(f"ang={ang_scale}")
        return ", ".join(parts)
    return ""


def _extract_sensor_type(cfg: Any) -> str:
    """Human-readable sensor type from a sensor config object."""
    cls_name = type(cfg).__name__
    for suffix in ("Cfg",):
        if cls_name.endswith(suffix):
            cls_name = cls_name[: -len(suffix)]
    return re.sub(r"(?<=[a-z])(?=[A-Z])", " ", cls_name)


def _extract_sensor_location(cfg: Any) -> str:
    """Extract a mount location hint from the sensor's prim_path."""
    prim_path: str = getattr(cfg, "prim_path", "")
    if not prim_path:
        return ""
    parts = prim_path.strip("/").split("/")
    if len(parts) > 2:
        return "/".join(parts[2:])
    return parts[-1] if parts else ""


def _categorize_robot(cls: type, robot_type: str) -> str:
    """Categorize robots by hierarchy, then by manufacturer for manipulators."""
    _METACLASS_LABELS: dict[str, str] = {
        "WheeledRobot": "Wheeled Robots",
        "LeggedRobot": "Legged Robots",
        "Multicopter": "Multicopters",
        "Lander": "Landers",
        "SurfaceVehicle": "Surface Vehicles",
        "Humanoid": "Humanoids",
        "CombinedMobileManipulator": "Combined Platforms",
    }

    _ENTRYPOINT_LABELS: dict[str, str] = {
        "GroundRobot": "Ground Robots",
        "AerialRobot": "Aerial Robots",
        "OrbitalRobot": "Orbital Robots",
        "AquaticRobot": "Aquatic Robots",
        "MobileManipulator": "Mobile Manipulators",
        "MobileRobot": "Mobile Robots",
    }

    for base in cls.__mro__:
        name = base.__name__
        if name in _METACLASS_LABELS:
            return _METACLASS_LABELS[name]
    for base in cls.__mro__:
        name = base.__name__
        if name in _ENTRYPOINT_LABELS:
            return _ENTRYPOINT_LABELS[name]

    if robot_type == "manipulator":
        if any(b.__name__ == "MultiManipulator" for b in cls.__mro__):
            return "Multi-Arm"
        module = cls.__module__.lower()
        # UR arms need an explicit module-name check.
        if module.rsplit(".", 1)[-1] == "universal_robots":
            return "Universal Robots"
        if "franka" in module:
            return "Franka"
        if "kinova" in module:
            return "Kinova"
        if "unitree" in module:
            return "Unitree"
        if "so_arm" in module:
            return "SO-ARM100"
        if "canadarm" in module or "vispa" in module:
            return "Space"
        return "Other"

    return _snake_to_title(robot_type)


def _first_set(obj: Any, *names: str) -> Any:
    """Return the first attribute among *names* that is not ``None``."""
    for name in names:
        value = getattr(obj, name, None)
        if value is not None:
            return value
    return None


def _get_field_default(cls: type, field_name: str) -> Any:
    """Get the default value for a Pydantic model field, or None."""
    fi = cls.model_fields.get(field_name)  # type: ignore[attr-defined]
    if fi is None:
        return None
    return fi.default


def _count_dof(cls: type) -> int:
    """Estimate DOF from explicit initial joints, then actuator patterns; wildcards are
    unknown.
    """
    asset_cfg = _get_field_default(cls, "asset_cfg")
    if asset_cfg is None:
        return 0

    init_state = getattr(asset_cfg, "init_state", None)
    if init_state is not None:
        joint_pos = getattr(init_state, "joint_pos", None)
        if isinstance(joint_pos, dict) and joint_pos:
            if not any("*" in k or "?" in k or "|" in k for k in joint_pos):
                return len(joint_pos)

    actuators = getattr(asset_cfg, "actuators", None)
    if not actuators:
        return 0
    total = 0
    has_wildcard = False
    for act_cfg in actuators.values():
        exprs = getattr(act_cfg, "joint_names_expr", [])
        for expr in exprs:
            m = re.search(r"\[(\d+)-(\d+)\]", expr)
            if m:
                total += int(m.group(2)) - int(m.group(1)) + 1
            elif "*" in expr or "?" in expr or "|" in expr:
                has_wildcard = True
            else:
                total += 1
    # Wildcards make the count a lower bound, so report it as unknown.
    if has_wildcard:
        return 0
    return total


def _scalar_or_none(val: object) -> ActuatorValue:
    """Coerce scalars to float; per-joint mappings give their single value, or the
    ``(min, max)`` span when the values differ.
    """
    if val is None:
        return None
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, dict):
        nums = [float(v) for v in val.values() if isinstance(v, (int, float))]
        if not nums:
            return None
        lo, hi = min(nums), max(nums)
        return lo if lo == hi else (lo, hi)
    return None


def _extract_actuators(cls: type) -> list[ActuatorInfo]:
    """Extract actuator info from the asset_cfg default."""
    asset_cfg = _get_field_default(cls, "asset_cfg")
    if asset_cfg is None:
        return []
    actuators = getattr(asset_cfg, "actuators", None)
    if not actuators:
        return []
    result = []
    for group_name, act_cfg in actuators.items():
        result.append(
            ActuatorInfo(
                group_name=group_name,
                joint_names_expr=list(getattr(act_cfg, "joint_names_expr", [])),
                effort_limit=_scalar_or_none(
                    _first_set(act_cfg, "effort_limit_sim", "effort_limit")
                ),
                velocity_limit=_scalar_or_none(
                    _first_set(act_cfg, "velocity_limit_sim", "velocity_limit")
                ),
                stiffness=_scalar_or_none(getattr(act_cfg, "stiffness", None)),
                damping=_scalar_or_none(getattr(act_cfg, "damping", None)),
            )
        )
    return result


def _extract_sensors(cls: type) -> list[SensorInfo]:
    """Extract sensor info from the sensor_cfgs default."""
    sensor_cfgs = _get_field_default(cls, "sensor_cfgs")
    if not sensor_cfgs:
        return []
    result = []
    for name, cfg in sensor_cfgs.items():
        result.append(
            SensorInfo(
                name=name,
                sensor_type=_extract_sensor_type(cfg),
                location=_extract_sensor_location(cfg),
            )
        )
    return result


def _extract_action_modes(cls: type, dof: int = 0) -> list[ActionModeInfo]:
    """Extract action modes; use robot DOF only for all-joint-space groups."""
    action_modes: dict[str, Any] | None = getattr(cls, "action_modes", None)
    if not action_modes:
        return []
    result = []
    for idx, (key, group) in enumerate(action_modes.items()):
        dims = _extract_action_dims(group)
        if dims == -1 and dof > 0 and _is_joint_space_group(group):
            dims = dof
        result.append(
            ActionModeInfo(
                key=key,
                is_default=(idx == 0),
                group_class_name=type(group).__name__,
                controller_type=_extract_controller_type_from_group(group),
                dims=dims,
                scale=_extract_scale(group),
            )
        )
    return result


def _extract_controller_type_from_group(group: Any) -> str:
    """Extract controller type from the first action term in a group."""
    for f in dataclasses.fields(group):
        term_cfg = getattr(group, f.name, None)
        if term_cfg is None:
            continue
        return _extract_controller_type(term_cfg)
    return "Unknown"


def _extract_init_joint_pos(cls: type) -> dict[str, float]:
    """Extract initial joint positions, keyed by joint name or regex pattern; omit
    the all-zero ``{".*": 0.0}`` default.
    """
    asset_cfg = _get_field_default(cls, "asset_cfg")
    if asset_cfg is None:
        return {}
    init_state = getattr(asset_cfg, "init_state", None)
    if init_state is None:
        return {}
    joint_pos = getattr(init_state, "joint_pos", None)
    if not isinstance(joint_pos, dict) or joint_pos == {".*": 0.0}:
        return {}
    return dict(joint_pos)


def _extract_mass(cls: type) -> float | None:
    """Return the declared ``spawn.mass_props.mass``; None when unset."""
    asset_cfg = _get_field_default(cls, "asset_cfg")
    spawn = getattr(asset_cfg, "spawn", None)
    mass = getattr(getattr(spawn, "mass_props", None), "mass", None)
    if isinstance(mass, (int, float)) and not isinstance(mass, bool) and mass > 0:
        return float(mass)
    return None


def _extract_frame(cls: type, field_name: str) -> str:
    """Extract a frame's prim_relpath from a model field default."""
    frame = _get_field_default(cls, field_name)
    if frame is None:
        return ""
    return getattr(frame, "prim_relpath", "")


def _extract_end_effector(cls: type) -> str:
    """Get end-effector class name from the default field value."""
    ee = _get_field_default(cls, "end_effector")
    if ee is None:
        return "None"
    return type(ee).__name__


def introspect_robots() -> dict[str, list[RobotInfo]]:
    """Return metadata for registered robots grouped by type."""
    import srb.assets.robot  # noqa: F401 — trigger concrete class registration
    from srb.core.asset.robot.robot import RobotRegistry

    results: dict[str, list[RobotInfo]] = {}

    for robot_type, robot_classes in RobotRegistry.registry.items():
        type_str = str(robot_type)
        group: list[RobotInfo] = []

        for cls in robot_classes:
            robot_name = cls.name()
            if robot_name.startswith("_"):
                continue
            display_name = _snake_to_title(robot_name)
            category = _categorize_robot(cls, type_str)

            actuators = _extract_actuators(cls)
            sensors = _extract_sensors(cls)
            dof = _count_dof(cls)
            action_modes_list = _extract_action_modes(cls, dof)

            action_modes_dict: dict[str, Any] | None = getattr(
                cls, "action_modes", None
            )
            default_mode = ""
            if action_modes_dict:
                default_mode = next(iter(action_modes_dict))

            source_path = ""
            try:
                src = inspect.getfile(cls)
                source_path = src
            except (TypeError, OSError):
                pass

            page_path = _PAGE_MAP.get(type_str, "")
            anchor = robot_name

            group.append(
                RobotInfo(
                    name=robot_name,
                    display_name=display_name,
                    robot_type=type_str,
                    category=category,
                    dof=dof,
                    end_effector=_extract_end_effector(cls),
                    frame_base=_extract_frame(cls, "frame_base"),
                    frame_flange=_extract_frame(cls, "frame_flange"),
                    sensor_count=len(sensors),
                    action_mode_count=len(action_modes_list),
                    default_action_mode=default_mode,
                    source_path=source_path,
                    actuators=actuators,
                    sensors=sensors,
                    action_modes=action_modes_list,
                    init_joint_pos=_extract_init_joint_pos(cls),
                    mass_kg=_extract_mass(cls),
                    page_path=page_path,
                    anchor=anchor,
                    _cls=cls,
                )
            )

        results[type_str] = group

    return results


@dataclass
class RewardTermInfo:
    name: str
    term_type: str  # e.g. "Hyperbolic", "Gaussian"
    weight: float
    scale: float | None
    threshold: float | None
    notes: str  # from docstring


@dataclass
class TaskInfo:
    name: str
    display_name: str
    tagline: str
    task_category: str  # e.g. "manipulation", "mobile_robotics"
    episode_length_s: float
    truncate_episodes: bool
    env_rate: float
    agent_rate: float
    default_robot_name: str
    default_robot_class_name: str
    compatible_robot_base: str  # e.g. "Manipulator"
    compatible_robot_type: str  # e.g. "manipulator" for registry lookup
    _compatible_robot_cls: type | None = field(default=None, repr=False)
    _compatible_robot_members: tuple[tuple[str, type | None], ...] = field(
        default=(), repr=False
    )
    reward_terms: list[RewardTermInfo] = field(default_factory=list)
    reward_term_count: int = 0
    multi_variant: str = ""
    page_path: str = ""


@dataclass
class ObjectInfo:
    name: str
    display_name: str
    object_type: str
    category: str
    variant: str
    domains: list[str] = field(default_factory=list)
    density: float | None = None
    collision_approx: str = ""


@dataclass
class SceneryInfo:
    name: str
    display_name: str
    scenery_type: str
    category: str
    variant: str
    domains: list[str] = field(default_factory=list)
    sub_assets: list[str] = field(default_factory=list)


def _extract_task_category(task_cfg_cls: type) -> str:
    """Derive the task category from its module path."""
    module = task_cfg_cls.__module__
    parts = module.split(".")
    try:
        tasks_idx = parts.index("tasks")
        return parts[tasks_idx + 1]
    except (ValueError, IndexError):
        return "unknown"


def _extract_tagline(cls: type) -> str:
    """Return the first sentence of the class docstring, or empty string."""
    doc = inspect.getdoc(cls)
    if not doc:
        return ""
    for line in doc.splitlines():
        line = line.strip()
        if line:
            return line
    return ""


def _extract_reward_terms(reward_cfg: Any) -> list[RewardTermInfo]:
    """Extract reward term metadata from a Pydantic BaseModel reward config."""
    from srb.core.reward.terms import RewardTerm

    result: list[RewardTermInfo] = []
    reward_cls = type(reward_cfg)
    if not hasattr(reward_cls, "model_fields"):
        return result

    for field_name, field_info in reward_cls.model_fields.items():
        val = getattr(reward_cfg, field_name, None)
        if not isinstance(val, RewardTerm):
            continue

        term_type_name = type(val).__name__
        for suffix in ("RewardTerm",):
            if term_type_name.endswith(suffix):
                term_type_name = term_type_name[: -len(suffix)]

        notes = ""
        if field_info.description:
            notes = field_info.description
        elif hasattr(field_info, "metadata"):
            for meta in field_info.metadata:
                if isinstance(meta, str):
                    notes = meta
                    break

        result.append(
            RewardTermInfo(
                name=field_name,
                term_type=term_type_name,
                weight=val.weight,
                scale=getattr(val, "scale", None),
                threshold=getattr(val, "threshold", None),
                notes=notes,
            )
        )
    return result


def _resolve_compatible_robot_members(
    task_cfg_cls: type,
) -> list[tuple[str, str, type | None]]:
    """Resolve every robot annotation member; exclude AssetVariant, and keep concrete
    asset annotations restricted to that asset.
    """
    from srb.core.asset.asset_variant import AssetVariant
    from srb.core.asset.robot.manipulation.manipulator import ManipulatorRegistry
    from srb.core.asset.robot.mobile.mobile_robot import MobileRobotRegistry
    from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
        MobileManipulatorRegistry,
    )
    from srb.core.asset.robot.robot import RobotRegistry

    def _resolve_name(name: str) -> tuple[str, str, type | None]:
        """Resolve a class *name* to (name, robot_type_str, cls)."""
        for rtype, base_cls in RobotRegistry.base_types.items():
            if base_cls.__name__ == name:
                return name, str(rtype), base_cls
        for sub_reg, parent_type in (
            (MobileRobotRegistry, "mobile_robot"),
            (MobileManipulatorRegistry, "mobile_manipulator"),
            (ManipulatorRegistry, "manipulator"),
        ):
            for base_cls in sub_reg.base_types.values():
                if base_cls.__name__ == name:
                    return name, parent_type, base_cls
            for meta_cls in sub_reg.meta_types:
                if meta_cls.__name__ == name:
                    return name, parent_type, meta_cls
        for robot_type, robots in RobotRegistry.registry.items():
            for robot_cls in robots:
                if robot_cls.__name__ == name:
                    return name, str(robot_type), robot_cls
        return name, name.lower(), None

    def _resolve_type(arg: type) -> tuple[str, str, type | None]:
        base_name = arg.__name__
        for rtype, base_cls in sorted(RobotRegistry.base_types.items()):
            if issubclass(arg, base_cls):
                return base_name, str(rtype), arg
        return base_name, base_name.lower(), arg

    for f in dataclasses.fields(task_cfg_cls):
        if f.name != "robot":
            continue
        ann = f.type
        if isinstance(ann, str):
            return [
                _resolve_name(part)
                for part in (part.strip() for part in ann.split("|"))
                if part and "AssetVariant" not in part
            ]
        args = getattr(ann, "__args__", None)
        if args:
            return [
                _resolve_type(arg)
                for arg in args
                if isinstance(arg, type) and arg is not AssetVariant
            ]
        if isinstance(ann, type) and ann is not AssetVariant:
            return [_resolve_type(ann)]

    return []


def introspect_tasks() -> dict[str, list[TaskInfo]]:
    """Return metadata for registered tasks grouped by category."""
    import gymnasium

    from srb.core.asset.asset_variant import AssetVariant
    from srb.tasks import ensure_registered
    from srb.utils.registry import get_srb_tasks

    # Registration is lazy; docgen starts Isaac Sim before introspection.
    ensure_registered()
    task_map = get_srb_tasks()
    results: dict[str, list[TaskInfo]] = {}
    failures: list[str] = []

    multi_map: dict[str, str] = {}  # base_name → multi_name
    for short_name in task_map:
        if short_name.startswith("multi_"):
            base = short_name[len("multi_") :]
            multi_map[base] = short_name

    for short_name, full_id in task_map.items():
        try:
            spec = gymnasium.registry[full_id]
            task_cfg_cls = spec.kwargs.get("task_cfg")
            if task_cfg_cls is None:
                continue

            cfg = task_cfg_cls()

            category = _extract_task_category(task_cfg_cls)

            episode_length_s = getattr(cfg, "episode_length_s", 0.0)
            env_rate = getattr(cfg, "env_rate", 0.0)
            agent_rate = getattr(cfg, "agent_rate", 0.0)

            robot = getattr(cfg, "robot", None)
            default_robot_name = ""
            default_robot_class_name = ""
            if robot is not None and not isinstance(robot, AssetVariant):
                try:
                    default_robot_name = type(robot).name()
                except Exception:
                    default_robot_name = type(robot).__name__
                default_robot_class_name = type(robot).__name__

            compatible_members = _resolve_compatible_robot_members(task_cfg_cls)
            compatible_base, compatible_type, compatible_cls = (
                compatible_members[0]
                if compatible_members
                else ("Unknown", "unknown", None)
            )

            reward_cfg = getattr(cfg, "reward", None)
            reward_terms: list[RewardTermInfo] = []
            if reward_cfg is not None:
                reward_terms = _extract_reward_terms(reward_cfg)

            entry_point_str = spec.entry_point or ""
            tagline = ""
            if entry_point_str:
                try:
                    module_path, cls_name = entry_point_str.rsplit(":", 1)
                    import importlib

                    mod = importlib.import_module(module_path)
                    task_cls = getattr(mod, cls_name, None)
                    if task_cls is not None:
                        tagline = _extract_tagline(task_cls)
                except Exception:
                    pass
            if not tagline:
                tagline = _extract_tagline(task_cfg_cls)

            info = TaskInfo(
                name=short_name,
                display_name=_snake_to_title(short_name),
                tagline=tagline,
                task_category=category,
                episode_length_s=episode_length_s,
                truncate_episodes=bool(getattr(cfg, "truncate_episodes", True)),
                env_rate=env_rate,
                agent_rate=agent_rate,
                default_robot_name=default_robot_name,
                default_robot_class_name=default_robot_class_name,
                compatible_robot_base=compatible_base,
                compatible_robot_type=compatible_type,
                _compatible_robot_cls=compatible_cls,
                _compatible_robot_members=tuple(
                    (member_type, member_cls)
                    for _name, member_type, member_cls in compatible_members
                ),
                reward_terms=reward_terms,
                reward_term_count=len(reward_terms),
                multi_variant=multi_map.get(short_name, ""),
            )

            if category not in results:
                results[category] = []
            results[category].append(info)

        except Exception as exc:
            failures.append(
                f"task {short_name!r} (id={full_id!r}): {type(exc).__name__}: {exc}"
            )

    if failures:
        details = "\n- ".join(sorted(failures))
        raise RuntimeError(f"Configured task docs introspection failed:\n- {details}")

    # Sort explicitly because registry import order varies across machines.
    for tasks in results.values():
        tasks.sort(key=lambda info: info.name)
    return results


def _classify_spawn_variant(spawn: Any) -> str:
    """Derive a human-readable variant string from the spawn config type."""
    cls_name = type(spawn).__name__
    if "Simforge" in cls_name or "SimForge" in cls_name:
        return "Procedural"
    if "UsdFile" in cls_name:
        return "Dataset"
    if "MultiAsset" in cls_name or getattr(spawn, "assets_cfg", None) is not None:
        return "Dataset"
    if "Shape" in cls_name or "Plane" in cls_name or "Ground" in cls_name:
        return "Primitive"
    if "Terrain" in cls_name:
        return "Terrain"
    return cls_name


def _shared_child_spawn_value(spawn: Any, group: str, field_name: str) -> Any:
    """Return a child spawn value when every child provides the same value."""
    children = getattr(spawn, "assets_cfg", None) or getattr(spawn, "assets", None)
    if not children:
        return None
    values = [
        getattr(getattr(child, group, None), field_name, None) for child in children
    ]
    if values[0] is None or any(value != values[0] for value in values[1:]):
        return None
    return values[0]


def _categorize_object(cls: type) -> str:
    """Categorize objects by tool hierarchy or source module."""
    for base in cls.__mro__:
        if base.__name__ == "ActiveTool":
            return "Active"
    if any(base.__name__ == "Tool" for base in cls.__mro__[1:]):
        return "Passive"

    module = cls.__module__.lower()
    if "peg_in_hole" in module or "bolt_and_nut" in module or "screw" in module:
        return "Hardware"
    if "rock" in module or "sample" in module:
        return "Geological Samples"
    if "lego" in module:
        return "Construction Blocks"
    if "container" in module or "box" in module or "panel" in module:
        return "Structures & Containers"
    if "shape" in module:
        return "Primitive Shapes"
    return "Other"


def introspect_objects() -> dict[str, list[ObjectInfo]]:
    """Return metadata for registered objects grouped by type."""
    import srb.assets.object  # noqa: F401 — trigger concrete class registration
    from srb.core.asset.object.object import ObjectRegistry

    results: dict[str, list[ObjectInfo]] = {}

    for object_type, object_classes in ObjectRegistry.registry.items():
        type_str = str(object_type)
        group: list[ObjectInfo] = []

        for cls in object_classes:
            obj_name = cls.name()
            display_name = _snake_to_title(obj_name)

            asset_cfg = _get_field_default(cls, "asset_cfg")
            variant = ""
            density: float | None = None
            collision_approx = ""

            if asset_cfg is not None:
                spawn = getattr(asset_cfg, "spawn", None)
                if spawn is not None:
                    asset_variant = getattr(cls(), "asset_variant", None)
                    variant = (
                        asset_variant.name.title()
                        if asset_variant is not None
                        else _classify_spawn_variant(spawn)
                    )

                    mesh_col = getattr(spawn, "mesh_collision_props", None)
                    if mesh_col is not None:
                        collision_approx = getattr(mesh_col, "mesh_approximation", "")
                    if not collision_approx:
                        collision_approx = (
                            _shared_child_spawn_value(
                                spawn, "mesh_collision_props", "mesh_approximation"
                            )
                            or ""
                        )

                    mass_props = getattr(spawn, "mass_props", None)
                    density = getattr(mass_props, "density", None)
                    if density is None:
                        density = _shared_child_spawn_value(
                            spawn, "mass_props", "density"
                        )

            domains_raw = getattr(cls, "DOMAINS", ())
            domains = [str(d) for d in domains_raw]

            category = _categorize_object(cls)

            group.append(
                ObjectInfo(
                    name=obj_name,
                    display_name=display_name,
                    object_type=type_str,
                    category=category,
                    variant=variant,
                    domains=domains,
                    density=density,
                    collision_approx=collision_approx,
                )
            )

        results[type_str] = group

    return results


def _categorize_scenery(scenery_type_str: str) -> str:
    """Assign a display category to a scenery based on its type."""
    lower = scenery_type_str.lower()
    if lower in ("terrain", "subterrane"):
        return "Terrain"
    return "Facilities"


def _extract_sub_assets(cls: type) -> list[str]:
    """Extract sub-asset names from a scenery class (e.g. facilities with
    terrain/lights).
    """
    result: list[str] = []
    if not hasattr(cls, "model_fields"):
        return result
    for field_name, field_info in cls.model_fields.items():
        default = field_info.default
        if default is not None and hasattr(default, "asset_cfg"):
            result.append(field_name)
    return result


def introspect_scenery() -> dict[str, list[SceneryInfo]]:
    """Return metadata for registered scenery grouped by type."""
    import srb.assets.scenery  # noqa: F401 — trigger concrete class registration
    from srb.core.asset.scenery.scenery import SceneryRegistry

    results: dict[str, list[SceneryInfo]] = {}

    for scenery_type, scenery_classes in SceneryRegistry.registry.items():
        type_str = str(scenery_type)
        group: list[SceneryInfo] = []

        for cls in scenery_classes:
            scn_name = cls.name()
            display_name = _snake_to_title(scn_name)

            asset_cfg = _get_field_default(cls, "asset_cfg")
            variant = ""
            if asset_cfg is not None:
                spawn = getattr(asset_cfg, "spawn", None)
                if spawn is not None:
                    variant = _classify_spawn_variant(spawn)

            domains_raw = getattr(cls, "DOMAINS", ())
            domains = [str(d) for d in domains_raw]

            category = _categorize_scenery(type_str)

            sub_assets = _extract_sub_assets(cls)

            group.append(
                SceneryInfo(
                    name=scn_name,
                    display_name=display_name,
                    scenery_type=type_str,
                    category=category,
                    variant=variant,
                    domains=domains,
                    sub_assets=sub_assets,
                )
            )

        results[type_str] = group

    return results
