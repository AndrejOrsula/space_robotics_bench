from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from functools import cache, cached_property, lru_cache
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Protocol,
    runtime_checkable,
)

import gymnasium
import numpy as np
from pydantic import BaseModel

from srb.core.registry_seam import register_registry_container
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthStatus, normalize_health_event
from srb.utils.str import convert_to_snake_case

if TYPE_CHECKING:
    from rclpy.node import Node as RosNode


class HardwareInterfaceCfg(BaseModel):
    name: str = ""


class HardwareDispatchNotReadyError(RuntimeError):
    """An action interface cannot safely accept a command."""

    def __init__(
        self,
        interface: str,
        reason: str,
        *,
        event: HealthEvent | None = None,
    ) -> None:
        self.interface = interface
        self.reason = reason
        self.event = event
        super().__init__(
            f"hardware interface '{interface}' is not dispatch-ready: {reason}"
        )


class PreparedHardwareAction:
    """Validated transport attempt followed by an in-memory state commit.

    Adapters capture owned payloads and staged state without publishing or mutating
    reusable messages. Preparation is side-effect free; dispatch is serialized by
    ``RealEnv``.
    """

    def __init__(
        self,
        owner: HardwareInterface,
        *,
        send: Callable[[], None],
        commit: Callable[[], None],
    ) -> None:
        if not callable(send) or not callable(commit):
            raise TypeError("prepared action callbacks must be callable")
        self._owner = owner
        self._revision = owner._action_revision
        self._send = send
        self._commit_state = commit
        self._consumed = False

    def validate(self, owner: HardwareInterface) -> None:
        if owner is not self._owner:
            raise ValueError("prepared action belongs to a different owner")
        if self._consumed:
            raise HardwareDispatchNotReadyError(owner.name, "prepared action consumed")
        if self._revision != owner._action_revision:
            raise HardwareDispatchNotReadyError(owner.name, "prepared action is stale")
        if owner._prepared_commit_active:
            raise HardwareDispatchNotReadyError(
                owner.name, "command commit in progress"
            )
        if owner._safe_stop_requested or owner._is_closed or owner._is_paused:
            raise HardwareDispatchNotReadyError(
                owner.name, "prepared action owner is stopped, closed or paused"
            )

    def commit(self, owner: HardwareInterface) -> None:
        self.validate(owner)
        # Spend the attempt before invoking any external callback, including
        # cancellation/failure paths. Another token cannot rewind this step.
        self._consumed = True
        owner._action_revision += 1
        revision = owner._action_revision
        owner._prepared_commit_active = True
        try:
            self._send()
            if owner._action_revision != revision:
                raise HardwareDispatchNotReadyError(
                    owner.name, "lifecycle or command state changed during transport"
                )
            self._commit_state()
        finally:
            # A callback may prepare a new token against the old committed
            # filters while this send is active. It must not survive completion.
            owner._action_revision += 1
            owner._prepared_commit_active = False


class DeploymentCapability(StrEnum):
    """Fixed tags; a claim asserts a proven hardware contract and evidence."""

    RAPH_DRIVE_VELOCITY = "raph.drive_velocity"
    RAPH_SHOVEL_PRISMATIC_VELOCITY = "raph.shovel_prismatic_velocity"
    LANDSCAPING_EXTERNAL_HEIGHTMAP_BATCH = "landscaping.external_heightmap_batch"
    LANDSCAPING_OBSERVABLE_TASK_EVALUATOR = "landscaping.observable_task_evaluator"


@runtime_checkable
class DeploymentCapabilityProvider(Protocol):
    """Read-only protocol for components exposing deployment claims."""

    @property
    def deployment_capabilities(self) -> frozenset[DeploymentCapability]: ...


class DeploymentNotReadyError(RuntimeError):
    """Typed preflight failure: required deployment capabilities are missing."""

    def __init__(self, missing: Iterable[DeploymentCapability]):
        self.missing: tuple[DeploymentCapability, ...] = tuple(
            sorted(set(missing), key=lambda cap: cap.value)
        )
        tags = ", ".join(cap.value for cap in self.missing)
        super().__init__(
            "Deployment is not ready: no supplied component claims the "
            f"required capability tag(s): {tags}. Provide hardware interfaces "
            "or capability providers that claim these exact tags."
        )


def validate_deployment_readiness(
    requirements: frozenset[DeploymentCapability],
    components: Sequence[object],
) -> None:
    """Raise if supplied interfaces or providers lack required capability claims."""
    if not requirements:
        return
    claimed: set[DeploymentCapability] = set()
    for component in components:
        if isinstance(component, HardwareInterface):
            claimed |= component.capabilities.deployment
        elif isinstance(component, DeploymentCapabilityProvider):
            claimed |= frozenset(component.deployment_capabilities)
    missing = requirements - claimed
    if missing:
        raise DeploymentNotReadyError(missing)


@dataclass(frozen=True)
class HardwareCapabilities:
    action_keys: tuple[str, ...] = ()
    observation_keys: tuple[str, ...] = ()
    reward: bool = False
    termination: bool = False
    pause: bool = False
    resume: bool = False
    deployment: frozenset[DeploymentCapability] = field(default_factory=frozenset)


class HardwareInterface:
    cfg: HardwareInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()
    # Deployment-capability claims. Claiming a tag asserts a proven
    # hardware contract; the base interface claims none.
    DEPLOYMENT_CLAIMS: ClassVar[frozenset[DeploymentCapability]] = frozenset()
    SUPPORTS_REWARD: ClassVar[bool] = False
    SUPPORTS_TERMINATION: ClassVar[bool] = False
    SUPPORTS_PAUSE: ClassVar[bool] = False
    SUPPORTS_RESUME: ClassVar[bool] = False

    def __init__(self, cfg: HardwareInterfaceCfg | None = None) -> None:
        self.cfg = cfg if cfg is not None else HardwareInterfaceCfg()
        self._is_paused: bool = False
        self._is_started: bool = False
        self._is_closed: bool = False
        self._safe_stop_requested: bool = False
        self._safe_stop_issued: bool = False
        self._action_revision: int = 0
        self._prepared_commit_active: bool = False

    def start(
        self,
        action_rate: float,
        action_scale: dict[str, float],
        ros_node: "RosNode",
    ) -> None:
        if self._safe_stop_requested:
            raise HardwareDispatchNotReadyError(self.name, "safe stop is latched")
        self._action_revision += 1
        logging.info(f"[{self.name}] Start")
        self._action_rate: float = action_rate
        self._action_scale: dict[str, float] = action_scale
        self._ros_node: "RosNode" = ros_node
        self._is_paused = False
        self._is_closed = False
        self._is_started = True

    def close(self) -> None:
        self._action_revision += 1
        logging.info(f"[{self.name}] Close")
        self._is_started = False
        self._is_closed = True

    def safe_stop(self) -> None:
        """Latch motion off and attempt the idempotent transport stop.

        A failed stop remains retryable; start, resume, and dispatch stay
        forbidden even before transport acknowledges the stop.
        """

        self._action_revision += 1
        self._safe_stop_requested = True
        if self._safe_stop_issued:
            return
        self._safe_stop()
        self._safe_stop_issued = True

    def _safe_stop(self) -> None:
        """Transport hook for a physical zero/deadman command."""

        self.pause()

    def prepare_action(
        self, action: dict[str, np.ndarray]
    ) -> PreparedHardwareAction | None:
        """Validate a staged command without side effects.

        The default rejects unsupported keys and non-finite values, then
        returns ``None`` for legacy ``apply_action`` behavior. Transforming
        adapters return a token containing prepared payload and next state.
        """

        supported = self.supported_action_spaces
        for key, value in action.items():
            if key not in supported.spaces:
                raise ValueError(f"[{self.name}] unsupported action key {key!r}")
            array = np.asarray(value)
            if array.dtype.kind in "fc" and not np.all(np.isfinite(array)):
                raise ValueError(f"[{self.name}] action {key!r} is not finite")
        return None

    def validate_prepared_action(self, prepared: PreparedHardwareAction | None) -> None:
        """Check every token before the aggregate's first transport attempt."""
        if prepared is None:
            if self._prepared_commit_active:
                raise HardwareDispatchNotReadyError(
                    self.name, "command commit in progress"
                )
            return
        if not isinstance(prepared, PreparedHardwareAction):
            raise TypeError("prepare_action must return PreparedHardwareAction or None")
        prepared.validate(self)

    def commit_prepared_action(
        self,
        action: dict[str, np.ndarray],
        prepared: PreparedHardwareAction | None,
    ) -> None:
        """Consume a prepared command, retaining None's legacy apply behavior."""
        self.validate_prepared_action(prepared)
        if prepared is None:
            self._action_revision += 1
            self._prepared_commit_active = True
            try:
                self.apply_action(action)
            finally:
                self._action_revision += 1
                self._prepared_commit_active = False
        else:
            prepared.commit(self)

    def validate_dispatch_ready(self) -> None:
        """Fail closed unless this interface is ready for one command."""

        if not self._is_started:
            raise HardwareDispatchNotReadyError(self.name, "not started")
        if self._is_closed:
            raise HardwareDispatchNotReadyError(self.name, "already closed")
        if self._safe_stop_requested:
            raise HardwareDispatchNotReadyError(self.name, "safe stop is latched")
        if self._is_paused:
            raise HardwareDispatchNotReadyError(self.name, "paused")
        try:
            event = normalize_health_event(self.health(), component=self.name)
        except Exception as exc:  # noqa: BLE001 - hardware health is untrusted
            raise HardwareDispatchNotReadyError(
                self.name,
                f"health check failed: {type(exc).__name__}: {exc}",
            ) from exc
        if event is not None and event.status is not HealthStatus.HEALTHY:
            raise HardwareDispatchNotReadyError(
                self.name,
                f"{event.code}: {event.message}",
                event=event,
            )

    def health(self) -> HealthEvent | dict[str, Any] | None:
        """Return an interface-health event, or ``None`` to skip health checks.

        Subclasses can report staleness or connection state as a
        :class:`HealthEvent` or the legacy dict shape:

            {"kind": "topic_stale", "interface": <self.NAME>, "age_ms": <float>}

        """
        return None

    def sync(self) -> None:
        logging.trace(f"[{self.name}] Sync")

    def reset(self) -> None:
        self._action_revision += 1
        logging.debug(f"[{self.name}] Reset")

    def pause(self) -> None:
        self._action_revision += 1
        logging.debug(f"[{self.name}] Pause")
        self._is_paused = True

    def resume(self) -> None:
        if self._safe_stop_requested:
            raise RuntimeError(f"[{self.name}] cannot resume after safe stop")
        self._action_revision += 1
        logging.debug(f"[{self.name}] Resume")
        self._is_paused = False

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict()

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict()

    @property
    def capabilities(self) -> HardwareCapabilities:
        return HardwareCapabilities(
            action_keys=tuple(self._declared_action_spaces.spaces.keys()),
            observation_keys=tuple(self._declared_observation_spaces.spaces.keys()),
            reward=self.SUPPORTS_REWARD,
            termination=self.SUPPORTS_TERMINATION,
            pause=self.SUPPORTS_PAUSE,
            resume=self.SUPPORTS_RESUME,
            deployment=self.DEPLOYMENT_CLAIMS,
        )

    @property
    def deployment_capabilities(self) -> frozenset[DeploymentCapability]:
        """Read-only deployment claims (``DeploymentCapabilityProvider``)."""
        return self.capabilities.deployment

    def apply_action(self, action: dict[str, np.ndarray]) -> None:
        raise NotImplementedError()

    @property
    def emitted_action(self) -> Mapping[str, np.ndarray]:
        """Report the adapter's scaled or filtered command.

        Keys and shapes match the declared action layout; values use adapter
        reporting units and must be finite and representable in policy dtype.
        Physical values need not meet policy bounds; adapters enforce hardware
        limits. This reports commands, not measured motion or delivery. Empty
        reports fall back to the routed policy action for ``prev_action``.
        """

        return {}

    @property
    def observation(self) -> dict[str, np.ndarray]:
        raise NotImplementedError()

    @property
    def reward(self) -> float:
        raise NotImplementedError()

    @property
    def termination(self) -> bool:
        raise NotImplementedError()

    @property
    def pause_signal(self) -> bool:
        raise NotImplementedError()

    @property
    def resume_signal(self) -> bool:
        raise NotImplementedError()

    @property
    def info(self) -> dict[str, Any]:
        return {}

    ### Internal logic ###

    # `@cache` here is keyed by `cls`, and every hardware interface class is
    # already held for the process lifetime by `HardwareInterfaceRegistry`, so
    # the cache pins nothing the registry does not.
    @classmethod
    @cache
    def class_name(cls) -> str:
        return convert_to_snake_case(cls.__name__).removesuffix("_interface")

    @cached_property
    def name(self) -> str:
        if self.cfg.name:
            return convert_to_snake_case(self.cfg.name)
        else:
            return self.class_name()

    @property
    def is_paused(self) -> bool:
        return self._is_paused

    @property
    def is_started(self) -> bool:
        return self._is_started

    @property
    def is_closed(self) -> bool:
        return self._is_closed

    @property
    def safe_stop_issued(self) -> bool:
        return self._safe_stop_issued

    @property
    def ros_node(self) -> "RosNode":
        return self._ros_node

    @property
    def action_rate(self) -> float:
        return self._action_rate

    @property
    def action_scale(self) -> dict[str, float]:
        return self._action_scale

    @cached_property
    def action_key_map(self) -> Mapping[str, str]:
        if not self._has_io_action:
            return {}
        return self._map_aliases(self.capabilities.action_keys)

    @property
    def observation_key_map(self) -> Mapping[str, str]:
        if not self._has_io_observation:
            return {}
        return self._map_aliases(self.capabilities.observation_keys)

    @cached_property
    def _has_io_action(self) -> bool:
        return bool(self.capabilities.action_keys)

    @cached_property
    def _declared_action_spaces(self) -> gymnasium.spaces.Dict:
        try:
            spaces = self.supported_action_spaces
        except NotImplementedError:
            return gymnasium.spaces.Dict()
        if not isinstance(spaces, gymnasium.spaces.Dict):
            raise TypeError(
                f"[{self.name}] supported_action_spaces must be "
                f"gymnasium.spaces.Dict, got {type(spaces).__name__}"
            )
        return spaces

    @property
    def _declared_observation_spaces(self) -> gymnasium.spaces.Dict:
        try:
            spaces = self.supported_observation_spaces
        except NotImplementedError:
            return gymnasium.spaces.Dict()
        if not isinstance(spaces, gymnasium.spaces.Dict):
            raise TypeError(
                f"[{self.name}] supported_observation_spaces must be "
                f"gymnasium.spaces.Dict, got {type(spaces).__name__}"
            )
        return spaces

    @property
    def _has_io_observation(self) -> bool:
        return bool(self.capabilities.observation_keys)

    @property
    def _has_io_reward(self) -> bool:
        return self.capabilities.reward

    @property
    def _has_io_termination(self) -> bool:
        return self.capabilities.termination

    @property
    def _has_io_pause(self) -> bool:
        return self.capabilities.pause

    @property
    def _has_io_resume(self) -> bool:
        return self.capabilities.resume

    __COMMON_ALIASES: Sequence[Sequence[str]] = (
        ("accel", "acceleration", "accelerometer"),
        ("cmd_vel", "velocity_command"),
        ("depth", "depth_image", "depth_img", "depth_map"),
        ("ee_displacement", "ee_velocity"),
        ("ft", "ft_sensor", "force_torque_sensor"),
        ("gripper", "gripper_toggle"),
        ("gyro", "gyroscope", "angular_velocity"),
        ("img", "image", "rgb", "rgb_image", "rgb_img"),
        ("imu", "inertial_measurement_unit"),
        ("joint_position", "joint_positions"),
        ("joint_state", "joint_states"),
        ("joint_torque", "joint_torques"),
        ("joint_velocity", "joint_velocities"),
    )

    def _map_alias(self, value: str) -> str:
        return HardwareInterface.__map_alias_cached(
            value, self.CUSTOM_ALIASES, self.__COMMON_ALIASES
        )

    def _map_aliases(self, value: Iterable[str]) -> Mapping[str, str]:
        return {self._map_alias(val): val for val in value}

    # Bounded: keyed by a joint/sensor name plus the two alias
    # tables, so the key space grows with the hardware description rather than
    # being fixed.
    @lru_cache(maxsize=2048)
    @staticmethod
    def __map_alias_cached(
        value: str,
        custom_aliases: Sequence[Sequence[str]],
        common_aliases: Sequence[Sequence[str]],
    ) -> str:
        for alias in custom_aliases:
            if value in alias:
                return alias[0]
        else:
            for alias in common_aliases:
                if value in alias:
                    return alias[0]
            else:
                return value

    def __str__(self) -> str:
        out = f"[{self.name}]\n"
        out += (
            "  Actions: "
            + (
                f"{','.join(self.action_key_map.keys())}"
                if self._has_io_action
                else "No"
            )
            + "\n"
        )
        out += f"  Action Rate: {1.0 / self._action_rate} Hz\n"
        out += f"  Action Scale: {self._action_scale}\n"
        out += (
            "  Observations: "
            + (
                f"{','.join(self.observation_key_map.keys())}"
                if self._has_io_observation
                else "No"
            )
            + "\n"
        )
        out += "  Reward: " + ("Yes" if self._has_io_reward else "No") + "\n"
        out += "  Termination: " + ("Yes" if self._has_io_termination else "No") + "\n"
        out += "  Pause Signal: " + ("Yes" if self._has_io_pause else "No") + "\n"
        out += "  Resume Signal: " + ("Yes" if self._has_io_resume else "No") + "\n"

        return out

    def __init_subclass__(
        cls,
        hardware_interface_metaclass: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init_subclass__(**kwargs)
        if hardware_interface_metaclass:
            return
        # Fail fast at class definition: a declared capability must ship its
        # implementation. Without this, an interface that sets SUPPORTS_<X>=True
        # but forgets to override the matching property only crashes at first
        # step() on real hardware (the abstract base raises NotImplementedError),
        # never at import — exactly the kind of gap a never-run s2r path hides.
        for _flag, _attr in (
            ("SUPPORTS_REWARD", "reward"),
            ("SUPPORTS_TERMINATION", "termination"),
            ("SUPPORTS_PAUSE", "pause_signal"),
            ("SUPPORTS_RESUME", "resume_signal"),
        ):
            if getattr(cls, _flag, False) and getattr(cls, _attr) is getattr(
                HardwareInterface, _attr
            ):
                raise TypeError(
                    f"{cls.__module__}:{cls.__name__} sets {_flag}=True but does "
                    f"not override the '{_attr}' property required to provide it."
                )
        duplicate = next(
            (
                hardware_interface
                for hardware_interface in HardwareInterfaceRegistry.registry
                if cls.class_name() == hardware_interface.class_name()
            ),
            None,
        )
        if duplicate is not None:
            raise TypeError(
                "Cannot register multiple hardware interfaces with an identical "
                f"name: '{cls.__module__}:{cls.__name__}' already exists as "
                f"'{duplicate.__module__}:{duplicate.__name__}'"
            )
        HardwareInterfaceRegistry.registry.append(cls)


class HardwareInterfaceRegistry:
    registry: ClassVar[list[type[HardwareInterface]]] = []

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {hardware_interface.__module__ for hardware_interface in cls.registry}

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> type[HardwareInterface] | None:
        for hardware_interface in cls.registry:
            if hardware_interface.class_name() == name:
                return hardware_interface
        return None


def observation_spaces_from_values(
    observation: Mapping[str, np.ndarray],
) -> gymnasium.spaces.Dict:
    return gymnasium.spaces.Dict(
        {
            key: _space_from_observation_value(value)
            for key, value in observation.items()
        }
    )


def _space_from_observation_value(value: np.ndarray) -> gymnasium.spaces.Box:
    array = np.asarray(value)
    dtype = np.dtype(array.dtype)
    if np.issubdtype(dtype, np.integer):
        info = np.iinfo(dtype)
        low: int | float = int(info.min)
        high: int | float = int(info.max)
    elif np.issubdtype(dtype, np.bool_):
        low = 0
        high = 1
    else:
        low = -np.inf
        high = np.inf
    return gymnasium.spaces.Box(
        low=low,
        high=high,
        shape=array.shape,
        dtype=array.dtype.type,
    )


# This registry is append-only and populated by `__init_subclass__`, like the
# nine in `srb.core`. `srb.core.registry_seam` owns the supported
# snapshot/restore seam over all of them but must not import this layer, so the
# container is registered from here instead.
register_registry_container(HardwareInterfaceRegistry, "registry")
