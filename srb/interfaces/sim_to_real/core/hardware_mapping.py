from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

import gymnasium
import numpy as np

from srb.core.env.runtime.action_layout import ActionLayout
from srb.core.env.runtime.observation_layout import ObservationSpaceLayout
from srb.interfaces.sim_to_real.core.hardware import HardwareInterface
from srb.utils.json_types import ImmutableValueMixin

HardwareRoutes = Mapping[HardwareInterface, tuple[tuple[str, str], ...]]


@dataclass(frozen=True)
class HardwareActionMapping(ImmutableValueMixin):
    interfaces: tuple[HardwareInterface, ...]
    routes: HardwareRoutes
    # RealEnv validates flat ingress against SINGLE_ACTION_SPACE. Direct
    # mapping callers use this detailed action domain before dispatch.
    action_spaces: Mapping[str, gymnasium.spaces.Box]

    @classmethod
    def build(
        cls,
        hardware: Sequence[HardwareInterface],
        layout: ActionLayout,
    ) -> HardwareActionMapping:
        interfaces = tuple(hw for hw in hardware if hw._has_io_action)
        routes: dict[HardwareInterface, list[tuple[str, str]]] = {}
        source_spaces = layout.source_spaces()
        for action_key, action_space in source_spaces:
            _map_action_to_hardware(
                action_key,
                action_space,
                interfaces=interfaces,
                routes=routes,
                layout=layout,
            )
        _validate_all_action_interfaces_routed(interfaces, routes)
        return cls(
            interfaces=interfaces,
            routes=_freeze_routes(routes),
            action_spaces=_freeze_action_spaces(dict(source_spaces)),
        )

    def apply(
        self,
        action: Mapping[str, np.ndarray],
        *,
        before_commit: Callable[[], None] | None = None,
    ) -> None:
        """Dispatch an action to routed interfaces in two phases.

        All interfaces prepare before transport starts. Transforming owners
        return validated tokens; legacy ``apply_action`` transforms cannot
        gain that guarantee here. Commit transport failures propagate so the
        session can latch and safe-stop every interface.
        """
        _validate_action_domain(action, self.action_spaces)
        staged = {
            hw: {
                hardware_key: np.array(action[action_key], copy=True)
                for action_key, hardware_key in routes
            }
            for hw, routes in self.routes.items()
        }
        prepared = {
            hw: hw.prepare_action(hardware_action)
            for hw, hardware_action in staged.items()
        }
        for hw, token in prepared.items():
            hw.validate_prepared_action(token)
        for hw, hardware_action in staged.items():
            if before_commit is not None:
                before_commit()
            hw.commit_prepared_action(hardware_action, prepared[hw])

    def effective_action(
        self,
        action: Mapping[str, np.ndarray],
    ) -> dict[str, np.ndarray]:
        """Return adapter reports in policy-key order.

        Reports may use physical units after scaling/filtering. This checks
        shape, numeric values and representability; adapters enforce physical
        limits before transport.
        """

        _validate_action_domain(action, self.action_spaces)
        effective = {
            action_key: np.array(value, copy=True)
            for action_key, value in action.items()
        }
        for hw, routes in self.routes.items():
            emitted = hw.emitted_action
            if not isinstance(emitted, Mapping):
                raise ValueError(
                    f'Emitted actions from hardware "{hw.name}" must be a mapping'
                )
            unexpected = set(emitted) - set(hw._declared_action_spaces.spaces)
            if unexpected:
                raise ValueError(
                    f'Emitted actions from hardware "{hw.name}" contain undeclared keys: '
                    f"{sorted(map(str, unexpected))}"
                )
            for action_key, hardware_key in routes:
                if hardware_key not in emitted:
                    continue
                expected = np.asarray(action[action_key])
                value = np.asarray(emitted[hardware_key])
                context = f'Emitted action "{hardware_key}" from hardware "{hw.name}"'
                if value.shape != expected.shape:
                    raise ValueError(
                        f"{context} has shape {value.shape}, expected {expected.shape}"
                    )
                allowed_kinds = "biuf" if expected.dtype.kind == "b" else "iuf"
                if value.dtype.kind not in allowed_kinds or not np.all(
                    np.isfinite(value)
                ):
                    raise ValueError(
                        f"{context} must contain finite real numeric values"
                    )
                with np.errstate(over="ignore", invalid="ignore"):
                    converted = value.astype(expected.dtype, copy=True)
                if not np.all(np.isfinite(converted)):
                    raise ValueError(
                        f"{context} is not finite in reporting dtype {expected.dtype}"
                    )
                if expected.dtype.kind in "biu" and not np.array_equal(
                    converted, value
                ):
                    raise ValueError(
                        f"{context} cannot be represented exactly in discrete "
                        f"reporting dtype {expected.dtype}"
                    )
                effective[action_key] = converted
        return effective

    def scale_for(
        self,
        layout: ActionLayout,
        hardware: HardwareInterface,
    ) -> dict[str, float]:
        routes = self.routes.get(hardware)
        if not routes:
            return {}
        return layout.scale_for_hardware(routes)


@dataclass(frozen=True)
class HardwareObservationMapping(ImmutableValueMixin):
    interfaces: tuple[HardwareInterface, ...]
    routes: HardwareRoutes

    @classmethod
    def build(
        cls,
        hardware: Sequence[HardwareInterface],
        layout: ObservationSpaceLayout,
    ) -> HardwareObservationMapping:
        interfaces = tuple(hw for hw in hardware if hw._has_io_observation)
        routes: dict[HardwareInterface, list[tuple[str, str]]] = {}
        for obs_key, obs_space in layout.source_spaces():
            _map_observation_to_hardware(
                obs_key,
                obs_space,
                interfaces=interfaces,
                routes=routes,
            )
        return cls(
            interfaces=interfaces,
            routes=_freeze_routes(routes),
        )

    def collect(self) -> dict[str, Any]:
        observation: dict[str, Any] = {}
        for hw, routes in self.routes.items():
            hw_observation = hw.observation
            for obs_key, hardware_key in routes:
                current = observation
                key_parts = obs_key.split("/")
                for part in key_parts[:-1]:
                    current = current.setdefault(part, {})
                current[key_parts[-1]] = hw_observation[hardware_key]
        return observation


@dataclass(frozen=True)
class HardwareSourceGroups:
    observation: tuple[HardwareInterface, ...]
    reward: tuple[HardwareInterface, ...]
    termination: tuple[HardwareInterface, ...]
    pause: tuple[HardwareInterface, ...]
    resume: tuple[HardwareInterface, ...]

    @classmethod
    def build(cls, hardware: Sequence[HardwareInterface]) -> HardwareSourceGroups:
        return cls(
            observation=tuple(hw for hw in hardware if hw._has_io_observation),
            reward=tuple(hw for hw in hardware if hw._has_io_reward),
            termination=tuple(hw for hw in hardware if hw._has_io_termination),
            pause=tuple(hw for hw in hardware if hw._has_io_pause),
            resume=tuple(hw for hw in hardware if hw._has_io_resume),
        )

    def summary(self, action: Sequence[HardwareInterface]) -> str:
        return (
            f"Action interfaces: {_names(action)}\n"
            f"Observation interfaces: {_names(self.observation)}\n"
            f"Reward interfaces: {_names(self.reward)}\n"
            f"Termination interfaces: {_names(self.termination)}\n"
            f"Pause interfaces: {_names(self.pause)}\n"
            f"Resume interfaces: {_names(self.resume)}"
        )


def _map_action_to_hardware(
    action_key: str,
    action_space: gymnasium.spaces.Box,
    *,
    interfaces: Sequence[HardwareInterface],
    routes: dict[HardwareInterface, list[tuple[str, str]]],
    layout: ActionLayout,
) -> None:
    found_hw: HardwareInterface | None = None

    for hw in interfaces:
        matched_key: str | None = None
        for key_to_try in layout.match_keys(action_key):
            alias_key = hw._map_alias(key_to_try)
            matched_key = hw.action_key_map.get(alias_key)
            if matched_key is not None:
                break
        if matched_key is None:
            continue

        already_mapped = {destination for _, destination in routes.get(hw, [])}
        if matched_key in already_mapped:
            raise ValueError(
                f'Action key "{action_key}" cannot map to hardware "{hw.name}": '
                f"route {matched_key} is already assigned"
            )

        if found_hw is not None:
            raise ValueError(
                f'Action key "{action_key}" must have a unique hardware interface '
                f"mapping but two were found: {found_hw.name} and {hw.name}"
            )
        found_hw = hw
        routes.setdefault(hw, []).append((action_key, matched_key))
        _validate_action_space(
            action_key=action_key,
            action_space=action_space,
            hardware=hw,
            hardware_key=matched_key,
        )

    if found_hw is None:
        raise ValueError(
            f'Action key "{action_key}" must be mapped to a hardware interface '
            "but no matches were found"
        )


def _validate_finite_action(action: Mapping[str, object]) -> None:
    """Reject invalid commands before any hardware adapter is called."""

    for action_key, value in action.items():
        candidate = value
        if hasattr(candidate, "detach"):
            candidate = candidate.detach()
        if hasattr(candidate, "cpu"):
            candidate = candidate.cpu()
        try:
            array = np.asarray(candidate)
            finite = bool(np.all(np.isfinite(array)))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise ValueError(
                f'Action "{action_key}" contains a non-finite deployment command'
            )


def _validate_action_domain(
    action: Mapping[str, object],
    action_spaces: Mapping[str, gymnasium.spaces.Box],
) -> None:
    """Reject malformed or out-of-domain detailed actions before dispatch."""

    _validate_finite_action(action)
    expected = set(action_spaces)
    actual = set(action)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        details: list[str] = []
        if missing:
            details.append(f"missing={missing}")
        if extra:
            details.append(f"extra={extra}")
        raise ValueError(
            "detailed action keys do not match the declared action space: "
            + ", ".join(details)
        )

    for action_key, action_space in action_spaces.items():
        candidate = action[action_key]
        if hasattr(candidate, "detach"):
            candidate = candidate.detach()
        if hasattr(candidate, "cpu"):
            candidate = candidate.cpu()
        try:
            valid = action_space.contains(np.asarray(candidate))
        except (TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError(
                f'Action "{action_key}" is outside its declared detailed '
                f"action space {action_space}"
            )


def _map_observation_to_hardware(
    obs_key: str,
    obs_space: gymnasium.spaces.Box,
    *,
    interfaces: Sequence[HardwareInterface],
    routes: dict[HardwareInterface, list[tuple[str, str]]],
) -> None:
    found_hw: HardwareInterface | None = None

    for hw in interfaces:
        alias_key = hw._map_alias(obs_key)
        hardware_key = hw.observation_key_map.get(alias_key)
        if hardware_key is None:
            continue

        if found_hw is not None:
            raise ValueError(
                f'Observation key "{obs_key}" must have a unique hardware interface '
                f"mapping but two were found: {found_hw.name} and {hw.name}"
            )
        found_hw = hw
        routes.setdefault(hw, []).append((obs_key, hardware_key))
        _validate_observation_space(
            obs_key=obs_key,
            obs_space=obs_space,
            hardware=hw,
            hardware_key=hardware_key,
        )

    if found_hw is None:
        raise ValueError(
            f'Observation key "{obs_key}" must be mapped to a hardware interface '
            "but no matches were found"
        )


def _validate_action_space(
    *,
    action_key: str,
    action_space: gymnasium.spaces.Box,
    hardware: HardwareInterface,
    hardware_key: str,
) -> None:
    hardware_space = hardware._declared_action_spaces.spaces[hardware_key]
    if not action_space.shape == hardware_space.shape:
        raise ValueError(
            f'Action "{action_key}" from hardware "{hardware.name}" does not match '
            f'expected space "{action_space}" with its shape "{hardware_space.shape}"'
        )
    if np.dtype(action_space.dtype) != np.dtype(hardware_space.dtype):
        raise ValueError(
            f'Action "{action_key}" from hardware "{hardware.name}" does not match '
            f'expected dtype "{action_space.dtype}" with its dtype '
            f'"{hardware_space.dtype}"'
        )


def _validate_all_action_interfaces_routed(
    interfaces: Sequence[HardwareInterface],
    routes: Mapping[HardwareInterface, Sequence[tuple[str, str]]],
) -> None:
    unrouted = tuple(hw for hw in interfaces if hw not in routes)
    if not unrouted:
        return
    details = "; ".join(
        f"{hw.name} declares {tuple(hw._declared_action_spaces.spaces.keys())}"
        for hw in unrouted
    )
    raise ValueError(
        f"Action hardware interface(s) have no mapped action routes: {details}"
    )


def _validate_observation_space(
    *,
    obs_key: str,
    obs_space: gymnasium.spaces.Box,
    hardware: HardwareInterface,
    hardware_key: str,
) -> None:
    if hardware_key in hardware._declared_observation_spaces.spaces:
        hardware_space = hardware._declared_observation_spaces.spaces[hardware_key]
        if not obs_space.shape == hardware_space.shape:
            raise ValueError(
                f'Observation "{obs_key}" from hardware "{hardware.name}" does not '
                f'match expected space "{obs_space}" with its shape '
                f'"{hardware_space.shape}"'
            )
        if np.dtype(obs_space.dtype) != np.dtype(hardware_space.dtype):
            raise ValueError(
                f'Observation "{obs_key}" from hardware "{hardware.name}" does not '
                f'match expected dtype "{obs_space.dtype}" with its dtype '
                f'"{hardware_space.dtype}"'
            )
        return

    value = hardware.observation[hardware_key]
    if not obs_space.contains(value):
        raise ValueError(
            f'Observation "{obs_key}" from hardware "{hardware.name}" does not '
            f'match expected space "{obs_space}" with its shape "{value.shape}" '
            f'and dtype "{value.dtype}"'
        )


def _freeze_routes(
    routes: dict[HardwareInterface, list[tuple[str, str]]],
) -> HardwareRoutes:
    return MappingProxyType({hw: tuple(keys) for hw, keys in routes.items()})


def _freeze_action_spaces(
    spaces: dict[str, gymnasium.spaces.Box],
) -> Mapping[str, gymnasium.spaces.Box]:
    return MappingProxyType(dict(spaces))


def _names(hardware: Sequence[HardwareInterface]) -> str:
    return ", ".join(hw.name for hw in hardware)


__all__ = [
    "HardwareActionMapping",
    "HardwareObservationMapping",
    "HardwareRoutes",
    "HardwareSourceGroups",
]
