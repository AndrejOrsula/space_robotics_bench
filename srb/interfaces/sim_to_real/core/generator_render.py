from __future__ import annotations

import json
import math
from typing import Any, Mapping, Sequence

import gymnasium
import numpy as np

from srb.interfaces.sim_to_real.deploy_profiles.generation import DeployProfileSpec

from .generator_types import EnvInfo, PyExpr


class PythonExprRenderer:
    """Render Python expressions with stable quoting (double quotes)."""

    def render(self, value: Any) -> str:
        if isinstance(value, PyExpr):
            return value.code
        if value is None:
            return "None"
        if value is True:
            return "True"
        if value is False:
            return "False"
        if isinstance(value, str):
            return json.dumps(value)
        if isinstance(value, (int, float)):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"Cannot render non-finite float {value!r}")
            return repr(value)
        if isinstance(value, np.bool_):
            return "True" if bool(value) else "False"
        if isinstance(value, (np.integer, np.floating)):
            if isinstance(value, np.floating) and not np.isfinite(value):
                raise ValueError(f"Cannot render non-finite float {value!r}")
            # Prefer human-stable scalar rendering over repr(np.float32(...)).
            return str(value)
        if isinstance(value, slice):
            return f"slice({value.start}, {value.stop}, {value.step})"
        if isinstance(value, Mapping):
            items = []
            for k, v in value.items():
                items.append(f"{self.render(k)}: {self.render(v)}")
            return "{" + ", ".join(items) + "}"
        if isinstance(value, tuple):
            inner = ", ".join(self.render(v) for v in value)
            if len(value) == 1:
                inner += ","
            return "(" + inner + ")"
        if isinstance(value, list):
            return "[" + ", ".join(self.render(v) for v in value) + "]"
        if isinstance(value, set):
            if not value:
                return "set()"
            return "{" + ", ".join(sorted(self.render(v) for v in value)) + "}"

        raise TypeError(f"Unsupported value type for rendering: {type(value).__name__}")


class SpaceFormatter:
    def format(self, space: gymnasium.Space) -> str:
        if isinstance(space, gymnasium.spaces.Box):
            low_limit, high_limit = _numpy_dtype_limits(space.dtype)
            low_limit_str, high_limit_str = _numpy_dtype_limits_str(space.dtype)

            def _format(value: bool | int | float) -> str:
                if value == low_limit or value == -np.inf:
                    return low_limit_str
                elif value == high_limit or value == np.inf:
                    return high_limit_str
                else:
                    # Shortest repr in the space dtype (``tolist`` widens float32).
                    return str(np.dtype(space.dtype).type(value))

            def _format_nested(value: Any) -> str:
                if isinstance(value, list):
                    return "[" + ", ".join(_format_nested(v) for v in value) + "]"
                return _format(value)

            # Scalar (shape ``()``) bounds flatten to one element as well.
            space_low_flattened = np.ravel(space.low).tolist()
            space_high_flattened = np.ravel(space.high).tolist()
            if (
                len(set(space_low_flattened)) == 1
                and len(set(space_high_flattened)) == 1
            ):
                low_str = _format(space_low_flattened[0])
                high_str = _format(space_high_flattened[0])
            else:
                low_str = f"np.array({_format_nested(space.low.tolist())})"
                high_str = f"np.array({_format_nested(space.high.tolist())})"
            return (
                "gymnasium.spaces.Box("
                f"low={low_str}, high={high_str}, shape={space.shape}, "
                f"dtype=np.{np.dtype(space.dtype).type.__name__}"
                ")"
            )
        if isinstance(space, gymnasium.spaces.Discrete):
            dtype = f"np.{np.dtype(space.dtype).name}"
            return (
                "gymnasium.spaces.Discrete("
                f"{space.n}, start={int(space.start)}, dtype={dtype})"
            )
        if isinstance(space, gymnasium.spaces.MultiDiscrete):
            dtype = f"np.{np.dtype(space.dtype).name}"
            return (
                "gymnasium.spaces.MultiDiscrete("
                f"{space.nvec.tolist()!r}, dtype={dtype}, "
                f"start={space.start.tolist()!r})"
            )
        if isinstance(space, gymnasium.spaces.MultiBinary):
            return f"gymnasium.spaces.MultiBinary({space.n})"
        if isinstance(space, gymnasium.spaces.Dict):
            # A sequence of (key, space) pairs preserves iteration order
            # (gymnasium re-sorts plain dicts), which is load-bearing for the
            # canonical vector packing order in the observation-space layout.
            items = []
            for key, subspace in space.spaces.items():
                items.append(f"({json.dumps(key)}, {self.format(subspace)})")
            return "gymnasium.spaces.Dict([" + ", ".join(items) + "])"
        if isinstance(space, gymnasium.spaces.Tuple):
            entries = [self.format(subspace) for subspace in space.spaces]
            # A list avoids the ``(x)`` non-tuple pitfall for one-element tuples.
            return f"gymnasium.spaces.Tuple([{', '.join(entries)}])"

        raise TypeError(f"Unsupported space type: {type(space).__name__}")


class RealEnvModuleRenderer:
    def __init__(
        self,
        *,
        expr_renderer: PythonExprRenderer | None = None,
        space_formatter: SpaceFormatter | None = None,
    ):
        self._expr = expr_renderer or PythonExprRenderer()
        self._spaces = space_formatter or SpaceFormatter()

    def render_offline_module(
        self,
        *,
        env_info: EnvInfo,
        env_spec_id: str,
        registration_kwargs: Mapping[str, Any],
        used_srb_paths: set[str],
        hardware: Sequence[str],
        deploy_profile: DeployProfileSpec | None = None,
    ) -> str:
        hw_unique = tuple(dict.fromkeys(hardware))
        hw_tuple = (
            "("
            + ", ".join(json.dumps(hw) for hw in hw_unique)
            + ("," if hw_unique else "")
            + ")"
        )
        reg_kwargs_expr = self._expr.render(registration_kwargs)
        is_canonical = env_info.io_schema_fingerprint is not None

        lines: list[str] = []
        lines.append(
            "## Note: This file is auto-generated by the SRB RealEnvGenerator."
        )
        lines.append("##       Manual edits are highly discouraged!")
        lines.append(f"## Schema provenance: {env_info.schema_provenance}")
        lines.append("")
        lines.append("from typing import ClassVar, Sequence")
        lines.append("")
        lines.append("import gymnasium")
        lines.append("import numpy as np")
        lines.append("")
        # Keep imports isort-ordered so pre-commit does not reorder them.
        if env_info.command_mapping is not None:
            lines.append("from srb.core.command_mapping import CommandMapping")
        lines.append("from srb.interfaces.sim_to_real import RealEnv")
        if is_canonical:
            lines.append(
                "from srb.interfaces.sim_to_real.core.hardware import "
                "DeploymentCapability"
            )
        if deploy_profile is not None:
            lines.append(f"from {deploy_profile.module} import (")
            lines.append(f"    {deploy_profile.symbol},")
            lines.append(")")
        if used_srb_paths:
            lines.append(
                "from srb.utils.path import " + ", ".join(sorted(used_srb_paths))
            )
        lines.append("")
        lines.append("")
        lines.append("class Env(RealEnv):")
        env_name = env_spec_id.rsplit("/", 1)[-1]
        if is_canonical:
            lines.append(f'    ENV_ID: ClassVar[str] = "srb_real/{env_name}"')
            lines.append(
                "    IO_SCHEMA_FINGERPRINT: ClassVar[str | None] = "
                f"{self._expr.render(env_info.io_schema_fingerprint)}"
            )
            lines.append(
                "    TARGET_MANIFEST_SHA256: ClassVar[str | None] = "
                f"{self._expr.render(env_info.target_manifest_sha256)}"
            )
            lines.append(
                "    REQUIRES_RUNTIME_TARGET_MANIFEST: ClassVar[bool] = "
                f"{self._expr.render(env_info.requires_runtime_target_manifest)}"
            )
            lines.append(
                "    DEPLOYMENT_REQUIREMENTS: ClassVar["
                "frozenset[DeploymentCapability]] = "
                f"{_render_deployment_requirements(env_info.deployment_requirements)}"
            )
            lines.append("")
        lines.append(
            f"    SINGLE_ACTION_SPACE: ClassVar[gymnasium.Space] = {self._spaces.format(env_info.single_action_space)}"
        )
        lines.append(
            f"    DETAILED_ACTION_SPACE: ClassVar[gymnasium.spaces.Dict] = {self._spaces.format(env_info.detailed_action_space)}"
        )
        lines.append(
            f"    ACTION_ORDERING: ClassVar[dict[str, slice]] = {self._expr.render(env_info.action_ordering)}"
        )
        lines.append(f"    ACTION_RATE: ClassVar[float] = {env_info.action_rate}")
        lines.append(
            f"    ACTION_SCALE: ClassVar[dict[str, float]] = {self._expr.render(env_info.action_scale)}"
        )
        if env_info.command_mapping is not None:
            mapping_json = json.dumps(
                env_info.command_mapping.model_dump(mode="json"),
                sort_keys=True,
                separators=(",", ":"),
            )
            lines.append(
                "    COMMAND_MAPPING: ClassVar[CommandMapping | None] = "
                "CommandMapping.model_validate_json("
                f"{self._expr.render(mapping_json)})"
            )
        lines.append("")
        lines.append(
            f"    SINGLE_OBSERVATION_SPACE: ClassVar[gymnasium.Space] = {self._spaces.format(env_info.single_observation_space)}"
        )
        lines.append(
            f"    DETAILED_OBSERVATION_SPACE: ClassVar[gymnasium.spaces.Dict] = {self._spaces.format(env_info.detailed_observation_space)}"
        )
        lines.append("")
        lines.append(
            f"    SIM_ROBOT: ClassVar[str] = {self._expr.render(env_info.robot)}"
        )
        lines.append(f"    HARDWARE: ClassVar[Sequence[str]] = {hw_tuple}")
        if deploy_profile is not None:
            lines.append(
                "    DEPLOY_HARDWARE_CONFIGURATOR = "
                f"staticmethod({deploy_profile.symbol})"
            )
        lines.append("")
        lines.append("")

        lines.append("gymnasium.register(")
        lines.append(f'    id="srb_real/{env_name}",')
        lines.append('    entry_point=f"{Env.__module__}:{Env.__name__}",')
        lines.append(f"    kwargs={reg_kwargs_expr},")
        lines.append(")")
        lines.append("")

        content = "\n".join(lines)
        if is_canonical and "np.finfo" in content:
            raise ValueError(
                "canonical generation must never emit np.finfo dtype-extrema "
                "bounds; every canonical Box carries finite physical bounds"
            )
        return content


def _render_deployment_requirements(tags: Sequence[str]) -> str:
    """Render tag values as a frozenset of DeploymentCapability members."""
    from srb.interfaces.sim_to_real.core.hardware import DeploymentCapability

    members = sorted(DeploymentCapability(tag).name for tag in tags)
    if not members:
        return "frozenset()"
    inner = ", ".join(f"DeploymentCapability.{name}" for name in members)
    return "frozenset({" + inner + "})"


def _numpy_dtype_limits(dtype: np.dtype) -> tuple[float, float]:
    if np.issubdtype(dtype, np.floating):
        return np.finfo(dtype).min, np.finfo(dtype).max  # type: ignore[return-value]
    if np.issubdtype(dtype, np.integer):
        return np.iinfo(dtype).min, np.iinfo(dtype).max
    if dtype == np.bool_:
        return 0.0, 1.0
    return -np.inf, np.inf


def _numpy_dtype_limits_str(dtype: np.dtype) -> tuple[str, str]:
    if np.issubdtype(dtype, np.floating):
        return f"np.finfo(np.{dtype}).min", f"np.finfo(np.{dtype}).max"
    if np.issubdtype(dtype, np.integer):
        return f"np.iinfo(np.{dtype}).min", f"np.iinfo(np.{dtype}).max"
    if dtype == np.bool_:
        return "0.0", "1.0"
    return "-np.inf", "np.inf"
