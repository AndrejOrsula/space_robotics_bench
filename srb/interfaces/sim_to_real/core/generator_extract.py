from __future__ import annotations

import math
from numbers import Real
from typing import TYPE_CHECKING, Any, Mapping, Sequence

import gymnasium
import numpy as np
import torch

from srb._typing import ObservationGroup
from srb.core.command_mapping import (
    CommandMapping,
    revalidate_command_mapping,
    validate_command_mapping,
)
from srb.core.env.introspect import EnvIntrospector
from srb.core.env.runtime.action_layout import ActionLayout
from srb.core.env.runtime.observation_layout import (
    ObservationLayout,
    ObservationSpaceLayout,
)
from srb.core.spec.models import ObservationSpec
from srb.core.spec.projections import to_gym_space
from srb.terrain.landscaping_contract import (
    LANDSCAPING_IO_CONTRACT,
    canonical_json,
    landscaping_deployment_recipe,
    load_crater_target_manifest,
    verify_target_manifest,
)

from .generator_types import EnvInfo

if TYPE_CHECKING:
    from srb._typing import AnyEnv, AnyEnvCfg
    from srb.interfaces.sim_to_real.deploy_profiles.generation import (
        CanonicalGenerationPolicy,
    )


class CanonicalGenerationError(RuntimeError):
    """Canonical generation failed a required contract or runtime check.

    Canonical schemas require typed ``env.srb_spec`` metadata and fail closed on missing
    metadata, fallback descriptors, absent bounds, or action-layout mismatch.
    """


def _is_manipulation_env(sim_env: Any) -> bool:
    """Resolve the simulator family only when extracting a declared mapping."""
    from srb.core.env.manipulation.env import ManipulationEnv

    return isinstance(sim_env, ManipulationEnv)


class EnvInfoExtractor:
    def extract(
        self,
        env: "AnyEnv",
        *,
        canonical_policy: "CanonicalGenerationPolicy | None" = None,
    ) -> EnvInfo:
        """Extract the generated module schema from a live environment.

        ``EnvIntrospector.of`` supplies spaces and environment count to both paths; the
        generated bridge is single-environment NumPy, so device is not emitted. It also
        requires ``unwrapped.device`` and ``unwrapped.num_envs``.
        """
        sim_env = getattr(env, "unwrapped", env)
        introspector = EnvIntrospector.of(sim_env)
        if canonical_policy is not None:
            return self._extract_canonical(
                sim_env,
                introspector,
                canonical_policy,
            )
        return self._extract_legacy(sim_env, introspector)

    @staticmethod
    def _declared_command_mapping(sim_env: Any, env_cfg: Any, action_layout: Any):
        """Read and validate the action mapping declared by the task config.

        The local simulator import stays on this path so generic generator imports do
        not initialize Isaac Lab.
        """
        actions = getattr(env_cfg, "actions", None)
        getter = getattr(actions, "get_command_mapping", None)
        if not callable(getter):
            return None
        declared = getter()
        if declared is None:
            return None
        mapping = (
            revalidate_command_mapping(declared)
            if isinstance(declared, CommandMapping)
            else CommandMapping.model_validate(declared)
        )
        mapping = mapping.model_copy(
            update={
                "task_family": (
                    "manipulation" if _is_manipulation_env(sim_env) else "other"
                )
            }
        )
        action_dim = int(action_layout.single_space.shape[0])
        validate_command_mapping(
            mapping,
            action_layout.ordering,
            action_dim,
        )
        return mapping

    # ------------------------------------------------------------------ #
    # Canonical contract-backed extraction                      #
    # ------------------------------------------------------------------ #

    def _extract_canonical(
        self,
        sim_env: Any,
        introspector: EnvIntrospector,
        policy: "CanonicalGenerationPolicy",
    ) -> EnvInfo:
        env_cfg: "AnyEnvCfg" = getattr(sim_env, "cfg", None)
        if env_cfg is None:
            raise CanonicalGenerationError(
                "canonical generation requires the live environment cfg"
            )
        spec = getattr(sim_env, "srb_spec", None)
        if spec is None:
            raise CanonicalGenerationError(
                "canonical generation requires the post-reset env.srb_spec seam"
            )
        self._validate_canonical_task_identity(
            spec,
            env_cfg,
            requested_task_name=policy.task_name,
            live_env_spec_id=getattr(getattr(sim_env, "spec", None), "id", None),
        )
        robot = _resolved_robot(env_cfg)
        try:
            effective_recipe = landscaping_deployment_recipe(robot)
        except (AttributeError, TypeError, ValueError) as exc:
            raise CanonicalGenerationError(
                f"canonical landscaping deployment recipe is invalid: {exc}"
            ) from exc

        metadata_root = getattr(spec, "metadata", None)
        metadata = (
            metadata_root.get("landscaping_contract")
            if isinstance(metadata_root, Mapping)
            else None
        )
        if not isinstance(metadata, Mapping):
            raise CanonicalGenerationError(
                "canonical generation requires "
                "EnvSpec.metadata['landscaping_contract'] (published by the "
                "task's SpecMetadataProvider hook); it is absent"
            )
        self._validate_canonical_contract_metadata(metadata, effective_recipe)
        # The metadata is an admission claim only.  All emitted schema facts
        # are bound to the immutable owner after the claim has been checked.
        contract = LANDSCAPING_IO_CONTRACT.canonical_dict()
        fingerprint = LANDSCAPING_IO_CONTRACT.fingerprint()
        action_contract = _require_contract_section(contract, "action", Mapping)
        observation_leaves = _require_contract_section(
            contract, "observation_leaves", Sequence
        )

        self._validate_actor_spec_terms(spec)

        action_terms = _snapshot_action_terms(sim_env.action_manager)
        action_layout = ActionLayout.from_action_manager(
            _ActionManagerSnapshot(action_terms)
        )
        self._cross_check_action_layout(action_contract, action_layout)
        self._validate_live_raph_action_config(
            action_terms,
            env_cfg,
            robot,
            sim_env,
        )

        detailed_observation_space = self._detailed_space_from_contract(
            observation_leaves
        )
        derived_layout = ObservationLayout.from_spaces(detailed_observation_space)
        self._cross_check_actor_projection(
            spec, detailed_observation_space, derived_layout
        )
        single_observation_space = derived_layout.build_spaces()
        self._cross_check_observation_runtime(
            introspector,
            sim_env,
            derived_layout,
            single_observation_space,
        )

        target_manifest_sha256: str | None
        if policy.requires_runtime_target_manifest:
            # The general task's missing runtime manifest is declared, not
            # papered over with the crater hash.
            target_manifest_sha256 = None
        else:
            try:
                target_manifest = load_crater_target_manifest()
                verify_target_manifest(target_manifest)
            except (OSError, TypeError, ValueError) as exc:
                raise CanonicalGenerationError(
                    "canonical crater target manifest verification failed"
                ) from exc
            expected_manifest_sha256 = target_manifest.manifest_sha256
            claimed_manifest_sha256 = metadata.get("target_manifest_sha256")
            if (
                type(expected_manifest_sha256) is not str
                or not expected_manifest_sha256
                or type(claimed_manifest_sha256) is not str
                or claimed_manifest_sha256 != expected_manifest_sha256
            ):
                raise CanonicalGenerationError(
                    "canonical generation requires target_manifest_sha256 "
                    "matching the verified crater target manifest"
                )
            target_manifest_sha256 = expected_manifest_sha256

        # Render the detailed action space in slice order (gymnasium re-sorts
        # plain dicts alphabetically; the contract order is the declared one).
        detailed_action_space = gymnasium.spaces.Dict(
            [(term.key, term.space) for term in action_layout.terms]
        )

        return EnvInfo(
            single_action_space=action_layout.single_space,
            detailed_action_space=detailed_action_space,
            action_ordering=action_layout.ordering,
            action_rate=env_cfg.agent_rate,
            action_scale=action_layout.scale,
            command_mapping=self._declared_command_mapping(
                sim_env, env_cfg, action_layout
            ),
            single_observation_space=single_observation_space,
            detailed_observation_space=detailed_observation_space,
            robot=effective_recipe["robot_kind"],
            schema_provenance=LANDSCAPING_IO_CONTRACT.contract_id,
            io_schema_fingerprint=fingerprint,
            target_manifest_sha256=target_manifest_sha256,
            requires_runtime_target_manifest=policy.requires_runtime_target_manifest,
            deployment_requirements=tuple(
                sorted(cap.value for cap in policy.deployment_requirements)
            ),
        )

    @staticmethod
    def _validate_canonical_task_identity(
        spec: Any,
        env_cfg: Any,
        *,
        requested_task_name: str,
        live_env_spec_id: str | None,
    ) -> None:
        """Bind the requested task to both live task identity seams."""
        if not isinstance(live_env_spec_id, str) or not live_env_spec_id:
            raise CanonicalGenerationError(
                "canonical generation requires the live Gym EnvSpec.id"
            )
        live_gym_task = _task_basename(live_env_spec_id)
        if not isinstance(requested_task_name, str) or not requested_task_name:
            raise CanonicalGenerationError(
                "canonical generation requires the canonical policy task_name"
            )
        requested_task = _task_basename(requested_task_name)
        if requested_task != live_gym_task:
            raise CanonicalGenerationError(
                "requested task disagrees with the live Gym EnvSpec.id: "
                f"{requested_task_name!r} vs {live_env_spec_id!r}"
            )
        spec_task = getattr(spec, "task", None)
        cfg_task = getattr(env_cfg, "task_name", None)
        for label, value in (
            ("SrbEnvSpec.task", spec_task),
            ("cfg.task_name", cfg_task),
        ):
            if value not in (None, "") and not isinstance(value, str):
                raise CanonicalGenerationError(
                    f"canonical generation requires {label} to be a string"
                )
        live_spec_task = _task_basename(spec_task) if spec_task else None
        live_cfg_task = _task_basename(cfg_task) if cfg_task else None
        if live_spec_task and live_cfg_task and live_spec_task != live_cfg_task:
            raise CanonicalGenerationError(
                "live SrbEnvSpec.task and cfg.task_name disagree: "
                f"{spec_task!r} != {cfg_task!r}"
            )
        for label, live_task, original in (
            ("SrbEnvSpec.task", live_spec_task, spec_task),
            ("cfg.task_name", live_cfg_task, cfg_task),
        ):
            if live_task and requested_task != live_task:
                raise CanonicalGenerationError(
                    "requested task disagrees with the live "
                    f"{label} identity: {live_env_spec_id!r} vs {original!r}"
                )

    @staticmethod
    def _validate_canonical_contract_metadata(
        metadata: Mapping[str, Any],
        effective_recipe: Mapping[str, str],
    ) -> None:
        """Reject forged metadata before rendering contract-backed schema."""
        claimed_recipe = metadata.get("deployment_recipe")
        if not isinstance(claimed_recipe, Mapping) or dict(claimed_recipe) != dict(
            effective_recipe
        ):
            raise CanonicalGenerationError(
                "landscaping deployment_recipe metadata does not match the "
                "effective resolved Raph recipe"
            )

        if metadata.get("contract_id") != LANDSCAPING_IO_CONTRACT.contract_id:
            raise CanonicalGenerationError(
                "landscaping contract_id does not match the frozen IO contract"
            )
        claimed_contract = metadata.get("contract")
        if not isinstance(claimed_contract, Mapping):
            raise CanonicalGenerationError(
                "landscaping contract metadata requires the complete 'contract' mapping"
            )
        try:
            claimed_json = canonical_json(claimed_contract)
        except (TypeError, ValueError) as exc:
            raise CanonicalGenerationError(
                "landscaping contract metadata contains a non-canonical payload"
            ) from exc
        if claimed_json != LANDSCAPING_IO_CONTRACT.canonical_json():
            # Keep the existing typed diagnostic for a truncated payload while
            # still requiring byte-identical canonical JSON for every field.
            if "observation_leaves" not in claimed_contract:
                raise CanonicalGenerationError(
                    "landscaping contract metadata is missing the "
                    "'observation_leaves' section"
                )
            raise CanonicalGenerationError(
                "landscaping contract payload does not match the frozen IO "
                "contract canonical_json"
            )
        if metadata.get("io_schema_fingerprint") != (
            LANDSCAPING_IO_CONTRACT.fingerprint()
        ):
            raise CanonicalGenerationError(
                "landscaping io_schema_fingerprint does not match the frozen "
                "IO contract"
            )

    @staticmethod
    def _validate_live_raph_action_config(
        action_terms: Sequence[tuple[Any, Any]],
        env_cfg: Any,
        robot: Any,
        sim_env: Any,
    ) -> None:
        """Check live action config against the frozen Raph IO schema."""
        expected_keys = [term.key for term in LANDSCAPING_IO_CONTRACT.action.terms]
        observed_keys: list[Any] = []
        terms_by_key: dict[str, Any] = {}
        for entry in action_terms:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise CanonicalGenerationError(
                    "canonical generation requires action-manager iter_terms "
                    "entries as (key, term) pairs"
                )
            key, term = entry
            observed_keys.append(key)
            if isinstance(key, str):
                if key in terms_by_key:
                    raise CanonicalGenerationError(
                        f"duplicate live action term '{key}'"
                    )
                terms_by_key[key] = term
        if observed_keys != expected_keys:
            raise CanonicalGenerationError(
                f"live action-manager terms {observed_keys} disagree with the "
                f"frozen Raph terms {expected_keys}"
            )

        action = LANDSCAPING_IO_CONTRACT.action
        drive_term = action.terms[0]
        linear, angular = drive_term.components
        drive_cfg = _term_cfg(terms_by_key[drive_term.key], drive_term.key)
        drive_declaration = _resolved_drive_declaration(robot)
        for name in (
            "drive_type",
            "drive_joint_names",
            "steering_joint_names",
            "steering_axle",
            "wheelbase_mid",
        ):
            _require_owner_attr_equal(
                drive_cfg,
                drive_declaration,
                name,
                live_context="drive action",
                owner_context="Raph robot drive declaration",
            )
        _require_float_equal(
            getattr(env_cfg, "agent_rate", None),
            1.0 / action.control_rate_hz,
            "cfg.agent_rate",
        )
        _require_float_equal(
            getattr(sim_env, "step_dt", None),
            1.0 / action.control_rate_hz,
            "env.step_dt",
        )
        for name, expected in (
            ("asset_name", "robot"),
            ("scale_linear", linear.scale),
            ("scale_angular", angular.scale),
            ("v_lin_max", linear.saturation),
            ("v_ang_max", angular.saturation),
            ("accel_lin_max", action.drive_filter.accel_lin_max_m_s2),
            ("accel_ang_max", action.drive_filter.accel_ang_max_rad_s2),
            ("filter_tau_s", action.drive_filter.filter_tau_s),
            ("wheel_radius", action.drive_filter.wheel_radius_m),
        ):
            if name == "asset_name":
                _require_exact_attr(drive_cfg, name, expected, "drive action")
            else:
                _require_float_equal(
                    _required_attr(drive_cfg, name, "drive action"),
                    expected,
                    f"drive action cfg.{name}",
                )
        _require_float_tuple_equal(
            _required_attr(drive_cfg, "wheelbase", "drive action"),
            action.drive_filter.wheelbase_m,
            "drive action cfg.wheelbase",
        )
        if _required_attr(drive_cfg, "rate_limit_override", "drive action") is not None:
            raise CanonicalGenerationError(
                "drive action cfg.rate_limit_override is not allowed for the "
                "canonical Raph contract"
            )
        for name, saturation in (
            ("max_linear_velocity", linear.saturation),
            ("max_angular_velocity", angular.saturation),
        ):
            value = _required_attr(drive_cfg, name, "drive action")
            if value is not None:
                numeric = _as_finite_float(value, f"drive action cfg.{name}")
                if numeric < saturation - 1e-9:
                    raise CanonicalGenerationError(
                        f"drive action cfg.{name}={numeric} tightens the frozen "
                        f"{name.removeprefix('max_')} saturation {saturation}"
                    )

        lift_term = action.terms[1]
        lift_cfg = _term_cfg(terms_by_key[lift_term.key], lift_term.key)
        lift_component = lift_term.components[0]
        for name, expected in (
            ("asset_name", "payload"),
            ("lift_joint_name", action.shovel_joint_name),
            ("lift_velocity_max_m_s", lift_component.scale),
        ):
            if name in {"asset_name", "lift_joint_name"}:
                _require_exact_attr(lift_cfg, name, expected, "lift action")
            else:
                _require_float_equal(
                    _required_attr(lift_cfg, name, "lift action"),
                    expected,
                    f"lift action cfg.{name}",
                )
        _require_float_tuple_equal(
            _required_attr(lift_cfg, "lift_range_m", "lift action"),
            action.shovel_position_range_m,
            "lift action cfg.lift_range_m",
        )
        # Coupled curl is an internal actuator path, but changing its joint or
        # range changes the accepted Raph semantics and must fail closed. The
        # Raph payload owns these values; no Isaac asset import is needed here.
        # A lift-only payload declares both as ``None``; one that declares
        # neither attribute is still rejected.
        payload = getattr(robot, "payload", None)
        payload_type = type(payload)
        missing = object()
        expected_curl_joint = getattr(payload_type, "CURL_JOINT_NAME", missing)
        expected_curl_range = getattr(payload_type, "CURL_POSITION_RANGE_RAD", missing)
        if expected_curl_joint is missing or expected_curl_range is missing:
            raise CanonicalGenerationError(
                "resolved Raph payload is missing its coupled-curl semantics"
            )
        if expected_curl_joint is None and expected_curl_range is None:
            _require_exact_attr(lift_cfg, "curl_joint_name", None, "lift action")
            if _required_attr(lift_cfg, "curl_range_rad", "lift action") is not None:
                raise CanonicalGenerationError(
                    "lift action cfg.curl_range_rad must be None for a lift-only "
                    "Raph payload"
                )
            return
        if not isinstance(expected_curl_joint, str) or expected_curl_range is None:
            raise CanonicalGenerationError(
                "resolved Raph payload has inconsistent coupled-curl semantics"
            )
        _require_exact_attr(
            lift_cfg, "curl_joint_name", expected_curl_joint, "lift action"
        )
        _require_float_tuple_equal(
            _required_attr(lift_cfg, "curl_range_rad", "lift action"),
            expected_curl_range,
            "lift action cfg.curl_range_rad",
        )

    @staticmethod
    def _validate_actor_spec_terms(spec: Any) -> None:
        """Reject fallback provenance and missing bounds in actor-visible terms."""
        privileged_prefix = f"{ObservationGroup.STATE_PRIVILEGED}__"
        for group_name, group in spec.observation.groups.items():
            if group_name == ObservationGroup.STATE_PRIVILEGED or group_name.startswith(
                privileged_prefix
            ):
                continue
            for term in group.terms.values():
                if term.privileged:
                    continue
                provenance = term.provenance or ""
                if provenance.startswith("<fallback>"):
                    raise CanonicalGenerationError(
                        f"actor-visible observation term "
                        f"'{group_name}/{term.name}' has fallback provenance; "
                        "canonical generation refuses inferred descriptors"
                    )
                if term.min is None or term.max is None:
                    raise CanonicalGenerationError(
                        f"actor-visible observation term "
                        f"'{group_name}/{term.name}' is missing physical bounds"
                    )

    @staticmethod
    def _cross_check_actor_projection(
        spec: Any,
        detailed_observation_space: gymnasium.spaces.Dict,
        derived_layout: ObservationSpaceLayout,
    ) -> None:
        """Compare registered actor descriptors with the emitted schema.

        Boxes must match shape, dtype, and bounds; actor leaf groups and keys must match
        exactly. Runtime layout separately verifies leaf order.
        """
        privileged_prefix = f"{ObservationGroup.STATE_PRIVILEGED}__"
        actor_groups = {
            name: group
            for name, group in spec.observation.groups.items()
            if name != ObservationGroup.STATE_PRIVILEGED
            and not name.startswith(privileged_prefix)
        }
        projected = to_gym_space(
            ObservationSpec(groups=actor_groups),
            exclude_privileged=True,
        )

        expected_group = {term.key: term.output_key for term in derived_layout.terms}

        projected_boxes: dict[str, gymnasium.spaces.Box] = {}
        projected_pairs: set[tuple[str, str]] = set()
        for group_name, group_space in projected.spaces.items():
            for leaf_name, box in group_space.spaces.items():
                if leaf_name in projected_boxes:
                    raise CanonicalGenerationError(
                        f"actor-visible term name '{leaf_name}' is declared in "
                        f"more than one observation group (last: "
                        f"'{group_name}'); canonical leaf names must be unique"
                    )
                projected_boxes[leaf_name] = box
                projected_pairs.add((group_name, leaf_name))

        expected_pairs = {
            (expected_group[leaf_name], leaf_name) for leaf_name in expected_group
        }
        if projected_pairs != expected_pairs:
            raise CanonicalGenerationError(
                "registered actor-visible (group, term) pairs "
                f"{sorted(projected_pairs)} disagree with the contract's "
                f"{sorted(expected_pairs)}"
            )

        # The pair comparison above already fixed the leaf set (the derived
        # layout is built from ``detailed_observation_space``), so only the
        # per-leaf Box comparison is left.
        emitted_boxes = {
            leaf_name: box
            for group_space in detailed_observation_space.spaces.values()
            for leaf_name, box in group_space.spaces.items()
        }
        for leaf_name, emitted in emitted_boxes.items():
            declared = projected_boxes[leaf_name]
            if tuple(declared.shape) != tuple(emitted.shape):
                raise CanonicalGenerationError(
                    f"actor-visible term '{leaf_name}' is declared with shape "
                    f"{tuple(declared.shape)}; the contract emits "
                    f"{tuple(emitted.shape)}"
                )
            if np.dtype(declared.dtype) != np.dtype(emitted.dtype):
                raise CanonicalGenerationError(
                    f"actor-visible term '{leaf_name}' is declared with dtype "
                    f"{declared.dtype}; the contract emits {emitted.dtype}"
                )
            for bound, declared_arr, emitted_arr in (
                ("low", declared.low, emitted.low),
                ("high", declared.high, emitted.high),
            ):
                if not np.allclose(
                    np.asarray(declared_arr, dtype=np.float64),
                    np.asarray(emitted_arr, dtype=np.float64),
                    rtol=0.0,
                    atol=1e-6,
                ):
                    raise CanonicalGenerationError(
                        f"actor-visible term '{leaf_name}' declares {bound} "
                        f"bounds that disagree with the contract's physical "
                        f"{bound} bounds"
                    )

    @staticmethod
    def _cross_check_action_layout(
        action_contract: Mapping[str, Any],
        action_layout: ActionLayout,
    ) -> None:
        contract_terms: Sequence[Mapping[str, Any]] = _require_contract_section(
            action_contract, "terms", Sequence
        )
        layout_keys = [term.key for term in action_layout.terms]
        contract_keys = [
            str(_require_contract_field(term, "key", context="action term"))
            for term in contract_terms
        ]
        if layout_keys != contract_keys:
            raise CanonicalGenerationError(
                f"runtime action ordering {layout_keys} disagrees with the "
                f"contract ordering {contract_keys}"
            )
        for term in contract_terms:
            key = str(term["key"])
            start, stop = (
                int(v)
                for v in _require_contract_field(
                    term, "slice", context=f"action term '{key}'"
                )
            )
            runtime_slice = action_layout.ordering[key]
            if (runtime_slice.start or 0) != start or runtime_slice.stop != stop:
                raise CanonicalGenerationError(
                    f"runtime action slice for '{key}' is {runtime_slice}, "
                    f"contract requires [{start}, {stop})"
                )
            context = f"action term '{key}'"
            components: Sequence[Mapping[str, Any]] = _require_contract_field(
                term, "components", context=context
            )
            if len(components) == 1:
                expected = {
                    key: float(
                        _require_contract_field(components[0], "scale", context=context)
                    )
                }
            else:
                expected = {
                    f"{key}_"
                    f"{_require_contract_field(component, 'name', context=context)}": float(
                        _require_contract_field(component, "scale", context=context)
                    )
                    for component in components
                }
            for scale_key, scale_value in expected.items():
                runtime_scale = action_layout.scale.get(scale_key)
                if runtime_scale is None or not math.isclose(
                    float(runtime_scale), scale_value, rel_tol=1e-9, abs_tol=1e-12
                ):
                    raise CanonicalGenerationError(
                        f"runtime action scale '{scale_key}'={runtime_scale} "
                        f"disagrees with the contract scale {scale_value}"
                    )

    @staticmethod
    def _detailed_space_from_contract(
        observation_leaves: Sequence[Mapping[str, Any]],
    ) -> gymnasium.spaces.Dict:
        """Build grouped Box spaces with finite physical bounds in contract order."""
        grouped: dict[str, list[tuple[str, gymnasium.spaces.Box]]] = {}
        for leaf in observation_leaves:
            key = str(_require_contract_field(leaf, "key", context="observation leaf"))
            context = f"observation leaf '{key}'"
            category, _, name = key.partition("/")
            shape = tuple(
                int(dim)
                for dim in _require_contract_field(leaf, "shape", context=context)
            )
            low = [
                float(v) for v in _require_contract_field(leaf, "low", context=context)
            ]
            high = [
                float(v) for v in _require_contract_field(leaf, "high", context=context)
            ]
            if not all(math.isfinite(v) for v in (*low, *high)):
                raise CanonicalGenerationError(
                    f"observation leaf '{key}' has non-finite physical bounds"
                )
            size = int(np.prod(shape, dtype=np.int64))
            low_arr = np.broadcast_to(
                np.asarray(low, dtype=np.float32)
                if len(low) > 1
                else np.float32(low[0]),
                (size,),
            ).reshape(shape)
            high_arr = np.broadcast_to(
                np.asarray(high, dtype=np.float32)
                if len(high) > 1
                else np.float32(high[0]),
                (size,),
            ).reshape(shape)
            grouped.setdefault(category, []).append(
                (
                    name,
                    gymnasium.spaces.Box(
                        low=low_arr.copy(),
                        high=high_arr.copy(),
                        shape=shape,
                        dtype=np.float32,
                    ),
                )
            )
        # Sequences of pairs preserve the contract's category and leaf order
        # (a plain dict would be re-sorted by gymnasium).
        return gymnasium.spaces.Dict(
            [
                (category, gymnasium.spaces.Dict(entries))
                for category, entries in grouped.items()
            ]
        )

    @staticmethod
    def _cross_check_observation_runtime(
        introspector: EnvIntrospector,
        sim_env: Any,
        derived_layout: ObservationSpaceLayout,
        single_observation_space: gymnasium.spaces.Dict,
    ) -> None:
        """Verify contract-derived observation keys, shapes, and vector order against
        the live runtime.

        Physical bounds come from the contract and registered descriptors because
        runtime spaces use dtype-extrema bounds.
        """
        runtime_single = introspector.single_observation_space
        if not isinstance(runtime_single, gymnasium.spaces.Dict):
            raise CanonicalGenerationError(
                "canonical generation requires a Dict "
                "single_observation_space to verify the generated schema "
                f"against the simulator; got {type(runtime_single).__name__}"
            )
        runtime_actor_items = {
            key: tuple(space.shape)
            for key, space in runtime_single.spaces.items()
            if not key.startswith(f"{ObservationGroup.STATE_PRIVILEGED}__")
        }
        derived_items = {
            key: tuple(space.shape)
            for key, space in single_observation_space.spaces.items()
        }
        if runtime_actor_items != derived_items:
            raise CanonicalGenerationError(
                "contract-derived observation outputs "
                f"{sorted(derived_items.items())} disagree with the "
                f"runtime schema {sorted(runtime_actor_items.items())}"
            )

        runtime_terms = _runtime_layout_terms(sim_env)
        if runtime_terms is None:
            raise CanonicalGenerationError(
                "canonical generation requires the env's runtime observation "
                "layout to verify the vector packing order; neither "
                "_observation_assembly.layout nor _observation_layout exposes "
                "one"
            )
        runtime_vector_order = [
            (term.category, term.key)
            for term in runtime_terms
            if term.vector and term.category != ObservationGroup.STATE_PRIVILEGED
        ]
        derived_vector_order = [
            (term.category, term.key) for term in derived_layout.terms if term.vector
        ]
        if runtime_vector_order != derived_vector_order:
            raise CanonicalGenerationError(
                f"vector packing order {derived_vector_order} disagrees "
                f"with the runtime layout {runtime_vector_order}"
            )

    # ------------------------------------------------------------------ #
    # Legacy sampled extraction (explicit fallback provenance)            #
    # ------------------------------------------------------------------ #

    def _extract_legacy(
        self,
        sim_env: Any,
        introspector: EnvIntrospector,
    ) -> EnvInfo:
        env_cfg: "AnyEnvCfg" = sim_env.cfg

        action_layout = ActionLayout.from_action_manager(sim_env.action_manager)

        # Extract observation spaces from extract_step_return (if available).
        # The base env classes declare it as a NotImplementedError stub; a
        # task may implement it directly or inherit it from a mixin listed
        # first (waypoint navigation), so ask the method rather than the MRO.
        single_observation_space = introspector.single_observation_space
        detailed_observation_space = introspector.single_observation_space
        extract_step_return = getattr(sim_env, "extract_step_return", None)
        step_return = None
        if callable(extract_step_return):
            try:
                step_return = extract_step_return()
            except NotImplementedError:
                step_return = None
        if step_return is not None:
            detailed_observation_space = self._detailed_space_from_step_return(
                step_return
            )
            single_observation_space = ObservationLayout.from_spaces(
                detailed_observation_space
            ).build_spaces()

        return EnvInfo(
            single_action_space=action_layout.single_space,
            detailed_action_space=action_layout.detailed_space,
            action_ordering=action_layout.ordering,
            action_rate=env_cfg.agent_rate,
            action_scale=action_layout.scale,
            command_mapping=self._declared_command_mapping(
                sim_env, getattr(sim_env, "cfg", None), action_layout
            ),
            single_observation_space=single_observation_space,
            detailed_observation_space=detailed_observation_space,
            robot=_robot_name(env_cfg),
        )

    def _detailed_space_from_step_return(
        self, step_return: Any
    ) -> gymnasium.spaces.Dict:
        observation_space_dict = {}
        for obs_cat, obs_group in step_return.observation.items():
            cat_spaces = {}
            for obs_key, obs_value in obs_group.items():
                if space := self.space_from_value(obs_value):
                    cat_spaces[obs_key] = space
            observation_space_dict[obs_cat] = gymnasium.spaces.Dict(cat_spaces)
        return gymnasium.spaces.Dict(observation_space_dict)

    def space_from_value(
        self, value: np.ndarray | torch.Tensor
    ) -> gymnasium.Space | None:
        if len(value.shape) == 0 or any(map(lambda dim: dim == 0, value.shape)):
            return None
        value_shape = tuple(value.shape[1:])
        if isinstance(value, torch.Tensor):
            dtype = _torch_dtype_to_numpy_dtype(value.dtype)
        else:
            dtype = np.dtype(value.dtype)
            if dtype == np.dtype(object):
                return None
            if dtype.subdtype is not None:
                dtype, sub_shape = dtype.subdtype
                value_shape = (*value_shape, *sub_shape)
            if not (
                np.issubdtype(dtype, np.floating)
                or np.issubdtype(dtype, np.integer)
                or dtype == np.dtype(bool)
            ):
                return None
        low, high = _numpy_dtype_limits(dtype)
        return gymnasium.spaces.Box(
            low=low,
            high=high,
            shape=value_shape,
            dtype=dtype,  # type: ignore[misc]
        )


class _ActionManagerSnapshot:
    """Stable view of the action manager's ``iter_terms`` seam."""

    def __init__(self, terms: tuple[tuple[Any, Any], ...]):
        self._terms = terms

    def iter_terms(self):
        return iter(self._terms)


def _snapshot_action_terms(action_manager: Any) -> tuple[tuple[Any, Any], ...]:
    iter_terms = getattr(action_manager, "iter_terms", None)
    if not callable(iter_terms):
        raise CanonicalGenerationError(
            "canonical generation requires the action manager's iter_terms seam"
        )
    try:
        terms = tuple(iter_terms())
    except Exception as exc:  # noqa: BLE001 - typed canonical admission error
        raise CanonicalGenerationError(
            f"canonical generation cannot read action-manager iter_terms: {exc}"
        ) from exc
    if not terms:
        raise CanonicalGenerationError(
            "canonical generation requires non-empty action-manager iter_terms"
        )
    return terms


def _required_attr(owner: Any, name: str, context: str) -> Any:
    missing = object()
    value = getattr(owner, name, missing)
    if value is missing:
        raise CanonicalGenerationError(
            f"{context} configuration is missing required '{name}'"
        )
    return value


def _term_cfg(term: Any, key: str) -> Any:
    cfg = getattr(term, "cfg", None)
    if cfg is None:
        raise CanonicalGenerationError(
            f"live action term '{key}' is missing its configuration"
        )
    return cfg


def _resolved_drive_declaration(robot: Any) -> Any:
    action_modes = getattr(type(robot), "action_modes", None)
    if not isinstance(action_modes, Mapping):
        raise CanonicalGenerationError(
            "resolved Raph robot is missing its class-level action_modes mapping"
        )
    drive_group = action_modes.get("drive_filtered")
    if drive_group is None:
        raise CanonicalGenerationError(
            "resolved Raph robot is missing the drive_filtered action mode"
        )
    return _required_attr(
        drive_group,
        "cmd_vel",
        "Raph robot drive_filtered action mode",
    )


def _require_owner_attr_equal(
    live_owner: Any,
    declared_owner: Any,
    name: str,
    *,
    live_context: str,
    owner_context: str,
) -> None:
    live_value = _required_attr(live_owner, name, live_context)
    declared_value = _required_attr(declared_owner, name, owner_context)
    if name in {"drive_joint_names", "steering_joint_names"}:
        try:
            live_value = tuple(live_value)
            declared_value = tuple(declared_value)
        except TypeError as exc:
            raise CanonicalGenerationError(
                f"{live_context} cfg.{name} and {owner_context} {name} "
                "must be sequences"
            ) from exc
    if live_value != declared_value:
        raise CanonicalGenerationError(
            f"{live_context} cfg.{name}={live_value!r} disagrees with "
            f"{owner_context} {name}={declared_value!r}"
        )


def _require_exact_attr(owner: Any, name: str, expected: Any, context: str) -> None:
    value = _required_attr(owner, name, context)
    if value != expected:
        raise CanonicalGenerationError(
            f"{context} cfg.{name}={value!r} disagrees with frozen {expected!r}"
        )


def _as_finite_float(value: Any, context: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise CanonicalGenerationError(f"{context} must be a finite number")
    try:
        numeric = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise CanonicalGenerationError(f"{context} must be a finite number") from exc
    if not math.isfinite(numeric):
        raise CanonicalGenerationError(f"{context} must be a finite number")
    return numeric


def _require_float_equal(value: Any, expected: float, context: str) -> None:
    numeric = _as_finite_float(value, context)
    if not math.isclose(numeric, float(expected), rel_tol=0.0, abs_tol=1e-9):
        raise CanonicalGenerationError(
            f"{context}={numeric!r} disagrees with frozen value {expected!r}"
        )


def _require_float_tuple_equal(
    value: Any, expected: Sequence[float], context: str
) -> None:
    if isinstance(value, (str, bytes)):
        raise CanonicalGenerationError(f"{context} must be a numeric sequence")
    try:
        actual = tuple(_as_finite_float(item, context) for item in value)
    except TypeError as exc:
        raise CanonicalGenerationError(f"{context} must be a numeric sequence") from exc
    if len(actual) != len(expected) or any(
        not math.isclose(got, float(want), rel_tol=0.0, abs_tol=1e-9)
        for got, want in zip(actual, expected, strict=True)
    ):
        raise CanonicalGenerationError(
            f"{context}={actual!r} disagrees with frozen value {tuple(expected)!r}"
        )


def _resolved_robot(env_cfg: Any) -> Any:
    missing = object()
    robot = getattr(env_cfg, "_robot", missing)
    if robot is missing:
        robot = getattr(env_cfg, "robot", missing)
    if robot is missing:
        raise CanonicalGenerationError(
            "canonical generation requires resolved cfg._robot or cfg.robot"
        )
    return robot


def _task_basename(task: str) -> str:
    return task.rsplit("/", 1)[-1]


def _runtime_layout_terms(sim_env: Any) -> Sequence[Any] | None:
    """Return the live env packed observation terms, or ``None``.

    DirectEnv stores its layout in ``_observation_assembly``; checking only
    ``_observation_layout`` misses its packing order.
    """
    for layout in (
        getattr(getattr(sim_env, "_observation_assembly", None), "layout", None),
        getattr(sim_env, "_observation_layout", None),
    ):
        terms = getattr(layout, "terms", None)
        if terms is not None:
            return terms
    return None


def _require_contract_section(
    contract: Mapping[str, Any],
    key: str,
    expected_type: type,
) -> Any:
    """Read a required contract section or raise ``CanonicalGenerationError``.

    Malformed contract data must not leak ``KeyError`` or ``TypeError``."""
    if key not in contract:
        raise CanonicalGenerationError(
            f"landscaping contract metadata is missing the '{key}' section"
        )
    value = contract[key]
    if isinstance(value, (str, bytes)) or not isinstance(value, expected_type):
        raise CanonicalGenerationError(
            f"landscaping contract section '{key}' must be a "
            f"{expected_type.__name__}, got {type(value).__name__}"
        )
    return value


def _require_contract_field(
    entry: Mapping[str, Any],
    key: str,
    *,
    context: str,
) -> Any:
    """Read a required contract-entry field or raise ``CanonicalGenerationError``."""
    if not isinstance(entry, Mapping) or key not in entry:
        raise CanonicalGenerationError(
            f"landscaping contract {context} is missing the '{key}' field"
        )
    return entry[key]


def _robot_name(env_cfg: Any) -> str:
    robot = getattr(env_cfg, "robot", None)
    if robot is None:
        robot = getattr(env_cfg, "_robot", None)
    if robot is None:
        return ""
    name = getattr(robot, "name", None)
    if callable(name):
        return str(name())
    return str(robot)


def _torch_dtype_to_numpy_dtype(dtype: torch.dtype) -> Any:
    match dtype:
        case torch.float16:
            return np.float16
        case torch.float32:
            return np.float32
        case torch.float64:
            return np.float64
        case torch.int8:
            return np.int8
        case torch.uint8:
            return np.uint8
        case torch.int16:
            return np.int16
        case torch.uint16:
            return np.uint16
        case torch.int32:
            return np.int32
        case torch.uint32:
            return np.uint32
        case torch.int64:
            return np.int64
        case torch.uint64:
            return np.uint64
        case torch.bool:
            return np.bool_
        case _:
            raise TypeError(f"Unsupported torch dtype '{dtype}'")


def _numpy_dtype_limits(dtype: np.dtype) -> tuple[float, float]:
    if np.issubdtype(dtype, np.floating):
        return np.finfo(dtype).min, np.finfo(dtype).max  # type: ignore[return-value]
    elif np.issubdtype(dtype, np.integer):
        return np.iinfo(dtype).min, np.iinfo(dtype).max
    elif dtype == np.bool_:
        return 0.0, 1.0
    else:
        return -np.inf, np.inf
