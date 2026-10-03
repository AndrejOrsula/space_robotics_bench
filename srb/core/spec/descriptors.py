"""Descriptor resolution for environment specs."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Literal

from srb.core.spec._protocols import SpecProvider
from srb.core.spec.models import DType, SensorEntry, TermSpec
from srb.core.spec.registry import ComponentDescriptor, SpecRegistry
from srb.utils.config_introspection import iter_config_items

logger = logging.getLogger(__name__)

DescriptorSource = Literal["provider", "registry", "fallback", "runtime"]


_NUMPY_TO_DTYPE: dict[str, DType] = {
    "float32": "float32",
    "float64": "float64",
    "int8": "int8",
    "int16": "int16",
    "int32": "int32",
    "int64": "int64",
    "bool": "bool",
    "uint8": "uint8",
    "uint16": "uint16",
    "uint32": "uint32",
    "uint64": "uint64",
}

_LEGACY_MULTICOPTER_TERM_NAME = "thrust"
_LEGACY_MULTICOPTER_SEMANTIC_LABEL = "multicopter.rotor_thrusts"
# Superseded action signatures; projections also reject them in metadata.
_LEGACY_ACTION_SIGNATURES: dict[tuple[str, str], str] = {
    ("ee_rot", "end_effector.orientation"): (
        "legacy task-space action descriptor is unsupported: the quaternion "
        "'ee_rot' cannot describe the runtime axis-angle 'delta_twist' term"
    ),
    ("lin", "body.linear_velocity"): (
        "legacy body-acceleration action descriptor is unsupported: "
        "'lin'/'body.linear_velocity' cannot describe the runtime 'body_vel' "
        "acceleration term"
    ),
}


@dataclass(frozen=True)
class ResolvedSpecGroup:
    """Resolved terms for one action or observation group."""

    terms: dict[str, TermSpec]
    schema_version: int
    source: DescriptorSource


@dataclass(frozen=True)
class ResolvedSensorEntry:
    """Resolved sensor entry plus descriptor source metadata."""

    entry: SensorEntry
    schema_version: int
    source: DescriptorSource


def np_to_spec_dtype(np_dtype: Any) -> DType:
    """Map a NumPy or torch dtype to a spec dtype; reject unsupported types."""
    name = getattr(np_dtype, "name", None) or str(np_dtype)
    name = name.removeprefix("torch.")
    try:
        return _NUMPY_TO_DTYPE[name]
    except KeyError:
        # A silent float32 label would misdescribe the runtime data.
        supported = ", ".join(_NUMPY_TO_DTYPE)
        raise ValueError(
            f"Unsupported spec dtype {np_dtype!r}; use one of: {supported}."
        ) from None


def resolve_action_group_spec(
    action_group: Any,
    runtime_shapes: dict[str, tuple[int, ...]] | None = None,
) -> ResolvedSpecGroup:
    """Resolve one action group through provider, registry, then fallback."""
    desc: ComponentDescriptor
    source: DescriptorSource
    if isinstance(action_group, SpecProvider):
        desc = action_group.spec()
        source = "provider"
    else:
        registered = SpecRegistry.lookup_with_mro(type(action_group))
        if registered is not None:
            desc = registered
            source = "registry"
        else:
            desc = _fallback_action_descriptor(action_group)
            source = "fallback"
    if desc.kind != "action":
        raise ValueError(f"expected action descriptor, got {desc.kind!r}")
    return _resolved_group_from_descriptor(desc, runtime_shapes, source)


def resolve_observation_group_spec(
    env: Any,
    group_name: str,
    sub_space: Any,
) -> ResolvedSpecGroup:
    """Resolve one observation group through env descriptor, registry, fallback."""
    desc = getattr(env, f"_observation_descriptor_{group_name}", None)
    source: DescriptorSource | None = "provider" if desc is not None else None
    if desc is None:
        cls_desc = SpecRegistry.lookup_with_mro(type(env))
        if _matches_observation_group(cls_desc, group_name):
            desc = cls_desc
            source = "registry"
    if desc is None:
        desc = _fallback_observation_descriptor(group_name, sub_space)
        source = "fallback"
    assert source is not None
    if desc.kind != "observation":
        raise ValueError(f"expected observation descriptor, got {desc.kind!r}")
    return _resolved_group_from_descriptor(
        desc,
        _runtime_shapes_from_space(sub_space),
        source,
    )


def resolve_sensor_entry(name: str, sensor: Any) -> ResolvedSensorEntry:
    """Resolve one sensor through provider, registry, then fallback."""
    desc: ComponentDescriptor | None
    source: DescriptorSource
    if isinstance(sensor, SpecProvider):
        desc = sensor.spec()
        if desc.kind != "sensor":
            raise ValueError(f"expected sensor descriptor, got {desc.kind!r}")
        source = "provider"
    else:
        desc = SpecRegistry.lookup_with_mro(type(sensor))
        if desc is not None and desc.kind == "sensor":
            source = "registry"
        else:
            return ResolvedSensorEntry(
                entry=_fallback_sensor_entry(name, sensor),
                schema_version=0,
                source="fallback",
            )
    return ResolvedSensorEntry(
        entry=_sensor_entry_from_descriptor(name, sensor, desc),
        schema_version=desc.schema_version,
        source=source,
    )


def _matches_observation_group(
    desc: ComponentDescriptor | None,
    group_name: str,
) -> bool:
    return (
        desc is not None
        and desc.kind == "observation"
        and desc.extra_metadata.get("group") == group_name
    )


def _resolved_group_from_descriptor(
    desc: ComponentDescriptor,
    runtime_shapes: dict[str, tuple[int, ...]] | None,
    source: DescriptorSource,
) -> ResolvedSpecGroup:
    _validate_descriptor_runtime_terms(desc, runtime_shapes)
    return ResolvedSpecGroup(
        terms=_terms_from_descriptor(desc, runtime_shapes or {}),
        schema_version=desc.schema_version,
        source=source,
    )


def _validate_descriptor_runtime_terms(
    desc: ComponentDescriptor,
    runtime_shapes: dict[str, tuple[int, ...]] | None,
) -> None:
    """Reject stale action metadata before it becomes authoritative.

    Schema v2 names must match runtime terms. Schema v1 may project semantic
    fields (for example, ``lin``/``ang`` onto ``body_vel``) if total dimensions
    match. Schema v0 reflection is unchecked. Known superseded signatures are
    rejected regardless of version.
    """
    if desc.kind == "action" and any(
        term.name == _LEGACY_MULTICOPTER_TERM_NAME
        and term.semantic_label == _LEGACY_MULTICOPTER_SEMANTIC_LABEL
        for term in desc.fields
    ):
        raise ValueError(
            "legacy multicopter action descriptor is unsupported: "
            "'thrust'/'multicopter.rotor_thrusts' cannot describe the runtime "
            "'cmd_vel' term; provide an independently verified migration"
        )
    if desc.kind == "action":
        for term in desc.fields:
            message = _LEGACY_ACTION_SIGNATURES.get(
                (term.name, term.semantic_label or "")
            )
            if message is not None:
                raise ValueError(message)

    if desc.kind != "action" or runtime_shapes is None:
        return

    descriptor_names = {term.name for term in desc.fields}
    runtime_names = set(runtime_shapes)
    if descriptor_names == runtime_names:
        return

    # V2 names are runtime contracts. V1 may project semantic fields if widths
    # match; v0 reflection makes no dimensional claim.
    if desc.schema_version >= 2 or descriptor_names & runtime_names:
        missing_runtime = sorted(descriptor_names - runtime_names)
        undeclared_runtime = sorted(runtime_names - descriptor_names)
        raise ValueError(
            "descriptor/runtime terms are unmatched: "
            f"descriptor_only={missing_runtime}, runtime_only={undeclared_runtime}"
        )
    if desc.schema_version >= 1:
        descriptor_dim = sum(_flat_dim(term.shape) for term in desc.fields)
        runtime_dim = sum(_flat_dim(shape) for shape in runtime_shapes.values())
        if descriptor_dim != runtime_dim:
            raise ValueError(
                "action descriptor/runtime dimension mismatch: "
                f"descriptor fields {sorted(descriptor_names)} sum to "
                f"{descriptor_dim}, runtime terms {sorted(runtime_names)} sum to "
                f"{runtime_dim}"
            )


def _flat_dim(shape: tuple[int, ...]) -> int:
    dim = 1
    for extent in shape:
        dim *= int(extent)
    return dim


def _terms_from_descriptor(
    desc: ComponentDescriptor,
    runtime_shapes: dict[str, tuple[int, ...]],
) -> dict[str, TermSpec]:
    out: dict[str, TermSpec] = {}
    for term in desc.fields:
        rt_shape = runtime_shapes.get(term.name)
        if rt_shape is not None and rt_shape != term.shape:
            # Resolve the documented (1,) placeholder quietly. Reject other
            # action shape mismatches; observations retain warning-and-reconcile.
            is_placeholder = term.shape == (1,)
            if desc.kind == "action" and not is_placeholder:
                raise ValueError(
                    "action descriptor/runtime shape mismatch for "
                    f"{term.name}: descriptor={term.shape} runtime={rt_shape}"
                )
            (logger.debug if is_placeholder else logger.warning)(
                "spec shape %s for %s: descriptor=%s runtime=%s; using runtime",
                "resolved" if is_placeholder else "mismatch",
                term.name,
                term.shape,
                rt_shape,
            )
            runtime_size = _flat_dim(rt_shape)
            for bound_name in ("min", "max"):
                bound = getattr(term, bound_name)
                if isinstance(bound, list) and len(bound) != runtime_size:
                    raise ValueError(
                        f"descriptor/runtime {bound_name} bound mismatch for "
                        f"{term.name}: {len(bound)} values for runtime shape "
                        f"{rt_shape} ({runtime_size} elements)"
                    )
            t2 = term.model_copy(update={"shape": rt_shape})
            t2 = t2.model_copy(
                update={
                    "provenance": (term.provenance or "")
                    + f"|shape_mismatch={term.shape}",
                }
            )
            out[term.name] = t2
        else:
            out[term.name] = term
    return out


def _runtime_shapes_from_space(sub_space: Any) -> dict[str, tuple[int, ...]]:
    if not hasattr(sub_space, "spaces"):
        return {}
    return {
        term_name: tuple(int(d) for d in term_space.shape)
        for term_name, term_space in sub_space.spaces.items()
    }


def _fallback_action_descriptor(action_group: Any) -> ComponentDescriptor:
    """Reflection fallback: walk config-like fields for shape-bearing terms."""
    fields: list[TermSpec] = []
    for attr_name, val in iter_config_items(action_group):
        if attr_name.startswith("_"):
            continue
        shape = getattr(val, "shape", None)
        if shape is None:
            continue
        dtype = np_to_spec_dtype(getattr(val, "dtype", "float32"))
        try:
            fields.append(
                TermSpec(
                    name=attr_name,
                    shape=tuple(int(d) for d in shape),
                    dtype=dtype,
                    provenance="<fallback>",
                )
            )
        except ValueError:
            continue
    return ComponentDescriptor(
        kind="action",
        schema_version=0,
        fields=fields,
        extra_metadata={"fallback": True},
    )


def _fallback_observation_descriptor(
    group_name: str,
    sub_space: Any,
) -> ComponentDescriptor:
    fields: list[TermSpec] = []
    if hasattr(sub_space, "spaces"):
        for term_name, term_space in sub_space.spaces.items():
            # Resolve outside the try: an unsupported dtype must not drop the term.
            dtype = np_to_spec_dtype(getattr(term_space, "dtype", "float32"))
            try:
                fields.append(
                    TermSpec(
                        name=term_name,
                        shape=tuple(int(d) for d in term_space.shape),
                        dtype=dtype,
                        provenance="<fallback>",
                    )
                )
            except ValueError:
                continue
    return ComponentDescriptor(
        kind="observation",
        schema_version=0,
        fields=fields,
        extra_metadata={"group": group_name, "fallback": True},
    )


def _sensor_entry_from_descriptor(
    name: str,
    sensor: Any,
    desc: ComponentDescriptor,
) -> SensorEntry:
    field = desc.fields[0] if desc.fields else None
    # The flat output fields describe the primary output; keep every declared
    # named output (e.g. heightmap ``heights`` and ``valid_mask``) in extras.
    extras: dict[str, Any] = {}
    if len(desc.fields) > 1:
        extras["outputs"] = {
            output.name: {"shape": list(output.shape), "dtype": output.dtype}
            for output in desc.fields
        }
    return SensorEntry(
        name=name,
        category=str(getattr(sensor, "category", "UNKNOWN")),
        output_shape=field.shape if field is not None else (),
        output_dtype=field.dtype if field is not None else "float32",
        provenance=type(sensor).__name__,
        extras=extras,
    )


def _fallback_sensor_entry(name: str, sensor: Any) -> SensorEntry:
    data_shape = getattr(getattr(sensor, "data", None), "shape", ())
    shape = tuple(int(d) for d in data_shape) if data_shape else (0,)
    if not shape or all(s == 0 for s in shape):
        shape = (1,)
    return SensorEntry(
        name=name,
        category=str(getattr(sensor, "category", "UNKNOWN")),
        output_shape=shape,
        output_dtype=np_to_spec_dtype(getattr(sensor, "dtype", "float32")),
        provenance=type(sensor).__name__,
    )


__all__ = (
    "DescriptorSource",
    "ResolvedSensorEntry",
    "ResolvedSpecGroup",
    "np_to_spec_dtype",
    "resolve_action_group_spec",
    "resolve_observation_group_spec",
    "resolve_sensor_entry",
)
