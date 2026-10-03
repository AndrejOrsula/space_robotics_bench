# Re-export Isaac Lab's ContactSensor with compatibility fixes for its concrete
# PhysX backends. The resolver must select rigid bodies, not same-named nested
# collision meshes. Filtered views also need explicit per-environment paths:
# wildcard sensor/filter patterns can drop bodies or misalign force rows when
# names collide across environments. Filters resolve to rigid-body descendants
# (or remain unchanged without any), matching PhysX's contact paths.
# Patch the concrete backend because the sensor factory is not in its MRO.

import functools
import importlib
from contextlib import suppress

from isaaclab.sensors import ContactSensor, ContactSensorCfg  # noqa: F401

from srb.core.sensor import SensorBase
from srb.core.sensor._contact_view_patterns import (
    expand_contact_view_patterns,
    physx_glob_to_prim_regex,
)


def _resolve_prim_paths(glob: str) -> list[str]:
    from isaaclab.sim.utils import find_matching_prim_paths

    return find_matching_prim_paths(physx_glob_to_prim_regex(glob))


def _resolve_rigid_body_paths(glob: str) -> list[str]:
    """Resolve a filter glob to the rigid bodies PhysX reports contacts on."""
    from isaaclab.sim.utils import find_matching_prims
    from pxr import Usd

    paths: list[str] = []
    for prim in find_matching_prims(physx_glob_to_prim_regex(glob)):
        if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
            paths.append(prim.GetPath().pathString)
            continue
        bodies = [
            descendant.GetPath().pathString
            for descendant in Usd.PrimRange(prim)
            if "PhysicsRigidBodyAPI" in descendant.GetAppliedSchemas()
        ]
        paths.extend(bodies or [prim.GetPath().pathString])
    return list(dict.fromkeys(paths))


class _ContactPhysicsViewProxy:
    """Fix batched filtered-contact arguments for one backend simulation view."""

    def __init__(self, inner):
        self._inner = inner

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def create_rigid_contact_view(
        self,
        pattern,
        filter_patterns=None,
        max_contact_data_count=0,
    ):
        filters = [] if filter_patterns is None else filter_patterns
        if filters:
            expanded = expand_contact_view_patterns(
                pattern,
                filters,
                _resolve_prim_paths,
                _resolve_rigid_body_paths,
            )
            if expanded is not None:
                pattern, filters = expanded

        return self._inner.create_rigid_contact_view(
            pattern,
            filter_patterns=filters,
            max_contact_data_count=max_contact_data_count,
        )


class _ContactSimulationManagerProxy:
    """Limit the contact-view argument fix to the contact-sensor backend."""

    _srb_expands_filtered_contact_patterns = True

    def __init__(self, inner):
        self._inner = inner

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def get_physics_sim_view(self):
        return _ContactPhysicsViewProxy(self._inner.get_physics_sim_view())


def _patch_contact_backend() -> None:
    """Patch concrete backends, not the unused contact-sensor factory class."""

    def _wrap(original):
        @functools.wraps(original)
        def _rigid_body_resolver(source_expr, predicate, *args, **kwargs):
            def _matches_rigid_body(prim):
                schemas = prim.GetAppliedSchemas()
                return "PhysicsRigidBodyAPI" in schemas and (
                    predicate is None or predicate(prim)
                )

            return original(source_expr, _matches_rigid_body, *args, **kwargs)

        _rigid_body_resolver._srb_rigid_body_only = True
        return _rigid_body_resolver

    for module_name in (
        "isaaclab_physx.sensors.contact_sensor.contact_sensor",
        "isaaclab_ovphysx.sensors.contact_sensor.contact_sensor",
    ):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        original = getattr(module, "resolve_matching_prims_from_source", None)
        if original is None or getattr(original, "_srb_rigid_body_only", False):
            pass
        else:
            module.resolve_matching_prims_from_source = _wrap(original)

        # Isaac Lab 3 owns the manager in the PhysX backend. Keep the proxy
        # scoped to whichever public manager name the concrete backend uses;
        # do not import the removed Isaac Sim core manager here.
        manager_name = "PhysxManager"
        manager = getattr(module, manager_name, None)
        if manager is None:
            manager_name = "SimulationManager"
            manager = getattr(module, manager_name, None)
        if manager is not None and not getattr(
            manager,
            "_srb_expands_filtered_contact_patterns",
            False,
        ):
            setattr(module, manager_name, _ContactSimulationManagerProxy(manager))


_patch_contact_backend()


def __safe_clear_callbacks(self):
    """``SensorBase._clear_callbacks`` that tolerates sensors whose ``__init__``
    never ran: ``__del__`` of such a ``ContactSensor`` otherwise raises
    ``AttributeError`` on the unset callback handles."""
    for name in (
        "_initialize_handle",
        "_invalidate_initialize_handle",
        "_prim_deletion_handle",
    ):
        handle = getattr(self, name, None)
        if handle is not None:
            with suppress(Exception):
                handle.deregister()
        setattr(self, name, None)
    with suppress(Exception):
        self._clear_debug_vis_handle()


SensorBase._clear_callbacks = __safe_clear_callbacks
