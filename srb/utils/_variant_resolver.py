"""Isaac-free resolver for ``AssetVariant`` requests against asset registries."""

from __future__ import annotations

import types
from typing import Any, Callable, Iterable, TypeVar

from srb.utils import logging

T = TypeVar("T")


def get_field_annotation(instance: object, field_name: str) -> Any:
    """Return the resolved type annotation for ``field_name`` on ``instance``.

    Reads ``__annotations__`` directly to avoid ``typing.get_type_hints``
    failures when class hierarchies contain ``TYPE_CHECKING``-gated imports
    (Isaac Lab's ``DirectRLEnvCfg``).
    """
    for cls in type(instance).__mro__:
        if field_name in cls.__dict__.get("__annotations__", {}):
            return cls.__annotations__[field_name]
    raise KeyError(field_name)


def asset_variant_hint(asset_cls: type) -> Any:
    """Return the class-level ``ASSET_VARIANT`` hint, if it's an enum member."""
    from enum import Enum

    hint = getattr(asset_cls, "ASSET_VARIANT", None)
    return hint if isinstance(hint, Enum) else None


def skip_asset_variant_candidate(asset_cls: type, requested: Any) -> bool:
    """Return whether a registry candidate can be rejected before init.

    Some asset classes pin their variant at class level via ``ASSET_VARIANT``.
    Real terrain shortcuts are dataset-backed and may fetch/bake during
    construction, so rejecting variant mismatches and known-bad manifest sites
    before instantiation avoids unnecessary network/download failures.
    """
    candidate_variant = asset_variant_hint(asset_cls)
    if candidate_variant is not None and candidate_variant is not requested:
        return True
    return candidate_declines_instantiation(asset_cls)


def candidate_declines_instantiation(asset_cls: type) -> bool:
    """Check a class's optional ``declines_instantiation`` predicate.

    Classes may define this zero-argument classmethod to reject themselves
    before construction. Predicate errors are logged and treated as no opinion
    so another candidate can still be tried.
    """
    predicate = getattr(asset_cls, "declines_instantiation", None)
    if not callable(predicate):
        return False
    try:
        return bool(predicate())
    except Exception as e:  # noqa: BLE001 - a broken probe must not veto
        logging.debug(
            'Availability probe on "%s" failed (%s); trying it anyway',
            getattr(asset_cls, "__name__", asset_cls),
            e,
        )
        return False


def resolve_variant(
    requested: Any,
    *,
    type_hints: Any,
    base_class: type[T],
    candidates_for_type: Callable[[type[T]], Iterable[type[T]]],
    instantiate: Callable[[type[T]], T],
    current_domain: Any,
    label: str,
) -> T | None:
    """Return the first candidate matching variant and domain, or ``None``.

    Non-union hints return ``None``; instantiation failures are logged/skipped.
    """
    if not isinstance(type_hints, types.UnionType):
        logging.error(
            f"Unsupported type hints for {label} variant request: "
            f"{type_hints} ({type(type_hints)})"
        )
        return None

    for typ in type_hints.__args__:
        if typ is type(None):
            continue
        # ``base_class``-subclass filter also rejects ``AssetVariant``-enum
        # arms (the variant marker is not a subclass of ``Scenery``/``Robot``)
        # and any other non-asset union members.
        if not isinstance(typ, type) or not issubclass(typ, base_class):
            continue
        for registered in candidates_for_type(typ):
            if not issubclass(registered, typ):
                continue
            if skip_asset_variant_candidate(registered, requested):
                logging.trace(
                    'Skipped instantiating "%s" for requested "%s" variant',
                    registered.__name__,
                    requested,
                )
                continue
            try:
                instance = instantiate(registered)
            except Exception as e:
                logging.warning(f'Failed to instantiate "{registered.__name__}": {e}')
                continue
            if not instance.is_variant(requested):
                logging.debug(
                    f'Registered {label} "{registered.__name__}" is a subclass '
                    f'of suitable type "{typ.__name__}" but its '
                    f'"{instance.asset_variant}" variant does not match the '
                    f'requested "{requested}" variant'
                )
                continue
            if instance.DOMAINS and current_domain not in instance.DOMAINS:
                logging.debug(
                    f'Registered {label} "{registered.__name__}" is a subclass '
                    f'of suitable type "{typ.__name__}" and '
                    f'"{instance.asset_variant}" variant but it does not '
                    f'support the "{current_domain}" domain'
                )
                continue
            logging.info(
                f'Selected {label} "{instance.__class__.__name__}" of variant '
                f'"{instance.asset_variant}" for domain "{current_domain}"'
            )
            return instance
    return None
