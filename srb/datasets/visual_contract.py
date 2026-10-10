"""Interpret dataset visual metadata without importing a simulator.

``demo_io_contract`` describes merged-image channels, encodings and dtypes.
"""

from __future__ import annotations

import io
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from srb.utils.json_types import thaw_json_value

# Channel groups that become LeRobot ``image`` features, with their channel
# count and the feature-name suffix.
_IMAGE_VIEW_ENCODINGS: dict[str, tuple[int, str]] = {
    "rgb_u8": (3, "rgb"),
    "depth_linear_u8": (1, "depth"),
}
LEROBOT_IMAGE_FEATURE_PREFIX = "observation.images."


@dataclass(frozen=True)
class VisualImageView:
    """One uint8 image derived from a channel slice of a merged visual leaf."""

    feature: str
    source_path: tuple[str, ...]
    data_type: str
    encoding: str
    start: int
    end: int
    height: int
    width: int
    depth_range_m: tuple[float, float] | None

    @property
    def channels(self) -> int:
        return self.end - self.start

    @property
    def shape(self) -> list[int]:
        return [self.height, self.width, self.channels]

    def describe(self, source_feature: str) -> dict[str, Any]:
        entry: dict[str, Any] = {
            "source_feature": source_feature,
            "source_path": list(self.source_path),
            "data_type": self.data_type,
            "encoding": self.encoding,
            "channels": [self.start, self.end],
        }
        if self.depth_range_m is not None:
            entry["depth_range_m"] = list(self.depth_range_m)
        return entry


def _observation_terms(contract: Mapping[str, Any] | None) -> list[Mapping[str, Any]]:
    if not isinstance(contract, Mapping):
        return []
    # Thaw frozen manifest metadata (mapping proxies and tuples).
    contract = thaw_json_value(contract)
    if not isinstance(contract, Mapping):
        return []
    observation = contract.get("observation")
    if not isinstance(observation, Mapping):
        return []
    terms = observation.get("terms")
    if not isinstance(terms, list):
        return []
    return [term for term in terms if isinstance(term, Mapping)]


def _term_path(term: Mapping[str, Any]) -> tuple[str, ...]:
    emitted = term.get("emitted_path")
    if (
        isinstance(emitted, list)
        and emitted
        and all(isinstance(part, str) for part in emitted)
    ):
        return tuple(emitted)
    output_key = term.get("output_key") or term.get("key")
    return tuple(str(output_key).split("/"))


def _term_paths(term: Mapping[str, Any]) -> tuple[tuple[str, ...], ...]:
    """Return the supported emitted-path and output-key aliases for a term."""
    paths: list[tuple[str, ...]] = []
    emitted = term.get("emitted_path")
    if (
        isinstance(emitted, (list, tuple))
        and emitted
        and all(isinstance(part, str) and part for part in emitted)
    ):
        paths.append(tuple(emitted))
    output_key = term.get("output_key")
    if isinstance(output_key, str) and output_key:
        path = tuple(output_key.split("/"))
        if all(path) and path not in paths:
            paths.append(path)
    if not paths:
        key = term.get("key")
        if isinstance(key, str) and key:
            paths.append(tuple(key.split("/")))
    return tuple(paths)


def declared_observation_terms(
    contract: Mapping[str, Any] | None,
) -> list[tuple[tuple[tuple[str, ...], ...], np.dtype | None, tuple[int, ...] | None]]:
    """Return each declared term with its supported aliases, dtype and shape."""
    terms = []
    for term in _observation_terms(contract):
        raw_dtype = term.get("dtype")
        dtype: np.dtype | None = None
        if isinstance(raw_dtype, str):
            try:
                parsed = np.dtype(raw_dtype.removeprefix("torch."))
            except TypeError:
                parsed = None
            if parsed is not None and parsed.kind in "biuf":
                dtype = parsed
        raw_shape = term.get("shape")
        shape = (
            tuple(raw_shape)
            if isinstance(raw_shape, (list, tuple))
            and all(
                isinstance(dim, int) and not isinstance(dim, bool) for dim in raw_shape
            )
            else None
        )
        paths = _term_paths(term)
        if paths:
            terms.append((paths, dtype, shape))
    return terms


def declared_observation_dtypes(
    contract: Mapping[str, Any] | None,
) -> dict[tuple[str, ...], np.dtype]:
    """Map each contract observation leaf path to its declared NumPy dtype."""
    dtypes: dict[tuple[str, ...], np.dtype] = {}
    for term in _observation_terms(contract):
        raw = term.get("dtype")
        if not isinstance(raw, str):
            continue
        try:
            dtype = np.dtype(raw.removeprefix("torch."))
        except TypeError:
            continue
        if dtype.kind in "biuf":
            dtypes[_term_path(term)] = dtype
            output_key = term.get("output_key")
            if isinstance(output_key, str) and output_key:
                dtypes.setdefault(tuple(output_key.split("/")), dtype)
    return dtypes


def declared_observation_shapes(
    contract: Mapping[str, Any] | None,
) -> dict[tuple[str, ...], tuple[int, ...]]:
    """Map each contract observation leaf path to its declared row shape."""
    shapes: dict[tuple[str, ...], tuple[int, ...]] = {}
    for term in _observation_terms(contract):
        raw = term.get("shape")
        if not isinstance(raw, list) or not all(
            isinstance(dim, int) and not isinstance(dim, bool) for dim in raw
        ):
            continue
        shape = tuple(raw)
        shapes[_term_path(term)] = shape
        output_key = term.get("output_key")
        if isinstance(output_key, str) and output_key:
            shapes.setdefault(tuple(output_key.split("/")), shape)
    return shapes


def has_legacy_batch_axis(
    row_shape: tuple[int, ...], declared_shape: tuple[int, ...]
) -> bool:
    """Whether a row has the legacy single-environment axis from Robomimic."""
    return tuple(row_shape) == (1, *declared_shape)


def contract_shape_error(
    label: str, row_shape: tuple[int, ...], declared_shape: tuple[int, ...]
) -> str | None:
    """Report a row whose shape is neither the contract shape nor legacy."""
    row_shape = tuple(row_shape)
    if row_shape == tuple(declared_shape) or has_legacy_batch_axis(
        row_shape, declared_shape
    ):
        return None
    return (
        f"{label} shape {row_shape} does not match its contract shape "
        f"{tuple(declared_shape)}"
    )


def storage_dtype_error(label: str, stored: Any, declared: np.dtype) -> str | None:
    """Check compatibility after conversion.

    :func:`needs_range_scan` determines whether widened values need a range check.
    """
    stored = np.dtype(stored)
    if declared.kind == "f":
        compatible = stored.kind in "biuf"
    elif declared.kind == "b":
        compatible = stored.kind == "b"
    else:
        compatible = stored.kind in "biu"
    if compatible:
        return None
    return f"{label} stores {stored}; declared {declared}"


def needs_range_scan(stored: Any, declared: np.dtype) -> bool:
    """Whether storage can contain values not representable by the declaration."""
    stored = np.dtype(stored)
    if declared.kind in "iu":
        return stored.kind in "biu" and not np.can_cast(
            stored, declared, casting="safe"
        )
    if declared.kind == "f":
        return stored.kind in "biuf" and not np.can_cast(
            stored, declared, casting="safe"
        )
    return False


def range_error(label: str, array: np.ndarray, declared: np.dtype) -> str | None:
    """Report integer values the declared integer dtype cannot represent."""
    if not array.size:
        return None
    info = np.iinfo(declared)
    if array.min() < info.min or array.max() > info.max:
        return f"{label} has values outside the {declared} range"
    return None


def floating_overflow_error(
    label: str, array: np.ndarray, declared: np.dtype
) -> str | None:
    """Reject finite numeric values that become infinity in a float dtype."""
    if declared.kind != "f" or not array.size or array.dtype.kind not in "biuf":
        return None
    with np.errstate(over="ignore", invalid="ignore"):
        narrowed = array.astype(declared, copy=False)
    if np.any(np.isfinite(array) & np.isinf(narrowed)):
        return f"{label} has finite values that overflows declared {declared}"
    return None


def declared_dtype_for(
    dtypes: Mapping[tuple[str, ...], np.dtype], path: tuple[str, ...]
) -> np.dtype | None:
    """Return the dtype the contract declares for an observation leaf path."""
    return dtypes.get(tuple(path))


def cast_declared(value: Any, dtype: np.dtype, *, resource: str) -> np.ndarray:
    """Cast to the declared dtype, rejecting values that do not fit.

    JSON has no dtype: floats narrow by rounding, while integers and booleans
    must be exactly representable.
    """
    array = np.asarray(value)
    if array.dtype == dtype:
        return array
    if dtype.kind == "f":
        if array.dtype.kind not in "biuf":
            raise ValueError(f"{resource} is not numeric; declared {dtype}")
        error = floating_overflow_error(resource, array, dtype)
        if error is not None:
            raise ValueError(error)
        with np.errstate(over="ignore", invalid="ignore"):
            return array.astype(dtype)
    if dtype.kind == "b":
        if array.dtype.kind != "b":
            raise ValueError(f"{resource} is not boolean; declared {dtype}")
        return array.astype(dtype)
    if array.dtype.kind == "f":
        if not np.all(np.isfinite(array)) or not np.array_equal(array, np.round(array)):
            raise ValueError(f"{resource} has non-integral values; declared {dtype}")
    elif array.dtype.kind not in "biu":
        raise ValueError(f"{resource} is not numeric; declared {dtype}")
    error = range_error(resource, array, dtype)
    if error is not None:
        raise ValueError(error)
    return array.astype(dtype)


def visual_image_views(
    contract: Mapping[str, Any] | None,
    *,
    source_path_overrides: Mapping[tuple[str, ...], tuple[str, ...]] | None = None,
) -> list[VisualImageView]:
    """List the LeRobot image views a contract's visual terms provide."""
    views: list[VisualImageView] = []
    for term in _observation_terms(contract):
        channels = term.get("channels")
        shape = term.get("shape")
        if not isinstance(channels, list) or not isinstance(shape, list):
            continue
        if len(shape) != 3:
            continue
        height, width = int(shape[0]), int(shape[1])
        canonical_path = _term_path(term)
        path = (source_path_overrides or {}).get(canonical_path, canonical_path)
        camera = canonical_path[-1].removeprefix("image_")
        raw_range = term.get("depth_range_m")
        depth_range = (
            (float(raw_range[0]), float(raw_range[1]))
            if isinstance(raw_range, list) and len(raw_range) == 2
            else None
        )
        for channel in channels:
            if not isinstance(channel, Mapping):
                continue
            encoding = channel.get("encoding")
            if encoding not in _IMAGE_VIEW_ENCODINGS:
                continue
            count, suffix = _IMAGE_VIEW_ENCODINGS[str(encoding)]
            start, end = int(channel["start"]), int(channel["end"])
            if end - start != count:
                continue
            if encoding == "depth_linear_u8" and depth_range is None:
                continue
            feature = f"{LEROBOT_IMAGE_FEATURE_PREFIX}{camera}_{suffix}"
            if any(view.feature == feature for view in views):
                # A second group of the same kind (``depth`` plus
                # ``distance_to_camera``) is named by its data type.
                feature = (
                    f"{LEROBOT_IMAGE_FEATURE_PREFIX}{camera}_{channel.get('data_type')}"
                )
                if any(view.feature == feature for view in views):
                    continue
            views.append(
                VisualImageView(
                    feature=feature,
                    source_path=path,
                    data_type=str(channel.get("data_type")),
                    encoding=str(encoding),
                    start=start,
                    end=end,
                    height=height,
                    width=width,
                    depth_range_m=depth_range
                    if encoding == "depth_linear_u8"
                    else None,
                )
            )
    return views


def certified_image_views(
    contract: Mapping[str, Any] | None,
    observation_paths: Mapping[str, Sequence[str]] | None,
) -> list[VisualImageView]:
    """Image views whose merged source leaf is a certified observation path."""
    if contract is None or not observation_paths:
        return []
    paths = {tuple(components) for components in observation_paths.values()}
    source_paths: dict[tuple[str, ...], tuple[str, ...]] = {}
    visual_paths: set[tuple[str, ...]] = set()
    for term in _observation_terms(contract):
        canonical_path = _term_path(term)
        channels = term.get("channels")
        shape = term.get("shape")
        has_image_channel = isinstance(channels, list) and any(
            isinstance(channel, Mapping)
            and channel.get("encoding") in _IMAGE_VIEW_ENCODINGS
            for channel in channels
        )
        if not has_image_channel or not isinstance(shape, list) or len(shape) != 3:
            continue
        if canonical_path in visual_paths:
            raise ValueError(
                "ambiguous canonical visual image path "
                f"{canonical_path!r}: multiple contract terms"
            )
        visual_paths.add(canonical_path)
        source_path = next((path for path in _term_paths(term) if path in paths), None)
        if source_path is not None:
            source_paths[canonical_path] = source_path
    return [
        view
        for view in visual_image_views(contract, source_path_overrides=source_paths)
        if view.source_path in paths
    ]


def describe_image_views(
    views: Sequence[VisualImageView],
    observation_paths: Mapping[str, Sequence[str]],
) -> dict[str, dict[str, Any]]:
    """The ``srb_visual_views`` entries a LeRobot ``info.json`` records."""
    sources = {
        tuple(components): f"observation.{suffix}"
        for suffix, components in observation_paths.items()
    }
    return {view.feature: view.describe(sources[view.source_path]) for view in views}


def view_pixels(value: Any, view: VisualImageView, *, resource: str) -> np.ndarray:
    """Extract a view's ``(H, W, C)`` uint8 pixels from one merged leaf value.

    The leaf may carry Robomimic's retained one-environment axis.
    """
    array = np.asarray(value)
    expected = (view.height, view.width)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or tuple(array.shape[:2]) != expected:
        raise ValueError(
            f"{resource} shape {tuple(np.shape(value))} does not hold a "
            f"{view.height}x{view.width} image"
        )
    if array.shape[2] < view.end:
        raise ValueError(f"{resource} has no channels {view.start}..{view.end}")
    return cast_declared(
        array[:, :, view.start : view.end], np.dtype("uint8"), resource=resource
    )


def encode_png(pixels: np.ndarray) -> bytes:
    """Encode ``(H, W, 1|3)`` uint8 pixels as PNG (lossless)."""
    from PIL import Image

    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] not in (1, 3):
        raise ValueError("PNG views require (H, W, 1|3) uint8 pixels")
    image = Image.fromarray(pixels[:, :, 0] if pixels.shape[2] == 1 else pixels)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def decode_png(data: bytes, *, channels: int) -> np.ndarray:
    """Decode PNG bytes to ``(H, W, channels)`` uint8 pixels."""
    from PIL import Image

    from srb.utils.resource_budget import (
        MAX_ARRAY_BYTES,
        MAX_ARRAY_ELEMENTS,
        checked_shape_nbytes,
    )

    with Image.open(io.BytesIO(data)) as image:
        width, height = image.size
        checked_shape_nbytes(
            (height, width, channels),
            np.dtype(np.uint8),
            resource="PNG decoded pixels",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        image.load()
        expected_mode = "L" if channels == 1 else "RGB"
        if image.mode != expected_mode:
            raise ValueError(
                f"PNG view mode {image.mode!r}, expected {expected_mode!r}"
            )
        pixels = np.asarray(image, dtype=np.uint8)
    if channels == 1:
        pixels = pixels[:, :, None]
    return pixels


def depth_metres(pixels: np.ndarray, depth_range_m: tuple[float, float]) -> np.ndarray:
    """Convert ``depth_linear_u8`` pixels to metres."""
    near, far = depth_range_m
    return near + pixels.astype(np.float32) / 255.0 * (far - near)


__all__ = [
    "LEROBOT_IMAGE_FEATURE_PREFIX",
    "VisualImageView",
    "cast_declared",
    "certified_image_views",
    "contract_shape_error",
    "declared_dtype_for",
    "declared_observation_dtypes",
    "declared_observation_shapes",
    "declared_observation_terms",
    "decode_png",
    "depth_metres",
    "describe_image_views",
    "encode_png",
    "has_legacy_batch_axis",
    "floating_overflow_error",
    "needs_range_scan",
    "range_error",
    "storage_dtype_error",
    "view_pixels",
    "visual_image_views",
]
