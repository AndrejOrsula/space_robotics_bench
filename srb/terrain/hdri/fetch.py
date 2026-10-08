"""Fetch the source texture (HTTP or file://), verify sha256, resize."""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import numpy as np

from srb.terrain._fetch import download_to as _download_to
from srb.terrain._fetch import sha256_file as _sha256
from srb.terrain.errors import ChecksumMismatchError, FetchError
from srb.terrain.hdri.schema import SkydomeEntry
from srb.utils.publication import ArtifactPublication
from srb.utils.resource_budget import (
    GIB,
    MAX_ARRAY_BYTES,
    MAX_IMAGE_PIXELS,
    ResourceLimitError,
    checked_shape_nbytes,
    require_file_size,
)

MAX_HDRI_WIRE_BYTES = 2 * GIB


def _lanczos_resampling(Image):
    return getattr(Image, "Resampling", Image).LANCZOS


def _one_line_error(exc: BaseException) -> str:
    return " ".join(str(exc).splitlines())


def _resize_float_planes(arr, *, Image, target_w: int, target_h: int):
    """Resample an ``(H, W[, C])`` float buffer channel-by-channel via PIL."""
    if arr.dtype == "uint8":
        arr = arr.astype("float32") / 255.0
    else:
        arr = arr.astype("float32", copy=False)
    planes = [arr] if arr.ndim == 2 else [arr[..., idx] for idx in range(arr.shape[-1])]
    resampled = []
    for plane in planes:
        img = Image.fromarray(np.ascontiguousarray(plane), mode="F")
        try:
            resized = img.resize((target_w, target_h), _lanczos_resampling(Image))
            try:
                resampled.append(np.asarray(resized, dtype="float32"))
            finally:
                resized.close()
        finally:
            img.close()
    if arr.ndim == 2:
        return resampled[0]
    return np.stack(resampled, axis=-1)


def _resize_via_imageio(
    src: Path, dst: Path, *, target_w: int, target_h: int, fmt: str
) -> None:
    """Read an EXR/HDR with imageio and resample with PIL on the float buffer.

    ``fmt`` is the catalog-declared format (``"hdr"`` or ``"exr"``); we pass
    it as an explicit ``extension`` hint to ``imageio`` because the staged
    download file ends in ``.raw`` (added by ``fetch_and_resize`` to keep the
    pre-resize artefact distinct).
    """
    try:
        import imageio.v3 as iio
        from PIL import Image
    except ImportError as e:
        raise FetchError(
            "EXR/HDR support requires imageio: pip install imageio "
            f"(got {_one_line_error(e)})"
        ) from e

    # Pin HDR to OpenCV with unchanged float reads; imageio's host-dependent
    # codec selection can quantize Radiance data. Keep imageio's EXR selection
    # because OpenCV EXR is disabled in headless builds.
    read_kwargs: dict[str, object] = {}
    write_kwargs: dict[str, object] = {}
    if fmt == "hdr":
        try:
            import cv2
        except ImportError:
            cv2 = None
        if cv2 is not None:
            read_kwargs = {"plugin": "opencv", "flags": cv2.IMREAD_UNCHANGED}
            write_kwargs = {"plugin": "opencv"}

    try:
        props = iio.improps(src, extension=f".{fmt}")
        shape = tuple(int(value) for value in props.shape)
        if len(shape) < 2 or shape[0] * shape[1] > MAX_IMAGE_PIXELS:
            pixels = 0 if len(shape) < 2 else shape[0] * shape[1]
            raise ResourceLimitError(
                "HDRI decoded pixels", limit=MAX_IMAGE_PIXELS, observed=pixels
            )
        checked_shape_nbytes(
            shape,
            np.dtype(props.dtype),
            resource="HDRI decoded array",
            max_bytes=MAX_ARRAY_BYTES,
        )
        arr = iio.imread(src, extension=f".{fmt}", **read_kwargs)
    except Exception as e:
        raise FetchError(
            f"failed to read .{fmt}: {_one_line_error(e)}. "
            "EXR may need imageio[freeimage] plugin."
        ) from e

    # Preserve HDR/EXR radiance: resample each float RGB channel through a
    # PIL 32-bit float plane, then stack the planes back into float32.
    out = _resize_float_planes(arr, Image=Image, target_w=target_w, target_h=target_h)

    # Use imageio to write float formats; PIL cannot save HDR or EXR.
    try:
        iio.imwrite(dst, out, extension=f".{fmt}", **write_kwargs)
    except Exception as e:
        raise FetchError(
            f"failed to write .{fmt}: {_one_line_error(e)}. "
            "EXR may need imageio[freeimage] plugin."
        ) from e


def fetch_and_resize(entry: SkydomeEntry, dst: Path) -> None:
    """Download, verify SHA-256, and resize to the catalog width.

    Output uses a 2:1 aspect ratio. Pillow handles PNG; imageio handles
    float-format HDR and EXR sources.
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=dst.parent,
        prefix=f".{dst.name}.",
        suffix=".raw",
        delete=False,
    ) as raw_file:
        raw = Path(raw_file.name)

    try:
        _download_to(entry.url, raw, max_bytes=MAX_HDRI_WIRE_BYTES)
        require_file_size(
            raw,
            resource="HDRI source",
            max_bytes=MAX_HDRI_WIRE_BYTES,
        )

        actual = _sha256(raw)
        if actual.lower() != entry.sha256.lower():
            raise ChecksumMismatchError(
                url=entry.url,
                expected=entry.sha256,
                actual=actual,
            )

        target_w = entry.resize_to
        target_h = max(1, target_w // 2)

        # Publish only a complete resized artifact; the raw download is
        # adapter scratch data and incomplete stages are discarded.
        with ArtifactPublication.file(dst).stage() as stage:
            resized = stage.path
            format_adapter = (
                resized.with_suffix(resized.suffix + f".{entry.format}")
                if entry.format in ("exr", "hdr")
                else None
            )
            try:
                if entry.format == "png":
                    from PIL import Image

                    with Image.open(raw) as img:
                        source_pixels = int(img.width) * int(img.height)
                        if source_pixels > MAX_IMAGE_PIXELS:
                            raise FetchError(
                                "HDRI decoded pixels exceed resource limit: "
                                f"observed {source_pixels}, limit {MAX_IMAGE_PIXELS}"
                            )
                        img.thumbnail((target_w, target_h), _lanczos_resampling(Image))
                        # Force exact dimensions for non-2:1 sources.
                        if img.size != (target_w, target_h):
                            resized_img = img.resize(
                                (target_w, target_h),
                                _lanczos_resampling(Image),
                            )
                            try:
                                resized_img.save(resized, format="PNG")
                            finally:
                                resized_img.close()
                        else:
                            img.save(resized, format="PNG")
                elif format_adapter is not None:
                    _resize_via_imageio(
                        raw,
                        format_adapter,
                        target_w=target_w,
                        target_h=target_h,
                        fmt=entry.format,
                    )
                    shutil.copyfile(format_adapter, resized)
                else:
                    raise FetchError(f"unsupported HDRI format: {entry.format!r}")
            finally:
                if format_adapter is not None:
                    format_adapter.unlink(missing_ok=True)
            stage.publish()
    finally:
        raw.unlink(missing_ok=True)
