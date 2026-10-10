"""Event-camera-specific dataset importers.

This module intentionally stays vendor-light. Prophesee RAW EVT3 is decoded
directly from the public 16-bit word format into SRB's canonical
``EVENT_HDF5`` layout so reference captures can be converted without requiring
Metavision/OpenEB on the host that performs the mirror-prep step.
"""

from __future__ import annotations

import operator
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

import numpy as np

from srb.core.sensor.event_camera import EventEpisodeMetadata, EventHDF5Writer
from srb.datasets._evt3 import Evt3Decoder, Evt3Event, Evt3ProtocolError
from srb.utils.resource_budget import (
    MAX_EVENT_BATCH_ROWS,
    MAX_EVENT_ROWS,
    MAX_EVT3_INPUT_BYTES,
    ResourceLimitError,
    require_file_size,
)

_HEADER_LINE_LIMIT = 256
_HEADER_LINE_BYTES = 64 * 1024
_HEADER_TOTAL_BYTES = 1024 * 1024
_READ_CHUNK_BYTES = 4 * 1024 * 1024
_DEFAULT_BATCH_EVENTS = 1_000_000
_WIDTH_RE = re.compile(r"(?:^|[;,\s])width=(\d+)(?:$|[;,\s])")
_HEIGHT_RE = re.compile(r"(?:^|[;,\s])height=(\d+)(?:$|[;,\s])")
_GEOMETRY_RE = re.compile(r"%\s*geometry\s+(\d+)x(\d+)")
_FORMAT_RE = re.compile(r"%\s*(?:format\s+EVT3(?:$|[;\s])|evt\s+3(?:\.0)?\s*$)")
_T0_LINE_RE = re.compile(r"% srb_t0_ns (\d{1,19})")
_T0_KEY_RE = re.compile(r"%\s*srb_t0_ns(?![A-Za-z0-9_])")
#: RAW header key written by ``to_prophesee_raw_evt3``: the nanosecond offset
#: subtracted from every event before encoding. Import adds it back.
SRB_T0_HEADER_KEY = "srb_t0_ns"
_INT64_MAX = (1 << 63) - 1


@dataclass(frozen=True)
class PropheseeRawEvt3ImportResult:
    """Summary of one RAW EVT3 -> EVENT_HDF5 import."""

    input_path: Path
    output_path: Path
    event_count: int
    resolution: tuple[int, int]
    time_wraps: int
    ignored_words: int
    invalid_events: int
    #: Words skipped before the first TIME_HIGH (mid-stream capture start).
    skipped_leading_words: int = 0
    #: ``% srb_t0_ns`` offset added to every decoded timestamp (0 if absent).
    time_offset_ns: int = 0


def from_prophesee_raw_evt3(
    in_raw: Path | str,
    out_h5: Path | str,
    *,
    task_id: str = "",
    policy_id: str = "",
    sensor_preset: str = "imx636_nominal",
    batch_events: int = _DEFAULT_BATCH_EVENTS,
) -> PropheseeRawEvt3ImportResult:
    """Convert a Prophesee RAW EVT3 recording to canonical EVENT_HDF5.

    The decoder supports CD address, vector, and timestamp words. It skips
    pre-timestamp words, counts non-CD words, and rejects malformed word
    sequences, invalid coordinates, or decreasing timestamps with
    :class:`~srb.datasets._evt3.Evt3ProtocolError`. SRB exports may include
    ``% srb_t0_ns``; its offset is restored during import.
    """
    raw_path = Path(in_raw)
    out_path = Path(out_h5)
    if isinstance(batch_events, bool):
        raise ValueError("batch_events must be a positive integer")
    try:
        batch_events = operator.index(batch_events)
    except TypeError as exc:
        raise ValueError("batch_events must be a positive integer") from exc
    if not 0 < batch_events <= MAX_EVENT_BATCH_ROWS:
        raise ValueError(
            "batch_events must be a positive integer no greater than "
            f"{MAX_EVENT_BATCH_ROWS}"
        )

    require_file_size(
        raw_path,
        resource="RAW EVT3 input",
        max_bytes=MAX_EVT3_INPUT_BYTES,
    )

    with raw_path.open("rb") as f:
        opened_size = os.fstat(f.fileno()).st_size
        if opened_size > MAX_EVT3_INPUT_BYTES:
            raise ResourceLimitError(
                "RAW EVT3 input",
                limit=MAX_EVT3_INPUT_BYTES,
                observed=opened_size,
            )
        header = _read_evt3_header(f)
        if not any(_FORMAT_RE.match(line) for line in header):
            raise ValueError(
                "RAW header does not declare EVT3 ('% format EVT3' or '% evt 3.0')"
            )
        resolution = _resolution_from_header(header)
        time_offset_ns = _time_offset_from_header(header)

        md = EventEpisodeMetadata(
            sensor_preset=sensor_preset,
            resolution=resolution,
            # EVT3 carries event timestamps but no SRB simulator render rate.
            # Keep the schema's positive-rate invariant with an explicit
            # import sentinel; downstream consumers must use the event
            # timestamps rather than treating this as measured capture FPS.
            sub_render_hz=1.0,
            backend_name="prophesee_raw_evt3",
            backend_version=_header_value(header, "evt") or "EVT3",
            representation="raw",
            time_bins=1,
            episode_id=0,
            policy_id=policy_id,
            task_id=task_id,
        )
        decoder_ref: Evt3Decoder | None = None
        event_count = 0
        time_wraps = 0
        ignored_words = 0
        skipped_leading_words = 0
        invalid_events = 0

        with EventHDF5Writer(
            out_path,
            md,
            async_writes=False,
            max_batch_rows=batch_events,
            max_event_rows=MAX_EVENT_ROWS,
        ) as writer:
            batch = np.empty((batch_events, 4), dtype=np.int64)
            batch_len = 0

            def consume(event: Evt3Event) -> None:
                nonlocal batch_len
                t_ns = event.t_ns + time_offset_ns
                if t_ns > _INT64_MAX:
                    raise Evt3ProtocolError(
                        "timestamp_overflow",
                        f"'% {SRB_T0_HEADER_KEY}' plus event time exceeds int64",
                    )
                batch[batch_len, 0] = t_ns
                batch[batch_len, 1] = event.x
                batch[batch_len, 2] = event.y
                batch[batch_len, 3] = event.p
                batch_len += 1
                if batch_len == batch.shape[0]:
                    writer.append(batch)
                    batch_len = 0

            decoder = Evt3Decoder(
                resolution[0],
                resolution[1],
                on_event=consume,
                max_input_bytes=MAX_EVT3_INPUT_BYTES,
                max_events=MAX_EVENT_ROWS,
            )
            decoder_ref = decoder
            while True:
                chunk = f.read(_READ_CHUNK_BYTES)
                if not chunk:
                    break
                decoder.feed_bytes(chunk)
            decoder.finish()
            if batch_len:
                writer.append(batch[:batch_len])
            event_count = decoder.event_count
            time_wraps = decoder.time_wraps
            ignored_words = decoder.ignored_words
            skipped_leading_words = decoder.skipped_leading_words
            invalid_events = decoder.invalid_events
        state = decoder_ref

    if state is None:
        raise RuntimeError("EVT3 decoder did not run")

    return PropheseeRawEvt3ImportResult(
        input_path=raw_path,
        output_path=out_path,
        event_count=event_count,
        resolution=resolution,
        time_wraps=time_wraps,
        ignored_words=ignored_words,
        invalid_events=invalid_events,
        skipped_leading_words=skipped_leading_words,
        time_offset_ns=time_offset_ns,
    )


def _read_evt3_header(f: BinaryIO) -> tuple[str, ...]:
    lines: list[str] = []
    total_bytes = 0
    for _ in range(_HEADER_LINE_LIMIT):
        line = f.readline(_HEADER_LINE_BYTES + 1)
        if not line:
            break
        if len(line) > _HEADER_LINE_BYTES:
            raise ValueError(f"RAW EVT3 header line exceeds {_HEADER_LINE_BYTES} bytes")
        total_bytes += len(line)
        if total_bytes > _HEADER_TOTAL_BYTES:
            raise ValueError(f"RAW EVT3 header exceeds {_HEADER_TOTAL_BYTES} bytes")
        text = line.decode("ascii", errors="replace").rstrip("\r\n")
        lines.append(text)
        if text.startswith("% end"):
            return tuple(lines)
    raise ValueError("RAW EVT3 header missing '% end' line")


def _resolution_from_header(header: tuple[str, ...]) -> tuple[int, int]:
    joined = "\n".join(header)
    width_match = _WIDTH_RE.search(joined)
    height_match = _HEIGHT_RE.search(joined)
    if width_match and height_match:
        width = int(width_match.group(1))
        height = int(height_match.group(1))
        if 0 < width <= 2048 and 0 < height <= 2048:
            return width, height

    for line in header:
        geometry = _GEOMETRY_RE.match(line)
        if geometry:
            width = int(geometry.group(1))
            height = int(geometry.group(2))
            if 0 < width <= 2048 and 0 < height <= 2048:
                return width, height
    raise ValueError("RAW EVT3 header missing width/height in 1..2048")


def _time_offset_from_header(header: tuple[str, ...]) -> int:
    """Parse the optional ``% srb_t0_ns`` rebase offset (0 when absent)."""

    # Any line naming the key must be exactly ``% srb_t0_ns <digits>``; a
    # near-miss is rejected rather than silently read as offset 0.
    lines = [line for line in header if _T0_KEY_RE.match(line)]
    if not lines:
        return 0
    match = _T0_LINE_RE.fullmatch(lines[0]) if len(lines) == 1 else None
    if match is None:
        raise ValueError(f"RAW EVT3 header has a malformed '% {SRB_T0_HEADER_KEY}'")
    offset = int(match.group(1))
    # The exporter may write any non-negative int64 origin; each decoded event
    # is checked for overflow when the offset is added back.
    if not 0 <= offset <= _INT64_MAX:
        raise ValueError(
            f"RAW EVT3 header '% {SRB_T0_HEADER_KEY}' is outside 0..{_INT64_MAX}"
        )
    return offset


def _header_value(header: tuple[str, ...], key: str) -> str:
    prefix = f"% {key} "
    for line in header:
        if line.startswith(prefix):
            return line[len(prefix) :].strip()
    return ""


__all__ = [
    "PropheseeRawEvt3ImportResult",
    "from_prophesee_raw_evt3",
]
