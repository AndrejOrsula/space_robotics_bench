"""Isaac-free Prophesee EVT3 protocol primitives.

EVT3 uses little-endian 16-bit words: a 4-bit kind and 12-bit payload. TIME_LOW (0x6)
and TIME_HIGH (0x8) form a wrapping 24-bit microsecond timestamp. CONTINUED_4 (0x7) and
CONTINUED_12 (0xF) extend a preceding non-continuation word; OTHERS (0xE) is optionally
followed by CONTINUED_12, CONTINUED_12, CONTINUED_4, as in OpenEB's decoder.

References: https://docs.prophesee.ai/stable/data/encoding_formats/evt3.html
https://github.com/prophesee-ai/openeb/blob/5.2.0/hal/cpp/include/metavision/hal/decoders/evt3/evt3_decoder.h
https://github.com/prophesee-ai/openeb/blob/5.2.0/hal/cpp/include/metavision/hal/decoders/evt3/evt3_event_types.h
https://github.com/prophesee-ai/openeb/blob/5.2.0/hal/cpp/include/metavision/hal/decoders/evt3/evt3_validator.h

OpenEB's wrap rule increments the loop when the previous high word is at least ``(1 <<
11) + new_high``. The decoder skips leading words until the first TIME_HIGH, counts them
in ``skipped_leading_words``, and rejects non-empty streams without TIME_HIGH. Reserved
kinds fail closed; ADDR_Y bit 11 (the EVT3 system-type bit) is ignored.

The encoder accepts non-negative whole-microsecond timestamps and enforces
``MAX_EVT3_GAP_NS``. Exporters must rebase absolute simulator time to the first event
and quantise nanoseconds to microseconds (see ``srb.datasets.event_camera_export``).
"""

from __future__ import annotations

import operator
import struct
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from srb.utils.resource_budget import (
    MAX_EVENT_BATCH_ROWS,
    MAX_EVENT_ROWS,
    MAX_EVT3_BUFFER_BYTES,
    MAX_EVT3_FEED_BYTES,
    MAX_EVT3_FEED_WORDS,
    MAX_EVT3_GAP_NS,
    MAX_EVT3_INPUT_BYTES,
)

EVT3_PERIOD_US = 1 << 24
EVT3_HIGH_MASK = (1 << 12) - 1
EVT3_PAYLOAD_MASK = (1 << 12) - 1
EVT3_COORD_MASK = (1 << 11) - 1


class WordKind(IntEnum):
    ADDR_Y = 0x0
    EVT_ADDR_Y = ADDR_Y
    EXT_TRIGGER = 0x1
    ADDR_X = 0x2
    EVT_ADDR_X = ADDR_X
    VECT_BASE_X = 0x3
    VECT_12 = 0x4
    VECT_8 = 0x5
    TIME_LOW = 0x6
    CONTINUED_4 = 0x7
    TIME_HIGH = 0x8
    OTHERS = 0xE
    CONTINUED_12 = 0xF


class Evt3ProtocolError(ValueError):
    def __init__(self, failure_class: str, details: str) -> None:
        self.failure_class = str(failure_class)
        self.details = str(details)
        super().__init__(f"{self.failure_class}: {self.details}")

    @property
    def detail(self) -> str:
        return self.details


class Evt3Event(Sequence[int]):
    __slots__ = ("t_ns", "x", "y", "p")

    def __init__(self, t_ns: int, x: int, y: int, p: int) -> None:
        self.t_ns = int(t_ns)
        self.x = int(x)
        self.y = int(y)
        self.p = int(p)

    def __getitem__(self, index: int | slice) -> int | tuple[int, ...]:
        return (self.t_ns, self.x, self.y, self.p)[index]

    def __len__(self) -> int:
        return 4

    def __iter__(self) -> Iterator[int]:
        yield self.t_ns
        yield self.x
        yield self.y
        yield self.p

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Evt3Event):
            return tuple(self) == tuple(other)
        if isinstance(other, (tuple, list)):
            return tuple(self) == tuple(other)
        return NotImplemented

    def __repr__(self) -> str:
        return f"Evt3Event({self.t_ns}, {self.x}, {self.y}, {self.p})"


Event = Evt3Event


def _index(value: Any, *, failure_class: str, label: str) -> int:
    try:
        return operator.index(value)
    except TypeError as exc:
        raise Evt3ProtocolError(failure_class, f"{label} must be an integer") from exc


def _bounded_setting(value: Any, *, label: str, maximum: int) -> int:
    if isinstance(value, bool):
        raise Evt3ProtocolError("resource_limit", f"{label} must be an integer")
    parsed = _index(value, failure_class="resource_limit", label=label)
    if parsed <= 0 or parsed > maximum:
        raise Evt3ProtocolError(
            "resource_limit",
            f"{label} must be in 1..{maximum}; got {parsed}",
        )
    return parsed


def pack_word(kind: WordKind | int, payload: int) -> int:
    kind_value = _index(kind, failure_class="invalid_word_kind", label="word kind")
    payload_value = _index(
        payload, failure_class="invalid_payload", label="word payload"
    )
    if not 0 <= kind_value <= 0xF:
        raise Evt3ProtocolError("invalid_word_kind", "word kind is outside 4-bit range")
    if not 0 <= payload_value <= EVT3_PAYLOAD_MASK:
        raise Evt3ProtocolError(
            "invalid_payload", "word payload is outside 12-bit range"
        )
    return (kind_value << 12) | payload_value


def unpack_word(word: int) -> tuple[WordKind, int]:
    value = _index(word, failure_class="invalid_word", label="word")
    if not 0 <= value <= 0xFFFF:
        raise Evt3ProtocolError("invalid_word", "word is outside 16-bit range")
    try:
        kind = WordKind(value >> 12)
    except ValueError as exc:
        raise Evt3ProtocolError(
            "invalid_word_kind",
            f"reserved EVT3 word kind 0x{value >> 12:X}",
        ) from exc
    return kind, value & EVT3_PAYLOAD_MASK


def pack_word_bytes(word: int) -> bytes:
    value = _index(word, failure_class="invalid_word", label="word")
    if not 0 <= value <= 0xFFFF:
        raise Evt3ProtocolError("invalid_word", "word is outside 16-bit range")
    return struct.pack("<H", value)


def words_to_bytes(words: Iterable[int]) -> bytes:
    payload = bytearray()
    for word in words:
        if len(payload) + 2 > MAX_EVT3_BUFFER_BYTES:
            raise Evt3ProtocolError(
                "resource_limit",
                f"encoded EVT3 byte buffer exceeds {MAX_EVT3_BUFFER_BYTES} bytes",
            )
        payload.extend(pack_word_bytes(word))
    return bytes(payload)


pack_evt3_word = pack_word
unpack_evt3_word = unpack_word


class Evt3ByteFramer:
    def __init__(self, *, max_feed_bytes: int = MAX_EVT3_FEED_BYTES) -> None:
        self._max_feed_bytes = _bounded_setting(
            max_feed_bytes,
            label="max_feed_bytes",
            maximum=MAX_EVT3_FEED_BYTES,
        )
        self._carry = b""
        self._finished = False

    def feed(self, data: bytes | bytearray | memoryview) -> tuple[int, ...]:
        if self._finished:
            raise Evt3ProtocolError("invalid_state", "feed after framer finish")
        try:
            incoming_bytes = memoryview(data).nbytes
        except (TypeError, ValueError) as exc:
            raise Evt3ProtocolError(
                "invalid_bytes", "input must be bytes-like"
            ) from exc
        if incoming_bytes > self._max_feed_bytes:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 feed has {incoming_bytes} bytes, exceeding "
                f"max_feed_bytes {self._max_feed_bytes}",
            )
        payload = self._carry + bytes(data)
        even_length = len(payload) & ~1
        self._carry = payload[even_length:]
        complete = payload[:even_length]
        return tuple(int(word[0]) for word in struct.iter_unpack("<H", complete))

    def finish(self) -> None:
        self._finished = True
        if self._carry:
            self._carry = b""
            raise Evt3ProtocolError(
                "odd_eof", "EVT3 byte stream ended with one trailing byte"
            )


Evt3WordFramer = Evt3ByteFramer


class Evt3Decoder:
    def __init__(
        self,
        width: int,
        height: int,
        *,
        on_event: Callable[[Evt3Event], None] | None = None,
        max_input_bytes: int = MAX_EVT3_INPUT_BYTES,
        max_events: int = MAX_EVENT_ROWS,
        max_time_gap_ns: int = MAX_EVT3_GAP_NS,
    ) -> None:
        self.width = self._dimension(width, "width")
        self.height = self._dimension(height, "height")
        self.on_event = on_event
        self._framer = Evt3ByteFramer()
        self.max_input_bytes = _bounded_setting(
            max_input_bytes,
            label="max_input_bytes",
            maximum=MAX_EVT3_INPUT_BYTES,
        )
        self.max_events = _bounded_setting(
            max_events,
            label="max_events",
            maximum=MAX_EVENT_ROWS,
        )
        self.max_time_gap_ns = _bounded_setting(
            max_time_gap_ns,
            label="max_time_gap_ns",
            maximum=MAX_EVT3_GAP_NS,
        )
        self._input_bytes = 0
        self._word_count = 0
        self._finished = False
        self._has_non_continuation = False
        self._continuation_open = False
        self._others_step = 0
        self._others_payload = 0
        self.last_others_payload: int | None = None
        self.time_high = 0
        self.time_low = 0
        self.time_high_seen = False
        self.time_wraps = 0
        self._loop_count = 0
        self.y = 0
        self._y_seen = False
        self.vect_x = 0
        self.vect_p = 0
        self._vector_seen = False
        self.event_count = 0
        self.ignored_words = 0
        self.skipped_leading_words = 0
        self.invalid_events = 0
        self._last_event_time_us: int | None = None

    @staticmethod
    def _dimension(value: int, label: str) -> int:
        value = _index(value, failure_class="invalid_state", label=label)
        if not 0 < value <= 2048:
            raise Evt3ProtocolError("invalid_state", f"{label} must be in 1..2048")
        return value

    @property
    def timestamp_us(self) -> int:
        return self._loop_count * EVT3_PERIOD_US | self.time_high << 12 | self.time_low

    @property
    def time_ns(self) -> int:
        return self.timestamp_us * 1000

    def feed(self, data: bytes | bytearray | memoryview) -> tuple[Evt3Event, ...]:
        return self.feed_bytes(data)

    def feed_bytes(self, data: bytes | bytearray | memoryview) -> tuple[Evt3Event, ...]:
        if self._finished:
            raise Evt3ProtocolError("invalid_state", "feed after decoder finish")
        try:
            incoming_bytes = memoryview(data).nbytes
        except (TypeError, ValueError) as exc:
            raise Evt3ProtocolError(
                "invalid_bytes", "input must be bytes-like"
            ) from exc
        observed_bytes = self._input_bytes + incoming_bytes
        if observed_bytes > self.max_input_bytes:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 input exceeds max_input_bytes {self.max_input_bytes}",
            )
        self._input_bytes = observed_bytes
        return self.feed_words(self._framer.feed(data))

    def feed_words(self, words: Iterable[int]) -> tuple[Evt3Event, ...]:
        if self._finished:
            raise Evt3ProtocolError("invalid_state", "feed after decoder finish")
        emitted: list[Evt3Event] = []
        for call_word_count, raw_word in enumerate(words, start=1):
            if call_word_count > MAX_EVT3_FEED_WORDS:
                raise Evt3ProtocolError(
                    "resource_limit",
                    f"EVT3 word feed exceeds {MAX_EVT3_FEED_WORDS} words",
                )
            self._word_count += 1
            if self._word_count > self.max_input_bytes // 2:
                raise Evt3ProtocolError(
                    "resource_limit",
                    f"EVT3 word input exceeds max_input_bytes {self.max_input_bytes}",
                )
            self._handle_word(raw_word, emitted)
        return tuple(emitted)

    def _handle_word(self, raw_word: int, emitted: list[Evt3Event]) -> None:
        word = _index(raw_word, failure_class="invalid_word", label="word")
        if not 0 <= word <= 0xFFFF:
            raise Evt3ProtocolError("invalid_word", "word is outside 16-bit range")
        kind_value = word >> 12
        payload = word & EVT3_PAYLOAD_MASK
        if not self.time_high_seen and kind_value != int(WordKind.TIME_HIGH):
            # A mid-stream prefix lacks a timestamp and may split a record.
            try:
                WordKind(kind_value)
            except ValueError as exc:
                raise Evt3ProtocolError(
                    "invalid_word_kind",
                    f"reserved EVT3 word kind 0x{kind_value:X}",
                ) from exc
            self.skipped_leading_words += 1
            return
        if self._others_step == 1 and kind_value != int(WordKind.CONTINUED_12):
            # OTHERS without a payload trailer (e.g. subtype 0xFF in Gen4 recordings).
            self._others_step = 0
            self.last_others_payload = 0
        if self._others_step:
            expected = (
                WordKind.CONTINUED_12,
                WordKind.CONTINUED_12,
                WordKind.CONTINUED_4,
            )[self._others_step - 1]
            if kind_value != int(expected):
                raise Evt3ProtocolError(
                    "malformed_continuation",
                    f"OTHERS expected {expected.name}, got 0x{kind_value:X}",
                )
            if expected is WordKind.CONTINUED_4 and payload & ~0xF:
                raise Evt3ProtocolError(
                    "malformed_continuation", "CONTINUED_4 has high payload bits"
                )
            self._others_payload |= payload << (0, 12, 24)[self._others_step - 1]
            self._others_step += 1
            self.ignored_words += 1
            if self._others_step == 4:
                self._others_step = 0
                self.last_others_payload = self._others_payload
                self._others_payload = 0
                self._continuation_open = True
            return
        if kind_value in (int(WordKind.CONTINUED_4), int(WordKind.CONTINUED_12)):
            if not self._has_non_continuation or self._continuation_open:
                raise Evt3ProtocolError(
                    "orphan_continuation",
                    "continuation has no available preceding non-continuation word",
                )
            if kind_value == int(WordKind.CONTINUED_4) and payload & ~0xF:
                raise Evt3ProtocolError(
                    "malformed_continuation", "CONTINUED_4 has high payload bits"
                )
            self._continuation_open = True
            self.ignored_words += 1
            return
        try:
            kind = WordKind(kind_value)
        except ValueError as exc:
            raise Evt3ProtocolError(
                "invalid_word_kind",
                f"reserved EVT3 word kind 0x{kind_value:X}",
            ) from exc
        self._has_non_continuation = True
        self._continuation_open = False
        if kind is WordKind.TIME_HIGH:
            if self.time_high_seen and self.time_high >= (1 << 11) + payload:
                self._loop_count += 1
                self.time_wraps += 1
            self.time_high = payload
            self.time_high_seen = True
        elif kind is WordKind.TIME_LOW:
            self.time_low = payload
        elif kind is WordKind.ADDR_Y:
            self._require_time()
            # Bit 11 is the EVT3 system-type ("orig") bit, not a y bit.
            y = payload & EVT3_COORD_MASK
            if y >= self.height:
                raise Evt3ProtocolError("invalid_coordinate", "invalid y coordinate")
            self.y = y
            self._y_seen = True
            self._vector_seen = False
        elif kind is WordKind.ADDR_X:
            self._require_time_and_y()
            x = payload & EVT3_COORD_MASK
            if x >= self.width:
                raise Evt3ProtocolError("invalid_coordinate", "invalid x coordinate")
            self._emit_event(x, 1 if payload & (1 << 11) else -1, emitted)
            self._vector_seen = False
        elif kind is WordKind.VECT_BASE_X:
            self._require_time_and_y()
            x = payload & EVT3_COORD_MASK
            if x >= self.width:
                raise Evt3ProtocolError("invalid_coordinate", "invalid vector base x")
            self.vect_x = x
            self.vect_p = 1 if payload & (1 << 11) else -1
            self._vector_seen = True
        elif kind is WordKind.VECT_12:
            self._emit_vector(payload, 12, emitted)
        elif kind is WordKind.VECT_8:
            if payload & ~0xFF:
                raise Evt3ProtocolError("invalid_state", "VECT_8 uses reserved bits")
            self._emit_vector(payload, 8, emitted)
        elif kind is WordKind.EXT_TRIGGER:
            self.ignored_words += 1
        elif kind is WordKind.OTHERS:
            self.ignored_words += 1
            self._others_step = 1
            self._others_payload = 0
        else:
            raise Evt3ProtocolError(
                "invalid_state", f"unsupported EVT3 word kind {kind.name}"
            )

    def _require_time(self) -> None:
        if not self.time_high_seen:
            raise Evt3ProtocolError(
                "invalid_state", "CD address encountered before TIME_HIGH"
            )

    def _require_time_and_y(self) -> None:
        self._require_time()
        if not self._y_seen:
            raise Evt3ProtocolError(
                "invalid_state", "EVT_ADDR_X encountered before EVT_ADDR_Y"
            )

    def _emit_vector(
        self, valid: int, vector_width: int, emitted: list[Evt3Event]
    ) -> None:
        self._require_time_and_y()
        if not self._vector_seen:
            raise Evt3ProtocolError(
                "invalid_state", "vector encountered before VECT_BASE_X"
            )
        active_x = [
            self.vect_x + index for index in range(vector_width) if valid & (1 << index)
        ]
        if any(x >= self.width for x in active_x):
            raise Evt3ProtocolError(
                "invalid_coordinate", "vector x is outside sensor width"
            )
        for x in active_x:
            self._emit_event(x, self.vect_p, emitted)
        self.vect_x += vector_width

    def _emit_event(self, x: int, p: int, emitted: list[Evt3Event]) -> None:
        timestamp_us = self.timestamp_us
        if (
            self._last_event_time_us is not None
            and timestamp_us < self._last_event_time_us
        ):
            raise Evt3ProtocolError("nonmonotonic_timestamp", "CD timestamp decreased")
        timestamp_ns = timestamp_us * 1000
        # The limit bounds gaps between consecutive events; a quiet lead-in
        # before the first event is not a gap.
        if (
            self._last_event_time_us is not None
            and timestamp_ns - self._last_event_time_us * 1000 > self.max_time_gap_ns
        ):
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 inter-event timestamp gap exceeds {self.max_time_gap_ns} ns",
            )
        if self.event_count >= self.max_events:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 event count exceeds max_events {self.max_events}",
            )
        if self.on_event is None and len(emitted) >= MAX_EVENT_BATCH_ROWS:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 buffered event batch exceeds {MAX_EVENT_BATCH_ROWS} rows",
            )
        event = Evt3Event(timestamp_ns, x, self.y, p)
        self._last_event_time_us = timestamp_us
        self.event_count += 1
        if self.on_event is not None:
            self.on_event(event)
        else:
            emitted.append(event)

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._framer.finish()
        if self._others_step > 1:
            self._others_step = 0
            raise Evt3ProtocolError(
                "incomplete_continuation",
                "OTHERS ended before CONTINUED_12, CONTINUED_12, CONTINUED_4",
            )
        if self._word_count and not self.time_high_seen:
            raise Evt3ProtocolError(
                "missing_time_high",
                f"EVT3 stream has {self._word_count} words but no TIME_HIGH",
            )


StreamingEvt3Decoder = Evt3Decoder


class Evt3Encoder:
    def __init__(
        self,
        width: int | None = None,
        height: int | None = None,
        *,
        on_word: Callable[[int], None] | None = None,
        max_events: int = MAX_EVENT_ROWS,
        max_time_gap_ns: int = MAX_EVT3_GAP_NS,
    ) -> None:
        if (width is None) != (height is None):
            raise Evt3ProtocolError("invalid_state", "width and height must be paired")
        self.width = None if width is None else Evt3Decoder._dimension(width, "width")
        self.height = (
            None if height is None else Evt3Decoder._dimension(height, "height")
        )
        self.on_word = on_word
        self.max_events = _bounded_setting(
            max_events,
            label="max_events",
            maximum=MAX_EVENT_ROWS,
        )
        self.max_time_gap_ns = _bounded_setting(
            max_time_gap_ns,
            label="max_time_gap_ns",
            maximum=MAX_EVT3_GAP_NS,
        )
        self._high_initialized = False
        self._current_loop = 0
        self._current_high = 0
        self._current_low: int | None = None
        self._current_y: int | None = None
        self._last_timestamp_ns: int | None = None
        self.event_count = 0

    def encode_event(self, event: Any) -> Iterator[int]:
        t_ns, x, y, p = self._coerce_event(event)
        self._validate_event(t_ns, x, y, p)
        t_us = t_ns // 1000
        target_loop = t_us // EVT3_PERIOD_US
        target_high = (t_us >> 12) & EVT3_HIGH_MASK
        target_low = t_us & EVT3_PAYLOAD_MASK
        high_advanced = False
        if not self._high_initialized:
            yield from self._yield_word(pack_word(WordKind.TIME_HIGH, 0))
            self._high_initialized = True
            high_advanced = True
        while self._current_loop < target_loop or self._current_high < target_high:
            if self._current_high == EVT3_HIGH_MASK:
                self._current_high = 0
                self._current_loop += 1
            else:
                self._current_high += 1
            high_advanced = True
            yield from self._yield_word(
                pack_word(WordKind.TIME_HIGH, self._current_high)
            )
        if (
            self._current_low is None
            or self._current_low != target_low
            or high_advanced
        ):
            self._current_low = target_low
            yield from self._yield_word(pack_word(WordKind.TIME_LOW, target_low))
        if self._current_y is None or self._current_y != y:
            self._current_y = y
            yield from self._yield_word(pack_word(WordKind.ADDR_Y, y))
        yield from self._yield_word(
            pack_word(WordKind.ADDR_X, x | ((1 << 11) if p > 0 else 0))
        )
        self._last_timestamp_ns = t_ns
        self.event_count += 1

    def iter_words(self, events: Iterable[Any]) -> Iterator[int]:
        for event in events:
            yield from self.encode_event(event)

    def feed(self, events: Iterable[Any]) -> tuple[int, ...]:
        return tuple(self.iter_words(events))

    def finish(self) -> None:
        return None

    def _yield_word(self, word: int) -> Iterator[int]:
        if self.on_word is not None:
            self.on_word(word)
        else:
            yield word

    def _coerce_event(self, event: Any) -> tuple[int, int, int, int]:
        if isinstance(event, Mapping):
            try:
                values = tuple(event[name] for name in ("t_ns", "x", "y", "p"))
            except KeyError as exc:
                raise Evt3ProtocolError(
                    "invalid_event", "event mapping has wrong fields"
                ) from exc
        elif all(hasattr(event, name) for name in ("t_ns", "x", "y")):
            name = "p" if hasattr(event, "p") else "polarity"
            if not hasattr(event, name):
                raise Evt3ProtocolError("invalid_event", "event has no polarity")
            values = (event.t_ns, event.x, event.y, getattr(event, name))
        else:
            try:
                values = tuple(event)
            except TypeError as exc:
                raise Evt3ProtocolError(
                    "invalid_event", "event must be a 4-field iterable"
                ) from exc
            if len(values) != 4:
                raise Evt3ProtocolError("invalid_event", "event must have four fields")
        return tuple(
            _index(value, failure_class="invalid_event", label=label)
            for value, label in zip(values, ("t_ns", "x", "y", "p"), strict=True)
        )  # type: ignore[return-value]

    def _validate_event(self, t_ns: int, x: int, y: int, p: int) -> None:
        if t_ns < 0:
            raise Evt3ProtocolError(
                "negative_timestamp", "timestamp must be non-negative"
            )
        if t_ns % 1000:
            raise Evt3ProtocolError(
                "sub_microsecond_timestamp", "timestamp must be whole microseconds"
            )
        if self._last_timestamp_ns is not None and t_ns < self._last_timestamp_ns:
            raise Evt3ProtocolError(
                "nonmonotonic_timestamp", "input timestamps must be nondecreasing"
            )
        previous_timestamp_ns = self._last_timestamp_ns or 0
        if t_ns - previous_timestamp_ns > self.max_time_gap_ns:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 inter-event timestamp gap exceeds {self.max_time_gap_ns} ns",
            )
        if self.event_count >= self.max_events:
            raise Evt3ProtocolError(
                "resource_limit",
                f"EVT3 event count exceeds max_events {self.max_events}",
            )
        if not 0 <= x <= EVT3_COORD_MASK or (
            self.width is not None and x >= self.width
        ):
            raise Evt3ProtocolError(
                "invalid_coordinate", "x coordinate is outside sensor"
            )
        if not 0 <= y <= EVT3_COORD_MASK or (
            self.height is not None and y >= self.height
        ):
            raise Evt3ProtocolError(
                "invalid_coordinate", "y coordinate is outside sensor"
            )
        if p not in (-1, 1):
            raise Evt3ProtocolError(
                "invalid_polarity", "polarity must be exactly -1 or +1"
            )


StreamingEvt3Encoder = Evt3Encoder


def iter_evt3_words(
    events: Iterable[Any],
    *,
    width: int | None = None,
    height: int | None = None,
) -> Iterator[int]:
    return Evt3Encoder(width, height).iter_words(events)


def decode_evt3_words(
    words: Iterable[int],
    *,
    width: int,
    height: int,
) -> tuple[Evt3Event, ...]:
    decoder = Evt3Decoder(width, height)
    result = decoder.feed_words(words)
    decoder.finish()
    return result


@dataclass(frozen=True)
class Evt3ValidationResult:
    event_count: int
    time_wraps: int
    ignored_words: int
    skipped_leading_words: int = 0


def validate_evt3_bytes(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    expected_count: int | None = None,
) -> Evt3ValidationResult:
    decoder = Evt3Decoder(width, height)
    decoder.feed_bytes(data)
    decoder.finish()
    if expected_count is not None and decoder.event_count != expected_count:
        raise Evt3ProtocolError(
            "event_count_mismatch",
            f"decoded {decoder.event_count} events, expected {expected_count}",
        )
    return Evt3ValidationResult(
        event_count=decoder.event_count,
        time_wraps=decoder.time_wraps,
        ignored_words=decoder.ignored_words,
        skipped_leading_words=decoder.skipped_leading_words,
    )


__all__ = [
    "EVT3_PERIOD_US",
    "Evt3ByteFramer",
    "Evt3Decoder",
    "Evt3Encoder",
    "Evt3Event",
    "Evt3ProtocolError",
    "Evt3ValidationResult",
    "Evt3WordFramer",
    "Event",
    "StreamingEvt3Decoder",
    "StreamingEvt3Encoder",
    "WordKind",
    "decode_evt3_words",
    "iter_evt3_words",
    "pack_evt3_word",
    "pack_word",
    "pack_word_bytes",
    "unpack_evt3_word",
    "unpack_word",
    "validate_evt3_bytes",
    "words_to_bytes",
]
