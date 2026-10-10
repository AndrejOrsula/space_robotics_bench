"""Bounded, preflighted Parquet export for rewindable row providers."""

from __future__ import annotations

import operator
import os
import sys
import tempfile
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import closing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, cast

from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    ByteBudget,
    ResourceLimitError,
)

MAX_BATCH_ROWS = 1_024
MAX_BATCH_BYTES = 64 * (1 << 20)


@dataclass
class _TypeInfo:
    """Small schema descriptor retained for one column, not one row."""

    kind: str
    arrow_type: Any = None
    nullable: bool = False
    minimum: int | None = None
    maximum: int | None = None
    python_int: bool = False
    item: _TypeInfo | None = None
    fields: dict[str, _TypeInfo] = field(default_factory=dict)


class _RowPass:
    """Own one provider iterator for the duration of one writer pass."""

    def __init__(
        self,
        row_factory: Callable[[], Iterator[Mapping[str, Any]]],
    ) -> None:
        self._row_factory = row_factory
        self._iterator: Iterator[Mapping[str, Any]] | None = None

    def __enter__(self) -> Iterator[Mapping[str, Any]]:
        rows = self._row_factory()
        try:
            self._iterator = iter(rows)
        except TypeError as exc:
            close = getattr(rows, "close", None)
            if callable(close):
                close()
            raise TypeError("row_factory must return an iterable of mappings") from exc
        return self._iterator

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> Literal[False]:
        iterator = self._iterator
        close = getattr(iterator, "close", None)
        if callable(close):
            close()
        return False


@dataclass
class _ParquetPlan:
    path: Path
    row_factory: Callable[[], Iterator[Mapping[str, Any]]]
    builder: Any
    schema: Any
    count: int
    arrow_bytes: int
    schema_buffer_bytes: int
    metadata_bytes: int


def _retained_plan_bytes(plan: _ParquetPlan) -> int:
    return plan.builder.memory_bytes() + plan.schema_buffer_bytes + plan.metadata_bytes


def preflight_parquet_tables(
    tables: Sequence[tuple[Path, Callable[[], Iterator[Mapping[str, Any]]]]],
    *,
    product_limit_bytes: int | None = None,
) -> tuple[_ParquetPlan, ...]:
    """Preflight tables with one Arrow product budget before creating writers.

    Only schema descriptors and scalar totals survive between batches.
    """
    if product_limit_bytes is None:
        product_limit_bytes = MAX_DECODED_AGGREGATE_BYTES
    if (
        isinstance(product_limit_bytes, bool)
        or not isinstance(product_limit_bytes, int)
        or product_limit_bytes <= 0
    ):
        raise ValueError("product_limit_bytes must be a positive integer")
    if product_limit_bytes > MAX_DECODED_AGGREGATE_BYTES:
        raise ValueError(
            "product_limit_bytes cannot exceed the decoded aggregate ceiling"
        )
    import pyarrow as pa

    plans: list[_ParquetPlan] = []
    product_budget = ByteBudget("Parquet decoded product bytes", product_limit_bytes)
    for path, row_factory in tables:
        retained_plan_bytes = sum(_retained_plan_bytes(plan) for plan in plans)
        plans.append(
            _preflight_table(
                Path(path),
                row_factory,
                pa,
                retained_plan_bytes=retained_plan_bytes,
                product_budget=product_budget,
            )
        )
    return tuple(plans)


def write_preflighted_parquet_tables(plans: Sequence[_ParquetPlan]) -> tuple[int, ...]:
    """Write plans returned by :func:`preflight_parquet_tables` atomically."""
    retained_plan_bytes = sum(_retained_plan_bytes(plan) for plan in plans)
    return tuple(
        _write_plan(
            plan,
            retained_plan_bytes=(retained_plan_bytes - _retained_plan_bytes(plan)),
        )
        for plan in plans
    )


def _preflight_table(
    path: Path,
    row_factory: Callable[..., Any],
    pa: Any,
    *,
    retained_plan_bytes: int,
    product_budget: ByteBudget,
) -> _ParquetPlan:
    if not callable(row_factory):
        raise TypeError("row_factory must be callable")
    builder = _SchemaBuilder(pa)
    first_count = 0
    with _RowPass(row_factory) as rows:
        while True:
            try:
                raw_row = next(rows)
            except StopIteration:
                break
            row = _owned_row(raw_row)
            row_bytes = _estimate_value(row)
            builder.observe(row)
            _check_decoded_bytes(
                row_bytes + builder.memory_bytes() + retained_plan_bytes
            )
            first_count += 1
            del raw_row, row, row_bytes
    schema = builder.schema()
    schema_buffer_bytes = _estimate_schema_buffers(schema, pa)
    metadata_budget = _estimate_metadata_buffers(builder)
    _check_array_bytes(schema_buffer_bytes + metadata_budget)
    _check_peak_bytes(
        retained_plan_bytes
        + builder.memory_bytes()
        + schema_buffer_bytes
        + metadata_budget
    )
    metadata = _pandas_schema_metadata(builder, schema, pa)
    metadata_bytes = _metadata_bytes(metadata)
    _check_array_bytes(schema_buffer_bytes + metadata_bytes)
    _check_peak_bytes(
        retained_plan_bytes
        + builder.memory_bytes()
        + schema_buffer_bytes
        + metadata_bytes
    )
    schema = schema.with_metadata(metadata)

    count = 0
    total_arrow_bytes = 0
    batches = _batches(row_factory, builder, schema, pa)
    with closing(batches):
        try:
            current_batch = next(batches)
        except StopIteration:
            current_batch = None
        while current_batch is not None:
            rows, source_bytes, row_cost, pending_row_bytes = current_batch
            table = _batch_table(
                rows,
                source_bytes,
                row_cost,
                builder,
                schema,
                pa,
                schema_buffer_bytes=schema_buffer_bytes,
                metadata_bytes=metadata_bytes,
                retained_state_bytes=retained_plan_bytes + builder.memory_bytes(),
                pending_row_bytes=pending_row_bytes,
            )
            actual_bytes = _table_buffer_bytes(table)
            _check_array_bytes(actual_bytes)
            _check_peak_bytes(
                source_bytes
                + pending_row_bytes
                + _estimate_conversion(rows, source_bytes, builder)
                + sum(_estimate_arrow_row(row, schema, pa) for row in rows)
                + actual_bytes
                + schema_buffer_bytes
                + metadata_bytes
                + retained_plan_bytes
                + builder.memory_bytes()
            )
            product_budget.consume(actual_bytes)
            total_arrow_bytes += actual_bytes
            count += len(rows)
            del table, rows, current_batch
            try:
                current_batch = next(batches)
            except StopIteration:
                current_batch = None
    if count != first_count:
        raise ValueError(
            "row_factory is not rewindable: schema and sizing passes yielded "
            f"different row counts ({first_count} != {count})"
        )
    return _ParquetPlan(
        path,
        row_factory,
        builder,
        schema,
        count,
        total_arrow_bytes,
        schema_buffer_bytes,
        metadata_bytes,
    )


def _batches(row_factory: Callable[..., Any], builder: Any, schema: Any, pa: Any):
    batch: list[dict[str, Any]] = []
    source_bytes = 0
    cost = 0
    with _RowPass(row_factory) as rows:
        while True:
            if batch and len(batch) >= MAX_BATCH_ROWS:
                yield batch, source_bytes, cost, 0
                batch, source_bytes, cost = [], 0, 0
            try:
                raw_row = next(rows)
            except StopIteration:
                break
            row = _owned_row(raw_row)
            builder.validate_row_keys(row)
            row_bytes = _estimate_value(row)
            arrow_bytes = _estimate_arrow_row(row, schema, pa)
            _check_array_bytes(arrow_bytes)
            row_cost = _estimate_row_cost(row, builder) + arrow_bytes
            if batch and (
                len(batch) >= MAX_BATCH_ROWS or cost + row_cost > MAX_BATCH_BYTES
            ):
                yield batch, source_bytes, cost, row_cost
                batch, source_bytes, cost = [], 0, 0
            batch.append(row)
            source_bytes += row_bytes
            cost += row_cost
            del raw_row, row, row_bytes, arrow_bytes, row_cost
        if batch:
            yield batch, source_bytes, cost, 0


def _batch_table(
    rows: list[dict[str, Any]],
    source_bytes: int,
    row_cost: int,
    builder: Any,
    schema: Any,
    pa: Any,
    *,
    schema_buffer_bytes: int = 0,
    metadata_bytes: int = 0,
    retained_state_bytes: int = 0,
    pending_row_bytes: int = 0,
) -> Any:
    conversion_bytes = _estimate_conversion(rows, source_bytes, builder)
    arrow_estimate = sum(_estimate_arrow_row(row, schema, pa) for row in rows)
    _check_array_bytes(arrow_estimate)
    fixed_bytes = schema_buffer_bytes + metadata_bytes + retained_state_bytes
    _check_peak_bytes(
        source_bytes
        + pending_row_bytes
        + conversion_bytes
        + arrow_estimate
        + fixed_bytes
    )
    converted = [builder.coerce_row(row, schema) for row in rows]
    table = pa.Table.from_pylist(converted, schema=schema)
    _check_peak_bytes(
        source_bytes
        + pending_row_bytes
        + conversion_bytes
        + arrow_estimate
        + _table_buffer_bytes(table)
        + fixed_bytes
    )
    return table


def _write_plan(plan: _ParquetPlan, *, retained_plan_bytes: int) -> int:
    import pyarrow.parquet as pq

    temporary = _make_temporary(plan.path)
    writer = None
    count = 0
    arrow_bytes = 0
    try:
        writer = cast(Any, pq.ParquetWriter)(temporary, plan.schema)
        batches = _batches(plan.row_factory, plan.builder, plan.schema, _pyarrow())
        with closing(batches):
            try:
                current_batch = next(batches)
            except StopIteration:
                current_batch = None
            while current_batch is not None:
                rows, source_bytes, row_cost, pending_row_bytes = current_batch
                table = _batch_table(
                    rows,
                    source_bytes,
                    row_cost,
                    plan.builder,
                    plan.schema,
                    _pyarrow(),
                    schema_buffer_bytes=plan.schema_buffer_bytes,
                    metadata_bytes=plan.metadata_bytes,
                    retained_state_bytes=retained_plan_bytes
                    + plan.builder.memory_bytes(),
                    pending_row_bytes=pending_row_bytes,
                )
                batch_bytes = _table_buffer_bytes(table)
                if arrow_bytes + batch_bytes > plan.arrow_bytes:
                    raise ValueError(
                        "Parquet output grew beyond its preflight declaration"
                    )
                writer.write_table(table)
                count += table.num_rows
                arrow_bytes += batch_bytes
                del table, rows, current_batch
                try:
                    current_batch = next(batches)
                except StopIteration:
                    current_batch = None
        if count != plan.count or arrow_bytes != plan.arrow_bytes:
            raise ValueError("Parquet output changed after preflight")
        closing_writer: Any = writer
        writer = None
        closing_writer.close()
        os.replace(temporary, plan.path)
        return count
    finally:
        try:
            if writer is not None:
                writer.close()
        finally:
            temporary.unlink(missing_ok=True)


def _pyarrow() -> Any:
    import pyarrow as pa

    return pa


def _check_peak_bytes(observed: int) -> None:
    if observed > MAX_DECODED_AGGREGATE_BYTES:
        raise ResourceLimitError(
            "Parquet live batch bytes",
            limit=MAX_DECODED_AGGREGATE_BYTES,
            observed=observed,
        )


def _owned_row(raw_row: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(raw_row, Mapping):
        raise TypeError("each Parquet row must be a mapping")
    row = dict(raw_row)
    if any(not isinstance(key, str) for key in row):
        raise TypeError("Parquet column names must be strings")
    return row


def _make_temporary(destination: Path) -> Path:
    descriptor, name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".tmp",
        dir=destination.parent,
    )
    try:
        os.close(descriptor)
    except OSError:
        os.unlink(name)
        raise
    return Path(name)


def _check_decoded_bytes(observed: int) -> None:
    if observed > MAX_DECODED_AGGREGATE_BYTES:
        raise ResourceLimitError(
            "Parquet decoded aggregate bytes",
            limit=MAX_DECODED_AGGREGATE_BYTES,
            observed=observed,
        )


def _pandas_schema_metadata(builder: _SchemaBuilder, schema: Any, pa: Any) -> Any:
    import pandas as pd

    columns = {
        name: pd.Series(
            [],
            dtype=_pandas_dtype(info, schema.field(name).type, pa),
        )
        for name, info in builder.fields.items()
    }
    empty_frame = pd.DataFrame(columns, columns=list(builder.fields))
    return (
        pa.Table.from_pandas(
            empty_frame,
            schema=schema,
            preserve_index=False,
        ).schema.metadata
        or {}
    )


def _pandas_dtype(info: _TypeInfo, arrow_type: Any, pa: Any) -> Any:
    if pa.types.is_boolean(arrow_type):
        return "bool" if not info.nullable else "object"
    if pa.types.is_integer(arrow_type) or pa.types.is_floating(arrow_type):
        return arrow_type.to_pandas_dtype()
    if pa.types.is_timestamp(arrow_type):
        return arrow_type.to_pandas_dtype()
    return "object"


def _metadata_bytes(metadata: Any) -> int:
    if not metadata:
        return 0
    return sum(len(key) + len(value) for key, value in metadata.items())


def _estimate_metadata_buffers(builder: _SchemaBuilder) -> int:
    """Reserve a conservative bound before pandas/Arrow metadata conversion."""
    # Reserve repeated field names and fixed descriptors before Arrow creates
    # an empty table; this estimate does not inspect provider rows.
    return 512 + sum(512 + 16 * len(name) for name in builder.fields)


def _estimate_schema_buffers(schema: Any, pa: Any) -> int:
    return sum(_estimate_arrow_type_buffers(field.type, pa) for field in schema)


def _estimate_arrow_type_buffers(arrow_type: Any, pa: Any) -> int:
    if pa.types.is_struct(arrow_type):
        return 1 + sum(
            _estimate_arrow_type_buffers(field.type, pa) for field in arrow_type
        )
    if pa.types.is_list(arrow_type) or pa.types.is_large_list(arrow_type):
        offset_bytes = 8 if pa.types.is_large_list(arrow_type) else 4
        return (
            1 + offset_bytes + _estimate_arrow_type_buffers(arrow_type.value_type, pa)
        )
    if pa.types.is_fixed_size_list(arrow_type):
        return 1 + _estimate_arrow_type_buffers(arrow_type.value_type, pa)
    if pa.types.is_string(arrow_type) or pa.types.is_binary(arrow_type):
        return 1 + 4
    if pa.types.is_large_string(arrow_type) or pa.types.is_large_binary(arrow_type):
        return 1 + 8
    return 1


def _estimate_arrow_row(row: Mapping[str, Any], schema: Any, pa: Any) -> int:
    return sum(
        _estimate_arrow_value(row.get(field.name), field.type, pa) for field in schema
    )


def _estimate_arrow_value(value: Any, arrow_type: Any, pa: Any) -> int:
    if pa.types.is_struct(arrow_type):
        total = 1
        mapping = value if isinstance(value, Mapping) else {}
        for field in arrow_type:
            total += _estimate_arrow_value(mapping.get(field.name), field.type, pa)
        return total
    if pa.types.is_list(arrow_type) or pa.types.is_large_list(arrow_type):
        # Reserve two offsets per slot, including the terminal offset paid by
        # the first row.
        offset_bytes = 16 if pa.types.is_large_list(arrow_type) else 8
        total = 1 + offset_bytes
        if value is None:
            return total
        shape = _array_shape(value)
        if shape is not None:
            return (
                total
                + _estimate_arrow_array_shape(shape, arrow_type, pa)
                + _estimate_value(value)
            )
        if isinstance(value, (list, tuple)):
            return total + sum(
                _estimate_arrow_value(element, arrow_type.value_type, pa)
                for element in value
            )
        return total + _estimate_value(value)
    if pa.types.is_fixed_size_list(arrow_type):
        total = 1
        list_size = int(arrow_type.list_size)
        if value is None:
            return total + list_size * _estimate_arrow_null(arrow_type.value_type, pa)
        if isinstance(value, (list, tuple)):
            return total + sum(
                _estimate_arrow_value(element, arrow_type.value_type, pa)
                for element in value
            )
        return total + _estimate_value(value)
    if pa.types.is_string(arrow_type) or pa.types.is_binary(arrow_type):
        total = 1 + 8
        return total if value is None else total + _text_or_bytes_size(value)
    if pa.types.is_large_string(arrow_type) or pa.types.is_large_binary(arrow_type):
        total = 1 + 16
        return total if value is None else total + _text_or_bytes_size(value)
    if pa.types.is_null(arrow_type):
        return 1
    if value is None:
        return _estimate_arrow_null(arrow_type, pa)
    if pa.types.is_boolean(arrow_type):
        return 2
    if pa.types.is_integer(arrow_type) or pa.types.is_floating(arrow_type):
        return 1 + max(1, int(arrow_type.bit_width) // 8)
    byte_width = getattr(arrow_type, "byte_width", None)
    if byte_width is not None:
        return 1 + max(1, int(byte_width))
    return 1 + _estimate_value(value)


def _estimate_arrow_array_shape(
    shape: tuple[int, ...],
    arrow_type: Any,
    pa: Any,
) -> int:
    """Account nested list buffers for an ndarray-like provider value."""
    if not shape:
        return _estimate_arrow_null(arrow_type, pa)
    elements = 1
    for dimension in shape:
        elements *= dimension
    if not (pa.types.is_list(arrow_type) or pa.types.is_large_list(arrow_type)):
        return elements * _estimate_arrow_null(arrow_type, pa)

    child_type = arrow_type.value_type
    child_shape = shape[1:]
    if pa.types.is_list(child_type) or pa.types.is_large_list(child_type):
        child_bytes = _estimate_arrow_array_shape(child_shape, child_type, pa)
        child_offset = 16 if pa.types.is_large_list(child_type) else 8
        return shape[0] * (1 + child_offset + child_bytes)
    return elements * _estimate_arrow_null(child_type, pa)


def _estimate_arrow_null(arrow_type: Any, pa: Any) -> int:
    """Reserve the typed buffers Arrow allocates for one null slot."""
    if pa.types.is_struct(arrow_type):
        return 1 + sum(_estimate_arrow_null(field.type, pa) for field in arrow_type)
    if pa.types.is_list(arrow_type) or pa.types.is_large_list(arrow_type):
        offset_bytes = 16 if pa.types.is_large_list(arrow_type) else 8
        return 1 + offset_bytes
    if pa.types.is_fixed_size_list(arrow_type):
        return 1 + int(arrow_type.list_size) * _estimate_arrow_null(
            arrow_type.value_type, pa
        )
    if pa.types.is_string(arrow_type) or pa.types.is_binary(arrow_type):
        return 1 + 8
    if pa.types.is_large_string(arrow_type) or pa.types.is_large_binary(arrow_type):
        return 1 + 16
    if pa.types.is_null(arrow_type):
        return 1
    if pa.types.is_boolean(arrow_type):
        return 2
    if pa.types.is_integer(arrow_type) or pa.types.is_floating(arrow_type):
        return 1 + max(1, int(arrow_type.bit_width) // 8)
    byte_width = getattr(arrow_type, "byte_width", None)
    if byte_width is not None:
        return 1 + max(1, int(byte_width))
    return 1


def _text_or_bytes_size(value: Any) -> int:
    if isinstance(value, str):
        return len(value) * 4
    try:
        return len(value)
    except TypeError:
        return sys.getsizeof(value)


def _estimate_conversion(
    rows: list[dict[str, Any]],
    row_bytes: int,
    builder: _SchemaBuilder,
) -> int:
    """Reserve row copies and schema padding before Arrow sees the values."""
    del row_bytes
    estimate = sys.getsizeof(rows)
    field_count = len(builder.fields)
    for row in rows:
        estimate += sys.getsizeof(row) + _estimate_value(row)
        missing = field_count - len(row)
        if missing:
            estimate += missing * (sys.getsizeof(None) + sys.getsizeof(""))
    return estimate


def _estimate_row_cost(row: Mapping[str, Any], builder: _SchemaBuilder) -> int:
    """Estimate one row's source plus converted representation."""
    source_bytes = _estimate_value(row)
    conversion_bytes = sys.getsizeof(row) + source_bytes
    missing = len(builder.fields) - len(row)
    if missing:
        conversion_bytes += missing * (sys.getsizeof(None) + sys.getsizeof(""))
    return source_bytes + conversion_bytes


def _estimate_type(info: _TypeInfo) -> int:
    estimate = sys.getsizeof(info) + sys.getsizeof(info.kind)
    if info.arrow_type is not None:
        estimate += sys.getsizeof(info.arrow_type)
    if info.item is not None:
        estimate += _estimate_type(info.item)
    for name, child in info.fields.items():
        estimate += sys.getsizeof(name) + _estimate_type(child)
    return estimate


def _table_buffer_bytes(table: Any) -> int:
    getter = getattr(table, "get_total_buffer_size", None)
    if callable(getter):
        value = getter()
    else:
        value = getattr(table, "nbytes", 0)
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise TypeError("Parquet provider returned an invalid buffer size") from exc
    if result < 0:
        raise ValueError("Parquet provider returned a negative buffer size")
    return result


class _SchemaBuilder:
    def __init__(self, pa: Any) -> None:
        self._pa = pa
        self.fields: dict[str, _TypeInfo] = {}
        self.row_count = 0

    def observe(self, row: Mapping[str, Any]) -> None:
        present = set(row)
        for name, value in row.items():
            if name not in self.fields:
                self.fields[name] = _TypeInfo("null", nullable=self.row_count > 0)
            self.fields[name] = _observe(self.fields[name], value, self._pa, root=True)
        for name, info in self.fields.items():
            if name not in present:
                info.nullable = True
        self.row_count += 1

    def schema(self) -> Any:
        return self._pa.schema(
            [
                self._pa.field(name, _arrow_type(info, self._pa, root=True))
                for name, info in self.fields.items()
            ]
        )

    def memory_bytes(self) -> int:
        return sys.getsizeof(self.fields) + sum(
            sys.getsizeof(name) + _estimate_type(info)
            for name, info in self.fields.items()
        )

    def validate_row_keys(self, row: Mapping[str, Any]) -> None:
        unexpected = [name for name in row if name not in self.fields]
        if unexpected:
            raise ValueError(
                "row_factory changed schema between passes; unexpected columns: "
                + ", ".join(unexpected)
            )

    def coerce_row(self, row: Mapping[str, Any], schema: Any) -> dict[str, Any]:
        self.validate_row_keys(row)
        converted: dict[str, Any] = {}
        for name, value in row.items():
            converted[name] = _coerce_value(
                value,
                self.fields[name],
                schema.field(name).type,
                self._pa,
            )
        return converted


def _observe(
    current: _TypeInfo,
    value: Any,
    pa: Any,
    *,
    root: bool,
) -> _TypeInfo:
    if value is None:
        current.nullable = True
        return current
    candidate = _type_for_value(value, pa)
    return _merge(current, candidate, pa, root=root)


def _type_for_value(value: Any, pa: Any) -> _TypeInfo:
    if value is None:
        return _TypeInfo("null", nullable=True)
    if isinstance(value, bool):
        return _TypeInfo("bool", arrow_type=pa.bool_())
    if isinstance(value, str):
        return _TypeInfo("string", arrow_type=pa.string())
    if isinstance(value, (bytes, bytearray, memoryview)):
        return _TypeInfo("binary", arrow_type=pa.binary())

    scalar_dtype = getattr(value, "dtype", None)
    scalar_shape = getattr(value, "shape", None)
    if scalar_dtype is not None and scalar_shape == ():
        try:
            scalar_type = pa.from_numpy_dtype(scalar_dtype)
        except (TypeError, ValueError):
            scalar_type = None
        if scalar_type is not None:
            if pa.types.is_boolean(scalar_type):
                return _TypeInfo("bool", arrow_type=scalar_type)
            if pa.types.is_integer(scalar_type):
                integer = _as_integer(value)
                return _TypeInfo(
                    "int",
                    arrow_type=scalar_type,
                    minimum=integer,
                    maximum=integer,
                )
            if pa.types.is_floating(scalar_type):
                return _TypeInfo("float", arrow_type=scalar_type)

    python_integer = _as_integer_or_none(value)
    if python_integer is not None and not isinstance(value, bool):
        return _TypeInfo(
            "int",
            minimum=python_integer,
            maximum=python_integer,
            python_int=True,
        )
    if isinstance(value, float):
        return _TypeInfo("float", arrow_type=pa.float64())

    array_shape = _array_shape(value)
    if array_shape is not None:
        return _type_for_array(value, array_shape, pa)
    if isinstance(value, (list, tuple)):
        item: _TypeInfo | None = None
        for element in value:
            item = _observe(
                item or _TypeInfo("null"),
                element,
                pa,
                root=False,
            )
        return _TypeInfo("list", item=item or _TypeInfo("null"))
    if isinstance(value, Mapping):
        fields: dict[str, _TypeInfo] = {}
        for name, child in value.items():
            if not isinstance(name, str):
                raise TypeError("nested Parquet field names must be strings")
            fields[name] = _observe(_TypeInfo("null"), child, pa, root=False)
        return _TypeInfo("struct", fields=fields)

    try:
        scalar_type = pa.array([value]).type
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError(
            f"unsupported Parquet value type: {type(value).__name__}"
        ) from exc
    return _info_from_arrow_type(scalar_type, pa)


def _type_for_array(value: Any, shape: tuple[int, ...], pa: Any) -> _TypeInfo:
    _check_array_declaration(value, shape)
    dtype = getattr(value, "dtype", None)
    item: _TypeInfo | None = None
    if dtype is not None:
        try:
            item_type = pa.from_numpy_dtype(dtype)
        except (TypeError, ValueError):
            item_type = None
        if item_type is not None and item_type != pa.null():
            item = _info_from_arrow_type(item_type, pa)
    if item is None:
        if len(shape) == 1:
            for element in value:
                item = _observe(item or _TypeInfo("null"), element, pa, root=False)
        else:
            # Let Arrow reject unsupported dimensions during writing; account
            # for their declared shape now.
            item = _TypeInfo("null")
    for _ in range(max(0, len(shape) - 1)):
        item = _TypeInfo("list", item=item)
    return _TypeInfo("list", item=item or _TypeInfo("null"))


def _info_from_arrow_type(arrow_type: Any, pa: Any) -> _TypeInfo:
    if pa.types.is_null(arrow_type):
        return _TypeInfo("null")
    if pa.types.is_boolean(arrow_type):
        return _TypeInfo("bool", arrow_type=arrow_type)
    if pa.types.is_integer(arrow_type):
        return _TypeInfo("int", arrow_type=arrow_type)
    if pa.types.is_floating(arrow_type):
        return _TypeInfo("float", arrow_type=arrow_type)
    if pa.types.is_string(arrow_type):
        return _TypeInfo("string", arrow_type=arrow_type)
    if pa.types.is_binary(arrow_type):
        return _TypeInfo("binary", arrow_type=arrow_type)
    if pa.types.is_list(arrow_type) or pa.types.is_large_list(arrow_type):
        return _TypeInfo(
            "list",
            arrow_type=arrow_type,
            item=_info_from_arrow_type(arrow_type.value_type, pa),
        )
    if pa.types.is_struct(arrow_type):
        return _TypeInfo(
            "struct",
            arrow_type=arrow_type,
            fields={
                field.name: _info_from_arrow_type(field.type, pa)
                for field in arrow_type
            },
        )
    return _TypeInfo("arrow", arrow_type=arrow_type)


def _merge(
    current: _TypeInfo,
    candidate: _TypeInfo,
    pa: Any,
    *,
    root: bool,
) -> _TypeInfo:
    nullable = current.nullable or candidate.nullable
    if current.kind == "null":
        candidate.nullable = nullable
        return candidate
    if candidate.kind == "null":
        current.nullable = nullable
        return current

    if current.kind == candidate.kind == "int":
        return _merge_int(current, candidate, nullable, pa, root=root)
    if {current.kind, candidate.kind} == {"int", "float"}:
        return _merge_float(current, candidate, nullable, pa)
    if current.kind == candidate.kind == "float":
        return _merge_float(current, candidate, nullable, pa)
    if {current.kind, candidate.kind} == {"string", "binary"}:
        return _TypeInfo("binary", arrow_type=pa.binary(), nullable=nullable)
    if current.kind == candidate.kind == "struct":
        fields = dict(current.fields)
        for name, child in candidate.fields.items():
            if name in fields:
                fields[name] = _merge(fields[name], child, pa, root=False)
            else:
                fields[name] = child
        return _TypeInfo("struct", fields=fields, nullable=nullable)
    if current.kind == candidate.kind == "list":
        item = current.item or _TypeInfo("null")
        other = candidate.item or _TypeInfo("null")
        return _TypeInfo(
            "list",
            item=_merge(item, other, pa, root=False),
            nullable=nullable,
        )
    if current.kind == candidate.kind and current.kind in {
        "bool",
        "string",
        "binary",
    }:
        return _TypeInfo(
            current.kind,
            arrow_type=_merge_arrow_types(current.arrow_type, candidate.arrow_type, pa),
            nullable=nullable,
        )
    if current.kind == candidate.kind == "arrow":
        return _TypeInfo(
            "arrow",
            arrow_type=_merge_arrow_types(current.arrow_type, candidate.arrow_type, pa),
            nullable=nullable,
        )
    raise TypeError(f"incompatible Parquet values: {current.kind} and {candidate.kind}")


def _merge_int(
    current: _TypeInfo,
    candidate: _TypeInfo,
    nullable: bool,
    pa: Any,
    *,
    root: bool,
) -> _TypeInfo:
    minimums = [
        value for value in (current.minimum, candidate.minimum) if value is not None
    ]
    maximums = [
        value for value in (current.maximum, candidate.maximum) if value is not None
    ]
    python_int = current.python_int or candidate.python_int
    if python_int and (
        current.arrow_type is not None or candidate.arrow_type is not None
    ):
        arrow_type = pa.int64()
        if (
            minimums
            and min(minimums) >= 0
            and any(
                value is not None and pa.types.is_unsigned_integer(value)
                for value in (current.arrow_type, candidate.arrow_type)
            )
        ):
            arrow_type = pa.uint64()
    else:
        arrow_type = _merge_arrow_types(current.arrow_type, candidate.arrow_type, pa)
    return _TypeInfo(
        "int",
        arrow_type=arrow_type,
        nullable=nullable,
        minimum=min(minimums) if minimums else None,
        maximum=max(maximums) if maximums else None,
        python_int=python_int,
    )


def _merge_float(
    current: _TypeInfo,
    candidate: _TypeInfo,
    nullable: bool,
    pa: Any,
) -> _TypeInfo:
    infos = (current, candidate)
    arrow_types = [info.arrow_type for info in infos if info.arrow_type is not None]
    arrow_type = pa.float64()
    plain_integer = any(
        info.kind == "int" and info.arrow_type is None for info in infos
    )
    if (
        not plain_integer
        and arrow_types
        and all(pa.types.is_floating(value) for value in arrow_types)
    ):
        arrow_type = _merge_arrow_types(arrow_types[0], arrow_types[0], pa)
        for other in arrow_types[1:]:
            arrow_type = _merge_arrow_types(arrow_type, other, pa)
    return _TypeInfo("float", arrow_type=arrow_type, nullable=nullable)


def _merge_arrow_types(left: Any, right: Any, pa: Any) -> Any:
    if left is None:
        return right
    if right is None or left.equals(right):
        return left
    if pa.types.is_integer(left) and pa.types.is_integer(right):
        if pa.types.is_signed_integer(left) and pa.types.is_signed_integer(right):
            return _integer_type(
                pa, signed=True, bits=max(left.bit_width, right.bit_width)
            )
        if pa.types.is_unsigned_integer(left) and pa.types.is_unsigned_integer(right):
            return _integer_type(
                pa, signed=False, bits=max(left.bit_width, right.bit_width)
            )
        return pa.int64()
    if pa.types.is_floating(left) and pa.types.is_floating(right):
        return _floating_type(pa, bits=max(left.bit_width, right.bit_width))
    try:
        return (
            pa.unify_schemas(
                [pa.schema([("value", left)]), pa.schema([("value", right)])],
                promote_options="permissive",
            )
            .field("value")
            .type
        )
    except (TypeError, ValueError, pa.ArrowException) as exc:
        raise TypeError(
            f"incompatible Parquet Arrow types: {left} and {right}"
        ) from exc


def _integer_type(pa: Any, *, signed: bool, bits: int) -> Any:
    bits = min(64, max(8, int(bits)))
    bits = 8 if bits <= 8 else 16 if bits <= 16 else 32 if bits <= 32 else 64
    return getattr(pa, f"{'int' if signed else 'uint'}{bits}")()


def _floating_type(pa: Any, *, bits: int) -> Any:
    if bits <= 16:
        return pa.float16()
    if bits <= 32:
        return pa.float32()
    return pa.float64()


def _arrow_type(info: _TypeInfo, pa: Any, *, root: bool) -> Any:
    if info.kind == "null":
        return pa.null()
    if info.kind == "bool":
        return info.arrow_type or pa.bool_()
    if info.kind == "float":
        return info.arrow_type or pa.float64()
    if info.kind == "string":
        return info.arrow_type or pa.string()
    if info.kind == "binary":
        return info.arrow_type or pa.binary()
    if info.kind == "int":
        minimum = info.minimum if info.minimum is not None else 0
        maximum = info.maximum if info.maximum is not None else 0
        if minimum < -(1 << 63) or maximum > (1 << 64) - 1:
            raise OverflowError("Python int too large to convert to C long")
        # Preserve legacy floats for small nullable integers, but use integer
        # types outside float64's exact range to avoid losing precision.
        if root and info.nullable and minimum >= -(1 << 53) and maximum <= 1 << 53:
            return pa.float64()
        if info.arrow_type is not None:
            if pa.types.is_signed_integer(info.arrow_type) and (
                minimum < -(1 << 63) or maximum > (1 << 63) - 1
            ):
                raise OverflowError("Python int too large to convert to C long")
            if pa.types.is_unsigned_integer(info.arrow_type) and (
                minimum < 0 or maximum > (1 << 64) - 1
            ):
                raise OverflowError("Python int too large to convert to C long")
            return info.arrow_type
        if minimum < 0:
            if maximum > (1 << 63) - 1:
                raise OverflowError("Python int too large to convert to C long")
            return pa.int64()
        return pa.int64() if maximum <= (1 << 63) - 1 else pa.uint64()
    if info.kind == "list":
        item = _arrow_type(info.item or _TypeInfo("null"), pa, root=False)
        return pa.list_(item)
    if info.kind == "struct":
        return pa.struct(
            [
                pa.field(name, _arrow_type(child, pa, root=False))
                for name, child in info.fields.items()
            ]
        )
    if info.kind == "arrow":
        return info.arrow_type
    raise TypeError(f"unknown Parquet schema kind: {info.kind}")


def _coerce_value(
    value: Any,
    info: _TypeInfo,
    expected: Any,
    pa: Any,
) -> Any:
    if value is None:
        return None
    if pa.types.is_floating(expected) and _as_integer_or_none(value) is not None:
        return float(value)
    if info.kind == "struct" and isinstance(value, Mapping):
        unknown = [name for name in value if name not in info.fields]
        if unknown:
            raise ValueError(
                "row_factory changed nested schema; unexpected fields: "
                + ", ".join(str(name) for name in unknown)
            )
        converted = {}
        for name, child in value.items():
            child_type = expected.field(name).type
            converted[name] = _coerce_value(child, info.fields[name], child_type, pa)
        return converted
    if info.kind == "list" and isinstance(value, (list, tuple)):
        item_info = info.item or _TypeInfo("null")
        item_type = expected.value_type
        return [_coerce_value(element, item_info, item_type, pa) for element in value]
    return value


def _as_integer(value: Any) -> int:
    integer = _as_integer_or_none(value)
    if integer is None:
        raise TypeError(f"{type(value).__name__} is not an integer")
    return integer


def _as_integer_or_none(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        integer = operator.index(value)
    except TypeError:
        return None
    return int(integer)


def _array_shape(value: Any) -> tuple[int, ...] | None:
    shape = getattr(value, "shape", None)
    if shape is None:
        return None
    try:
        result = tuple(operator.index(dimension) for dimension in shape)
    except (TypeError, ValueError):
        return None
    if any(dimension < 0 for dimension in result):
        raise ValueError("array shape dimensions must be non-negative")
    if not result:
        return None
    return result


def _check_array_declaration(value: Any, shape: tuple[int, ...]) -> None:
    elements = 1
    for dimension in shape:
        if dimension > MAX_ARRAY_ELEMENTS:
            raise ResourceLimitError(
                "Parquet array dimension",
                limit=MAX_ARRAY_ELEMENTS,
                observed=dimension,
            )
        elements *= dimension
        if elements > MAX_ARRAY_ELEMENTS:
            raise ResourceLimitError(
                "Parquet array elements",
                limit=MAX_ARRAY_ELEMENTS,
                observed=elements,
            )
    nbytes = getattr(value, "nbytes", None)
    if nbytes is None:
        dtype = getattr(value, "dtype", None)
        itemsize = getattr(dtype, "itemsize", None)
        if itemsize is not None:
            nbytes = elements * int(itemsize)
    if nbytes is not None:
        try:
            observed = operator.index(nbytes)
        except TypeError as exc:
            raise TypeError("array nbytes must be an integer") from exc
        if observed < 0:
            raise ValueError("array nbytes must be non-negative")
        if observed > MAX_ARRAY_BYTES:
            raise ResourceLimitError(
                "Parquet array bytes",
                limit=MAX_ARRAY_BYTES,
                observed=observed,
            )


def _estimate_value(value: Any, active: set[int] | None = None) -> int:
    """Estimate source and conversion memory without materialising providers."""
    if active is None:
        active = set()
    if value is None:
        return 0
    if isinstance(value, str):
        length = len(value)
        estimate = sys.getsizeof(value) + (length * 4)
        _check_array_bytes(estimate)
        return estimate
    if isinstance(value, (bytes, bytearray, memoryview)):
        try:
            length = len(value)
        except TypeError:
            length = 0
        estimate = sys.getsizeof(value) + length
        _check_array_bytes(estimate)
        return estimate
    if isinstance(value, (bool, int, float)):
        return sys.getsizeof(value)

    identity = id(value)
    if identity in active:
        raise ValueError("cyclic Parquet row value")
    active.add(identity)
    try:
        shape = _array_shape(value)
        if shape is not None:
            _check_array_declaration(value, shape)
            nbytes = getattr(value, "nbytes", None)
            estimate = int(nbytes) if nbytes is not None else sys.getsizeof(value)
            dtype = getattr(value, "dtype", None)
            if getattr(dtype, "kind", None) == "O":
                estimate += sum(_estimate_value(element, active) for element in value)
            return estimate + sys.getsizeof(value)
        if isinstance(value, Mapping):
            return sys.getsizeof(value) + sum(
                sys.getsizeof(key) + _estimate_value(child, active)
                for key, child in value.items()
            )
        if isinstance(value, (list, tuple)):
            if len(value) > MAX_ARRAY_ELEMENTS:
                raise ResourceLimitError(
                    "Parquet array elements",
                    limit=MAX_ARRAY_ELEMENTS,
                    observed=len(value),
                )
            estimate = sys.getsizeof(value) + sum(
                _estimate_value(element, active) for element in value
            )
            if estimate > MAX_ARRAY_BYTES:
                raise ResourceLimitError(
                    "Parquet array bytes",
                    limit=MAX_ARRAY_BYTES,
                    observed=estimate,
                )
            return estimate
        return sys.getsizeof(value)
    finally:
        active.remove(identity)


def _check_array_bytes(observed: int) -> None:
    if observed > MAX_ARRAY_BYTES:
        raise ResourceLimitError(
            "Parquet array bytes",
            limit=MAX_ARRAY_BYTES,
            observed=observed,
        )
