"""Immutable, Isaac-free declarations for direct command-to-action mappings."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Annotated, Literal, TypeAlias, Union

import torch
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class _FrozenModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


def _tuple_tree(value):
    """Accept JSON arrays at the model_validate boundary, then freeze them."""
    if isinstance(value, (list, tuple)):
        return tuple(_tuple_tree(item) for item in value)
    return value


class Identity(_FrozenModel):
    kind: Literal["identity"] = "identity"


class SliceFirst(_FrozenModel):
    kind: Literal["slice_first"] = "slice_first"
    count: int

    @model_validator(mode="after")
    def _check_count(self) -> SliceFirst:
        if not 1 <= self.count <= 6:
            raise ValueError("count must be in [1, 6]")
        return self


class Select(_FrozenModel):
    kind: Literal["select"] = "select"
    indices: tuple[int, ...]

    @field_validator("indices", mode="before")
    @classmethod
    def _freeze_indices(cls, value):
        return _tuple_tree(value)

    @model_validator(mode="after")
    def _check_indices(self) -> Select:
        if not self.indices or any(not 0 <= index < 6 for index in self.indices):
            raise ValueError("indices must be a non-empty selection from [0, 6)")
        return self


class OptionalThird(_FrozenModel):
    kind: Literal["optional_third"] = "optional_third"
    index: int = 2

    @model_validator(mode="after")
    def _check_index(self) -> OptionalThird:
        if not 0 <= self.index < 6:
            raise ValueError("index must be in [0, 6)")
        return self


class EventSign(_FrozenModel):
    kind: Literal["event_sign"] = "event_sign"


class AppendOnes(_FrozenModel):
    kind: Literal["append_ones"] = "append_ones"
    count: int

    @model_validator(mode="after")
    def _check_count(self) -> AppendOnes:
        if self.count <= 0:
            raise ValueError("count must be positive")
        return self


class Thrust(_FrozenModel):
    kind: Literal["thrust"] = "thrust"
    directions: tuple[tuple[float, float, float], ...]
    gimbal: tuple[bool, ...]
    # Normalized torque per unit throttle; empty maps forces only.
    moments: tuple[tuple[float, float, float], ...] = ()

    @field_validator("directions", "gimbal", "moments", mode="before")
    @classmethod
    def _freeze_configuration(cls, value):
        return _tuple_tree(value)

    @model_validator(mode="after")
    def _check_configuration(self) -> Thrust:
        if len(self.directions) != len(self.gimbal):
            raise ValueError("gimbal flags must match the thruster directions")
        if any(
            not math.isfinite(value)
            for direction in self.directions
            for value in direction
        ):
            raise ValueError("thruster directions must be finite")
        if self.moments and len(self.moments) != len(self.directions):
            raise ValueError("thruster moments must match the thruster directions")
        if any(not math.isfinite(value) for moment in self.moments for value in moment):
            raise ValueError("thruster moments must be finite")
        return self


class Matrix(_FrozenModel):
    kind: Literal["matrix"] = "matrix"
    values: tuple[tuple[float, ...], ...]

    @field_validator("values", mode="before")
    @classmethod
    def _freeze_values(cls, value):
        return _tuple_tree(value)

    @model_validator(mode="after")
    def _check_values(self) -> Matrix:
        if not self.values or any(len(row) != 6 for row in self.values):
            raise ValueError("matrix must have at least one row of six values")
        if any(not math.isfinite(value) for row in self.values for value in row):
            raise ValueError("matrix values must be finite")
        return self


class LevionTail2(_FrozenModel):
    kind: Literal["levion_tail2"] = "levion_tail2"


_SimpleOperation: TypeAlias = Annotated[
    Union[
        Identity,
        SliceFirst,
        Select,
        OptionalThird,
        EventSign,
        AppendOnes,
        Thrust,
        Matrix,
        LevionTail2,
    ],
    Field(discriminator="kind"),
]


class Concat(_FrozenModel):
    kind: Literal["concat"] = "concat"
    parts: tuple[CommandOperation, ...]

    @field_validator("parts", mode="before")
    @classmethod
    def _freeze_parts(cls, value):
        return _tuple_tree(value)

    @model_validator(mode="after")
    def _check_parts(self) -> Concat:
        if not self.parts:
            raise ValueError("concat requires at least one operation")
        return self


CommandOperation: TypeAlias = Annotated[
    Union[_SimpleOperation, Concat], Field(discriminator="kind")
]
Concat.model_rebuild(_types_namespace={"CommandOperation": CommandOperation})


class CommandDestination(_FrozenModel):
    name: str
    start: int
    stop: int

    @model_validator(mode="after")
    def _check_slice(self) -> CommandDestination:
        if not self.name.strip():
            raise ValueError("destination name must not be empty")
        if self.start < 0 or self.stop < self.start:
            raise ValueError("destination slice must satisfy 0 <= start <= stop")
        return self


class CommandMapping(_FrozenModel):
    version: int = 1
    operation: CommandOperation
    destinations: tuple[CommandDestination, ...]
    task_family: Literal["manipulation", "other"] = "other"

    @field_validator("version", mode="before")
    @classmethod
    def _check_version_type_and_value(cls, value):
        if type(value) is not int or value != 1:
            raise ValueError("version must be the integer 1")
        return value

    @field_validator("destinations", mode="before")
    @classmethod
    def _freeze_destinations(cls, value):
        return _tuple_tree(value)

    @model_validator(mode="after")
    def _check_layout(self) -> CommandMapping:
        width = operation_width(self.operation)
        if not self.destinations and width != 0:
            raise ValueError("command mapping requires at least one destination")
        names = tuple(destination.name for destination in self.destinations)
        if len(names) != len(set(names)):
            raise ValueError("destination names must be unique")
        position = 0
        for destination in self.destinations:
            if destination.start != position:
                raise ValueError("destinations must provide complete ordered coverage")
            position = destination.stop
        if position != width:
            raise ValueError(
                f"destination width {position} does not match operation width {width}"
            )
        return self


def revalidate_command_mapping(mapping: CommandMapping) -> CommandMapping:
    """Rebuild a typed value, checking even copies made with Pydantic's unsafe update API."""
    if not isinstance(mapping, CommandMapping):
        raise TypeError("mapping must be a CommandMapping")
    return CommandMapping.model_validate(mapping.model_dump(mode="python"))


def operation_width(operation: CommandOperation) -> int:
    """Return the operation's declared conventional action width."""
    match operation:
        case Identity():
            return 6
        case SliceFirst(count=count):
            return count
        case Select(indices=indices):
            return len(indices)
        case OptionalThird() | EventSign():
            return 1
        case AppendOnes(count=count):
            return count
        case Thrust(directions=directions, gimbal=gimbal):
            active_gimbals = sum(
                has_gimbal and any(component != 0.0 for component in direction)
                for direction, has_gimbal in zip(directions, gimbal, strict=True)
            )
            return len(directions) + 2 * active_gimbals
        case Matrix(values=values):
            return len(values)
        case LevionTail2():
            return 4
        case Concat(parts=parts):
            return sum(operation_width(part) for part in parts)
        case _:
            raise TypeError(
                f"unsupported command operation: {type(operation).__name__}"
            )


def _apply(
    operation: CommandOperation, twist: torch.Tensor, event: bool
) -> torch.Tensor:
    match operation:
        case Identity():
            return twist
        case SliceFirst(count=count):
            return twist[:count]
        case Select(indices=indices):
            return torch.stack(tuple(twist[index] for index in indices))
        case OptionalThird(index=index):
            if twist.shape[-1] > index:
                return twist[..., index : index + 1]
            return torch.zeros(
                twist.shape[:-1] + (1,), dtype=twist.dtype, device=twist.device
            )
        case EventSign():
            return torch.tensor(
                (-1.0 if event else 1.0,), dtype=twist.dtype, device=twist.device
            )
        case AppendOnes(count=count):
            return torch.ones(count, dtype=twist.dtype, device=twist.device)
        case Thrust(directions=directions, gimbal=gimbal, moments=moments):
            desired_force = twist[:3]
            desired_torque = twist[3:6] if moments and twist.shape[-1] >= 6 else None
            thrust_actions: list[torch.Tensor] = []
            gimbal_actions: list[torch.Tensor] = []
            for index, (direction_values, has_gimbal) in enumerate(
                zip(directions, gimbal, strict=True)
            ):
                direction = torch.as_tensor(
                    direction_values, device=twist.device, dtype=twist.dtype
                )
                direction_norm = torch.linalg.vector_norm(direction)
                if float(direction_norm) <= 0.0:
                    thrust_actions.append(
                        torch.zeros(1, device=twist.device, dtype=twist.dtype)
                    )
                    continue
                direction = direction / direction_norm
                throttle = torch.dot(-direction, desired_force)
                if desired_torque is not None:
                    moment = torch.as_tensor(
                        moments[index], device=twist.device, dtype=twist.dtype
                    )
                    throttle = throttle + torch.dot(moment, desired_torque)
                thrust_actions.append(torch.clamp(throttle, 0.0, 1.0).unsqueeze(0))
                if has_gimbal:
                    gimbal_actions.append(
                        torch.zeros(2, device=twist.device, dtype=twist.dtype)
                    )
            if not thrust_actions:
                return torch.empty(0, device=twist.device, dtype=twist.dtype)
            return torch.cat([*thrust_actions, *gimbal_actions])
        case Matrix(values=values):
            matrix = torch.tensor(values, device=twist.device, dtype=twist.dtype)
            return twist @ matrix.T
        case LevionTail2():
            action = torch.zeros(4, dtype=twist.dtype, device=twist.device)
            action[:2] = twist[-2:]
            return action
        case Concat(parts=parts):
            return torch.cat([_apply(part, twist, event) for part in parts], dim=0)
        case _:
            raise TypeError(
                f"unsupported command operation: {type(operation).__name__}"
            )


def apply_command_mapping(
    mapping: CommandMapping, twist: torch.Tensor, event: bool
) -> torch.Tensor:
    """Evaluate a declaration using the source mapper's torch semantics."""
    mapping = revalidate_command_mapping(mapping)
    if not isinstance(twist, torch.Tensor):
        raise TypeError("twist must be a torch.Tensor")
    return _apply(mapping.operation, twist, event)


def validate_command_mapping(
    mapping: CommandMapping, ordering: Mapping[str, slice], action_dim: int
) -> None:
    """Require exact destination names, order and slices for a live action layout."""
    mapping = revalidate_command_mapping(mapping)
    if type(action_dim) is not int or action_dim < 0:
        raise ValueError("action_dim must be a non-negative integer")
    if not isinstance(ordering, Mapping):
        raise TypeError("ordering must be a mapping of names to slices")
    declared = tuple(
        (item.name, item.start, item.stop) for item in mapping.destinations
    )
    live: list[tuple[str, int, int]] = []
    for name, term_slice in ordering.items():
        if not isinstance(name, str) or not isinstance(term_slice, slice):
            raise ValueError("ordering entries must map names to slices")
        if (
            type(term_slice.start) is not int
            or type(term_slice.stop) is not int
            or term_slice.step not in (None, 1)
        ):
            raise ValueError(
                f"ordering slice for {name!r} must have integer bounds and unit step"
            )
        live.append((name, term_slice.start, term_slice.stop))
    if tuple(live) != declared:
        raise ValueError("command destinations do not match action ordering exactly")
    if not live:
        if action_dim != 0:
            raise ValueError(
                "action ordering must cover the full declared action dimension"
            )
        return
    if live[0][1] != 0 or live[-1][2] != action_dim:
        raise ValueError(
            "action ordering must cover the full declared action dimension"
        )


def rename_destinations(
    mapping: CommandMapping, names: Mapping[str, str]
) -> CommandMapping:
    """Return a copy with every destination renamed by an exact name map."""
    current = {destination.name for destination in mapping.destinations}
    if set(names) != current:
        raise ValueError("rename map must name every destination exactly once")
    if any(not isinstance(value, str) or not value.strip() for value in names.values()):
        raise ValueError("renamed destinations must be non-empty strings")
    renamed = tuple(
        destination.model_copy(update={"name": names[destination.name]})
        for destination in mapping.destinations
    )
    return CommandMapping(
        version=mapping.version,
        operation=mapping.operation,
        destinations=renamed,
        task_family=mapping.task_family,
    )


def concat_command_mappings(
    *mappings: CommandMapping,
    task_family: Literal["manipulation", "other"] = "other",
) -> CommandMapping:
    """Compose mappings in order, matching ``torch.cat(..., dim=0)`` semantics."""
    if not mappings:
        raise ValueError("at least one command mapping is required")
    destinations: list[CommandDestination] = []
    offset = 0
    for mapping in mappings:
        if not isinstance(mapping, CommandMapping):
            raise TypeError("all mappings must be CommandMapping instances")
        for destination in mapping.destinations:
            destinations.append(
                CommandDestination(
                    name=destination.name,
                    start=destination.start + offset,
                    stop=destination.stop + offset,
                )
            )
        offset += operation_width(mapping.operation)
    return CommandMapping(
        operation=Concat(parts=tuple(mapping.operation for mapping in mappings)),
        destinations=tuple(destinations),
        task_family=task_family,
    )
