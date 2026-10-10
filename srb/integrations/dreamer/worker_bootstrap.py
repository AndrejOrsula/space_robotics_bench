"""Strict process bootstrap for one scalar Dreamer worker.

This module owns launch-spec validation, passed-FD reconstruction, and the
numeric protocol serve loop.  The SRB/Isaac projection boundary is owned by
the lazily imported child runtime.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from collections.abc import Callable, Mapping, Sequence
from contextlib import suppress
from dataclasses import dataclass, field
from multiprocessing.connection import Connection
from typing import Any, Protocol, runtime_checkable
from urllib.parse import parse_qsl, urlsplit

import numpy as np

from .process_projection import (
    DreamerProcessProjection,
    DreamerProcessProjectionError,
)
from .process_protocol import (
    INFO_ALLOWLIST,
    MAX_ERROR_TEXT_BYTES,
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    ZERO_SCHEMA_HASHES,
    ClosedResponse,
    CloseRequest,
    ErrorResponse,
    HelloRequest,
    HelloResponse,
    ResetRequest,
    StepRequest,
    TransitionResponse,
    recv_message,
    schema_hashes_for,
    send_message,
    validate_landscaping_contract,
    validate_schema_hashes,
)

MAX_SPEC_JSON_BYTES = 32 * 1024
MAX_TASK_ID_BYTES = 160
MAX_OVERRIDE_ITEMS = 96
MAX_KIT_ARG_ITEMS = 96
MAX_ITEM_BYTES = 1024
MAX_EXPERIENCE_BYTES = 1024
MAX_WORKER_INDEX = 65_535
MAX_WORKER_SEED = 2**32 - 1
MIN_CONTROL_FD = 3
MAX_CONTROL_FD = 2**31 - 1
_OBSERVATION_MODES = frozenset({"state", "visual"})
_RENDERING_MODES = frozenset({"performance", "balanced", "quality", "xr"})

_SPEC_FIELDS = frozenset(
    {
        "base_seed",
        "device",
        "enable_cameras",
        "experience",
        "headless",
        "hydra_overrides",
        "kit_args",
        "num_envs",
        "observation_mode",
        "protocol_name",
        "protocol_version",
        "rendering",
        "rendering_mode",
        "task_id",
        "video",
        "visualizer",
        "worker_index",
        "worker_seed",
        "xr",
    }
)
_SENSITIVE_FRAGMENTS = (
    "api_key",
    "apikey",
    "authorization",
    "bearer",
    "credential",
    "password",
    "passwd",
    "private_key",
    "secret",
    "signedurl",
    "token",
)
_SIGNED_QUERY_KEYS = frozenset(
    {
        "access_token",
        "api-key",
        "apikey",
        "authorization",
        "awsaccesskeyid",
        "client-secret",
        "credential",
        "expires",
        "googleaccessid",
        "key-pair-id",
        "policy",
        "sas-token",
        "sharedaccesssignature",
        "signature",
        "sig",
        "se",
        "skoid",
        "sks",
        "skt",
        "sktid",
        "skv",
        "sp",
        "spr",
        "srt",
        "ss",
        "st",
        "sv",
        "token",
        "x-amz-algorithm",
        "x-amz-credential",
        "x-amz-date",
        "x-amz-expires",
        "x-amz-security-token",
        "x-amz-signature",
        "x-amz-signedheaders",
        "x-goog-algorithm",
        "x-goog-credential",
        "x-goog-date",
        "x-goog-expires",
        "x-goog-signature",
        "x-goog-signedheaders",
    }
)
_NORMALIZED_SIGNED_QUERY_KEYS = frozenset(
    key.casefold().replace("_", "-") for key in _SIGNED_QUERY_KEYS
)
_SHELL_TOKENS = ("&&", "||", "$(`", "$(", "${", "`", ";", "|", ">", "<", "\\")
_SHELL_PREFIXES = (
    "bash ",
    "cmd ",
    "curl ",
    "env ",
    "exec ",
    "powershell ",
    "python ",
    "rm ",
    "sh ",
    "sudo ",
    "wget ",
)
_IMPORT_MARKERS = (
    "_partial_",
    "_target_",
    "callable=",
    "factory=",
    "import ",
    "module=",
    "py:",
    "python:",
)
_IMPORT_PATH = re.compile(r"^[A-Za-z_]\w*(?:\.[A-Za-z_]\w*){2,}(?::[A-Za-z_]\w*)?$")
_TASK_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_./-]*$")
_DEVICE = re.compile(r"^(?:cpu|cuda(?::[0-9]{1,4})?)$")
_OVERRIDE_KEY = re.compile(r"^[A-Za-z0-9_./@-]+$")


class DreamerWorkerSpecError(ValueError):
    """The launch specification is not canonical, bounded, or safe."""


class DreamerWorkerBootstrapError(RuntimeError):
    """The scalar worker could not establish or serve its protocol."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise DreamerWorkerSpecError(f"duplicate launch-spec field {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise DreamerWorkerSpecError(f"non-finite launch-spec number {value!r}")


def _canonical_json(value: Mapping[str, Any]) -> str:
    try:
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise DreamerWorkerSpecError("launch spec is not JSON data") from exc


def _strict_int(value: Any, *, name: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise DreamerWorkerSpecError(
            f"{name} must be an integer in [{minimum}, {maximum}]"
        )
    return value


def _strict_bool(value: Any, *, name: str) -> bool:
    if type(value) is not bool:
        raise DreamerWorkerSpecError(f"{name} must be boolean")
    return value


def _validate_url_security(value: str, *, name: str) -> None:
    candidate = value.strip()
    starts_quote = candidate[:1] in {"'", '"'}
    ends_quote = candidate[-1:] in {"'", '"'}
    if starts_quote or ends_quote:
        if len(candidate) < 2 or candidate[0] != candidate[-1]:
            raise DreamerWorkerSpecError(f"{name} has mismatched URL-value quotes")
        candidate = candidate[1:-1].strip()
    try:
        parsed = urlsplit(candidate)
    except ValueError as exc:
        raise DreamerWorkerSpecError(f"{name} contains a malformed URI") from exc
    if parsed.netloc and (parsed.username is not None or parsed.password is not None):
        raise DreamerWorkerSpecError(f"{name} contains URI userinfo")
    query_keys = {
        key.casefold().replace("_", "-")
        for key, _value in parse_qsl(parsed.query, keep_blank_values=True)
    }
    if query_keys & _NORMALIZED_SIGNED_QUERY_KEYS:
        raise DreamerWorkerSpecError(
            f"{name} contains a signed or credential-bearing URL"
        )


def _safe_text(
    value: Any,
    *,
    name: str,
    max_bytes: int,
    allow_empty: bool = False,
    reject_import_path: bool = True,
    validate_url_security: bool = True,
) -> str:
    if type(value) is not str:
        raise DreamerWorkerSpecError(f"{name} must be a string")
    encoded = value.encode("utf-8")
    if len(encoded) > max_bytes or (not value and not allow_empty):
        raise DreamerWorkerSpecError(f"{name} is empty or exceeds its size limit")
    if value != value.strip() or any(char in value for char in ("\x00", "\n", "\r")):
        raise DreamerWorkerSpecError(f"{name} contains forbidden control text")
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise DreamerWorkerSpecError(f"{name} contains a control character")

    lowered = value.lower()
    normalized = lowered.replace("-", "_")
    if any(fragment in normalized for fragment in _SENSITIVE_FRAGMENTS):
        raise DreamerWorkerSpecError(f"{name} contains credential-like text")
    if any(token in value for token in _SHELL_TOKENS) or lowered.startswith(
        _SHELL_PREFIXES
    ):
        raise DreamerWorkerSpecError(f"{name} contains shell text")
    if any(marker in lowered for marker in _IMPORT_MARKERS):
        raise DreamerWorkerSpecError(f"{name} contains an import or callable marker")
    if reject_import_path and _IMPORT_PATH.fullmatch(value):
        raise DreamerWorkerSpecError(f"{name} resembles an arbitrary import path")

    if validate_url_security:
        _validate_url_security(value, name=name)
    return value


def _validate_override(value: Any, *, index: int) -> str:
    override = _safe_text(
        value,
        name=f"hydra_overrides[{index}]",
        max_bytes=MAX_ITEM_BYTES,
        validate_url_security=False,
    )
    if "=" not in override:
        raise DreamerWorkerSpecError("Hydra overrides must use key=value form")
    key_text, assigned_value = override.split("=", 1)
    key = key_text.lstrip("+~")
    if not key or not _OVERRIDE_KEY.fullmatch(key):
        raise DreamerWorkerSpecError("Hydra override key is invalid")
    if key == "env.seed" or key.endswith(".env.seed"):
        raise DreamerWorkerSpecError("env.seed is owned by the worker launch spec")
    if key == "scene.num_envs" or key.endswith(".scene.num_envs"):
        raise DreamerWorkerSpecError("scene.num_envs is fixed to one per worker")
    _validate_url_security(
        assigned_value, name=f"hydra_overrides[{index}] assigned value"
    )
    return override


def _strict_string_tuple(
    value: Any,
    *,
    name: str,
    maximum_items: int,
    item_validator: Callable[[Any, int], str],
) -> tuple[str, ...]:
    if type(value) is not tuple or len(value) > maximum_items:
        raise DreamerWorkerSpecError(f"{name} must be a bounded tuple")
    return tuple(item_validator(item, index) for index, item in enumerate(value))


@dataclass(frozen=True, slots=True)
class DreamerWorkerLaunchSpec:
    """Complete, inert launch data for exactly one scalar worker."""

    task_id: str
    hydra_overrides: tuple[str, ...] = ()
    device: str = "cuda:0"
    headless: bool = True
    enable_cameras: bool = False
    rendering: bool = False
    rendering_mode: str | None = None
    experience: str | None = None
    kit_args: tuple[str, ...] = ()
    xr: bool = False
    visualizer: bool = False
    video: bool = False
    observation_mode: str = "state"
    base_seed: int = 0
    worker_index: int = 0
    protocol_name: str = field(default=PROTOCOL_NAME, init=False)
    protocol_version: int = field(default=PROTOCOL_VERSION, init=False)
    worker_seed: int = field(init=False)
    num_envs: int = field(default=1, init=False)

    def __post_init__(self) -> None:
        task_id = _safe_text(
            self.task_id,
            name="task_id",
            max_bytes=MAX_TASK_ID_BYTES,
            reject_import_path=False,
        )
        if not _TASK_ID.fullmatch(task_id):
            raise DreamerWorkerSpecError("task_id contains an invalid character")
        object.__setattr__(self, "task_id", task_id)

        overrides = _strict_string_tuple(
            self.hydra_overrides,
            name="hydra_overrides",
            maximum_items=MAX_OVERRIDE_ITEMS,
            item_validator=lambda item, index: _validate_override(item, index=index),
        )
        object.__setattr__(self, "hydra_overrides", overrides)

        device = _safe_text(
            self.device, name="device", max_bytes=32, reject_import_path=False
        )
        if not _DEVICE.fullmatch(device):
            raise DreamerWorkerSpecError("device must be cpu, cuda, or cuda:<index>")
        object.__setattr__(self, "device", device)
        for name in (
            "headless",
            "enable_cameras",
            "rendering",
            "xr",
            "visualizer",
            "video",
        ):
            _strict_bool(getattr(self, name), name=name)

        if self.rendering_mode is not None:
            if (
                type(self.rendering_mode) is not str
                or self.rendering_mode not in _RENDERING_MODES
            ):
                raise DreamerWorkerSpecError(
                    "rendering_mode must be performance, balanced, quality, or xr"
                )
            if not self.rendering:
                raise DreamerWorkerSpecError(
                    "rendering_mode requires rendering to be enabled"
                )
            if self.rendering_mode == "xr" and not self.xr:
                raise DreamerWorkerSpecError("XR rendering mode requires xr=true")

        if type(self.observation_mode) is not str:
            raise DreamerWorkerSpecError("observation_mode must be a string")
        if self.observation_mode not in _OBSERVATION_MODES:
            raise DreamerWorkerSpecError(
                "observation_mode must be exactly 'state' or 'visual'"
            )

        if self.experience is not None:
            object.__setattr__(
                self,
                "experience",
                _safe_text(
                    self.experience,
                    name="experience",
                    max_bytes=MAX_EXPERIENCE_BYTES,
                ),
            )
        kit_args = _strict_string_tuple(
            self.kit_args,
            name="kit_args",
            maximum_items=MAX_KIT_ARG_ITEMS,
            item_validator=lambda item, index: _safe_text(
                item,
                name=f"kit_args[{index}]",
                max_bytes=MAX_ITEM_BYTES,
            ),
        )
        object.__setattr__(self, "kit_args", kit_args)

        base_seed = _strict_int(
            self.base_seed,
            name="base_seed",
            minimum=0,
            maximum=MAX_WORKER_SEED,
        )
        worker_index = _strict_int(
            self.worker_index,
            name="worker_index",
            minimum=0,
            maximum=MAX_WORKER_INDEX,
        )
        worker_seed = base_seed + worker_index
        if worker_seed > MAX_WORKER_SEED:
            raise DreamerWorkerSpecError("derived worker seed overflows the seed range")
        object.__setattr__(self, "worker_seed", worker_seed)

    def to_mapping(self) -> dict[str, Any]:
        return {
            "base_seed": self.base_seed,
            "device": self.device,
            "enable_cameras": self.enable_cameras,
            "experience": self.experience,
            "headless": self.headless,
            "hydra_overrides": list(self.hydra_overrides),
            "kit_args": list(self.kit_args),
            "num_envs": self.num_envs,
            "observation_mode": self.observation_mode,
            "protocol_name": self.protocol_name,
            "protocol_version": self.protocol_version,
            "rendering": self.rendering,
            "rendering_mode": self.rendering_mode,
            "task_id": self.task_id,
            "video": self.video,
            "visualizer": self.visualizer,
            "worker_index": self.worker_index,
            "worker_seed": self.worker_seed,
            "xr": self.xr,
        }

    def to_json(self) -> str:
        encoded = _canonical_json(self.to_mapping())
        if len(encoded.encode("utf-8")) > MAX_SPEC_JSON_BYTES:
            raise DreamerWorkerSpecError("launch spec exceeds its size limit")
        return encoded

    @classmethod
    def from_json(cls, encoded: str) -> "DreamerWorkerLaunchSpec":
        if (
            type(encoded) is not str
            or len(encoded.encode("utf-8")) > MAX_SPEC_JSON_BYTES
        ):
            raise DreamerWorkerSpecError("launch spec must be a bounded JSON string")
        try:
            value = json.loads(
                encoded,
                object_pairs_hook=_reject_duplicate_keys,
                parse_constant=_reject_json_constant,
            )
        except DreamerWorkerSpecError:
            raise
        except (json.JSONDecodeError, UnicodeError, TypeError) as exc:
            raise DreamerWorkerSpecError("launch spec is not valid JSON") from exc
        if not isinstance(value, dict):
            raise DreamerWorkerSpecError("launch spec root must be an object")
        if _canonical_json(value) != encoded:
            raise DreamerWorkerSpecError("launch spec JSON is not canonical")
        if set(value) != _SPEC_FIELDS:
            unknown = sorted(set(value) - _SPEC_FIELDS)
            missing = sorted(_SPEC_FIELDS - set(value))
            raise DreamerWorkerSpecError(
                f"launch spec schema mismatch; unknown={unknown}, missing={missing}"
            )
        if value["protocol_name"] != PROTOCOL_NAME:
            raise DreamerWorkerSpecError("unknown worker protocol name")
        if (
            type(value["protocol_version"]) is not int
            or value["protocol_version"] != PROTOCOL_VERSION
        ):
            raise DreamerWorkerSpecError(
                "protocol_version must be the exact supported integer"
            )
        if value["num_envs"] != 1 or type(value["num_envs"]) is not int:
            raise DreamerWorkerSpecError("num_envs must be the integer one")
        if type(value["hydra_overrides"]) is not list:
            raise DreamerWorkerSpecError("hydra_overrides must be a JSON list")
        if type(value["kit_args"]) is not list:
            raise DreamerWorkerSpecError("kit_args must be a JSON list")
        supplied_worker_seed = _strict_int(
            value["worker_seed"],
            name="worker_seed",
            minimum=0,
            maximum=MAX_WORKER_SEED,
        )
        spec = cls(
            task_id=value["task_id"],
            hydra_overrides=tuple(value["hydra_overrides"]),
            device=value["device"],
            headless=value["headless"],
            enable_cameras=value["enable_cameras"],
            rendering=value["rendering"],
            rendering_mode=value["rendering_mode"],
            experience=value["experience"],
            kit_args=tuple(value["kit_args"]),
            xr=value["xr"],
            visualizer=value["visualizer"],
            video=value["video"],
            observation_mode=value["observation_mode"],
            base_seed=value["base_seed"],
            worker_index=value["worker_index"],
        )
        if supplied_worker_seed != spec.worker_seed:
            raise DreamerWorkerSpecError(
                "worker_seed does not match base_seed + worker_index"
            )
        return spec


def encode_launch_spec(spec: DreamerWorkerLaunchSpec) -> str:
    return spec.to_json()


def decode_launch_spec(encoded: str) -> DreamerWorkerLaunchSpec:
    return DreamerWorkerLaunchSpec.from_json(encoded)


@dataclass(frozen=True, slots=True)
class WorkerResetResult:
    observation: Any
    info: Mapping[str, bool | int | float] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class WorkerStepResult:
    observation: Any
    reward: float
    terminated: bool
    truncated: bool
    info: Mapping[str, bool | int | float] = field(default_factory=dict)
    post_reset_observation: Any | None = None


@runtime_checkable
class WorkerEnvironment(Protocol):
    action_projection: DreamerProcessProjection
    observation_projection: DreamerProcessProjection
    schema_hashes: Mapping[str, str]

    def reset(self, *, seed: int | None) -> WorkerResetResult: ...

    def step(self, action: Any) -> WorkerStepResult: ...

    def close(self) -> None: ...


@runtime_checkable
class WorkerCleanupOwner(Protocol):
    def close(self) -> None: ...


WorkerEnvironmentFactory = Callable[[DreamerWorkerLaunchSpec], WorkerEnvironment]


def build_srb_worker_environment(spec: DreamerWorkerLaunchSpec) -> WorkerEnvironment:
    """Lazily build the production SRB child runtime after process startup."""

    from .agent_runtime import build_srb_worker_environment as build_runtime

    return build_runtime(spec)


def _validate_environment(environment: Any) -> WorkerEnvironment:
    if not isinstance(environment, WorkerEnvironment):
        raise DreamerWorkerBootstrapError(
            "worker factory must return the scalar WorkerEnvironment contract"
        )
    if not isinstance(
        environment.action_projection, DreamerProcessProjection
    ) or not isinstance(environment.observation_projection, DreamerProcessProjection):
        raise DreamerWorkerBootstrapError(
            "worker projections must be DreamerProcessProjection values"
        )
    hashes = validate_schema_hashes(environment.schema_hashes)
    expected = schema_hashes_for(
        environment.action_projection.schema_hash,
        environment.observation_projection.schema_hash,
        info_keys=INFO_ALLOWLIST,
    )
    if hashes != expected:
        raise DreamerWorkerBootstrapError(
            "worker schema hashes do not match the declared numeric schemas"
        )
    demo_projection = getattr(environment, "demo_projection", None)
    if demo_projection is not None:
        try:
            from .demo_projection import validate_demo_projection_descriptor

            validate_demo_projection_descriptor(
                demo_projection,
                action_projection=environment.action_projection,
                observation_projection=environment.observation_projection,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            raise DreamerWorkerBootstrapError(
                "worker demo projection does not match its negotiated projections"
            ) from exc
    landscaping_contract = getattr(environment, "landscaping_contract", None)
    if landscaping_contract is not None:
        try:
            validate_landscaping_contract(landscaping_contract)
        except (AttributeError, TypeError, ValueError) as exc:
            raise DreamerWorkerBootstrapError(
                "worker landscaping contract metadata is invalid"
            ) from exc
    return environment


class WorkerRuntimeContext:
    """Context that closes the environment before its app/runtime owner."""

    def __init__(
        self,
        spec: DreamerWorkerLaunchSpec,
        factory: WorkerEnvironmentFactory,
        owner: WorkerCleanupOwner | None = None,
    ) -> None:
        self._spec = spec
        self._factory = factory
        self._owner = owner
        self._environment: Any | None = None
        self._owner_closed = False

    @staticmethod
    def _diagnostic(errors: Sequence[tuple[str, BaseException]]) -> str:
        parts: list[str] = []
        for name, error in errors:
            detail = " ".join(str(error).splitlines())
            raw = detail.encode("utf-8", errors="replace")[:160]
            parts.append(
                f"{name}: {type(error).__name__}: "
                f"{raw.decode('utf-8', errors='ignore')}"
            )
        combined = "; ".join(parts)
        raw = combined.encode("utf-8", errors="replace")[:MAX_ERROR_TEXT_BYTES]
        return raw.decode("utf-8", errors="ignore")

    @classmethod
    def _annotate_cleanup(
        cls,
        primary: BaseException,
        errors: Sequence[tuple[str, BaseException]],
    ) -> None:
        if errors and hasattr(primary, "add_note"):
            primary.add_note(
                f"Dreamer worker cleanup diagnostics: {cls._diagnostic(errors)}"
            )

    def _cleanup(self) -> list[tuple[str, BaseException]]:
        errors: list[tuple[str, BaseException]] = []
        if self._environment is not None:
            close = getattr(self._environment, "close", None)
            if callable(close):
                try:
                    close()
                except BaseException as error:
                    errors.append(("environment.close", error))
        if self._owner is not None and not self._owner_closed:
            self._owner_closed = True
            close = getattr(self._owner, "close", None)
            if not callable(close):
                errors.append(
                    (
                        "owner.close",
                        DreamerWorkerBootstrapError(
                            "worker owner exposes no close method"
                        ),
                    )
                )
            else:
                try:
                    close()
                except BaseException as error:
                    errors.append(("owner.close", error))
        return errors

    def __enter__(self) -> WorkerEnvironment:
        try:
            environment = self._factory(self._spec)
            self._environment = environment
            return _validate_environment(environment)
        except BaseException as primary:
            self._annotate_cleanup(primary, self._cleanup())
            raise

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        del exc_type, traceback
        errors = self._cleanup()
        if not errors:
            return False
        if isinstance(exc, BaseException):
            self._annotate_cleanup(exc, errors)
            return False
        error = DreamerWorkerBootstrapError(
            f"worker cleanup failed: {self._diagnostic(errors)}"
        )
        raise error from errors[0][1]


def _pack_observation(
    value: Any, projection: DreamerProcessProjection, *, name: str
) -> Mapping[str, np.ndarray]:
    try:
        return projection.pack(value)
    except DreamerProcessProjectionError as exc:
        raise DreamerWorkerBootstrapError(
            f"{name} observation does not match the negotiated projection"
        ) from exc


def _validate_info(value: Any) -> dict[str, bool | int | float]:
    if not isinstance(value, Mapping):
        raise DreamerWorkerBootstrapError("worker info must be a mapping")
    return dict(value)


def _coerce_reset_result(value: Any) -> WorkerResetResult:
    if isinstance(value, WorkerResetResult):
        return value
    try:
        return WorkerResetResult(
            observation=value.observation,
            info=value.info,
        )
    except (AttributeError, TypeError) as exc:
        raise DreamerWorkerBootstrapError(
            "reset must return the WorkerResetResult protocol"
        ) from exc


def _coerce_step_result(value: Any) -> WorkerStepResult:
    if isinstance(value, WorkerStepResult):
        return value
    try:
        return WorkerStepResult(
            observation=value.observation,
            reward=value.reward,
            terminated=value.terminated,
            truncated=value.truncated,
            info=value.info,
            post_reset_observation=getattr(value, "post_reset_observation", None),
        )
    except (AttributeError, TypeError) as exc:
        raise DreamerWorkerBootstrapError(
            "step must return the WorkerStepResult protocol"
        ) from exc


def _validate_request_identity(
    request: Any,
    spec: DreamerWorkerLaunchSpec,
    *,
    expected_sequence: int,
    schema_hashes: Mapping[str, str],
    hello: bool,
) -> None:
    if request.worker_id != spec.worker_index or request.seed != spec.worker_seed:
        raise DreamerWorkerBootstrapError("request worker identity or seed is wrong")
    if request.sequence != expected_sequence:
        raise DreamerWorkerBootstrapError(
            f"request sequence {request.sequence} is stale or future; expected {expected_sequence}"
        )
    # Request PID identifies the controller/parent; response PID identifies this child.
    request_pid = getattr(request, "pid", None)
    parent_pid = os.getppid()
    if (
        type(request_pid) is not int
        or request_pid <= 0
        or type(parent_pid) is not int
        or parent_pid <= 0
        or request_pid != parent_pid
    ):
        raise DreamerWorkerBootstrapError(
            "request PID does not identify the controller parent"
        )
    request_hashes = dict(request.schema_hashes)
    if hello:
        if request_hashes not in (dict(ZERO_SCHEMA_HASHES), dict(schema_hashes)):
            raise DreamerWorkerBootstrapError("HELLO requested an incompatible schema")
    elif request_hashes != dict(schema_hashes):
        raise DreamerWorkerBootstrapError("request schema hashes changed after HELLO")


def _bounded_error_text(error: BaseException) -> str:
    text = f"{type(error).__name__}: {error}".replace("\x00", "?")
    text = " ".join(text.splitlines())
    raw = text.encode("utf-8", errors="replace")[:MAX_ERROR_TEXT_BYTES]
    return raw.decode("utf-8", errors="ignore") or "worker bootstrap failure"


def _send_error(
    connection: Any,
    spec: DreamerWorkerLaunchSpec,
    error: BaseException,
    *,
    sequence: int,
    episode_id: int,
    step_index: int,
    schema_hashes: Mapping[str, str],
) -> None:
    with suppress(Exception):
        send_message(
            connection,
            ErrorResponse(
                worker_id=spec.worker_index,
                pid=os.getpid(),
                seed=spec.worker_seed,
                sequence=max(0, sequence),
                episode_id=episode_id,
                step_index=max(0, step_index),
                schema_hashes=schema_hashes,
                code="BOOTSTRAP_ERROR",
                message=_bounded_error_text(error),
            ),
        )


def serve_worker(
    connection: Any,
    spec: DreamerWorkerLaunchSpec,
    *,
    environment_factory: WorkerEnvironmentFactory | None = None,
    owner: WorkerCleanupOwner | None = None,
) -> int:
    """Serve HELLO then scalar RESET/STEP/CLOSE until clean close or one error."""

    factory = environment_factory or build_srb_worker_environment
    hashes: Mapping[str, str] = dict(ZERO_SCHEMA_HASHES)
    expected_sequence = 0
    episode_id = -1
    step_index = 0
    post_reset_observation: Any | None = None
    close_response: ClosedResponse | None = None
    close_ack_sent = False
    try:
        runtime_context = WorkerRuntimeContext(spec, factory, owner)
        environment = runtime_context.__enter__()
        runtime_error: BaseException | None = None
        try:
            hashes = validate_schema_hashes(environment.schema_hashes)
            request = recv_message(connection)
            if not isinstance(request, HelloRequest) or request.flags != {
                "initial": True
            }:
                raise DreamerWorkerBootstrapError("first request must be HELLO")
            _validate_request_identity(
                request,
                spec,
                expected_sequence=expected_sequence,
                schema_hashes=hashes,
                hello=True,
            )
            if request.episode_id != -1 or request.step_index != 0:
                raise DreamerWorkerBootstrapError(
                    "HELLO has an invalid episode position"
                )
            send_message(
                connection,
                HelloResponse(
                    worker_id=spec.worker_index,
                    pid=os.getpid(),
                    seed=spec.worker_seed,
                    sequence=0,
                    episode_id=-1,
                    step_index=0,
                    schema_hashes=hashes,
                    action_projection=environment.action_projection.metadata,
                    observation_projection=environment.observation_projection.metadata,
                    demo_projection=getattr(environment, "demo_projection", None),
                    landscaping_contract=getattr(
                        environment, "landscaping_contract", None
                    ),
                ),
            )
            expected_sequence = 1

            while True:
                request = recv_message(connection)
                _validate_request_identity(
                    request,
                    spec,
                    expected_sequence=expected_sequence,
                    schema_hashes=hashes,
                    hello=False,
                )
                if request.episode_id != episode_id or request.step_index != step_index:
                    raise DreamerWorkerBootstrapError(
                        "request episode position does not match worker state"
                    )

                if isinstance(request, ResetRequest):
                    if request.flags != {"reset": True}:
                        raise DreamerWorkerBootstrapError("RESET flag is false")
                    if post_reset_observation is not None:
                        reset = WorkerResetResult(
                            observation=post_reset_observation,
                        )
                        post_reset_observation = None
                    else:
                        reset = _coerce_reset_result(
                            environment.reset(
                                seed=spec.worker_seed if episode_id == -1 else None
                            )
                        )
                    observation = _pack_observation(
                        reset.observation,
                        environment.observation_projection,
                        name="reset",
                    )
                    episode_id += 1
                    step_index = 0
                    send_message(
                        connection,
                        TransitionResponse(
                            worker_id=spec.worker_index,
                            pid=os.getpid(),
                            seed=spec.worker_seed,
                            sequence=expected_sequence,
                            episode_id=episode_id,
                            step_index=step_index,
                            schema_hashes=hashes,
                            observation=observation,
                            reward=0.0,
                            terminated=False,
                            truncated=False,
                            is_first=True,
                            step_delta=0,
                            info=_validate_info(reset.info),
                        ),
                    )
                elif isinstance(request, StepRequest):
                    if request.flags != {"reset": False} or episode_id < 0:
                        raise DreamerWorkerBootstrapError(
                            "STEP is invalid before reset or has the reset flag"
                        )
                    try:
                        action = environment.action_projection.unpack(request.action)
                    except DreamerProcessProjectionError as exc:
                        raise DreamerWorkerBootstrapError(
                            "STEP action does not match the negotiated projection"
                        ) from exc
                    stepped = _coerce_step_result(environment.step(action))
                    if (
                        type(stepped.terminated) is not bool
                        or type(stepped.truncated) is not bool
                    ):
                        raise DreamerWorkerBootstrapError("step flags must be boolean")
                    if type(stepped.reward) not in (int, float) or not math.isfinite(
                        float(stepped.reward)
                    ):
                        raise DreamerWorkerBootstrapError("step reward must be finite")
                    observation = _pack_observation(
                        stepped.observation,
                        environment.observation_projection,
                        name="step",
                    )
                    done = stepped.terminated or stepped.truncated
                    if done and stepped.post_reset_observation is not None:
                        _pack_observation(
                            stepped.post_reset_observation,
                            environment.observation_projection,
                            name="post-reset first observation",
                        )
                        post_reset_observation = stepped.post_reset_observation
                    step_index += 1
                    send_message(
                        connection,
                        TransitionResponse(
                            worker_id=spec.worker_index,
                            pid=os.getpid(),
                            seed=spec.worker_seed,
                            sequence=expected_sequence,
                            episode_id=episode_id,
                            step_index=step_index,
                            schema_hashes=hashes,
                            observation=observation,
                            reward=float(stepped.reward),
                            terminated=stepped.terminated,
                            truncated=stepped.truncated,
                            is_first=False,
                            step_delta=1,
                            info=_validate_info(stepped.info),
                        ),
                    )
                elif isinstance(request, CloseRequest):
                    if request.flags != {"graceful": True}:
                        raise DreamerWorkerBootstrapError("CLOSE is not graceful")
                    close_response = ClosedResponse(
                        worker_id=spec.worker_index,
                        pid=os.getpid(),
                        seed=spec.worker_seed,
                        sequence=expected_sequence,
                        episode_id=episode_id,
                        step_index=step_index,
                        schema_hashes=hashes,
                    )
                    prepare_close = getattr(environment, "prepare_close", None)
                    if callable(prepare_close):
                        prepare_close()
                        send_message(connection, close_response)
                        close_ack_sent = True
                    break
                else:
                    raise DreamerWorkerBootstrapError(
                        f"unexpected request {type(request).__name__} after HELLO"
                    )
                expected_sequence += 1
        except EOFError as error:
            runtime_error = error
            return 1
        except BaseException as error:
            runtime_error = error
            _send_error(
                connection,
                spec,
                error,
                sequence=expected_sequence,
                episode_id=episode_id,
                step_index=step_index,
                schema_hashes=hashes,
            )
            return 1
        finally:
            runtime_context.__exit__(
                type(runtime_error) if runtime_error is not None else None,
                runtime_error,
                runtime_error.__traceback__ if runtime_error is not None else None,
            )
        if close_response is None:
            raise DreamerWorkerBootstrapError("worker loop exited without CLOSE")
        if not close_ack_sent:
            send_message(connection, close_response)
        return 0
    except EOFError:
        return 1
    except BaseException as error:
        _send_error(
            connection,
            spec,
            error,
            sequence=expected_sequence,
            episode_id=episode_id,
            step_index=step_index,
            schema_hashes=hashes,
        )
        return 1


@dataclass(frozen=True, slots=True)
class WorkerBootstrapArgs:
    control_fd: int
    spec_json: str


class _StrictArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise DreamerWorkerBootstrapError(f"invalid worker argv: {message}")


def _parse_control_fd(value: str) -> int:
    if not value.isdecimal():
        raise argparse.ArgumentTypeError("control FD must be a decimal integer")
    fd = int(value)
    if not MIN_CONTROL_FD <= fd <= MAX_CONTROL_FD:
        raise argparse.ArgumentTypeError("control FD is outside the allowed range")
    return fd


def _parse_spec_argument(value: str) -> str:
    if len(value.encode("utf-8")) > MAX_SPEC_JSON_BYTES:
        raise argparse.ArgumentTypeError("spec JSON exceeds the size limit")
    return value


def parse_worker_argv(argv: Sequence[str]) -> WorkerBootstrapArgs:
    parser = _StrictArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--control-fd", required=True, type=_parse_control_fd)
    parser.add_argument("--spec-json", required=True, type=_parse_spec_argument)
    parsed = parser.parse_args(tuple(argv))
    return WorkerBootstrapArgs(control_fd=parsed.control_fd, spec_json=parsed.spec_json)


def connection_from_control_fd(control_fd: int) -> Connection:
    """Take ownership of exactly one explicitly passed POSIX connection FD."""

    if os.name != "posix":
        raise DreamerWorkerBootstrapError("worker control FD bootstrap requires POSIX")
    if (
        type(control_fd) is not int
        or not MIN_CONTROL_FD <= control_fd <= MAX_CONTROL_FD
    ):
        raise DreamerWorkerBootstrapError("control FD is outside the allowed range")
    try:
        os.fstat(control_fd)
    except OSError as exc:
        raise DreamerWorkerBootstrapError("control FD is not open") from exc
    try:
        os.set_inheritable(control_fd, False)
    except OSError as exc:
        raise DreamerWorkerBootstrapError(
            "control FD could not be made non-inheritable"
        ) from exc
    return Connection(control_fd, readable=True, writable=True)


def main(argv: Sequence[str] | None = None) -> int:
    parsed = parse_worker_argv(sys.argv[1:] if argv is None else argv)
    spec = DreamerWorkerLaunchSpec.from_json(parsed.spec_json)
    connection = connection_from_control_fd(parsed.control_fd)
    try:
        return serve_worker(connection, spec)
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DreamerWorkerBootstrapError",
    "DreamerWorkerLaunchSpec",
    "DreamerWorkerSpecError",
    "MAX_SPEC_JSON_BYTES",
    "WorkerBootstrapArgs",
    "WorkerCleanupOwner",
    "WorkerEnvironment",
    "WorkerEnvironmentFactory",
    "WorkerResetResult",
    "WorkerRuntimeContext",
    "WorkerStepResult",
    "build_srb_worker_environment",
    "connection_from_control_fd",
    "decode_launch_spec",
    "encode_launch_spec",
    "main",
    "parse_worker_argv",
    "serve_worker",
]
