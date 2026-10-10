import math
import time
from collections.abc import Callable, Hashable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from contextvars import ContextVar, Token
from dataclasses import dataclass
from functools import cached_property
from inspect import Parameter, signature
from numbers import Integral, Real
from threading import RLock, get_ident
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    SupportsFloat,
    TypeVar,
)

import gymnasium
import numpy as np
import torch

from srb._typing import ObservationGroup
from srb.core.command_mapping import (
    CommandMapping,
    revalidate_command_mapping,
    validate_command_mapping,
)
from srb.core.env.runtime.action_layout import ActionLayout
from srb.core.env.runtime.observation_layout import (
    ObservationLayout,
    ObservationSpaceLayout,
)
from srb.interfaces._ros_node import (
    RosCleanupError,
    RosExecutorCleanupError,
    acquire_ros_node_runtime,
)
from srb.interfaces.sim_to_real.core.hardware import (
    DeploymentCapability,
    DeploymentCapabilityProvider,
    HardwareDispatchNotReadyError,
    HardwareInterface,
    HardwareInterfaceCfg,
    HardwareInterfaceRegistry,
    validate_deployment_readiness,
)
from srb.interfaces.sim_to_real.core.hardware_mapping import (
    HardwareActionMapping,
    HardwareObservationMapping,
    HardwareSourceGroups,
)
from srb.interfaces.sim_to_real.core.lifecycle import raise_cleanup_errors
from srb.interfaces.sim_to_real.core.task_evaluator import (
    TIME_LIMIT_TRUNCATION,
    AcceptedFrameProvider,
    ManifestMismatchAbort,
    OperatorAbort,
    RealSessionAbort,
    RealTaskEvaluation,
    RealTaskEvaluator,
    SensorIncompleteAbort,
    SensorStaleAbort,
    verify_model_artifact_identity,
)
from srb.terrain.landscaping_external_heightmap import TargetManifest
from srb.utils import logging
from srb.utils.failures import (
    HealthStatus,
    attempt_cleanup,
    normalize_health_event,
)

if TYPE_CHECKING:
    from rclpy.node import Node as RosNode


_T = TypeVar("_T", bound=Hashable)


@dataclass(frozen=True)
class RealResourceCleanupState:
    """Read-only cleanup progress for one RealEnv hardware resource.

    Non-sinks report ``stop_done=True``. Stops count as confirmed only when the
    adapter hook returns normally; an unconfirmed sink retains its handle.
    """

    index: int
    name: str
    action_sink: bool
    stop_done: bool
    stop_attempts: int
    stop_unconfirmed: bool
    close_done: bool
    close_attempts: int


@dataclass(frozen=True)
class RealCleanupState:
    """Read-only snapshot of RealEnv close progress."""

    close_requested: bool
    completed: bool
    resources: tuple[RealResourceCleanupState, ...]
    ros_done: bool
    ros_attempts: int
    max_stop_attempts: int

    @property
    def stop_unconfirmed(self) -> bool:
        return any(resource.stop_unconfirmed for resource in self.resources)


class RealCleanupIncomplete(RosCleanupError):
    """Safe-stop retries failed; the sink and required ROS runtime stay owned.

    ``errors`` contains this attempt's failures; later calls do not retry the
    exhausted sink.
    """

    def __init__(
        self,
        message: str,
        *,
        errors: tuple[BaseException, ...] = (),
        cleanup_owner: Any | None = None,
        cleanup_state: RealCleanupState,
    ) -> None:
        super().__init__(message, errors=errors, cleanup_owner=cleanup_owner)
        self.cleanup_state = cleanup_state


def _attach_cleanup_note(
    primary: BaseException,
    secondary: BaseException,
    *,
    context: str,
) -> None:
    """Keep a secondary cleanup failure visible without replacing cancellation."""

    primary.add_note(f"{context}: {type(secondary).__name__}: {secondary}")


def _raise_cleanup_failures(
    errors: Sequence[BaseException],
    *,
    context: str,
) -> None:
    """Raise the first cancellation, while retaining all cleanup failures."""

    if not errors:
        return

    cancellation = next(
        (error for error in errors if not isinstance(error, Exception)),
        None,
    )
    if cancellation is not None:
        for error in errors:
            if error is not cancellation:
                _attach_cleanup_note(cancellation, error, context=context)
        raise cancellation

    raise_cleanup_errors(
        tuple(error for error in errors if isinstance(error, Exception)),
        context=context,
    )


class _RealEnvInternalObservation(HardwareInterface, hardware_interface_metaclass=True):
    """Private adapter for deploy-local observations; never joins the registry."""

    def __init__(self, layout: ObservationSpaceLayout):
        super().__init__(HardwareInterfaceCfg(name="real_env_internal_observation"))
        spaces: dict[str, gymnasium.spaces.Box] = {}
        for term in layout.terms:
            if term.key == "prev_action":
                spaces[term.source_key] = term.space
        self._spaces = gymnasium.spaces.Dict(spaces)
        self._zero_obs()

    def reset(self):
        super().reset()
        self._zero_obs()

    def set_prev_action(self, action: np.ndarray) -> None:
        value = np.asarray(action)
        if value.dtype.kind not in "biuf" or not np.all(np.isfinite(value)):
            raise ValueError("Previous action must contain finite real values")
        with np.errstate(over="ignore", invalid="ignore"):
            flat = value.astype(np.float32, copy=False).reshape(-1)
        if not np.all(np.isfinite(flat)):
            raise ValueError("Previous action is not finite in float32 telemetry")
        updates: dict[str, np.ndarray] = {}
        for key, space in self._spaces.spaces.items():
            if not key.endswith("/prev_action"):
                continue
            expected_size = int(np.prod(space.shape, dtype=np.int64))
            if flat.size != expected_size:
                raise ValueError(
                    f"Internal observation '{key}' expects previous action size "
                    f"{expected_size}, got {flat.size}"
                )
            with np.errstate(over="ignore", invalid="ignore"):
                converted = flat.reshape(space.shape).astype(space.dtype, copy=True)
            if not np.all(np.isfinite(converted)):
                raise ValueError(
                    f"Previous action '{key}' is not finite in dtype {space.dtype}"
                )
            if np.dtype(space.dtype).kind in "biu" and not np.array_equal(
                converted.reshape(-1), flat
            ):
                raise ValueError(
                    f"Previous action '{key}' is not exactly representable in dtype {space.dtype}"
                )
            updates[key] = converted
        self._obs.update(updates)

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return self._spaces

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self._obs

    def _zero_obs(self) -> None:
        self._obs = {
            key: np.zeros(space.shape, dtype=space.dtype)
            for key, space in self._spaces.spaces.items()
        }


def _configure_deploy_hardware(
    configurator: Callable[..., Sequence[HardwareInterface | str]],
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: ActionLayout,
) -> Sequence[HardwareInterface | str]:
    if _callable_accepts_keyword(configurator, "action_layout"):
        return configurator(hardware, action_layout=action_layout)
    return configurator(hardware)


def _callable_accepts_keyword(func: Callable[..., object], name: str) -> bool:
    try:
        parameters = signature(func).parameters.values()
    except (TypeError, ValueError):
        return False
    for parameter in parameters:
        if parameter.kind == Parameter.VAR_KEYWORD:
            return True
        if parameter.name == name and parameter.kind in (
            Parameter.KEYWORD_ONLY,
            Parameter.POSITIONAL_OR_KEYWORD,
        ):
            return True
    return False


def _validate_runtime_target_manifest(
    manifest: TargetManifest | None,
    *,
    required: bool,
    env_name: str,
) -> TargetManifest | None:
    if manifest is None:
        if required:
            raise ValueError(
                f"{env_name} declares "
                "REQUIRES_RUNTIME_TARGET_MANIFEST=True but no "
                "runtime_target_manifest was supplied; the general landscaping "
                "task remains non-ready until a verified per-episode target "
                "manifest is supplied"
            )
        return None

    try:
        manifest_sha256 = manifest.manifest_sha256
    except AttributeError as exc:
        raise ValueError("runtime_target_manifest must expose manifest_sha256") from exc
    if not isinstance(manifest_sha256, str) or not manifest_sha256.strip():
        raise ValueError(
            "runtime_target_manifest.manifest_sha256 must be a non-empty string"
        )

    verifier = getattr(manifest, "verify", None)
    if not callable(verifier):
        raise ValueError(
            "runtime_target_manifest must expose a callable verify operation"
        )
    try:
        verifier()
    except Exception as exc:
        raise ValueError(f"runtime_target_manifest failed verification: {exc}") from exc
    return manifest


class RealEnv(gymnasium.Env):
    # Keep scalar Gym actions; retain one-env compatibility properties below.
    is_vector_env: ClassVar[bool] = False

    SINGLE_ACTION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_ACTION_SPACE: ClassVar[gymnasium.spaces.Dict]
    ACTION_ORDERING: ClassVar[dict[str, slice]]
    ACTION_RATE: ClassVar[float]
    ACTION_SCALE: ClassVar[dict[str, float]]
    COMMAND_MAPPING: ClassVar[CommandMapping | None] = None

    SINGLE_OBSERVATION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_OBSERVATION_SPACE: ClassVar[gymnasium.spaces.Dict]

    SIM_ROBOT: ClassVar[str]
    HARDWARE: ClassVar[Sequence[str]]
    DEPLOY_HARDWARE_CONFIGURATOR: ClassVar[
        Callable[..., Sequence[HardwareInterface | str]] | None
    ] = None

    # Legacy generated tasks default these fields; canonical modules pin
    # identity/readiness for startup validation and policy matching.
    ENV_ID: ClassVar[str] = ""
    IO_SCHEMA_FINGERPRINT: ClassVar[str | None] = None
    TARGET_MANIFEST_SHA256: ClassVar[str | None] = None
    REQUIRES_RUNTIME_TARGET_MANIFEST: ClassVar[bool] = False
    DEPLOYMENT_REQUIREMENTS: ClassVar[frozenset[DeploymentCapability]] = frozenset()

    IS_RUNNING_AT_START: ClassVar[bool] = True

    LIMIT_RATE: ClassVar[bool] = True
    MEASURE_PERFORMANCE: ClassVar[bool] = not LIMIT_RATE
    DEPLOY_EXCLUDED_OBSERVATION_GROUPS: ClassVar[Sequence[str]] = (
        ObservationGroup.STATE_PRIVILEGED,
    )

    _HOT_START_DURATION: ClassVar[float] = 5.0
    _HOT_START_ITERATIONS: ClassVar[int] = 10
    _MIN_SLEEP_DURATION: ClassVar[float] = 1.0 / 1000.0
    _PAUSE_SLEEP_DURATION: ClassVar[float] = 1.0 / 100.0
    _SPIN_WAIT_THRESHOLD: ClassVar[float] = 2.0 / 1000.0
    _FREQ_EST_EMA_ALPHA: ClassVar[float] = 0.9
    _PERF_LOG_INTERVAL_S: ClassVar[float] = 10.0
    _PERF_LOG_INTERVAL_STEPS: ClassVar[int] = 100
    # Reset clears sensor evidence; wait for first-sample states while held.
    _RESET_READY_POLL_S: ClassVar[float] = 1.0 / 100.0
    _RESET_READY_WAIT_STATES: ClassVar[frozenset[str]] = frozenset(
        {"pre_message", "partial", "incomplete"}
    )
    # Bounded escalation for a sink whose safe stop keeps failing in close().
    _MAX_CLOSE_STOP_ATTEMPTS: ClassVar[int] = 3

    def __init__(
        self,
        hardware: Sequence[HardwareInterface | str],
        ros_node: "RosNode | None" = None,
        capability_providers: Sequence[DeploymentCapabilityProvider] = (),
        runtime_target_manifest: TargetManifest | None = None,
        task_evaluator: RealTaskEvaluator | None = None,
        model_artifact: Mapping[str, Any] | None = None,
        horizon_steps: int | None = None,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None,
        pause_timeout_s: float = 5.0,
        reset_ready_timeout_s: float = 5.0,
        _adopt_cleanup_owner: Callable[["RealEnv"], None] | None = None,
        **kwargs,
    ):
        super().__init__()
        if (
            isinstance(pause_timeout_s, bool)
            or not np.isfinite(pause_timeout_s)
            or pause_timeout_s <= 0.0
        ):
            raise ValueError("pause_timeout_s must be finite and positive")
        if (
            isinstance(reset_ready_timeout_s, bool)
            or not np.isfinite(reset_ready_timeout_s)
            or reset_ready_timeout_s <= 0.0
        ):
            raise ValueError("reset_ready_timeout_s must be finite and positive")
        # ACTION_RATE is a seconds-valued control period; reject invalid values
        # before hardware acquisition to prevent broken pacing after dispatch.
        action_rate = getattr(self, "ACTION_RATE", None)
        if (
            isinstance(action_rate, bool)
            or not isinstance(action_rate, Real)
            or not math.isfinite(action_rate)
            or action_rate <= 0.0
        ):
            raise ValueError(
                f"{type(self).__name__}.ACTION_RATE (control period in seconds) "
                f"must be finite and positive, got {action_rate!r}"
            )
        self._clock = time.monotonic if clock is None else clock
        self._sleep = time.sleep if sleep is None else sleep
        self._pause_timeout_s = float(pause_timeout_s)
        self._reset_ready_timeout_s = float(reset_ready_timeout_s)
        self._session_abort_reason: str | None = None
        # A reentrant callback may latch abort before claiming stop; track
        # whether cleanup started so the outer failure stops once.
        self._session_abort_cleanup_started: bool = False
        # ``_closed`` blocks dispatch as soon as close is requested; completion
        # is separate so failed resources remain retryable.
        self._closed: bool = False
        self._close_requested: bool = False
        self._close_completed: bool = False
        # Guard readiness and command commit together. Claim lifecycle stops
        # under this lock, but run transport hooks after releasing it.
        self._dispatch_lock = RLock()
        self._dispatch_owner: int | None = None
        self._dispatch_depth: int = 0
        self._close_lock = RLock()
        self._close_owner: int | None = None
        self._stop_owner: int | None = None
        self._close_safe_stop_done: set[int] = set()
        self._close_hardware_done: set[int] = set()
        self._close_ros_done: bool = False
        self._close_super_done: bool = False
        self._close_stop_attempts: dict[int, int] = {}
        self._close_hardware_attempts: dict[int, int] = {}
        self._close_ros_attempts: int = 0
        self._close_stop_unconfirmed: set[int] = set()
        self._command_dispatch_attempted: bool = False
        self._command_dispatch_precondition_failure: ContextVar[
            BaseException | None
        ] = ContextVar(
            f"real_env_command_dispatch_precondition_failure_{id(self)}",
            default=None,
        )
        self._command_dispatch_precondition: ContextVar[Callable[[], None] | None] = (
            ContextVar(
                f"real_env_command_dispatch_precondition_{id(self)}", default=None
            )
        )

        # A pinned schema may omit a static manifest hash only when each
        # episode supplies its manifest.
        if (
            self.IO_SCHEMA_FINGERPRINT is not None
            and self.TARGET_MANIFEST_SHA256 is None
            and not self.REQUIRES_RUNTIME_TARGET_MANIFEST
        ):
            raise ValueError(
                f"{type(self).__name__}: TARGET_MANIFEST_SHA256 may be None "
                "only when REQUIRES_RUNTIME_TARGET_MANIFEST=True"
            )

        # Preserve both flat ingress and structured routing contracts.
        self._action_layout = ActionLayout.from_spaces(
            self.DETAILED_ACTION_SPACE,
            ordering=self.ACTION_ORDERING,
            scale=self.ACTION_SCALE,
            single_space=self.SINGLE_ACTION_SPACE,
        )
        declared_mapping = type(self).COMMAND_MAPPING
        if declared_mapping is None:
            self._command_mapping: CommandMapping | None = None
        else:
            self._command_mapping = (
                revalidate_command_mapping(declared_mapping)
                if isinstance(declared_mapping, CommandMapping)
                else CommandMapping.model_validate(declared_mapping)
            )
            validate_command_mapping(
                self._command_mapping,
                self._action_layout.ordering,
                int(self._action_layout.single_space.shape[0]),
            )
        self._strict_action_layout = ActionLayout.from_structured_space(
            self.DETAILED_ACTION_SPACE
        )
        self._single_action_space = self._action_layout.single_space
        self._action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, 1
        )
        self._observation_layout = ObservationLayout.from_spaces(
            self.DETAILED_OBSERVATION_SPACE,
            output_space=self.SINGLE_OBSERVATION_SPACE,
            exclude_categories=self.DEPLOY_EXCLUDED_OBSERVATION_GROUPS,
        )
        self._internal_observation = _RealEnvInternalObservation(
            self._observation_layout
        )
        self._single_observation_space = self._observation_layout.build_spaces()
        self._observation_space = self.single_observation_space

        hardware_specs = tuple(hardware or self.HARDWARE)
        configurator = type(self).DEPLOY_HARDWARE_CONFIGURATOR
        if configurator is not None:
            hardware_specs = tuple(
                _configure_deploy_hardware(
                    configurator,
                    hardware_specs,
                    action_layout=self._action_layout,
                )
            )
        self._hardware: Sequence[HardwareInterface] = tuple(
            dict.fromkeys(self._resolve_hardware_interface(hw) for hw in hardware_specs)
        )

        # Preflight resolved, unstarted hardware and all capability providers
        # before ROS acquisition, hardware start, or policy loading. The
        # manifest hash/verifier are checked here; episode materialization and
        # reset comparison belong to the target-map/evaluator seam.
        self._task_evaluator = task_evaluator
        self._capability_providers: tuple[DeploymentCapabilityProvider, ...] = tuple(
            (*capability_providers, task_evaluator)
            if task_evaluator is not None
            else capability_providers
        )
        self._runtime_target_manifest = _validate_runtime_target_manifest(
            runtime_target_manifest,
            required=self.REQUIRES_RUNTIME_TARGET_MANIFEST,
            env_name=type(self).__name__,
        )
        validate_deployment_readiness(
            self.DEPLOYMENT_REQUIREMENTS,
            (*self._hardware, *self._capability_providers),
        )
        # Validate evaluator wiring and model identity before acquiring ROS or
        # starting hardware.
        if horizon_steps is not None and (
            isinstance(horizon_steps, bool)
            or not isinstance(horizon_steps, Integral)
            or horizon_steps < 1
        ):
            raise ValueError("horizon_steps must be a positive integer")
        self._horizon_steps: int | None = (
            int(horizon_steps) if horizon_steps is not None else None
        )
        self._frame_provider: AcceptedFrameProvider | None = None
        if task_evaluator is not None:
            self._prepare_task_evaluator(task_evaluator, model_artifact)
        self._episode_steps: int = 0
        self._horizon_reached: bool = False

        for i, hw in enumerate(self._hardware, 1):
            logging.info(f"Hardware interface #{i}: {hw.name}")

        if isinstance(self.DETAILED_ACTION_SPACE, gymnasium.spaces.Dict):
            self._action_mapping = HardwareActionMapping.build(
                self._hardware,
                self._action_layout,
            )
        else:
            raise ValueError(
                f"Unexpected action space type: {self.DETAILED_ACTION_SPACE}"
            )
        self._sink_action = self._action_mapping.interfaces
        self._hardware_action_map = self._action_mapping.routes

        # Validate routes before ROS acquisition or hardware startup.
        self._source_groups = HardwareSourceGroups.build(self._hardware)
        internal_observation = (
            (self._internal_observation,)
            if self._internal_observation._has_io_observation
            else ()
        )
        self._src_observation = self._source_groups.observation + internal_observation
        self._src_reward = self._source_groups.reward
        self._src_termination = self._source_groups.termination
        self._src_pause = self._source_groups.pause
        self._src_resume = self._source_groups.resume
        if isinstance(self.DETAILED_OBSERVATION_SPACE, gymnasium.spaces.Dict):
            self._observation_mapping = HardwareObservationMapping.build(
                self._src_observation,
                self._observation_layout,
            )
        else:
            raise ValueError(
                f"Unexpected observation space type: {self.DETAILED_OBSERVATION_SPACE}"
            )
        self._hardware_observation_map = self._observation_mapping.routes

        if _adopt_cleanup_owner is not None:
            _adopt_cleanup_owner(self)

        self._ros_runtime = acquire_ros_node_runtime(
            ros_node, name="real", namespace="srb", spin=True
        )
        self._ros_node = self._ros_runtime.node

        # Roll back every attempted interface and the owned ROS runtime if
        # startup or hot sync fails.
        attempted: list[HardwareInterface] = []
        try:
            for hw in self._hardware:
                attempted.append(hw)
                hw.start(
                    action_rate=self.ACTION_RATE,
                    action_scale=self._action_mapping.scale_for(
                        self._action_layout, hw
                    ),
                    ros_node=self._ros_node,
                )

            hot_start_sleep_period = (
                self._HOT_START_DURATION / self._HOT_START_ITERATIONS
            )
            for _ in range(self._HOT_START_ITERATIONS):
                self._sleep(hot_start_sleep_period)
                for hw in self._hardware:
                    hw.sync()
        except BaseException as startup_error:
            self._rollback_hardware_startup(attempted, startup_error)
            raise

        logging.info(self._source_groups.summary(self._sink_action))

        self._is_running: bool = self.IS_RUNNING_AT_START
        self._extract_duration_ema: float = 0.0
        self._last_step_time: float = 0.0
        self._step_freq_ema: float = 0.0
        self._perf_log_last_time: float = 0.0
        self._perf_log_last_step: int = 0
        self._step_count: int = 0
        self._next_step_deadline: float = 0.0

    @property
    def ros_node(self) -> "RosNode":
        """Return the environment's borrowed ROS node handle."""
        return self._ros_node

    @contextmanager
    def command_dispatch_precondition(
        self, check: Callable[[], None]
    ) -> Iterator[None]:
        """Bind one synchronous step's final policy-input validity check."""
        if not callable(check):
            raise TypeError("command dispatch precondition must be callable")
        token: Token[Callable[[], None] | None] = (
            self._command_dispatch_precondition.set(check)
        )
        try:
            yield
        finally:
            self._command_dispatch_precondition.reset(token)

    def _rollback_hardware_startup(
        self,
        attempted: Sequence[HardwareInterface],
        startup_error: BaseException,
    ) -> None:
        """Best-effort rollback that preserves the initiating startup error."""

        self._session_abort_reason = (
            f"hardware startup failed: {type(startup_error).__name__}: {startup_error}"
        )
        cleanup_errors: list[BaseException] = []
        hardware = tuple(getattr(self, "_hardware", ()))
        hardware_index = {id(hw): index for index, hw in enumerate(hardware)}
        attempted_ids = {id(hw) for hw in attempted}
        safe_stop_failed: set[int] = set()
        for hw in self._sink_action:
            if not hw.is_started or hw.safe_stop_issued:
                if id(hw) in hardware_index:
                    self._close_safe_stop_done.add(hardware_index[id(hw)])
                continue
            if id(hw) in hardware_index:
                self._record_cleanup_attempt(
                    self._close_stop_attempts, hardware_index[id(hw)]
                )
            try:
                hw.safe_stop()
            except BaseException as exc:  # stop all interfaces before reporting
                cleanup_errors.append(exc)
                if id(hw) in hardware_index:
                    safe_stop_failed.add(hardware_index[id(hw)])
            else:
                if id(hw) in hardware_index:
                    self._close_safe_stop_done.add(hardware_index[id(hw)])
        for hw in reversed(tuple(attempted)):
            index = hardware_index.get(id(hw))
            if index is not None and index in safe_stop_failed:
                continue
            if index is not None:
                self._record_cleanup_attempt(self._close_hardware_attempts, index)
            try:
                hw.close()
            except BaseException as exc:  # close all attempted interfaces
                cleanup_errors.append(exc)
            else:
                if index is not None:
                    self._close_hardware_done.add(index)

        # Never-started interfaces need no cleanup. A failed attempted close
        # remains unfinished and is retried by close().
        self._close_hardware_done.update(
            index for index, hw in enumerate(hardware) if id(hw) not in attempted_ids
        )
        # Keep ROS alive while unfinished hardware may need its transport for
        # a later close() retry.
        all_hardware_done = len(self._close_hardware_done) == len(hardware)
        if all_hardware_done:
            self._close_ros_attempts += 1
            try:
                self._ros_runtime.close()
            except BaseException as exc:
                cleanup_errors.append(exc)
            else:
                self._close_ros_done = True
        self._closed = True
        self._close_requested = True
        self._close_completed = (
            all_hardware_done and self._close_ros_done and not cleanup_errors
        )
        self._close_super_done = True
        for cleanup_error in cleanup_errors:
            startup_error.add_note(
                "startup rollback also failed: "
                f"{type(cleanup_error).__name__}: {cleanup_error}"
            )
        if not self._close_completed:
            # Preserve cancellation identity and expose the retry owner.
            setattr(startup_error, "cleanup_errors", tuple(cleanup_errors))
            setattr(startup_error, "cleanup_owner", self)

    @staticmethod
    def _record_cleanup_attempt(attempts: dict[int, int], index: int) -> int:
        count = attempts.get(index, 0) + 1
        attempts[index] = count
        return count

    def _prepare_task_evaluator(
        self,
        task_evaluator: RealTaskEvaluator,
        model_artifact: Mapping[str, Any] | None,
    ) -> None:
        """Validate task-evaluator wiring and the loaded model's identity.

        Runs inside ``__init__`` before ROS acquisition, hardware start, and
        any policy load, so a deployment whose evaluator cannot be fed, whose
        reward would be double-counted, or whose model artifact is not this
        task's never reaches inference.
        """

        providers = tuple(
            hw for hw in self._hardware if isinstance(hw, AcceptedFrameProvider)
        )
        invalid_providers = tuple(
            hw for hw in providers if not callable(getattr(hw, "accepted_frame", None))
        )
        if invalid_providers:
            raise TypeError(
                "accepted-frame provider(s) must expose callable accepted_frame(): "
                + ", ".join(hw.name for hw in invalid_providers)
            )
        if len(providers) != 1:
            raise ValueError(
                f"{type(self).__name__} was given a task evaluator but "
                f"{len(providers)} hardware interfaces provide accepted sensor "
                "frames; exactly one accepted-frame provider is required so the "
                "evaluator grades one synchronized source"
            )
        self._frame_provider = providers[0]

        reward_sources = tuple(hw for hw in self._hardware if hw._has_io_reward)
        if reward_sources:
            raise ValueError(
                "a real task evaluator owns the task reward, but hardware "
                f"interface(s) {[hw.name for hw in reward_sources]} also emit a "
                "reward; generic hardware reward messages are not a task "
                "evaluator"
            )

        identity = getattr(task_evaluator, "identity", None)
        if identity is None:
            raise ManifestMismatchAbort(
                "task evaluator declares no deployment identity; refusing to "
                "start a real environment without a manifest-backed task "
                "identity"
            )
        self._verify_generated_identity(identity)
        verify_model_artifact_identity(identity, model_artifact)

        if not callable(getattr(task_evaluator, "validate_frame", None)):
            raise TypeError(
                "task evaluator must expose callable validate_frame(frame) so "
                "RealEnv can reject an invalid accepted frame before dispatch"
            )
        if not callable(getattr(task_evaluator, "evaluate", None)):
            raise TypeError(
                "task evaluator must expose callable evaluate(frame, action)"
            )

        if self._horizon_steps is None:
            evaluator_horizon = task_evaluator.horizon_steps
            if (
                isinstance(evaluator_horizon, bool)
                or not isinstance(evaluator_horizon, Integral)
                or evaluator_horizon < 1
            ):
                raise ValueError(
                    "task evaluator horizon_steps must be a positive integer"
                )
            self._horizon_steps = int(evaluator_horizon)

    def _verify_generated_identity(self, identity: object) -> None:
        """Refuse an evaluator whose contract is not this generated module's.

        The generated module pins the IO-schema fingerprint and (for a
        static-manifest task) the target-manifest hash its spaces were
        rendered from. The model artifact is compared against the evaluator's
        identity, so an evaluator built for a different contract or manifest
        would otherwise pass startup with mismatched spaces.
        """

        mismatches: list[str] = []
        declared_fingerprint = type(self).IO_SCHEMA_FINGERPRINT
        deployed_fingerprint = getattr(identity, "io_schema_fingerprint", None)
        if (
            declared_fingerprint is not None
            and declared_fingerprint != deployed_fingerprint
        ):
            mismatches.append(
                "io_schema_fingerprint: generated module="
                f"{declared_fingerprint!r} evaluator={deployed_fingerprint!r}"
            )
        declared_manifest = type(self).TARGET_MANIFEST_SHA256
        deployed_manifest = getattr(identity, "target_manifest_sha256", None)
        if declared_manifest is not None and declared_manifest != deployed_manifest:
            mismatches.append(
                "target_manifest_sha256: generated module="
                f"{declared_manifest!r} evaluator={deployed_manifest!r}"
            )
        if mismatches:
            raise ManifestMismatchAbort(
                f"{type(self).__name__} was generated for a different deployment "
                "identity than the task evaluator declares; refusing to start "
                "(" + "; ".join(mismatches) + ")"
            )

    def _accepted_task_frame(self, *, phase: str) -> Any:
        """Return one complete accepted frame or fail the physical session."""

        if self._frame_provider is None:
            raise RuntimeError("task evaluator has no accepted-frame provider")
        frame = self._frame_provider.accepted_frame()
        if frame is None:
            source = getattr(self._frame_provider, "name", self._frame_provider)
            raise SensorIncompleteAbort(
                f"accepted-frame provider '{source}' has no complete frame "
                f"{phase}; a real session never dispatches or evaluates using "
                "substituted, zero-filled, or silently held data"
            )
        return frame

    def _validate_action_interfaces_ready(self) -> None:
        """Validate every hardware participant without mutating deployment state."""

        for hw in self._hardware:
            try:
                hw.validate_dispatch_ready()
            except HardwareDispatchNotReadyError as exc:
                event = exc.event
                code = event.code.casefold() if event is not None else ""
                if "stale" in code:
                    raise SensorStaleAbort(str(exc)) from exc
                raise OperatorAbort(str(exc)) from exc

    def _synchronize_dispatch_inputs(self) -> Any | None:
        """Read transport inputs under the dispatch guard before slow checks."""

        self._ensure_session_active()
        for hw in self._hardware:
            hw.sync()
        self._validate_action_interfaces_ready()
        if self._task_evaluator is None:
            return None
        return self._accepted_task_frame(phase="before command dispatch")

    def _validate_dispatch_preconditions(
        self,
        pre_dispatch_frame: Any | None = None,
    ) -> None:
        """Validate the task frame after guarded transport reads complete."""

        if self._task_evaluator is None:
            return
        frame = (
            pre_dispatch_frame
            if pre_dispatch_frame is not None
            else self._accepted_task_frame(phase="before command dispatch")
        )
        try:
            self._task_evaluator.validate_frame(frame)
        except RealSessionAbort:
            raise
        except Exception as exc:  # noqa: BLE001 - frame provider is a trust seam
            abort = SensorIncompleteAbort(
                "task evaluator rejected the pre-dispatch accepted frame: "
                f"{type(exc).__name__}: {exc}"
            )
            raise abort from exc

    def _evaluate_task(self, action: np.ndarray) -> RealTaskEvaluation:
        """Grade this step's accepted frame with the task-owned evaluator."""

        if self._task_evaluator is None:
            raise RuntimeError("task evaluation requested without an evaluator")
        frame = self._accepted_task_frame(phase="after command dispatch")
        return self._task_evaluator.evaluate(frame=frame, action=action)

    def _dispatch_is_reentrant(self) -> bool:
        """Return whether this thread is already inside a dispatch transaction."""

        return (
            getattr(self, "_dispatch_owner", None) == get_ident()
            and getattr(self, "_dispatch_depth", 0) > 0
        )

    @contextmanager
    def _dispatch_scope(self) -> Iterator[None]:
        """Serialize command/read transactions and track callback re-entry."""

        dispatch_lock = getattr(self, "_dispatch_lock", None)
        if dispatch_lock is None:
            dispatch_lock = RLock()
            self._dispatch_lock = dispatch_lock
        with dispatch_lock:
            self._dispatch_owner = get_ident()
            self._dispatch_depth = getattr(self, "_dispatch_depth", 0) + 1
            try:
                yield
            finally:
                self._dispatch_depth -= 1
                if self._dispatch_depth == 0:
                    self._dispatch_owner = None

    @contextmanager
    def _close_scope(self, close_lock: Any) -> Iterator[None]:
        """Claim cleanup without holding its lock across resource teardown."""

        owner = get_ident()
        with close_lock:
            active_owner = getattr(self, "_close_owner", None)
            if active_owner is not None:
                if active_owner == owner:
                    raise OperatorAbort(
                        "real hardware session close was requested reentrantly "
                        "during cleanup; the current cleanup attempt must return "
                        "before close can be retried"
                    )
                raise RuntimeError(
                    "real hardware session cleanup is already in progress; "
                    "retry close after the active cleanup attempt returns"
                )
            self._close_owner = owner
        try:
            yield
        finally:
            with close_lock:
                self._close_owner = None

    def _reject_reentrant_lifecycle(self, operation: str) -> None:
        """Abort a callback that would otherwise continue a partial dispatch."""

        if not self._dispatch_is_reentrant():
            return
        if getattr(self, "_session_abort_reason", None) is None:
            self._session_abort_reason = f"{operation} requested during dispatch"
        raise OperatorAbort(
            f"real hardware session {operation} was requested from an active "
            "dispatch callback; dispatch is aborted and safe stop will run "
            "after the callback returns"
        )

    def _reject_stop_conflict(self, operation: str) -> None:
        """Fail immediately when another thread owns the physical stop phase."""

        owner = getattr(self, "_stop_owner", None)
        if owner is None:
            return
        if owner == get_ident():
            raise OperatorAbort(
                f"real hardware session {operation} was requested reentrantly "
                "during safe stop; retry after the stop callback returns"
            )
        raise RuntimeError(
            f"real hardware session {operation} overlaps an active safe stop; "
            "retry after the stop attempt returns"
        )

    def _reject_close_conflict(self, operation: str) -> None:
        """Fail lifecycle callbacks that would wait on active cleanup."""

        owner = getattr(self, "_close_owner", None)
        if owner is None:
            return
        if owner == get_ident():
            raise OperatorAbort(
                f"real hardware session {operation} was requested reentrantly "
                "during cleanup; retry after cleanup returns"
            )
        raise RuntimeError(
            f"real hardware session {operation} overlaps active cleanup; "
            "retry after the cleanup attempt returns"
        )

    def _claim_stop(self) -> None:
        """Claim the stop phase while the dispatch guard is held."""

        if getattr(self, "_stop_owner", None) is not None:
            raise RuntimeError("real hardware safe stop is already in progress")
        self._stop_owner = get_ident()

    def _release_stop(self) -> None:
        """Release the stop phase while the dispatch guard is held."""

        self._stop_owner = None

    def _extract_step_result(
        self,
        flat_effective_action: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], SupportsFloat, bool, bool, dict[str, Any]]:
        """Read one post-dispatch frame while teardown is excluded."""

        with self._dispatch_scope():
            self._ensure_session_active()

            # Hold the dispatch lock through sync/read so close cannot tear
            # down an adapter still using its transport.
            if self.LIMIT_RATE:
                pre_extract_time: float = self._clock()
            for hw in self._hardware:
                hw.sync()
            # A failed refresh may retain its last frame for diagnosis. Check
            # every participant, including observation-only adapters, before
            # any old buffer can become policy input or task evidence.
            self._validate_action_interfaces_ready()

            observation = self._build_observation_structure()

            reward: float = 0.0
            for hw in self._src_reward:
                reward += hw.reward
            terminated: bool = False
            for hw in self._src_termination:
                if hw.termination:
                    if self._task_evaluator is not None:
                        # With an evaluator, hardware stops abort the session;
                        # they are not task terminations.
                        raise OperatorAbort(
                            f"hardware interface '{hw.name}' signalled a stop; a "
                            "real landscaping session treats this as an operator "
                            "abort, not a task termination"
                        )
                    terminated = True
                    break
            info: dict[str, Any] = {hw.name: hw.info for hw in self._hardware}

            # Grade only this step's accepted frame and verified manifest.
            truncated: bool = False
            evaluation: RealTaskEvaluation | None = None
            if self._task_evaluator is not None:
                evaluation = self._evaluate_task(flat_effective_action)
                reward = evaluation.reward
                info.update(evaluation.metrics)
            # Reject aggregate NaN/Inf before it reaches a policy or dataset.
            if not math.isfinite(float(reward)):
                raise SensorIncompleteAbort(
                    f"real step produced a non-finite reward ({reward!r}); "
                    "the session is aborted instead of returning it"
                )
            self._episode_steps += 1
            if (
                self._horizon_steps is not None
                and self._episode_steps >= self._horizon_steps
            ):
                # Preserve the final observation; a horizon is a truncation.
                truncated = True
                self._horizon_reached = True
                info["term_reason"] = TIME_LIMIT_TRUNCATION
                info[TIME_LIMIT_TRUNCATION] = True
                info["episode_steps"] = self._episode_steps
                if self._task_evaluator is not None:
                    if evaluation is None:
                        raise RuntimeError("final task evaluation is unavailable")
                    info.update(self._task_evaluator.final_info())
                    info["success"] = bool(evaluation.success)

            if self.MEASURE_PERFORMANCE:
                info["controller_freq"] = self._step_freq_ema
                if self._step_freq_ema > 0:
                    info["controller_latency_ms"] = (1.0 / self._step_freq_ema) * 1000
            if self.LIMIT_RATE:
                self._extract_duration_ema = (
                    self._FREQ_EST_EMA_ALPHA * self._extract_duration_ema
                    + (1.0 - self._FREQ_EST_EMA_ALPHA)
                    * (self._clock() - pre_extract_time)
                )

            result = observation, reward, terminated, truncated, info

        # Reset outside the read transaction to preserve lifecycle ordering.
        if terminated:
            logging.info("Resetting environment due to termination")
            self.reset()

        if self.MEASURE_PERFORMANCE:
            self._last_step_time = self._clock()

        return result

    def step(
        self,
        action: np.ndarray | torch.Tensor | dict[str, np.ndarray | torch.Tensor],
    ) -> tuple[dict[str, np.ndarray], SupportsFloat, bool, bool, dict[str, Any]]:
        bound_precondition = self._command_dispatch_precondition.get()
        if bound_precondition is not None:
            self._command_dispatch_precondition.set(None)
        _, entry_error = attempt_cleanup(self._ensure_session_active)
        if entry_error is not None:
            if isinstance(entry_error, Exception):
                raise entry_error
            self._latch_session_failure(
                f"step entry cancellation: {type(entry_error).__name__}: {entry_error}",
                initiating=entry_error,
            )
            raise entry_error
        failure_token = self._command_dispatch_precondition_failure.set(None)
        try:
            return self._step_active(action, bound_precondition)
        except BaseException as exc:
            if isinstance(exc, RealSessionAbort):
                self._latch_session_failure(f"{exc.outcome}: {exc}", initiating=exc)
            elif self._command_dispatch_precondition_failure.get() is exc:
                self._latch_session_failure(
                    "command source final validity check failed: "
                    f"{type(exc).__name__}: {exc}",
                    initiating=exc,
                )
            elif not isinstance(exc, Exception):
                self._latch_session_failure(
                    f"step cancellation: {type(exc).__name__}: {exc}",
                    initiating=exc,
                )
            elif self._command_dispatch_attempted:
                self._latch_session_failure(
                    f"step failure after dispatch began: {type(exc).__name__}: {exc}"
                )
            raise
        finally:
            self._command_dispatch_attempted = False
            self._command_dispatch_precondition_failure.reset(failure_token)

    def _step_active(
        self,
        action: np.ndarray | torch.Tensor | dict[str, np.ndarray | torch.Tensor],
        bound_precondition: Callable[[], None] | None = None,
    ) -> tuple[dict[str, np.ndarray], SupportsFloat, bool, bool, dict[str, Any]]:
        # A truncated horizon stays latched until reset; another step would
        # command hardware for a finished episode and grade it twice.
        if self._horizon_reached:
            raise RuntimeError(
                f"{type(self).__name__}.step() was called after the configured "
                f"horizon of {self._horizon_steps} steps truncated the episode; "
                "call reset() before stepping again"
            )

        # Compatibility action_space has one batch row; accept only that shape.
        action = self._unbatch_compat_action(action)

        # Validate the whole action before pause/resume or hardware mutation.
        was_flat_action = not isinstance(action, Mapping)
        flat_action = action
        action = self._action_layout.structure(action)
        self._strict_action_layout.coerce_batched(
            {key: value.reshape((1, *value.shape)) for key, value in action.items()},
            num_envs=1,
            device="cpu",
            strict_domain=True,
        )
        # Validate terms first for channel diagnostics; the aggregate space
        # remains authoritative for flat ingress.
        if was_flat_action:
            self._validate_flat_action_domain(flat_action)

        if self.MEASURE_PERFORMANCE:
            current_time = self._clock()
            if self._last_step_time > 0:
                step_duration = current_time - self._last_step_time
                if step_duration > 0:
                    step_freq = 1.0 / step_duration
                    self._step_freq_ema = (
                        self._FREQ_EST_EMA_ALPHA * self._step_freq_ema
                        + (1.0 - self._FREQ_EST_EMA_ALPHA) * step_freq
                    )
            self._step_count += 1

            if (
                current_time - self._perf_log_last_time > self._PERF_LOG_INTERVAL_S
                and self._step_count - self._perf_log_last_step
                > self._PERF_LOG_INTERVAL_STEPS
            ):
                if self._step_freq_ema > 0:
                    latency_ms = (1.0 / self._step_freq_ema) * 1000
                    logging.info(
                        f"Controller frequency: {self._step_freq_ema:.2f} Hz "
                        f"({latency_ms:.2f} ms)"
                    )
                else:
                    logging.info(f"Controller frequency: {self._step_freq_ema:.2f} Hz")
                self._perf_log_last_time = current_time
                self._perf_log_last_step = self._step_count

        if self._is_running:
            with self._dispatch_scope():
                self._ensure_session_active()
                for hw in self._src_pause:
                    if hw.pause_signal:
                        logging.info(f"Pause signal received from {hw.name}")
                        pause_errors: list[BaseException] = []
                        for hw in self._hardware:
                            _, pause_error = attempt_cleanup(hw.pause)
                            if pause_error is not None:
                                pause_errors.append(pause_error)
                        self._is_running = False
                        if pause_errors:
                            # Cancellation (e.g. KeyboardInterrupt) wins over
                            # ordinary pause failures.
                            first_error = next(
                                (
                                    e
                                    for e in pause_errors
                                    if not isinstance(e, Exception)
                                ),
                                pause_errors[0],
                            )
                            failure = (
                                first_error
                                if not isinstance(first_error, Exception)
                                else OperatorAbort(
                                    "hardware pause failed: "
                                    f"{type(first_error).__name__}: {first_error}"
                                )
                            )
                            for pause_error in pause_errors:
                                if pause_error is not first_error:
                                    _attach_cleanup_note(
                                        failure,
                                        pause_error,
                                        context="Additional hardware pause failure",
                                    )
                            if failure is first_error:
                                raise failure
                            raise failure from first_error
                        for hw in self._src_resume:
                            _discard = hw.resume_signal
                        break
        if not self._is_running:
            pause_started = self._clock()
            while True:
                elapsed = self._clock() - pause_started
                if elapsed >= self._pause_timeout_s:
                    raise OperatorAbort(
                        "paused real environment did not receive a resume signal "
                        f"within {self._pause_timeout_s:.3f} s"
                    )
                self._sleep(
                    min(
                        self._PAUSE_SLEEP_DURATION,
                        self._pause_timeout_s - elapsed,
                    )
                )
                resumed = False
                with self._dispatch_scope():
                    self._ensure_session_active()
                    for hw in self._src_resume:
                        hw.sync()
                        if hw.resume_signal:
                            logging.info(f"Resume signal received from {hw.name}")
                            for hw in self._hardware:
                                hw.resume()
                            self._is_running = True
                            for hw in self._src_pause:
                                _discard = hw.pause_signal
                            resumed = True
                            break
                if not resumed:
                    continue
                break

        # Refresh feedback and capture the accepted frame under the guard;
        # evaluator validation runs outside it. Failed refreshes stay fail-closed.
        with self._dispatch_scope():
            pre_dispatch_frame = self._synchronize_dispatch_inputs()

        # Require the current accepted frame to pass its fail-closed gate.
        self._validate_dispatch_preconditions(pre_dispatch_frame)

        # Keep slow checks outside the lock so lifecycle stops can finish;
        # recheck readiness before commit because the earlier gate is a snapshot.
        with self._dispatch_scope():
            self._ensure_session_active()
            # Hold and adapter-local health changes do not latch the session.
            self._validate_action_interfaces_ready()
            self._ensure_session_active()

            # Exclude paused time from the action-rate budget.
            if self.LIMIT_RATE:
                pre_action_time: float = self._clock()

            def validate_bound_precondition() -> None:
                if bound_precondition is None:
                    return
                _, precondition_error = attempt_cleanup(bound_precondition)
                if precondition_error is not None:
                    self._command_dispatch_precondition_failure.set(precondition_error)
                    raise precondition_error

            validate_bound_precondition()

            # Exclude lifecycle stops from the physical command commit.
            self._command_dispatch_attempted = True
            if bound_precondition is None:
                self._action_mapping.apply(action)
            else:
                self._action_mapping.apply(
                    action, before_commit=validate_bound_precondition
                )
            effective_action = self._action_mapping.effective_action(action)
            flat_effective_action = self._flatten_structured_action(effective_action)
            self._internal_observation.set_prev_action(flat_effective_action)

        # Deadlines mark post-action read starts; scheduled deadlines advance by
        # ACTION_RATE. Re-anchor subtracts extraction time; subtracting each tick
        # speeds the controller.
        if self.LIMIT_RATE:
            period: float = self.ACTION_RATE
            lateness = pre_action_time - self._next_step_deadline
            if self._next_step_deadline <= 0.0 or lateness > period:
                if self._next_step_deadline > 0.0:
                    logging.warning(
                        f"Action rate of {1.0 / self.ACTION_RATE:.1f} Hz cannot "
                        f"be maintained (command was {lateness:.3f} s late); "
                        "re-anchoring the control period"
                    )
                # Re-anchor after start/reset or lateness over one period; do
                # not burst catch-up commands.
                self._next_step_deadline = (
                    pre_action_time + period - self._extract_duration_ema
                )
            sleep_remaining: float = self._next_step_deadline - self._clock()
            if sleep_remaining > self._SPIN_WAIT_THRESHOLD + self._MIN_SLEEP_DURATION:
                self._sleep(sleep_remaining - self._SPIN_WAIT_THRESHOLD)
            if sleep_remaining > 0.0:
                while self._clock() < self._next_step_deadline:
                    pass
            self._next_step_deadline += period

        return self._extract_step_result(flat_effective_action)

    def _ensure_session_active(self) -> None:
        abort_reason = getattr(self, "_session_abort_reason", None)
        if abort_reason is not None:
            raise OperatorAbort("real hardware session is aborted: " + abort_reason)
        if getattr(self, "_closed", False) or getattr(self, "_close_requested", False):
            raise OperatorAbort("real hardware session is closed")
        self._raise_if_ros_runtime_failed()

    def _raise_if_ros_runtime_failed(self) -> None:
        """Abort when the owned ROS executor stopped servicing callbacks.

        Sensor and command callbacks run on that executor; after its spin
        thread fails, every callback-fed input is frozen. The failure latches
        the session. Inside a dispatch transaction or an active stop/cleanup,
        only the reason is latched here, and the enclosing operation performs
        the safe stop once that transaction has returned.
        """

        raise_if_failed = getattr(
            getattr(self, "_ros_runtime", None), "raise_if_failed", None
        )
        if not callable(raise_if_failed):
            return
        try:
            raise_if_failed()
        except RosExecutorCleanupError as failure:
            reason = f"ROS executor failed: {failure}"
            abort = OperatorAbort("real hardware session is aborted: " + reason)
            abort.__cause__ = failure
            if (
                self._dispatch_is_reentrant()
                or getattr(self, "_stop_owner", None) is not None
                or getattr(self, "_close_owner", None) is not None
            ):
                if getattr(self, "_session_abort_reason", None) is None:
                    self._session_abort_reason = reason
            else:
                self._latch_session_failure(reason, initiating=abort)
            raise abort from failure

    def _latch_session_failure(
        self,
        reason: str,
        *,
        initiating: BaseException | None = None,
    ) -> None:
        if getattr(self, "_session_abort_reason", None) is not None and getattr(
            self, "_session_abort_cleanup_started", False
        ):
            return
        _, cleanup_error = attempt_cleanup(lambda: self.abort_session(reason))
        if cleanup_error is not None:
            if initiating is not None and cleanup_error is not initiating:
                _attach_cleanup_note(
                    initiating,
                    cleanup_error,
                    context="Real-session safe stop failed while latching",
                )
            else:
                logging.error(
                    "Real-session safe stop failed while latching "
                    f"{reason!r}: {type(cleanup_error).__name__}: {cleanup_error}"
                )

    def abort_session(self, reason: str) -> None:
        """Latch one session-wide abort and safe-stop every action interface."""

        def validate_abort_entry() -> None:
            self._reject_reentrant_lifecycle("abort")
            self._reject_stop_conflict("abort")
            self._reject_close_conflict("abort")

        normalized_reason = str(reason).strip()
        if not normalized_reason:
            normalized_reason = "unspecified safety abort"
        _, entry_error = attempt_cleanup(validate_abort_entry)
        if isinstance(entry_error, Exception):
            raise entry_error
        errors: list[BaseException] = [entry_error] if entry_error is not None else []
        if entry_error is not None:
            with self._dispatch_scope():
                if getattr(self, "_session_abort_reason", None) is None:
                    self._session_abort_reason = normalized_reason
            _, guard_error = attempt_cleanup(validate_abort_entry)
            if guard_error is not None:
                errors.append(guard_error)
                _raise_cleanup_failures(
                    errors,
                    context="real-session safe stop deferred by lifecycle guard",
                )
        with self._dispatch_scope():
            if getattr(self, "_session_abort_reason", None) is None:
                self._session_abort_reason = normalized_reason
            self._claim_stop()
            self._session_abort_cleanup_started = True

        try:
            for hw in getattr(self, "_sink_action", ()):

                def stop_if_needed(hw: HardwareInterface = hw) -> None:
                    if not hw.is_started or hw.is_closed or hw.safe_stop_issued:
                        return
                    hw.safe_stop()

                _, stop_error = attempt_cleanup(stop_if_needed)
                if stop_error is not None:
                    errors.append(stop_error)
        finally:
            with self._dispatch_scope():
                self._release_stop()
        _raise_cleanup_failures(errors, context="real-session safe stop failed")

    def hold_motion(self, reason: str) -> None:
        """Pause every action sink without latching an ordinary session abort.

        Used between episodes when a rollout ends without an environment
        ``done`` (harness timeout, policy fault, evidence failure). Each sink's
        ``pause()`` is its physical zero command; ``reset()`` resumes the sinks
        for the next episode. A latched safe stop is never resumed. A
        cancellation during the hold is a session failure and is latched.
        """

        try:
            self._reject_reentrant_lifecycle("motion hold")
            self._reject_stop_conflict("motion hold")
            self._reject_close_conflict("motion hold")
            with self._dispatch_scope():
                self._ensure_session_active()
                errors: list[BaseException] = []
                for hw in getattr(self, "_sink_action", ()):

                    def pause_if_needed(hw: HardwareInterface = hw) -> None:
                        if not hw.is_started or hw.is_closed or hw.safe_stop_issued:
                            return
                        if hw.is_paused:
                            return
                        hw.pause()

                    _, pause_error = attempt_cleanup(pause_if_needed)
                    if pause_error is not None:
                        errors.append(pause_error)
                _, log_error = attempt_cleanup(
                    lambda: logging.info(f"Real environment motion held: {reason}")
                )
                if log_error is not None:
                    errors.append(log_error)
            _raise_cleanup_failures(errors, context="real-session motion hold failed")
        except BaseException as exc:
            if not isinstance(exc, Exception):
                self._latch_session_failure(
                    f"motion hold cancellation: {type(exc).__name__}: {exc}",
                    initiating=exc,
                )
            elif isinstance(exc, OperatorAbort) and isinstance(
                exc.__cause__, RosExecutorCleanupError
            ):
                # Stop safely after leaving the dispatch guard.
                self._latch_session_failure(str(exc), initiating=exc)
            raise

    def _reset_readiness_pending(self) -> Any | None:
        """Return the first participant still awaiting its first sample.

        A participant that reports any other non-healthy state, or whose health
        check fails, aborts reset immediately exactly as before the readiness
        wait existed; only first-sample states are waited on.
        """

        pending: tuple[HardwareInterface, Any] | None = None
        for hw in self._hardware:
            try:
                event = normalize_health_event(hw.health(), component=hw.name)
            except (
                AttributeError,
                ExceptionGroup,
                KeyError,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                raise OperatorAbort(
                    f"hardware interface '{hw.name}' health check failed during "
                    f"reset: {type(exc).__name__}: {exc}"
                ) from exc
            if event is None or event.status is HealthStatus.HEALTHY:
                continue
            if self._is_reset_wait_state(event):
                if pending is None:
                    pending = (hw, event)
                continue
            raise self._reset_health_abort(hw, event)
        return pending

    def _is_reset_wait_state(self, event: Any) -> bool:
        state = event.details.get("state") if event.details else None
        if not isinstance(state, str):
            state = event.code.rsplit(".", maxsplit=1)[-1]
        return state.casefold() in self._RESET_READY_WAIT_STATES

    @staticmethod
    def _reset_health_abort(
        hw: HardwareInterface,
        event: Any,
        *,
        suffix: str = "",
    ) -> RealSessionAbort:
        if "stale" in event.code.casefold():
            return SensorStaleAbort(
                f"hardware interface '{hw.name}' is not fresh during reset: "
                f"{event.code}: {event.message}{suffix}"
            )
        return OperatorAbort(
            f"hardware interface '{hw.name}' is not ready during reset: "
            f"{event.code}: {event.message}{suffix}"
        )

    def _validate_reset_frame(self) -> None:
        """Validate the evaluator's initial post-reset accepted frame."""

        if self._task_evaluator is None:
            return
        frame = self._accepted_task_frame(phase="after reset")
        try:
            self._task_evaluator.validate_frame(frame)
        except RealSessionAbort:
            raise
        except (
            AttributeError,
            ExceptionGroup,
            KeyError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            raise SensorIncompleteAbort(
                "task evaluator rejected the initial post-reset frame: "
                f"{type(exc).__name__}: {exc}"
            ) from exc

    def _consume_pause_requests(self) -> bool:
        """Read every pause source; a pending pause keeps the sinks held."""

        requested = False
        for hw in self._src_pause:
            if hw.pause_signal:
                logging.info(f"Pause signal pending from {hw.name} during reset")
                requested = True
        if requested:
            # Mirror step(): a pause supersedes resume signals seen with it.
            for hw in self._src_resume:
                _discard = hw.resume_signal
        return requested

    def reset(self, **kwargs) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        def validate_reset_entry() -> None:
            self._reject_reentrant_lifecycle("reset")
            self._ensure_session_active()

        _, entry_error = attempt_cleanup(validate_reset_entry)
        if entry_error is not None:
            if isinstance(entry_error, Exception):
                raise entry_error
            self._latch_session_failure(
                f"reset entry cancellation: {type(entry_error).__name__}: "
                f"{entry_error}",
                initiating=entry_error,
            )
            raise entry_error
        try:
            super().reset(**kwargs)

            # Hold before changing reset state; never resume a latched safe stop.
            self.hold_motion("episode reset")

            ready_wait_started = self._clock()
            first_poll = True
            while True:
                with self._dispatch_scope():
                    self._ensure_session_active()
                    if first_poll:
                        first_poll = False
                        for hw in self._hardware:
                            hw.reset()
                        self._internal_observation.reset()
                        if self._task_evaluator is not None:
                            self._task_evaluator.reset()

                    # Drain fresh sensor messages while sinks stay held. The
                    # guard excludes teardown and is released between polls.
                    for hw in self._hardware:
                        hw.sync()
                    pending = self._reset_readiness_pending()
                    if pending is None:
                        return self._commit_reset()

                elapsed = self._clock() - ready_wait_started
                if elapsed >= self._reset_ready_timeout_s:
                    raise self._reset_health_abort(
                        *pending,
                        suffix=(
                            " (no ready sample within the "
                            f"{self._reset_ready_timeout_s:.3f} s reset "
                            "readiness timeout)"
                        ),
                    )
                self._sleep(
                    min(
                        self._RESET_READY_POLL_S,
                        self._reset_ready_timeout_s - elapsed,
                    )
                )
        except BaseException as reset_error:
            self._latch_session_failure(
                f"reset failed: {type(reset_error).__name__}: {reset_error}",
                initiating=reset_error,
            )
            raise

    def _commit_reset(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Finish reset under the dispatch guard once every participant is ready."""

        self._validate_reset_frame()

        observation = self._build_observation_structure()
        info: dict[str, Any] = {hw.name: hw.info for hw in self._hardware}

        # Publish episode state only after readiness; resume is the final commit.
        self._episode_steps = 0
        self._horizon_reached = False
        self._last_step_time = 0.0
        self._step_freq_ema = 0.0
        self._perf_log_last_time = 0.0
        self._perf_log_last_step = 0
        self._step_count = 0
        self._next_step_deadline = 0.0
        self._ensure_session_active()
        if self._consume_pause_requests():
            # Preserve acknowledged pauses; step() waits for explicit resume.
            self._is_running = False
            return observation, info
        for hw in getattr(self, "_sink_action", ()):
            if hw.is_paused and hw.is_started and not hw.safe_stop_issued:
                hw.resume()
        self._is_running = self.IS_RUNNING_AT_START
        return observation, info

    def close(self):
        """Request close and retry only resources whose cleanup is unfinished."""

        # Materialize close state lazily for partial construction and __del__.
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = RLock()
            self._close_lock = close_lock
        for name, default in (
            ("_close_completed", False),
            ("_close_requested", False),
            ("_close_safe_stop_done", set()),
            ("_close_hardware_done", set()),
            ("_close_ros_done", False),
            ("_close_super_done", False),
            ("_close_stop_attempts", {}),
            ("_close_hardware_attempts", {}),
            ("_close_ros_attempts", 0),
            ("_close_stop_unconfirmed", set()),
        ):
            if not hasattr(self, name):
                setattr(self, name, default)

        # Publish the barrier before waiting for the lifecycle lock so a
        # concurrent step cannot enter.
        self._close_requested = True
        self._closed = True
        errors: list[BaseException] = []

        def validate_close_entry() -> None:
            if self._dispatch_is_reentrant():
                if getattr(self, "_session_abort_reason", None) is None:
                    self._session_abort_reason = "environment closed"
                self._reject_reentrant_lifecycle("close")
            self._reject_stop_conflict("close")
            self._reject_close_conflict("close")

        _, entry_error = attempt_cleanup(validate_close_entry)
        if isinstance(entry_error, Exception):
            raise entry_error
        if entry_error is not None:
            errors.append(entry_error)
            with self._dispatch_scope():
                if getattr(self, "_session_abort_reason", None) is None:
                    self._session_abort_reason = "environment closed"
            _, guard_error = attempt_cleanup(validate_close_entry)
            if guard_error is not None:
                errors.append(guard_error)
                _raise_cleanup_failures(
                    errors,
                    context="real environment cleanup deferred by lifecycle guard",
                )
        with self._close_scope(close_lock):
            if self._close_completed:
                _raise_cleanup_failures(
                    errors,
                    context="real environment cleanup failed",
                )
                return
            self._close_requested = True
            self._closed = True
            hardware = tuple(getattr(self, "_hardware", ()))
            sink_ids = {id(hw) for hw in getattr(self, "_sink_action", ())}

            # Stop sinks first and retain handles after failed stops. Claim the
            # phase under the guard, then call hooks outside it to avoid
            # deadlocking callbacks or deferred close requests.
            stop_candidates: list[tuple[int, HardwareInterface]] = []
            with self._dispatch_scope():
                if getattr(self, "_session_abort_reason", None) is None:
                    self._session_abort_reason = "environment closed"
                for index, hw in enumerate(hardware):
                    if id(hw) not in sink_ids or index in self._close_safe_stop_done:
                        continue
                    if not hw.is_started or hw.is_closed or hw.safe_stop_issued:
                        self._close_safe_stop_done.add(index)
                        continue
                    if index in self._close_stop_unconfirmed:
                        # Exhausted sinks retain their handles without retries.
                        continue
                    stop_candidates.append((index, hw))
                if stop_candidates:
                    self._claim_stop()

            stop_successes: set[int] = set()
            try:
                for index, hw in stop_candidates:
                    attempts = self._record_cleanup_attempt(
                        self._close_stop_attempts, index
                    )
                    _, stop_error = attempt_cleanup(hw.safe_stop)
                    if stop_error is None:
                        stop_successes.add(index)
                    else:
                        errors.append(stop_error)
                        if attempts >= self._MAX_CLOSE_STOP_ATTEMPTS:
                            self._close_stop_unconfirmed.add(index)
            finally:
                with self._dispatch_scope():
                    self._close_safe_stop_done.update(stop_successes)
                    if stop_candidates:
                        self._release_stop()

            # Close in reverse startup order; retain sinks whose stop failed.
            for index in range(len(hardware) - 1, -1, -1):
                if index in self._close_hardware_done:
                    continue
                hw = hardware[index]
                if id(hw) in sink_ids and index not in self._close_safe_stop_done:
                    if hw.safe_stop_issued:
                        self._close_safe_stop_done.add(index)
                    else:
                        continue
                self._record_cleanup_attempt(self._close_hardware_attempts, index)
                _, close_error = attempt_cleanup(hw.close)
                if close_error is None:
                    self._close_hardware_done.add(index)
                else:
                    errors.append(close_error)

            if not self._close_super_done:
                _, super_error = attempt_cleanup(super().close)
                if super_error is None:
                    self._close_super_done = True
                else:
                    errors.append(super_error)

            # Retain ROS while unfinished real hardware may need it for retry.
            # Partial-construction probes have no transport dependency.
            all_hardware_done = len(self._close_hardware_done) == len(hardware)
            has_real_hardware = any(
                isinstance(hw, HardwareInterface) for hw in hardware
            )
            if (
                (all_hardware_done or not has_real_hardware)
                and not self._close_ros_done
                and hasattr(self, "_ros_runtime")
            ):
                self._close_ros_attempts += 1
                _, ros_error = attempt_cleanup(self._ros_runtime.close)
                if ros_error is None:
                    self._close_ros_done = True
                else:
                    errors.append(ros_error)
            elif not hasattr(self, "_ros_runtime"):
                self._close_ros_done = True

            with close_lock:
                self._close_completed = (
                    all_hardware_done
                    and self._close_super_done
                    and self._close_ros_done
                    and not errors
                    and not self._close_stop_unconfirmed
                )
            if self._close_stop_unconfirmed and all(
                isinstance(error, Exception) for error in errors
            ):
                # Preserve cancellation precedence; otherwise report typed
                # incomplete cleanup with its retry owner.
                unconfirmed = ", ".join(
                    repr(getattr(hardware[index], "name", index))
                    for index in sorted(self._close_stop_unconfirmed)
                )
                raise RealCleanupIncomplete(
                    "real environment cleanup incomplete: safe stop unconfirmed "
                    f"for {unconfirmed} after {self._MAX_CLOSE_STOP_ATTEMPTS} "
                    "attempts; the transport handle is retained and no physical "
                    "stop is claimed",
                    errors=tuple(errors),
                    cleanup_owner=self,
                    cleanup_state=self.cleanup_state,
                )
            _raise_cleanup_failures(errors, context="real environment cleanup failed")

    @property
    def cleanup_state(self) -> RealCleanupState:
        """Read-only per-resource close progress (bounded, partial-safe)."""

        hardware = tuple(getattr(self, "_hardware", ()))
        sink_ids = {id(hw) for hw in getattr(self, "_sink_action", ())}
        stop_done = frozenset(getattr(self, "_close_safe_stop_done", ()))
        hardware_done = frozenset(getattr(self, "_close_hardware_done", ()))
        stop_attempts = dict(getattr(self, "_close_stop_attempts", {}))
        close_attempts = dict(getattr(self, "_close_hardware_attempts", {}))
        unconfirmed = frozenset(getattr(self, "_close_stop_unconfirmed", ()))
        resources = tuple(
            RealResourceCleanupState(
                index=index,
                name=str(getattr(hw, "name", index)),
                action_sink=id(hw) in sink_ids,
                stop_done=id(hw) not in sink_ids or index in stop_done,
                stop_attempts=stop_attempts.get(index, 0),
                stop_unconfirmed=index in unconfirmed,
                close_done=index in hardware_done,
                close_attempts=close_attempts.get(index, 0),
            )
            for index, hw in enumerate(hardware)
        )
        return RealCleanupState(
            close_requested=bool(getattr(self, "_close_requested", False)),
            completed=bool(getattr(self, "_close_completed", False)),
            resources=resources,
            ros_done=bool(getattr(self, "_close_ros_done", False)),
            ros_attempts=int(getattr(self, "_close_ros_attempts", 0)),
            max_stop_attempts=self._MAX_CLOSE_STOP_ATTEMPTS,
        )

    def seed(self, seed: int | None = None) -> list[int | None]:
        return [seed]

    def __del__(self):
        # Explicit close owns retryable cleanup; the finalizer only logs.
        _, close_error = attempt_cleanup(self.close)
        if close_error is not None:
            attempt_cleanup(
                lambda: logging.error(
                    "real environment cleanup failed during finalization: "
                    f"{type(close_error).__name__}: {close_error}; "
                    "call close() explicitly to retry"
                )
            )

    @property
    def single_action_space(self) -> gymnasium.Space:
        return self._single_action_space

    @property
    def command_mapping(self) -> CommandMapping | None:
        """Validated immutable command metadata for direct input adapters."""
        return self._command_mapping

    @property
    def action_space(self) -> gymnasium.Space:
        return self._action_space

    @property
    def single_observation_space(self) -> gymnasium.Space:
        return self._single_observation_space

    @property
    def observation_space(self) -> gymnasium.Space:
        return self._observation_space

    @property
    def hardware_interfaces(self) -> Sequence[HardwareInterface]:
        return self._hardware

    def _build_observation_structure(self) -> dict[str, Any]:
        return RealEnv._flatten_observations(
            self._observation_mapping.collect(),
            layout=self._observation_layout,
        )

    def _flatten_structured_action(
        self,
        action: Mapping[str, np.ndarray],
    ) -> np.ndarray:
        flat = np.zeros(self._action_layout.single_space.shape, dtype=np.float32)
        with np.errstate(over="ignore", invalid="ignore"):
            for term in self._action_layout.terms:
                flat[term.action_slice] = action[term.key].reshape(-1)
        if not np.all(np.isfinite(flat)):
            raise ValueError("Emitted action is not finite in float32 telemetry")
        return flat

    def _unbatch_compat_action(
        self,
        action: np.ndarray | torch.Tensor | dict[str, np.ndarray | torch.Tensor],
    ) -> np.ndarray | torch.Tensor | dict[str, np.ndarray | torch.Tensor]:
        """Remove the single leading row of the batched compatibility space."""

        if isinstance(action, Mapping):
            return action
        single_shape = tuple(self._single_action_space.shape or ())
        if isinstance(action, torch.Tensor):
            if tuple(action.shape) == (1, *single_shape):
                return action[0]
            return action
        if isinstance(action, np.ndarray) and action.shape == (1, *single_shape):
            return action[0]
        return action

    def _validate_flat_action_domain(self, action: object) -> None:
        candidate = action
        if isinstance(candidate, torch.Tensor):
            candidate = candidate.detach().cpu().numpy()
        if not self._single_action_space.contains(candidate):
            raise ValueError("flat action is outside the declared SINGLE_ACTION_SPACE")

    @staticmethod
    def _resolve_hardware_interface(
        hardware: HardwareInterface | str,
    ) -> HardwareInterface:
        if not isinstance(hardware, str):
            return hardware
        hardware_cls = HardwareInterfaceRegistry.get_by_name(hardware)
        if hardware_cls is None:
            raise ValueError(f"Hardware interface '{hardware}' is not registered")
        return hardware_cls()

    @staticmethod
    def _flatten_observations(
        obs_dict: dict[str, dict[str, np.ndarray]],
        *,
        layout: ObservationSpaceLayout | None = None,
    ) -> dict[str, np.ndarray]:
        if layout is not None:
            return layout.flatten(obs_dict)
        return {
            obs_cat: obs_group
            if isinstance(obs_group, np.ndarray)
            else np.concatenate(
                [
                    obs_group[obs_key].reshape((-1,))
                    for obs_key in sorted(obs_group.keys())
                ],
                axis=0,
            )
            for obs_cat, obs_group in obs_dict.items()
        }

    @cached_property
    def device(self) -> torch.device:
        return torch.device("cpu")

    @property
    def num_envs(self) -> int:
        return 1
