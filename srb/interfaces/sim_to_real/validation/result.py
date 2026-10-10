"""Session result schema and exit policy for validation runs."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from numbers import Real
from types import MappingProxyType

from srb.interfaces.sim_to_real.validation.criteria import CriterionResult
from srb.interfaces.sim_to_real.validation.exit_codes import VALIDATION_EXIT_CODES
from srb.utils.json_types import ImmutableValueMixin

_EXIT_CODE_BY_CRITERION: dict[str, int] = {
    str(exit_code.criterion): int(exit_code.code) for exit_code in VALIDATION_EXIT_CODES
}
_VALID_EXIT_CODES: frozenset[int] = frozenset(
    int(exit_code.code) for exit_code in VALIDATION_EXIT_CODES
)


@dataclass(frozen=True, slots=True)
class SessionResult(ImmutableValueMixin):
    pass_overall: bool
    criterion_a: CriterionResult
    criterion_b: CriterionResult
    criterion_c: CriterionResult
    metrics: Mapping[str, float]
    sim_metrics: Mapping[str, float] | None
    sim_real_ratio: Mapping[str, float] | None
    drift_z: Mapping[str, float] | None
    exit_code: int
    schema_version: int = 1
    hardware_evidence: bool = True

    def __post_init__(self) -> None:
        _validate_bool(self.pass_overall, field_name="pass_overall")
        criterion_a = _snapshot_criterion(self.criterion_a, field_name="criterion_a")
        criterion_b = _snapshot_criterion(self.criterion_b, field_name="criterion_b")
        criterion_c = _snapshot_criterion(self.criterion_c, field_name="criterion_c")
        if criterion_a.optional or criterion_b.optional:
            raise ValueError("only criterion_c may be explicitly optional")
        if criterion_c.optional and criterion_c.status != "skipped":
            raise ValueError("criterion_c.optional requires a skipped criterion")
        if not isinstance(self.exit_code, int) or isinstance(self.exit_code, bool):
            raise TypeError("session result exit_code must be an integer")
        if self.exit_code not in _VALID_EXIT_CODES:
            raise ValueError(
                f"session result exit_code is unsupported: {self.exit_code}"
            )
        if not isinstance(self.schema_version, int) or isinstance(
            self.schema_version, bool
        ):
            raise TypeError("session result schema_version must be an integer")
        if self.schema_version < 1:
            raise ValueError("session result schema_version must be positive")
        _validate_bool(self.hardware_evidence, field_name="hardware_evidence")
        if _mapping_is_degraded(self.sim_metrics) and self.hardware_evidence:
            raise ValueError(
                "degraded sim baseline evidence cannot be hardware evidence"
            )
        expected_exit = _expected_exit_code(criterion_a, criterion_b, criterion_c)
        if self.hardware_evidence:
            if self.exit_code != expected_exit:
                if (
                    criterion_c.status == "skipped"
                    and not criterion_c.optional
                    and self.exit_code == 0
                ):
                    raise ValueError("criterion_c is configured or invalid but skipped")
                raise ValueError(
                    "session result exit_code contradicts criterion states: "
                    f"expected {expected_exit}, got {self.exit_code}"
                )
        elif self.exit_code == 0:
            raise ValueError("hardware_evidence=False cannot use exit code 0")
        expected_pass = (
            self.hardware_evidence
            and self.exit_code == 0
            and _criteria_are_passable(criterion_a, criterion_b, criterion_c)
        )
        if self.pass_overall != expected_pass:
            raise ValueError(
                "session result pass_overall contradicts criterion/evidence state"
            )
        object.__setattr__(self, "criterion_a", criterion_a)
        object.__setattr__(self, "criterion_b", criterion_b)
        object.__setattr__(self, "criterion_c", criterion_c)
        object.__setattr__(self, "metrics", _freeze_metrics(self.metrics))
        object.__setattr__(
            self,
            "sim_metrics",
            _freeze_optional_metrics(self.sim_metrics),
        )
        object.__setattr__(
            self,
            "sim_real_ratio",
            _freeze_optional_metrics(self.sim_real_ratio),
        )
        object.__setattr__(self, "drift_z", _freeze_optional_metrics(self.drift_z))


def build_session_result(
    *,
    metrics: Mapping[str, float | int],
    sim_metrics: Mapping[str, float | int] | None,
    sim_real_ratio: Mapping[str, float | int] | None,
    drift_z: Mapping[str, float | int] | None,
    criterion_a: CriterionResult,
    criterion_b: CriterionResult,
    criterion_c: CriterionResult,
) -> SessionResult:
    """Build the validation result and preserve historical exit priority."""

    exit_code = _expected_exit_code(criterion_a, criterion_b, criterion_c)
    degraded = _mapping_is_degraded(sim_metrics)
    hardware_evidence = not degraded
    pass_overall = (
        hardware_evidence
        and exit_code == 0
        and _criteria_are_passable(criterion_a, criterion_b, criterion_c)
    )
    return SessionResult(
        pass_overall=pass_overall,
        criterion_a=criterion_a,
        criterion_b=criterion_b,
        criterion_c=criterion_c,
        metrics=metrics,
        sim_metrics=sim_metrics,
        sim_real_ratio=sim_real_ratio,
        drift_z=drift_z,
        exit_code=exit_code,
        hardware_evidence=hardware_evidence,
    )


def _expected_exit_code(
    criterion_a: CriterionResult,
    criterion_b: CriterionResult,
    criterion_c: CriterionResult,
) -> int:
    """Return the stable priority code for fail or unresolved criteria."""

    if criterion_b.status != "pass":
        return _EXIT_CODE_BY_CRITERION["criterion_b"]
    if criterion_a.status != "pass":
        return _EXIT_CODE_BY_CRITERION["criterion_a"]
    if not _criterion_c_is_passable(criterion_c):
        return _EXIT_CODE_BY_CRITERION["criterion_c"]
    return _EXIT_CODE_BY_CRITERION["all criteria pass"]


def _criterion_c_is_passable(criterion: CriterionResult) -> bool:
    status = str(criterion.status)
    optional = bool(criterion.optional)
    return status == "pass" or (status == "skipped" and optional)


def _criteria_are_passable(
    criterion_a: CriterionResult,
    criterion_b: CriterionResult,
    criterion_c: CriterionResult,
) -> bool:
    criterion_a_status = str(criterion_a.status)
    criterion_b_status = str(criterion_b.status)
    return (
        criterion_a_status == "pass"
        and criterion_b_status == "pass"
        and _criterion_c_is_passable(criterion_c)
    )


def _mapping_is_degraded(value: object) -> bool:
    degraded = getattr(value, "degraded", False)
    if type(degraded) is not bool:
        raise ValueError("sim baseline degraded flag must be a boolean")
    return degraded


def _freeze_metrics(metrics: Mapping[str, object]) -> Mapping[str, float]:
    if not isinstance(metrics, Mapping):
        raise TypeError("session result metrics must be a mapping")
    frozen: dict[str, float] = {}
    for key, value in metrics.items():
        if not isinstance(key, str):
            raise TypeError("session result metric keys must be strings")
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"session result metric {key!r} must be a finite number")
        try:
            numeric = float(value)
        except (OverflowError, ValueError) as exc:
            raise ValueError(
                f"session result metric {key!r} must be a finite number"
            ) from exc
        if not isfinite(numeric):
            raise ValueError(f"session result metric {key!r} must be a finite number")
        frozen[key] = numeric
    return MappingProxyType(frozen)


def _freeze_optional_metrics(
    metrics: Mapping[str, object] | None,
) -> Mapping[str, float] | None:
    return None if metrics is None else _freeze_metrics(metrics)


def _validate_bool(value: object, *, field_name: str) -> None:
    if not isinstance(value, bool):
        raise TypeError(f"session result {field_name} must be a boolean")


def _snapshot_criterion(value: object, *, field_name: str) -> CriterionResult:
    if not isinstance(value, CriterionResult):
        raise TypeError(f"session result {field_name} must be a CriterionResult")
    return CriterionResult(
        status=value.status,
        detail=value.detail,
        breakdown=value.breakdown,
        optional=value.optional,
        score=value.score,
    )
