"""Composable reward-term models and kernels."""

from functools import cached_property
from typing import Annotated, Any, Callable, ClassVar, Literal

import torch
from pydantic import BaseModel, ConfigDict, Field, NonNegativeFloat, PositiveFloat

_OP_FN_MAP: dict[str, Callable[[torch.Tensor, Any], torch.Tensor]] = {
    ">=": torch.ge,
    ">": torch.gt,
    "<": torch.lt,
    "<=": torch.le,
}
_SIGN_MAP: dict[str, float] = {
    ">": 1.0,
    "<": -1.0,
}


def _log_cosh_stable(x: torch.Tensor) -> torch.Tensor:
    """Stable ``log(cosh(x))`` for nonnegative ``x``."""
    return x + torch.nn.functional.softplus(-2.0 * x) - 0.69314718  # log(2)


class RewardTerm(BaseModel):
    model_config: ConfigDict = ConfigDict(
        ignored_types=(cached_property,), extra="forbid"
    )

    weight: float
    """Scaling factor"""

    reduces_last_dim: ClassVar[bool] = False
    """Whether ``term()`` reduces the trailing input dimension (e.g. LogSumExp)"""

    def __call__(self, value: torch.Tensor) -> torch.Tensor:
        if not self.enabled:
            # Preserve the enabled path's shape for reducing terms.
            if self.reduces_last_dim:
                return value.new_zeros(value.shape[:-1])
            return torch.zeros_like(value)
        return self.term(value, self.weight, *self.term_args)

    @cached_property
    def enabled(self) -> bool:
        return self.weight != 0.0

    @cached_property
    def reward_limit(self) -> float:
        """Max theoretical signed contribution for normalization bounds."""
        return self.weight

    @cached_property
    def reward_bounds(self) -> tuple[float, float]:
        """Signed ``(worst, best)`` bounds; sign-changing terms override this."""
        rl = self.reward_limit
        return (min(rl, 0.0), max(rl, 0.0))

    @cached_property
    def term_args(self) -> tuple[Any, ...]:
        """Arguments passed to the static ``term()`` method."""
        return ()

    @staticmethod
    def term(value: torch.Tensor, weight: float, *args) -> torch.Tensor:
        raise NotImplementedError()

    def reset_cache(self):
        """Clear cached properties and recompute them."""
        for key in ("enabled", "reward_limit", "reward_bounds", "term_args"):
            self.__dict__.pop(key, None)
            _ = getattr(self, key)


class ConstantRewardTerm(RewardTerm):
    """Fixed reward per step."""

    @staticmethod
    def term(value: torch.Tensor, weight: float) -> torch.Tensor:
        return torch.full_like(value, weight)


class ConditionalRewardTerm(RewardTerm):
    """Binary condition."""

    @staticmethod
    def term(condition: torch.Tensor, weight: float) -> torch.Tensor:
        return condition.float() * weight


class ThresholdRewardTerm(RewardTerm):
    """Sharp threshold."""

    threshold: float
    """Comparison cutoff"""
    operator: Literal[">=", ">", "<", "<="]
    """Comparison operator"""

    @cached_property
    def term_args(self) -> tuple[float, Callable]:
        return (self.threshold, _OP_FN_MAP[self.operator])

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        threshold: float,
        op_fn: Callable[[torch.Tensor, Any], torch.Tensor],
    ) -> torch.Tensor:
        return op_fn(value, threshold).float() * weight


class SoftThresholdRewardTerm(RewardTerm):
    """Soft threshold using sigmoid."""

    threshold: float
    """Inflection point"""
    operator: Literal[">=", ">", "<", "<="]
    """Direction of the constraint"""
    sharpness: NonNegativeFloat
    """Transition sharpness"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.threshold, self.sharpness * _SIGN_MAP[self.operator[0]])

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        threshold: float,
        sharpness: float,
    ) -> torch.Tensor:
        return torch.sigmoid((value - threshold) * sharpness) * weight


class _ClampedRewardTerm(RewardTerm):
    limit: PositiveFloat
    """Maximum magnitude"""

    @cached_property
    def reward_limit(self) -> float:
        if not self.enabled:
            return 0.0
        return self.limit if self.weight > 0 else -self.limit

    @cached_property
    def term_args(self) -> tuple[float]:
        return (self.limit,)


class _SymmetricClampedRewardTerm(_ClampedRewardTerm):
    """Clamped term spanning ``[-limit, +limit]`` regardless of weight sign."""

    @cached_property
    def reward_bounds(self) -> tuple[float, float]:
        if not self.enabled:
            return (0.0, 0.0)
        return (-self.limit, self.limit)


class LinearRewardTerm(_ClampedRewardTerm):
    """Linear value scaling."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
    ) -> torch.Tensor:
        return torch.clamp(torch.abs(value) * weight, -limit, limit)


class ScaledRewardTerm(_SymmetricClampedRewardTerm):
    """Direct signed scaling for signed metrics with explicit bounds."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
    ) -> torch.Tensor:
        return torch.clamp(value * weight, -limit, limit)


class QuadraticRewardTerm(_ClampedRewardTerm):
    """Quadratic value scaling."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
    ) -> torch.Tensor:
        return torch.clamp(torch.square(value) * weight, -limit, limit)


class LogCoshRewardTerm(_ClampedRewardTerm):
    """Log-Hyperbolic-Cosine value scaling [Quadratic near 0 | Linear far from 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
    ) -> torch.Tensor:
        abs_value = torch.abs(value)
        return torch.clamp(
            _log_cosh_stable(abs_value) * weight,
            -limit,
            limit,
        )


class PowerRewardTerm(_ClampedRewardTerm):
    """Polynomial value scaling."""

    exponent: PositiveFloat
    """Polynomial degree"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.limit, self.exponent)

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        exponent: float,
    ) -> torch.Tensor:
        return torch.clamp(
            torch.pow(torch.abs(value), exponent) * weight, -limit, limit
        )


class InverseRewardTerm(_ClampedRewardTerm):
    """Inverse value scaling."""

    @staticmethod
    def term(value: torch.Tensor, weight: float, limit: float) -> torch.Tensor:
        return torch.clamp(weight / (torch.abs(value) + 1e-8), min=-limit, max=limit)


class InverseSquareRewardTerm(_ClampedRewardTerm):
    """Inverse square value scaling."""

    @staticmethod
    def term(value: torch.Tensor, weight: float, limit: float) -> torch.Tensor:
        return torch.clamp(weight / (torch.square(value) + 1e-8), min=-limit, max=limit)


class _OffsetRewardTerm(_ClampedRewardTerm):
    threshold: float
    """Sharp activation point"""
    direction: Literal[">", "<"]
    """Direction of the constraint"""

    @cached_property
    def term_args(self) -> tuple[float, float, float]:
        return (self.limit, self.threshold, _SIGN_MAP[self.direction])


class OffsetLinearRewardTerm(_OffsetRewardTerm):
    """Linear value scaling with sharp activation threshold."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return torch.clamp(
            torch.clamp_min((value - threshold) * sign, 0.0) * weight,
            min=-limit,
            max=limit,
        )


class OffsetQuadraticRewardTerm(_OffsetRewardTerm):
    """Quadratic value scaling with sharp activation threshold."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return torch.clamp(
            torch.square(torch.clamp_min((value - threshold) * sign, 0.0)) * weight,
            min=-limit,
            max=limit,
        )


class OffsetLogCoshRewardTerm(_OffsetRewardTerm):
    """Log-Hyperbolic-Cosine value scaling with sharp activation threshold [Quadratic
    near 0 | Linear far from 0].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        threshold = torch.clamp_min((value - threshold) * sign, 0.0)
        return torch.clamp(
            _log_cosh_stable(threshold) * weight,
            min=-limit,
            max=limit,
        )


class OffsetPowerRewardTerm(_OffsetRewardTerm):
    """Polynomial value scaling with sharp activation threshold."""

    exponent: PositiveFloat
    """Polynomial degree"""

    @cached_property
    def term_args(self) -> tuple[float, float, float, float]:
        return (
            self.limit,
            self.threshold,
            _SIGN_MAP[self.direction],
            self.exponent,
        )

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
        exponent: float,
    ) -> torch.Tensor:
        return torch.clamp(
            torch.pow(torch.clamp_min((value - threshold) * sign, 0.0), exponent)
            * weight,
            min=-limit,
            max=limit,
        )


class _KernelRewardTerm(RewardTerm):
    scale: PositiveFloat
    """Kernel width/spread"""

    @cached_property
    def term_args(self) -> tuple[float]:
        return (self.scale,)


class LaplacianRewardTerm(_KernelRewardTerm):
    """Laplacian error minimization [sharp around 0]."""

    @staticmethod
    def term(error: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return torch.exp(-torch.abs(error) / scale) * weight


class HyperbolicRewardTerm(_KernelRewardTerm):
    """Hyperbolic error minimization [linear around 0]."""

    @staticmethod
    def term(error: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (1.0 - torch.tanh(torch.abs(error) / scale)) * weight


class GaussianRewardTerm(_KernelRewardTerm):
    """Gaussian error minimization [flat around 0]."""

    @staticmethod
    def term(error: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return torch.exp(-torch.square(error) / (2.0 * scale * scale)) * weight


class LorentzianRewardTerm(_KernelRewardTerm):
    """Lorentzian error minimization [flat around 0 | fat tail]."""

    @staticmethod
    def term(error: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (1.0 / (1.0 + torch.square(error / scale))) * weight


class _BarrierRewardTerm(_KernelRewardTerm):
    threshold: float
    """Sharp activation point"""
    direction: Literal[">", "<"]
    """Direction of the constraint"""

    @cached_property
    def term_args(self) -> tuple[float, float, float]:
        return (self.scale, self.threshold, _SIGN_MAP[self.direction])


class LaplacianBarrierRewardTerm(_BarrierRewardTerm):
    """Laplacian error minimization with sharp activation threshold [sharp around
    threshold].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        scale: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return (
            torch.exp(-torch.clamp_min((value - threshold) * sign, 0.0) / scale)
            * weight
        )


class HyperbolicBarrierRewardTerm(_BarrierRewardTerm):
    """Hyperbolic error minimization with sharp activation threshold [linear around
    threshold].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        scale: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return (
            1.0 - torch.tanh(torch.clamp_min((value - threshold) * sign, 0.0) / scale)
        ) * weight


class GaussianBarrierRewardTerm(_BarrierRewardTerm):
    """Gaussian error minimization with sharp activation threshold [flat around
    threshold].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        scale: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return (
            torch.exp(
                -torch.square(torch.clamp_min((value - threshold) * sign, 0.0))
                / (2.0 * scale * scale)
            )
            * weight
        )


class LorentzianBarrierRewardTerm(_BarrierRewardTerm):
    """Lorentzian error minimization with sharp activation threshold [flat around
    threshold | fat tail].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        scale: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return (
            1.0
            / (
                1.0
                + torch.square(torch.clamp_min((value - threshold) * sign, 0.0) / scale)
            )
        ) * weight


class InverseBarrierRewardTerm(_OffsetRewardTerm):
    """Inverse scaling activates on the threshold side selected by ``direction``."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return torch.clamp(
            weight / (torch.clamp_min((threshold - value) * sign, 0.0) + 1e-8),
            min=-limit,
            max=limit,
        )


class InverseSquareBarrierRewardTerm(_OffsetRewardTerm):
    """Inverse square scaling activates on the side selected by ``direction``."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        limit: float,
        threshold: float,
        sign: float,
    ) -> torch.Tensor:
        return torch.clamp(
            weight
            / (torch.square(torch.clamp_min((threshold - value) * sign, 0.0)) + 1e-8),
            min=-limit,
            max=limit,
        )


class _AlignmentRewardTerm(RewardTerm):
    scale: PositiveFloat
    """Angular width/spread (radians)"""

    @cached_property
    def term_args(self) -> tuple[float]:
        return (self.scale,)


class LaplacianAlignmentRewardTerm(_AlignmentRewardTerm):
    """Laplacian angular alignment minimization [sharp around 0]."""

    @staticmethod
    def term(cos_sim: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (
            torch.exp(-torch.acos(torch.clamp(cos_sim, -0.9999999, 0.9999999)) / scale)
            * weight
        )


class HyperbolicAlignmentRewardTerm(_AlignmentRewardTerm):
    """Hyperbolic angular alignment minimization [linear around 0]."""

    @staticmethod
    def term(cos_sim: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (
            1.0
            - torch.tanh(
                torch.acos(torch.clamp(cos_sim, -0.9999999, 0.9999999)) / scale
            )
        ) * weight


class GaussianAlignmentRewardTerm(_AlignmentRewardTerm):
    """Gaussian angular alignment minimization [flat around 0]."""

    @staticmethod
    def term(cos_sim: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (
            torch.exp(
                -torch.square(torch.acos(torch.clamp(cos_sim, -0.9999999, 0.9999999)))
                / (2.0 * scale * scale)
            )
            * weight
        )


class LorentzianAlignmentRewardTerm(_AlignmentRewardTerm):
    """Lorentzian angular alignment minimization [flat around 0 | fat tail]."""

    @staticmethod
    def term(cos_sim: torch.Tensor, weight: float, scale: float) -> torch.Tensor:
        return (
            1.0
            / (
                1.0
                + torch.square(
                    torch.acos(torch.clamp(cos_sim, -0.9999999, 0.9999999)) / scale
                )
            )
        ) * weight


class _TrackingRewardTerm(_KernelRewardTerm):
    target: float = 0.0
    """Target value"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.target, self.scale)


class LaplacianTrackingRewardTerm(_TrackingRewardTerm):
    """Laplacian tracking error minimization [sharp around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        scale: float,
    ) -> torch.Tensor:
        return torch.exp(-torch.abs(value - target) / scale) * weight


class HyperbolicTrackingRewardTerm(_TrackingRewardTerm):
    """Hyperbolic tracking error minimization [linear around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        scale: float,
    ) -> torch.Tensor:
        return (1.0 - torch.tanh(torch.abs(value - target) / scale)) * weight


class GaussianTrackingRewardTerm(_TrackingRewardTerm):
    """Gaussian tracking error minimization [flat around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        scale: float,
    ) -> torch.Tensor:
        return torch.exp(-torch.square(value - target) / (2.0 * scale * scale)) * weight


class LorentzianTrackingRewardTerm(_TrackingRewardTerm):
    """Lorentzian tracking error minimization [flat around 0 | fat tail]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        scale: float,
    ) -> torch.Tensor:
        return (1.0 / (1.0 + torch.square((value - target) / scale))) * weight


class _ZoneTrackingRewardTerm(_TrackingRewardTerm):
    tolerance: NonNegativeFloat = 0.0
    """Deadband margin"""

    @cached_property
    def term_args(self) -> tuple[float, float, float]:
        return (self.target, self.tolerance, self.scale)


class LaplacianZoneTrackingRewardTerm(_ZoneTrackingRewardTerm):
    """Laplacian tracking error minimization with dead tolerance [sharp around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        tolerance: float,
        scale: float,
    ) -> torch.Tensor:
        return (
            torch.exp(
                -torch.clamp_min(torch.abs(value - target) - tolerance, 0.0) / scale
            )
            * weight
        )


class HyperbolicZoneTrackingRewardTerm(_ZoneTrackingRewardTerm):
    """Hyperbolic tracking error minimization with dead tolerance [linear around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        tolerance: float,
        scale: float,
    ) -> torch.Tensor:
        return (
            1.0
            - torch.tanh(
                torch.clamp_min(torch.abs(value - target) - tolerance, 0.0) / scale
            )
        ) * weight


class GaussianZoneTrackingRewardTerm(_ZoneTrackingRewardTerm):
    """Gaussian tracking error minimization with dead tolerance [flat around 0]."""

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        tolerance: float,
        scale: float,
    ) -> torch.Tensor:
        return (
            torch.exp(
                -torch.square(
                    torch.clamp_min(torch.abs(value - target) - tolerance, 0.0)
                )
                / (2.0 * scale * scale)
            )
            * weight
        )


class LorentzianZoneTrackingRewardTerm(_ZoneTrackingRewardTerm):
    """Lorentzian tracking error minimization with dead tolerance [flat around 0 | fat
    tail].
    """

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        target: float,
        tolerance: float,
        scale: float,
    ) -> torch.Tensor:
        return (
            1.0
            / (
                1.0
                + torch.square(
                    torch.clamp_min(torch.abs(value - target) - tolerance, 0.0) / scale
                )
            )
        ) * weight


class SmoothMaxRewardTerm(_SymmetricClampedRewardTerm):
    """Smooth maximum of multiple inputs.
    Handles signed inputs and may span ``[-limit, +limit]``."""

    temperature: PositiveFloat
    """Selection sharpness"""

    reduces_last_dim: ClassVar[bool] = True

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.limit, self.temperature)

    @staticmethod
    def term(
        values: torch.Tensor, weight: float, limit: float, temperature: float
    ) -> torch.Tensor:
        return torch.clamp(
            torch.logsumexp(values / temperature, dim=-1) * temperature * weight,
            -limit,
            limit,
        )


class SmoothMinRewardTerm(_SymmetricClampedRewardTerm):
    """Smooth minimum of multiple inputs.
    Handles signed inputs and may span ``[-limit, +limit]``."""

    temperature: PositiveFloat
    """Selection sharpness"""

    reduces_last_dim: ClassVar[bool] = True

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.limit, self.temperature)

    @staticmethod
    def term(
        values: torch.Tensor, weight: float, limit: float, temperature: float
    ) -> torch.Tensor:
        return torch.clamp(
            -torch.logsumexp(-values / temperature, dim=-1) * temperature * weight,
            -limit,
            limit,
        )


class _PeriodicRewardTerm(RewardTerm):
    frequency: PositiveFloat
    """Cycles per unit of time (Hz)"""

    @cached_property
    def term_args(self) -> tuple[float]:
        return (self.frequency,)


class _SymmetricPeriodicRewardTerm(_PeriodicRewardTerm):
    """Periodic terms oscillating across zero; bounds span ``[-abs(weight),
    abs(weight)]``. One-sided subclasses retain the inherited signed bounds.
    """

    @cached_property
    def reward_bounds(self) -> tuple[float, float]:
        if not self.enabled:
            return (0.0, 0.0)
        mag = abs(self.weight)
        return (-mag, mag)


class PeriodicSineRewardTerm(_SymmetricPeriodicRewardTerm):
    """Periodic sine wave modulation based on time."""

    @staticmethod
    def term(time: torch.Tensor, weight: float, frequency: float) -> torch.Tensor:
        return (
            torch.sin(
                6.2831853  # 2 * pi
                * frequency
                * time
            )
            * weight
        )


class PeriodicRectifiedSineRewardTerm(_PeriodicRewardTerm):
    """Periodic rectified sine wave modulation based on time."""

    @staticmethod
    def term(time: torch.Tensor, weight: float, frequency: float) -> torch.Tensor:
        return (
            torch.clamp_min(
                torch.sin(
                    6.2831853  # 2 * pi
                    * frequency
                    * time
                ),
                0.0,
            )
            * weight
        )


class PeriodicSquareRewardTerm(_SymmetricPeriodicRewardTerm):
    """Periodic square wave modulation based on time."""

    @staticmethod
    def term(time: torch.Tensor, weight: float, frequency: float) -> torch.Tensor:
        return (
            torch.sign(
                torch.sin(
                    6.2831853  # 2 * pi
                    * frequency
                    * time
                )
            )
            * weight
        )


class PeriodicSoftSquareRewardTerm(_SymmetricPeriodicRewardTerm):
    """Periodic soft square wave modulation based on time."""

    sharpness: PositiveFloat
    """Transition sharpness"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.frequency, self.sharpness)

    @staticmethod
    def term(
        time: torch.Tensor, weight: float, frequency: float, sharpness: float
    ) -> torch.Tensor:
        return (
            torch.tanh(
                sharpness
                * torch.sin(
                    6.2831853  # 2 * pi
                    * frequency
                    * time
                )
            )
            * weight
        )


class PeriodicTriangleRewardTerm(_SymmetricPeriodicRewardTerm):
    """Periodic triangle wave modulation based on time."""

    @staticmethod
    def term(time: torch.Tensor, weight: float, frequency: float) -> torch.Tensor:
        return (
            2.0
            * torch.abs(2.0 * (time * frequency - torch.floor(0.5 + time * frequency)))
            - 1.0
        ) * weight


class PeriodicSawtoothRewardTerm(_SymmetricPeriodicRewardTerm):
    """Periodic sawtooth wave modulation based on time."""

    @staticmethod
    def term(time: torch.Tensor, weight: float, frequency: float) -> torch.Tensor:
        return (2.0 * (time * frequency - torch.floor(0.5 + time * frequency))) * weight


class PeriodicPulseRewardTerm(_PeriodicRewardTerm):
    """Periodic pulse modulation based on time."""

    sharpness: PositiveFloat
    """Concentration of the pulse"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.frequency, self.sharpness)

    @staticmethod
    def term(
        time: torch.Tensor, weight: float, frequency: float, sharpness: float
    ) -> torch.Tensor:
        return (
            torch.exp(
                sharpness
                * (
                    torch.cos(
                        6.2831853  # 2 * pi
                        * frequency
                        * time
                    )
                    - 1.0
                )
            )
            * weight
        )


class PeriodicDutyCycleRewardTerm(_PeriodicRewardTerm):
    """Periodic duty cycle wave modulation based on time."""

    duty: Annotated[float, Field(ge=0.0, le=1.0)]
    """Duty cycle ratio [0.0, 1.0]"""

    @cached_property
    def term_args(self) -> tuple[float, float]:
        return (self.frequency, self.duty)

    @staticmethod
    def term(
        time: torch.Tensor, weight: float, frequency: float, duty: float
    ) -> torch.Tensor:
        return (
            torch.where(
                (time * frequency - torch.floor(time * frequency)) < duty, 1.0, 0.0
            )
            * weight
        )
