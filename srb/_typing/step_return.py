from typing import Any, NamedTuple, TypeAlias

from torch import Tensor

# ``from_tuple`` accepts the optional trailing ``info`` item as well.
_StepReturn: TypeAlias = (
    tuple[dict[str, dict[str, Tensor]], dict[str, Tensor], Tensor, Tensor]
    | tuple[
        dict[str, dict[str, Tensor]],
        dict[str, Tensor],
        Tensor,
        Tensor,
        dict[str, Any] | None,
    ]
)


class StepReturn(NamedTuple):
    """Values returned by a task's ``extract_step_return``.

    Observation, reward, termination, and truncation are batched tensors on
    ``env.device``. ``info`` contains host-bound episode diagnostics and is
    neither batched nor validated; see ``validate_step_return``.
    """

    observation: dict[str, dict[str, Tensor]]
    reward: dict[str, Tensor]
    termination: Tensor
    truncation: Tensor
    info: dict[str, Any] | None = None

    @staticmethod
    def from_tuple(step_return: _StepReturn) -> "StepReturn":
        return StepReturn(*step_return)
