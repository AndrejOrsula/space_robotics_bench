"""On-demand event-camera dataset fidelity metrics.

The statistical and reconstruction submodules provide pure-function APIs
used by the dataset validation CLI.
"""

from __future__ import annotations

from srb.datasets.event_validation.reconstruction import (
    ReconstructionScoreCard,
    score_reconstruction,
)
from srb.datasets.event_validation.statistical import (
    ScoreCard,
    compute_metrics,
    score_pair,
)

__all__ = [
    "ReconstructionScoreCard",
    "ScoreCard",
    "compute_metrics",
    "score_pair",
    "score_reconstruction",
]
