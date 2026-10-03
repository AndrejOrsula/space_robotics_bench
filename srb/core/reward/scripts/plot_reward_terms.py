#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot
import numpy as np
import torch

from srb.core.reward.terms import (
    ConditionalRewardTerm,
    ConstantRewardTerm,
    GaussianAlignmentRewardTerm,
    GaussianBarrierRewardTerm,
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    GaussianZoneTrackingRewardTerm,
    HyperbolicAlignmentRewardTerm,
    HyperbolicBarrierRewardTerm,
    HyperbolicRewardTerm,
    HyperbolicTrackingRewardTerm,
    HyperbolicZoneTrackingRewardTerm,
    InverseBarrierRewardTerm,
    InverseRewardTerm,
    InverseSquareBarrierRewardTerm,
    InverseSquareRewardTerm,
    LaplacianAlignmentRewardTerm,
    LaplacianBarrierRewardTerm,
    LaplacianRewardTerm,
    LaplacianTrackingRewardTerm,
    LaplacianZoneTrackingRewardTerm,
    LinearRewardTerm,
    LogCoshRewardTerm,
    LorentzianAlignmentRewardTerm,
    LorentzianBarrierRewardTerm,
    LorentzianRewardTerm,
    LorentzianTrackingRewardTerm,
    LorentzianZoneTrackingRewardTerm,
    OffsetLinearRewardTerm,
    OffsetLogCoshRewardTerm,
    OffsetPowerRewardTerm,
    OffsetQuadraticRewardTerm,
    PeriodicDutyCycleRewardTerm,
    PeriodicPulseRewardTerm,
    PeriodicRectifiedSineRewardTerm,
    PeriodicSawtoothRewardTerm,
    PeriodicSineRewardTerm,
    PeriodicSoftSquareRewardTerm,
    PeriodicSquareRewardTerm,
    PeriodicTriangleRewardTerm,
    PowerRewardTerm,
    QuadraticRewardTerm,
    ScaledRewardTerm,
    SmoothMaxRewardTerm,
    SmoothMinRewardTerm,
    SoftThresholdRewardTerm,
    ThresholdRewardTerm,
)


def instantiate_and_run(term_cls, x_np, weight, limit, **kwargs):
    """Instantiate a reward term and evaluate it on ``x_np``."""
    x = torch.from_numpy(x_np).float()

    valid_fields = term_cls.model_fields.keys()

    config = {"weight": weight}

    if "limit" in valid_fields:
        config["limit"] = limit

    unknown = kwargs.keys() - valid_fields
    if unknown:
        raise TypeError(f"{term_cls.__name__} has no fields {sorted(unknown)}")
    config.update(kwargs)

    # Let invalid configs raise; a silent zero curve would misrepresent the term.
    term = term_cls(**config)

    with torch.no_grad():
        if term_cls == ConditionalRewardTerm:
            condition = (x % 2.0) < 0.1
            return term(condition).numpy()

        if term_cls in {SmoothMaxRewardTerm, SmoothMinRewardTerm}:
            inputs = torch.stack([x, torch.zeros_like(x)], dim=-1)
            return term(inputs).numpy()

        return term(x).numpy()


def plot_library_final_order(
    weight: float,
    limit: float,
    output_path: Path,
):
    x_common = np.linspace(-5.0, 5.0, 1000)
    # Cosine similarity is bounded to [-1, 1].
    x_ali = np.linspace(-1.0, 1.0, 1000)

    fig, axes = matplotlib.pyplot.subplots(4, 3, figsize=(24, 18))
    fig.suptitle(
        f"{'Reward' if weight >= 0 else 'Penalty'} Term Library [weight={weight}, limit={limit}]",
        fontsize=22,
        weight="bold",
    )
    axes = axes.flatten()

    def plot_cell(idx, title, x_data, config_list, ylim=None, xlabel="Input Value"):
        ax = axes[idx]
        ax.set_title(f"{idx + 1}. {title}", weight="bold", fontsize=11)
        ax.set_xlabel(xlabel)
        if ylim:
            ax.set_ylim(ylim)

        ax.grid(True, alpha=0.3, linestyle="--")
        ax.axhline(0, color="black", linewidth=0.8, alpha=0.5)
        ax.axvline(0, color="black", linewidth=0.8, alpha=0.5)

        for item in config_list:
            if len(item) == 2:
                cls, kwargs = item
                label_desc = ""
            else:
                cls, kwargs, label_desc = item

            y = instantiate_and_run(cls, x_data, weight, limit, **kwargs)

            name = cls.__name__.removesuffix("RewardTerm")

            params = [
                f'{k}="{v}"' if isinstance(v, str) else f"{k}={v}"
                for k, v in kwargs.items()
            ]
            param_str = f"[{', '.join(params)}]" if params else ""

            full_label = f"{name} {label_desc} {param_str}".strip()

            full_label = " ".join(full_label.split())

            ax.plot(
                x_data,
                y,
                label=full_label,
                linewidth=2.0,
                alpha=0.6,
            )

        ax.legend(fontsize=7, loc="best", framealpha=0.9)

    idx = 0
    plot_cell(
        idx,
        "Logical",
        x_common,
        [
            (
                ConstantRewardTerm,
                {},
            ),
            (
                ConditionalRewardTerm,
                {},
                "[condition=(x % 2.0 < 0.1)]",
            ),
            (
                ThresholdRewardTerm,
                {"threshold": 2.0, "operator": ">"},
            ),
            (
                SoftThresholdRewardTerm,
                {
                    "threshold": 2.0,
                    "operator": ">",
                    "sharpness": 5.0,
                },
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Periodic <-1.0, 1.0>",
        x_common,
        [
            (
                PeriodicSineRewardTerm,
                {"frequency": 0.2},
            ),
            (
                PeriodicSquareRewardTerm,
                {"frequency": 0.2},
            ),
            (
                PeriodicSoftSquareRewardTerm,
                {"frequency": 0.2, "sharpness": 5.0},
            ),
            (
                PeriodicSawtoothRewardTerm,
                {"frequency": 0.2},
            ),
            (
                PeriodicTriangleRewardTerm,
                {"frequency": 0.2},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Periodic <0.0, 1.0>",
        x_common,
        [
            (
                PeriodicRectifiedSineRewardTerm,
                {"frequency": 0.2},
            ),
            (
                PeriodicPulseRewardTerm,
                {"frequency": 0.2, "sharpness": 5.0},
            ),
            (
                PeriodicDutyCycleRewardTerm,
                {"frequency": 0.2, "duty": 0.2},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Value Scaling",
        x_common,
        [
            (
                LinearRewardTerm,
                {},
            ),
            (
                ScaledRewardTerm,
                {},
            ),
            (
                QuadraticRewardTerm,
                {},
            ),
            (
                LogCoshRewardTerm,
                {},
            ),
            (
                PowerRewardTerm,
                {"exponent": 3.0},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Value Scaling (Offset)",
        x_common,
        [
            (
                OffsetLinearRewardTerm,
                {"threshold": -1, "direction": "<"},
            ),
            (
                OffsetQuadraticRewardTerm,
                {"threshold": -1, "direction": "<"},
            ),
            (
                OffsetLogCoshRewardTerm,
                {"threshold": -1, "direction": "<"},
            ),
            (
                OffsetPowerRewardTerm,
                {"threshold": -1, "direction": "<", "exponent": 3.0},
            ),
            (
                OffsetLinearRewardTerm,
                {"threshold": 1, "direction": ">"},
            ),
            (
                OffsetQuadraticRewardTerm,
                {"threshold": 1, "direction": ">"},
            ),
            (
                OffsetLogCoshRewardTerm,
                {"threshold": 1, "direction": ">"},
            ),
            (
                OffsetPowerRewardTerm,
                {"threshold": 1, "direction": ">", "exponent": 3.0},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Inverse Functions",
        x_common,
        [
            (
                InverseRewardTerm,
                {},
            ),
            (
                InverseSquareRewardTerm,
                {},
            ),
            (
                InverseBarrierRewardTerm,
                {"threshold": -2.0, "direction": "<"},
            ),
            (
                InverseSquareBarrierRewardTerm,
                {"threshold": -2.0, "direction": "<"},
            ),
            (
                InverseBarrierRewardTerm,
                {"threshold": 2.0, "direction": ">"},
            ),
            (
                InverseSquareBarrierRewardTerm,
                {"threshold": 2.0, "direction": ">"},
            ),
        ],
    )
    idx += 1
    plot_cell(
        idx,
        "Error Minimization",
        x_common,
        [
            (
                GaussianRewardTerm,
                {"scale": 1.0},
            ),
            (
                LaplacianRewardTerm,
                {"scale": 1.0},
            ),
            (
                HyperbolicRewardTerm,
                {"scale": 1.0},
            ),
            (
                LorentzianRewardTerm,
                {"scale": 1.0},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Target Tracking",
        x_common,
        [
            (
                GaussianTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0},
            ),
            (
                LaplacianTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0},
            ),
            (
                HyperbolicTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0},
            ),
            (
                LorentzianTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Zone Tracking",
        x_common,
        [
            (
                GaussianZoneTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0, "tolerance": 0.5},
            ),
            (
                LaplacianZoneTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0, "tolerance": 0.5},
            ),
            (
                HyperbolicZoneTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0, "tolerance": 0.5},
            ),
            (
                LorentzianZoneTrackingRewardTerm,
                {"scale": 1.0, "target": 2.0, "tolerance": 0.5},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Barrier Functions",
        x_common,
        [
            (
                GaussianBarrierRewardTerm,
                {"scale": 1.0, "threshold": -2.0, "direction": "<"},
            ),
            (
                LaplacianBarrierRewardTerm,
                {"scale": 1.0, "threshold": -2.0, "direction": "<"},
            ),
            (
                HyperbolicBarrierRewardTerm,
                {"scale": 1.0, "threshold": -2.0, "direction": "<"},
            ),
            (
                LorentzianBarrierRewardTerm,
                {"scale": 1.0, "threshold": -2.0, "direction": "<"},
            ),
            (
                GaussianBarrierRewardTerm,
                {"scale": 1.0, "threshold": 2.0, "direction": ">"},
            ),
            (
                LaplacianBarrierRewardTerm,
                {"scale": 1.0, "threshold": 2.0, "direction": ">"},
            ),
            (
                HyperbolicBarrierRewardTerm,
                {"scale": 1.0, "threshold": 2.0, "direction": ">"},
            ),
            (
                LorentzianBarrierRewardTerm,
                {"scale": 1.0, "threshold": 2.0, "direction": ">"},
            ),
        ],
    )

    idx += 1
    plot_cell(
        idx,
        "Angular Alignment (Cosine Similarity)",
        x_ali,
        [
            (
                GaussianAlignmentRewardTerm,
                {"scale": 0.5236},
                "~ 30°",
            ),
            (
                LaplacianAlignmentRewardTerm,
                {"scale": 0.5236},
                "~ 30°",
            ),
            (
                HyperbolicAlignmentRewardTerm,
                {"scale": 0.5236},
                "~ 30°",
            ),
            (
                LorentzianAlignmentRewardTerm,
                {"scale": 0.5236},
                "~ 30°",
            ),
        ],
        xlabel="Input Value: torch.nn.functional.cosine_similarity(x1, x2)",
    )

    idx += 1
    plot_cell(
        idx,
        "Multi-Input Aggregation {x, 0.0}",
        x_common,
        [
            (
                SmoothMaxRewardTerm,
                {"temperature": 1.0},
            ),
            (
                SmoothMinRewardTerm,
                {"temperature": 1.0},
            ),
        ],
    )

    fig.tight_layout()
    fig.subplots_adjust(top=0.93)
    fig.savefig(output_path)
    matplotlib.pyplot.close(fig)
    print(f"Saved plot to: {output_path}")


if __name__ == "__main__":
    plot_library_final_order(
        weight=1.0,
        limit=4.0,
        output_path=Path(__file__).parent / "reward_terms.pdf",
    )
    plot_library_final_order(
        weight=-2.0,
        limit=64.0,
        output_path=Path(__file__).parent / "penalty_terms.pdf",
    )
