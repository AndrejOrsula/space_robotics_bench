"""``srb model`` -- Manage SRB model artifacts."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

from srb.cli._args import complete_algorithms
from srb.cli._base import BaseCommand
from srb.cli._output import print_json
from srb.cli._parsing import SrbHelpFormatter
from srb.core.algorithms import canonical_algorithm_label
from srb.utils.failures import InvalidInputError


def _invalid_input(message: str) -> InvalidInputError:
    # Keep CLI errors concise while preserving ValueError compatibility.
    return InvalidInputError(
        message=message,
        code="invalid_input.model_cli",
        component="cli.model",
    )


class ModelCommand(BaseCommand):
    """Create, inspect, validate, and publish SRB model artifacts."""

    NAME = "model"
    HELP = "Manage model artifacts"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        **_kwargs,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        model_subs = parser.add_subparsers(
            title="Model actions",
            dest="model_action",
            required=True,
        )

        create = model_subs.add_parser(
            "create",
            help="Create a self-contained SRB model artifact",
            formatter_class=SrbHelpFormatter,
        )
        create.add_argument("checkpoint", type=Path, help="Source checkpoint file")
        create.add_argument(
            "--output",
            type=Path,
            required=True,
            help="Output model artifact directory",
        )
        create.add_argument(
            "--task",
            default=None,
            help="SRB task slug; inferred from --dataset when omitted",
        )
        create.add_argument(
            "--embodiment",
            default=None,
            help="Embodiment slug; inferred from --dataset when omitted",
        )
        create.add_argument(
            "--action-mode",
            dest="action_mode",
            default=None,
            help="Action mode slug; inferred from --dataset when omitted",
        )
        create.add_argument(
            "--algo",
            required=True,
            type=canonical_algorithm_label,
            help="Registered SRB algorithm label",
        ).completer = complete_algorithms  # type: ignore[attr-defined]
        create.add_argument(
            "--framework",
            default=None,
            help="Override framework inferred from --algo",
        )
        create.add_argument(
            "--dataset",
            type=Path,
            default=None,
            help=(
                "Training dataset manifest, artifact directory, or dataset file "
                "used to infer repository provenance"
            ),
        )
        create.add_argument(
            "--dataset-repo-id",
            default=None,
            help="Training dataset repository id",
        )
        create.add_argument(
            "--dataset-revision",
            default=None,
            help="Training dataset revision",
        )
        create.add_argument(
            "--metric",
            action="append",
            default=None,
            help="Metric as key=value; may be passed multiple times",
        )
        create.add_argument(
            "--metadata",
            action="append",
            default=None,
            help="Metadata as key=value; may be passed multiple times",
        )
        create.add_argument(
            "--repo-id",
            default=None,
            help="Override Hugging Face model repository id",
        )
        create.add_argument(
            "--checkpoint-scope",
            dest="checkpoint_scope",
            choices=("weights", "agent_evaluation"),
            default=None,
            help=(
                "Declare what the checkpoint contains; file checkpoints "
                "default to 'weights' and sanitized Dreamer directories to "
                "'agent_evaluation'"
            ),
        )

        from_run = model_subs.add_parser(
            "from-run",
            help="Create a model artifact from a completed SRB run directory",
            formatter_class=SrbHelpFormatter,
        )
        from_run.add_argument("run", type=Path, help="SRB run directory")
        from_run.add_argument(
            "--output",
            type=Path,
            required=True,
            help="Output model artifact directory",
        )
        from_run.add_argument(
            "--task",
            default=None,
            help=(
                "SRB task slug; inferred from the run's metadata.json "
                "(run.env_id), or from --dataset when one is given"
            ),
        )
        from_run.add_argument(
            "--embodiment",
            default=None,
            help="Embodiment slug; inferred from --dataset when omitted",
        )
        from_run.add_argument(
            "--action-mode",
            dest="action_mode",
            default=None,
            help="Action mode slug; inferred from --dataset when omitted",
        )
        from_run.add_argument(
            "--algo",
            default=None,
            type=canonical_algorithm_label,
            help="Algorithm slug; inferred from metadata.json when omitted",
        ).completer = complete_algorithms  # type: ignore[attr-defined]
        from_run.add_argument(
            "--checkpoint",
            type=Path,
            default=None,
            help="Explicit checkpoint path, absolute or relative to the run",
        )
        from_run.add_argument(
            "--framework",
            default=None,
            help="Override framework inferred from the resolved algorithm",
        )
        from_run.add_argument(
            "--dataset",
            type=Path,
            default=None,
            help=(
                "Training dataset manifest, artifact directory, or dataset file "
                "used to infer repository provenance"
            ),
        )
        from_run.add_argument(
            "--dataset-repo-id",
            default=None,
            help="Training dataset repository id",
        )
        from_run.add_argument(
            "--dataset-revision",
            default=None,
            help="Training dataset revision",
        )
        from_run.add_argument(
            "--metric",
            action="append",
            default=None,
            help="Metric as key=value; may be passed multiple times",
        )
        from_run.add_argument(
            "--metadata",
            action="append",
            default=None,
            help="Metadata as key=value; may be passed multiple times",
        )
        from_run.add_argument(
            "--repo-id",
            default=None,
            help="Override Hugging Face model repository id",
        )
        from_run.add_argument(
            "--checkpoint-scope",
            dest="checkpoint_scope",
            choices=("weights", "agent_evaluation"),
            default=None,
            help=(
                "Declare what the checkpoint contains; file checkpoints "
                "default to 'weights' and sanitized Dreamer directories to "
                "'agent_evaluation'"
            ),
        )

        info = model_subs.add_parser(
            "info",
            help="Print model artifact manifest summary",
            formatter_class=SrbHelpFormatter,
        )
        info.add_argument("path", type=Path, help="Manifest or artifact directory")

        validate = model_subs.add_parser(
            "validate",
            help="Validate model artifact manifest and checkpoint",
            formatter_class=SrbHelpFormatter,
        )
        validate.add_argument(
            "path",
            type=Path,
            help="Manifest or artifact directory",
        )
        validate.add_argument(
            "--strict",
            action="store_true",
            help="Treat validation warnings as errors",
        )

        push = model_subs.add_parser(
            "push",
            help="Upload or dry-run a model publication to Hugging Face Hub",
            formatter_class=SrbHelpFormatter,
        )
        push.add_argument("path", type=Path, help="Manifest or artifact directory")
        push.add_argument(
            "--repo-id",
            default=None,
            help="Override Hugging Face repository id",
        )
        push.add_argument(
            "--dry-run",
            action="store_true",
            help="Print upload plan without network access",
        )
        push.add_argument(
            "--private",
            action="store_true",
            help="Create the Hub model repository as private",
        )
        push.add_argument(
            "--commit-message",
            default="Upload SRB model artifact",
            help="Commit message to use for the Hub upload",
        )

        card = model_subs.add_parser(
            "card",
            help="Write a Hugging Face README.md model card",
            formatter_class=SrbHelpFormatter,
        )
        card.add_argument("path", type=Path, help="Manifest or artifact directory")
        card.add_argument(
            "--output",
            type=Path,
            default=None,
            help="Optional output path; defaults to README.md in the artifact root",
        )
        card.add_argument(
            "--overwrite",
            dest="overwrite_card",
            action="store_true",
            help="Overwrite an existing model card",
        )
        return parser

    @classmethod
    def execute(
        cls,
        model_action: str,
        path: Path | None = None,
        run: Path | None = None,
        checkpoint: Path | None = None,
        output: Path | None = None,
        task: str | None = None,
        embodiment: str | None = None,
        action_mode: str | None = None,
        algo: str | None = None,
        framework: str | None = None,
        dataset: Path | None = None,
        dataset_repo_id: str | None = None,
        dataset_revision: str | None = None,
        metric: list[str] | None = None,
        metadata: list[str] | None = None,
        repo_id: str | None = None,
        checkpoint_scope: str | None = None,
        dry_run: bool = False,
        private: bool = False,
        commit_message: str = "Upload SRB model artifact",
        strict: bool = False,
        overwrite_card: bool = False,
        **_kwargs,
    ) -> None:
        if model_action == "create":
            if checkpoint is None or output is None:
                raise _invalid_input("model create requires checkpoint and --output")
            if algo is None:
                raise _invalid_input("model create requires --algo")
            task, embodiment, action_mode = _resolve_create_identity(
                task=task,
                embodiment=embodiment,
                action_mode=action_mode,
                dataset=dataset,
            )
            cls._execute_create(
                checkpoint=checkpoint,
                output=output,
                task=task,
                embodiment=embodiment,
                action_mode=action_mode,
                algo=algo,
                framework=framework,
                dataset=dataset,
                dataset_repo_id=dataset_repo_id,
                dataset_revision=dataset_revision,
                metrics=_parse_key_values(metric, value_type="metric"),
                metadata=_parse_key_values(metadata, value_type="metadata"),
                repo_id=repo_id,
                checkpoint_scope=checkpoint_scope,
            )
        elif model_action == "from-run":
            if run is None or output is None:
                raise _invalid_input("model from-run requires run and --output")
            cls._execute_from_run(
                run=run,
                output=output,
                task=task,
                embodiment=embodiment,
                action_mode=action_mode,
                algo=algo,
                checkpoint=checkpoint,
                framework=framework,
                dataset=dataset,
                dataset_repo_id=dataset_repo_id,
                dataset_revision=dataset_revision,
                metrics=_parse_key_values(metric, value_type="metric"),
                metadata=_parse_key_values(metadata, value_type="metadata"),
                repo_id=repo_id,
                checkpoint_scope=checkpoint_scope,
            )
        elif model_action == "info":
            cls._execute_info(_require_path(path))
        elif model_action == "validate":
            cls._execute_validate(_require_path(path), strict=strict)
        elif model_action == "push":
            cls._execute_push(
                _require_path(path),
                repo_id=repo_id,
                dry_run=dry_run,
                private=private,
                commit_message=commit_message,
            )
        elif model_action == "card":
            cls._execute_card(
                _require_path(path),
                output=output,
                overwrite=overwrite_card,
            )
        else:
            raise ValueError(f"unknown model action: {model_action!r}")

    @classmethod
    def _execute_create(
        cls,
        *,
        checkpoint: Path,
        output: Path,
        task: str,
        embodiment: str,
        action_mode: str,
        algo: str,
        framework: str | None,
        dataset: Path | None,
        dataset_repo_id: str | None,
        dataset_revision: str | None,
        metrics: dict[str, Any],
        metadata: dict[str, Any],
        repo_id: str | None,
        checkpoint_scope: str | None = None,
    ) -> None:
        from srb.experiments.artifacts import ArtifactIdentity
        from srb.experiments.models import prepare_model_artifact

        result = prepare_model_artifact(
            checkpoint_path=checkpoint,
            output_dir=output,
            identity=ArtifactIdentity(
                task=task,
                embodiment=embodiment,
                action_mode=action_mode,
                algo=algo,
            ),
            framework=framework,
            training_dataset_path=dataset,
            dataset_repo_id=dataset_repo_id,
            dataset_revision=dataset_revision,
            metrics=metrics,
            metadata=metadata,
            hf_repo_id=repo_id,
            checkpoint_scope=checkpoint_scope,
        )
        print_json(result)

    @classmethod
    def _execute_from_run(
        cls,
        *,
        run: Path,
        output: Path,
        task: str | None,
        embodiment: str | None,
        action_mode: str | None,
        algo: str | None,
        checkpoint: Path | None,
        framework: str | None,
        dataset: Path | None,
        dataset_repo_id: str | None,
        dataset_revision: str | None,
        metrics: dict[str, Any],
        metadata: dict[str, Any],
        repo_id: str | None,
        checkpoint_scope: str | None = None,
    ) -> None:
        from srb.experiments.models import prepare_model_artifact_from_run

        result = prepare_model_artifact_from_run(
            run_dir=run,
            output_dir=output,
            task=task,
            embodiment=embodiment,
            action_mode=action_mode,
            algo=algo,
            checkpoint_path=checkpoint,
            framework=framework,
            training_dataset_path=dataset,
            dataset_repo_id=dataset_repo_id,
            dataset_revision=dataset_revision,
            metrics=metrics,
            metadata=metadata,
            hf_repo_id=repo_id,
            checkpoint_scope=checkpoint_scope,
        )
        print_json(result)

    @classmethod
    def _execute_info(cls, path: Path) -> None:
        from srb.experiments.models import (
            find_model_artifact_manifest,
            read_model_artifact_manifest,
        )

        try:
            manifest = read_model_artifact_manifest(find_model_artifact_manifest(path))
        except (OSError, TypeError, ValueError) as exc:
            raise _invalid_input(f"model info: {exc}") from exc
        print_json(manifest)

    @classmethod
    def _execute_validate(cls, path: Path, *, strict: bool = False) -> None:
        from srb.experiments.models import validate_model_artifact_layout

        report = validate_model_artifact_layout(path, strict=strict)
        print_json(report)
        if not report.valid:
            raise SystemExit(1)

    @classmethod
    def _execute_push(
        cls,
        path: Path,
        *,
        repo_id: str | None,
        dry_run: bool,
        private: bool,
        commit_message: str,
    ) -> None:
        from srb.experiments.huggingface import push_model_to_hub

        plan = push_model_to_hub(
            path,
            repo_id=repo_id,
            dry_run=dry_run,
            private=private,
            commit_message=commit_message,
        )
        print_json(plan, dry_run=dry_run)

    @classmethod
    def _execute_card(
        cls,
        path: Path,
        *,
        output: Path | None,
        overwrite: bool,
    ) -> None:
        from srb.experiments.huggingface import write_huggingface_model_card

        result = write_huggingface_model_card(
            path,
            output_path=output,
            overwrite=overwrite,
        )
        print_json(result)


def _require_path(path: Path | None) -> Path:
    if path is None:
        raise _invalid_input("model action requires a path")
    return path


def _parse_key_values(
    values: list[str] | None, *, value_type: str = "generic"
) -> dict[str, Any]:
    parsed: dict[str, Any] = {}
    for value in values or []:
        key, sep, raw_value = value.partition("=")
        if not sep or not key.strip():
            raise _invalid_input(f"expected key=value, got {value!r}")
        raw_value = raw_value.strip()
        if value_type == "metadata":
            parsed[key.strip()] = raw_value
        elif value_type == "metric":
            try:
                numeric_value = int(raw_value)
            except ValueError:
                try:
                    numeric_value = float(raw_value)
                except ValueError as exc:
                    raise _invalid_input(
                        f"metric {key.strip()!r} must be a finite number, got "
                        f"{raw_value!r}"
                    ) from exc
            if isinstance(numeric_value, float) and not math.isfinite(numeric_value):
                raise _invalid_input(
                    f"metric {key.strip()!r} must be a finite number, got {raw_value!r}"
                )
            parsed[key.strip()] = numeric_value
        else:
            parsed[key.strip()] = _parse_scalar(raw_value)
    return parsed


def _parse_scalar(value: str) -> Any:
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _resolve_create_identity(
    *,
    task: str | None,
    embodiment: str | None,
    action_mode: str | None,
    dataset: Path | None,
) -> tuple[str, str, str]:
    if dataset is None:
        missing = [
            flag
            for flag, value in (
                ("--task", task),
                ("--embodiment", embodiment),
                ("--action-mode", action_mode),
            )
            if value is None
        ]
        if missing:
            raise _invalid_input(
                "model create requires "
                + ", ".join(missing)
                + " unless --dataset is provided"
            )
        return task, embodiment, action_mode  # type: ignore[return-value]

    from srb.datasets.manifest import find_dataset_manifest, read_dataset_manifest

    manifest = read_dataset_manifest(find_dataset_manifest(dataset))
    return (
        _resolve_dataset_identity_field(
            field_name="task",
            explicit_value=task,
            manifest_value=manifest.identity.task,
        ),
        _resolve_dataset_identity_field(
            field_name="embodiment",
            explicit_value=embodiment,
            manifest_value=manifest.identity.embodiment,
        ),
        _resolve_dataset_identity_field(
            field_name="action_mode",
            explicit_value=action_mode,
            manifest_value=manifest.identity.action_mode,
        ),
    )


def _resolve_dataset_identity_field(
    *,
    field_name: str,
    explicit_value: str | None,
    manifest_value: str,
) -> str:
    if explicit_value is None:
        return manifest_value

    from srb.experiments.artifacts import slugify_underscore

    if slugify_underscore(explicit_value) != manifest_value:
        raise _invalid_input(
            f"{field_name} {explicit_value!r} does not match dataset "
            f"manifest {field_name} {manifest_value!r}"
        )
    return explicit_value
