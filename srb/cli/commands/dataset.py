"""``srb dataset`` -- Inspect and validate SRB dataset artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from srb.cli._args import complete_algorithms
from srb.cli._base import BaseCommand
from srb.cli._output import print_json
from srb.cli._parsing import SrbHelpFormatter

_DATASET_CONTROLLER_CHOICES = ("human", "policy", "random", "zero", "real")


def _dataset_controller_label(value: str) -> str:
    if value not in _DATASET_CONTROLLER_CHOICES:
        raise argparse.ArgumentTypeError(
            "controller must be one of: " + ", ".join(_DATASET_CONTROLLER_CHOICES)
        )
    return value


class DatasetCommand(BaseCommand):
    """Manage SRB dataset metadata."""

    NAME = "dataset"
    HELP = "Manage dataset artifacts"

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
        dataset_subs = parser.add_subparsers(
            title="Dataset actions",
            dest="dataset_action",
            required=True,
        )

        info = dataset_subs.add_parser(
            "info",
            help="Print dataset manifest summary",
            formatter_class=SrbHelpFormatter,
        )
        info.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )

        validate = dataset_subs.add_parser(
            "validate",
            help="Validate dataset manifest and referenced files",
            formatter_class=SrbHelpFormatter,
        )
        validate.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )
        validate.add_argument(
            "--deep",
            action="store_true",
            help="Validate canonical dataset rows and declared metadata",
        )
        validate.add_argument(
            "--strict",
            action="store_true",
            help="Treat validation warnings as errors",
        )

        compatibility = dataset_subs.add_parser(
            "compatibility",
            help="Inspect dataset compatibility with learning algorithms",
            formatter_class=SrbHelpFormatter,
        )
        compatibility.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )
        compatibility.add_argument(
            "--algo",
            default=None,
            help="SRB algorithm name to assess",
        ).completer = complete_algorithms  # type: ignore[attr-defined]
        compatibility.add_argument(
            "--purpose",
            choices=("train", "collect_eval", "publish"),
            default="train",
            help="Compatibility purpose to assess",
        )

        export = dataset_subs.add_parser(
            "export",
            help="Export dataset to another supported layout",
            formatter_class=SrbHelpFormatter,
        )
        export.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or source data file",
        )
        export.add_argument(
            "--output",
            type=Path,
            required=True,
            help="Output dataset directory",
        )
        export.add_argument(
            "--format",
            choices=("srb_jsonl", "robomimic_hdf5", "lerobot"),
            default="srb_jsonl",
            help="Output dataset format",
        )

        pull = dataset_subs.add_parser(
            "pull",
            help="Download an SRB dataset artifact from Hugging Face Hub",
            formatter_class=SrbHelpFormatter,
        )
        pull.add_argument(
            "--repo-id",
            default=None,
            help="Hugging Face dataset repository id ('<org>/<name>')",
        )
        pull.add_argument(
            "--task",
            default=None,
            help="Artifact task slug (used with --embodiment/--action-mode to derive repo-id)",
        )
        pull.add_argument(
            "--embodiment",
            default=None,
            help="Artifact embodiment slug",
        )
        pull.add_argument(
            "--action-mode",
            dest="action_mode",
            default=None,
            help="Artifact action-mode slug",
        )
        pull.add_argument(
            "--output",
            type=Path,
            default=None,
            help="Local destination directory (defaults to the dataset name)",
        )
        pull.add_argument(
            "--revision",
            default=None,
            help="Optional Hub revision (commit SHA, branch, or tag)",
        )
        pull.add_argument(
            "--dry-run",
            action="store_true",
            help="Print the download plan without network access",
        )

        push = dataset_subs.add_parser(
            "push",
            help="Upload or dry-run a dataset publication to Hugging Face Hub",
            formatter_class=SrbHelpFormatter,
        )
        push.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )
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
            help="Create the Hub dataset repository as private",
        )
        push.add_argument(
            "--commit-message",
            default="Upload SRB dataset artifact",
            help="Commit message to use for the Hub upload",
        )

        card = dataset_subs.add_parser(
            "card",
            help="Write a Hugging Face README.md dataset card",
            formatter_class=SrbHelpFormatter,
        )
        card.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )
        card.add_argument(
            "--output",
            type=Path,
            default=None,
            help="Optional output path; defaults to README.md in the dataset root",
        )
        card.add_argument(
            "--overwrite",
            dest="overwrite_card",
            action="store_true",
            help="Overwrite an existing dataset card",
        )

        replay = dataset_subs.add_parser(
            "replay",
            help="Preview or visualize a canonical dataset",
            formatter_class=SrbHelpFormatter,
        )
        replay.add_argument(
            "path",
            type=Path,
            help="Manifest, dataset directory, or data file",
        )
        replay.add_argument(
            "--backend",
            choices=("summary", "rerun"),
            default="summary",
            help="Replay backend",
        )
        replay.add_argument(
            "--max-episodes",
            type=int,
            default=None,
            help="Maximum number of episodes to replay",
        )
        replay.add_argument(
            "--max-steps",
            type=int,
            default=None,
            help="Maximum number of steps to replay",
        )
        replay.add_argument(
            "--save",
            type=Path,
            default=None,
            help="Optional Rerun .rrd output path",
        )
        replay.add_argument(
            "--spawn",
            action="store_true",
            help="Spawn a Rerun viewer for the replay",
        )

        from_validation = dataset_subs.add_parser(
            "from-validation",
            help="Convert a real validation session to canonical SRB JSONL",
            formatter_class=SrbHelpFormatter,
        )
        from_validation.add_argument(
            "path",
            type=Path,
            help="Real validation session directory",
        )
        from_validation.add_argument(
            "--output",
            type=Path,
            required=True,
            help="Output dataset directory",
        )
        from_validation.add_argument(
            "--embodiment",
            default="real_hardware",
            help="Embodiment identifier for the dataset identity",
        )
        from_validation.add_argument(
            "--action-mode",
            dest="action_mode",
            default="real",
            help="Action-mode identifier for the dataset identity",
        )
        from_validation.add_argument(
            "--controller",
            default=None,
            choices=_DATASET_CONTROLLER_CHOICES,
            type=_dataset_controller_label,
            help=(
                "Controller/source label for the dataset manifest. Defaults to "
                "policy for validation specs with an algorithm/checkpoint, else real."
            ),
        )
        from_validation.add_argument(
            "--task",
            default=None,
            help="Override task when spec.yaml is unavailable",
        )
        from_validation.add_argument(
            "--algo",
            default=None,
            help="Override algorithm when spec.yaml is unavailable",
        ).completer = complete_algorithms  # type: ignore[attr-defined]

        from srb.cli.commands import dataset_event

        dataset_event.add_subparsers(dataset_subs)

        return parser

    @classmethod
    def execute(
        cls,
        dataset_action: str,
        path: Path | None = None,
        output: Path | None = None,
        format: str = "srb_jsonl",
        repo_id: str | None = None,
        dry_run: bool = False,
        private: bool = False,
        deep: bool = False,
        strict: bool = False,
        commit_message: str = "Upload SRB dataset artifact",
        backend: str = "summary",
        max_episodes: int | None = None,
        max_steps: int | None = None,
        save: Path | None = None,
        spawn: bool = False,
        embodiment: str = "real_hardware",
        action_mode: str = "real",
        controller: str | None = None,
        task: str | None = None,
        algo: str | None = None,
        purpose: str = "train",
        overwrite_card: bool = False,
        episodes: int = 1,
        num_envs: int = 1,
        steps_per_episode: int = 10,
        seed: int | None = None,
        preset: str = "imx636_nominal",
        event_backend: str = "v2e",
        policy_id: str = "walking_skeleton",
        validate_events_action: str | None = None,
        synth: Path | None = None,
        real: Path | None = None,
        resolution: str | None = None,
        sparsity_window_ms: float = 1.0,
        events: Path | None = None,
        rgb_npy: Path | None = None,
        times_npy: Path | None = None,
        operator: str = "baseline",
        include_lpips: bool = False,
        citation_md: Path | None = None,
        license_id: str = "CC-BY-4.0",
        doi: str | None = None,
        msg_type: str = "event_camera_msgs",
        topic: str = "/events",
        revision: str | None = None,
        **_kwargs,
    ) -> None:
        if dataset_action == "pull":
            cls._execute_pull(
                repo_id=repo_id,
                task=task,
                embodiment=embodiment,
                action_mode=action_mode,
                output=output,
                revision=revision,
                dry_run=dry_run,
            )
            return

        if dataset_action == "info":
            cls._execute_info(path)
        elif dataset_action == "validate":
            cls._execute_validate(path, deep=deep, strict=strict)
        elif dataset_action == "compatibility":
            cls._execute_compatibility(path, algo=algo, purpose=purpose)
        elif dataset_action == "export":
            if output is None:
                raise ValueError("dataset export requires --output")
            cls._execute_export(path, output=output, output_format=format)
        elif dataset_action == "push":
            cls._execute_push(
                path,
                repo_id=repo_id,
                dry_run=dry_run,
                private=private,
                commit_message=commit_message,
            )
        elif dataset_action == "card":
            cls._execute_card(
                path,
                output=output,
                overwrite=overwrite_card,
            )
        elif dataset_action == "replay":
            cls._execute_replay(
                path,
                backend=backend,
                max_episodes=max_episodes,
                max_steps=max_steps,
                save_path=save,
                spawn=spawn,
            )
        elif dataset_action == "from-validation":
            if output is None:
                raise ValueError("dataset from-validation requires --output")
            cls._execute_from_validation(
                path,
                output=output,
                embodiment=embodiment,
                action_mode=action_mode,
                controller=controller,
                task=task,
                algo=algo,
            )
        else:
            from srb.cli.commands import dataset_event

            if dataset_event.execute_if_event(
                dataset_action,
                path=path,
                output=output,
                task=task,
                episodes=episodes,
                num_envs=num_envs,
                steps_per_episode=steps_per_episode,
                seed=seed,
                preset=preset,
                event_backend=event_backend,
                policy_id=policy_id,
                validate_events_action=validate_events_action,
                synth=synth,
                real=real,
                resolution=resolution,
                sparsity_window_ms=sparsity_window_ms,
                events=events,
                rgb_npy=rgb_npy,
                times_npy=times_npy,
                operator=operator,
                include_lpips=include_lpips,
                citation_md=citation_md,
                license_id=license_id,
                doi=doi,
                msg_type=msg_type,
                topic=topic,
            ):
                pass
            else:
                raise ValueError(f"unknown dataset action: {dataset_action!r}")

    @classmethod
    def _execute_info(cls, path: Path) -> None:
        from srb.datasets.manifest import find_dataset_manifest, read_dataset_manifest
        from srb.utils.failures import InvalidInputError

        try:
            manifest = read_dataset_manifest(find_dataset_manifest(path))
        except (OSError, TypeError, ValueError) as exc:
            raise InvalidInputError(
                message=f"dataset info: {exc}",
                code="invalid_input.dataset_cli",
                component="cli.dataset",
            ) from exc
        print_json(_dataset_info_summary(manifest))

    @classmethod
    def _execute_validate(
        cls,
        path: Path,
        *,
        deep: bool = False,
        strict: bool = False,
    ) -> None:
        from srb.cli.commands import dataset_event

        if dataset_event.execute_validate_event_run_if_present(path, strict=strict):
            return

        from srb.datasets.manifest import validate_dataset_layout

        report = validate_dataset_layout(path, deep=deep, strict=strict)
        print_json(report)
        if not report.valid:
            raise SystemExit(1)

    @classmethod
    def _execute_compatibility(
        cls,
        path: Path,
        *,
        algo: str | None,
        purpose: str,
    ) -> None:
        from srb.datasets.manifest import find_dataset_manifest
        from srb.experiments.compatibility import (
            assess_algorithm_dataset_compatibility,
            assess_dataset_compatibility_matrix,
        )

        manifest_path = find_dataset_manifest(path)
        if algo is None:
            report = assess_dataset_compatibility_matrix(
                manifest_path,
                purpose=purpose,
            )
            print_json(report)
            return

        report = assess_algorithm_dataset_compatibility(
            algo,
            manifest_path,
            purpose=purpose,
        )
        print_json(report)
        if not report.compatible:
            raise SystemExit(1)

    @classmethod
    def _execute_export(
        cls,
        path: Path,
        *,
        output: Path,
        output_format: str,
    ) -> None:
        from srb.datasets.export import export_dataset_with_result

        result = export_dataset_with_result(
            source_path=path,
            output_dir=output,
            output_format=output_format,
        )
        print_json(result)

    @classmethod
    def _execute_pull(
        cls,
        *,
        repo_id: str | None,
        task: str | None,
        embodiment: str | None,
        action_mode: str | None,
        output: Path | None,
        revision: str | None,
        dry_run: bool,
    ) -> None:
        from srb.datasets.huggingface import pull_dataset_from_hub

        plan = pull_dataset_from_hub(
            repo_id=repo_id,
            task=task,
            embodiment=embodiment,
            action_mode=action_mode,
            output=output,
            revision=revision,
            dry_run=dry_run,
        )
        print_json(plan, dry_run=dry_run)

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
        from srb.datasets.huggingface import push_dataset_to_hub

        plan = push_dataset_to_hub(
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
        from srb.datasets.huggingface import write_huggingface_dataset_card

        result = write_huggingface_dataset_card(
            path,
            output_path=output,
            overwrite=overwrite,
        )
        print_json(result)

    @classmethod
    def _execute_replay(
        cls,
        path: Path,
        *,
        backend: str,
        max_episodes: int | None,
        max_steps: int | None,
        save_path: Path | None,
        spawn: bool,
    ) -> None:
        if backend == "summary" and (save_path is not None or spawn):
            from srb.utils.failures import InvalidInputError

            raise InvalidInputError(
                message="dataset replay: --save and --spawn need --backend rerun",
                code="invalid_input.dataset_cli",
                component="cli.dataset",
            )
        from srb.datasets.replay import replay_dataset

        summary = replay_dataset(
            path,
            backend=backend,
            max_episodes=max_episodes,
            max_steps=max_steps,
            save_path=save_path,
            spawn=spawn,
        )
        print_json(summary)

    @classmethod
    def _execute_from_validation(
        cls,
        path: Path,
        *,
        output: Path,
        embodiment: str,
        action_mode: str,
        controller: str | None,
        task: str | None,
        algo: str | None,
    ) -> None:
        from srb.datasets.real import export_real_validation_session

        result = export_real_validation_session(
            session_dir=path,
            output_dir=output,
            embodiment=embodiment,
            action_mode=action_mode,
            controller=controller,
            task=task,
            algo=algo,
        )
        print_json(result)


def _dataset_info_summary(manifest: Any) -> dict[str, Any]:
    # Convert immutable nested metadata before JSON encoding.
    metadata = manifest.to_dict()["metadata"]
    summary: dict[str, Any] = {
        "task": manifest.identity.task,
        "embodiment": manifest.identity.embodiment,
        "action_mode": manifest.identity.action_mode,
        "algo": manifest.identity.algo,
        "split": manifest.split.value,
        "controller": manifest.controller,
        "storage_format": manifest.storage_format.value,
        "dataset_path": manifest.dataset_path,
        "hf_repo_id": manifest.hf_repo_id,
        "artifact_hash": manifest.artifact_hash,
        "metadata": metadata,
    }

    action_dim = metadata.get("action_dim")
    if action_dim is not None:
        summary["action_dim"] = action_dim

    action_schema = _action_schema_from_metadata(metadata)
    if action_schema is not None:
        summary["action_components"] = _action_component_names(action_schema)
        summary["action_schema"] = action_schema
    return summary


def _action_schema_from_metadata(
    metadata: dict[str, Any],
) -> list[dict[str, Any]] | None:
    schema = metadata.get("action_schema")
    if not isinstance(schema, list) or not schema:
        return None
    if not all(isinstance(component, dict) for component in schema):
        return None
    return [dict(component) for component in schema]


def _action_component_names(schema: list[dict[str, Any]]) -> list[str]:
    names: list[str] = []
    for index, component in enumerate(schema):
        name = component.get("name")
        if name is None or not str(name).strip():
            name = f"component_{index}"
        names.append(str(name))
    return names
