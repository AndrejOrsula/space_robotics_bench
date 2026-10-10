"""``srb event-camera`` — manage event-camera assets (weights, backends)."""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from srb.cli._base import BaseCommand
from srb.cli._output import print_json
from srb.cli._parsing import SrbHelpFormatter
from srb.utils.failures import InvalidInputError


def _complete_weight_mirrors(**_: object) -> list[str]:
    from srb.core.sensor.event_camera.weights import list_known_mirrors

    return [mirror.short_name for mirror in list_known_mirrors()]


def _complete_fixture_mirrors(**_: object) -> list[str]:
    from srb.core.sensor.event_camera.fixtures import list_known_fixtures

    return [fixture.short_name for fixture in list_known_fixtures()]


def _invalid_fetch_input(message: str) -> InvalidInputError:
    # The dispatcher reports FailureError as one line (exit 2); direct callers
    # still receive a ValueError.
    return InvalidInputError(
        message=message,
        code="invalid_input.event_camera_fetch",
        component="cli.event-camera",
    )


class EventCameraCommand(BaseCommand):
    """Manage event-camera optional assets (weights, fixtures, backend info)."""

    NAME = "event-camera"
    HELP = "Manage event-camera weights, fixtures, and backends"

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
        actions = parser.add_subparsers(
            title="Event-camera actions",
            dest="event_camera_action",
            required=True,
        )

        fetch = actions.add_parser(
            "fetch-weights",
            help=("Download event-camera weights (E2VID or V2CE) with SHA256 verify"),
            formatter_class=SrbHelpFormatter,
        )
        fetch.add_argument(
            "--artifact",
            default="e2vid",
            choices=("e2vid", "v2ce"),
            help=(
                "Weight artifact fetched with --url; sets the default saved "
                "filename. See event_camera_weight_mirrors.md."
            ),
        )
        fetch.add_argument(
            "--cache-dir",
            type=Path,
            default=None,
            help="Override cache directory (default: ~/.cache/srb/event_weights/)",
        )
        fetch.add_argument(
            "--mirror",
            default=None,
            help=(
                "Short-name of a registered mirror (see "
                "`srb event-camera list-mirrors`). Mutually exclusive with --url. "
                "The mirror's artifact identity wins; --artifact is ignored."
            ),
        ).completer = _complete_weight_mirrors  # type: ignore[attr-defined]
        fetch.add_argument(
            "--url",
            default=None,
            help=(
                "Override download URL. Mutually exclusive with --mirror. "
                "Requires --sha256."
            ),
        )
        fetch.add_argument(
            "--sha256",
            default=None,
            help=(
                "Override expected SHA256. Required when --url is supplied. "
                "Ignored when --mirror is used (the mirror's pinned hash wins)."
            ),
        )
        fetch.add_argument(
            "--name",
            default=None,
            help="Override saved filename (defaults to upstream artifact name)",
        )
        fetch.add_argument(
            "--force",
            action="store_true",
            help="Re-download even when a verified cached copy exists",
        )

        actions.add_parser(
            "list-mirrors",
            help=("List registered event-camera weight mirrors (E2VID + V2CE)"),
            formatter_class=SrbHelpFormatter,
        )

        fetch_fixture = actions.add_parser(
            "fetch-fixture",
            help=(
                "Download an event-camera reference fixture (Gen4 HDF5) "
                "with SHA256 verify"
            ),
            formatter_class=SrbHelpFormatter,
        )
        fetch_fixture.add_argument(
            "--artifact",
            default="gen4",
            choices=("gen4",),
            help=(
                "Reference fixture fetched with --url; sets the default saved "
                "filename and metadata. Use --mirror for registered fixture "
                "mirrors; see event_camera_fixture_mirrors.md."
            ),
        )
        fetch_fixture.add_argument(
            "--cache-dir",
            type=Path,
            default=None,
            help="Override cache directory (default: ~/.cache/srb/event_fixtures/)",
        )
        fetch_fixture.add_argument(
            "--mirror",
            default=None,
            help=(
                "Short-name of a registered fixture mirror (see "
                "`srb event-camera list-fixtures`). Mutually exclusive with --url."
            ),
        ).completer = _complete_fixture_mirrors  # type: ignore[attr-defined]
        fetch_fixture.add_argument(
            "--url",
            default=None,
            help=(
                "Override download URL. Mutually exclusive with --mirror. "
                "Requires --sha256."
            ),
        )
        fetch_fixture.add_argument(
            "--sha256",
            default=None,
            help=(
                "Override expected SHA256. Required when --url is supplied. "
                "Ignored when --mirror is used (the mirror's pinned hash wins)."
            ),
        )
        fetch_fixture.add_argument(
            "--name",
            default=None,
            help="Override saved filename (defaults to upstream artifact name)",
        )
        fetch_fixture.add_argument(
            "--force",
            action="store_true",
            help="Re-download even when a verified cached copy exists",
        )

        actions.add_parser(
            "list-fixtures",
            help=("List registered event-camera reference-fixture mirrors (Gen4 HDF5)"),
            formatter_class=SrbHelpFormatter,
        )

        backends = actions.add_parser(
            "backends",
            help="List registered event-generator backends",
            formatter_class=SrbHelpFormatter,
        )
        backends.add_argument(
            "--detailed",
            action="store_true",
            help="Print backend metadata (name, batching, class, module)",
        )

        return parser

    @classmethod
    def execute(
        cls,
        event_camera_action: str,
        cache_dir: Path | None = None,
        mirror: str | None = None,
        url: str | None = None,
        sha256: str | None = None,
        name: str | None = None,
        force: bool = False,
        detailed: bool = False,
        artifact: str = "e2vid",
        **_kwargs,
    ) -> None:
        if event_camera_action == "fetch-weights":
            cls._execute_fetch_weights(
                artifact=artifact,
                cache_dir=cache_dir,
                mirror=mirror,
                url=url,
                sha256=sha256,
                name=name,
                force=force,
            )
        elif event_camera_action == "list-mirrors":
            cls._execute_list_mirrors()
        elif event_camera_action == "fetch-fixture":
            cls._execute_fetch_fixture(
                artifact=artifact,
                cache_dir=cache_dir,
                mirror=mirror,
                url=url,
                sha256=sha256,
                name=name,
                force=force,
            )
        elif event_camera_action == "list-fixtures":
            cls._execute_list_fixtures()
        elif event_camera_action == "backends":
            cls._execute_list_backends(detailed=detailed)
        else:
            raise ValueError(f"unknown event-camera action: {event_camera_action!r}")

    @classmethod
    def _execute_fetch_weights(
        cls,
        *,
        artifact: str,
        cache_dir: Path | None,
        mirror: str | None,
        url: str | None,
        sha256: str | None,
        name: str | None,
        force: bool,
    ) -> None:
        from srb.core.sensor.event_camera.weights import (
            ARTIFACT_DEFAULTS,
            WeightArtifact,
            fetch_weight_artifact,
            get_known_mirror,
        )

        if mirror is not None and url is not None:
            raise _invalid_fetch_input(
                "--mirror and --url are mutually exclusive: pick one source "
                "of truth for the download URL."
            )
        if mirror is None and url is None:
            raise _invalid_fetch_input(
                "pass --mirror <short_name> or --url <url> --sha256 <sha256>"
            )

        if artifact not in ARTIFACT_DEFAULTS:
            raise _invalid_fetch_input(
                f"unknown --artifact {artifact!r}; expected one of "
                f"{sorted(ARTIFACT_DEFAULTS)}"
            )
        default = ARTIFACT_DEFAULTS[artifact]

        if mirror is not None:
            try:
                entry = get_known_mirror(mirror)
            except KeyError as exc:
                raise _invalid_fetch_input(str(exc.args[0])) from exc
            resolved = WeightArtifact(
                name=name or entry.artifact.name,
                url=entry.artifact.url,
                sha256=entry.artifact.sha256,
                size_bytes=entry.artifact.size_bytes,
            )
            resolved_kind = cls._artifact_kind_for_weight(entry.artifact.name, "")
        else:
            if url is None or sha256 is None:
                raise _invalid_fetch_input(
                    "--sha256 is required when --url is supplied (so the "
                    "fetcher verifies integrity against an explicit "
                    "expectation)."
                )
            resolved = WeightArtifact(
                name=name or default.name,
                url=url,
                sha256=sha256,
                size_bytes=default.size_bytes,
            )
            resolved_kind = artifact

        with _explain_denied_download(resolved.url):
            path = fetch_weight_artifact(
                resolved,
                cache_dir=cache_dir,
                force=force,
            )
        print_json(
            {
                "artifact": resolved.name,
                "artifact_kind": resolved_kind,
                "mirror": mirror,
                "path": str(path),
                "sha256": resolved.sha256,
                "url": resolved.url,
            }
        )

    @staticmethod
    def _artifact_kind_for_weight(name: str, fallback: str) -> str:
        from srb.core.sensor.event_camera.weights import ARTIFACT_DEFAULTS

        for kind, default in ARTIFACT_DEFAULTS.items():
            if default.name == name:
                return kind
        return fallback

    @classmethod
    def _execute_list_mirrors(cls) -> None:
        from srb.core.sensor.event_camera.weights import list_known_mirrors

        mirrors = list_known_mirrors()
        print_json(
            {
                "mirrors": [
                    {
                        "short_name": m.short_name,
                        "artifact": m.artifact.name,
                        "url": m.artifact.url,
                        "sha256": m.artifact.sha256,
                        "size_bytes": m.artifact.size_bytes,
                        "license_note": m.license_note,
                        "notes": m.notes,
                    }
                    for m in mirrors
                ]
            }
        )

    @classmethod
    def _execute_fetch_fixture(
        cls,
        *,
        artifact: str,
        cache_dir: Path | None,
        mirror: str | None,
        url: str | None,
        sha256: str | None,
        name: str | None,
        force: bool,
    ) -> None:
        from srb.core.sensor.event_camera.fixtures import (
            FIXTURE_DEFAULTS,
            FixtureArtifact,
            fetch_fixture,
            get_known_fixture,
        )

        if mirror is not None and url is not None:
            raise _invalid_fetch_input(
                "--mirror and --url are mutually exclusive: pick one source "
                "of truth for the download URL."
            )
        if mirror is None and url is None:
            raise _invalid_fetch_input(
                "pass --mirror <short_name> or --url <url> --sha256 <sha256>"
            )

        if artifact not in FIXTURE_DEFAULTS:
            raise _invalid_fetch_input(
                f"unknown --artifact {artifact!r}; expected one of "
                f"{sorted(FIXTURE_DEFAULTS)}"
            )
        default = FIXTURE_DEFAULTS[artifact]

        if mirror is not None:
            try:
                entry = get_known_fixture(mirror)
            except KeyError as exc:
                raise _invalid_fetch_input(str(exc.args[0])) from exc
            resolved = FixtureArtifact(
                name=name or entry.artifact.name,
                url=entry.artifact.url,
                sha256=entry.artifact.sha256,
                size_bytes=entry.artifact.size_bytes,
                schema_version=entry.artifact.schema_version,
                sensor_model=entry.artifact.sensor_model,
                num_episodes=entry.artifact.num_episodes,
            )
        else:
            if url is None or sha256 is None:
                raise _invalid_fetch_input(
                    "--sha256 is required when --url is supplied (so the "
                    "fetcher verifies integrity against an explicit "
                    "expectation)."
                )
            resolved = FixtureArtifact(
                name=name or default.name,
                url=url,
                sha256=sha256,
                size_bytes=default.size_bytes,
                schema_version=default.schema_version,
                sensor_model=default.sensor_model,
                num_episodes=default.num_episodes,
            )

        with _explain_denied_download(resolved.url):
            path = fetch_fixture(
                resolved,
                cache_dir=cache_dir,
                force=force,
            )
        print_json(
            {
                "artifact": resolved.name,
                "artifact_kind": artifact,
                "mirror": mirror,
                "path": str(path),
                "schema_version": resolved.schema_version,
                "sensor_model": resolved.sensor_model,
                "sha256": resolved.sha256,
                "url": resolved.url,
            }
        )

    @classmethod
    def _execute_list_fixtures(cls) -> None:
        from srb.core.sensor.event_camera.fixtures import list_known_fixtures

        fixtures = list_known_fixtures()
        print_json(
            {
                "fixtures": [
                    {
                        "short_name": f.short_name,
                        "artifact": f.artifact.name,
                        "url": f.artifact.url,
                        "sha256": f.artifact.sha256,
                        "size_bytes": f.artifact.size_bytes,
                        "schema_version": f.artifact.schema_version,
                        "sensor_model": f.artifact.sensor_model,
                        "num_episodes": f.artifact.num_episodes,
                        "license_note": f.license_note,
                        "notes": f.notes,
                    }
                    for f in fixtures
                ]
            }
        )

    @classmethod
    def _execute_list_backends(cls, *, detailed: bool) -> None:
        from srb.core.sensor.event_camera import (
            get_backend_factory,
            list_backends,
        )

        names = list_backends()
        if not detailed:
            print_json({"backends": list(names)})
            return

        rows = []
        for name in names:
            factory = get_backend_factory(name)
            rows.append(
                {
                    "name": name,
                    "supports_batching": getattr(factory, "supports_batching", False),
                    "class": factory.__name__,
                    "module": factory.__module__,
                }
            )
        print_json({"backends": rows})


@contextmanager
def _explain_denied_download(url: str) -> Iterator[None]:
    """Turn an HTTP 401/403 into an actionable message instead of a traceback."""
    import urllib.error

    try:
        yield
    except urllib.error.HTTPError as error:
        if error.code not in (401, 403):
            raise
        hint = (
            "set HF_TOKEN for an account with access, or pass --url with --sha256."
            if "huggingface.co" in url
            else "check that the URL is reachable without credentials."
        )
        raise _invalid_fetch_input(
            f"Access denied ({error.code}) for {url}: {hint}"
        ) from None
