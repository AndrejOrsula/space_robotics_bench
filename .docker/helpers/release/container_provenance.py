#!/usr/bin/env python3
"""Create and validate the signed source-to-container predicate."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import stat
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import ModuleType
from typing import NoReturn, cast

SCHEMA_VERSION = 1
PREDICATE_TYPE = (
    "https://space-robotics-bench.github.io/attestations/container-source/v1"
)
SOURCE_HELPER_PATHS = (
    ".docker/helpers/release/source_provenance.bash",
    ".docker/helpers/release/source_provenance.py",
)
MAX_SOURCE_MANIFEST_BYTES = 64 * 1024 * 1024
ACCEPTED_ISAAC_SIM_SHA = "7c206f75bdadd9e05fc457f19863ca4c3f0cb693"
ACCEPTED_ISAAC_LAB_SHA = "fdc650c88dfd86aa158b9a1760b6537fef360555"
ACCEPTED_ISAAC_SIM_VERSION = "6.1.0"
ACCEPTED_ISAAC_LAB_VERSION = "3.0.0"
ACCEPTED_ISAAC_LAB_ISAAC_SIM_VERSION = "6.1.0.0"

_HEX_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
_GIT_OBJECT_RE = re.compile(r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$")
_UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
_REPOSITORY_RE = re.compile(r"^[a-z0-9][a-z0-9./_-]*[a-z0-9]$")
_RELEASE_VERSION_RE = re.compile(
    r"^[0-9]+(?:\.[0-9]+){2,}(?:[-+][0-9A-Za-z]+(?:[.-][0-9A-Za-z]+)*)?$"
)

REQUIRED_FIELDS = frozenset(
    {
        "schema_version",
        "source_repository",
        "source_commit",
        "source_ref",
        "source_snapshot_id",
        "source_tree_digest",
        "source_manifest_sha256",
        "helper_sha256",
        "repository",
        "image_iid",
        "registry_digest",
        "architecture",
        "base_image_digest",
        "isaac_sim_sha",
        "isaac_lab_sha",
        "isaac_sim_version",
        "isaac_lab_version",
        "isaac_lab_isaac_sim_version",
    }
)


class ProvenanceError(ValueError):
    """Raised when a predicate or signed attestation is not admissible."""


def _source_decoder() -> ModuleType:
    """Load the reviewed sibling decoder without ambient import lookup."""

    name = "_srb_release_source_provenance"
    path = Path(__file__).resolve().with_name("source_provenance.py")
    loaded = sys.modules.get(name)
    if loaded is not None:
        if Path(getattr(loaded, "__file__", "")).resolve() != path:
            _fail("cached source manifest decoder is not the reviewed sibling")
        return loaded
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        _fail("cannot load the reviewed source manifest decoder")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(name, None)
        raise
    return module


def decode_source_manifest(raw: bytes) -> dict[str, object]:
    """Validate manifest bytes with the reviewed standalone source owner."""

    try:
        return _source_decoder().decode_manifest(raw)
    except (ValueError, RuntimeError) as exc:
        raise ProvenanceError(f"source manifest is invalid: {exc}") from exc


def _fail(message: str) -> NoReturn:
    raise ProvenanceError(message)


def _mapping(value: object, context: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        _fail(f"{context} must be an object")
    return value


def _text(value: object, context: str) -> str:
    if not isinstance(value, str) or not value:
        _fail(f"{context} must be a non-empty string")
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in value):
        _fail(f"{context} must not contain control characters")
    return value


def _release_version(value: object, context: str) -> str:
    result = _text(value, context)
    if _RELEASE_VERSION_RE.fullmatch(result) is None:
        _fail(f"{context} must be a canonical release version")
    return result


def _repository(value: object, context: str) -> str:
    result = _text(value, context)
    registry = result.split("/", 1)[0]
    if (
        "@" in result
        or ":" in result
        or "/" not in result
        or "." not in registry
        or _REPOSITORY_RE.fullmatch(result) is None
    ):
        _fail(f"{context} must be a fully-qualified name without a tag or digest")
    return result


def _sha256(value: object, context: str) -> str:
    result = _text(value, context)
    if _HEX_SHA256_RE.fullmatch(result) is None:
        _fail(f"{context} must be a lowercase SHA-256 hex digest")
    return result


def _digest(value: object, context: str) -> str:
    result = _text(value, context)
    if _DIGEST_RE.fullmatch(result) is None:
        _fail(f"{context} must be sha256:<64 lowercase hex digits>")
    return result


def _commit(value: object, context: str) -> str:
    result = _text(value, context)
    if _COMMIT_RE.fullmatch(result) is None:
        _fail(f"{context} must be a full lowercase Git commit")
    return result


def _expected_matches(
    predicate: Mapping[str, object], expected: Mapping[str, object] | None
) -> None:
    if expected is None:
        return
    for key, expected_value in expected.items():
        if key not in predicate:
            _fail(f"predicate is missing expected field {key!r}")
        actual_value = predicate[key]
        if actual_value != expected_value:
            _fail(
                f"predicate {key} mismatch: expected "
                f"{actual_value!r} != {expected_value!r}"
            )


def validate_predicate(
    predicate: Mapping[str, object],
    *,
    expected: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Validate a closed version-1 predicate and return a detached copy."""

    raw = _mapping(predicate, "predicate")
    unexpected = sorted(set(raw) - REQUIRED_FIELDS)
    if unexpected:
        _fail(f"predicate contains unknown fields: {', '.join(unexpected)}")
    missing = sorted(REQUIRED_FIELDS - set(raw))
    if missing:
        _fail(f"predicate is missing required fields: {', '.join(missing)}")
    if (
        isinstance(raw["schema_version"], bool)
        or not isinstance(raw["schema_version"], int)
        or raw["schema_version"] != SCHEMA_VERSION
    ):
        _fail(f"predicate schema_version must be {SCHEMA_VERSION}")

    source_repository = _text(raw["source_repository"], "source_repository")
    if "/" not in source_repository:
        _fail("source_repository must be OWNER/REPOSITORY")
    _text(raw["source_ref"], "source_ref")
    snapshot_id = _text(raw["source_snapshot_id"], "source_snapshot_id")
    if _UUID_RE.fullmatch(snapshot_id) is None:
        _fail("source_snapshot_id must be a lowercase canonical UUID")
    _commit(raw["source_commit"], "source_commit")
    _sha256(raw["source_tree_digest"], "source_tree_digest")
    _sha256(raw["source_manifest_sha256"], "source_manifest_sha256")

    helper_hashes = _mapping(raw["helper_sha256"], "helper_sha256")
    for path in SOURCE_HELPER_PATHS:
        if path not in helper_hashes:
            _fail(f"helper_sha256 is missing {path}")
        _sha256(helper_hashes[path], f"helper_sha256[{path!r}]")
    for path, digest in helper_hashes.items():
        _text(path, "helper_sha256 path")
        _sha256(digest, f"helper_sha256[{path!r}]")
    if set(helper_hashes) != set(SOURCE_HELPER_PATHS):
        _fail("helper_sha256 must contain exactly the two pinned source helper files")

    _repository(raw["repository"], "repository")
    _digest(raw["image_iid"], "image_iid")
    _digest(raw["registry_digest"], "registry_digest")
    architecture = _text(raw["architecture"], "architecture")
    if architecture != "linux/amd64":
        _fail("architecture must be linux/amd64")
    _digest(raw["base_image_digest"], "base_image_digest")
    _commit(raw["isaac_sim_sha"], "isaac_sim_sha")
    _commit(raw["isaac_lab_sha"], "isaac_lab_sha")
    sim_version = _release_version(raw["isaac_sim_version"], "isaac_sim_version")
    _release_version(raw["isaac_lab_version"], "isaac_lab_version")
    lab_isaac_sim_version = _release_version(
        raw["isaac_lab_isaac_sim_version"], "isaac_lab_isaac_sim_version"
    )
    if lab_isaac_sim_version != f"{sim_version}.0":
        _fail("isaac_lab_isaac_sim_version must equal isaac_sim_version plus .0")
    if (
        raw["isaac_sim_sha"] != ACCEPTED_ISAAC_SIM_SHA
        or raw["isaac_lab_sha"] != ACCEPTED_ISAAC_LAB_SHA
        or sim_version != ACCEPTED_ISAAC_SIM_VERSION
        or raw["isaac_lab_version"] != ACCEPTED_ISAAC_LAB_VERSION
        or lab_isaac_sim_version != ACCEPTED_ISAAC_LAB_ISAAC_SIM_VERSION
    ):
        _fail("predicate does not match the accepted simulator pair")

    _expected_matches(raw, expected)
    return cast(dict[str, object], json.loads(json.dumps(raw, sort_keys=True)))


def _verified_statements(document: object) -> list[Mapping[str, object]]:
    """Return authenticated in-record statements from GitHub CLI JSON."""

    if not isinstance(document, list) or not document:
        _fail("attestation verification output must be a non-empty JSON array")
    records = document
    statements: list[Mapping[str, object]] = []
    for record in records:
        if not isinstance(record, Mapping):
            _fail("attestation verification output contains a malformed record")
        if "verified" in record and record["verified"] is not True:
            _fail("verified attestation record is explicitly unverified")
        result = record.get("verificationResult")
        if not isinstance(result, Mapping):
            _fail("attestation verification record has no verificationResult")
        if "verified" in result and result["verified"] is not True:
            _fail("verified attestation record is explicitly unverified")
        if not isinstance(record.get("attestation"), Mapping):
            _fail("verified attestation record has no attestation object")
        statement = result.get("statement")
        if not isinstance(statement, Mapping):
            _fail("verified attestation record has no in-record statement")
        statements.append(statement)
    return statements


def validate_verified_attestation(
    document: object,
    *,
    expected_predicate: Mapping[str, object] | None = None,
    predicate_type: str = PREDICATE_TYPE,
) -> dict[str, object]:
    """Validate authenticated statements from ``gh attestation verify``.

    An absent ``verified`` flag relies on successful CLI completion. Identical
    duplicates pass; contradictory statements fail.
    """

    statements = _verified_statements(document)
    if not statements:
        _fail("attestation verification output contains no statements")
    expected: Mapping[str, object] | None = None
    if expected_predicate is not None:
        expected = _mapping(expected_predicate, "expected predicate")
        unknown_expected = sorted(set(expected) - REQUIRED_FIELDS)
        if unknown_expected:
            _fail(
                "expected predicate contains unknown fields: "
                + ", ".join(unknown_expected)
            )

    canonical_predicate: dict[str, object] | None = None
    canonical_subject: tuple[str, str] | None = None
    canonical_statement: str | None = None
    for statement in statements:
        if statement.get("predicateType") != predicate_type:
            _fail("verified attestation statement has no expected predicate type")
        predicate = statement.get("predicate")
        if not isinstance(predicate, Mapping):
            _fail("verified attestation statement contains no predicate")
        validated = validate_predicate(predicate)
        subject = statement.get("subject")
        if not isinstance(subject, list) or len(subject) != 1:
            _fail("verified attestation statement must have one OCI subject")
        subject_record = subject[0]
        if not isinstance(subject_record, Mapping):
            _fail("verified attestation subject is malformed")
        subject_name = subject_record.get("name")
        digest_record = subject_record.get("digest")
        canonical_name = _repository(subject_name, "verified attestation subject.name")
        if not isinstance(digest_record, Mapping):
            _fail("verified attestation subject digest is malformed")
        if set(digest_record) != {"sha256"}:
            _fail("verified attestation subject digest must contain exactly sha256")
        canonical_digest = "sha256:" + _sha256(
            digest_record.get("sha256"),
            "verified attestation subject digest.sha256",
        )
        if canonical_name != validated["repository"]:
            _fail("verified attestation subject repository does not match predicate")
        if canonical_digest != validated["registry_digest"]:
            _fail("verified attestation subject digest does not match predicate")
        _expected_matches(validated, expected)
        identity = (canonical_name, canonical_digest)
        try:
            statement_bytes = json.dumps(
                statement,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except (TypeError, ValueError) as exc:
            raise ProvenanceError(
                f"verified attestation statement is not canonical JSON: {exc}"
            ) from exc
        if canonical_predicate is None:
            canonical_predicate = validated
            canonical_subject = identity
            canonical_statement = statement_bytes
        elif (
            validated != canonical_predicate
            or identity != canonical_subject
            or statement_bytes != canonical_statement
        ):
            _fail("attestation verification output contains contradictory statements")
    assert canonical_predicate is not None
    return canonical_predicate


def _inspect_records(document: object) -> list[Mapping[str, object]]:
    if isinstance(document, list):
        records = document
    else:
        records = [document]
    if not records or not all(isinstance(item, Mapping) for item in records):
        _fail("Docker inspect output must be a non-empty object list")
    return [item for item in records if isinstance(item, Mapping)]


def validate_image_identity(
    document: object,
    *,
    image_iid: str,
    repository: str,
    registry_digest: str,
    source_snapshot_id: str,
    source_tree_digest: str,
) -> dict[str, str]:
    """Validate a pulled Docker inspect record against the fresh build IID."""

    _digest(image_iid, "image_iid")
    _digest(registry_digest, "registry_digest")
    _repository(repository, "repository")
    _text(source_snapshot_id, "source_snapshot_id")
    _sha256(source_tree_digest, "source_tree_digest")

    records = _inspect_records(document)
    if len(records) != 1:
        _fail("Docker inspect must return exactly one image")
    record = records[0]
    observed_iid = record.get("Id")
    if observed_iid != image_iid:
        _fail(f"pulled image config IID does not match fresh IID: {observed_iid!r}")

    observed_os = record.get("Os")
    observed_architecture = record.get("Architecture")
    if observed_os != "linux" or observed_architecture != "amd64":
        _fail(
            "pulled image config platform must be linux/amd64: "
            f"{observed_os!r}/{observed_architecture!r}"
        )

    repo_digests = record.get("RepoDigests")
    if (
        not isinstance(repo_digests, list)
        or f"{repository}@{registry_digest}" not in repo_digests
    ):
        _fail(
            "pulled image RepoDigests does not contain the expected repository digest"
        )

    config = _mapping(record.get("Config"), "Docker inspect Config")
    labels = _mapping(config.get("Labels"), "Docker inspect labels")
    expected_labels = {
        "org.space-robotics-bench.source.provenance-version": "1",
        "org.space-robotics-bench.source.snapshot-id": source_snapshot_id,
        "org.space-robotics-bench.source.tree-digest": source_tree_digest,
    }
    for key, expected_value in expected_labels.items():
        if labels.get(key) != expected_value:
            _fail(f"Docker image label {key} does not match expected identity")

    result: dict[str, str] = {
        "image_iid": image_iid,
        "repository": repository,
        "registry_digest": registry_digest,
        "architecture": "linux/amd64",
    }
    label_fields = {
        "isaac_sim_sha": "org.space-robotics-bench.isaacsim.commit",
        "isaac_lab_sha": "org.space-robotics-bench.isaaclab.commit",
        "isaac_sim_version": "org.space-robotics-bench.isaacsim.release-version",
        "isaac_lab_version": "org.space-robotics-bench.isaaclab.source-version",
        "isaac_lab_isaac_sim_version": "org.space-robotics-bench.isaaclab.isaacsim-version",
    }
    for output_key, label_key in label_fields.items():
        value = labels.get(label_key)
        if not isinstance(value, str) or not value:
            _fail(f"Docker image label {label_key} is missing")
        if output_key.endswith("_sha"):
            _commit(value, label_key)
        else:
            _release_version(value, label_key)
        result[output_key] = value
    if (
        result["isaac_sim_sha"] != ACCEPTED_ISAAC_SIM_SHA
        or result["isaac_lab_sha"] != ACCEPTED_ISAAC_LAB_SHA
        or result["isaac_sim_version"] != ACCEPTED_ISAAC_SIM_VERSION
        or result["isaac_lab_version"] != ACCEPTED_ISAAC_LAB_VERSION
        or result["isaac_lab_isaac_sim_version"] != ACCEPTED_ISAAC_LAB_ISAAC_SIM_VERSION
    ):
        _fail("Docker image labels do not match the accepted simulator pair")
    return result


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ProvenanceError(f"JSON object contains duplicate key {key!r}")
        result[key] = value
    return result


def _stat_signature(metadata: os.stat_result) -> tuple[int, int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _read_bounded_regular_file(path: Path, *, label: str) -> bytes:
    """Read one bounded, regular, no-follow file and return its exact bytes."""

    try:
        path_before = os.lstat(path)
    except OSError as exc:
        raise ProvenanceError(f"cannot inspect {label} {path}: {exc}") from exc
    if not stat.S_ISREG(path_before.st_mode):
        _fail(f"{label} must be a regular file")
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ProvenanceError(f"cannot open {label} {path}: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or _stat_signature(
            before
        ) != _stat_signature(path_before):
            _fail(f"{label} changed before being read")
        if before.st_size > MAX_SOURCE_MANIFEST_BYTES:
            _fail(f"{label} exceeds the 64 MiB bound")
        try:
            raw = os.read(descriptor, MAX_SOURCE_MANIFEST_BYTES + 1)
        except OSError as exc:
            raise ProvenanceError(f"cannot read {label} {path}: {exc}") from exc
        after = os.fstat(descriptor)
        try:
            path_after = os.lstat(path)
        except OSError as exc:
            raise ProvenanceError(
                f"cannot inspect {label} after reading: {exc}"
            ) from exc
        if (
            not stat.S_ISREG(after.st_mode)
            or _stat_signature(after) != _stat_signature(before)
            or _stat_signature(path_after) != _stat_signature(before)
            or len(raw) != before.st_size
        ):
            _fail(f"{label} changed while being read")
        return raw
    finally:
        os.close(descriptor)


def _reject_json_constant(value: str) -> NoReturn:
    _fail(f"JSON contains non-finite number {value}")


def _parse_json_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        _fail(f"JSON contains non-finite number {value}")
    return parsed


def _parse_json(raw: bytes, *, label: str) -> object:
    try:
        return json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_float=_parse_json_float,
            parse_constant=_reject_json_constant,
        )
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
        RecursionError,
        ValueError,
    ) as exc:
        raise ProvenanceError(f"{label} is not valid UTF-8 JSON: {exc}") from exc


def _load_json(path: Path) -> object:
    return _parse_json(
        _read_bounded_regular_file(path, label="JSON input"), label="JSON input"
    )


def _read_manifest(path: Path) -> tuple[bytes, Mapping[str, object]]:
    raw = _read_bounded_regular_file(path, label="source manifest")
    manifest = decode_source_manifest(raw)
    return raw, manifest


def _git_object(value: object, context: str) -> str:
    result = _text(value, context)
    if _GIT_OBJECT_RE.fullmatch(result) is None:
        _fail(f"{context} must be a lowercase Git object ID")
    return result


def validate_source_manifest_identity(
    manifest: Mapping[str, object],
    *,
    source_commit: str,
    source_snapshot_id: str,
    source_tree_digest: str,
) -> None:
    snapshot_id = _text(manifest["snapshot_id"], "source manifest snapshot_id")
    if _UUID_RE.fullmatch(snapshot_id) is None or snapshot_id != source_snapshot_id:
        _fail("source manifest snapshot_id does not match the requested snapshot")
    tree_digest = _sha256(manifest["tree_digest"], "source manifest tree_digest")
    if tree_digest != source_tree_digest:
        _fail("source manifest tree_digest does not match the requested tree")
    source = _mapping(manifest["source"], "source manifest source")
    root_commit = _commit(source["root_commit"], "source manifest source.root_commit")
    if root_commit != source_commit:
        _fail("source manifest root_commit does not match source_commit")
    _git_object(source["root_tree"], "source manifest source.root_tree")
    if source["worktree"] != "clean":
        _fail("source manifest source.worktree must be clean")
    submodules = source["submodules"]
    if not isinstance(submodules, list):
        _fail("source manifest source.submodules must be a list")
    for index, item in enumerate(submodules):
        record = _mapping(item, f"source manifest submodule {index}")
        path = _text(record["path"], f"source manifest submodule {index}.path")
        gitlink = _git_object(
            record["gitlink"], f"source manifest submodule {index}.gitlink"
        )
        head = _git_object(record["head"], f"source manifest submodule {index}.head")
        _git_object(record["tree"], f"source manifest submodule {index}.tree")
        if head != gitlink:
            _fail(f"source manifest submodule {path} HEAD does not match gitlink")
        if record["worktree"] != "clean":
            _fail(f"source manifest submodule {path} worktree must be clean")


def _manifest_helpers(manifest: Mapping[str, object]) -> dict[str, str]:
    producer = _mapping(manifest.get("producer"), "source manifest producer")
    files = producer.get("files")
    if not isinstance(files, list):
        _fail("source manifest producer.files must be a list")
    result: dict[str, str] = {}
    for item in files:
        record = _mapping(item, "source manifest producer file")
        path = _text(record.get("path"), "source manifest producer file.path")
        digest = _sha256(record.get("sha256"), f"source manifest producer file {path}")
        if path in result:
            _fail(f"source manifest producer files contain duplicate path {path}")
        result[path] = digest
    return result


def build_predicate(
    *,
    source_repository: str,
    source_commit: str,
    source_ref: str,
    source_snapshot_id: str,
    source_tree_digest: str,
    source_manifest_sha256: str,
    helper_sha256: Mapping[str, str],
    repository: str,
    image_iid: str,
    registry_digest: str,
    architecture: str,
    base_image_digest: str,
    isaac_sim_sha: str,
    isaac_lab_sha: str,
    isaac_sim_version: str,
    isaac_lab_version: str,
    isaac_lab_isaac_sim_version: str,
) -> dict[str, object]:
    """Build a validated predicate from explicit release identities."""

    predicate: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "source_repository": source_repository,
        "source_commit": source_commit,
        "source_ref": source_ref,
        "source_snapshot_id": source_snapshot_id,
        "source_tree_digest": source_tree_digest,
        "source_manifest_sha256": source_manifest_sha256,
        "helper_sha256": dict(sorted(helper_sha256.items())),
        "repository": repository,
        "image_iid": image_iid,
        "registry_digest": registry_digest,
        "architecture": architecture,
        "base_image_digest": base_image_digest,
        "isaac_sim_sha": isaac_sim_sha,
        "isaac_lab_sha": isaac_lab_sha,
        "isaac_sim_version": isaac_sim_version,
        "isaac_lab_version": isaac_lab_version,
    }
    predicate["isaac_lab_isaac_sim_version"] = isaac_lab_isaac_sim_version
    return validate_predicate(predicate)


def _write_json(path: Path, value: object) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except OSError as exc:
        raise ProvenanceError(f"cannot write JSON {path}: {exc}") from exc


def _generate(args: argparse.Namespace) -> int:
    manifest_path = Path(args.source_manifest)
    manifest_bytes, manifest = _read_manifest(manifest_path)
    validate_source_manifest_identity(
        manifest,
        source_commit=args.source_commit,
        source_snapshot_id=args.source_snapshot_id,
        source_tree_digest=args.source_tree_digest,
    )
    helper_hashes = _manifest_helpers(manifest)
    for path in SOURCE_HELPER_PATHS:
        if path not in helper_hashes:
            _fail(f"source manifest does not pin helper {path}")
    predicate = build_predicate(
        source_repository=args.source_repository,
        source_commit=args.source_commit,
        source_ref=args.source_ref,
        source_snapshot_id=args.source_snapshot_id,
        source_tree_digest=args.source_tree_digest,
        source_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        helper_sha256=helper_hashes,
        repository=args.repository,
        image_iid=args.image_iid,
        registry_digest=args.registry_digest,
        architecture=args.architecture,
        base_image_digest=args.base_image_digest,
        isaac_sim_sha=args.isaac_sim_sha,
        isaac_lab_sha=args.isaac_lab_sha,
        isaac_sim_version=args.isaac_sim_version,
        isaac_lab_version=args.isaac_lab_version,
        isaac_lab_isaac_sim_version=args.isaac_lab_isaac_sim_version,
    )
    output = Path(args.output)
    _write_json(output, predicate)
    if args.json:
        print(json.dumps(predicate, sort_keys=True))
    return 0


def _validate(args: argparse.Namespace) -> int:
    predicate = _mapping(_load_json(Path(args.predicate)), "predicate")
    expected: Mapping[str, object] | None = None
    if args.expected:
        expected = _mapping(_load_json(Path(args.expected)), "expected predicate")
    validated = validate_predicate(predicate, expected=expected)
    if args.json:
        print(json.dumps(validated, sort_keys=True))
    return 0


def _verify(args: argparse.Namespace) -> int:
    document = _load_json(Path(args.attestation))
    expected: Mapping[str, object] | None = None
    if args.expected:
        expected = _mapping(_load_json(Path(args.expected)), "expected predicate")
    validated = validate_verified_attestation(
        document,
        expected_predicate=expected,
        predicate_type=args.predicate_type,
    )
    if args.json:
        print(json.dumps(validated, sort_keys=True))
    return 0


def _inspect(args: argparse.Namespace) -> int:
    document = _load_json(Path(args.inspect_json))
    result = validate_image_identity(
        document,
        image_iid=args.image_iid,
        repository=args.repository,
        registry_digest=args.registry_digest,
        source_snapshot_id=args.source_snapshot_id,
        source_tree_digest=args.source_tree_digest,
    )
    print(json.dumps(result, sort_keys=True))
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser(
        "generate", aliases=("build",), help="write a version-1 predicate"
    )
    generate.add_argument("--source-manifest", required=True)
    generate.add_argument("--source-repository", required=True)
    generate.add_argument("--source-commit", required=True)
    generate.add_argument("--source-ref", required=True)
    generate.add_argument("--source-snapshot-id", required=True)
    generate.add_argument("--source-tree-digest", required=True)
    generate.add_argument("--repository", required=True)
    generate.add_argument("--image-iid", required=True)
    generate.add_argument("--registry-digest", required=True)
    generate.add_argument("--architecture", default="linux/amd64")
    generate.add_argument("--base-image-digest", required=True)
    generate.add_argument("--isaac-sim-sha", required=True)
    generate.add_argument("--isaac-lab-sha", required=True)
    generate.add_argument("--isaac-sim-version", required=True)
    generate.add_argument("--isaac-lab-version", required=True)
    generate.add_argument("--isaac-lab-isaac-sim-version", required=True)
    generate.add_argument("--output", required=True)
    generate.add_argument("--json", action="store_true")
    generate.set_defaults(handler=_generate)

    validate = subparsers.add_parser("validate", help="validate a predicate JSON")
    validate.add_argument("--predicate", required=True)
    validate.add_argument("--expected")
    validate.add_argument("--json", action="store_true")
    validate.set_defaults(handler=_validate)

    verify = subparsers.add_parser(
        "verify", help="validate the predicate extracted from gh attestation output"
    )
    verify.add_argument("--attestation", required=True)
    verify.add_argument("--expected")
    verify.add_argument("--predicate-type", default=PREDICATE_TYPE)
    verify.add_argument("--json", action="store_true")
    verify.set_defaults(handler=_verify)

    inspect = subparsers.add_parser(
        "inspect", help="validate a pulled Docker inspect JSON record"
    )
    inspect.add_argument("--inspect-json", required=True)
    inspect.add_argument("--image-iid", required=True)
    inspect.add_argument("--repository", required=True)
    inspect.add_argument("--registry-digest", required=True)
    inspect.add_argument("--source-snapshot-id", required=True)
    inspect.add_argument("--source-tree-digest", required=True)
    inspect.set_defaults(handler=_inspect)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        return int(args.handler(args))
    except ProvenanceError as exc:
        print(f"container provenance: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
