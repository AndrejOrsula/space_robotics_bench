"""Pipeline: per-rock fetch -> cleanup -> USD writer."""

from __future__ import annotations

import json
from pathlib import Path

import trimesh

from srb.terrain._fetch import download_to as _download_to
from srb.terrain._fetch import sha256_file as _sha256
from srb.terrain.errors import ChecksumMismatchError
from srb.terrain.rocks.catalog_loader import load_rocks_catalog
from srb.terrain.rocks.mesh_cleanup import cleanup_rock
from srb.terrain.rocks.schema import Rock
from srb.terrain.rocks.usd_writer import write_rock_usd
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROCKS
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublishedGeneration,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_MESH_FACES,
    MAX_MESH_VERTICES,
    checked_shape_nbytes,
    require_file_size,
)

_ROCK_IDENTITY_SCHEMA = "srb-terrain-rock-identity/1"
_ROCK_ALGORITHM_VERSION = 1
_ROCK_PUBLICATION_DIR = ".rock.publication"
_ROCK_IDENTITY_MEMBER = "identity.json"
_ROCK_USD_MEMBER = "rock.usd"
_ROCK_REQUIRED_MEMBERS = (_ROCK_IDENTITY_MEMBER, _ROCK_USD_MEMBER)
_GENERATION_PENDING_MEMBER = ".srb-publication.pending"
_VALID_BODIES = frozenset(("moon", "mars", "earth"))
MAX_ROCK_WIRE_BYTES = 256 * (1 << 20)


def _validate_mesh_resource_bounds(mesh: trimesh.Trimesh, *, rock_id: str) -> None:
    """Reject decoded mesh arrays above finite process ceilings."""

    checked_shape_nbytes(
        mesh.vertices.shape,
        mesh.vertices.dtype,
        resource=f"rock {rock_id!r} vertices",
        max_elements=MAX_MESH_VERTICES * 3,
        max_bytes=MAX_ARRAY_BYTES,
    )
    checked_shape_nbytes(
        mesh.faces.shape,
        mesh.faces.dtype,
        resource=f"rock {rock_id!r} faces",
        max_elements=MAX_MESH_FACES * 3,
        max_bytes=MAX_ARRAY_BYTES,
    )
    if int(mesh.vertices.shape[0]) > MAX_MESH_VERTICES:
        raise ValueError(f"rock {rock_id!r} vertex count exceeds {MAX_MESH_VERTICES}")
    if int(mesh.faces.shape[0]) > MAX_MESH_FACES:
        raise ValueError(f"rock {rock_id!r} face count exceeds {MAX_MESH_FACES}")


def _canonical_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _rock_identity(
    *,
    catalog_version: int,
    set_name: str,
    rock_id: str,
    rock: Rock,
    body: str,
) -> dict[str, object]:
    """Return the complete semantic input identity for one generated rock."""
    return {
        "algorithm_version": _ROCK_ALGORITHM_VERSION,
        "body": body,
        "catalog_version": catalog_version,
        "rock": rock.model_dump(mode="json"),
        "rock_id": rock_id,
        "schema": _ROCK_IDENTITY_SCHEMA,
        "set_name": set_name,
    }


def _decode_identity(payload: bytes) -> dict[str, object]:
    """Validate the stored identity before comparing it with a request."""
    try:
        parsed = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise PublicationError("rock identity is not valid JSON") from exc
    if not isinstance(parsed, dict):
        raise PublicationError("rock identity must be a JSON object")
    if _canonical_json(parsed) != payload:
        raise PublicationError("rock identity is not canonical JSON")

    expected_keys = {
        "algorithm_version",
        "body",
        "catalog_version",
        "rock",
        "rock_id",
        "schema",
        "set_name",
    }
    if set(parsed) != expected_keys:
        raise PublicationError("rock identity has an invalid shape")
    if parsed["schema"] != _ROCK_IDENTITY_SCHEMA:
        raise PublicationError("rock identity has an unsupported schema")
    for key in ("algorithm_version", "catalog_version"):
        value = parsed[key]
        if isinstance(value, bool) or not isinstance(value, int):
            raise PublicationError(f"rock identity has an invalid {key}")
    for key in ("body", "rock_id", "set_name"):
        if not isinstance(parsed[key], str):
            raise PublicationError(f"rock identity has an invalid {key}")
    if parsed["body"] not in _VALID_BODIES:
        raise PublicationError("rock identity has an invalid body")

    raw_rock = parsed["rock"]
    if not isinstance(raw_rock, dict):
        raise PublicationError("rock identity has an invalid rock")
    try:
        validated_rock = Rock.model_validate(raw_rock)
    except (TypeError, ValueError) as exc:
        raise PublicationError("rock identity has invalid rock fields") from exc
    if validated_rock.model_dump(mode="json") != raw_rock:
        raise PublicationError("rock identity rock fields are not canonical")
    return parsed


def _write_rock_to_stage(stage_path: Path, mesh: trimesh.Trimesh, *, body: str) -> None:
    write_rock_usd(stage_path, mesh, body=body)
    if stage_path.is_symlink() or not stage_path.is_file():
        raise PublicationError("rock USD writer did not produce a regular stage")


def _reject_orphaned_generation(root: Path) -> None:
    """Do not turn a missing pointer over existing generations into a miss."""
    if not root.is_dir() or (root / "current.json").exists():
        return
    generations = root / "generations"
    if generations.is_dir() and any(
        not (child.is_dir() and (child / _GENERATION_PENDING_MEMBER).is_file())
        for child in generations.iterdir()
    ):
        raise PublicationError(
            f"rock publication has generations but no current pointer: {root}"
        )


def _bake_one(
    rock_id: str,
    rock: Rock,
    out_dir: Path,
    *,
    body: str,
    set_name: str | None = None,
    catalog_version: int = 0,
) -> Path:
    """Bake one rock into an immutable generation and return its USD member."""
    publication_root = out_dir / _ROCK_PUBLICATION_DIR
    _reject_orphaned_generation(publication_root)
    publication = ArtifactPublication.generation(publication_root)
    identity = _rock_identity(
        catalog_version=catalog_version,
        set_name=out_dir.parent.name if set_name is None else set_name,
        rock_id=rock_id,
        rock=rock,
        body=body,
    )

    def build(staging: Path) -> None:
        (staging / _ROCK_IDENTITY_MEMBER).write_bytes(_canonical_json(identity))
        raw = staging / f".raw.{rock.format}"
        try:
            _download_to(rock.url, raw, max_bytes=MAX_ROCK_WIRE_BYTES)
            require_file_size(
                raw,
                resource=f"rock {rock_id!r} source mesh",
                max_bytes=MAX_ROCK_WIRE_BYTES,
            )
            actual = _sha256(raw)
            if actual.lower() != rock.sha256.lower():
                raise ChecksumMismatchError(
                    url=rock.url, expected=rock.sha256, actual=actual
                )

            # The checksum gate is deliberately before mesh decoding/use.
            mesh = trimesh.load(str(raw), force="mesh")
            if not isinstance(mesh, trimesh.Trimesh):
                raise TypeError(
                    f"trimesh.load(force='mesh') returned {type(mesh).__name__}, "
                    f"expected Trimesh (rock id={rock_id!r})"
                )
            _validate_mesh_resource_bounds(mesh, rock_id=rock_id)
            cleaned = cleanup_rock(mesh, target_face_count=rock.target_face_count)
            _write_rock_to_stage(staging / _ROCK_USD_MEMBER, cleaned, body=body)
        finally:
            raw.unlink(missing_ok=True)

    def reuse_if(current: PublishedGeneration) -> bool:
        stored_identity = _decode_identity(
            current.read_member_bytes(_ROCK_IDENTITY_MEMBER)
        )
        return stored_identity == identity

    published = publication.publish_or_reuse(
        build,
        required_members=_ROCK_REQUIRED_MEMBERS,
        reuse_if=reuse_if,
        repair_invalid=True,
    )
    return published.path / _ROCK_USD_MEMBER


def bake_rock_set(set_name: str) -> list[Path]:
    """Bake every rock in ``set_name``. Returns list of rock.usd paths."""
    catalog = load_rocks_catalog()
    if set_name not in catalog.sets:
        raise KeyError(
            f"unknown rock set {set_name!r}; available: {sorted(catalog.sets)}"
        )
    rs = catalog.sets[set_name]
    out: list[Path] = []
    for rock_id, rock in rs.rocks.items():
        out_dir = SRB_ASSETS_DIR_SRB_ROCKS / set_name / rock_id
        out.append(
            _bake_one(
                rock_id,
                rock,
                out_dir,
                body=rs.body,
                set_name=set_name,
                catalog_version=catalog.version,
            )
        )
    return out
