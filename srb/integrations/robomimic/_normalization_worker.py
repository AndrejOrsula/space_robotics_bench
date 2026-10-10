"""Adapter for the bounded dataset normalization worker."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import BinaryIO


def _normalize(
    stream: BinaryIO, limits: dict[str, int], decoded: int, batch: int
) -> bytes | None:
    from srb.integrations.robomimic import normalization

    normalization.MAX_ARRAY_BYTES = limits["max_array_bytes"]
    normalization.MAX_ARRAY_ELEMENTS = limits["max_array_elements"]
    normalization.MAX_DIRECTORY_ENTRIES = limits["max_entries"]
    normalization.MAX_EXTERNAL_FILE_BYTES = limits["max_output_bytes"]
    normalization.MAX_METADATA_AGGREGATE_BYTES = limits["max_metadata_bytes"]
    normalization.MAX_METADATA_ATTRIBUTE_BYTES = limits["max_attribute_bytes"]
    normalization.MAX_METADATA_ATTRIBUTES = limits["max_attributes"]
    return normalization._transform_materialized_hdf5(
        stream, max_decoded_bytes=decoded, max_batch_bytes=batch
    )


def main() -> int:
    # Derive the import root from this entrypoint, not request data.
    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root))
    spec = importlib.util.spec_from_file_location(
        "_srb_hdf5_native_worker", root / "srb/datasets/_hdf5_native_worker.py"
    )
    if spec is None or spec.loader is None:
        return 2
    worker = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = worker
    spec.loader.exec_module(worker)
    return int(worker.main(normalizer=_normalize))


if __name__ == "__main__":
    raise SystemExit(main())
