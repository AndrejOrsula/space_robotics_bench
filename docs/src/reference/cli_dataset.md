# `srb dataset` — Manage Dataset Artifacts

Manage SRB datasets in canonical JSONL, robomimic HDF5, LeRobot-style layouts, or real-validation telemetry.

## Usage

```bash
srb dataset <action> [options]
```

## Actions

| Action | Purpose |
| :--- | :--- |
| `info` | Print manifest, artifact hash and action-schema summaries (JSON). |
| `validate` | Check manifest, files and declared physical counts; `--deep` checks rows/tables; `--strict` makes warnings errors. |
| `compatibility` | Report learning, collection/eval or publication compatibility; `--algo ALGO` focuses on one algorithm, otherwise reports the matrix. |
| `export` | Convert `srb_jsonl` to `srb_jsonl`, `robomimic_hdf5` or `lerobot`, and `robomimic_hdf5` to `srb_jsonl` or `lerobot`; `lerobot` is an export target only. |
| `pull` | Download from Hugging Face Hub by `--repo-id` or `--task`/`--embodiment`/`--action-mode`; `--dry-run` plans without network access. |
| `push` | Upload to Hugging Face or plan with `--dry-run`. |
| `card` | Write a Hugging Face `README.md` dataset card. |
| `replay` | Summarize canonical data or preview in Rerun (`--backend rerun`). |
| `from-validation` | Convert real validation data to canonical SRB (`split=real`). |

See [Event Camera](../sensors/event_camera.md) for event capture, validation and export actions: `record-events`, `validate-events` (`stats`, `reconstruction`), `export-lerobot-v3`, `export-zenodo-bundle`, `export-raw-evt3`, `import-raw-evt3` and `export-rosbag`.

For fixture generation, seed/provenance markers, export restrictions, explicit task provenance, LeRobot/Zenodo mixed selections, scorecards, and RAW EVT3/ROS bag behavior, see [Event Camera recording](../sensors/event_camera.md#recording-during-rollouts).

NPZ exports warn and preserve fixture provenance.

For task data, import with explicit provenance. Example: `srb dataset import-raw-evt3 INPUT.raw --output INPUT.h5 --task landing --policy-id POLICY`.

`export-zenodo-bundle` preserves manifest-row JSON text. It normalizes surrounding whitespace and line endings only. Derived Hub repo ids go only in `zenodo_metadata.json`, so bundles pass SRB validation.

Missing `--citation-md` files fail before writes. `--doi` adds a DOI to metadata and README. Without `--doi`, no DOI is added. `export-rosbag` writes `metadata.yaml` and `bag_0.db3` in `--output`.

## Algorithm Compatibility

`srb dataset compatibility DATASET --purpose train --algo ALGO` reports implemented training input.

`training_dataset_mode` is `offline`, `replay_prefill`, or `unsupported`. Collection, eval, and publication compatibility does not imply demonstration-training support.

Unsupported focused training returns `compatible=false` with exit 1. Omit `--algo` for the matrix.

Checks cover manifest metadata and storage. The trainer checks payload, action, and observation semantics.

## Publication Format

Publish as `srb_jsonl` (preferred) or `lerobot` (optional data dependencies).
Convert training-oriented `robomimic_hdf5` before `push`.

### Admission and file references

Limits: 100,000 entries, 32 directory levels, 16 GiB per file, and 16 GiB total member bytes.

`.srb.json` selects the manifest and referenced file or directory. Sibling files are excluded. Referenced directories set traversal depth.

External manifests use one entry and share the byte budget. Embedded manifests count once. LeRobot `data` selects the format root, which includes sibling `meta`.

References stay relative to the manifest dataset root. Robomimic sidecars store the data file relative to the sidecar, so a run directory stays valid after it is moved or copied.

A legacy absolute reference that no longer exists resolves to the same-named entry beside the manifest. Absolute/external references are readable, but external sidecar refs cannot be contained uploads or strict demonstration replay inputs.

Validation, export, and replay read verified private copies. Caller changes cannot alter reads, and replacement during copy is rejected. Unversioned directories lack atomic multi-file writer generations.

Provenance and summaries retain original paths. Temporary copies are released when the operation or iterator closes.

### Conversion and export limits

Robomimic HDF5 payloads must be direct. Validation rejects external or symbolic links, virtual datasets, and external raw storage. Internal hard links are supported. Sidecars may reference directly stored HDF5 elsewhere.

Canonical conversion uses a temporary disk row-location index, up to 16 GiB. It preserves numeric episode and stable step order without retaining whole episodes.

HDF5 exports declare demo array shapes and dtypes before samples. LeRobot discovers the full column schema before Parquet batches. Canonical JSONL lines, including the newline, are limited to 16 MiB. Resource-limit failures preserve published output.

### LeRobot timing and Parquet

LeRobot infers type and size per value. It is slow for image-heavy data; use `srb_jsonl`.

LeRobot v3 requires a finite, positive nominal source `fps`. `TrajectoryRecorder` accepts an optional `fps` keyword and stores it in the SRB sidecar.

Fractional rates work. Missing timing causes an error; it is not inferred. Unknown-rate JSONL/Robomimic data remain readable and convertible.

Timestamps are float32 seconds (`frame_index / fps`). The frame index resets per episode. Global `index` values are contiguous. Episode `dataset_from_index`/`dataset_to_index` values match the `data_from`/`data_to` aliases.

Deep validation checks fields, timing, order, and counts. It retains physical row counts on feature-dtype mismatch.

Parquet uses bounded schema, sizing, and write passes. One decoded Arrow ceiling covers frame, episode, and task tables. Released rows and batches do not accumulate as live memory.

Preflight rejects oversized products before opening a writer. The product limit is 16 GiB; the array limit is 2 GiB.

### Cards and upload behavior

Fresh publishable exports include `README.md`. `card` defaults to returning its verified card without rewriting.

To create a missing card, provide external `--output`. `push` can generate one privately.

`push --dry-run` plans a bounded manifest/file inventory without payload copies. It neither deep-validates nor guarantees upload success.

Actual push validates a held private snapshot through upload. Caller-owned bytes stay unchanged.

### Replay and JSON input limits

For canonical JSONL, `--max-episodes` counts distinct first-seen `episode_index` values across files. It retains later continuations in stream order. Interleaving may require scanning unselected rows. `--max-steps` limits yielded rows.

Present keys per episode index must agree. Replay summaries reject action-dimension changes and bound episode, observation-key, success, and action-schema bookkeeping.

Canonical JSONL and strict demonstration readers structurally check JSON before parsing. Each document allows at most 1,000,000 containers, keys, and scalar values. Maximum depth is 64, with the root at zero.

Duplicate entries count even if only the last is parsed. Strings and escaped delimiters follow JSON rules. Programmatic whole-read lines are limited to 256 MiB; normal lines are limited to 16 MiB. These limits apply to input, not process memory.

APIs that retain full episodes or transitions share 16,000,000 nodes across rows and files. Streaming export applies cumulative byte limits and admits rows separately.

Manifests, LeRobot JSON, and Robomimic contract/environment metadata use this decoder. Native training shares the allowance across manifests, environment metadata, and normalized inputs. Unchanged owned metadata is reused.

### LeRobot validation

Parquet reads reject unsupported Arrow extensions and ambiguous dotted paths before decoding. Conservative footer bounds may reject variable-width or nested rows that would otherwise fit.

Validation checks row shapes, primitive compatibility, and termination agreement. Manifest roots may name slash-separated leaves. Omitted dtypes and unknown shapes remain unspecified. Extra provider columns are allowed. `meta/stats.json` is a bounded JSON object.

Nullable columns and structured `object` observations work, including exporter `[1]` for scalar observations and bookkeeping. Actions and non-scalar arrays match the declared rank.

Fixed shapes describe non-null values. Nulls provide no shape or value evidence. Object arrays keep outer dimensions and may contain nullable mappings. Termination flags must be non-null booleans.

### Robomimic validation

Validation and export check physical scalar types before count and success attributes.

Known HDF5 text fields (`env_name`, `env_kwargs`, `env_args`, `action_schema`) use bounded Linux decoding: 64 MiB per value and 128 MiB aggregate metadata. Fixed- and variable-length strings retain their values.

Bounded Linux workers inspect HDF5 for count-only and deep validation. Count-only checks counts and present masks. Deep validation also checks demos, observations, and termination.

Ordered diagnostics and observation keys share a finite report allowance. Exhaustion fails rather than returning partial success.

### Merge admission and behavior

Merge rejects inputs when demos differ in action dimension or observation keys, or when `demo_io_contract`, `action_schema`, or `env_name` differs.

The output keeps the first input's `env_kwargs` and the shared `demo_io_contract` and `action_schema`. Merge validates private input copies and prepares the full private output before staging. Invalid input creates no destination parent.

Publication is atomic. Pre-promotion failure preserves prior output. A shared copy allowance admits fixed-width extra demo members and declared chunks.

Copied scalar variable-length text counts encoded bytes against per-attribute and shared metadata limits, not descriptors. Short strings, exact fits, and following empty values or inputs remain valid.

Initialized-stage `copy_merged_hdf5` admits each input first. The caller discards a failed private stage. Uncopied unknown attributes use descriptor-only admission.

Scalar text is materialized in the finite worker before byte checks, without a per-attribute limit. Other variable-length attribute and dataset heaps are unchecked.

### Native worker limits

Linux workers run native inspection and merge admission/copy under these ceilings, including startup:

- Address space: 2 GiB.
- Output file: 16 GiB.
- Time: 120 seconds.

`Hdf5NativeLimits` may lower these limits with `merge_datasets(..., native_limits=...)`.
Verify cleanup and output identity before publication.

These ceilings limit damage, but admitted arrays may still exceed native overhead.
Lower source-byte limits apply before parent hashing and copying and are shared
across inputs.

Merge selects sorted `demo_*` and builds fresh masks. `train_split` must be in
`[0, 1]`; positive splits retain a training demo.

Merge uses the first metadata result, even if empty. It regenerates `env_args`
and drops `action_schema` and original attribute storage. Generated attributes
must fit lowered metadata limits before private output. Oversized regenerated
`env_args` fails.

Arbitrary HDF5 reference-graph fidelity is not guaranteed. Some decoder heaps are
unbounded. Install h5py in the selected interpreter. Metadata workers ignore
caller `PYTHONPATH`, `PYTHONHOME`, and user site.

### Publication cleanup errors

`PublicationCommitCleanupError` may follow commitment if private-stage/marker
cleanup fails. `committed_paths` lists destinations. `retry_cleanup()` retries
without merge/output replacement.

Cleanup identities persist, and foreign replacements stay untouched. If merge
and input cleanup fail pre-promotion, `PublicationCleanupError` retains the
original failure and pending stage/input owners. Paths are empty if nothing was
committed.

Typed native/metadata cleanup errors retain process/source copies for retry.
Context exit/GC retain pending copies. Snapshots cannot enter another consumer
context.

### Training and recorder configuration

Native Robomimic training retains verified copies. It admits configured fields,
masks, metadata, and dataset allocations before provider reads. Per-demo
`model_file` uses bounded metadata decoding.

See [offline learning](cli_agent_learn.md#dataset-admission-and-normalization)
for cache, worker, and normalization behavior.

`TrajectoryRecorder` retains observation/action copies. It rejects invalid or
oversized rows before adding. Bounded-batch save raises `ResourceLimitError`
above `max_demos`.

Keyword-only `max_decoded_bytes`, `max_batch_bytes`, `max_metadata_bytes`, and
`max_file_bytes` lower defaults of 16 GiB, 256 MiB, 128 MiB, and 16 GiB. Arrays,
config, and rows share allowances.

Construction captures the action declaration, environment metadata, observation
keys, action dimensions, provenance, and certification. Caller changes or
public diagnostic edits cannot reconfigure it.

Present HDF5 environment metadata/schema must match before append/finalize (JSON
key order is ignored). If absent, dimensions/observation roots must match.
Append validates masks; finalize may rebuild. Nested paths/shapes agree across
observations, next observations, and episodes.

Present SRB sidecar must match identity, data target, and captured
config/provenance. Stale counts/splits may rebuild. Manifest JSON is
sorted/indented with a newline; maximum size is 64 MiB.

### Recorder publication and decoder limits

Append/finalization modify private copies under a publication lock. Each
save/finalize copies the current HDF5, so I/O grows over time. Pre-promotion
failure preserves the previous file/episode for retry.

After HDF5 commit, the episode is consumed and counts advance even if cleanup
fails. Cleanup retry must not append it again.

Finalization prepares the sidecar before HDF5 promotion and serializes both
writes with cooperating writers. HDF5 and sidecar publish separately.
Post-promotion sidecar failure is reported and retryable with `finalize()`.

Native HDF5 decoder allocations outside metadata, inspection, merge and normalization workers need separate limits, including recorder append reads. Encoded-storage checks on other attributes do not bound later consumers.

## Action Metadata

Actions are flat vectors for framework compatibility. Grouped actions add
`metadata.action_schema`, which records ordered component names, paths, shapes,
dimensions, and flat offsets.

`info` exposes `action_dim`, `action_components`, and `action_schema`. Export
preserves schema in canonical JSONL, robomimic HDF5, and LeRobot.

Validation checks `metadata.action_dim`. LeRobot copies schema to
`meta/info.json` as `srb_action_schema`; deep validation checks manifest
agreement. Cards include a compact table. Replay summaries print schema;
Rerun logs `actions/<component>`.

Validation always reconciles declared episode/sample/action counts with physical JSONL, HDF5 or Parquet lengths. If a reader (e.g. `h5py`, pandas Parquet backend) is unavailable, count validation fails explicitly; unchecked data is never reported valid.

## Demonstration I/O Metadata

Robomimic collection records `metadata.demo_io_contract` v`srb.demo_io.v1`.
It contains the observation projection, ordered action components/dimensions/
bounds, and available runtime scales. Export preserves it in the canonical
manifest.

Each observation row has its term `shape`; simulated depth is `uint8`. See
[collection output](cli_agent_collect.md#output) for layout/limits.

Deep validation requires declared leaf term shape, capable dtype, and in-range
values. `obs`/`next_obs` leaves must agree. HDF5
`data.demo_io_contract` must equal the manifest.

LeRobot `meta/info.json` carries `srb_demo_io_contract`, implied image views as
`srb_visual_views`, and matching feature shapes. Collection smoothing/stacking
config is under `env_wrappers`. Recorded actions are pre-smoothing policy
commands.

Certified rows require explicit `next_obs`, `terminated`, and `truncated`.
Nested observation leaves/shapes must match next observations.

Certified LeRobot v3 exports carry transition extension
`srb.lerobot.transitions.v1`. Each action row pairs `observation.*` and
`next_observation.*` features, including the terminal endpoint. The manifest and
`meta/info.json` share the `srb_transition_extension` path mapping.

Component-wise percent encoding distinguishes literal dots, slashes, and
percent signs from nested paths. Strict validation checks the mapping, paired
shapes, physical counts, and three Boolean flags. SRB replay reconstructs
original nested observations.

LeRobot 0.4 stores one-element features as scalars. Certified exports retain
original shapes in manifest `obs_shapes` (`[]` vs `[1]`) for replay. Feature
ranks above five are rejected before publication.

Stock-consumer compatibility applies to the pinned LeRobot 0.4 reader. Endpoint
columns do not ensure every upstream policy consumes demonstration transitions.

Uncertified exports omit the transition extension. Markerless LeRobot replay
does not invent next observations or termination flags.

Data without a demo contract remains readable, but conversion cannot add a
contract or recover unrecorded termination flags. Strict replay checks metadata
against its live wrapper before inserting transitions.

## Examples

```bash
srb dataset validate /data/srb_excavation --deep --strict
srb dataset export /logs/excavation/collect/datasets/demos.hdf5 \
  --output /data/srb_excavation_policy --format srb_jsonl
srb dataset replay /data/srb_excavation_policy \
  --backend rerun --save /tmp/srb_excavation.rrd
srb dataset push /data/srb_excavation_policy --dry-run
srb dataset pull --task excavation --embodiment kinova_gen3 \
  --action-mode smooth_osc --dry-run
srb dataset pull --repo-id space_robotics_bench/srb_excavation_kinova_gen3_smooth_osc \
  --output /data/srb_excavation_policy
srb dataset from-validation logs/real_validation/excavation/<session> \
  --output /data/srb_excavation_real \
  --embodiment kinova_gen3 --action-mode smooth_osc
```

`pull --output` requires an absent or empty directory. Non-empty destinations
are refused. Downloads never merge revisions.

`from-validation` sets `split=real`. Sessions with a validation
algorithm/checkpoint default to `controller=policy`; other sessions default to
`real`.

`--embodiment`/`--action-mode` default to `real_hardware`/`real`.
`--controller` choices: `human`, `policy`, `random`, `zero`, `real`.
