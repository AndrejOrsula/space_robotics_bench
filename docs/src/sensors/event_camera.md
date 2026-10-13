# Event Camera (IMX636)

Configure `EventCameraCfg`, enable a task camera, and record with `srb agent --record-events`; use `srb dataset record-events` for synthetic data.

```bash
srb agent zero --headless --env velocity_tracking --record-events 'env.active_sensors=[proprioception,contact,transform,events]'
```

SRB models the Prophesee IMX636 (EVK4). Per-pixel brightness changes produce
microsecond-timestamped events for high-dynamic-range, fast-motion scenes such
as lunar shadows, direct sunlight, and docking. Synthetic events are not
calibrated against EVK4 data.

The standalone sensor is not a `Camera` subclass. It has its own pose,
intrinsics, high-rate renderer, IMX636 preset, and per-env RNG. Patched
`v2e_streaming` is signal-only and is included in the default image only when
the image is built with the provider patch option ([Runtime producer bounds](#runtime-producer-bounds)).

## Configuration

`EventCameraCfg` configures the sensor. `BaseEnvCfg` creates its native RGB
camera in Isaac; a scene binding retains the Isaac-free config and adds
observations.

```python
from srb.core.sensor.event_camera import EventCameraCfg

cfg = EventCameraCfg(
    prim_path="/World/robot/ee/event_cam",
    resolution=(1280, 720),
    sub_render_hz=250.0,
    preset_name="imx636_nominal",
    backend="v2e",
    representation="voxel_grid",
    rng_seed=None,
)
```

| Field | Meaning |
|---|---|
| `resolution: (W, H)` | Render size before crop/downsample; native IMX636 `(1280, 720)`. |
| `roi_crop: (x, y, w, h)?`; `downsample: int` | Pre-backend crop and integer downsampling factor; affects observations and stored events. |
| `preset_name: str` | `IMX636Preset`, default `imx636_nominal`; explicit threshold/noise fields override it. |
| `sub_render_hz: float` | Tick cap, default 250 Hz; see [Render-rate guidance](#render-rate-guidance). |
| `representation: str`; `time_bins: int` | `voxel_grid`, `histogram`, `time_surface`, `stacked`, or `raw`; grid defaults to T=5. |
| `event_chunk_rows: int` | Transport rows/chunk, default 65,536; capped by `provider_workspace_rows` and 1,000,000. |
| `observation_window_events: int?` | Per-step retained-event/memory cap. `None` = `4·W·H` at final geometry (3,686,400 rows, about 118 MB/env/sensor at 32 bytes/event for 1280×720). Overflow fails closed. |
| `provider_workspace_rows: int` | Provider timestamp/event/permutation workspace cap, default 1,048,576; separate from transport chunks. |
| `backend: str` | Registered `EventGenerator` name. |
| `backend_device: str?` | Torch provider device (v2e, V2CE); `None` selects `cuda` if available, else `cpu`. |
| `v2ce_model_path: str?` | Local V2CE checkpoint; a V2CE backend without one fails at construction. |
| `hot_pixel_rate_hz: float` | Per-hot-pixel rate from preset `hot_pixel_frac` mask (200 Hz default). Each tick injects Poisson ON events, capped at 64 per started `1 / sub_render_hz` period. Rate is metadata; first post-reset reference frame has none. |
| `rng_seed: int?` | Base seed; reset mixes env index and episode ID. `None` uses run seed (`env.cfg.seed`, via `torch.initial_seed()`). Name salts sensor seed collisions; backends use private RNGs. |

## Presets

Built-in presets (`srb.core.sensor.event_camera.preset`):

| Preset | `pos_thres` | `neg_thres` | `sigma_thres` | `cutoff_hz` | `shot_noise_rate_hz` | Use case |
|---|---|---|---|---|---|---|
| `imx636_nominal` | 0.20 | 0.20 | 0.03 | 50 | 5 | Generic indoor / orbital lighting. |
| `imx636_low_light` | 0.20 | 0.20 | 0.05 | 20 | 20 | Lunar permanently-shadowed regions, deep night. |
| `imx636_outdoor_sunlit` | 0.20 | 0.20 | 0.02 | 70 | 2 | Direct-sun Mars / lunar daylight. |
| `clean` | 0.20 | 0.20 | 0.00 | 0 | 0 | Noise-free reference. |

Register a custom preset:

```python
from srb.core.sensor.event_camera.preset import (
    IMX636Preset, register_preset,
)

register_preset(IMX636Preset(
    name="excavation_dust",
    pos_thres=0.18, neg_thres=0.18, sigma_thres=0.04,
    cutoff_hz=40.0, leak_rate_hz=0.15,
    shot_noise_rate_hz=12.0, refractory_period_s=5e-4,
    hot_pixel_frac=2e-4,
))
```

Presets are not calibrated against EVK4. Illumination increases in order:
`imx636_low_light`, `imx636_nominal`, `imx636_outdoor_sunlit`. Keep hardware
properties `refractory_period_s` and `hot_pixel_frac` constant when retuning.

## Backends, extras, licenses

Select the backend with `cfg.backend = "<name>"`.

| Backend | Module | Extra | License | Notes |
|---|---|---|---|---|
| `noop` | `srb.core.sensor.event_camera.backend.NoopBackend` | Built-in | MIT | `supports_batching=False`; emits zero events for tests and does not consume RGB frames. |
| `v2e` | `srb.core.sensor.event_camera.backends.v2e.V2eBackend` | `srb[event-v2e]` + pinned source | MIT (v2ecore) | Default, `supports_batching=False`; IMX636 noise, no interpolation, private RNG, episode-relative time. |
| `v2e_streaming` | `srb.core.sensor.event_camera.backends.v2e.V2eStreamingBackend` | `srb[event-v2e]` + pinned source + local patch | MIT (v2ecore) | Bounded signal-only `EventEmulator.iter_events`; no shot-noise ordering claim. |
| `metavision` | `srb.core.sensor.event_camera.backends.metavision.MetavisionBackend` | `srb[event-metavision]` | Apache-2.0 (openeb) | `supports_batching=True`; `GPUEventSimulator` from `metavision_core_ml.event_simulator`. |
| `v2ce` | `srb.core.sensor.event_camera.backends.v2ce.V2CEBackend` | `srb[event-v2ce]` | MIT | `supports_batching=True`, learned torch simulator. Uses `v2ce.inference.V2CE` or `ucsd-hdsi-dvs/V2CE-Toolbox` (`scripts.v2ce_3d` + `scripts.LDATI`). Fetch `hf-andrejorsula-v2ce`; set `v2ce_model_path` or `model_path=<path>` or construction fails. |

### Env-runtime batched dispatch

`EventBackendDispatch` checks `factory.supports_batching` at construction.
`noop`/`v2e` use one `_per_env` generator per `(env_id, sensor)`, fed on
refresh. `metavision`/`v2ce` share one `_shared` adapter/sensor. On each tick,
dispatch calls `process_batch(rgb_batch, t_ns)` and routes outputs to env
buffers. No env mask is used: stale slots are `None`. Adapters own temporal
state and lazily
allocate a simulator/model per env. Reset calls
`reset_env(env_id, seed=…, episode_id=…)` for that slice. Batched adapters
must override it or dispatch raises `ValueError`; per-env adapters default to
`reset()` and private-RNG reseeding. Both paths share buffers, manifests, HDF5.

> **AGPL forbidden.** AGPL-licensed event repositories (notably
> `ev-ultralytics`) MUST NOT appear in any SRB extra or be vendored into the
> tree.

Install the extra and pinned v2e source:

```sh
pip install 'srb[event-v2e]' 'v2e @ git+https://github.com/SensorsINI/v2e.git@cac638444557a224ba6ee5c29260916f152c00bd'
```

v2e and reconstruction E2VID are Git-pinned because PyPI rejects direct
references, so extras omit them. In checkouts run
`uv sync --all-extras --group source-event-v2e`; `source-event-reconstruction`
carries E2VID and `vcs-sources` selects all pinned sources.

Missing backend extras or pinned sources raise `MissingExtraError` with the
exact `pip install` command.

## Representations

`cfg.representation` selects the fixed-shape RL tensor:

| Representation | Output shape | Description |
|---|---|---|
| `voxel_grid` | `(B, T, H, W)` float32 | Polarity-signed `time_bins` grid (default). |
| `histogram` | `(B, 2, H, W)` float32 | Polarity-separated event counts. |
| `time_surface` | `(B, 2, H, W)` float32 | Exponentially decayed age of last event, per polarity. |
| `stacked` | `(B, 2T, H, W)` float32 | Per-polarity voxel grid stacked along channel. |
| `raw` | `(B, N, 4)` int64 + `(B,) int64` counts | Padded raw event list (length `raw_max_events`). |

`B` is env count, `T = cfg.time_bins`, and `H/W` are effective resolution
(after ROI and downsample).

## Render-rate guidance

Each new frame is sent once at simulation time; frames less than
`1 / cfg.sub_render_hz` apart are skipped. The cap cannot raise render rate.
Shipped tasks render once per decimated env step; increase render rate for more
ticks. Pinned v2e spreads events over the prior-frame interval, so long gaps
lose intermediate motion. Manifest/HDF5 `sub_render_hz` is measured cumulative
frame rate (configured cap until frame two). Recommended minimum rates:

| Task class | Recommended render and event tick rate |
|---|---|
| Static / slow-manipulation (e.g. inspection) | 100 Hz |
| Mobile robotics (rover traverse) | 250 Hz (`sub_render_hz` default) |
| Excavation, drilling, scoop impact | 500 Hz |
| Landing touchdown, docking contact | 1000 Hz |

## Recording during rollouts

1. Enable `events` in `env.active_sensors` (alias of `event_camera`). Tasks with
   an event camera spawn it then: front (`velocity_tracking`,
   `waypoint_navigation`), onboard (`landing`, `moonfall`, `zerog_lab`), or
   wrist (`excavation`, `sample_collection`). Without it, `--record-events`
   fails because the scene has no event camera.

1. Run `srb agent` with `--record-events` (`zero`, `rand`, `teleop`, `ros`,
   `train`, `eval`, `collect`). `BaseEnvCfg` adds the camera, `DirectEnv` binds
   it, and the recorder writes one file per env/episode/sensor under
   `<run logdir>/camera_videos/<timestamp>/` unless `env.camera_record_dir` is set.

Live observations always use the configured backend, ROI, downsampling,
representation, and validation. HDF5/manifest recording needs `camera_record`
or `event_record_requested`; RGB MP4 needs `camera_record`. `--record-events`
sets both. Add `visual` to `env.active_sensors` for paired RGB (`rgb_mp4_path`):

```bash
srb agent zero --headless --env velocity_tracking --record-events 'env.active_sensors=[proprioception,contact,transform,visual,events]'
```

Backends that require RGB fail closed without native RGB. `noop` accepts
missing RGB and emits zero events.

Known `v2e_streaming` task limits:

- `landing` and `moonfall` (onboard, downward camera): random rollouts can
  record episodes without events; v2e logs `no signal events generated` on
  every frame, and a longer `landing` rollout can reach the per-episode
  event-observation element limit while still empty.
- `excavation` (wrist camera): depending on the rollout, finalizing an episode
  can exceed the default `observation_window_events` bound (3,686,400 rows).
  Closing the environment raises that failure, so collection does not report
  success without the episode.
- Very short episodes can exceed the same bound at reset on other tasks too,
  for example `velocity_tracking` with `env.episode_length_s=2`; its default
  20 s episodes stay within it.

For synthetic recordings without Isaac, run `srb dataset record-events`; its
`EVENT_HDF5` output works with storage, validation, and export tools.

This command records a deterministic moving-patch fixture, not task or sensor
events.
Provenance reserves `task_id="fixture_moving_patch"`,
`policy_id="synthetic_fixture"`, empty `isaaclab_version`, and detected SRB
revision. `--task` sets summary task; `--policy-id` is compatibility-only.
Summary source is `synthetic_fixture`. Detection requires the exact task ID;
missing markers/metadata or backend names alone do not identify fixtures.

LeRobot, Zenodo, and NPZ warn and preserve the fixture marker and task/policy
identity. LeRobot/Zenodo set `contains_synthetic_fixture: true` if any selected
item is a fixture, including mixed sets. `to_zenodo_bundle` ignores
`extra_metadata` overrides of that flag/identity. Statistics/reconstruction
scorecards warn and save `source`, `task_id`, `policy_id`.

1. Pass `--seed INT` to seed the synthetic environment/event-camera run:

```bash
srb dataset record-events --output events --backend noop --seed 42
```

Without it, the process run seed applies. The recorder derives backend seeds
per env/episode and records them in manifest `seed`.

## Dataset layout

Files use `StorageFormat.EVENT_HDF5`. The writer streams into a private
staging file (HDF5 SWMR when available, else a warning) and publishes the
final path only when the episode closes, so readers never see a partial file.
It prefers Blosc:zstd-3 compression, falling back to gzip-6.

The synthetic `srb dataset record-events` path produces a timestamped
output directory under `--output`; its sensor is named `event_camera`:

```
20260521T140312/
├── events_manifest.jsonl
├── events_env0_ep0/
│   └── events_event_camera_env0_ep0.h5
├── events_env0_ep1/
│   └── events_event_camera_env0_ep1.h5
├── events_env1_ep0/
│   └── events_event_camera_env1_ep0.h5
├── events_env1_ep1/
│   └── events_event_camera_env1_ep1.h5
└── ...
```

`event_hdf5_path` and `rgb_mp4_path` are relative to the timestamped run
directory and point into the env/episode directory.

`srb agent --record-events` uses the same layout with a concrete backend and
the task's sensor name (e.g. `event_camera_front`). `rgb_mp4_path` pairs
`event_camera_<name>` with the co-located camera's `image_<name>_rgb` clip and
is empty when that clip is absent; another camera's clip or a depth clip is
never paired.

### HDF5 layout

```
/events/t    int64    ns           (N,)   chunked, compressed
/events/x    uint16                (N,)
/events/y    uint16                (N,)
/events/p    int8     ±1            (N,)
/metadata    group    attrs:
                       schema_version, sensor_preset, resolution,
                       sub_render_hz, backend_name, backend_version,
                       backend_commit_sha, representation, time_bins,
                       episode_id, policy_id, policy_checkpoint_hash,
                       task_id, seed, srb_git_sha, isaaclab_version,
                       rgb_mp4_path (optional), success_flag, reward_summary,
                       compression, event_count, event_sha256,
                       dropped_batches, writer_telemetry,
                       hot_pixel_mask, hf_repo_ids   (JSON-encoded),
                       hot_pixel_rate_hz
                       (optional; absent means unknown)
```

Schema v1 is frozen; breaking changes bump `EVENT_HDF5_VERSION` and require a
converter and republish.

Before assignment, the writer checks attribute count, UTF-8/JSON bytes, and
cumulative storage against fixed ceilings. Rejection aborts its private stage.

### Manifest JSONL

One row per `(env_id, episode_id, sensor_name)`:

```json
{
  "sensor_name": "event_camera_onboard",
  "env_id": 0,
  "episode_id": 0,
  "task_id": "landing",
  "seed": 42,
  "event_hdf5_path": "events_env0_ep0/events_event_camera_onboard_env0_ep0.h5",
  "rgb_mp4_path": "",
  "event_count": 184321,
  "event_sha256": "<64 hex digits>",
  "success_flag": 1,
  "reward_summary": "{\"total\": 2.5}",
  "srb_git_sha": "deadbeef",
  "isaaclab_version": "2.x.y",
  "backend_name": "v2e",
  "backend_version": "1.5.0",
  "backend_commit_sha": "",
  "sensor_preset": "imx636_nominal",
  "resolution": [1280, 720],
  "representation": "voxel_grid",
  "time_bins": 5,
  "sub_render_hz": 250.0,
  "policy_id": "ppo_landing_v3",
  "policy_checkpoint_hash": "abc123…"
}
```

Manifest rows and HDF5 `/metadata` share `EventEpisodeProvenance`. Rows also
carry routing (`sensor_name`, `env_id`, `event_hdf5_path`, `event_count`,
`event_sha256`). Both
store `hot_pixel_mask` and `hf_repo_ids` (JSON in HDF5 attrs).

`reward_summary` `total` and `final_reward` use the (optionally normalized)
reward the agent receives; `terms` holds the raw per-term values of the final
step.

`srb_git_sha`, `isaaclab_version`, `backend_version`,
`policy_checkpoint_hash`, and `seed` describe configuration, not deterministic
replay. Dreamer directory SHA-256 covers sorted member paths, sizes, and file
hashes; symlinks are recorded, not followed. Empty/unreadable dirs yield an
empty hash and warning.

Random/human collection records its controller as `policy_id`. Strict
validation permits empty checkpoint hashes for `random`, `human`, `zero`,
`rand`, `teleop`, `ros`, `synthetic_fixture`; other policies require one.
Missing SRB or Isaac Lab provenance fails. `srb_git_sha` is the Git revision
of the SRB checkout, which images also include. It identifies build source,
not unchanged runtime files.
`isaaclab_version` is the installed package version and may differ from pinned
source.

### Reading events

```python
from srb.core.sensor.event_camera import read_event_hdf5

ev = read_event_hdf5("events_env0_ep0/events_event_camera_env0_ep0.h5")
ev["t"], ev["x"], ev["y"], ev["p"]      # numpy arrays
ev["metadata"]                           # dict of attrs
```

`read_event_hdf5` checks row count, decoded column sizes, and cumulative bytes
before materializing. Over-limit reads fail without truncation. Iterate larger
files instead:

```python
from srb.core.sensor.event_camera.storage import iter_event_hdf5

for chunk in iter_event_hdf5("events_env0_ep0/events_event_camera_env0_ep0.h5", 4096):
    consume(chunk["t"], chunk["x"], chunk["y"], chunk["p"])
```

Chunks respect requested rows and the repository batch ceiling; final count
and digest are validated without assembling the file in memory.

## Runtime producer bounds

`event_chunk_rows` caps transport chunks: dispatch slices oversized one-shot
results and rejects oversized streaming yields. With a writer sink, dispatch
writes each validated chunk synchronously before pulling again, propagating
backpressure to genuine iterators without retaining a result map. Batched
iterators yield exactly one entry per env. Iterator, shape, cardinality, sink,
or size failures fail that env and abort its staged writer.

Streams must progress: per-env yields cannot be empty; batched yields may have
idle envs but need events for one active env. End the iterator for an empty
interval or dispatch raises `event_camera.stream_no_progress` to stop an
unbounded empty-yield loop.

`strict_streaming=True` rejects backends without declared support and a
dispatch iterator override (`iter_frame` per-env, `iter_batch` batched).
Upstream v2e, Metavision, and V2CE return complete arrays/lists, not streams.
`v2e_streaming` uses the patch in
`srb/core/sensor/event_camera/backends/v2e_pin.toml`. Install the pinned base,
then apply/check the patch from the SRB wheel:

```bash
python -m srb.core.sensor.event_camera.backends.v2e_install --apply
python -m srb.core.sensor.event_camera.backends.v2e_install --check
```

Docker uses `--build-arg SRB_EVENT_V2E_STREAMING=true`. The patch bounds
timestamp-vector, per-iteration event-matrix, and permutation allocations by
`provider_workspace_rows`; `event_chunk_rows` caps yields. Set
`shot_noise_rate_hz=0`. Construction/reset require the imported
`v2ecore.emulator` digest to match the patch and provider photoreceptor noise,
CSDVS, and SCIDVS off. It emits signal chunks and resets on cancellation. If
the recorder rejects a chunk (e.g. writer backpressure timeout), it closes the
iterator and rebuilds the emulator. Unpatched materialized output is not
strict streaming; shot-noise streaming is unsupported.

## Async writer

`EventHDF5Writer` defaults to async daemon-thread writes so env steps stay
non-blocking:

- Bounded `queue.Queue(maxsize=256)` between env and writer threads.
- Saturation raises `EventHDF5BackpressureError` and aborts the affected staged
  episode. Published datasets never silently drop event batches.
- `close()` enqueues a sentinel and joins the thread, draining everything
  still queued.

Force synchronous mode (tests, deterministic CI):

```python
from srb.core.sensor.event_camera import EventHDF5Writer

with EventHDF5Writer(path, metadata, async_writes=False) as w:
    w.append(events)
```

## Validation

Run CLI validation on demand, outside the inner loop:

| Check | Command | What it checks |
|---|---|---|
| Statistics | `srb dataset validate-events stats` | Event rate, polarity ratio, ISI distribution vs. real Gen4. |
| Reconstruction | `srb dataset validate-events reconstruction` | Reconstructed frame SSIM (and optional LPIPS) vs. source RGB. |

Both need explicit EVENT_HDF5 inputs, e.g. `srb dataset validate-events stats --synth synthetic.h5 --real recorded.h5 --output stats.json`. Fetch reference captures by URL
or fixture mirror; SRB does not commit them.

`stats` requires equal declared native resolutions; `--resolution` fills only
missing metadata. Out-of-grid events fail. Per-pixel rate distance is exact
1-D Wasserstein over counts/duration-seconds, allowing unequal stream lengths;
non-positive duration yields `NaN`. Scorecards include `on_count` and
`off_count`: infinite ON/OFF (no OFF) is JSON `null`; no events is undefined.
`--sparsity-window-ms` must be finite and at least 1 ns.

Reconstruction-fidelity:

```sh
srb dataset validate-events reconstruction \
  --events events.h5 \
  --rgb-npy frames.npy \
  --times-npy frame_times_ns.npy \
  --output recon_card.json \
  --operator baseline      # 'baseline' (pure-numpy) or 'e2vid' (gated)
  # --lpips                  # optional, requires srb[event-reconstruction]
```

`baseline` bins events between successive RGB timestamps and min-max
normalises per-pixel polarity sums (pure numpy, always available). `e2vid`
requires `srb[event-reconstruction]`. Fetch its registered checkpoint
mirror with
`srb event-camera fetch-weights --mirror hf-andrejorsula-e2vid` before using
the operator. SRB uses the MIT `e2vid` provider from `uzh-rpg/e2calib` and
does not vendor GPL `rpg_e2vid` code.

When an input carries the exact moving-patch fixture marker, reconstruction
scoring warns and persists its source, task, and policy identity in the
scorecard. `validate-events stats` does the same for the marked stream in a
pairwise comparison. Marker-free and metadata-free inputs are not
inferred to be fixtures.

## Model weights

SRB does not redistribute E2VID or V2CE checkpoints.
`srb event-camera fetch-weights` downloads from a registered mirror or explicit
URL with its SHA256 and aborts on a digest mismatch. See
[Weight mirrors](event_camera_weight_mirrors.md) for fetch commands, mirror
names, hashes, and how to add a mirror.

## Publishing datasets

```sh
# LeRobot v3 (HuggingFace Hub / behaviour-cloning pipelines)
srb dataset export-lerobot-v3 <event_hdf5_dir> --output ./bundles/lerobot

# Zenodo archival deposit
srb dataset export-zenodo-bundle <event_hdf5_dir> --output ./bundles/zenodo.tar.gz
```

LeRobot and Zenodo warn and preserve fixture markers. RAW EVT3 and ROS bag
reject fixtures because they cannot preserve those markers. Import ordinary task
recordings with explicit provenance:

```sh
srb dataset import-raw-evt3 landing.raw \
  --output landing.h5 \
  --task landing \
  --policy-id prophesee_landing
srb dataset export-rosbag landing.h5 --output ./landing_bag
```

Never export `record-events` output to RAW EVT3 or ROS bag: it always carries
the reserved fixture marker, regardless of `--task`/`--policy-id`.

LeRobot v3 has no canonical event-camera schema; bundles use SRB's extended
`event_camera` modality:

```
bundle/
├── meta/
│   ├── info.json            # codebase_version v3.0 + srb_event_camera schema
│   ├── stats.json
│   ├── tasks.parquet
│   └── episodes/chunk-000/file-000.parquet  # one row per (env, episode, sensor)
├── data/chunk-000/file-000.parquet          # one row per episode, event pointers
└── events/<sensor_name>/chunk-000/episode-{n:06d}.h5  # canonical EVENT_HDF5
```

`info.json` adds `event_path` beside `data_path`/`video_path` and an
`srb_event_camera` block (schema version, registered sensors, presets,
backends, resolutions). Episode parquet copies manifest provenance
(`srb_git_sha`, `policy_checkpoint_hash`, `backend_version`, `seed`,
`sub_render_hz`, etc.) for filtering without HDF5 reads; these fields do not
prove deterministic replay.

`fps` is the maximum recorded `sub_render_hz` (measured event tick rate) across
rows. Mixed-task bundles (e.g. landing 1000 Hz + traversal 250 Hz) declare the
highest rate as temporal resolution.

## RAW EVT3 import

Normalize Prophesee RAW EVT3 into canonical EVENT_HDF5 without Metavision/OpenEB:

```sh
srb dataset import-raw-evt3 active_marker.raw \
  --output active_marker.h5 \
  --task active_marker \
  --policy-id prophesee_source
```

The importer requires a `% format EVT3` or `% evt 3.0` header line, decodes
EVT3 address/vector CD events, unwraps 24-bit timestamps
to nanosecond `t`, and stores source resolution in `/metadata/resolution`. It
skips/counts words before the first `EVT_TIME_HIGH` (mid-stream files), prints
`skipped_leading_words`, and warns if nonzero. Non-empty files without it fail
`missing_time_high`. It ignores bit 11 of `EVT_ADDR_Y` (system type). After the
first `EVT_TIME_HIGH`, trigger, marker, and continued words have no CD events
and are ignored; malformed structure fails closed. The SRB header
`% srb_t0_ns <digits>` is added to each timestamp; any other line naming that
key is rejected.

`export-raw-evt3` (also without Metavision/OpenEB) and `export-rosbag` rebase
to the first event, floored to
microseconds, and record origin in `% srb_t0_ns` (ROS `time_base` uses that
origin); late-start episodes export correctly. EVT3 timing floors to its
microsecond precision. Gaps over 60 s between consecutive events fail.

## References

- Module: `srb/core/sensor/event_camera/`
- v2e — Hu et al., "v2e: From Video Frames to Realistic DVS Events".
- Prophesee IMX636 / EVK4 product page and datasheet.
