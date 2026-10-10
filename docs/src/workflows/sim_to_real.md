# Sim-to-Real Transfer

Generate and verify a `RealEnv` bridge before running a simulation-trained policy with `srb real_agent`. This page covers adapter wiring and evaluation, not validated hardware operation; operators remain responsible for physical safety.

**Runtime and reporting limits:**

- `RealEnv` checks every adapter before actions and post-action observations. Degraded fallback frames fail.
- Transports are not physically atomic. A later send failure cannot undo earlier sends, so `RealEnv` latches and attempts safe stops.
- Kortex reports show submitted commands (Cartesian metres/radians per step; joint velocity radians/s), not measured motion or controller acknowledgements.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/BGAxdTRTG80?si=ub6p2hDG9bnTAJSK&mute=1&autoplay=1&loop=1&playlist=BGAxdTRTG80" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 1. Train a policy in simulation

Train a stable checkpoint; evaluation depends on algorithm, backend, and preflight gates. Dreamer's `train_real` fine-tuning is experimental. Example: Dreamer on `waypoint_navigation`, Leo Rover, one environment, Earth gravity:

```bash
srb agent train --headless --algo dreamer --env waypoint_navigation env.robot=leo_rover env.num_envs=1 env.domain=earth
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/ot43Q3tuIv4?si=wNcYdfcmGBPscqRD&mute=1&autoplay=1&loop=1&playlist=ot43Q3tuIv4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

See the [Reinforcement Learning Workflow](./reinforcement_learning.md).

## 2. Generate and verify the bridge

`srb real_agent gen` inspects a simulated Gymnasium environment and writes a lightweight hardware counterpart independent of the simulation backend. `--hardware` selects `HardwareInterface` drivers; for Leo Rover, `ros_cmd_vel` sends velocity, `ros_tf` supplies pose, and `ros_mw` provides pause/resume. Specify the robot so generation can extract settings such as action scaling.

```bash
srb real_agent gen --env waypoint_navigation env.robot=leo_rover --hardware ros_cmd_vel ros_tf ros_mw
```

Generation starts a temporary headless SRB session and replaces a Python module in [`sim_to_real/env`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/interfaces/sim_to_real/env). The module defines `RealEnv` with selected hardware IDs as defaults and registers under `srb_real/`; shipped waypoint code defaults to `dummy`. Find IDs with `srb real_agent gen --help` or browse [`sim_to_real/hardware`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/interfaces/sim_to_real/hardware).

Regenerate the waypoint module with your robot and hardware IDs before evaluation; shipped code uses `cadre` and `dummy`. Its deployment profile, `configure_waypoint_navigation_hardware` in `srb/interfaces/sim_to_real/deploy_profiles/multi_robot.py`, configures `ros_tf` for `robot`→`target` with 2D position and yaw, and `ros_cmd_vel` with the drive filter and simulator-scale clamps. The waypoint profile sets `required_dynamic_frame_pairs` to `robot`→`target`; these required pairs also declare observation keys. Bare `ros_tf` declares none. Verify profiles on the target robot.

Registration is lazy: importing `srb` or `srb.interfaces.sim_to_real.env` registers no `srb_real/` ID. `register_all()` sweeps generated modules, and `srb real_agent` commands call it. `srb_real/` tasks therefore do not appear in `srb ls env` or [environment pages](../envs/index.md), which list only `srb/`. Shipped modules register `srb_real/excavation`, `srb_real/formation_following`, `srb_real/terrain_landscaping`, `srb_real/terrain_landscaping_crater`, and `srb_real/waypoint_navigation`. Agent verbs (not `validate`) generate a missing module in a subprocess, then exit and ask for a re-run.

See the [`srb real_agent gen` reference](../reference/cli_real_agent_gen.md).

## 3. Deploy and evaluate on hardware

### Run policy evaluation

Check bridge routes and the production profile, then run generated `RealEnv` on configured hardware without launching simulation.

```bash
srb real_agent eval --env waypoint_navigation --algo dreamer
```

The regenerated profile routes actions through `ros_cmd_vel`/`RosCmdVelInterface` and supplies next-step pose observations through `ros_tf`/`RosTfInterface`; `ros_mw` provides middleware services.

### Runtime cleanup and validation

Runtime verbs (`eval`, `rand`, `zero`, `validate`) close the environment on Ctrl-C, SIGTERM, errors, or normal exit while keeping ROS alive. Cleanup latches the session, submits final stops, then releases an owned context. Repeated signals cannot interrupt cleanup, but a blocking transport can delay it. Successful SIGTERM cleanup exits `143`. Failed stops retain environment/context for retry; submission proves neither controller receipt nor that the robot stopped.

Validation cleans up the environment, abort watcher, and telemetry in one scope: environment closes before telemetry; ROS context releases only after successful closure. Failed closes retain resources for retry; completed closes are not repeated. ROS initializes immediately before environment construction, not on dry runs or earlier preflight failures. Compatible signal ownership also protects preflight resources. Ctrl-C/SIGTERM during rollout propagates after cleanup without guaranteeing an interrupted-rollout report. Ordinary failures report and exit `4` if cleanup succeeds; persistent cleanup failure raises and retains resources, so a report alone does not prove shutdown.

Embedding processes must invoke the CLI runtime on the main thread. An initialized default ROS context is allowed only with native handlers disabled (`SignalHandlerOptions.NO`); conflicts are rejected before hardware construction. Borrowed contexts remain active and Python handlers are restored on exit. Library-created `RealEnv` leaves process signal/context handling to the caller.

### ROS policy input

`srb real_agent ros` accepts flat policy vectors on the environment's `action/raw` topic, using generated action order and hardware mapping without simulator configuration. Missing, invalid, or stale input aborts through hardware stop methods. See the [ROS input contract](../reference/cli_agent_ros.md#realenv-raw-policy-input) for message bounds, startup/freshness timeouts, and cleanup.

### Dataset collection

Nonhuman `real_agent collect` sources (`random`, `zero`, or loaded policy) use RealEnv scalar steps and write datasets with single-environment batch axes and the `real` split. Without a success term, success-only collection uses explicit terminal-step boolean metadata; collector time limits do not make intermediate metadata a final outcome.

### Native teleoperation and human collection

Native direct teleop and human collection need generated `COMMAND_MAPPING` (declared by shipped modules); ROS is default, with SpaceMouse or haptic supported. Keyboard/gamepad input and `teleop --algo`/`--model` are rejected. See the [generation contract](../reference/cli_real_agent_gen.md#srb-real_agent-gen) and [native input details](../reference/cli_real_agent_gen.md#native-direct-input).

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/inHHDDrnjrw?si=XRYcYqmrOQqla9jg&mute=1&autoplay=1&loop=1&playlist=inHHDDrnjrw" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 4. Advanced workflows

### Zero and random agents

Use only after hardware validation and operator-controlled bring-up. `zero` sends zero policy input, which may actuate after mapping; it is not an emergency stop. `rand` sends random actions, not a safety check.

```bash
srb real_agent zero --env waypoint_navigation
```

```bash
srb real_agent rand --env waypoint_navigation
```

Dreamer real-data fine-tuning (`srb real_agent train --algo dreamer --continue`) remains experimental.

## 5. Additional task examples

### 5.1 Excavation

`excavation` targets a Husky + Kinova mobile manipulator with scoop payload. OSC and joint-velocity depend on the robot motion-generation stack. **Deployment is unsupported:** both commands raise route `ValueError` before ROS acquisition. OSC needs `ros_optitrack` for `proprio/base_pose_world_optitrack` (missing from generated `HARDWARE`), and no shipped adapter supplies `state/ee_pose_world`. Joint-velocity cannot route `robot/delta_twist`, accepted only by `ros_kortex_cartesian` (alias) and `moveit_servo`. The excavation profile configures only `ros_depth_heightmap` to the training grid: 16×16 cells at 3.75 cm, in `kinova_base_link`, centered on the default scoop target, invalid cells at 1.5 m. Shipped code omits this profile.

OSC (operational-space control):

```bash
srb real_agent eval --env excavation --algo skrl_ppo \
  --hardware ros_depth_heightmap ros_tf ros_imu ros_kortex_cartesian ros_scoop_target ros_mw
```

Joint-velocity variant:

```bash
srb real_agent eval --env excavation --algo skrl_ppo \
  --hardware ros_depth_heightmap ros_tf ros_imu ros_kortex_joint_vel ros_scoop_target ros_mw
```

| Interface | Role and limitation |
| --- | --- |
| `ros_depth_heightmap` | Projects RealSense depth into a local heightmap grid. |
| `ros_tf` | Supplies TF frame-pair observations; OptiTrack base pose additionally needs omitted `ros_optitrack`. |
| `ros_imu` | Supplies IMU angular velocity and linear acceleration. |
| `ros_kortex_cartesian` | Cartesian-admittance Kinova arm control, FK, and contact-force readings. |
| `ros_scoop_target` | Supplies policy-required scoop-target command observations. |
| `ros_mw` | Middleware pause/resume services. |
| `ros_kortex_joint_vel` | Sends Kinova joint velocities; use when an external motion-generation layer (e.g. NVIDIA fabrics) translates policy output into reactive joint targets. |

For simulation rollouts, monitor `success_rate`, `terminal_reward`, `scoop_particle_count`, and `action_smoothness`. Hardware `RealEnv` does not emit `scoop_particle_count` because it has no particle simulation.

### 5.2 Terrain Landscaping

Canonical deployment uses **RaphRover + prismatic-velocity RaphShovel** and one external heightmap. It has no onboard depth camera, IMU observation, blade-pitch channel (measured or synthetic), previous-action feedback, or Leo Rover profile.

| Task ID | Target manifest | Generated module |
| --- | --- | --- |
| `terrain_landscaping_crater` | Fixed crater; hash pinned in module | `srb/interfaces/sim_to_real/env/terrain_landscaping_crater.py` |
| `terrain_landscaping` | Per-episode mission; module sets `REQUIRES_RUNTIME_TARGET_MANIFEST = True` | `srb/interfaces/sim_to_real/env/terrain_landscaping.py` |

`terrain_landscaping_mound` is simulation-only, with no generated module. General landscaping requires its hashed per-episode mission manifest to construct deployment `RealEnv`.

Regenerate checked-in bridge modules in place, then compare each with a fresh render:

```bash
# Regenerate (writes modules in place)
srb real_agent gen --env terrain_landscaping_crater
srb real_agent gen --env terrain_landscaping

# Check byte-identical fresh renders
srb real_agent gen --env terrain_landscaping_crater --check
srb real_agent gen --env terrain_landscaping --check
```

See [`--check`](../reference/cli_real_agent_gen.md#--check--the-freshness-gate) for the drift check.

Generation derives modules through `env.srb_spec` from the frozen IO contract; it emits no `np.finfo` dtype-extrema bounds. The schema has **3 normalized velocity actions**: `robot/cmd_vel` `[0:2]` (linear, angular) and `payload/joint_vel` `[2:3]`; plus **7 finite-bound actor observation leaves** (1287 `float32` values, in contract order): `proprio_dyn/heightmap_current_global`, `proprio_dyn/heightmap_target_global`, `proprio_dyn/heightmap_current_local`, `proprio_dyn/heightmap_target_local`, `proprio/mission_pose`, `proprio/base_velocity`, `proprio_dyn/heightmap_age_s`. `SINGLE_OBSERVATION_SPACE` assembles four map outputs and one packed `vector`, not seven top-level outputs. Deployment identity constants are `ENV_ID`, `IO_SCHEMA_FINGERPRINT`, `TARGET_MANIFEST_SHA256`, `REQUIRES_RUNTIME_TARGET_MANIFEST`, and `DEPLOYMENT_REQUIREMENTS`; `target_map_sha256`, `reset_layout_sha256`, `manifest_sha256`, and `io_schema_fingerprint` are distinct hashes.

**Capability preflight blocks deployment.** Before ROS acquisition, hardware start, or policy loading, canonical landscaping `RealEnv` checks the union of tags claimed by hardware interfaces and capability providers. All four are required: `raph.drive_velocity`, `raph.shovel_prismatic_velocity`, `landscaping.external_heightmap_batch`, and `landscaping.observable_task_evaluator`. `srb.interfaces.sim_to_real.validation.landscaping.LandscapingRealEvaluator` is the only shipped claimant (evaluator tag). Pass it as `RealEnv(..., task_evaluator=...)`; it grades accepted external frames against the verified target manifest, owns reward, and rejects model artifacts with mismatched task ID, IO-schema fingerprint, target manifest, reset layout, or normalization policy before ROS, hardware start, or inference. Other tags have no shipped claimants, so construction raises typed `DeploymentNotReadyError` listing them. Deployment needs Raph ROS drive/shovel adapters and a live external heightmap mapper claiming those tags; SRB ships none.

`hyperparams/validation/terrain_landscaping_crater.yaml` declares no hardware and fails capability preflight (exit `4`). Metrics are `success_rate`, `final_grading_mae_m`, `final_fraction_within_tolerance`, `action_smoothness`, and `map_age_s`; final-step info adds central-ROI/local MAE, sustained success, four distinct hashes, and calibration version. See [Terrain Landscaping Training](./terrain_landscaping_training.md) for learners, observation contract, and limitations. TD-MPC2 is unsupported for both IDs and fails before expensive side effects.

## 6. Validation Workflow

### 6.1 Capture a simulation baseline

Evaluate each checkpoint once in simulation:

```bash
srb agent eval --env excavation --algo skrl_ppo --model <ckpt> \
    env.num_envs=1
```

Use `--model` and Hydra override `env.num_envs=1` for this simulator baseline; the validation spec does not configure the run. It configures hardware `n_episodes` and `.rrd` recording; `srb real_agent validate --wandb` enables W&B. The harness reads baseline scalars (`success_rate`, `terminal_reward`, `action_smoothness`, etc.) from `sim_baseline.metrics_json` or, failing that, a W&B run ID (see [Validation spec YAML](#62-validation-spec-yaml)). `srb agent eval` does not write this file; provide a flat JSON object of finite numbers for every tracked metric (real-only metrics such as `success_rate` need no baseline).

### 6.2 Validation spec YAML

The spec defines the contract among the operator, generated `RealEnv`, and harness. CLI flags override selected runtime fields; YAML holds task, policy, hardware, criteria, telemetry, baseline, and drift settings.

| Field | Type | Required | Default / limit |
| --- | --- | --- | --- |
| `spec_id` | string or null | no | `null` |
| `task` | string | yes | required |
| `algo` | string | yes | required |
| `checkpoint` | string | yes | required |
| `hardware` | list[string] | yes | required |
| `metrics_to_track` | list[string] | yes | required |
| `n_episodes` | integer | no | `10` |
| `max_episode_seconds` | float seconds | no | `120.0` |
| `thresholds` | object | no | see below |
| `rerun` | object | no | see below |
| `sim_baseline` | object | no | see below |
| `drift_window` | integer sessions | no | `5` |
| `storage_root` | path | no | `logs/real_validation` |
| `notes` | string | no | `""` |

`checkpoint` accepts a raw checkpoint or SRB model artifact path; `notes` is copied to the spec snapshot.

| Threshold option | Type | Default / limit |
| --- | --- | --- |
| `thresholds.success_rate_min` | float | `0.5`; minimum real success rate |
| `thresholds.sim_real_ratio_min` | float | `0.7`; floor for higher-is-better real/sim ratios |
| `thresholds.lower_is_better_metrics` | list[string] | `[]`; names lower-is-better metrics; error/loss names auto-detected |
| `thresholds.action_smoothness_max_ratio` | float | `1.5`; max smoothness ratio |
| `thresholds.crash_rate_max` | float | `0.0`; max crash/safety-event rate |
| `thresholds.drift_z_max` | float | `3.0`; max z-score across recent sessions |

Lower-is-better real/sim ratios cap at `1/sim_real_ratio_min`; add metric names to `lower_is_better_metrics`.

| Rerun option | Type | Default |
| --- | --- | --- |
| `rerun.app_id` | string | `srb-validation` |
| `rerun.web_port` | integer | `9090` |
| `rerun.grpc_port` | integer | `9876` |
| `rerun.bind_address` | string or null | `0.0.0.0` |
| `rerun.keep_historical_data` | bool | `true` |
| `rerun.keep_scalar_history` | bool | `true` |
| `rerun.record_to_rrd` | bool | `true`; saves `rollout.rrd` |

| Simulation baseline option | Type | Default |
| --- | --- | --- |
| `sim_baseline.wandb_run_id` | string or null | `null`; W&B run with sim summary scalars |
| `sim_baseline.local_rrd` | path or null | `null`; not parsed for metrics; as the only source it yields a degraded baseline |
| `sim_baseline.metrics_json` | path or null | `null`; local metrics JSON, tried before W&B |

### 6.3 Run validation on the real robot

```bash
srb real_agent validate \
    --spec hyperparams/validation/excavation.yaml \
    --checkpoint <real-or-shared-ckpt> \
    --episodes 10 \
    --storage-root /tmp/srb_validation
```

Flags: `--spec`, `--checkpoint`, `--episodes`, `--wandb`, `--no-tensorboard`, `--no-rerun-web`, `--storage-root`, `--post-status`, `--dry-run`. Artifacts and TensorBoard are local by default; W&B is opt-in.

For graceful abort from another process, touch the exact `.abort` path logged at information level. It is in the session's unique workspace beside private `.staging`:

```bash
touch '/exact/session/workspace/.abort'
```

Programmatic callers can read `ValidationHarness.abort_request_path`; information logging exposes the CLI path. Checks occur between reset, policy, and step calls: `external_abort` latches, stops later episodes, and records an incomplete failed session; an active operation may finish first. This is not an emergency stop or controller watchdog. Cleanup removes unpublished `.abort`. Inherited `SRB_ABORT=1` also cancels; exporting it in another shell cannot affect a running process.

**Dry runs never pass:** `--dry-run` skips environment and policy construction, then finalizes with `pass_overall: false`, `hardware_evidence: false`, `exit_code: 4`, `metrics.n_episodes: 0`; all three criteria are `skipped`, the badge is red with `non-hardware evidence`, and the session is excluded from drift history.

The harness starts Rerun at `http://localhost:9090`, runs `--episodes` rollouts through generated `RealEnv`, writes session and storage-root artifacts, and checks:

1. **Threshold:** success floor and sim/real ratios pass. The success floor applies without a baseline; with a positive `success_rate_min`, zero successes cannot pass.
1. **Crash:** no `policy_nan`, `hardware_event`, `disk_full`, or safety abort (`sensor_stale`, `sensor_incomplete`, `calibration_mismatch`, `manifest_mismatch`).
1. **Drift:** tracked-metric z-scores are at most `thresholds.drift_z_max` over the last K sessions.

Horizon endings are `time_limit_truncation` with final metrics, not wall-clock `timeout`. Out-of-MDP abort reasons/counts (`sensor_stale`, `sensor_incomplete`, `calibration_mismatch`, `manifest_mismatch`, `operator_abort`) are neither task termination nor truncation. Operator stop fails the success floor, not crash criterion.

| Edge case | Result |
| --- | --- |
| Incomplete rollout (`n_episodes` or `--episodes`), out-of-MDP abort, or `SRB_ABORT` (`external_abort`) | (a) fails with counts/reasons; safety abort also fails (b). The session is excluded from drift history. |
| Baseline comparison | Direction-aware around `|sim|`: higher-is-better requires `real >= sim - (1 - sim_real_ratio_min) * |sim|`; lower-is-better requires `real <= sim + (1 / sim_real_ratio_min - 1) * |sim|`. `action_smoothness` uses its max ratio. Positive baselines reduce to ratio rules; zero makes the band undefined and fails (a). |
| Drift history | Requires two earlier sessions; less history or no comparable metric skips optional (c). `drift_window < 2` disables drift. |
| Post-rollout evaluation error (malformed baseline, missing metric) | Invalid session; exit `4`, `hardware_evidence: false`; report published. |
| Unreachable W&B baseline | Degraded baseline; (a) fails, exit `1`, `hardware_evidence: false`. `success_rate` is real-only. |

Exit codes:

| Code | Name / criterion | Meaning |
| --- | --- | --- |
| `0` | `pass` / all | Threshold, crash, and drift pass. |
| `1` | `threshold_fail` / `criterion_a` | Threshold metrics fail, e.g. success rate or sim/real ratio. |
| `2` | `crash_fail` / `criterion_b` | Crash or safety-relevant termination. |
| `3` | `drift_fail` / `criterion_c` | Tracked metrics exceed z-score limit. |
| `4` | `preflight_fail` / preflight | Rollout cannot start (e.g. missing generated code, hardware wiring, incompatible checkpoint, or dry-run), or post-rollout evaluation fails; `hardware_evidence: false`. |

Validation runs only generated `srb_real/<task>` environments; unresolved IDs fail with exit `4`, never falling back to `srb/<task>` Isaac tasks. Shipped specs:

| Spec under `hyperparams/validation/` | Module generated? | `validate --spec` result |
| --- | --- | --- |
| `excavation.yaml` | yes | Exit `4` preflight: generated environment cannot construct; see [excavation](#51-excavation). |
| `formation_following.yaml` | yes | Runs. |
| `terrain_landscaping_crater.yaml` | yes | Exit `4` preflight: no hardware declared; no Raph adapters ship. |

### 6.4 Validation artifacts

```text
logs/real_validation/
+-- _index.jsonl
+-- index.html
+-- <task>/
    +-- <uuid>_<utc-timestamp>/
        +-- current.json
        +-- generations/
            +-- <generation-uuid>/
                +-- spec.yaml
                +-- episodes.jsonl
                +-- rollout.rrd
                +-- tensorboard/
                +-- summary.json
                +-- report.md
                +-- badge.json
```

| Artifact | Scope; writer | Produced when | Purpose |
| --- | --- | --- | --- |
| `spec.yaml` | Session; `write_spec_snapshot` | Preflight | Rebuild-summary input. |
| `episodes.jsonl` | Session; `Telemetry` | Rollout | Append-only step, event, and episode-end metrics. |
| `rollout.rrd` | Session; `Telemetry` | `rerun.record_to_rrd=true` and Rerun starts | Inspection and twin replay. |
| `tensorboard/` | Session; `Telemetry` | Enabled and available | Optional scalar logs. |
| `summary.json`, `report.md`, `badge.json` | Session; `write_session_report` | Finalization or rebuild-summary | Machine result, report, Shields.io payload. |
| `_index.jsonl` | Storage root; `DriftStore` | Finalization | Append-only drift history/dashboard rows. |
| `index.html` | Storage root; `write_validation_index` | Finalization | Cross-session pass/fail and metrics dashboard. |

Each run regenerates `logs/real_validation/index.html`; trend charts group by `(task, algo, metric)`. Compare the failed session's simulation ghost (transparent) with its real run (solid), aligned by episode:

```bash
srb real_agent replay-twin \
    --session logs/real_validation/excavation/<uuid>_<ts>/generations/<generation-uuid> \
    --baseline baselines/excavation/<sha>.rrd
```

If a crash leaves `summary.json` missing, rebuild it from `spec.yaml` and `episodes.jsonl`:

```bash
srb real_agent rebuild-summary logs/real_validation/excavation/<uuid>_<ts>/generations/<generation-uuid>
```

The harness records estop, joint-limit, and contact-force events for criterion (b), but does not enforce safety or replace deadman switches, joint-limit watchdogs, or estop circuitry.

## 7. Creating custom hardware interfaces

Add a Python file under [`sim_to_real/hardware`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/interfaces/sim_to_real/hardware), subclass `HardwareInterface`, declare `supported_action_spaces`/`supported_observation_spaces` (used for routing), and implement `apply_action` (send commands), `sync` (refresh readings), and `observation`; override `start`/`close` (calling `super()`) to connect and clean up. `_template.py` is an example. Discovery exposes it to `--hardware`.

Next: [Terrain Landscaping Training](terrain_landscaping_training.md)
