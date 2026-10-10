# `srb agent ros` — ROS 2 Agent

## Usage and options

```bash
srb agent ros --env ENV_ID [options]
```

See [Shared Agent Options](cli.md#shared-agent-options).

| Option | Default | Behavior |
| --- | --- | --- |
| `--ros_sync`/`--ros-sync` | `false` | First step may use the available action; later steps wait for a fresh one. Reset requests apply after the next step, so publish an action after requesting a reset. Use for closed-loop RL, LQR, or MPC. |
| `--ros_action_timeout`/`--ros-action-timeout` SECONDS | Wait indefinitely | Limit fresh-action waits with `--ros_sync`; on timeout, clear cached commands and apply zero. |
| `--ros_unthrottled`/`--ros-unthrottled` | `false` | Disable non-sync pacing and run sample-and-hold as fast as possible. |

Steps use the latest action at `env.agent_rate`. Slower publishers repeat cached actions. `env.agent_rate` must be finite and positive; otherwise SRB rejects it. If unset, the loop is unpaced.

Subscriptions use best-effort, latest-only QoS (`KEEP_LAST`, depth 1) to prevent backlogs. Commands expire after **0.5 seconds of monotonic wall time**, including during sync waits. Each message renews only its environment/action slices. Velocity and event inputs expire separately; events cannot renew velocity. Malformed or non-finite messages do not renew commands.

Mapped actions must be finite. SRB clamps them to batched Box bounds, which must admit zero. Raw and term overrides own their addressed slices until expiry or reset. Velocity and event traffic updates only mapped or unclaimed slices. Fully overridden updates change no action slice but still refresh the velocity and event caches.

Reset, expiry, and buffer publication share a lock. A contended nonblocking read returns zero. SRB does not apply cleared, expired, or rejected samples. Embedders can set `RosInterface(env, command_timeout_s=0.25)` or disable expiry with `None`. The CLI wait timeout is separate.

Accessors return a borrowed device buffer. The ROS agent follows this pattern; embedders sharing reset or close callbacks should too:

```python
with interface.action_transaction() as (action, sequence):
    step_result = env.step(action)
```

Do not use the buffer after exit. Non-transactional accessors require serialized reads and consumption. Nested reads are rejected, and same-thread reset clears the buffer. If close finds active consumption, ingress and endpoints remain for retry after it ends. CUDA work completes before buffer clearing and scope release. A hung consumer can delay reset. Keep polling; zero policy action is not a watchdog or emergency stop.

## RealEnv raw policy input

### Send raw actions

`srb real_agent ros --env ENV_ID` accepts flat policy actions on `action/raw` in the environment namespace, normally `/srb/action/raw`. Publish `std_msgs/Float64MultiArray` values in generated-environment action order and bounds. RealEnv applies hardware mapping and scales. Simulation term, velocity, and event mappings do not define real teleoperation semantics.

QoS is best-effort, volatile, keep-last, depth one. Callbacks run independently of stepping. Messages have no producer timestamps. Age starts at callback receipt and continues through sensor checks and wrapper processing. Later messages cannot extend it.

Missing, malformed, out-of-bounds, or expired commands abort the session. Hardware interfaces stop themselves; invalid input is never replaced with zero. Cleanup stops the session, then closes the command source, RealEnv, and process-owned ROS context. Failed cleanup retains resources for retry.

Checks cannot interrupt blocked hardware calls or act as a watchdog. Hardware operation is not validated; test physical stopping under supervision.

### Settings

| Setting | Default | Behavior |
| --- | --- | --- |
| Initial input wait | 5 seconds | Require valid first message before dispatch. |
| Command lifetime | 0.5 seconds | Check age before dispatch and each prepared hardware command. |
| `--ros_action_timeout` SECONDS | Unset | Positive finite value overrides wait and lifetime. |
| `--ros_sync`/`--ros-sync` | `false` | First step may use current sample; later steps await newer sample, aborting on expiry. |
| `--ros_unthrottled`/`--ros-unthrottled` | `false` | Does not bypass expiry or RealEnv action rate. |

## Examples

See [ROS 2 Workflow](../workflows/ros2.md).
