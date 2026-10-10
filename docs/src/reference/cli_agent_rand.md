# `srb agent rand` — Random Agent

Run an environment with randomly sampled actions to exercise its response to varied inputs.

## Usage

```bash
srb agent rand --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

No additional options are available for this subcommand.

## Example

Run the random agent in the `_aerial` template environment:

- Hide most of the simulation UI
- Use the `body_acc` action mode (default for the Ingenuity rotorcraft)

```bash
srb agent rand -e _aerial --hide_ui env.robot.action_mode=body_acc
```
