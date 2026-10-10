# `srb agent zero` — Zero Agent

Run an environment with zero-valued actions. If zero is the equilibrium action, internal dynamics alone drive the environment.

## Usage

```bash
srb agent zero --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

No additional options are available for this subcommand.

## Example

Run the zero agent in the `_ground` template environment:

- Create 4 parallel environment instances
- Use the Cassie bipedal robot

```bash
srb agent zero -e _ground env.num_envs=4 env.robot=cassie
```
