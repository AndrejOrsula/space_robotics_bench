# `srb agent learn` — Learn from Demonstrations

Train offline on [`srb agent collect`](cli_agent_collect.md) data without
Isaac Sim, for faster startup and lower resource use.

## Usage

```bash
srb agent learn --algo ALGO --env ENV_ID [options]
```

## Options

| Option | Description | Default |
| --- | --- | --- |
| `-e`/`--env`/`--task` ENV | Environment ID for logs/specs | **REQUIRED** |
| `--cfg` PATH | YAML whose `agent` section overrides the algorithm config | `DEFAULT` (none) |
| `--algo` ALGO | Imitation-learning algorithm | **REQUIRED** |
| `--dataset` PATH | Optional HDF5 demos; omission searches new run directory, errors if none | `None` |
| `--num_epochs` N | Epoch count | Algo default |
| `--normalize_actions` | Normalize dataset actions | `False` |
| `--logdir`/`--logs` | Output root | `logs/` at checkout root or current directory when installed |

Trailing `agent.KEY=VALUE` overrides apply after `--cfg`, for example
`agent.train.batch_size=16`; other overrides and arguments after `--` are
rejected.

## Dataset admission and normalization

Training uses verified private HDF5 copies until workers, handles, and logging
close; source changes cannot affect training. Config keeps the source path for
provenance, not as an archive.

Admission rejects demos if declared `num_samples` differs from the `actions` or
any selected per-step dataset length. Every configured filter key must name a
dataset under the HDF5 `mask/` group; an empty mask fails with the filter key.
When all three arrays exist, `dones` must equal
`terminated OR truncated`.
Empty validation (split `1.0`, or one demo with any positive split) requires
`agent.experiment.validate=false` and
`agent.train.hdf5_validation_filter_key=null`.

Cache, sequence, batch, and worker allocations share a 16 GiB ceiling. Arrays
also have byte and element limits; excess declarations fail before payload
reads. Retained metadata is capped at 128 MiB. Limits cover dataset allocations,
not total process, model, optimizer, or device memory. Native configs can use
custom action keys without unused SRB transition fields.

`--normalize_actions` globally scalar-normalizes a private copy in a Linux
worker (2 GiB address space, 16 GiB output, 120 seconds); `native_limits=Hdf5NativeLimits(...)`
can lower these limits. Source stays unchanged; training readmits transformed
input and drops lossy scale-offset compression. `normalization_params.txt`
records the range unless actions are constant or absent. Standalone
normalization publishes files atomically one at a time; failure restores the
previous pair.
`RobomimicNormalizationCleanupError.retry_cleanup()` retries failed cleanup.

## Supported Algorithms

Offline learning uses [robomimic](https://robomimic.github.io).

| Algorithm | Description |
| ------------------------------- | ------------------------------------------- |
| `robomimic_bc` | Behavioral Cloning (MLP) |
| `robomimic_bc_rnn` | BC with RNN policy |
| `robomimic_bc_transformer` | BC with Transformer policy |
| `robomimic_bcq` | Batch-Constrained Q-Learning |
| `robomimic_cql` | Conservative Q-Learning |
| `robomimic_iql` | Implicit Q-Learning |
| `robomimic_td3_bc` | TD3 with BC regularization |
| `robomimic_gl` | Goal-conditioned learning |
| `robomimic_hbc` | Hierarchical BC |
| `robomimic_iris` | IRIS (latent skill discovery) |
| `robomimic_diffusion_policy` | Diffusion Policy |

See [Robomimic algorithm documentation](https://robomimic.github.io/docs/algorithms/overview.html) for hyperparameters.

> **Note:** `hyperparams/robomimic/bcq.json` and `iris.json` disable VAE/GMM
> action samplers and the distributional critic. BCQ and IRIS's BCQ value
> component requires at least one enabled. Training rejects these profiles
> before data access. Enable one (Robomimic default:
> `action_sampler.vae.enabled=true`) to use either algorithm.

## Examples

```bash
srb agent learn --algo robomimic_bc --env peg_in_hole \
  --dataset logs/peg_in_hole/collect/<run>/datasets/demos_<timestamp>.hdf5
```

```bash
srb agent learn --algo robomimic_bc_rnn --env peg_in_hole \
  --dataset /path/to/demos.hdf5 --num_epochs 500
```

## Output

Checkpoints and metrics go to `<logdir>/<env_id>/<algo>/<timestamp>/`. Evaluate:

```bash
srb agent eval --algo robomimic_bc --env peg_in_hole
```

Collect more demonstrations:

```bash
srb agent collect --algo robomimic_bc --env peg_in_hole --num_episodes 200
```

`--model-artifact-output OUT` packages the run as an SRB model artifact; the other
`--model-artifact-*` options match [`srb agent train`](cli_agent_train.md#model-artifacts).
`--dataset` records training-data provenance unless artifact dataset options
override it.

## See Also

- [`srb agent collect`](cli_agent_collect.md)
- [`srb model`](cli_model.md)
- [Imitation Learning Workflow](../workflows/imitation_learning.md)
- [Imitation Learning Integration](../integrations/imitation_learning.md)
