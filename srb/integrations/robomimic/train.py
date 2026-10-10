# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Train robomimic algorithms on demonstration datasets."""

import json
import os
import sys
import tempfile
import time
import traceback
from copy import deepcopy
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import psutil
import torch

if TYPE_CHECKING:
    from typing import Any

    from robomimic.config import Config

    from srb.datasets.robomimic_training import (
        RobomimicProviderAdmission,
        RobomimicTrainingDataset,
    )
    from srb.integrations.robomimic.training_resources import (
        RobomimicTrainingResources,
    )
    from srb.utils.json_types import JsonDecodeBudget

_robomimic_available = None


def _check_robomimic() -> bool:
    global _robomimic_available
    if _robomimic_available is None:
        try:
            import robomimic  # noqa: F401

            _robomimic_available = True
        except ImportError:
            _robomimic_available = False
    return _robomimic_available


def _import_robomimic() -> "dict[str, Any]":
    if not _check_robomimic():
        raise ImportError(
            "robomimic is not installed; install it with `pip install robomimic`."
        )

    import robomimic.utils.env_utils as EnvUtils
    import robomimic.utils.file_utils as FileUtils
    import robomimic.utils.obs_utils as ObsUtils
    import robomimic.utils.torch_utils as TorchUtils
    import robomimic.utils.train_utils as TrainUtils
    from robomimic.algo import algo_factory
    from robomimic.config import Config, config_factory
    from robomimic.utils.log_utils import DataLogger, PrintLogger

    return {
        "EnvUtils": EnvUtils,
        "FileUtils": FileUtils,
        "ObsUtils": ObsUtils,
        "TorchUtils": TorchUtils,
        "TrainUtils": TrainUtils,
        "algo_factory": algo_factory,
        "Config": Config,
        "config_factory": config_factory,
        "DataLogger": DataLogger,
        "PrintLogger": PrintLogger,
    }


SUPPORTED_ALGOS = {
    "bc": "Behavior Cloning",
    "bc_rnn": "Behavior Cloning with RNN",
    "bc_transformer": "Behavior Cloning with Transformer",
    "bcq": "Batch-Constrained Q-learning",
    "cql": "Conservative Q-Learning",
    "iql": "Implicit Q-Learning",
    "td3_bc": "TD3 with Behavioral Cloning",
    "gl": "Goal-conditioned Learning",
    "hbc": "Hierarchical Behavioral Cloning",
    "iris": "IRIS",
    "diffusion_policy": "Diffusion Policy",
}

_ALGO_NAME_MAP = {
    "bc_rnn": "bc",
    "bc_transformer": "bc",
}


def normalize_hdf5_actions(
    dataset_path: str,
    log_dir: str,
    output_path: "str | None" = None,
    *,
    max_decoded_bytes: int | None = None,
    max_batch_bytes: int | None = None,
) -> str:
    """Normalize actions through bounded, verified HDF5 staging."""
    from srb.integrations.robomimic.normalization import (
        normalize_hdf5_actions as normalize,
    )

    return normalize(
        dataset_path,
        log_dir,
        output_path,
        max_decoded_bytes=max_decoded_bytes,
        max_batch_bytes=max_batch_bytes,
    )


def load_config(
    algo: str,
    config_path: "str | Path | None" = None,
    config_overrides: "dict[str, Any] | None" = None,
) -> "Config":
    """Load a robomimic configuration."""
    rm = _import_robomimic()

    robomimic_algo = _ALGO_NAME_MAP.get(algo, algo)

    if config_path is not None and not Path(config_path).is_file():
        raise FileNotFoundError(f"robomimic config file does not exist: {config_path}")

    if config_path is not None:
        with open(config_path) as f:
            ext_cfg = json.load(f)
        if not isinstance(ext_cfg, dict):
            raise ValueError(
                f"robomimic config {config_path} must be a JSON object, "
                f"got {type(ext_cfg).__name__}"
            )
        # Require the file's algorithm to match the caller's selection.
        file_algo = ext_cfg.get("algo_name")
        if file_algo is not None and file_algo != robomimic_algo:
            raise ValueError(
                f"robomimic config {config_path} declares algo_name "
                f"{file_algo!r}, but the requested algorithm is {algo!r} "
                f"(robomimic {robomimic_algo!r}); pass a matching --algo or "
                "drop algo_name from the file"
            )
        config = rm["config_factory"](robomimic_algo)
        with config.values_unlocked():
            config.update(ext_cfg)
    else:
        config = rm["config_factory"](robomimic_algo)

    if config_overrides:
        with config.values_unlocked():
            config.update(deepcopy(config_overrides))

    return config


def train(
    config: "Config",
    device: str,
    log_dir: str,
    ckpt_dir: str,
    video_dir: str,
    *,
    max_decoded_bytes: int | None = None,
    max_metadata_bytes: int | None = None,
) -> None:
    """Train the algorithm configured by ``config``.

    ``max_decoded_bytes`` excludes model memory and process RSS;
    ``max_metadata_bytes`` excludes Python JSON object-graph expansion.
    """
    from srb.integrations.robomimic.training_resources import (
        RobomimicTrainingResources,
    )

    _require_constructible_algorithm(config)
    with RobomimicTrainingResources() as resources:
        runtime_config, inputs, _json_budget = _open_training_config(
            config,
            resources,
            max_decoded_bytes=max_decoded_bytes,
            max_metadata_bytes=max_metadata_bytes,
        )
        source_config = config
        if _disable_cache_for_nested_obs_keys(runtime_config):
            # Keep the caller's dataset list but record the effective mode.
            source_config = deepcopy(config)
            with source_config.values_unlocked():
                source_config.train.hdf5_cache_mode = None
        _admit_provider_inputs(
            runtime_config, inputs, max_decoded_bytes=max_decoded_bytes
        )
        _execute_training(
            runtime_config,
            device,
            log_dir,
            ckpt_dir,
            video_dir,
            env_meta=inputs[0].env_meta,
            source_config=source_config,
            resources=resources,
        )


def _require_constructible_algorithm(config: "Config") -> None:
    """Reject BCQ configs that robomimic 0.5.0 otherwise rejects with an assert.

    BCQ requires a distributional critic, GMM sampler, or VAE sampler. IRIS
    uses BCQ as its value component, so validate both before opening data.
    """

    components = {
        "bcq": ("algo", config.algo),
        "iris": (
            "algo.value_planner.value",
            config.algo.get("value_planner", {}).get("value"),
        ),
    }
    if config.algo_name not in components:
        return
    where, bcq = components[config.algo_name]
    if not isinstance(bcq, dict):
        return
    critic = bcq.get("critic", {}).get("distributional", {})
    sampler = bcq.get("action_sampler", {})
    if (
        critic.get("enabled")
        or sampler.get("gmm", {}).get("enabled")
        or sampler.get("vae", {}).get("enabled")
    ):
        return
    raise ValueError(
        f"robomimic {config.algo_name} config cannot build its BCQ component: "
        f"{where}.action_sampler.vae.enabled, {where}.action_sampler.gmm.enabled "
        f"and {where}.critic.distributional.enabled are all false; enable "
        f"one (robomimic's default is {where}.action_sampler.vae.enabled=true)"
    )


def _open_training_config(
    config: "Config",
    resources: "RobomimicTrainingResources",
    *,
    max_decoded_bytes: int | None = None,
    max_metadata_bytes: int | None = None,
) -> "tuple[Config, list[RobomimicTrainingDataset], JsonDecodeBudget]":
    """Clone provider configuration and replace every input with an owned path."""
    from srb.datasets.robomimic_training import open_robomimic_training_dataset
    from srb.utils.json_types import JsonDecodeBudget
    from srb.utils.resource_budget import (
        MAX_DIRECTORY_ENTRIES,
        MAX_EXTERNAL_FILE_BYTES,
        MAX_METADATA_AGGREGATE_BYTES,
        ByteBudget,
        ResourceLimitError,
        require_bounded_int,
    )

    runtime_config = deepcopy(config)
    configured = runtime_config.train.data
    if isinstance(configured, (str, Path)):
        configured = [{"path": str(configured)}]
    if not isinstance(configured, list):
        raise ValueError("robomimic train.data must be a path or a dataset list")
    # robomimic's MetaDataset has no observation normalization statistics.
    if runtime_config.train.hdf5_normalize_obs and len(configured) > 1:
        raise ValueError("hdf5_normalize_obs requires a single training dataset")
    require_bounded_int(
        len(configured),
        resource="robomimic training sources",
        minimum=1,
        maximum=MAX_DIRECTORY_ENTRIES,
    )
    inputs = []
    operational = []
    wire = ByteBudget("robomimic training source bytes", MAX_EXTERNAL_FILE_BYTES)
    metadata = ByteBudget(
        "robomimic training metadata bytes",
        require_bounded_int(
            MAX_METADATA_AGGREGATE_BYTES
            if max_metadata_bytes is None
            else max_metadata_bytes,
            resource="robomimic training metadata limit",
            minimum=1,
            maximum=MAX_METADATA_AGGREGATE_BYTES,
        ),
    )
    json_budget = JsonDecodeBudget()
    for entry in configured:
        if not isinstance(entry, dict) or not isinstance(
            entry.get("path"), (str, Path)
        ):
            raise ValueError("robomimic training dataset entry requires a path")
        remaining_source_bytes = wire.limit - wire.consumed
        if remaining_source_bytes == 0:
            raise ResourceLimitError(
                wire.resource, limit=wire.limit, observed=wire.consumed + 1
            )
        remaining_metadata_bytes = metadata.limit - metadata.consumed
        if remaining_metadata_bytes == 0:
            raise ResourceLimitError(
                metadata.resource,
                limit=metadata.limit,
                observed=metadata.consumed + 1,
            )
        owner = resources.own_input(
            open_robomimic_training_dataset(
                entry["path"],
                max_source_bytes=remaining_source_bytes,
                max_decoded_bytes=max_decoded_bytes,
                max_metadata_bytes=remaining_metadata_bytes,
                dataset_keys=config.train.dataset_keys,
                action_keys=config.train.action_keys,
                load_next_obs=config.train.hdf5_load_next_obs,
                json_budget=json_budget,
            )
        )
        wire.consume(owner.source_bytes)
        metadata.consume(owner.metadata_decoded_bytes)
        inputs.append(owner)
        operational.append({**entry, "path": str(owner.dataset_path)})
    with runtime_config.values_unlocked():
        runtime_config.train.data = operational
    return runtime_config, inputs, json_budget


def _admit_provider_inputs(
    config: "Config",
    inputs: "list[RobomimicTrainingDataset]",
    *,
    max_decoded_bytes: int | None = None,
) -> None:
    """Check configured fields against shared input and decoding budgets."""
    from robomimic.utils.lang_utils import LANG_EMB_OBS_KEY

    from srb.utils.resource_budget import (
        MAX_DECODED_AGGREGATE_BYTES,
        MAX_EXTERNAL_FILE_BYTES,
        ByteBudget,
        require_bounded_int,
    )

    wire = ByteBudget("robomimic training source bytes", MAX_EXTERNAL_FILE_BYTES)
    decoded = ByteBudget(
        "robomimic training decoded bytes",
        require_bounded_int(
            MAX_DECODED_AGGREGATE_BYTES
            if max_decoded_bytes is None
            else max_decoded_bytes,
            resource="robomimic training decoded limit",
            minimum=1,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        ),
    )
    obs_keys = [key for key in config.all_obs_keys if key != LANG_EMB_OBS_KEY]
    for entry, owner in zip(config.train.data, inputs, strict=True):
        wire.consume(owner.source_bytes)
        mask_keys = [
            key
            for key in (
                entry.get("filter_key"),
                config.train.hdf5_filter_key,
                config.train.hdf5_validation_filter_key,
            )
            if key is not None
        ]
        admitted = owner.admit_provider(
            config,
            obs_keys=obs_keys,
            mask_keys=mask_keys,
            max_decoded_bytes=max(1, decoded.limit - decoded.consumed),
        )
        decoded.consume(admitted.declared_decoded_bytes)
        # Charge both retained metadata copies; provider cache replicas follow.
        decoded.consume(owner.metadata_decoded_bytes)
        decoded.consume(admitted.metadata_decoded_bytes)
        decoded.consume(_provider_allocation_bytes(config, entry, admitted))


def _provider_allocation_bytes(
    config: "Config",
    entry: "dict[str, Any]",
    admitted: "RobomimicProviderAdmission",
) -> int:
    """Bound provider dataset allocations, including caches and worker copies.

    This excludes process RSS and model/optimizer memory. Batch estimates use
    row widths rather than multiplying full input arrays.
    """
    from srb.utils.resource_budget import (
        MAX_ARRAY_BYTES,
        MAX_ARRAY_ELEMENTS,
        MAX_DIRECTORY_ENTRIES,
        require_bounded_int,
    )

    def bounded(value: object, name: str, *, minimum: int = 1) -> int:
        return require_bounded_int(
            value,
            resource=f"robomimic training {name}",
            minimum=minimum,
            maximum=MAX_ARRAY_ELEMENTS,
        )

    sequence = bounded(config.train.seq_length, "sequence length")
    frames = bounded(config.train.frame_stack, "frame stack")
    batch = bounded(config.train.batch_size, "batch size")
    workers = require_bounded_int(
        config.train.num_data_workers,
        resource="robomimic training workers",
        minimum=0,
        maximum=MAX_DIRECTORY_ENTRIES,
    )
    length = bounded(sequence + frames - 1, "stacked sequence length")
    mode = config.train.hdf5_cache_mode
    if mode not in (None, "all", "low_dim"):
        raise ValueError(f"unsupported robomimic cache mode: {mode!r}")
    declaration = admitted.declaration

    language_elements = 0
    from robomimic.utils.lang_utils import LANG_EMB_OBS_KEY

    if (
        LANG_EMB_OBS_KEY in config.all_obs_keys
        and entry.get("lang", "dummy") is not None
    ):
        from robomimic.utils.lang_utils import get_lang_emb_shape

        language_elements = 1
        for dimension in get_lang_emb_shape():
            language_elements = bounded(
                language_elements * bounded(dimension, "language dimension"),
                "language elements",
            )
    # Allow float64 promotion and wider source dtypes; get_item also joins actions.
    row_bytes = (
        declaration.selected_row_bytes
        + declaration.dataset_row_bytes
        + 8 * language_elements
    )
    largest_row_bytes = max(
        declaration.max_row_bytes,
        declaration.dataset_row_bytes,
        8 * language_elements,
        8,  # int64 sampled indices and temporary padding arrays
    )
    largest_row_elements = max(
        declaration.max_row_elements,
        declaration.dataset_row_elements,
        language_elements,
        1,
    )
    for copies, label in ((length, "sequence"), (length * batch, "batch")):
        bounded(copies * largest_row_elements, f"{label} array elements")
        require_bounded_int(
            copies * largest_row_bytes,
            resource=f"robomimic training {label} array bytes",
            minimum=0,
            maximum=MAX_ARRAY_BYTES,
        )

    obs_count = len(admitted.obs_keys or ())
    field_count = len(admitted.dataset_keys) + obs_count * (
        2 if config.train.hdf5_load_next_obs else 1
    )
    key_storage = 4 * (
        sum(len(key) for key in admitted.dataset_keys)
        + sum(len(key) for key in admitted.obs_keys or ())
        * (2 if config.train.hdf5_load_next_obs else 1)
    )
    # All-mode caches retain containers and NumPy headers; payloads are separate.
    item = length * row_bytes + 512 * (field_count + 4) + key_storage
    if config.train.goal_mode == "last":
        item += (
            declaration.next_observation_row_bytes
            + 512 * (obs_count + 1)
            + 4 * sum(len(key) for key in admitted.obs_keys or ())
        )
    # Include sequence padding, masks, and four data/obs/next/goal calls.
    sequence_scratch = 3 * length * row_bytes + 4 * 64 * length
    # Entry-level ``filter_key=None`` can override a global mask.
    multiplicity = max(1, declaration.max_demo_multiplicity)
    sequence_count = admitted.total_samples * multiplicity
    index_storage = (
        256 * sequence_count + 1024 * admitted.num_demos + 512 * declaration.mask_rows
    )
    raw_cache = 0
    if mode in ("all", "low_dim"):
        # Charge all selected observations, even when low_dim excludes images.
        raw_cache = (
            declaration.selected_source_bytes
            + declaration.converted_float32_bytes
            + admitted.model_file_bytes
        )
    item_cache = sequence_count * item if mode == "all" else 0
    # Bound trajectory reads, casts, processed data, and normalization statistics.
    statistics = declaration.dataset_source_bytes + 12 * (
        declaration.dataset_converted_float32_bytes
    )
    if config.train.hdf5_normalize_obs:
        statistics += declaration.observation_source_bytes + 12 * (
            declaration.observation_converted_float32_bytes
        )
    resident = (
        index_storage
        + declaration.mask_decoded_bytes
        + raw_cache
        + item_cache
        + statistics
    )
    # Include two prefetched batches per worker, collation, and the active batch.
    batches = (2 * workers + 2) * 3 * batch * item
    worker_storage = (workers + 1) * (resident + sequence_scratch)
    # Validation retains a separate dataset and uses at most one worker.
    lanes = 2 if config.experiment.validate else 1
    return lanes * (worker_storage + batches)


def _execute_training(
    config: "Config",
    device: str,
    log_dir: str,
    ckpt_dir: str,
    video_dir: str,
    *,
    env_meta: "dict[str, Any]",
    source_config: "Config",
    resources: "RobomimicTrainingResources",
) -> None:
    """Restore process streams before the owner closes native logging files."""
    original_stdout, original_stderr = sys.stdout, sys.stderr
    try:
        _train_admitted(
            config,
            device,
            log_dir,
            ckpt_dir,
            video_dir,
            env_meta=deepcopy(env_meta),
            source_config=source_config,
            resources=resources,
        )
    finally:
        sys.stdout, sys.stderr = original_stdout, original_stderr


def _train_admitted(
    config: "Config",
    device: str,
    log_dir: str,
    ckpt_dir: str,
    video_dir: str,
    *,
    env_meta: "dict[str, Any]",
    source_config: "Config",
    resources: "RobomimicTrainingResources",
) -> None:
    """Run the native provider while its input and cleanup owners remain alive."""
    rm = _import_robomimic()

    np.random.seed(config.train.seed)
    torch.manual_seed(config.train.seed)

    print("\n============= New Training Run with Config =============")
    print(config)
    print("")

    print(f">>> Saving logs into directory: {log_dir}")
    print(f">>> Saving checkpoints into directory: {ckpt_dir}")
    print(f">>> Saving videos into directory: {video_dir}")

    if config.experiment.logging.terminal_output_to_txt:
        logger = rm["PrintLogger"](os.path.join(log_dir, "log.txt"))
        resources.own_logger(logger)
        sys.stdout = logger
        sys.stderr = logger

    rm["ObsUtils"].initialize_obs_utils_with_config(config)

    # ``config.train.data`` became a list of per-dataset configs; robomimic's
    # own trainer normalizes a bare path the same way. SRB's ``--dataset``
    # names exactly one dataset, so the first entry is the one to describe.
    if isinstance(config.train.data, str):
        with config.values_unlocked():
            config.train.data = [{"path": config.train.data}]
    dataset_cfg = config.train.data[0]

    dataset_path = os.path.expanduser(dataset_cfg["path"])
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"Training dataset not found: {dataset_path}")

    print("\n============= Loaded Environment Metadata =============")
    rm["EnvUtils"].set_env_specific_obs_processing(env_meta=env_meta)
    shape_meta = rm["FileUtils"].get_shape_metadata_from_dataset(
        dataset_config=dataset_cfg,
        action_keys=config.train.action_keys,
        all_obs_keys=config.all_obs_keys,
        verbose=True,
    )

    if config.experiment.env is not None:
        env_meta["env_name"] = config.experiment.env
        print("=" * 30 + "\n" + f"Replacing Env to {env_meta['env_name']}\n" + "=" * 30)

    print("")

    data_logger = rm["DataLogger"](
        log_dir, config=config, log_tb=config.experiment.logging.log_tb
    )
    resources.own_logger(data_logger)
    model = rm["algo_factory"](
        algo_name=config.algo_name,
        config=config,
        obs_key_shapes=shape_meta["all_shapes"],
        ac_dim=shape_meta["ac_dim"],
        device=device,
    )

    config_save_path = os.path.join(log_dir, "..", "config.json")
    os.makedirs(os.path.dirname(config_save_path), exist_ok=True)
    with open(config_save_path, "w") as outfile:
        json.dump(source_config, outfile, indent=4)

    print("\n============= Model Summary =============")
    print(model)
    print("")

    # Register each native dataset before construction can open its file.
    from srb.integrations.robomimic.provider_data import load_training_datasets

    trainset, validset = load_training_datasets(
        config, obs_keys=shape_meta["all_obs_keys"], resources=resources
    )
    train_sampler = trainset.get_dataset_sampler()
    print("\n============= Training Dataset =============")
    print(trainset)
    print("")

    # robomimic applies these per batch, so every run_epoch needs the stats.
    obs_normalization_stats = None
    if config.train.hdf5_normalize_obs:
        obs_normalization_stats = trainset.get_obs_normalization_stats()

    # Checkpoints use these stats to unnormalize policy actions; validation must
    # use the same training-split stats as the policy.
    action_normalization_stats = trainset.get_action_normalization_stats()
    if validset is not None:
        _share_action_normalization_stats(config, validset, action_normalization_stats)

    train_loader = resources.data_loader(
        dataset=trainset,
        sampler=train_sampler,
        batch_size=config.train.batch_size,
        shuffle=(train_sampler is None),
        num_workers=config.train.num_data_workers,
        drop_last=True,
    )

    if config.experiment.validate:
        num_workers = min(config.train.num_data_workers, 1)
        valid_sampler = validset.get_dataset_sampler()
        valid_loader = resources.data_loader(
            dataset=validset,
            sampler=valid_sampler,
            batch_size=config.train.batch_size,
            shuffle=(valid_sampler is None),
            num_workers=num_workers,
            drop_last=True,
        )
    else:
        valid_loader = None
    for split, loader in (("training", train_loader), ("validation", valid_loader)):
        if loader is not None and len(loader) == 0:
            raise ValueError(
                f"robomimic {split} split is smaller than one batch "
                f"(train.batch_size={config.train.batch_size}); collect more "
                "demonstrations or lower agent.train.batch_size"
            )

    best_valid_loss = None
    # Wall-clock adjustments can stall time-based saving; monotonic time cannot.
    last_ckpt_time = time.monotonic()

    train_num_steps = config.experiment.epoch_every_n_steps
    valid_num_steps = config.experiment.validation_epoch_every_n_steps

    for epoch in range(1, config.train.num_epochs + 1):
        with resources.epoch_iterators():
            step_log = rm["TrainUtils"].run_epoch(
                model=model,
                data_loader=train_loader,
                epoch=epoch,
                num_steps=train_num_steps,
                obs_normalization_stats=obs_normalization_stats,
            )
        model.on_epoch_end(epoch)

        epoch_ckpt_name = f"model_epoch_{epoch}"

        should_save_ckpt = False
        if config.experiment.save.enabled:
            time_check = (config.experiment.save.every_n_seconds is not None) and (
                time.monotonic() - last_ckpt_time
                > config.experiment.save.every_n_seconds
            )
            epoch_check = (
                (config.experiment.save.every_n_epochs is not None)
                and (epoch > 0)
                and (epoch % config.experiment.save.every_n_epochs == 0)
            )
            epoch_list_check = epoch in config.experiment.save.epochs
            last_epoch_check = epoch == config.train.num_epochs
            should_save_ckpt = (
                time_check or epoch_check or epoch_list_check or last_epoch_check
            )

        ckpt_reason = None
        if should_save_ckpt:
            last_ckpt_time = time.monotonic()
            ckpt_reason = "time"

        print(f"Train Epoch {epoch}")
        print(json.dumps(step_log, sort_keys=True, indent=4))
        for k, v in step_log.items():
            if k.startswith("Time_"):
                data_logger.record(f"Timing_Stats/Train_{k[5:]}", v, epoch)
            else:
                data_logger.record(f"Train/{k}", v, epoch)

        if config.experiment.validate and valid_loader is not None:
            with torch.no_grad(), resources.epoch_iterators():
                step_log = rm["TrainUtils"].run_epoch(
                    model=model,
                    data_loader=valid_loader,
                    epoch=epoch,
                    validate=True,
                    num_steps=valid_num_steps,
                    obs_normalization_stats=obs_normalization_stats,
                )
            for k, v in step_log.items():
                if k.startswith("Time_"):
                    data_logger.record(f"Timing_Stats/Valid_{k[5:]}", v, epoch)
                else:
                    data_logger.record(f"Valid/{k}", v, epoch)

            print(f"Validation Epoch {epoch}")
            print(json.dumps(step_log, sort_keys=True, indent=4))

            valid_check = "Loss" in step_log
            if valid_check and (
                best_valid_loss is None or (step_log["Loss"] <= best_valid_loss)
            ):
                best_valid_loss = step_log["Loss"]
                if (
                    config.experiment.save.enabled
                    and config.experiment.save.on_best_validation
                ):
                    epoch_ckpt_name += f"_best_validation_{best_valid_loss}"
                    should_save_ckpt = True
                    ckpt_reason = "valid" if ckpt_reason is None else ckpt_reason

        if should_save_ckpt:
            rm["TrainUtils"].save_model(
                model=model,
                config=source_config,
                env_meta=env_meta,
                shape_meta=shape_meta,
                ckpt_path=os.path.join(ckpt_dir, epoch_ckpt_name + ".pth"),
                obs_normalization_stats=obs_normalization_stats,
                action_normalization_stats=action_normalization_stats,
            )

        process = psutil.Process(os.getpid())
        mem_usage = int(process.memory_info().rss / 1000000)
        data_logger.record("System/RAM Usage (MB)", mem_usage, epoch)
        print(f"\nEpoch {epoch} Memory Usage: {mem_usage} MB\n")

    # The resource owner closes datasets and logging before releasing inputs.


def _share_action_normalization_stats(
    config: "Config",
    validset: "Any",
    action_normalization_stats: "dict[str, Any]",
) -> None:
    """Make validation normalize actions with the training statistics."""

    from srb.utils import logging

    validset.set_action_normalization_stats(action_normalization_stats)
    normalized = [
        key
        for key in config.train.action_keys
        if (config.train.action_config.get(key) or {}).get("normalization") is not None
    ]
    if normalized and getattr(validset, "hdf5_cache_mode", None) == "all":
        # robomimic's ``all`` cache materialises validation items, including
        # their normalized actions, while constructing the dataset.
        logging.warning(
            "robomimic validation uses hdf5_cache_mode='all', which caches "
            f"actions {normalized} normalized with the validation split's own "
            "statistics before training statistics can be shared; validation "
            "losses (and on_best_validation checkpoints) are not comparable to "
            "training. Use hdf5_cache_mode='low_dim' or None to score "
            "validation with the training statistics."
        )


def _avoid_stale_validation_action_cache(config: "Config") -> None:
    """Keep validation actions live when their normalization stats are replaced."""
    normalized = any(
        (config.train.action_config.get(key) or {}).get("normalization") is not None
        for key in config.train.action_keys
    )
    if (
        normalized
        and config.experiment.validate
        and config.train.hdf5_cache_mode == "all"
    ):
        from srb.utils import logging

        # ``all`` materializes normalized actions in ``getitem_cache`` during
        # dataset construction. Validation receives training stats afterward,
        # so those cached actions would still use validation-only statistics.
        # ``low_dim`` retains observation caching but normalizes actions when
        # each item is fetched.
        config.train.hdf5_cache_mode = "low_dim"
        logging.warning(
            "robomimic validation uses hdf5_cache_mode='low_dim' because "
            "hdf5_cache_mode='all' would retain actions normalized with the "
            "validation split's statistics instead of the training statistics"
        )


def _dataset_observation_leaves(
    dataset_path: "str | Path",
) -> "list[tuple[str, tuple[int, ...], np.dtype]]":
    """Return each first-demo observation leaf's key, shape, and dtype."""
    import h5py

    # Deeper keys exceed the admission limit and cannot be trained on.
    from srb.datasets.robomimic_training import _MAX_HDF5_KEY_COMPONENTS
    from srb.utils.resource_budget import (
        MAX_DIRECTORY_ENTRIES,
        ResourceLimitError,
        require_dataset_file_size,
    )

    # Enforce the file-size ceiling before h5py reads it.
    require_dataset_file_size(
        os.path.expanduser(str(dataset_path)),
        resource="robomimic training dataset",
    )
    leaves: list[tuple[str, tuple[int, ...], np.dtype]] = []
    with h5py.File(os.path.expanduser(str(dataset_path)), "r") as handle:
        data = handle.get("data")
        if data is None:
            return leaves
        demo_keys = [key for key in data if key.startswith("demo_")]
        if not demo_keys:
            return leaves
        obs = data[sorted(demo_keys)[0]].get("obs")
        if not isinstance(obs, h5py.Group):
            return leaves
        # Explicit stack: an HDF5 hard link can point a group at an ancestor.
        pending: list[tuple[tuple[str, ...], h5py.Group]] = [((), obs)]
        visited = 0
        while pending:
            prefix, group = pending.pop()
            for name in sorted(group, reverse=True):
                visited += 1
                if visited > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        "robomimic observation tree entries",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=visited,
                    )
                path = (*prefix, name)
                if len(path) > _MAX_HDF5_KEY_COMPONENTS:
                    raise ValueError(
                        f"robomimic observation obs/{'/'.join(path)} is nested "
                        f"deeper than {_MAX_HDF5_KEY_COMPONENTS} levels"
                    )
                node = group.get(name)
                if isinstance(node, h5py.Group):
                    pending.append((path, node))
                elif isinstance(node, h5py.Dataset):
                    # Leading axis is time.
                    leaves.append(("/".join(path), tuple(node.shape[1:]), node.dtype))
    leaves.sort(key=lambda leaf: leaf[0])
    return leaves


def _classify_observation_leaves(
    leaves: "list[tuple[str, tuple[int, ...], np.dtype]]",
) -> "tuple[list[str], list[str], list[str]]":
    """Classify observation leaves; image shapes use ``(H, W, C)``."""

    low_dim: list[str] = []
    rgb: list[str] = []
    depth: list[str] = []
    for key, shape, dtype in leaves:
        if dtype.kind not in "biuf":
            raise ValueError(
                f"robomimic cannot train on observation {key!r}: dtype {dtype} "
                "is not numeric; set observation.modalities explicitly to "
                "exclude it"
            )
        if len(shape) < 3:
            low_dim.append(key)
        elif len(shape) == 3 and shape[-1] == 3 and dtype == np.uint8:
            rgb.append(key)
        elif len(shape) == 3 and shape[-1] == 1 and dtype.kind == "f":
            depth.append(key)
        else:
            raise ValueError(
                f"robomimic cannot classify observation {key!r} with per-sample "
                f"shape {shape} and dtype {dtype}: robomimic images must be "
                "uint8 (H, W, 3) rgb or floating-point (H, W, 1) depth. SRB "
                "cameras merge rgb and uint8 depth into one uint8 (H, W, 4) "
                "image by default; collect with 'env.camera_data_types=[rgb]' "
                "for rgb-only images, or set observation.modalities explicitly "
                "to exclude this observation"
            )
    return low_dim, rgb, depth


def _disable_cache_for_nested_obs_keys(config: "Config") -> bool:
    """Disable caching for nested keys unsupported by robomimic 0.5.0.

    Return whether the mode changed; checkpoints record the effective mode.
    """

    from srb.utils import logging

    mode = config.train.hdf5_cache_mode
    if mode not in ("all", "low_dim"):
        return False
    nested = sorted(key for key in config.all_obs_keys if "/" in key)
    if not nested:
        return False
    logging.warning(
        f"robomimic hdf5_cache_mode={mode!r} cannot cache nested observation "
        f"keys {nested} (robomimic 0.5.0 splits cached keys on a single '/'); "
        "training with hdf5_cache_mode=None, which reads every batch from the "
        "HDF5 file. The saved config records the effective cache mode."
    )
    with config.values_unlocked():
        config.train.hdf5_cache_mode = None
    return True


def _observation_modalities_trees(node: "Any") -> "list[Any]":
    """Return all ``modalities`` subtrees, including nested algorithm configs."""

    if not isinstance(node, dict):
        return []
    if "modalities" in node:
        return [node["modalities"]]
    trees = []
    for key in sorted(node):
        trees.extend(_observation_modalities_trees(node[key]))
    return trees


def _align_observation_modalities(config: "Config", dataset_path: str) -> None:
    """Replace configured modalities that the dataset cannot satisfy."""

    from srb.utils import logging

    # Leave missing-file errors to robomimic's loader.
    if not os.path.isfile(os.path.expanduser(str(dataset_path))):
        return

    leaves = _dataset_observation_leaves(dataset_path)
    available = {key for key, _shape, _dtype in leaves}
    if not available:
        return

    stale = []
    for tree in _observation_modalities_trees(config.observation):
        for group_name in sorted(tree):
            group = tree[group_name]
            if not isinstance(group, dict):
                continue
            configured = {
                key
                for keys in group.values()
                if isinstance(keys, (list, tuple))
                for key in keys
            }
            missing = sorted(configured - available)
            if missing:
                stale.append((group_name, group, missing))
    if not stale:
        return

    # Classify only when the dataset has to supply the keys, so an explicit
    # config naming present keys never trips over an unrelated observation.
    low_dim, rgb, depth = _classify_observation_leaves(leaves)
    derived = {"low_dim": low_dim, "rgb": rgb, "depth": depth}
    # ``Config.__setitem__`` is not lock-checked; the scope documents intent.
    with config.values_unlocked():
        for group_name, group, missing in stale:
            # A hierarchical planner predicts ``subgoal`` observations; only
            # low-dimensional subgoals are supported by its decoder.
            replacement = {"low_dim": low_dim} if group_name == "subgoal" else derived
            logging.info(
                f"robomimic observation modalities {missing} ({group_name}) are "
                "absent from the dataset; using its own keys instead "
                f"({replacement})"
            )
            for modality in list(group):
                group[modality] = list(replacement.get(modality, []))
            for modality, keys in replacement.items():
                group[modality] = list(keys)


def run_training(
    dataset_path: str,
    algo: str,
    logdir: "str | Path",
    config_path: "str | Path | None" = None,
    config_overrides: "dict[str, Any] | None" = None,
    normalize_actions: bool = False,
    num_epochs: "int | None" = None,
    seed: int = 1,
    *,
    max_decoded_bytes: int | None = None,
    max_metadata_bytes: int | None = None,
) -> Path | None:
    """Run robomimic training.

    ``max_decoded_bytes`` bounds dataset declarations and provider allocations;
    it excludes model memory and RSS. ``max_metadata_bytes`` bounds shared input
    metadata, excluding Python object-graph expansion.

    Returns:
        Checkpoint directory, or ``None`` when saving is disabled.

    When normalizing actions, checkpoints store the global range in
    ``env_metadata`` as ``norm_factor_min`` and ``norm_factor_max`` for
    evaluation to recover the original scale.
    """
    rm = _import_robomimic()

    logdir = Path(logdir)

    config = load_config(algo, config_path, config_overrides)
    _require_constructible_algorithm(config)

    # Keep checkpoints inside the run directory for cleanup and evaluation discovery.
    with config.values_unlocked():
        config.train.data = dataset_path
        config.train.seed = seed
        config.train.output_dir = logdir.as_posix()
        if num_epochs is not None:
            config.train.num_epochs = num_epochs
    epochs = config.train.num_epochs
    if isinstance(epochs, bool) or not isinstance(epochs, int) or epochs < 1:
        raise ValueError(f"robomimic train.num_epochs must be >= 1, got {epochs!r}")

    from srb.datasets.robomimic_training import open_robomimic_training_dataset
    from srb.integrations.robomimic.training_resources import (
        RobomimicTrainingResources,
    )

    action_range: tuple[float, float] | None = None
    try:
        with RobomimicTrainingResources() as resources:
            runtime_config, inputs, json_budget = _open_training_config(
                config,
                resources,
                max_decoded_bytes=max_decoded_bytes,
                max_metadata_bytes=max_metadata_bytes,
            )

            _align_observation_modalities(runtime_config, str(inputs[0].dataset_path))
            _disable_cache_for_nested_obs_keys(runtime_config)
            _avoid_stale_validation_action_cache(runtime_config)
            _admit_provider_inputs(
                runtime_config, inputs, max_decoded_bytes=max_decoded_bytes
            )

            # Admit input before the provider creates output directories.
            # Robomimic 0.5.0 appends a time directory to the original triple.
            exp_dirs = rm["TrainUtils"].get_exp_dir(runtime_config)
            if len(exp_dirs) < 3:
                raise ValueError(
                    "robomimic TrainUtils.get_exp_dir returned "
                    f"{len(exp_dirs)} paths; expected at least log, output and video"
                )
            log_dir, ckpt_dir, video_dir = exp_dirs[:3]

            if normalize_actions:
                from srb.utils.resource_budget import (
                    MAX_METADATA_AGGREGATE_BYTES,
                    ResourceLimitError,
                )

                metadata_limit = (
                    MAX_METADATA_AGGREGATE_BYTES
                    if max_metadata_bytes is None
                    else max_metadata_bytes
                )
                retained_metadata = sum(
                    owner.metadata_decoded_bytes for owner in inputs
                )
                remaining_metadata = metadata_limit - retained_metadata
                if remaining_metadata <= 0:
                    raise ResourceLimitError(
                        "robomimic training metadata bytes",
                        limit=metadata_limit,
                        observed=retained_metadata + 1,
                    )
                temporary = resources.own_temporary(
                    Path(tempfile.mkdtemp(prefix="srb-robomimic-normalization-"))
                )
                normalized_path = normalize_hdf5_actions(
                    str(inputs[0].dataset_path),
                    str(log_dir),
                    output_path=str(temporary / "normalized.hdf5"),
                    max_decoded_bytes=max_decoded_bytes,
                )
                normalized = resources.own_input(
                    open_robomimic_training_dataset(
                        normalized_path,
                        max_decoded_bytes=max_decoded_bytes,
                        max_metadata_bytes=remaining_metadata,
                        dataset_keys=runtime_config.train.dataset_keys,
                        action_keys=runtime_config.train.action_keys,
                        load_next_obs=runtime_config.train.hdf5_load_next_obs,
                        json_budget=json_budget,
                    )
                )
                inputs = [normalized]
                action_range = _read_action_normalization_range(Path(log_dir))
                with runtime_config.values_unlocked():
                    runtime_config.train.data = [{"path": str(normalized.dataset_path)}]

                _admit_provider_inputs(
                    runtime_config, inputs, max_decoded_bytes=max_decoded_bytes
                )
            # Persist reproducible source locations, never private temporary
            # paths that disappear when provider cleanup completes.
            source_config = deepcopy(runtime_config)
            with source_config.values_unlocked():
                source_config.train.data = config.train.data

            env_meta = inputs[0].env_meta
            if action_range is not None:
                env_meta["norm_factor_min"], env_meta["norm_factor_max"] = action_range

            device = rm["TorchUtils"].get_torch_device(
                try_to_use_cuda=runtime_config.train.cuda
            )
            runtime_config.lock()
            source_config.lock()
            _execute_training(
                runtime_config,
                device,
                log_dir,
                ckpt_dir,
                video_dir,
                env_meta=env_meta,
                source_config=source_config,
                resources=resources,
            )
    except Exception:
        print(f"Run failed with error:\n{traceback.format_exc()}")
        raise
    print("Finished run successfully!")

    # Saving can be disabled, in which case robomimic creates no model directory.
    return Path(ckpt_dir) if ckpt_dir is not None else None


_NORMALIZATION_PARAMETER_FILE = "normalization_params.txt"
_NORMALIZATION_PARAMETER_BYTES = 4096


def _read_action_normalization_range(log_dir: Path) -> "tuple[float, float] | None":
    """Read the action range published by ``normalize_hdf5_actions``."""

    import math

    path = log_dir / _NORMALIZATION_PARAMETER_FILE
    if not path.is_file():
        return None
    with path.open("rb") as stream:
        payload = stream.read(_NORMALIZATION_PARAMETER_BYTES + 1)
    if len(payload) > _NORMALIZATION_PARAMETER_BYTES:
        raise ValueError(f"robomimic action normalization parameters {path} too large")
    values: dict[str, float] = {}
    for line in payload.decode("utf-8").splitlines():
        name, separator, text = line.partition(":")
        if separator and name.strip() in ("min", "max"):
            values[name.strip()] = float(text)
    minimum, maximum = values.get("min"), values.get("max")
    if (
        minimum is None
        or maximum is None
        or not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or not maximum > minimum
    ):
        raise ValueError(
            f"robomimic action normalization parameters {path} do not hold a "
            f"finite range with max > min: {payload!r}"
        )
    return minimum, maximum
