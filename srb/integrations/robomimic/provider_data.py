"""Owned dataset construction for the pinned Robomimic provider."""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

from robomimic.utils.dataset import MetaDataset, SequenceDataset


# The optional provider has no type information for this base.
class _OwnedSequenceDataset(SequenceDataset):  # type: ignore[misc]
    """Register a dataset before the provider can open its HDF5 input."""

    def __init__(self, resources: Any, **kwargs: Any) -> None:
        resources.own_dataset(self)
        super().__init__(**kwargs)

    def close_and_delete_hdf5_handle(self) -> None:
        """Close an initialized provider handle."""

        if not hasattr(self, "_hdf5_file"):
            return
        super().close_and_delete_hdf5_handle()


def _owned_dataset_factory(resources: Any) -> Callable[..., _OwnedSequenceDataset]:
    """Bind the lifecycle owner without putting it in provider kwargs."""

    def factory(**kwargs: Any) -> _OwnedSequenceDataset:
        return _OwnedSequenceDataset(resources, **kwargs)

    return factory


def _dataset_factory(
    config: Any,
    obs_keys: list[str],
    resources: Any,
    *,
    lang_conditioned: bool,
    filter_by_attribute: str | None = None,
) -> Any:
    """Build datasets through Robomimic's factory with owned leaves."""

    from robomimic.utils import train_utils as TrainUtils

    if lang_conditioned:
        ds_langs = [ds_cfg.get("lang", "dummy") for ds_cfg in config.train.data]
    else:
        ds_langs = [None for _ in config.train.data]

    ds_kwargs = dict(
        obs_keys=obs_keys,
        action_keys=config.train.action_keys,
        dataset_keys=config.train.dataset_keys,
        action_config=config.train.action_config,
        load_next_obs=config.train.hdf5_load_next_obs,
        frame_stack=config.train.frame_stack,
        seq_length=config.train.seq_length,
        pad_frame_stack=config.train.pad_frame_stack,
        pad_seq_length=config.train.pad_seq_length,
        get_pad_mask=False,
        goal_mode=config.train.goal_mode,
        hdf5_cache_mode=config.train.hdf5_cache_mode,
        hdf5_use_swmr=config.train.hdf5_use_swmr,
        hdf5_normalize_obs=config.train.hdf5_normalize_obs,
        filter_by_attribute=filter_by_attribute,
    )
    ds_kwargs["hdf5_path"] = [ds_cfg["path"] for ds_cfg in config.train.data]
    ds_kwargs["filter_by_attribute"] = [
        ds_cfg.get("filter_key", filter_by_attribute) for ds_cfg in config.train.data
    ]
    ds_kwargs["demo_limit"] = [
        ds_cfg.get("demo_limit", None) for ds_cfg in config.train.data
    ]
    ds_weights = [ds_cfg.get("weight", 1.0) for ds_cfg in config.train.data]

    return TrainUtils.get_dataset(
        ds_class=_owned_dataset_factory(resources),
        ds_kwargs=ds_kwargs,
        ds_weights=ds_weights,
        ds_langs=ds_langs,
        normalize_weights_by_ds_size=config.train.normalize_weights_by_ds_size,
        meta_ds_class=MetaDataset,
        meta_ds_kwargs={},
    )


def load_training_datasets(
    config: Any,
    obs_keys: list[str],
    resources: Any,
) -> tuple[Any, Any]:
    """Load train and validation datasets with constructor ownership."""

    from robomimic.utils import file_utils as FileUtils
    from robomimic.utils import lang_utils as LangUtils

    # Robomimic drops this synthetic key from the caller's list in place, and
    # the saved checkpoint shape metadata relies on that.  Do it once so the
    # validation split stays language-conditioned too.
    lang_conditioned = LangUtils.LANG_EMB_OBS_KEY in obs_keys
    if lang_conditioned:
        obs_keys.remove(LangUtils.LANG_EMB_OBS_KEY)

    train_filter_by_attribute = config.train.hdf5_filter_key
    valid_filter_by_attribute = config.train.hdf5_validation_filter_key
    if valid_filter_by_attribute is not None:
        assert config.experiment.validate, (
            "specified validation filter key {}, but config.experiment.validate "
            "is not set"
        ).format(valid_filter_by_attribute)

    if config.experiment.validate:
        assert (train_filter_by_attribute is not None) and (
            valid_filter_by_attribute is not None
        ), (
            "did not specify filter keys corresponding to train and valid split "
            "in dataset - please fill config.train.hdf5_filter_key and "
            "config.train.hdf5_validation_filter_key"
        )
        assert isinstance(config.train.data, list), (
            "config.train.data should be a list of datasets, not a single dataset"
        )
        for dataset_cfg in config.train.data:
            train_demo_keys = FileUtils.get_demos_for_filter_key(
                hdf5_path=os.path.expanduser(dataset_cfg["path"]),
                filter_key=train_filter_by_attribute,
            )
            valid_demo_keys = FileUtils.get_demos_for_filter_key(
                hdf5_path=os.path.expanduser(dataset_cfg["path"]),
                filter_key=valid_filter_by_attribute,
            )
            assert set(train_demo_keys).isdisjoint(set(valid_demo_keys)), (
                "training demonstrations overlap with validation demonstrations!"
            )
        train_dataset = _dataset_factory(
            config,
            obs_keys,
            resources,
            lang_conditioned=lang_conditioned,
            filter_by_attribute=train_filter_by_attribute,
        )
        valid_dataset = _dataset_factory(
            config,
            obs_keys,
            resources,
            lang_conditioned=lang_conditioned,
            filter_by_attribute=valid_filter_by_attribute,
        )
    else:
        train_dataset = _dataset_factory(
            config,
            obs_keys,
            resources,
            lang_conditioned=lang_conditioned,
            filter_by_attribute=train_filter_by_attribute,
        )
        valid_dataset = None

    return train_dataset, valid_dataset


__all__ = ["load_training_datasets"]
