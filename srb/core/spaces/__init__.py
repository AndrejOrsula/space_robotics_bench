"""Pure gym-space reflection and tensor coercion primitives."""

from srb.core.spaces.coerce import (
    coerce_gym_step_outputs,
    coerce_obs,
    to_numpy,
    to_torch,
)
from srb.core.spaces.flatten import autobound_action_space, flatten_dict_to_box
from srb.core.spaces.keys import ROOT_OBSERVATION_KEY
from srb.core.spaces.predicates import is_box, is_dict, is_discrete, is_tuple
from srb.core.spaces.walk import apply_to_obs, map_boxes, walk_leaves

__all__ = [
    "ROOT_OBSERVATION_KEY",
    "apply_to_obs",
    "autobound_action_space",
    "coerce_gym_step_outputs",
    "coerce_obs",
    "flatten_dict_to_box",
    "is_box",
    "is_dict",
    "is_discrete",
    "is_tuple",
    "map_boxes",
    "to_numpy",
    "to_torch",
    "walk_leaves",
]
