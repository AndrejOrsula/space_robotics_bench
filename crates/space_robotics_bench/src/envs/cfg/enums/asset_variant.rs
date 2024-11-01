use pyo3::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "json", derive(display_json::DisplayAsJson))]
#[pyclass(frozen, eq, eq_int, hash, rename_all = "SCREAMING_SNAKE_CASE")]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AssetVariant {
    #[serde(alias = "none")]
    None,
    #[default]
    #[serde(alias = "primitive", alias = "PRIM", alias = "prim")]
    Primitive,
    #[serde(alias = "dataset", alias = "DB", alias = "db")]
    Dataset,
    #[serde(
        alias = "procedural",
        alias = "PROC",
        alias = "proc",
        alias = "PROCGEN",
        alias = "procgen"
    )]
    Procedural,
}
