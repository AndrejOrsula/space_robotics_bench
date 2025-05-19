use crate::env_cfg::{ObjectType, RobotType, SceneryType};
use serde::de::DeserializeOwned;
use std::{collections::HashMap, fs, path::PathBuf, str::FromStr};
use tracing::{error, warn};

const ENV_CACHE_FILE: &str = "env.json";
const ROBOT_CACHE_FILE: &str = "robot.json";
const OBJECT_CACHE_FILE: &str = "object.json";
const SCENERY_CACHE_FILE: &str = "scenery.json";

pub type EnvCache = Vec<String>;
pub type RobotCache = HashMap<RobotType, HashMap<String, Vec<String>>>;
pub type ObjectCache = HashMap<ObjectType, Vec<String>>;
pub type SceneryCache = HashMap<SceneryType, Vec<String>>;

fn get_cache_path(filename: &str) -> Option<PathBuf> {
    let path = PathBuf::from_str(concat!(env!("CARGO_MANIFEST_DIR"), "/../../.cache"))
        .unwrap()
        .join(filename);
    if path.exists() {
        Some(path)
    } else {
        warn!(
            "Cache file not found at expected SRB root location: {}",
            path.display()
        );
        None
    }
}

pub fn load_cache<T: DeserializeOwned + Default>(filename: &str) -> T {
    if let Some(path) = get_cache_path(filename) {
        match fs::read_to_string(&path) {
            Ok(content) => match serde_json::from_str(&content) {
                Ok(data) => data,
                Err(e) => {
                    error!(
                        "Failed to parse cache file '{}': {}. Using default.",
                        path.display(),
                        e
                    );
                    Default::default()
                }
            },
            Err(e) => {
                error!(
                    "Failed to read cache file '{}': {}. Using default.",
                    path.display(),
                    e
                );
                Default::default()
            }
        }
    } else {
        error!("Cache file '{}' not found. Using default.", filename);
        Default::default()
    }
}

pub fn load_env_cache() -> EnvCache {
    load_cache(ENV_CACHE_FILE)
}

pub fn load_robot_cache() -> RobotCache {
    load_cache(ROBOT_CACHE_FILE)
}

pub fn load_object_cache() -> ObjectCache {
    load_cache(OBJECT_CACHE_FILE)
}

pub fn load_scenery_cache() -> SceneryCache {
    load_cache(SCENERY_CACHE_FILE)
}

pub fn get_all_robot_names(cache: &RobotCache) -> Vec<String> {
    let mut names: Vec<String> = cache
        .values()
        .flat_map(|subtypes_map| subtypes_map.values())
        .flatten()
        .cloned()
        .collect();
    names.sort();
    names.dedup();
    names
}

pub fn get_all_asset_names<T>(cache: &HashMap<T, Vec<String>>) -> Vec<String>
where
    T: Eq + std::hash::Hash,
{
    let mut names: Vec<String> = cache.values().flatten().cloned().collect();
    names.sort();
    names.dedup();
    names
}

pub fn get_object_names_by_subtype(cache: &ObjectCache, subtype_str: &str) -> Vec<String> {
    let mut names = vec![String::new(), "none".to_string()];

    match subtype_str.parse::<ObjectType>() {
        Ok(key) => {
            if let Some(entries) = cache.get(&key) {
                names.extend(entries.iter().cloned());
            }
        }
        Err(_) => {
            if !subtype_str.is_empty() {
                warn!(
                    "Unknown or unmapped object subtype requested: {}",
                    subtype_str
                );
            }
        }
    }

    names.sort();
    names.dedup();
    names
}
