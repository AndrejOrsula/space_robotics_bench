from pathlib import Path

# Path to repository root directory
SRB_DIR = Path(__file__).resolve().parent.parent.parent

# Path to apps (experience) directory
SRB_APPS_DIR = SRB_DIR.joinpath("apps")

# Path to assets directory
SRB_ASSETS_DIR = SRB_DIR.joinpath("assets")
SRB_ASSETS_DIR_SRB = SRB_ASSETS_DIR.joinpath("srb_assets")
SRB_ASSETS_DIR_SRB_OBJECT = SRB_ASSETS_DIR_SRB.joinpath("object")
SRB_ASSETS_DIR_SRB_ROBOT = SRB_ASSETS_DIR_SRB.joinpath("robot")
SRB_ASSETS_DIR_SRB_SCENERY = SRB_ASSETS_DIR_SRB.joinpath("scenery")
SRB_ASSETS_DIR_SRB_SKYDOME = SRB_ASSETS_DIR_SRB.joinpath("skydome")
SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES = SRB_ASSETS_DIR_SRB_SKYDOME.joinpath("low_res")
SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES = SRB_ASSETS_DIR_SRB_SKYDOME.joinpath("high_res")

# Path to hyperparameters directory
SRB_HYPERPARAMS_DIR = SRB_DIR.joinpath("hyperparams")

# Path to logs
SRB_LOGS_DIR = SRB_DIR.joinpath("logs")

# Path to a cached list of registered entities
SRB_CACHE_PATH = SRB_DIR.joinpath(".cache")
SRB_ENV_CACHE_PATH = SRB_CACHE_PATH.joinpath("env.json")
SRB_OBJECT_CACHE_PATH = SRB_CACHE_PATH.joinpath("object.json")
SRB_ROBOT_CACHE_PATH = SRB_CACHE_PATH.joinpath("robot.json")
SRB_SCENERY_CACHE_PATH = SRB_CACHE_PATH.joinpath("scenery.json")
SRB_HARDWARE_INTERFACE_CACHE_PATH = SRB_CACHE_PATH.joinpath("hardware.json")
