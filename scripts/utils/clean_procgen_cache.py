#!/root/isaac-sim/python.sh
"""
Utility script for removing the cache of procedural assets generated by the Space Robotics Bench

Usage:
    ros2 run space_robotics_bench clean_procgen_cache.py
"""

import shutil

import platformdirs


def main():
    cache_dir = platformdirs.user_cache_dir("srb")

    response = input(
        f"Do you want to remove the cache of space_robotics_bench located at '{cache_dir}'? (y/N): "
    )

    if response.lower() in ["y", "yes"]:
        shutil.rmtree(cache_dir, ignore_errors=True)
        print("Cache removed")
    else:
        print("Exiting without removing the cache")


if __name__ == "__main__":
    main()
