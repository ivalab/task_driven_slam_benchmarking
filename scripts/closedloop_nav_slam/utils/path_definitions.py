#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file path_definitions.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-28-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import os
from pathlib import Path

import rospkg

# Get the path of a package
ROSPACK = rospkg.RosPack()
PKG_NAME = "closedloop_nav_slam"
PKG_PATH = Path(ROSPACK.get_path(PKG_NAME))

CONFIGS_PATH = PKG_PATH / "configs"
PARAMS_PATH = CONFIGS_PATH / "params"
SLAM_CONFIGS_PATH = PARAMS_PATH / "slam"
SCRIPTS_PATH = PKG_PATH / "scripts"
NODES_PATH = SCRIPTS_PATH / "nodes"
UTILS_PATH = SCRIPTS_PATH / PKG_NAME / "utils"
TOOLS_PATH = SCRIPTS_PATH / "tools"

if __name__ == "__main__":

    path_list = [PKG_PATH, CONFIGS_PATH, SLAM_CONFIGS_PATH, SCRIPTS_PATH, NODES_PATH, UTILS_PATH, TOOLS_PATH]
    for p in path_list:
        print(f"Path: {p}")
        assert os.path.exists(p), f"{p} does NOT exist."
