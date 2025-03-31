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

SCRIPTS_PATH = Path(__file__).resolve().parent.parent
UTILS_PATH = SCRIPTS_PATH / "utils"
CONFIG_PATH = SCRIPTS_PATH / "config"
TEST_PATH = SCRIPTS_PATH / "test"

if __name__ == "__main__":
    print(f"SCRIPTS_PATH: {SCRIPTS_PATH}")
    assert os.path.exists(SCRIPTS_PATH)
    print(f"UTILS_PATH: {UTILS_PATH}")
    assert os.path.exists(UTILS_PATH)
    print(f"CONFIG_PATH: {CONFIG_PATH}")
    assert os.path.exists(CONFIG_PATH)
