#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file test_onekey.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-25-2024
@version 1.0
@license Copyright (c) 2024
@desc None
'''

import unittest
from pathlib import Path

from closedloop_nav_slam.ros.onekey import CentralManager

CONFIG_FILE = Path(__file__).resolve().parent.parent.parent / "closedloop_nav_slam/settings/config.yaml"

class TestOnekey(unittest.TestCase):
    """Test class onekey."""

    def setUp(self):
        super().setUp()

        self.obj: CentralManager = CentralManager(CONFIG_FILE)

    def test_save_map(self):

        self.obj.__set_map("dummy_slam", "dummy_path", 0)

if __name__ == "__main__":
    unittest.main()