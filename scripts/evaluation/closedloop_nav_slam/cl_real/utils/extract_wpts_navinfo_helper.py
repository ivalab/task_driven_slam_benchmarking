#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file extract_wpts_navinfo_helper.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 04-29-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


"""
This helper creates visited waypoints with nav info.
Input file: timestamp x y theta
Outpu file: timestamp x y theta wpt_index loop_index
"""

import glob
from pathlib import Path

import numpy as np

DATA_DIR = "/mnt/DATA/experiments/closedloop_nav_slam/cl_nav_slam_test/realworld/0406/orb3/path1"

nav_dirs = sorted(glob.glob(DATA_DIR + "/*"))

for nav_dir in nav_dirs:
    prefix = Path(nav_dir)
    # Load planned wpts
    planned_wpts = np.loadtxt(prefix / "planned_waypoints.txt", ndmin=2)
    wpts_count = planned_wpts.shape[0]

    # Load visited wpts
    visited_wpts = np.loadtxt(prefix / "visited_waypoints.txt", ndmin=2)

    nav_info = []
    for wpt_index, wpt in enumerate(visited_wpts[:, 1:]):
        diff = np.linalg.norm(planned_wpts - wpt, axis=-1)
        print(diff)
        min_val = np.min(diff)
        assert min_val < 1e-6
        min_index = np.argmin(diff)

        nav_info.append([min_index, wpt_index // wpts_count])

    np.savetxt(
        prefix / "visited_waypoints_nav_info.txt",
        np.column_stack([visited_wpts, nav_info]),
        fmt="%.06f",
        header="timestamp x y theta wpt_index loop_index",
    )
