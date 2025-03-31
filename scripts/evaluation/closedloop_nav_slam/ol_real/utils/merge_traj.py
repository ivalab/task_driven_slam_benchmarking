#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file tmp_merge_traj.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 07-08-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


from ol_real.utils.utils import merge_trajectory
from common.utils import align_trajectories
from common.datasets import EUROC_SEQUENCES
import numpy as np
from pathlib import Path

# Define Parameters (@TODO: yaml)
RESULT_DIR = Path("/mnt/DATA/experiments/closedloop_nav_slam/cl_nav_slam_test/ol/euroc/regular/")
METHODS = ["gfgg", "orb3"]
FEATURE_NUM = {
    "gfgg": 300,
    "orb3": 800,
}
ROUNDS = 5

for mehtod_index, method_name in enumerate(METHODS):
    for seq_index, seq_name in enumerate(EUROC_SEQUENCES):
        input_dir = RESULT_DIR / method_name / "_Speedx1.0"
        output_filename = Path(str(input_dir).replace("_Speedx1.0", "")) / f"{seq_name}_AllFrameTrajectory.txt"

        trajs = []
        for round_index in range(1, ROUNDS + 1):
            input_filename = (
                input_dir
                / f"ObsNumber_{FEATURE_NUM[method_name]}_Round{round_index}"
                / f"{seq_name}_AllFrameTrajectory.txt"
            )

            msg = f"{method_name}-{seq_name}-round{round_index}"
            print(f"Processing {msg} ...")

            poses = np.loadtxt(input_filename)
            if round_index == 1:
                trajs.append(poses)
            else:
                aligned_poses = np.empty_like(poses)
                try:
                    _, aligned_poses = align_trajectories(trajs[0], poses)
                except Exception as e:
                    print(f"Trajectory alignment error: {msg}, {e}")
                trajs.append(aligned_poses)

        trajs_arr = merge_trajectory(trajs)
        np.savetxt(output_filename, trajs_arr, fmt="%.6f")
        # break
    # break

print("Done")
