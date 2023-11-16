#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file main.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

# from evo.core.trajectory import PosePath3D, PoseTrajectory3D
# from evo.core import metrics, sync
# from evo.tools import file_interface, plot
# from evo.main_rpe import rpe
# from evo.main_ape import ape
# from evo.core.result import Result


from dataclasses import dataclass
import numpy as np
import os
from typing import Dict, List

# from matplotlib import pyplot as plt

import yaml

from pathlib import Path

from utils import MethodBase, TestResult, NavSlamError, EvoEvaluation

CONFIG_PREFIX = Path(__file__).resolve().parent.parent

class Evaluate:
    def __init__(self, mlist, config_file: Path):
        self._methods_list = mlist
        self._params = None
        with open(config_file, "r") as f:
            self._params = yaml.safe_load(f)
        assert self._params

        self._methods = []

    def run(self):
        # Loop over paths.
        for pfile in self._params["path_files"]:
            print(f"Processing {pfile} ...")
            # Loop over methods.
            for method_name in self._methods_list:
                print(f"Processing {method_name} ...")
                method = MethodBase.create_slam_method(method_name)
                # Loop over rounds.
                for trial in range(self._params["trials"]):
                    print(f"Processing trial {trial} ...")
                    prefix = Path(self._params["result_dir"]) / pfile / ("trial" + str(trial))
                    # Load test result.
                    test_result = self.__load_test_result(prefix)

                    # Evaluate slam error.
                    nav_slam_error = self.__compute_error(test_result)

                    print(f"est_rmse: {nav_slam_error.est_rmse:.02f} m, nav_rmse: {nav_slam_error.nav_rmse:.02f} m")

                    method.add_result(test_result, nav_slam_error)
                    
                    print(f"Done trial {trial}.")

                # Evaluate navigation precision.
                self.__compute_wpt_precision(method)
                self._methods.append(method)
                print(f"Done {method_name}.")
            print(f"Done {pfile}.")

    def __compute_error(self, test_result: TestResult) -> NavSlamError:
        # Call evo package
        return EvoEvaluation().run(test_result)

    def __compute_wpt_precision(self, method: MethodBase):
        act_wpts = np.full([method.errs[0].wpts_count, 3, len(method.errs)], np.nan)
        for k, result in enumerate(method.results):
            count = result.act_stamped_wpts.shape[0]
            act_wpts[:count, :, k] = result.act_stamped_wpts[:, 1:]
        xy_mean = np.nanmean(act_wpts[:, :2, :], axis=-1)
        print(xy_mean)
        print(act_wpts)
        xy_std = np.nanstd(np.linalg.norm(act_wpts[:, :2, :] - xy_mean[:, :, None], axis=1), axis=-1)
        theta_std = np.nanstd(act_wpts[:, 2, :], axis=-1)
        method.set_wpt_precision(xy_std, theta_std)

    def __load_test_result(self, prefix: Path)->TestResult:
        # @TODO (yanwei) does it hold true for robot angle?
        offset = np.array(self._params["robot_init_pose"])
        planned_wpts = np.loadtxt(prefix / "planned_waypoints.txt") - offset
        # Any efficient way to do the following in-place subtraction?
        reached_stamped_wpts = np.loadtxt(prefix / "actual_path.txt")
        wpts = reached_stamped_wpts[:, 1:]
        wpts = wpts - offset
        reached_stamped_wpts[:, 1:] = wpts
        act_poses = np.loadtxt(prefix / "gt_poses.txt")
        est_poses = np.loadtxt(prefix / "et_poses.txt")

        act_stamped_wpts = []
        for wpt in reached_stamped_wpts:
            timediffs = np.abs(wpt[0] - act_poses[:, 0])
            val = np.min(timediffs)
            index = np.argmin(timediffs)
            assert val < 1e-2 # seconds
            act_stamped_wpts.append(act_poses[index, [0, 1, 2]].tolist() + [np.arccos(act_poses[index, -1]) * 2.0])

        # @TODO slam poses and extrinsics
        return TestResult(planned_wpts, reached_stamped_wpts, np.array(act_stamped_wpts), act_poses, est_poses)

        @property
        def methods(self):
            return self._methods

if __name__ == "__main__":

    methods_list = ["slam_toolbox",
    ]
    config_file = CONFIG_PREFIX / "config.yml"

    # Run evaluation.
    eva = Evaluate(methods_list, config_file)
    eva.run()

    # Run visualization.
    vis = Visualizer(config_file, eva.methods)
    vis.run()

    print("Main Script Done.")
