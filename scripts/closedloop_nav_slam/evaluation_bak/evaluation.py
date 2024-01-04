#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file evaluation.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 11-16-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from pathlib import Path

from typing import List, Optional

import numpy as np
import yaml
from closedloop_nav_slam.evaluation.utils import EvoEvaluation, MethodBase, NavSlamError, NavSlamData, TrajectoryInfo

from closedloop_nav_slam.utils.path_definitions import *

class Evaluation:
    def __init__(self, mlist, config_file: Path, save_results: bool = False):
        self._methods_list = mlist
        self._params = None
        with open(config_file, "r") as f:
            self._params = yaml.safe_load(f)
        assert self._params

        self._methods: List[MethodBase] = []
        self._result_prefix = Path(self._params["result_dir"]) / self._params["test_type"] / self._params["env_name"]
        self._save_results = save_results

    @property
    def methods(self):
        return self._methods

    def run(self):
        # Loop over methods.
        for method_name in self._methods_list:
            # Load SLAM testing parameters.
            self._params.update(self.__load_slam_params(SLAM_SETTINGS_PATH / (method_name + ".yaml")))
            method = MethodBase.create_slam_method(self._params)
            method_dir = self._result_prefix / method_name
            # Loop over paths.
            for pfile in self._params["path_files"]:
                pfile_dir = method_dir / pfile
                # Loop over rounds.
                for trial in range(self._params["trials"]):
                    print(f"Processing {method_name}, {pfile}, {trial} ...")
                    prefix = pfile_dir / ("trial" + str(trial))
                    # Load nav slam data
                    nav_slam_data = self.__load_nav_slam_data(prefix, method_name)
                    if nav_slam_data is None:
                        continue
                    # Evaluate slam error.
                    nav_slam_error = self.__compute_nav_slam_error(nav_slam_data)
                    print(f"est_rmse: {nav_slam_error.est_rmse:.02f} m, nav_rmse: {nav_slam_error.nav_rmse:.02f} m success rate: {nav_slam_error.success_rate}")

                    # Add result to method.
                    method.add_result(pfile, trial, nav_slam_data, nav_slam_error)
                    print(f"Done {method_name}, {pfile}, {trial} ...")

            # Evaluate navigation precision.
            self.__compute_precision(method)

            # Add method.
            self._methods.append(method)
            print(f"Done {method_name}.")

    def __load_slam_params(self, slam_params_file: Path):
        params = None
        with open(slam_params_file, "r") as f:
            params = yaml.safe_load(f)
        assert params
        return params

    def __load_nav_slam_data(self, prefix: Path, method_name: str) -> Optional[NavSlamData]:

        gt_poses_path = prefix / "gt_poses.txt"
        et_poses_path = prefix / "et_poses.txt"
        et_slam_poses_path = prefix / "et_slam_poses.txt"
        planned_wpts_path = prefix / "planned_waypoints.txt"
        actual_wpts_path = prefix / "actual_path.txt"

        if not gt_poses_path.exists() or not (et_poses_path.exists() or et_slam_poses_path.exists()) or not planned_wpts_path.exists() or not actual_wpts_path.exists():
            return None


        # @TODO (yanwei) does it hold true for robot angle?
        offset = np.array(self._params["robot_init_pose"])
        planned_wpts = np.loadtxt(planned_wpts_path, ndmin=2)
        # Any efficient way to do the following in-place subtraction?
        reached_stamped_wpts = np.loadtxt(actual_wpts_path, ndmin=2)
        # wpts = reached_stamped_wpts[:, 1:]
        # wpts = wpts - offset
        # reached_stamped_wpts[:, 1:] = wpts

        # Find the start point(timestamp), skip the mapping data.
        start_timestamp, end_timestamp = self.__find_start_and_end_timestamp(planned_wpts, reached_stamped_wpts)
        reached_stamped_wpts = reached_stamped_wpts[(end_timestamp >= reached_stamped_wpts[:, 0]) & (reached_stamped_wpts[:, 0] >= start_timestamp), :]
        act_poses = np.loadtxt(gt_poses_path, ndmin=2)
        et_poses = []
        if et_poses_path.exists():
            est_poses = np.loadtxt(et_poses_path, ndmin=2)
        else:
            est_poses = np.loadtxt(et_slam_poses_path, ndmin=2)
        if "perfect_odometry" != method_name:
            est_poses[:, 1:3] += offset[:2]
        act_poses = act_poses[act_poses[:, 0] >= start_timestamp, :]
        est_poses = est_poses[est_poses[:, 0] >= start_timestamp, :]

        # Find the robot actual pose at each waypoint.
        act_stamped_wpts = []
        for wpt in reached_stamped_wpts:
            timediffs = np.abs(wpt[0] - act_poses[:, 0])
            val = np.min(timediffs)
            index = np.argmin(timediffs)
            assert val < 1e-2  # seconds
            act_stamped_wpts.append(act_poses[index, [0, 1, 2]].tolist() + [self.__quat_to_yaw(act_poses[index, -4:])])

        traj_length = self.__compute_traj_length(act_poses)
        # @TODO slam poses and extrinsics
        return NavSlamData(
            planned_wpts, reached_stamped_wpts, np.array(act_stamped_wpts), act_poses, est_poses, traj_length
        )

    def __find_start_and_end_timestamp(self, planned_wpts, reached_stamped_wpts) -> [float, float]:
        """Find start timestamp where the robot starts doing localization in a known map if has"""
        assert self._params["loops"] > 0
        if self._params["loops"] == 1: # Localization or Odometry
            return [-1.0, reached_stamped_wpts[-1, 0]]
        index = planned_wpts.shape[0] * (self._params["loops"] - 1) - 1# the last mapping point
        assert index >= 0
        if reached_stamped_wpts.shape[0] <= index + 1:
            # It means mission failed, the robot does NOT even finish mapping.
            start_timestamp = reached_stamped_wpts[-1, 0] + 1e-3 # safe margin, ignore all mapping data
            end_timestamp = start_timestamp
        else:
            start_timestamp = reached_stamped_wpts[index, 0] + 1e-3 # safe margin
            end_timestamp = reached_stamped_wpts[min(index + planned_wpts.shape[0], reached_stamped_wpts.shape[0]-1), 0]
        return start_timestamp, end_timestamp

    def __quat_to_yaw(self, quat: np.array) -> float:
        """Convert quat (x, y, z, w) to yaw in radian, assume zerop pitch and roll"""
        assert quat.shape == (4,) or quat.shape == (4, 1) or quat.shape == (1, 4)
        # assert np.abs(quat[0]) < 1e-5 and np.abs(quat[1]) < 1e-5
        return 2.0 * np.arctan2(quat[2], quat[3])


    def __compute_traj_length(self, stamped_poses: np.ndarray) -> float:
        """Compute trajectory length in xy plane"""
        return np.sum(np.linalg.norm(np.diff(stamped_poses[:, 1:3], axis=0), axis=1))

    def __compute_nav_slam_error(self, nav_slam_data: NavSlamData) -> NavSlamError:
        # Call evo package
        return EvoEvaluation().run(nav_slam_data)

    def __compute_precision(self, method: MethodBase):
        for name, traj in method.traj_dict.items():
            traj.compute_accuracy_and_precision()