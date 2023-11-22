#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file utils.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""


from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Optional
import yaml

import numpy as np
from evo.core import metrics, sync
from evo.core.result import Result as EvoResult
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.main_ape import ape as evo_ape
from evo.main_rpe import rpe as evo_rpe
from evo.tools import file_interface, plot


# class syntax
class MethodType(Enum):
    MAPPING = 1
    LOCALIZATION = 2
    SLAM = 3
    ODOMETRY = 4


class SensorType(Enum):
    LASER = 1
    LIDAR = 2
    STEREO = 3
    GROUND_TRUTH = 4


@dataclass
class NavSlamError:
    nav_errs: np.array  # Nx1
    nav_rmse: float
    est_errs: np.array  # Mx1
    est_rmse: float
    success_rate: float  # len(reached_wpts) / len(planned_wpts)
    wpts_errs: np.ndarray # wpts_num * 3 (x, y, theta)


@dataclass
class NavSlamData:
    planned_wpts: np.ndarray  # [x, y, theta]
    reached_stamped_wpts: np.ndarray  # [timestamp, x, y, theta]
    act_stamped_wpts: np.ndarray  # [timestamp, x, y, theta]
    act_poses: np.ndarray  # [timestamp, x, y, z, qx, qy, qz, qw]
    est_poses: np.ndarray  # [timestamp, x, y, z, qx, qy, qz, qw]
    traj_length: float  # meters


@dataclass
class SingleTrialInfo:
    data: NavSlamData
    err: NavSlamError


class TrajectoryInfo:
    def __init__(self):
        self._data_dict: Dict[int, SingleTrialInfo] = {}
        self._all_wpts_errs = np.full((0, 3), np.nan)
        self._accuracy = np.full((0, 2), np.nan)  # xy, angle
        self._precision = np.full((0, 2), np.nan) # xy, angle
        self._success_rate = None

    def is_valid(self) -> bool:
        return len(self._data_dict) > 0

    def get_planned_wpts_count(self) -> int:
        assert len(self._data_dict) > 0
        return self._data_dict[list(self._data_dict.keys())[0]].data.planned_wpts.shape[0]

    def add_result(self, trial: int, data: NavSlamData, err: NavSlamError):
        self._data_dict[trial] = SingleTrialInfo(data, err)
        self._all_wpts_errs = np.vstack([self._all_wpts_errs, err.wpts_errs])

    def get_result(self, trial: int) -> Optional[SingleTrialInfo]:
        if trial in self._data_dict.keys():
            return self._data_dict[trial]
        return None

    def compute_accuracy_and_precision(self):
        """Compuate accuracy and precision of each planned wpt"""
        self._success_rate = np.full((self.get_planned_wpts_count()), 0.0)
        planned_wpts = None
        act_wpts = np.full((self.get_planned_wpts_count(), 3, len(self._data_dict)), np.nan)
        for trial, result in self._data_dict.items():
            data = result.data
            act_wpts[:len(data.reached_stamped_wpts), :, trial] = data.act_stamped_wpts[:, 1:] # assumes the wpts are reached sequentially without skipping
            planned_wpts = data.planned_wpts # stays the same for each trial
            # Compute success rate``
            self._success_rate[:data.reached_stamped_wpts.shape[0]] += 1.0
        self._success_rate /= len(self._data_dict.keys())

        # Compute accuracy
        self._accuracy = np.full((self.get_planned_wpts_count(), 2), np.nan)
        self._accuracy[:, 0] = np.nanmean(
            np.linalg.norm(act_wpts[:, :2, :] - planned_wpts[:, :2, None], axis=1), axis=-1
        )
        self._accuracy[:, 1] = np.nanmean(
            np.linalg.norm(act_wpts[:, -1, :] - planned_wpts[:, -1, None], axis=1), axis=-1
        )

        # Compute precision
        self._precision = np.full((self.get_planned_wpts_count(), 2), np.nan)
        wpt_mean = np.nanmean(act_wpts, axis=-1)
        self._precision[:, 0] = np.nanmean(
            np.linalg.norm(act_wpts[:, :2, :] - wpt_mean[:, :2, None], axis=1), axis=-1
        )
        self._precision[:, 1] = np.nanmean(
            np.linalg.norm(act_wpts[:, -1, :] - wpt_mean[:, -1, None], axis=1), axis=-1
        )

    @property
    def accuracy(self):
        return self._accuracy

    @property
    def precision(self):
        return self._precision

    @property
    def success_rate(self):
        return self._success_rate

    @property
    def data_dict(self):
        return self._data_dict

    @property
    def all_wpts_errs(self):
        return self._all_wpts_errs

    def get_avg_accuracy(self):
        result = np.nanmean(self._accuracy, axis=0)
        assert  2 == result.shape[0]
        return result

    def get_avg_precision(self):
        return np.nanmean(self._precision, axis=0)

    def get_avg_success_rate(self):
        return np.nanmean(self._success_rate)

class MethodBase:
    def __init__(self, name: str, sensor_type: str, method_type: str):
        self.name = name
        self.sensor_type = SensorType[sensor_type.upper()]
        self.method_type = MethodType[method_type.upper()]

        # Navigation data.
        self.traj_dict: Dict[str, TrajectoryInfo] = {}

    def add_result(self, name: str, trial: int, data: NavSlamData, err: NavSlamError):
        if name not in self.traj_dict.keys():
            self.traj_dict[name] = TrajectoryInfo()
        self.traj_dict[name].add_result(trial, data, err)

    @staticmethod
    def create_slam_method(params: dict):
        return MethodBase(params["slam_method"], params["slam_sensor_type"], params["mode"])

class EvoEvaluation:
    def __init__(self):
        pass

    def run(self, nav_slam_data: NavSlamData) -> NavSlamError:
        # Compute estimation error.
        traj_act = self.__convertToEvoFormat(nav_slam_data.act_poses)
        traj_est = self.__convertToEvoFormat(nav_slam_data.est_poses)

        # Assuming act and est are from the same frame id (e.g body)
        traj_act, traj_est = sync.associate_trajectories(traj_act, traj_est)

        # traj_method.align_origin(traj_ref)
        # Call APE
        ape_result = evo_ape(traj_act, traj_est, pose_relation=metrics.PoseRelation.translation_part)
        est_rmse = ape_result.stats["rmse"]
        est_errs = ape_result.np_arrays["error_array"]  # ["timestamps"]

        # Compute success rate
        wpts_count = nav_slam_data.planned_wpts.shape[0]
        wpts_act_count = nav_slam_data.reached_stamped_wpts.shape[0]
        s_ratio = wpts_act_count / wpts_count

        # Assume the robot navigates to each wpt sequentially and won't skip any previous one.
        wpts_errs = np.full((wpts_act_count, 3), np.nan)
        nav_errs = np.full((wpts_act_count, 1), np.nan)
        nav_rmse = np.nan
        if s_ratio > 0.0:
            wpts_errs = nav_slam_data.reached_stamped_wpts[:, 1:] - nav_slam_data.act_stamped_wpts[:, 1:]
            nav_errs = np.linalg.norm(wpts_errs[:, :2], axis=1) # ignore orientation
            nav_rmse = np.sqrt(np.mean(nav_errs**2))

        return NavSlamError(nav_errs, nav_rmse, est_errs, est_rmse, s_ratio, wpts_errs)

    def __convertToEvoFormat(self, mat: np.ndarray) -> PoseTrajectory3D:
        stamps = mat[:, 0]  # n x 1
        xyz = mat[:, 1:4]  # n x 3
        quat = mat[:, 4:8]  # n x 4
        quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
        return PoseTrajectory3D(xyz, quat, stamps)
