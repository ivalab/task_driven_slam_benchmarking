#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file utils.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''


import numpy as np

from enum import Enum

from dataclasses import dataclass

from pathlib import Path


from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.core import metrics, sync
from evo.tools import file_interface, plot
from evo.main_rpe import rpe as evo_rpe
from evo.main_ape import ape as evo_ape
from evo.core.result import Result as EvoResult

# class syntax
class MethodType(Enum):
    MAPPING = 1
    LOCALIZATION = 2
    SLAM = 3

class SensorType(Enum):
    LASER = 1
    LIDAR = 2
    CAMERA = 3

@dataclass
class NavSlamError:
    nav_errs: np.array # Nx1
    nav_rmse: float
    est_errs: np.array # Mx1
    est_rmse: float
    wpts_count: int
    wpts_act_count: int

@dataclass
class TestResult:
    planned_wpts: np.ndarray
    reached_stamped_wpts: np.ndarray
    act_stamped_wpts: np.ndarray
    act_poses: np.ndarray
    est_poses: np.ndarray


class MethodBase:
    def __init__(self, name: str, sensor_type: SensorType, method_type: MethodType):
        self.name = name
        self.sensor_type = sensor_type
        self.method_type = method_type

        # Navigation data.
        self.results = []
        self.errs = []

    def add_result(self, result: TestResult, err: NavSlamError):
        self.results.append(result)
        self.errs.append(err)

    def set_wpt_precision(self, xy_std, theta_std):
        self.wpt_precision = np.vstack([xy_std, theta_std]).reshape(-1, 2)


    @staticmethod
    def create_slam_method(name):
        if "slam_toolbox" == name:
            return MethodBase(name, SensorType.LASER, MethodType.SLAM)
        return None


class EvoEvaluation:
    def __init__(self):
        pass

    def run(self, test_result: TestResult) -> NavSlamError:

        # Compute estimation error.
        traj_act = self.__convertToEvoFormat(test_result.act_poses)
        traj_est = self.__convertToEvoFormat(test_result.est_poses)

        # Assuming act and est are from the same frame id (e.g body)
        traj_act, traj_est = sync.associate_trajectories(traj_act, traj_est)

        # traj_method.align_origin(traj_ref)
        # Call APE
        ape_result = evo_ape(traj_act, traj_est, pose_relation=metrics.PoseRelation.translation_part)
        est_rmse = ape_result.stats["rmse"]
        est_errs = ape_result.np_arrays["error_array"] # ["timestamps"]

        # Compute navigation error
        wpts_count = test_result.planned_wpts.shape[0]
        wpts_act_count = test_result.reached_stamped_wpts.shape[0]

        # Assume the robot navigates to each wpt sequentially and won't skip any one.
        nav_errs = np.linalg.norm(test_result.reached_stamped_wpts[:, 1:3] - test_result.act_stamped_wpts[:, 1:3], axis=1)
        nav_rmse = np.sqrt(np.mean(nav_errs**2))

        return NavSlamError(nav_errs, nav_rmse, est_errs, est_rmse, wpts_count, wpts_act_count)

    def __convertToEvoFormat(self, mat: np.ndarray) -> PoseTrajectory3D:
        stamps = mat[:, 0]  # n x 1
        xyz = mat[:, 1:4]  # n x 3
        quat = mat[:, 4:8]  # n x 4
        quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
        return PoseTrajectory3D(xyz, quat, stamps)