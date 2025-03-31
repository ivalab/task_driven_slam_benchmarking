#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file evo_utils.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-28-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""


import logging

import numpy as np
from cl_sim.utils.datatypes import NavSlamData, NavSlamError
from evo.core import metrics, sync
from evo.core.result import Result as EvoResult
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.main_ape import ape as evo_ape
from evo.main_rpe import rpe as evo_rpe
from evo.tools import file_interface, plot


class EvoEvaluation:
    def __init__(self):
        pass

    def run(self, nav_slam_data: NavSlamData) -> NavSlamError:
        # Compute estimation error.
        traj_act = self.__convert_to_evo_format(nav_slam_data.act_poses)
        traj_est = self.__convert_to_evo_format(nav_slam_data.est_poses)

        # Assuming act and est are from the same frame id (e.g body)
        traj_act, traj_est = sync.associate_trajectories(traj_act, traj_est)

        # traj_method.align_origin(traj_ref)
        # Call APE
        logging.info("Evaluate APE of GT and EST SLAM poses.")
        ape_result = evo_ape(traj_act, traj_est, pose_relation=metrics.PoseRelation.translation_part)
        est_rmse = ape_result.stats["rmse"]
        est_errs = ape_result.np_arrays["error_array"]  # ["timestamps"]

        # Compute success rate
        sr = nav_slam_data.act_wpts.shape[0] / nav_slam_data.planned_wpts_count

        wpts_errs = nav_slam_data.visited_wpts[:, 1:] - nav_slam_data.act_wpts[:, 1:]
        nav_errs = np.linalg.norm(wpts_errs[:, :2], axis=1)  # ignore orientation
        nav_rmse = np.sqrt(np.mean(nav_errs**2))

        # Compute est rmse against gt slam.
        est_errs_gt_slam = None
        est_rmse_gt_slam = None
        if False and nav_slam_data.gt_slam_poses is not None:
            logging.info("Evaluate APE of GT SLAM and EST SLAM poses.")
            traj_gt_slam = self.__convert_to_evo_format(nav_slam_data.gt_slam_poses)
            traj_est = self.__convert_to_evo_format(nav_slam_data.est_poses)
            traj_gt_slam, traj_est = sync.associate_trajectories(traj_gt_slam, traj_est, 0.03)
            ape_result = evo_ape(traj_gt_slam, traj_est, pose_relation=metrics.PoseRelation.translation_part)
            est_rmse_gt_slam = ape_result.stats["rmse"]
            est_errs_gt_slam = ape_result.np_arrays["error_array"]  # ["timestamps"]

        return NavSlamError(nav_errs, nav_rmse, est_errs, est_rmse, sr, wpts_errs, est_errs_gt_slam, est_rmse_gt_slam)

    def __convert_to_evo_format(self, mat: np.ndarray) -> PoseTrajectory3D:
        stamps = mat[:, 0]  # n x 1
        xyz = mat[:, 1:4]  # n x 3
        quat = mat[:, 4:8]  # n x 4
        quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
        return PoseTrajectory3D(xyz, quat, stamps)
