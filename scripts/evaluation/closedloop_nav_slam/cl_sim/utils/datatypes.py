#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file datatypes.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-26-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import pickle
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class MethodType(Enum):
    """_summary_

    Args:
        Enum (_type_): _description_
    """

    UNKNOWN = 0
    MAPPING = 1
    LOCALIZATION = 2
    SLAM = 3
    ODOMETRY = 4


class SensorType(Enum):
    """_summary_

    Args:
        Enum (_type_): _description_
    """

    UNKNOWN = 0
    LASER = 1
    LIDAR = 2
    STEREO = 3
    GROUND_TRUTH = 4


@dataclass
class NavSlamError:
    """_summary_"""

    nav_errs: np.ndarray  # len(wpts) x 1
    nav_rmse: float
    est_errs: np.ndarray  # len(frames) x 1
    est_rmse: float
    completion: float  # len(actual_wpts) / len(planned_wpts)
    wpts_errs: np.ndarray  # len(wpts) * 3 (x, y, theta)
    est_errs_gt_slam: Optional[np.ndarray]  # len(frames) x 1
    est_rmse_gt_slam: Optional[float]


@dataclass
class NavSlamData:
    """_summary_"""

    planned_wpts_count: int
    visited_wpts: np.ndarray  # [timestamp, x, y, theta]
    act_wpts: np.ndarray  # [timestamp, x, y, theta]
    wpts_indices: np.ndarray  # indices in planned wpts
    act_poses: np.ndarray
    est_poses: np.ndarray
    traj_length: float  # meters
    traj_duration: float  # seconds
    gt_slam_poses: Optional[np.ndarray]


@dataclass
class WptError:
    """_summary_"""

    pos: float  # meters
    theta: float  # radian
    x: float = -1.0  # meters
    y: float = -1.0  # meters


@dataclass
class WptStat:
    """_summary_"""

    index: int
    visits: int  # The number of visits
    accuracy: WptError
    precision: WptError
    success_rate: float  # visits / rounds


class RobotNavigationData:
    """_summary_"""

    def __init__(self, name: str, sensor_type: str = "unknown", method_type: str = "unknown"):
        """_summary_

        Args:
            name (str): _description_
            sensor_type (str, optional): _description_. Defaults to "unknown".
            method_type (str, optional): _description_. Defaults to "unknown".
        """
        self.name = name
        self.sensor_type = SensorType[sensor_type.upper()]
        self.method_type = MethodType[method_type.upper()]
        self.data = []

    def add_experiment(self, path_name, waypoints):
        """_summary_

        Args:
            path_name (_type_): _description_
            waypoints (_type_): _description_
        """
        experiment = {
            "path_name": path_name,
            "waypoints": waypoints,
            "rounds": [],
            "wpts_stats": [],
            "accuracy": None,
            "precision": None,
            "completeness": None,
        }  # len(waypoints)
        self.data.append(experiment)

    def add_round(self, path_name, round_number, nav_data: Optional[NavSlamData], nav_err: Optional[NavSlamError]):
        """_summary_

        Args:
            path_name (_type_): _description_
            round_number (_type_): _description_
            nav_data (Optional[NavSlamData]): _description_
            nav_err (Optional[NavSlamError]): _description_
        """
        for experiment in self.data:
            if experiment["path_name"] == path_name:
                # Sanity check.
                if nav_err is not None and nav_data is not None:
                    assert len(nav_err.wpts_errs) == len(nav_data.wpts_indices)
                    if len(nav_data.wpts_indices) > 0:
                        assert max(nav_data.wpts_indices) < len(experiment["waypoints"])
                        assert min(nav_data.wpts_indices) >= 0
                # Add round data.
                experiment["rounds"].append(
                    {
                        "round": round_number,
                        "nav_data": nav_data,
                        "nav_err": nav_err,
                        "completed": (
                            False if nav_data is None else len(nav_data.act_wpts) >= len(experiment["waypoints"])
                        ),
                    }
                )

    def compute_accuracy_and_precision(self):
        """_summary_"""
        # Per each path/experiment.
        for experiment in self.data:
            planned_wpts = experiment["waypoints"]
            act_wpts = np.full(experiment["waypoints"].shape + (len(experiment["rounds"]),), np.nan)
            # Fill-in actual wpts from all the rounds.
            for round_data in experiment["rounds"]:
                nav_data = round_data["nav_data"]
                if nav_data is None:
                    continue
                act_wpts[nav_data.wpts_indices, :, round_data["round"]] = nav_data.act_wpts[:, 1:]

            # Compute accuracy and precision for the whole path (all planned waypoints).
            accuracy, precision, completeness = self.compute_path_accuracy_and_precision(planned_wpts, act_wpts)
            # Ensure accuracy and precision have the correct shape (the number of waypoints).
            assert accuracy.shape[0] == planned_wpts.shape[0]
            assert precision.shape[0] == planned_wpts.shape[0]
            assert completeness.shape[0] == planned_wpts.shape[0]
            experiment["accuracy"] = accuracy
            experiment["precision"] = precision
            experiment["completeness"] = completeness

            # Add wpts_stats.
            for index in range(act_wpts.shape[0]):
                visits = np.sum(~np.isnan(act_wpts[index, 0, :]))
                sr = visits / len(experiment["rounds"])
                experiment["wpts_stats"].append(
                    WptStat(
                        index,
                        visits,
                        WptError(accuracy[index, 0], accuracy[index, 1]),
                        WptError(precision[index, 0], precision[index, 1]),
                        sr,
                    )
                )

    def compute_path_accuracy_and_precision(self, planned_wpts, act_wpts):
        """_summary_

        Args:
            planned_wpts (_type_): _description_
            act_wpts (_type_): _description_

        Returns:
            _type_: _description_
        """
        accuracy = np.full((act_wpts.shape[0], 2), np.nan)  # [[xy, theta], ... ]
        precision = np.full((act_wpts.shape[0], 2), np.nan)  # [[xy, theta], ... ]
        completeness = np.full((act_wpts.shape[0]), 0.0)

        # @TODO (yanwei) Vectorize the following code.
        tgt_count = act_wpts.shape[-1]
        # Loop over each waypoint.
        for wpt_index in range(act_wpts.shape[0]):

            # Skip wpt with only one success (accuracy and precision are not valid).
            act_count = np.sum(~np.isnan(act_wpts[wpt_index, 0, :]))
            # if act_count == 1:
            #     continue
            completeness[wpt_index] = float(act_count) / tgt_count

            # Extract wpt info.
            act_xy = act_wpts[wpt_index, :2, :]
            act_theta = act_wpts[wpt_index, -1, :]
            act_theta_xy = np.column_stack((np.cos(act_theta), np.sin(act_theta)))
            planned_theta = planned_wpts[wpt_index, -1]
            planned_theta_xy = np.array([[np.cos(planned_theta)], [np.sin(planned_theta)]])  # 2x1
            # Compute accuracy.
            accuracy[wpt_index, 0] = np.nanmean(np.linalg.norm(act_xy - planned_wpts[wpt_index, :2, None], axis=0))
            accuracy[wpt_index, 1] = np.nanmean(np.arccos(act_theta_xy @ planned_theta_xy))
            # Compute precision.
            if act_count == 1:
                continue
            act_xy_mean = np.nanmean(act_xy, axis=1)
            act_theta_xy_mean = np.nanmean(act_theta_xy, axis=0)
            precision[wpt_index, 0] = np.nanmean(np.linalg.norm(act_xy - act_xy_mean[..., None], axis=0))
            precision[wpt_index, 1] = np.nanmean(np.arccos(act_theta_xy @ act_theta_xy_mean))
        return accuracy, precision, completeness

    def save_to_file(self, filename):
        """_summary_

        Args:
            filename (_type_): _description_
        """
        with open(filename, "wb") as file:
            pickle.dump(self.data, file)

    def load_from_file(self, filename):
        """_summary_

        Args:
            filename (_type_): _description_
        """
        with open(filename, "rb") as file:
            self.data = pickle.load(file)

    @staticmethod
    def create(params: dict):
        return RobotNavigationData(params["slam_method"])
