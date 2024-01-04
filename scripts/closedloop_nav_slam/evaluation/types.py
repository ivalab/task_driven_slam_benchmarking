#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file types.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-26-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import pickle
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np


class MethodType(Enum):
    UNKNOWN = 0
    MAPPING = 1
    LOCALIZATION = 2
    SLAM = 3
    ODOMETRY = 4


class SensorType(Enum):
    UNKNOWN = 0
    LASER = 1
    LIDAR = 2
    STEREO = 3
    GROUND_TRUTH = 4


@dataclass
class NavSlamError:
    nav_errs: np.ndarray  # len(wpts) x 1
    nav_rmse: float
    est_errs: np.ndarray  # len(frames) x 1
    est_rmse: float
    success_rate: float  # len(actual_wpts) / len(planned_wpts)
    wpts_errs: np.ndarray  # len(wpts) * 3 (x, y, theta)


@dataclass
class NavSlamData:
    planned_wpts_count: int
    visited_wpts: np.ndarray  # [timestamp, x, y, theta]
    act_wpts: np.ndarray  # [timestamp, x, y, theta]
    wpts_indices: np.ndarray  # indices in planned wpts
    act_poses: np.ndarray
    est_poses: np.ndarray
    traj_length: float  # meters
    traj_duration: float  # seconds


@dataclass
class WptError:
    pos: float  # meters
    theta: float  # radian
    x: float = -1.0  # meters
    y: float = -1.0  # meters


@dataclass
class WptStat:
    index: int
    visits: int  # The number of visits
    precision: WptError
    accuracy: WptError
    success_rate: float  # visits / rounds


class RobotNavigationData:
    def __init__(self, name: str, sensor_type: str = "unknown", method_type: str = "unknown"):
        self.name = name
        self.sensor_type = SensorType[sensor_type.upper()]
        self.method_type = MethodType[method_type.upper()]
        self.data = []

    def add_experiment(self, path_name, waypoints):
        experiment = {
            "path_name": path_name,
            "waypoints": waypoints,
            "rounds": [],
            "wpts_stats": [],
            "accuracy": None,
            "precision": None,
        }  # len(waypoints)
        self.data.append(experiment)

    def add_round(self, path_name, round_number, nav_data: Optional[NavSlamData], nav_err: Optional[NavSlamError]):
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
                        "completed": False
                        if nav_data is None
                        else len(nav_data.act_wpts) >= len(experiment["waypoints"]),
                    }
                )
                break

    def compute_accuracy_and_precision(self):
        # Per each path/experiment.
        for experiment in self.data:
            planned_wpts = experiment["waypoints"]
            act_wpts = np.full(experiment["waypoints"].shape + (len(experiment["rounds"]),), np.nan)
            # Fill-in actual wpts from all the rounds.
            for round_data in experiment["rounds"]:
                nav_data = round_data["nav_data"]
                act_wpts[nav_data.wpts_indices, :, round_data["round"]] = nav_data.act_wpts[:, 1:]

            accuracy, precision = self.compute_path_accuracy_and_precision(planned_wpts, act_wpts)
            experiment["accuracy"] = accuracy
            experiment["precision"] = precision

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
        accuracy = np.full((act_wpts.shape[0], 2), np.nan)
        precision = np.full((act_wpts.shape[0], 2), np.nan)

        # Compute accuracy
        # Position(xy)
        accuracy[:, 0] = np.nanmean(np.linalg.norm(act_wpts[:, :2, :] - planned_wpts[:, :2, None], axis=1), axis=-1)
        # Angle (theta)
        accuracy[:, 1] = np.nanmean(np.linalg.norm(act_wpts[:, -1, :] - planned_wpts[:, -1, None], axis=1), axis=-1)

        # Compute precision
        wpt_mean = np.nanmean(act_wpts, axis=-1)
        # Position(xy)
        precision[:, 0] = np.nanmean(np.linalg.norm(act_wpts[:, :2, :] - wpt_mean[:, :2, None], axis=1), axis=-1)
        # Angle(theta)
        precision[:, 1] = np.nanmean(np.linalg.norm(act_wpts[:, -1, :] - wpt_mean[:, -1, None], axis=1), axis=-1)

        return accuracy, precision

    def save_to_file(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self.data, file)

    def load_from_file(self, filename):
        with open(filename, "rb") as file:
            self.data = pickle.load(file)

    @staticmethod
    def create(params: dict):
        return RobotNavigationData(params["slam_method"], params["slam_sensor_type"], params["mode"])
