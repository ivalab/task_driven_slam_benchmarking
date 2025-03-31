#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file utils.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-27-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import logging
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml
from cl_sim.utils.datatypes import NavSlamData, RobotNavigationData
from scipy.spatial.transform import Rotation


def load_params(params_file: str):
    """Load params"""
    params = None
    # Load the main parameters.
    with open(params_file, encoding="utf-8") as pf:
        params = yaml.safe_load(pf)
    # Consider mode.
    # params["result_dir"] += params["mode"]
    # Set output result dir.
    if params["save_to_data_dir"]:
        params["result_dir"] = str(Path(params["data_dir"]) / "evaluation")
    assert params
    return params


def yaw_from_quaternion(quat: np.ndarray) -> float:
    """Extract yaw from quaternion (x, y, z, w) in range[-pi, pi]."""
    assert quat.shape == (4,) or quat.shape == (4, 1) or quat.shape == (1, 4)
    rot = Rotation.from_quat(quat).as_matrix()
    return np.arctan2(rot[1, 0], rot[0, 0])


def normalize_angle(angles: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angles), np.cos(angles))


def compute_traj_length(stamped_poses: np.ndarray) -> float:
    """Compute trajectory length in xy plane"""
    return np.sum(np.linalg.norm(np.diff(stamped_poses[:, 1:3], axis=0), axis=1))


def compute_traj_duration(stamped_poses: np.ndarray) -> float:
    """Compute trajectory duration"""
    return np.sum(np.diff(stamped_poses[:, 0]))


def cvt_pose_vec2tf(pos_quat_vec: np.ndarray) -> np.ndarray:
    """
    pos_quat_vec: (px, py, pz, qx, qy, qz, qw)
    """
    pose_tf = np.eye(4)
    pose_tf[:3, 3] = pos_quat_vec[:3].flatten()
    rot = Rotation.from_quat(pos_quat_vec[3:].flatten())
    pose_tf[:3, :3] = rot.as_matrix()
    return pose_tf


def load_yaml(filepath: Path):
    """load parameters"""
    params = None
    with open(filepath, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    assert params
    return params


def load_planned_waypoints(filename):
    """load planned waypoints"""
    return np.loadtxt(filename, ndmin=2)


def load_nav_slam_data(
    prefix_path: Path,
    loops: int = 1,
    robot_init_xyztheta: np.ndarray = np.zeros((4)),
    compensate_map_offset: bool = False,
) -> Optional[NavSlamData]:
    """load nav slam data"""
    # Compose files paths.
    act_poses_path = prefix_path / "act_poses.txt"
    est_poses_path = prefix_path / "est_poses.txt"
    est_slam_poses_path = prefix_path / "est_slam_poses.txt"
    planned_wpts_path = prefix_path / "planned_waypoints.txt"
    visited_wpts_path = prefix_path / "visited_waypoints.txt"
    gt_slam_poses_path = prefix_path / "gt_slam_poses.txt"

    if (
        not act_poses_path.exists()
        or not (est_poses_path.exists() or est_slam_poses_path.exists())
        or not planned_wpts_path.exists()
        or not visited_wpts_path.exists()
    ):
        logging.error("Result file NOT exists.")
        return None

    # Deal with waypoints, format: x, y, theta.
    planned_wpts = np.loadtxt(planned_wpts_path, ndmin=2)
    if planned_wpts.shape[0] <= 0:
        logging.error("No active planned waypoints.")
        return None
    planned_wpts[:, -1] = normalize_angle(planned_wpts[:, -1])

    # We would like to skip the mapping data.
    visited_wpts = np.loadtxt(visited_wpts_path, ndmin=2)
    start_timestamp, end_timestamp = find_localization_range(planned_wpts, visited_wpts, loops)
    visited_wpts = visited_wpts[(end_timestamp >= visited_wpts[:, 0]) & (visited_wpts[:, 0] >= start_timestamp), :]
    if visited_wpts.shape[0] <= 0:
        logging.error("No visited waypoints.")
        return None

    # Load act(gt) and est pose.
    act_poses = load_stamped_data(act_poses_path, start_timestamp, end_timestamp)
    if est_slam_poses_path.exists():
        est_poses = load_stamped_data(est_slam_poses_path, start_timestamp, end_timestamp)
    elif est_poses_path.exists():
        est_poses = load_stamped_data(est_poses_path, start_timestamp, end_timestamp)
    else:
        logging.error("No est poses available.")
        return None
    if compensate_map_offset:
        est_poses[:, 1:] = compensate_offset(est_poses[:, 1:], robot_init_xyztheta)

    if act_poses.shape[0] <= 0 or est_poses.shape[0] <= 0:
        logging.error("No act(gt) or est poses.")
        return None

    # Find the robot actual pose at each waypoint.
    act_wpts = []
    for wpt in visited_wpts:
        timediffs = np.abs(wpt[0] - act_poses[:, 0])
        val = np.min(timediffs)
        index = np.argmin(timediffs)
        if val > 5e-2:
            break
        assert val < 5e-2, f"val={val}, wpt_timestamp={wpt[0]}"  # seconds
        act_wpts.append(act_poses[index, [0, 1, 2]].tolist() + [yaw_from_quaternion(act_poses[index, -4:])])

    print(len(act_wpts))

    if len(act_wpts) <= 0:
        logging.error("No act waypoints.")
        return None

    traj_length = compute_traj_length(act_poses)
    traj_duration = compute_traj_duration(act_poses)

    if traj_duration <= 0.0 or traj_length <= 0.0:
        logging.error("Invalid traj duration or traj length.")
        return None

    # Load gt slam poses.
    gt_slam_poses = None
    if gt_slam_poses_path.exists():
        gt_slam_poses = load_stamped_data(gt_slam_poses_path, start_timestamp, end_timestamp)
        gt_slam_poses = np.loadtxt(gt_slam_poses_path, ndmin=2) if gt_slam_poses_path.exists() else None
        if compensate_map_offset:
            gt_slam_poses[:, 1:] = compensate_offset(gt_slam_poses[:, 1:], robot_init_xyztheta)

    # @TODO slam poses and extrinsics
    return NavSlamData(
        planned_wpts.shape[0],
        visited_wpts,
        np.array(act_wpts),
        np.arange(len(act_wpts), dtype=np.int8),
        act_poses,
        est_poses,
        traj_length,
        traj_duration,
        gt_slam_poses,
    )


def find_localization_range(planned_wpts, gt_wpts, loops: int) -> Tuple[float, float]:
    """Find start timestamp where the robot starts doing pure localization in a known map."""
    assert loops >= 0
    if loops == 0:  # Localization or Odometry mode.
        index = -1 if gt_wpts.shape[0] <= planned_wpts.shape[0] else planned_wpts.shape[0] - 1
        return (-1.0, gt_wpts[index, 0] + 1e-3)
    index = planned_wpts.shape[0] * loops - 1  # the last mapping point
    assert index >= 0
    if gt_wpts.shape[0] <= index + 1:
        # It means mission failed, the robot does NOT even finish mapping.
        # safe margin, ignore all mapping data
        start_timestamp = gt_wpts[-1, 0] + 1e-3
        end_timestamp = start_timestamp
    else:
        start_timestamp = gt_wpts[index, 0] + 1e-3  # safe margin
        end_timestamp = gt_wpts[min(index + planned_wpts.shape[0], gt_wpts.shape[0] - 1), 0]
    return (start_timestamp, end_timestamp)


def load_stamped_data(filename: str, start_timestamp: float = -1.0, end_timestamp: float = -1.0) -> np.ndarray:
    data = np.loadtxt(filename, ndmin=2)
    if start_timestamp >= 0:
        data = data[data[:, 0] >= start_timestamp, :]
    if end_timestamp >= 0:
        data = data[data[:, 0] <= end_timestamp, :]
    return data


def compensate_offset(pose_array: np.ndarray, xyztheta: np.ndarray):
    """Compensate pose offset."""
    wTm = np.eye(4)
    wTm[:3, :3] = Rotation.from_euler("z", xyztheta[-1], degrees=False).as_matrix()
    wTm[:3, 3] = xyztheta[:-1]

    mTb = np.repeat(np.eye(4)[None, :, :], pose_array.shape[0], axis=0)  # Nx4x4
    mTb[:, :3, :3] = Rotation.from_quat(pose_array[:, 3:]).as_matrix()
    mTb[:, :3, 3] = pose_array[:, :3]

    wTb = wTm @ mTb

    out_array = np.column_stack([wTb[:, :3, 3].reshape(-1, 3), Rotation.from_matrix(wTb[:, :3, :3]).as_quat()])
    return out_array


def save_evaluation(prefix: str, eval_data: Dict[str, Dict[str, RobotNavigationData]], overwrite: bool = True):
    """_summary_"""
    for env_name, methods in eval_data.items():
        # Create dir.
        input_dir = Path(prefix) / env_name / "evals"
        input_dir.mkdir(parents=True, exist_ok=True)
        # Loop over method.
        for method_name, nav_data in methods.items():
            filename_path = input_dir / (method_name + "_eval.pkl")
            filename = str(filename_path)
            if filename_path.exists() and not overwrite:
                postfix = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
                cmd_mv = "mv " + filename + " " + filename + "." + postfix
                subprocess.call(cmd_mv, shell=True)
            nav_data.save_to_file(filename)


def load_evaluation(prefix: Path, envs: list, method_list: list) -> Dict[str, Dict[str, RobotNavigationData]]:
    """Load evaluation."""
    eval_data = {}
    for env_name in envs:
        methods = {}
        for method_name in method_list:
            filename = prefix / env_name / "evals" / (method_name + "_eval.pkl")
            if not filename.is_file():
                print(f"env: {env_name}, method: {method_name} evaluation result: {filename} does NOT exist, skip.")
                continue
            methods[method_name] = RobotNavigationData(method_name)
            methods[method_name].load_from_file(str(filename))
        eval_data[env_name] = methods
    return eval_data


def compose_dir_path(dir_prefix, params):
    """Compose dir path"""
    return Path(dir_prefix) / params["test_type"] / params["env_name"]


def compute_the_closest_square_root_num(num: int):
    return num


def compute_weighted_accuracy_and_precision(wpts_count, wpts_stats, in_cm=False, in_degree=False):
    xy_scale = 100.0 if in_cm else 1.0
    theta_scale = 180.0 / np.pi if in_degree else 1.0
    precision = np.full((wpts_count, 2), np.nan)
    accuracy = np.full((wpts_count, 2), np.nan)
    weights_sum = 0.0
    for wpt_stat in wpts_stats:
        if wpt_stat.success_rate <= 0.0:
            continue
        weight = 1.0 / wpt_stat.success_rate
        weights_sum += weight
        accuracy[wpt_stat.index, :] = [
            wpt_stat.accuracy.pos * weight * xy_scale,
            wpt_stat.accuracy.theta * weight * theta_scale,
        ]
        precision[wpt_stat.index, :] = [
            wpt_stat.precision.pos * weight * xy_scale,
            wpt_stat.precision.theta * weight * theta_scale,
        ]

    one_over_weight_sum = 1.0 / weights_sum
    precision *= one_over_weight_sum
    accuracy *= one_over_weight_sum

    wpts_completeness = np.sum(~np.isnan(precision[:, 0])) / wpts_count
    average_precision = np.nansum(precision, axis=0) / wpts_completeness
    average_accuracy = np.nansum(accuracy, axis=0) / wpts_completeness
    return accuracy, precision, average_accuracy, average_precision


def compute_weighted_accuracy_and_precision_deprecated(planned_wpts, act_wpts):
    assert planned_wpts.shape[0] > 0 and planned_wpts.shape[1] == 3
    assert act_wpts.shape[0] == planned_wpts.shape[0] and act_wpts.shape[1] == 3 and act_wpts.shape[2] > 1

    rounds = act_wpts.shape[2]
    accuracy = np.full((rounds - 1, 2), np.nan)
    precision = np.full((rounds - 1, 2), np.nan)

    for round_num in range(2, rounds + 1):
        index = round_num - 2
        weights = 1.0 / np.sum(~np.isnan(act_wpts[:, 0, :round_num]), axis=-1)
        weights[np.isinf(weights)] = np.nan
        weights /= np.nansum(weights)

        # accuracy[index, 0] = np.nansum(np.linalg.norm(wpt_mean[:, :2] - planned_wpts[:, :2], axis=-1) * weights)
        # accuracy[index, 1] = np.nansum(np.abs(wpt_mean[:, -1] - planned_wpts[:, -1]) * weights)

        accuracy[index, 0] = np.nansum(
            np.nanmean(np.linalg.norm(act_wpts[:, :2, :round_num] - planned_wpts[:, :2, None], axis=1), axis=-1)
            * weights
        )
        accuracy[index, 1] = np.nansum(
            np.nanmean(np.abs(act_wpts[:, -1, :round_num] - planned_wpts[:, -1, None]), axis=-1) * weights
        )

        # Compute precision
        wpt_mean = np.nanmean(act_wpts[:, :, :round_num], axis=-1)
        std_xy = np.sqrt(
            np.nanmean(np.sum((act_wpts[:, :2, :round_num] - wpt_mean[:, :2, None]) ** 2, axis=1), axis=-1)
        )
        std_theta = np.nanstd(act_wpts[:, -1, :round_num] - wpt_mean[:, -1, None])
        # weights = 1.0 / np.sum(~np.isnan(std_xy), axis=-1)
        # print(std_xy)
        # print(weights)
        # weights[np.isinf(weights)] = np.nan
        # weights /= np.nansum(weights)
        # Position(xy)
        precision[index, 0] = np.nansum(std_xy * weights)
        # Angle(theta)
        precision[index, 1] = np.nansum(std_theta * weights)

    return accuracy, precision


def compute_averaged_accuracy_precision(
    wpts_accuracy, wpts_precision, wpts_stats, rounds, in_cm=False, in_degree=False
):

    assert wpts_accuracy.shape == wpts_precision.shape
    assert len(wpts_stats) == wpts_accuracy.shape[0]

    xy_scale = 100.0 if in_cm else 1.0
    theta_scale = 180.0 / np.pi if in_degree else 1.0

    wpts_count = wpts_accuracy.shape[0]
    total = float(rounds * wpts_count)

    accuracy = np.nanmean(wpts_accuracy, axis=0)
    precision = np.nanmean(wpts_precision, axis=0)

    accuracy[0] *= xy_scale
    accuracy[1] *= theta_scale
    precision[0] *= xy_scale
    precision[1] *= theta_scale

    assert accuracy.shape == (2,)
    assert precision.shape == (2,)

    act_wpts_count = 0
    for wpt in wpts_stats:
        act_wpts_count += wpt.visits
    completeness = act_wpts_count / total

    return accuracy, precision, completeness, accuracy / completeness, precision / completeness
