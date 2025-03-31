#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file utils.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 06-27-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from evo.core import lie_algebra as lie

from pathlib import Path
from typing import List


def average_quaternions(quaternions):
    A = np.zeros((4, 4))
    for q in quaternions:
        q = np.outer(q, q)
        A += q
    A /= len(quaternions)
    eigenvalues, eigenvectors = np.linalg.eig(A)
    avg_quaternion = eigenvectors[:, np.argmax(eigenvalues)]
    return avg_quaternion


def average_translations(translations):
    return np.mean(translations, axis=0)


def average_poses_mat(poses):
    translations = [pose[:3, 3] for pose in poses]
    rotations = [pose[:3, :3] for pose in poses]
    quaternions = [R.from_matrix(rot).as_quat() for rot in rotations]
    avg_pose = average_poses_tq(translations, quaternions)

    errs_pose = [avg_pose @ np.linalg.inv(pose) for pose in poses]
    errs_trans = [np.linalg.norm(err[:3, 3]) for err in errs_pose]  # m
    errs_rots = [abs(lie.so3_log_angle(err[:3, :3], False)) for err in errs_pose]  # radian

    return avg_pose, errs_trans, errs_rots


def average_poses_tq(translations, quaternions):
    avg_quat = average_quaternions(quaternions)
    avg_rot = R.from_quat(avg_quat).as_matrix()

    avg_translation = average_translations(translations)

    avg_pose = np.eye(4)
    avg_pose[:3, :3] = avg_rot
    avg_pose[:3, 3] = avg_translation

    return avg_pose


def pose_array_to_mat(arr):
    pose = np.eye(4)
    pose[:3, 3] = arr[:3]
    pose[:3, :3] = R.from_quat(arr[3:]).as_matrix()
    return pose


def load_frame_poses(filename: Path) -> np.ndarray:
    """Load frame poses in format of [timestamp, x, y, z, qx, qy, qz, qw]

    Args:
        filename (Path): _description_

    Returns:
        np.ndarray: poses of Mx8
    """
    assert filename.exists(), f"{filename} does NOT exist!"
    poses = np.loadtxt(filename, ndmin=2)
    assert poses.shape[0] > 0 and poses.shape[1] == 8
    return poses


def split_trajectory_by_loops(traj: np.ndarray):
    """Split a single trajectory into multiple loops

    Args:
        traj (np.ndarray): _description_

    Returns:
        _type_: _description_
    """
    timestamps = traj[:, 0]
    indices = np.arange(1, len(timestamps))[np.diff(timestamps) < 0]
    result = []
    r0 = 0
    for r1 in indices:
        result.append(traj[r0:r1, :])
        r0 = r1
    result.append(traj[indices[-1] :, :])
    return result


def split_trajectory_by_frames(traj):
    frames_poses = {}
    for data in traj:
        timestamp = data[0]
        pose = pose_array_to_mat(data[1:])
        if timestamp not in frames_poses:
            frames_poses[timestamp] = []
        frames_poses[timestamp].append(pose)
    return frames_poses


def merge_trajectory(trajs: List[np.ndarray]) -> np.ndarray:
    return np.concatenate(trajs, axis=0)


class SlamResult:
    """_summary_"""

    def __init__(self, loops: int):
        self.loops: int = loops
        self.rmse: np.array = np.full((self.loops), np.nan, dtype=np.float32)
        self.completeness: np.array = np.full((self.loops), np.nan, dtype=np.float32)
        self.stamped_acc_pre: np.ndarray = np.full(
            (0, 6), np.nan, dtype=np.float32
        )  # [timestamp, acc_t, acc_r, pre_t, pre_r, completeness] N * 6

    @property
    def frames_accuracy(self):
        return self.stamped_acc_pre[:, 1:3]

    @property
    def frames_precision(self):
        return self.stamped_acc_pre[:, 3:5]

    @property
    def frames_timestamp(self):
        return self.stamped_acc_pre[:, 0]

    @property
    def frames_completeness(self):
        return self.stamped_acc_pre[:, -1]

    def is_valid(self) -> bool:
        """_summary_

        Returns:
            bool: _description_
        """
        return self.loops > 0 and np.sum(~np.isnan(self.rmse)) == self.loops and self.stamped_acc_pre.shape[0] > 0

    def save(self, filename: str):
        """_summary_

        Args:
            filename (str): _description_
        """
        data = {
            "loops": self.loops,
            "rmse": self.rmse,
            "completeness": self.completeness,
            "stamped_acc_pre": self.stamped_acc_pre,
        }
        np.savez(filename, **data)

    @classmethod
    def from_npz(cls, filename: str):
        filename = filename if filename.endswith(".npz") else filename + ".npz"
        data = np.load(filename)
        slam_result = cls(data["loops"])
        slam_result.rmse = data["rmse"]
        slam_result.completeness = data["completeness"]
        slam_result.stamped_acc_pre = data["stamped_acc_pre"]
        return slam_result
