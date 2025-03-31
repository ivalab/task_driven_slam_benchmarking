#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file evaluation.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 06-27-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import logging

from typing import Dict, List, Optional
import numpy as np
from cl_common.utils import (
    run_evo_evaluation,
)
from ol_real.utils.utils import (
    average_poses_mat,
    split_trajectory_by_frames,
    split_trajectory_by_loops,
    SlamResult,
)


class Evaluation:
    """_summary_"""

    def __init__(self):
        pass

    def run(self, gt_poses: np.ndarray, et_poses_loops: np.ndarray, loops: int, frame_count: int = 0) -> SlamResult:
        """_summary_

        Args:
            gt_poses (np.ndarray): _description_
            et_poses_loops (np.ndarray): _description_
            loops (int): _description_

        Returns:
            np.ndarray: _description_
        """
        slam_result = SlamResult(loops)

        # Compute raw accuracy.
        looped_et_poses = split_trajectory_by_loops(et_poses_loops)
        # assert len(looped_et_poses) == loops, f"actual v.s. gt {len(looped_et_poses)}, {loops}"
        if len(looped_et_poses) < loops:
            logging.info(f"Insufficient data: actual v.s. target loops ({len(looped_et_poses)}, {loops})")
            return slam_result
        stamped_acc_errs = {}  # {timestamp: [err, ...]}
        for loop_index in range(loops):
            et_poses = looped_et_poses[loop_index]
            try:
                slam_result.rmse[loop_index], timestamps, trans_errs, rot_errs = run_evo_evaluation(gt_poses, et_poses)
            except Exception:
                print(f"Bad evaluation in loop {loop_index}, set NaN")
                continue

            # TODO (yanwei): add sequence completeness
            if frame_count > 0:
                slam_result.completeness[loop_index] = float(et_poses.shape[0]) / frame_count

            for stamp, t_err, r_err in zip(timestamps, trans_errs, rot_errs):
                if stamp not in stamped_acc_errs:
                    stamped_acc_errs[stamp] = np.full((loops, 2), np.nan)
                stamped_acc_errs[stamp][loop_index] = [t_err * 100.0, r_err * np.rad2deg(1.0)]

        # Compute raw precision.
        framed_et_poses = split_trajectory_by_frames(et_poses_loops)
        stamped_pre_errs = {}
        for frame_stamp, frame_poses in framed_et_poses.items():
            if len(frame_poses) < loops:  # Discard invalid data.
                continue
            stamped_pre_errs[frame_stamp] = np.full((loops, 2), np.nan)
            avg_pose, t_stds, r_stds = average_poses_mat(frame_poses[1:])  # Skip the 1st loop for mapping.
            for loop_index in range(1, loops):
                stamped_pre_errs[frame_stamp][loop_index] = [
                    t_stds[loop_index - 1] * 100.0,
                    r_stds[loop_index - 1] * np.rad2deg(1.0),
                ]

        # Associate frame accuracy and precision.
        slam_result.stamped_acc_pre = self.__compute_accuracy_precision_completeness(
            stamped_acc_errs, stamped_pre_errs, loops
        )
        return slam_result

    def __compute_accuracy_precision_completeness(
        self, stamped_acc_errs: Dict[float, np.ndarray], stamped_pre_errs: Dict[float, np.ndarray], loops: int
    ) -> np.ndarray:
        """_summary_

        Args:
            stamped_acc_errs (Dict[float, np.ndarray]): _description_
            stamped_pre_errs (Dict[float, np.ndarray]): _description_

        Returns:
            np.ndarray: [[timestamp, acc, pre, completeness], ..., ]
        """

        loc_loops = loops - 1
        stamped_acc_pre_completeness = []
        timestamp_pre = np.array(list(stamped_pre_errs.keys()))
        for frame_acc_stamp, frame_acc_errs in stamped_acc_errs.items():
            assert len(frame_acc_errs) == loops
            # Discard invalid data
            count = np.sum(~np.isnan(frame_acc_errs[1:, 0]))  # Skip mapping phase
            if count < loc_loops:
                continue
            completeness = np.float64(count) / loc_loops
            # Find corresponding frame pre data.
            stamp_diff = np.abs(timestamp_pre - frame_acc_stamp)
            val = np.min(stamp_diff)
            if val > 1e-6:
                continue
            index = np.argmin(stamp_diff)
            frame_pre_errs = stamped_pre_errs[timestamp_pre[index]]
            assert len(frame_pre_errs) == loops
            assert np.sum(~np.isnan(frame_pre_errs[1:, 0])) == loc_loops
            # Compute acc and pre
            # acc = np.sqrt(np.nanmean(acc_err_array[1:] ** 2))
            # pre = np.sqrt(np.nanmean(pre_err_array[1:] ** 2))
            # print(acc_err_array)
            # print(np.linalg.norm(acc_err_array[1:], axis=-1))
            # break
            acc = np.mean(frame_acc_errs[1:, :], axis=0)  # nan values should not exist
            pre = np.mean(frame_pre_errs[1:, :], axis=0)  # nan values should not exist
            stamped_acc_pre_completeness.append([frame_acc_stamp, acc[0], acc[1], pre[0], pre[1], completeness])

        return np.array(stamped_acc_pre_completeness)
