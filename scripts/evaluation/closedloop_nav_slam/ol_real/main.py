#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file main.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 07-22-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import logging
from pathlib import Path
from cl_common.datasets import (
    EUROC_SEQUENCES,
    EUROC_DATASET,
)
from cl_common.utils import (
    setup_logger,
)
from ol_real.utils.utils import (
    load_frame_poses,
    SlamResult,
)
from ol_real.evaluation import Evaluation
from ol_real.visualization import Visualization

# Define Parameters (@TODO: yaml)
RESULT_DIR = Path("/mnt/DATA/experiments/cl_nav_slam/openloop/euroc/")
# RESULT_DIR = Path("/mnt/DATA/experiments/good_graph/openloop/taskdriven/")
GROUND_TRUTH_DIR = Path("/mnt/DATA/datasets/euroc/gt_pose")


def create_params():
    """_summary_

    Returns:
        _type_: _description_
    """
    params = {
        "result_dir": RESULT_DIR,
        "gt_dir": GROUND_TRUTH_DIR,
        "sequences": EUROC_SEQUENCES[:5],
        "methods": ["gfgg", "orb3"],
        "rounds": 1,
        "loops": 5,
        "modes": ["slomo", "regular"],
        "load_evaluation": True,
        "save_figs": True,
        "enable_visualization": True,
        "grid_xy": 5,  # cm
    }

    return params


def main():
    """_summary_"""

    params = create_params()
    setup_logger(params["result_dir"])

    results = {}
    eva = Evaluation()
    logging.info("Evaluating ...")
    for mode_index, mode_name in enumerate(params["modes"]):
        if mode_name not in results:
            results[mode_name] = {}
        mode_dir = RESULT_DIR / mode_name
        for method_index, method_name in enumerate(params["methods"]):
            for round_index in range(1, params["rounds"] + 1):
                # method_dir = mode_dir / method_name / f"_Speedx1.0/ObsNumber_1200_Round{round_index}"
                method_dir = mode_dir / method_name
                if method_name not in results[mode_name]:
                    results[mode_name][method_name] = {}
                for seq_index, seq_name in enumerate(params["sequences"]):
                    logging.info(f"{mode_name}-{method_name}-{seq_name}")
                    slam_result = None
                    slam_result_filename = method_dir / f"{seq_name}_slam_result"
                    if params["load_evaluation"]:
                        slam_result = SlamResult.from_npz(str(slam_result_filename))
                    else:
                        # Load GT poses.
                        gt_filename = GROUND_TRUTH_DIR / f"{seq_name}_cam0.txt"
                        gt_poses = load_frame_poses(gt_filename)

                        # Load ET poses.
                        et_filename = method_dir / f"{seq_name}_AllFrameTrajectory.txt"
                        et_poses_loops = load_frame_poses(et_filename)

                        # Compute seq frame count. (EuRoC = 20Hz)
                        frame_count = EUROC_DATASET.get_sequence(seq_name).image_count

                        # Evaluate and save results.
                        print(round_index, seq_name)
                        slam_result = eva.run(gt_poses, et_poses_loops, params["loops"], frame_count)
                        slam_result.save(slam_result_filename)

                    # Save to results
                    results[mode_name][method_name][seq_name] = slam_result

    logging.info("Evaluation Done.")

    if params["enable_visualization"]:
        logging.info("Visualizing .. ")
        vis = Visualization(params)
        vis.run(results)
        logging.info("Visualization Done.")


if __name__ == "__main__":
    main()
