#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file evaluation.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-26-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from copy import deepcopy
import logging
from pathlib import Path
from typing import Dict

import numpy as np
import yaml
from closedloop_nav_slam.evaluation.evo_utils import EvoEvaluation
from closedloop_nav_slam.evaluation.types import (
    NavSlamData,
    NavSlamError,
    RobotNavigationData,
)
from closedloop_nav_slam.evaluation.utils import (
    load_nav_slam_data,
    load_params,
    load_planned_waypoints,
)
from closedloop_nav_slam.utils.path_definitions import *


class Evaluation:
    def __init__(self, params):
        self._params = deepcopy(params)
        self._result_prefix = Path(self._params["result_dir"]) / self._params["test_type"] / self._params["env_name"]

    def run(self) -> Dict[str, RobotNavigationData]:
        methods = {}
        # Loop over methods.
        for method_name in self._params["method_list"]:
            # Load SLAM testing parameters.
            self._params.update(load_params(SLAM_SETTINGS_PATH / (method_name + ".yaml")))
            robot_nav_data = RobotNavigationData.create(self._params)
            method_dir = self._result_prefix / method_name
            # Loop over paths.
            for pfile in self._params["path_files"]:
                pfile_dir = method_dir / pfile
                robot_nav_data.add_experiment(
                    pfile,
                    load_planned_waypoints(pfile_dir / "trial0" / "planned_waypoints.txt"),
                )
                # Loop over rounds.
                for trial in range(self._params["trials"]):
                    logging.info(f"Processing {method_name}, {pfile}, {trial} ...")
                    prefix = pfile_dir / ("trial" + str(trial))
                    # Load nav slam data
                    nav_slam_data = load_nav_slam_data(
                        prefix,
                        loops=self._params["loops"],
                        robot_init_xytheta=self._params["robot_init_pose"],
                        compensate_map_offset=(method_name != "perfect_odometry"),
                    )

                    # Evaluate nav slam error.
                    nav_slam_error = None if nav_slam_data is None else self.__compute_nav_slam_error(nav_slam_data)
                    if nav_slam_error is not None:
                        logging.info(
                            f"est_rmse: {nav_slam_error.est_rmse:.02f} m, nav_rmse: {nav_slam_error.nav_rmse:.02f} m success rate: {nav_slam_error.success_rate}"
                        )

                    # Add result to method.
                    robot_nav_data.add_round(pfile, trial, nav_slam_data, nav_slam_error)
                    logging.info(f"Done {method_name}, {pfile}, {trial} ...")

                # Compute accuracy and precision
                robot_nav_data.compute_accuracy_and_precision()
            methods[method_name] = robot_nav_data
        return methods

    def __compute_nav_slam_error(self, nav_slam_data: NavSlamData) -> NavSlamError:
        # Call evo package
        return EvoEvaluation().run(nav_slam_data)
