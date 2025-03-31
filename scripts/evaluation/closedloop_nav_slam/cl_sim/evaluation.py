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

import logging
from copy import deepcopy
from pathlib import Path
from typing import Dict

from cl_sim.utils.evo import EvoEvaluation
from cl_sim.utils.datatypes import (
    NavSlamData,
    NavSlamError,
    RobotNavigationData,
)
from cl_sim.utils.utils import (
    load_nav_slam_data,
    load_planned_waypoints,
    load_params,
    load_yaml,
)

from cl_sim.utils.path_definitions import CONFIG_PATH


class Evaluation:
    """Main evaluation script."""

    def __init__(self, params):
        self._params = deepcopy(params)
        self._data_dir = Path(self._params["data_dir"]) / self._params["mode"]

        # Check mode.
        if "mapping" == self._params["mode"]:
            assert self._params["rounds"] > 1 and self._params["loops"] == 1
        elif "localization" == self._params["mode"]:
            assert self._params["rounds"] == 1 and self._params["loops"] > 1
        else:
            logging.error("Unsupported evaluation mode %s.", self._params["mode"])

    def run(self) -> Dict[str, Dict[str, RobotNavigationData]]:
        """_summary_

        Returns:
            Dict[str, Dict[str, RobotNavigationData]]: {env_name: {method_name: Data}}
        """
        envs = {}
        for env_name in self._params["envs"]:
            env_params = load_yaml(CONFIG_PATH / "envs" / (env_name + ".yaml"))
            result_dir = self._data_dir / env_name
            methods = {}
            # Loop over methods.
            for method_name in self._params["methods"]:
                self._params["slam_method"] = method_name
                robot_nav_data = RobotNavigationData.create(self._params)
                method_dir = result_dir / method_name
                # Loop over paths.
                for pfile in env_params["path"]:
                    pfile_dir = method_dir / pfile
                    robot_nav_data.add_experiment(
                        pfile,
                        load_planned_waypoints(pfile_dir / "trial0" / "planned_waypoints.txt"),
                    )
                    # Loop over rounds.
                    localization_mode = False
                    rounds = self._params["rounds"]
                    if "localization" == self._params["mode"]:
                        localization_mode = True
                        rounds = self._params["loops"]
                        logging.info("Evaluating map_based_localization, using loops.")
                    for round_index in range(rounds):
                        logging.info("Processing %s, %s, round%d ...", method_name, pfile, round_index)
                        prefix = pfile_dir / ("trial" + str(0 if localization_mode else round_index))
                        # Load nav slam data
                        nav_slam_data = load_nav_slam_data(
                            prefix,
                            loops=round_index if localization_mode else 0,
                            robot_init_xyztheta=env_params["robot_init_pose"],
                            compensate_map_offset=(method_name != "perfect_odometry"),
                        )

                        # Evaluate nav slam error.
                        nav_slam_error = None if nav_slam_data is None else self.__compute_nav_slam_error(nav_slam_data)
                        if nav_slam_error is not None:
                            logging.info(
                                "est_rmse: %.2f m, nav_rmse: %.2f m success rate: %.2f",
                                nav_slam_error.est_rmse,
                                nav_slam_error.nav_rmse,
                                nav_slam_error.completion,
                            )

                        # Add result to method.
                        robot_nav_data.add_round(pfile, round_index, nav_slam_data, nav_slam_error)
                        logging.info("Done %s, %s, round%d ... ", method_name, pfile, round_index)

                    # Compute accuracy and precision
                    robot_nav_data.compute_accuracy_and_precision()
                # Push to methods.
                methods[method_name] = robot_nav_data
            # Push to envs.
            envs[env_name] = methods
        return envs

    def __compute_nav_slam_error(self, nav_slam_data: NavSlamData) -> NavSlamError:
        # Call evo package
        return EvoEvaluation().run(nav_slam_data)
