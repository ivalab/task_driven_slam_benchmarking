#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file main.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

# from evo.core.trajectory import PosePath3D, PoseTrajectory3D
# from evo.core import metrics, sync
# from evo.tools import file_interface, plot
# from evo.main_rpe import rpe
# from evo.main_ape import ape
# from evo.core.result import Result


from pathlib import Path

from evaluation import Evaluation
from visualization import Visualization

SETTINGS_PREFIX = Path(__file__).resolve().parent.parent / "settings"

if __name__ == "__main__":
    methods_list = [
        "perfect_odometry",
        "gfgg",
        "orb3",
        "dsol",
        "msckf",
    ]
    config_file = SETTINGS_PREFIX / "config.yaml"

    # Run evaluation.
    print("Run evalaution ... ")
    eva = Evaluation(methods_list, config_file, save_results=True)
    eva.run()
    print("Evaluation Done.\n")

    # Run visualization.
    print("Run visualization ...")
    vis = Visualization(config_file, eva.methods, save_figs=True)
    vis.run()
    print("Visualization Done.\n")

    print("Main Script Done.")
