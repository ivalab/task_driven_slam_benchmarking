#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file visualization.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-29-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull
from copy import deepcopy

from closedloop_nav_slam.evaluation.utils import (
    RobotNavigationData,
    compose_dir_path,
)

from typing import List, Dict

from pathlib import Path
import numpy as np
import random
import logging


class Visualization:
    COLORS = [
        "lime",
        "darkorange",
        "maroon",
        "darkcyan",
        "cyan",
        "violet",
        "blue",
    ]
    MARKERS = [
        "*",
        "+",
        "^",
        "o",
        "s",
        "<",
        "p",
    ]

    def __init__(self, params, methods: Dict[str, RobotNavigationData]):
        self._params = deepcopy(params)
        self._methods = methods
        self._colors = [plt.cm.jet(random.random()) for _ in range(50)]  # Generate random colors
        self._output_dir = Path(compose_dir_path(self._params["output_dir"], self._params))
        self._output_dir.mkdir(exist_ok=True, parents=True)

    def run(self):
        # self.__plot_trajectory()
        logging.info("plot waypoints errors ... ")
        self.__plot_waypoints_errors()
        logging.info("plot accuracy and precision ... ")
        self.__plot_accuracy_and_precision()

    def __plot_trajectory(self):
        # Loop over each method.
        for method_name, robot_nav_data in self._methods.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                # Loop over each round(trial).
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]

                    fig = plt.figure()
                    # actual traj.
                    plt.plot(nav_data.act_poses[:, 1], nav_data.act_poses[:, 2], color="g", label="actual")
                    # esti traj.
                    plt.plot(nav_data.est_poses[:, 1], nav_data.est_poses[:, 2], color="r", label="estimated")
                    # start point.
                    plt.plot(
                        nav_data.act_poses[0, 1],
                        nav_data.act_poses[0, 2],
                        color="g",
                        marker="*",
                        markersize=12,
                        linestyle="None",
                        label="start",
                    )
                    # planned waypoints.
                    for index, pt in enumerate(experiment["waypoints"]):
                        plt.plot(pt[0], pt[1], color="y", marker="o", linestyle="None")
                        plt.text(pt[0], pt[1], "wpt " + str(index))
                    # actual waypoints.
                    for pt in nav_data.act_wpts:
                        plt.scatter(pt[1], pt[2], s=80, facecolors="none", edgecolors="r")
                    plt.title(
                        f"{method_name} \n trajectory length {nav_data.traj_length:.2f} m, trajectory_duration {nav_data.traj_duration:.2f} s \n est rmse {nav_err.est_rmse:.2f} m, nav rmse {nav_err.nav_rmse:.2f} m"
                    )
                    plt.xlabel("x(m)")
                    plt.ylabel("y(m)")
                    plt.legend()
                    if self._params["save_figs"]:
                        plt.savefig(
                            fname=self._output_dir
                            / f"{method_name}_{experiment['path_name']}_round_{str(single_round_data['round'])}_trajectory.png",
                            dpi=fig.dpi,
                        )
                    plt.show(block=False)
                    plt.pause(1)
                    plt.close()

    def __plot_waypoints_errors(self):
        # Loop over each method.
        for method_name, robot_nav_data in self._methods.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                wpts_errs = {}
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    for index, err in zip(nav_data.wpts_indices, nav_err.wpts_errs):
                        if index not in wpts_errs:
                            wpts_errs[index] = [err]
                        else:
                            wpts_errs[index].append(err)
                fig = plt.figure()
                ax = plt.gca()
                max_abs_xy_err = 0.0
                max_abs_theta_err = 0.0
                for index, wpt_errs in wpts_errs.items():
                    xy_err, theta_err = self.__draw_waypoints_errors(
                        np.array(wpt_errs), ax, color=self._colors[index], unit="cm"
                    )
                    max_abs_xy_err = max(max_abs_xy_err, xy_err)
                    max_abs_theta_err = max(max_abs_theta_err, theta_err)
                self.__draw_navigation_control_tolerance(ax)
                max_abs_xy_err *= 1.1
                ax.set_xlim([-max_abs_xy_err, max_abs_xy_err])
                ax.set_ylim([-max_abs_xy_err, max_abs_xy_err])
                ax.set_title(f"{method_name}\nWaypoint navigation position errors")
                ax.set_xlabel("x(cm)")
                ax.set_ylabel("y(cm)")
                ax.set_aspect("equal", adjustable="box")
                ax.legend()
                if self._params["save_figs"]:
                    plt.savefig(
                        fname=self._output_dir / f"{method_name}_{experiment['path_name']}_waypoints_errors.png",
                        dpi=fig.dpi,
                    )
                plt.show(block=False)
                plt.pause(1)
                plt.close()

    def __draw_waypoints_errors(self, wpt_errs: np.ndarray, ax, color, marker="o", unit="m"):
        assert isinstance(wpt_errs, np.ndarray)
        assert wpt_errs.shape[0] > 0
        assert wpt_errs.shape[1] == 3
        scale = 100.0 if unit == "cm" else 1.0
        xy_errs = wpt_errs[:, :2] * scale
        theta_errs = wpt_errs[:, 2]
        max_abs_xy_err = np.max(np.abs(xy_errs))
        max_abs_theta_err = np.max(np.abs(theta_errs))
        ax.plot(xy_errs[:, 0], xy_errs[:, 1], color=color, marker=marker, markersize=5, linestyle="None")
        if xy_errs.shape[0] >= 3:
            hull = ConvexHull(xy_errs)
            for simplex in hull.simplices:
                ax.plot(
                    xy_errs[simplex, 0],
                    xy_errs[simplex, 1],
                    color=color,
                    linestyle="dashdot",
                    markersize=0.5,
                    markeredgecolor="none",
                )
        return max_abs_xy_err, max_abs_theta_err

    def __draw_navigation_control_tolerance(self, ax, is_label_on=True):
        theta = np.linspace(0, 2.0 * np.pi, 150)
        radius = self._params["xy_goal_tolerance"] * 100.0  # m to cm
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        label = "control tolerance" if is_label_on else None
        ax.plot(x, y, label=label, color="r", linewidth=0.3)
        # @TODO orientation

    def __plot_accuracy_and_precision(self):
        # Loop over each method.
        for method_name, robot_nav_data in self._methods.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                fig, axs = plt.subplots(2, 1)
                xdata = range(experiment["waypoints"].shape[0])
                xlabels = ["wpt0"] + [str(i) for i in xdata[1:]]
                axs[0].plot(xdata, experiment["accuracy"][:, 0] * 100.0)
                axs[1].plot(xdata, experiment["precision"][:, 0] * 100.0)
                axs[0].set_xticks(xdata)
                axs[0].set_xticklabels(xlabels)
                axs[1].set_xticks(xdata)
                axs[1].set_xticklabels(xlabels)
                axs[0].set_ylabel("accuracy (cm)")
                axs[1].set_ylabel("precision (cm)")
                # axs[0].legend()
                # axs[1].legend()

                fig2 = plt.figure()
                ax2 = plt.gca()
                ax2.plot(
                    experiment["accuracy"][:, 0] * 100.0,
                    experiment["precision"][:, 0] * 100.0,
                    marker="o",
                    linestyle="none",
                )
                ax2.set_xlabel("accuracy (cm)")
                ax2.set_ylabel("precision (cm)")
                ax2.set_xlim([0.0, 100.0])
                ax2.set_ylim([0.0, 100.0])
                ax2.set_aspect("equal", adjustable="box")
                ax2.legend()

                if self._params["save_figs"]:
                    fig.savefig(
                        fname=self._output_dir / f"{method_name}_{experiment['path_name']}_wpt_accuracy_precision.png",
                        dpi=fig.dpi,
                    )
                    fig2.savefig(
                        fname=self._output_dir / f"{method_name}_{experiment['path_name']}_accuracy_precision.png",
                        dpi=fig.dpi,
                    )

                plt.show(block=False)
                plt.pause(1)
                plt.close(fig)
                plt.close(fig2)
