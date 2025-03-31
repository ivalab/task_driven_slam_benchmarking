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

import itertools
import logging
import math
import pickle
from copy import deepcopy
from pathlib import Path
from typing import Dict

import matplotlib
from matplotlib import pyplot as plt
import numpy as np
from cl_sim.utils.utils import (
    RobotNavigationData,
    load_params,
    load_yaml,
)
from cl_common.slam_methods import (
    METHODS_VISUALS,
    SENSOR_TO_MARKER,
)
from cl_common.utils import (
    draw_grids,
    draw_x_eq_y,
    draw_circles,
)
from scipy.spatial import ConvexHull
from cl_sim.utils.path_definitions import CONFIG_PATH

# print(matplotlib.style.available)

# from matplotlib import rc
# from matplotlib import rcParams
# import matplotlib.font_manager

# rc("font", **{"family": "serif", "serif": ["Computer Modern"]})
# rc("text", usetex=True)


# Enable LaTeX in Matplotlib
plt.rcParams.update(
    {
        "text.usetex": True,  # Use LaTeX for text rendering
        "font.family": "serif",  # Use serif font
        "font.serif": ["Computer Modern"],  # Use the Computer Modern font
        "text.latex.preamble": r"\usepackage{amsmath}",  # Use AMSMath package for more advanced math
    }
)
plt.style.use("default")


class Visualization:
    """_summary_

    Returns:
        _type_: _description_
    """

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

    def __init__(self, params):
        self._params = deepcopy(params)
        self._env_params = None
        self._results = None
        self._output_dir = None
        # self._colors = [plt.cm.jet(random.random()) for _ in range(50)]  # Generate random colors
        # markers = itertools.cycle(Visualization.MARKERS)
        # self._markers = [next(markers) for _ in range(len(self._results))]
        # self._colors = matplotlib.cm.tab20(range(20))

        self._grid_xy = self._params["grid_xy"]
        self._grid_theta = self._params["grid_theta"]
        self._grids = [self._grid_xy, self._grid_theta]
        self._grid_res = [10, 5]  # cm & degree
        self._subplot_labels = ["Position", "Orientation"]

    def run(self, eval_data: Dict[str, Dict[str, RobotNavigationData]]):
        """_summary_

        Args:
            eval_data (Dict[str, Dict[str, RobotNavigationData]]): _description_
        """
        for env_name, results in eval_data.items():

            # Update parameters before visualization.
            self._output_dir = Path(self._params["result_dir"]) / self._params["mode"] / env_name / "figs"
            self._output_dir.mkdir(parents=True, exist_ok=True)
            print(self._output_dir)
            if self._params["evaluation_prefix"]:
                (self._output_dir / self._params["evaluation_prefix"]).mkdir(parents=True, exist_ok=True)
            self._params["env_name"] = env_name
            self._env_params = load_yaml(CONFIG_PATH / "envs" / (env_name + ".yaml"))
            self._results = results
            for method_name, _ in self._results.items():
                (self._output_dir / method_name).mkdir(parents=True, exist_ok=True)

            # Start plotting
            self.__plot_trajectory()
            self.__plot_waypoints_errors()
            # self.__plot_waypoints_errors_subplots()
            # self.__plot_wpts_stats()
            # self.__plot_accuracy_and_precision_by_sensor_type()
            # self.__plot_methods_rank()

    def __plot_trajectory(self):
        """_summary_"""
        logging.info("plot trajectory ...")
        # Loop over each method.
        for method_name, robot_nav_data in self._results.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                # Loop over each round(trial).
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]

                    if nav_data is None or nav_err is None:
                        continue

                    fig = plt.figure()
                    # actual traj.
                    plt.plot(nav_data.act_poses[:, 1], nav_data.act_poses[:, 2], color="g", label="actual")
                    # esti traj.
                    plt.plot(nav_data.est_poses[:, 1], nav_data.est_poses[:, 2], color="r", label="estimated")
                    # gt slam traj if has
                    if nav_data.gt_slam_poses is not None:
                        plt.plot(nav_data.gt_slam_poses[:, 1], nav_data.gt_slam_poses[:, 2], color="y", label="gt slam")
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
                        plt.plot(pt[0], pt[1], color="b", marker="o", linestyle="None")
                        plt.text(pt[0], pt[1], "wpt " + str(index))
                    # actual waypoints.
                    for pt in nav_data.act_wpts:
                        plt.scatter(pt[1], pt[2], s=80, facecolors="none", edgecolors="g")
                    plt.title(
                        f"{method_name} \n length {nav_data.traj_length:.2f} m, duration {nav_data.traj_duration:.2f} s \n est rmse {nav_err.est_rmse:.2f} m, nav rmse {nav_err.nav_rmse:.2f} m, completion {nav_err.completion:.2f}"
                    )
                    plt.xlabel("X(m)")
                    plt.ylabel("Y(m)")
                    # plt.legend()
                    plt.legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
                    plt.tight_layout()
                    if self._params["save_figs"]:
                        fig_dir = self._output_dir / method_name / experiment["path_name"]
                        fig_dir.mkdir(exist_ok=True, parents=True)
                        plt.savefig(
                            fname=fig_dir / f"round_{str(single_round_data['round'])}_trajectory.png",
                            dpi=fig.dpi,
                        )
                    plt.show(block=False)
                    plt.pause(1)
                    plt.close()

    def __plot_waypoints_errors(self):
        logging.info("plot waypoints errors ... ")
        # Loop over each method.
        for method_name, robot_nav_data in self._results.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                indexed_wpts_errs = {}
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    if nav_data is None or nav_err is None:
                        continue
                    for index, err in zip(nav_data.wpts_indices, nav_err.wpts_errs):
                        if index not in indexed_wpts_errs:
                            indexed_wpts_errs[index] = []
                        indexed_wpts_errs[index].append(err)
                # Start plotting
                fig, ax = plt.subplots(1, 1)
                axs = [ax]
                max_abs_xy_err = 0.0
                for wpt_index, wpt_errs in indexed_wpts_errs.items():
                    xy_err = self.__draw_wpt_position_errs(
                        axs[0], np.array(wpt_errs)[:, :2], color="k", unit="cm", markersize=1
                    )
                    max_abs_xy_err = max(max_abs_xy_err, xy_err)
                self.__draw_navigation_control_tolerance(axs[0])
                max_abs_xy_err = max(self._params["goal_reached_thresh"] * 100.0, max_abs_xy_err)
                max_abs_xy_err *= 1.1
                axs[0].set_xlim([-max_abs_xy_err, max_abs_xy_err])
                axs[0].set_ylim([-max_abs_xy_err, max_abs_xy_err])
                axs[0].set_title(f"{method_name}\nWaypoint {self._subplot_labels[0]} Errors")
                axs[0].set_xlabel("X (cm)")
                axs[0].set_ylabel("Y (cm)")
                axs[0].set_aspect("equal", adjustable="box")
                axs[0].legend()
            if self._params["save_figs"]:
                plt.savefig(
                    fname=self._output_dir / method_name / f"{self._env_params['path'][0]}_waypoints_errors.png",
                    dpi=fig.dpi,
                )
            plt.show(block=False)
            plt.pause(1)
            plt.close()

        # Plot all methods to a single plot.
        # Loop over each method.
        fig, ax = plt.subplots(1, 1)
        max_abs_xy_err = 0.0
        for method_index, (method_name, robot_nav_data) in enumerate(self._results.items()):
            all_wpts_errs = []
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    if nav_data is None or nav_err is None:
                        continue
                    all_wpts_errs.append(nav_err.wpts_errs)
            xy_err = self.__draw_wpt_position_errs(
                ax,
                np.vstack(all_wpts_errs)[:, :2],
                color=METHODS_VISUALS[method_name].color,
                marker=METHODS_VISUALS[method_name].marker,
                markersize=3,
                unit="cm",
                label=METHODS_VISUALS[method_name].label,
            )
            max_abs_xy_err = max(max_abs_xy_err, xy_err)
        self.__draw_navigation_control_tolerance(ax)
        max_abs_xy_err = max(self._params["goal_reached_thresh"] * 100.0, max_abs_xy_err)
        max_abs_xy_err *= 1.1
        ax.set_xlim([-max_abs_xy_err, max_abs_xy_err])
        ax.set_ylim([-max_abs_xy_err, max_abs_xy_err])
        ax.set_title("Waypoints Position Error (cm)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_aspect("equal", adjustable="box")
        ax.legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
        plt.tight_layout()
        if self._params["save_figs"]:
            plt.savefig(
                fname=self._output_dir
                / self._params["evaluation_prefix"]
                / f"{self._env_params['path'][0]}_waypoints_errors.png",
                dpi=fig.dpi,
            )
        plt.show(block=False)
        plt.pause(1)
        plt.close()

    def __draw_wpt_position_errs(self, ax, xy_errs: np.ndarray, color, markersize=5, marker="o", unit="cm", label=None):
        """_summary_

        Args:
            ax (_type_): _description_
            wpt_errs (np.ndarray): _description_
            color (_type_): _description_
            markersize (int, optional): _description_. Defaults to 5.
            marker (str, optional): _description_. Defaults to "o".
            unit (str, optional): _description_. Defaults to "cm".
            label (_type_, optional): _description_. Defaults to None.

        Returns:
            _type_: _description_
        """
        assert isinstance(xy_errs, np.ndarray)
        assert xy_errs.shape[0] > 0
        assert xy_errs.shape[1] == 2  # x, y, ...
        scale = 100.0 if unit == "cm" else 1.0
        xy_errs *= scale
        max_abs_xy_err = np.max(np.abs(xy_errs))
        ax.plot(
            xy_errs[:, 0],
            xy_errs[:, 1],
            color=color,
            marker=marker,
            markersize=markersize,
            linestyle="None",
            label=label,
        )
        if xy_errs.shape[0] >= 3:
            hull = ConvexHull(xy_errs)
            for simplex in hull.simplices:
                ax.plot(
                    xy_errs[simplex, 0],
                    xy_errs[simplex, 1],
                    color="green",
                    linestyle="dashdot",
                    linewidth=0.7,
                    markeredgecolor="none",
                )
        return max_abs_xy_err

    def __draw_wpt_orientation_errs(
        self, ax, theta_errs: np.ndarray, color, markersize=5, marker="o", unit="degree", label=None
    ):
        """_summary_

        Args:
            ax (_type_): _description_
            theta_errs (np.ndarray): _description_
            color (_type_): _description_
            markersize (int, optional): _description_. Defaults to 5.
            marker (str, optional): _description_. Defaults to "o".
            unit (str, optional): _description_. Defaults to "cm".
            label (_type_, optional): _description_. Defaults to None.

        Returns:
            _type_: _description_
        """
        assert isinstance(theta_errs, np.ndarray)
        if unit == "degree":
            theta_errs = np.rad2deg(theta_errs)
        max_abs_theta_err = np.max(np.abs(theta_errs))
        # TOOD (yanwei): plot orientation
        ax.hist(theta_errs, bins=10, color="blue", alpha=0.7, label="Orientation Error")
        return max_abs_theta_err

    def __draw_navigation_control_tolerance(self, ax, is_label_on=True):
        theta = np.linspace(0, 2.0 * np.pi, 150)
        radius = self._params["goal_reached_thresh"] * 100.0  # m to cm
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        label = "control tolerance" if is_label_on else None
        ax.plot(x, y, label=label, color="r", linewidth=0.5)
        # @TODO orientation

    def __plot_waypoints_errors_subplots(self):
        logging.info("plot waypoints errors ... ")
        num_methods = len(self._results)
        num_rows = 1 if num_methods < 5 else 2
        num_cols = int(np.ceil(float(num_methods) / num_rows))
        fig, axs = plt.subplots(num_rows, num_cols, sharex=True, sharey=True)
        if num_methods == 1:
            axs = [axs]  # Ensure axes is iterable if there's only one subplot
        axs = axs.flatten()
        assert len(axs) >= num_methods, print(len(axs), num_methods)
        # Loop over each method.
        max_abs_xy_err = 0.0
        for ax, (method_name, robot_nav_data) in zip(axs[: num_methods + 1], self._results.items()):
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                indexed_wpts_errs = {}
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    if nav_data is None or nav_err is None:
                        continue
                    for index, err in zip(nav_data.wpts_indices, nav_err.wpts_errs):
                        if index not in indexed_wpts_errs:
                            indexed_wpts_errs[index] = []
                        indexed_wpts_errs[index].append(err)
                # Start plotting
                for wpt_index, wpt_errs in indexed_wpts_errs.items():
                    xy_err = self.__draw_wpt_position_errs(
                        ax, np.array(wpt_errs)[:, :2], color="red", unit="cm", markersize=1
                    )
                    max_abs_xy_err = max(max_abs_xy_err, xy_err)
                # self.__draw_navigation_control_tolerance(ax)
                # max_abs_xy_err = max(self._params["goal_reached_thresh"] * 100.0, max_abs_xy_err)
                # max_abs_xy_err *= 1.1
                # ax.set_title(f"{method_name}\nWaypoint {self._subplot_labels[0]} Errors (cm)")
                # ax.set_xlabel("X")
                # ax.set_ylabel("Y")
                ax.set_aspect("equal", adjustable="box")
                ax.set_title(method_name)
                # ax.legend()
        axs[0].set_xlim([-max_abs_xy_err, max_abs_xy_err])
        axs[0].set_ylim([-max_abs_xy_err, max_abs_xy_err])
        for ax in axs:
            draw_circles(ax, self._grid_xy, max_abs_xy_err, enable_xy_ticks=False)
        fig.supxlabel("X (cm)")
        fig.supylabel("Y (cm)")
        fig.suptitle("Waypoint Position Error")
        # fig.tight_layout()
        if self._params["save_figs"]:
            plt.savefig(
                fname=self._output_dir / f"{self._env_params['path'][0]}_waypoints_errors_all.png",
                dpi=fig.dpi,
            )
        plt.show(block=False)
        plt.pause(1)
        plt.close()

    def __plot_wpts_stats(self):
        """_summary_"""
        logging.info("plot wpts stats ... ")
        # Plot waypoint stats for each method, all wpts are considered, even those do not have full completeness.
        for method_name, robot_nav_data in self._results.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                fig, axs = plt.subplots(3, 1, sharex=True)
                xdata = range(experiment["waypoints"].shape[0])
                scales = [100.0, 180.0 / np.pi]  # Unit conversion.
                for sp_index, sp_label in enumerate(self._subplot_labels):
                    axs[sp_index].plot(
                        xdata, experiment["accuracy"][:, sp_index] * scales[sp_index], "--bo", label="Accuracy"
                    )
                    axs[sp_index].plot(
                        xdata, experiment["precision"][:, sp_index] * scales[sp_index], "--go", label="Precision"
                    )
                    axs[sp_index].legend(loc="upper left")
                    axs[sp_index].set_xticks(xdata)
                    axs[sp_index].set_xticklabels([])
                axs[0].set_ylabel("Error(cm)")
                axs[1].set_ylabel("Error(degree)")

                wpts_sr = [s.success_rate * 100.0 for s in experiment["wpts_stats"]]
                axs[2].plot(xdata, wpts_sr)
                axs[2].set_ylim([0.0, 102.0])
                axs[2].set_ylabel("Completeness(%)")

                # set xticks
                xlabels = ["wpt0"] + [str(i) for i in xdata[1:]]
                axs[2].set_xticks(xdata)
                axs[2].set_xticklabels(xlabels)

                # axs[0].legend()
                # axs[1].legend()
                fig.suptitle(f"{method_name} wpt stats")
                if self._params["save_figs"]:
                    fig.savefig(
                        fname=self._output_dir / method_name / f"{experiment['path_name']}_wpt_stats.png",
                        dpi=fig.dpi,
                    )
                plt.show(block=False)
                plt.pause(1)
                plt.close(fig)

    def __plot_accuracy_and_precision(self):
        # Plot scattered waypoints.
        # Consider only valid waypoints (waypoint that have full success).
        fig, axs = plt.subplots(1, 2, figsize=(14, 6))
        max_pre_acc = np.zeros((2))
        rad2deg = 180.0 / np.pi
        result = {}
        for method_index, (method_name, robot_nav_data) in enumerate(self._results.items()):
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                indices = experiment["completeness"] > 0.99
                completeness = np.sum(indices) / float(experiment["completeness"].shape[0]) * 100.0
                acc = experiment["accuracy"][indices, :] * [100.0, rad2deg]
                pre = experiment["precision"][indices, :] * [100.0, rad2deg]
                # Position
                axs[0].plot(
                    acc[:, 0],
                    pre[:, 0],
                    linestyle="none",
                    label=METHODS_VISUALS[method_name].label,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markersize=8,
                    markerfacecolor="none",
                    markeredgewidth=2,
                )

                # Orientation
                axs[1].plot(
                    acc[:, 1],
                    pre[:, 1],
                    linestyle="none",
                    label=METHODS_VISUALS[method_name].label,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markersize=8,
                    markerfacecolor="none",
                    markeredgewidth=2,
                )

                # Get the max.
                max_pre_acc = np.fmax(
                    max_pre_acc,
                    np.fmax(
                        np.nanmax(acc, axis=0),
                        np.nanmax(pre, axis=0),
                    ),
                )

                # stack to list
                result[method_name] = [np.mean(acc, axis=0), np.mean(pre, axis=0), completeness]

        # Plot a precision==accuracy boundary and grids.
        for sp_index in range(2):
            max_err = np.ceil(max_pre_acc[sp_index] / self._grids[sp_index]) * self._grids[sp_index]
            draw_grids(axs[sp_index], self._grids[sp_index], [0, max_err])
            draw_x_eq_y(axs[sp_index], 0.0, max_err)

        # ax.set_xlabel("Accuracy")
        # ax.set_ylabel("Precision")
        axs[0].set_aspect("equal", adjustable="box")
        axs[1].set_aspect("equal", adjustable="box")
        axs[0].set_title("Position (cm)")
        axs[1].set_title("Orientation (degree)")
        # axs[0].legend()
        axs[0].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
        fig.supylabel("Precision")
        fig.supxlabel("Accuracy")
        plt.tight_layout()
        # ax.grid(False)

        # Plot a second axis
        # Set scond x-axis
        # ax2 = ax.twiny()
        # ax2.xaxis.set_ticks_position("bottom")
        # ax2.xaxis.set_label_position("bottom")
        # ax2.spines["bottom"].set_position(("axes", -0.35))
        # ax2.set_frame_on(True)
        # ax2.patch.set_visible(False)
        # for sp in ax2.spines.values():
        #     sp.set_visible(False)
        # ax2.spines["bottom"].set_visible(True)
        # newlabel = range(4)
        # # Dim of turtlebot2 in cm
        # newpos = [i * self._grid_xy for i in newlabel]
        # ax2.set_xticks(newpos)
        # ax2.set_xticklabels(newlabel)
        # ax2.set_xlabel("Metric as Turtlebot2 Size (354mm)")
        # ax2.set_xlim(ax.get_xlim())
        # ax2.grid(False)

        result_filename = str(
            self._output_dir / self._params["evaluation_prefix"] / f"{self._params['env_name']}_eval.pkl"
        )
        with open(result_filename, "wb") as file:
            pickle.dump(result, file)

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir
                / self._params["evaluation_prefix"]
                / f"{self._env_params['path'][0]}_accuracy_precision.png",
                dpi=fig.dpi,
            )

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)

    def __plot_accuracy_and_precision_by_sensor_type(self):
        # Plot scattered waypoints.
        # Consider only valid waypoints (waypoint that have full success).
        num_sensors = len(SENSOR_TO_MARKER)
        sensor_to_sp_index = {sensor: index for index, (sensor, _) in enumerate(SENSOR_TO_MARKER.items())}
        sp_index_to_sensor = {index: sensor for sensor, index in sensor_to_sp_index.items()}
        fig, axs = plt.subplots(2, num_sensors)
        fig.subplots_adjust(wspace=0.5)
        max_pre_acc = np.zeros((2))
        rad2deg = 180.0 / np.pi
        for method_index, (method_name, robot_nav_data) in enumerate(self._results.items()):
            # Loop over each experiment (path).
            sp_index = sensor_to_sp_index[METHODS_VISUALS[method_name].sensor_type]
            for experiment in robot_nav_data.data:
                indices = experiment["completeness"] > 0.99
                if np.sum(indices) == 0:
                    continue
                # completeness = np.sum(indices) / float(experiment["completeness"].shape[0]) * 100.0
                acc = experiment["accuracy"][indices, :] * [100.0, rad2deg]
                pre = experiment["precision"][indices, :] * [100.0, rad2deg]
                # Position
                axs[0, sp_index].plot(
                    acc[:, 0],
                    pre[:, 0],
                    linestyle="none",
                    label=METHODS_VISUALS[method_name].label,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markersize=8,
                    markerfacecolor="none",
                    markeredgewidth=2,
                )

                # Orientation
                axs[1, sp_index].plot(
                    acc[:, 1],
                    pre[:, 1],
                    linestyle="none",
                    # label=METHODS_VISUALS[method_name].label,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markersize=8,
                    markerfacecolor="none",
                    markeredgewidth=2,
                )

                # Get the max.
                max_pre_acc = np.fmax(
                    max_pre_acc,
                    np.fmax(
                        np.nanmax(acc, axis=0),
                        np.nanmax(pre, axis=0),
                    ),
                )

        # Plot a precision==accuracy boundary and grids.
        for sp_index in range(2):
            max_err = np.ceil(max_pre_acc[sp_index] / self._grids[sp_index]) * self._grids[sp_index]
            for sensor_index in range(num_sensors):
                draw_grids(axs[sp_index, sensor_index], self._grids[sp_index], [0, max_err])
                draw_x_eq_y(axs[sp_index, sensor_index], 0.0, max_err)
                axs[0, sensor_index].set_title(f"{sp_index_to_sensor[sensor_index]}")
                axs[sp_index, sensor_index].set_aspect("equal", adjustable="box")
                if sp_index == 0:
                    axs[sp_index, sensor_index].legend(loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=1)

        # ax.set_xlabel("Accuracy")
        # ax.set_ylabel("Precision")
        # axs[1].set_aspect("equal", adjustable="box")
        axs[0, 0].set_ylabel("Position (cm)")
        axs[1, 0].set_ylabel("Orientation (degree)")
        # axs[0, 0].legend(facecolor='white', framealpha=0.5, loc="best")
        fig.supylabel("Precision")
        fig.supxlabel("Accuracy")
        plt.tight_layout()
        # ax.grid(False)

        # Plot a second axis
        # Set scond x-axis
        # ax2 = ax.twiny()
        # ax2.xaxis.set_ticks_position("bottom")
        # ax2.xaxis.set_label_position("bottom")
        # ax2.spines["bottom"].set_position(("axes", -0.35))
        # ax2.set_frame_on(True)
        # ax2.patch.set_visible(False)
        # for sp in ax2.spines.values():
        #     sp.set_visible(False)
        # ax2.spines["bottom"].set_visible(True)
        # newlabel = range(4)
        # # Dim of turtlebot2 in cm
        # newpos = [i * self._grid_xy for i in newlabel]
        # ax2.set_xticks(newpos)
        # ax2.set_xticklabels(newlabel)
        # ax2.set_xlabel("Metric as Turtlebot2 Size (354mm)")
        # ax2.set_xlim(ax.get_xlim())
        # ax2.grid(False)

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir
                / self._params["evaluation_prefix"]
                / f"{self._env_params['path'][0]}_accuracy_precision_sensor_type.png",
                dpi=fig.dpi,
            )

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)

    def __plot_methods_rank(self):
        fig = plt.figure()
        ax0 = fig.add_subplot(2, 2, 1)
        ax1 = fig.add_subplot(2, 2, 2, sharey=ax0, sharex=ax0)
        ax2 = fig.add_subplot(2, 2, 3, sharex=ax0)
        ax3 = fig.add_subplot(2, 2, 4, sharex=ax0, sharey=ax2)
        axs = np.array(([[ax0, ax1], [ax2, ax3]]))
        max_pre = np.zeros((2))
        max_acc = np.zeros((2))
        min_completeness = 100
        max_completeness = 0
        rad2deg = 180.0 / np.pi
        for method_index, (method_name, robot_nav_data) in enumerate(self._results.items()):
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                indices = experiment["completeness"] > 0.99
                acc = np.mean(experiment["accuracy"][indices, :] * [100.0, rad2deg], axis=0)
                pre = np.mean(experiment["precision"][indices, :] * [100.0, rad2deg], axis=0)
                completeness = np.sum(indices) / float(experiment["completeness"].shape[0]) * 100.0

                for sp_index, sp_label in enumerate(self._subplot_labels):
                    axs[sp_index, 0].plot(
                        completeness,
                        acc[sp_index],
                        color=METHODS_VISUALS[method_name].color,
                        marker=METHODS_VISUALS[method_name].marker,
                        markersize=8,
                        markeredgewidth=2,
                        markerfacecolor="none",
                        label=METHODS_VISUALS[method_name].label,
                        linestyle="None",
                    )

                    axs[sp_index, 1].plot(
                        completeness,
                        pre[sp_index],
                        color=METHODS_VISUALS[method_name].color,
                        marker=METHODS_VISUALS[method_name].marker,
                        markersize=8,
                        markeredgewidth=2,
                        markerfacecolor="none",
                        label=METHODS_VISUALS[method_name].label,
                        linestyle="None",
                    )
                min_completeness = min(np.min(completeness), min_completeness)
                max_completeness = max(np.min(completeness), max_completeness)
                max_pre = np.fmax(pre, max_pre)
                max_acc = np.fmax(acc, max_acc)

        max_acc *= 1.2
        max_pre *= 1.2
        # xdata = range(int(min_completeness / 10) * 10, int(math.ceil(max_completeness / 10) * 10) + 5, 10)
        # axs[0, 0].set_xticks(xdata)
        # axs[0, 1].set_xticks(xdata)
        # axs[1, 0].set_xticks(xdata)
        # axs[1, 1].set_xticks(xdata)
        axs[0, 0].set_xlim([min_completeness * 0.8, 105])
        axs[1, 0].set_xlabel("Completeness (%)")
        axs[1, 1].set_xlabel("Completeness (%)")
        axs[0, 0].set_ylabel("Position (cm)")
        axs[1, 0].set_ylabel("Orientation (degree)")
        axs[0, 0].set_title("Accuracy")
        axs[0, 1].set_title("Precision")
        # axs[0].set_ylim([0, max_accuracy ])
        # axs[1].set_ylim([0, max_precision])
        axs[0, 1].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
        # axs[1].legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0)
        # axs[0, 0].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

        plt.tight_layout()

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir
                / self._params["evaluation_prefix"]
                / f"{self._env_params['path'][0]}_accuracy_precision_completeness.png",
                dpi=fig.dpi,
                bbox_inches="tight",
            )

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)
