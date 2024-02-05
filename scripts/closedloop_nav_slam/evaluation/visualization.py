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
    compute_weighted_accuracy_and_precision,
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
        self._output_dir = Path(self._params["output_dir"]) / "figs"
        self._output_dir.mkdir(exist_ok=True, parents=True)

    def run(self):
        logging.info("plot trajectory ...")
        self.__plot_trajectory()
        logging.info("plot waypoints errors ... ")
        self.__plot_waypoints_errors()
        logging.info("plot accuracy and precision ... ")
        self.__plot_accuracy_and_precision()
        logging.info('plot estimation error against gt slam method ... ')
        self.__plot_estimation_errors_gt_slam()
        logging.info('plot weighted accuracy and precision ... ')
        self.__plot_accuracy_and_precision_against_rounds()

    def __plot_trajectory(self):
        # Loop over each method.
        for method_name, robot_nav_data in self._methods.items():
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
                        f"{method_name} \n trajectory length {nav_data.traj_length:.2f} m, trajectory_duration {nav_data.traj_duration:.2f} s \n est rmse {nav_err.est_rmse:.2f} m, nav rmse {nav_err.nav_rmse:.2f} m, completion {nav_err.completion:.2f}"
                    )
                    plt.xlabel("x(m)")
                    plt.ylabel("y(m)")
                    plt.legend()
                    if self._params["save_figs"]:
                        fig_dir = self._output_dir / method_name / experiment['path_name']
                        fig_dir.mkdir(exist_ok=True, parents=True)
                        plt.savefig(
                            fname= fig_dir / f"round_{str(single_round_data['round'])}_trajectory.png",
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
                    if nav_data is None or nav_err is None:
                        continue
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
                        np.array(wpt_errs), ax, color="k", unit="cm", markersize=1
                    )
                    max_abs_xy_err = max(max_abs_xy_err, xy_err)
                    max_abs_theta_err = max(max_abs_theta_err, theta_err)
                self.__draw_navigation_control_tolerance(ax)
                max_abs_xy_err = max(self._params["goal_reached_thresh"] * 100.0, max_abs_xy_err)
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

        # Plot all methods to a single plot.
        # Loop over each method.
        fig = plt.figure()
        ax = plt.gca()
        max_abs_xy_err = 0.0
        max_abs_theta_err = 0.0
        for method_index, (method_name, robot_nav_data) in enumerate(self._methods.items()):
            all_wpts_errs = []
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    if nav_data is None or nav_err is None:
                        continue
                    all_wpts_errs.append(nav_err.wpts_errs)
            xy_err, theta_err = self.__draw_waypoints_errors(
                np.vstack(all_wpts_errs), ax, color=Visualization.COLORS[method_index], markersize=1, unit="cm", label=method_name
            )
            max_abs_xy_err = max(max_abs_xy_err, xy_err)
            max_abs_theta_err = max(max_abs_theta_err, theta_err)
        self.__draw_navigation_control_tolerance(ax)
        max_abs_xy_err = max(self._params["goal_reached_thresh"] * 100.0, max_abs_xy_err)
        max_abs_xy_err *= 1.1
        ax.set_xlim([-max_abs_xy_err, max_abs_xy_err])
        ax.set_ylim([-max_abs_xy_err, max_abs_xy_err])
        ax.set_title("Waypoint navigation position errors")
        ax.set_xlabel("x(cm)")
        ax.set_ylabel("y(cm)")
        ax.set_aspect("equal", adjustable="box")
        ax.legend()
        if self._params["save_figs"]:
            plt.savefig(
                fname=self._output_dir / f"{experiment['path_name']}_waypoints_errors.png",
                dpi=fig.dpi,
            )
        plt.show(block=False)
        plt.pause(1)
        plt.close()

    def __draw_waypoints_errors(self, wpt_errs: np.ndarray, ax, color, markersize=5, marker="o", unit="m", label=None):
        assert isinstance(wpt_errs, np.ndarray)
        assert wpt_errs.shape[0] > 0
        assert wpt_errs.shape[1] == 3
        scale = 100.0 if unit == "cm" else 1.0
        xy_errs = wpt_errs[:, :2] * scale
        theta_errs = wpt_errs[:, 2]
        max_abs_xy_err = np.max(np.abs(xy_errs))
        max_abs_theta_err = np.max(np.abs(theta_errs))
        ax.plot(xy_errs[:, 0], xy_errs[:, 1], color=color, marker=marker, markersize=markersize, linestyle="None", label=label)
        if xy_errs.shape[0] >= 3:
            hull = ConvexHull(xy_errs)
            for simplex in hull.simplices:
                ax.plot(
                    xy_errs[simplex, 0],
                    xy_errs[simplex, 1],
                    color=color,
                    linestyle="dashdot",
                    linewidth=0.5,
                    markeredgecolor="none",
                )
        return max_abs_xy_err, max_abs_theta_err

    def __draw_navigation_control_tolerance(self, ax, is_label_on=True):
        theta = np.linspace(0, 2.0 * np.pi, 150)
        radius = self._params["goal_reached_thresh"] * 100.0  # m to cm
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
                fig, axs = plt.subplots(3, 1)
                xdata = range(experiment["waypoints"].shape[0])
                xlabels = ["wpt0"] + [str(i) for i in xdata[1:]]
                axs[0].plot(xdata, experiment["accuracy"][:, 0] * 100.0)
                axs[1].plot(xdata, experiment["precision"][:, 0] * 100.0)
                wpts_sr = [s.success_rate * 100.0 for s in experiment["wpts_stats"]]
                axs[2].plot(xdata, wpts_sr)
                for ax_index in range(3):
                    axs[ax_index].set_xticks(xdata)
                    axs[ax_index].set_xticklabels(xlabels)
                axs[2].set_ylim([0.0, 102.0])
                axs[0].set_ylabel("accuracy (cm)")
                axs[1].set_ylabel("precision (cm)")
                axs[2].set_ylabel("success rate (%)")
                axs[0].set_title(f"{method_name} \n Waypoints Accuracy & Precision")
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
                ax2.set_title(f"{method_name} \n Accuracy & Precision")
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

        # Plot all methods on the same plot.
        # Loop over each method.
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111)
        for method_index, (method_name, robot_nav_data) in enumerate(self._methods.items()):
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                ax.plot(
                    experiment["accuracy"][:, 0] * 100.0,
                    experiment["precision"][:, 0] * 100.0,
                    marker="o",
                    linestyle="none",
                    label=method_name,
                    color=Visualization.COLORS[method_index],
                )

        # Plot a precision==accuracy boundary
        xx = np.arange(0.0, 110.0)
        ax.plot(xx, xx, linestyle="dashed", linewidth=1, color="r")
        ax.set_xlabel("accuracy (cm)")
        ax.set_ylabel("precision (cm)")
        ax.set_xlim([0.0, 110.0])
        ax.set_ylim([0.0, 110.0])
        # ax.set_aspect("equal", adjustable="box")
        ax.set_title("Accuracy & Precision")
        ax.legend()
        # ax.grid(False)

        # Plot a second axis
        # Set scond x-axis
        ax2 = ax.twiny()
        # ax2.xaxis.set_ticks_position("bottom")
        # ax2.xaxis.set_label_position("bottom")
        # ax2.spines["bottom"].set_position(("axes", -0.35))
        # ax2.set_frame_on(True)
        # ax2.patch.set_visible(False)
        # for sp in ax2.spines.values():
        #     sp.set_visible(False)
        # ax2.spines["bottom"].set_visible(True)
        newlabel = range(4)
        newpos = [i * 35.4 for i in newlabel] # Dim of turtlebot2 in cm
        ax2.set_xticks(newpos)
        ax2.set_xticklabels(newlabel)
        ax2.set_xlabel("Metric as Turtlebot2 Size (354mm)")
        ax2.set_xlim(ax.get_xlim())
        ax2.grid(False)

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir / f"{experiment['path_name']}_accuracy_precision.png",
                dpi=fig.dpi,
            )

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)

    def __plot_estimation_errors_gt_slam(self):
        # Loop over each method.
        for method_name, robot_nav_data in self._methods.items():
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                rmses = []
                rounds = []
                for single_round_data in experiment["rounds"]:
                    nav_data = single_round_data["nav_data"]
                    nav_err = single_round_data["nav_err"]
                    if nav_data is None or nav_data.gt_slam_poses is None or nav_err is None:
                        continue
                    else:
                        rmses.append([nav_err.est_rmse, nav_err.est_rmse_gt_slam])
                        rounds.append(single_round_data["round"])
                if len(rmses) == 0:
                    continue
                rmses_arr = np.array(rmses)
                fig = plt.figure()
                ax = fig.add_subplot(211)
                ax.set_title(f"{method_name} \n Ground Truth Comparison.")
                positions = [1.0, 2.0]
                vplot = ax.boxplot(rmses_arr, positions = positions, showmeans=True)
                # for pc, color in zip(vplot['bodies'], ["g", "y"]):
                #     pc.set_facecolor(color)
                #     pc.set_edgecolor(color)
                #     pc.set_alpha(0.5)
                # vplot["cmeans"].set_color("black")
                # vplot["cmedians"].set_color("r")
                    
                ax.set_xticks(positions)
                ax.set_xticklabels(["GT", "slam_toolbox"])
                ax.set_ylabel("RMSE (m)")

                ax = fig.add_subplot(212)
                ax.plot(rounds, rmses_arr[:, 0], label="GT", color="g")
                ax.plot(rounds, rmses_arr[:, 1], label="slam_toolbox", color="y")
                ax.set_ylabel("RMSE (m)")
                ax.set_xlabel("Rounds")
                ax.set_xticks(rounds)
                ax.legend()

                if self._params["save_figs"]:
                    fig.savefig(
                        fname=self._output_dir / f"{method_name}_{experiment['path_name']}_estimation_error.png",
                        dpi=fig.dpi,
                    )
                plt.show(block=False)
                plt.pause(1)
                plt.close(fig)

    def __plot_accuracy_and_precision_against_rounds(self):
        fig, axs = plt.subplots(2, 1)
        # Loop over each method.
        for method_index, (method_name, robot_nav_data) in enumerate(self._methods.items()):
            # Loop over each experiment (path).
            for experiment in robot_nav_data.data:
                planned_wpts = experiment["waypoints"]
                act_wpts = np.full(experiment["waypoints"].shape + (len(experiment["rounds"]),), np.nan)
                rounds = 0
                # Fill-in actual wpts from all the rounds.
                for round_data in experiment["rounds"]:
                    rounds += 1
                    nav_data = round_data["nav_data"]
                    if nav_data is None:
                        continue
                    act_wpts[nav_data.wpts_indices, :, round_data["round"]] = nav_data.act_wpts[:, 1:]

                accuracy, precision = compute_weighted_accuracy_and_precision(planned_wpts, act_wpts)

                xdata = range(2, rounds+1)
                axs[0].plot(xdata, accuracy[:, 0] * 100.0, color=Visualization.COLORS[method_index], label=method_name)
                axs[1].plot(xdata, precision[:, 0] * 100.0, color=Visualization.COLORS[method_index], label=method_name)
                axs[0].set_xticks(xdata)
                axs[1].set_xticks(xdata)
                axs[0].set_ylabel("accuracy (cm)")
                axs[1].set_ylabel("precision (cm)")
                axs[0].set_title(f"Accuracy & Precision V.S. Rounds")
                axs[1].set_xlabel("rounds")
                axs[0].set_ylim([0, 100])
                axs[1].set_ylim([0, 50])
                axs[0].legend()
                axs[1].legend()

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir / f"{experiment['path_name']}_accuracy_precision_rounds.png",
                dpi=fig.dpi,
            )

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)