#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file visualization.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 11-16-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull

from closedloop_nav_slam.evaluation.utils import MethodBase

from typing import List

from pathlib import Path
import numpy as np
import yaml


class Visualization:
    # plot everything on one figure.
    # https://matplotlib.org/stable/gallery/color/named_colors.html
    COLORS = ["lime", "darkorange", "maroon", "darkcyan", "cyan", "violet", "blue", "lime", "darkorange", "maroon", "darkcyan", "cyan", "violet", "blue"]
    MARKERS = ["*", "+", "^", "o", "s", "<", "p", "<", "s", "o", "^", "+", "*"]

    def __init__(self, config_file: str, methods: List[MethodBase], save_figs: bool = False):
        self._methods = methods
        self._params = None
        with open(config_file, "r") as f:
            self._params = yaml.safe_load(f)
        self._params["save_figs"] = save_figs
        assert self._params
        assert len(self._methods) > 0
        self._result_prefix = Path(self._params["result_dir"]) / self._params["test_type"] / self._params["env_name"]

    def run(self):
        # Loop over each method
        for index, method in enumerate(self._methods):
            # - Plot trajectory
            self.__plot_trajectory(method)

            # - Plot accuracy and precision
            self.__plot_accuracy_and_precision(method)

        # - Plot accuracy and precision
        self.__plot_all_methods(self._methods)

    def __plot_trajectory(self, method: MethodBase):
        for pfile, traj in method.traj_dict.items():
            for trial, result in traj.data_dict.items():
                data = result.data
                err = result.err
                fig = plt.figure()
                plt.plot(data.act_poses[:, 1], data.act_poses[:, 2], color="g", label="actual")
                plt.plot(data.est_poses[:, 1], data.est_poses[:, 2], color="r", label="estimated")
                plt.plot(
                    data.act_poses[0, 1],
                    data.act_poses[0, 2],
                    color="g",
                    marker="*",
                    markersize=12,
                    linestyle="None",
                    label="start",
                )
                # Plot waypoints.
                for index, pt in enumerate(data.planned_wpts):
                    plt.plot(pt[0], pt[1], color="y", marker="o", linestyle="None")
                    plt.text(pt[0], pt[1], "wpt " + str(index))
                for pt in data.reached_stamped_wpts:
                    plt.scatter(pt[1], pt[2], s=80, facecolors="none", edgecolors="r")
                plt.title(
                    f"{method.name} \n trajectory length {data.traj_length:.2f} m, est rmse {err.est_rmse:.2f} m, nav rmse {err.nav_rmse:.2f} m"
                )
                plt.xlabel("x(m)")
                plt.ylabel("y(m)")
                plt.legend()
                if self._params["save_figs"]:
                    prefix = self._result_prefix / method.name / pfile
                    plt.savefig(fname=prefix / ("trial" + str(trial) + "_trajectory.png"), dpi=fig.dpi)
                plt.show(block=False)
                plt.pause(1)
                plt.close()

    def __plot_accuracy_and_precision(self, method: MethodBase):
        for pfile, traj in method.traj_dict.items():
            fig_pos_err = plt.figure()
            fig_pos_err.add_subplot(111)
            ax_pos_err = fig_pos_err.get_axes()[0]
            ordered_wpts_errs = np.full((traj.get_planned_wpts_count(), 3, self._params["trials"]), np.nan)
            for trial, result in traj.data_dict.items():
                act_count = result.err.wpts_errs.shape[0]
                ordered_wpts_errs[:act_count, :, trial] = result.err.wpts_errs * 100.0 # m to cm.
            max_abs_err = self.__plot_navigation_position_errors(ax_pos_err, ordered_wpts_errs)
            max_abs_err += max_abs_err * 0.1
            self.__plot_navigation_control_tolerance(ax_pos_err)
            ax_pos_err.set_xlim([-max_abs_err, max_abs_err])
            ax_pos_err.set_ylim([-max_abs_err, max_abs_err])
            ax_pos_err.set_title(f"{method.name}\nWaypoint navigation position errors")
            ax_pos_err.set_xlabel("x(cm)")
            ax_pos_err.set_ylabel("y(cm)")
            ax_pos_err.set_aspect('equal',adjustable='box')
            ax_pos_err.legend() 
            plt.show(block=False)
            plt.pause(1)
            plt.close()


            # accuracy and precision
            xdata = range(traj.accuracy.shape[0] + 1)
            xlabels = ["wpt" + str(i) for i in xdata[:-1]] + ["avg"]
            fig_ap, axs_ap = plt.subplots(3, 1, sharex=True, figsize=(6, 10))
            axs_ap[0].plot(xdata, np.append(traj.accuracy[:, 0],  traj.get_avg_accuracy()[0])* 100.0, marker="*") # m to cm
            axs_ap[1].plot(xdata, np.append(traj.precision[:, 0], traj.get_avg_precision()[0]) * 100.0, marker="*") # m to cm
            axs_ap[2].plot(xdata, np.append(traj.success_rate, traj.get_avg_success_rate()) * 100.0, marker="*")
            axs_ap[0].set_xticks(xdata)
            axs_ap[0].set_xticklabels(xlabels)
            axs_ap[0].set_title(f"{method.name}")
            axs_ap[0].set_ylabel("accuracy (cm)")
            axs_ap[1].set_ylabel("precision (cm)")
            axs_ap[2].set_ylabel("success rate (%)")
            axs_ap[2].set_ylim([-5.0, 108.0])
            if self._params["save_figs"]:
                prefix = self._result_prefix / method.name / pfile
                fig_pos_err.savefig(fname=prefix / "wpt_position_errors.png")
                fig_ap.savefig(fname=prefix / "wpt_accuracy_precision.png")

            plt.show(block=False)
            plt.pause(1)
            plt.close()

            fig = plt.figure()
            ax = plt.gca()
            ax.plot(traj.accuracy[:, 0] * 100.0, traj.precision[:, 0] * 100.0, marker="o", linestyle="None")
            max_y_bound = max(np.max(traj.accuracy[:, 0] * 100.0), np.max(traj.precision[:, 0] * 100.0))
            max_y_bound += max_y_bound * 0.1
            ax.set_xlabel("accuracy (cm)")
            ax.set_ylabel("precision (cm)")
            ax.set_xlim([-0.1, max_y_bound])
            ax.set_ylim([-0.1, max_y_bound])
            ax.set_aspect('equal',adjustable='box')
            if self._params["save_figs"]:
                prefix = self._result_prefix / method.name / pfile
                fig.savefig(fname=prefix / "accuracy_precision.png")

            plt.show(block=False)
            plt.pause(1)
            plt.close()

    def __plot_all_methods(self, methods: List[MethodBase]):
        indexed_figs = dict()
        indexed_wap_figs = dict()
        indexed_ap_figs = dict()
        indexed_max_abs_errs = dict()
        for method_index, method in enumerate(methods):
            for pfile, traj in method.traj_dict.items():          
                if pfile not in indexed_figs.keys():
                    indexed_max_abs_errs[pfile] = 0.0
                    fig = plt.figure()
                    fig.add_subplot(111)
                    indexed_figs[pfile] = fig
                ax = indexed_figs[pfile].get_axes()[0]
                all_wpts_errs = traj.all_wpts_errs * 100.0 # m to cm
                indexed_max_abs_errs[pfile] = max(indexed_max_abs_errs[pfile], np.max(np.abs(all_wpts_errs[:, :2])))
                ax.plot(all_wpts_errs[:, 0], all_wpts_errs[:, 1], color=Visualization.COLORS[method_index], marker=Visualization.MARKERS[method_index], linestyle="None", markersize=5, label=method.name)
                hull = ConvexHull(traj.all_wpts_errs[:, :2])
                for simplex in hull.simplices:
                    ax.plot(all_wpts_errs[simplex, 0], all_wpts_errs[simplex, 1], color=Visualization.COLORS[method_index], linestyle="dashdot", markersize=0.5)
                # Plot navigation control tolerance
                self.__plot_navigation_control_tolerance(ax, plot_label=(method_index == 0))
                plt.close(indexed_figs[pfile])


                # accuracy and precision
                if pfile not in indexed_wap_figs.keys():
                    indexed_wap_figs[pfile]  = plt.subplots(3, 1, sharex=True, figsize=(6, 10))[0]
                axs_ap = indexed_wap_figs[pfile].get_axes()
                xdata = range(traj.accuracy.shape[0] + 1)
                axs_ap[0].plot(xdata, np.append(traj.accuracy[:, 0],  traj.get_avg_accuracy()[0]) * 100.0, color=Visualization.COLORS[method_index], marker=Visualization.MARKERS[method_index], label=method.name) # m to cm
                axs_ap[1].plot(xdata, np.append(traj.precision[:, 0],  traj.get_avg_precision()[0]) * 100.0, color=Visualization.COLORS[method_index], marker=Visualization.MARKERS[method_index], label=method.name) # m to cm
                axs_ap[2].plot(xdata, np.append(traj.success_rate,  traj.get_avg_success_rate()) * 100.0, color=Visualization.COLORS[method_index],    marker=Visualization.MARKERS[method_index], label=method.name)
                plt.close(indexed_wap_figs[pfile])

                if pfile not in indexed_ap_figs.keys():
                    indexed_ap_figs[pfile] = plt.figure()
                    indexed_ap_figs[pfile].add_subplot(111)
                ax = indexed_ap_figs[pfile].get_axes()[0]
                ax.plot(traj.accuracy[:, 0] * 100.0, traj.precision[:, 0] * 100.0, color=Visualization.COLORS[method_index], marker=Visualization.MARKERS[method_index], linestyle="None", label=method.name)
                plt.close(indexed_ap_figs[pfile])

        for pfile, fig in indexed_figs.items():
            ax = fig.get_axes()[0]
            max_abs_err = indexed_max_abs_errs[pfile]
            max_abs_err += max_abs_err * 0.1
            ax.set_aspect('equal',adjustable='box')
            # ax.set_xlim([-max_abs_err, max_abs_err])
            # ax.set_ylim([-max_abs_err, max_abs_err])
            ax.set_xlim([-100.0, 100.0])
            ax.set_ylim([-100.0, 100.0])
            ax.set_title("Waypoint navigation position errors")
            ax.set_xlabel("x(cm)")
            ax.set_ylabel("y(cm)")
            ax.legend()
            if self._params["save_figs"]:
                fig.savefig(fname=self._result_prefix / f"{pfile}_wpt_position_errors.png")

            plt.show(block=False)
            plt.pause(1)
            plt.close(fig)


        for pfile, fig in indexed_wap_figs.items():
            xdata = range(methods[0].traj_dict[pfile].accuracy.shape[0] + 1)
            xlabels = ["wpt" + str(i) for i in xdata[:-1]] + ["avg"]
            axs = fig.get_axes()
            axs[0].set_xticks(xdata)
            axs[0].set_xticklabels(xlabels)
            axs[0].set_ylabel("accuracy (cm)")
            axs[1].set_ylabel("precision (cm)")
            axs[2].set_ylabel("success rate (%)")
            axs[2].set_ylim([-5.0, 105.0])
            axs[0].legend()
            axs[1].legend()
            axs[2].legend()
            if self._params["save_figs"]:
                fig.savefig(fname=self._result_prefix / f"{pfile}_wpt_accuracy_precision.png")

            plt.show(block=False)
            plt.pause(1)
            plt.close(fig)


        for pfile, fig in indexed_ap_figs.items():
            ax = fig.get_axes()[0]
            ax.set_xlabel("accuracy (cm)")
            ax.set_ylabel("precision (cm)")
            ax.set_xlim([0.0, 100.0])
            ax.set_ylim([0.0, 100.0])
            ax.set_aspect('equal',adjustable='box')
            ax.legend()
            if self._params["save_figs"]:
                fig.savefig(fname=self._result_prefix / f"{pfile}_accuracy_precision.png")
            plt.show(block=False)
            plt.pause(1)
            plt.close(fig)
    
    def __plot_navigation_position_errors(self, ax, wpts_errs: np.ndarray):
        max_abs_err = 0.0
        for wpt_index in range(wpts_errs.shape[0]):
            xy_errs = wpts_errs[wpt_index, :2, :].transpose()
            xy_errs = xy_errs[~np.isnan(xy_errs).any(axis=1)]
            max_abs_err = max(max_abs_err, np.max(np.abs(xy_errs)))
            ax.plot(xy_errs[:, 0], xy_errs[:, 1], color=Visualization.COLORS[wpt_index], marker="o", markersize=5, linestyle="None", label=f"wpt{wpt_index}")
            if xy_errs.shape[0] < 3:
                continue
            hull = ConvexHull(xy_errs)
            for simplex in hull.simplices:
                ax.plot(xy_errs[simplex, 0], xy_errs[simplex, 1], color=Visualization.COLORS[wpt_index], linestyle="dashdot", markersize=0.5)
        return max_abs_err

    def __plot_navigation_control_tolerance(self, ax, plot_label = True):
        theta = np.linspace(0, 2.0 * np.pi, 150)
        radius = 0.1 * 100.0 # m to cm
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        label = "control tolerance" if plot_label else None
        ax.plot(x, y, linestyle="dashdot", label=label, color="r", markersize=1)
