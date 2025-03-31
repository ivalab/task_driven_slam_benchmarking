#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file gg_case_study.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-07-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""

from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd
import seaborn as sns
from matplotlib import pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import (
    inset_axes,
    mark_inset,
    zoomed_inset_axes,
)
from scipy.spatial import ConvexHull

from cl_sim.utils.path_definitions import CONFIG_PATH
from cl_sim.utils.utils import RobotNavigationData, load_params
from cl_common.slam_methods import METHODS_VISUALS
from cl_common.utils import (
    color_violin_plot_stats_lines,
    draw_circles,
    draw_grids,
    draw_x_eq_y,
    draw_wpt_orientation_errs,
    draw_wpt_position_errs,
    save_figs,
)
from cl_common import matplotlib_config


class VisTaskDrivenResult:
    """_summary_"""

    def __init__(self, params):
        self._params = params
        print(self._params)
        self._modes = {"slam": "w/o map", "localization": "w/ map"}
        self._scales = [100.0, np.rad2deg(1.0)]
        self._result_prefix = Path(self._params["result_dir"]).resolve()
        self._grid_xy = self._params["grid_xy"]
        self._grid_theta = self._params["grid_theta"]
        self._grids = [30, 5]  # [self._grid_xy, self._grid_theta]
        self._grid_res = [10, 5]  # cm & degree
        self._subplot_labels = ["Position", "Orientation"]

    def run(self):
        """_summary_"""

        all_results = {}
        for mode_name, mode_label in self._modes.items():
            methods_results = self.__load_results(mode_name, mode_label)
            if len(methods_results) < 1:
                continue
            # self.__plot_acc_pre(methods_results)
            # self.__plot_avg_acc_pre(methods_results, mode_name, mode_label)
            # self.__plot_polygon_acc_pre(methods_results)
            # self.__plot_waypoint_errors(methods_results, mode_name)
            self.__plot_cumulative_errors(methods_results, mode_name, mode_label)

            all_results[mode_label] = methods_results
        # self.__plot_cumulative_precision(all_results)

    def __load_results(self, mode_name, mode_label):
        """_summary_"""
        methods_results = {}
        for method_index, method_name in enumerate(self._params["methods"]):
            mode_dir = self._result_prefix / mode_name
            for seq_index, seq_name in enumerate(self._params["envs"]):
                seq_dir = mode_dir / seq_name / "evals"
                filename = seq_dir / f"{method_name}_eval.pkl"
                if not filename.is_file():
                    continue
                result = RobotNavigationData(method_name)
                result.load_from_file(str(filename))
                if method_name not in methods_results:
                    methods_results[method_name] = []
                methods_results[method_name].append(result)
        return methods_results

    def __plot_cumulative_errors(self, methods_results: Dict[str, List[RobotNavigationData]], mode_name, mode_label):
        """_summary_

        Args:
            methods_results (Dict[str, List[RobotNavigationData]]): _description_
        """
        fig_acc, axs_acc = plt.subplots(1, 2, sharey=True)
        fig_pre, axs_pre = plt.subplots(1, 2, sharey=True)
        all_figs = [fig_acc, fig_pre]
        all_axs = [axs_acc, axs_pre]
        max_acc_pre = [-1, -1]
        for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
            acc_arr = []
            pre_arr = []
            gt_wpts_count = 0
            for robot_nav_data in seqs_results:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
                    # print(method_name)
                    # print(experiment)
                    gt_wpts_count += len(experiment["completeness"])
                    indices = experiment["completeness"] > 0.99
                    # compl = np.sum(indices) / float(experiment["completeness"].shape[0])
                    # inv_compl = 1.0 / compl
                    single_acc = experiment["accuracy"][indices, :] * self._scales
                    single_pre = experiment["precision"][indices, :] * self._scales
                    acc_arr.append(single_acc)
                    pre_arr.append(single_pre)

            acc = np.vstack(acc_arr)
            pre = np.vstack(pre_arr)
            inv_wpts_count = 100.0 / gt_wpts_count
            metrics = ["Accuracy", "Precision"]

            for metric_index, (metric_name, metric_data) in enumerate(zip(metrics, [acc, pre])):
                # print(f"inv_wpts_count: {inv_wpts_count}")
                xdata = np.arange(0.0, 122.0, self._grid_xy / 4.0)
                ydata = np.nansum(metric_data[:, 0, None] < xdata, axis=0) * inv_wpts_count  # Position.
                all_axs[metric_index][0].plot(
                    xdata,
                    ydata,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markerfacecolor="none",
                    linestyle=METHODS_VISUALS[method_name].line,
                    markersize=7,
                )
                xdata = np.arange(0.0, 21.0, self._grid_theta / 5.0)
                ydata = np.nansum(metric_data[:, 1, None] < xdata, axis=0) * inv_wpts_count
                all_axs[metric_index][1].plot(
                    xdata,
                    ydata,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markerfacecolor="none",
                    linestyle="-",
                    markersize=7,
                    label=METHODS_VISUALS[method_name].label,
                )
                # axs[0].fill_between(xdata, ydata, alpha=0.3)  # Fill under the curve

        for fig, axs, metric_name in zip(all_figs, all_axs, metrics):
            axs[0].set_ylim([0, 102])
            axs[0].set_xlim([0, 122])
            axs[1].set_xlim([0, 21])
            yticks = np.arange(0, 102, 20)
            axs[0].set_yticks(yticks)
            xticks = np.arange(0, 122, self._grid_xy)
            axs[0].set_xticks(xticks)
            # Draw second axis.
            ax2 = axs[0].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            ax2.set_xticks(xticks)
            ax2.set_xticklabels([f"{i}" for i in range(len(xticks))])  # Example: Double the original ticks
            ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            ax2.set_xlabel("Robot Diameter", fontdict=dict(weight="bold"))
            xticks = np.arange(0, 21, self._grid_theta)
            axs[1].set_xticks(xticks)

            # Draw second axis.
            ax2 = axs[1].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            ax2.set_xticks(xticks)
            ax2.set_xticklabels([f"{i}/8" for i in range(len(xticks))])  # Example: Double the original ticks
            ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            ax2.set_xlabel("Camera FOV")

            axs[0].set_ylabel("Completeness (\%)")
            axs[0].set_xlabel(f"Position {metric_name} (cm)")
            axs[1].set_xlabel(f"Orientation {metric_name} (degree)")
            if metric_name == "Precision":
                axs[1].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
            # fig.suptitle(f"Waypoint {method_name} Cumulative Plot")
            plt.tight_layout()
            if self._params["save_figs"]:
                # plt.savefig(
                #     fname=self._result_prefix / "waypoints_cumulative_error_plot.png",
                #     dpi=fig.dpi,
                # )
                save_figs(fig, self._result_prefix / f"sim_wpt_{metric_name.lower()[:3]}_cum_{mode_name}")
            plt.show(block=False)
            plt.pause(1)
            plt.close()


def main():
    # Load params.
    params_file = CONFIG_PATH / "params.yaml"
    params = load_params(params_file)
    VisTaskDrivenResult(params).run()


if __name__ == "__main__":
    main()
