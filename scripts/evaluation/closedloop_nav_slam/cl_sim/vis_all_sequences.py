#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file vis_all_sequences.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 08-15-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

from pathlib import Path
from typing import Dict, List

from sklearn import metrics as skmetrics

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
from cl_common.slam_methods import (
    METHODS_VISUALS,
    AXIS_VISUALS,
)
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

import matplotlib

matplotlib.rcParams.update({"font.size": 12})


class VisAllSequences:
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
        self._subplot_names = ["Position", "Orientation"]
        self._metrics = ["Accuracy", "Precision"]

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
            self.__plot_waypoint_errors(methods_results, mode_name)
            # self.__plot_cumulative_errors(methods_results, mode_name, mode_label)

            all_results[mode_label] = methods_results
        # self.__plot_cumulative_precision(all_results)

    def __load_results(self, mode_name, mode_label):
        """_summary_"""
        methods_results = {}
        methods_names = self._params["methods"]
        # for method_index, method_name in enumerate(self._params["methods"]):
        for method_index, method_name in enumerate(methods_names):
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

    def __plot_acc_pre(self, methods_results: Dict[str, List[RobotNavigationData]]):
        # Plot scattered waypoints.
        # Consider only valid waypoints (waypoint that have full success).
        fig, axs = plt.subplots(1, 2, figsize=(14, 6))
        # axsins = [
        #     inset_axes(
        #         axs[0],
        #         width="50%",
        #         height="50%",
        #         loc="upper left",
        #         # bbox_to_anchor=(0.0, -0.3, 1.2, 1.2),
        #         # bbox_transform=axs[0].figure.transFigure,
        #         borderpad=2,
        #     ),
        #     inset_axes(axs[1], width="50%", height="50%", loc="upper right", borderpad=2),
        # ]
        max_acc_pre = np.zeros((2))
        for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
            enable_label: bool = True
            for robot_nav_data in seqs_results:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
                    indices = experiment["completeness"] > 0.99
                    if np.sum(indices) == 0:
                        continue
                    # completeness = np.sum(indices) / float(experiment["completeness"].shape[0]) * 100.0
                    acc = experiment["accuracy"][indices, :] * self._scales
                    pre = experiment["precision"][indices, :] * self._scales
                    label = METHODS_VISUALS[method_name].label if enable_label else ""  # Plot label once.
                    enable_label = False if enable_label else enable_label  # Disable label.

                    # Position and Orientation in that order.
                    for sp_index in range(2):
                        axs[sp_index].plot(
                            acc[:, sp_index],
                            pre[:, sp_index],
                            linestyle="none",
                            label=label,
                            color=METHODS_VISUALS[method_name].color,
                            marker=METHODS_VISUALS[method_name].marker,
                            markersize=6,
                            markerfacecolor="none",
                            markeredgewidth=1,
                        )
                        # axsins[sp_index].plot(
                        #     acc[:, sp_index],
                        #     pre[:, sp_index],
                        #     linestyle="none",
                        #     label=label,
                        #     color=METHODS_VISUALS[method_name].color,
                        #     marker=METHODS_VISUALS[method_name].marker,
                        #     markersize=6,
                        #     markerfacecolor="none",
                        #     markeredgewidth=1,
                        # )

                    # Get the max.
                    max_acc_pre = np.fmax(
                        max_acc_pre,
                        np.fmax(
                            np.nanmax(acc, axis=0),
                            np.nanmax(pre, axis=0),
                        ),
                    )

        # Plot a precision==accuracy boundary and grids.
        for sp_index in range(2):
            max_err = np.ceil(max_acc_pre[sp_index] / self._grids[sp_index]) * self._grids[sp_index]
            draw_grids(axs[sp_index], self._grids[sp_index], [0, max_err])
            draw_x_eq_y(axs[sp_index], 0.0, max_err)
            # draw_grids(axsins[sp_index], self._grids[sp_index], [0, max_err * 0.5])
            # draw_x_eq_y(axsins[sp_index], 0.0, max_err * 0.5)

        # ax.set_xlabel("Accuracy")
        # ax.set_ylabel("Precision")
        axs[0].set_aspect("equal", adjustable="box")
        axs[1].set_aspect("equal", adjustable="box")
        axs[0].set_title("Position (cm)")
        axs[1].set_title("Orientation (degree)")
        # axs[0].legend()
        axs[0].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)

        # axsins[0].set_xlim([0, grids[0]])
        # axsins[0].set_ylim([0, 20.0])
        # axsins[1].set_xlim([0, grids[1]])
        # axsins[1].set_ylim([0, grids[1] / 10.0])
        # axsins[0].grid()
        # axsins[1].grid()
        # mark_inset(axs[0], axsins[0], loc1=2, loc2=4, fc="none", ec="0.5")
        # mark_inset(axs[1], axsins[1], loc1=2, loc2=4, fc="none", ec="0.5")

        fig.supylabel("Precision")
        fig.supxlabel("Accuracy")
        plt.tight_layout()
        # ax.grid(False)

        if self._params["save_figs"]:
            fig.savefig(fname=self._result_prefix / "all_acc_pre.png", dpi=fig.dpi)

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)

    def __plot_avg_acc_pre(self, methods_results: Dict[str, List[RobotNavigationData]], mode_name, mode_label):
        # Plot scattered waypoints.
        # Consider only valid waypoints (waypoint that have full success).
        fig, axs = plt.subplots(1, 2)  # , figsize=(8, 6))
        plt.subplots_adjust(wspace=1.4)  # Increase space between rows
        # axsins = [
        #     inset_axes(
        #         axs[0],
        #         width="50%",
        #         height="50%",
        #         loc="upper right",
        #         # bbox_to_anchor=(0.0, -0.3, 1.2, 1.2),
        #         # bbox_transform=axs[0].figure.transFigure,
        #         borderpad=2,
        #     ),
        #     inset_axes(axs[1], width="50%", height="50%", loc="upper right", borderpad=2),
        # ]
        max_acc_pre = np.zeros((2))
        methods_completeness = []
        methods_colors = []
        methods_names = []
        for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
            acc_arr = []
            pre_arr = []
            weighted_acc_arr = []
            weighted_pre_arr = []
            gt_wpts_count = 0
            act_wpts_count = 0
            for robot_nav_data in seqs_results:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
                    indices = experiment["completeness"] > 0.99
                    completeness = np.sum(indices) / float(experiment["completeness"].shape[0])
                    gt_wpts_count += experiment["completeness"].shape[0]
                    act_wpts_count += np.sum(indices)
                    if np.sum(indices) == 0:
                        continue
                    acc = experiment["accuracy"][indices, :] * self._scales
                    pre = experiment["precision"][indices, :] * self._scales
                    acc_arr.append(acc)
                    pre_arr.append(pre)
                    weighted_acc_arr.append(acc / completeness)
                    weighted_pre_arr.append(pre / completeness)
            # Take mean.
            completeness = float(act_wpts_count) / gt_wpts_count
            weight = 1.0
            methods_completeness.append(completeness)
            methods_colors.append(METHODS_VISUALS[method_name].color)
            methods_names.append(method_name)
            if completeness >= 0.999:
                weight = 1.0
            elif completeness >= 0.499:
                # weight = 3.0 / 4.0
                # elif weight >= 0.349:
                weight = 2.0 / 3.0
            else:
                weight = 1.0 / 3.0
            acc_mean = np.mean(np.vstack(acc_arr), axis=0)
            pre_mean = np.mean(np.vstack(pre_arr), axis=0)
            weighted_acc_mean = np.mean(np.vstack(weighted_acc_arr), axis=0)
            weighted_pre_mean = np.mean(np.vstack(weighted_pre_arr), axis=0)
            # weighted_acc_mean = acc_mean * weight
            # weighted_pre_mean = pre_mean * weight

            # Record the max value for plotting.
            # max_acc_pre = np.fmax(
            #     max_acc_pre,
            #     np.fmax(
            #         np.fmax(acc_mean, pre_mean),
            #         np.fmax(weighted_acc_mean, weighted_pre_mean),
            #     ),
            # )
            max_acc_pre = np.fmax(
                max_acc_pre,
                np.fmax(acc_mean, pre_mean),
            )
            # Position and Orientation in that order.
            for sp_index in range(2):
                method_label = METHODS_VISUALS[method_name].label
                compl_str = "({:.1f}\%)".format(completeness * 100.0)
                # compl_str = "    "
                label = METHODS_VISUALS[method_name].label
                # label = f"{method_label: <12} {compl_str: >5}"
                latex_label = label
                if weight > 0.99:
                    axs[sp_index].plot(
                        acc_mean[sp_index],
                        pre_mean[sp_index],
                        linestyle="none",
                        color=METHODS_VISUALS[method_name].color,
                        marker=METHODS_VISUALS[method_name].marker,
                        markersize=10,
                        label=rf"{label}",
                        # markerfacecolor="none",
                        # markeredgewidth=5 * weight,
                        # alpha=0.3,
                    )
                # axs[sp_index].plot(
                #     [acc_mean[sp_index], weighted_acc_mean[sp_index]],
                #     [pre_mean[sp_index], weighted_pre_mean[sp_index]],
                #     linestyle="dashdot",
                #     # label=METHODS_VISUALS[method_name].label,
                #     color=METHODS_VISUALS[method_name].color,
                #     # marker=METHODS_VISUALS[method_name].marker,
                #     # markersize=7,
                #     # markerfacecolor="none",
                #     # markeredgewidth=1,
                #     linewidth=0.5,
                # )
                else:
                    axs[sp_index].plot(
                        acc_mean[sp_index],
                        pre_mean[sp_index],
                        linestyle="none",
                        label=latex_label,
                        color=METHODS_VISUALS[method_name].color,
                        marker=METHODS_VISUALS[method_name].marker,
                        markersize=10,
                        markerfacecolor="none",
                        markeredgewidth=1,
                    )

                # axsins[sp_index].plot(
                #     acc_mean[sp_index],
                #     pre_mean[sp_index],
                #     linestyle="none",
                #     label=METHODS_VISUALS[method_name].label,
                #     color=METHODS_VISUALS[method_name].color,
                #     marker=METHODS_VISUALS[method_name].marker,
                #     markersize=6,
                #     # markerfacecolor="none",
                #     markeredgewidth=1,
                # )

                # axsins[sp_index].plot(
                #     weighted_acc_mean[sp_index],
                #     weighted_pre_mean[sp_index],
                #     linestyle="none",
                #     # label=METHODS_VISUALS[method_name].label,
                #     color=METHODS_VISUALS[method_name].color,
                #     marker=METHODS_VISUALS[method_name].marker,
                #     markersize=9 * weight,
                #     # markerfacecolor="none",
                #     markeredgewidth=1,
                # )
                # axsins[sp_index].plot(
                #     [acc_mean[sp_index], weighted_acc_mean[sp_index]],
                #     [pre_mean[sp_index], weighted_pre_mean[sp_index]],
                #     linestyle="dashdot",
                #     # label=METHODS_VISUALS[method_name].label,
                #     color=METHODS_VISUALS[method_name].color,
                #     # marker=METHODS_VISUALS[method_name].marker,
                #     # markersize=7,
                #     # markerfacecolor="none",
                #     # markeredgewidth=1,
                #     linewidth=0.5,
                # )

        # Add a vertical line between subplots
        # plt.plot(
        #     [0.5, 0.5],
        #     [0.15, 0.8],
        #     linestyle="dashed",
        #     color="black",
        #     lw=1,
        #     transform=plt.gcf().transFigure,
        #     clip_on=False,
        # )

        # Plot a precision==accuracy boundary and grids.
        td_steps = [30.0, 5.0]
        td_ticks = ["", "/16"]
        for sp_index, ax_name in enumerate(self._subplot_names):
            max_err = np.ceil(max_acc_pre[sp_index] / self._grids[sp_index]) * self._grids[sp_index]
            draw_grids(axs[sp_index], self._grids[sp_index], [0, max_err])
            draw_x_eq_y(axs[sp_index], 0.0, max_err)
            # draw_grids(axsins[sp_index], self._grids[sp_index], [0, max_err])
            # draw_x_eq_y(axsins[sp_index], 0.0, max_err)
            xaxis_visual = AXIS_VISUALS[f"{ax_name} Accuracy"]
            yaxis_visual = AXIS_VISUALS[f"{ax_name} Precision"]
            axs[sp_index].set_xlabel(f"{xaxis_visual.symbol} ({xaxis_visual.unit})")
            axs[sp_index].set_ylabel(f"{yaxis_visual.symbol} ({yaxis_visual.unit})")

            self.__draw_second_axis(
                axs[sp_index],
                td_steps[sp_index],
                td_ticks[sp_index],
                f"{xaxis_visual.symbol} ({xaxis_visual.td_unit})",
                f"{yaxis_visual.symbol} ({yaxis_visual.td_unit})",
            )

        # Draw second axis.
        # self.__draw_second_axis(axs[0], 30.0, "", "Robot Diameter")
        # self.__draw_second_axis(axs[1], 5.0, "/16", "Camera FOV")

        # ax.set_xlabel("Accuracy")
        # ax.set_ylabel("Precision")
        axs[0].set_aspect("equal", adjustable="box")
        axs[1].set_aspect("equal", adjustable="box")
        axs[0].set_title("Position", fontweight="black", y=-0.35, pad=-14)
        axs[1].set_title("Orientation", fontweight="black", y=-0.35, pad=-14)
        # print(methods_completeness)
        methods_order = np.argsort(methods_completeness)[::-1]
        # val = methods_order[0]
        # methods_order[0] = methods_order[1]
        # methods_order[1] = val
        # axs[1].legend(loc="upper right")
        handles, labels = axs[1].get_legend_handles_labels()
        handles = [handles[idx] for idx in methods_order]
        labels = [labels[idx] for idx in methods_order]
        # compl_str = "({:.1f}\%)".format(completeness * 100.0)
        fig.legend(
            handles,
            labels,
            # [f"{labels[idx]: <15} ({(methods_completeness[idx] * 100.0): >5.1f}\%)" for idx in methods_order],
            bbox_to_anchor=(0.5, 0.92),  # (1.42, 1), # Upper
            # bbox_to_anchor=(0.5, 0.05),  # (1.42, 1), # Lower
            # bbox_to_anchor=(0.53, 0.5),  # Left
            ncol=3,
            # fancybox=True,
            # shadow=True,
            loc="upper center",
            # loc="center right",
            # borderaxespad=0,
            # fontsize="8",
            # columnspacing=4.8,
        )

        # axsins[0].set_xlim([0, grids[0]])
        # axsins[0].set_ylim([0, 20.0])
        # axsins[0].grid()
        # axsins[1].grid()
        # mark_inset(axs[0], axsins[0], loc1=2, loc2=4, fc="none", ec="red", lw=0.3)
        # mark_inset(axs[1], axsins[1], loc1=2, loc2=4, fc="none", ec="red", lw=0.3)
        # for i in range(2):
        # axsins[i].set_xlim([0, self._grids[i]])
        # axsins[i].set_ylim([0, self._grids[i]])

        # fig.supylabel("Precision")
        # fig.supxlabel("Accuracy")
        # plt.tight_layout()
        # ax.grid(False)

        if self._params["save_figs"]:
            # fig.savefig(fname=self._result_prefix / "all_avg_acc_pre.png", dpi=fig.dpi)
            save_figs(fig, self._result_prefix / f"sim_pre_acc_{mode_name}")
        plt.show()
        # plt.show(block=False)
        # plt.pause(1)
        # plt.close(fig)

        # Add a barchart
        # methods_order = methods_order[::-1]
        fig = plt.figure(figsize=(1, 4))
        positions = []
        for i, method_idx in enumerate(methods_order):
            pos = -i
            completeness = methods_completeness[method_idx]
            method_name = methods_names[method_idx]
            ypos = pos * 2.0
            plt.axhline(
                y=ypos,
                xmin=0,
                xmax=completeness,
                # color=METHODS_VISUALS[method_name].color,
                color="tab:blue",
                # linestyle="dotted",
                linestyle="solid",
                linewidth=4,
            )
            plt.text(1.1, ypos, rf"\textbf{{{completeness * 100.0:.1f}}}", va="center", ha="left", fontsize=20)
            positions.append(ypos)

        # plt.barh(
        #     [labels[idx] for idx in methods_order],
        #     [methods_completeness[idx] for idx in methods_order],
        #     color=[methods_colors[idx] for idx in methods_order],
        # )

        # Add numerical values on top of the bars
        # for i, value in enumerate([methods_completeness[idx] for idx in methods_order]):
        #     plt.axhline(y=-i, color=methods_colors[idx])
        #     plt.text(1.05, -i, f"{value * 100.0:.1f}", va="center", ha="left")

        ax = plt.gca()  # Get the current axis
        ax.spines["top"].set_visible(False)  # Hide the top spine
        ax.spines["right"].set_visible(False)  # Hide the right spine
        ax.spines["bottom"].set_visible(False)  # Hide the bottom spine
        ax.spines["left"].set_visible(False)  # Hide the left spine
        ax.set_xticks([])  # Hide x-axis ticks
        ax.set_yticks(positions)  # Hide y-axis ticks
        ax.set_yticklabels(labels, fontsize=15)  # Hide y-axis ticks

        # Add labels and title (optional, since the axis is hidden)
        plt.xlabel("C (\%)")
        # plt.ylabel("")

        if self._params["save_figs"]:
            # fig.savefig(fname=self._result_prefix / "all_avg_acc_pre.png", dpi=fig.dpi)
            save_figs(fig, self._result_prefix / f"sim_pre_acc_{mode_name}_legend_v1")

        # Add labels and title
        # plt.xlabel("Values")
        # plt.ylabel("Categories")
        # plt.title("Horizontal Bar Plot with Different Colors per Category")
        plt.show()

    def __draw_second_axis(self, ax, step, unit_label, xaxis_label, yaxis_label):
        # Draw second axis.
        xlims = ax.get_xlim()
        # xlims = [-29, 30]
        xticks = np.arange(xlims[0], xlims[1] * 1.02, step)
        # ax_x = ax.secondary_xaxis("bottom")
        ax_x = ax.secondary_xaxis("top")
        ax_x.set_xticks(xticks)
        ax_x.set_xticklabels([f"{i}{unit_label}" for i in range(len(xticks))])
        # ax_x.spines["bottom"].set_position(("outward", 45))
        ax_x.set_xlabel(xaxis_label)

        # ylims = ax.get_ylim()
        if len(yaxis_label) > 0:
            ylims = xlims
            yticks = np.arange(ylims[0], ylims[1] * 1.02, step)
            ax_y = ax.secondary_yaxis("right")
            ax_y.set_yticks(yticks)
            ax_y.set_yticklabels([f"{i}{unit_label}" for i in range(len(yticks))])
            ax_y.set_ylabel(yaxis_label)

    def __plot_polygon_acc_pre(self, methods_results: Dict[str, List[RobotNavigationData]]):
        # Plot scattered waypoints.
        # Consider only valid waypoints (waypoint that have full success).
        fig, axs = plt.subplots(1, 2, figsize=(14, 6))
        # axsins = [
        #     inset_axes(
        #         axs[0],
        #         width="50%",
        #         height="50%",
        #         loc="upper right",
        #         # bbox_to_anchor=(0.0, -0.3, 1.2, 1.2),
        #         # bbox_transform=axs[0].figure.transFigure,
        #         borderpad=2,
        #     ),
        #     inset_axes(axs[1], width="50%", height="50%", loc="upper right", borderpad=2),
        # ]
        max_acc_pre = np.zeros((2))
        for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
            acc_list = []
            pre_list = []
            for robot_nav_data in seqs_results:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
                    indices = experiment["completeness"] > 0.99
                    if np.sum(indices) == 0:
                        continue
                    # completeness = np.sum(indices) / float(experiment["completeness"].shape[0])
                    acc = experiment["accuracy"][indices, :] * self._scales
                    pre = experiment["precision"][indices, :] * self._scales
                    acc_list.append(acc)
                    pre_list.append(pre)

                    # Record the max for plotting.
                    max_acc_pre = np.fmax(
                        max_acc_pre,
                        np.fmax(
                            np.nanmax(acc, axis=0),
                            np.nanmax(pre, axis=0),
                        ),
                    )
            # Take mean.
            acc_arr = np.vstack(acc_list)
            pre_arr = np.vstack(pre_list)
            acc_mean = np.mean(acc_arr, axis=0)
            pre_mean = np.mean(pre_arr, axis=0)

            # Position and Orientation in that order.
            for sp_index in range(2):
                axs[sp_index].plot(
                    acc_mean[sp_index],
                    pre_mean[sp_index],
                    linestyle="none",
                    label=METHODS_VISUALS[method_name].label,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markersize=10,
                    markerfacecolor="none",
                    markeredgewidth=1,
                )

                # Fit a polygon.
                fitted_data = np.column_stack([acc_arr[:, sp_index], pre_arr[:, sp_index]])
                hull = ConvexHull(fitted_data)
                for simplex in hull.simplices:
                    axs[sp_index].plot(
                        fitted_data[simplex, 0],
                        fitted_data[simplex, 1],
                        color=METHODS_VISUALS[method_name].color,
                        linestyle="dashdot",
                        linewidth=1.0,
                        markeredgecolor="none",
                    )

        # Plot a precision==accuracy boundary and grids.
        for sp_index in range(2):
            max_err = np.ceil(max_acc_pre[sp_index] / self._grids[sp_index]) * self._grids[sp_index]
            draw_grids(axs[sp_index], self._grids[sp_index], [0, max_err])
            draw_x_eq_y(axs[sp_index], 0.0, max_err)
            # draw_grids(axsins[sp_index], self._grids[sp_index], [0, max_err])
            # draw_x_eq_y(axsins[sp_index], 0.0, max_err)

        # ax.set_xlabel("Accuracy")
        # ax.set_ylabel("Precision")
        axs[0].set_aspect("equal", adjustable="box")
        axs[1].set_aspect("equal", adjustable="box")
        axs[0].set_title("Position")
        axs[1].set_title("Orientation")
        # axs[0].legend()
        axs[0].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)

        # axsins[0].set_xlim([0, grids[0]])
        # axsins[0].set_ylim([0, 20.0])
        # axsins[0].grid()
        # axsins[1].grid()
        # mark_inset(axs[0], axsins[0], loc1=2, loc2=4, fc="none", ec="red", lw=0.3)
        # mark_inset(axs[1], axsins[1], loc1=2, loc2=4, fc="none", ec="red", lw=0.3)
        # for i in range(2):
        #     axsins[i].set_xlim([0, self._grids[i]])
        #     axsins[i].set_ylim([0, self._grids[i]])

        fig.supylabel("Precision (cm)")
        fig.supxlabel("Accuracy (cm)")
        plt.tight_layout()
        # ax.grid(False)

        if self._params["save_figs"]:
            fig.savefig(fname=self._result_prefix / "all_polygon_acc_pre.png", dpi=fig.dpi)

        plt.show(block=False)
        plt.pause(1)
        plt.close(fig)

    def __plot_waypoint_errors(self, methods_results: Dict[str, List[RobotNavigationData]], mode_name):
        # if mode_name == "slam":
        # ordered_methods = ["gfgg", "orb3_new", "msckf", "dsol", "slam_toolbox", "hector_slam", "fast_lio2", "liorf"]
        # else:
        ordered_methods = list(methods_results.keys())
        num_methods = len(ordered_methods)
        assert num_methods > 0
        num_rows = 1 if num_methods < 5 else 2
        num_cols = int(np.ceil(float(num_methods) / num_rows))
        assert num_rows > 0
        assert num_cols > 0
        fig, axs = plt.subplots(
            2, 2, sharex=True, sharey=True, figsize=(8, 8)
        )  # , figsize=(14, 10))  # , sharex=True, sharey=True)
        if num_methods == 1:
            axs = np.array([axs])  # Ensure axes is iterable if there's only one subplot
        axs = axs.flatten()
        assert len(axs) >= num_methods, print(len(axs), num_methods)
        # Loop over each method.
        max_abs_xy_err = 0.0
        # for ax, (method_name, seqs_results) in zip(axs[: num_methods + 1], methods_results.items()):
        for method_idx, method_name in enumerate(ordered_methods):
            ax = axs[method_idx]
            for robot_nav_data in methods_results[method_name]:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
                    indexed_wpts_errs = {}
                    round_num = 0
                    for single_round_data in experiment["rounds"]:
                        nav_data = single_round_data["nav_data"]
                        nav_err = single_round_data["nav_err"]
                        if nav_data is None or nav_err is None:
                            continue
                        for index, err in zip(nav_data.wpts_indices, nav_err.wpts_errs):
                            if index not in indexed_wpts_errs:
                                indexed_wpts_errs[index] = []
                            indexed_wpts_errs[index].append(err)
                        round_num += 1
                        if round_num >= 5:
                            break
                    # Start plotting
                    for wpt_index, wpt_errs in indexed_wpts_errs.items():
                        xy_err = draw_wpt_position_errs(
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
            # ax.set_xlabel("X (cm)")
            # ax.set_ylabel("Y (cm)")
            ax.set_title(METHODS_VISUALS[method_name].label)
            # ax.set_title("Waypoint Position Error")
            # ax.legend()
        # max_abs_xy_err *= 1.56
        axs[0].set_xlim([-max_abs_xy_err, max_abs_xy_err])
        axs[0].set_ylim([-max_abs_xy_err, max_abs_xy_err])
        # axs[0].set_xticks(np.arange())
        # axs[0].set_yscale("log")
        # axs[0].set_xscale("log")
        if False and mode_name == "slam":
            for ax in axs[[0, 1, 4, 5]]:
                draw_circles(ax, self._grids[0] / 2.0, 150, enable_xy_ticks=True)
            for ax in axs[[2, 3, 6, 7]]:
                draw_circles(ax, self._grids[0] / 2.0, 195, enable_xy_ticks=True)
        else:
            for ax in axs:
                draw_circles(ax, self._grids[0] / 2.0, max_abs_xy_err, enable_xy_ticks=True)

        # xaxis_visual = AXIS_VISUALS["Position Accuracy"]
        # # yaxis_visual = AXIS_VISUALS["Position Precision"]
        # ax.set_xlabel(f"{xaxis_visual.symbol} ({xaxis_visual.unit})")
        # self.__draw_second_axis(
        #     ax,
        #     self._grids[0],
        #     "",
        #     f"{xaxis_visual.symbol} ({xaxis_visual.td_unit})",
        #     "",
        # )

        # if mode_name == "slam":
        #     for ax in axs[[0, 1, 4, 5]]:
        #         ax.set_xlim([-55, 55])
        #         ax.set_ylim([-55, 55])
        #         ax.set_xticks([])
        #         ax.set_yticks([])

        #     for ax in axs[[2, 3, 6, 7]]:
        #         ax.set_xlim([-195, 195])
        #         ax.set_ylim([-195, 195])
        #         ax.set_xticks([])
        #         ax.set_yticks([])

        # axs[0].set_xticks([])
        # axs[0].set_yticks([])

        fig.supxlabel("X (cm)")
        fig.supylabel("Y (cm)")
        fig.suptitle("Waypoint Position Error")
        fig.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._result_prefix / f"waypoints_positions_errors_{mode_name}")
        # plt.show(block=False)
        # plt.pause(1)
        # plt.close()
        plt.show()

    def __plot_cumulative_errors(self, methods_results: Dict[str, List[RobotNavigationData]], mode_name, mode_label):
        """_summary_

        Args:
            methods_results (Dict[str, List[RobotNavigationData]]): _description_
        """
        fig_acc, axs_acc = plt.subplots(1, 2, sharey=True)
        fig_pre, axs_pre = plt.subplots(1, 2, sharey=True)
        all_figs = [fig_acc, fig_pre]
        all_axs = [axs_acc, axs_pre]
        all_cum_areas = [[], []]
        # max_acc_pre = [-1, -1]
        for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
            acc_arr = []
            pre_arr = []
            gt_wpts_count = 0
            for robot_nav_data in seqs_results:
                # Loop over each experiment (path).
                for experiment in robot_nav_data.data:
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

            for metric_index, (metric_name, metric_data) in enumerate(zip(self._metrics, [acc, pre])):
                # print(f"inv_wpts_count: {inv_wpts_count}")
                # Position.
                xdata = np.arange(0.0, 122.0, self._grid_xy / 4.0)
                ydata = np.nansum(metric_data[:, 0, None] < xdata, axis=0) * inv_wpts_count
                inv_area = 1.0 / ((max(xdata) - min(xdata)) * 100.0)
                cum_perc = skmetrics.auc(xdata, ydata) * inv_area
                all_cum_areas[metric_index].append(cum_perc)
                all_axs[metric_index][0].plot(
                    xdata,
                    ydata,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markerfacecolor="none",
                    linestyle=METHODS_VISUALS[method_name].line,
                    markersize=7,
                    label=f"{METHODS_VISUALS[method_name].label} ({cum_perc:.2f})",
                )
                # Orientation.
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
                )
                # axs[0].fill_between(xdata, ydata, alpha=0.3)  # Fill under the curve

        # Sort the handles and labels based on the labels
        for fig, axs, areas in zip(all_figs, all_axs, all_cum_areas):
            handles, labels = axs[0].get_legend_handles_labels()  # Only apply to position (index = 0)
            sorted_indices = sorted(range(len(areas)), key=lambda k: -areas[k])
            handles = [handles[i] for i in sorted_indices]
            labels = [labels[i] for i in sorted_indices]
            fig.legend(
                handles,
                labels,
                bbox_to_anchor=(0.5, 0.98),  # (1.42, 1), # Upper
                # bbox_to_anchor=(0.5, 0.05),  # (1.42, 1), # Lower
                ncol=3,
                fancybox=True,
                shadow=True,
                loc="lower center",
                # borderaxespad=0,
                fontsize="11",
            )

        td_ticks = ["", "/16"]
        td_lines = [30, 5.0]
        for fig, axs, metric_name in zip(all_figs, all_axs, self._metrics):
            axs[0].set_ylim([0, 102])
            axs[0].set_xlim([0, 122])
            axs[1].set_xlim([0, 21])
            yticks = np.arange(0, 102, 20)
            axs[0].set_yticks(yticks)
            xticks = np.arange(0, 122, self._grid_xy)
            axs[0].set_xticks(xticks)
            axs[0].set_ylabel("Cumulative Percentage (\%)")

            for sp_index, (ax, ax_name) in enumerate(zip(axs, self._subplot_names)):
                xaxis_visual = AXIS_VISUALS[f"{ax_name} {metric_name}"]
                a1_xlabel = f"{xaxis_visual.symbol} ({xaxis_visual.unit})"
                a2_xlabel = f"{xaxis_visual.symbol} ({xaxis_visual.td_unit})"
                axs[sp_index].set_xlabel(a1_xlabel)
                self.__draw_second_axis(
                    ax,
                    self._grids[sp_index],
                    td_ticks[sp_index],
                    a2_xlabel,
                    "",
                )
                ax.plot([td_lines[sp_index], td_lines[sp_index]], [0, 100], color="grey", linestyle="--", linewidth=0.5)
                ax.set_title(ax_name, y=-0.2, pad=-14)

            # Draw second axis.
            # ax2 = axs[0].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            # ax2 = axs[0].secondary_xaxis("top")  # Create the second X-axis to the bottom
            # ax2.set_xticks(xticks)
            # ax2.set_xticklabels([f"{i}" for i in range(len(xticks))])  # Example: Double the original ticks
            # ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            # ax2.set_xlabel("Robot Diameter", fontdict=dict(weight="bold"))
            # xticks = np.arange(0, 21, self._grid_theta)
            # axs[1].set_xticks(xticks)

            # Draw second axis.
            # ax2 = axs[1].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            # ax2.set_xticks(xticks)
            # ax2.set_xticklabels([f"{i}/8" for i in range(len(xticks))])  # Example: Double the original ticks
            # ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            # ax2.set_xlabel("Camera FOV")

            # axs[0].set_title(f"Position {metric_name}",  y=-0.2, pad=-14)
            # axs[0].set_ylabel("Percentage of Waypoints")
            # axs[0].set_xlabel("Centimeter")
            # axs[1].set_title(f"Orientation {metric_name}" y=-0.2, pad=-14)
            # axs[1].set_xlabel("Degree")
            # if metric_name == "Precision":
            #     axs[1].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
            # fig.suptitle(f"Waypoint {method_name} Cumulative Plot")
        # fig_acc.set_tight_layout(True)
        # fig_pre.set_tight_layout(True)
        if self._params["save_figs"]:
            # plt.savefig(
            #     fname=self._result_prefix / "waypoints_cumulative_error_plot.png",
            #     dpi=fig.dpi,
            # )
            save_figs(fig_acc, self._result_prefix / f"sim_wpt_acc_cum_{mode_name}")
            save_figs(fig_pre, self._result_prefix / f"sim_wpt_pre_cum_{mode_name}")
        # plt.show()
        plt.show(block=False)
        plt.pause(1)
        plt.close()

    def __plot_cumulative_precision(self, all_results):
        """_summary_

        Args:
            methods_results (Dict[str, List[RobotNavigationData]]): _description_
        """
        fig, axs = plt.subplots(1, 2, sharey=True)  # , figsize=(8, 6))
        mode_xmax = [122, 122]
        # cum_areas = [[], []]
        methods_cum_areas = {}
        # methods_labels = {}
        for mode_index, (mode_label, methods_results) in enumerate(all_results.items()):
            for method_index, (method_name, seqs_results) in enumerate(methods_results.items()):
                if method_name not in methods_cum_areas:
                    methods_cum_areas[method_name] = [0.0, 0.0]
                pre_arr = []
                gt_wpts_count = 0
                for robot_nav_data in seqs_results:
                    # Loop over each experiment (path).
                    for experiment in robot_nav_data.data:
                        gt_wpts_count += len(experiment["completeness"])
                        indices = experiment["completeness"] > 0.99
                        # compl = np.sum(indices) / float(experiment["completeness"].shape[0])
                        # inv_compl = 1.0 / compl
                        single_pre = experiment["precision"][indices, :] * self._scales
                        pre_arr.append(single_pre)

                pre = np.vstack(pre_arr)
                inv_wpts_count = 100.0 / gt_wpts_count

                xdata = np.arange(0.0, mode_xmax[mode_index], self._grid_xy / 2.0)
                ydata = np.nansum(pre[:, 0, None] < xdata, axis=0) * inv_wpts_count  # Position.

                inv_area = 1.0 / ((max(xdata) - min(xdata)) * 100.0)
                cum_perc = skmetrics.auc(xdata, ydata) * inv_area
                methods_cum_areas[method_name][mode_index] = cum_perc

                # methods_labels[method_name][mode_index] = cum_perc
                (method_plot,) = axs[mode_index].plot(
                    xdata,
                    ydata,
                    color=METHODS_VISUALS[method_name].color,
                    marker=METHODS_VISUALS[method_name].marker,
                    markerfacecolor="none",
                    linestyle=METHODS_VISUALS[method_name].line,
                    markersize=7,
                    label=METHODS_VISUALS[method_name].label,
                )

            axs[mode_index].set_title(rf"\textbf{{{mode_label}}}", y=-0.2, pad=-14)
            axs[mode_index].set_ylim([0, 102])
            axs[mode_index].set_xlim([0, mode_xmax[mode_index]])
            yticks = np.arange(0, 102, 20)
            axs[mode_index].set_yticks(yticks)
            xticks = np.arange(0, mode_xmax[mode_index], self._grid_xy)
            axs[mode_index].set_xticks(xticks)

            axs[mode_index].plot([30, 30], [0, 100], color="grey", linestyle="--", linewidth=0.5)

            xaxis_visual = AXIS_VISUALS["Position Precision"]
            axs[mode_index].set_xlabel(f"{xaxis_visual.symbol} ({xaxis_visual.unit})")
            self.__draw_second_axis(
                axs[mode_index],
                self._grids[0],
                "",
                f"{xaxis_visual.symbol} ({xaxis_visual.td_unit})",
                "",
            )

            # # Draw second axis.
            # ax2 = axs[mode_index].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            # ax2.set_xticks(xticks)
            # ax2.set_xticklabels([f"{i}" for i in range(len(xticks))])  # Example: Double the original ticks
            # ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
            # ax2.set_xlabel("Robot Diameter")
        axs[0].set_ylabel("Cumulative Percentage (\%)")
        handles, labels = axs[0].get_legend_handles_labels()  # Only apply to position (index = 0)
        # handles = [handles[i] for i in sorted_indices]
        # labels = [labels[i] for i in sorted_indices]
        fig.legend(
            handles,
            labels,
            bbox_to_anchor=(0.7, 0.98),  # (1.42, 1), # Upper
            # bbox_to_anchor=(0.5, 0.05),  # (1.42, 1), # Lower
            ncol=5,
            fancybox=True,
            shadow=True,
            loc="lower center",
            # borderaxespad=0,
            fontsize="13",
        )

        plt.tight_layout()
        if self._params["save_figs"]:
            # plt.savefig(
            #     fname=self._result_prefix / "waypoints_cumulative_error_plot.png",
            #     dpi=fig.dpi,
            # )
            save_figs(fig, self._result_prefix / "sim_wpt_pre_cum")
        plt.show()
        # plt.pause(1)
        # plt.close()

        sorted_methods_areas = sorted(
            methods_cum_areas.items(), key=lambda x: (x[1][0] + x[1][1]), reverse=True
        )  # use w/o map area
        # fig, axs = plt.subplots(1, 2, figsize=(2, 16), sharey=True, sharex=True)
        fig = plt.figure(figsize=(1, 6))
        ax = plt.gca()
        positions = []
        yticklabels = []
        # modes = ["w/o map", "w/ map"]
        # linestyles = ["dotted", "solid"]
        linestyles = ["solid", "solid"]
        colors = ["tab:blue", "tab:orange"]
        for i, (method_name, areas) in enumerate(sorted_methods_areas):
            yticklabels.append(METHODS_VISUALS[method_name].label)
            pos = -i
            for index, area in enumerate(areas):
                ypos = (pos - index * 0.5) * 10
                if index == 0:
                    positions.append(ypos)
                if np.isnan(area) or area <= 0.0:
                    continue
                ax.axhline(
                    y=ypos,
                    xmin=0,
                    xmax=area,
                    # color=METHODS_VISUALS[method_name].color,
                    color=colors[index],
                    linestyle=linestyles[index],
                    # linestyle="dotted",
                    linewidth=3,
                    markersize=3,
                )
                ax.text(1.01, ypos, rf"\textbf{{{area:.2f}}}", va="center", ha="left", fontsize=20)
        ax.spines["top"].set_visible(False)  # Hide the top spine
        ax.spines["right"].set_visible(False)  # Hide the right spine
        ax.spines["bottom"].set_visible(False)  # Hide the bottom spine
        ax.spines["left"].set_visible(False)  # Hide the left spine
        # ax.set_title(modes[index])
        ax.set_xticks([])  # Hide x-axis ticks
        # ax.set_yticks([])  # Hide x-axis ticks
        ax.set_yticks(positions)  # Hide y-axis ticks
        ax.set_yticklabels(yticklabels, fontsize=15)  # Hide y-axis ticks

        # Add labels and title (optional, since the axis is hidden)
        # fig.supxlabel("Normalized Area-Under-Curve")
        fig.supxlabel(r"\textbf{N-AUC}")
        # plt.ylabel("")

        if self._params["save_figs"]:
            # fig.savefig(fname=self._result_prefix / "all_avg_acc_pre.png", dpi=fig.dpi)
            save_figs(fig, self._result_prefix / "sim_wpt_pre_cum_legend_v1")

        # Add labels and title
        # plt.xlabel("Values")
        # plt.ylabel("Categories")
        # plt.title("Horizontal Bar Plot with Different Colors per Category")
        plt.show()


def main():
    """Main Script"""

    # Load params.
    params_file = CONFIG_PATH / "params.yaml"
    params = load_params(params_file)

    # Run visualization.
    VisAllSequences(params).run()


if __name__ == "__main__":
    main()
