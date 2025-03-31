#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file vis_in_modes.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 09-12-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

from pathlib import Path
import seaborn as sns
import pandas as pd
from matplotlib import pyplot as plt
import numpy as np
from cl_common.slam_methods import (
    METHODS_VISUALS,
    AXIS_VISUALS,
)

from sklearn import metrics as skmetrics
from cl_common.utils import (
    draw_horizon_grids,
    draw_horizon_grids_orientation,
    color_violin_plot_stats_lines,
    save_figs,
    draw_second_axis,
)
from cl_sim.utils.utils import (
    load_params,
    load_yaml,
)

from cl_real.utils.path_definitions import CONFIG_PATH
from cl_common import matplotlib_config


class VisInModes:
    """_summary_"""

    def __init__(self, params):
        self._params = params
        # self._path_params = load_yaml(CONFIG_PATH / f"{params['path']}.yaml")
        self._modes = {"slam": "w/o map", "localization": "w/ map"}
        self._colors = ["tab:blue", "tab:orange"]
        self._methods = self._params["methods"]
        self._paths = ["path1", "path4"]
        self._scales = [100.0, np.rad2deg(1.0)]
        self._grid_xy = 30.0
        self._result_prefix = Path(self._params["result_dir"]).parent.resolve()
        self._gt_wpts_count = 0
        self._stats = np.full(
            (len(self._methods), 3, len(self._modes)), np.nan
        )  # (method_count x metric_count x mode_count), metric = [pre_t, pre_r, compl]

    def run(self):
        """_summary_"""
        df_xy, df_theta, all_results = self.__prepare_data()
        df_xy.to_csv("/tmp/cl_precision_test_xy.csv")
        df_theta.to_csv("/tmp/cl_precision_test_theta.csv")

        self.__plot_acc_pre_violin(df_xy, "Position")
        # self.__plot_acc_pre_violin(df_theta, "Orientation")
        # self.__plot_cumulative_precision(all_results)
        # self.__plot_precision_completeness(all_results)

    def __prepare_data(self):

        self._gt_wpts_count = 0
        for seq_index, seq_name in enumerate(self._paths):
            # Load planned wpts.
            path_params = load_yaml(CONFIG_PATH / f"{seq_name}.yaml")
            exp_wpts_counts = path_params["wpts_count"] - len(
                path_params["wpts_skipping_indices"]
            )  # Some wpts in the planned path, however cannot be captured by camera.
            self._gt_wpts_count += exp_wpts_counts

        # Loop over method.
        data_xy = []
        data_theta = []
        all_results = {}
        for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
            all_results[mode_label] = {}
            mode_dir = self._result_prefix / mode_name
            for method_index, method_name in enumerate(self._methods):
                method_pre = []
                method_result = []
                act_counts = 0
                exp_counts = 0

                # seq_name = "path1"
                # seq_name = self._params["path"]
                for seq_index, seq_name in enumerate(self._paths):

                    # Load planned wpts.
                    path_params = load_yaml(CONFIG_PATH / f"{seq_name}.yaml")
                    exp_wpts_counts = path_params["wpts_count"] - len(
                        path_params["wpts_skipping_indices"]
                    )  # Some wpts in the planned path, however cannot be captured by camera.
                    exp_counts += exp_wpts_counts

                    # Load existing file.
                    seq_dir = mode_dir / seq_name / "evaluation"
                    filename = seq_dir / f"{method_name}_{mode_name}_wpts_precision.txt"
                    if not filename.is_file():
                        print(filename)
                        continue
                    result = np.loadtxt(filename)[:-1, :]  # Skip the mean value in the last row
                    indices = result[:, -1] > 0.99
                    act_wpts_counts = np.sum(indices)
                    act_counts += act_wpts_counts
                    # exp_wpts_counts = result.shape[0]

                    # print(mode_name, method_name, act_wpts_counts, exp_wpts_counts)
                    if np.sum(indices) == 0:
                        continue
                    method_seq_pre = result[indices, :2]
                    for pre in method_seq_pre:
                        data_xy.append((mode_label, seq_name, METHODS_VISUALS[method_name].label, pre[0]))
                        data_theta.append((mode_label, seq_name, METHODS_VISUALS[method_name].label, pre[1]))

                    method_pre.append(method_seq_pre)
                    method_result.append(result)
                    # all_results[mode_label][method_name] = result
                if len(method_pre) > 0:
                    all_results[mode_label][method_name] = np.concatenate(method_result, axis=0)
                    self._stats[method_index, :2, mode_index] = np.mean(np.concatenate(method_pre, axis=0), axis=0)
                    self._stats[method_index, -1, mode_index] = act_counts / float(exp_counts)
                    # print(method_name, act_counts / exp_counts)
                else:
                    self._stats[method_index, -1, mode_index] = 0.0

        df_xy = pd.DataFrame(data_xy, columns=["mode", "sequence", "method", "value"])
        df_theta = pd.DataFrame(data_theta, columns=["mode", "sequence", "method", "value"])
        # assert np.sum(np.isnan(self._stats)) == 0
        return df_xy, df_theta, all_results

    def __plot_acc_pre_violin(self, df, name):
        # Create the violin plot
        # sequences = df["sequence"].unique()
        # Do a copy of all stats. Sort according to the completeness in w/o map in all sequences.
        local_stats = np.copy(self._stats)
        for method_data in local_stats:
            if np.isnan(method_data[-1, 1]):
                method_data[-1, 1] = method_data[-1, 0]
        mode_index = 0
        # local_stats = np.mean(self._stats, axis=1)
        # print(self._stats.shape, local_stats.shape)
        # print(local_stats)
        index_of_interest = 0 if "Position" in name else 1
        # indices = np.lexsort((local_stats[:, index_of_interest, mode_index], -local_stats[:, -1, mode_index]))
        indices = np.lexsort(
            (
                local_stats[:, -1, 0],
                local_stats[:, -1, 1],
                # local_stats[:, index_of_interest, 0],
                # local_stats[:, index_of_interest, 1],
                # -local_stats[:, -1, mode_index],
            )
        )[::-1]
        # tmp_v = indices[-1]
        # indices[-1] = indices[-2]
        # indices[-2] = tmp_v
        my_order = [METHODS_VISUALS[self._methods[i]].label for i in indices]
        fig, ax = plt.subplots(1, 1, sharex=True, figsize=(15, 4))  # , figsize=(4, 14))
        ylims = [-1.0, -1.0]
        # my_order = subset.groupby(by=["method"])["value"].mean().sort_values().iloc[::-1].index
        ax = sns.violinplot(
            x="method",
            y="value",
            hue="mode",
            data=df,
            gap=0.3,
            dodge=True,
            inner="quart",
            split=True,
            palette="muted",
            ax=ax,
            cut=0,
            order=my_order,
            density_norm="area",
            scale="count",
        )

        ylims[0] = min(ylims[0], ax.get_ylim()[0])
        ylims[1] = max(ylims[1], ax.get_ylim()[1])

        color_violin_plot_stats_lines(ax)
        # ax.set_title("Precision")
        # ax.legend(loc="upper right")
        ax.set(xlabel=None)
        y_vis1 = AXIS_VISUALS[f"{name} Precision"]
        ylabel1 = f"{y_vis1.symbol} ({y_vis1.unit})"
        ax.set_ylabel(ylabel1)
        # ax.set_ylabel("Position Precision \n (cm)", fontsize=25)
        # ax.tick_params(axis="x", labelrotation=45)
        plt.setp(ax.get_xticklabels(), rotation=0, ha="center", rotation_mode="anchor")
        ax.text(
            -0.75,
            -0.15,
            "C (\%):",
            ha="center",
            va="top",
            transform=ax.get_xaxis_transform(),
            fontsize=20,
        )

        for pos, index in enumerate(indices):
            # comp = self._stats[index, -1, :] * 100.0
            comp = local_stats[index, -1, :] * 100.0
            if np.isnan(comp[1]) or comp[1] == 0:
                text = f"({comp[0]:.1f})"
            else:
                text = f"({comp[0]:.1f} $|$ {comp[1]:.1f})"
            ax.text(
                pos,
                -0.15,
                rf"\textbf{{{text}}}",
                ha="center",
                va="top",
                fontsize=20,
                transform=ax.get_xaxis_transform(),
            )

        # axs.stxticks(rotation=45)
        ax.legend(fontsize=20, loc="upper right")
        ax.set_title(f"{name} Precision")
        ylims[0] = 0
        # Draw a vertical line to seperate the completeness
        # ax.plot([5.5, 5.5], ylims, color="r", linestyle="dashdot", linewidth=2.0)
        ax.plot([1.5, 1.5], ylims, color="r", linestyle="dashdot", linewidth=2.0)
        if "Position" in name:
            y_vis2 = AXIS_VISUALS[f"Position Precision"]
            ylabel2 = f"{y_vis2.symbol} ({y_vis2.td_unit})"
            draw_horizon_grids(ax, 30.0, ylabel2, ymin=ylims[0], ymax=ylims[1])
        elif "Orientation" in name:
            y_vis2 = AXIS_VISUALS[f"Orientation Precision"]
            ylabel2 = f"{y_vis2.symbol} ({y_vis2.td_unit})"
            draw_horizon_grids_orientation(ax, 5.0, ylabel2, ymin=ylims[0], ymax=ylims[1])
        ax.set_ylim(ylims)

        # if "Position" in dummy_name:
        #     for i, method_index in enumerate(indices):
        #         vals = self._stats[method_index, -1, :] * 100.0
        #         for mode_index in range(2):
        #             axs[0].text(
        #                 i,
        #                 65 - mode_index * 7,
        #                 f"{vals[mode_index]:.1f}\, \%",
        #                 ha="left",
        #                 va="top",
        #                 fontsize=9,
        #                 color=self._colors[mode_index],
        #             )
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._result_prefix / f"realworld_pre_{name}_violin")
        plt.show()

    def __plot_cumulative_precision(self, all_results):
        """_summary_

        Args:
            methods_results (Dict[str, List[RobotNavigationData]]): _description_
        """
        fig, axs = plt.subplots(1, 2, sharey=True)  # , figsize=(18, 5))
        mode_xmax = [92, 92]
        # mode_xmax = [192, 192]
        methods_cum_areas = {}
        for mode_index, (mode_label, methods_results) in enumerate(all_results.items()):
            for method_index, (method_name, pre_data) in enumerate(methods_results.items()):
                if method_name not in methods_cum_areas:
                    methods_cum_areas[method_name] = [0.0, 0.0]

                # gt_wpts_count = pre_data.shape[0]
                # print(mode_index, gt_wpts_count)
                indices = pre_data[:, -1] > 0.99
                pre = pre_data[indices, :2]
                inv_wpts_count = 100.0 / self._gt_wpts_count

                xdata = np.arange(0.0, mode_xmax[mode_index], self._grid_xy / 5.0)
                ydata = np.nansum(pre[:, 0, None] < xdata, axis=0) * inv_wpts_count  # Position.

                inv_area = 1.0 / ((max(xdata) - min(xdata)) * 100.0)
                cum_perc = skmetrics.auc(xdata, ydata) * inv_area
                methods_cum_areas[method_name][mode_index] = cum_perc

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

            axs[mode_index].set_title(rf"\textbf{{{mode_label}}}", y=-0.3, pad=-14)
            axs[mode_index].set_ylim([0, 102])
            axs[mode_index].set_xlim([0, mode_xmax[mode_index]])
            yticks = np.arange(0, 102, 20)
            axs[mode_index].set_yticks(yticks)
            xticks = np.arange(0, mode_xmax[mode_index], self._grid_xy)
            axs[mode_index].set_xticks(xticks)

            axs[mode_index].plot([30, 30], [0, 100], color="grey", linestyle="--", linewidth=0.5)
            xaxis_visual = AXIS_VISUALS["Position Precision"]
            axs[mode_index].set_xlabel(f"{xaxis_visual.symbol} ({xaxis_visual.unit})")
            draw_second_axis(
                axs[mode_index],
                30.0,
                "",
                f"{xaxis_visual.symbol} ({xaxis_visual.td_unit})",
                "",
            )

            # # Draw second axis.
            # ax2 = axs[mode_index].secondary_xaxis("bottom")  # Create the second X-axis to the bottom
            # ax2.set_xticks(xticks)
            # ax2.set_xticklabels([f"{i}" for i in range(len(xticks))])  # Example: Double the original ticks
            # ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            # ax2.set_xlabel("Robot Diameter", fontsize=16)

            axs[0].set_ylabel("Cumulative Percentage (\%)")
            # axs[mode_index].set_xlabel("Position Precision (cm)")
        # axs[1].legend(bbox_to_anchor=(1.52, 1), loc="upper left", borderaxespad=0)
        # handles, labels = axs[0].get_legend_handles_labels()
        # fig.legend(
        #     handles,
        #     labels,
        #     bbox_to_anchor=(-0.28, 0.6),  # (1.42, 1),
        #     ncol=1,
        #     fancybox=True,
        #     shadow=True,
        #     loc="center left",
        #     fontsize="13",
        #     # borderaxespad=0,
        # )

        handles, labels = axs[0].get_legend_handles_labels()  # Only apply to position (index = 0)
        cum_areas = np.array(list(methods_cum_areas.values()))
        print(cum_areas.shape)
        sorted_indices = sorted(range(cum_areas.shape[0]), key=lambda k: np.sum(cum_areas[k, :]), reverse=True)
        handles = [handles[i] for i in sorted_indices]
        labels = [labels[i] for i in sorted_indices]
        fig.legend(
            handles,
            labels,
            bbox_to_anchor=(0.55, 0.98),  # (1.42, 1), # Upper
            # bbox_to_anchor=(0.5, 0.05),  # (1.42, 1), # Lower
            ncol=3,
            fancybox=True,
            shadow=True,
            loc="lower center",
            # borderaxespad=0,
            fontsize="13",
        )

        # fig.suptitle(f"Waypoint {method_name} Cumulative Plot")
        plt.tight_layout()
        if self._params["save_figs"]:
            # plt.savefig(
            #     fname=self._result_prefix / "waypoints_cumulative_error_plot.png",
            #     dpi=fig.dpi,
            # )
            save_figs(fig, self._result_prefix / "realworld_wpt_pre_cum")
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
        for i, (method_name, areas) in enumerate(sorted_methods_areas):
            yticklabels.append(METHODS_VISUALS[method_name].label)
            pos = -i
            for index, area in enumerate(areas):
                if np.isnan(area) or area <= 0.0:
                    continue
                ypos = (pos - index * 0.5) * 2.5
                if index == 0:
                    positions.append(ypos)
                ax.axhline(
                    y=ypos,
                    xmin=0,
                    xmax=area,
                    # color=METHODS_VISUALS[method_name].color,
                    color=self._colors[index],
                    # linestyle=linestyles[index],
                    linestyle=linestyles[index],
                    linewidth=4,
                )
                ax.text(0.95, ypos, rf"\textbf{{{area:.2f}}}", va="center", ha="left", fontsize=22)
        ax.spines["top"].set_visible(False)  # Hide the top spine
        ax.spines["right"].set_visible(False)  # Hide the right spine
        ax.spines["bottom"].set_visible(False)  # Hide the bottom spine
        ax.spines["left"].set_visible(False)  # Hide the left spine
        # ax.set_title(modes[index])
        ax.set_xticks([])  # Hide x-axis ticks
        # ax.set_yticks([])  # Hide x-axis ticks
        ax.set_yticks(positions)  # Hide y-axis ticks
        # yticklabels = [rf"\textbf{{{val}}}" for val in yticklabels]
        ax.set_yticklabels(yticklabels)  # Hide y-axis ticks

        # Add labels and title (optional, since the axis is hidden)
        fig.supxlabel(r"\textbf{N-AUC}")
        # plt.ylabel("")

        if self._params["save_figs"]:
            # fig.savefig(fname=self._result_prefix / "all_avg_acc_pre.png", dpi=fig.dpi)
            save_figs(fig, self._result_prefix / "realworld_wpt_pre_cum_legend_v1")

        # Add labels and title
        # plt.xlabel("Values")
        # plt.ylabel("Categories")
        # plt.title("Horizontal Bar Plot with Different Colors per Category")
        plt.show()

    def __plot_precision_completeness(self, all_results):
        """_summary_

        Args:
            methods_results (Dict[str, List[RobotNavigationData]]): _description_
        """
        method_data = {}
        for mode_index, (mode_label, methods_results) in enumerate(all_results.items()):
            for method_index, (method_name, pre_data) in enumerate(methods_results.items()):
                gt_wpts_count = pre_data.shape[0]
                indices = pre_data[:, -1] > 0.99
                pre = pre_data[indices, :2]
                completeness = pre.shape[0] * 100.0 / gt_wpts_count

                if method_name not in method_data:
                    method_data[method_name] = np.full((2, 3), np.nan)
                method_data[method_name][mode_index, :2] = np.mean(pre, axis=0)
                method_data[method_name][mode_index, 2] = completeness

        fig, axs = plt.subplots(1, 2, sharey=True)  # , figsize=(18, 5))
        sp_labels = ["Position", "Orientation"]
        td_steps = [30.0, 5.0]
        for sp_index, sp_label in enumerate(sp_labels):
            ymax = 0
            for name, data in method_data.items():
                axs[sp_index].plot(
                    data[0, -1],
                    data[0, sp_index],
                    color=METHODS_VISUALS[name].color,
                    marker=METHODS_VISUALS[name].marker,
                    markerfacecolor="none",
                    linestyle="-",
                    markersize=6,
                    label=METHODS_VISUALS[name].label,
                )

                axs[sp_index].plot(
                    data[1, -1],
                    data[1, sp_index],
                    color=METHODS_VISUALS[name].color,
                    marker=METHODS_VISUALS[name].marker,
                    markerfacecolor="none",
                    linestyle="-",
                    markersize=12,
                    label=METHODS_VISUALS[name].label,
                )

                axs[sp_index].plot(
                    [data[0, -1], data[1, -1]],
                    [data[0, sp_index], data[1, sp_index]],
                    color="black",
                    linestyle="-",
                    linewidth=0.8,
                )

                ymax = max(ymax, np.max(data[:, sp_index]))

            ymax *= 1.2
            print(ymax)
            axs[sp_index].set_title(sp_label, y=-0.2, pad=-14)
            axs[sp_index].set_xlim([0, 102])
            axs[sp_index].set_ylim([0, ymax])
            xticks = np.arange(0, 102, 20)
            axs[mode_index].set_xticks(xticks)
            yticks = np.arange(0, ymax, td_steps[sp_index])
            axs[mode_index].set_yticks(yticks)

            yaxis_visual = AXIS_VISUALS[f"{sp_label} Precision"]
            axs[sp_index].set_ylabel(f"{yaxis_visual.symbol} ({yaxis_visual.unit})")
            draw_second_axis(
                axs[sp_index], td_steps[sp_index], "", yaxis_label=f"{yaxis_visual.symbol} ({yaxis_visual.td_unit})"
            )

            # Draw second axis.
            # ax2 = axs[mode_index].secondary_yaxis("right")  # Create the second X-axis to the bottom
            # ax2.set_yticks(yticks)
            # ax2.set_yticklabels([f"{i}" for i in range(len(yticks))])  # Example: Double the original ticks
            # ax2.spines["bottom"].set_position(("outward", 45))  # Place the second axis even lower
            # ax2.set_ylabel("Robot Diameter")
            # axs[sp_index].plot([0, 100], [30, 30], color="grey", linestyle="--", linewidth=0.5)

            axs[sp_index].set_xlabel("Completeness (\%)")
            # axs[mode_index].set_ylabel("Position Precision (cm)")
        # axs[1].legend(bbox_to_anchor=(1.52, 1), loc="upper left", borderaxespad=0)
        handles, labels = axs[0].get_legend_handles_labels()
        fig.legend(
            handles,
            labels,
            bbox_to_anchor=(0.5, 1.21),  # (1.42, 1),
            ncol=3,
            fancybox=True,
            shadow=True,
            loc="upper center",
            # borderaxespad=0,
        )
        # fig.suptitle(f"Waypoint {method_name} Cumulative Plot")
        # plt.tight_layout()
        if self._params["save_figs"]:
            # plt.savefig(
            #     fname=self._result_prefix / "waypoints_cumulative_error_plot.png",
            #     dpi=fig.dpi,
            # )
            save_figs(fig, self._result_prefix / "realworld_wpt_pre_completeness")
        plt.show()
        # plt.pause(1)
        # plt.close()


def main():
    """Main Script"""

    # Load params.
    params_file = CONFIG_PATH / "params.yaml"
    params = load_params(params_file)

    # Run visualization.
    VisInModes(params).run()


if __name__ == "__main__":
    main()
