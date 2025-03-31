#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file vis_modes.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 08-13-2024
@version 1.0
@license Copyright (c) 2024
@desc This script is to visualize simulation results w.r.t SLAM running modes 
      (w/ and w/o map) in all the sequences.
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

from cl_common.utils import (
    draw_horizon_grids,
    draw_horizon_grids_orientation,
    color_violin_plot_stats_lines,
    save_figs,
)
from cl_sim.utils.utils import (
    load_params,
    RobotNavigationData,
)

from cl_sim.utils.path_definitions import CONFIG_PATH
from cl_common import matplotlib_config


class VisInModes:
    """_summary_"""

    def __init__(self, params):
        self._params = params
        self._modes = {"slam": "w/o map", "localization": "w/ map"}
        self._colors = ["tab:blue", "tab:orange"]
        self._methods = self._params["methods"]
        self._scales = [100.0, np.rad2deg(1.0)]
        self._result_prefix = Path(self._params["result_dir"]).resolve()
        self._stats = np.full(
            (len(self._methods), 5, len(self._modes)), np.nan
        )  # (method_count x metric_count x mode_count), metric = [acc_t, acc_r, pre_t, pre_r, compl]

    def run(self):
        """_summary_"""
        df_xy, df_theta = self.__prepare_data()
        df_xy.to_csv("/tmp/cl_precision_test_xy.csv")
        df_theta.to_csv("/tmp/cl_precision_test_theta.csv")

        # self.__plot_acc_pre(df_xy, "Position")
        self.__plot_acc_pre_violin(df_xy, "Position")
        # self.__plot_acc_pre_violin(df_theta, "Orientation")
        # self.__plot_acc_pre_2(df_theta, "Orientation (degree)")

    def __prepare_data(self):
        # Loop over method.
        data_xy = []
        data_theta = []
        for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
            mode_dir = self._result_prefix / mode_name
            for method_index, method_name in enumerate(self._methods):
                method_arr = []
                method_pre = []
                act_wpts_counts = 0
                exp_wpts_counts = 0
                for seq_index, seq_name in enumerate(self._params["envs"]):
                    seq_dir = mode_dir / seq_name / "evals"
                    filename = seq_dir / f"{method_name}_eval.pkl"
                    if not filename.is_file():
                        print(filename)
                        continue
                    result = RobotNavigationData(method_name)
                    result.load_from_file(str(filename))
                    for experiment in result.data:
                        indices = experiment["completeness"] > 0.99
                        act_wpts_counts += np.sum(indices)
                        exp_wpts_counts += experiment["completeness"].shape[0]
                        if np.sum(indices) == 0:
                            continue
                        acc_arr = experiment["accuracy"][indices, :] * self._scales
                        pre_arr = experiment["precision"][indices, :] * self._scales
                        method_arr.append(acc_arr)
                        method_pre.append(pre_arr)
                        for acc, pre in zip(acc_arr, pre_arr):
                            data_xy.append(
                                (mode_label, seq_name, METHODS_VISUALS[method_name].label, acc[0], "Accuracy")
                            )
                            data_xy.append(
                                (mode_label, seq_name, METHODS_VISUALS[method_name].label, pre[0], "Precision")
                            )
                            data_theta.append(
                                (mode_label, seq_name, METHODS_VISUALS[method_name].label, acc[1], "Accuracy")
                            )
                            data_theta.append(
                                (mode_label, seq_name, METHODS_VISUALS[method_name].label, pre[1], "Precision")
                            )

                if len(method_arr) > 0:
                    self._stats[method_index, :2, mode_index] = np.mean(np.vstack(method_arr), axis=0)
                    self._stats[method_index, 2:4, mode_index] = np.mean(np.vstack(method_pre), axis=0)
                    self._stats[method_index, 4, mode_index] = act_wpts_counts / float(exp_wpts_counts)

        df_xy = pd.DataFrame(data_xy, columns=["mode", "sequence", "method", "value", "metric"])
        df_theta = pd.DataFrame(data_theta, columns=["mode", "sequence", "method", "value", "metric"])
        # assert np.sum(np.isnan(self._stats)) == 0
        return df_xy, df_theta

    def __plot_acc_pre(self, df, dummy_name=""):
        # Create the violin plot
        sequences = df["sequence"].unique()
        num_sequences = len(sequences)
        # mode_labels = ["w/o map", "w/ map"]
        fig, axs = plt.subplots(2, num_sequences, figsize=(6 * num_sequences, 10), sharex=True, sharey=True)
        for seq_index, sequence in enumerate(sequences):
            for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
                subset = df[(df["sequence"] == sequence) & (df["mode"] == mode_label)]
                ax = sns.violinplot(
                    x="method",
                    y="value",
                    hue="metric",
                    data=subset,
                    gap=0.1,
                    dodge=True,
                    inner="quart",
                    split=True,
                    palette="muted",
                    ax=axs[mode_index, seq_index],
                    cut=0,
                )
                color_violin_plot_stats_lines(ax)
                if mode_index == 0:
                    axs[mode_index, seq_index].set_title(f"Sequence: {sequence}")
                axs[mode_index, seq_index].legend(loc="upper left")
                axs[mode_index, 0].set_ylabel(f"Error {mode_label} (cm)")
                axs[mode_index, seq_index].set(xlabel=None, ylabel=None)
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._result_prefix / "sequence_acc_pre_in_modes.png")
        plt.show()

        # Plot all sequences
        fig, axs = plt.subplots(1, 2, sharey=True, sharex=True, figsize=(14, 6))
        for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
            subset = df[df["mode"] == mode_label]
            ax = sns.violinplot(
                x="method",
                y="value",
                hue="metric",
                data=subset,
                gap=0.3,
                dodge=True,
                inner="quart",
                split=True,
                palette="muted",
                ax=axs[mode_index],
                cut=0,
            )
            color_violin_plot_stats_lines(ax)
            axs[mode_index].set_title(mode_label)
            axs[mode_index].legend(loc="upper left")
            axs[mode_index].set(xlabel=None, ylabel=None)
            axs[mode_index].set_ylabel("Error (cm)")
            # ax.tick_params(axis="x", labelrotation=45)
            plt.setp(axs[mode_index].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
            # axs.stxticks(rotation=45)
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._result_prefix / "acc_pre_in_modes.png")
        plt.show()

    # def __plot_acc_pre_2(self, df, dummy_name=""):
    #     # Create the violin plot
    #     sequences = df["sequence"].unique()
    #     num_sequences = len(sequences)
    #     # mode_labels = ["w/o map", "w/ map"]
    #     fig, axs = plt.subplots(2, num_sequences, figsize=(6 * num_sequences, 10), sharex=True, sharey=True)
    #     for seq_index, sequence in enumerate(sequences):
    #         for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
    #             subset = df[(df["sequence"] == sequence) & (df["mode"] == mode_label)]
    #             ax = sns.violinplot(
    #                 x="method",
    #                 y="value",
    #                 hue="metric",
    #                 data=subset,
    #                 gap=0.1,
    #                 dodge=True,
    #                 inner="quart",
    #                 split=True,
    #                 palette="muted",
    #                 ax=axs[mode_index, seq_index],
    #                 cut=0,
    #             )
    #             color_violin_plot_stats_lines(ax)
    #             if mode_index == 0:
    #                 axs[mode_index, seq_index].set_title(f"Sequence: {sequence}")
    #             axs[mode_index, seq_index].legend(loc="upper left")
    #             axs[mode_index, 0].set_ylabel(f"Error {mode_label} (cm)")
    #             axs[mode_index, seq_index].set(xlabel=None, ylabel=None)
    #     plt.tight_layout()
    #     if self._params["save_figs"]:
    #         save_figs(fig, self._result_prefix / "sequence_modes_acc_pre.png")
    #     plt.show()

    #     # Plot all sequences
    #     indices = np.lexsort((self._stats[:, 0, 0], -self._stats[:, -1, 0]))
    #     my_order = [METHODS_LABELS[self._methods[i]] for i in indices]
    #     fig, axs = plt.subplots(1, 2, sharex=True, figsize=(10, 4))
    #     metrics = df["metric"].unique()
    #     for metric_index, metric in enumerate(metrics):
    #         subset = df[df["metric"] == metric]
    #         # my_order = subset.groupby(by=["method"])["value"].mean().sort_values().iloc[::-1].index
    #         ax = sns.violinplot(
    #             x="method",
    #             y="value",
    #             hue="mode",
    #             data=subset,
    #             gap=0.3,
    #             dodge=True,
    #             inner="quart",
    #             split=True,
    #             palette="muted",
    #             ax=axs[metric_index],
    #             cut=0,
    #             order=my_order,
    #         )

    #         # Draw a vertical line to seperate the completeness
    #         axs[metric_index].plot([1.5, 1.5], [0, 95], color="r", linestyle="dashdot", linewidth=5.0)

    #         color_violin_plot_stats_lines(ax)
    #         axs[metric_index].set_title(metric)
    #         axs[metric_index].legend(loc="upper right")
    #         axs[metric_index].set(xlabel=None)
    #         axs[metric_index].set_ylabel("Value (cm)")
    #         draw_horizon_grids(axs[metric_index], 30.0, "Robot Diameter", ymin=0, ymax=90.0)
    #         axs[metric_index].set_ylim([0, 95])
    #         # ax.tick_params(axis="x", labelrotation=45)
    #         plt.setp(axs[metric_index].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
    #         # axs.stxticks(rotation=45)

    #     for i, method_index in enumerate(indices):
    #         vals = self._stats[method_index, -1, :] * 100.0
    #         for mode_index in range(2):
    #             axs[0].text(
    #                 i,
    #                 65 - mode_index * 7,
    #                 f"{vals[mode_index]:.1f}%",
    #                 ha="center",
    #                 va="top",
    #                 fontsize=9,
    #                 color=self._colors[mode_index],
    #             )
    #     plt.tight_layout()
    #     if self._params["save_figs"]:
    #         save_figs(fig, self._result_prefix / "modes_acc_pre")
    #     plt.show()

    def __plot_acc_pre_violin(self, df, name):
        # Create the violin plot
        sequences = df["sequence"].unique()
        num_sequences = len(sequences)
        # mode_labels = ["w/o map", "w/ map"]
        fig, axs = plt.subplots(2, num_sequences, figsize=(6 * num_sequences, 10), sharex=True, sharey=True)
        for seq_index, sequence in enumerate(sequences):
            for mode_index, (mode_name, mode_label) in enumerate(self._modes.items()):
                subset = df[(df["sequence"] == sequence) & (df["mode"] == mode_label)]
                ax = sns.violinplot(
                    x="method",
                    y="value",
                    hue="metric",
                    data=subset,
                    gap=0.1,
                    dodge=True,
                    inner="quart",
                    split=True,
                    palette="muted",
                    ax=axs[mode_index, seq_index],
                    cut=0,
                )
                color_violin_plot_stats_lines(ax)
                if mode_index == 0:
                    axs[mode_index, seq_index].set_title(f"Sequence: {sequence}")
                axs[mode_index, seq_index].legend(loc="upper left")
                axs[mode_index, 0].set_ylabel(f"Error {mode_label} (cm)")
                axs[mode_index, seq_index].set(xlabel=None, ylabel=None)
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._result_prefix / "sequence_modes_acc_pre.png")
        # plt.show()

        # Plot all sequences
        index_of_interest = 0 # if "Position" in name else 1
        indices = np.lexsort((self._stats[:, index_of_interest, 1], -self._stats[:, -1, 1]))
        my_order = [METHODS_VISUALS[self._methods[i]].label for i in indices]
        # my_order = ["SLAM-Toolbox", "GF-GG", "ORB_SLAM3"]
        fig, axs = plt.subplots(2, 1, sharey=True)
        metrics = df["metric"].unique()
        ylims = [-1.0, -1.0]
        for metric_index, metric in enumerate(metrics):
            subset = df[df["metric"] == metric]
            # my_order = subset.groupby(by=["method"])["value"].mean().sort_values().iloc[::-1].index
            ax = sns.violinplot(
                x="method",
                y="value",
                hue="mode",
                data=subset,
                dodge=True,
                inner="quart",
                split=True,
                palette="muted",
                ax=axs[metric_index],
                cut=0,
                order=my_order,
                gap=0.1,
            )

            ylims[0] = min(ylims[0], ax.get_ylim()[0])
            ylims[1] = max(ylims[1], ax.get_ylim()[1])

            color_violin_plot_stats_lines(ax)
            axs[metric_index].set_title(f"{name} {metric}", fontsize=12)
            axs[metric_index].legend(loc="upper left")
            axs[metric_index].set(xlabel=None)
            axs[metric_index].set(ylabel=None)
            y_vis1 = AXIS_VISUALS[f"{name} {metric}"]
            ylabel1 = f"{y_vis1.symbol} ({y_vis1.unit})"
            axs[metric_index].set_ylabel(ylabel1)
            # ax.tick_params(axis="x", labelrotation=45)
            plt.setp(axs[metric_index].get_xticklabels(), rotation=0, ha="center", rotation_mode="anchor", fontsize=12)
        axs[1].text(
            -0.75,
            -0.25,
            "C (\%):",
            ha="center",
            va="top",
            transform=axs[1].get_xaxis_transform(),
            fontsize=10,
        )

        for pos, index in enumerate(indices):
            comp = self._stats[index, -1, :] * 100.0
            text = f"({comp[0]:.1f} $|$ {comp[1]:.1f})"
            axs[1].text(
                pos,
                -0.25,
                rf"\textbf{{{text}}}",
                ha="center",
                va="top",
                fontsize=10,
                transform=axs[1].get_xaxis_transform(),
            )

            # axs.stxticks(rotation=45)
        axs[0].legend().set_visible(False)
        # fig.supylabel(dummy_name, x=0.04)
        ylims[0] = 0
        for metric_index, metric in enumerate(metrics):
            # Draw a vertical line to seperate the completeness
            axs[metric_index].plot([1.5, 1.5], ylims, color="r", linestyle="dashdot", linewidth=2.0)
            if "Position" in name:
                y_vis2 = AXIS_VISUALS[f"Position {metric}"]
                ylabel2 = f"{y_vis2.symbol} ({y_vis2.td_unit})"
                ax2 = draw_horizon_grids(axs[metric_index], 30.0, ylabel2, ymin=ylims[0], ymax=ylims[1])
                # if metric_index == 1:
                #     ax2.set_ylabel(None)
                # else:
                #     ax2.set_ylabel("Robot Diameter", y=-1)
            elif "Orientation" in name:
                y_vis2 = AXIS_VISUALS[f"Position {metric}"]
                ylabel2 = f"{y_vis2.symbol} ({y_vis2.td_unit})"
                draw_horizon_grids_orientation(axs[metric_index], 5.0, ylabel2, ymin=ylims[0], ymax=ylims[1])
            axs[metric_index].set_ylim(ylims)

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
            save_figs(fig, self._result_prefix / f"sim_acc_pre_{name}_violin")
        plt.show()


def main():
    """Main Script"""

    # Load params.
    params_file = CONFIG_PATH / "params.yaml"
    params = load_params(params_file)

    # Run visualization.
    VisInModes(params).run()


if __name__ == "__main__":
    main()
