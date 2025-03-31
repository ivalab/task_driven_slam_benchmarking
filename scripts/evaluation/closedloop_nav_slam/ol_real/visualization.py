#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file visualization.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 07-22-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.axes_grid1.inset_locator import (
    inset_axes,
    mark_inset,
    zoomed_inset_axes,
)
import seaborn as sns
import pandas as pd
import numpy as np
from scipy.stats import ks_2samp
from pathlib import Path

from cl_common.slam_methods import METHODS_VISUALS
from cl_common.datasets import EUROC_DATASET
from cl_common.utils import (
    draw_grids,
    draw_x_eq_y,
    save_figs,
    draw_second_x_axis,
    draw_second_y_axis,
    draw_second_axis,
)
from cl_common.slam_methods import (
    AXIS_VISUALS,
)
from cl_common import matplotlib_config


class Visualization:

    def __init__(self, params):
        self._params = params
        self._mode_labels = {"regular": "w/o map", "slomo": "w/ map"}
        self._metrics = ["Accuracy", "Precision"]
        self._dims = ["Position", "Orientation"]
        self._output_dir = Path(self._params["result_dir"])
        self._markers = ["s", "o", "*", "+"]
        self._grids = [5.0, 1.0]  # cm, degree
        self._colors = ["tab:blue", "tab:green"]

    def run(self, results):
        """_summary_

        Args:
            results (_type_): _description_
        """
        # Flatten the data structure
        data = []
        for mode_name, methods in results.items():
            for method_name, sequences in methods.items():
                method_label = METHODS_VISUALS[method_name].label
                for sequence_name, slam_result in sequences.items():
                    if not slam_result.is_valid():
                        continue
                    for idx in range(slam_result.frames_accuracy.shape[0]):
                        for dim_index, dim_label in enumerate(self._dims):
                            data.append(
                                (
                                    sequence_name,
                                    method_label + "\nAccuracy",
                                    self._mode_labels[mode_name],
                                    slam_result.frames_accuracy[idx, dim_index],
                                    dim_label,
                                )
                            )
                            data.append(
                                (
                                    sequence_name,
                                    method_label + "\nPrecision",
                                    self._mode_labels[mode_name],
                                    slam_result.frames_precision[idx, dim_index],
                                    dim_label,
                                )
                            )

        # Create a DataFrame for easier plotting
        df = pd.DataFrame(data, columns=["sequence", "method", "mode", "value", "dimension"])
        df.to_csv("/tmp/ol_precision_test.csv")

        self._sequence_count = len(df["sequence"].unique())
        self._method_count = len(df["method"].unique())

        # Plot avg acc v.s. pre.
        self.__plot_avg_acc_pre(results)

        # Plot acc v.s. pre.
        # self.__plot_acc_pre(df)

        # Plot regular v.s. slomo.
        # self.__plot_regular_slomo(df)

        # Plot KS Test.
        # self.__plot_KS_test(df)

        # Plot cumulative plot.
        # self.__plot_cumulative_curve(results)

    def __plot_avg_acc_pre(self, results):
        """_summary_

        Args:
            results (_type_): _description_
        """
        mode_count = len(self._mode_labels)
        metric_count = len(self._metrics)
        seq_count = self._sequence_count
        dim_count = len(self._dims)
        data = {}
        for mode_index, (mode_name, mode_label) in enumerate(self._mode_labels.items()):
            if mode_name not in results.keys():
                methods_results = results["slomo"]
            else:
                methods_results = results[mode_name]
            for method_index, (method_name, sequences) in enumerate(methods_results.items()):
                if method_name not in data:
                    data[method_name] = np.full((seq_count, mode_count, metric_count, dim_count), np.nan)
                for seq_index, (_, slam_result) in enumerate(sequences.items()):
                    if not slam_result.is_valid():
                        continue
                    data[method_name][seq_index, mode_index, 0, :] = np.nanmean(slam_result.frames_accuracy, axis=0)
                    data[method_name][seq_index, mode_index, 1, :] = np.nanmean(slam_result.frames_precision, axis=0)

        fig, axs = plt.subplots(1, 2, figsize=(10, 7))
        plt.subplots_adjust(wspace=0.8)  # Increase space between rows
        td_labels = ["/6", "/70"]
        for dim_index, dim_name in enumerate(self._dims):
            max_err = -1
            for method_index, (method_name, errs) in enumerate(data.items()):
                for seq_index in range(seq_count):
                    for mode_index in range(mode_count):
                        label = (
                            f"{METHODS_VISUALS[method_name].label} ({list(self._mode_labels.values())[mode_index]})"
                            if seq_index == 0
                            else None
                        )
                        axs[dim_index].plot(
                            errs[seq_index, mode_index, 0, dim_index],
                            errs[seq_index, mode_index, 1, dim_index],
                            color=self._colors[mode_index],
                            marker=self._markers[method_index],
                            markersize=4 + mode_index * 6,
                            markerfacecolor="none",
                            markeredgewidth=2,
                            label=label,
                            linestyle="none",
                        )

                    axs[dim_index].plot(
                        errs[seq_index, :, 0, dim_index],
                        errs[seq_index, :, 1, dim_index],
                        # color="tab:green",
                        color="gray",
                        linestyle="dashed",
                        linewidth=1.0,
                    )
                max_err = max(max_err, np.max(errs[..., dim_index]))

            # max_err *= 1.1
            grid_value = self._grids[dim_index]
            max_err = np.ceil(max_err / grid_value) * grid_value
            draw_x_eq_y(axs[dim_index], 0.0, max_err)
            draw_grids(axs[dim_index], grid_value, [0, max_err], [0, max_err])
            # axs[dim_index].legend()
            axs[dim_index].set_aspect("equal")
            axs[dim_index].set_title(dim_name, y=-0.25, pad=-14)

            xvis = AXIS_VISUALS[f"{dim_name} Accuracy"]
            yvis = AXIS_VISUALS[f"{dim_name} Precision"]
            xlabel = f"{xvis.symbol} ({xvis.unit})"
            ylabel = f"{yvis.symbol} ({yvis.unit})"
            xlabel2 = f"{xvis.symbol} ({xvis.td_unit})"
            ylabel2 = f"{yvis.symbol} ({yvis.td_unit})"
            axs[dim_index].set_xlabel(xlabel, fontsize=18)
            axs[dim_index].set_ylabel(ylabel, fontsize=18)
            draw_second_axis(axs[dim_index], self._grids[dim_index], td_labels[dim_index], xlabel2, ylabel2)

        # fig.suptitle("Distribution of Frame Position Accuracy and Precision")
        # axs[1].legend()
        # fig.supxlabel("Accuracy")
        # fig.supylabel("Precision")
        handles, labels = axs[1].get_legend_handles_labels()
        fig.legend(
            handles,
            labels,
            bbox_to_anchor=(0.5, 0.92),
            ncol=2,
            fancybox=True,
            shadow=True,
            loc="upper center",
        )

        # Add a vertical line between subplots
        plt.plot(
            [0.5, 0.5],
            [0.15, 0.8],
            linestyle="dashed",
            color="black",
            lw=1,
            transform=plt.gcf().transFigure,
            clip_on=False,
        )

        # axs[0].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
        # fig.suptitle("EuRoC Machine Hall Result (Five Sequences)")
        # plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "ol_euroc_pre_acc")
        plt.show()

    def __plot_acc_pre(self, df):
        # Create the violin plot
        sequences = df["sequence"].unique()
        num_sequences = len(sequences)
        modes = self._mode_labels.values()
        fig, axs = plt.subplots(2, num_sequences, figsize=(6 * num_sequences, 10), sharex=True, sharey=True)
        for seq_index, sequence in enumerate(sequences):
            for mode_index, mode_name in enumerate(modes):
                subset = df[(df["sequence"] == sequence) & (df["mode"] == mode_name)]
                ax = sns.violinplot(
                    x="method",
                    y="value",
                    hue="dimension",
                    data=subset,
                    gap=0.1,
                    dodge=True,
                    inner="quart",
                    split=True,
                    palette="muted",
                    ax=axs[mode_index, seq_index],
                    cut=0,
                )
                self.__color_the_lines(ax)
                if mode_index == 0:
                    axs[mode_index, seq_index].set_title(f"Sequence: {sequence}")
                axs[mode_index, seq_index].legend(loc="upper right")
                axs[mode_index, 0].set_ylabel(f"Error {mode_name} (cm)")
                axs[mode_index, seq_index].set(xlabel=None, ylabel=None)
        # plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "sequence_acc_pre")
        plt.show()

        # Plot all sequences
        fig, axs = plt.subplots(1, 2, sharey=True, sharex=True)
        for mode_index, mode_name in enumerate(modes):
            subset = df[df["mode"] == mode_name]
            ax = sns.violinplot(
                x="method",
                y="value",
                hue="dimension",
                data=subset,
                gap=0.1,
                dodge=True,
                inner="quart",
                split=True,
                palette="muted",
                ax=axs[mode_index],
                cut=0,
            )
            self.__color_the_lines(ax)
            axs[mode_index].set_title(mode_name)
            axs[mode_index].legend(loc="upper right")
            axs[mode_index].set(xlabel=None, ylabel=None)
            axs[mode_index].set_ylabel(f"Error (cm)")
        # plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "all_sequences_acc_pre")
        plt.show()

    def __plot_regular_slomo(self, df):
        # Create the violin plot
        sequences = df["sequence"].unique()
        num_sequences = len(sequences)
        fig, axs = plt.subplots(2, num_sequences, figsize=(6 * num_sequences, 10), sharex=True, sharey=True)
        for seq_index, sequence in enumerate(sequences):
            for dim_index, dim_label in enumerate(self._dim_labels):
                subset = df[(df["sequence"] == sequence) & (df["dimension"] == dim_label)]
                ax = sns.violinplot(
                    x="method",
                    y="value",
                    hue="mode",
                    hue_order=["w/o map", "w/ map"],
                    gap=0.1,
                    data=subset,
                    dodge=True,
                    inner="quart",
                    split=True,
                    palette="muted",
                    ax=axs[dim_index, seq_index],
                    cut=0,
                )
                self.__color_the_lines(ax)
                if dim_index == 0:
                    axs[dim_index, seq_index].set_title(f"Sequence: {sequence}")
                axs[dim_index, seq_index].legend(loc="upper right")
                axs[dim_index, seq_index].set(xlabel=None, ylabel=None)
        axs[0, 0].set_ylabel("Position (cm)")
        axs[1, 0].set_ylabel("Orientation (degree)")
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "mode_acc_pre")
        # plt.tight_layout()
        plt.show()

        # Plot all sequences
        fig, ax = plt.subplots(1, 1, figsize=(5, 4))
        axs = [ax]
        for dim_index, dim_label in enumerate(self._dim_labels):
            subset = df[(df["dimension"] == dim_label)]
            ax = sns.violinplot(
                x="method",
                y="value",
                hue="mode",
                hue_order=["w/o map", "w/ map"],
                gap=0.1,
                data=subset,
                dodge=True,
                inner="quart",
                split=True,
                palette="muted",
                ax=axs[dim_index],
                cut=0,
            )
            self.__color_the_lines(ax)
            # self.__draw_horizon_grids(ax, 15.0)
            axs[dim_index].set_ylabel(self._dim_labels[dim_index])
            axs[dim_index].legend(loc="upper right")
            axs[dim_index].set(xlabel=None)
            break  # Only plot position.
        self.__draw_horizon_grids(axs[0], 15.0)
        # fig.suptitle("Frame Position Accuracy and Precision Distribution")
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "ol_euroc_violin")
        plt.show()

    def __color_the_lines(self, ax):
        for l in ax.lines:
            l.set_linestyle("--")
            l.set_linewidth(1)
            l.set_color("red")
            l.set_alpha(0.8)
        for l in ax.lines[1::3]:
            l.set_linestyle("-")
            l.set_linewidth(1.5)
            l.set_color("black")
            l.set_alpha(0.8)

    def __draw_horizon_grids(self, ax, step):
        xlims = ax.get_xlim()
        ylims = ax.get_ylim()
        yticks = np.arange(0, ylims[1] * 1.01, step)
        ax.set_yticks(yticks)
        ax.set_yticklabels(yticks)
        ax.set_ylim([yticks[0], ylims[-1]])
        for yy in yticks[1:]:
            ax.plot(xlims, [yy, yy], color="grey", linestyle="--", linewidth=0.5)

        ax2 = ax.secondary_yaxis("right")  # Create the second Y-axis to the right
        ax2.set_ylim(ylims)  # Match the limits of the first x-axis
        ax2.set_yticks(yticks)
        ax2.set_yticklabels([f"{i}/2" for i in range(len(yticks))])  # Example: Double the original ticks
        # ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
        ax2.set_ylabel("Robot Diameter")
        return ax2

    def __plot_KS_test(self, df):
        modes = df["mode"].unique()
        methods = df["method"].unique()
        for mode_name in modes:
            for method_name in methods:
                print(mode_name, method_name)
                acc_subset = df[
                    (df["metric"] == "accuracy") & (df["mode"] == mode_name) & (df["method"] == method_name)
                ]
                pre_subset = df[
                    (df["metric"] == "precision") & (df["mode"] == mode_name) & (df["method"] == method_name)
                ]
            print(
                f"Accuracy \n mean: {acc_subset.stack().mean()}, median {acc_subset.median()}, std: {acc_subset.std()}"
            )
            print(f"Precision \n mean: {pre_subset.mean()}, median {pre_subset.median()}, std: {pre_subset.std()}")
            # Perform KS test
            # ks_statistic, ks_p_value = ks_2samp(data_mode1, data_mode2)
            # ks_results[metric] = {"KS Statistic": ks_statistic, "p-value": ks_p_value}

        # # Convert results to DataFrame for plotting
        # df_ks = pd.DataFrame(ks_results).T

        # # Plotting KS Test Results
        # plt.figure(figsize=(10, 6))

        # sns.barplot(x=df_ks.index, y="KS Statistic", data=df_ks, palette="viridis")
        # plt.ylabel("KS Statistic")
        # plt.title("KS Test Statistic Between Modes")
        # plt.show()

    def __plot_cumulative_curve(self, results):
        fig, axs = plt.subplots(1, 2, figsize=(10, 6))
        fig.subplots_adjust(bottom=0.2)  # Shift the main axis up to the top a bit.
        # Prepare zoom-in section.
        axsins = [
            inset_axes(axs[0], width="50%", height="50%", loc="lower right", borderpad=2),
            inset_axes(axs[1], width="50%", height="50%", loc="lower right", borderpad=2),
        ]
        linestyles = ["solid", "dashdot", "dotted"]
        colors = ["tab:orange", "tab:blue"]
        markers = ["s", "^"]
        for mode_index, (mode_name, methods) in enumerate(results.items()):
            for method_index, (method_name, sequences) in enumerate(methods.items()):
                for metric_index, metric_name in enumerate(self._metrics):
                    all_errs = []
                    count = 0
                    for sequence_name, slam_result in sequences.items():
                        acc_pre_array = slam_result.stamped_acc_pre[:, 1:3] * 100.0  # Scale to cm.
                        all_errs.extend(acc_pre_array[:, metric_index])
                        count += EUROC_DATASET.get_sequence(sequence_name).image_count
                        break

                    # sorted_errs = np.sort(all_errs)
                    # cumulative_counts = np.arange(1, len(sorted_errs) + 1) / count * 100
                    count_inv = 100.0 / count

                    xdata = np.arange(0, 61, 5)
                    ydata = np.nansum(np.array(all_errs)[:, None] < xdata, axis=0) * count_inv
                    axs[metric_index].plot(
                        xdata,
                        ydata,
                        label=f"{method_name} ({self._mode_labels[mode_name]})",
                        color=colors[mode_index],
                        marker=markers[method_index],
                        linestyle=linestyles[mode_index],
                        fillstyle="none",
                    )

                    axsins[metric_index].plot(
                        xdata,
                        ydata,
                        # label=f"{method_name} ({self._mode_labels[mode_name]})",
                        color=colors[mode_index],
                        marker=markers[method_index],
                        linestyle=linestyles[mode_index],
                        fillstyle="none",
                    )

                    axs[metric_index].set_title(f"{metric_name}")

        for i in range(2):
            axs[i].set_ylim([0, 105])
            axs[i].set_xlim([0, 61])
            axs[i].set_xlabel("Frame Error (cm)")
            axsins[i].set_xlim(0, 15)
            axsins[i].set_ylim(90, 100)
            mark_inset(axs[i], axsins[i], loc1=1, loc2=3, fc="none", ec="red", lw=0.3)
            draw_second_x_axis(axs[i], step=self._grids[0], ticklabel="R", axislabel="Task Unit As The Robot Size")

        axs[1].legend(bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0)
        fig.supylabel("Frame Completeness (%)")
        plt.tight_layout()
        if self._params["save_figs"]:
            save_figs(fig, self._output_dir / "cumulative_acc_pre")
        plt.show()
