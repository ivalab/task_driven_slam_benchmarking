#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file visualization.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 03-05-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import logging
import pickle

from typing import Optional
import numpy as np
from matplotlib import pyplot as plt

from cl_common.slam_methods import METHODS_VISUALS


class Visualization:
    """Main visualization script."""

    def __init__(self, params: dict):
        """Initiation

        Args:
            params (dict): _description_
        """
        self._params = params
        self._output_dir = self._params["data_dir"] / "evaluation"
        self._output_dir.mkdir(exist_ok=True, parents=True)

        # Define position and orientation grid
        self._grid_xy = self._params["grid_xy"]
        self._grid_theta = self._params["grid_theta"]
        self._grid_res = [10, 5]  # cm & degree
        self._scales = [100.0, np.rad2deg(1.0)]  # cm and degree.

        # Check mode.
        if "slam" == self._params["mode"]:
            assert self._params["rounds"] > 1 and self._params["loops"] == 1
        elif "localization" == self._params["mode"]:
            assert self._params["rounds"] == 1 and self._params["loops"] > 1
        else:
            logging.error("Unsupported evaluation mode: %s.", self._params["mode"])
            assert False

    def run(self, data: dict):
        """_summary_

        Args:
            data (dict): _description_
        """
        methods_results = {}
        eval_mode = self._params["mode"]
        wpts_skipping_indices = self._params["wpts_skipping_indices"]
        for method_name, all_wpts in data.items():
            print(f"{method_name}, wpts size: {all_wpts.shape}")
            # all_wpts size: [loops, wpts_count, dim, rounds]
            wpts = self.__extract_wpts(all_wpts, eval_mode, wpts_skipping_indices)
            assert wpts is not None
            precision, completion = self.__compute_wpts_precision_and_completion(wpts)
            np.savetxt(
                self._output_dir / (f"{method_name}_{eval_mode}_wpts_precision.txt"),
                np.column_stack([precision * self._scales, completion]),
                fmt="%.4f",
                header="xy(cm) theta(degree) completion",
            )
            methods_results[method_name] = (precision, completion)

        # Save data.
        with open(self._output_dir / "all_methods_wpts_precision.pkl", "wb") as f:
            pickle.dump(methods_results, f)

        # Visualize result.
        self.__visualize_result(methods_results)

    def __extract_wpts(self, all_wpts: np.ndarray, eval_mode: str, wpts_skipping_indices: list) -> Optional[np.ndarray]:
        """This method extracts wpts from all_wpts pool.

        According to the mode, the repeatability can be rounds (slam) or
        loops(localization), also a few wpt indices are skipped if any.

        Args:
            all_wpts (np.ndarray): dim = [loops, wpts_count, 3, rounds]
            eval_mode (str):
            wpts_skipping_indices (list):

        Returns:
            np.ndarray: dim = [wpts_count, 3, repeatability(rounds|loops)]
        """
        wpts = None
        if "slam" == eval_mode:
            wpts = all_wpts[0, ~np.isin(np.arange(all_wpts.shape[1]), wpts_skipping_indices), ...]
        elif "localization" == eval_mode:
            # Skip the first mapping phase
            wpts = all_wpts[1:, ~np.isin(np.arange(all_wpts.shape[1]), wpts_skipping_indices), ...].transpose(
                1, 2, 3, 0
            )
        return wpts

    def __compute_wpts_precision_and_completion(self, wpts):
        rounds = wpts.shape[-1]
        wpts_count = wpts.shape[0]
        precision = np.full((wpts_count + 1, 2), np.nan)  # xy, theta, the last row is the average of all wpts
        completion = np.full((wpts_count + 1), 0.0)  # completion, the last row is the average of all wpts

        for wpt_index in range(wpts_count):
            # Skip wpt with only one success.
            act_rounds = rounds - np.sum(np.isnan(wpts[wpt_index, 0, :]))
            # if act_rounds < rounds:
            if act_rounds < 2:
                continue
            completion[wpt_index] = act_rounds / rounds
            # Compute precision
            # print(act_wpts[wpt_index, :, :])
            xy = wpts[wpt_index, :2, :]
            theta = wpts[wpt_index, -1, :].reshape(-1, 1)
            theta_in_xy = np.column_stack([np.cos(theta), np.sin(theta)])
            xy_mean = np.nanmean(xy, axis=1)
            theta_in_xy_mean = np.nanmean(theta_in_xy, axis=0)
            precision[wpt_index, 0] = np.nanmean(np.linalg.norm(xy - xy_mean[..., None], axis=0))
            precision[wpt_index, 1] = np.nanmean(np.arccos(theta_in_xy @ theta_in_xy_mean))

        # Compute average
        precision[-1, :] = np.nanmean(precision[:-1, :], axis=0)
        # completion[-1] = np.sum(~np.isnan(wpts[:, 0, :])) / (rounds * wpts_count)
        completion[-1] = np.mean(completion[:-1])
        return precision, completion

    def __visualize_result(self, methods_results):
        named_colors = {name: METHODS_VISUALS[name].color for name, _ in methods_results.items()}
        self.__visualize_one(methods_results, named_colors)
        self.__visualize_two(methods_results, named_colors)
        self.__visualize_three(methods_results, named_colors)
        self.__visualize_table(methods_results)

    def __visualize_one(self, methods_results, colors):
        eval_mode = self._params["mode"]
        fig, axs = plt.subplots(3, 2, sharex=True)
        xdata = None
        for method_name, (p, c) in methods_results.items():
            color = colors[method_name]
            precision = np.full_like(p, np.nan)
            precision[:, 0] = p[:, 0] * 100.0  # m to cm
            precision[:, 1] = np.rad2deg(p[:, 1])  # radian to degree
            completion = np.copy(c)
            weighted_precision = precision / completion[:, None]
            weighted_precision[-1, :] = np.nanmean(weighted_precision[:-1, :], axis=0)
            outlier_indices = np.where(completion[:-1] < 1.0)[0].astype(int)
            xdata = np.arange(0, precision.shape[0])
            completion *= 100.0  # to percent
            axs[0, 0].plot(xdata, precision[:, 0], color=color, marker="o", linestyle="dashed", label=method_name)
            axs[0, 1].plot(xdata, precision[:, 1], color=color, marker="o", linestyle="dashed", label=method_name)
            axs[1, 0].plot(xdata, completion, color=color, marker="o", linestyle="dashed", label=method_name)
            axs[1, 1].plot(xdata, completion, color=color, marker="o", linestyle="dashed", label=method_name)
            axs[2, 0].plot(
                xdata, weighted_precision[:, 0], color=color, marker="o", linestyle="dashed", label=method_name
            )
            axs[2, 1].plot(
                xdata, weighted_precision[:, 1], color=color, marker="o", linestyle="dashed", label=method_name
            )
            axs[2, 0].plot(
                xdata[outlier_indices],
                weighted_precision[outlier_indices, 0],
                color=color,
                marker="o",
                markersize=10,
                mfc="none",
                linestyle="none",
            )
            axs[2, 1].plot(
                xdata[outlier_indices],
                weighted_precision[outlier_indices, 1],
                color=color,
                marker="o",
                markersize=10,
                mfc="none",
                linestyle="none",
            )
        axs[0, 0].set_title("position (cm)")
        axs[0, 0].set_ylabel("precision")
        axs[0, 0].legend()
        axs[0, 1].set_title("orientation (degree)")
        axs[1, 0].set_ylabel("completion (%)")
        axs[2, 0].set_ylabel("weighted precision")
        axs[2, 0].set_xticks(xdata)
        axs[2, 0].set_xticklabels(["wpt" + str(i) for i in xdata[:-1]] + ["average"], rotation=45, ha="right")
        axs[2, 1].set_xticks(xdata)
        axs[2, 1].set_xticklabels(["wpt" + str(i) for i in xdata[:-1]] + ["average"], rotation=45, ha="right")
        plt.tight_layout()
        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir / f"visualization_one_{eval_mode}.png",
                dpi=fig.dpi,
                bbox_inches="tight",
            )
        # plt.show()
        plt.close(fig)

    def __visualize_two(self, methods_results, colors):
        eval_mode = self._params["mode"]
        fig, axs = plt.subplots(2, 2)
        for method_name, (p, c) in methods_results.items():
            color = colors[method_name]
            precision = np.full_like(p, np.nan)
            precision[:, 0] = p[:, 0] * 100.0  # m to cm
            precision[:, 1] = np.rad2deg(p[:, 1])  # radian to degree
            completion = np.copy(c)
            # weighted_precision = precision / completion[:, None]
            # outlier_indices = np.where(completion[:-1] < 1.0)[0].astype(int)
            completion *= 100.0  # to percent
            # print(completion, precision)
            axs[0, 0].plot(
                completion[:-1], precision[:-1, 0], color=color, marker="o", label=method_name, linestyle="none"
            )
            axs[1, 0].plot(
                completion[-1], precision[-1, 0], color=color, marker="o", label=method_name, linestyle="none"
            )

        axs[0, 0].set_ylabel("precision (cm)")
        axs[0, 0].set_xlabel("completion (%)")
        axs[0, 0].legend()
        axs[1, 0].set_ylabel("precision (cm)")
        axs[1, 0].set_xlabel("completion (%)")

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir / f"visualization_two_{eval_mode}.png",
                dpi=fig.dpi,
                bbox_inches="tight",
            )
        # plt.show()
        plt.close(fig)

    def __visualize_three(self, methods_results, colors):
        eval_mode = self._params["mode"]
        # Sort method according to precision and completion.
        unordered_methods = {}
        for method_name, (p, c) in methods_results.items():
            precision = np.full_like(p, np.nan)
            precision[:, 0] = p[:, 0] * 100.0  # m to cm
            precision[:, 1] = np.rad2deg(p[:, 1])  # radian to degree
            completion = np.copy(c)
            weighted_precision = precision / completion[:, None]
            weighted_precision[-1, :] = np.nanmean(weighted_precision[:-1, :], axis=0)
            unordered_methods[method_name] = (weighted_precision, completion)

        ordered_methods = {
            name: val for name, val in sorted(unordered_methods.items(), key=lambda x: (-x[1][1][-1], x[1][0][-1][0]))
        }

        # Do the plot.
        max_pre = np.zeros((2))
        fig, axs = plt.subplots(2, 1, sharex=True)
        methods_positions = np.arange(len(ordered_methods.keys()))
        for method_index, (method_name, (precision, c)) in enumerate(ordered_methods.items()):
            # precision = np.full_like(p, np.nan)
            # precision[:, 0] = p[:, 0] * 100.0 # m to cm
            # precision[:, 1] = np.rad2deg(p[:, 1]) # radian to degree
            completion = np.copy(c)
            # weighted_precision = precision / completion[:, None]
            # outlier_indices = np.where(completion[:-1] < 1.0)[0].astype(int)
            completion *= 100.0  # to percent
            xy = precision[:-1, 0]
            theta = precision[:-1, 1]
            max_pre = np.fmax(max_pre, [np.nanmax(xy), np.nanmax(theta)])
            color = colors[method_name]
            axs[0].boxplot(
                xy[~np.isnan(xy)],
                positions=[methods_positions[method_index]],
                showmeans=True,
                patch_artist=True,
                boxprops=dict(facecolor=color, color=color),
                capprops=dict(color=color),
                whiskerprops=dict(color=color),
                flierprops=dict(color=color, markeredgecolor=color),
                medianprops=dict(color=color),
            )

            axs[1].boxplot(
                theta[~np.isnan(theta)],
                positions=[methods_positions[method_index]],
                showmeans=True,
                patch_artist=True,
                boxprops=dict(facecolor=color, color=color),
                capprops=dict(color=color),
                whiskerprops=dict(color=color),
                flierprops=dict(color=color, markeredgecolor=color),
                medianprops=dict(color=color),
            )

        # xticks = range(1, len(self._params) + 1)
        # for m, method_pos in zip(data[:, :-2], xticks): # skip the average and completeness
        #     plt.boxplot(m[~np.isnan(m)] * 100.0, positions=[method_pos], showmeans=True, patch_artist=True)
        # plt.ylabel("Precision (cm)")
        # # plt.ylim([0, 25])
        # plt.xticks(range(1, len(self._params["methods"]) + 1), self._params["methods"], rotation=45)
        # plt.tight_layout()
        # plt.savefig(f"precision_loop_{loop_index}")
        grids = [self._grid_xy, self._grid_theta]
        for i in range(2):
            # if max_pre[i] > grids[i]:
            yticks = np.arange(grids[i], max_pre[i], grids[i])
            ylabels = np.arange(0, max_pre[i], self._grid_res[i])
            for val in yticks:
                axs[i].plot(
                    [-0.5, methods_positions[-1] + 0.5], [val, val], color="k", linestyle="dotted", linewidth=0.5
                )
            axs[i].set_yticks(ylabels)
            axs[i].set_yticklabels(ylabels)

        axs[0].set_title("position (cm)")
        # axs[0].set_xticks(range(len(ordered_methods.keys())))
        # axs[0].set_xticklabels(ordered_methods.keys(), rotation=45)
        fig.supylabel("precision / completeness")
        # axs[0].set_ylabel("precision / completeness")
        axs[1].set_title("orientation (degree)")
        # axs[1].set_ylabel("precision / completeness")
        axs[1].set_xticks(range(len(ordered_methods.keys())))
        axs[1].set_xticklabels(ordered_methods.keys(), rotation=45, ha="right")
        plt.tight_layout()

        if self._params["save_figs"]:
            fig.savefig(
                fname=self._output_dir / f"visualization_three_{eval_mode}.png",
                dpi=fig.dpi,
                bbox_inches="tight",
            )

        plt.show()

    def __visualize_table(self, method_results):
        """_summary_

        Args:
            method_results (_type_): _description_
        """
        # Path to save the file
        output_file = str(self._output_dir / f"{self._params['mode']}_result_table.txt")
        output_file_xy = str(self._output_dir / f"{self._params['mode']}_result_table_xy.txt")
        output_file_theta = str(self._output_dir / f"{self._params['mode']}_result_table_theta.txt")
        # Open the file to write
        with open(output_file, "w") as file, open(output_file_xy, "w") as file_xy, open(
            output_file_theta, "w"
        ) as file_theta:
            for method, (precision, completion) in method_results.items():
                scaled_precision = precision[:-1, :] * self._scales  # Skip the last average point.
                # Initialize an empty list to store the formatted strings
                formatted_points = []
                formatted_xys = []
                formatted_thetas = []

                for point in scaled_precision:
                    if np.isnan(point[0]):
                        formatted_point = "-"
                        formatted_xy = "-"
                        formatted_theta = "-"
                    else:
                        # Format each point as "position/orientation"
                        formatted_point = f"{point[0]:.1f}({point[1]:.1f})"
                        formatted_xy = f"{point[0]:.1f}"
                        formatted_theta = f"{point[1]:.1f}"
                    formatted_points.append(formatted_point)
                    formatted_xys.append(formatted_xy)
                    formatted_thetas.append(formatted_theta)

                # Calculate the average position and orientation
                avg_position = np.nanmean(scaled_precision[:, 0])
                avg_orientation = np.nanmean(scaled_precision[:, 1])
                avg_formatted = f"{avg_position:.1f}({avg_orientation:.1f})"

                # Add the average to the row data
                formatted_points.append(avg_formatted)
                formatted_xys.append(f"{avg_position:.1f}")
                formatted_thetas.append(f"{avg_orientation:.1f}")

                # Join all formatted points with a space separator and write them as a row
                file.write(f"{method} " + " ".join(formatted_points) + "\n")
                file_xy.write(f"{method} " + " ".join(formatted_xys) + "\n")
                file_theta.write(f"{method} " + " ".join(formatted_thetas) + "\n")

        print(f"Data saved to {output_file}")
