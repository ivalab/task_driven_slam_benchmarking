#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file utils.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 05-22-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


import logging
from pathlib import Path
from typing import Tuple, List
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

import numpy as np
from evo.core import metrics, sync
from evo.core.result import Result as EvoResult
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.main_ape import ape as evo_ape
from evo.main_rpe import rpe as evo_rpe
from evo.tools import file_interface, plot


def yaw_from_quaternion(quat: np.ndarray) -> float:
    """Extract yaw from quaternion (x, y, z, w) in range[-pi, pi]."""
    assert quat.shape == (4,) or quat.shape == (4, 1) or quat.shape == (1, 4)
    rot = Rotation.from_quat(quat).as_matrix()
    return np.arctan2(rot[1, 0], rot[0, 0])


def pose_array_to_evo_trajectory(mat: np.ndarray) -> PoseTrajectory3D:
    """_summary_

    Args:
        mat (np.ndarray): _description_

    Returns:
        PoseTrajectory3D: _description_
    """
    stamps = mat[:, 0]  # n x 1
    xyz = mat[:, 1:4]  # n x 3
    quat = mat[:, 4:8]  # n x 4
    quat = np.roll(quat, 1, axis=1)  # shift 1 column right -> w in first column
    return PoseTrajectory3D(xyz, quat, stamps)


def evo_trajectory_to_pose_array(traj: PoseTrajectory3D) -> np.ndarray:
    """_summary_

    Args:
        traj (PoseTrajectory3D): _description_

    Returns:
        np.ndarray: _description_
    """
    quat = np.roll(traj.orientations_quat_wxyz, -1, axis=1)  # shift 1 column left -> w in last column
    return np.column_stack([traj.timestamps, traj.positions_xyz, quat])


def run_evo_evaluation(gt_poses: np.ndarray, et_poses: np.ndarray):
    """_summary_

    Args:
        gt_poses (np.ndarray): ground truth poses with each row as [timestamp, x, y, z, qx, qy, qz, qw]
        et_poses (np.ndarray): estimate poses with each row as [timestamp, x, y, z, qx, qy, qz, qw]

    Returns:
        _type_: _description_
    """
    assert gt_poses.shape[0] > 0 and et_poses.shape[0] > 0
    assert gt_poses.shape[1] == 8 and et_poses.shape[1] == 8

    traj_act = pose_array_to_evo_trajectory(gt_poses)
    traj_est = pose_array_to_evo_trajectory(et_poses)

    # Align the coordinate and associate the trajectories.
    traj_act, traj_est = sync.associate_trajectories(traj_act, traj_est)
    traj_est.align(traj_act)

    # Call APE
    logging.info("Evaluate APE ...")
    ape_result = evo_ape(traj_act, traj_est, pose_relation=metrics.PoseRelation.translation_part)
    ape_result_rot = evo_ape(traj_act, traj_est, pose_relation=metrics.PoseRelation.rotation_angle_rad)

    # Query result.
    rmse = ape_result.stats["rmse"]
    trans_errs = ape_result.np_arrays["error_array"]
    timestamps = ape_result.np_arrays["timestamps"]
    rot_errs = ape_result_rot.np_arrays["error_array"]
    assert trans_errs.shape == rot_errs.shape
    return rmse, timestamps, trans_errs, rot_errs
    # stamped_errors = [
    #     [timestamp, error]
    #     for timestamp, error in zip(ape_result.np_arrays["timestamps"], ape_result.np_arrays["error_array"])
    # ]
    # return np.array(stamped_errors)


def align_trajectories(target_poses: np.ndarray, source_poses: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """_summary_

    Args:
        target_poses (np.ndarray): _description_
        source_poses (np.ndarray): _description_

    Returns:
        Tuple[np.ndarray, np.ndarray]: _description_
    """

    assert target_poses.shape[0] > 0 and target_poses.shape[0] > 0
    assert source_poses.shape[1] == 8 and source_poses.shape[1] == 8

    traj_tgt = pose_array_to_evo_trajectory(target_poses)
    traj_src = pose_array_to_evo_trajectory(source_poses)

    # Align the coordinate and associate the trajectories.
    traj_tgt, traj_src = sync.associate_trajectories(traj_tgt, traj_src)
    traj_tgt.align(traj_src)

    output_tgt = evo_trajectory_to_pose_array(traj_tgt)
    output_src = evo_trajectory_to_pose_array(traj_src)
    return output_tgt, output_src


def draw_grids(ax, grid_value: float, xrange: List[float], yrange: List[float] = None):
    """_summary_

    Args:
        ax (_type_): _description_
        xrange (_type_): _description_
        yrange (_type_): _description_
        grid (_type_): _description_
    """
    # Draw grids.
    assert xrange
    assert len(xrange) == 2

    # Calculate the grid lines for x-axis
    x_grid_lines = np.arange(
        np.floor(xrange[0] / grid_value) * grid_value,
        np.ceil(xrange[1] / grid_value) * grid_value + grid_value,
        grid_value,
    )
    xrange = x_grid_lines[[0, -1]]

    if yrange is None:
        yrange = xrange

    # Calculate the grid lines for y-axis
    y_grid_lines = np.arange(
        np.floor(yrange[0] / grid_value) * grid_value,
        np.ceil(yrange[1] / grid_value) * grid_value + grid_value,
        grid_value,
    )
    yrange = y_grid_lines[[0, -1]]

    # Draw grid lines for x-axis
    for x in x_grid_lines:
        ax.plot([x, x], yrange, color="grey", linestyle="--", linewidth=0.5)

    # Draw grid lines for y-axis
    for y in y_grid_lines:
        ax.plot(xrange, [y, y], color="grey", linestyle="dotted", linewidth=0.5)

    ax.set_xticks(x_grid_lines)
    ax.set_xticklabels(x_grid_lines)
    ax.set_yticks(y_grid_lines)
    ax.set_yticklabels(y_grid_lines)
    ax.set_xlim(xrange)
    ax.set_ylim(yrange)


def draw_circles(ax, radius: float, max_value: float, enable_xy_ticks=True):
    # Check for valid input values
    if radius <= 0 or max_value <= 0:
        raise ValueError("Both radius and max_value must be positive numbers.")

    # Initialize the current radius
    current_radius = radius * 2

    # Draw circles incrementally until reaching max_value
    while current_radius <= max_value:
        circle = plt.Circle((0, 0), current_radius, color="gray", fill=False, linestyle="dashed", linewidth=0.5)
        ax.add_artist(circle)
        current_radius += radius * 2

    if not enable_xy_ticks:
        return
    current_radius -=  2 * radius
    # minor_ticks = np.arange(-current_radius, current_radius + radius, radius)
    major_ticks = np.arange(-current_radius, current_radius + 1 * radius, radius * 2.0)

    ax.set_xticks(major_ticks)
    # ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    # ax.set_yticks(minor_ticks, minor=True)

    # Set the labels for the ticks
    ax.set_xticklabels([f"{tick:.1f}" for tick in major_ticks])
    ax.set_yticklabels([f"{tick:.1f}" for tick in major_ticks])

    # Add grid lines
    # ax.grid(True, which="both", linestyle="--", linewidth=0.5)


def draw_x_eq_y(ax, min_xy: float, max_xy: float, delta_xy: float = 0.5):
    """_summary_

    Args:
        ax (_type_): _description_
        min_xy (_type_): _description_
        max_xy (_type_): _description_
        delta_xy (int, optional): _description_. Defaults to 1.
    """
    xys = np.arange(min_xy, max_xy + delta_xy, delta_xy)
    ax.plot(xys, xys, color="r", linestyle="dashed", linewidth=2.0)


def draw_wpt_position_errs(ax, xy_errs: np.ndarray, color, markersize=5, marker="o", unit="cm", label=None):
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
    elif xy_errs.shape[0] == 2:
        ax.plot(
            xy_errs[:, 0],
            xy_errs[:, 1],
            color="green",
            linestyle="dashdot",
            linewidth=0.7,
            markeredgecolor="none",
        )
    return max_abs_xy_err


def draw_wpt_orientation_errs(ax, theta_errs: np.ndarray, color, markersize=5, marker="o", unit="degree", label=None):
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


def draw_second_x_axis(ax, step, ticklabel, axislabel):
    """_summary_

    Args:
        ax (_type_): _description_
        step (_type_): _description_
        ticklabel (_type_): _description_

    Returns:
        _type_: _description_
    """
    xlims = ax.get_xlim()
    ticks = np.arange(xlims[0], xlims[1], step)
    ax.set_xticks(ticks)
    ax.set_xticklabels(ticks)
    ax2 = ax.secondary_xaxis("bottom")  # Create the second X-axis at the bottom
    ax2.set_xlim(xlims)  # Match the limits of the first x-axis
    ax2.set_xticks(ticks)
    ax2.set_xticklabels([f"{i}{ticklabel}" for i in range(len(ticks))])  # Example: Double the original ticks
    ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
    ax2.set_xlabel(axislabel)
    return ax2


def draw_second_y_axis(ax, step, ticklabel, axislabel):
    """_summary_

    Args:
        ax (_type_): _description_
        step (_type_): _description_
        ticklabel (_type_): _description_

    Returns:
        _type_: _description_
    """
    ylims = ax.get_ylim()
    ticks = np.arange(ylims[0], ylims[1], step)
    ax.set_yticks(ticks)
    ax.set_yticklabels(ticks)
    ax2 = ax.secondary_yaxis("right")  # Create the second Y-axis to the right
    ax2.set_ylim(ylims)  # Match the limits of the first x-axis
    ax2.set_yticks(ticks)
    ax2.set_yticklabels([f"{i}{ticklabel}" for i in range(len(ticks))])  # Example: Double the original ticks
    # ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
    ax2.set_ylabel(axislabel)
    return ax2


def draw_second_axis(ax, step, unit_label, xaxis_label="", yaxis_label=""):
    # Draw second axis.
    if len(xaxis_label) > 0:
        xlims = ax.get_xlim()
        xticks = np.arange(xlims[0], xlims[1] * 1.01, step)
        ax_x = ax.secondary_xaxis("top")
        ax_x.set_xticks(xticks)
        ax_x.set_xticklabels([f"{i}{unit_label}" for i in range(len(xticks))])
        # ax_x.spines["bottom"].set_position(("outward", 40))
        ax_x.set_xlabel(xaxis_label, fontsize=18)  # , fontsize=18

    if len(yaxis_label) > 0:
        ylims = ax.get_ylim()
        yticks = np.arange(ylims[0], ylims[1] * 1.01, step)
        ax_y = ax.secondary_yaxis("right")
        ax_y.set_yticks(yticks)
        ax_y.set_yticklabels([f"{i}{unit_label}" for i in range(len(yticks))])
        ax_y.set_ylabel(yaxis_label, fontsize=18)  # , fontsize=18)


def color_violin_plot_stats_lines(ax):
    """_summary_, coloring the mean, median, 75%, 25% lines.

    Args:
        ax (_type_): _description_
    """
    for l in ax.lines:
        l.set_linestyle("--")
        l.set_linewidth(1)
        l.set_color("black")
        # l.set_alpha(0.8)
        l.set_alpha(1.0)
    for l in ax.lines[1::3]:
        l.set_linestyle("-")
        l.set_linewidth(2.5)
        l.set_color("lightgreen")
        l.set_alpha(0.8)


def draw_horizon_grids(ax, step, ylabel, ymin=None, ymax=None):
    """_summary_

    Args:
        ax (_type_): _description_
        step (_type_): _description_

    Returns:
        _type_: _description_
    """
    xlims = ax.get_xlim()
    ylims = ax.get_ylim()
    if ymin is None:
        ymin = ylims[0]
    if ymax is None:
        ymax = ylims[-1]
    yticks = np.arange(ymin, ymax * 1.01, step)
    ax.set_yticks(yticks)
    ax.set_yticklabels(yticks)
    for yy in yticks[1:]:
        ax.plot(xlims, [yy, yy], color="grey", linestyle="--", linewidth=0.5)

    ax2 = ax.secondary_yaxis("right")  # Create the second Y-axis to the right
    ax2.set_ylim(ylims)  # Match the limits of the first x-axis
    ax2.set_yticks(yticks)
    ax2.set_yticklabels([f"{i}" for i in range(len(yticks))])  # Example: Double the original ticks
    # ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
    ax2.set_ylabel(ylabel)
    return ax2


def draw_horizon_grids_orientation(ax, step, ylable, ymin=None, ymax=None):
    """_summary_

    Args:
        ax (_type_): _description_
        step (_type_): _description_

    Returns:
        _type_: _description_
    """
    xlims = ax.get_xlim()
    ylims = ax.get_ylim()
    if ymin is None:
        ymin = ylims[0]
    if ymax is None:
        ymax = ylims[-1]
    yticks = np.arange(ymin, ymax * 1.01, step)
    ax.set_yticks(yticks)
    ax.set_yticklabels(yticks)
    for yy in yticks[1:]:
        # ax.plot(xlims, [yy, yy], color="grey", linestyle="--", linewidth=0.5)
        plt.axhline(yy, color="grey", linestyle="--", linewidth=0.5)

    ax2 = ax.secondary_yaxis("right")  # Create the second Y-axis to the right
    ax2.set_ylim(ylims)  # Match the limits of the first x-axis
    ax2.set_yticks(yticks)
    ax2.set_yticklabels([f"{i}/16" for i in range(len(yticks))])  # Example: Double the original ticks
    # ax2.spines["bottom"].set_position(("outward", 40))  # Place the second axis even lower
    ax2.set_ylabel(ylable)
    return ax2


def save_figs(fig, filename: str):
    """_summary_

    Args:
        fig (_type_): _description_
        filename (str): _description_
    """
    filename = str(filename)
    fig.savefig(fname=filename + ".png", dpi=fig.dpi, bbox_inches="tight")
    fig.savefig(fname=filename + ".pdf", dpi=fig.dpi, bbox_inches="tight")


def setup_logger(logging_dir: str):
    """Setup logger"""

    # Create log dir if not exists
    logging_dir_path = Path(logging_dir)
    logging_dir_path.mkdir(parents=True, exist_ok=True)

    # Setup logger.
    # Logger cookbook: https://docs.python.org/3/howto/logging-cookbook.html
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)-12s %(levelname)-8s %(message)s",
        datefmt="%m-%d %H:%M",
        filename=str(logging_dir_path / "evaluation.log"),
        filemode="w",
    )

    # Setup also console logging.
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter("%(name)-12s: %(levelname)-8s %(message)s")
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger("").addHandler(console)
