#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file evaluation.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 09-14-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.core import metrics, sync
from evo.tools import file_interface, plot
from evo.main_rpe import rpe
from evo.main_ape import ape
from evo.core.result import Result


from dataclasses import dataclass
import numpy as np
import os
from typing import Dict, List

from matplotlib import pyplot as plt


# def run_evo_evaluation(act_filename, est_filename) -> Tuple[Result, Result, np.ndarray, np.ndarray]:

#     traj_ref = file_interface.read_tum_trajectory_file(act_filename)
#     traj_method = file_interface.read_tum_trajectory_file(est_filename)

#     traj_ref, traj_method = sync.associate_trajectories(traj_ref, traj_method)
#     traj_method.align(traj_ref, correct_scale=False)
#     traj_method.align_origin(traj_ref)

#     rpe_result = rpe(
#         traj_ref,
#         traj_method,
#         pose_relation=metrics.PoseRelation.translation_part,
#         delta=0.5,
#         delta_unit=metrics.Unit.meters,
#         all_pairs=True,
#     )
#     ape_result = ape(traj_ref, traj_method, pose_relation=metrics.PoseRelation.translation_part)

#     return ape_result, rpe_result, traj_ref.positions_xyz, traj_method.positions_xyz


def convertToEvoFormat(mat: np.ndarray) -> PoseTrajectory3D:
    stamps = mat[:, 0]  # n x 1
    xyz = mat[:, 1:4]  # n x 3
    quat = mat[:, 4:8]  # n x 4
    quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
    return PoseTrajectory3D(xyz, quat, stamps)


def findWaypoints(act_odoms: np.ndarray) -> list:
    """The robot stops at each waypoint for a short duration (twist = 0)"""
    idle_time = 1.0  # seconds

    # skip the start point
    idx = 0
    for odo in act_odoms:
        pos = np.linalg.norm(odo[1:4])
        idx += 1
        if pos > 0:
            break

    linear_vel = np.linalg.norm(act_odoms[idx:, 8:11], axis=-1)
    angular_vel = np.linalg.norm(act_odoms[idx:, 11:], axis=-1)
    indices = np.argwhere((linear_vel == 0) & (angular_vel == 0))
    # print(indices)
    return []


def compute_trajectory_length(positions: np.ndarray) -> float:
    assert positions.shape[1] == 3
    assert positions.shape[0] > 1
    return np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))


# Find unique vertices
def compute_vertex_indices(points: np.ndarray) -> Dict[int, List]:
    vertices = [points[0, :]]
    clusters: Dict[int, List] = {0: []}
    for i in range(points.shape[0]):
        pt = points[i, :]
        dists = np.linalg.norm(pt - vertices, axis=-1)
        min_dist = np.min(dists)
        if min_dist < 1e-3:
            index = np.argmin(dists)
            clusters[index].append(i)
        else:
            clusters[len(vertices)] = [i]
            vertices = np.append(vertices, [pt], axis=0)
    return clusters


@dataclass
class MethodData:
    """Class for book keeping results"""

    name: str
    planned_xys: np.ndarray  # Lx2, x y
    act_xys: np.ndarray  # Lx2, x y
    act_poses: np.ndarray  # Mx8, timestamp x y z qx qy qz qw
    est_poses: np.ndarray  # Nx8, timestamp x y z qx qy qz qw
    nav_errs: np.ndarray  # Lx1 in meters
    nav_rmse: float  # in meters
    est_rmse: float  # in meters
    traj_length: float  # in meters
    wpt_indices: np.ndarray  # Kx1, unique waypiont wpt_indices
    round_num: int  # round number, L = K * round_num
    indexed_clusters: Dict[int, list]  # each element stores waypoint index and associated act pos as key and value

    def __init__(
        self,
        name,
        planned_xys,
        act_xys,
        act_poses,
        est_poses,
        nav_errs,
        nav_rmse,
        est_rmse,
        traj_length,
        wpt_indices,
        round_num,
        indexed_clusters,
    ):
        self.name = name
        self.planned_xys = planned_xys
        self.act_xys = act_xys
        self.act_poses = act_poses
        self.est_poses = est_poses
        self.nav_errs = nav_errs
        self.nav_rmse = nav_rmse
        self.est_rmse = est_rmse
        self.traj_length = traj_length
        self.wpt_indices = wpt_indices
        self.round_num = round_num
        self.indexed_clusters = indexed_clusters

        assert self.planned_xys.shape[0] > 0
        assert self.planned_xys.shape[1] == 2
        assert self.act_xys.shape[0] > 0
        assert self.act_xys.shape[1] == 2
        assert self.act_poses.shape[0] > 1
        assert self.act_poses.shape[1] == 8
        assert self.est_poses.shape[0] > 1
        assert self.est_poses.shape[1] == 8


METHOD_TYPES = {
    "perfect_odometry": "lidar_loc",
    "amcl": "lidar_loc",
    "hector_mapping": "lidar_slam",
    "slam_toolbox_mapping": "lidar_slam",
    "slam_toolbox_localization": "lidar_loc",
    "dsol": "camera_odom",
    "gfgg": "camera_slam",
    "orb3": "camera_slam",
}

################################################################################
################################################################################

RESULT_PREFIX = "/mnt/DATA/rosbags/cl_test"
METHOD_NAMES = [
    # "perfect_odometry",
    # "amcl",
    # "hector_mapping",
    # "slam_toolbox_mapping",
    # "slam_toolbox_localization",
    # "dsol",
    "gfgg",
    # "orb3"
]
PATH_FILES = ["path0", "path1", "path2", "path3"]
PLOT_ALL = False
REPEATABILITY = 5


for pfile in PATH_FILES:

    print(f"Processing {pfile}")

    methods: list = []
    box_ylim = 0.0

    name = "turtlebot_" + pfile

    for method_name in METHOD_NAMES:

        method_type = METHOD_TYPES[method_name]
        print(f"Processing {method_name}, {method_type}")

        result_dir = os.path.join(RESULT_PREFIX, method_name)

        act_odoms = np.loadtxt(os.path.join(result_dir, name + "_gt_odoms.txt"))
        est_poses = np.loadtxt(os.path.join(result_dir, name + "_et_poses.txt"))
        # planned_path = np.loadtxt("/mnt/DATA/rosbags/turtlebot_planned_path.txt")
        stamped_planned_path = np.loadtxt(os.path.join(result_dir, name + "_stamped_planned_path.txt"))

        clusters = compute_vertex_indices(stamped_planned_path[:, 1:4])
        # print(clusters)

        # Looking for actual poses correspond to the waypoints.
        planned_xys = []
        actual_xys = []
        for stamped_pose in stamped_planned_path:
            timediff = np.abs(act_odoms[:, 0] - stamped_pose[0])
            value = np.min(timediff)
            assert value <= 1e-2  # seconds
            index = np.argmin(timediff)
            planned_xys.append(stamped_pose[1:3])
            actual_xys.append(act_odoms[index, 1:3])

        planned_xys = np.array(planned_xys)
        actual_xys = np.array(actual_xys)
        nav_errs = np.linalg.norm(planned_xys - actual_xys, axis=1)
        nav_rmse = np.sqrt(np.mean(nav_errs**2))
        nav_errs = nav_errs * 100.0  # Convert m to cm.
        # print(f"nav_rmse: {nav_rmse:.2f}")

        traj_ref = convertToEvoFormat(act_odoms)
        traj_method = convertToEvoFormat(est_poses)

        # Start EVO APIs.
        traj_ref, traj_method = sync.associate_trajectories(traj_ref, traj_method)
        if "camera" in method_type:
            transformation = traj_method.align(traj_ref, correct_scale=False)
            print(f"Aligned transformation: {transformation}")

        # traj_method.align_origin(traj_ref)
        # Call APE
        ape_result = ape(traj_ref, traj_method, pose_relation=metrics.PoseRelation.translation_part)
        traj_length = compute_trajectory_length(act_odoms[:, 1:4])  # traj_method.path_length
        est_rmse = ape_result.stats["rmse"]
        # print(f"traj_length: {traj_length:.2f} m, RMSE: {est_rmse:.2f} m")

        plt.figure()
        # Plot trajectory.
        if "camera" in method_type:
            plt.plot(traj_ref.positions_xyz[:, 0], traj_ref.positions_xyz[:, 1], color="g", label="ground truth")
            plt.plot(traj_method.positions_xyz[:, 0], traj_method.positions_xyz[:, 1], color="r", label="estimation")
        else:
            plt.plot(act_odoms[:, 1], act_odoms[:, 2], color="g", label="ground truth")
            plt.plot(est_poses[:, 1], est_poses[:, 2], color="r", label="estimation")
        plt.plot(
            act_odoms[0, 1],
            act_odoms[0, 2],
            color="g",
            marker="*",
            markersize=12,
            linestyle="None",
            label="start point",
        )
        # Plot waypoints.
        for index, _ in clusters.items():
            pt = stamped_planned_path[index, 1:3]
            plt.plot(pt[0], pt[1], color="y", marker="o", linestyle="None")
            plt.text(pt[0], pt[1], "wpt " + str(index))
        plt.title(
            f"{method_name} \n trajectory length {traj_length:.2f} m, est rmse {est_rmse:.2f} m, nav rmse {nav_rmse:.2f} m"
        )
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.legend()
        plt.savefig(fname=os.path.join(result_dir, name + "_trajectory.png"))

        # Box plot.
        plt.figure()
        waypoints_nav_errors = []
        rounds = 0
        wpts_count = 0
        for keys, indices in clusters.items():
            waypoints_nav_errors.append(nav_errs[indices])
            wpts_count += 1
            rounds = max(rounds, len(waypoints_nav_errors[-1]))
        # print(f"Wpt Count: {wpts_count}, Rounds:{rounds}")

        # append method
        methods.append(
            MethodData(
                method_name,
                planned_xys,
                actual_xys,
                act_odoms[:, :8],
                est_poses,
                nav_errs,
                nav_rmse,
                est_rmse,
                traj_length,
                clusters.keys(),
                rounds,
                clusters,
            )
        )

        xticks_pos = range(wpts_count)
        xticks = ["wpt " + str(i) for i in range(wpts_count)]
        max_ylim = max(100.0, np.max(nav_errs) * 1.2)
        fig, axs = plt.subplots(2, 1)
        axs[0].boxplot(waypoints_nav_errors, showmeans=True)
        axs[0].set_xticks(1 + np.array(xticks_pos))
        axs[0].set_xticklabels(xticks)
        axs[0].set_ylabel("Navigation Error (cm)")
        axs[0].set_title(f"{method_name} Repeatability Test (Loop = {REPEATABILITY})")
        axs[0].set_ylim([0.0, max_ylim])

        for col in range(rounds):
            values = []
            for row in range(wpts_count):
                if len(waypoints_nav_errors[row]) <= col:
                    break
                values.append(waypoints_nav_errors[row][col])
            axs[1].plot(range(len(values)), values, label=f"loop {col}")
        axs[1].legend()
        axs[1].set_xticks(xticks_pos)
        axs[1].set_xticklabels(xticks)
        axs[1].set_ylabel("Navigation Error (cm)")
        axs[1].set_ylim([0.0, max_ylim])
        plt.savefig(fname=os.path.join(result_dir, name + "_waypoints_errors.png"))

        # plt.show()
        box_ylim = max(box_ylim, max_ylim)

        print(f"Done {method_name}")

    if not PLOT_ALL:
        continue

    # plot everything on one figure.
    # https://matplotlib.org/stable/gallery/color/named_colors.html
    colors = ["lime", "darkorange", "maroon", "darkcyan", "cyan", "violet", "blue"]
    markers = ["*", "+", "^", "o", "s", "<", "p"]
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))
    # plot trajectory
    box_positions = []
    box_width = 0.5
    for index, m in enumerate(methods):

        # plot planned path
        if index == 0:
            wpts = m.planned_xys[list(m.wpt_indices), :]
            for i, pt in enumerate(wpts):
                axs[0].plot(pt[0], pt[1], marker="o", markersize=10, color="g", linestyle="None")
                axs[0].text(pt[0], pt[1], "wpt " + str(i))

                # axs[1].plot(pt[0], pt[1], marker="o", markersize=10, color="g", linestyle="None")
                # axs[1].text(pt[0], pt[1], "wpt " + str(i))
            box_positions = (np.arange(0, len(m.wpt_indices), 1) + 1) * box_width * len(methods) * 2

        # plot robot actual path
        axs[0].plot(m.act_poses[:, 1], m.act_poses[:, 2], color=colors[index], label=m.name, linewidth=1.0)

        # plot boxplot
        waypoint_errors: list = []
        waypoint_stds: list = []
        for key, indices in m.indexed_clusters.items():
            waypoint_errors.append(m.nav_errs[indices])
            rsi = 1 if "slam" in method_type else 0  # skip first mapping round
            waypoint_stds.append(np.std(m.nav_errs[indices][rsi:]))
        print(len(waypoint_errors))
        box = axs[1].boxplot(
            waypoint_errors,
            positions=box_positions[: len(waypoint_errors)] + (box_width + box_width / 5.0) * index,
            widths=[box_width] * len(waypoint_errors),
            patch_artist=True,
            showmeans=True,
        )
        axs[2].plot(
            box_positions[: len(waypoint_errors)] + (box_width + box_width / 5.0) * index,
            waypoint_stds,
            color=colors[index],
            marker=markers[index],
            markersize=10,
            linestyle="None",
        )
        for wpt_index, wpt_err in enumerate(waypoint_errors):
            if len(wpt_err) < REPEATABILITY:
                ratio = len(wpt_err) / REPEATABILITY
                xx = box_positions[wpt_index] + (box_width * 1.5) * index - box_width / 2.0
                axs[1].text(xx, max(wpt_err), f"{(ratio):.2f}")
        for item in ["boxes", "whiskers", "fliers", "medians", "caps"]:
            plt.setp(box[item], color=colors[index])
            plt.setp(box["boxes"], facecolor=colors[index])
            plt.setp(box["fliers"], markeredgecolor=colors[index])

    # plot
    axs[0].set_xlabel("x(m)")
    axs[0].set_ylabel("y(m)")
    axs[0].set_title("robot trajectory")
    axs[0].legend()
    axs[1].set_xticks(box_positions + box_width * len(methods) / 2.0)
    axs[1].set_xticklabels(["wpt " + str(i) for i in range(len(box_positions))])
    axs[1].set_ylim([0.0, box_ylim * 1.2])
    axs[1].set_ylabel("Navigation Error (cm)")
    # axs[2].set_ylim([0.0, box_ylim * 1.2])
    axs[2].set_xticks(box_positions + box_width * len(methods) / 2.0)
    axs[2].set_xticklabels(["wpt " + str(i) for i in range(len(box_positions))])
    axs[2].set_ylabel("Repeatability (cm)")
    plt.savefig(fname=os.path.join(RESULT_PREFIX, pfile + "_errors.png"))
    # plt.show()

    print(f"Done {pfile}")
