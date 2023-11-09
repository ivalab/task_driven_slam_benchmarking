#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file onekey.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 09-17-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import os
from pathlib import Path
import subprocess
import time
import numpy as np


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


# Logging parameters.
RESULT_DIR = Path("/mnt/DATA/rosbags/cl_nav_slam_test/slam_toolbox")
RESULT_DIR.mkdir(exist_ok=True)

# Testing parameters.
env = "tsrb"
robot_init_pose = {
    "tsrb": "34.0 -6.0 0.0",
}
path_files = ["path0"]
trials = 1

# Utility function to estimate the navigation duration.
def estimate_duration(pfile: str) -> float:
    PATH_DIR = "/home/yanwei/turtlebot_ws/src/closedloop_nav_slam/configs/path/" + env
    vertices = np.loadtxt(os.path.join(PATH_DIR, "waypoints.txt"))
    path = np.loadtxt(os.path.join(PATH_DIR, pfile + ".txt"), dtype=int)

    xyz = vertices[path, 1:]
    xyz_diff = np.diff(xyz, axis=0)
    traj_length = np.sum(np.linalg.norm(xyz_diff, axis=1))
    speed = 0.3
    return traj_length / speed + path.shape[0] * 5.0


for pfile in path_files:
    print(f"Navigate path file: {pfile} ... ")

    for trial in range(trials):

        print(f"Trial: {trial}")

        path_dir = RESULT_DIR / pfile / ("trial" + str(trial))
        path_dir.mkdir(exist_ok=True)

        # Assuming gazebo is already started in a separated window.

        # 1. Start waypoint navigator.
        print("Start waypoint navigator ...")
        cmd_cl_test = (
            "roslaunch closedloop_nav_slam waypoints_navigator.launch" + " env:=" + env
            + " path_file:=" + pfile + ".txt"
            + " trials:=" + str(trials)
            + " output_dir:=" + str(path_dir)
        )
        print(cmd_cl_test)
        subprocess.Popen(cmd_cl_test, shell=True)
        time.sleep(5.0)

        # 2. Reset everything.
        print("Reset everything ...")
        cmd_reset_robot = "rostopic pub -1 /mobile_base/events/button kobuki_msgs/ButtonEvent '{button: 1, state: 1}'"
        print(cmd_reset_robot)
        subprocess.Popen(cmd_reset_robot, shell=True)
        time.sleep(10.0)

        # 3. Start slam
        print("Start SLAM ...")
        # cmd_slam = "roslaunch hector_mapping closedloop_nav_test.launch"
        cmd_slam = "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping"
        # cmd_slam = "bash closedloop/call_dsol.sh"
        # cmd_slam = "bash closedloop/call_gfgg.sh"
        print(cmd_slam)
        subprocess.Popen(cmd_slam, shell=True)
        time.sleep(5.0)

        # 3. Start msf
        print("Start MSF ...")
        cmd_msf = "roslaunch closedloop_nav_slam msf.launch"
        print(cmd_msf)
        subprocess.Popen(cmd_msf, shell=True)
        time.sleep(10.0)

        # 4. Start move_base
        print("Start move_base ...")
        cmd_nav = "roslaunch closedloop_nav_slam move_base.launch vis:=true"
        print(cmd_nav)
        subprocess.Popen(cmd_nav, shell=True)
        time.sleep(5.0)

        # 5. Start rosbag logging.
        # path_data_logging = RESULT_DIR / ("turtlebot_" + pfile)  # + "_round" + str(round_index))
        # cmd_rosbag = (
        #     "roslaunch turtlebot_roboslam data_logging.launch topics:='/odom_sparse /scanmatch/pose /planned_path /gfgg/pose' path_data_logging:="
        #     + str(path_data_logging)
        # )
        # print("Start rosbag logging ...")
        # print(cmd_rosbag)
        # subprocess.Popen(cmd_rosbag, shell=True)
        # time.sleep(5.0)

        # 6. Start robot
        print("Start the robot ...")
        cmd_start_robot = "rostopic pub -1 /mobile_base/events/button kobuki_msgs/ButtonEvent '{button: 0, state: 1}'"
        print(cmd_start_robot)
        subprocess.Popen(cmd_start_robot, shell=True)

        # 7. Idle to wait for the process.
        # (yanwei) Compute a reasonable waiting length or monitor a success signal.
        duration = estimate_duration(pfile)
        print(f"Idle for {duration:.2f} seconds ...")
        time.sleep(duration)

        # # Killing rosbag
        # cmd_kill_rosbag = "rosnode kill /data_logging"
        # print("Killing rosbag ...")
        # print(cmd_kill_rosbag)
        # subprocess.call(cmd_kill_rosbag, shell=True)
        # time.sleep(5)

        # Killing SLAM
        print("Killing SLAM ...")
        # subprocess.call("rosnode kill /dsol_odom", shell=True)
        subprocess.call("rosnode kill /slam_toolbox", shell=True)
        subprocess.call("rosnode kill /msf_pose_sensor", shell=True)
        subprocess.call("rosnode kill /odometry_converter", shell=True)
        subprocess.call("rosnode kill /visual_robot_publisher", shell=True)
        # subprocess.call("rosnode kill /map", shell=True)
        # subprocess.call("rosnode kill /map_nav_broadcaster", shell=True)
        time.sleep(5)

        # Killing Navigation
        print("Killing move_base ...")
        subprocess.call("rosnode kill  /move_base", shell=True)
        subprocess.call("rosnode kill  /navigation_velocity_smoother", shell=True)
        time.sleep(5)

        # Killing navigation script
        print("Killing waypoints_navigator ...")
        subprocess.call("rosnode kill /waypoints_navigator", shell=True)
        print(f"Done {pfile}")
        time.sleep(15)
