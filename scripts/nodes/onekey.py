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

import signal
import sys
import subprocess
import time
from pathlib import Path

import rospy
import yaml
from nav_msgs.msg import Path as PathMsg

from closedloop_nav_slam.utils.node_module import (
    MapToOdomPublisherNode,
    MoveBaseNode,
    WaypointsNavigatorNode,
)
from closedloop_nav_slam.utils.path_definitions import (
    PARAMS_PATH,
    SLAM_CONFIGS_PATH,
    UTILS_PATH,
    TOOLS_PATH,
)
from closedloop_nav_slam.utils.slam_module import (
    CreateSlamNode,
    GroundTruthSlamNode,
    MsfNode,
    WheelOdometryNode,
)


class RosNodeManager:
    """_summary_"""

    def __init__(self, config_file: Path):
        """_summary_

        Args:
            config_file (Path): _description_
        """
        # Load yaml.
        self._common_params = self.__load_params(config_file)
        assert self._common_params

        # Assure the trial for realworld test is 1, because location reset manually is required !!
        if "realworld" == self._common_params["test_type"]:
            rospy.loginfo("! Enforce trials to be 1 for realworld test, manual reset is REQUIRED!")
            self._common_params["trials"] = 1

        # Create result dir.
        self._prefix = (
            Path(self._common_params["result_dir"]) / self._common_params["test_type"] / self._common_params["env_name"]
        )
        self._prefix.mkdir(parents=True, exist_ok=True)

        # Ros node.
        rospy.init_node("onekey_node")
        # We used this message signal to determine if a path navigation has been completed.
        self._sub = rospy.Subscriber("/visited_waypoints", PathMsg, self.__path_callback)
        self._stop = False
        self._launched_nodes = []

    def __load_params(self, config_file: Path):
        params = None
        with open(config_file, "r") as file:
            params = yaml.safe_load(file)
        return params

    def __path_callback(self, msg):
        rospy.loginfo("Task Completion Message Received!")
        self._stop = True

    # def __set_move_base_params(self):
    #     if "move_base_name" not in self._common_params:
    #         return
    #     cmd_prefix = "rosparam set " + self._common_params["move_base_name"] + "/"
    #     for s in ["xy", "yaw"]:
    #         msg = s + "_goal_tolerance"
    #         if msg not in self._common_params:
    #             continue
    #         cmd = f"{cmd_prefix}{msg} {self._common_params[msg]}"
    #         rospy.loginfo(cmd)
    #         subprocess.call(cmd, shell=True)

    def __save_map(self, slam_name: str, path_name: str, trial: int):
        rospy.loginfo("Saving map ... ")
        filename = slam_name + "_" + path_name + "_" + "trial" + str(trial) + "_" + time.strftime("%Y%m%d-%H%M%S")
        cmd = f"rosrun map_server map_saver -f {filename} map:=/slam_map"
        print(cmd)
        subprocess.call(cmd, shell=True)

    def start(self):
        """_summary_"""
        # Register signal handler to catch Ctrl+C or system-level shutdown signals.
        signal.signal(signal.SIGINT, self.__shutdown)
        signal.signal(signal.SIGTERM, self.__shutdown)

        for method_index, method_name in enumerate(self._common_params["slam_methods"]):
            method_dir = self._prefix / method_name
            method_dir.mkdir(parents=True, exist_ok=True)
            params = self._common_params
            slam_params = self.__load_params(SLAM_CONFIGS_PATH / (method_name + ".yaml"))
            params.update(slam_params)
            for pfile in params["path_files"]:
                rospy.loginfo(f"Navigate path file: {pfile} ... ")

                for trial in range(params["trials"]):
                    rospy.loginfo(f"Start Trial: {trial}")

                    # Create postfix for realworld test to differentiate trials
                    postfix = "_" + time.strftime("%Y%m%d-%H%M%S") if "realworld" == params["test_type"] else ""
                    path_dir = method_dir / pfile / ("trial" + str(trial) + postfix)
                    path_dir.mkdir(parents=True, exist_ok=True)
                    # Update params to save other SLAM results.
                    params["path_dir"] = str(path_dir) + "/"

                    # - Assuming gazebo is already started in a separated window.

                    # = Start waypoint navigator.
                    rospy.loginfo("Start waypoints navigator ...")
                    wpt_nav_node = WaypointsNavigatorNode(params, pfile, str(path_dir))
                    wpt_nav_node.start()
                    self._launched_nodes.append(wpt_nav_node)
                    time.sleep(5.0)

                    # - Compose robot bash script
                    robot_bash_script = "bash " + str(TOOLS_PATH) + "/start_robot.sh"
                    # - Reset everything.
                    if "gazebo" == self._common_params["test_type"]:
                        rospy.loginfo("Reset everything ...")
                        cmd_reset_robot = robot_bash_script + " 1 1"  # BUTTON STATE
                        subprocess.Popen(cmd_reset_robot, shell=True)
                        time.sleep(10.0)

                    # - Start gt slam if enabled
                    gt_slam_node = None
                    if self._common_params["enable_gt_slam_method"]:
                        rospy.loginfo("Start GT SLAM Method.")
                        gt_slam_node = GroundTruthSlamNode(params)
                        gt_slam_node.start()
                        self._launched_nodes.append(gt_slam_node)
                        time.sleep(5.0)

                    # - Start slam
                    rospy.loginfo("Start SLAM ...")
                    slam_node = CreateSlamNode(params)
                    slam_node.start()
                    self._launched_nodes.append(slam_node)
                    time.sleep(5.0)

                    # - Start msf
                    msf_node = None
                    if params["enable_msf"]:
                        rospy.loginfo("Start MSF ...")
                        msf_node = MsfNode(params)
                        msf_node.start()
                        self._launched_nodes.append(msf_node)
                        time.sleep(10.0)
                    # elif "perfect_odometry" != method_name:
                    #     print("MSF disabled, start odometry converter node ...")
                    #     msf_node = OdometryConverterNode(params)
                    #     msf_node.start()
                    #     time.sleep(2.0)

                    # - Start map_to_odom_publisher if needed
                    m2o_tf_node = None
                    if params["need_map_to_odom_tf"]:
                        rospy.loginfo("Start map_to_odom_publisher")
                        m2o_tf_node = MapToOdomPublisherNode(params)
                        m2o_tf_node.start()
                        self._launched_nodes.append(m2o_tf_node)
                        time.sleep(3.0)

                    # - Start move_base
                    rospy.loginfo("Start move_base ...")
                    mb_node = MoveBaseNode(params)
                    mb_node.start()
                    self._launched_nodes.append(mb_node)
                    time.sleep(10.0)

                    # # - Set move_base params
                    # rospy.loginfo("Setting move_base params ...")
                    # self.__set_move_base_params()
                    # time.sleep(3.0)

                    # Start wheel odometry
                    wo_node = None
                    if params["enable_wheel_odometry"]:
                        rospy.loginfo("start wo ...")
                        wo_node = WheelOdometryNode(params)
                        wo_node.start()
                        self._launched_nodes.append(wo_node)
                        time.sleep(1.0)

                    # - Start robot
                    rospy.loginfo("Start the robot ...")
                    cmd_start_robot = robot_bash_script + " 0 1"  # BUTTON STATE
                    subprocess.Popen(cmd_start_robot, shell=True)

                    # Wait for the test to finish.
                    rate = rospy.Rate(5.0)
                    self._stop = False
                    while not rospy.is_shutdown():
                        if self._stop:
                            break
                        rate.sleep()

                    if not self._stop:
                        rospy.loginfo("Stop requested by user.")

                    # - Save map if requested
                    if self._common_params["save_map"]:
                        self.__save_map(method_name, pfile, trial)

                    # - Stop nodes.
                    self.__stop_all_nodes()
                    # if gt_slam_node:
                    #     rospy.loginfo("Killing gt slam method ...")
                    #     gt_slam_node.stop()
                    #     time.sleep(3)

                    # # - Stop navigation script
                    # rospy.loginfo("Killing waypoints_navigator ...")
                    # wpt_nav_node.stop()
                    # time.sleep(5)

                    # # - Stop wheel odometry node
                    # if wo_node:
                    #     rospy.loginfo("Killing wo ...")
                    #     wo_node.stop()
                    #     time.sleep(1)

                    # # - Stop move_base
                    # rospy.loginfo("Killing move_base ...")
                    # mb_node.stop()
                    # time.sleep(5)

                    # # - Killing SLAM
                    # rospy.loginfo("Killing SLAM ...")
                    # if m2o_tf_node:
                    #     m2o_tf_node.stop()
                    #     time.sleep(3.0)
                    # if msf_node:
                    #     msf_node.stop()
                    #     time.sleep(5)
                    # slam_node.stop()
                    # time.sleep(5)

                    rospy.loginfo(f"Done Trial {trial}")
                rospy.loginfo(f"Done {pfile}")
            rospy.loginfo(f"Done {method_name}")

    def __stop_all_nodes(self):
        """_summary_"""
        rospy.loginfo("Stopping all nodes ...")
        for node in self._launched_nodes:
            try:
                rospy.loginfo(f"Terminating {node.name()}...")
                node.stop()
                time.sleep(5)
            except Exception as e:
                rospy.logerr(f"Error terminating {node.name()}: {e}")

    def __shutdown(self, signum=None, frame=None):
        """_summary_

        Args:
            signum (_type_, optional): _description_. Defaults to None.
            frame (_type_, optional): _description_. Defaults to None.
        """
        rospy.loginfo("Shutdown request received... Terminating all launched nodes.")
        self.__stop_all_nodes()
        rospy.loginfo("All nodes terminated. Exiting.")
        rospy.signal_shutdown("Shutdown complete.")  # Signal shutdown to ROS
        sys.exit(0)  # Ensure the program exits


def main():
    """_summary_"""
    # Start manager.
    manager = RosNodeManager(PARAMS_PATH / "config.yaml")
    try:
        manager.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("RosNodeManager: exception caught !!!")


if __name__ == "__main__":
    # Run main.
    main()
