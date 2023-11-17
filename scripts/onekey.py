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
from nav_msgs.msg import Path as PathMsg

import yaml

import rospy

from modular.node_module import MoveBaseNode, WaypointsNavigatorNode, OdometryConverterNode
from modular.slam_module import SlamToolboxNode, MsfNode, CreateSlamNode

CONFIG_PREFIX = Path(__file__).parent.resolve()

class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
class CentralManager:

    def __init__(self, config_file: str):

        # Load yaml.
        self._params = None
        with open(config_file, "r") as file:
            self._params = yaml.safe_load(file)
        assert self._params

        # Create result dir.
        self._prefix = Path(self._params["result_dir"]) / self._params["slam_method"]
        self._prefix.mkdir(parents=True, exist_ok=True)

        # Ros node.
        rospy.init_node("onekey_node", anonymous=True)
        # We used this message signal to determine if a path has been completed.
        self._sub = rospy.Subscriber("/actual_path", PathMsg, self.__path_callback)
        self._stop = False

    def __path_callback(self, msg):
        rospy.loginfo("Task completion message received!")
        self._stop = True

    def run(self):

        for pfile in self._params["path_files"]:
            print(f"Navigate path file: {pfile} ... ")

            for trial in range(self._params["trials"]):

                print(f"Trial: {trial}")

                path_dir = self._prefix / pfile / ("trial" + str(trial))
                path_dir.mkdir(parents=True, exist_ok=True)

                # - Assuming gazebo is already started in a separated window.

                # = Start waypoint navigator.
                print("Start waypoints navigator ...")
                wpt_nav_node = WaypointsNavigatorNode(self._params, pfile, str(path_dir))
                wpt_nav_node.start()
                time.sleep(5.0)

                # - Reset everything.
                print("Reset everything ...")
                robot_bash_cmd = "bash " + str(CONFIG_PREFIX) + "/utils/start_robot.sh"
                cmd_reset_robot = robot_bash_cmd + " 1 1" # BUTTON STATE
                subprocess.Popen(cmd_reset_robot, shell=True)
                time.sleep(10.0)

                # - Start slam
                print("Start SLAM ...")
                slam_node = CreateSlamNode(self._params)
                slam_node.start()
                time.sleep(5.0)

                # - Start msf
                msf_node = None
                if self._params["enable_msf"]:
                    print("Start MSF ...")
                    msf_node = MsfNode(self._params)
                    msf_node.start()
                    time.sleep(10.0)
                elif "perfect_odometry" != self._params["slam_method"]:
                    print("MSF disabled, start odometry converter node ...")
                    msf_node = OdometryConverterNode(self._params)
                    msf_node.start()
                    time.sleep(2.0)

                # - Start move_base
                print("Start move_base ...")
                mb_node = MoveBaseNode(self._params)
                mb_node.start()
                time.sleep(5.0)

                # - Start robot
                print("Start the robot ...")
                cmd_start_robot = robot_bash_cmd + " 0 1" # BUTTON STATE
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

                ## Stop nodes.

                # - Stop navigation script
                print("Killing waypoints_navigator ...")
                wpt_nav_node.stop()
                time.sleep(5)

                # - Stop move_base
                print("Killing move_base ...")
                mb_node.stop()
                time.sleep(5)

                # - Killing SLAM
                print("Killing SLAM ...")
                if msf_node:
                    msf_node.stop()
                    time.sleep(5)

                slam_node.stop()
                time.sleep(5)

                print(f"Done {pfile}")

if __name__ == "__main__":

    try:
        cm = CentralManager(CONFIG_PREFIX / "config.yml")
        cm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("CM exception caught !!!")