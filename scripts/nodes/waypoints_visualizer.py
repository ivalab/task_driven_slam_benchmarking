#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file waypoints_saver.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""


from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, Pose


class WaypointsVisualizer:
    def __init__(self, waypoints_file, path_file):
        rospy.init_node("waypoints_visualizer_node")
        self._pose_array_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self._waypoints = []
        self._pose_array = PoseArray()
        self.__load_waypoints(waypoints_file, path_file)
        self.__convert_wpts_to_pose_array()

    def run(self):

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self._pose_array.header.stamp = rospy.Time.now()
            self._pose_array.header.frame_id = "map"
            self._pose_array_pub.publish(self._pose_array)
            r.sleep()

    def __load_waypoints(self, waypoints_file, path_file):
        indices = np.loadtxt(path_file, dtype=np.int8)
        self._waypoints = np.loadtxt(waypoints_file, dtype=np.float, ndmin=2)[indices, 1:]
        print(self._waypoints)

    def __convert_wpts_to_pose_array(self):
        for wpt in self._waypoints:
            p = Pose()
            p.position.x = wpt[0]
            p.position.y = wpt[1]
            p.orientation.w = np.cos(wpt[2] / 2.0)
            p.orientation.z = np.sin(wpt[2] / 2.0)
            self._pose_array.poses.append(p)

def main():
    try:
        waypoints_file = "/home/roboslam/turtlebot_ws/src/closedloop_nav_slam/configs/path/realworld/waypoints.txt"
        path_file = "/home/roboslam/turtlebot_ws/src/closedloop_nav_slam/configs/path/realworld/path1.txt"
        wv = WaypointsVisualizer(waypoints_file, path_file)
        wv.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints saver exception caught !!!")

if __name__ == "__main__":
    main()