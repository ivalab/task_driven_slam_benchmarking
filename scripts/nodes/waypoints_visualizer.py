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
import os

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class WaypointsVisualizer:
    """_summary_"""

    def __init__(self, waypoints_file, path_file, odom_file: str = ""):
        rospy.init_node("waypoints_visualizer_node")
        self._pose_array_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self._marker_pub = rospy.Publisher("/waypoints_indices", MarkerArray, queue_size=10)
        self._path_pub = rospy.Publisher("/robot_path", PathMsg, queue_size=10)
        self._pose_array = PoseArray()
        self._waypoints = []
        self._robot_odom = []
        self.__load(waypoints_file, path_file, odom_file)
        self.__convert_wpts_to_pose_array()
        self._marker_array = self.__plot_text(self._pose_array)

    def __load_robot_odom(self, odom_file):
        poses = np.loadtxt(odom_file)

        path = PathMsg()
        path.header = Header(stamp=rospy.Time.now(), frame_id="map")
        for row in poses:
            pose = PoseStamped()
            pose.header = Header(stamp=rospy.Time.from_sec(row[0]), frame_id="map")
            pose.pose.position.x = row[2]
            pose.pose.position.y = -row[1]
            pose.pose.position.z = row[3]
            pose.pose.orientation.x = row[4]
            pose.pose.orientation.y = row[5]
            pose.pose.orientation.z = row[6]
            pose.pose.orientation.w = row[7]
            path.poses.append(pose)

        return path

    def run(self):
        """_summary_"""

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self._pose_array.header.stamp = rospy.Time.now()
            self._pose_array.header.frame_id = "map"
            self._pose_array_pub.publish(self._pose_array)
            self._path_pub.publish(self._robot_odom)
            self._marker_pub.publish(self._marker_array)
            r.sleep()

    def __load(self, waypoints_file, path_file, odom_file):
        wpts = np.loadtxt(waypoints_file, dtype=float, ndmin=2)
        if os.path.exists(path_file):
            indices = np.loadtxt(path_file, dtype=np.int8)
        else:
            indices = range(wpts.shape[0])
        self._waypoints = wpts[indices, 1:]
        if len(odom_file) > 0:
            self._robot_odom = self.__load_robot_odom(odom_file)
        print("Waypoints:\n")
        print(self._waypoints)

    def __convert_wpts_to_pose_array(self):
        for wpt in self._waypoints:
            p = Pose()
            p.position.x = wpt[0]
            p.position.y = wpt[1]
            p.orientation.w = np.cos(wpt[2] / 2.0)
            p.orientation.z = np.sin(wpt[2] / 2.0)
            self._pose_array.poses.append(p)

    def __plot_text(self, pose_array_msg):
        marker_array = MarkerArray()
        for index, p in enumerate(pose_array_msg.poses):
            # Create the text marker
            text_marker = Marker()
            text_marker.header = pose_array_msg.header
            text_marker.ns = "waypoints"
            text_marker.id = index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.z = p.position.z
            text_marker.pose.position.x = p.position.x + 0.3  # Position the text above the arrow
            text_marker.pose.position.y = p.position.y + 0.3  # Position the text above the arrow
            text_marker.scale.z = 1.3  # Text size
            text_marker.color.a = 1.0  # Alpha (transparency)
            text_marker.color.r = 1.0  # Red
            text_marker.color.g = 1.0  # Green
            text_marker.color.b = 0.0  # Blue
            text_marker.text = str(index)  # The numeric value to display
            marker_array.markers.append(text_marker)
        return marker_array


def main():
    try:
        waypoints_file = "/home/yanwei/turtlebot_ws/src/closedloop_nav_slam/configs/path/realworld/waypoints.txt"
        path_file = "/home/yanwei/turtlebot_ws/src/closedloop_nav_slam/configs/path/realworld/path4.txt"
        odom_file = ""
        wv = WaypointsVisualizer(waypoints_file, path_file, odom_file)
        wv.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints saver exception caught !!!")


if __name__ == "__main__":
    main()
