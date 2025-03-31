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
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class WaypointsSaver:
    """_summary_"""

    def __init__(self):
        rospy.init_node("waypoints_saver_node")
        self._goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.__goal_callback)
        self._pose_array_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self._marker_pub = rospy.Publisher("/wapoints_indices", MarkerArray, queue_size=10)
        self._goals = []
        self._waypoints = []
        self._pose_array = PoseArray()
        self._marker_array = MarkerArray()
        rospy.spin()

    def __goal_callback(self, msg):
        self._goals.append(self.__convert_pose_to_vec(msg))
        self._waypoints.append(self.__convert_pose_to_xytheta(msg))
        self._marker_array.markers.append(self.__plot_text(msg, len(self._waypoints) - 1))

        self._pose_array.poses.append(msg.pose)
        self._pose_array.header = msg.header
        self._pose_array_pub.publish(self._pose_array)
        self._marker_pub.publish(self._marker_array)

    def __plot_text(self, msg, index):
        # Create the text marker
        text_marker = Marker()
        text_marker.header = msg.header
        text_marker.ns = "waypoints"
        text_marker.id = index
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = msg.pose.position.z
        text_marker.pose.position.x = msg.pose.position.x + 0.3  # Position the text above the arrow
        text_marker.pose.position.y = msg.pose.position.y + 0.3  # Position the text above the arrow
        text_marker.scale.z = 1.3  # Text size
        text_marker.color.a = 1.0  # Alpha (transparency)
        text_marker.color.r = 1.0  # Red
        text_marker.color.g = 1.0  # Green
        text_marker.color.b = 0.0  # Blue
        text_marker.text = str(index)  # The numeric value to display
        return text_marker

    def __convert_pose_to_vec(self, msg):
        return [
            msg.header.stamp.to_sec(),
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

    def __convert_pose_to_xytheta(self, msg):
        return [
            msg.pose.position.x,
            msg.pose.position.y,
            np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2.0,
        ]

    def save(self, prefix_path):
        """_summary_

        Args:
            prefix_path (_type_): _description_
        """
        np.savetxt(prefix_path / "goals.txt", self._goals, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
        np.savetxt(
            prefix_path / "waypoints.txt",
            np.column_stack([np.arange(len(self._waypoints)), self._waypoints]),
            fmt="%.6f",
            header="index x y theta",
        )
        rospy.loginfo(f"Saved waypoints to file: {prefix_path / 'waypoints.txt'}")


if __name__ == "__main__":
    try:
        ws = WaypointsSaver()
        ws.save(Path(__file__).resolve().parent)
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints saver exception caught !!!")
