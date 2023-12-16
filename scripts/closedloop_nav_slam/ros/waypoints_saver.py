#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file waypoints_saver.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''


from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
import numpy as np

class WaypointsSaver:

    def __init__(self):
        rospy.init_node("waypoints_saver_node")
        self._goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.__goal_callback)
        self._pose_array_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
        self._goals = []
        self._waypoints = []
        self._pose_array = PoseArray()
        rospy.spin()

    def __goal_callback(self, msg):
        self._goals.append(self.__convert_pose_to_vec(msg))
        self._waypoints.append(self.__convert_pose_to_xytheta(msg))

        self._pose_array.poses.append(msg.pose)
        self._pose_array.header = msg.header
        self._pose_array_pub.publish(self._pose_array)

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
        np.savetxt(prefix_path / "goals.txt", self._goals, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
        np.savetxt(prefix_path / "waypoints.txt", np.column_stack([np.arange(len(self._waypoints)), self._waypoints]), fmt="%.6f", header="index x y theta")
        rospy.loginfo(f"Saved waypoints to file: {prefix_path / 'waypoints.txt'}")


if __name__ == "__main__":
    try:
        ws = WaypointsSaver()
        ws.save(Path(__file__).resolve().parent)
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints saver exception caught !!!")