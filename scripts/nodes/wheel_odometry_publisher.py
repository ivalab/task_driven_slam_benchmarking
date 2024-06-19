#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file wheel_odometry_publisher.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-14-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from copy import deepcopy

import numpy as np

#! /usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

class WheelOdometryPublisher1:
    """
    Wheel Odometry Publisher.
    The implementation is from https://blog.lxsang.me/post/id/16.0.
    The math theory locates https://docs.mrpt.org/reference/latest/tutorial-motion-models.html
    """
    def __init__(self):
        self._alphas = [0.05, 0.001, 0.05, 0.05]  # https://docs.mrpt.org/reference/latest/tutorial-motion-models.html
        self._last_odom = None
        self._last_disturbed_pose = None  # [x, y, theta]

        rospy.init_node("wheel_odometry_publisher_node")

        # ROS params.
        self._new_odom_frame = rospy.get_param("new_odom_frame", "new_odom")
        self._base_frame = rospy.get_param("base_frame", "new_base_footprint")
        self._enable_odom_to_base_tf = rospy.get_param("enable_odom_to_base_tf", True)
        # Load noise parameters, default no noise.
        # self._alphas = [rospy.get_param("alpha"+str(v), 0.0) for v in range(4)]

        # ROS subscriber and publisher.
        self._old_odom_sub = rospy.Subscriber("/odom_sparse", Odometry, self.__odom_callback)
        self._new_odom_pub = rospy.Publisher("/visual/odom", Odometry, queue_size=1)

        # Tf broadcaster
        self._tf_br = tf.TransformBroadcaster()

        rospy.spin()

    def __odom_callback(self, msg):
        if self._last_odom is None:
            self._last_odom = msg
            self._last_disturbed_pose = self.__convert_odom_to_pose_2d(msg)
            return

        new_odom = self.__compute_new_odom(self._last_odom, msg)

        self._new_odom_pub.publish(new_odom)
        if self._enable_odom_to_base_tf:
            self._tf_br.sendTransform(
                (
                    new_odom.pose.pose.position.x,
                    new_odom.pose.pose.position.y,
                    new_odom.pose.pose.position.z,
                ),
                (
                    new_odom.pose.pose.orientation.x,
                    new_odom.pose.pose.orientation.y,
                    new_odom.pose.pose.orientation.z,
                    new_odom.pose.pose.orientation.w,
                ),
                new_odom.header.stamp,
                self._base_frame,
                self._new_odom_frame,
            )
        self._last_odom = msg

    def __convert_odom_to_pose_2d(self, odom):
        quat = [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ]
        return [
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            tf.transformations.euler_from_quaternion(quat)[-1],  # [roll, pitch, yaw]
        ]

    def __compute_new_odom(self, last_odom, cur_odom):
        theta1 = self.__convert_odom_to_pose_2d(last_odom)[-1]
        theta2 = self.__convert_odom_to_pose_2d(cur_odom)[-1]
        dx = cur_odom.pose.pose.position.x - last_odom.pose.pose.position.x
        dy = cur_odom.pose.pose.position.y - last_odom.pose.pose.position.y

        trans = np.sqrt(dx * dx + dy * dy)
        rot1 = np.arctan2(dy, dx) - theta1
        rot2 = theta2 - theta1 - rot1
        a1, a2, a3, a4 = self._alphas
        sd_rot1 = a1 * np.abs(rot1) + a2 * trans
        sd_rot2 = a1 * np.abs(rot2) + a2 * trans
        sd_trans = a3 * trans + a4 * (np.abs(rot1) + np.abs(rot2))

        trans += np.random.normal(0, sd_trans * sd_trans)
        rot1 += np.random.normal(0, sd_rot1 * sd_rot1)
        rot2 += np.random.normal(0, sd_rot2 * sd_rot2)

        self._last_disturbed_pose[0] += trans * np.cos(theta1 + rot1)
        self._last_disturbed_pose[1] += trans * np.sin(theta1 + rot1)
        self._last_disturbed_pose[2] += rot1 + rot2

        new_odom = deepcopy(cur_odom)
        new_odom.pose.pose.position.x = self._last_disturbed_pose[0]
        new_odom.pose.pose.position.y = self._last_disturbed_pose[1]
        quat = tf.transformations.quaternion_from_euler(0, 0, self._last_disturbed_pose[-1])
        new_odom.pose.pose.orientation.x = quat[0]
        new_odom.pose.pose.orientation.y = quat[1]
        new_odom.pose.pose.orientation.z = quat[2]
        new_odom.pose.pose.orientation.w = quat[3]
        return new_odom


class WheelOdometryPublisher:
    """
    Wheel Odometry Publisher.

    Algorithm Flow:
    - Read odometry from ground truth source (gazebo).
    - Compute the relative transformation between two consecutive frames.
    - Add gaussian noise to the relative transformation.
    - Re-compute the next frame pose.
    """
    def __init__(self):
        rospy.init_node("wheel_odometry_publisher")

        # Model parameters.
        self._last_wTb = None
        self._last_disturbed_wTb = None
        self._trans_err = 0.01 # m/m
        self._rot_err = np.deg2rad(0.01) # deg/m

        # ROS params.
        self._new_odom_frame = rospy.get_param("new_odom_frame", "odom")
        self._base_frame = rospy.get_param("base_frame", "base_footprint")
        self._enable_odom_to_base_tf = rospy.get_param("enable_odom_to_base_tf", True)
        self._new_odom_topic = rospy.get_param("new_odom_topic", "/visual/odom")
        self._odom_topic = rospy.get_param("odom_topic", "/odom_sparse")

        # ROS subscriber and publisher.
        self._old_odom_sub = rospy.Subscriber(self._odom_topic, Odometry, self.__odom_callback)
        self._new_odom_pub = rospy.Publisher(self._new_odom_topic, Odometry, queue_size=1)

        # Tf broadcaster
        self._tf_br = tf.TransformBroadcaster()

        rospy.spin()

    def __convert_odom_to_mat(self, odom):
        mTb = np.eye(4)
        mTb[:3, :3] = Rotation.from_quat([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,]
        ).as_matrix()
        mTb[:3, 3] = [
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ]
        return mTb

    def __convert_mat_to_pose(self, mat):
        pose = Pose()
        pose.position.x = mat[0, 3]
        pose.position.y = mat[1, 3]
        pose.position.z = mat[2, 3]
        quat = Rotation.from_matrix(mat[:3, :3]).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def __apply_noise(self, delta_T):
        dist = np.linalg.norm(delta_T[:3, 3])
        trans_sigma = dist * self._trans_err
        theta_sigma = dist * self._rot_err
        mat = np.eye(4)
        mat[0, 3] = np.random.normal(0.0, trans_sigma)
        mat[1, 3] = np.random.normal(0.0, trans_sigma)
        mat[:3, :3] = Rotation.from_euler("z", theta_sigma).as_matrix()
        return mat

    def __odom_callback(self, msg):
        if self._last_wTb is None:
            self._last_wTb = self.__convert_odom_to_mat(msg)
            self._last_disturbed_wTb = deepcopy(self._last_wTb)
            return

        # Compute relative T
        wTb = self.__convert_odom_to_mat(msg)
        delta_T = np.linalg.inv(self._last_wTb) @ wTb

        # Apply noise
        err_T = np.eye(4)
        if np.abs(msg.twist.twist.linear.x) > 1e-3 or np.abs(msg.twist.twist.angular.z) > 1e-3:
            err_T = self.__apply_noise(delta_T)
        disturbed_wTb = self._last_disturbed_wTb @ delta_T @ err_T

        new_odom = deepcopy(msg)
        new_odom.pose.pose = self.__convert_mat_to_pose(disturbed_wTb)

        self._new_odom_pub.publish(new_odom)
        if self._enable_odom_to_base_tf:
            self._tf_br.sendTransform(
                (
                    new_odom.pose.pose.position.x,
                    new_odom.pose.pose.position.y,
                    new_odom.pose.pose.position.z,
                ),
                (
                    new_odom.pose.pose.orientation.x,
                    new_odom.pose.pose.orientation.y,
                    new_odom.pose.pose.orientation.z,
                    new_odom.pose.pose.orientation.w,
                ),
                new_odom.header.stamp,
                self._base_frame,
                self._new_odom_frame,
            )
        self._last_wTb = wTb
        self._last_disturbed_wTb = disturbed_wTb

def main():
    """Call main func"""
    try:
        wop = WheelOdometryPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("WheelOdometryPublisher: exception caught, the node must be terminated.")

if __name__ == "__main__":
    main()
