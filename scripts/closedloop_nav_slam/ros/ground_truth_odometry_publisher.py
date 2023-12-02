#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file ground_truth_odometry_publisher.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-01-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''

#! /usr/bin/env python

import time
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetWorldProperties
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import rospy

class GroundTruthOdometryPublisher:

    def __init__(self):

        rospy.init_node('ground_truth_odometry_publisher')
        self._odom_pub = rospy.Publisher('ground_truth/odom', Odometry, queue_size=10)

        rospy.wait_for_service('/gazebo/get_model_state')
        self._get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self._unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self._map_frame = rospy.get_param("map_frame", "map")
        self._base_frame = rospy.get_param("base_frame", "base_footprint")

        self._model = GetModelStateRequest()
        self._model.model_name="mobile_base"
        self._models = []

        # pause_timeout = time.time() + 4.0
        # while time.time() < pause_timeout:
        #     rospy.logwarn("Waiting %.2f seconds to unpause physics", pause_timeout - time.time())
        #     time.sleep(1.0)
        # self._unpause_physics()
        # rospy.loginfo("Physics unpaused.")

    def run(self):
        odom = Odometry()
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame

        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            if self._model.model_name not in self._models:
                self._models = self._get_world_properties().model_names
                rospy.logwarn("Waiting for %s to spawn to publish ground truth odometry", self._model.model_name)
            else:
                result = self._get_model_srv(self._model)
                odom.pose.pose = result.pose
                odom.twist.twist = result.twist
                odom.header.stamp = rospy.Time.now()
                self._odom_pub.publish(odom)
            rate.sleep()

if __name__ == "__main__":
    try:
        gt_publisher = GroundTruthOdometryPublisher()
        gt_publisher.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("Exception Caught!")
