#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file explore.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-13-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''

"""
This code is originally borrowed 
from https://github.com/YaelBenShalom/Turtlebot3-Navigation-with-SLAM/blob/master/nodes/explore.py
with our own modifications.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time

class Explore:

    def __init__(self):
        """ Initialize environment
        """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.goal = []
        self.completion = 0
        self.pose = []

        # Initialize subscribers:
        self.map_frame = rospy.get_param("map_frame", "map")
        self.map = OccupancyGrid()
        self.map_metadata = None
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.sub_map_metadata = rospy.Subscriber('/map_metadata', MapMetaData, self.map_metadata_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.count = 0
        time.sleep(8)


    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        if self.map_metadata is None:
            return

        valid = False

        while valid is False:
            map_size = randrange(len(data.data))
            self.map = data.data[map_size]

            edges = self.check_neighbors(data, map_size)
            if self.map != -1 and self.map <= 0.2 and edges is True:
                valid = True
            
        row = map_size / self.map_metadata.width
        col = map_size % self.map_metadata.width

        gx = col * self.map_metadata.resolution + self.map_metadata.origin.position.x  # column * resolution + origin_x
        gy = row * self.map_metadata.resolution + self.map_metadata.origin.position.y  # row * resolution + origin_x
        self.goal = [gx, gy]
        
        if self.completion % 2 == 0:
            self.completion += 1
            # Start the robot moving toward the goal
            self.set_goal()

    def map_metadata_callback(self, msg):
        self.map_metadata = msg

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def set_goal(self):
        """ Set goal position for move_base.
        """
        rospy.loginfo("Setting goal ... ")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal[0]
        goal.target_pose.pose.position.y = self.goal[1]
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)


    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal Succeeded.")

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal Aborted.")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal Rejected.")


    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * self.map_metadata.width + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False


def main():
    """ The main() function """
    rospy.init_node('exploration_node')
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ExplorationNode exception caught !!!")