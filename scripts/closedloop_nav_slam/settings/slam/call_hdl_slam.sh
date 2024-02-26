#!/bin/bash

# This script is to run dsol in a separate terminal.

# Source ros workspace.
ROS_WS=$1
SLAM_POSE_TOPIC=$2

cd $ROS_WS
source devel/setup.bash

# Launch hdl_slam.
echo "Launching hdl_slam."
roslaunch hdl_graph_slam nav_slam_test.launch slam_pose_topic:=$SLAM_POSE_TOPIC