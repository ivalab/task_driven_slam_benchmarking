#!/bin/bash

# This script is to run dsol in a separate terminal.

ROS_WS=$1
ROS_WS=/home/yanwei/catkin_ws/
TRACK_LOG_PATH=$2
CELL_SIZE=$3 # feature_num = image_(w or h) / cell_size
SLAM_POSE_TOPIC=$4
LAUNCH_FILE_NAME=gazebo_DSOL_stereo.launch # RW: dsol_d435i.launch

# Source ros workspace.
cd $ROS_WS
source devel/setup.bash

# Launch dsol.
echo "Launching DSOL."
roslaunch dsol $LAUNCH_FILE_NAME save:=$TRACK_LOG_PATH slam_pose_topic:=$SLAM_POSE_TOPIC
#  cell_size:=10
#  min_log_level:=0 log:=100
