#!/bin/bash

# This script is to run dsol in a separate terminal.

# Source ros workspace.
ROS_WS=$1
TRACK_LOG_PATH=$2
FEATURE_NUM=$3
SLAM_POSE_TOPIC=$4

cd $ROS_WS
source devel/setup.bash
source src/ORB_SLAM3/export_ros_path.bash
cd src/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/launch

# Launch GFGG.
echo "Launching ORB3."
roslaunch ORB3_d435i.launch path_slam_config:=$ROS_WS/src/ORB_Data/ path_track_logging:=$TRACK_LOG_PATH do_rectify:=false num_all_feature:=$FEATURE_NUM slam_pose_topic:=$SLAM_POSE_TOPIC
