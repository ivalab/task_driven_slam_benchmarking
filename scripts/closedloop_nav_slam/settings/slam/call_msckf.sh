#!/bin/bash

# This script is to run msckf in a separate terminal.

ROS_WS=$1
TRACK_LOG_PATH=$2
# FEATURE_NUM=$3
FEATURE_NUM=240
SLAM_POSE_TOPIC=$4

# Source ros workspace.
cd $ROS_WS
source devel/setup.bash

# Launch msckf with arguments $(num_feature) $(imu_type).
LAUNCH_FILE="msckf_vio_d435i_nav_slam.launch output_prefix:=$TRACK_LOG_PATH slam_pose_topic:=$SLAM_POSE_TOPIC"
echo $LAUNCH_FILE
roslaunch msckf_vio $LAUNCH_FILE
