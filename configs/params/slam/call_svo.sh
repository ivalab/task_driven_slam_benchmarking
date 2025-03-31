#!/bin/bash

# This script is to run svo in a separate terminal.

# Source ros workspace.
cd ~/svo_ws
source ~/svo_ws/devel/setup.bash
# LAUNCH_FILE_NAME=gazebo_stereo_only.launch # d435i_stereo_imu.launch
LAUNCH_FILE_NAME=d435i_stereo_imu.launch

# Launch svo with arguments $(num_feature) $(dataset) $(dir).
LAUNCH_FILE="$LAUNCH_FILE_NAME grid_size:=$1 dataset:=$2 trace_dir:=$3 slam_pose_topic:=$4"
echo $LAUNCH_FILE
roslaunch svo_ros $LAUNCH_FILE
