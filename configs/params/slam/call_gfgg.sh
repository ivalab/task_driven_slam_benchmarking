#!/bin/bash

# This script is to run dsol in a separate terminal.

# Source ros workspace.
ROS_WS=$1
ROS_WS=/home/yanwei/roboslam_ws/
TRACK_LOG_PATH=$2
GOOD_FEATURE_NUM=$3
SLAM_POSE_TOPIC=$4
LAUNCH_FILE_NAME=gazebo_GF_stereo.launch # RW: GF_d435i.launch

cd $ROS_WS
source devel/setup.bash
cd src/gf_orb_slam2/Examples/ROS/GF_ORB_SLAM2/launch

# Launch GFGG.
echo "Launching GFGG."
roslaunch $LAUNCH_FILE_NAME path_slam_config:=$ROS_WS/src/ORB_Data/ path_track_logging:=$TRACK_LOG_PATH do_rectify:=false num_good_feature:=$GOOD_FEATURE_NUM slam_pose_topic:=$SLAM_POSE_TOPIC
