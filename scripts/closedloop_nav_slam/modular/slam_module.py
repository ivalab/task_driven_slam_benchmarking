#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file slam_module.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-10-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

from typing import Dict

from .node_module import NodeBase


import os

from pathlib import Path

from closedloop_nav_slam.utils.path_definitions import *

# Gazebo odometry source node.
class PerfectOdometryNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["map_to_odom_publisher"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch closedloop_nav_slam perfect_odometry.launch"
        )

class RobotOdometryNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["slam_map_to_odom_publisher"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam robot_odometry.launch"

# 2D Laser SLAM
class SlamToolboxNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["slam_toolbox"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping output_pose_topic:="
            + self._params["et_pose_topic"]
        )

class HectorSlamNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["hector_mapping"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch hector_mapping nav_slam_test.launch output_pose_topic:="
            + self._params["et_pose_topic"]
        )

class AmclNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["amcl", "slam_map_server"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch closedloop_nav_slam amcl.launch output_pose_topic:="
            + self._params["et_pose_topic"]
        )

class GmappingNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["gmapping", "tf_to_pose_converter"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch closedloop_nav_slam gmapping.launch output_pose_topic:="
            + self._params["et_pose_topic"]
        )

class GroundTruthSlamNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["slam_toolbox"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        # @TODO (yanwei) what is the correct mode when used as ground truth?
        return (
            "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping output_pose_topic:="
            + self._params["gt_pose_topic"]
        )

# Visual SLAM.
class GfggNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["visual_slam", "Stereo"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "closedloop_ws")
        TRACK_LOG_DIR = os.path.join(os.environ["HOME"], "slam_ws/result/gf_orb_slam2/gazebo")
        cmd = "bash " + str(SLAM_SETTINGS_PATH / "call_gfgg.sh ") + ROS_WS + " " + TRACK_LOG_DIR + " " + str(self._params["good_feature_num"]) + " " + self._params["et_pose_topic"]
        return cmd

class Orb3Node(NodeBase):
    def __init__(self, params: Dict):
        names = ["visual_slam", "Stereo"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "closedloop_ws")
        TRACK_LOG_DIR = os.path.join(os.environ["HOME"], "slam_ws/result/orb3/gazebo")
        cmd = "bash " + str(SLAM_SETTINGS_PATH / "call_orb3.sh ") + ROS_WS + " " + TRACK_LOG_DIR + " " + str(self._params["feature_num"]) + " " + self._params["et_pose_topic"]
        return cmd

class MsckfNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["msckf/vio", "msckf/image_processor"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "svo_ws")
        TRACK_LOG_DIR = os.path.join(os.environ["HOME"], "slam_ws/result/msckf/gazebo")
        cmd = "bash " + str(SLAM_SETTINGS_PATH / "call_msckf.sh ") + ROS_WS + " " + TRACK_LOG_DIR + " " + str(self._params["feature_num"]) + " " + self._params["et_pose_topic"]
        return cmd

class DsolNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["dsol_odom"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "catkin_ws")
        TRACK_LOG_DIR = os.path.join(os.environ["HOME"], "slam_ws/result/dsol/gazebo")
        cmd = "bash " + str(SLAM_SETTINGS_PATH / "call_dsol.sh ") + ROS_WS + " " + TRACK_LOG_DIR + " " + str(self._params["cell_size"]) + " " + self._params["et_pose_topic"]
        return cmd

# Fusion node.
class MsfNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["msf_pose_sensor", "odometry_converter", "visual_robot_publisher"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return ("roslaunch closedloop_nav_slam msf.launch"
            + " slam_sensor_type:=" + self._params["slam_sensor_type"]
            + " slam_pose_topic:=" + self._params["et_pose_topic"]
            + " source_msg_parent_frame:=" + self._params["source_msg_parent_frame"]
            + " source_msg_child_frame:=" + self._params["source_msg_child_frame"]
        )

# Factory.
def CreateSlamNode(params: Dict) -> NodeBase:
    slam_method = params["slam_method"]
    if "slam_toolbox" == slam_method:
        return SlamToolboxNode(params)
    elif "hector_slam" == slam_method:
        return HectorSlamNode(params)
    elif "amcl" == slam_method:
        return AmclNode(params)
    elif "gmapping" == slam_method:
        return GmappingNode(params)
    elif "gfgg" == slam_method:
        return GfggNode(params)
    elif "orb3" == slam_method:
        return Orb3Node(params)
    elif "msckf" == slam_method:
        return MsckfNode(params)
    elif "dsol" == slam_method:
        return DsolNode(params)
    elif "perfect_odometry" == slam_method:
        return PerfectOdometryNode(params)
    elif "robot_odometry" == slam_method:
        return RobotOdometryNode(params)
    else:
        print(f"{slam_method} NOT DEFINED!")
        raise RuntimeError
    return None