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

import os
from typing import Dict

from closedloop_nav_slam.utils.node_module import NodeBase

from .path_definitions import SLAM_CONFIGS_PATH


# Gazebo odometry source node.
class PerfectOdometryNode(NodeBase):
    """Create perfect odometry node.

    Args:
        NodeBase (_type_): _description_
    """

    def __init__(self, params: Dict):
        nodes = ["map_to_odom_publisher"]
        super().__init__("PerfectOdometry", nodes, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam perfect_odometry.launch"


class RobotOdometryNode(NodeBase):
    """Create Robot Odometry Node.

    Args:
        NodeBase (_type_): _description_
    """

    def __init__(self, params: Dict):
        nodes = ["slam_map_to_odom_publisher"]
        super().__init__("RobotOdometry", nodes, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam robot_odometry.launch"


class WheelOdometryNode(NodeBase):
    """Create Wheel Odcometry Node.

    Args:
        NodeBase (_type_): _description_
    """

    def __init__(self, params: Dict):
        nodes = ["wheel_odometry_publisher"]
        super().__init__("WheelOdometry", nodes, params)

    def compose_start_cmd(self) -> str:
        return "rosrun closedloop_nav_slam wheel_odometry_publisher.py"


# 2D Laser SLAM
class SlamToolboxNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["slam_toolbox"]
        super().__init__("SlamToolbox", nodes, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping output_pose_topic:="
            + self._params["et_pose_topic"]
        )


class HectorSlamNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["hector_mapping"]
        super().__init__("HectorSLAM", nodes, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch hector_mapping nav_slam_test.launch output_pose_topic:=" + self._params["et_pose_topic"]


class AmclNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["amcl", "slam_map_server"]
        super().__init__("AMCL", nodes, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam amcl.launch output_pose_topic:=" + self._params["et_pose_topic"]


class GmappingNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["gmapping", "tf_to_pose_converter"]
        super().__init__("Gmapping", nodes, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam gmapping.launch output_pose_topic:=" + self._params["et_pose_topic"]


class GroundTruthSlamNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["slam_toolbox"]
        super().__init__("GroundTruth", nodes, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch slam_toolbox nav_slam_test.launch"
            + " mode:=mapping map_frame:=gt_slam_map map_name:=/gt_slam_map"
            + " invert_tf:=true transform_publish_period:=0.5"
            + " output_pose_topic:="
            + self._params["gt_pose_topic"]
        )


# Visual SLAM.
class GfggNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["visual_slam", "Stereo"]
        super().__init__("GF-GG", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "closedloop_ws")
        TRACK_LOG_DIR = self._params["path_dir"]
        cmd = (
            "bash "
            + str(SLAM_CONFIGS_PATH / "call_gfgg.sh ")
            + ROS_WS
            + " "
            + TRACK_LOG_DIR
            + " "
            + str(self._params["good_feature_num"])
            + " "
            + self._params["et_pose_topic"]
        )
        return cmd


class Orb3Node(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["visual_slam", "Stereo"]
        super().__init__("ORB3", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "closedloop_ws")
        TRACK_LOG_DIR = self._params["path_dir"]
        cmd = (
            "bash "
            + str(SLAM_CONFIGS_PATH / "call_orb3.sh ")
            + ROS_WS
            + " "
            + TRACK_LOG_DIR
            + " "
            + str(self._params["feature_num"])
            + " "
            + self._params["et_pose_topic"]
        )
        return cmd


class MsckfNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["msckf/vio", "msckf/image_processor"]
        super().__init__("MSCKF", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "svo_ws")
        TRACK_LOG_DIR = self._params["path_dir"]
        cmd = (
            "bash "
            + str(SLAM_CONFIGS_PATH / "call_msckf.sh ")
            + ROS_WS
            + " "
            + TRACK_LOG_DIR
            + " "
            + str(self._params["feature_num"])
            + " "
            + self._params["et_pose_topic"]
        )
        return cmd


class DsolNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["dsol_odom"]
        super().__init__("DSOL", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "turtlebot_ws")
        TRACK_LOG_DIR = self._params["path_dir"]
        cmd = (
            "bash "
            + str(SLAM_CONFIGS_PATH / "call_dsol.sh ")
            + ROS_WS
            + " "
            + TRACK_LOG_DIR
            + " "
            + str(self._params["cell_size"])
            + " "
            + self._params["et_pose_topic"]
        )
        return cmd


class SvoNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["svo"]
        super().__init__("SVO", nodes, params)

    def compose_start_cmd(self) -> str:
        # ROS_WS = os.path.join(os.environ["HOME"], "svo_ws")
        DATASET_NAME = "dummy_dataset"
        TRACK_LOG_DIR = self._params["path_dir"]
        cmd = (
            "bash "
            + str(SLAM_CONFIGS_PATH / "call_svo.sh ")
            + str(self._params["grid_size"])
            + " "
            + DATASET_NAME
            + " "
            + TRACK_LOG_DIR
            + " "
            + self._params["et_pose_topic"]
        )
        return cmd


# Fusion node.
class MsfNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = ["msf_pose_sensor", "odometry_converter", "visual_robot_publisher"]
        super().__init__("MSF", nodes, params)

    def compose_start_cmd(self) -> str:
        return (
            "roslaunch closedloop_nav_slam msf.launch"
            + " slam_sensor_type:="
            + self._params["slam_sensor_type"]
            + " slam_pose_topic:="
            + self._params["et_pose_topic"]
            + " source_msg_parent_frame:="
            + self._params["source_msg_parent_frame"]
            + " source_msg_child_frame:="
            + self._params["source_msg_child_frame"]
        )


# 3D Lidar SLAM
class HdlSlamNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = [
            "hdl_graph_slam_nodelet",
            "prefiltering_nodelet",
            "scan_matching_odometry_nodelet",
            "velodyne_nodelet_manager",
        ]
        super().__init__("HDL-SLAM", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "catkin_ws")
        cmd = "bash " + str(SLAM_CONFIGS_PATH / "call_hdl_slam.sh ") + ROS_WS + " " + self._params["et_pose_topic"]
        return cmd


class FastLio2Node(NodeBase):
    def __init__(self, params: Dict):
        nodes = [
            "laserMapping",
        ]
        super().__init__("FAST-LIO2", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "catkin_ws")
        cmd = "bash " + str(SLAM_CONFIGS_PATH / "call_fast_lio2.sh ") + ROS_WS + " " + self._params["et_pose_topic"]
        return cmd


class LiorfNode(NodeBase):
    def __init__(self, params: Dict):
        nodes = [
            "liorf_imageProjection",
            "liorf_imuPreintegration",
            "liorf_mapOptmization",
        ]
        super().__init__("LIO-SAM", nodes, params)

    def compose_start_cmd(self) -> str:
        ROS_WS = os.path.join(os.environ["HOME"], "catkin_ws")
        cmd = "bash " + str(SLAM_CONFIGS_PATH / "call_liorf.sh ") + ROS_WS + " " + self._params["et_pose_topic"]
        return cmd


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
    elif "svo" == slam_method:
        return SvoNode(params)
    elif "perfect_odometry" == slam_method:
        return PerfectOdometryNode(params)
    elif "robot_odometry" == slam_method:
        return RobotOdometryNode(params)
    elif "hdl_slam" == slam_method:
        return HdlSlamNode(params)
    elif "fast_lio2" == slam_method:
        return FastLio2Node(params)
    elif "liorf" == slam_method:
        return LiorfNode(params)
    else:
        print(f"{slam_method} NOT DEFINED!")
        raise RuntimeError
    return None
