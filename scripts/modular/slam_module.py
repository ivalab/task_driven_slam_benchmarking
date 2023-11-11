#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file slam_module.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-10-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''

from .node_module import NodeBase

from typing import Dict

class SlamToolboxNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["slam_toolbox"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping output_pose_topic:=" + self._params["et_pose_topic"]

class GroundTruthSlamNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["slam_toolbox"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        # @TODO (yanwei) what is the correct mode when used as ground truth?
        return "roslaunch slam_toolbox nav_slam_test.launch mode:=mapping output_pose_topic:=" + self._params["gt_pose_topic"]

class MsfNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["msf_pose_sensor", "odometry_converter", "visual_robot_publisher"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return ("roslaunch closedloop_nav_slam msf.launch"
            + " slam_sensor_type:=" + self._params["slam_sensor_type"]
            + " slam_pose_topic:=" + self._params["et_pose_topic"]
        )


def CreateSlamNode(params: Dict) -> NodeBase:
    slam_method = params["slam_method"]
    if "slam_toolbox" == slam_method:
        return SlamToolboxNode(params)
    else:
        print(f"{slam_method} NOT DEFINED!")
        raise RuntimeError
    return None