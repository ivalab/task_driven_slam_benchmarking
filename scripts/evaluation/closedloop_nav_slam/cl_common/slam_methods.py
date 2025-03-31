#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file slam_methods.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 04-29-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

from dataclasses import dataclass


@dataclass
class MethodMetaData:
    """_summary_"""

    name: str
    label: str
    sensor: str
    marker: str
    color: str
    line: str


SENSOR_TO_MARKER = {
    "lidar2d": "o",
    "camera": "s",
    "lidar3d": "^",
}


METHODS_VISUALS = {
    "slam_toolbox": MethodMetaData(
        name="slam_toolbox",
        label="SLAM-Toolbox",
        sensor="lidar2d",
        marker=SENSOR_TO_MARKER["lidar2d"],
        color="tab:blue",
        line="solid",
    ),
    "hector_slam": MethodMetaData(
        name="hector_slam",
        label="HectorSLAM",
        sensor=SENSOR_TO_MARKER["lidar2d"],
        marker="o",
        color="tab:orange",
        line="dashed",
    ),
    "gfgg": MethodMetaData(
        name="gfgg",
        label="GF-GG",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="tab:green",
        line="solid",
    ),
    "orb3": MethodMetaData(
        name="orb3",
        label="ORB_SLAM3",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="tab:red",
        line="dashed",
    ),
    "orb3_new": MethodMetaData(
        name="orb3_new",
        label="ORB_SLAM3",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="tab:red",
        line="dashed",
    ),
    "dsol": MethodMetaData(
        name="dsol",
        label="DSOL",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="tab:purple",
        line="dashdot",
    ),
    "svo": MethodMetaData(
        name="svo",
        label="SVO-Pro",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="magenta",
        line="dotted",
    ),
    "msckf": MethodMetaData(
        name="msckf",
        label="MSCKF",
        sensor="camera",
        marker=SENSOR_TO_MARKER["camera"],
        color="lime",
        line="dashed",
    ),
    "fast_lio2": MethodMetaData(
        name="fast_lio2",
        label="FAST-LIO2",
        sensor="lidar3d",
        marker=SENSOR_TO_MARKER["lidar3d"],
        color="tab:gray",
        line="solid",
    ),
    "liorf": MethodMetaData(
        name="liorf",
        label="LIO-SAM",
        sensor="lidar3d",
        marker=SENSOR_TO_MARKER["lidar3d"],
        color="tab:olive",
        line="dashed",
    ),
}


@dataclass
class AxisVisualMetaData:
    state: str = ""
    metric: str = ""
    unit: str = ""
    td_unit: str = ""
    symbol: str = ""


AXIS_VISUALS = {
    "Position Accuracy": AxisVisualMetaData("Position", "Accuracy", "cm", "D", "$ e $"),
    "Position Precision": AxisVisualMetaData("Position", "Precision", "cm", "D", "$ E $"),
    "Orientation Accuracy": AxisVisualMetaData("Orientation", "Accuracy", "degree", "FoV", "$ \delta $"),
    "Orientation Precision": AxisVisualMetaData("Orientation", "Precision", "degree", "FoV", "$ \Delta $"),
}
