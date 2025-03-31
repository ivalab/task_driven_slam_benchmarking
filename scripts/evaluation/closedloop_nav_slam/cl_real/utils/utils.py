#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file utils.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 04-29-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


import glob
from pathlib import Path

import cv2
import numpy as np
import yaml
from cl_real.utils.path_definitions import CONFIG_PATH


def extract_image_filenames(filenames, stamps):
    """return filenames bounded by start and end timestamp"""
    result = []
    for filename in filenames:
        timestamp = float(Path(filename).stem)
        if timestamp > stamps[0] and timestamp < stamps[1]:
            result.append(filename)
    return result


def read_calibration_batch(filenames):
    named_camera_info = dict()
    for filename in filenames:
        named_camera_info[Path(filename).stem] = read_calibration(filename)
    return named_camera_info


def load_stamped_methods(params):
    stamped_methods = dict()
    for method_name in params["methods"]:
        robot_odom_files = sorted(glob.glob(str(params["data_dir"] / method_name / params["path"] / "*")))
        start_timestamp = np.loadtxt(Path(robot_odom_files[0]) / "robot_odoms.txt", ndmin=2)[0, 0] - 5.0
        end_timestamp = np.loadtxt(Path(robot_odom_files[-1]) / "robot_odoms.txt", ndmin=2)[-1, 0] + 5.0
        stamped_methods[method_name] = [start_timestamp, end_timestamp]
    return stamped_methods


def load_evaluation(params):
    results = {}
    for method_name in params["methods"]:
        filename = params["data_dir"] / "evaluation" / (method_name + "_waypoints.npy")
        if not filename.is_file():
            print(f"{method_name} wpts file does NOT exist, skip.")
            continue
        results[method_name] = np.load(filename)
    return results


def read_calibration(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    camera_info = dict()
    # Parse
    camera_info["width"] = calib_data["image_width"]
    camera_info["height"] = calib_data["image_height"]
    camera_info["K"] = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)
    camera_info["D"] = np.array(calib_data["distortion_coefficients"]["data"])
    camera_info["R"] = np.array(calib_data["rectification_matrix"]["data"])
    camera_info["P"] = np.array(calib_data["projection_matrix"]["data"])
    camera_info["distortion_model"] = calib_data["distortion_model"]
    return camera_info


def undistort_image(image, camera_info):
    undistorted = cv2.undistort(image, camera_info["K"], camera_info["D"])
    return undistorted


def load_params(filename: Path):
    """load parameters"""
    params = None
    with open(filename, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    assert params
    # Load path file
    path_filename = CONFIG_PATH / (params["path"] + ".yaml")
    with open(path_filename, encoding="utf-8") as f:
        params.update(yaml.safe_load(f))
    return params
