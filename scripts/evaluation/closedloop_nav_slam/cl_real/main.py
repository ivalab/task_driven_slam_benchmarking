#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file main.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 03-05-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


import glob
import logging
from pathlib import Path

from cl_real.utils.path_definitions import CONFIG_PATH
from cl_real.utils.utils import (
    load_evaluation,
    load_params,
    load_stamped_methods,
    read_calibration,
    read_calibration_batch,
)
from cl_real.visualization import Visualization
from cl_real.waypoints_extractor import WaypointsExtractor


def setup_logger(params):
    """Setup logger"""

    # Create log dir if not exists
    log_dir = params["data_dir"] / "evaluation"
    log_dir.mkdir(parents=True, exist_ok=True)

    # Setup logger.
    # Logger cookbook: https://docs.python.org/3/howto/logging-cookbook.html
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)-12s %(levelname)-8s %(message)s",
        datefmt="%m-%d %H:%M",
        filename=str(log_dir / "evaluation.log"),
        filemode="w",
    )

    # Setup also console logging.
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter("%(name)-12s: %(levelname)-8s %(message)s")
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger("").addHandler(console)

    logging.info(params)


def main():
    """main func."""
    params = load_params(CONFIG_PATH / "params.yaml")
    params["data_dir"] = Path(params["data_dir"]) / params["mode"] / params["path"]
    # params["data_dir"] = Path(params["data_dir"])
    if params["save_to_data_dir"]:
        params["result_dir"] = params["data_dir"]  # Save evaluation to result_dir

    # Setup logger.
    setup_logger(params)

    # Get evaluation result
    results = {}
    if params["load_evaluation"]:
        results = load_evaluation(params)
    # else:
    #     # Load camera info.
    #     calib_filenames = sorted(glob.glob(str(CONFIG_PATH / "calibration/*.yaml")))
    #     named_camera_info = read_calibration_batch(calib_filenames)

    #     # Load stamped method names: {"method_name": [start_stamp, end_stamp]}
    #     stamped_methods = load_stamped_methods(params)

    #     # Create waypoints extractor.
    #     wpts_extractor = WaypointsExtractor(params, named_camera_info)
    #     # Process image data.
    #     for name, stamps in stamped_methods.items():
    #         logging.info(f"Processing method: {name}, stamps: {stamps}")
    #         results[name] = wpts_extractor.run(name, stamps)
    else:
        # For flipped case. Camera on robot, tag on ceiling.
        calib_filenames = sorted(glob.glob(str(CONFIG_PATH / "calibration/pointgrey_*.yaml")))
        named_camera_info = read_calibration_batch(calib_filenames)

        # Load stamped method names: {"method_name": [start_stamp, end_stamp]}
        # stamped_methods = load_stamped_methods(params)

        # Create waypoints extractor.
        wpts_extractor = WaypointsExtractor(params, named_camera_info)
        # Process image data.
        calib = named_camera_info["pointgrey_14366756"]
        for method_name in params["methods"]:
            logging.info(f"Processing method: {method_name}")
            results[method_name] = wpts_extractor.run_flip_version(method_name, calib)

    # Run visualation.
    vis_node = Visualization(params)
    vis_node.run(results)


if __name__ == "__main__":
    main()
