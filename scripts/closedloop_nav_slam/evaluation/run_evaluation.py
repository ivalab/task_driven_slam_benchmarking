#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_evaluation.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-26-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""
import logging
from pathlib import Path

import yaml
from closedloop_nav_slam.evaluation.evaluation import Evaluation
from closedloop_nav_slam.evaluation.visualization import Visualization
from closedloop_nav_slam.evaluation.utils import (
    load_evaluation,
    save_evaluation,
    compose_dir_path,
)
from closedloop_nav_slam.utils.path_definitions import (
    SETTINGS_PATH,
)

EVALUATION_PATH = Path(__file__).resolve().parent


def load_params(test_config_file: str, eval_config_file: str):
    """Load params"""
    params = None
    with open(test_config_file, encoding="utf-8") as tf, open(eval_config_file, encoding="utf-8") as ef:
        params = yaml.safe_load(tf)
        params.update(yaml.safe_load(ef))
    if params["save_to_result_dir"]:
        params["output_dir"] = str(
            Path(params["result_dir"]) / params["test_type"] / params["env_name"] / "evaluation")
    assert params
    return params

def setup_logger(params):
    """Setup logger"""

    # Create log dir if not exists
    log_dir = Path(params["output_dir"])
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

    # Logging params.
    logging.info(params)

def main():
    """Main Script"""

    # Load params.
    test_config_file = SETTINGS_PATH / "config.yaml"
    eval_config_file = EVALUATION_PATH / "config.yaml"
    params = load_params(test_config_file, eval_config_file)

    # Setup logger.
    setup_logger(params)

    # Acquire evaluation data.
    if params["load_evaluation"]:
        # Load evaluation data.
        logging.info("Load Evaluation ...")
        methods = load_evaluation(params["output_dir"], params["method_list"])
        logging.info("Load Evauation Done.")
    else:
        # Run evaluation.
        logging.info("Run Evaluation ...")
        evaluation = Evaluation(params)
        methods = evaluation.run()
        logging.info("Evaluation Done.\n")
        # Save data.
        if params["save_data"]:
            logging.info("Save Evaluation ...")
            save_evaluation(params["output_dir"], methods, params["overwrite_data"])
            logging.info("Save Evaluation Done.")

    # Run visualization.
    logging.info("Run Visualization ...")
    vis = Visualization(params, methods)
    vis.run()
    logging.info("Visualization Done.")
    logging.info("Main Script Done.")

    # End


if __name__ == "__main__":
    main()
