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

from cl_sim.evaluation import Evaluation
from cl_sim.utils.path_definitions import CONFIG_PATH
from cl_sim.utils.utils import (
    load_evaluation,
    save_evaluation,
    load_params,
)
from cl_sim.visualization import Visualization

EVALUATION_PATH = Path(__file__).resolve().parent


def setup_logger(params):
    """Setup logger"""

    # Create log dir if not exists
    log_dir = Path(params["result_dir"])
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
    params_file = CONFIG_PATH / "params.yaml"
    params = load_params(params_file)

    # Setup logger.
    setup_logger(params)

    # Acquire evaluation data.
    if params["load_evaluation"]:
        # Load evaluation data.
        logging.info("Load Evaluation ...")
        prefix = Path(params["result_dir"]) / params["mode"]
        eval_data = load_evaluation(prefix, params["envs"], params["methods"])
        logging.info("Load Evaluation Done.")
    else:
        # Run evaluation.
        logging.info("Run Evaluation ...")
        evaluation = Evaluation(params)
        eval_data = evaluation.run()
        logging.info("Evaluation Done.\n")
        # Save data.
        if params["save_evaluation"]:
            logging.info("Save Evaluation ...")
            prefix = Path(params["result_dir"]) / params["mode"]
            save_evaluation(prefix, eval_data, params["overwrite_evaluation"])
            logging.info("Save Evaluation Done.")

    # Run visualization.
    if params["enable_visualization"]:
        logging.info("Run Visualization ...")
        vis = Visualization(params)
        vis.run(eval_data)
        logging.info("Visualization Done.")
    logging.info("Main Script Done.")

    # End


if __name__ == "__main__":
    main()
