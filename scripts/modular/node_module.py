#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file node_module.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-10-2023
@version 1.0
@license Copyright (c) 2023
@desc None
'''

from abc import ABC, abstractmethod
import os
import subprocess
from typing import Dict
import time

class NodeBase(ABC):

    def __init__(self, names: list, params: Dict):
        self._names = names
        self._params = params
        assert len(self._names) > 0
        assert self._params

    def names(self) -> list:
        return self._names

    def start(self) -> bool:
        cmd = self.compose_start_cmd()
        assert cmd
        print(cmd)
        subprocess.Popen(cmd, shell=True)
        return True

    def stop(self) -> bool:
        for name in self.names():
            subprocess.call("rosnode kill /" + name, shell=True)
        return True

    @abstractmethod
    def compose_start_cmd(self) -> str:
        return ""

    # @abstractmethod
    def reset(self) -> bool:
        return False

class MoveBaseNode(NodeBase):
    def __init__(self, params: Dict):
        names = ["move_base", "navigation_velocity_smoother"]
        if params["vis"]:
            names.append("closedloop_rviz")
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        return "roslaunch closedloop_nav_slam move_base.launch vis:=" + self._params["vis"]

    def reset(self) -> bool:
        raise NotImplementedError

class WaypointsNavigatorNode(NodeBase):
    def __init__(self, params: Dict, path_file: str, output_dir: str):
        super().__init__(["waypoints_navigator"], params)
        self._path_file = path_file
        self._output_dir = output_dir

    def compose_start_cmd(self) -> str:
        cmd = (
            "roslaunch closedloop_nav_slam waypoints_navigator.launch" + " env:=" + self._params["env_name"]
            + " path_file:=" + self._path_file + ".txt"
            + " trials:=" + str(self._params["trials"])
            + " output_dir:=" + self._output_dir
        )
        return cmd

    def reset(self) -> bool:
        raise NotImplementedError

# Unit test
if __name__ == "__main__":
    mb = MoveBaseNode({"vis": "true"})
    mb.start()
    time.sleep(10)
    mb.stop()
    mb.reset()