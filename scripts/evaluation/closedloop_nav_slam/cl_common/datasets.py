#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file datasets.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 07-22-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

from typing import List
from dataclasses import dataclass


EUROC_SEQUENCES = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
]


@dataclass
class Sequence:
    """_summary_"""

    name: str = ""
    image_count: int = 0
    duration: float = 0
    traj_length: float = 0


class Dataset:
    """_summary_"""

    def __init__(self, name: str = "", sequences: List[Sequence] = None):
        self.name = name
        self.sequences = sequences
        self._name2index = {self.sequences[index].name: index for index in range(len(self.sequences))}

    def sequence_names(self) -> List[str]:
        assert self.sequences
        return [seq.name for seq in self.sequences]

    def get_sequence(self, name: str = "") -> Sequence:
        return self.sequences[self._name2index[name]]


EUROC_DATASET = Dataset(
    "euroc",
    [
        Sequence("MH_01_easy", 3682),
        Sequence("MH_02_easy", 3040),
        Sequence("MH_03_medium", 2700),
        Sequence("MH_04_difficult", 2033),
        Sequence("MH_05_difficult", 2273),
        Sequence("V1_01_easy", 2912),
        Sequence("V1_02_medium", 1710),
        Sequence("V1_03_difficult", 2149),
        Sequence("V2_01_easy", 2280),
        Sequence("V2_02_medium", 2348),
        Sequence("V2_03_difficult", 1922),
    ],
)
