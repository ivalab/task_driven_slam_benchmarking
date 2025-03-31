#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file matplotlib_config.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 09-09-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""


import matplotlib
from matplotlib import pyplot as plt

# print(matplotlib.style.available)

# from matplotlib import rc
# from matplotlib import rcParams
# import matplotlib.font_manager

# rc("font", **{"family": "serif", "serif": ["Computer Modern"]})
# rc("text", usetex=True)


# Enable LaTeX in Matplotlib
# plt.rcParams.update(
#     {
#         "text.usetex": True,  # Use LaTeX for text rendering
#         "font.family": "serif",  # Use serif font
#         "font.serif": ["Computer Modern"],  # Use the Computer Modern font
#         "text.latex.preamble": r"\usepackage{amsmath}",  # Use AMSMath package for more advanced math
#     }
# )

plt.style.use("default")

# Set Matplotlib to use LaTeX
matplotlib.rcParams["text.usetex"] = True

# Optional: Set LaTeX font to match Overleaf settings (adjust as needed)
matplotlib.rcParams["font.family"] = "serif"
matplotlib.rcParams["font.serif"] = ["Times New Roman"]  # or any other serif font used in your LaTeX document

matplotlib.rcParams.update({"font.size": 15})
