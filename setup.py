## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["closedloop_nav_slam"],
    package_dir={"": "scripts"},
)

setup(**setup_args)


# setup(
#     name="my_ros_package",
#     version="0.0.1",
#     packages=["my_python_library"],
#     package_dir={"": "src"},
#     install_requires=[
#         "numpy",  # Add your non-ROS Python package here
#         "scipy",  # Add more packages if needed
#     ],
#     entry_points={
#         "console_scripts": [
#             "my_node = my_ros_package.my_node:main",
#         ],
#     },
# )
