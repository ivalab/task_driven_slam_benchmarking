# closedloop_nav_slam

This file provides steps to install and run ros packages on both gazebo and real turtlebot in **ROS Noetic** and **Ubuntu 20.04**.

## Install
1. Install wstool.
        
        sudo apt-get install python3-rosdep  python3-wstool  build-essential python3-rosinstall-generator python3-rosinstall python3-pip

2. Install sensor drivers (for real robot testing) and other libs.

        sudo apt install ros-noetic-urg-node # hokuyo laser
        sudo apt install ros-noetic-realsense2-camera # realsense camera
        sudo apt install ros-noetic-realsense2-description # camera urdf
        sudo apt install ros-noetic-teb-local-planner # move_base
        sudo apt install ros-noetic-sparse-bundle-adjustment # slam_toolbox

3. Initialize workspace.

        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        
        git clone https://github.gatech.edu/RoboSLAM/closedloop_nav_slam.git

        cd ~/catkin_ws && wstool init src

        wstool merge -t src src/closedloop_nav_slam/closedloop.rosinstall # turtlebot packages

        wstool update -t src -j20
        rosdep install --from-paths src -i -y

4. Build ROS Nodes.

        cd ~/catkin_ws
        catkin build -j8 -DCMAKE_BUILD_TYPE=Release

5. Install non-ROS Code.

        cd ~/catkin_ws/src/closedloop_nav_slam/scripts
        pip install -e .


6. Build SLAM methods.
   Please follow the REAME in each repo to build the library.

   - [slam_toolbox](https://github.com/ivalab/slam_toolbox)
   - [hector_slam](https://github.com/ivalab/hector_slam)
   - [gf_orb_slam2](https://github.com/ivalab/gf_orb_slam2)
   - [orb3](https://github.gatech.edu/RoboSLAM/ORB_SLAM3)
   - [msckf](https://github.gatech.edu/RoboSLAM/msckf_vio)
   - [dsol](https://github.gatech.edu/RoboSLAM/dsol)
   - [svo](https://github.gatech.edu/RoboSLAM/rpg_svo_pro_open)

## Run Mapping with GPF
[Mapping](Mapping.md)

## Run Real World Test
[RealWorldTest](RealWorldTest.md)


## Run Simulation


**!!! Please remember to source the workspace in each new terminal.!!!**
    
    source catkin_ws/devel/setup.bash

1. Set map and robot init pose. The map and robot_init_pose are recorded [here](configs/map//README.md).
   1. Set them in the gazebo launch file.
        
        [launch/gazebo/gazebo_turtlebot.launch](launch/gazebo//gazebo_turtlebot.launch)

   2. Set them in the config file.
   
        [scripts/closedloop_nav_slam/settings/config.yaml](scripts/closedloop_nav_slam/settings/config.yaml)

2. Start launch files.
```bash
# Start roscore.
roscore

# Start gazebo.
roslaunch closedloop_nav_slam gazebo_turtlebot.launch

# (Optional) How to use vlp16 in gazebo.
# roslaunch closedloop_nav_slam gazebo_turtlebot.launch laser_type:=vlp16

# Run onekey testing script.
roscd closedloop_nav_slam

cd scripts/closedloop_nav_slam/ros

python onekey.py

```

## Extension
### Add a new SLAM method.
1. Define a new class called `${SLAM_NAME}Node` in `scripts/closedloop_nav_slam/modular/slam_module.py`. For example, when adding `amcl`:

```python
    class AmclNode(NodeBase):
    def __init__(self, params: Dict):
        # Define the rosnode names when using amcl. It includes amcl itself and any other helper/tool nodes amcl needs.
        names = ["amcl", "slam_map_server"]
        super().__init__(names, params)

    def compose_start_cmd(self) -> str:
        # Defines the amcl start command.
        return (
            "roslaunch closedloop_nav_slam amcl.launch output_pose_topic:="
            + self._params["et_pose_topic"]
        )
```
2. Add the new method to the factory class in `scripts/closedloop_nav_slam/modular/slam_module.py`. It direcly maps the `slam_method_name` to the defined `slam_node` class.
```python
def CreateSlamNode(params: Dict) -> NodeBase:
    ...
```

3. Add slam parameters in `scripts/closedloop_nav_slam/settings/slam/slam_method.yaml`. For example: 
```yaml
## amcl
slam_method: "amcl"
mode: "localization"
enable_msf: false
slam_sensor_type: "laser"
source_msg_parent_frame: "base_footprint" # Define the parent frame that aligns with map frame in slam. VSLAM typically is left_camera_frame, 2D laser is base_footprint.
source_msg_child_frame: "gyro_link" # Define the child frame of which the pose is estimated in parent frame. VSLAM typically is left_camera_optical_frame, 2D laser is base_footprint.
loops: 1 # Define the number of loops in a single trial.
need_map_to_odom_tf: false # Whether needs an additional map_to_odom_tf publisher node. Most 2D laser methods in ros publish this tf inside their class. Some do not and need this publisher node.
```

### Define Waypoints From A Known Map.
```
# First set the proper map file in the launch file.
roslaunch closedloop_nav_slam map.launch

# Start waypoints saver
rosrun closedloop_nav_slam waypoints_saver.py

# Start rviz and select 2D nav goal.
rviz -d launch/closedloop_viz.rviz

# The waypoints will be saved under `scripts/closedloop_nav_slam/ros/` and can later be moved to `configs/path/`
```

## Evaluation
Please follow the [steps](scripts/closedloop_nav_slam/evaluation/README.md).

## Issue Tracking.
- How to disable odom_to_base tf from kobuki_gazebo?
```bash
cd ${YOUR_CATKIN_WS}/src/kobuki_ros/kobuki_desktop/kobuki_gazebo_plugins/src

# Change line 166 of file gazebo_ros_kobuki_updates.cpp
if (publish_tf_)
# to
if (false && publish_tf_)

# Rebuild.
catkin build -j16

# Run new wheel odometry publisher.
# It subscribes the gazebo odometry and publishes the disturbed (noise) wheel odometry.
rosrun closedloop_nav_slam wheel_odometry_publisher.py

```
