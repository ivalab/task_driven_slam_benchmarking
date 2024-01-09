# closedloop_nav_slam

This file provides steps to install and run ros packages on both gazebo and real turtlebot in **ROS Noetic** and **Ubuntu 20.04**.

## Install.
1. Install wstool.
        
        sudo apt-get install python3-rosdep  python3-wstool  build-essential python3-rosinstall-generator python3-rosinstall

2. Install sensor drivers (for real robot testing).

        sudo apt install ros-noetic-urg-node # hokuyo laser
        sudo apt install ros-noetic-realsense2-camera # realsense camera
        sudo apt install ros-noetic-realsense2-description # camera urdf

3. Initialize workspace.

        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        
        git clone https://github.gatech.edu/RoboSLAM/closedloop_nav_slam.git

        cd ~/catkin_ws && wstool init src

        wstool merge -t src src/closedloop_nav_slam/closedloop.rosinstall # turtlebot packages

        wstool update -t src -j20
        rosdep install --from-paths src -i -y

4. Build.

        cd ~/catkin_ws
        catkin build -j8 -DCMAKE_BUILD_TYPE=Release


5. Build SLAM methods.

        # TODO

## Run Simulation.
```bash
roscore

# Start gazebo.
roslaunch closedloop_nav_slam gazebo_turtlebot.launch
# Launch vlp16.
# roslaunch closedloop_nav_slam gazebo_turtlebot.launch laser_type:=vlp16

# Run onekey testing script.
roscd closedloop_nav_slam
cd scripts/closedloop_nav_slam
# Configure the parameters under ./settings/config.yaml, ./settings/slam/${SLAM_METHOD}.yaml
python ros/onekey.py
```

## Add a new SLAM method.
1. Define a new class called `${SLAM_NAME}Node` in `modular/slam_module.py`. For example, when adding a `amcl`:

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
2. Add the new method to the factor class in `modular/slam_module.py`. It direcly maps the `slam_method_name` to the defined `slam_node` class.
```python
def CreateSlamNode(params: Dict) -> NodeBase:
    ...
```

3. Add slam parameters in `settings/slam/slam_method.yaml`. For example: 
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

## Define Waypoints From A Known Map.
```
roslaunch closedloop_nav_slam gazebo_turtlebot.launch

# Start waypoints saver
rosrun closedloop_nav_slam waypoints_saver.py

# Start rviz and pick 2D nav goal.
rviz -d launch/closedloop_viz.rviz

```
