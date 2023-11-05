# closedloop_nav_slam

This file provides steps to install and run ros packages on both gazebo and real turtlebot with **ROS Noetic** and **Ubuntu 20.04**.

## Install.
1. Install wstool.
        
        sudo apt-get install python3-rosdep  python3-wstool  build-essential python3-rosinstall-generator python3-rosinstall

2. Install sensor drivers.

        sudo apt install ros-noetic-urg-node # lidar
        sudo apt install ros-noetic-realsense2-camera # camera
        sudo apt install ros-noetic-realsense2-description # camera urdf

3. Initialize workspace.

        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        
        git clone https://github.gatech.edu/RoboSLAM/closedloop_nav_slam.git
        git clone https://github.gatech.edu/ivabots/turtlebot_demo.git -b noetic/roboslam # navigation planner
        git clone https://github.com/TurtlebotAdventures/gazebo_fake_localization -b more_trans_offset # map-to-odom-tf

        cd ~/catkin_ws && wstool init src

        wstool merge -t src src/closedloop_nav_slam/closedloop.rosinstall # turtlebot packages

        wstool update -t src -j20
        rosdep install --from-paths src -i -y

4. Build.

        cd ~/catkin_ws
        catkin build -j8 -DCMAKE_BUILD_TYPE=Release


## Running Simulation.
```bash
roscore

# Start gazebo.
roslaunch closedloop_nav_slam gazebo_turtlebot.launch

# Start navigation stack (in urtlebot_demo pkg).
# Make sure move_base use visual estimation (has "visual" prefix in tf and topics)
roslaunch closedloop_nav_slam nav_slam_test.launch vis:=true

# Start slam.
roslaunch slam_toolbox nav_slam_test.launch mode:=localization

# Start waypoints_sender node
roscd turtlebot_roboslam
python scripts/closedloop_nav_test.py --mode localization

# Trigger button to start sending goals
roscd turtlebot_roboslam
sh scripts/start_robot_button0.sh

# Or skip the above two steps and start onekey.py script and record rosbags
roscd turtlebot_robslam
python scripts/onekey.py
```
