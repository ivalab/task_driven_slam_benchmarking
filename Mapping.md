# Mapping

## Mapping with ROS Navigation Stack

The default local planner is `TEB`, please follow the steps in the next section to use `GPF`

```bash

# 1. Start gazebo, assure the gazebo world is configured in the launch file.
roslaunch closedloop_nav_slam gazebo_turtlebot.launch
# To use GPF, please run the following instead:
# roslaunch closedloop_nav_slam gazebo_turtlebot_gpf.launch

# 2. Start move base. Default planner (nav_name:=teb (default) | gpf)
roslaunch closedloop_nav_slam move_base.launch goal_reached_thresh:=0.3
# To use GPF, please move to the next section to install GPF first, then the following launch command.
# roslaunch closedloop_nav_slam move_base.launch nav_name:=gpf goal_reached_thresh:=0.3

# 3. Start slam_toolbox
roslaunch slam_toolbox online_sync.launch vis:=true

# 4. #
#   a. Send Nav Goals through rviz and do mapping as in the ROS gmapping tutorial.
#   b. Fine tune the nav planner parameters.
#       1. TEB: {YOUR_closedloop_nav_slam_PKG}/configs/params/nav
#       2. GPF: {YOUR_slam_nav_PKG}/config/turtlebot_nav/
#   c. Fine tune the slam parameters.
#       1. SLAM: ${YOUR_slam_toolbox_PKG}/slam_toolbox/config

# 5. When mapping is done, run the following script to save the map.
cd scripts/tools/
sh save_slam_toolbox_map.sh $PATH_TO_SAVE_MAP
# OR simple run ros map_saver
rosrun map_server map_saver
```

---

## Acquire GPF Launch Files

1. Install [`GPF`](https://github.gatech.edu/ivabots/meta_stixel/blob/main/depth_gpf.rosinstall).

2. Clone and compile `slam_nav`, which contains both launch and parameters files for `GPF`.
```bash
cd catkin_ws/src
git clone git@github.gatech.edu:ivabots/slam_nav.git
cd ../src
catkin build -j8
```