# Mapping

The default local planner is `teb`, please follow the steps below to use `gpf`

## Acquire GPF launch files.

Assume you have already compiled and installed `gpf` ros packages.

1. Clone and compile `slam_nav` repo, which contains the launch and parameters files for `gpf`
```bash
cd catkin_ws/src

git clone git@github.gatech.edu:ivabots/slam_nav.git

cd ../src
catkin build -j8
```

2. Install dependencies contained in the rosinstall file through `wstool`, remember to merge the rosinstall file (assume you already have one)

## Mapping with GPF
```bash

# Start gazebo.
roslaunch closedloop_nav_slam gazebo_turtlebot_gpf.launch

# 1. Start move base. Default planner (nav_name:=teb (default) | gpf)
roslaunch closedloop_nav_slam move_base.launch nav_name:=gpf goal_reached_thresh:=0.3

# 2. Start slam_toolbox
roslaunch slam_toolbox online_sync.launch vis:=true

# 3. Drop nav goal through rviz and start the mapping.

# 3.a Fine tune the nav planner parameters.
cd slam_nav/config/turtlebot_nav/

# 3.b Fine tune the slam parameters.
cd ${SLAM_TOOLBOX_PATH}/slam_toolbox/slam_toolbox/config

# 4. After mapping is done, execute the following script to save the map.
cd scripts/closedloop_nav_slam/utils/
sh save_slam_toolbox_map.sh $PATH_TO_SAVE_MAP
# OR just use ros map saver
rosrun map_server map_saver
```
