# closedloop_nav_slam

 Exploration Steps

```bash

# Start roscore.
roscore

# Start gazebo (plz configure world model inside tieh launch file).
roslaunch closedloop_nav_slam gazebo_turtlebot.launch mode:=exploration

# Start move_base (plz configure robot speed.)
# (Sample values: max_vel_x: 0.5, max_vel_x_backwards: 0.2, max_vel_theta: 0.5, acc_lim_x: 1.0, acc_lim_theta: 0.2)
roslaunch closedloop_nav_slam move_base.launch

# Start slam.
roslaunch slam_toolbox online_async.launch

# Start exploration node.
rosrun closedloop_nav_slam exploration.py

# Save map (default to configs/map/slam_toolbox).
cd scripts/closedloop_nav_slam/utils
sh save_slam_toolbox_map.sh

```
