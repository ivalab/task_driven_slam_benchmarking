#!/bin/bash

echo "cmd to stop the robot ..."
rostopic pub -r 10 /cmd_vel_mux/input/navi geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "Done (cmd stop robot)."