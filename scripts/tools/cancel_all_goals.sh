#!/bin/bash

echo "cmd to cancel all goals ..."
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}
echo "Done (cancelled all goals)."