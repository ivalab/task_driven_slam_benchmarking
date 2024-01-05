#!/bin/bash

echo "cmd to save slam_toolbox map (map && pose_graph) ..."

if [ -z "$1" ]
then
    PREFIX=$(rospack find closedloop_nav_slam)/configs/map/slam_toolbox
else
    PREFIX=$1
fi

NAME=map_$(date '+%Y-%m-%d-%H:%M:%S')

echo mapname=$PREFIX/$NAME

rosservice call /slam_toolbox/save_map "name: {data: '$PREFIX/$NAME'}"
rosservice call /slam_toolbox/serialize_map "filename: '$PREFIX/$NAME'"

echo "Done (cmd save map)."