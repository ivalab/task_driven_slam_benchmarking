#!/bin/bash

BUMPER_VALUE=1 # CENTER (LEFT: 0, RIGHT: 2)
STATE_VALUE=1 # PRESSED (RELEASED: 0)

if [ ! -z "$1" ]
then
    BUMPER_VALUE=$1
fi

if [ ! -z "$2" ]
then
    STATE_VALUE=$2
fi

echo "cmd to hit bumper $BUMPER_VALUE state $STATE_VALUE..."
rostopic pub -1 /mobile_base/events/bumper kobuki_msgs/BumperEvent "{bumper: $BUMPER_VALUE, state: $STATE_VALUE}"
echo "Done (hit bumper)."