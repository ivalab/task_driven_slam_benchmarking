#!/bin/bash

BUTTON_VALUE=0
STATE_VALUE=1

if [ ! -z "$1" ]
then
    BUTTON_VALUE=$1
fi

if [ ! -z "$2" ]
then
    STATE_VALUE=$2
fi

echo "cmd to press button $BUTTON_VALUE state $STATE_VALUE..."
rostopic pub -1 /mobile_base/events/button kobuki_msgs/ButtonEvent "{button: $BUTTON_VALUE, state: $STATE_VALUE}"
echo "Done (press button)."