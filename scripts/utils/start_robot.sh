#!/bin/bash

BUTTON_VALUE=0
STATE_VALUE=1
echo "cmd to press button $BUTTON_VALUE ..."
rostopic pub -1 /mobile_base/events/button kobuki_msgs/ButtonEvent "{button: $BUTTON_VALUE, state: $STATE_VALUE}"
echo "Done"