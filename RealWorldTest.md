# closedloop_nav_slam

## Real World Test Steps

**!!! Please remember to source the workspace in each new terminal.!!!**
    
    source catkin_ws/devel/setup.bash

**All the following steps should be executed on the laptop (can through `ssh`) which connects to the turtlebot.**

### Connect Hardware
- [ ] Connect the usb hub cable to the laptop.
- ~~[ ] Connect d435i camera cable to the laptop.~~
- [ ] Turn on the turtlebot.

### Activate Sensors
```bash
# Start roscore.
roscore

# Start all in one file.
roslaunch closedloop_nav_slam robot.launch

# OR start things separately.
# a. Start turtlebot bringup. Make sure `/dev/kobuki` (symlink) exists.
roslaunch closedloop_nav_slam minimal.launch
# b. Start lidar. Make sure `/dev/rplidar` (symlink) exists.
roslaunch rplidar_ros rplidar_s2.launch
# c. Start camera.
roslaunch closedloop_nav_slam realsense_stereo_nodelet.launch enable_depth:=true # d435i
```

### Mapping (Skip if alreay done)

- **Build the map**
    ```bash
    # 1. Start move base. Default planner (nav_name:=teb (default) | gpf)
    roslaunch closedloop_nav_slam move_base.launch goal_reached_thresh:=0.3

    # 2. Start slam_toolbox
    roslaunch slam_toolbox online_sync.launch vis:=true

    # 3. Drop nav goal through rviz and start the mapping.

    # 3.a Fine tune the nav planner parameters.
    cd configs/params/nav

    # 3.b Fine tune the slam parameters.
    cd ${SLAM_TOOLBOX_PATH}/slam_toolbox/slam_toolbox/config

    # 4. After mapping is done, execute the following script to save the map.
    cd scripts/closedloop_nav_slam/utils/
    sh save_slam_toolbox_map.sh $PATH_TO_SAVE_MAP
    ```

- **Define Waypoints**
    [Tutorial](README.md)
    - [ ] Save map to `configs/map/realworld/`
    - [ ] Save waypoints and path to `configs/path/realworld/`

### Main test
**1. Set parameters.**
- [ ] **Shutdown** `move_base` and `slam_toolbox` launched in the Mapping phase. (skip if not on)
- [ ] Set the correct map and robot init pose in the following file.

    [launch/realworld/map.launch](launch/realworld/map.launch)

- [ ] Set running parameters in the following file

    [config.yaml](configs/params/config.yaml)

        `test_type: "realworld"`
        `result_dir: ${PATH_TO_SAVE_RESULT}`

        `env_name: "realworld"`
        `robot_init_pose: [0, 0, 0, 0]`

**2. Start map launch file.**

    roslaunch closedloop_nav_slam map.launch

**3. Start the running script.**
```bash
cd scripts/nodes

python onekey.py

# The script will terminate automatically.

# Collect results from the path defined the the previous step.
    
    `result_dir`
```

### Other Useful Tools
- Tele-operate the robot
```bash
# Via keyboard.
roslaunch turtlebot_teleop keyboard_teleop.launch

# Via joystick.
roslaunch turtlebot_teleop logitech.launch
```
- Record a Rosbag
```bash
# Please set the topic names as args: topics:="$TOPIC_1 $TOPIC_2"
roslaunch closedloop_nav_slam data_logging.launch path_data_logging:=$PATH_TO_SAVE_ROSBAGS
```

- To Stop the Robot ([stop_robot.sh](scripts/tools/stop_robot.sh))
*Or simply lift the robot a bit.*
```bash
cd scripts/tools
sh stop_robot.sh
```

- To simulate a bumper-hit action, it will terminate the `waypoints_navigator.py` script safely
```bash
cd scripts/tools
sh hit_bumper.sh
```

- To Kill the `onekey.py` Script
```bash
kill -9 $(ps aux | grep onekey.py)
```