# Gazebo Worlds, Maps and Robot Init Poses

The robot init pose is defined in `x, y, z, yaw` order with the units of `meters` and `radians`.

## AWS World
    
    name: aws_small_warehouse
    robot_init_pose: [0.0, 0.0, 0.0, 0.034]

    name: aws_small_house
    robot_init_pose: [0.0, 0.0, 0.0, 0.0]
    
## TSRB World
    
    name: fourth_floor_stixel
    robot_init_pose: [-40, 14, 0, 0]

## Classroom

    name: classroom_textured_collid
    robot_init_pose: [9, -5, 0.0, 3.14159265359]

- [ ] Create a config file for each world, the file should be accessible by both a ros launch file and a yaml file.