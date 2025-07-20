# firebot_description

URDF and Xacro description package for the Firebot firefighting robot.

## Overview

This package defines the robot model using URDF and Xacro, including components such as:
- Robot base and frame
- Camera
- Lidar
- Inertial properties

It also includes a launch file to start the `robot_state_publisher` for publishing the robot’s transform tree.

## Package Structure


## Dependencies

- `xacro`
- `robot_state_publisher`
- `launch`
- `launch_ros`

## Usage

To launch the robot description and state publisher:

```bash
ros2 launch firebot_description description.launch.py
