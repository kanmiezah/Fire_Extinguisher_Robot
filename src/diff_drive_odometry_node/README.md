# diff_drive_odometry_node

A ROS 2 node that reads wheel encoder data from an Arduino over serial and publishes odometry messages (`nav_msgs/Odometry`) for use in SLAM and the Nav2 stack. It also broadcasts the `odom → base_link` transform.

## Features

- Reads left and right encoder tick counts from Arduino via serial
- Calculates robot pose using differential drive kinematics
- Publishes `/odom` topic
- Publishes TF between `odom` and `base_link`

## Parameters

- `port` (string, default: `/dev/ttyUSB0`): Serial port connected to Arduino
- `baudrate` (int, default: `9600`): Baud rate for serial communication
- `frame_id` (string, default: `'odom'`): Odometry frame
- `child_frame_id` (string, default: `'base_link'`): Robot base frame

## Topic Published

- `/odom` (`nav_msgs/Odometry`): Odometry message with position and orientation

## TF

- Broadcasts: `odom` → `base_link`

## Launch

To launch the node with default parameters:
```bash
ros2 launch diff_drive_odometry_node diff_drive_odometry.launch.py

