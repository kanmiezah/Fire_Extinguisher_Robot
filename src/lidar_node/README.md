# lidar_node

This package provides a launch file to start the RPLIDAR driver using the `rplidar_ros` package.

## Features
- Launches the `rplidar_composition` node
- Configurable serial port and baudrate
- Publishes laser scan data to `/scan`

## Launch

```bash
ros2 launch lidar_node lidar.launch.py
