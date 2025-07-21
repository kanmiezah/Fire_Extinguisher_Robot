# bringup

The `bringup` package provides launch files to bring up all core components of the autonomous firefighting robot system, including perception, state management, navigation, and suppression.

## Features

- Launches all nodes in the system:
  - Camera, YOLO-based fire detection
  - Smoke sensor, fire confidence estimation
  - State machine controller
  - Suppression system
  - Motor control and odometry
  - LIDAR and SLAM (via `slam_toolbox`)
  - Navigation stack (`Nav2`)
- Loads configuration parameters for SLAM and navigation
- Brings up the full robot system using a single launch file

## Launch Files

### `bringup.launch.py`

Launches the complete robot system, including:

- `camera_node`, `yolo_inference`, `fire_confidence_node`
- `smoke_sensor_node`, `suppression_node`, `state_machine_node`
- `motor_control_node`, `diff_drive_odometry_node`
- `rplidar_ros`, `slam_toolbox`, `nav2_bringup`
- `robot_state_publisher`

```bash
ros2 launch bringup bringup.launch.py
