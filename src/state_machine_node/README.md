# state_machine_node

A ROS 2 node that controls the high-level behavior of a fire-fighting robot using a finite state machine.

## States
- **PATROL**: Move forward for a fixed duration.
- **SCAN**: Wait for fire detection.
- **SUPPRESS**: Stop and execute suppression routine if fire is detected.
- **REST**: Pause before restarting patrol.

## Topics

| Name               | Type             | Direction | Description                         |
|--------------------|------------------|-----------|-------------------------------------|
| `/nav_command`     | `std_msgs/String`| Publisher | Sends motion commands to motors.    |
| `/robot_state`     | `std_msgs/String`| Publisher | Broadcasts the robot's current state.|
| `/suppress_fire`   | `std_msgs/Bool`  | Subscriber| Listens for fire detection trigger. |

## Launch
```bash
ros2 launch state_machine_node state_machine.launch.py
