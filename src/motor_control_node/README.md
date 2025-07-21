# motor_control_node

ROS 2 node that controls a robot's motors via serial communication. It listens for navigation commands on the `/nav_command` topic and forwards valid commands (e.g., `forward`, `left`, `stop`) to a motor controller (e.g., Arduino) over a serial port.

## Features

- Subscribes to `/nav_command` (`std_msgs/String`)
- Sends serial commands to microcontroller
- Configurable serial port and baud rate via parameters
- Graceful serial reconnection handling

## Requirements

- ROS 2 (tested on Jazzy)
- Python 3.8+
- `pyserial`

## Installation

```bash
sudo apt install python3-serial
