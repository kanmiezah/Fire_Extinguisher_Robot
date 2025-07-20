# suppression_node

A ROS 2 node that controls the fire suppression system (e.g., pump or actuator) based on incoming alerts.

## Features

- Listens on `/activate_suppression` (`std_msgs/Bool`)
- Activates a GPIO relay for 5 seconds on `True` signal
- Deactivates on timeout or `False` signal
- Gracefully handles shutdown and GPIO cleanup
- Supports simulation mode if not running on Raspberry Pi

## Topics

| Name | Type | Direction | Description |
|------|------|-----------|-------------|
| `/activate_suppression` | `std_msgs/Bool` | Subscription | Triggers fire suppression mechanism |

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `relay_pin` | `int` | `17` | BCM GPIO pin for relay |
| `timer_duration` | `float` | `5.0` | Duration (in seconds) relay remains active |

## Run

```bash
ros2 run suppression_node suppression_node
