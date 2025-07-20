# Sensor Fusion Node

## Overview

This ROS 2 node fuses fire detection confidence and smoke sensor data to determine the likelihood of a fire. If the fused alert level exceeds a threshold, the node triggers a suppression system.

## Node Details

- **Node Name:** `sensor_fusion_node`

### Subscribed Topics

| Topic            | Message Type | Description                            |
|------------------|--------------|----------------------------------------|
| `/fire_confidence` | `std_msgs/Float32` | Fire detection confidence from vision model |
| `/smoke_level`     | `std_msgs/Float32` | Smoke sensor analog voltage |

### Published Topics

| Topic                  | Message Type     | Description                     |
|------------------------|------------------|---------------------------------|
| `/fire_alert_level`    | `std_msgs/Float32` | Fused fire threat level         |
| `/activate_suppression`| `std_msgs/Bool`  | Trigger to start suppression    |

## Fusion Logic

The alert level is computed as:

If `alert_level > 0.7`, the suppression system is activated.

## Launch

```bash
ros2 launch sensor_fusion_node sensor_fusion.launch.py
