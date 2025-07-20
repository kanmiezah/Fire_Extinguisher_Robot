
---

### ✅ 3. `smoke_sensor_node/README.md`

```markdown
# Smoke Sensor Node

ROS 2 node to interface with the MQ-2 smoke sensor via MCP3008 ADC on Raspberry Pi.

## Features

- Reads analog voltage from MQ-2 via MCP3008
- Publishes voltage to `/smoke_level`
- Threshold alert logging
- Parameterized interval and channel

## Dependencies

- `rclpy`
- `std_msgs`
- `adafruit-circuitpython-mcp3xxx`
- `busio`, `digitalio`, `board` (from `adafruit-blinka`)

## Launch

```bash
ros2 launch smoke_sensor_node smoke_sensor_node.launch.py
