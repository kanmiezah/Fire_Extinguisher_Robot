
---

### ✅ 2. `fire_confidence_node/README.md`

```markdown
# Fire Confidence Node

This ROS 2 node estimates the confidence of fire presence based on number of YOLO detections published to `/fire_targets`.

## Features

- Computes confidence from detection count
- Publishes normalized confidence score [0.0 - 1.0]
- Subscribes to `/fire_targets` (PoseArray)
- Publishes to `/fire_confidence` (Float32)

## Dependencies

- `rclpy`
- `geometry_msgs`
- `std_msgs`

## Launch

```bash
ros2 launch fire_confidence_node fire_confidence_node.launch.py
