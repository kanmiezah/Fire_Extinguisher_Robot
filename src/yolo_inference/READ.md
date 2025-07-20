# YOLO Inference Node

This ROS 2 package runs YOLOv8 object detection using a PyTorch `.pt` model for fire detection. It processes camera input and publishes fire detection locations.

## Features

- Loads a YOLOv8 PyTorch model (`.pt`)
- Subscribes to `/image_raw` (sensor_msgs/Image)
- Publishes `/fire_targets` (geometry_msgs/PoseArray) with normalized (x, y) detections
- Designed for real-time fire detection

## Dependencies

- `rclpy`
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `opencv-python`
- `torch` and `ultralytics`

## Launch

```bash
ros2 launch yolo_inference yolo_node.launch.py
