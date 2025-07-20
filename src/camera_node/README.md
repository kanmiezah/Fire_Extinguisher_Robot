# camera_node

This ROS 2 node captures video frames from a connected USB or Pi camera and publishes them to the `/camera/image_raw` topic as `sensor_msgs/msg/Image`.

## Features

- Publishes raw image frames at 10 Hz
- Uses OpenCV for image capture
- Converts OpenCV images to ROS 2 Image messages using `cv_bridge`

## Topic

| Topic               | Type                  | Direction | Description                    |
|--------------------|-----------------------|-----------|--------------------------------|
| `/camera/image_raw`| `sensor_msgs/Image`   | Publisher | Raw image frames from camera   |

## Dependencies

- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `opencv-python`

## Launch

To run the node using launch:

```bash
ros2 launch camera_node camera_node.launch.py
