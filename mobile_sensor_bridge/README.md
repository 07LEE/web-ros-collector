# Mobile Sensor Bridge

A ROS 2 pipeline package that utilizes a smartphone's web browser as a camera and IMU sensor node. It streams JPEG video frames and IMU sensor data (acceleration, angular velocity, orientation) to a PC, publishing them as ROS 2 `image_raw/compressed` and `imu/data_raw` topics over a local HTTPS connection without requiring any mobile app installation.

## Prerequisites

- OS: Ubuntu 24.04 LTS (or compatible Linux distribution)
- ROS 2: Jazzy Jalisco (Desktop version recommended)
- Network: Both the PC and the smartphone must be connected to the same local Wi-Fi network.

## Build & Installation

Build the package using `colcon`:

```bash
colcon build --packages-select mobile_sensor_bridge
```

## Running the Package

1. Launch via Execution Script (Recommended):

   ```bash
   ./scripts/run.sh
   ```

   Or run manually:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 run mobile_sensor_bridge mobile_sensor_bridge_node
   ```

2. Stream from Smartphone:
   - Open a web browser on your smartphone and navigate to the HTTPS address displayed in the execution log (e.g., `https://<PC_IP>:8443`). Make sure to include the `https://` prefix.
   - Bypass the SSL certificate warning page by clicking 'Advanced' -> 'Proceed to website (unsafe)'.
   - Accept camera permissions, tap 'IMU PERMISSION' (for iOS), and tap 'START STREAMING'.

3. Verify Data Reception:
   Open a new terminal window and run:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

   # Check IMU topic
   ros2 topic echo imu/data_raw

   # Check Camera Topic Frame Rate & Format
   ros2 topic echo image_raw/compressed --field format
   ```

## Key Features & Quality of Service (QoS)

- QoS Profile: Uses qos_profile_sensor_data (Best Effort, Volatile) to minimize latency and discard stale frames during real-time streaming.
- Time Synchronization: Performs RTT-based clock offset estimation between smartphone and PC upon page load (/sync_time endpoint), attaching precision client-side timestamps to Header.stamp.
- Thread Safety: HTTP request handler routes incoming data into a thread-safe publishing queue, decoupled from ROS 2 node executor execution.

## Published Topics

| Topic | Message Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Best Effort | JPEG compressed camera image stream |
| `imu/data_raw` | `sensor_msgs/msg/Imu` | Best Effort | Linear acceleration, angular velocity, orientation quaternion |
| `mobile_sensor_bridge/device_info` | `std_msgs/msg/String` | Reliable | Connected mobile device metadata (JSON string) |
| `image_raw` | `sensor_msgs/msg/Image` | Best Effort | Decompressed raw image (published by `camera_info_publisher`) |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort | Camera intrinsic parameters (published by `camera_info_publisher`) |

## Node Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | int | `8443` | HTTPS server listening port |
| `image_topic` | string | `image_raw/compressed` | Topic name for compressed image stream |
| `imu_topic` | string | `imu/data_raw` | Topic name for IMU sensor data |
| `frame_id_camera` | string | `phone_camera` | Frame ID for camera image headers |
| `frame_id_imu` | string | `phone_imu` | Frame ID for IMU data headers |

## File Structure

- `package.xml`: Package metadata, license, and ROS 2 dependencies (`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `ament_index_python`).
- `setup.py`: Build and installation configuration script for the Python package.
- `web/index.html`: Client-side web interface served to the smartphone browser with camera frame capture, IMU sampling, and time sync logic.
- `mobile_sensor_bridge/mobile_sensor_bridge_node.py`: Main ROS 2 executable node managing HTTPS server, parameters, SSL certs in `~/.ros/mobile_sensor_bridge_certs`, `/sync_time` endpoint, and thread-safe publishing queue.
- `mobile_sensor_bridge/camera_bridge.py`: Camera image module handling `image_raw/compressed` topic publishing with `qos_profile_sensor_data`.
- `mobile_sensor_bridge/imu_bridge.py`: IMU sensor module handling Euler-to-Quaternion conversion and `imu/data_raw` topic publishing.
- `mobile_sensor_bridge/camera_info_publisher.py`: Auxiliary node for image decompression, static TF broadcasting, and `camera_info` publishing.
- `scripts/run.sh`: Automated launch script with log cleanup and environment setup.
- `scripts/clean.sh`: Cleanup script for `build/`, `install/`, and `log/` directories.

