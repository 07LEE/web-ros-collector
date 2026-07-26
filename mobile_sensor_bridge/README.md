# Mobile Sensor Bridge

A ROS 2 pipeline project that utilizes a smartphone's web browser as a camera and IMU sensor node. It streams video frames and IMU sensor data (acceleration, angular velocity, orientation) to a PC, publishing them as ROS 2 `/image_raw/compressed` and `/imu/data_raw` topics over a local HTTPS connection without requiring any mobile app installation.

## Prerequisites

- OS: Ubuntu 24.04 LTS
- ROS 2: Jazzy Jalisco (Desktop version recommended)
- Network: Both the PC and the smartphone must be connected to the same Wi-Fi network.

## Installation & Build

1. Install ROS 2 Jazzy (run only if ROS 2 is not installed):

   ```bash
   chmod +x scripts/install_ros2.sh
   ./scripts/install_ros2.sh
   source ~/.bashrc
   ```

2. Build the Package:

   ```bash
   colcon build
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
   - Open a web browser on your smartphone and navigate to the address displayed in the execution log (e.g., `https://<PC_IP>:8443`).
   - Bypass the SSL certificate warning page by clicking 'Advanced' -> 'Proceed to website (unsafe)'.
   - Accept the camera permission, tap 'Request IMU Sensor Permission' (for iOS), and tap 'Start Streaming'.

3. Verify Data Reception:
   Open a new terminal window and run:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

   # Check IMU topic
   ros2 topic echo /imu/data_raw

   # Check Camera Topic Frame Rate
   ros2 topic hz /image_raw/compressed
   ```

## Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Raw compressed image payload (JPEG/PNG) |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Linear acceleration, angular velocity, orientation |
| `/mobile_sensor_bridge/device_info` | `std_msgs/msg/String` | Connected mobile device metadata (JSON string) |

## File Structure

- `package.xml`: Package metadata, license, maintainer details, and ROS 2 dependencies (`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`).
- `setup.py`: Build and installation configuration script for the Python package, defining executable entry points and installing resource files.
- `setup.cfg`: Installation script path configuration pointing to `lib/mobile_sensor_bridge`.
- `resource/mobile_sensor_bridge`: Empty marker file used by the ament index to register the package.
- `web/index.html`: Client-side web interface served to the smartphone browser to capture camera frames and IMU sensor data, displaying a real-time sensor dashboard.
- `mobile_sensor_bridge/mobile_sensor_bridge_node.py`: Main ROS 2 executable node orchestrating `CameraBridge` and `ImuBridge` and running the HTTPS server.
- `mobile_sensor_bridge/camera_bridge.py`: Camera image module handling `/image_raw/compressed` topic publishing.
- `mobile_sensor_bridge/imu_bridge.py`: IMU sensor module handling Euler-to-Quaternion conversion and `/imu/data_raw` topic publishing.
- `scripts/run.sh`: Automated launch script with log directory cleanup and environment setup.
- `scripts/clean.sh`: One-touch cleanup script for `build/`, `install/`, and `log/` directories.
