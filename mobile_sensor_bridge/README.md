# Mobile Sensor Bridge

A ROS 2 pipeline package that utilizes a smartphone's web browser as a camera and IMU sensor node. It streams JPEG video frames and IMU sensor data (acceleration, angular velocity, orientation) to a PC, publishing them as ROS 2 image_raw/compressed and imu/data_raw topics over a local HTTPS connection without requiring any mobile app installation.

## Prerequisites & Verified Devices

- OS: Ubuntu 24.04 LTS (or compatible Linux distribution)
- ROS 2: Jazzy Jalisco (Desktop version recommended)
- Network: Both the PC and the smartphone must be connected to the same local Wi-Fi network.
- Verified Target Hardware: Samsung Galaxy S26 Ultra (Android) - Verified with Camera, IMU, Battery Telemetry, and Geolocation GPS.

## Build & Installation

Build the package using colcon:

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
   - Open a web browser on your smartphone and navigate to the HTTPS address displayed in the execution log (e.g., https://<PC_IP>:8443).
   - Bypass the SSL certificate warning page by clicking Advanced -> Proceed to website.
   - Accept camera permissions and tap START STREAMING.

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

## Quality of Service (QoS)

- QoS Profile: qos_profile_sensor_data (Best Effort, Volatile)
- Time Sync: RTT-based clock offset estimation via /sync_time endpoint
- Thread Safety: Queue-decoupled publishing handler

## Published Topics

| Topic | Message Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| image_raw/compressed | sensor_msgs/msg/CompressedImage | Best Effort | JPEG compressed camera image stream |
| imu/data_raw | sensor_msgs/msg/Imu | Best Effort | Linear acceleration, angular velocity, orientation quaternion |
| /robot/battery | sensor_msgs/msg/BatteryState | Best Effort | Smartphone battery percentage and charging status |
| /robot/gps | sensor_msgs/msg/NavSatFix | Best Effort | Geolocation latitude, longitude, altitude, and covariance |
| mobile_sensor_bridge/device_info | std_msgs/msg/String | Reliable | Connected mobile device metadata (JSON string) |
| /camera/exposure_metadata | std_msgs/msg/String | Best Effort | Per-frame camera exposure time, ISO, and sequence metadata (JSON) |
| image_raw | sensor_msgs/msg/Image | Best Effort | Decompressed raw image (published by camera_info_publisher) |
| camera_info | sensor_msgs/msg/CameraInfo | Best Effort | Camera intrinsic parameters (published by camera_info_publisher) |

## Node Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| port | int | 8443 | HTTPS server listening port |
| image_topic | string | image_raw/compressed | Topic name for compressed image stream |
| imu_topic | string | imu/data_raw | Topic name for IMU sensor data |
| battery_topic | string | /robot/battery | Topic name for Battery state data |
| gps_topic | string | /robot/gps | Topic name for GPS NavSatFix data |
| frame_id_camera | string | phone_camera | Frame ID for camera image headers |
| frame_id_imu | string | phone_imu | Frame ID for IMU data headers |
| frame_id_gps | string | gps_link | Frame ID for GPS data headers |
