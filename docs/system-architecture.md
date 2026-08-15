# Mobile Sensor Bridge System Architecture

## 1. Executive Summary

Mobile Sensor Bridge is a ROS 2 Jazzy package that transforms any modern smartphone browser into a wireless sensor node and edge robot device. It streams camera frames (image_raw/compressed), IMU orientation and kinematics (imu/data_raw), battery telemetry (/robot/battery), and GPS coordinates (/robot/gps) over HTTPS without requiring mobile app installation.

## 2. High-Level Architecture Diagram

```
+-------------------------------------------------------+
|                 Smartphone Web Client                 |
|  - HTML5 Camera Capture (Canvas -> JPEG)              |
|  - DeviceMotion & DeviceOrientation API (IMU)         |
|  - Battery Status API (Level, Charging State)         |
|  - Geolocation API (Latitude, Longitude, Accuracy)    |
|  - Client-Server Time Sync (/sync_time)     |
+-------------------------------------------------------+
                           |
                     HTTPS / JSON
                           v
+-------------------------------------------------------+
|             MobileSensorBridgeNode (ROS 2)            |
|  - Multithreaded SecureHTTPServer (0.0.0.0:8443)      |
|  - Auto-generated Self-Signed Multi-IP SSL Certs     |
|  - Thread-Safe Queue & Spin Timers                    |
+-------------------------------------------------------+
     |                 |                 |            |
     v                 v                 v            v
+----------+     +----------+     +----------+   +----------+
|  Camera  |     |   IMU    |     | Battery  |   |   GPS    |
|  Bridge  |     |  Bridge  |     |  Bridge  |   |  Bridge  |
+----------+     +----------+     +----------+   +----------+
     |                 |                 |            |
     v                 v                 v            v
  ROS 2             ROS 2             ROS 2        ROS 2
 Topic:            Topic:            Topic:       Topic:
 /image_raw/       /imu/             /robot/      /robot/
 compressed        data_raw          battery      gps
```

## 3. Core Modules & Component Responsibilities

### 3.1 MobileSensorBridgeNode (mobile_sensor_bridge_node.py)

- Manages node lifecycle, ROS 2 parameters (port, image_topic, imu_topic, battery_topic, gps_topic, frame IDs).
- Hosts a multi-threaded SecureHTTPServer with SSL context wrapping.
- Auto-generates SAN (Subject Alternative Name) SSL certificates covering all host IPv4 interfaces.
- Enqueues incoming HTTP requests into a thread-safe Queue and dispatches them via spin timers.
- Controls rosbag2 process groups using preexec_fn=os.setsid and os.killpg.

### 3.2 Camera Bridge (camera_bridge.py)

- Processes incoming JPEG binary payloads.
- Attaches synchronized std_msgs/msg/Header timestamps.
- Publishes sensor_msgs/msg/CompressedImage to image_raw/compressed with qos_profile_sensor_data (Best Effort).

### 3.3 IMU Bridge (imu_bridge.py)

- Receives linear acceleration, angular velocity, and Euler orientation angles from browser.
- Converts Euler angles to normalized Quaternion orientation (x, y, z, w).
- Publishes sensor_msgs/msg/Imu to imu/data_raw with Best Effort QoS.

### 3.4 Battery Bridge (battery_bridge.py)

- Parses level (0.0 - 1.0) and charging boolean from Battery Status API.
- Publishes sensor_msgs/msg/BatteryState to /robot/battery.

### 3.5 GPS Bridge (gps_bridge.py)

- Parses latitude, longitude, altitude, and accuracy (meters).
- Computes 3x3 horizontal position covariance matrix (var = accuracy^2).
- Publishes sensor_msgs/msg/NavSatFix to /robot/gps.

## 4. Published Topics Specification

| Topic Name | Type | QoS Profile | Description |
| :--- | :--- | :--- | :--- |
| image_raw/compressed | sensor_msgs/msg/CompressedImage | Best Effort | JPEG compressed camera frames |
| imu/data_raw | sensor_msgs/msg/Imu | Best Effort | Linear accel, angular velocity, orientation quaternion |
| /robot/battery | sensor_msgs/msg/BatteryState | Best Effort | Battery percentage and charging status |
| /robot/gps | sensor_msgs/msg/NavSatFix | Best Effort | Latitude, longitude, altitude, and covariance |
| mobile_sensor_bridge/device_info | std_msgs/msg/String | Reliable | Connected smartphone metadata (JSON) |
| /camera/exposure_metadata | std_msgs/msg/String | Best Effort | Per-frame camera exposure time, ISO, and sequence metadata (JSON) |

## 5. Hardware Validation & Verification

- Primary Target Device: Samsung Galaxy S26 Ultra (Android)
- Verified Sensors & Pipelines:
  - Camera Video Capture & JPEG Compression Pipeline
  - 9-Axis IMU (Acceleration, Angular Velocity, Orientation Quaternion)
  - Battery Telemetry Pipeline (/robot/battery)
  - Geolocation GPS Pipeline (/robot/gps with Covariance)
  - Multithreaded rosbag2 Session Recording (.mcap storage)
