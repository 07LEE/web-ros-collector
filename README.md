# web-ros-collector

A high-performance Web-based ROS 2 Sensor Data Collector that streams camera frames and IMU sensor data from smartphone browsers to ROS 2 topics (`image_raw/compressed` and `imu/data_raw`).

- Package Documentation: [mobile_sensor_bridge/README.md](mobile_sensor_bridge/README.md)

## Quick Start

### Build Package

```bash
colcon build
```

### Launch Node

```bash
./scripts/run.sh
```

### Clean Artifacts

```bash
./scripts/clean.sh
```
