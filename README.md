# web-ros-collector

A Web-based ROS 2 Sensor Data Collector that streams camera frames and IMU sensor data from smartphone browsers to ROS 2 topics (image_raw/compressed and imu/data_raw).

## Documentation

- Package Documentation: [mobile_sensor_bridge/README.md](mobile_sensor_bridge/README.md)
- System Architecture: [docs/system-architecture.md](docs/system-architecture.md)

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
