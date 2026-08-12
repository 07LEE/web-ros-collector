# SLAM & 3DGS Dataset Pipeline Architecture Guide

## 1. Overview

When utilizing Mobile Sensor Bridge as a dataset collector across arbitrary smartphone hardware without pre-existing checkerboard calibration files, a decoupled two-phase pipeline architecture is recommended.

## 2. Decoupled Pipeline Comparison

| Pipeline Phase | Data Collection Phase (ROS 2 Bridge) | Post-Processing Phase (3DGS / SfM) |
| :--- | :--- | :--- |
| Execution Mode | Raw Sensor Data Capture (Image, IMU, GPS, Battery) | Offline COLMAP + 3DGS Pipeline Execution |
| Intrinsics | Rough Initial Value based on ~75° FOV (f ≈ W × 0.65) (Optional) | COLMAP Auto-Intrinsics Optimization (Joint Refinement) |
| Advantages | Universal Plug-and-Play across any mobile browser | Maximum 3D Reconstruction Quality without Prior Calibration |

## 3. Workflow Details

### 3.1 Data Collection Phase (Online Capture)

- The mobile web client streams JPEG camera frames and IMU sensor data over HTTPS.
- The ROS 2 node records raw /image_raw/compressed and /imu/data_raw topics directly into rosbag2 files.
- If a camera calibration YAML file is not specified via camera_info_url, /camera_info topic publishing is explicitly bypassed to prevent publishing unverified intrinsics.

### 3.2 Post-Processing Phase (Offline Reconstruction)

- Extracted image frames from the recorded rosbag2 are fed into COLMAP.
- COLMAP uses camera models (such as SIMPLE_RADIAL or OPENCV) to estimate camera focal lengths (fx, fy), principal points (cx, cy), and radial distortion coefficients (k1, k2) directly from image feature correspondences.
- The estimated camera poses and intrinsic parameters are then used for high-fidelity 3D Gaussian Splatting (3DGS) training.
