#!/usr/bin/env python3
"""Filter extracted frames based on IMU rotational velocity and exposure time
to remove motion-blurred frames before COLMAP / 3DGS processing.
"""

import argparse
import json
import os
import shutil
import sys
from typing import List, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def parse_imu_gyro(bag_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Reads IMU rotational velocity from bag and returns (timestamps, gyro_magnitudes_rad_s).

    Args:
        bag_path (str): Path to rosbag2.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Timestamps array and gyro magnitude rad/s array.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] Failed to open rosbag at {bag_path}: {e}")
        sys.exit(1)

    timestamps = []
    gyro_mags = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/imu/data_raw':
            msg = deserialize_message(data, Imu)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z
            mag = np.sqrt(gx**2 + gy**2 + gz**2)
            timestamps.append(ts)
            gyro_mags.append(mag)

    return np.array(timestamps), np.array(gyro_mags)


def select_frames(bag_path: str, frames_dir: str, output_dir: str, threshold: float):
    """Filters frames by computing motion blur score from IMU gyro and exposure time.

    Blur Score (rad) ≈ angular_velocity_rad_s * exposure_time_seconds

    Args:
        bag_path (str): Path to rosbag2.
        frames_dir (str): Directory containing extracted frames and metadata_index.json.
        output_dir (str): Directory to copy selected non-blurred frames.
        threshold (float): Angular motion threshold (in radians during exposure window).
    """
    meta_path = os.path.join(frames_dir, "metadata_index.json")
    if not os.path.exists(meta_path):
        print(f"[ERROR] Metadata index file not found at {meta_path}. Run extract_frames.py first.")
        sys.exit(1)

    with open(meta_path, 'r') as f:
        frames_meta = json.load(f)

    imu_ts, imu_gyro = parse_imu_gyro(bag_path)
    if len(imu_ts) == 0:
        print("[WARN] No IMU data found in bag file. Cannot compute IMU-based blur score.")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)

    kept_count = 0
    discarded_count = 0

    print(f"[INFO] Filtering {len(frames_meta)} frames with angular motion threshold = {threshold:.5f} rad")

    selected_meta = []
    for info in frames_meta:
        ts = info['timestamp']
        exp_us = info.get('exposure_time_us') or 8300  # Default to 1/120s (8300us) if missing
        exp_sec = exp_us * 1e-6

        # Find IMU sample closest in time to frame timestamp
        idx = np.searchsorted(imu_ts, ts)
        idx = np.clip(idx, 0, len(imu_ts) - 1)

        # Average gyro magnitude in a +/- 50ms window around frame capture
        window_mask = np.abs(imu_ts - ts) <= 0.05
        if np.any(window_mask):
            avg_gyro = np.mean(imu_gyro[window_mask])
        else:
            avg_gyro = imu_gyro[idx]

        # Blur score estimate: total rotation angle during exposure (radians)
        blur_score_rad = avg_gyro * exp_sec

        info['blur_score_rad'] = blur_score_rad
        info['avg_gyro_rad_s'] = avg_gyro

        src_path = os.path.join(frames_dir, info['filename'])
        dst_path = os.path.join(output_dir, info['filename'])

        if blur_score_rad <= threshold:
            if os.path.exists(src_path):
                shutil.copy2(src_path, dst_path)
            selected_meta.append(info)
            kept_count += 1
        else:
            discarded_count += 1

    selected_meta_path = os.path.join(output_dir, "selected_metadata.json")
    with open(selected_meta_path, 'w') as f:
        json.dump(selected_meta, f, indent=2)

    print(f"[SUCCESS] Filtered completed: Kept {kept_count} frames, Discarded {discarded_count} frames.")
    print(f"[SUCCESS] Saved selected frames to {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Filter out motion-blurred frames using IMU + exposure metadata.")
    parser.add_argument("bag_path", help="Path to rosbag2 directory or mcap file")
    parser.add_argument("frames_dir", help="Directory containing extracted frames from extract_frames.py")
    parser.add_argument("--output", "-o", default="colmap_input_frames", help="Output directory for selected sharp frames")
    parser.add_argument("--threshold", "-t", type=float, default=0.005, help="Angular blur threshold in radians (default: 0.005 rad)")
    args = parser.parse_args()

    select_frames(args.bag_path, args.frames_dir, args.output, args.threshold)


if __name__ == '__main__':
    main()
