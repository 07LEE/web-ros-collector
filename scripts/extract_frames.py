#!/usr/bin/env python3
"""Extracts compressed images from a rosbag2 MCAP file, sorts them by timestamp,
and analyzes frame sequence continuity to detect frame drops vs out-of-order delivery.
"""

import argparse
import json
import os
import sys
from typing import Dict, List, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def extract_bag(bag_path: str, output_dir: str):
    """Extracts images and metadata from rosbag2.

    Args:
        bag_path (str): Path to the rosbag2 directory or mcap file.
        output_dir (str): Directory where extracted JPEG frames will be saved.
    """
    os.makedirs(output_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] Failed to open rosbag at {bag_path}: {e}")
        sys.exit(1)

    images: List[Tuple[float, str, bytes]] = []  # (stamp_sec, format, raw_bytes)
    metadata_map: Dict[float, dict] = {}

    print(f"[INFO] Reading rosbag: {bag_path}")
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/image_raw/compressed':
            msg = deserialize_message(data, CompressedImage)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            fmt = msg.format or 'jpeg'
            images.append((ts, fmt, bytes(msg.data)))
        elif topic == '/camera/exposure_metadata':
            msg = deserialize_message(data, String)
            try:
                meta = json.loads(msg.data)
                ts = meta.get('stamp_sec', 0) + meta.get('stamp_nanosec', 0) * 1e-9
                metadata_map[ts] = meta
            except Exception:
                pass

    print(f"[INFO] Total images found: {len(images)}")
    if not images:
        print("[WARN] No images found in bag file.")
        return

    # Sort images strictly by timestamp
    images.sort(key=lambda x: x[0])

    # Sequence analysis
    seqs = []
    for ts, _, _ in images:
        if ts in metadata_map and metadata_map[ts].get('frame_seq') is not None:
            seqs.append(metadata_map[ts]['frame_seq'])

    if seqs:
        out_of_order_count = sum(1 for i in range(len(seqs)-1) if seqs[i+1] < seqs[i])
        seq_diffs = [seqs[i+1] - seqs[i] for i in range(len(seqs)-1)]
        dropped_frames = sum(d - 1 for d in seq_diffs if d > 1)
        print(f"[ANALYSIS] Frame sequence range: {seqs[0]} -> {seqs[-1]}")
        print(f"[ANALYSIS] Out-of-order arrivals (network/thread level): {out_of_order_count}")
        print(f"[ANALYSIS] Dropped frames (client-side skipped): {dropped_frames}")
    else:
        print("[INFO] No frame_seq metadata available in this bag.")

    # Save extracted frames with zero-padded index and timestamp in filename
    meta_out_path = os.path.join(output_dir, "metadata_index.json")
    frame_index = []

    for idx, (ts, fmt, raw_bytes) in enumerate(images):
        ext = 'jpg' if 'jpeg' in fmt.lower() else 'png'
        filename = f"frame_{idx:05d}_{ts:.6f}.{ext}"
        filepath = os.path.join(output_dir, filename)

        with open(filepath, 'wb') as f:
            f.write(raw_bytes)

        meta = metadata_map.get(ts, {})
        frame_info = {
            'index': idx,
            'filename': filename,
            'timestamp': ts,
            'frame_seq': meta.get('frame_seq'),
            'exposure_time_us': meta.get('exposure_time_us'),
            'iso': meta.get('iso')
        }
        frame_index.append(frame_info)

    with open(meta_out_path, 'w') as f:
        json.dump(frame_index, f, indent=2)

    print(f"[SUCCESS] Extracted {len(images)} frames to {output_dir}")
    print(f"[SUCCESS] Saved frame metadata index to {meta_out_path}")


def main():
    parser = argparse.ArgumentParser(description="Extract timestamp-sorted frames from ROS2 bag.")
    parser.add_argument("bag_path", help="Path to rosbag2 directory or mcap file")
    parser.add_argument("--output", "-o", default="extracted_frames", help="Output directory for extracted JPEGs")
    args = parser.parse_args()

    extract_bag(args.bag_path, args.output)


if __name__ == '__main__':
    main()
