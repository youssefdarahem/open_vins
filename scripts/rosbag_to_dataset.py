#!/usr/bin/env python3
"""
Convert ROS bag to EuRoC-like dataset format for use with OpenVINS.
This script doesn't require ROS installation.

Usage:
    python rosbag_to_dataset.py <rosbag_path> <output_dir> [options]

Example:
    python rosbag_to_dataset.py recording.bag ./my_dataset --imu_topic /imu0 --cam0_topic /cam0/image_raw
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from rosbags.rosbag1 import Reader as Reader1
    from rosbags.rosbag2 import Reader as Reader2
    # from rosbags.serde import deserialize_cdr, ros1_to_cdr
    from rosbags.typesys import get_types_from_msg, Stores, get_typestore
except ImportError as e:
    print(f"Please install rosbags: pip install rosbags. Error: {e}")
    sys.exit(1)


def create_output_dirs(output_dir, num_cameras):
    """Create EuRoC-style directory structure."""
    dirs = {
        'imu': os.path.join(output_dir, 'mav0', 'imu0'),
        'cameras': []
    }
    os.makedirs(dirs['imu'], exist_ok=True)
    
    for i in range(num_cameras):
        cam_dir = os.path.join(output_dir, 'mav0', f'cam{i}', 'data')
        os.makedirs(cam_dir, exist_ok=True)
        dirs['cameras'].append(cam_dir)
    
    return dirs


def detect_bag_version(bag_path):
    """Detect if bag is ROS1 or ROS2 format."""
    if os.path.isdir(bag_path):
        return 2  # ROS2 bags are directories
    with open(bag_path, 'rb') as f:
        header = f.read(13)
        if header == b'#ROSBAG V2.0\n':
            return 1
    return 1  # Default to ROS1


def process_ros1_bag(bag_path, output_dir, imu_topic, cam_topics):
    """Process ROS1 bag file."""
    num_cameras = len(cam_topics)
    dirs = create_output_dirs(output_dir, num_cameras)
    
    # IMU data file
    imu_file = open(os.path.join(dirs['imu'], 'data.csv'), 'w')
    imu_file.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],'
                   'a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
    
    # Camera data files
    cam_files = []
    for i in range(num_cameras):
        cam_csv = open(os.path.join(output_dir, 'mav0', f'cam{i}', 'data.csv'), 'w')
        cam_csv.write('#timestamp [ns],filename\n')
        cam_files.append(cam_csv)
    
    imu_count = 0
    cam_counts = [0] * num_cameras
    
    typestore = get_typestore(Stores.ROS1_NOETIC)
    
    with Reader1(bag_path) as reader:
        # Get all connections
        connections = [c for c in reader.connections]
        topics = {c.topic: c for c in connections}
        
        print(f"Available topics in bag:")
        for topic in topics:
            print(f"  {topic}")
        
        # Check if requested topics exist
        all_topics = [imu_topic] + cam_topics
        for topic in all_topics:
            if topic not in topics:
                print(f"WARNING: Topic '{topic}' not found in bag!")
        
        # Process messages
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            
            if topic == imu_topic:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                ts_ns = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
                wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                imu_file.write(f'{ts_ns},{wx},{wy},{wz},{ax},{ay},{az}\n')
                imu_count += 1
                
            elif topic in cam_topics:
                cam_idx = cam_topics.index(topic)
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                ts_ns = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
                
                # Decode image
                if 'compressed' in connection.msgtype.lower():
                    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
                else:
                    # Raw image
                    if msg.encoding in ['mono8', '8UC1']:
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                    elif msg.encoding in ['bgr8', '8UC3']:
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    elif msg.encoding in ['rgb8']:
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                    elif msg.encoding in ['mono16', '16UC1']:
                        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                        img = (img / 256).astype(np.uint8)
                    else:
                        print(f"Unknown encoding: {msg.encoding}, skipping")
                        continue
                
                # Save image
                img_filename = f'{ts_ns}.png'
                img_path = os.path.join(dirs['cameras'][cam_idx], img_filename)
                cv2.imwrite(img_path, img)
                cam_files[cam_idx].write(f'{ts_ns},{img_filename}\n')
                cam_counts[cam_idx] += 1
                
                if cam_counts[cam_idx] % 100 == 0:
                    print(f"  Processed {cam_counts[cam_idx]} images from cam{cam_idx}")
    
    # Close files
    imu_file.close()
    for f in cam_files:
        f.close()
    
    print(f"\nExtraction complete!")
    print(f"  IMU messages: {imu_count}")
    for i, count in enumerate(cam_counts):
        print(f"  Camera {i} images: {count}")
    
    return dirs


def main():
    parser = argparse.ArgumentParser(description='Convert ROS bag to EuRoC-like dataset format')
    parser.add_argument('bag_path', help='Path to ROS bag file')
    parser.add_argument('output_dir', help='Output directory for dataset')
    parser.add_argument('--imu_topic', default='/imu0', help='IMU topic name (default: /imu0)')
    parser.add_argument('--cam0_topic', default='/cam0/image_raw', help='Camera 0 topic (default: /cam0/image_raw)')
    parser.add_argument('--cam1_topic', default=None, help='Camera 1 topic for stereo (optional)')
    parser.add_argument('--list_topics', action='store_true', help='Just list available topics and exit')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag file not found: {args.bag_path}")
        sys.exit(1)
    
    bag_version = detect_bag_version(args.bag_path)
    print(f"Detected ROS{bag_version} bag format")
    
    if args.list_topics:
        typestore = get_typestore(Stores.ROS1_NOETIC)
        with Reader1(args.bag_path) as reader:
            print("Available topics:")
            for conn in reader.connections:
                print(f"  {conn.topic} [{conn.msgtype}]")
        return
    
    cam_topics = [args.cam0_topic]
    if args.cam1_topic:
        cam_topics.append(args.cam1_topic)
    
    print(f"Converting bag: {args.bag_path}")
    print(f"Output directory: {args.output_dir}")
    print(f"IMU topic: {args.imu_topic}")
    print(f"Camera topics: {cam_topics}")
    print()
    
    if bag_version == 1:
        process_ros1_bag(args.bag_path, args.output_dir, args.imu_topic, cam_topics)
    else:
        print("ROS2 bag support - please extend this script for ROS2 bags")
        sys.exit(1)


if __name__ == '__main__':
    main()
