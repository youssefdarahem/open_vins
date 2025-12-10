#!/usr/bin/env python3
"""
Run OpenVINS on extracted dataset (EuRoC format).
This is a helper script that shows how to prepare data for OpenVINS.

After extracting your rosbag with rosbag_to_dataset.py, you need to:
1. Create a config YAML file with proper calibration
2. Write a C++ program that loads images/IMU and feeds to VioManager

This script helps verify your extracted data is correct.
"""

import argparse
import os
import csv
from pathlib import Path


def verify_dataset(dataset_dir):
    """Verify the extracted dataset structure."""
    mav0 = os.path.join(dataset_dir, 'mav0')
    
    if not os.path.exists(mav0):
        print(f"ERROR: {mav0} not found!")
        return False
    
    # Check IMU
    imu_csv = os.path.join(mav0, 'imu0', 'data.csv')
    if os.path.exists(imu_csv):
        with open(imu_csv, 'r') as f:
            reader = csv.reader(f)
            imu_count = sum(1 for row in reader if not row[0].startswith('#'))
        print(f"✓ IMU data: {imu_count} measurements")
    else:
        print(f"✗ IMU data not found: {imu_csv}")
        return False
    
    # Check cameras
    cam_idx = 0
    while True:
        cam_dir = os.path.join(mav0, f'cam{cam_idx}')
        if not os.path.exists(cam_dir):
            break
        
        cam_csv = os.path.join(cam_dir, 'data.csv')
        cam_data = os.path.join(cam_dir, 'data')
        
        if os.path.exists(cam_csv):
            with open(cam_csv, 'r') as f:
                reader = csv.reader(f)
                img_count = sum(1 for row in reader if not row[0].startswith('#'))
            
            # Count actual images
            if os.path.exists(cam_data):
                actual_imgs = len([f for f in os.listdir(cam_data) if f.endswith('.png')])
                print(f"✓ Camera {cam_idx}: {img_count} entries, {actual_imgs} images on disk")
            else:
                print(f"✓ Camera {cam_idx}: {img_count} entries (no data folder)")
        else:
            print(f"✗ Camera {cam_idx} CSV not found")
        
        cam_idx += 1
    
    if cam_idx == 0:
        print("✗ No camera data found!")
        return False
    
    print(f"\nDataset structure looks valid!")
    return True


def get_time_range(dataset_dir):
    """Get time range of the dataset."""
    imu_csv = os.path.join(dataset_dir, 'mav0', 'imu0', 'data.csv')
    
    first_ts = None
    last_ts = None
    
    with open(imu_csv, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            ts = int(line.split(',')[0])
            if first_ts is None:
                first_ts = ts
            last_ts = ts
    
    duration = (last_ts - first_ts) / 1e9
    print(f"Dataset duration: {duration:.2f} seconds")
    print(f"Start time: {first_ts} ns")
    print(f"End time: {last_ts} ns")
    
    return first_ts, last_ts


def main():
    parser = argparse.ArgumentParser(description='Verify extracted dataset for OpenVINS')
    parser.add_argument('dataset_dir', help='Path to extracted dataset directory')
    
    args = parser.parse_args()
    
    print(f"Verifying dataset: {args.dataset_dir}\n")
    
    if verify_dataset(args.dataset_dir):
        print()
        get_time_range(args.dataset_dir)
        print()
        print("To use with OpenVINS (ROS-free), you need to write a C++ loader.")
        print("See run_simulation.cpp for an example of feeding data to VioManager.")


if __name__ == '__main__':
    main()
