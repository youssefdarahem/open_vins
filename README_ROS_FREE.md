# Running OpenVINS without ROS

This guide explains how to convert a ROS bag into a format compatible with the ROS-free `run_dataset` loader and visualize the results.

## Prerequisites

- Python 3 with `rosbag` installed (usually comes with `python3-rosbag` or standard ROS installation).
- OpenVINS built with the `run_dataset` executable.

## Step 1: Convert ROS Bag to Dataset

Use the provided script to extract images and IMU data from your bag file.

```bash
python3 scripts/rosbag_to_dataset.py <path_to_bag> <output_directory> \
    --imu_topic /imu0 \
    --cam0_topic /cam0/image_raw \
    --cam1_topic /cam1/image_raw
```

**Example:**
```bash
python3 scripts/rosbag_to_dataset.py sim_bags/aruco/indoor.bag ov_data/sim_bags_extracted/indoor \
    --imu_topic /imu0 \
    --cam0_topic /cam0/image_raw \
    --cam1_topic /cam1/image_raw
```

## Step 2: Build the Loader

Ensure the `run_dataset` executable is compiled.

```bash
cd ov_msckf/build
cmake ..
make -j4
```

## Step 3: Run the Estimator

Run the estimator using a configuration file and the converted dataset path.

```bash
./run_dataset <path_to_config> <path_to_dataset>
```

**Example:**
```bash
./run_dataset ../../config/rpng_aruco/estimator_config.yaml ../../ov_data/sim_bags_extracted/indoor
```

This will generate a `trajectory.txt` file in the current directory.

## Step 4: Visualize the Trajectory

Use the python script to plot the generated trajectory.

```bash
python3 plot_traj.py trajectory.txt
```

This will save the plot to `trajectory_plot.png`.
