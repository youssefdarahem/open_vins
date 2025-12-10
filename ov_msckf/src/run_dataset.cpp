/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace std;

// Global pointer to the system
std::shared_ptr<VioManager> sys;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

struct Measurement {
    double timestamp;
    bool is_imu;
    // IMU data
    Eigen::Vector3d wm, am;
    // Camera data
    string filename;
    int cam_id;
    
    bool operator<(const Measurement& other) const {
        return timestamp < other.timestamp;
    }
};

void load_imu_data(string path, vector<Measurement>& measurements) {
    ifstream file(path);
    if (!file.is_open()) {
        PRINT_ERROR(RED "Could not open IMU file: %s\n" RESET, path.c_str());
        exit(EXIT_FAILURE);
    }
    
    string line;
    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        stringstream ss(line);
        string item;
        vector<string> parts;
        while (getline(ss, item, ',')) {
            parts.push_back(item);
        }
        
        if (parts.size() < 7) continue;
        
        Measurement m;
        m.timestamp = stod(parts[0]) * 1e-9; // ns to s
        m.is_imu = true;
        m.wm << stod(parts[1]), stod(parts[2]), stod(parts[3]);
        m.am << stod(parts[4]), stod(parts[5]), stod(parts[6]);
        measurements.push_back(m);
    }
}

void load_camera_data(string path, int cam_id, vector<Measurement>& measurements) {
    ifstream file(path);
    if (!file.is_open()) {
        PRINT_ERROR(RED "Could not open Camera file: %s\n" RESET, path.c_str());
        exit(EXIT_FAILURE);
    }
    
    string line;
    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        stringstream ss(line);
        string item;
        vector<string> parts;
        while (getline(ss, item, ',')) {
            parts.push_back(item);
        }
        
        if (parts.size() < 2) continue;
        
        Measurement m;
        m.timestamp = stod(parts[0]) * 1e-9; // ns to s
        m.is_imu = false;
        m.cam_id = cam_id;
        m.filename = parts[1];
        measurements.push_back(m);
    }
}

int main(int argc, char **argv) {
    
    // Handle signals
    signal(SIGINT, signal_callback_handler);

    if (argc < 3) {
        PRINT_ERROR(RED "Usage: ./run_dataset <config_path> <dataset_path>\n" RESET);
        return EXIT_FAILURE;
    }

    string config_path = argv[1];
    string dataset_path = argv[2];

    // Load configuration
    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    
    // Verbosity
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    // Create VIO system
    VioManagerOptions params;
    params.print_and_load(parser);
    sys = std::make_shared<VioManager>(params);

    if (!parser->successful()) {
        PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
        std::exit(EXIT_FAILURE);
    }

    // Load measurements
    vector<Measurement> imu_measurements;
    vector<Measurement> cam_measurements;
    string mav0_path = dataset_path + "/mav0";
    
    PRINT_INFO("Loading IMU data...\n");
    load_imu_data(mav0_path + "/imu0/data.csv", imu_measurements);
    
    PRINT_INFO("Loading Camera data...\n");
    // Check how many cameras we have
    for (int i = 0; i < params.state_options.num_cameras; i++) {
        string cam_path = mav0_path + "/cam" + to_string(i) + "/data.csv";
        load_camera_data(cam_path, i, cam_measurements);
    }
    
    PRINT_INFO("Sorting measurements...\n");
    sort(imu_measurements.begin(), imu_measurements.end());
    sort(cam_measurements.begin(), cam_measurements.end());
    
    PRINT_INFO("Starting processing...\n");
    
    // Processing loop
    size_t imu_idx = 0;
    size_t cam_idx = 0;
    
    while (cam_idx < cam_measurements.size()) {
        // Get current camera timestamp
        double cam_time = cam_measurements[cam_idx].timestamp;
        
        // Feed IMU data up to cam_time + buffer
        // We use a small buffer (e.g. 20ms) to ensure we have IMU measurements *after* the camera time
        // This is required for interpolation in the propagator
        while (imu_idx < imu_measurements.size()) {
             if (imu_measurements[imu_idx].timestamp > cam_time + 0.05) 
                 break;
                 
             const auto& m = imu_measurements[imu_idx];
             ImuData imu_data;
             imu_data.timestamp = m.timestamp;
             imu_data.wm = m.wm;
             imu_data.am = m.am;
             sys->feed_measurement_imu(imu_data);
             imu_idx++;
        }
        
        // Process camera(s) at this timestamp
        CameraData cam_data;
        cam_data.timestamp = cam_time;
        
        // Collect all camera measurements at this timestamp (stereo/multi-cam)
        while (cam_idx < cam_measurements.size()) {
             const auto& m_curr = cam_measurements[cam_idx];
             if (abs(m_curr.timestamp - cam_time) > 1e-9) {
                 break;
             }
             
             string img_path = mav0_path + "/cam" + to_string(m_curr.cam_id) + "/data/" + m_curr.filename;
             cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
             
             if (!img.empty()) {
                 cam_data.sensor_ids.push_back(m_curr.cam_id);
                 cam_data.images.push_back(img);
                 cam_data.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1)); // Empty mask
             } else {
                 PRINT_WARNING(YELLOW "Could not read image: %s\n" RESET, img_path.c_str());
             }
             
             cam_idx++;
        }
        
        if (!cam_data.images.empty()) {
            PRINT_INFO("Processing camera at %.9f\n", cam_data.timestamp);
            sys->feed_measurement_camera(cam_data);
        }
    }
    
    return EXIT_SUCCESS;
}
