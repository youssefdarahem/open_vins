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

#include <algorithm>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#if REALSENSE_AVAILABLE
#include <librealsense2/rs.hpp>
#endif

#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "state/State.h"
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
  cv::Mat image;
  int cam_id;

  bool operator<(const Measurement &other) const { return timestamp < other.timestamp; }
};

// Thread-safe queue for measurements
class SafeQueue {
public:
  void push(const Measurement &m) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(m);
    cond_.notify_one();
  }

  bool pop(Measurement &m, int timeout_ms = 2000) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty(); })) {
      return false;
    }
    m = queue_.front();
    queue_.pop();
    return true;
  }

private:
  std::queue<Measurement> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#if REALSENSE_AVAILABLE
struct ImuSync {
  std::deque<std::pair<double, Eigen::Vector3d>> accel_queue;
  std::deque<std::pair<double, Eigen::Vector3d>> gyro_queue;

  // Try to match gyro and accel
  // Return true if a combined measurement is ready
  bool process(const rs2::motion_frame &mf, Measurement &out) {
    double t = mf.get_timestamp() * 1e-3;
    rs2_vector data = mf.get_motion_data();
    Eigen::Vector3d val(data.x, data.y, data.z);

    if (mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
      accel_queue.push_back({t, val});
    } else {
      gyro_queue.push_back({t, val});
    }

    if (gyro_queue.empty() || accel_queue.empty())
      return false;

    // Process oldest gyro
    auto gyro = gyro_queue.front();

    // Find accel samples around gyro time
    if (accel_queue.back().first < gyro.first)
      return false; // Need more accel

    // Find accel before and after
    auto it = std::lower_bound(
        accel_queue.begin(), accel_queue.end(), gyro,
        [](const std::pair<double, Eigen::Vector3d> &a, const std::pair<double, Eigen::Vector3d> &b) { return a.first < b.first; });

    if (it == accel_queue.begin()) {
      gyro_queue.pop_front();
      return false;
    }

    auto it_prev = it - 1;

    // Interpolate
    double t1 = it_prev->first;
    double t2 = it->first;
    double alpha = (gyro.first - t1) / (t2 - t1);

    Eigen::Vector3d accel_interp = it_prev->second * (1.0 - alpha) + it->second * alpha;

    out.timestamp = gyro.first;
    out.is_imu = true;
    out.wm = gyro.second;
    out.am = accel_interp;

    gyro_queue.pop_front();

    // Clean up old accels
    while (accel_queue.size() > 1 && accel_queue[1].first < (gyro_queue.empty() ? 0 : gyro_queue.front().first)) {
      accel_queue.pop_front();
    }

    return true;
  }
};
#endif

void load_imu_data(string path, vector<Measurement> &measurements) {
  ifstream file(path);
  if (!file.is_open()) {
    PRINT_ERROR(RED "Could not open IMU file: %s\n" RESET, path.c_str());
    exit(EXIT_FAILURE);
  }

  string line;
  while (getline(file, line)) {
    if (line.empty() || line[0] == '#')
      continue;

    stringstream ss(line);
    string item;
    vector<string> parts;
    while (getline(ss, item, ',')) {
      parts.push_back(item);
    }

    if (parts.size() < 7)
      continue;

    Measurement m;
    m.timestamp = stod(parts[0]) * 1e-9; // ns to s
    m.is_imu = true;
    m.wm << stod(parts[1]), stod(parts[2]), stod(parts[3]);
    m.am << stod(parts[4]), stod(parts[5]), stod(parts[6]);
    measurements.push_back(m);
  }
}

void load_camera_data(string path, int cam_id, vector<Measurement> &measurements) {
  ifstream file(path);
  if (!file.is_open()) {
    PRINT_ERROR(RED "Could not open Camera file: %s\n" RESET, path.c_str());
    exit(EXIT_FAILURE);
  }

  string line;
  while (getline(file, line)) {
    if (line.empty() || line[0] == '#')
      continue;

    stringstream ss(line);
    string item;
    vector<string> parts;
    while (getline(ss, item, ',')) {
      parts.push_back(item);
    }

    if (parts.size() < 2)
      continue;

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

  // Open output file
  ofstream traj_file("trajectory.txt");
  traj_file << fixed << setprecision(9);

  if (dataset_path == "realsense") {
#if REALSENSE_AVAILABLE
    PRINT_INFO("Starting RealSense D457...\n");

    SafeQueue queue;
    ImuSync imu_sync;
    std::mutex imu_mutex;

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    if (params.state_options.num_cameras >= 1)
      cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    if (params.state_options.num_cameras >= 2)
      cfg.enable_stream(RS2_STREAM_INFRARED, 2);

    std::function<void(const rs2::frame &)> process_frame;
    process_frame = [&](const rs2::frame &frame) {
      // PRINT_INFO("Frame received: %s %d format: %s\n", frame.get_profile().stream_name().c_str(), frame.get_profile().stream_index(),
      // rs2_format_to_string(frame.get_profile().format()));
      if (auto fs = frame.as<rs2::frameset>()) {
        for (const rs2::frame &f : fs) {
          process_frame(f);
        }
        return;
      }

      if (auto mf = frame.as<rs2::motion_frame>()) {
        Measurement m;
        std::lock_guard<std::mutex> lock(imu_mutex);
        if (imu_sync.process(mf, m)) {
          // PRINT_INFO("Pushing IMU\n");
          queue.push(m);
        }
      } else if (auto vf = frame.as<rs2::video_frame>()) {
        // PRINT_INFO("Processing Video Frame: %s %d\n", vf.get_profile().stream_name().c_str(), vf.get_profile().stream_index());
        Measurement m;
        m.timestamp = vf.get_timestamp() * 1e-3;
        m.is_imu = false;

        void *data = (void *)vf.get_data();
        if (!data) {
          PRINT_ERROR(RED "Video frame has no data!\n" RESET);
          return;
        }

        // Handle different formats
        if (vf.get_profile().format() == RS2_FORMAT_Y8) {
          m.image = cv::Mat(cv::Size(vf.get_width(), vf.get_height()), CV_8UC1, data, cv::Mat::AUTO_STEP).clone();
        } else if (vf.get_profile().format() == RS2_FORMAT_Y16) {
          // Convert Y16 to Y8
          cv::Mat img16(cv::Size(vf.get_width(), vf.get_height()), CV_16UC1, data, cv::Mat::AUTO_STEP);
          img16.convertTo(m.image, CV_8UC1, 1.0 / 256.0); // Simple scaling
        } else if (vf.get_profile().format() == RS2_FORMAT_RGB8 || vf.get_profile().format() == RS2_FORMAT_BGR8) {
          cv::Mat imgColor(cv::Size(vf.get_width(), vf.get_height()), CV_8UC3, data, cv::Mat::AUTO_STEP);
          cv::cvtColor(imgColor, m.image, cv::COLOR_RGB2GRAY);
        } else {
          PRINT_WARNING(YELLOW "Unsupported frame format: %s\n" RESET, rs2_format_to_string(vf.get_profile().format()));
          return;
        }

        if (vf.get_profile().stream_index() == 1)
          m.cam_id = 0;
        else if (vf.get_profile().stream_index() == 2)
          m.cam_id = 1;
        else
          m.cam_id = 0;

        // PRINT_INFO("Pushing camera frame %d at %.4f\n", m.cam_id, m.timestamp);
        queue.push(m);
      } else {
        PRINT_WARNING(YELLOW "Frame is neither motion nor video! Profile: %s\n" RESET, frame.get_profile().stream_name().c_str());
      }
    };

    auto callback = [&](const rs2::frame &frame) {
      try {
        process_frame(frame);
      } catch (const std::exception &e) {
        PRINT_ERROR(RED "Exception in callback: %s\n" RESET, e.what());
      }
    };

    try {
      pipe.start(cfg, callback);
      PRINT_INFO("RealSense pipeline started successfully.\n");
    } catch (const rs2::error &e) {
      PRINT_ERROR(RED "RealSense error calling start: %s\n" RESET, e.what());
      return EXIT_FAILURE;
    } catch (const std::exception &e) {
      PRINT_ERROR(RED "Error calling start: %s\n" RESET, e.what());
      return EXIT_FAILURE;
    }

    Measurement m;
    int imu_count = 0;
    while (true) {
      if (queue.pop(m)) {
        if (m.is_imu) {
          imu_count++;
          if (imu_count % 100 == 0) {
            PRINT_INFO("Processed 100 IMU messages. Last TS: %.4f\n", m.timestamp);
          }
          ImuData imu_data;
          imu_data.timestamp = m.timestamp;
          imu_data.wm = m.wm;
          imu_data.am = m.am;
          sys->feed_measurement_imu(imu_data);
        } else {
          CameraData cam_data;
          cam_data.timestamp = m.timestamp;
          cam_data.sensor_ids.push_back(m.cam_id);
          cam_data.images.push_back(m.image);
          cam_data.masks.push_back(cv::Mat::zeros(m.image.rows, m.image.cols, CV_8UC1));

          // PRINT_INFO("Popped camera frame %d at %.9f. IMU count: %d. Initialized: %s\n", m.cam_id, m.timestamp, imu_count,
          // sys->initialized() ? "YES" : "NO");
          sys->feed_measurement_camera(cam_data);

          if (sys->initialized()) {
            auto state = sys->get_state();
            Eigen::Vector3d p = state->_imu->pos();
            Eigen::Vector4d q = state->_imu->quat();
            traj_file << m.timestamp << " " << p(0) << " " << p(1) << " " << p(2) << " " << q(0) << " " << q(1) << " " << q(2) << " "
                      << q(3) << endl;
          }
        }
      } else {
        PRINT_WARNING(YELLOW "Waiting for data from RealSense...\n" RESET);
      }
    }
#else
    PRINT_ERROR(RED "RealSense support not compiled.\n" RESET);
    return EXIT_FAILURE;
#endif
  } else {
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

        const auto &m = imu_measurements[imu_idx];
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
        const auto &m_curr = cam_measurements[cam_idx];
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

        // Save trajectory
        if (sys->initialized()) {
          auto state = sys->get_state();
          Eigen::Vector3d p = state->_imu->pos();
          Eigen::Vector4d q = state->_imu->quat();

          // Format: timestamp tx ty tz qx qy qz qw
          traj_file << cam_data.timestamp << " " << p(0) << " " << p(1) << " " << p(2) << " " << q(0) << " " << q(1) << " " << q(2) << " "
                    << q(3) << endl;
        }
      }
    }
  }

  traj_file.close();
  return EXIT_SUCCESS;
}
