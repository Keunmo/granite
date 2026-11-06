// SPDX-License-Identifier: MIT
// Parts of the Raspberry Pi live run that granite_pi_cam_imu_vio and
// granite_vio --live use: the EuRoC recording, the "camera.*" keys of the
// config file, and the command line options of the sensors.
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <CLI/CLI.hpp>
#include <cereal/archives/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/concurrent_queue.h>

#include <granite/calibration/calibration.hpp>
#include <granite/device/pi_camera_imu.h>
#include <granite/optical_flow/optical_flow.h>
#include <granite/utils/imu_types.h>

namespace granite {
namespace pi_live {

// Writes the EuRoC layout that granite_calibrate, granite_calibrate_imu and
// granite_vio read.
class EurocRecorder {
 public:
  explicit EurocRecorder(const std::string& path) : root(path + "/mav0") {
    std::filesystem::create_directories(root + "/cam0/data");
    std::filesystem::create_directories(root + "/imu0");
    images.open(root + "/cam0/data.csv");
    imu.open(root + "/imu0/data.csv");
    if (!images || !imu) throw std::runtime_error("Cannot create recording in " + path);
    images << "#timestamp [ns],filename\n";
    imu << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
           "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        << std::setprecision(10);
  }

  void write(const OpticalFlowInput& frame) {
    const auto& image = *frame.img_data[0].img;
    cv::Mat pixels(image.h, image.w, CV_8UC1);
    for (size_t y = 0; y < image.h; ++y) {
      for (size_t x = 0; x < image.w; ++x)
        pixels.at<uint8_t>(y, x) = static_cast<uint8_t>(image(x, y));
    }
    const std::string name = std::to_string(frame.t_ns) + ".png";
    if (!cv::imwrite(root + "/cam0/data/" + name, pixels, {cv::IMWRITE_PNG_COMPRESSION, 1}))
      throw std::runtime_error("Cannot write image " + name);
    images << frame.t_ns << ',' << name << '\n';
  }

  void write(const ImuData& sample) {
    imu << sample.t_ns << ',' << sample.gyro.x() << ',' << sample.gyro.y() << ','
        << sample.gyro.z() << ',' << sample.accel.x() << ',' << sample.accel.y() << ','
        << sample.accel.z() << '\n';
  }

 private:
  std::string root;
  std::ofstream images, imu;
};

// Writes the recording on its own thread: PNG encoding must not delay the
// main loop. A full image queue drops the image from the recording.
class RecordWriter {
 public:
  explicit RecordWriter(const std::string& path) : recorder(path) {
    images.set_capacity(60);
    thread = std::thread([this] { run(); });
  }
  ~RecordWriter() { finish(); }

  void finish() {
    if (!thread.joinable()) return;
    images.push(nullptr);
    imu.push(nullptr);
    thread.join();
  }

  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr> images;
  tbb::concurrent_bounded_queue<ImuData::Ptr> imu;
  std::atomic<uint64_t> written_images{0}, written_imu{0};

 private:
  void run() {
    bool images_done = false, imu_done = false;
    while (!images_done || !imu_done) {
      bool idle = true;
      OpticalFlowInput::Ptr image;
      if (!images_done && images.try_pop(image)) {
        idle = false;
        if (!image) images_done = true;
        else {
          recorder.write(*image);
          ++written_images;
        }
      }
      ImuData::Ptr sample;
      while (!imu_done && imu.try_pop(sample)) {
        idle = false;
        if (!sample) imu_done = true;
        else {
          recorder.write(*sample);
          ++written_imu;
        }
      }
      if (idle) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  EurocRecorder recorder;
  std::thread thread;
};

inline double percentile(std::vector<double> values, double p) {
  if (values.empty()) return -1;
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1, static_cast<size_t>(p * (values.size() - 1) + 0.5))];
}

inline int64_t monotonicNs() {
  timespec time{};
  clock_gettime(CLOCK_MONOTONIC, &time);
  return int64_t(time.tv_sec) * 1'000'000'000 + time.tv_nsec;
}

// Adds the sensor options to the command line. loadCameraConfig() reads the
// same values from the "camera.*" keys of the config file.
inline void addSensorOptions(CLI::App& app, PiCameraImuOptions& options, bool& resize_half) {
  app.add_option("--width", options.width, "Sensor mode width (1920, 1280)");
  app.add_option("--height", options.height, "Sensor mode height (1200, 1080, 720)");
  app.add_option("--fps", options.fps, "Camera frame rate");
  app.add_option("--downscale", options.downscale,
                 "Output image at 1/N of the sensor mode (N x N average without Bayer pattern)");
  app.add_flag("--resize-half", resize_half, "Same as --downscale 2");
  app.add_option("--metering-circle", options.metering_circle,
                 "Auto exposure meters this circle cx cy r of the output image")
      ->expected(3);
  app.add_option("--exposure", options.exposure_ms, "Manual exposure in ms (0: auto)");
  app.add_option("--max-exposure", options.max_exposure_ms, "Auto exposure limit in ms");
  app.add_option("--gain", options.gain_db, "Analogue gain in dB for manual exposure (0-11)");
  app.add_option("--max-gain", options.max_gain, "Auto exposure analogue gain limit (analogue_gain / 100)");
  app.add_option("--brightness", options.brightness_target, "Auto exposure target mean (0-255)");
  app.add_option("--i2c", options.i2c_device, "IMU I2C adapter device");
  app.add_option("--imu-address", options.imu_address, "IMU 7-bit I2C address");
  app.add_option("--imu-rate", options.imu_rate, "IMU sample rate in Hz");
}

// Reads the optional "camera.*" keys next to the VioConfig keys. VioConfig
// ignores them. A command line option has priority over its key.
inline void loadCameraConfig(const std::string& path, const CLI::App& app,
                             PiCameraImuOptions& options) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open config " + path);
  cereal::JSONInputArchive archive(input);
  archive.setNextName("value0");
  archive.startNode();
  const auto load = [&](const char* key, const char* option, auto& value) {
    if (app.count(option) > 0) return;
    try {
      archive(cereal::make_nvp(key, value));
    } catch (const cereal::Exception&) {
      // The key is not in the file; keep the default.
    }
  };
  load("camera.width", "--width", options.width);
  load("camera.height", "--height", options.height);
  load("camera.fps", "--fps", options.fps);
  load("camera.exposure_ms", "--exposure", options.exposure_ms);
  load("camera.gain_db", "--gain", options.gain_db);
  load("camera.max_exposure_ms", "--max-exposure", options.max_exposure_ms);
  load("camera.max_gain", "--max-gain", options.max_gain);
  load("camera.brightness_target", "--brightness", options.brightness_target);
  load("camera.metering_circle", "--metering-circle", options.metering_circle);
  load("camera.i2c_device", "--i2c", options.i2c_device);
  load("camera.imu_rate", "--imu-rate", options.imu_rate);
  bool resize_half = false;
  load("camera.resize_half", "--resize-half", resize_half);
  if (resize_half && app.count("--downscale") == 0) options.downscale = 2;
  load("camera.downscale", "--downscale", options.downscale);
  archive.finishNode();
}

// Scales a calibration at the sensor mode to the output image, or checks that
// it is already at the output image.
inline void fitCalibration(Calibration<double>& calib, const PiCameraImuOptions& options) {
  if (calib.intrinsics.size() != 1 || calib.T_i_c.size() != 1 || calib.resolution.size() != 1)
    throw std::runtime_error("Expected a calibration with one camera");
  const Eigen::Vector2i sensor_size(options.width, options.height);
  const Eigen::Vector2i output_size = sensor_size / options.downscale;
  if (calib.resolution[0] == sensor_size) {
    scaleCalibration(calib, options.downscale);
  } else if (calib.resolution[0] != output_size) {
    throw std::runtime_error(
        "Calibration resolution " + std::to_string(calib.resolution[0].x()) + "x" +
        std::to_string(calib.resolution[0].y()) + " is neither the sensor mode " +
        std::to_string(sensor_size.x()) + "x" + std::to_string(sensor_size.y()) +
        " nor the output " + std::to_string(output_size.x()) + "x" +
        std::to_string(output_size.y()));
  }
}

}  // namespace pi_live
}  // namespace granite
