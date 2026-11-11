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
// same values from the "camera.*" and "imu.*" keys of the config file. The
// options of the xeye models and of imu_mavlink have the names of the xeye
// capture keys (camera: --<key>, IMU: --imu_<key>).
inline void addSensorOptions(CLI::App& app, PiCameraImuOptions& options, bool& resize_half) {
  auto& ae = options.auto_exposure;
  app.add_option(
      "--model", options.model,
      "kurokesu_ar0234_pi or b0429_pi (as the xeye node), or b0429 (granite)");
  app.add_option(
      "--port", options.port,
      "Video node of the rp1-cfe (empty: the node of the sensor graph)");
  app.add_option("--width", options.width,
                 "Sensor mode width (1920, 1280, 960; 0: 1920 for "
                 "kurokesu_ar0234_pi, else 1280)");
  app.add_option("--height", options.height,
                 "Sensor mode height (1200, 1080, 720, 600; 0: 1200 or 720)");
  app.add_option("--fps,--frame_rate", options.fps, "Camera frame rate");
  app.add_option("--downscale", options.downscale,
                 "Output image at 1/N of the sensor mode (N x N average without Bayer pattern)");
  app.add_flag("--resize-half", resize_half, "Same as --downscale 2");
  app.add_option("--sensor_binning", options.sensor_binning,
                 "kurokesu_ar0234_pi at --downscale 2 of 1920x1200: the 2x2 "
                 "binning mode with the halved crop in place of the host "
                 "reduction");
  app.add_option("--pixel_format", options.pixel_format,
                 "xeye models: gray8 (demosaiced)");
  app.add_option(
      "--exposure_time", options.exposure_time_ms,
      "xeye models: exposure in ms, the start value of the auto exposure");
  app.add_option(
      "--gain", options.gain,
      "xeye models: analogue_gain control value (kurokesu_ar0234_pi 13-64, "
      "b0429_pi 100-1200; -1: 14 or 100)");
  app.add_option("--num_v4l_buffer", options.num_v4l_buffer,
                 "xeye models: V4L2 buffers");
  app.add_option("--h_flip", options.h_flip,
                 "xeye models: sensor horizontal flip");
  app.add_option("--v_flip", options.v_flip,
                 "xeye models: sensor vertical flip");
  app.add_option("--crop", options.crop,
                 "xeye models: crop x y w h of the sensor mode (default: 288 0 "
                 "1344 1200 for kurokesu_ar0234_pi at 0x0; 0 0 1920 1200: no "
                 "crop)")
      ->expected(4);
  app.add_option("--auto_white_balance", options.auto_white_balance,
                 "xeye models: WB");
  app.add_option("--fused_wb", options.fused_wb,
                 "xeye models: WB gains from the previous frame");
  app.add_option("--black_level", options.black_level,
                 "xeye models: black level [10-bit]");
  app.add_option("--enable_gamma", options.enable_gamma,
                 "xeye models: gamma curve");
  app.add_option("--gamma_value", options.gamma_value,
                 "xeye models: gamma exponent");
  app.add_option("--auto_exposure", ae.auto_exposure,
                 "xeye models: auto exposure");
  app.add_option("--auto_gain", ae.auto_gain, "xeye models: auto gain");
  app.add_option("--auto_exposure_range", ae.exposure_range_ms,
                 "xeye models: auto exposure min max [ms]")
      ->expected(2);
  app.add_option(
         "--auto_gain_range", ae.gain_range_db,
         "xeye models: auto gain min max [dB] (default: the driver range)")
      ->expected(2);
  app.add_option("--auto_exp_prior", ae.prior,
                 "xeye models: exposure or gain changes first");
  app.add_option("--target_gray_value", ae.target_gray_value,
                 "xeye models: auto exposure target [%] of the full scale");
  app.add_option("--auto_exposure_ev_compensation", ae.ev_compensation,
                 "xeye models: target x 2^ev, [-3, 3]");
  app.add_option(
      "--auto_exposure_damping", ae.damping,
      "xeye models: part of the error that an update does not correct, [0, 1)");
  app.add_option("--auto_exposure_metering", ae.metering,
                 "xeye models: average, spot, or partial");
  app.add_option("--auto_exposure_roi", ae.roi,
                 "xeye models: metered region x y w h")
      ->expected(4);
  app.add_option("--auto_exposure_circle", ae.circle,
                 "xeye models: only the pixels in the circle cx cy r")
      ->expected(3);
  app.add_option(
         "--metering-circle", options.metering_circle,
         "b0429: auto exposure meters this circle cx cy r of the output image")
      ->expected(3);
  app.add_option("--exposure", options.exposure_ms,
                 "b0429: manual exposure in ms (0: auto)");
  app.add_option("--max-exposure", options.max_exposure_ms,
                 "b0429: auto exposure limit in ms");
  app.add_option("--gain-db", options.gain_db,
                 "b0429: analogue gain in dB for manual exposure (0-11)");
  app.add_option(
      "--max-gain", options.max_gain,
      "b0429: auto exposure analogue gain limit (analogue_gain / 100)");
  app.add_option("--brightness", options.brightness_target,
                 "b0429: auto exposure target mean (0-255)");
  app.add_option("--imu_type", options.imu_type,
                 "imu_mavlink (PX4 HIGHRES_IMU, as the xeye node) or mpu_i2c");
  app.add_option("--imu_udp", options.imu_udp,
                 "imu_mavlink: <bind address>:<port> of the router endpoint "
                 "(default 127.0.0.1:14601 without --imu_port)");
  app.add_option("--imu_port", options.imu_port,
                 "imu_mavlink: serial device in place of udp");
  app.add_option("--imu_baudrate", options.imu_baudrate,
                 "imu_mavlink: serial baud rate");
  app.add_option(
      "--imu_freq_hz", options.imu_freq_hz,
      "imu_mavlink: requested HIGHRES_IMU rate [Hz] (0: the FC rate)");
  app.add_option("--imu_send_odometry", options.imu_send_odometry,
                 "imu_mavlink: send each VIO state to the FC as ODOMETRY on "
                 "the same link");
  app.add_option("--i2c", options.i2c_device,
                 "mpu_i2c: IMU I2C adapter device");
  app.add_option("--imu-address", options.imu_address,
                 "mpu_i2c: IMU 7-bit I2C address");
  app.add_option("--imu-rate", options.imu_rate,
                 "mpu_i2c: IMU sample rate in Hz");
}

// Reads the optional "camera.*" and "imu.*" keys next to the VioConfig keys.
// VioConfig ignores them. A command line option has priority over its key.
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
  auto& ae = options.auto_exposure;
  load("camera.model", "--model", options.model);
  load("camera.port", "--port", options.port);
  load("camera.width", "--width", options.width);
  load("camera.height", "--height", options.height);
  load("camera.fps", "--fps", options.fps);
  load("camera.frame_rate", "--fps", options.fps);
  load("camera.pixel_format", "--pixel_format", options.pixel_format);
  load("camera.exposure_time", "--exposure_time", options.exposure_time_ms);
  load("camera.gain", "--gain", options.gain);
  load("camera.num_v4l_buffer", "--num_v4l_buffer", options.num_v4l_buffer);
  load("camera.h_flip", "--h_flip", options.h_flip);
  load("camera.v_flip", "--v_flip", options.v_flip);
  load("camera.crop", "--crop", options.crop);
  load("camera.auto_white_balance", "--auto_white_balance",
       options.auto_white_balance);
  load("camera.fused_wb", "--fused_wb", options.fused_wb);
  load("camera.black_level", "--black_level", options.black_level);
  load("camera.enable_gamma", "--enable_gamma", options.enable_gamma);
  load("camera.gamma_value", "--gamma_value", options.gamma_value);
  load("camera.auto_exposure", "--auto_exposure", ae.auto_exposure);
  load("camera.auto_gain", "--auto_gain", ae.auto_gain);
  load("camera.auto_exposure_range", "--auto_exposure_range",
       ae.exposure_range_ms);
  load("camera.auto_gain_range", "--auto_gain_range", ae.gain_range_db);
  load("camera.auto_exp_prior", "--auto_exp_prior", ae.prior);
  load("camera.target_gray_value", "--target_gray_value", ae.target_gray_value);
  load("camera.auto_exposure_ev_compensation",
       "--auto_exposure_ev_compensation", ae.ev_compensation);
  load("camera.auto_exposure_damping", "--auto_exposure_damping", ae.damping);
  load("camera.auto_exposure_metering", "--auto_exposure_metering",
       ae.metering);
  load("camera.auto_exposure_roi", "--auto_exposure_roi", ae.roi);
  load("camera.auto_exposure_circle", "--auto_exposure_circle", ae.circle);
  load("camera.exposure_ms", "--exposure", options.exposure_ms);
  load("camera.gain_db", "--gain-db", options.gain_db);
  load("camera.max_exposure_ms", "--max-exposure", options.max_exposure_ms);
  load("camera.max_gain", "--max-gain", options.max_gain);
  load("camera.brightness_target", "--brightness", options.brightness_target);
  load("camera.metering_circle", "--metering-circle", options.metering_circle);
  load("imu.type", "--imu_type", options.imu_type);
  load("imu.udp", "--imu_udp", options.imu_udp);
  load("imu.port", "--imu_port", options.imu_port);
  load("imu.baudrate", "--imu_baudrate", options.imu_baudrate);
  load("imu.freq_hz", "--imu_freq_hz", options.imu_freq_hz);
  load("imu.send_odometry", "--imu_send_odometry", options.imu_send_odometry);
  load("camera.i2c_device", "--i2c", options.i2c_device);
  load("camera.imu_rate", "--imu-rate", options.imu_rate);
  bool resize_half = false;
  load("camera.resize_half", "--resize-half", resize_half);
  if (resize_half && app.count("--downscale") == 0) options.downscale = 2;
  load("camera.downscale", "--downscale", options.downscale);
  load("camera.sensor_binning", "--sensor_binning", options.sensor_binning);
  archive.finishNode();
}

// Scales a calibration at the sensor mode to the output image, or checks that
// it is already at the output image.
inline void fitCalibration(Calibration<double>& calib, const PiCameraImuOptions& options) {
  if (calib.intrinsics.size() != 1 || calib.T_i_c.size() != 1 || calib.resolution.size() != 1)
    throw std::runtime_error("Expected a calibration with one camera");
  // The xeye models send the crop of the sensor mode.
  Eigen::Vector2i sensor_size(options.sensorWidth(), options.sensorHeight());
  const std::vector<int> crop = options.cropRect();
  if (options.xeyeProcessing() && crop.size() == 4)
    sensor_size =
        Eigen::Vector2i(crop[2] - crop[2] % 2, crop[3] - crop[3] % 2);
  // With sensor_binning at downscale 2, the output is the halved crop of the
  // binning mode: the same size and the same scaleCalibration as the host
  // reduction (output pixel i covers the full resolution pixels 2i and 2i + 1).
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
