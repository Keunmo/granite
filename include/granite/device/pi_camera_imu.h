// SPDX-License-Identifier: MIT
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <tbb/concurrent_queue.h>
#include <granite/device/pi_sensor_utils.h>
#include <granite/optical_flow/optical_flow.h>
#include <granite/utils/imu_types.h>

namespace granite {

struct PiCameraImuOptions {
  // Sensor entity name prefix in the rp1-cfe media graph.
  std::string sensor = "arducam-pivariety";
  int width = 1280;
  int height = 720;
  double fps = 30;
  bool resize_half = false;
  double exposure_ms = 0;  // 0: automatic exposure
  double max_exposure_ms = 8;
  double gain_db = 0;  // Analogue gain for manual exposure
  double max_gain = 8;  // Automatic exposure limit, analogue_gain / 100
  double brightness_target = 100;
  // Added to image times; the calibration convention is t_imu = t_cam + offset.
  int64_t camera_time_offset_ns = 0;
  std::string i2c_device = "/dev/i2c-15";
  int imu_address = 0x68;
  double imu_rate = 200;
};

// Arducam B0429 (AR0234) on a Raspberry Pi 5 CSI port through rp1-cfe, and an
// MPU-6500/9250 through an I2C adapter such as the CP2112 USB bridge. Both
// streams use CLOCK_MONOTONIC timestamps.
class PiCameraImuDevice {
 public:
  explicit PiCameraImuDevice(const PiCameraImuOptions& options);
  ~PiCameraImuDevice();
  PiCameraImuDevice(const PiCameraImuDevice&) = delete;
  PiCameraImuDevice& operator=(const PiCameraImuDevice&) = delete;

  // Configures both sensors and starts their threads. Data is discarded
  // until enableCapture().
  void start();
  void enableCapture();
  void stop(bool send_end_of_stream = true);

  int outputWidth() const;
  int outputHeight() const;
  double lineTimeNs() const { return line_ns; }
  // The frame rate of the sensor timing; the sensor limits the requested fps.
  double frameRate() const { return frame_rate; }
  // The longest exposure that does not make the frame period longer.
  double maxExposureMs() const { return frame_exposure_lines * line_ns * 1e-6; }
  std::string errorMessage() const;

  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr>* image_data_queue = nullptr;
  tbb::concurrent_bounded_queue<ImuData::Ptr>* imu_data_queue = nullptr;
  std::atomic<uint64_t> image_frames{0}, queued_frames{0}, dropped_frames{0};
  std::atomic<uint64_t> sensor_frame_drops{0}, imu_samples{0}, imu_overflows{0};
  std::atomic<int64_t> last_image_t_ns{0};
  std::atomic<int> exposure_lines{0}, analogue_gain{0};
  std::atomic<double> mean_brightness{0}, imu_period_ns{0};
  std::atomic<bool> failed{false};

 private:
  void setupCamera();
  void setupImu();
  void cameraLoop();
  void imuLoop();
  void fail(const std::string& message);
  void setSensorControl(uint32_t id, int value);
  void writeImu(uint8_t reg, uint8_t value);
  void readImu(uint8_t reg, uint8_t* data, size_t size);

  PiCameraImuOptions options;
  int video_fd = -1, sensor_fd = -1, i2c_fd = -1;
  std::vector<std::pair<void*, size_t>> buffers;
  size_t bytes_per_line = 0;
  double line_ns = 0, frame_rate = 0;
  int max_exposure_lines = 0, frame_exposure_lines = 0, min_gain = 100, max_gain = 100;
  double accel_scale = 0, gyro_scale = 0;
  FifoSampleClock imu_clock{5e6};
  std::atomic<bool> capturing{false}, running{false};
  std::thread camera_thread, imu_thread;
  mutable std::mutex error_mutex;
  std::string error_message;
};

}  // namespace granite
