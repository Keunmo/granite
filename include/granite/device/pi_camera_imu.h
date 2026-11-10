// SPDX-License-Identifier: MIT
#pragma once

#include <granite/device/pi_sensor_utils.h>
#include <granite/optical_flow/optical_flow.h>
#include <granite/utils/imu_types.h>
#include <linux/videodev2.h>
#include <tbb/concurrent_queue.h>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace granite {

struct PiCameraImuOptions {
  // The names of the keys are the names of the xeye capture keys
  // (multipleye cross-pi5: camera_v4l2_gen2 with model kurokesu_ar0234_pi or
  // b0429_pi, and imu_mavlink). The defaults are the values of
  // configs/capture_kurokesu_ar0234_pi.yaml.
  // model:
  //   "kurokesu_ar0234_pi": the Kurokesu AR0234 driver on rp1-cfe (Pi 5).
  //   "b0429_pi": the Arducam B0429 (arducam-pivariety) on rp1-cfe (Pi 5).
  //   Both process the frames as the xeye node.
  //   "b0429": the B0429 on rp1-cfe or unicam (Pi 4) with the processing of
  //   granite: packed RAW10, grbgToGray, and the exposure center as the image
  //   time.
  std::string model = "kurokesu_ar0234_pi";
  // The video node of the rp1-cfe. Empty: the node of the media graph that has
  // the sensor.
  std::string port = "/dev/video0";
  // Sensor entity name prefix in the media graph. Empty: the prefix of the
  // model ("ar0234" or "arducam-pivariety").
  std::string sensor;
  // Sensor mode. 0: 1920x1200 for kurokesu_ar0234_pi, 1280x720 for the B0429.
  int width = 0;
  int height = 0;
  double fps = 30;  // frame_rate
  // The output image is 1/downscale of the sensor mode (grbgToGray).
  int downscale = 1;
  // Added to image times; the calibration convention is t_imu = t_cam + offset.
  int64_t camera_time_offset_ns = 0;

  // kurokesu_ar0234_pi and b0429_pi
  std::string pixel_format = "gray8";  // The VIO takes gray8 only
  double exposure_time_ms =
      8.0;        // exposure_time, the start value of the auto exposure
  int gain = -1;  // analogue_gain control value. -1: 14 (kurokesu_ar0234_pi) or
                  // 100 (b0429_pi)
  int num_v4l_buffer = 4;
  bool h_flip = true;
  bool v_flip = true;
  // [x, y, w, h] in the sensor frame, even values. Empty: kDefaultCrop for
  // kurokesu_ar0234_pi in its default sensor mode, else the full frame.
  std::vector<int> crop;
  bool auto_white_balance = true;
  bool fused_wb =
      true;  // The WB gains come from the statistics of the previous frame
  int black_level =
      42;  // [10-bit], the AR0234 value that xeye measured on Helios
  bool enable_gamma = false;
  double gamma_value = 0.45;
  XeyeAutoExposureOptions auto_exposure;

  // b0429
  double exposure_ms = 0;  // 0: automatic exposure
  double max_exposure_ms = 8;
  double gain_db = 0;  // Analogue gain for manual exposure
  double max_gain = 8;  // Automatic exposure limit, analogue_gain / 100
  double brightness_target = 100;
  // Automatic exposure meters the pixels in this circle [cx, cy, radius] of
  // the output image, e.g. the image circle of a fisheye lens. Empty: all pixels.
  std::vector<double> metering_circle;

  // imu type:
  //   "imu_mavlink": HIGHRES_IMU of a PX4 flight controller, as the xeye
  //   imu_mavlink node, through a MAVLink router (udp) or a serial port
  //   (port, baudrate).
  //   "mpu_i2c": an MPU-6500/9250 through an I2C adapter.
  std::string imu_type = "imu_mavlink";
  // Set one of udp and port. Both empty: udp 127.0.0.1:14601.
  std::string imu_udp;   // <bind address>:<port> of the router endpoint
  std::string imu_port;  // Serial device
  int imu_baudrate = 921600;
  // HIGHRES_IMU rate requested with MAV_CMD_SET_MESSAGE_INTERVAL. 0 keeps the
  // FC rate.
  int imu_freq_hz = 400;
  std::string i2c_device = "/dev/i2c-15";
  int imu_address = 0x68;
  double imu_rate = 200;

  bool xeyeProcessing() const { return model != "b0429"; }
  bool kurokesu() const { return model == "kurokesu_ar0234_pi"; }
  bool mavlink() const { return imu_type == "imu_mavlink"; }
  int sensorWidth() const {
    return width > 0 ? width : kurokesu() ? 1920 : 1280;
  }
  int sensorHeight() const {
    return height > 0 ? height : kurokesu() ? 1200 : 720;
  }
  // The center 1344 columns of the default mode, as in
  // configs/capture_kurokesu_ar0234_pi.yaml of xeye.
  static constexpr int kDefaultCrop[4] = {288, 0, 1344, 1200};
  std::vector<int> cropRect() const {
    if (!crop.empty() || !kurokesu() || width > 0 || height > 0) return crop;
    return {std::begin(kDefaultCrop), std::end(kDefaultCrop)};
  }
};

// A camera on a Raspberry Pi CSI port and an IMU. Both streams use
// CLOCK_MONOTONIC timestamps.
// - kurokesu_ar0234_pi or b0429_pi, and imu_mavlink: the same images and times
//   as the xeye capture nodes. The image time is the V4L2 buffer time. The IMU
//   time is the FC sample time mapped to the receive time.
// - b0429 (Pi 5 rp1-cfe or Pi 4 unicam) and mpu_i2c (for example through a
//   CP2112 USB bridge): the image time is the center of the exposure.
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
  // The CSI receiver of the camera: "rp1-cfe" or "unicam" (after start()).
  const std::string& receiver() const { return receiver_driver; }
  // The longest exposure that does not make the frame period longer.
  double maxExposureMs() const { return frame_exposure_lines * line_ns * 1e-6; }
  // The analogue gain of analogue_gain in dB.
  double gainDb() const;
  std::string errorMessage() const;

  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr>* image_data_queue = nullptr;
  tbb::concurrent_bounded_queue<ImuData::Ptr>* imu_data_queue = nullptr;
  // Optional copies of the same data for a recording. A full image queue
  // drops the image from the recording (record_dropped_frames).
  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr>* record_image_queue = nullptr;
  tbb::concurrent_bounded_queue<ImuData::Ptr>* record_imu_queue = nullptr;
  std::atomic<uint64_t> record_dropped_frames{0};
  std::atomic<uint64_t> image_frames{0}, queued_frames{0}, dropped_frames{0};
  std::atomic<uint64_t> sensor_frame_drops{0}, imu_samples{0}, imu_overflows{0};
  // I2C transfers that failed, and the longest time that one failure took.
  std::atomic<uint64_t> imu_i2c_errors{0};
  std::atomic<int64_t> imu_error_max_ns{0};
  std::atomic<int64_t> last_image_t_ns{0}, first_capture_t_ns{0};
  // Time from the buffer time stamp to the push into the image queue.
  std::atomic<int64_t> image_delay_sum_ns{0}, image_delay_max_ns{0};
  std::atomic<int> exposure_lines{0}, analogue_gain{0};
  std::atomic<double> mean_brightness{0}, imu_period_ns{0};
  std::atomic<bool> failed{false};

 private:
  void setupCamera();
  void setupXeyeCamera();
  void setupImu();
  void setupMavlink();
  void cameraLoop();
  void xeyeCameraLoop();
  void imuLoop();
  void mavlinkLoop();
  bool processXeyeFrame(const v4l2_buffer& buffer, cv::Mat* gray);
  void runXeyeAutoExposure(const cv::Mat& gray);
  void setXeyeExposure(double exposure_ms);
  void setXeyeGain(int value);
  // The linear gain of an analogue_gain value of the xeye models, and the
  // value nearest to a linear gain.
  double linearGain(int value) const;
  int gainValue(double linear_gain) const;
  void fail(const std::string& message);
  void setSensorControl(uint32_t id, int value);
  void writeImu(uint8_t reg, uint8_t value);
  void readImu(uint8_t reg, uint8_t* data, size_t size);

  PiCameraImuOptions options;
  int video_fd = -1, sensor_fd = -1, i2c_fd = -1, mavlink_fd = -1;
  bool mavlink_udp = false;
  std::vector<std::pair<void*, size_t>> buffers;
  // kurokesu_ar0234_pi and b0429_pi: V4L2_MEMORY_DMABUF with buffers from a
  // dma-heap, which the CPU reads through its cache, or V4L2_MEMORY_MMAP when
  // the heap fails.
  uint32_t buffer_memory = 0;
  std::vector<int> dma_buffer_fds;
  int crop_x = 0, crop_y = 0, crop_w = 0, crop_h = 0;
  double exposure_time_ms = 0;
  std::optional<std::array<double, 2>> wb_gains;
  std::vector<uint8_t> gamma_curve;
  std::array<uint16_t, 256> gamma_linear10{};
  std::vector<uint8_t> bayer8;
  int ae_wait_frames = 0;
  size_t bytes_per_line = 0;
  double line_ns = 0, frame_rate = 0;
  int max_exposure_lines = 0, frame_exposure_lines = 0, min_gain = 100, max_gain = 100;
  double accel_scale = 0, gyro_scale = 0;
  // unicam stamps the start of a frame, rp1-cfe its end.
  bool frame_start_stamp = false;
  std::string receiver_driver;
  cv::Mat metering_mask;
  FifoSampleClock imu_clock{5e6};
  std::atomic<bool> capturing{false}, running{false};
  std::thread camera_thread, imu_thread;
  mutable std::mutex error_mutex;
  std::string error_message;
};

}  // namespace granite
