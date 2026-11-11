// SPDX-License-Identifier: MIT
#include <granite/device/pi_camera_imu.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <granite/utils/imu_types.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <mavlink/common/mavlink.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

namespace granite {

namespace {

constexpr uint8_t MPU_SMPLRT_DIV = 0x19, MPU_CONFIG = 0x1A, MPU_GYRO_CONFIG = 0x1B,
                  MPU_ACCEL_CONFIG = 0x1C, MPU_ACCEL_CONFIG2 = 0x1D, MPU_FIFO_EN = 0x23,
                  MPU_USER_CTRL = 0x6A, MPU_PWR_MGMT_1 = 0x6B, MPU_PWR_MGMT_2 = 0x6C,
                  MPU_FIFO_COUNT = 0x72, MPU_FIFO_R_W = 0x74, MPU_WHO_AM_I = 0x75;
constexpr size_t MPU_FIFO_BYTES = 512;
// A CP2112 transfer takes at least 5 ms and reads of 256 bytes fail, so the
// IMU loop reads the FIFO in few large transfers.
constexpr size_t MPU_READ_CHUNK = 16 * MPU_FIFO_SAMPLE_BYTES;
constexpr auto IMU_POLL_PERIOD = std::chrono::milliseconds(20);
constexpr int VIDEO_BUFFERS = 8;
// An exposure longer than the frame period minus this margin makes the frame
// period longer. xeye measured the margin with the same module on Jetson.
constexpr double EXPOSURE_MARGIN_NS = 100'000;
constexpr uint32_t STARTUP_FRAMES = 3;
// The sensor applies exposure and gain controls this many frames after the write.
constexpr uint32_t CONTROL_DELAY_FRAMES = 2;
// The CSI receivers: rp1-cfe on the Raspberry Pi 5, unicam on the Raspberry Pi 4.
constexpr const char* RP1_CFE = "rp1-cfe";
constexpr const char* UNICAM = "unicam";

// kurokesu_ar0234_pi and b0429_pi: the values of the xeye node.
constexpr int FIRST_FRAME_TIMEOUT_MS = 2000;
// Contiguous memory, as the rp1-cfe needs.
constexpr const char* DMA_HEAP = "/dev/dma_heap/linux,cma";
// The frames are GR16 (GRBG): the non-green pixel of the even rows is
// in column 1. The sensor flip keeps the GRBG order.
constexpr int GRBG_NON_GREEN_COL = 1;
constexpr double MIN_EXPOSURE_MS = 0.03;
// The image_sensor::Profile of the xeye models.
struct XeyeSensorProfile {
  std::vector<std::pair<int, int>> resolutions;
  int min_exposure_us, max_exposure_us;
  int min_gain, max_gain;
};
const XeyeSensorProfile KUROKESU_AR0234_PROFILE{
    {{1920, 1200}, {1920, 1080}, {1280, 720}, {960, 600}}, 2, 65535, 13, 64};
const XeyeSensorProfile AR0234_PROFILE{
    {{1920, 1200}, {1920, 1080}, {1280, 720}}, 2, 65535, 100, 1200};
// b0429_pi: measured on the Pi, 100 is 1x and each 408.5 adds 1x, with an
// error of at most 3 %. Values from 1123 give less gain.
constexpr double B0429_PI_GAIN_PER_X = 408.5;

// imu_mavlink: the values of the xeye imu_mavlink node. Requests come from an
// onboard computer of the vehicle system.
constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
constexpr uint8_t MAVLINK_COMPONENT_ID = MAV_COMP_ID_ONBOARD_COMPUTER;
constexpr int MAVLINK_READ_TIMEOUT_MS = 100;
constexpr int64_t MAVLINK_REQUEST_RETRY_NS = 2'000'000'000;
// No HIGHRES_IMU for this long while the rate is settled requests it again.
constexpr int64_t MAVLINK_IMU_TIMEOUT_NS = 3'000'000'000;

int64_t monotonicNs() {
  timespec time{};
  clock_gettime(CLOCK_MONOTONIC, &time);
  return int64_t(time.tv_sec) * 1'000'000'000 + time.tv_nsec;
}

std::runtime_error systemError(const std::string& what) {
  return std::runtime_error(what + ": " + std::strerror(errno));
}

int xioctl(int fd, unsigned long request, void* argument) {
  int result;
  do result = ioctl(fd, request, argument);
  while (result == -1 && errno == EINTR);
  return result;
}

std::string charDevicePath(uint32_t major, uint32_t minor) {
  std::ifstream uevent("/sys/dev/char/" + std::to_string(major) + ":" +
                       std::to_string(minor) + "/uevent");
  std::string line;
  while (std::getline(uevent, line)) {
    if (line.rfind("DEVNAME=", 0) == 0) return "/dev/" + line.substr(8);
  }
  throw std::runtime_error("No device node for char device " + std::to_string(major) +
                           ":" + std::to_string(minor));
}

struct MediaEntity {
  uint32_t id = 0;
  std::string name, node;
  uint16_t pads = 0, links = 0;
};

// Finds the media graph of a CSI receiver (rp1-cfe or unicam) that contains
// the sensor. The Pi 5 has one graph per CSI port.
class MediaGraph {
 public:
  // video_node: a video node that the graph must also have (empty: any).
  explicit MediaGraph(const std::string& sensor_prefix,
                      const std::string& video_node = "") {
    for (int index = 0; index < 16 && fd < 0; ++index) {
      const std::string path = "/dev/media" + std::to_string(index);
      const int candidate = open(path.c_str(), O_RDWR | O_CLOEXEC);
      if (candidate < 0) continue;
      media_device_info info{};
      if (xioctl(candidate, MEDIA_IOC_DEVICE_INFO, &info) == 0 &&
          (std::string(info.driver) == RP1_CFE || std::string(info.driver) == UNICAM)) {
        fd = candidate;
        entities = enumerate();
        if (findPrefix(sensor_prefix) &&
            (video_node.empty() || findNode(video_node))) {
          driver_name = info.driver;
          break;
        }
        entities.clear();
        fd = -1;
      }
      close(candidate);
    }
    if (fd < 0)
      throw std::runtime_error("No rp1-cfe or unicam media graph contains sensor '" +
                               sensor_prefix +
                               "'. Check the camera cable and the dtoverlay in "
                               "/boot/firmware/config.txt");
  }
  ~MediaGraph() { if (fd >= 0) close(fd); }

  const std::string& driver() const { return driver_name; }

  const MediaEntity* findNode(const std::string& path) const {
    std::error_code error;
    const auto target = std::filesystem::canonical(path, error);
    if (error) return nullptr;
    for (const auto& entity : entities) {
      if (!entity.node.empty() &&
          std::filesystem::canonical(entity.node, error) == target)
        return &entity;
    }
    return nullptr;
  }

  const MediaEntity* findPrefix(const std::string& prefix) const {
    for (const auto& entity : entities)
      if (entity.name.rfind(prefix, 0) == 0) return &entity;
    return nullptr;
  }

  const MediaEntity& get(const std::string& name) const {
    for (const auto& entity : entities)
      if (entity.name == name) return entity;
    throw std::runtime_error("Media entity '" + name + "' not found");
  }

  std::vector<media_link_desc> links(const MediaEntity& source) const {
    std::vector<media_pad_desc> pads(source.pads);
    std::vector<media_link_desc> result(source.links);
    media_links_enum query{};
    query.entity = source.id;
    query.pads = pads.data();
    query.links = result.data();
    if (xioctl(fd, MEDIA_IOC_ENUM_LINKS, &query) < 0)
      throw systemError("MEDIA_IOC_ENUM_LINKS");
    return result;
  }

  // Enables the link between two entities and returns the source pad.
  uint16_t enableLink(const MediaEntity& source, const MediaEntity& sink) {
    std::vector<media_pad_desc> pads(source.pads);
    std::vector<media_link_desc> links(source.links);
    media_links_enum query{};
    query.entity = source.id;
    query.pads = pads.data();
    query.links = links.data();
    if (xioctl(fd, MEDIA_IOC_ENUM_LINKS, &query) < 0) throw systemError("MEDIA_IOC_ENUM_LINKS");
    for (auto& link : links) {
      if (link.source.entity != source.id || link.sink.entity != sink.id) continue;
      link.flags |= MEDIA_LNK_FL_ENABLED;
      if (xioctl(fd, MEDIA_IOC_SETUP_LINK, &link) < 0) throw systemError("MEDIA_IOC_SETUP_LINK");
      return link.source.index;
    }
    throw std::runtime_error("No link from '" + source.name + "' to '" + sink.name + "'");
  }

 private:
  std::vector<MediaEntity> enumerate() const {
    std::vector<MediaEntity> result;
    media_entity_desc desc{};
    desc.id = MEDIA_ENT_ID_FLAG_NEXT;
    while (xioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &desc) == 0) {
      MediaEntity entity;
      entity.id = desc.id;
      entity.name = desc.name;
      entity.pads = desc.pads;
      entity.links = desc.links;
      if (desc.dev.major != 0) entity.node = charDevicePath(desc.dev.major, desc.dev.minor);
      result.push_back(entity);
      desc.id |= MEDIA_ENT_ID_FLAG_NEXT;
    }
    return result;
  }

  int fd = -1;
  std::vector<MediaEntity> entities;
  std::string driver_name;
};

void setSubdevFormat(const std::string& node, uint32_t pad, int width,
                     int height, uint32_t code = MEDIA_BUS_FMT_SGRBG10_1X10) {
  const int fd = open(node.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) throw systemError("Cannot open " + node);
  v4l2_subdev_format format{};
  format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  format.pad = pad;
  format.format.width = width;
  format.format.height = height;
  format.format.code = code;
  // CFE link validation compares the field of both pads.
  format.format.field = V4L2_FIELD_NONE;
  format.format.colorspace = V4L2_COLORSPACE_RAW;
  const int result = xioctl(fd, VIDIOC_SUBDEV_S_FMT, &format);
  close(fd);
  if (result < 0) throw systemError("VIDIOC_SUBDEV_S_FMT " + node);
  if (int(format.format.width) != width ||
      int(format.format.height) != height || format.format.code != code) {
    throw std::runtime_error(node + " pad " + std::to_string(pad) +
                             " does not take " + std::to_string(width) + "x" +
                             std::to_string(height) + " code " +
                             std::to_string(code));
  }
}

v4l2_mbus_framefmt getSubdevFormat(const std::string& node, uint32_t pad) {
  const int fd = open(node.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) throw systemError("Cannot open " + node);
  v4l2_subdev_format format{};
  format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  format.pad = pad;
  const int result = xioctl(fd, VIDIOC_SUBDEV_G_FMT, &format);
  close(fd);
  if (result < 0) throw systemError("VIDIOC_SUBDEV_G_FMT " + node);
  return format.format;
}

v4l2_queryctrl queryControl(int fd, uint32_t id) {
  v4l2_queryctrl control{};
  control.id = id;
  if (xioctl(fd, VIDIOC_QUERYCTRL, &control) < 0) throw systemError("VIDIOC_QUERYCTRL");
  return control;
}

int getControl(int fd, uint32_t id) {
  v4l2_control control{id, 0};
  if (xioctl(fd, VIDIOC_G_CTRL, &control) < 0) throw systemError("VIDIOC_G_CTRL");
  return control.value;
}

int64_t getControl64(int fd, uint32_t id) {
  v4l2_ext_control control{};
  control.id = id;
  v4l2_ext_controls controls{};
  controls.which = V4L2_CTRL_WHICH_CUR_VAL;
  controls.count = 1;
  controls.controls = &control;
  if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &controls) < 0) throw systemError("VIDIOC_G_EXT_CTRLS");
  return control.value64;
}

}  // namespace

PiCameraImuDevice::PiCameraImuDevice(const PiCameraImuOptions& requested)
    : options(requested), imu_clock(1e9 / requested.imu_rate) {
  if (options.model == "arducam_b0429_pi") options.model = "b0429_pi";
  if (options.model != "kurokesu_ar0234_pi" && options.model != "b0429_pi" &&
      options.model != "b0429")
    throw std::invalid_argument(
        "model must be kurokesu_ar0234_pi, b0429_pi, or b0429: " +
        options.model);
  if (options.imu_type != "imu_mavlink" && options.imu_type != "mpu_i2c")
    throw std::invalid_argument("imu type must be imu_mavlink or mpu_i2c: " +
                                options.imu_type);
  if (options.imu_send_odometry && !options.mavlink())
    throw std::invalid_argument(
        "send_odometry needs imu type imu_mavlink, got " + options.imu_type);
  if (options.mavlink()) {
    if (options.imu_freq_hz < 0)
      throw std::invalid_argument(
          "freq_hz must be >= 0 (0 keeps the FC rate), got " +
          std::to_string(options.imu_freq_hz));
    if (options.imu_udp.empty() && options.imu_port.empty())
      options.imu_udp = "127.0.0.1:14601";
    if (!options.imu_udp.empty() && !options.imu_port.empty())
      throw std::invalid_argument(
          "set exactly one of 'port' (serial device) or 'udp' (<bind "
          "address>:<port>)");
  }
  if (options.sensor.empty())
    options.sensor = options.kurokesu() ? "ar0234" : "arducam-pivariety";
  // Resolve the default crop before the default sensor mode.
  options.crop = options.cropRect();
  options.width = options.sensorWidth();
  options.height = options.sensorHeight();
  if (options.width % 4 != 0 || options.height % 2 != 0 || !(options.fps > 0) ||
      options.downscale < 1 || options.imu_rate < 4 ||
      options.imu_rate > 1000) {
    throw std::invalid_argument("Invalid camera or IMU option");
  }
  if (options.xeyeProcessing()) {
    // The Kurokesu driver has a 2x2 binning mode, 960x600 of the full array.
    // At downscale 2 of 1920x1200 the sensor halves the image in place of the
    // host: the output is the crop of the binning mode. The crop, the AE ROI
    // and the AE circle are in the crop coordinates of the requested mode, so
    // they are halved too. The binning crop needs even values.
    if (options.kurokesu() && options.sensor_binning &&
        options.downscale == 2 && options.width == 1920 &&
        options.height == 1200) {
      std::vector<int> crop = options.crop;
      if (crop.empty()) crop = {0, 0, options.width, options.height};
      const bool even =
          crop.size() == 4 && std::all_of(crop.begin(), crop.end(),
                                          [](int v) { return v % 4 == 0; });
      if (even) {
        for (int& v : crop) v /= 2;
        options.crop = crop;
        for (int& v : options.auto_exposure.roi) v /= 2;
        for (int& v : options.auto_exposure.circle) v /= 2;
        options.width = 960;
        options.height = 600;
        options.downscale = 1;
        binning = true;
      } else {
        std::cerr << "Camera: the crop values are not multiples of 4, so the "
                     "binning mode cannot keep the Bayer phase. The host "
                     "halves the image."
                  << std::endl;
      }
    }
    // With a host reduction, the AE meters the output image: the ROI and the
    // circle of the requested mode scale to it.
    if (options.downscale > 1) {
      for (int& v : options.auto_exposure.roi) v /= options.downscale;
      for (int& v : options.auto_exposure.circle) v /= options.downscale;
    }
    // ValidateProperty and CheckProcessingProperty of the xeye node.
    if (options.gain < 0) options.gain = options.kurokesu() ? 14 : 100;
    if (options.pixel_format != "gray8")
      throw std::invalid_argument(
          "pixel_format must be gray8: the VIO takes gray images, got " +
          options.pixel_format);
    if (options.num_v4l_buffer < 2)
      throw std::invalid_argument("num_v4l_buffer must be 2 or more");
    const XeyeSensorProfile& profile =
        options.kurokesu() ? KUROKESU_AR0234_PROFILE : AR0234_PROFILE;
    if (std::find(profile.resolutions.begin(), profile.resolutions.end(),
                  std::make_pair(options.width, options.height)) ==
        profile.resolutions.end()) {
      std::string available;
      for (const auto& r : profile.resolutions)
        available +=
            std::to_string(r.first) + "x" + std::to_string(r.second) + ", ";
      throw std::invalid_argument(
          "Invalid resolution: " + std::to_string(options.width) + "x" +
          std::to_string(options.height) + ". Available resolutions: [" +
          available + "]");
    }
    const int exposure_us = static_cast<int>(options.exposure_time_ms * 1000);
    if (exposure_us < profile.min_exposure_us ||
        exposure_us > profile.max_exposure_us) {
      std::cerr << "Camera: Exposure time out of range: " << exposure_us
                << " us. Fallback to 1000 us" << std::endl;
      options.exposure_time_ms = 1.0;
    }
    if (options.gain < profile.min_gain || options.gain > profile.max_gain) {
      std::cerr << "Camera: Gain out of range: " << options.gain
                << ". Fallback to 100" << std::endl;
      options.gain = 100;
    }
    const auto floor2 = [](int v) { return v - v % 2; };
    crop_w = options.width;
    crop_h = options.height;
    const std::vector<int> crop_rect = options.cropRect();
    if (!crop_rect.empty()) {
      if (crop_rect.size() != 4)
        throw std::invalid_argument("crop must be [x, y, w, h]");
      // Even values keep the Bayer phase.
      crop_x = floor2(crop_rect[0]);
      crop_y = floor2(crop_rect[1]);
      crop_w = floor2(crop_rect[2]);
      crop_h = floor2(crop_rect[3]);
      if (crop_x < 0 || crop_y < 0 || crop_w < 2 || crop_h < 2 ||
          crop_x + crop_w > options.width || crop_y + crop_h > options.height) {
        throw std::invalid_argument("crop [" + std::to_string(crop_rect[0]) +
                                    ", " + std::to_string(crop_rect[1]) +
                                    ", " + std::to_string(crop_rect[2]) +
                                    ", " + std::to_string(crop_rect[3]) +
                                    "] is outside the " +
                                    std::to_string(options.width) + "x" +
                                    std::to_string(options.height) + " frame");
      }
    }
    if (!options.auto_white_balance) options.fused_wb = false;
    if (options.black_level < 0 || options.black_level > 255)
      throw std::invalid_argument("black_level must be in [0, 255]: " +
                                  std::to_string(options.black_level));
    if (options.enable_gamma &&
        (options.gamma_value < 0.25 || options.gamma_value > 1.0))
      throw std::invalid_argument("gamma_value must be in [0.25, 1]: " +
                                  std::to_string(options.gamma_value));
    const auto& ae = options.auto_exposure;
    const auto& exposure = ae.exposure_range_ms;
    if (exposure.size() != 2 ||
        !(exposure[0] > 0.0 && exposure[0] <= exposure[1]))
      throw std::invalid_argument(
          "auto_exposure_range must be [min, max] in ms, 0 < min <= max");
    if (!ae.gain_range_db.empty() &&
        (ae.gain_range_db.size() != 2 ||
         !(ae.gain_range_db[0] <= ae.gain_range_db[1])))
      throw std::invalid_argument("auto_gain_range must be [min, max] in dB");
    if (ae.prior != "exposure" && ae.prior != "gain")
      throw std::invalid_argument("auto_exp_prior must be exposure or gain: " +
                                  ae.prior);
    if (!(ae.target_gray_value >= 1.0 && ae.target_gray_value <= 99.0))
      throw std::invalid_argument("target_gray_value must be in [1, 99] (%)");
    if (!(std::abs(ae.ev_compensation) <= 3.0))
      throw std::invalid_argument(
          "auto_exposure_ev_compensation must be in [-3, 3]");
    if (!(ae.damping >= 0.0 && ae.damping < 1.0))
      throw std::invalid_argument("auto_exposure_damping must be in [0, 1)");
    if (ae.metering != "average" && ae.metering != "spot" &&
        ae.metering != "partial")
      throw std::invalid_argument(
          "auto_exposure_metering must be average, spot, or partial: " +
          ae.metering);
    if (!ae.roi.empty() && ae.roi.size() != 4)
      throw std::invalid_argument("auto_exposure_roi must be [x, y, w, h]");
    const int out_w = crop_w / options.downscale;
    const int out_h = crop_h / options.downscale;
    if (ae.roi.size() == 4 &&
        (ae.roi[0] < 0 || ae.roi[1] < 0 || ae.roi[2] < 2 || ae.roi[3] < 2 ||
         ae.roi[0] + ae.roi[2] > out_w || ae.roi[1] + ae.roi[3] > out_h)) {
      throw std::invalid_argument(
          "auto_exposure_roi must have x, y >= 0, w, h >= 2, and be in the " +
          std::to_string(out_w) + "x" + std::to_string(out_h) + " image");
    }
    if (!ae.circle.empty() && (ae.circle.size() != 3 || ae.circle[2] <= 0))
      throw std::invalid_argument(
          "auto_exposure_circle must be [cx, cy, r] with r > 0");
    // grbgToGray needs whole blocks: N x N for an even N, 2N x 2N for an odd N
    // > 1.
    const int unit = options.downscale == 1       ? 1
                     : options.downscale % 2 == 0 ? options.downscale
                                                  : 2 * options.downscale;
    if (crop_w % unit != 0 || crop_h % unit != 0)
      throw std::invalid_argument(
          "The crop is not a multiple of the downscale block " +
          std::to_string(unit));
    return;
  }
  if (options.exposure_ms < 0 || !(options.max_exposure_ms > 0) ||
      !(options.gain_db >= 0 && options.gain_db <= B0429_MAX_GAIN_DB) ||
      !(options.max_gain >= 1) ||
      !(options.brightness_target > 0 && options.brightness_target < 255) ||
      !(options.metering_circle.empty() ||
        options.metering_circle.size() == 3)) {
    throw std::invalid_argument("Invalid b0429 camera option");
  }
  // grbgToGray needs whole blocks: N x N for an even N, 2N x 2N for an odd N > 1.
  const int unit = options.downscale == 1 ? 1
                   : options.downscale % 2 == 0 ? options.downscale : 2 * options.downscale;
  if (options.width % unit != 0 || options.height % unit != 0)
    throw std::invalid_argument("The sensor mode is not a multiple of the downscale block " +
                                std::to_string(unit));
  if (!options.metering_circle.empty()) {
    metering_mask = cv::Mat::zeros(outputHeight(), outputWidth(), CV_8UC1);
    cv::circle(metering_mask,
               cv::Point(static_cast<int>(std::lround(options.metering_circle[0])),
                         static_cast<int>(std::lround(options.metering_circle[1]))),
               static_cast<int>(std::lround(options.metering_circle[2])), cv::Scalar(255),
               cv::FILLED);
    if (cv::countNonZero(metering_mask) == 0)
      throw std::invalid_argument("The metering circle has no pixel in the image");
  }
}

PiCameraImuDevice::~PiCameraImuDevice() {
  try {
    stop(false);
  } catch (...) {
  }
}

int PiCameraImuDevice::outputWidth() const {
  return (options.xeyeProcessing() ? crop_w : options.width) /
         options.downscale;
}

int PiCameraImuDevice::outputHeight() const {
  return (options.xeyeProcessing() ? crop_h : options.height) /
         options.downscale;
}

double PiCameraImuDevice::linearGain(int value) const {
  if (options.kurokesu()) return kurokesuLinearGain(value);
  return 1.0 + (value - B0429_MIN_DRIVER_GAIN) / B0429_PI_GAIN_PER_X;
}

int PiCameraImuDevice::gainValue(double linear_gain) const {
  if (options.kurokesu()) return kurokesuGainValue(linear_gain);
  return std::clamp(
      static_cast<int>(std::lround(B0429_MIN_DRIVER_GAIN +
                                   B0429_PI_GAIN_PER_X * (linear_gain - 1.0))),
      B0429_MIN_DRIVER_GAIN, B0429_MAX_DRIVER_GAIN);
}

double PiCameraImuDevice::gainDb() const {
  if (options.xeyeProcessing())
    return 20 * std::log10(linearGain(analogue_gain.load()));
  return b0429DbFromDriverGain(analogue_gain.load());
}

std::string PiCameraImuDevice::errorMessage() const {
  std::lock_guard<std::mutex> lock(error_mutex);
  return error_message;
}

void PiCameraImuDevice::fail(const std::string& message) {
  {
    std::lock_guard<std::mutex> lock(error_mutex);
    if (error_message.empty()) error_message = message;
  }
  failed = true;
  capturing = false;
}

void PiCameraImuDevice::setSensorControl(uint32_t id, int value) {
  v4l2_control control{id, value};
  if (xioctl(sensor_fd, VIDIOC_S_CTRL, &control) < 0) throw systemError("VIDIOC_S_CTRL");
}

void PiCameraImuDevice::setupCamera() {
  if (options.xeyeProcessing()) {
    setupXeyeCamera();
    return;
  }
  MediaGraph graph(options.sensor);
  const MediaEntity& sensor = *graph.findPrefix(options.sensor);
  receiver_driver = graph.driver();
  const bool unicam = receiver_driver == UNICAM;
  const MediaEntity& video = graph.get(unicam ? "unicam-image" : "rp1-cfe-csi2_ch0");
  video_fd = open(video.node.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (video_fd < 0) throw systemError("Cannot open " + video.node);
  v4l2_capability capability{};
  if (xioctl(video_fd, VIDIOC_QUERYCAP, &capability) < 0) throw systemError("VIDIOC_QUERYCAP");
  const uint32_t device_caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
                                   ? capability.device_caps
                                   : capability.capabilities;
  // unicam without the media controller (overlay parameter media-controller=0):
  // the video node format sets the sensor mode, and the video node has the
  // sensor controls.
  const bool video_centric = unicam && !(device_caps & V4L2_CAP_IO_MC);
  if (!unicam) {
    const MediaEntity& csi2 = graph.get("csi2");
    const uint16_t csi2_source = graph.enableLink(csi2, video);
    setSubdevFormat(sensor.node, 0, options.width, options.height);
    setSubdevFormat(csi2.node, 0, options.width, options.height);
    setSubdevFormat(csi2.node, csi2_source, options.width, options.height);
  } else if (!video_centric) {
    // unicam has no csi2 entity; the link from the sensor to unicam-image is
    // fixed and enabled.
    setSubdevFormat(sensor.node, 0, options.width, options.height);
  }
  // The video format comes before the sensor controls: in the video centric
  // case it sets the sensor mode, which changes the control ranges.
  v4l2_format format{};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = options.width;
  format.fmt.pix.height = options.height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_SGRBG10P;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(video_fd, VIDIOC_S_FMT, &format) < 0) throw systemError("VIDIOC_S_FMT");
  if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_SGRBG10P ||
      int(format.fmt.pix.width) != options.width || int(format.fmt.pix.height) != options.height)
    throw std::runtime_error(receiver_driver +
                             " does not accept packed SGRBG10 at the sensor mode");
  bytes_per_line = format.fmt.pix.bytesperline;
  frame_start_stamp = unicam;

  sensor_fd = video_centric ? dup(video_fd) : open(sensor.node.c_str(), O_RDWR | O_CLOEXEC);
  if (sensor_fd < 0) throw systemError("Cannot open the sensor controls");
  const double pixel_rate = getControl64(sensor_fd, V4L2_CID_PIXEL_RATE);
  const int hblank = getControl(sensor_fd, V4L2_CID_HBLANK);
  line_ns = (options.width + hblank) * 1e9 / pixel_rate;
  const auto vblank_range = queryControl(sensor_fd, V4L2_CID_VBLANK);
  // The minimum vertical blanking sets the highest frame rate of the mode.
  setSensorControl(V4L2_CID_VBLANK,
                   std::clamp(static_cast<int>(std::lround(1e9 / options.fps / line_ns)) -
                                  options.height,
                              vblank_range.minimum, vblank_range.maximum));
  const int vblank = getControl(sensor_fd, V4L2_CID_VBLANK);
  frame_rate = 1e9 / ((options.height + vblank) * line_ns);

  const auto exposure_range = queryControl(sensor_fd, V4L2_CID_EXPOSURE);
  const auto gain_range = queryControl(sensor_fd, V4L2_CID_ANALOGUE_GAIN);
  frame_exposure_lines = std::clamp(
      options.height + vblank - static_cast<int>(std::ceil(EXPOSURE_MARGIN_NS / line_ns)),
      exposure_range.minimum, exposure_range.maximum);
  // The sensor reports gain as 100 x linear gain. The driver range goes past
  // the last value with an effect.
  min_gain = std::max(gain_range.minimum, B0429_MIN_DRIVER_GAIN);
  max_gain = std::clamp(static_cast<int>(std::lround(options.max_gain * min_gain)), min_gain,
                        std::min(gain_range.maximum, B0429_MAX_DRIVER_GAIN));
  max_exposure_lines =
      std::clamp(static_cast<int>(options.max_exposure_ms * 1e6 / line_ns),
                 exposure_range.minimum, frame_exposure_lines);
  ExposureSetting setting;
  if (options.exposure_ms > 0) {
    setting.lines = std::clamp(static_cast<int>(std::lround(options.exposure_ms * 1e6 / line_ns)),
                               exposure_range.minimum, frame_exposure_lines);
    setting.gain = std::clamp(b0429DriverGainFromDb(options.gain_db), min_gain,
                              std::min(gain_range.maximum, B0429_MAX_DRIVER_GAIN));
  } else {
    setting.lines = max_exposure_lines / 2;
    setting.gain = min_gain;
  }
  setSensorControl(V4L2_CID_EXPOSURE, setting.lines);
  setSensorControl(V4L2_CID_ANALOGUE_GAIN, setting.gain);
  exposure_lines = setting.lines;
  analogue_gain = setting.gain;

  v4l2_requestbuffers request{};
  request.count = VIDEO_BUFFERS;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  if (xioctl(video_fd, VIDIOC_REQBUFS, &request) < 0) throw systemError("VIDIOC_REQBUFS");
  for (uint32_t index = 0; index < request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (xioctl(video_fd, VIDIOC_QUERYBUF, &buffer) < 0) throw systemError("VIDIOC_QUERYBUF");
    void* memory = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd,
                        buffer.m.offset);
    if (memory == MAP_FAILED) throw systemError("mmap");
    buffers.emplace_back(memory, buffer.length);
    if (xioctl(video_fd, VIDIOC_QBUF, &buffer) < 0) throw systemError("VIDIOC_QBUF");
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(video_fd, VIDIOC_STREAMON, &type) < 0) throw systemError("VIDIOC_STREAMON");
}

void PiCameraImuDevice::writeImu(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  i2c_msg message{static_cast<uint16_t>(options.imu_address), 0, 2, data};
  i2c_rdwr_ioctl_data transfer{&message, 1};
  if (xioctl(i2c_fd, I2C_RDWR, &transfer) != 1) throw systemError("IMU I2C write");
}

void PiCameraImuDevice::readImu(uint8_t reg, uint8_t* data, size_t size) {
  i2c_msg messages[2] = {
      {static_cast<uint16_t>(options.imu_address), 0, 1, &reg},
      {static_cast<uint16_t>(options.imu_address), I2C_M_RD, static_cast<uint16_t>(size), data}};
  i2c_rdwr_ioctl_data transfer{messages, 2};
  if (xioctl(i2c_fd, I2C_RDWR, &transfer) != 2) throw systemError("IMU I2C read");
}

void PiCameraImuDevice::setupImu() {
  if (options.mavlink()) {
    setupMavlink();
    return;
  }
  i2c_fd = open(options.i2c_device.c_str(), O_RDWR | O_CLOEXEC);
  if (i2c_fd < 0) throw systemError("Cannot open " + options.i2c_device);
  uint8_t id = 0;
  readImu(MPU_WHO_AM_I, &id, 1);
  // 0x70: MPU-6500, 0x71: MPU-9250, 0x73: MPU-9255.
  if (id != 0x70 && id != 0x71 && id != 0x73) {
    throw std::runtime_error("Unexpected MPU WHO_AM_I value " + std::to_string(id));
  }
  writeImu(MPU_PWR_MGMT_1, 0x80);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  writeImu(MPU_PWR_MGMT_1, 0x01);  // PLL with gyro reference
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  writeImu(MPU_PWR_MGMT_2, 0x00);
  // FIFO_MODE=1 stops at full FIFO. Overwriting would split 12-byte samples.
  writeImu(MPU_CONFIG, 0x40 | 0x02);  // gyro DLPF 92 Hz, 1 kHz internal rate
  // The sample rate is 1 kHz / (1 + SMPLRT_DIV).
  const long divider = std::lround(1000 / options.imu_rate);
  writeImu(MPU_SMPLRT_DIV, static_cast<uint8_t>(divider - 1));
  writeImu(MPU_GYRO_CONFIG, 0x10);    // +-1000 deg/s
  writeImu(MPU_ACCEL_CONFIG, 0x08);   // +-4 g
  writeImu(MPU_ACCEL_CONFIG2, 0x02);  // accel DLPF 99 Hz
  gyro_scale = M_PI / 180.0 / 32.8;
  accel_scale = constants::g.norm() / 8192.0;
  imu_clock = FifoSampleClock(1e6 * divider);
}

void PiCameraImuDevice::start() {
  if (running) return;
  failed = false;
  error_message.clear();
  setupImu();
  setupCamera();
  running = true;
  camera_thread = std::thread([this] {
    pthread_setname_np(pthread_self(), "pi_camera");
    if (options.xeyeProcessing())
      xeyeCameraLoop();
    else
      cameraLoop();
  });
  imu_thread = std::thread([this] {
    pthread_setname_np(pthread_self(), "pi_imu");
    if (options.mavlink())
      mavlinkLoop();
    else
      imuLoop();
  });
}

void PiCameraImuDevice::enableCapture() { capturing = true; }

void PiCameraImuDevice::cameraLoop() {
  try {
    bool have_sequence = false;
    uint32_t last_sequence = 0;
    std::vector<uint8_t> frame_bytes(bytes_per_line * options.height);
    // The requested setting, and the exposure of the frame in process. A
    // request waits in `pending` until the frame that it applies to.
    ExposureSetting setting{exposure_lines.load(), analogue_gain.load()};
    int frame_lines = setting.lines;
    struct Request {
      uint32_t sequence;
      int lines;
    };
    std::deque<Request> pending;
    while (running) {
      pollfd descriptor{video_fd, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 200);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0) throw systemError("poll camera");
      if (ready == 0) continue;
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      if (xioctl(video_fd, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno == EAGAIN) continue;
        throw systemError("VIDIOC_DQBUF");
      }
      const bool corrupt = buffer.flags & V4L2_BUF_FLAG_ERROR;
      const bool monotonic =
          (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
      const bool start_of_exposure =
          (buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
      const int64_t buffer_ns =
          monotonic ? int64_t(buffer.timestamp.tv_sec) * 1'000'000'000 +
                          buffer.timestamp.tv_usec * 1000
                    : monotonicNs();
      cv::Mat gray;
      if (!corrupt) {
        // The capture buffers are not CPU cached. One block copy reads them
        // much faster than the byte reads of the unpacking.
        std::memcpy(frame_bytes.data(), buffers[buffer.index].first, frame_bytes.size());
        gray = grbgToGray(
            unpackRaw10Msb(frame_bytes.data(), options.width, options.height, bytes_per_line),
            options.downscale);
      }
      // QBUF writes the queued state back into the struct, including sequence.
      const uint32_t sequence = buffer.sequence;
      if (xioctl(video_fd, VIDIOC_QBUF, &buffer) < 0) throw systemError("VIDIOC_QBUF");
      if (have_sequence && sequence > last_sequence + 1)
        sensor_frame_drops += sequence - last_sequence - 1;
      have_sequence = true;
      last_sequence = sequence;
      // The first frames after STREAMON use the default frame length.
      if (corrupt || sequence < STARTUP_FRAMES) continue;

      while (!pending.empty() && pending.front().sequence <= sequence) {
        frame_lines = pending.front().lines;
        pending.pop_front();
      }
      // Global shutter: exposure, then readout of all rows. The image time is
      // the exposure center. rp1-cfe stamps the end of the frame: a readout
      // and half an exposure earlier. unicam stamps the start of the frame
      // (its first data): half an exposure earlier.
      const double exposure_ns = frame_lines * line_ns;
      const double center_offset_ns =
          start_of_exposure   ? exposure_ns / 2
          : frame_start_stamp ? -exposure_ns / 2
                              : -(options.height * line_ns + exposure_ns / 2);
      const int64_t t_ns = buffer_ns + static_cast<int64_t>(center_offset_ns) +
                           options.camera_time_offset_ns;
      const double brightness =
          metering_mask.empty() ? cv::mean(gray)[0] : cv::mean(gray, metering_mask)[0];
      mean_brightness = brightness;
      ++image_frames;
      last_image_t_ns = t_ns;
      if (capturing && image_data_queue) {
        auto data = std::make_shared<OpticalFlowInput>();
        data->t_ns = t_ns;
        data->img_data.resize(1);
        data->img_data[0].img = copyGrayImage(gray);
        data->img_data[0].exposure = exposure_ns * 1e-9;
        // A full queue drops its oldest image: the newest image has the
        // newest information.
        while (!image_data_queue->try_push(data)) {
          OpticalFlowInput::Ptr old;
          if (image_data_queue->try_pop(old)) ++dropped_frames;
        }
        if (queued_frames++ == 0) first_capture_t_ns = t_ns;
        if (record_image_queue && !record_image_queue->try_push(data)) ++record_dropped_frames;
        const int64_t delay_ns = monotonicNs() - buffer_ns;
        image_delay_sum_ns += delay_ns;
        if (delay_ns > image_delay_max_ns) image_delay_max_ns = delay_ns;
      }

      // Automatic exposure after the image is out: a control write takes some
      // milliseconds of I2C. The next change waits until the sensor applies
      // the last one.
      if (options.exposure_ms == 0 && pending.empty()) {
        const auto next = nextAutoExposure(setting, brightness, options.brightness_target,
                                           max_exposure_lines, min_gain, max_gain);
        if (next.lines != setting.lines) setSensorControl(V4L2_CID_EXPOSURE, next.lines);
        if (next.gain != setting.gain) setSensorControl(V4L2_CID_ANALOGUE_GAIN, next.gain);
        if (next.lines != setting.lines || next.gain != setting.gain)
          pending.push_back({sequence + CONTROL_DELAY_FRAMES, next.lines});
        setting = next;
        exposure_lines = setting.lines;
        analogue_gain = setting.gain;
      }
    }
  } catch (const std::exception& error) {
    fail(std::string("Camera: ") + error.what());
  }
}

void PiCameraImuDevice::imuLoop() {
  try {
    const auto reset_fifo = [this] {
      writeImu(MPU_FIFO_EN, 0x00);
      writeImu(MPU_USER_CTRL, 0x04);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      writeImu(MPU_USER_CTRL, 0x40);
      writeImu(MPU_FIFO_EN, 0x78);  // gyro x/y/z and accel
      imu_clock.reset();
    };
    reset_fifo();
    std::vector<uint8_t> bytes;
    auto next_poll = std::chrono::steady_clock::now();
    int consecutive_errors = 0, empty_polls = 0;
    bool need_reset = false, realign = false, reading_data = false;
    while (running) {
      next_poll += IMU_POLL_PERIOD;
      std::this_thread::sleep_until(next_poll);
      const int64_t poll_start_ns = monotonicNs();
      try {
        // An empty FIFO for 10 polls (200 ms) means that it stopped, e.g.
        // after a reset that failed half way.
        if (need_reset || empty_polls >= 10) {
          need_reset = true;
          reset_fifo();
          need_reset = false;
          empty_polls = 0;
          next_poll = std::chrono::steady_clock::now();
          continue;
        }
        uint8_t count_bytes[2];
        const int64_t before_ns = monotonicNs();
        readImu(MPU_FIFO_COUNT, count_bytes, 2);
        const int64_t after_ns = monotonicNs();
        const size_t count = (count_bytes[0] & 0x1F) << 8 | count_bytes[1];
        // A full FIFO stops sampling, which breaks the sample index clock.
        if (count + MPU_FIFO_SAMPLE_BYTES > MPU_FIFO_BYTES) {
          ++imu_overflows;
          reset_fifo();
          realign = false;
          next_poll = std::chrono::steady_clock::now();
          continue;
        }
        // After a failed transfer, a FIFO that does not hold whole samples
        // starts with the end of a sample: read and discard those bytes.
        if (realign) {
          realign = false;
          const size_t partial = count % MPU_FIFO_SAMPLE_BYTES;
          if (partial != 0) {
            uint8_t discard[MPU_FIFO_SAMPLE_BYTES];
            readImu(MPU_FIFO_R_W, discard, partial);
            imu_clock.reset();
            continue;
          }
        }
        const size_t samples = count / MPU_FIFO_SAMPLE_BYTES;
        consecutive_errors = 0;
        if (samples == 0) {
          ++empty_polls;
          continue;
        }
        empty_polls = 0;
        bytes.resize(samples * MPU_FIFO_SAMPLE_BYTES);
        reading_data = true;
        for (size_t offset = 0; offset < bytes.size(); offset += MPU_READ_CHUNK) {
          readImu(MPU_FIFO_R_W, bytes.data() + offset,
                  std::min(MPU_READ_CHUNK, bytes.size() - offset));
        }
        reading_data = false;
        const auto times = imu_clock.assign(before_ns, after_ns, samples);
        imu_period_ns = imu_clock.periodNs();
        for (size_t i = 0; i < samples; ++i) {
          ++imu_samples;
          if (!capturing || !imu_data_queue) continue;
          auto data = std::make_shared<ImuData>();
          data->t_ns = times[i];
          decodeMpuFifoSample(bytes.data() + i * MPU_FIFO_SAMPLE_BYTES, accel_scale, gyro_scale,
                              data->accel, data->gyro);
          imu_data_queue->push(data);
          if (record_imu_queue) record_imu_queue->push(data);
        }
      } catch (const std::exception& error) {
        // The CP2112 bridge of the Pi 4 rig times out transfers after about
        // 54 ms (about one in 200 polls, also without the camera, also with
        // the xeye ImuMPU6050 loop), and the next transfer often fails too.
        // Do not reset the FIFO: a reset drops the samples in the FIFO
        // (4.5 % of the samples in 60 s). A failed FIFO_COUNT read loses no
        // sample. A failed FIFO data read lost the samples of that read
        // (whole samples in a 60 s test), which breaks the sample index of
        // the clock: restart the clock. Then check the sample framing at the
        // next FIFO_COUNT (realign). The loss is then about 0.3 %.
        const int64_t lost_ns = monotonicNs() - poll_start_ns;
        ++imu_i2c_errors;
        if (lost_ns > imu_error_max_ns) imu_error_max_ns = lost_ns;
        if (++consecutive_errors > 5) throw;
        if (reading_data) imu_clock.reset();
        reading_data = false;
        realign = true;
        next_poll = std::chrono::steady_clock::now();
      }
      // A slow bridge must not accumulate a backlog of sleep deadlines.
      next_poll = std::max(next_poll, std::chrono::steady_clock::now() - IMU_POLL_PERIOD);
    }
  } catch (const std::exception& error) {
    fail(std::string("IMU: ") + error.what());
  }
}

void PiCameraImuDevice::setXeyeExposure(double exposure_ms) {
  // The exposure control takes sensor lines. An exposure longer than the
  // frame period makes the frame period longer.
  const auto range = queryControl(sensor_fd, V4L2_CID_EXPOSURE);
  const int lines =
      std::clamp(static_cast<int>(std::lround(exposure_ms * 1e6 / line_ns)),
                 range.minimum, range.maximum);
  setSensorControl(V4L2_CID_EXPOSURE, lines);
  exposure_time_ms = exposure_ms;
  exposure_lines = lines;
}

void PiCameraImuDevice::setXeyeGain(int value) {
  const auto range = queryControl(sensor_fd, V4L2_CID_ANALOGUE_GAIN);
  value = std::clamp(value, range.minimum, range.maximum);
  setSensorControl(V4L2_CID_ANALOGUE_GAIN, value);
  analogue_gain = value;
}

// The media graph, the controls, and the buffers of the xeye node
// (ArducamB0429Pi::Initialize and OnPreCaptureStart).
void PiCameraImuDevice::setupXeyeCamera() {
  MediaGraph graph(options.sensor, options.port);
  receiver_driver = graph.driver();
  if (receiver_driver != RP1_CFE)
    throw std::runtime_error(options.model +
                             " needs the rp1-cfe of a Raspberry Pi 5");
  const MediaEntity& sensor = *graph.findPrefix(options.sensor);
  const MediaEntity& csi2 = graph.get("csi2");
  const MediaEntity& video = options.port.empty()
                                 ? graph.get("rp1-cfe-csi2_ch0")
                                 : *graph.findNode(options.port);
  video_fd = open(video.node.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (video_fd < 0) throw systemError("Cannot open " + video.node);
  const int width = options.width;
  const int height = options.height;
  const uint16_t csi2_source = graph.enableLink(csi2, video);
  setSubdevFormat(sensor.node, 0, width, height);
  setSubdevFormat(csi2.node, 0, width, height);
  // The 16-bit source code makes csi2 unpack RAW10 to 16-bit words with the
  // value in the upper 10 bits. With the 10-bit code, STREAMON of GR16 fails.
  setSubdevFormat(csi2.node, csi2_source, width, height,
                  MEDIA_BUS_FMT_SGRBG16_1X16);
  // STREAMON fails with EPIPE when another enabled source pad of the sensor,
  // for example the embedded data pad, has a format other than its csi2 sink
  // pad.
  for (const auto& link : graph.links(sensor)) {
    if (link.source.entity != sensor.id || link.source.index == 0 ||
        link.sink.entity != csi2.id || !(link.flags & MEDIA_LNK_FL_ENABLED))
      continue;
    const v4l2_mbus_framefmt source =
        getSubdevFormat(sensor.node, link.source.index);
    setSubdevFormat(csi2.node, link.sink.index, source.width, source.height,
                    source.code);
  }

  v4l2_format format{};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = width;
  format.fmt.pix.height = height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_SGRBG16;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(video_fd, VIDIOC_S_FMT, &format) < 0)
    throw systemError("VIDIOC_S_FMT");
  if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_SGRBG16 ||
      format.fmt.pix.bytesperline != width * 2u)
    throw std::runtime_error(
        "rp1-cfe does not give GR16 at the sensor mode without row padding");
  bytes_per_line = format.fmt.pix.bytesperline;

  sensor_fd = open(sensor.node.c_str(), O_RDWR | O_CLOEXEC);
  if (sensor_fd < 0) throw systemError("Cannot open " + sensor.node);
  if (options.kurokesu()) {
    line_ns = KUROKESU_LINE_TIME_NS;
  } else {
    const int hblank = getControl(sensor_fd, V4L2_CID_HBLANK);
    line_ns = (width + hblank) * 1e9 /
              static_cast<double>(getControl64(sensor_fd, V4L2_CID_PIXEL_RATE));
  }
  // The frame period is (height + vertical blanking) lines.
  const auto vblank_range = queryControl(sensor_fd, V4L2_CID_VBLANK);
  const int vblank = std::clamp(
      static_cast<int>(std::lround(1e9 / options.fps / line_ns)) - height,
      vblank_range.minimum, vblank_range.maximum);
  setSensorControl(V4L2_CID_VBLANK, vblank);
  frame_rate = 1e9 / ((height + vblank) * line_ns);
  // An exposure longer than the frame period lowers the frame rate.
  const double limit_ms = std::max(
      1000.0 / options.fps - EXPOSURE_MARGIN_NS * 1e-6, MIN_EXPOSURE_MS);
  frame_exposure_lines = static_cast<int>(limit_ms * 1e6 / line_ns);

  auto& ae = options.auto_exposure;
  const auto gain_range = queryControl(sensor_fd, V4L2_CID_ANALOGUE_GAIN);
  const double min_gain_db = 20 * std::log10(linearGain(gain_range.minimum));
  const double max_gain_db = 20 * std::log10(linearGain(gain_range.maximum));
  if (ae.gain_range_db.empty()) ae.gain_range_db = {min_gain_db, max_gain_db};
  if (ae.gain_range_db[0] < min_gain_db - 0.01 ||
      ae.gain_range_db[1] > max_gain_db + 0.01)
    throw std::invalid_argument("auto_gain_range must be [min, max] in [" +
                                std::to_string(min_gain_db) + ", " +
                                std::to_string(max_gain_db) + "] dB");
  if (ae.auto_exposure) {
    auto& range = ae.exposure_range_ms;
    if (range[0] > limit_ms)
      throw std::invalid_argument(
          "auto_exposure_range is [" + std::to_string(range[0]) + ", " +
          std::to_string(range[1]) + "] ms, but the max exposure at " +
          std::to_string(options.fps) + " fps is " + std::to_string(limit_ms) +
          " ms");
    if (range[1] > limit_ms) {
      std::cerr << "Camera: the max of auto_exposure_range changes from "
                << range[1] << " to " << limit_ms
                << " ms. A longer exposure lowers the frame rate." << std::endl;
      range[1] = limit_ms;
    }
  }
  setXeyeExposure(options.exposure_time_ms);
  setXeyeGain(options.gain);
  // The driver locks the flip controls during streaming.
  setSensorControl(V4L2_CID_HFLIP, options.h_flip ? 1 : 0);
  setSensorControl(V4L2_CID_VFLIP, options.v_flip ? 1 : 0);

  gamma_curve = options.enable_gamma ? buildGammaCurve(options.gamma_value)
                                     : std::vector<uint8_t>{};
  gamma_linear10 = buildGammaLinear10(options.gamma_value, options.black_level);
  bayer8.assign(static_cast<size_t>(crop_w) * crop_h, 0);
  wb_gains.reset();
  ae_wait_frames = 0;

  // The CPU reads the MMAP buffers of the rp1-cfe without cache, at about
  // 1.2 GB/s. It reads buffers from the dma-heap through its cache, at about
  // 5 GB/s.
  const size_t size = format.fmt.pix.sizeimage;
  buffer_memory = V4L2_MEMORY_DMABUF;
  try {
    const int heap_fd = open(DMA_HEAP, O_RDWR | O_CLOEXEC);
    if (heap_fd < 0) throw systemError(std::string("Cannot open ") + DMA_HEAP);
    v4l2_requestbuffers request{};
    request.count = options.num_v4l_buffer;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_DMABUF;
    if (xioctl(video_fd, VIDIOC_REQBUFS, &request) < 0) {
      close(heap_fd);
      throw systemError("VIDIOC_REQBUFS (DMABUF)");
    }
    for (uint32_t index = 0; index < request.count; ++index) {
      dma_heap_allocation_data allocation{};
      allocation.len = size;
      allocation.fd_flags = O_RDWR | O_CLOEXEC;
      if (xioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0) {
        close(heap_fd);
        throw systemError("DMA_HEAP_IOCTL_ALLOC");
      }
      dma_buffer_fds.push_back(static_cast<int>(allocation.fd));
      void* memory = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                          static_cast<int>(allocation.fd), 0);
      if (memory == MAP_FAILED) {
        close(heap_fd);
        throw systemError("mmap of a DMABUF");
      }
      buffers.emplace_back(memory, size);
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_DMABUF;
      buffer.index = index;
      buffer.m.fd = static_cast<int>(allocation.fd);
      buffer.length = size;
      if (xioctl(video_fd, VIDIOC_QBUF, &buffer) < 0) {
        close(heap_fd);
        throw systemError("VIDIOC_QBUF (DMABUF)");
      }
    }
    close(heap_fd);
  } catch (const std::exception& error) {
    std::cerr << "Camera: " << error.what()
              << ". The camera uses the uncached MMAP buffers." << std::endl;
    v4l2_requestbuffers release{};
    release.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    release.memory = V4L2_MEMORY_DMABUF;
    xioctl(video_fd, VIDIOC_REQBUFS, &release);
    for (const auto& buffer : buffers) munmap(buffer.first, buffer.second);
    buffers.clear();
    for (const int fd : dma_buffer_fds) close(fd);
    dma_buffer_fds.clear();
    buffer_memory = V4L2_MEMORY_MMAP;
    v4l2_requestbuffers request{};
    request.count = options.num_v4l_buffer;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(video_fd, VIDIOC_REQBUFS, &request) < 0)
      throw systemError("VIDIOC_REQBUFS");
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (xioctl(video_fd, VIDIOC_QUERYBUF, &buffer) < 0)
        throw systemError("VIDIOC_QUERYBUF");
      void* memory = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, video_fd, buffer.m.offset);
      if (memory == MAP_FAILED) throw systemError("mmap");
      buffers.emplace_back(memory, buffer.length);
      if (xioctl(video_fd, VIDIOC_QBUF, &buffer) < 0)
        throw systemError("VIDIOC_QBUF");
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(video_fd, VIDIOC_STREAMON, &type) < 0)
    throw systemError("VIDIOC_STREAMON");
  pollfd descriptor{video_fd, POLLIN, 0};
  int ready = 0;
  do ready = poll(&descriptor, 1, FIRST_FRAME_TIMEOUT_MS);
  while (ready < 0 && errno == EINTR);
  if (ready <= 0)
    throw std::runtime_error(
        "No frame for " + std::to_string(FIRST_FRAME_TIMEOUT_MS) +
        " ms after stream on. Check the cable of the camera.");
}

// FillFrame of the xeye node: drops an invalid frame, and makes the gray
// image of the crop from the 10-bit Bayer buffer.
bool PiCameraImuDevice::processXeyeFrame(const v4l2_buffer& buffer,
                                         cv::Mat* gray) {
  const size_t frame_bytes =
      static_cast<size_t>(options.width) * options.height * 2;
  const int64_t buffer_ns = int64_t(buffer.timestamp.tv_sec) * 1'000'000'000 +
                            buffer.timestamp.tv_usec * 1000;
  if ((buffer.flags & V4L2_BUF_FLAG_ERROR) || buffer.bytesused < frame_bytes ||
      buffer_ns <= 0)
    return false;
  const auto sync = [&](uint64_t flags) {
    if (buffer_memory != V4L2_MEMORY_DMABUF) return;
    // The device wrote the buffer, so the CPU cache can hold old data of it.
    // DMA_BUF_SYNC_START before the reads drops that data.
    dma_buf_sync request{};
    request.flags = flags | DMA_BUF_SYNC_READ;
    if (xioctl(dma_buffer_fds.at(buffer.index), DMA_BUF_IOCTL_SYNC, &request) <
        0)
      throw systemError("DMA_BUF_IOCTL_SYNC");
  };
  sync(DMA_BUF_SYNC_START);
  const auto* src =
      static_cast<const uint16_t*>(buffers.at(buffer.index).first) +
      static_cast<size_t>(crop_y) * options.width + crop_x;
  if (!options.auto_white_balance && !options.enable_gamma) {
    copyBayer10LShift8(src, options.width, crop_w, crop_h, bayer8.data());
  } else {
    // Without fused_wb, or at the first frame, the gains come from the
    // statistics of the same frame.
    std::optional<std::array<double, 2>> gains =
        options.fused_wb ? wb_gains : std::nullopt;
    if (options.auto_white_balance && !gains) {
      gains = gainsFromWbStats(
          accumulateBayer10LStats(src, options.width, crop_w, crop_h,
                                  GRBG_NON_GREEN_COL, options.black_level));
    }
    Bayer8Lut lut;
    buildBayer8Lut(gamma_curve, gains.value_or(std::array<double, 2>{1.0, 1.0}),
                   options.black_level, &lut);
    WbStats stats{0, 0, 0};
    const bool fused = options.auto_white_balance && options.fused_wb;
    copyBayer10LTo8(src, options.width, crop_w, crop_h, GRBG_NON_GREEN_COL, lut,
                    options.black_level, fused ? &stats : nullptr,
                    bayer8.data());
    if (fused) {
      // Keep the previous gains when the frame has no usable estimate.
      const auto new_gains = gainsFromWbStats(stats);
      if (new_gains) wb_gains = new_gains;
    }
  }
  sync(DMA_BUF_SYNC_END);
  if (options.downscale == 1) {
    *gray = cv::Mat(crop_h, crop_w, CV_8UC1);
    bayer8ToGray8(bayer8.data(), crop_w, crop_h, GRBG_NON_GREEN_COL,
                  gray->data);
  } else {
    *gray = grbgToGray(cv::Mat(crop_h, crop_w, CV_8UC1, bayer8.data()),
                       options.downscale);
  }
  return true;
}

// The automatic exposure of the xeye node, on the output image.
void PiCameraImuDevice::runXeyeAutoExposure(const cv::Mat& gray) {
  const auto& ae = options.auto_exposure;
  const PixelRegion region = autoExposureRegion(ae, gray.cols, gray.rows);
  const auto* pixels = gray.ptr<uint8_t>(0);
  const double percent =
      options.enable_gamma
          ? meanPercent(pixels, gray.cols, region, ae.circle, BAYER10_MAX,
                        [&](uint8_t v) { return gamma_linear10[v]; })
          : meanPercent(pixels, gray.cols, region, ae.circle, 255.0,
                        [](uint8_t v) { return v; });
  mean_brightness = percent * 2.55;
  if (ae_wait_frames > 0) {
    --ae_wait_frames;
    return;
  }
  if (!ae.auto_exposure && !ae.auto_gain) return;
  const double gain = linearGain(analogue_gain.load());
  const auto next = nextXeyeExposure(ae, percent, exposure_time_ms, gain);
  if (!next) return;
  const int new_gain = gainValue(next->second);
  const bool set_exposure =
      std::abs(next->first - exposure_time_ms) > 1e-3 * exposure_time_ms;
  const bool set_gain = new_gain != analogue_gain.load();
  if (!set_exposure && !set_gain) return;
  if (set_exposure) setXeyeExposure(next->first);
  if (set_gain) setXeyeGain(new_gain);
  ae_wait_frames = AE_SETTLE_FRAMES;
}

void PiCameraImuDevice::xeyeCameraLoop() {
  try {
    bool have_sequence = false;
    uint32_t last_sequence = 0;
    while (running) {
      pollfd descriptor{video_fd, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 200);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0) throw systemError("poll camera");
      if (ready == 0) continue;
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = buffer_memory;
      if (xioctl(video_fd, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno == EAGAIN) continue;
        throw systemError("VIDIOC_DQBUF");
      }
      // The image time is the V4L2 buffer time, as in the xeye node.
      const int64_t buffer_ns =
          int64_t(buffer.timestamp.tv_sec) * 1'000'000'000 +
          buffer.timestamp.tv_usec * 1000;
      cv::Mat gray;
      const bool valid = processXeyeFrame(buffer, &gray);
      const uint32_t sequence = buffer.sequence;
      if (xioctl(video_fd, VIDIOC_QBUF, &buffer) < 0)
        throw systemError("VIDIOC_QBUF");
      if (have_sequence && sequence > last_sequence + 1)
        sensor_frame_drops += sequence - last_sequence - 1;
      have_sequence = true;
      last_sequence = sequence;
      if (!valid) continue;

      const int64_t t_ns = buffer_ns + options.camera_time_offset_ns;
      ++image_frames;
      last_image_t_ns = t_ns;
      if (capturing && image_data_queue) {
        auto data = std::make_shared<OpticalFlowInput>();
        data->t_ns = t_ns;
        data->img_data.resize(1);
        data->img_data[0].img = copyGrayImage(gray);
        data->img_data[0].exposure = exposure_time_ms * 1e-3;
        // A full queue drops its oldest image: the newest image has the
        // newest information.
        while (!image_data_queue->try_push(data)) {
          OpticalFlowInput::Ptr old;
          if (image_data_queue->try_pop(old)) ++dropped_frames;
        }
        if (queued_frames++ == 0) first_capture_t_ns = t_ns;
        if (record_image_queue && !record_image_queue->try_push(data))
          ++record_dropped_frames;
        const int64_t delay_ns = monotonicNs() - buffer_ns;
        image_delay_sum_ns += delay_ns;
        if (delay_ns > image_delay_max_ns) image_delay_max_ns = delay_ns;
      }
      runXeyeAutoExposure(gray);
    }
  } catch (const std::exception& error) {
    fail(std::string("Camera: ") + error.what());
  }
}

void PiCameraImuDevice::setupMavlink() {
  if (!options.imu_port.empty()) {
    // A serial port, 8N1 without flow control, as the xeye SerialLink.
    speed_t speed;
    switch (options.imu_baudrate) {
      case 57600:
        speed = B57600;
        break;
      case 115200:
        speed = B115200;
        break;
      case 230400:
        speed = B230400;
        break;
      case 460800:
        speed = B460800;
        break;
      case 500000:
        speed = B500000;
        break;
      case 921600:
        speed = B921600;
        break;
      case 1000000:
        speed = B1000000;
        break;
      case 1500000:
        speed = B1500000;
        break;
      default:
        throw std::invalid_argument("unsupported baudrate " +
                                    std::to_string(options.imu_baudrate));
    }
    mavlink_fd = open(options.imu_port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (mavlink_fd < 0) throw systemError("Cannot open " + options.imu_port);
    termios tty{};
    if (tcgetattr(mavlink_fd, &tty) != 0)
      throw systemError("tcgetattr " + options.imu_port);
    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    if (tcsetattr(mavlink_fd, TCSANOW, &tty) != 0)
      throw systemError("tcsetattr " + options.imu_port);
    tcflush(mavlink_fd, TCIFLUSH);
    mavlink_udp = false;
    return;
  }
  const std::string& address = options.imu_udp;
  const size_t colon = address.rfind(':');
  if (colon == std::string::npos)
    throw std::invalid_argument("udp must be <address>:<port>, got '" +
                                address + "'");
  const std::string port_text = address.substr(colon + 1);
  if (port_text.empty() || port_text.size() > 5 ||
      !std::all_of(port_text.begin(), port_text.end(),
                   [](char c) { return c >= '0' && c <= '9'; }) ||
      std::stoi(port_text) < 1 || std::stoi(port_text) > 65535)
    throw std::invalid_argument("invalid udp port in '" + address + "'");
  sockaddr_in socket_address{};
  socket_address.sin_family = AF_INET;
  socket_address.sin_port = htons(static_cast<uint16_t>(std::stoi(port_text)));
  if (inet_pton(AF_INET, address.substr(0, colon).c_str(),
                &socket_address.sin_addr) != 1)
    throw std::invalid_argument("invalid udp address '" + address + "'");
  mavlink_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (mavlink_fd < 0) throw systemError("socket");
  if (bind(mavlink_fd, reinterpret_cast<const sockaddr*>(&socket_address),
           sizeof(socket_address)) != 0)
    throw systemError("bind " + address);
  mavlink_udp = true;
}

// The read loop of the xeye imu_mavlink node. The host time is CLOCK_MONOTONIC.
void PiCameraImuDevice::mavlinkLoop() {
  try {
    mavlink_message_t parse_buffer{};
    mavlink_status_t parse_status{};
    FcClockMapper clock_mapper;
    // System and component id of the autopilot, from its first HEARTBEAT.
    std::optional<std::pair<uint8_t, uint8_t>> fc_address;
    const int rate = options.imu_freq_hz;
    bool interval_settled = rate == 0;
    // Whether the current request went out: an ACK before that answers
    // another client on the link.
    bool interval_requested = false;
    int64_t last_request_ns = 0;
    int64_t last_imu_rx_ns = monotonicNs();
    int64_t last_fc_ns = -1;
    std::vector<uint8_t> datagram(65536);

    const auto from_fc = [&](const mavlink_message_t& message) {
      return fc_address && message.sysid == fc_address->first &&
             message.compid == fc_address->second;
    };
    const auto request_again = [&] {
      interval_settled = rate == 0;
      interval_requested = false;
      last_request_ns = 0;
    };
    const auto send = [&](const mavlink_message_t& message) {
      uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
      const uint16_t size = mavlink_msg_to_send_buffer(bytes, &message);
      // A lost request is sent again until the FC answers. UDP: nothing
      // received yet, so no one to answer.
      if (writeMavlink(bytes, size) < 0 && !mavlink_udp)
        std::cerr << "IMU: write failed" << std::endl;
    };
    const auto handle = [&](const mavlink_message_t& message, int64_t rx_ns) {
      switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          if (fc_address || message.compid != MAV_COMP_ID_AUTOPILOT1) return;
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&message, &heartbeat);
          if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID ||
              heartbeat.type == MAV_TYPE_GCS)
            return;
          fc_address.emplace(message.sysid, message.compid);
          std::cout << "IMU: autopilot " << int(heartbeat.autopilot)
                    << " found (sysid " << int(message.sysid) << ", compid "
                    << int(message.compid) << ")" << std::endl;
          return;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
          if (!from_fc(message) || interval_settled || !interval_requested)
            return;
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&message, &ack);
          // Another client on the link sends the same command; a zero target
          // is unaddressed.
          if (ack.command != MAV_CMD_SET_MESSAGE_INTERVAL ||
              (ack.target_system != 0 &&
               ack.target_system != MAVLINK_SYSTEM_ID) ||
              (ack.target_component != 0 &&
               ack.target_component != MAVLINK_COMPONENT_ID))
            return;
          if (ack.result == MAV_RESULT_ACCEPTED) {
            std::cout << "IMU: FC streams HIGHRES_IMU at " << rate << " Hz"
                      << std::endl;
            interval_settled = true;
          } else if (ack.result != MAV_RESULT_TEMPORARILY_REJECTED &&
                     ack.result != MAV_RESULT_IN_PROGRESS) {
            // Retrying cannot change a denied or unsupported request.
            std::cerr << "IMU: FC rejected the HIGHRES_IMU rate (MAV_RESULT "
                      << int(ack.result) << "); using the FC stream rate"
                      << std::endl;
            interval_settled = true;
          }
          return;
        }
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          if (!from_fc(message)) return;
          mavlink_highres_imu_t imu;
          mavlink_msg_highres_imu_decode(&message, &imu);
          const int64_t fc_ns = static_cast<int64_t>(imu.time_usec) * 1000;
          if (clock_mapper.isFcReboot(fc_ns)) {
            std::cerr << "IMU: FC sample time went back; assuming an FC reboot"
                      << std::endl;
            clock_mapper.reset();
            // A rebooted FC is back to its configured stream rates.
            request_again();
            last_fc_ns = -1;
          }
          last_imu_rx_ns = rx_ns;
          if (last_fc_ns >= 0 && fc_ns > last_fc_ns) {
            const double interval = static_cast<double>(fc_ns - last_fc_ns);
            imu_period_ns = imu_period_ns == 0
                                ? interval
                                : 0.99 * imu_period_ns + 0.01 * interval;
          }
          last_fc_ns = std::max(last_fc_ns, fc_ns);
          const std::optional<int64_t> host_ns =
              clock_mapper.toHostNs(fc_ns, rx_ns);
          if (!host_ns) return;
          ++imu_samples;
          if (!capturing || !imu_data_queue) return;
          // HIGHRES_IMU of PX4: the FC body frame (FRD after SENS_BOARD_ROT),
          // with the EKF2 bias estimates already subtracted.
          auto data = std::make_shared<ImuData>();
          data->t_ns = *host_ns;
          data->accel << imu.xacc, imu.yacc, imu.zacc;
          data->gyro << imu.xgyro, imu.ygyro, imu.zgyro;
          imu_data_queue->push(data);
          if (record_imu_queue) record_imu_queue->push(data);
          return;
        }
        default:
          return;
      }
    };

    while (running) {
      pollfd descriptor{mavlink_fd, POLLIN, 0};
      const int ready = poll(&descriptor, 1, MAVLINK_READ_TIMEOUT_MS);
      if (ready < 0 && errno != EINTR) throw systemError("poll MAVLink");
      const int64_t rx_ns = monotonicNs();
      while (ready > 0) {
        ssize_t size = 0;
        if (mavlink_udp) {
          sockaddr_in from{};
          socklen_t from_size = sizeof(from);
          size = recvfrom(mavlink_fd, datagram.data(), datagram.size(),
                          MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from),
                          &from_size);
          if (size > 0) {
            std::lock_guard<std::mutex> lock(mavlink_mutex);
            mavlink_peer = from;
          }
        } else {
          pollfd more{mavlink_fd, POLLIN, 0};
          if (poll(&more, 1, 0) <= 0) break;
          size = read(mavlink_fd, datagram.data(), datagram.size());
        }
        if (size <= 0) break;
        for (ssize_t i = 0; i < size; ++i) {
          mavlink_message_t message;
          mavlink_status_t status;
          const uint8_t result = mavlink_frame_char_buffer(
              &parse_buffer, &parse_status, datagram[i], &message, &status);
          if (result == MAVLINK_FRAMING_OK) {
            handle(message, rx_ns);
          } else if (result == MAVLINK_FRAMING_BAD_CRC ||
                     result == MAVLINK_FRAMING_BAD_SIGNATURE) {
            // The resynchronization of mavlink_parse_char().
            parse_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
            parse_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
            if (datagram[i] == MAVLINK_STX) {
              parse_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
              parse_buffer.len = 0;
              mavlink_start_checksum(&parse_buffer);
            }
          }
        }
      }
      if (fc_address && !interval_settled &&
          rx_ns - last_request_ns >= MAVLINK_REQUEST_RETRY_NS) {
        mavlink_message_t message;
        // sendOdometry() packs on the same channel from the state thread.
        std::lock_guard<std::mutex> lock(mavlink_mutex);
        // param7 = 0: the response target is the flight stack default.
        mavlink_msg_command_long_pack_chan(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &message,
            fc_address->first, fc_address->second, MAV_CMD_SET_MESSAGE_INTERVAL,
            0, static_cast<float>(MAVLINK_MSG_ID_HIGHRES_IMU),
            1e6f / static_cast<float>(rate), 0, 0, 0, 0, 0);
        send(message);
        interval_requested = true;
        last_request_ns = rx_ns;
      }
      // An FC that rebooted into a stream configuration without HIGHRES_IMU
      // sends no sample time that would show the reboot.
      if (interval_settled && rate > 0 &&
          rx_ns - last_imu_rx_ns >= MAVLINK_IMU_TIMEOUT_NS) {
        std::cerr << "IMU: no HIGHRES_IMU for 3 s; requesting it again"
                  << std::endl;
        request_again();
        last_imu_rx_ns = rx_ns;
      }
    }
  } catch (const std::exception& error) {
    fail(std::string("IMU: ") + error.what());
  }
}

ssize_t PiCameraImuDevice::writeMavlink(const uint8_t* bytes, size_t size) {
  if (!mavlink_udp) return write(mavlink_fd, bytes, size);
  if (!mavlink_peer) return 0;
  return sendto(mavlink_fd, bytes, size, 0,
                reinterpret_cast<const sockaddr*>(&*mavlink_peer),
                sizeof(*mavlink_peer));
}

void PiCameraImuDevice::sendOdometry(const PoseVelBiasState& state,
                                     uint8_t reset_counter, int8_t quality) {
  if (!options.imu_send_odometry) return;
  if (!state.T_w_i.matrix().allFinite() || !state.vel_w_i.allFinite()) return;
  // The VIO world is gravity aligned with z up (constants::g); LOCAL_FRD has
  // z down: a rotation of 180 deg about x. The IMU frame i is the FC body
  // frame (FRD), because mavlinkLoop() keeps the HIGHRES_IMU axes.
  const Eigen::Matrix3d R_frd_w = Eigen::Vector3d(1, -1, -1).asDiagonal();
  const Eigen::Matrix3d R_frd_body = R_frd_w * state.T_w_i.so3().matrix();
  const Eigen::Vector3d position = R_frd_w * state.T_w_i.translation();
  // ArduPilot rotates the velocity with q, so it is in the body frame.
  const Eigen::Vector3d velocity =
      R_frd_body.transpose() * (R_frd_w * state.vel_w_i);
  const Eigen::Quaterniond q(R_frd_body);
  const float nan = std::numeric_limits<float>::quiet_NaN();

  mavlink_odometry_t odometry{};
  odometry.time_usec = static_cast<uint64_t>(state.t_ns / 1000);
  odometry.frame_id = MAV_FRAME_LOCAL_FRD;
  odometry.child_frame_id = MAV_FRAME_BODY_FRD;
  odometry.x = static_cast<float>(position.x());
  odometry.y = static_cast<float>(position.y());
  odometry.z = static_cast<float>(position.z());
  odometry.q[0] = static_cast<float>(q.w());
  odometry.q[1] = static_cast<float>(q.x());
  odometry.q[2] = static_cast<float>(q.y());
  odometry.q[3] = static_cast<float>(q.z());
  odometry.vx = static_cast<float>(velocity.x());
  odometry.vy = static_cast<float>(velocity.y());
  odometry.vz = static_cast<float>(velocity.z());
  // ArduPilot does not use the angular speed. A NaN covariance makes it use
  // VISO_POS_M_NSE and VISO_YAW_M_NSE.
  odometry.rollspeed = nan;
  odometry.pitchspeed = nan;
  odometry.yawspeed = nan;
  std::fill(std::begin(odometry.pose_covariance),
            std::end(odometry.pose_covariance), nan);
  std::fill(std::begin(odometry.velocity_covariance),
            std::end(odometry.velocity_covariance), nan);
  odometry.reset_counter = reset_counter;
  odometry.estimator_type = MAV_ESTIMATOR_TYPE_VIO;
  odometry.quality = quality;

  std::lock_guard<std::mutex> lock(mavlink_mutex);
  if (mavlink_fd < 0) return;
  if (state.t_ns <= last_odometry_t_ns) {
    std::cerr << "ODOMETRY: state time " << state.t_ns
              << " ns is not after the last sent " << last_odometry_t_ns
              << " ns; not sent" << std::endl;
    return;
  }
  mavlink_message_t message;
  mavlink_msg_odometry_encode_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   MAVLINK_COMM_0, &message, &odometry);
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const uint16_t size = mavlink_msg_to_send_buffer(bytes, &message);
  const ssize_t written = writeMavlink(bytes, size);
  if (written < 0) {
    const int error = errno;
    ++odometry_errors;
    // One message per failure period; the VIO continues.
    if (!odometry_failing)
      std::cerr << "ODOMETRY: send failed: " << std::strerror(error)
                << std::endl;
    odometry_failing = true;
    return;
  }
  // UDP: no datagram from the router yet.
  if (written == 0) return;
  last_odometry_t_ns = state.t_ns;
  odometry_failing = false;
  ++odometry_sent;
}

void PiCameraImuDevice::stop(bool send_end_of_stream) {
  capturing = false;
  running = false;
  if (camera_thread.joinable()) camera_thread.join();
  if (imu_thread.joinable()) imu_thread.join();
  if (video_fd >= 0) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(video_fd, VIDIOC_STREAMOFF, &type);
    for (const auto& buffer : buffers) munmap(buffer.first, buffer.second);
    buffers.clear();
    close(video_fd);
    video_fd = -1;
  }
  for (const int fd : dma_buffer_fds) close(fd);
  dma_buffer_fds.clear();
  if (mavlink_fd >= 0) {
    std::lock_guard<std::mutex> lock(mavlink_mutex);
    close(mavlink_fd);
    mavlink_fd = -1;
    mavlink_peer.reset();
    last_odometry_t_ns = -1;
    odometry_failing = false;
  }
  if (sensor_fd >= 0) {
    close(sensor_fd);
    sensor_fd = -1;
  }
  if (i2c_fd >= 0) {
    try {
      writeImu(MPU_FIFO_EN, 0x00);
      writeImu(MPU_PWR_MGMT_1, 0x40);  // sleep
    } catch (const std::exception&) {
    }
    close(i2c_fd);
    i2c_fd = -1;
  }
  if (!send_end_of_stream) return;
  if (image_data_queue) image_data_queue->push(nullptr);
  if (imu_data_queue) imu_data_queue->push(nullptr);
}

}  // namespace granite
