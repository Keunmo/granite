// SPDX-License-Identifier: MIT
#include <granite/device/pi_camera_imu.h>

#include <cerrno>
#include <chrono>
#include <deque>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <granite/utils/imu_types.h>

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
  explicit MediaGraph(const std::string& sensor_prefix) {
    for (int index = 0; index < 16 && fd < 0; ++index) {
      const std::string path = "/dev/media" + std::to_string(index);
      const int candidate = open(path.c_str(), O_RDWR | O_CLOEXEC);
      if (candidate < 0) continue;
      media_device_info info{};
      if (xioctl(candidate, MEDIA_IOC_DEVICE_INFO, &info) == 0 &&
          (std::string(info.driver) == RP1_CFE || std::string(info.driver) == UNICAM)) {
        fd = candidate;
        entities = enumerate();
        if (findPrefix(sensor_prefix)) {
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

void setSubdevFormat(const std::string& node, uint32_t pad, int width, int height) {
  const int fd = open(node.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) throw systemError("Cannot open " + node);
  v4l2_subdev_format format{};
  format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  format.pad = pad;
  format.format.width = width;
  format.format.height = height;
  format.format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
  // CFE link validation compares the field of both pads.
  format.format.field = V4L2_FIELD_NONE;
  format.format.colorspace = V4L2_COLORSPACE_RAW;
  const int result = xioctl(fd, VIDIOC_SUBDEV_S_FMT, &format);
  close(fd);
  if (result < 0) throw systemError("VIDIOC_SUBDEV_S_FMT " + node);
  if (int(format.format.width) != width || int(format.format.height) != height ||
      format.format.code != MEDIA_BUS_FMT_SGRBG10_1X10) {
    throw std::runtime_error("Sensor mode " + std::to_string(width) + "x" +
                             std::to_string(height) + " SGRBG10 is not supported");
  }
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

PiCameraImuDevice::PiCameraImuDevice(const PiCameraImuOptions& options)
    : options(options), imu_clock(1e9 / options.imu_rate) {
  if (options.width <= 0 || options.height <= 0 || options.width % 4 != 0 ||
      options.height % 2 != 0 || !(options.fps > 0) || options.exposure_ms < 0 ||
      !(options.max_exposure_ms > 0) ||
      !(options.gain_db >= 0 && options.gain_db <= B0429_MAX_GAIN_DB) || !(options.max_gain >= 1) ||
      !(options.brightness_target > 0 && options.brightness_target < 255) ||
      options.imu_rate < 4 || options.imu_rate > 1000 || options.downscale < 1 ||
      !(options.metering_circle.empty() || options.metering_circle.size() == 3)) {
    throw std::invalid_argument("Invalid camera or IMU option");
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

int PiCameraImuDevice::outputWidth() const { return options.width / options.downscale; }

int PiCameraImuDevice::outputHeight() const { return options.height / options.downscale; }

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
    cameraLoop();
  });
  imu_thread = std::thread([this] {
    pthread_setname_np(pthread_self(), "pi_imu");
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
