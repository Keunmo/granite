// SPDX-License-Identifier: MIT
#include <granite/utils/vis_publisher.h>

#include <arpa/inet.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#ifdef GRANITE_WITH_X264
#include <x264.h>
#endif

namespace granite {

namespace {

// Field numbers of GraniteVis (doc/granite_vis.proto). 14, 15 and 20 are the projections and the float keypoints
// of version 1.
enum VisField : int {
  kVersion = 1,
  kSeq = 2,
  kTimeNs = 3,
  kMapIdx = 4,
  kPose = 5,
  kVelocity = 6,
  kBiasGyro = 7,
  kBiasAccel = 8,
  kHealthy = 9,
  kKeyframe = 10,
  kWindowStates = 11,
  kWindowFrames = 12,
  kPoints = 13,
  kImage = 16,
  kImageWidth = 17,
  kImageHeight = 18,
  kSkipped = 19,
  kPointIds = 21,
  kKeypoints = 22,
  kKeypointIds = 23,
  kImageCodec = 24,
  kImageKeyframe = 25,
  kCameraModel = 26,
  kCameraParams = 27,
  kTic = 28,
  kImageSeq = 29,
};
constexpr int kVisVersion = 2;
constexpr uint64_t kCodecH264 = 1;
// Keypoint pixels as uint16 in 1/8 pixel
constexpr float kKeypointScale = 8.0f;

// The largest UDP payload over IPv4.
constexpr size_t kMaxDatagram = 65507;
constexpr size_t kIpUdpHeader = 28;

// Writes the protobuf wire format (https://protobuf.dev/programming-guides/encoding/). Each field is a key
// varint (field number << 3 | wire type) and a value. A varint has 7 bits in each byte, the low bits first, and
// the high bit set on all bytes but the last. Floats are little-endian IEEE 754, the byte order of the Pi and of
// x86. Zero values are not written (proto3 defaults).
class ProtoWriter {
 public:
  void uint(int field, uint64_t value) {
    if (value == 0) return;
    key(field, 0);
    varint(value);
  }
  void floats(int field, const std::vector<float>& values) {
    if (values.empty()) return;
    key(field, 2);
    varint(values.size() * sizeof(float));
    const size_t at = out.size();
    out.resize(at + values.size() * sizeof(float));
    std::memcpy(&out[at], values.data(), values.size() * sizeof(float));
  }
  // Packed repeated varints, each the difference to the previous value (the first to 0). The values must not
  // decrease.
  void deltaUints(int field, const std::vector<uint64_t>& values) {
    if (values.empty()) return;
    ProtoWriter body;
    uint64_t previous = 0;
    for (uint64_t v : values) {
      body.varint(v - previous);
      previous = v;
    }
    bytes(field, body.out);
  }
  void bytes(int field, const std::string& value) {
    if (value.empty()) return;
    key(field, 2);
    varint(value.size());
    out.append(value);
  }
  std::string take() { return std::move(out); }

 private:
  void key(int field, int wire_type) { varint((static_cast<uint64_t>(field) << 3) | wire_type); }
  void varint(uint64_t value) {
    while (value >= 0x80) {
      out.push_back(static_cast<char>((value & 0x7f) | 0x80));
      value >>= 7;
    }
    out.push_back(static_cast<char>(value));
  }
  std::string out;
};

void appendPose(std::vector<float>& out, const Sophus::SE3d& T) {
  const Eigen::Vector3d& p = T.translation();
  const Eigen::Quaterniond& q = T.unit_quaternion();
  for (double v : {p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z()}) out.push_back(static_cast<float>(v));
}

void appendVector(std::vector<float>& out, const Eigen::Vector3d& v) {
  for (int i = 0; i < 3; ++i) out.push_back(static_cast<float>(v[i]));
}

void appendUint16(std::string& out, float value) {
  const auto v = static_cast<uint16_t>(std::clamp(std::lround(value), 0L, 65535L));
  out.push_back(static_cast<char>(v & 0xff));
  out.push_back(static_cast<char>(v >> 8));
}

const ManagedImage<PixelType>* imageOf(const VioVisualizationData& data) {
  if (!data.opt_flow_res || !data.opt_flow_res->input_images) return nullptr;
  const auto& images = data.opt_flow_res->input_images->img_data;
  return images.empty() || !images[0].img ? nullptr : images[0].img.get();
}

// A message time that is due for a rate of `hz`: the times have jitter, so 0.75 of the period is enough. A
// camera at `hz` then gives every frame.
class RateGate {
 public:
  explicit RateGate(double hz) : period_ns(hz > 0 ? static_cast<int64_t>(0.75e9 / hz) : -1) {}
  bool due(int64_t t_ns) const {
    return period_ns >= 0 && (!any || t_ns - last_ns >= period_ns || t_ns < last_ns);
  }
  void take(int64_t t_ns) {
    last_ns = t_ns;
    any = true;
  }

 private:
  const int64_t period_ns;
  int64_t last_ns = 0;
  bool any = false;
};

}  // namespace

#ifdef GRANITE_WITH_X264
// x264 for the gray camera image: one Annex B access unit for each image, the SPS and PPS before each IDR frame.
// One thread, no B frames and no lookahead (preset ultrafast, tune zerolatency), so each output belongs to the
// input image.
class VisPublisher::H264Encoder {
 public:
  explicit H264Encoder(const VisPublisherOptions& options)
      : hz(std::max(1.0, options.vis_msg_image_hz)),
        crf(options.vis_msg_h264_crf),
        capped(options.vis_msg_h264_rate_control == "bitrate_cap") {}
  ~H264Encoder() { close(); }

  // Empty when the encoder fails.
  std::string encode(const ManagedImage<PixelType>& image, int64_t t_ns, bool* keyframe) {
    const int w = static_cast<int>(image.w), h = static_cast<int>(image.h);
    if (encoder && (w != width || h != height)) close();
    if (!encoder && !open(w, h)) return {};
    for (int y = 0; y < h; ++y)
      std::memcpy(picture.img.plane[0] + static_cast<size_t>(y) * picture.img.i_stride[0],
                  reinterpret_cast<const uint8_t*>(image.ptr) + static_cast<size_t>(y) * image.pitch, w);
    picture.i_pts = t_ns;
    picture.i_type = force_idr ? X264_TYPE_IDR : X264_TYPE_AUTO;
    force_idr = false;
    x264_nal_t* nals = nullptr;
    int count = 0;
    x264_picture_t out;
    const int size = x264_encoder_encode(encoder, &nals, &count, &picture, &out);
    if (size <= 0 || count <= 0) return {};
    *keyframe = out.b_keyframe;
    // The payloads of one encode call are contiguous.
    return std::string(reinterpret_cast<const char*>(nals[0].p_payload), static_cast<size_t>(size));
  }

  // After a lost frame: the next frame does not depend on the frames before it.
  void forceKeyframe() { force_idr = true; }

  // bitrate_cap: the rate limit of the video [kbit/s]
  void setMaxKbps(double kbps) {
    max_kbps = std::max(100.0, kbps);
    if (!encoder || !capped) return;
    x264_param_t param;
    x264_encoder_parameters(encoder, &param);
    setRateControl(param);
    x264_encoder_reconfig(encoder, &param);
  }

 private:
  bool open(int w, int h) {
    x264_param_t param;
    if (x264_param_default_preset(&param, "ultrafast", "zerolatency") < 0) return false;
    param.i_threads = 1;
    param.b_sliced_threads = 0;
    param.i_width = w;
    param.i_height = h;
    param.i_csp = X264_CSP_I420;
    param.i_log_level = X264_LOG_ERROR;
    param.b_vfr_input = 1;
    param.i_timebase_num = 1;
    param.i_timebase_den = 1000000000;
    param.i_fps_num = static_cast<uint32_t>(std::lround(hz));
    param.i_fps_den = 1;
    // An IDR frame each second: a lost datagram spoils the images until the next one.
    param.i_keyint_max = static_cast<int>(std::lround(hz));
    param.b_repeat_headers = 1;
    param.b_annexb = 1;
    // The gray values 0-255 pass unchanged (no 16-235 video range).
    param.vui.b_fullrange = 1;
    param.rc.i_rc_method = X264_RC_CRF;
    param.rc.f_rf_constant = static_cast<float>(crf);
    setRateControl(param);
    if (x264_param_apply_profile(&param, "baseline") < 0) return false;
    encoder = x264_encoder_open(&param);
    if (!encoder) return false;
    if (x264_picture_alloc(&picture, X264_CSP_I420, w, h) < 0) {
      x264_encoder_close(encoder);
      encoder = nullptr;
      return false;
    }
    // Gray: no color in the chroma planes
    std::memset(picture.img.plane[1], 128, static_cast<size_t>(picture.img.i_stride[1]) * (h / 2));
    std::memset(picture.img.plane[2], 128, static_cast<size_t>(picture.img.i_stride[2]) * (h / 2));
    width = w;
    height = h;
    force_idr = true;
    return true;
  }

  void close() {
    if (!encoder) return;
    x264_picture_clean(&picture);
    x264_encoder_close(encoder);
    encoder = nullptr;
  }

  // The VBV buffer limits the size of one frame: at most 400 kbit (50 kB), below the UDP limit. crf: a high
  // rate limit, so the buffer is the only limit.
  void setRateControl(x264_param_t& param) const {
    const double kbps = capped ? max_kbps : 400.0 * hz;
    param.rc.i_vbv_max_bitrate = static_cast<int>(kbps);
    param.rc.i_vbv_buffer_size = static_cast<int>(std::min(400.0, kbps / 4.0));
  }

  const double hz;
  const double crf;
  const bool capped;
  double max_kbps = 1000.0;
  x264_t* encoder = nullptr;
  x264_picture_t picture{};
  int width = 0, height = 0;
  bool force_idr = true;
};
#else
class VisPublisher::H264Encoder {};
#endif

VisPublisher::VisPublisher(const VisPublisherOptions& options, const Calibration<double>& calib)
    : options(options) {
  peer.sin_family = AF_INET;
  if (options.vis_msg_port < 1 || options.vis_msg_port > 65535)
    throw std::invalid_argument("vis_msg_port must be 1-65535, not " + std::to_string(options.vis_msg_port));
  peer.sin_port = htons(static_cast<uint16_t>(options.vis_msg_port));
  if (inet_pton(AF_INET, options.vis_msg_ip.c_str(), &peer.sin_addr) != 1)
    throw std::invalid_argument("vis_msg_ip is not an IPv4 address: " + options.vis_msg_ip);
  if (options.vis_msg_encode == "h264") {
#ifdef GRANITE_WITH_X264
    if (options.vis_msg_h264_rate_control != "crf" && options.vis_msg_h264_rate_control != "bitrate_cap")
      throw std::invalid_argument("vis_msg_h264_rate_control must be crf or bitrate_cap, not " +
                                  options.vis_msg_h264_rate_control);
    h264 = std::make_unique<H264Encoder>(options);
#else
    throw std::invalid_argument("vis_msg_encode h264: this build has no x264 (use mjpeg)");
#endif
  } else if (options.vis_msg_encode != "mjpeg") {
    throw std::invalid_argument("vis_msg_encode must be mjpeg or h264, not " + options.vis_msg_encode);
  }
  if (!calib.intrinsics.empty()) {
    camera_model = calib.intrinsics[0].getName();
    const auto params = calib.intrinsics[0].getParam();
    for (int i = 0; i < params.size(); ++i) camera_params.push_back(static_cast<float>(params[i]));
  }
  if (!calib.T_i_c.empty()) T_i_c = calib.T_i_c[0];
  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) throw std::runtime_error(std::string("send_vis_msg: socket failed: ") + std::strerror(errno));
  thread = std::thread([this] { run(); });
  std::cout << "send_vis_msg: UDP to " << options.vis_msg_ip << ':' << options.vis_msg_port << " (image "
            << options.vis_msg_image_hz << " Hz " << options.vis_msg_encode;
  if (h264) {
    std::cout << " " << options.vis_msg_h264_rate_control << " crf " << options.vis_msg_h264_crf;
  } else {
    std::cout << " quality " << options.vis_msg_jpeg_quality;
  }
  std::cout << ", landmarks " << options.vis_msg_points_hz << " Hz, at most " << options.vis_msg_max_kbps
            << " kbit/s)" << std::endl;
}

VisPublisher::~VisPublisher() {
  {
    std::lock_guard<std::mutex> lock(mutex);
    stopping = true;
  }
  wake.notify_all();
  if (thread.joinable()) thread.join();
  if (fd >= 0) close(fd);
}

void VisPublisher::publish(const VioVisualizationData::Ptr& data) {
  if (!data) return;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (pending) ++skipped;
    pending = data;
  }
  wake.notify_one();
}

void VisPublisher::setState(const VioStateData& data) {
  std::lock_guard<std::mutex> lock(mutex);
  state.vel_w_i = data.state.vel_w_i;
  state.bias_gyro = data.state.bias_gyro;
  state.bias_accel = data.state.bias_accel;
  state.healthy = data.healthy;
}

void VisPublisher::run() {
  sched_param idle{};
  idle.sched_priority = 0;
  if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &idle) != 0)
    std::cerr << "send_vis_msg: SCHED_IDLE is not available; the publisher keeps the normal priority" << std::endl;

  RateGate image_gate(options.vis_msg_image_hz), points_gate(options.vis_msg_points_hz);
  // Send debt [byte]: each sent message (with the IP and UDP headers) adds to it, and it goes down at
  // vis_msg_max_kbps to 0. mjpeg: an image message goes only at 0, so the mean rate stays below the limit.
  using Clock = std::chrono::steady_clock;
  const bool limit_rate = options.vis_msg_max_kbps > 0;
  const double bytes_per_second = options.vis_msg_max_kbps * 1000.0 / 8.0;
  double debt = 0;
  Clock::time_point last_debt = Clock::now();
  // h264 bitrate_cap: the bytes of everything but the video in the last second
  double other_bytes = 0;
  Clock::time_point other_start = Clock::now();
#ifdef GRANITE_WITH_X264
  if (h264) h264->setMaxKbps(options.vis_msg_max_kbps * 0.7);
#endif
  uint64_t seq = 0, image_seq = 0;
  while (true) {
    VioVisualizationData::Ptr data;
    State current;
    {
      std::unique_lock<std::mutex> lock(mutex);
      wake.wait(lock, [this] { return stopping || pending; });
      if (stopping) break;
      data.swap(pending);
      current = state;
    }
    const int64_t t_ns = static_cast<int64_t>(data->t_ns);
    const Clock::time_point now = Clock::now();
    if (limit_rate)
      debt = std::min(0.0, debt + bytes_per_second * std::chrono::duration<double>(now - last_debt).count());
    last_debt = now;
#ifdef GRANITE_WITH_X264
    if (h264 && now - other_start >= std::chrono::seconds(1)) {
      const double other_kbps = other_bytes * 8.0 / 1000.0 / std::chrono::duration<double>(now - other_start).count();
      // 5 % below the rest of the limit: the VBV of x264 keeps the mean rate only near its limit.
      if (limit_rate) h264->setMaxKbps(0.95 * (options.vis_msg_max_kbps - other_kbps));
      other_bytes = 0;
      other_start = now;
    }
#endif

    const ManagedImage<PixelType>* image = imageOf(*data);
    const int width = image ? static_cast<int>(image->w) : 0;
    const int height = image ? static_cast<int>(image->h) : 0;
    // h264 encodes every due image: a frame that is not sent spoils the frames after it.
    const bool image_due = image && image_gate.due(t_ns) && (h264 || !limit_rate || debt >= 0);

    Detail detail;
    detail.h264 = static_cast<bool>(h264);
    detail.points = points_gate.due(t_ns) && !data->points.empty();
    std::string encoded;
    int quality = options.vis_msg_jpeg_quality;
    const auto encodeJpeg = [&] {
      encoded.clear();
      const cv::Mat gray(height, width, CV_8UC1, image->ptr, image->pitch);
      std::vector<uchar> buffer;
      if (cv::imencode(".jpg", gray, buffer, {cv::IMWRITE_JPEG_QUALITY, quality}))
        encoded.assign(buffer.begin(), buffer.end());
    };
    if (image_due) {
      image_gate.take(t_ns);
#ifdef GRANITE_WITH_X264
      if (h264) encoded = h264->encode(*image, t_ns, &detail.image_keyframe);
#endif
      if (!h264) encodeJpeg();
      if (!encoded.empty()) {
        detail.image = &encoded;
        detail.keypoints = true;
        detail.image_seq = ++image_seq;
      }
    }

    // Above the UDP limit: mjpeg tries a lower JPEG quality (at least 20); then the message goes without the
    // image, and then without the landmarks.
    std::string message = encode(*data, current, seq, detail, width, height);
    while (message.size() > kMaxDatagram && detail.image && !h264 && quality > 20) {
      quality = std::max(20, quality * 2 / 3);
      encodeJpeg();
      message = encode(*data, current, seq, detail, width, height);
    }
    if (message.size() > kMaxDatagram && detail.image) {
      detail.image = nullptr;
      detail.keypoints = false;
#ifdef GRANITE_WITH_X264
      if (h264) h264->forceKeyframe();
#endif
      message = encode(*data, current, seq, detail, width, height);
    }
    if (message.size() > kMaxDatagram && detail.points) {
      detail.points = false;
      message = encode(*data, current, seq, detail, width, height);
    }
    data.reset();
    ++seq;
    if (message.size() > kMaxDatagram) {
      ++skipped;
      continue;
    }
    // MSG_DONTWAIT: a full socket buffer drops this message, it does not stop the thread.
    const ssize_t written = sendto(fd, message.data(), message.size(), MSG_DONTWAIT,
                                   reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
    if (written == static_cast<ssize_t>(message.size())) {
      debt -= static_cast<double>(message.size() + kIpUdpHeader);
      other_bytes += static_cast<double>(message.size() + kIpUdpHeader - (detail.image ? encoded.size() : 0));
      if (detail.points) points_gate.take(t_ns);
      ++sent;
      if (detail.image) ++images;
    } else {
#ifdef GRANITE_WITH_X264
      if (h264 && detail.image) h264->forceKeyframe();
#endif
      if (send_errors++ == 0)
        std::cerr << "send_vis_msg: sendto " << options.vis_msg_ip << ':' << options.vis_msg_port
                  << " failed: " << std::strerror(errno) << " (later failures are counted, not printed)"
                  << std::endl;
    }
  }
}

std::string VisPublisher::encode(const VioVisualizationData& data, const State& state, uint64_t seq,
                                 const Detail& detail, int width, int height) const {
  ProtoWriter w;
  std::vector<float> values;
  const auto floats = [&](int field, auto&& fill) {
    values.clear();
    fill();
    w.floats(field, values);
  };
  w.uint(kVersion, kVisVersion);
  w.uint(kSeq, seq);
  w.uint(kTimeNs, static_cast<uint64_t>(data.t_ns));
  w.uint(kMapIdx, data.map_idx);
  floats(kPose, [&] {
    if (!data.states.empty()) appendPose(values, data.states.back());
  });
  floats(kVelocity, [&] { appendVector(values, state.vel_w_i); });
  floats(kBiasGyro, [&] { appendVector(values, state.bias_gyro); });
  floats(kBiasAccel, [&] { appendVector(values, state.bias_accel); });
  w.uint(kHealthy, state.healthy);
  w.uint(kKeyframe, data.take_kf);
  floats(kWindowStates, [&] {
    for (const auto& T : data.states) appendPose(values, T);
  });
  floats(kWindowFrames, [&] {
    for (const auto& T : data.frames) appendPose(values, T);
  });
  if (detail.points) {
    // Sorted by id for the delta coding of the ids
    std::vector<std::pair<uint64_t, size_t>> order;
    for (size_t i = 0; i < data.points.size() && i < data.point_ids.size(); ++i)
      if (data.point_ids[i] >= 0) order.emplace_back(static_cast<uint64_t>(data.point_ids[i]), i);
    std::sort(order.begin(), order.end());
    std::vector<uint64_t> ids;
    floats(kPoints, [&] {
      for (const auto& [id, i] : order) {
        appendVector(values, data.points[i]);
        ids.push_back(id);
      }
    });
    w.deltaUints(kPointIds, ids);
  }
  if (detail.keypoints && data.opt_flow_res && !data.opt_flow_res->observations.empty()) {
    // std::map: the ids are in increasing order.
    std::string pixels;
    std::vector<uint64_t> ids;
    for (const auto& kv : data.opt_flow_res->observations[0]) {
      appendUint16(pixels, kv.second.translation().x() * kKeypointScale);
      appendUint16(pixels, kv.second.translation().y() * kKeypointScale);
      ids.push_back(static_cast<uint64_t>(kv.first));
    }
    w.bytes(kKeypoints, pixels);
    w.deltaUints(kKeypointIds, ids);
  }
  if (detail.image) {
    w.bytes(kImage, *detail.image);
    w.uint(kImageCodec, detail.h264 ? kCodecH264 : 0);
    w.uint(kImageKeyframe, detail.image_keyframe);
    w.uint(kImageSeq, detail.image_seq);
    w.bytes(kCameraModel, camera_model);
    w.floats(kCameraParams, camera_params);
    values.clear();
    appendPose(values, T_i_c);
    w.floats(kTic, values);
  }
  w.uint(kImageWidth, static_cast<uint64_t>(width));
  w.uint(kImageHeight, static_cast<uint64_t>(height));
  w.uint(kSkipped, skipped.load());
  return w.take();
}

}  // namespace granite
