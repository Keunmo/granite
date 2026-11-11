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

#ifdef GRANITE_WITH_OPENH264
#include <wels/codec_api.h>
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
  // A fragment of a message above kMaxPacketPayload: only these fields
  kFragmentSeq = 40,
  kFragmentIndex = 41,
  kFragmentCount = 42,
  kFragmentData = 43,
};
constexpr int kVisVersion = 2;
constexpr uint64_t kCodecH264 = 1;
// Keypoint pixels as uint16 in 1/8 pixel
constexpr float kKeypointScale = 8.0f;

// A message above kMaxPacketPayload goes in fragments of kFragmentBytes bytes: with the fragment fields
// (kFragmentHeader bytes at most) and the IP and UDP headers, a datagram stays below the Ethernet MTU of 1500
// bytes, so IP does not fragment it. A message above kMaxMessage does not go.
constexpr size_t kMaxPacketPayload = 1400;
constexpr size_t kFragmentBytes = 1400;
constexpr size_t kFragmentHeader = 24;
constexpr size_t kMaxMessage = 1 << 20;
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

#ifdef GRANITE_WITH_OPENH264
// OpenH264 for the gray camera image: one Annex B access unit for each image, the SPS and PPS before each IDR
// frame. One thread, constrained baseline profile, no B frames, so each output belongs to the input image.
class VisPublisher::H264Encoder {
 public:
  explicit H264Encoder(const VisPublisherOptions& options)
      : hz(std::max(1.0, options.vis_msg_image_hz)),
        qp(std::clamp(options.vis_msg_h264_qp, 0, 51)),
        capped(options.vis_msg_h264_rate_control == "bitrate_cap") {}
  ~H264Encoder() { close(); }

  // Empty when the encoder fails.
  std::string encode(const ManagedImage<PixelType>& image, int64_t t_ns, bool* keyframe) {
    const int w = static_cast<int>(image.w), h = static_cast<int>(image.h);
    if (encoder && (w != width || h != height)) close();
    if (!encoder && !open(w, h)) return {};
    for (int y = 0; y < h; ++y)
      std::memcpy(planes.data() + static_cast<size_t>(y) * w,
                  reinterpret_cast<const uint8_t*>(image.ptr) + static_cast<size_t>(y) * image.pitch, w);
    SSourcePicture picture{};
    picture.iColorFormat = videoFormatI420;
    picture.iPicWidth = w;
    picture.iPicHeight = h;
    picture.iStride[0] = w;
    picture.iStride[1] = picture.iStride[2] = w / 2;
    picture.pData[0] = planes.data();
    picture.pData[1] = planes.data() + static_cast<size_t>(w) * h;
    picture.pData[2] = picture.pData[1] + static_cast<size_t>(w / 2) * (h / 2);
    picture.uiTimeStamp = t_ns / 1000000;  // [ms]
    if (force_idr) encoder->ForceIntraFrame(true);
    force_idr = false;
    SFrameBSInfo info{};
    if (encoder->EncodeFrame(&picture, &info) != cmResultSuccess || info.eFrameType == videoFrameTypeSkip ||
        info.eFrameType == videoFrameTypeInvalid)
      return {};
    std::string unit;
    for (int i = 0; i < info.iLayerNum; ++i) {
      const SLayerBSInfo& layer = info.sLayerInfo[i];
      size_t size = 0;
      for (int n = 0; n < layer.iNalCount; ++n) size += static_cast<size_t>(layer.pNalLengthInByte[n]);
      unit.append(reinterpret_cast<const char*>(layer.pBsBuf), size);
    }
    *keyframe = info.eFrameType == videoFrameTypeIDR;
    return unit;
  }

  // After a lost frame: the next frame does not depend on the frames before it.
  void forceKeyframe() { force_idr = true; }

  // bitrate_cap: the rate limit of the video [kbit/s]
  void setMaxKbps(double kbps) {
    max_kbps = std::max(100.0, kbps);
    if (!encoder || !capped) return;
    SBitrateInfo rate{};
    rate.iLayer = SPATIAL_LAYER_ALL;
    rate.iBitrate = static_cast<int>(max_kbps * 1000.0);
    encoder->SetOption(ENCODER_OPTION_BITRATE, &rate);
    encoder->SetOption(ENCODER_OPTION_MAX_BITRATE, &rate);
  }

 private:
  bool open(int w, int h) {
    if (WelsCreateSVCEncoder(&encoder) != 0 || !encoder) {
      encoder = nullptr;
      return false;
    }
    SEncParamExt param;
    encoder->GetDefaultParams(&param);
    const int bps = static_cast<int>((capped ? max_kbps : 20000.0) * 1000.0);
    param.iUsageType = CAMERA_VIDEO_REAL_TIME;
    param.iPicWidth = w;
    param.iPicHeight = h;
    param.fMaxFrameRate = static_cast<float>(hz);
    param.iSpatialLayerNum = 1;
    param.iTemporalLayerNum = 1;
    param.iMultipleThreadIdc = 1;
    param.iEntropyCodingModeFlag = 0;  // CAVLC: constrained baseline
    param.iComplexityMode = LOW_COMPLEXITY;
    // An IDR frame each second: a lost datagram spoils the images until the next one.
    param.uiIntraPeriod = static_cast<unsigned int>(std::lround(hz));
    param.eSpsPpsIdStrategy = CONSTANT_ID;
    param.bEnableLongTermReference = false;
    param.bEnableSceneChangeDetect = false;
    param.bEnableBackgroundDetection = false;
    // quality: a constant QP. bitrate_cap: the rate control keeps the mean rate near the limit and the QP at
    // vis_msg_h264_qp or higher, so an easy image uses less than the limit. No frame skip: every image goes, and a
    // hard scene can go above the limit for a short time.
    param.iRCMode = capped ? RC_BITRATE_MODE : RC_OFF_MODE;
    param.bEnableFrameSkip = false;
    param.iTargetBitrate = param.iMaxBitrate = bps;
    param.iMinQp = qp;
    param.iMaxQp = capped ? 51 : qp;
    SSpatialLayerConfig& layer = param.sSpatialLayers[0];
    layer.iVideoWidth = w;
    layer.iVideoHeight = h;
    layer.fFrameRate = static_cast<float>(hz);
    layer.iSpatialBitrate = layer.iMaxSpatialBitrate = bps;
    layer.uiProfileIdc = PRO_BASELINE;
    layer.iDLayerQp = qp;
    layer.sSliceArgument.uiSliceMode = SM_SINGLE_SLICE;
    // The gray values 0-255 pass unchanged (no 16-235 video range).
    layer.bVideoSignalTypePresent = true;
    layer.bFullRange = true;
    int log_level = WELS_LOG_ERROR;
    encoder->SetOption(ENCODER_OPTION_TRACE_LEVEL, &log_level);
    if (encoder->InitializeExt(&param) != cmResultSuccess) {
      WelsDestroySVCEncoder(encoder);
      encoder = nullptr;
      return false;
    }
    int format = videoFormatI420;
    encoder->SetOption(ENCODER_OPTION_DATAFORMAT, &format);
    // Gray: no color in the chroma planes
    planes.assign(static_cast<size_t>(w) * h + 2 * static_cast<size_t>(w / 2) * (h / 2), 128);
    width = w;
    height = h;
    force_idr = true;
    return true;
  }

  void close() {
    if (!encoder) return;
    encoder->Uninitialize();
    WelsDestroySVCEncoder(encoder);
    encoder = nullptr;
  }

  const double hz;
  const int qp;
  const bool capped;
  double max_kbps = 1000.0;
  ISVCEncoder* encoder = nullptr;
  std::vector<uint8_t> planes;  // I420: Y, then U and V
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
#ifdef GRANITE_WITH_OPENH264
    if (options.vis_msg_h264_rate_control != "quality" && options.vis_msg_h264_rate_control != "bitrate_cap")
      throw std::invalid_argument("vis_msg_h264_rate_control must be quality or bitrate_cap, not " +
                                  options.vis_msg_h264_rate_control);
    h264 = std::make_unique<H264Encoder>(options);
#else
    // The visualization must not stop the VIO.
    std::cerr << "send_vis_msg: vis_msg_encode h264, but this build has no OpenH264; the images go as mjpeg"
              << std::endl;
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
            << options.vis_msg_image_hz << " Hz " << (h264 ? "h264" : "mjpeg");
  if (h264) {
    std::cout << " " << options.vis_msg_h264_rate_control << " qp " << options.vis_msg_h264_qp;
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
#ifdef GRANITE_WITH_OPENH264
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
#ifdef GRANITE_WITH_OPENH264
    if (h264 && now - other_start >= std::chrono::seconds(1)) {
      const double other_kbps = other_bytes * 8.0 / 1000.0 / std::chrono::duration<double>(now - other_start).count();
      // 5 % below the rest of the limit: the rate control keeps the mean rate only near its target.
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
    const auto encodeJpeg = [&] {
      const cv::Mat gray(height, width, CV_8UC1, image->ptr, image->pitch);
      std::vector<uchar> buffer;
      if (cv::imencode(".jpg", gray, buffer, {cv::IMWRITE_JPEG_QUALITY, options.vis_msg_jpeg_quality}))
        encoded.assign(buffer.begin(), buffer.end());
    };
    if (image_due) {
      image_gate.take(t_ns);
#ifdef GRANITE_WITH_OPENH264
      if (h264) encoded = h264->encode(*image, t_ns, &detail.image_keyframe);
#endif
      if (!h264) encodeJpeg();
      if (!encoded.empty()) {
        detail.image = &encoded;
        detail.keypoints = true;
        detail.image_seq = ++image_seq;
      }
    }

    std::string message = encode(*data, current, seq, detail, width, height);
    data.reset();
    ++seq;
    if (message.size() > kMaxMessage) {
      ++skipped;
#ifdef GRANITE_WITH_OPENH264
      if (h264 && detail.image) h264->forceKeyframe();
#endif
      continue;
    }
    const size_t packets = sendMessage(message, seq - 1);
    const size_t wire = message.size() + packets * (kIpUdpHeader + (packets > 1 ? kFragmentHeader : 0));
    if (packets > 0) {
      debt -= static_cast<double>(wire);
      other_bytes += static_cast<double>(wire - (detail.image ? encoded.size() : 0));
      if (detail.points) points_gate.take(t_ns);
      ++sent;
      if (detail.image) ++images;
    } else {
#ifdef GRANITE_WITH_OPENH264
      if (h264 && detail.image) h264->forceKeyframe();
#endif
      if (send_errors++ == 0)
        std::cerr << "send_vis_msg: sendto " << options.vis_msg_ip << ':' << options.vis_msg_port
                  << " failed: " << std::strerror(errno) << " (later failures are counted, not printed)"
                  << std::endl;
    }
  }
}

size_t VisPublisher::sendMessage(const std::string& message, uint64_t seq) {
  // MSG_DONTWAIT: a full socket buffer drops the message, it does not stop the thread.
  const auto send = [&](const std::string& packet) {
    return sendto(fd, packet.data(), packet.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&peer),
                  sizeof(peer)) == static_cast<ssize_t>(packet.size());
  };
  if (message.size() <= kMaxPacketPayload) return send(message) ? 1 : 0;
  // Fragments of at most kFragmentBytes bytes, each a GraniteVis with the fragment fields only. One UDP datagram
  // stays below the Ethernet MTU, so IP does not fragment it.
  const size_t count = (message.size() + kFragmentBytes - 1) / kFragmentBytes;
  for (size_t i = 0; i < count; ++i) {
    ProtoWriter w;
    w.uint(kVersion, kVisVersion);
    w.uint(kFragmentSeq, seq + 1);
    w.uint(kFragmentIndex, i);
    w.uint(kFragmentCount, count);
    w.bytes(kFragmentData, message.substr(i * kFragmentBytes, kFragmentBytes));
    if (!send(w.take())) return 0;
  }
  return count;
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
