// SPDX-License-Identifier: MIT
#include <granite/utils/vis_publisher.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace granite {

namespace {

// Field numbers of GraniteVis (doc/granite_vis.proto).
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
  kProjections = 14,
  kKeypoints = 15,
  kImageJpeg = 16,
  kImageWidth = 17,
  kImageHeight = 18,
  kSkipped = 19,
  kProjectionIds = 20,
};

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
  // Packed repeated varints
  void uints(int field, const std::vector<uint64_t>& values) {
    if (values.empty()) return;
    ProtoWriter body;
    for (uint64_t v : values) body.varint(v);
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

const ManagedImage<PixelType>* imageOf(const VioVisualizationData& data) {
  if (!data.opt_flow_res || !data.opt_flow_res->input_images) return nullptr;
  const auto& images = data.opt_flow_res->input_images->img_data;
  return images.empty() || !images[0].img ? nullptr : images[0].img.get();
}

}  // namespace

VisPublisher::VisPublisher(const VisPublisherOptions& options) : options(options) {
  peer.sin_family = AF_INET;
  if (options.vis_msg_port < 1 || options.vis_msg_port > 65535)
    throw std::invalid_argument("vis_msg_port must be 1-65535, not " + std::to_string(options.vis_msg_port));
  peer.sin_port = htons(static_cast<uint16_t>(options.vis_msg_port));
  if (inet_pton(AF_INET, options.vis_msg_ip.c_str(), &peer.sin_addr) != 1)
    throw std::invalid_argument("vis_msg_ip is not an IPv4 address: " + options.vis_msg_ip);
  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) throw std::runtime_error(std::string("send_vis_msg: socket failed: ") + std::strerror(errno));
  thread = std::thread([this] { run(); });
  std::cout << "send_vis_msg: UDP to " << options.vis_msg_ip << ':' << options.vis_msg_port
            << " (image " << options.vis_msg_image_hz << " Hz, JPEG quality " << options.vis_msg_jpeg_quality
            << ", at most " << options.vis_msg_max_kbps << " kbit/s)" << std::endl;
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
  const bool send_images = options.vis_msg_image_hz > 0;
  // The limit uses the image times, which have jitter: with 0.75 of the period, a camera at the
  // vis_msg_image_hz rate sends every frame.
  const int64_t image_period_ns = send_images ? static_cast<int64_t>(0.75e9 / options.vis_msg_image_hz) : 0;
  int64_t last_image_ns = 0;
  bool any_image = false;
  uint64_t seq = 0;
  // Send debt [byte]: each sent message (with the IP and UDP headers) adds to it, and it goes down at
  // vis_msg_max_kbps to 0. An image message goes only at 0, so the mean rate stays below the limit.
  using Clock = std::chrono::steady_clock;
  const bool limit_rate = options.vis_msg_max_kbps > 0;
  const double bytes_per_second = options.vis_msg_max_kbps * 1000.0 / 8.0;
  double budget = 0;
  Clock::time_point last_budget = Clock::now();
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

    // The image and its keypoints, projections and landmarks go with at most vis_msg_image_hz messages; the
    // other messages have the state only.
    const int64_t t_ns = static_cast<int64_t>(data->t_ns);
    const Clock::time_point now = Clock::now();
    if (limit_rate) {
      budget = std::min(0.0, budget + bytes_per_second * std::chrono::duration<double>(now - last_budget).count());
    }
    last_budget = now;
    const bool detail = send_images && (!limit_rate || budget >= 0) &&
                        (!any_image || t_ns - last_image_ns >= image_period_ns || t_ns < last_image_ns);
    std::string jpeg;
    int width = 0, height = 0;
    const ManagedImage<PixelType>* image = imageOf(*data);
    if (image) {
      width = static_cast<int>(image->w);
      height = static_cast<int>(image->h);
    }
    const auto encodeJpeg = [&](int quality) {
      jpeg.clear();
      const cv::Mat gray(height, width, CV_8UC1, image->ptr, image->pitch);
      std::vector<uchar> buffer;
      if (cv::imencode(".jpg", gray, buffer, {cv::IMWRITE_JPEG_QUALITY, quality}))
        jpeg.assign(buffer.begin(), buffer.end());
    };
    int quality = options.vis_msg_jpeg_quality;
    if (detail) {
      if (image) encodeJpeg(quality);
      last_image_ns = t_ns;
      any_image = true;
    }

    // Above the UDP limit, a lower JPEG quality (at least 20) and then no image
    std::string message = encode(*data, current, seq, detail, jpeg, width, height);
    while (message.size() > kMaxDatagram && !jpeg.empty()) {
      if (quality > 20) {
        quality = std::max(20, quality * 2 / 3);
        encodeJpeg(quality);
      } else {
        jpeg.clear();
      }
      message = encode(*data, current, seq, detail, jpeg, width, height);
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
      budget -= static_cast<double>(message.size() + kIpUdpHeader);
      ++sent;
      if (!jpeg.empty()) ++images;
    } else if (send_errors++ == 0) {
      std::cerr << "send_vis_msg: sendto " << options.vis_msg_ip << ':' << options.vis_msg_port
                << " failed: " << std::strerror(errno) << " (later failures are counted, not printed)"
                << std::endl;
    }
  }
}

std::string VisPublisher::encode(const VioVisualizationData& data, const State& state, uint64_t seq, bool detail,
                                 const std::string& jpeg, int width, int height) const {
  ProtoWriter w;
  std::vector<float> values;
  const auto floats = [&](int field, auto&& fill) {
    values.clear();
    fill();
    w.floats(field, values);
  };
  w.uint(kVersion, 1);
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
  if (detail) {
    floats(kPoints, [&] {
      for (const auto& p : data.points) appendVector(values, p);
    });
    std::vector<uint64_t> ids;
    floats(kProjections, [&] {
      if (data.projections.empty()) return;
      for (const auto& proj : data.projections[0]) {
        for (int i = 0; i < 3; ++i) values.push_back(static_cast<float>(proj[i]));
        ids.push_back(static_cast<uint64_t>(proj[3]));
      }
    });
    w.uints(kProjectionIds, ids);
    floats(kKeypoints, [&] {
      if (!data.opt_flow_res || data.opt_flow_res->observations.empty()) return;
      for (const auto& kv : data.opt_flow_res->observations[0]) {
        values.push_back(kv.second.translation().x());
        values.push_back(kv.second.translation().y());
      }
    });
    w.bytes(kImageJpeg, jpeg);
  }
  w.uint(kImageWidth, static_cast<uint64_t>(width));
  w.uint(kImageHeight, static_cast<uint64_t>(height));
  w.uint(kSkipped, skipped.load());
  return w.take();
}

}  // namespace granite
