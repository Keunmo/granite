// SPDX-License-Identifier: MIT
#pragma once

#include <granite/vi_estimator/vio_estimator.h>
#include <netinet/in.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace granite {

// send_vis_msg: the GUI elements of each VIO state as one UDP datagram to vis_msg_ip:vis_msg_port (message
// GraniteVis of doc/granite_vis.proto), for a viewer on the ground.
struct VisPublisherOptions {
  bool send_vis_msg = false;
  std::string vis_msg_ip = "10.20.10.42";
  int vis_msg_port = 5100;
  // The camera image (gray JPEG) with its keypoints, landmark projections and the landmarks of the window goes
  // with at most this many messages per second (0: none). The other messages have the state only (a few
  // hundred bytes).
  double vis_msg_image_hz = 20.0;
  int vis_msg_jpeg_quality = 80;
  // Upper limit of the mean send rate [kbit/s]. An image message goes only when the rate allows it; the state
  // messages always go (0: no limit).
  double vis_msg_max_kbps = 2000.0;
};

// Sends on its own thread and keeps only the newest data: publish() and setState() never wait for the network,
// and the memory does not grow when the receiver is slow or absent. A message that a newer state replaces
// before the thread sends it is skipped.
class VisPublisher {
 public:
  explicit VisPublisher(const VisPublisherOptions& options);
  ~VisPublisher();
  VisPublisher(const VisPublisher&) = delete;
  VisPublisher& operator=(const VisPublisher&) = delete;

  // The caller must not change *data after the call.
  void publish(const VioVisualizationData::Ptr& data);
  // Velocity, biases and health of the newest state (not in VioVisualizationData).
  void setState(const VioStateData& state);

  std::atomic<uint64_t> sent{0}, skipped{0}, send_errors{0}, images{0};

 private:
  struct State {
    Eigen::Vector3d vel_w_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
    bool healthy = true;
  };

  void run();
  std::string encode(const VioVisualizationData& data, const State& state, uint64_t seq, bool detail,
                     const std::string& jpeg, int width, int height) const;

  const VisPublisherOptions options;
  int fd = -1;
  sockaddr_in peer{};
  std::mutex mutex;
  std::condition_variable wake;
  VioVisualizationData::Ptr pending;
  State state;
  bool stopping = false;
  std::thread thread;
};

}  // namespace granite
