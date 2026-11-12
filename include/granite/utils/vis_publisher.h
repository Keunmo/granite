// SPDX-License-Identifier: MIT
#pragma once

#include <granite/calibration/calibration.hpp>
#include <granite/vi_estimator/vio_estimator.h>
#include <netinet/in.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace granite {

// send_vis_msg: the GUI elements of each VIO state as one UDP datagram to vis_msg_ip:vis_msg_port (message
// GraniteVis of doc/granite_vis.proto), for a viewer on the ground.
struct VisPublisherOptions {
  bool send_vis_msg = false;
  std::string vis_msg_ip = "192.168.168.102";
  int vis_msg_port = 5100;
  // The camera image with its tracked keypoints goes with at most this many messages per second (0: none). The
  // other messages have the state only (a few hundred bytes).
  double vis_msg_image_hz = 20.0;
  // The landmarks of the window go with at most this many messages per second (0: none). The viewer projects
  // them into the images.
  double vis_msg_points_hz = 2.0;
  // "h264" or "mjpeg" (each image a gray JPEG). A build without OpenH264 sends mjpeg for h264.
  std::string vis_msg_encode = "h264";
  int vis_msg_jpeg_quality = 80;
  // Upper limit of the mean send rate [kbit/s] (0: no limit). The state messages always go. mjpeg: an image goes
  // only when the rate allows it. h264 with bitrate_cap: the video gets this limit less the rate of the other
  // data.
  double vis_msg_max_kbps = 2000.0;
  // h264: "quality" (constant QP vis_msg_h264_qp, no rate limit) or "bitrate_cap" (QP vis_msg_h264_qp or higher,
  // and the rate at most vis_msg_max_kbps)
  std::string vis_msg_h264_rate_control = "bitrate_cap";
  int vis_msg_h264_qp = 24;
};

// Sends on its own thread and keeps only the newest data: publish() and setState() never wait for the network,
// and the memory does not grow when the receiver is slow or absent. A message that a newer state replaces
// before the thread sends it is skipped. The thread has the lowest priority (SCHED_IDLE), so it uses only CPU
// time that the VIO does not use.
class VisPublisher {
 public:
  // Throws std::invalid_argument for a wrong option.
  VisPublisher(const VisPublisherOptions& options, const Calibration<double>& calib);
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
  // The parts of one message beyond the state
  struct Detail {
    const std::string* image = nullptr;  // encoded image (JPEG or one H.264 access unit), or nothing
    bool h264 = false;
    bool image_keyframe = false;
    uint64_t image_seq = 0;  // counter of the encoded images: a gap is a lost image
    bool keypoints = false;
    bool points = false;
  };
  class H264Encoder;

  void run();
  // Sends the message in one datagram, or in fragments when it is large. Returns the number of datagrams, 0 when a
  // send failed.
  size_t sendMessage(const std::string& message, uint64_t seq);
  std::string encode(const VioVisualizationData& data, const State& state, uint64_t seq, const Detail& detail,
                     int width, int height) const;

  const VisPublisherOptions options;
  // Camera 0, for the projection of the landmarks in the viewer
  std::string camera_model;
  std::vector<float> camera_params;
  Sophus::SE3d T_i_c;
  std::unique_ptr<H264Encoder> h264;
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
