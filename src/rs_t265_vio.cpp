/**
MIT License

This file is part of the Granite project which is based on Basalt.
https://github.com/DLR-RM/granite

Copyright (c) Martin Wudenka, Deutsches Zentrum für Luft- und Raumfahrt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
Original license of Basalt:
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <thread>
#include <CLI/CLI.hpp>
#include <tbb/global_control.h>
#include <granite/device/rs_t265.h>
#include <opencv2/core.hpp>
#include <granite/serialization/headers_serialization.h>
#include <granite/vi_estimator/vio_estimator.h>
#include <granite/vi_estimator/mono_vio.h>
#include <cereal/archives/json.hpp>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
}

int runT265(int argc, char** argv, int mono_default, bool use_imu_default) {
  int mono_camera = mono_default;
  bool use_imu = use_imu_default;
  bool resize_half = false, capture_only = false;
  int num_threads = 3, skip_frames = 1;
  double duration = 0, exposure = 0;
  std::string config_path, trajectory_path, calibration_path;
  CLI::App app{"Granite T265 live stereo/mono visual-inertial odometry (headless)"};
  app.add_flag("--resize-half", resize_half, "Resize 848x800 to 424x400 and adjust camera intrinsics");
  app.add_option("--use-imu", use_imu, "1: VIO, 0: visual odometry");
  app.add_option("--mono-camera", mono_camera, "0: stereo, 1: left fisheye, 2: right fisheye");
  app.add_option("--num-threads", num_threads, "TBB worker limit (default: 3)");
  app.add_option("--skip-frames", skip_frames, "Process every Nth image; keep all IMU samples");
  app.add_option("--duration", duration, "Stop after this many seconds (0: until SIGINT)");
  app.add_option("--exposure", exposure, "Manual exposure in ms (0: auto)");
  app.add_option("--config-path", config_path, "Granite VioConfig JSON");
  app.add_option("--trajectory", trajectory_path, "Output timestamp, pose and velocity CSV");
  app.add_option("--save-calib", calibration_path, "Export the selected/scaled calibration JSON");
  app.add_flag("--capture-only", capture_only, "Validate image and IMU delivery without running VIO");
  CLI11_PARSE(app, argc, argv);
  if (mono_camera < 0 || mono_camera > 2 || num_threads < 1 || skip_frames < 1 ||
      !std::isfinite(duration) || duration < 0 || !std::isfinite(exposure) || exposure < 0) {
    std::cerr << "Invalid camera, thread count, frame stride, duration or exposure" << std::endl;
    return 2;
  }
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  try {
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, num_threads);
    cv::setNumThreads(1);
    granite::VioConfig config;
    config.vio_enforce_realtime = true;
    config.vio_max_kfs = 5;
    config.vio_max_iterations = 4;
    config.vio_use_lm = true;
    if (!config_path.empty()) config.load(config_path);
    std::ofstream trajectory;
    if (!trajectory_path.empty()) {
      trajectory.open(trajectory_path);
      if (!trajectory) throw std::runtime_error("Cannot open trajectory output");
      trajectory << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw,vx,vy,vz\n" << std::setprecision(16);
    }

    granite::RsT265Device device(exposure > 0, skip_frames, 90, exposure,
                                mono_camera, resize_half);
    device.start();
    const auto calib = device.exportCalibration();
    if (!calibration_path.empty()) {
      std::ofstream output(calibration_path);
      if (!output) throw std::runtime_error("Cannot open calibration output");
      cereal::JSONOutputArchive archive(output);
      archive(*calib);
    }
    std::cout << "mode=" << (mono_camera ? "mono" : "stereo")
              << " estimator=" << (use_imu ? "VIO" : "VO")
              << " fisheye=" << mono_camera << " cameras=" << calib->intrinsics.size()
              << " resolution=" << calib->resolution[0].transpose()
              << " imu_rate=" << calib->imu_update_rate << " threads=" << num_threads << std::endl;
    for (size_t i = 0; i < calib->intrinsics.size(); ++i) {
      std::cout << "camera[" << i << "] intrinsics=" << calib->intrinsics[i].getParam().transpose() << std::endl;
    }

    tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu_queue;
    tbb::concurrent_bounded_queue<granite::OpticalFlowInput::Ptr> capture_queue;
    tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> state_queue;
    capture_queue.set_capacity(2);
    granite::OpticalFlowBase::Ptr flow;
    granite::VioEstimatorBase::Ptr vio;
    if (!capture_only) {
      flow = granite::OpticalFlowFactory::getOpticalFlow(config, *calib);
      flow->input_queue.set_capacity(2);
      device.image_data_queue = &flow->input_queue;
    } else {
      device.image_data_queue = &capture_queue;
    }
    device.imu_data_queue = &imu_queue;
    device.enableCapture();
    const auto start = std::chrono::steady_clock::now();
    // The estimator expects its first IMU measurement to be present.
    if (!capture_only) {
      while (imu_queue.empty() && !device.failed && !interrupted &&
             std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (imu_queue.empty()) {
        device.stop(false);
        flow->quit(true);
        flow->join();
        throw std::runtime_error("T265 did not deliver initial IMU data");
      }
      vio = granite::VioEstimatorFactory::getVioEstimator(config, *calib, granite::constants::g, use_imu);
      vio->vision_data_queue = &flow->output_queue;
      vio->imu_data_queue = &imu_queue;
      vio->out_state_queue = &state_queue;
      vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    uint64_t states = 0, previous_states = 0, previous_images = 0;
    int64_t last_state_ns = 0;
    double max_lag_ms = 0;
    bool invalid_state = false, stream_stalled = false;
    auto last_image_time = start;
    uint64_t watchdog_images = 0;
    const auto mono_vio = std::dynamic_pointer_cast<granite::MonoVioEstimator>(vio);
    size_t landmarks = 0, finite_landmarks = 0;
    auto report_time = start;
    auto consume = [&] {
      if (capture_only) {
        granite::OpticalFlowInput::Ptr image;
        while (capture_queue.try_pop(image)) {
          if (!image) continue;
          for (const auto& data : image->img_data) {
            if (!data.img || data.img->w != static_cast<size_t>(calib->resolution[0][0]) ||
                data.img->h != static_cast<size_t>(calib->resolution[0][1])) invalid_state = true;
          }
        }
        granite::ImuData::Ptr imu;
        while (imu_queue.try_pop(imu)) {
          if (imu && (!imu->accel.allFinite() || !imu->gyro.allFinite())) invalid_state = true;
        }
      } else {
        if (!use_imu) {
          // VO has no IMU consumer; do not accumulate an unused stream.
          granite::ImuData::Ptr sample;
          while (imu_queue.try_pop(sample)) {}
        }
        granite::VioStateData::Ptr result;
        while (state_queue.try_pop(result)) {
          if (!result) continue;
          const auto& state = result->state;
          if (!state.T_w_i.matrix().allFinite() || !state.vel_w_i.allFinite() ||
              state.t_ns <= last_state_ns) invalid_state = true;
          last_state_ns = state.t_ns;
          ++states;
          landmarks = result->num_landmarks;
          finite_landmarks = result->num_finite_landmarks;
          max_lag_ms = std::max(max_lag_ms, (device.last_image_t_ns.load() - last_state_ns) * 1e-6);
          if (trajectory) {
            const auto& p = state.T_w_i.translation();
            const auto q = state.T_w_i.unit_quaternion();
            trajectory << state.t_ns << ',' << p.x() << ',' << p.y() << ',' << p.z()
                       << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
                       << ',' << state.vel_w_i.x() << ',' << state.vel_w_i.y() << ',' << state.vel_w_i.z() << '\n';
          }
        }
      }
    };
    while (!interrupted && !device.failed && !invalid_state) {
      consume();
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      const double interval = std::chrono::duration<double>(now - report_time).count();
      if (interval >= 1) {
        const auto images = device.image_frames.load();
        std::cout << std::fixed << std::setprecision(2)
                  << "t=" << elapsed << " input_fps=" << (images - previous_images) / interval
                  << " state_fps=" << (states - previous_states) / interval
                  << " lag_ms=" << (last_state_ns ? (device.last_image_t_ns.load() - last_state_ns) * 1e-6 : 0)
                  << " image_drops=" << device.dropped_frames.load()
                  << " imu_queue=" << imu_queue.size()
                  << " raw_images=" << device.raw_image_callbacks.load()
                  << " pair_rejects=" << device.unmatched_pairs.load()
                  << " time_rejects=" << device.nonmonotonic_images.load()
                  << " landmarks=" << landmarks << " finite=" << finite_landmarks
                  << " metric_initialized=" << (mono_vio ? mono_vio->metricInitialized() : (!capture_only && use_imu)) << std::endl;
        previous_images = images;
        previous_states = states;
        report_time = now;
      }
      if (duration > 0 && elapsed >= duration) break;
      if (device.image_frames.load() != watchdog_images) {
        watchdog_images = device.image_frames.load();
        last_image_time = now;
      } else if (elapsed > 5 && now - last_image_time > std::chrono::seconds(3)) {
        std::cerr << "T265 image stream stalled" << std::endl;
        stream_stalled = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Join the SDK callbacks before sending end-of-stream markers.
    bool stop_failed = false;
    try { device.stop(false); }
    catch (const std::exception& error) {
      stop_failed = true;
      std::cerr << "T265 stop failed: " << error.what() << std::endl;
    }
    // An IMU consumer may be waiting while the flow output is full. Cancel
    // both flow queues and release the IMU wait without blocking on image EOF.
    if (vio) vio->quit();
    imu_queue.push(nullptr);
    if (flow) {
      flow->quit(true);
      // abort releases pending waits; sentinels also cover a pop that starts
      // just after the abort while its worker is leaving the processing loop.
      flow->input_queue.try_push(nullptr);
      flow->output_queue.try_push(nullptr);
    }
    if (flow) flow->join();
    if (vio) vio->join();
    consume();
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "SUMMARY seconds=" << elapsed << " images=" << device.image_frames.load()
              << " queued=" << device.queued_frames.load() << " dropped=" << device.dropped_frames.load()
              << " imu=" << device.imu_samples.load() << " states=" << states
              << " avg_state_fps=" << states / elapsed << " max_lag_ms=" << max_lag_ms
              << " metric_initialized=" << (mono_vio ? mono_vio->metricInitialized() : (!capture_only && use_imu))
              << " valid=" << !invalid_state << std::endl;
    if (device.failed) std::cerr << device.errorMessage() << std::endl;
    return device.failed || invalid_state || stream_stalled || stop_failed || device.image_frames == 0 ||
                   device.imu_samples == 0 || (!capture_only && !mono_vio && states == 0) ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "T265 VIO: " << error.what() << std::endl;
    return 1;
  }
}
