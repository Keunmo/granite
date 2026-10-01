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

#include <granite/vi_estimator/mono_vio.h>

namespace granite {
MonoVioEstimator::MonoVioEstimator(const Eigen::Vector3d& gravity,
    const Calibration<double>& calibration, const VioConfig& configuration)
    : gravity(gravity), calibration(calibration), configuration(configuration) {
  if (calibration.intrinsics.size() != 1 || calibration.T_i_c.size() != 1) {
    throw std::invalid_argument("Mono VIO needs exactly one calibrated camera");
  }
}
MonoVioEstimator::~MonoVioEstimator() { join(); }
void MonoVioEstimator::join() { if (processing_thread.joinable()) processing_thread.join(); }
void MonoVioEstimator::quit() {
  VioEstimatorBase::quit();
  if (vision_data_queue) vision_data_queue->abort();
  if (imu_data_queue) imu_data_queue->abort();
}
void MonoVioEstimator::initialize(int64_t t_ns, const Sophus::SE3d& pose,
    const Eigen::Vector3d& velocity, const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  // Automatic alignment does not assume that startup velocity or scale is known.
  UNUSED(t_ns);
  UNUSED(pose);
  UNUSED(velocity);
  initialize(bg, ba);
}
void MonoVioEstimator::initialize(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  if (!vision_data_queue || !imu_data_queue) throw std::invalid_argument("Mono VIO requires image and IMU queues");
  processing_thread = std::thread([this, bg, ba] { run(bg, ba); });
}

void MonoVioEstimator::run(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  VioConfig bootstrap_config = configuration;
  bootstrap_config.vio_enforce_realtime = false;
  bootstrap_config.vio_max_kfs = std::max(bootstrap_config.vio_max_kfs, 8);
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> bootstrap_vision;
  tbb::concurrent_bounded_queue<VioStateData::Ptr> bootstrap_states;
  auto visual = std::make_shared<KeypointVoEstimator>(calibration, bootstrap_config);
  visual->vision_data_queue = &bootstrap_vision;
  visual->out_state_queue = &bootstrap_states;
  visual->initialize(bg, ba);

  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> metric_vision;
  tbb::concurrent_bounded_queue<ImuData::Ptr> metric_imu;
  std::shared_ptr<KeypointVioEstimator> metric;
  Eigen::aligned_deque<ImuData> samples;
  Eigen::aligned_map<int64_t, OpticalFlowResult::Ptr> history;
  ImuData::Ptr pending;
  size_t visual_map = 0;
  uint64_t frames_processed = 0;
  int64_t forwarded_imu_ns = -1;
  bool visual_stopped = false;
  try {
    while (!should_quit) {
      OpticalFlowResult::Ptr frame;
      vision_data_queue->pop(frame);
      if (!frame) break;
      if (configuration.vio_enforce_realtime) {
        OpticalFlowResult::Ptr newer;
        while (vision_data_queue->try_pop(newer)) {
          if (!newer) { frame.reset(); break; }
          frame = newer;
        }
        if (!frame) break;
      }
      bool imu_finished = false;
      while (!pending || pending->t_ns <= frame->t_ns) {
        ImuData::Ptr next;
        imu_data_queue->pop(next);
        if (!next) { imu_finished = true; break; }
        pending = next;
        samples.push_back(*pending);
        if (metric && pending->t_ns > forwarded_imu_ns) {
          metric_imu.push(std::make_shared<ImuData>(*pending));
          forwarded_imu_ns = pending->t_ns;
        }
      }
      if (imu_finished) break;
      ++frames_processed;
      if (metric) {
        metric_vision.push(frame);
        tracking_state = TrackingState::TRACKING;
        last_processed_t_ns = frame->t_ns;
        while (!samples.empty() && samples.front().t_ns < frame->t_ns - 1000000000LL) samples.pop_front();
        continue;
      }
      history[frame->t_ns] = frame;
      bootstrap_vision.push(frame);
      VioStateData::Ptr visual_state;
      bootstrap_states.pop(visual_state);
      if (!visual_state) break;
      last_processed_t_ns = frame->t_ns;
      if (visual_state->map_idx != visual_map) {
        visual_map = visual_state->map_idx;
        history.clear();
        history[frame->t_ns] = frame;
      }
      while (!samples.empty() && samples.front().t_ns < frame->t_ns - 15000000000LL) samples.pop_front();
      while (!history.empty() && history.begin()->first < frame->t_ns - 15000000000LL) history.erase(history.begin());
      if (frames_processed % 5 != 0) continue;
      std::vector<MonoVioFrame> frames;
      for (size_t i = 0; i < visual_state->frames.size(); ++i) {
        const int64_t timestamp = visual_state->frames_t_ns[i];
        if (!samples.empty() && timestamp >= samples.front().t_ns && history.count(timestamp)) {
          frames.push_back({timestamp, visual_state->frames[i]});
        }
      }
      const auto alignment = alignMonoVisualInertial(frames, samples, calibration, bg, ba);
      if (!alignment) continue;

      // Stop the bootstrap worker before reading its optimized map.
      bootstrap_vision.push(nullptr);
      visual->join();
      visual_stopped = true;
      metric = std::make_shared<KeypointVioEstimator>(gravity, calibration, configuration);
      metric->vision_data_queue = &metric_vision;
      metric->imu_data_queue = &metric_imu;
      metric->out_state_queue = out_state_queue;
      metric->out_vis_queue = out_vis_queue;
      metric->out_marg_queue = out_marg_queue;
      metric_imu.push(std::make_shared<ImuData>(*pending));
      forwarded_imu_ns = pending->t_ns;
      metric->initializeFromMonoMap(*visual, *alignment, frames, history, ba);
      initial_pose = metric->getT_w_i_init();
      metric_scale = alignment->scale;
      metric_initialized = true;
      tracking_state = TrackingState::TRACKING;
      std::cout << "MONO_VIO_INITIALIZED scale=" << alignment->scale
                << " gravity=" << alignment->gravity.transpose()
                << " gyro_bias=" << alignment->gyro_bias.transpose()
                << " residual=" << alignment->residual << std::endl;
      visual.reset();
      history.clear();
    }
  } catch (const tbb::user_abort&) {
  }
  if (!visual_stopped) {
    bootstrap_vision.push(nullptr);
    visual->join();
  }
  if (metric) {
    metric_vision.push(nullptr);
    metric_imu.push(nullptr);
    metric->join();
  } else {
    if (out_state_queue) out_state_queue->push(nullptr);
    if (out_vis_queue) out_vis_queue->push(nullptr);
    if (out_marg_queue) out_marg_queue->push(nullptr);
  }
  finished = true;
}
}  // namespace granite
