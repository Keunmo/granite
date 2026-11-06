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
#include <granite/vi_estimator/mono_inertial_initializer.h>
#include <sstream>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <tuple>

namespace granite {
MonoVioEstimator::MonoVioEstimator(const Eigen::Vector3d& gravity,
    const Calibration<double>& calibration, const VioConfig& configuration)
    : gravity(gravity), calibration(calibration), configuration(configuration) {
  if (calibration.intrinsics.size() != 1 || calibration.T_i_c.size() != 1) {
    throw std::invalid_argument("Mono VIO needs exactly one calibrated camera");
  }
  if (configuration.vio_mono_init_method != "vo_alignment" &&
      configuration.vio_mono_init_method != "inertial") {
    throw std::invalid_argument("vio_mono_init_method must be vo_alignment or inertial");
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

namespace {
// One metric estimator and the thread that publishes its states in the output
// world: T_out_w maps its world to the world of the first estimator (identity
// for the first one), so a re-initialization does not move the output frame.
struct MetricRun {
  std::shared_ptr<KeypointVioEstimator> estimator;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<ImuData::Ptr> imu;
  tbb::concurrent_bounded_queue<VioStateData::Ptr> states;
  std::thread publisher;
  Sophus::SE3d T_out_w;
  int64_t init_t_ns = 0;
  // Publish no state after this time (set when a successor takes over).
  std::atomic<int64_t> publish_until_ns{std::numeric_limits<int64_t>::max()};
  std::atomic<int64_t> last_seen_ns{-1};
  std::atomic<bool> finished{false};
  std::atomic<bool> unhealthy{false};
  std::mutex mutex;
  Eigen::aligned_map<int64_t, Sophus::SE3d> published;  // recent output poses
};

// Rotation about the gravity axis (world z) closest to R.
Sophus::SO3d yawOnly(const Eigen::Matrix3d& R) {
  const double yaw = std::atan2(R(1, 0) - R(0, 1), R(0, 0) + R(1, 1));
  return Sophus::SO3d::rotZ(yaw);
}
}  // namespace

void MonoVioEstimator::run(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  if (configuration.vio_mono_init_method == "inertial") {
    runInertial(bg, ba);
    return;
  }
  VioConfig bootstrap_config = configuration;
  bootstrap_config.vio_enforce_realtime = false;
  bootstrap_config.vio_max_kfs = std::max(bootstrap_config.vio_max_kfs, 8);
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> bootstrap_vision;
  tbb::concurrent_bounded_queue<VioStateData::Ptr> bootstrap_states;
  std::shared_ptr<KeypointVoEstimator> visual;
  auto start_bootstrap = [&] {
    bootstrap_vision.clear();
    bootstrap_states.clear();
    visual = std::make_shared<KeypointVoEstimator>(calibration, bootstrap_config);
    visual->vision_data_queue = &bootstrap_vision;
    visual->out_state_queue = &bootstrap_states;
    // A shadow bootstrap must not close the visualization stream.
    visual->out_vis_queue = metric_initialized ? nullptr : out_vis_queue;
    visual->initialize(bg, ba);
  };
  start_bootstrap();

  std::unique_ptr<MetricRun> metric;
  Eigen::aligned_deque<ImuData> samples;
  Eigen::aligned_map<int64_t, OpticalFlowResult::Ptr> history;
  ImuData::Ptr pending;
  size_t visual_map = 0;
  uint64_t frames_processed = 0;
  int64_t forwarded_imu_ns = -1;
  int reinitializations = 0;
  // Successive successful alignments of the current bootstrap map that agree.
  std::vector<MonoVioAlignment> agreeing;

  const double settle_s = configuration.vio_mono_health_settle_s;
  const double window_s = configuration.vio_mono_health_window_s;
  const double rot_limit = configuration.vio_mono_health_rot_deg * M_PI / 180.0;
  const double pos_limit = configuration.vio_mono_health_pos_m;
  const double max_ba = configuration.vio_mono_health_max_accel_bias;
  const double max_bg = configuration.vio_mono_health_max_gyro_bias;
  auto publish = [this, settle_s, window_s, rot_limit, pos_limit, max_ba, max_bg](MetricRun* run) {
    std::deque<std::tuple<int64_t, double, double>> recent;
    VioStateData::Ptr data;
    while (true) {
      run->states.pop(data);
      if (!data) break;
      const int64_t t = data->state.t_ns;
      run->last_seen_ns = t;
      // Health: the optimization of every frame moved the previously
      // published state by a large rotation but by almost no translation in
      // the metric states that never recovered (bad initialization before a
      // near-stop); healthy states move both by a little.
      if (std::isfinite(data->prev_correction_rot) && std::isfinite(data->prev_correction_pos)) {
        recent.emplace_back(t, data->prev_correction_rot, data->prev_correction_pos);
        while (!recent.empty() && std::get<0>(recent.front()) < t - int64_t(window_s * 1e9)) recent.pop_front();
        if ((t - run->init_t_ns) * 1e-9 >= settle_s + window_s && recent.size() >= 10) {
          double rot2 = 0, pos2 = 0;
          for (const auto& r : recent) {
            rot2 += std::get<1>(r) * std::get<1>(r);
            pos2 += std::get<2>(r) * std::get<2>(r);
          }
          const double rot = std::sqrt(rot2 / recent.size()), pos = std::sqrt(pos2 / recent.size());
          if (rot > rot_limit && pos < pos_limit && !run->unhealthy) {
            run->unhealthy = true;
            std::cout << std::setprecision(6) << "MONO_VIO_UNHEALTHY t_ns=" << t << " correction_rot_deg="
                      << rot * 180 / M_PI << " correction_pos_m=" << pos << std::endl;
          }
        }
      }
      // Plausibility: the bias estimates of a diverged estimator grow far
      // beyond any IMU bias (a Cam0 run reached |ba| = 6 m/s^2, |bg| = 0.13
      // rad/s at a true 0.45 m/s^2, 0.02 rad/s).
      if (!run->unhealthy && (t - run->init_t_ns) * 1e-9 >= settle_s &&
          (data->state.bias_accel.norm() > max_ba || data->state.bias_gyro.norm() > max_bg)) {
        run->unhealthy = true;
        std::cout << std::setprecision(6) << "MONO_VIO_UNHEALTHY t_ns=" << t
                  << " bias_accel=" << data->state.bias_accel.norm()
                  << " bias_gyro=" << data->state.bias_gyro.norm() << std::endl;
      }
      data->healthy = !run->unhealthy;
      if (t > run->publish_until_ns) continue;
      if (!run->T_out_w.matrix().isIdentity(0)) {
        data->state.T_w_i = run->T_out_w * data->state.T_w_i;
        data->state.vel_w_i = run->T_out_w.so3() * data->state.vel_w_i;
      }
      {
        std::lock_guard<std::mutex> lock(run->mutex);
        run->published[t] = data->state.T_w_i;
        while (run->published.size() > 64) run->published.erase(run->published.begin());
      }
      if (out_state_queue) out_state_queue->push(data);
    }
    run->finished = true;
  };
  auto stop_metric = [](MetricRun* run) {
    run->vision.push(nullptr);
    run->imu.push(nullptr);
    run->estimator->join();
    if (run->publisher.joinable()) run->publisher.join();
  };

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
          metric->imu.push(std::make_shared<ImuData>(*pending));
          forwarded_imu_ns = pending->t_ns;
        }
      }
      if (imu_finished) break;
      ++frames_processed;
      if (metric) {
        metric->vision.push(frame);
        tracking_state = metric->estimator->tracking_state.load();
        last_processed_t_ns = frame->t_ns;
        if (!visual && metric->unhealthy && reinitializations < configuration.vio_mono_reinit_max) {
          // Keep publishing the current estimator and bootstrap a successor.
          ++reinitializations;
          std::cout << "MONO_VIO_REINIT_START t_ns=" << frame->t_ns << std::endl;
          start_bootstrap();
          visual_map = 0;
          history.clear();
          agreeing.clear();
        }
        if (!visual) {
          while (!samples.empty() && samples.front().t_ns < frame->t_ns - 1000000000LL) samples.pop_front();
          continue;
        }
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
        agreeing.clear();
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
      if (configuration.vio_mono_init_consistent_alignments > 1) {
        // The noise-only scale std of one alignment (about 0.3 % on the
        // Unreal city sequence) does not cover bootstrap pose errors or the
        // unknown accelerometer bias: the scale of the next window, 0.33 s
        // later, differed by up to 8 %. Most first alignments 2 s after the
        // motion onset gave a metric state that never recovered. Require that
        // the estimate stays the same while the window slides.
        const double cos_tolerance = std::cos(
            configuration.vio_mono_init_consistency_gravity_deg * M_PI / 180.0);
        bool agrees = true;
        for (const auto& previous : agreeing) {
          const double ratio = alignment->scale / previous.scale;
          if (std::abs(std::log(ratio)) >
                  std::log1p(configuration.vio_mono_init_consistency_scale) ||
              alignment->gravity.normalized().dot(previous.gravity.normalized()) <
                  cos_tolerance) {
            agrees = false;
            break;
          }
        }
        if (!agrees) agreeing.clear();
        agreeing.push_back(*alignment);
        std::cout << std::setprecision(6) << "MONO_VIO_ALIGNMENT t_ns=" << frame->t_ns
                  << " scale=" << alignment->scale << " agreeing=" << agreeing.size()
                  << std::endl;
        if (static_cast<int>(agreeing.size()) <
            configuration.vio_mono_init_consistent_alignments)
          continue;
      }

      // Stop the bootstrap worker before reading its optimized map.
      bootstrap_vision.push(nullptr);
      visual->join();
      auto next = std::make_unique<MetricRun>();
      next->estimator = std::make_shared<KeypointVioEstimator>(gravity, calibration, configuration);
      next->estimator->vision_data_queue = &next->vision;
      next->estimator->imu_data_queue = &next->imu;
      next->estimator->out_state_queue = &next->states;
      next->imu.push(std::make_shared<ImuData>(*pending));
      next->estimator->initializeFromMonoMap(*visual, *alignment, frames, history, ba);
      next->init_t_ns = frames.back().t_ns;
      if (metric) {
        // Hand over at the newest frame. Wait until the current estimator has
        // published it, then express the successor in the output world: keep
        // the published position and heading (gravity fixes roll and pitch).
        metric->publish_until_ns = next->init_t_ns;
        const auto t0 = std::chrono::steady_clock::now();
        while (metric->last_seen_ns < next->init_t_ns && !metric->finished &&
               std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10))
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        std::optional<Sophus::SE3d> anchor;
        {
          std::lock_guard<std::mutex> lock(metric->mutex);
          auto found = metric->published.upper_bound(next->init_t_ns);
          if (found != metric->published.begin()) anchor = std::prev(found)->second;
        }
        const Sophus::SE3d T_w_i = next->estimator->getT_w_i_init();
        if (anchor) {
          const Sophus::SO3d R = yawOnly((anchor->so3() * T_w_i.so3().inverse()).matrix());
          next->T_out_w = Sophus::SE3d(R, anchor->translation() - R * T_w_i.translation());
        }
        stop_metric(metric.get());
        std::cout << std::setprecision(6) << "MONO_VIO_REINITIALIZED t_ns=" << next->init_t_ns
                  << " scale=" << alignment->scale << std::endl;
      } else {
        next->estimator->out_vis_queue = out_vis_queue;
        next->estimator->out_marg_queue = out_marg_queue;
        initial_pose = next->estimator->getT_w_i_init();
        std::ostringstream initialization;
        initialization << std::setprecision(6)
                       << "MONO_VIO_INITIALIZED scale=" << alignment->scale
                       << " gravity=" << alignment->gravity.transpose()
                       << " gyro_bias=" << alignment->gyro_bias.transpose()
                       << " residual=" << alignment->residual
                       << " relative_scale_std=" << alignment->relative_scale_std;
        std::cout << initialization.str() << std::endl;
      }
      forwarded_imu_ns = pending->t_ns;
      metric = std::move(next);
      MetricRun* run = metric.get();
      metric->publisher = std::thread([publish, run] { publish(run); });
      metric_scale = alignment->scale;
      metric_initialized = true;
      tracking_state = TrackingState::TRACKING;
      visual.reset();
      history.clear();
      agreeing.clear();
    }
  } catch (const tbb::user_abort&) {
  }
  if (visual) {
    bootstrap_vision.push(nullptr);
    visual->join();
  }
  if (metric) stop_metric(metric.get());
  if (out_state_queue) out_state_queue->push(nullptr);
  if (out_vis_queue && !metric) out_vis_queue->push(nullptr);
  if (out_marg_queue && !metric) out_marg_queue->push(nullptr);
  finished = true;
}

void MonoVioEstimator::runInertial(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  MonoInertialInitOptions options = MonoInertialInitOptions::fromConfig(configuration);
  options.gyro_bias_prior = bg;
  options.accel_bias_prior = ba;
  MonoInertialInitializer initializer(options, calibration, gravity);
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> metric_vision;
  tbb::concurrent_bounded_queue<ImuData::Ptr> metric_imu;
  std::shared_ptr<KeypointVioEstimator> metric;
  Eigen::aligned_deque<ImuData> samples;  // Raw samples (the metric VIO calibrates them itself)
  Eigen::aligned_map<int64_t, OpticalFlowResult::Ptr> history;
  ImuData::Ptr pending;
  std::string last_reason;
  const int64_t keep_ns =
      static_cast<int64_t>((std::max(options.window_time, options.moving_window_time) + 1.0) * 1e9);
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
        if (metric) {
          metric_imu.push(std::make_shared<ImuData>(*pending));
        } else {
          samples.push_back(*pending);
          ImuData calibrated = *pending;
          calibrated.accel = calibration.calib_accel_bias.getCalibrated(calibrated.accel);
          calibrated.gyro = calibration.calib_gyro_bias.getCalibrated(calibrated.gyro);
          initializer.addImu(calibrated);
        }
      }
      if (imu_finished) break;
      last_processed_t_ns = frame->t_ns;
      if (metric) {
        metric_vision.push(frame);
        tracking_state = metric->tracking_state.load();
        continue;
      }
      history[frame->t_ns] = frame;
      while (!history.empty() && history.begin()->first < frame->t_ns - keep_ns) history.erase(history.begin());
      while (!samples.empty() && samples.front().t_ns < frame->t_ns - keep_ns) samples.pop_front();
      initializer.addFrame(*frame);
      const MonoInertialInitResult result = initializer.tryInitialize();
      if (!result.success) {
        if (!result.method.empty() && result.reason != last_reason) {
          std::cout << "MONO_INIT_TRY " << result.summary() << std::endl;
          last_reason = result.reason;
        }
        continue;
      }

      // Prior of the start state: the initializer's information, with Granite's
      // gauge weight on position and yaw (unobservable in VIO).
      Eigen::Matrix<double, 15, 15> information =
          result.covariance.ldlt().solve(Eigen::Matrix<double, 15, 15>::Identity());
      for (int i : {0, 1, 2, 5}) {
        information.row(i).setZero();
        information.col(i).setZero();
        information(i, i) = configuration.vio_init_pose_weight;
      }
      information = 0.5 * (information + information.transpose()).eval();
      if (!information.allFinite()) {
        std::cout << "MONO_INIT_TRY prior not finite" << std::endl;
        continue;
      }
      metric = std::make_shared<KeypointVioEstimator>(gravity, calibration, configuration);
      metric->vision_data_queue = &metric_vision;
      metric->imu_data_queue = &metric_imu;
      metric->out_state_queue = out_state_queue;
      metric->out_vis_queue = out_vis_queue;
      metric->out_marg_queue = out_marg_queue;
      // IMU from the last sample at or before the start state, then the images
      // from the start state on (replayed from the history)
      size_t first = 0;
      while (first + 1 < samples.size() && samples[first + 1].t_ns <= result.t_ns) ++first;
      for (size_t k = first; k < samples.size(); ++k) metric_imu.push(std::make_shared<ImuData>(samples[k]));
      metric->initializeFromInertialStart(result.t_ns, result.T_w_i, result.vel_w_i, result.bias_gyro,
                                          result.bias_accel, information);
      for (auto it = history.lower_bound(result.t_ns); it != history.end(); ++it) metric_vision.push(it->second);
      initial_pose = result.T_w_i;
      metric_scale = 1.0;
      metric_initialized = true;
      tracking_state = TrackingState::TRACKING;
      std::cout << "MONO_VIO_INITIALIZED init=inertial " << result.summary() << std::endl;
      history.clear();
      samples.clear();
    }
  } catch (const tbb::user_abort&) {
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
