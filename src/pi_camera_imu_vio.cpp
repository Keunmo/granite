// SPDX-License-Identifier: MIT
// Live mono VIO/VO on a Raspberry Pi 5 or 4 with an Arducam B0429 (AR0234)
// CSI camera and an MPU-6500/9250 IMU on an I2C adapter.
//
// VIO and VO run the live mode of granite_vio (src/vio_app.h) with its
// Pangolin GUI: this program is the same as granite_vio --live with the
// options of this program. --capture-only and --record without --calib run
// no estimator and show the images in an OpenCV window.
//
// The estimator path is that of the offline runner eval/src/mono_vio_dataset.cpp
// in its --realtime mode: the same VioConfig file, vio_enforce_realtime on,
// initialize() with zero biases, a flow input queue of --queue-limit images
// that drops its oldest image when it is full. --out writes the TUM trajectory
// and the state CSV of the runner. --record writes the images and the IMU of
// the same run as an EuRoC folder, which the runner replays.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <mutex>
#include <thread>
#include <utility>
#include <CLI/CLI.hpp>
#include <cereal/archives/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>
#include <granite/device/pi_camera_imu.h>
#include <granite/device/pi_live_support.h>
#include <granite/device/t265_gui.h>
#include <granite/serialization/headers_serialization.h>
#include <granite/vi_estimator/mono_vio.h>
#include <granite/vi_estimator/vio_estimator.h>

#include "vio_app.h"

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }

using granite::pi_live::RecordWriter;
using granite::pi_live::loadCameraConfig;
using granite::pi_live::monotonicNs;
using granite::pi_live::percentile;

// Keeps the newest visualization on a separate thread. The estimator blocks
// on a full output queue, so a slow window must not hold its queue.
class VisualizationDrain {
 public:
  explicit VisualizationDrain(
      tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr>& queue)
      : queue(queue), thread([this] { run(); }) {}
  ~VisualizationDrain() {
    stopping = true;
    queue.push(nullptr);
    thread.join();
  }

  granite::VioVisualizationData::Ptr take() {
    std::lock_guard<std::mutex> lock(mutex);
    return std::exchange(newest, nullptr);
  }

 private:
  void run() {
    granite::VioVisualizationData::Ptr data;
    while (true) {
      queue.pop(data);
      // The estimators send one end marker per stage, so only the destructor
      // ends the loop.
      if (!data) {
        if (stopping) return;
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex);
      newest = data;
    }
  }

  tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr>& queue;
  std::atomic<bool> stopping{false};
  std::mutex mutex;
  granite::VioVisualizationData::Ptr newest;
  std::thread thread;
};
}  // namespace

int main(int argc, char** argv) {
  granite::PiCameraImuOptions options;
  bool use_imu = true, capture_only = false, no_gui = false, resize_half = false;
  int num_threads = 3, queue_limit = 4;
  double duration = 0;
  std::string calib_path, config_path, trajectory_path, record_path, out_prefix;
  CLI::App app{"Granite live mono VIO: Raspberry Pi 5 or 4, Arducam B0429 and MPU-6500/9250"};
  app.add_option("--calib", calib_path,
                 "Granite camera/IMU calibration JSON at the sensor mode or at 1/downscale of it");
  app.add_option("--config-path", config_path,
                 "Granite VioConfig JSON; optional camera.* keys (width, height, fps, downscale, "
                 "exposure_ms, gain_db, max_exposure_ms, max_gain, brightness_target, "
                 "metering_circle, i2c_device, imu_rate) set the sensors");
  app.add_option("--use-imu", use_imu, "1: VIO, 0: visual odometry");
  granite::pi_live::addSensorOptions(app, options, resize_half);
  app.add_option("--num-threads", num_threads, "TBB worker limit");
  app.add_option("--duration", duration, "Stop after this many seconds (0: until SIGINT)");
  app.add_option("--trajectory", trajectory_path, "Output timestamp, pose and velocity CSV");
  app.add_option("--out", out_prefix,
                 "Output prefix: <out>_traj_tum.txt and <out>_states.csv of the offline runner");
  app.add_option("--queue-limit", queue_limit,
                 "Images that wait at the flow input; a new image drops the oldest above it");
  app.add_option("--record", record_path,
                 "Record the images and the IMU of this run as an EuRoC dataset (with VIO "
                 "when --calib is given)");
  app.add_flag("--capture-only", capture_only, "Check image and IMU delivery without running VIO");
  app.add_flag("--no-gui", no_gui, "Disable the window (the Pangolin GUI of granite_vio for VIO)");
  CLI11_PARSE(app, argc, argv);
  if (resize_half && app.count("--downscale") == 0) options.downscale = 2;
  const bool estimate = !capture_only && !(calib_path.empty() && !record_path.empty());
  if (num_threads < 1 || queue_limit < 1 || !std::isfinite(duration) || duration < 0) {
    std::cerr << "Invalid thread count, queue limit or duration" << std::endl;
    return 2;
  }
  if (estimate && calib_path.empty()) {
    std::cerr << "VIO needs --calib. Use --capture-only or --record without a calibration."
              << std::endl;
    return 2;
  }
  // The window needs X11; a headless SSH session has no DISPLAY.
  if (!std::getenv("DISPLAY")) no_gui = true;
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);

  if (estimate) {
    // VIO and VO use the Pangolin GUI of granite_vio (src/vio_app.h).
    try {
      tbb::global_control threads(tbb::global_control::max_allowed_parallelism, num_threads);
      cv::setNumThreads(1);
      if (!config_path.empty()) loadCameraConfig(config_path, app, options);
      VioRunOptions run;
      run.live = true;
      run.pi = options;
      run.queue_limit = queue_limit;
      run.out_prefix = out_prefix;
      run.trajectory_path = trajectory_path;
      run.record_path = record_path;
      run.duration = duration;
      std::signal(SIGINT, granite_vio_interrupt);
      std::signal(SIGTERM, granite_vio_interrupt);
      VioDataset vd(calib_path, config_path, "", "", "", "", "", "", !no_gui, use_imu, false, false,
                    run);
      vd.run();
      return vd.liveFailed() ? 1 : 0;
    } catch (const std::exception& error) {
      std::cerr << "Pi VIO: " << error.what() << std::endl;
      return 1;
    }
  }

  // --capture-only and a recording without VIO: no estimator. The OpenCV
  // window shows the images.
  try {
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, num_threads);
    cv::setNumThreads(1);
    granite::VioConfig config;
    config.vio_enforce_realtime = true;
    config.vio_max_kfs = 5;
    config.vio_max_iterations = 4;
    config.vio_use_lm = true;
    if (!config_path.empty()) {
      config.load(config_path);
      loadCameraConfig(config_path, app, options);
    }
    // The keyframe decision needs an iteration before the last one; otherwise
    // the marginalization of the decision fails an assertion.
    if (config.vio_take_keyframe_iteration >= config.vio_max_iterations)
      throw std::runtime_error("vio_take_keyframe_iteration must be less than vio_max_iterations");

    granite::Calibration<double> calib;
    if (!calib_path.empty()) {
      std::ifstream input(calib_path);
      if (!input) throw std::runtime_error("Cannot open calibration " + calib_path);
      cereal::JSONInputArchive archive(input);
      archive(calib);
      granite::pi_live::fitCalibration(calib, options);
      calib.main_cam_idx = {0};
      calib.stereo_pairs.clear();
      // The estimators do not apply the calibrated camera time offset.
      options.camera_time_offset_ns = calib.cam_time_offset_ns;
    }

    std::ofstream trajectory;
    if (!trajectory_path.empty()) {
      trajectory.open(trajectory_path);
      if (!trajectory) throw std::runtime_error("Cannot open trajectory output");
      trajectory << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw,vx,vy,vz\n" << std::setprecision(16);
    }
    std::ofstream tum, states_csv;
    if (!out_prefix.empty()) {
      tum.open(out_prefix + "_traj_tum.txt");
      states_csv.open(out_prefix + "_states.csv");
      if (!tum || !states_csv) throw std::runtime_error("Cannot open the --out files");
      tum << std::setprecision(19);
      states_csv << "t_ns,wall_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,support,"
                    "connected_finite,connected_infinite,landmarks,map_idx,prev_corr_rot,"
                    "prev_corr_pos,healthy,latency_ms\n"
                 << std::setprecision(12);
    }
    std::unique_ptr<RecordWriter> recorder;
    if (!record_path.empty()) recorder = std::make_unique<RecordWriter>(record_path);
    std::unique_ptr<granite::T265Gui> gui;
    if (!no_gui) {
      gui = std::make_unique<granite::T265Gui>();
      gui->open();
    }

    granite::PiCameraImuDevice device(options);
    device.start();
    std::cout
        << "mode="
        << (estimate ? (use_imu ? "mono VIO" : "mono VO")
                     : (recorder ? "record" : "capture"))
        << " receiver=" << device.receiver() << " camera=" << options.model
        << " binning=" << device.sensorBinning()
        << " sensor=" << options.sensorWidth() << "x" << options.sensorHeight()
        << " output=" << device.outputWidth() << "x" << device.outputHeight()
        << " fps=" << device.frameRate()
        << " line_us=" << device.lineTimeNs() * 1e-3 << " exposure_ms="
        << ((options.xeyeProcessing() ? !options.auto_exposure.auto_exposure
                                      : options.exposure_ms > 0)
                ? std::to_string(device.exposure_lines.load() *
                                 device.lineTimeNs() * 1e-6)
                : "auto")
        << " gain_db=" << device.gainDb() << " imu="
        << (options.mavlink()
                ? "imu_mavlink " + (options.imu_port.empty()
                                        ? (options.imu_udp.empty()
                                               ? std::string("127.0.0.1:14601")
                                               : options.imu_udp)
                                        : options.imu_port)
                : "mpu_i2c " + options.i2c_device)
        << " threads=" << num_threads << " queue_limit=" << queue_limit
        << std::endl;
    if (std::abs(device.frameRate() - options.fps) > 0.5)
      std::cerr << "The sensor mode limits " << options.fps << " fps to " << device.frameRate()
                << " fps" << std::endl;
    if ((options.xeyeProcessing()
             ? options.exposure_time_ms
             : options.exposure_ms) > device.maxExposureMs())
      std::cerr << "The exposure of "
                << (options.xeyeProcessing() ? options.exposure_time_ms
                                             : options.exposure_ms)
                << " ms is longer than the frame; "
                << "the camera uses " << device.maxExposureMs() << " ms"
                << std::endl;

    tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu_queue;
    tbb::concurrent_bounded_queue<granite::OpticalFlowInput::Ptr> capture_queue;
    tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> state_queue;
    tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr> visualization_queue;
    capture_queue.set_capacity(30);
    // The estimator blocks on a full state queue; the window can hold the
    // main loop for some frames.
    state_queue.set_capacity(120);
    visualization_queue.set_capacity(2);
    granite::OpticalFlowBase::Ptr flow;
    granite::VioEstimatorBase::Ptr vio;
    std::unique_ptr<VisualizationDrain> visualization_drain;
    if (estimate) {
      flow = granite::OpticalFlowFactory::getOpticalFlow(config, calib);
      flow->input_queue.set_capacity(queue_limit);
      device.image_data_queue = &flow->input_queue;
    } else {
      device.image_data_queue = &capture_queue;
    }
    device.imu_data_queue = &imu_queue;
    if (recorder) {
      device.record_image_queue = &recorder->images;
      device.record_imu_queue = &recorder->imu;
    }
    const uint64_t start_images = device.image_frames.load();
    const uint64_t start_imu = device.imu_samples.load();
    device.enableCapture();
    const auto start = std::chrono::steady_clock::now();
    const auto expired = [&] {
      return duration > 0 &&
             std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= duration;
    };
    if (estimate) {
      // The estimator expects its first IMU measurement to be present.
      while (imu_queue.empty() && !device.failed && !interrupted &&
             std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (imu_queue.empty()) throw std::runtime_error("The IMU did not deliver data");
      vio = granite::VioEstimatorFactory::getVioEstimator(config, calib, granite::constants::g, use_imu);
      vio->vision_data_queue = &flow->output_queue;
      vio->imu_data_queue = &imu_queue;
      vio->out_state_queue = &state_queue;
      if (gui) {
        vio->out_vis_queue = &visualization_queue;
        visualization_drain = std::make_unique<VisualizationDrain>(visualization_queue);
      }
      vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    const auto mono_vio = std::dynamic_pointer_cast<granite::MonoVioEstimator>(vio);

    uint64_t states = 0, previous_states = 0, previous_images = 0, previous_imu = 0;
    std::vector<double> latencies, brightness_samples;
    double first_state_wall = -1;
    int64_t first_state_t_ns = 0, first_image_t_ns = 0;
    int64_t last_state_ns = 0, previous_image_ns = 0, previous_imu_ns = 0;
    bool invalid = false;
    size_t landmarks = 0, finite_landmarks = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    std::string support = estimate ? "initializing" : "capture";
    auto report_time = start, display_time = start;
    double input_fps = 0, state_fps = 0;
    auto consume = [&] {
      granite::OpticalFlowInput::Ptr image;
      while (capture_queue.try_pop(image)) {
        if (!image) continue;
        if (image->t_ns <= previous_image_ns) invalid = true;
        previous_image_ns = image->t_ns;
        if (gui) gui->setFrame(image);
      }
      if (!estimate || !use_imu) {
        // Without a VIO consumer the IMU stream is checked or recorded here.
        granite::ImuData::Ptr sample;
        while (imu_queue.try_pop(sample)) {
          if (!sample) continue;
          if (sample->t_ns <= previous_imu_ns || !sample->accel.allFinite() ||
              !sample->gyro.allFinite())
            invalid = true;
          previous_imu_ns = sample->t_ns;
        }
      }
      granite::VioStateData::Ptr result;
      while (state_queue.try_pop(result)) {
        if (!result) continue;
        const auto& state = result->state;
        if (!state.T_w_i.matrix().allFinite() || !state.vel_w_i.allFinite() ||
            state.t_ns <= last_state_ns)
          invalid = true;
        last_state_ns = state.t_ns;
        ++states;
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        // From the exposure center (device clock) to the state output.
        const double latency_ms =
            (monotonicNs() - (state.t_ns - options.camera_time_offset_ns)) * 1e-6;
        latencies.push_back(latency_ms);
        if (first_state_wall < 0) {
          first_state_wall = wall;
          first_state_t_ns = state.t_ns;
        }
        landmarks = result->num_landmarks;
        finite_landmarks = result->num_finite_landmarks;
        position = state.T_w_i.translation();
        if (gui && !invalid) gui->setState(result);
        switch (result->mono_vio_support) {
          case granite::MonoVioSupport::METRIC: support = "metric"; break;
          case granite::MonoVioSupport::BEARING_ONLY: support = "bearing only"; break;
          case granite::MonoVioSupport::INERTIAL_ONLY: support = "inertial only"; break;
          case granite::MonoVioSupport::NOT_APPLICABLE: support = use_imu ? "visual-inertial" : "visual only"; break;
        }
        if (tum) {
          const auto& p = state.T_w_i.translation();
          const auto q = state.T_w_i.unit_quaternion();
          tum << state.t_ns * 1e-9 << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x()
              << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
          const int code = static_cast<int>(result->mono_vio_support);
          const char* label = code == 1 ? "metric" : code == 2 ? "bearing" : code == 3 ? "inertial" : "n/a";
          states_csv << state.t_ns << ',' << wall << ',' << p.x() << ',' << p.y() << ',' << p.z()
                     << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
                     << state.vel_w_i.x() << ',' << state.vel_w_i.y() << ',' << state.vel_w_i.z()
                     << ',' << state.bias_gyro.x() << ',' << state.bias_gyro.y() << ','
                     << state.bias_gyro.z() << ',' << state.bias_accel.x() << ','
                     << state.bias_accel.y() << ',' << state.bias_accel.z() << ',' << label << ','
                     << result->num_connected_finite_landmarks << ','
                     << result->num_connected_infinite_landmarks << ',' << result->num_landmarks
                     << ',' << result->map_idx << ',' << result->prev_correction_rot << ','
                     << result->prev_correction_pos << ',' << result->healthy << ','
                     << latency_ms << '\n';
        }
        if (trajectory) {
          const auto& p = state.T_w_i.translation();
          const auto q = state.T_w_i.unit_quaternion();
          trajectory << state.t_ns << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
                     << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
                     << state.vel_w_i.x() << ',' << state.vel_w_i.y() << ',' << state.vel_w_i.z() << '\n';
        }
      }
      if (visualization_drain) {
        if (auto visualization = visualization_drain->take()) gui->setVisualization(visualization);
      }
    };
    const auto tracking = [&]() -> std::string {
      if (!vio) return "capture";
      switch (vio->tracking_state.load()) {
        case granite::VioEstimatorBase::TrackingState::UNINITIALIZED: return "initializing";
        case granite::VioEstimatorBase::TrackingState::TRACKING: return "tracking";
        case granite::VioEstimatorBase::TrackingState::LOST: return "lost";
      }
      return "unknown";
    };

    auto last_image_time = start;
    uint64_t watchdog_images = 0;
    bool stalled = false;
    while (!interrupted && !device.failed && !invalid && !expired()) {
      consume();
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      const double interval = std::chrono::duration<double>(now - report_time).count();
      if (interval >= 1) {
        const auto images = device.image_frames.load();
        const auto imu = device.imu_samples.load();
        input_fps = (images - previous_images) / interval;
        state_fps = (states - previous_states) / interval;
        std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed
                  << " input_fps=" << input_fps << " state_fps=" << state_fps
                  << " imu_hz=" << (imu - previous_imu) / interval
                  << " exposure_ms="
                  << device.exposure_lines.load() * device.lineTimeNs() * 1e-6
                  << " gain_db=" << device.gainDb()
                  << " brightness=" << device.mean_brightness.load()
                  << " drops=" << device.dropped_frames.load() << "/"
                  << device.sensor_frame_drops.load()
                  << " imu_overflows=" << device.imu_overflows.load()
                  << " imu_errors=" << device.imu_i2c_errors.load()
                  << " lag_ms="
                  << (last_state_ns
                          ? (device.last_image_t_ns.load() - last_state_ns) *
                                1e-6
                          : 0)
                  << " tracking=" << tracking() << " support=" << support
                  << " landmarks=" << landmarks
                  << " finite=" << finite_landmarks << " metric_initialized="
                  << (mono_vio ? mono_vio->metricInitialized() : false)
                  << " p=" << position.transpose() << std::endl;
        if (first_image_t_ns == 0) first_image_t_ns = device.first_capture_t_ns.load();
        brightness_samples.push_back(device.mean_brightness.load());
        previous_images = images;
        previous_states = states;
        previous_imu = imu;
        report_time = now;
      }
      if (gui && now - display_time >= std::chrono::milliseconds(33)) {
        granite::T265GuiStatus status;
        status.mode = estimate ? (use_imu ? "MONO VIO" : "MONO VO") : (recorder ? "RECORD" : "CAPTURE");
        status.capture_only = !estimate;
        status.metric_initialized = mono_vio && mono_vio->metricInitialized();
        status.support = support;
        status.tracking = tracking();
        status.elapsed = elapsed;
        status.input_fps = input_fps;
        status.state_fps = state_fps;
        status.lag_ms = last_state_ns ? (device.last_image_t_ns.load() - last_state_ns) * 1e-6 : 0;
        status.images = device.image_frames.load();
        status.states = states;
        status.dropped = device.dropped_frames.load();
        status.landmarks = landmarks;
        status.finite_landmarks = finite_landmarks;
        if (!gui->draw(status)) break;
        display_time = now;
      }
      if (device.image_frames.load() != watchdog_images) {
        watchdog_images = device.image_frames.load();
        last_image_time = now;
      } else if (now - last_image_time > std::chrono::seconds(3)) {
        std::cerr << "Camera stream stalled: no frames for 3 seconds" << std::endl;
        stalled = true;
        break;
      }
      // The offline runner polls its state queue every 2 ms.
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    const double capture_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const uint64_t captured_images = device.image_frames.load() - start_images;
    const uint64_t captured_imu = device.imu_samples.load() - start_imu;
    device.stop(false);
    if (recorder) recorder->finish();
    // End the streams with sentinels instead of OpticalFlowBase::quit(true).
    // quit(true) clears the flow output queue while the flow thread can push
    // into it; TBB clear() is not thread safe and the push then never returns.
    imu_queue.push(nullptr);
    bool flow_input_closed = !flow, flow_output_closed = !flow;
    auto last_flow_output = std::chrono::steady_clock::now();
    while (true) {
      if (!flow_input_closed) flow_input_closed = flow->input_queue.try_push(nullptr);
      consume();
      const bool estimator_done = !vio || vio->isFinished();
      if (flow && estimator_done) {
        // Mono VIO can stop at the IMU sentinel before it reads all images.
        // Nothing reads the flow output then, so drain it here.
        granite::OpticalFlowResult::Ptr result;
        while (flow->output_queue.try_pop(result)) {
          last_flow_output = std::chrono::steady_clock::now();
          if (!result) flow_output_closed = true;
        }
      }
      // The estimator can consume the flow sentinel itself; a quiet output
      // queue then shows that the flow thread has left its loop.
      if (estimator_done && flow_input_closed &&
          (flow_output_closed ||
           std::chrono::steady_clock::now() - last_flow_output > std::chrono::milliseconds(500)))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (flow) flow->join();
    if (vio) vio->join();
    consume();
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "SUMMARY seconds=" << elapsed << " images=" << device.image_frames.load()
              << " queued=" << device.queued_frames.load() << " dropped=" << device.dropped_frames.load()
              << " sensor_drops=" << device.sensor_frame_drops.load()
              << " imu=" << device.imu_samples.load() << " imu_overflows=" << device.imu_overflows.load()
              << " imu_period_ms=" << device.imu_period_ns.load() * 1e-6
              << " states=" << states << " metric_initialized="
              << (mono_vio ? mono_vio->metricInitialized() : false)
              << " valid=" << !invalid << std::endl;
    double brightness_mean = 0;
    for (double b : brightness_samples) brightness_mean += b;
    if (!brightness_samples.empty()) brightness_mean /= brightness_samples.size();
    if (first_image_t_ns == 0) first_image_t_ns = device.first_capture_t_ns.load();
    const uint64_t queued = device.queued_frames.load();
    std::cout << std::setprecision(4) << "LIVE fps=" << captured_images / capture_seconds
              << " sensor_fps=" << device.frameRate() << " sensor_drops=" << device.sensor_frame_drops.load()
              << " queue_drops=" << device.dropped_frames.load()
              << " imu_hz=" << captured_imu / capture_seconds
              << " imu_period_ms=" << device.imu_period_ns.load() * 1e-6
              << " imu_overflows=" << device.imu_overflows.load()
              << " imu_i2c_errors=" << device.imu_i2c_errors.load()
              << " imu_error_max_ms=" << device.imu_error_max_ns.load() * 1e-6
              << " image_delay_ms_mean="
              << (queued ? device.image_delay_sum_ns.load() * 1e-6 / queued : -1.0)
              << " image_delay_ms_max=" << device.image_delay_max_ns.load() * 1e-6
              << " latency_ms_p50=" << percentile(latencies, 0.5)
              << " latency_ms_p95=" << percentile(latencies, 0.95)
              << " latency_ms_max=" << percentile(latencies, 1.0)
              << " brightness_mean=" << brightness_mean
              << " first_state_wall_s=" << first_state_wall << " first_state_data_s="
              << (first_state_t_ns && first_image_t_ns ? (first_state_t_ns - first_image_t_ns) * 1e-9 : -1.0)
              << " states=" << states;
    if (recorder)
      std::cout << " recorded_images=" << recorder->written_images.load()
                << " record_drops=" << device.record_dropped_frames.load()
                << " recorded_imu=" << recorder->written_imu.load();
    std::cout << std::endl;
    if (device.failed) std::cerr << device.errorMessage() << std::endl;
    return invalid || stalled || device.failed || device.image_frames == 0 ||
                   device.imu_samples == 0 ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "Pi VIO: " << error.what() << std::endl;
    return 1;
  }
}
