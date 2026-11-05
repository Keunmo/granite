// SPDX-License-Identifier: MIT
// Live mono VIO/VO on a Raspberry Pi 5 with an Arducam B0429 (AR0234) CSI
// camera and an MPU-6500/9250 IMU on an I2C adapter.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <utility>
#include <CLI/CLI.hpp>
#include <cereal/archives/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/global_control.h>
#include <granite/device/pi_camera_imu.h>
#include <granite/device/t265_gui.h>
#include <granite/serialization/headers_serialization.h>
#include <granite/vi_estimator/mono_vio.h>
#include <granite/vi_estimator/vio_estimator.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }

// Writes the EuRoC layout that granite_calibrate, granite_calibrate_imu and
// granite_vio read.
class EurocRecorder {
 public:
  explicit EurocRecorder(const std::string& path) : root(path + "/mav0") {
    std::filesystem::create_directories(root + "/cam0/data");
    std::filesystem::create_directories(root + "/imu0");
    images.open(root + "/cam0/data.csv");
    imu.open(root + "/imu0/data.csv");
    if (!images || !imu) throw std::runtime_error("Cannot create recording in " + path);
    images << "#timestamp [ns],filename\n";
    imu << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
           "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        << std::setprecision(10);
  }

  void write(const granite::OpticalFlowInput& frame) {
    const auto& image = *frame.img_data[0].img;
    cv::Mat pixels(image.h, image.w, CV_8UC1);
    for (size_t y = 0; y < image.h; ++y) {
      for (size_t x = 0; x < image.w; ++x)
        pixels.at<uint8_t>(y, x) = static_cast<uint8_t>(image(x, y));
    }
    const std::string name = std::to_string(frame.t_ns) + ".png";
    if (!cv::imwrite(root + "/cam0/data/" + name, pixels))
      throw std::runtime_error("Cannot write image " + name);
    images << frame.t_ns << ',' << name << '\n';
  }

  void write(const granite::ImuData& sample) {
    imu << sample.t_ns << ',' << sample.gyro.x() << ',' << sample.gyro.y() << ','
        << sample.gyro.z() << ',' << sample.accel.x() << ',' << sample.accel.y() << ','
        << sample.accel.z() << '\n';
  }

 private:
  std::string root;
  std::ofstream images, imu;
};

// Reads the optional "camera.*" keys next to the VioConfig keys. VioConfig
// ignores them. A command line option has priority over its key.
void loadCameraConfig(const std::string& path, const CLI::App& app,
                      granite::PiCameraImuOptions& options) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open config " + path);
  cereal::JSONInputArchive archive(input);
  archive.setNextName("value0");
  archive.startNode();
  const auto load = [&](const char* key, const char* option, auto& value) {
    if (app.count(option) > 0) return;
    try {
      archive(cereal::make_nvp(key, value));
    } catch (const cereal::Exception&) {
      // The key is not in the file; keep the default.
    }
  };
  load("camera.fps", "--fps", options.fps);
  load("camera.exposure_ms", "--exposure", options.exposure_ms);
  load("camera.gain_db", "--gain", options.gain_db);
  load("camera.resize_half", "--resize-half", options.resize_half);
  archive.finishNode();
}

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
  bool use_imu = true, capture_only = false, no_gui = false;
  int num_threads = 3;
  double duration = 0;
  std::string calib_path, config_path, trajectory_path, record_path;
  CLI::App app{"Granite live mono VIO: Raspberry Pi 5, Arducam B0429 and MPU-6500/9250"};
  app.add_option("--calib", calib_path, "Granite camera/IMU calibration JSON at capture resolution");
  app.add_option("--config-path", config_path,
                 "Granite VioConfig JSON; optional camera.fps, camera.exposure_ms, "
                 "camera.gain_db and camera.resize_half keys set the camera");
  app.add_option("--use-imu", use_imu, "1: VIO, 0: visual odometry");
  app.add_option("--width", options.width, "Sensor mode width (1920, 1280)");
  app.add_option("--height", options.height, "Sensor mode height (1200, 1080, 720)");
  app.add_option("--fps", options.fps, "Camera frame rate");
  app.add_flag("--resize-half", options.resize_half, "Average 2x2 Bayer quads and halve the intrinsics");
  app.add_option("--exposure", options.exposure_ms, "Manual exposure in ms (0: auto)");
  app.add_option("--max-exposure", options.max_exposure_ms, "Auto exposure limit in ms");
  app.add_option("--gain", options.gain_db, "Analogue gain in dB for manual exposure (0-11)");
  app.add_option("--max-gain", options.max_gain, "Auto exposure analogue gain limit (analogue_gain / 100)");
  app.add_option("--brightness", options.brightness_target, "Auto exposure target mean (0-255)");
  app.add_option("--i2c", options.i2c_device, "IMU I2C adapter device");
  app.add_option("--imu-address", options.imu_address, "IMU 7-bit I2C address");
  app.add_option("--imu-rate", options.imu_rate, "IMU sample rate in Hz");
  app.add_option("--num-threads", num_threads, "TBB worker limit");
  app.add_option("--duration", duration, "Stop after this many seconds (0: until SIGINT)");
  app.add_option("--trajectory", trajectory_path, "Output timestamp, pose and velocity CSV");
  app.add_option("--record", record_path, "Record images and IMU as an EuRoC dataset; no VIO");
  app.add_flag("--capture-only", capture_only, "Check image and IMU delivery without running VIO");
  app.add_flag("--no-gui", no_gui, "Disable the live window");
  CLI11_PARSE(app, argc, argv);
  const bool estimate = !capture_only && record_path.empty();
  if (num_threads < 1 || !std::isfinite(duration) || duration < 0) {
    std::cerr << "Invalid thread count or duration" << std::endl;
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
      if (calib.intrinsics.size() != 1 || calib.T_i_c.size() != 1 || calib.resolution.size() != 1)
        throw std::runtime_error("Expected a calibration with one camera");
      if (calib.resolution[0] != Eigen::Vector2i(options.width, options.height))
        throw std::runtime_error("Calibration resolution does not match the sensor mode");
      calib.main_cam_idx = {0};
      calib.stereo_pairs.clear();
      // The estimators do not apply the calibrated camera time offset.
      options.camera_time_offset_ns = calib.cam_time_offset_ns;
      if (options.resize_half) granite::scaleCalibrationHalf(calib);
    }

    std::ofstream trajectory;
    if (!trajectory_path.empty()) {
      trajectory.open(trajectory_path);
      if (!trajectory) throw std::runtime_error("Cannot open trajectory output");
      trajectory << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw,vx,vy,vz\n" << std::setprecision(16);
    }
    std::unique_ptr<EurocRecorder> recorder;
    if (!record_path.empty()) recorder = std::make_unique<EurocRecorder>(record_path);
    std::unique_ptr<granite::T265Gui> gui;
    if (!no_gui) {
      gui = std::make_unique<granite::T265Gui>();
      gui->open();
    }

    granite::PiCameraImuDevice device(options);
    device.start();
    std::cout << "mode=" << (estimate ? (use_imu ? "mono VIO" : "mono VO") :
                             (recorder ? "record" : "capture"))
              << " sensor=" << options.width << "x" << options.height
              << " output=" << device.outputWidth() << "x" << device.outputHeight()
              << " fps=" << device.frameRate() << " line_us=" << device.lineTimeNs() * 1e-3
              << " exposure_ms="
              << (options.exposure_ms > 0 ? std::to_string(device.exposure_lines.load() *
                                                           device.lineTimeNs() * 1e-6)
                                          : "auto")
              << " gain_db=" << granite::b0429DbFromDriverGain(device.analogue_gain.load())
              << " imu_rate=" << options.imu_rate << " threads=" << num_threads << std::endl;
    if (std::abs(device.frameRate() - options.fps) > 0.5)
      std::cerr << "The sensor mode limits " << options.fps << " fps to " << device.frameRate()
                << " fps" << std::endl;
    if (options.exposure_ms > device.maxExposureMs())
      std::cerr << "The exposure of " << options.exposure_ms << " ms is longer than the frame; "
                << "the camera uses " << device.maxExposureMs() << " ms" << std::endl;

    tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu_queue;
    tbb::concurrent_bounded_queue<granite::OpticalFlowInput::Ptr> capture_queue;
    tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> state_queue;
    tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr> visualization_queue;
    // PNG encoding is slower than capture; keep a short burst for recording.
    capture_queue.set_capacity(recorder ? 30 : 2);
    // The estimator blocks on a full state queue; the window can hold the
    // main loop for some frames.
    state_queue.set_capacity(120);
    visualization_queue.set_capacity(2);
    granite::OpticalFlowBase::Ptr flow;
    granite::VioEstimatorBase::Ptr vio;
    std::unique_ptr<VisualizationDrain> visualization_drain;
    if (estimate) {
      flow = granite::OpticalFlowFactory::getOpticalFlow(config, calib);
      flow->input_queue.set_capacity(2);
      device.image_data_queue = &flow->input_queue;
    } else {
      device.image_data_queue = &capture_queue;
    }
    device.imu_data_queue = &imu_queue;
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
        if (recorder) recorder->write(*image);
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
          if (recorder) recorder->write(*sample);
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
                  << " exposure_ms=" << device.exposure_lines.load() * device.lineTimeNs() * 1e-6
                  << " gain_db=" << granite::b0429DbFromDriverGain(device.analogue_gain.load())
                  << " brightness=" << device.mean_brightness.load()
                  << " drops=" << device.dropped_frames.load() << "/" << device.sensor_frame_drops.load()
                  << " imu_overflows=" << device.imu_overflows.load()
                  << " lag_ms=" << (last_state_ns ? (device.last_image_t_ns.load() - last_state_ns) * 1e-6 : 0)
                  << " tracking=" << tracking() << " support=" << support
                  << " landmarks=" << landmarks << " finite=" << finite_landmarks
                  << " metric_initialized=" << (mono_vio ? mono_vio->metricInitialized() : false)
                  << " p=" << position.transpose() << std::endl;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    device.stop(false);
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
    if (device.failed) std::cerr << device.errorMessage() << std::endl;
    return invalid || stalled || device.failed || device.image_frames == 0 ||
                   device.imu_samples == 0 ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "Pi VIO: " << error.what() << std::endl;
    return 1;
  }
}
