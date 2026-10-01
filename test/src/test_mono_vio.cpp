#include <granite/vi_estimator/mono_vio.h>
#include <tbb/global_control.h>
#include <gtest/gtest.h>

namespace {
granite::Calibration<double> monoCalibration() {
  granite::Calibration<double> c;
  granite::GenericCamera<double> camera;
  granite::KannalaBrandtCamera4<double>::VecN p;
  p << 250, 250, 212, 200, 0, 0, 0, 0;
  camera.variant = granite::KannalaBrandtCamera4<double>(p);
  c.intrinsics.push_back(camera);
  c.T_i_c.emplace_back();
  c.resolution.emplace_back(424, 400);
  c.main_cam_idx = {0};
  return c;
}

granite::OpticalFlowResult::Ptr observations(const granite::Calibration<double>& c,
                                           int64_t timestamp, const Sophus::SE3d& pose) {
  auto frame = std::make_shared<granite::OpticalFlowResult>();
  frame->t_ns = timestamp;
  frame->observations.resize(1);
  frame->pyramid_levels.resize(1);
  for (int i = 0; i < 80; ++i) {
    const Eigen::Vector3d point((i % 10 - 4.5) * 0.35, (i / 10 - 3.5) * 0.3,
                                3.0 + (i % 7) * 0.35);
    const Eigen::Vector3d local = pose.inverse() * point;
    Eigen::Vector4d ray;
    ray << local, 1;
    Eigen::Vector2d pixel;
    if (!c.intrinsics[0].project(ray, pixel)) continue;
    Eigen::AffineCompact2f affine = Eigen::AffineCompact2f::Identity();
    affine.translation() = pixel.cast<float>();
    frame->observations[0][i] = affine;
    frame->pyramid_levels[0][i] = 0;
  }
  return frame;
}

struct Result {
  granite::VioStateData::Ptr last, first;
  size_t landmarks = 0, finite = 0;
  double yaw_error = 0;
  bool initialized = false;
};

Result run(bool moving, const Eigen::Vector3d& accel_bias = Eigen::Vector3d::Zero()) {
  tbb::global_control control(tbb::global_control::max_allowed_parallelism, 2);
  auto c = monoCalibration();
  granite::VioConfig config;
  config.vio_enforce_realtime = false;
  config.vio_use_lm = true;
  config.vio_max_iterations = 7;
  config.vio_max_kfs = 2;
  config.vio_min_frames_after_kf = 2;
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> states;
  constexpr int64_t origin = 1000000000;
  constexpr double period = 3.0;
  const double omega = 2 * M_PI / period;
  for (int i = 0; i <= 620; ++i) {
    double t = i * 0.005;
    auto sample = std::make_shared<granite::ImuData>();
    sample->t_ns = origin + i * 5000000LL;
    double yaw = moving ? 0.15 * t : 0;
    Eigen::Vector3d acceleration(moving ? std::sin(omega * t) : 0, 0, 0);
    sample->accel = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw)).inverse() *
                    (acceleration - granite::constants::g) + accel_bias;
    sample->gyro = Eigen::Vector3d(0, 0, moving ? 0.19 : 0.04);
    imu.push(sample);
  }
  Sophus::SE3d truth;
  for (int i = 0; i <= 90; ++i) {
    double t = i / 30.0;
    double x = moving ? t / omega - std::sin(omega * t) / (omega * omega) : 0;
    truth = Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(0, 0, moving ? 0.15 * t : 0)),
                         Eigen::Vector3d(x, 0, 0));
    vision.push(observations(c, origin + static_cast<int64_t>(t * 1e9), truth));
  }
  vision.push(nullptr);
  imu.push(nullptr);
  auto base = granite::VioEstimatorFactory::getVioEstimator(config, c, granite::constants::g, true);
  auto vio = std::dynamic_pointer_cast<granite::MonoVioEstimator>(base);
  EXPECT_TRUE(vio);
  vio->vision_data_queue = &vision;
  vio->imu_data_queue = &imu;
  vio->out_state_queue = &states;
  vio->initialize(Eigen::Vector3d::Zero(), accel_bias);
  vio->join();
  Result result;
  granite::VioStateData::Ptr state;
  while (states.try_pop(state)) if (state) { if (!result.first) result.first = state; result.last = state; }
  result.initialized = vio->metricInitialized();
  if (result.last) {
    result.landmarks = result.last->num_landmarks;
    result.finite = result.last->num_finite_landmarks;
  }
  if (result.last) result.yaw_error = truth.unit_quaternion().angularDistance(result.last->state.T_w_i.unit_quaternion());
  return result;
}
}

TEST(MonoVio, StationaryInputDoesNotClaimMetricInitialization) {
  auto result = run(false);
  EXPECT_FALSE(result.initialized);
  EXPECT_FALSE(result.last);
}

TEST(MonoVio, AcceleratedMotionUsesImuMetricScaleAndTemporalLandmarks) {
  auto result = run(true);
  ASSERT_TRUE(result.last);
  EXPECT_GE(result.finite, 20u);
  EXPECT_LT(result.yaw_error, 0.04);
  const double first_t = (result.first->state.t_ns - 1000000000LL) * 1e-9;
  const double omega = 2 * M_PI / 3.0;
  const double first_x = first_t / omega - std::sin(omega * first_t) / (omega * omega);
  EXPECT_NEAR(result.last->state.T_w_i.translation().x() - result.first->state.T_w_i.translation().x(),
              3.0 / omega - first_x, 0.08);
  EXPECT_LT(std::abs(result.last->state.T_w_i.translation().y()), 0.15);
}

TEST(MonoVio, PreservesInitialAccelerometerBias) {
  const Eigen::Vector3d bias(0.12, -0.08, 0.04);
  auto result = run(true, bias);
  ASSERT_TRUE(result.initialized);
  ASSERT_TRUE(result.first);
  EXPECT_LT((result.first->state.bias_accel - bias).norm(), 0.01);
  EXPECT_LT(result.yaw_error, 0.04);
}

TEST(VioShutdown, ImuEofBeforeFirstImageDoesNotDereferenceNull) {
  granite::VioConfig config;
  auto vio = std::make_shared<granite::KeypointVioEstimator>(granite::constants::g, monoCalibration(), config);
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  auto sample = std::make_shared<granite::ImuData>();
  sample->t_ns = 1000000000;
  sample->accel = -granite::constants::g;
  sample->gyro.setZero();
  imu.push(sample);
  imu.push(nullptr);
  auto frame = std::make_shared<granite::OpticalFlowResult>();
  frame->t_ns = 1050000000;
  frame->observations.resize(1);
  vision.push(frame);
  vision.push(nullptr);
  vio->vision_data_queue = &vision;
  vio->imu_data_queue = &imu;
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  vio->join();
  EXPECT_TRUE(vio->isFinished());
}

TEST(VioShutdown, AbortUnblocksInitialImuWait) {
  granite::VioConfig config;
  auto vio = std::make_shared<granite::KeypointVioEstimator>(granite::constants::g, monoCalibration(), config);
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  vio->vision_data_queue = &vision;
  vio->imu_data_queue = &imu;
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (imu.size() >= 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const bool waiting = imu.size() < 0;
  EXPECT_TRUE(waiting);
  if (waiting) imu.abort();
  else imu.push(nullptr);
  vio->join();
  EXPECT_TRUE(vio->isFinished());
}

TEST(VioShutdown, FirstImuEofSignalsEveryOutputConsumer) {
  granite::VioConfig config;
  auto vio = std::make_shared<granite::KeypointVioEstimator>(granite::constants::g, monoCalibration(), config);
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr> visuals;
  tbb::concurrent_bounded_queue<granite::MargData::Ptr> marginalization;
  imu.push(nullptr);
  vio->vision_data_queue = &vision;
  vio->imu_data_queue = &imu;
  vio->out_vis_queue = &visuals;
  vio->out_marg_queue = &marginalization;
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  vio->join();
  granite::VioVisualizationData::Ptr visual;
  granite::MargData::Ptr marg;
  EXPECT_TRUE(visuals.try_pop(visual));
  EXPECT_FALSE(visual);
  EXPECT_TRUE(marginalization.try_pop(marg));
  EXPECT_FALSE(marg);
}
