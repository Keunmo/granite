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

TEST(MonoLandmarkLifecycle, MarginalizesLandmarkWithNoAcceptedObservations) {
  granite::LandmarkDatabase database;
  const granite::TimeCamId host(1000000000, 0), retained_host(2000000000, 0);
  granite::KeypointPosition point;
  point.kf_id = host;
  point.dir = Eigen::Vector2d::Zero();
  point.id = 0;
  point.weight = 1;
  // Mono reprojection gates can reject every observation of a newly born
  // bearing. Its host must still be removable at the next marginalization.
  database.addLandmark(1, point);
  point.kf_id = retained_host;
  point.id = 0.2;
  database.addLandmark(2, point);
  database.addObservation(retained_host, {2, Eigen::Vector2d(212, 200)});
  database.removeKeyframes({host.frame_id}, {host.frame_id}, {});
  EXPECT_FALSE(database.landmarkExists(1));
  ASSERT_TRUE(database.landmarkExists(2));
  EXPECT_EQ(database.numLandmarks(), 1u);
  EXPECT_EQ(database.numObservations(), 1);
  EXPECT_EQ(database.numObservations(2), 1);
  EXPECT_EQ(database.getObservationsOfLandmark(2).size(), 1u);
}

TEST(MonoLandmarkLifecycle, RemovesLandmarkBeforeItsFirstObservation) {
  granite::LandmarkDatabase database;
  granite::KeypointPosition point;
  point.kf_id = granite::TimeCamId(1000000000, 0);
  point.dir = Eigen::Vector2d::Zero();
  point.id = 0;
  point.weight = 1;
  database.addLandmark(1, point);
  EXPECT_EQ(database.numObservations(1), 0);
  EXPECT_TRUE(database.getObservationsOfLandmark(1).empty());
  database.removeLandmark(1);
  EXPECT_FALSE(database.landmarkExists(1));
  EXPECT_EQ(database.numLandmarks(), 0u);
  EXPECT_EQ(database.numObservations(), 0);
  database.removeKeyframes({point.kf_id.frame_id}, {}, {});
  EXPECT_TRUE(database.getHostKfs().empty());
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

namespace {
// Metric trajectory: move, decelerate to a stop, rotate with all finite tracks
// replaced, then accelerate again using the same replacement feature IDs.
struct DegenerateResult {
  std::vector<granite::VioStateData::Ptr> states;
  granite::VioVisualizationData::Ptr last_visual;
  size_t replacement_infinite = 0, replacement_finite = 0;
};
DegenerateResult runDegenerate(double end, bool bearings_only = false,
                              double no_motion_weight = 1e-6,
                              bool empty_gap = false, bool inject_outliers = false,
                              const Eigen::Vector3d& seed_bg_error = Eigen::Vector3d::Zero(),
                              int outlier_count = 8, bool delayed_recovery = false,
                              double resumed_acceleration = 1.0) {
  tbb::global_control control(tbb::global_control::max_allowed_parallelism, 2);
  auto c = monoCalibration();
  granite::VioConfig config;
  config.vio_enforce_realtime = false;
  config.vio_use_lm = true;
  config.vio_max_iterations = 7;
  config.vio_max_kfs = 4;
  config.vio_min_frames_after_kf = 2;
  config.vio_no_motion_reg_weight = no_motion_weight;
  const Eigen::Vector3d bg(0, 0, 0.03), ba(0.02, -0.01, 0.03);
  auto motion = [&](double t) {
    Eigen::Vector3d p, v, a;
    p.setZero(); v.setZero(); a.setZero();
    if (bearings_only) { p.x() = 0.5*t; v.x() = 0.5; }
    else if (t < 1) { p.x() = 0.5*t; v.x() = 0.5; }
    else if (t < 2) { double u=t-1; p.x()=0.5+0.5*u-0.25*u*u; v.x()=0.5-0.5*u; a.x()=-0.5; }
    else if (t < 4) p.x() = 0.75;
    else { double u=t-4; p.x()=0.75+0.5*resumed_acceleration*u*u; v.x()=resumed_acceleration*u; a.x()=resumed_acceleration; }
    return std::make_tuple(Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(0,0,0.2*t)),p),v,a);
  };
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> states;
  constexpr int64_t start = 1000000000;
  for (int i=0; i<=int((end+0.1)*200); ++i) {
    double t=i*0.005;
    auto [pose,vel,acceleration]=motion(t);
    auto sample=std::make_shared<granite::ImuData>();
    sample->t_ns=start+i*5000000LL;
    sample->accel=pose.so3().inverse()*(acceleration-granite::constants::g)+ba;
    sample->gyro=Eigen::Vector3d(0,0,0.2)+bg;
    if(delayed_recovery && t>=2) sample->gyro.z()+=0.02;
    imu.push(sample);
  }
  for (int i=0; i<=int(end*30); ++i) {
    double t=i/30.0;
    auto [pose,vel,acceleration]=motion(t);
    auto frame=observations(c,start+llround(t*1e9),pose);
    if ((t>=2 && !delayed_recovery) || bearings_only) {
      frame->observations[0].clear(); frame->pyramid_levels[0].clear();
      for (int k=0;k<80;++k) {
        Eigen::Vector3d point((k%10-4.5)*0.35+0.75,(k/10-3.5)*0.3,3+(k%7)*0.35);
        Eigen::Vector3d local = (t<4 || bearings_only)
            ? pose.so3().inverse()*(point-Eigen::Vector3d(0.75,0,0))
            : pose.inverse()*point;
        Eigen::Vector4d ray; ray << local,1;
        Eigen::Vector2d pixel;
        if (!c.intrinsics[0].project(ray,pixel)) continue;
        if (inject_outliers && k<outlier_count && t>4.5) pixel+=Eigen::Vector2d(60,-40);
        Eigen::AffineCompact2f affine=Eigen::AffineCompact2f::Identity();
        affine.translation()=pixel.cast<float>();
        frame->observations[0][100+k]=affine;
        frame->pyramid_levels[0][100+k]=k%3;
      }
    }
    if (empty_gap && (delayed_recovery ? (t>=2.2 && t<4) : (t>=2.7 && t<3.2))) {
      frame->observations[0].clear(); frame->pyramid_levels[0].clear();
    }
    vision.push(frame);
  }
  vision.push(nullptr); imu.push(nullptr);
  tbb::concurrent_bounded_queue<granite::VioVisualizationData::Ptr> visuals;
  auto vio=std::make_shared<granite::KeypointVioEstimator>(granite::constants::g,c,config);
  vio->vision_data_queue=&vision; vio->imu_data_queue=&imu; vio->out_state_queue=&states;
  vio->out_vis_queue=&visuals;
  vio->initialize(start,Sophus::SE3d(),Eigen::Vector3d(0.5,0,0),bg+seed_bg_error,ba);
  vio->join();
  DegenerateResult result;
  granite::VioStateData::Ptr state;
  while(states.try_pop(state)) if(state) result.states.push_back(state);
  granite::VioVisualizationData::Ptr visual;
  while(visuals.try_pop(visual)) if(visual) result.last_visual=visual;
  for(int id=100;id<180;++id) if(vio->lmdb.landmarkExists(id)) {
    if(vio->lmdb.getLandmark(id).id==0) ++result.replacement_infinite;
    else ++result.replacement_finite;
  }
  return result;
}
}

TEST(MonoVioRobustness, NewBearingsSurvivePureRotationWithoutFiniteDepth) {
  auto result=runDegenerate(3.5);
  ASSERT_FALSE(result.states.empty());
  EXPECT_GE(result.replacement_infinite,40u);
  EXPECT_EQ(result.replacement_finite,0u);
  const auto& last=result.states.back()->state;
  EXPECT_NEAR(last.T_w_i.translation().x(),0.75,0.03);
  EXPECT_LT((last.T_w_i.so3().inverse()*Sophus::SO3d::exp(Eigen::Vector3d(0,0,0.7))).log().norm(),0.01);
}

TEST(MonoVioRobustness, ResumedTranslationRecoversSameTracksAtMetricScale) {
  auto result=runDegenerate(6);
  ASSERT_FALSE(result.states.empty());
  EXPECT_GE(result.replacement_finite,30u);
  const auto& last=result.states.back()->state;
  EXPECT_NEAR(last.T_w_i.translation().x(),2.75,0.06);
  EXPECT_NEAR(last.vel_w_i.x(),2,0.06);
  EXPECT_LT((last.bias_gyro-Eigen::Vector3d(0,0,0.03)).norm(),0.01);
  EXPECT_LT((last.bias_accel-Eigen::Vector3d(0.02,-0.01,0.03)).norm(),0.03);
}

TEST(MonoVioRobustness, InfinityBearingsDoNotForceZeroMetricVelocity) {
  auto result=runDegenerate(3.5,true,100.0);
  ASSERT_FALSE(result.states.empty());
  EXPECT_GE(result.replacement_infinite,40u);
  EXPECT_EQ(result.replacement_finite,0u);
  const auto& last=result.states.back()->state;
  EXPECT_NEAR(last.T_w_i.translation().x(),1.75,0.02);
  EXPECT_NEAR(last.vel_w_i.x(),0.5,0.01);
}

TEST(MonoVioRobustness, InactiveOldDepthDoesNotClaimCurrentMetricVisualSupport) {
  auto result=runDegenerate(3.5);
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_GE(last.num_connected_infinite_landmarks,40u);
  EXPECT_EQ(last.num_connected_finite_landmarks,0u);
  EXPECT_EQ(last.mono_vio_support,granite::MonoVioSupport::BEARING_ONLY);
}

TEST(MonoVioRobustness, EmptyVisualGapPropagatesImuAndRecoversWithoutReset) {
  auto result=runDegenerate(6,false,1e-6,true);
  ASSERT_FALSE(result.states.empty());
  bool saw_inertial=false, saw_bearings=false;
  for(const auto& data:result.states) {
    EXPECT_TRUE(data->state.T_w_i.matrix().allFinite());
    EXPECT_EQ(data->map_idx,0u);
    if(data->state.t_ns>=3800000000LL && data->state.t_ns<4200000000LL) {
      EXPECT_EQ(data->mono_vio_support,granite::MonoVioSupport::INERTIAL_ONLY);
      saw_inertial=true;
    }
    if(data->mono_vio_support==granite::MonoVioSupport::BEARING_ONLY) saw_bearings=true;
  }
  EXPECT_TRUE(saw_inertial); EXPECT_TRUE(saw_bearings);
  const auto& last=*result.states.back();
  EXPECT_EQ(last.mono_vio_support,granite::MonoVioSupport::METRIC);
  EXPECT_GE(last.num_connected_finite_landmarks,30u);
  EXPECT_NEAR(last.state.T_w_i.translation().x(),2.75,0.06);
}

TEST(MonoVioRobustness, BadReprojectionsDoNotCorruptRecoveryOrBookkeeping) {
  auto result=runDegenerate(6,false,1e-6,false,true);
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_GE(last.num_connected_finite_landmarks,30u);
  EXPECT_NEAR(last.state.T_w_i.translation().x(),2.75,0.08);
  EXPECT_NEAR(last.state.vel_w_i.x(),2,0.08);
}

TEST(MonoVioRobustness, BearingsCorrectInitialGyroscopeBiasError) {
  auto result=runDegenerate(3.5,true,1e-6,false,false,Eigen::Vector3d(0,0,0.01));
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_GE(last.num_connected_infinite_landmarks,40u);
  EXPECT_LT((last.state.bias_gyro-Eigen::Vector3d(0,0,0.03)).norm(),0.005);
  EXPECT_LT((last.state.T_w_i.so3().inverse()*Sophus::SO3d::exp(Eigen::Vector3d(0,0,0.7))).log().norm(),0.01);
  EXPECT_NEAR(last.state.vel_w_i.x(),0.5,0.015);
}

TEST(MonoVioRobustness, CompleteOutlierReplacementKeepsFiniteInertialOutput) {
  auto result=runDegenerate(6,false,1e-6,false,true,Eigen::Vector3d::Zero(),80);
  ASSERT_FALSE(result.states.empty());
  for(const auto& data:result.states) {
    EXPECT_TRUE(data->state.T_w_i.matrix().allFinite());
    EXPECT_TRUE(data->state.vel_w_i.allFinite());
    EXPECT_EQ(data->map_idx,0u);
  }
  const auto& last=*result.states.back();
  EXPECT_EQ(last.mono_vio_support,granite::MonoVioSupport::INERTIAL_ONLY);
  // A sanity bound on metric continuity, not a claim of visual position
  // accuracy while every association is corrupt and translation unobserved.
  EXPECT_NEAR(last.state.T_w_i.translation().x(),2.75,0.25);
  EXPECT_LT((last.state.bias_gyro-Eigen::Vector3d(0,0,0.03)).norm(),0.01);
}

TEST(MonoVioRobustness, VisualInformationRunsWhenOptimizerConvergesImmediately) {
  auto result=runDegenerate(3.5,true);
  ASSERT_TRUE(result.last_visual);
  EXPECT_GT(result.last_visual->negative_entropy_last_frame,0);
}

TEST(MonoVioRobustness, RetainedMetricTracksRecoverAfterInertialYawDrift) {
  auto result=runDegenerate(6,false,1e-6,true,false,Eigen::Vector3d::Zero(),8,true);
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_EQ(last.mono_vio_support,granite::MonoVioSupport::METRIC);
  EXPECT_GE(last.num_connected_finite_landmarks,30u);
  EXPECT_LT((last.state.T_w_i.so3().inverse()*Sophus::SO3d::exp(Eigen::Vector3d(0,0,1.2))).log().norm(),0.03);
  EXPECT_NEAR(last.state.T_w_i.translation().x(),2.75,0.1);
}

TEST(MonoVioRobustness, InfinityTracksRecoverRotationAfterVisualGapAndBiasDrift) {
  auto result=runDegenerate(5,true,1e-6,true,false,Eigen::Vector3d::Zero(),8,true);
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_GE(last.num_connected_infinite_landmarks,40u);
  EXPECT_EQ(last.num_connected_finite_landmarks,0u);
  EXPECT_LT((last.state.T_w_i.so3().inverse()*Sophus::SO3d::exp(Eigen::Vector3d(0,0,1.0))).log().norm(),0.02);
  EXPECT_NEAR(last.state.T_w_i.translation().x(),2.5,0.05);
}

TEST(MonoVioRobustness, SmallTranslationParallaxDoesNotTriggerRotationRecovery) {
  const auto result=runDegenerate(4.8,false,1e-6,false,false,Eigen::Vector3d::Zero(),8,false,0.1);
  ASSERT_FALSE(result.states.empty());
  const auto& last=*result.states.back();
  EXPECT_EQ(last.num_connected_finite_landmarks,0u);
  EXPECT_LT((last.state.T_w_i.so3().inverse()*Sophus::SO3d::exp(Eigen::Vector3d(0,0,0.96))).log().norm(),0.02);
  EXPECT_NEAR(last.state.T_w_i.translation().x(),0.782,0.04);
  EXPECT_NEAR(last.state.vel_w_i.x(),0.08,0.04);
}

TEST(MonoVioRobustness, NearPlaneTranslationDuringImageGapDoesNotFakeRotation) {
  tbb::global_control control(tbb::global_control::max_allowed_parallelism,2);
  const auto c=monoCalibration();
  granite::VioConfig config;
  config.vio_enforce_realtime=false; config.vio_use_lm=true;
  config.vio_max_iterations=7; config.vio_max_kfs=4; config.vio_min_frames_after_kf=2;
  tbb::concurrent_bounded_queue<granite::OpticalFlowResult::Ptr> vision;
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu;
  tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> states;
  constexpr int64_t start=1000000000;
  auto motion=[](double t) {
    Eigen::Vector3d p=Eigen::Vector3d::Zero(),a=Eigen::Vector3d::Zero();
    if(t>=0.7) p.x()=0.005;
    else if(t>=0.5) {
      const double u=(t-0.5)/0.2;
      p.x()=0.005*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5));
      a.x()=0.005/(0.2*0.2)*(60*u-180*u*u+120*u*u*u);
    }
    return std::make_pair(p,a);
  };
  for(int i=0;i<=220;++i) {
    const double t=i*0.005;
    auto sample=std::make_shared<granite::ImuData>();
    sample->t_ns=start+i*5000000LL;
    sample->accel=motion(t).second-granite::constants::g;
    sample->gyro.setZero(); imu.push(sample);
  }
  for(int i=0;i<=30;++i) {
    const double t=i/30.0;
    auto frame=std::make_shared<granite::OpticalFlowResult>();
    frame->t_ns=start+llround(t*1e9);
    frame->observations.resize(1); frame->pyramid_levels.resize(1);
    if(t<0.5 || t>=0.733) for(int k=0;k<80;++k) {
      const Eigen::Vector3d point((k%10-4.5)*0.06,(k/10-3.5)*0.05,0.8);
      Eigen::Vector4d ray;ray<<point-motion(t).first,1;
      Eigen::Vector2d pixel;
      if(!c.intrinsics[0].project(ray,pixel)) continue;
      Eigen::AffineCompact2f observation=Eigen::AffineCompact2f::Identity();
      observation.translation()=pixel.cast<float>();
      frame->observations[0][k]=observation; frame->pyramid_levels[0][k]=0;
    }
    vision.push(frame);
  }
  vision.push(nullptr); imu.push(nullptr);
  auto vio=std::make_shared<granite::KeypointVioEstimator>(granite::constants::g,c,config);
  vio->vision_data_queue=&vision; vio->imu_data_queue=&imu; vio->out_state_queue=&states;
  vio->initialize(start,Sophus::SE3d(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  vio->join();
  granite::VioStateData::Ptr state,last;
  while(states.try_pop(state)) if(state) {
    last=state;
    EXPECT_LT(state->state.T_w_i.so3().log().norm(),0.001);
    EXPECT_LT(state->state.bias_gyro.norm(),0.001);
  }
  ASSERT_TRUE(last);
  EXPECT_NEAR(last->state.T_w_i.translation().x(),0.005,0.001);
}
