#include <granite/vi_estimator/mono_vio_alignment.h>
#include <gtest/gtest.h>

namespace {
struct SyntheticData {
  std::vector<granite::MonoVioFrame> frames;
  Eigen::aligned_deque<granite::ImuData> imu;
  granite::Calibration<double> calibration;
  Sophus::SO3d visual_rotation;
  Eigen::Vector3d gyro_bias;
  double scale;
};

SyntheticData synthetic(bool excited, double scale = 2.3) {
  SyntheticData data;
  data.scale = scale;
  data.visual_rotation = Sophus::SO3d::exp(Eigen::Vector3d(0.3, -0.2, 0.1));
  data.gyro_bias = Eigen::Vector3d(0.02, -0.015, 0.03);
  const Eigen::Vector3d lever(0.05, -0.025, 0.03);
  data.calibration.T_i_c.emplace_back(Sophus::SO3d(), lever);
  const Eigen::Vector3d angular(0.1, -0.2, 0.3);
  auto position = [&](double t) -> Eigen::Vector3d {
    if (!excited) return Eigen::Vector3d(0.3 * t, 0, 0);
    return Eigen::Vector3d(0.3 * t + 0.4 * std::sin(2 * t),
                           0.2 * t + 0.25 * std::cos(3 * t), 0.15 * std::sin(t));
  };
  for (int i = 0; i <= 500; ++i) {
    const double t = i * 0.005;
    const auto rotation = Sophus::SO3d::exp(angular * t);
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    if (excited) acceleration << -1.6 * std::sin(2 * t), -2.25 * std::cos(3 * t), -0.15 * std::sin(t);
    granite::ImuData sample;
    sample.t_ns = 1000000000LL + i * 5000000LL;
    sample.accel = rotation.inverse() * (acceleration - granite::constants::g);
    sample.gyro = angular + data.gyro_bias;
    data.imu.push_back(sample);
  }
  for (int i = 0; i <= 12; ++i) {
    const double t = i * 0.2;
    const auto rotation = Sophus::SO3d::exp(angular * t);
    const auto visual_rotation = data.visual_rotation * rotation;
    Eigen::Vector3d visual_camera = data.visual_rotation * (position(t) + rotation * lever) / scale + Eigen::Vector3d(3, 2, -1);
    granite::MonoVioFrame frame;
    frame.t_ns = 1000000000LL + i * 200000000LL;
    frame.T_v_i = Sophus::SE3d(visual_rotation, visual_camera - visual_rotation * lever);
    data.frames.push_back(frame);
  }
  return data;
}
}

TEST(MonoVioAlignment, RecoversScaleGravityVelocityAndGyroBiasWithLeverArm) {
  auto data = synthetic(true);
  const auto result = granite::alignMonoVisualInertial(data.frames, data.imu, data.calibration,
                                                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->scale, data.scale, 0.04);
  EXPECT_LT((result->gravity - data.visual_rotation * granite::constants::g).norm(), 0.1);
  EXPECT_LT((result->gyro_bias - data.gyro_bias).norm(), 0.005);
  const Eigen::Vector3d initial_velocity(1.1, 0.2, 0.15);
  EXPECT_LT((result->velocities.front() - data.visual_rotation * initial_velocity).norm(), 0.08);
  EXPECT_LT(result->residual, 0.02);
}

TEST(MonoVioAlignment, RejectsConstantVelocityScaleAmbiguity) {
  auto data = synthetic(false);
  EXPECT_FALSE(granite::alignMonoVisualInertial(data.frames, data.imu, data.calibration,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()).has_value());
}

TEST(MonoVioAlignment, RejectsMissingImuAndInvalidTimestamps) {
  auto data = synthetic(true);
  data.imu.clear();
  EXPECT_FALSE(granite::alignMonoVisualInertial(data.frames, data.imu, data.calibration,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()).has_value());
  data = synthetic(true);
  data.frames[3].t_ns = data.frames[2].t_ns;
  EXPECT_FALSE(granite::alignMonoVisualInertial(data.frames, data.imu, data.calibration,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()).has_value());
}
