// SPDX-License-Identifier: MIT
#include <random>
#include <gtest/gtest.h>
#include <granite/device/pi_sensor_utils.h>

TEST(PiSensorUtils, UnpacksRaw10MostSignificantBytes) {
  // Two rows of 4 pixels, padded to a 8-byte stride.
  const uint8_t packed[16] = {10, 20, 30, 40, 0xff, 0, 0, 0,
                              50, 60, 70, 80, 0x00, 0, 0, 0};
  const cv::Mat bayer = granite::unpackRaw10Msb(packed, 4, 2, 8);
  ASSERT_EQ(bayer.cols, 4);
  ASSERT_EQ(bayer.rows, 2);
  EXPECT_EQ(bayer.at<uint8_t>(0, 0), 10);
  EXPECT_EQ(bayer.at<uint8_t>(0, 3), 40);
  EXPECT_EQ(bayer.at<uint8_t>(1, 0), 50);
  EXPECT_EQ(bayer.at<uint8_t>(1, 3), 80);
  EXPECT_THROW(granite::unpackRaw10Msb(packed, 6, 2, 8), std::invalid_argument);
  EXPECT_THROW(granite::unpackRaw10Msb(packed, 8, 2, 8), std::invalid_argument);
}

TEST(PiSensorUtils, HalfResolutionAveragesBayerQuads) {
  cv::Mat bayer(4, 4, CV_8UC1);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      // G R / B G with G=100, R=40, B=60 averages to 75.
      const bool green = (x + y) % 2 == 0;
      bayer.at<uint8_t>(y, x) = green ? 100 : (y % 2 == 0 ? 40 : 60);
    }
  }
  const cv::Mat gray = granite::grbgToGray(bayer, 2);
  ASSERT_EQ(gray.size(), cv::Size(2, 2));
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 2; ++x) EXPECT_EQ(gray.at<uint8_t>(y, x), 75);
  EXPECT_EQ(granite::grbgToGray(bayer, 1).size(), cv::Size(4, 4));
}

TEST(PiSensorUtils, OddDownscaleHasNoBayerPattern) {
  // A uniform scene through G R / B G: a plain 3x3 average would give
  // different values in neighbouring output pixels.
  cv::Mat bayer(12, 24, CV_8UC1);
  for (int y = 0; y < bayer.rows; ++y)
    for (int x = 0; x < bayer.cols; ++x)
      bayer.at<uint8_t>(y, x) = (x + y) % 2 == 0 ? 100 : (y % 2 == 0 ? 40 : 60);
  const cv::Mat gray = granite::grbgToGray(bayer, 3);
  ASSERT_EQ(gray.size(), cv::Size(8, 4));
  for (int y = 0; y < gray.rows; ++y)
    for (int x = 0; x < gray.cols; ++x) EXPECT_EQ(gray.at<uint8_t>(y, x), 75);
  EXPECT_THROW(granite::grbgToGray(cv::Mat(12, 15, CV_8UC1), 3), std::invalid_argument);
}

TEST(PiSensorUtils, ScalesCalibrationToHalfResolution) {
  granite::Calibration<double> calib;
  Eigen::Matrix<double, 4, 1> params;
  params << 800, 810, 639.5, 359.5;
  granite::GenericCamera<double> camera;
  camera.variant = granite::PinholeCamera<double>(params);
  calib.intrinsics = {camera};
  calib.resolution = {Eigen::Vector2i(1280, 720)};
  granite::scaleCalibrationHalf(calib);
  const auto scaled = calib.intrinsics[0].getParam();
  EXPECT_DOUBLE_EQ(scaled[0], 400);
  EXPECT_DOUBLE_EQ(scaled[1], 405);
  EXPECT_DOUBLE_EQ(scaled[2], 319.5);
  EXPECT_DOUBLE_EQ(scaled[3], 179.5);
  EXPECT_EQ(calib.resolution[0], Eigen::Vector2i(640, 360));
}

TEST(PiSensorUtils, ScalesCalibrationToOneThird) {
  granite::Calibration<double> calib;
  Eigen::Matrix<double, 8, 1> params;
  params << 300, 300, 931, 608, 0.1, 0.01, 0, 0;
  granite::GenericCamera<double> camera;
  camera.variant = granite::KannalaBrandtCamera4<double>(params);
  calib.intrinsics = {camera};
  calib.resolution = {Eigen::Vector2i(1920, 1200)};
  granite::scaleCalibration(calib, 3);
  const auto scaled = calib.intrinsics[0].getParam();
  EXPECT_DOUBLE_EQ(scaled[0], 100);
  EXPECT_DOUBLE_EQ(scaled[2], (931 + 0.5) / 3 - 0.5);
  EXPECT_DOUBLE_EQ(scaled[3], (608 + 0.5) / 3 - 0.5);
  EXPECT_DOUBLE_EQ(scaled[4], 0.1);
  EXPECT_EQ(calib.resolution[0], Eigen::Vector2i(640, 400));
}

TEST(PiSensorUtils, DecodesMpuFifoSample) {
  // accel x=+8192, y=-8192, z=1; gyro x=+328, y=-328, z=0
  const uint8_t sample[12] = {0x20, 0x00, 0xe0, 0x00, 0x00, 0x01,
                              0x01, 0x48, 0xfe, 0xb8, 0x00, 0x00};
  Eigen::Vector3d accel, gyro;
  granite::decodeMpuFifoSample(sample, 0.5, 0.25, accel, gyro);
  EXPECT_EQ(accel, Eigen::Vector3d(4096, -4096, 0.5));
  EXPECT_EQ(gyro, Eigen::Vector3d(82, -82, 0));
}

TEST(PiSensorUtils, FifoClockFollowsSensorPeriodAndReadBounds) {
  // The sensor runs 1% slower than nominal. Like a CP2112 read, the count
  // snapshot happens at a random point inside a 5 ms transfer.
  const double true_period = 5.05e6;
  const int64_t sensor_start = 1'000'000'000;
  std::mt19937 random(7);
  std::uniform_real_distribution<double> half_transfer(1e6, 4e6);
  granite::FifoSampleClock clock(5e6);
  uint64_t produced = 0, delivered = 0;
  int64_t previous = 0, previous_interval_end = 0;
  double max_error_ns = 0;
  for (int64_t before = sensor_start; before < sensor_start + 6'000'000'000; before += 20'000'000) {
    const int64_t snapshot = before + static_cast<int64_t>(half_transfer(random));
    const int64_t after = snapshot + static_cast<int64_t>(half_transfer(random));
    while (sensor_start + true_period * produced <= snapshot) ++produced;
    const auto times = clock.assign(before, after, produced - delivered);
    for (size_t i = 0; i < times.size(); ++i) {
      EXPECT_GT(times[i], previous);
      previous = times[i];
      const double truth = sensor_start + true_period * (delivered + i);
      if (before > sensor_start + 3'000'000'000) {
        max_error_ns = std::max(max_error_ns, std::abs(times[i] - truth));
        // Preintegration needs intervals close to the sensor period.
        EXPECT_NEAR(times[i] - previous_interval_end, true_period, 0.03 * true_period);
      }
      previous_interval_end = times[i];
    }
    delivered = produced;
  }
  EXPECT_NEAR(clock.periodNs(), true_period, 0.01e6);
  EXPECT_LT(max_error_ns, 2e6);
}

TEST(PiSensorUtils, AutoExposureUsesExposureBeforeGain) {
  const granite::ExposureSetting dark{100, 100};
  const auto brighter = granite::nextAutoExposure(dark, 25, 100, 800, 100, 800);
  EXPECT_EQ(brighter.lines, 200);
  EXPECT_EQ(brighter.gain, 100);
  const granite::ExposureSetting limited{800, 100};
  const auto gained = granite::nextAutoExposure(limited, 25, 100, 800, 100, 800);
  EXPECT_EQ(gained.lines, 800);
  EXPECT_EQ(gained.gain, 200);
  EXPECT_EQ(granite::nextAutoExposure(limited, 98, 100, 800, 100, 800).lines, 800);
}

TEST(PiSensorUtils, B0429GainMapsDecibelsToDriverValues) {
  EXPECT_EQ(granite::b0429DriverGainFromDb(-1), 100);
  EXPECT_EQ(granite::b0429DriverGainFromDb(0), 100);
  // 6 dB is about 2x: 100 + 382 x (10^(6.32/20) - 1).
  EXPECT_EQ(granite::b0429DriverGainFromDb(6), 509);
  EXPECT_EQ(granite::b0429DriverGainFromDb(20), 1122);
  for (const double gain_db : {1.0, 3.0, 6.0, 9.0, 11.0})
    EXPECT_NEAR(granite::b0429DbFromDriverGain(granite::b0429DriverGainFromDb(gain_db)),
                gain_db, 0.05);
}
