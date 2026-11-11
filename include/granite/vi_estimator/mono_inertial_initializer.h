// SPDX-License-Identifier: MIT
/**
 * @file mono_inertial_initializer.h
 * @brief Visual-inertial initializer of the monocular VIO (start at rest or moving start), ported from openvio.
 *
 * Origin: openvio (git@github.com:uvify/openvio.git), src/vio/estimators/initializer.{h,cc} and
 * src/vio/estimators/startup_calibration.{h,cc} at commit 9a05653, with the refinement fix a3ff0e5 (branch
 * pi4-optim: observations that do not project at the start point are left out). Author of the original code:
 * Seungwon Choi <csw3575@snu.ac.kr>.
 *
 * Changes in this port:
 *   - Granite types (OpticalFlowResult, ImuData, Calibration, Sophus poses, z-up world with g = (0, 0, -9.81)).
 *   - Granite's IntegratedImuMeasurement replaces the closed-form preintegration; its residual and Jacobians are
 *     the ones the metric VIO uses.
 *   - The Ceres maximum-likelihood refinement is replaced by a dense Levenberg-Marquardt in Eigen (Granite has no
 *     Ceres). One bias pair is shared by the window (the original has a bias per pose with a random walk).
 *   - The tracks of the moving start are limited to the longest max_tracks tracks.
 *   - The startup time-offset estimate is not ported (the Granite VIO has no online time offset).
 *
 * Papers:
 *   [1] P. Geneva, G. Huang, "Revisiting Visual-Inertial Structure-from-Motion for Odometry and SLAM
 *       Initialization", IEEE RA-L 8(4), 2023 (linear moving start with the gravity-magnitude constraint, then
 *       maximum-likelihood refinement).
 *   [2] T.-C. Dong-Si, A. I. Mourikis, "Estimator initialization in vision-aided inertial navigation with unknown
 *       camera-IMU calibration", IROS 2012 (gravity magnitude by a Lagrange multiplier).
 *   [3] A. Edelman, H. Murakami, "Polynomial roots from companion matrix eigenvalues", Math. Comp. 64(210), 1995.
 *   [4] P. D. Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd ed., 2013,
 *       Ch. 5 (start at rest).
 *
 * License of the original code (openvio):
 *
 * MIT License
 *
 * Copyright (c) 2025 Seungwon Choi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <granite/calibration/calibration.hpp>
#include <granite/optical_flow/optical_flow.h>
#include <granite/utils/imu_types.h>
#include <granite/utils/vio_config.h>

namespace granite {

struct MonoInertialInitOptions {
  double window_time = 2.0;            // Data window [s]
  double excitation_accel_std = 1.0;   // Accel sample std separating still / excited [m/s^2] (0: no start at rest)
  double max_still_disparity = 8.7;    // Mean pixel disparity separating still / moving [px]
  int max_tracks = 50;                 // Tracks of the moving start (the longest ones); 75 % must exist
  int num_poses = 6;                   // Poses spread over the window
  double min_rotation_deg = 10.0;      // Minimum summed gyro rotation over the window [deg]
  int refine_max_iterations = 50;      // Refinement iterations (0: linear solution only)
  double pixel_sigma = 1.0;            // Pixel noise [px]
  double prior_sigma_gyro_bias = 0.05;   // Bias prior of the refinement [rad/s]
  double prior_sigma_accel_bias = 0.10;  // [m/s^2]
  double covariance_scale_orientation = 10.0;  // Inflation of the recovered covariance
  double covariance_scale_velocity = 1.0;
  double covariance_scale_gyro_bias = 10.0;
  double covariance_scale_accel_bias = 9.0;
  double rest_accel_bias_sigma = 0.03;  // Accel bias along gravity from a start at rest [m/s^2] (0: off)
  bool replay_window = false;           // Moving start: start at the oldest pose and replay the window
  // Start at rest (not in openvio): sigma of the horizontal accel bias, coupled with the tilt it hides [m/s^2]
  // (0: the independent sigmas of openvio only)
  double rest_tilt_bias_sigma = 0.0;
  // Moving start (not in openvio): reject a start whose speed sigma (refined covariance, before the inflation) is
  // above this fraction of the speed. Near-constant velocity leaves the metric scale unobservable. (0: no check)
  double max_relative_speed_sigma = 0.05;
  // Not in openvio: start at rest only when the older half moves at most this [px] (0: off)
  double max_rest_disparity = 0.0;
  // Not in openvio: data window of the moving start [s] (0: window_time)
  double moving_window_time = 0.0;
  // Not in openvio: moving start with the tracks of the largest gyro-compensated parallax (not the longest ones)
  bool parallax_tracks = false;
  // Not in openvio: moving start, minimum fraction of the tracks with a point in front of every ray (0: off)
  double min_refined_fraction = 0.0;
  bool moving_start = true;  // false: never use the moving start, wait for a start at rest
  // Not in openvio: start at rest, the still check of the older half uses the accel means of blocks of this time [s]
  // (0: the samples). A block mean removes the motor vibration.
  double rest_average_time = 0.0;
  Eigen::Vector3d gyro_bias_prior = Eigen::Vector3d::Zero();   // Bias linearization point and prior mean
  Eigen::Vector3d accel_bias_prior = Eigen::Vector3d::Zero();

  static MonoInertialInitOptions fromConfig(const VioConfig& config);
};

/// Result of one attempt. The state is at t_ns, in the z-up world (yaw and position are gauge).
/// Covariance order [p, theta, v, bg, ba] with the left orientation increment of Granite (R <- Exp(d) R).
struct MonoInertialInitResult {
  bool success = false;
  std::string method;  // "at rest", "moving" or empty when nothing was tried
  std::string reason;  // Why the attempt stopped (empty on success)

  int64_t t_ns = -1;
  Sophus::SE3d T_w_i;
  Eigen::Vector3d vel_w_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
  int64_t newest_t_ns = -1;  // Newest image of the window (the data the decision used)

  // Diagnostics
  double disparity_old = -1, disparity_new = -1;
  double accel_std_old = -1, accel_std_new = -1;
  int num_tracks = 0, num_valid_tracks = 0, num_poses = 0;
  double rotation_deg = -1;
  double linear_speed = -1;  // |v| of the linear solution at the oldest pose
  double relative_speed_sigma = -1;  // Moving: speed sigma / speed of the refined start state
  double refined_speed = -1;         // Moving: |v| of the refined start state
  int num_refined_features = 0, num_unprojected_observations = 0, refine_iterations = 0;
  double refine_initial_cost = 0, refine_final_cost = 0;
  bool rest_bias_applied = false;
  double rest_bias = 0;

  std::string summary() const;
};

class MonoInertialInitializer {
 public:
  MonoInertialInitializer(const MonoInertialInitOptions& options, const Calibration<double>& calib,
                          const Eigen::Vector3d& gravity);

  /// IMU sample after the IMU calibration (the same correction as the metric VIO)
  void addImu(const ImuData& sample);
  /// Tracked features of one image (camera 0)
  void addFrame(const OpticalFlowResult& frame);
  /// Try to initialize with the data so far
  MonoInertialInitResult tryInitialize();

 private:
  struct Track {
    std::vector<int64_t> t_ns;
    std::vector<Eigen::Vector2d> pixels;
    std::vector<Eigen::Vector3d> bearings;  // Unit rays in the camera frame
  };

  bool startAtRest(MonoInertialInitResult& result) const;
  bool startMoving(MonoInertialInitResult& result);
  void applyRestAccelBias(MonoInertialInitResult& result) const;
  void dropMeasurementsUpTo(int64_t t_ns);
  int64_t newestImageTime() const;
  double movingWindowTime() const;
  double bufferTime() const;
  void meanTrackMotion(int64_t before, int64_t after, double* mean, int* count) const;

  MonoInertialInitOptions options;
  Calibration<double> calib;
  Eigen::Vector3d gravity;     // World gravity acceleration (z up: (0, 0, -G))
  double gravity_magnitude;
  Eigen::aligned_deque<ImuData> imu;   // Window IMU buffer
  std::map<int, Track> tracks;         // Feature tracks of the window by id

  // Startup data for the accel bias at rest: every accel sample and the mean ray motion per image
  std::vector<std::pair<int64_t, Eigen::Vector3d>> startup_accel;
  std::vector<std::pair<int64_t, double>> startup_motion;
  std::map<int, Eigen::Vector3d> previous_rays;
};

}  // namespace granite
