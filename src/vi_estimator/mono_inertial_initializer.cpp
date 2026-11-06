// SPDX-License-Identifier: MIT
/**
 * @file mono_inertial_initializer.cpp
 * @brief Implements the visual-inertial initializer of the monocular VIO, ported from openvio.
 *
 * Origin, changes, papers and the MIT license of the original code (Copyright (c) 2025 Seungwon Choi):
 * see include/granite/vi_estimator/mono_inertial_initializer.h.
 *
 * Conventions: the world is z up with gravity acceleration g = (0, 0, -G). A pose increment is
 * [dp, dtheta, dv] with p <- p + dp, R <- Exp(dtheta) R, v <- v + dv (Granite's PoseVelState::applyInc).
 */

#include <granite/vi_estimator/mono_inertial_initializer.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

#include <Eigen/Eigenvalues>

#include <granite/imu/preintegration.h>
#include <granite/utils/sophus_utils.hpp>

namespace granite {
namespace {

using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Poly = std::vector<double>;  // Polynomial coefficients, ascending degree

constexpr int64_t kNoBound = std::numeric_limits<int64_t>::min();

double seconds(int64_t ns) { return 1e-9 * static_cast<double>(ns); }

Poly polyAdd(const Poly& a, const Poly& b) {
  Poly r(std::max(a.size(), b.size()), 0.0);
  for (size_t i = 0; i < a.size(); ++i) r[i] += a[i];
  for (size_t i = 0; i < b.size(); ++i) r[i] += b[i];
  return r;
}

Poly polyScale(const Poly& a, double s) {
  Poly r = a;
  for (double& c : r) c *= s;
  return r;
}

Poly polyMul(const Poly& a, const Poly& b) {
  Poly r(a.size() + b.size() - 1, 0.0);
  for (size_t i = 0; i < a.size(); ++i)
    for (size_t j = 0; j < b.size(); ++j) r[i + j] += a[i] * b[j];
  return r;
}

// Two unit rows spanning the plane orthogonal to a unit ray: T * ray = 0
Eigen::Matrix<double, 2, 3> orthogonalRows(const Eigen::Vector3d& bearing) {
  int axis = 0;
  bearing.cwiseAbs().minCoeff(&axis);
  const Eigen::Vector3d t1 = bearing.cross(Eigen::Vector3d::Unit(axis)).normalized();
  const Eigen::Vector3d t2 = bearing.cross(t1).normalized();
  Eigen::Matrix<double, 2, 3> T;
  T.row(0) = t1.transpose();
  T.row(1) = t2.transpose();
  return T;
}

double sampleStd(const std::vector<Eigen::Vector3d>& samples, const Eigen::Vector3d& mean) {
  double sum = 0.0;
  for (const auto& s : samples) sum += (s - mean).squaredNorm();
  return std::sqrt(sum / static_cast<double>(samples.size() - 1));
}

/**
 * Polynomial whose roots are the Lagrange multipliers of |g| = G ([2]): for min |A1 x + A2 g - b|^2 s.t. |g| = G
 * with the reduced normal equations D g = d, the stationary points satisfy (D - l I) g = d and |g| = G, i.e.
 * G^2 det(D - l I)^2 - |adj(D - l I) d|^2 = 0. Monic coefficients, highest degree first.
 */
Eigen::Matrix<double, 7, 1> gravityConstraintPolynomial(const Eigen::Matrix3d& D, const Eigen::Vector3d& d,
                                                        double gravity_magnitude) {
  Poly M[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) M[i][j] = (i == j) ? Poly{D(i, j), -1.0} : Poly{D(i, j)};
  Poly C[3][3];
  for (int i = 0; i < 3; ++i) {
    const int r0 = (i == 0) ? 1 : 0;
    const int r1 = (i == 2) ? 1 : 2;
    for (int j = 0; j < 3; ++j) {
      const int c0 = (j == 0) ? 1 : 0;
      const int c1 = (j == 2) ? 1 : 2;
      const Poly minor = polyAdd(polyMul(M[r0][c0], M[r1][c1]), polyScale(polyMul(M[r0][c1], M[r1][c0]), -1.0));
      C[i][j] = ((i + j) % 2 == 0) ? minor : polyScale(minor, -1.0);
    }
  }
  Poly det{0.0};
  for (int j = 0; j < 3; ++j) det = polyAdd(det, polyMul(M[0][j], C[0][j]));
  Poly adj_norm2{0.0};
  for (int i = 0; i < 3; ++i) {
    Poly u{0.0};
    for (int j = 0; j < 3; ++j) u = polyAdd(u, polyScale(C[j][i], d(j)));
    adj_norm2 = polyAdd(adj_norm2, polyMul(u, u));
  }
  const double g2 = gravity_magnitude * gravity_magnitude;
  Poly p = polyAdd(polyScale(polyMul(det, det), g2), polyScale(adj_norm2, -1.0));
  p.resize(7, 0.0);
  Eigen::Matrix<double, 7, 1> coeff;
  for (int k = 0; k <= 6; ++k) coeff(k) = p[6 - k] / p[6];
  coeff(0) = 1.0;
  return coeff;
}

/**
 * min |A x - b|^2 subject to |g| = G for g = x.tail(3) ([2]). The multipliers are the roots of
 * gravityConstraintPolynomial, found as the eigenvalues of its companion matrix ([3]); the smallest real root is
 * the minimum on the sphere.
 */
bool solveGravityConstrainedLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                         double gravity_magnitude, Eigen::VectorXd& x, std::string& reason) {
  constexpr double kMaxGravityError = 1e-3;  // Accepted | |g| - G | [m/s^2]
  const Eigen::Index n = A.cols();
  const Eigen::MatrixXd A1 = A.leftCols(n - 3);
  const Eigen::MatrixXd A2 = A.rightCols(3);
  const Eigen::MatrixXd N11_inv = (A1.transpose() * A1).llt().solve(Eigen::MatrixXd::Identity(n - 3, n - 3));
  const Eigen::MatrixXd N21 = A2.transpose() * A1;
  const Eigen::Matrix3d D = A2.transpose() * A2 - N21 * N11_inv * N21.transpose();
  const Eigen::Vector3d d = A2.transpose() * b - N21 * N11_inv * (A1.transpose() * b);

  const Eigen::Matrix<double, 7, 1> coeff = gravityConstraintPolynomial(D, d, gravity_magnitude);
  Eigen::Matrix<double, 6, 6> companion = Eigen::Matrix<double, 6, 6>::Zero();
  companion.diagonal(-1).setOnes();
  for (int row = 0; row < 6; ++row) companion(row, 5) = -coeff(6 - row);
  if (!companion.allFinite() || Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>>(companion).rank() != 6) {
    reason = "gravity polynomial companion matrix is rank deficient";
    return false;
  }
  const Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> eigen(companion, false);
  if (eigen.info() != Eigen::Success) {
    reason = "gravity polynomial eigen decomposition failed";
    return false;
  }
  bool found = false;
  double lambda = 0.0;
  for (int k = 0; k < 6; ++k) {
    const std::complex<double> root = eigen.eigenvalues()(k);
    if (root.imag() == 0.0 && (!found || root.real() < lambda)) {
      found = true;
      lambda = root.real();
    }
  }
  if (!found) {
    reason = "gravity polynomial has no real root";
    return false;
  }
  const Eigen::Vector3d g = (D - lambda * Eigen::Matrix3d::Identity()).fullPivLu().solve(d);
  x.resize(n);
  x.head(n - 3) = N11_inv * (A1.transpose() * b - A1.transpose() * (A2 * g));
  x.tail(3) = g;
  if (!x.allFinite() || std::abs(g.norm() - gravity_magnitude) > kMaxGravityError) {
    std::ostringstream text;
    text << "gravity magnitude error " << std::abs(g.norm() - gravity_magnitude) << " > " << kMaxGravityError;
    reason = text.str();
    return false;
  }
  return true;
}

// Granite preintegration from t0 to t1 with the same sample handling as the metric VIO; false when the IMU does
// not cover the interval
bool preintegrate(const Eigen::aligned_deque<ImuData>& imu, int64_t t0, int64_t t1, const Eigen::Vector3d& bg,
                  const Eigen::Vector3d& ba, const Eigen::Vector3d& accel_cov, const Eigen::Vector3d& gyro_cov,
                  IntegratedImuMeasurement& out) {
  if (imu.empty() || imu.front().t_ns > t0 || imu.back().t_ns < t1 || t1 <= t0) return false;
  out = IntegratedImuMeasurement(t0, bg, ba);
  size_t k = 0;
  while (k < imu.size() && imu[k].t_ns <= t0) ++k;
  for (; k < imu.size(); ++k) {
    ImuData sample = imu[k];
    if (sample.t_ns >= t1) {
      sample.t_ns = t1;
      out.integrate(sample, accel_cov, gyro_cov);
      break;
    }
    out.integrate(sample, accel_cov, gyro_cov);
  }
  return out.get_dt_ns() == t1 - t0;
}

}  // namespace

MonoInertialInitOptions MonoInertialInitOptions::fromConfig(const VioConfig& config) {
  MonoInertialInitOptions o;
  o.window_time = config.vio_mono_init_window_time;
  o.excitation_accel_std = config.vio_mono_init_excitation_accel_std;
  o.max_still_disparity = config.vio_mono_init_max_still_disparity;
  o.max_tracks = config.vio_mono_init_max_tracks;
  o.num_poses = config.vio_mono_init_num_poses;
  o.min_rotation_deg = config.vio_mono_init_min_rotation_deg;
  o.refine_max_iterations = config.vio_mono_init_refine_max_iterations;
  o.pixel_sigma = config.vio_mono_init_pixel_sigma;
  o.prior_sigma_gyro_bias = config.vio_mono_init_prior_sigma_gyro_bias;
  o.prior_sigma_accel_bias = config.vio_mono_init_prior_sigma_accel_bias;
  o.covariance_scale_orientation = config.vio_mono_init_covariance_scale_orientation;
  o.covariance_scale_velocity = config.vio_mono_init_covariance_scale_velocity;
  o.covariance_scale_gyro_bias = config.vio_mono_init_covariance_scale_gyro_bias;
  o.covariance_scale_accel_bias = config.vio_mono_init_covariance_scale_accel_bias;
  o.rest_accel_bias_sigma = config.vio_mono_init_rest_accel_bias_sigma;
  o.replay_window = config.vio_mono_init_replay_window;
  o.rest_tilt_bias_sigma = config.vio_mono_init_rest_tilt_bias_sigma;
  o.max_relative_speed_sigma = config.vio_mono_init_max_relative_speed_sigma;
  o.max_rest_disparity = config.vio_mono_init_max_rest_disparity;
  o.moving_window_time = config.vio_mono_init_moving_window_time;
  o.parallax_tracks = config.vio_mono_init_parallax_tracks;
  o.min_refined_fraction = config.vio_mono_init_min_refined_fraction;
  return o;
}

std::string MonoInertialInitResult::summary() const {
  std::ostringstream s;
  s << std::fixed << std::setprecision(3) << "method=\"" << method << "\" success=" << success;
  if (!success) s << " reason=\"" << reason << "\"";
  s << " t=" << seconds(t_ns) << " newest=" << seconds(newest_t_ns) << " disparity_old=" << disparity_old
    << " disparity_new=" << disparity_new << " tracks=" << num_tracks;
  if (method == "at rest") s << " accel_std_old=" << accel_std_old << " accel_std_new=" << accel_std_new;
  if (method == "moving")
    s << " valid_tracks=" << num_valid_tracks << " poses=" << num_poses << " rotation_deg=" << rotation_deg
      << " linear_speed=" << linear_speed << " refined_speed=" << refined_speed
      << " relative_speed_sigma=" << relative_speed_sigma
      << " refined_features=" << num_refined_features
      << " unprojected=" << num_unprojected_observations << " iterations=" << refine_iterations
      << " cost=" << refine_initial_cost << "->" << refine_final_cost;
  if (success) {
    s << std::setprecision(4) << " v=" << vel_w_i.transpose() << " bg=" << bias_gyro.transpose() << " ba=" << bias_accel.transpose()
      << " sigma_v=" << covariance.block<3, 3>(6, 6).diagonal().cwiseSqrt().transpose()
      << " sigma_ba=" << covariance.block<3, 3>(12, 12).diagonal().cwiseSqrt().transpose()
      << " rest_bias=" << (rest_bias_applied ? rest_bias : std::nan(""));
  }
  return s.str();
}

MonoInertialInitializer::MonoInertialInitializer(const MonoInertialInitOptions& options,
                                                 const Calibration<double>& calib, const Eigen::Vector3d& gravity)
    : options(options), calib(calib), gravity(gravity), gravity_magnitude(gravity.norm()) {}

void MonoInertialInitializer::addImu(const ImuData& sample) {
  imu.push_back(sample);
  const int64_t oldest = sample.t_ns - static_cast<int64_t>((bufferTime() + 0.10) * 1e9);
  while (!imu.empty() && imu.front().t_ns < oldest) imu.pop_front();
  startup_accel.emplace_back(sample.t_ns, sample.accel);
}

void MonoInertialInitializer::addFrame(const OpticalFlowResult& frame) {
  std::map<int, Eigen::Vector3d> rays;
  for (const auto& observation : frame.observations.at(0)) {
    const Eigen::Vector2d pixel = observation.second.translation().cast<double>();
    Eigen::Vector4d ray;
    if (!calib.intrinsics[0].unproject(pixel, ray) || !ray.head<3>().allFinite()) continue;
    const Eigen::Vector3d bearing = ray.head<3>().normalized();
    Track& track = tracks[observation.first];
    track.t_ns.push_back(frame.t_ns);
    track.pixels.push_back(pixel);
    track.bearings.push_back(bearing);
    rays[observation.first] = bearing;
  }
  // Mean ray motion of the image [rad] (rest check of the accel-bias estimate)
  if (!previous_rays.empty()) {
    double angle_sum = 0.0;
    size_t matched = 0;
    for (const auto& entry : rays) {
      const auto it = previous_rays.find(entry.first);
      if (it == previous_rays.end()) continue;
      angle_sum += std::acos(std::min(1.0, entry.second.dot(it->second)));
      ++matched;
    }
    if (matched >= 15) startup_motion.emplace_back(frame.t_ns, angle_sum / static_cast<double>(matched));
  }
  previous_rays.swap(rays);
}

void MonoInertialInitializer::dropMeasurementsUpTo(int64_t t_ns) {
  for (auto it = tracks.begin(); it != tracks.end();) {
    Track& track = it->second;
    size_t keep = 0;
    for (size_t i = 0; i < track.t_ns.size(); ++i) {
      if (track.t_ns[i] > t_ns) {
        track.t_ns[keep] = track.t_ns[i];
        track.pixels[keep] = track.pixels[i];
        track.bearings[keep] = track.bearings[i];
        ++keep;
      }
    }
    track.t_ns.resize(keep);
    track.pixels.resize(keep);
    track.bearings.resize(keep);
    if (keep == 0) it = tracks.erase(it);
    else ++it;
  }
}

double MonoInertialInitializer::movingWindowTime() const {
  return options.moving_window_time > 0.0 ? options.moving_window_time : options.window_time;
}

double MonoInertialInitializer::bufferTime() const { return std::max(options.window_time, movingWindowTime()); }

int64_t MonoInertialInitializer::newestImageTime() const {
  int64_t newest = -1;
  for (const auto& entry : tracks)
    for (int64_t t : entry.second.t_ns) newest = std::max(newest, t);
  return newest;
}

// Per track: pixel distance from its first observation after `after` to its last later observation before
// `before` (kNoBound: no bound)
void MonoInertialInitializer::meanTrackMotion(int64_t before, int64_t after, double* mean, int* count) const {
  double sum = 0.0;
  int n = 0;
  for (const auto& entry : tracks) {
    const Track& track = entry.second;
    const size_t num = track.t_ns.size();
    if (num < 2) continue;
    size_t first = num;
    for (size_t i = 0; i < num; ++i) {
      if (after == kNoBound || track.t_ns[i] > after) {
        first = i;
        break;
      }
    }
    size_t last = num;
    for (size_t i = first + 1; i < num; ++i)
      if (before == kNoBound || track.t_ns[i] < before) last = i;
    if (first == num || last == num) continue;
    sum += (track.pixels[last] - track.pixels[first]).norm();
    ++n;
  }
  *count = n;
  *mean = n > 0 ? sum / n : -1.0;
}


MonoInertialInitResult MonoInertialInitializer::tryInitialize() {
  constexpr int kMinDisparityTracks = 15;  // Tracks needed by each disparity statistic
  MonoInertialInitResult result;
  const int64_t newest = newestImageTime();
  result.newest_t_ns = newest;
  const int64_t window_ns = static_cast<int64_t>(options.window_time * 1e9);
  const int64_t oldest = newest - window_ns - 100000000LL;
  if (newest < 0 || startup_accel.empty() || startup_accel.front().first > oldest) {
    result.reason = "no image window yet";
    return result;
  }
  // The buffer keeps the longer of the two windows; the still / moving decision uses window_time
  const int64_t oldest_buffer = newest - static_cast<int64_t>(bufferTime() * 1e9) - 100000000LL;
  dropMeasurementsUpTo(oldest_buffer);
  while (!imu.empty() && imu.front().t_ns < oldest_buffer) imu.pop_front();
  result.num_tracks = static_cast<int>(tracks.size());

  // Visual motion of the older and the newer half of the window
  bool moving_old = false, moving_new = false;
  if (options.max_still_disparity > 0.0) {
    const int64_t mid = newest - window_ns / 2;
    int count_old = 0, count_new = 0;
    meanTrackMotion(mid, oldest_buffer == oldest ? kNoBound : oldest, &result.disparity_old, &count_old);
    meanTrackMotion(newest, mid, &result.disparity_new, &count_new);
    if (count_old < kMinDisparityTracks || count_new < kMinDisparityTracks) {
      result.reason = "not enough tracks for disparity";
      return result;
    }
    moving_old = result.disparity_old > options.max_still_disparity;
    moving_new = result.disparity_new > options.max_still_disparity;
  }

  // Still, then moving (a jerk): start at rest. Moving: the moving start. Still all along: wait.
  // With max_rest_disparity, "still" also needs an older half at rest: a constant velocity has no IMU excitation
  // and can stay below max_still_disparity (2.8 px at 4 m/s in the Unreal city sequence, 0.004 px at rest).
  const bool rest_old = options.max_rest_disparity <= 0.0 || result.disparity_old <= options.max_rest_disparity;
  const bool jerk = !moving_old && moving_new && rest_old;
  const bool still = !moving_old && !moving_new && rest_old;
  if (jerk && options.excitation_accel_std > 0.0) {
    result.method = "at rest";
    result.success = startAtRest(result);
  } else if (!still) {
    result.method = "moving";
    result.success = startMoving(result);
  } else {
    result.reason = "platform still (no visual jerk)";
  }
  if (result.success) applyRestAccelBias(result);
  return result;
}

bool MonoInertialInitializer::startAtRest(MonoInertialInitResult& result) const {
  if (imu.size() < 2) {
    result.reason = "not enough IMU samples";
    return false;
  }
  const int64_t t_end = imu.back().t_ns;
  const int64_t W = static_cast<int64_t>(options.window_time * 1e9);
  if (t_end - imu.front().t_ns < W) {
    result.reason = "IMU buffer shorter than the window";
    return false;
  }
  // [4]: the older half (t_end - W, t_end - W/2] must be still and the newer half excited
  std::vector<Eigen::Vector3d> accel_newer, accel_older, gyro_older;
  int64_t older_end = -1;
  for (const ImuData& s : imu) {
    if (s.t_ns > t_end - W / 2 && s.t_ns <= t_end) accel_newer.push_back(s.accel);
    if (s.t_ns > t_end - W && s.t_ns <= t_end - W / 2) {
      accel_older.push_back(s.accel);
      gyro_older.push_back(s.gyro);
      older_end = s.t_ns;
    }
  }
  if (accel_newer.size() < 2 || accel_older.size() < 2) {
    result.reason = "not enough IMU samples in a half window";
    return false;
  }
  auto mean_of = [](const std::vector<Eigen::Vector3d>& values) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& v : values) sum += v;
    return Eigen::Vector3d(sum / static_cast<double>(values.size()));
  };
  const Eigen::Vector3d a_newer = mean_of(accel_newer);
  const Eigen::Vector3d a_older = mean_of(accel_older);
  const Eigen::Vector3d w_older = mean_of(gyro_older);
  result.accel_std_new = sampleStd(accel_newer, a_newer);
  result.accel_std_old = sampleStd(accel_older, a_older);
  if (result.accel_std_new < options.excitation_accel_std) {
    result.reason = "no IMU excitation in the newer half";
    return false;
  }
  if (result.accel_std_old > options.excitation_accel_std) {
    result.reason = "IMU excitation in the older half (was not stationary)";
    return false;
  }
  // The state is at the newest image of the still half (the metric VIO starts at an image)
  int64_t t_state = -1;
  for (const auto& entry : tracks)
    for (int64_t t : entry.second.t_ns)
      if (t <= older_end) t_state = std::max(t_state, t);
  if (t_state < 0) {
    result.reason = "no image in the still half";
    return false;
  }
  // At rest the mean specific force is gravity plus the accel bias, the mean rate the gyro bias
  const Sophus::SO3d R_w_i(Eigen::Quaterniond::FromTwoVectors(a_older, -gravity));
  result.t_ns = t_state;
  result.T_w_i = Sophus::SE3d(R_w_i, Eigen::Vector3d::Zero());
  result.vel_w_i.setZero();
  result.bias_gyro = w_older;
  result.bias_accel = a_older - R_w_i.inverse() * (-gravity);
  // Sigmas [p, theta, v, bg, ba] of the original
  const double sigma[5] = {0.05, 0.02, 0.01, 0.02, 0.02};
  result.covariance.setZero();
  for (int k = 0; k < 5; ++k) result.covariance.block<3, 3>(3 * k, 3 * k) = sigma[k] * sigma[k] * Eigen::Matrix3d::Identity();
  if (options.rest_tilt_bias_sigma > 0.0) {
    // Not in openvio: at rest a horizontal accel bias db (IMU frame, orthogonal to the specific force) and the tilt
    // dtheta = [e_z]x R_w_i db / G give the same specific force. The pair is one unobservable direction per axis:
    // its covariance is sigma^2 G G^T with G = [dtheta / ds; db / ds].
    const Eigen::Vector3d f = a_older.normalized();
    Eigen::Matrix<double, 3, 2> B;
    B.col(0) = f.unitOrthogonal();
    B.col(1) = f.cross(B.col(0));
    Eigen::Matrix<double, 15, 2> G = Eigen::Matrix<double, 15, 2>::Zero();
    G.block<3, 2>(3, 0) = Sophus::SO3d::hat(Eigen::Vector3d::UnitZ()) * R_w_i.matrix() * B / gravity_magnitude;
    G.block<3, 2>(12, 0) = B;
    result.covariance += options.rest_tilt_bias_sigma * options.rest_tilt_bias_sigma * G * G.transpose();
  }
  return true;
}

namespace {

struct Observation {
  int pose;
  int point;
  Eigen::Vector2d pixel;
};

/**
 * Maximum-likelihood refinement of the moving start ([1]) in the gravity-aligned world: poses [p, R, v], one bias
 * pair, the points; preintegrated IMU between consecutive poses (Granite's residual), reprojections with a Cauchy
 * loss, a prior on the first pose (position, yaw) and bias priors. Dense Levenberg-Marquardt.
 */
class Refinement {
 public:
  Eigen::aligned_vector<PoseVelState> states;
  Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
  Eigen::aligned_vector<Eigen::Vector3d> points;
  Eigen::aligned_vector<IntegratedImuMeasurement> imu;  // imu[k]: pose k-1 to pose k (imu[0] unused)
  std::vector<Observation> observations;

  const Calibration<double>* calib = nullptr;
  Eigen::Vector3d gravity;
  double pixel_sigma = 1.0;
  Eigen::Vector3d bg_prior, ba_prior;
  double sigma_bg = 0.05, sigma_ba = 0.1;
  Sophus::SE3d first_pose_prior;

  int size() const { return 9 * static_cast<int>(states.size()) + 6 + 3 * static_cast<int>(points.size()); }
  int biasOffset() const { return 9 * static_cast<int>(states.size()); }
  int pointOffset(int j) const { return biasOffset() + 6 + 3 * j; }

  // Cost; H and b when given. False when an observation does not project.
  bool evaluate(double* cost, Eigen::MatrixXd* H, Eigen::VectorXd* b) const {
    constexpr double kSigmaGauge = 1e-5;  // First-pose gauge [m, rad]
    const bool lin = H != nullptr;
    if (lin) {
      H->setZero(size(), size());
      b->setZero(size());
    }
    double c = 0.0;
    const int bo = biasOffset();
    // IMU
    for (size_t k = 1; k < states.size(); ++k) {
      IntegratedImuMeasurement::MatNN J0, J1;
      IntegratedImuMeasurement::MatN3 Jbg, Jba;
      const IntegratedImuMeasurement::VecN r =
          imu[k].residual(states[k - 1], gravity, states[k], bg, ba, &J0, &J1, &Jbg, &Jba);
      const auto& W = imu[k].get_cov_inv();
      c += r.dot(W * r);
      if (!lin) continue;
      Eigen::Matrix<double, 9, Eigen::Dynamic> Jblocks[4] = {J0, J1, Jbg, Jba};
      const int cols[4] = {9 * static_cast<int>(k - 1), 9 * static_cast<int>(k), bo, bo + 3};
      for (int u = 0; u < 4; ++u) {
        const Eigen::MatrixXd WJu = W * Jblocks[u];
        b->segment(cols[u], Jblocks[u].cols()) += WJu.transpose() * r;
        for (int v = 0; v < 4; ++v)
          H->block(cols[v], cols[u], Jblocks[v].cols(), Jblocks[u].cols()) += Jblocks[v].transpose() * WJu;
      }
    }
    // Reprojections (Cauchy loss)
    const Sophus::SE3d T_c_i = calib->T_i_c[0].inverse();
    const Eigen::Matrix3d R_ci = T_c_i.so3().matrix();
    for (const Observation& o : observations) {
      const PoseVelState& s = states[o.pose];
      const Eigen::Matrix3d Rt = s.T_w_i.so3().inverse().matrix();
      const Eigen::Vector3d x = points[o.point] - s.T_w_i.translation();
      const Eigen::Vector3d p_C = R_ci * (Rt * x) + T_c_i.translation();
      Eigen::Vector4d p4;
      p4 << p_C, 1.0;
      Eigen::Vector2d proj;
      Eigen::Matrix<double, 2, 4> Jp;
      if (!calib->intrinsics[0].project(p4, proj, &Jp) || !proj.allFinite()) return false;
      const Eigen::Vector2d r = (proj - o.pixel) / pixel_sigma;
      const double s2 = r.squaredNorm();
      c += std::log1p(s2);
      if (!lin) continue;
      const double w = 1.0 / (1.0 + s2);
      const Eigen::Matrix3d A = R_ci * Rt;
      const Eigen::Matrix<double, 2, 3> Jc = Jp.leftCols<3>() * A / pixel_sigma;
      Eigen::Matrix<double, 2, 9> Jpose = Eigen::Matrix<double, 2, 9>::Zero();
      Jpose.leftCols<3>() = -Jc;
      Jpose.middleCols<3>(3) = Jc * Sophus::SO3d::hat(x);
      const int pc = 9 * o.pose, qc = pointOffset(o.point);
      H->block<9, 9>(pc, pc) += w * Jpose.transpose() * Jpose;
      H->block<9, 3>(pc, qc) += w * Jpose.transpose() * Jc;
      H->block<3, 9>(qc, pc) += w * Jc.transpose() * Jpose;
      H->block<3, 3>(qc, qc) += w * Jc.transpose() * Jc;
      b->segment<9>(pc) += w * Jpose.transpose() * r;
      b->segment<3>(qc) += w * Jc.transpose() * r;
    }
    // First-pose gauge: position and yaw
    const double wg = 1.0 / (kSigmaGauge * kSigmaGauge);
    const Eigen::Vector3d dp = states[0].T_w_i.translation() - first_pose_prior.translation();
    const Eigen::Vector3d e = (states[0].T_w_i.so3() * first_pose_prior.so3().inverse()).log();
    c += wg * (dp.squaredNorm() + e.z() * e.z());
    if (lin) {
      Eigen::Matrix3d Jl;
      Sophus::leftJacobianInvSO3(e, Jl);
      const Eigen::RowVector3d Jyaw = Jl.row(2);
      H->block<3, 3>(0, 0) += wg * Eigen::Matrix3d::Identity();
      b->segment<3>(0) += wg * dp;
      H->block<3, 3>(3, 3) += wg * Jyaw.transpose() * Jyaw;
      b->segment<3>(3) += wg * Jyaw.transpose() * e.z();
    }
    // Bias priors
    const double wbg = 1.0 / (sigma_bg * sigma_bg), wba = 1.0 / (sigma_ba * sigma_ba);
    c += wbg * (bg - bg_prior).squaredNorm() + wba * (ba - ba_prior).squaredNorm();
    if (lin) {
      H->block<3, 3>(bo, bo) += wbg * Eigen::Matrix3d::Identity();
      H->block<3, 3>(bo + 3, bo + 3) += wba * Eigen::Matrix3d::Identity();
      b->segment<3>(bo) += wbg * (bg - bg_prior);
      b->segment<3>(bo + 3) += wba * (ba - ba_prior);
    }
    *cost = c;
    return std::isfinite(c);
  }

  void apply(const Eigen::VectorXd& delta) {
    for (size_t k = 0; k < states.size(); ++k) states[k].applyInc(delta.segment<9>(9 * k));
    bg += delta.segment<3>(biasOffset());
    ba += delta.segment<3>(biasOffset() + 3);
    for (size_t j = 0; j < points.size(); ++j) points[j] += delta.segment<3>(pointOffset(static_cast<int>(j)));
  }
};

}  // namespace

bool MonoInertialInitializer::startMoving(MonoInertialInitResult& result) {
  constexpr int kMinTracks = 8;  // Tracks the moving start must keep
  const int64_t newest = newestImageTime();
  const int64_t W = static_cast<int64_t>(movingWindowTime() * 1e9);
  const int64_t oldest = newest - W;
  dropMeasurementsUpTo(oldest);
  const bool imu_before_window = !imu.empty() && imu.front().t_ns < oldest;
  while (imu.size() > 1 && imu[1].t_ns <= oldest) imu.pop_front();  // keep one sample at or before the window
  result.num_tracks = static_cast<int>(tracks.size());
  if (tracks.size() < 0.75 * options.max_tracks) {
    result.reason = "only " + std::to_string(tracks.size()) + " tracks in the window";
    return false;
  }
  if (imu.size() < 2 || !imu_before_window) {
    result.reason = "IMU window not yet full";
    return false;
  }

  // ---- The longest tracks, then the pose times they observe (spacing W / (num_poses + 1))
  std::vector<int> ids;
  for (const auto& entry : tracks) ids.push_back(entry.first);
  std::stable_sort(ids.begin(), ids.end(),
                   [&](int a, int b) { return tracks.at(a).t_ns.size() > tracks.at(b).t_ns.size(); });
  if (options.parallax_tracks) {
    // Not in openvio: the longest tracks see the farthest points, whose parallax can be below the gyro error. Rank
    // the tracks that span half the window by their parallax after the gyro rotation (bias prior) instead.
    std::set<int64_t> image_times;
    for (const auto& entry : tracks) image_times.insert(entry.second.t_ns.begin(), entry.second.t_ns.end());
    std::map<int64_t, Sophus::SO3d> R_of_time;  // R_{I(first IMU sample) <- I(t)}
    Sophus::SO3d R;
    size_t k = 0;
    for (int64_t t : image_times) {
      if (t < imu.front().t_ns) continue;
      while (k + 1 < imu.size() && imu[k + 1].t_ns <= t) {
        R = R * Sophus::SO3d::exp((imu[k].gyro - options.gyro_bias_prior) * seconds(imu[k + 1].t_ns - imu[k].t_ns));
        ++k;
      }
      R_of_time[t] = R * Sophus::SO3d::exp((imu[k].gyro - options.gyro_bias_prior) * seconds(t - imu[k].t_ns));
    }
    const Eigen::Matrix3d R_ic = calib.T_i_c[0].so3().matrix();
    std::map<int, double> parallax;
    for (int id : ids) {
      const Track& track = tracks.at(id);
      double angle = -1.0;
      const auto r0 = R_of_time.find(track.t_ns.front());
      const auto r1 = R_of_time.find(track.t_ns.back());
      if (track.t_ns.back() - track.t_ns.front() >= W / 2 && r0 != R_of_time.end() && r1 != R_of_time.end()) {
        const Eigen::Vector3d d0 = r0->second * (R_ic * track.bearings.front());
        const Eigen::Vector3d d1 = r1->second * (R_ic * track.bearings.back());
        angle = std::atan2(d0.cross(d1).norm(), d0.dot(d1));
      }
      parallax[id] = angle;
    }
    std::stable_sort(ids.begin(), ids.end(), [&](int a, int b) { return parallax.at(a) > parallax.at(b); });
  }
  if (static_cast<int>(ids.size()) > options.max_tracks) ids.resize(options.max_tracks);
  const int min_kept = static_cast<int>(movingWindowTime());
  const int64_t spacing = W / (options.num_poses + 1);
  std::set<int64_t> times{newest};
  std::vector<int> track_ids;
  int num_rows = 0;
  int64_t first = std::numeric_limits<int64_t>::max();
  for (int id : ids) {
    std::vector<int64_t> kept;
    for (int64_t t : tracks.at(id).t_ns) {
      int64_t gap = std::numeric_limits<int64_t>::max();
      for (int64_t s : times) gap = std::min(gap, std::abs(t - s));
      for (int64_t s : kept) gap = std::min(gap, std::abs(t - s));
      if (gap >= spacing || gap == 0) kept.push_back(t);
    }
    if (static_cast<int>(kept.size()) < min_kept) continue;
    times.insert(kept.begin(), kept.end());
    first = std::min(first, *std::min_element(kept.begin(), kept.end()));
    num_rows += 2 * static_cast<int>(kept.size());
    track_ids.push_back(id);
  }
  result.num_valid_tracks = static_cast<int>(track_ids.size());
  result.num_poses = static_cast<int>(times.size());
  if (result.num_poses < options.num_poses) {
    result.reason = "only " + std::to_string(times.size()) + " pose times";
    return false;
  }
  if (result.num_valid_tracks < kMinTracks) {
    result.reason = "only " + std::to_string(track_ids.size()) + " valid tracks";
    return false;
  }

  // ---- Rotation over the pose window: the gravity direction needs excitation
  double rotation = 0.0;
  for (size_t k = 0; k + 1 < imu.size(); ++k) {
    if (imu[k + 1].t_ns <= first || imu[k].t_ns >= newest) continue;
    const double dt = seconds(std::min(imu[k + 1].t_ns, newest) - std::max(imu[k].t_ns, first));
    rotation += (0.5 * (imu[k].gyro + imu[k + 1].gyro) - options.gyro_bias_prior).norm() * dt;
  }
  result.rotation_deg = rotation * 180.0 / M_PI;
  if (result.rotation_deg < options.min_rotation_deg) {
    result.reason = "rotation " + std::to_string(result.rotation_deg) + " deg below threshold";
    return false;
  }
  const int num_points = static_cast<int>(track_ids.size());
  if (num_rows < 3 * num_points + 6) {
    result.reason = "fewer constraints than unknowns";
    return false;
  }

  // ---- Preintegration from the first pose (linear system) and from the previous pose (refinement)
  const Eigen::Vector3d accel_cov = calib.dicrete_time_accel_noise_std().array().square();
  const Eigen::Vector3d gyro_cov = calib.dicrete_time_gyro_noise_std().array().square();
  const std::vector<int64_t> pose_times(times.begin(), times.end());
  const int N = static_cast<int>(pose_times.size());
  Eigen::aligned_vector<IntegratedImuMeasurement> from_first(N), from_previous(N);
  for (int k = 1; k < N; ++k) {
    if (!preintegrate(imu, pose_times[0], pose_times[k], options.gyro_bias_prior, options.accel_bias_prior,
                      accel_cov, gyro_cov, from_first[k]) ||
        !preintegrate(imu, pose_times[k - 1], pose_times[k], options.gyro_bias_prior, options.accel_bias_prior,
                      accel_cov, gyro_cov, from_previous[k])) {
      result.reason = "IMU does not cover the pose interval ending at " + std::to_string(seconds(pose_times[k]));
      return false;
    }
  }
  std::map<int64_t, int> pose_of_time;
  for (int k = 0; k < N; ++k) pose_of_time[pose_times[k]] = k;
  // Motion of pose k from the first pose (I0): R_{I0<-Ik}, alpha, beta, DT
  auto motion = [&](int k, Eigen::Matrix3d* R, Eigen::Vector3d* alpha, Eigen::Vector3d* beta, double* DT) {
    if (k == 0) {
      R->setIdentity();
      alpha->setZero();
      beta->setZero();
      *DT = 0.0;
      return;
    }
    const PoseVelState& d = from_first[k].getDeltaState();
    *R = d.T_w_i.so3().matrix();
    *alpha = d.T_w_i.translation();
    *beta = d.vel_w_i;
    *DT = seconds(from_first[k].get_dt_ns());
  };

  // ---- Linear solution ([1]): x = [p_F of every used track, v_I0, g_I0] in I0. The IMU at pose k is at
  // p_k = v DT + 1/2 g DT^2 + alpha_k; a ray b gives T(b) (R_ci R_k^T (p_F - p_k) + t_ci) = 0.
  const Sophus::SE3d T_c_i = calib.T_i_c[0].inverse();
  const Eigen::Matrix3d R_ci = T_c_i.so3().matrix();
  const Eigen::Vector3d t_ci = T_c_i.translation();
  const int v_col = 3 * num_points;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_rows, v_col + 6);
  Eigen::VectorXd bvec = Eigen::VectorXd::Zero(num_rows);
  int row = 0;
  for (int f = 0; f < num_points; ++f) {
    const Track& track = tracks.at(track_ids[f]);
    for (size_t i = 0; i < track.t_ns.size(); ++i) {
      const auto pose = pose_of_time.find(track.t_ns[i]);
      if (pose == pose_of_time.end()) continue;
      if (row + 2 > num_rows) {
        result.reason = "linear system row count mismatch";
        return false;
      }
      Eigen::Matrix3d R;
      Eigen::Vector3d alpha, beta;
      double DT;
      motion(pose->second, &R, &alpha, &beta, &DT);
      const Eigen::Matrix<double, 2, 3> T = orthogonalRows(track.bearings[i]);
      const Eigen::Matrix<double, 2, 3> Y = T * R_ci * R.transpose();
      A.block<2, 3>(row, 3 * f) = Y;
      A.block<2, 3>(row, v_col) = -DT * Y;
      A.block<2, 3>(row, v_col + 3) = -0.5 * DT * DT * Y;
      bvec.segment<2>(row) = Y * alpha - T * t_ci;
      row += 2;
    }
  }
  if (row != num_rows) {
    result.reason = "linear system row count mismatch";
    return false;
  }
  Eigen::VectorXd x;
  if (!solveGravityConstrainedLeastSquares(A, bvec, gravity_magnitude, x, result.reason)) return false;
  const Eigen::Vector3d v_I0 = x.segment<3>(v_col);
  const Eigen::Vector3d g_I0 = x.tail<3>();
  result.linear_speed = v_I0.norm();

  // ---- Poses in I0 and the points in front of every ray that observed them at a pose time
  Eigen::aligned_vector<PoseVelState> poses_I0(N);
  for (int k = 0; k < N; ++k) {
    Eigen::Matrix3d R;
    Eigen::Vector3d alpha, beta;
    double DT;
    motion(k, &R, &alpha, &beta, &DT);
    poses_I0[k] = PoseVelState(pose_times[k],
                               Sophus::SE3d(Sophus::SO3d::fitToSO3(R), v_I0 * DT + 0.5 * g_I0 * DT * DT + alpha),
                               v_I0 + g_I0 * DT + R * beta);
  }
  const Sophus::SO3d R_w_0(Eigen::Quaterniond::FromTwoVectors(g_I0, gravity));
  Refinement refinement;
  refinement.calib = &calib;
  refinement.gravity = gravity;
  refinement.pixel_sigma = options.pixel_sigma;
  refinement.bg_prior = options.gyro_bias_prior;
  refinement.ba_prior = options.accel_bias_prior;
  refinement.sigma_bg = options.prior_sigma_gyro_bias;
  refinement.sigma_ba = options.prior_sigma_accel_bias;
  refinement.bg = options.gyro_bias_prior;
  refinement.ba = options.accel_bias_prior;
  for (int k = 0; k < N; ++k) {
    const PoseVelState& s = poses_I0[k];
    refinement.states.emplace_back(s.t_ns, Sophus::SE3d(R_w_0 * s.T_w_i.so3(), R_w_0 * s.T_w_i.translation()),
                                   R_w_0 * s.vel_w_i);
  }
  refinement.first_pose_prior = refinement.states[0].T_w_i;
  refinement.imu = from_previous;
  for (int f = 0; f < num_points; ++f) {
    const Track& track = tracks.at(track_ids[f]);
    const Eigen::Vector3d p_F = x.segment<3>(3 * f);
    bool in_front = true;
    for (size_t i = 0; i < track.t_ns.size() && in_front; ++i) {
      const auto pose = pose_of_time.find(track.t_ns[i]);
      if (pose == pose_of_time.end()) continue;
      const PoseVelState& s = poses_I0[pose->second];
      const Eigen::Vector3d p_C = R_ci * (s.T_w_i.so3().inverse() * (p_F - s.T_w_i.translation())) + t_ci;
      in_front = track.bearings[i].dot(p_C) > 0.0;
    }
    if (!in_front) continue;
    const int j = static_cast<int>(refinement.points.size());
    refinement.points.push_back(R_w_0 * p_F);
    for (size_t i = 0; i < track.t_ns.size(); ++i) {
      const auto pose = pose_of_time.find(track.t_ns[i]);
      if (pose == pose_of_time.end()) continue;
      // a3ff0e5: an observation whose point does not project at the start is left out
      const PoseVelState& s = refinement.states[pose->second];
      Eigen::Vector4d p4;
      p4 << R_ci * (s.T_w_i.so3().inverse() * (refinement.points[j] - s.T_w_i.translation())) + t_ci, 1.0;
      Eigen::Vector2d proj;
      if (!calib.intrinsics[0].project(p4, proj) || !proj.allFinite()) {
        ++result.num_unprojected_observations;
        continue;
      }
      refinement.observations.push_back({pose->second, j, track.pixels[i]});
    }
  }
  result.num_refined_features = static_cast<int>(refinement.points.size());
  if (result.num_refined_features < kMinTracks ||
      result.num_refined_features < options.min_refined_fraction * result.num_valid_tracks) {
    result.reason = "only " + std::to_string(refinement.points.size()) + " features in front of their bearings";
    return false;
  }

  // ---- Levenberg-Marquardt; the IMU is preintegrated again at the refined biases once
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  double cost = 0.0;
  if (!refinement.evaluate(&cost, &H, &g)) {
    result.reason = "refinement start does not project";
    return false;
  }
  result.refine_initial_cost = cost;
  bool converged = options.refine_max_iterations == 0;
  double lambda = 1e-4;
  int relinearizations = 0;
  for (int it = 0; it < options.refine_max_iterations && !converged; ++it) {
    ++result.refine_iterations;
    Eigen::MatrixXd Hd = H;
    Hd.diagonal() += lambda * H.diagonal().cwiseMax(1e-6);
    const Eigen::VectorXd delta = -Hd.ldlt().solve(g);
    Refinement trial = refinement;
    double trial_cost = 0.0;
    bool accepted = false;
    if (delta.allFinite()) {
      trial.apply(delta);
      accepted = trial.evaluate(&trial_cost, nullptr, nullptr) && trial_cost < cost;
    }
    if (!accepted) {
      lambda *= 10.0;
      if (lambda > 1e12) converged = true;  // No decrease is possible: a minimum
      continue;
    }
    const double decrease = (cost - trial_cost) / std::max(cost, 1e-300);
    refinement = trial;
    cost = trial_cost;
    lambda = std::max(lambda / 10.0, 1e-12);
    bool done = decrease < 1e-5;
    if (done && relinearizations == 0 &&
        ((refinement.bg - options.gyro_bias_prior).norm() > 1e-4 ||
         (refinement.ba - options.accel_bias_prior).norm() > 1e-2)) {
      // Preintegrate again at the refined biases
      ++relinearizations;
      for (int k = 1; k < N; ++k)
        preintegrate(imu, pose_times[k - 1], pose_times[k], refinement.bg, refinement.ba, accel_cov, gyro_cov,
                     refinement.imu[k]);
      done = false;
    }
    refinement.evaluate(&cost, &H, &g);
    converged = done || g.lpNorm<Eigen::Infinity>() < 1e-10;
  }
  result.refine_final_cost = cost;
  if (!converged) {
    result.reason = "refinement did not converge";
    return false;
  }

  // ---- Marginal covariance of the chosen pose [p, theta, v] and the biases
  refinement.evaluate(&cost, &H, &g);
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
  if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().array() > 0).all()) {
    result.reason = "covariance recovery failed";
    return false;
  }
  const int chosen = options.replay_window ? 0 : N - 1;
  Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(H.rows(), 15);
  selector.block<9, 9>(9 * chosen, 0).setIdentity();
  selector.block<6, 6>(refinement.biasOffset(), 9).setIdentity();
  const Eigen::MatrixXd columns = ldlt.solve(selector);
  Matrix15 P = selector.transpose() * columns;
  if (!P.allFinite()) {
    result.reason = "covariance recovery failed";
    return false;
  }
  {
    const Eigen::Vector3d v = refinement.states[chosen].vel_w_i;
    const double speed = v.norm();
    const Eigen::Vector3d u = speed > 1e-6 ? Eigen::Vector3d(v / speed) : Eigen::Vector3d::UnitX();
    const double sigma = std::sqrt(std::max(0.0, u.dot(P.block<3, 3>(6, 6) * u)));
    result.relative_speed_sigma = sigma / std::max(speed, 1e-6);
    result.refined_speed = speed;
    if (options.max_relative_speed_sigma > 0.0 && result.relative_speed_sigma > options.max_relative_speed_sigma) {
      result.reason = "speed not observable (relative sigma " + std::to_string(result.relative_speed_sigma) + ")";
      return false;
    }
  }
  P.block<3, 3>(3, 3) *= options.covariance_scale_orientation;
  P.block<3, 3>(6, 6) *= options.covariance_scale_velocity;
  P.block<3, 3>(9, 9) *= options.covariance_scale_gyro_bias;
  P.block<3, 3>(12, 12) *= options.covariance_scale_accel_bias;
  result.covariance = 0.5 * (P + P.transpose());
  const PoseVelState& s = refinement.states[chosen];
  result.t_ns = s.t_ns;
  result.T_w_i = s.T_w_i;
  result.vel_w_i = s.vel_w_i;
  result.bias_gyro = refinement.bg;
  result.bias_accel = refinement.ba;
  return true;
}

// Accel-bias component along gravity when the data start at rest (median ray motion of the first second below half
// a pixel): the mean specific force up to the state is gravity plus the bias plus the mean acceleration v_up / T.
void MonoInertialInitializer::applyRestAccelBias(MonoInertialInitResult& result) const {
  if (options.rest_accel_bias_sigma <= 0.0 || startup_accel.empty() || startup_motion.empty()) return;
  const int64_t t0 = startup_accel.front().first;
  std::vector<double> start;
  for (const auto& motion : startup_motion)
    if (motion.first <= t0 + 1000000000LL) start.push_back(motion.second);
  const double half_pixel = 0.5 / std::abs(calib.intrinsics[0].getParam()[0]);
  if (start.size() < 10) return;
  std::nth_element(start.begin(), start.begin() + static_cast<long>(start.size() / 2), start.end());
  if (!(start[start.size() / 2] < half_pixel)) return;
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  size_t n = 0;
  int64_t t_last = t0;
  for (const auto& sample : startup_accel) {
    if (sample.first > result.t_ns) break;
    f += sample.second;
    t_last = sample.first;
    ++n;
  }
  if (n < 2 || t_last <= t0) return;
  f /= static_cast<double>(n);
  const double elapsed = seconds(t_last - t0);
  const double velocity_up = -result.vel_w_i.dot(gravity.normalized());
  const double bias = f.norm() - gravity_magnitude - velocity_up / elapsed;
  const Eigen::Vector3d d = f.normalized();
  // Kalman update of the state with the measurement d . ba = bias
  Eigen::Matrix<double, 1, 15> Hrow = Eigen::Matrix<double, 1, 15>::Zero();
  Hrow.segment<3>(12) = d.transpose();
  const double S = (Hrow * result.covariance * Hrow.transpose())(0, 0) +
                   options.rest_accel_bias_sigma * options.rest_accel_bias_sigma;
  const Eigen::Matrix<double, 15, 1> K = result.covariance * Hrow.transpose() / S;
  const Eigen::Matrix<double, 15, 1> dx = K * (bias - d.dot(result.bias_accel));
  result.T_w_i.translation() += dx.segment<3>(0);
  result.T_w_i.so3() = Sophus::SO3d::exp(dx.segment<3>(3)) * result.T_w_i.so3();
  result.vel_w_i += dx.segment<3>(6);
  result.bias_gyro += dx.segment<3>(9);
  result.bias_accel += dx.segment<3>(12);
  result.covariance = (Matrix15::Identity() - K * Hrow) * result.covariance;
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose()).eval();
  result.rest_bias_applied = true;
  result.rest_bias = bias;
}

}  // namespace granite
