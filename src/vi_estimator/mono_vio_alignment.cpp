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

#include <granite/vi_estimator/mono_vio_alignment.h>
#include <Eigen/SVD>

namespace granite {
namespace {
std::vector<IntegratedImuMeasurement> integrate(
    const std::vector<MonoVioFrame>& frames,
    const Eigen::aligned_deque<ImuData>& samples,
    const Calibration<double>& calibration,
    const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  std::vector<IntegratedImuMeasurement> result;
  const Eigen::Vector3d ac = calibration.dicrete_time_accel_noise_std().array().square();
  const Eigen::Vector3d gc = calibration.dicrete_time_gyro_noise_std().array().square();
  size_t sample_index = 0;
  for (size_t i = 1; i < frames.size(); ++i) {
    IntegratedImuMeasurement measurement(frames[i - 1].t_ns, bg, ba);
    while (sample_index < samples.size() && samples[sample_index].t_ns <= frames[i - 1].t_ns) ++sample_index;
    size_t k = sample_index;
    while (k < samples.size()) {
      ImuData sample = samples[k];
      sample.t_ns = std::min(sample.t_ns, frames[i].t_ns);
      sample.accel = calibration.calib_accel_bias.getCalibrated(sample.accel);
      sample.gyro = calibration.calib_gyro_bias.getCalibrated(sample.gyro);
      measurement.integrate(sample, ac, gc);
      if (samples[k].t_ns >= frames[i].t_ns) break;
      ++k;
    }
    result.push_back(measurement);
    sample_index = k;
  }
  return result;
}

bool solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& x) {
  const Eigen::VectorXd norms = A.colwise().norm();
  if ((norms.array() < 1e-12).any()) return false;
  const Eigen::MatrixXd normalized = A * norms.cwiseInverse().asDiagonal();
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues().tail<1>()[0] < svd.singularValues()[0] * 1e-6) return false;
  x = norms.cwiseInverse().asDiagonal() * svd.solve(b);
  return x.allFinite();
}
}

std::optional<MonoVioAlignment> alignMonoVisualInertial(
    const std::vector<MonoVioFrame>& frames,
    const Eigen::aligned_deque<ImuData>& samples,
    const Calibration<double>& calibration,
    const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias) {
  const size_t n = frames.size();
  if (n < 6 || calibration.T_i_c.size() != 1 || samples.size() < 2 ||
      samples.front().t_ns > frames.front().t_ns || samples.back().t_ns < frames.back().t_ns ||
      frames.back().t_ns - frames.front().t_ns < 500000000) return std::nullopt;
  for (size_t i = 1; i < n; ++i) {
    if (frames[i].t_ns <= frames[i - 1].t_ns || !frames[i].T_v_i.matrix().allFinite()) return std::nullopt;
  }
  for (size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].t_ns <= samples[i - 1].t_ns ||
        !samples[i].accel.allFinite() || !samples[i].gyro.allFinite()) return std::nullopt;
  }
  MonoVioAlignment result;
  result.gyro_bias = gyro_bias;
  auto preintegrations = integrate(frames, samples, calibration, result.gyro_bias, accel_bias);
  Eigen::MatrixXd rotation_A(3 * (n - 1), 3);
  Eigen::VectorXd rotation_b(3 * (n - 1));
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto observed = frames[i].T_v_i.so3().inverse() * frames[i + 1].T_v_i.so3();
    rotation_A.block<3, 3>(3 * i, 0) = preintegrations[i].get_d_state_d_bg().block<3, 3>(3, 0);
    rotation_b.segment<3>(3 * i) = (preintegrations[i].getDeltaState().T_w_i.so3().inverse() * observed).log();
  }
  Eigen::VectorXd delta_bg;
  if (!solve(rotation_A, rotation_b, delta_bg)) return std::nullopt;
  result.gyro_bias += delta_bg;
  if (result.gyro_bias.norm() > 0.5) return std::nullopt;
  preintegrations = integrate(frames, samples, calibration, result.gyro_bias, accel_bias);

  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  Eigen::aligned_vector<Eigen::Vector3d> average_accelerations;
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dt = preintegrations[i].get_dt_ns() * 1e-9;
    const Eigen::Vector3d acceleration = frames[i].T_v_i.so3() * preintegrations[i].getDeltaState().vel_w_i / dt;
    average_accelerations.push_back(acceleration);
    mean_acceleration += acceleration;
  }
  mean_acceleration /= average_accelerations.size();
  double excitation = 0;
  for (const auto& acceleration : average_accelerations) excitation += (acceleration - mean_acceleration).squaredNorm();
  if (excitation / average_accelerations.size() < 0.01) return std::nullopt;

  const Eigen::Vector3d lever = calibration.T_i_c[0].translation();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * (n - 1), 3 * n + 4);
  Eigen::VectorXd b(6 * (n - 1));
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dt = (frames[i + 1].t_ns - frames[i].t_ns) * 1e-9;
    const Eigen::Matrix3d Ri = frames[i].T_v_i.rotationMatrix();
    const Eigen::Matrix3d Rj = frames[i + 1].T_v_i.rotationMatrix();
    const Eigen::Vector3d Pi = frames[i].T_v_i.translation() + Ri * lever;
    const Eigen::Vector3d Pj = frames[i + 1].T_v_i.translation() + Rj * lever;
    const int row = 6 * i;
    A.block<3, 3>(row, 3 * i) = -dt * Eigen::Matrix3d::Identity();
    A.block<3, 3>(row, 3 * n) = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
    A.block<3, 1>(row, 3 * n + 3) = Pj - Pi;
    b.segment<3>(row) = Ri * preintegrations[i].getDeltaState().T_w_i.translation() + (Rj - Ri) * lever;
    A.block<3, 3>(row + 3, 3 * i) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(row + 3, 3 * (i + 1)) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(row + 3, 3 * n) = -dt * Eigen::Matrix3d::Identity();
    b.segment<3>(row + 3) = Ri * preintegrations[i].getDeltaState().vel_w_i;
  }
  Eigen::VectorXd x;
  if (!solve(A, b, x)) return std::nullopt;
  result.gravity = x.segment<3>(3 * n);
  const double gravity_norm = constants::g.norm();
  if (std::abs(result.gravity.norm() - gravity_norm) > 1.0 || x.tail<1>()[0] <= 0) return std::nullopt;

  // Refine only the two gravity-direction degrees of freedom.
  for (int iteration = 0; iteration < 4; ++iteration) {
    result.gravity = result.gravity.normalized() * gravity_norm;
    const Eigen::Vector3d tangent = result.gravity.normalized().unitOrthogonal();
    Eigen::Matrix<double, 3, 2> basis;
    basis.col(0) = tangent;
    basis.col(1) = result.gravity.normalized().cross(tangent);
    Eigen::MatrixXd refined(A.rows(), 3 * n + 3);
    refined.leftCols(3 * n) = A.leftCols(3 * n);
    refined.middleCols(3 * n, 2) = A.middleCols(3 * n, 3) * basis;
    refined.rightCols(1) = A.rightCols(1);
    const Eigen::VectorXd rhs = b - A.middleCols(3 * n, 3) * result.gravity;
    Eigen::VectorXd update;
    if (!solve(refined, rhs, update)) return std::nullopt;
    result.gravity += basis * update.segment<2>(3 * n);
    x.head(3 * n) = update.head(3 * n);
    x.tail<1>() = update.tail<1>();
  }
  result.gravity = result.gravity.normalized() * gravity_norm;
  result.scale = x.tail<1>()[0];
  x.segment<3>(3 * n) = result.gravity;
  result.residual = (A * x - b).norm() / std::sqrt(static_cast<double>(b.size()));
  if (result.scale <= 0 || !std::isfinite(result.scale) || result.residual > 0.05) return std::nullopt;
  for (size_t i = 0; i < n; ++i) result.velocities.push_back(x.segment<3>(3 * i));
  return result;
}
}  // namespace granite
