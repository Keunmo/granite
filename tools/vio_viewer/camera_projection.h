// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

// Projection of a point in the camera frame with the camera models of granite (thirdparty/granite-headers,
// include/granite/camera). The parameters are those of getParam() for the model. Returns false for a model that
// this viewer does not know and for a point that the model cannot project.
inline bool projectPoint(const std::string& model, const std::vector<float>& p, const Eigen::Vector3d& x,
                         Eigen::Vector2d* uv) {
  if (p.size() < 4) return false;
  const double fx = p[0], fy = p[1], cx = p[2], cy = p[3];
  const double r2 = x.x() * x.x() + x.y() * x.y();
  double mx = 0, my = 0;
  if (model == "pinhole") {
    if (x.z() <= 1e-6) return false;
    mx = x.x() / x.z();
    my = x.y() / x.z();
  } else if (model == "kb4" && p.size() >= 8) {
    if (x.z() <= 1e-6) return false;
    const double r = std::sqrt(r2);
    if (r < 1e-9) {
      mx = x.x() / x.z();
      my = x.y() / x.z();
    } else {
      const double theta = std::atan2(r, x.z());
      const double t2 = theta * theta;
      const double d = theta * (1 + t2 * (p[4] + t2 * (p[5] + t2 * (p[6] + t2 * p[7]))));
      mx = d * x.x() / r;
      my = d * x.y() / r;
    }
  } else if (model == "ds" && p.size() >= 6) {
    const double xi = p[4], alpha = p[5];
    const double d1 = std::sqrt(r2 + x.z() * x.z());
    const double k = xi * d1 + x.z();
    const double d2 = std::sqrt(r2 + k * k);
    const double norm = alpha * d2 + (1 - alpha) * k;
    if (norm <= 1e-6) return false;
    mx = x.x() / norm;
    my = x.y() / norm;
  } else if (model == "eucm" && p.size() >= 6) {
    const double alpha = p[4], beta = p[5];
    const double d = std::sqrt(beta * r2 + x.z() * x.z());
    const double norm = alpha * d + (1 - alpha) * x.z();
    if (norm <= 1e-6) return false;
    mx = x.x() / norm;
    my = x.y() / norm;
  } else if (model == "ucm" && p.size() >= 5) {
    const double alpha = p[4];
    const double d = std::sqrt(r2 + x.z() * x.z());
    const double norm = alpha * d + (1 - alpha) * x.z();
    if (norm <= 1e-6) return false;
    mx = x.x() / norm;
    my = x.y() / norm;
  } else {
    return false;
  }
  *uv = Eigen::Vector2d(fx * mx + cx, fy * my + cy);
  return true;
}
