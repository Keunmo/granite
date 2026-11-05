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

#pragma once
#include <granite/vi_estimator/keypoint_vio.h>
#include <granite/vi_estimator/keypoint_vo.h>
#include <granite/vi_estimator/mono_vio_alignment.h>

namespace granite {
class MonoVioEstimator : public VioEstimatorBase {
 public:
  MonoVioEstimator(const Eigen::Vector3d& gravity, const Calibration<double>& calibration,
                   const VioConfig& configuration);
  ~MonoVioEstimator();
  void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                  const Eigen::Vector3d& velocity, const Eigen::Vector3d& bg,
                  const Eigen::Vector3d& ba) override;
  void initialize(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) override;
  const Sophus::SE3d& getT_w_i_init() override { return initial_pose; }
  void join() override;
  void quit() override;
  bool metricInitialized() const { return metric_initialized.load(); }
  double metricScale() const { return metric_scale.load(); }

 private:
  void run(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);
  // Initialization by the visual-inertial initializer (vio_mono_init_method "inertial")
  void runInertial(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);
  Eigen::Vector3d gravity;
  Calibration<double> calibration;
  VioConfig configuration;
  Sophus::SE3d initial_pose;
  std::thread processing_thread;
  std::atomic<bool> metric_initialized{false};
  std::atomic<double> metric_scale{0};
};
}  // namespace granite
