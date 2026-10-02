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

#include <deque>
#include <string>
#include <opencv2/core.hpp>
#include <granite/vi_estimator/vio_estimator.h>

namespace granite {

struct T265GuiStatus {
  std::string mode;
  std::string tracking;
  std::string support;
  bool capture_only = false;
  bool metric_initialized = false;
  double elapsed = 0;
  double input_fps = 0;
  double state_fps = 0;
  double lag_ms = 0;
  uint64_t images = 0;
  uint64_t states = 0;
  uint64_t dropped = 0;
  size_t landmarks = 0;
  size_t finite_landmarks = 0;
  size_t connected_finite_landmarks = 0;
  size_t connected_infinite_landmarks = 0;
};

class T265Gui {
 public:
  T265Gui() = default;
  void open();
  ~T265Gui();
  T265Gui(const T265Gui&) = delete;
  T265Gui& operator=(const T265Gui&) = delete;

  void setFrame(const OpticalFlowInput::Ptr& frame);
  void setVisualization(const VioVisualizationData::Ptr& visualization);
  void setState(const VioStateData::Ptr& state);
  bool draw(const T265GuiStatus& status);

 private:
  cv::Mat compose(const T265GuiStatus& status) const;
  void drawImage(cv::Mat& canvas, size_t camera) const;
  void drawTrajectory(cv::Mat& canvas, const cv::Rect& area, int second_axis) const;
  bool opened = false;
  OpticalFlowInput::Ptr frame;
  VioVisualizationData::Ptr visualization;
  VioStateData::Ptr state;
  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> trajectory;
};

}  // namespace granite
