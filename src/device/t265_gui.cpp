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

#include <granite/device/t265_gui.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace granite {
namespace {
constexpr const char* window_name = "Granite T265";
constexpr int width = 1200;
constexpr int image_height = 480;
constexpr int height = 780;

void text(cv::Mat& canvas, const std::string& value, int x, int y,
          const cv::Scalar& color = cv::Scalar(220, 220, 220)) {
  cv::putText(canvas, value, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
              0.5, color, 1, cv::LINE_AA);
}

cv::Scalar depthColor(double inverse_depth) {
  const double near = std::clamp(inverse_depth, 0.0, 1.0);
  return cv::Scalar(255 * (1 - near), 80, 255 * near);
}
}  // namespace

void T265Gui::open() {
  if (opened) return;
#if defined(__linux__)
  if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY") &&
      !std::getenv("QT_QPA_PLATFORM")) {
    throw std::runtime_error("T265 GUI needs a display. Use --no-gui for headless operation.");
  }
#endif
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  opened = true;
  cv::resizeWindow(window_name, width, height);
  cv::imshow(window_name, compose(T265GuiStatus{}));
  cv::waitKey(1);
}

T265Gui::~T265Gui() {
  if (!opened) return;
  try {
    cv::destroyWindow(window_name);
  } catch (const cv::Exception&) {
  }
}

void T265Gui::setFrame(const OpticalFlowInput::Ptr& next) {
  if (!next) return;
  frame = next;
  visualization.reset();
}

void T265Gui::setVisualization(const VioVisualizationData::Ptr& next) {
  if (!next || !next->opt_flow_res || !next->opt_flow_res->input_images) return;
  visualization = next;
  frame = next->opt_flow_res->input_images;
}

void T265Gui::setState(const VioStateData::Ptr& next) {
  if (!next || !next->state.T_w_i.matrix().allFinite()) return;
  if (state && next->map_idx != state->map_idx) trajectory.clear();
  state = next;
  trajectory.push_back(state->state.T_w_i.translation());
  if (trajectory.size() > 6000) trajectory.pop_front();
}

void T265Gui::drawImage(cv::Mat& canvas, size_t camera) const {
  const int left = static_cast<int>(camera) * width / 2;
  if (!frame || camera >= frame->img_data.size() || !frame->img_data[camera].img) {
    text(canvas, camera == 0 ? "Waiting for images" : "No second camera", left + 20, 60);
    return;
  }
  const auto& image = *frame->img_data[camera].img;
  if (!image.ptr || !image.w || !image.h) return;
  const double scale = std::min((width / 2.0 - 20) / image.w,
                               (image_height - 50.0) / image.h);
  const int view_width = cvRound(image.w * scale);
  const int view_height = cvRound(image.h * scale);
  const int x = left + (width / 2 - view_width) / 2;
  const int y = 35 + (image_height - 45 - view_height) / 2;
  cv::Mat gray(static_cast<int>(image.h), static_cast<int>(image.w),
               cv::traits::Type<PixelType>::value, image.ptr, image.pitch);
  cv::Mat gray8, resized, rgb;
  gray.convertTo(gray8, CV_8U, sizeof(PixelType) == 2 ? 1.0 / 256.0 : 1.0);
  cv::resize(gray8, resized, cv::Size(view_width, view_height));
  cv::cvtColor(resized, rgb, cv::COLOR_GRAY2BGR);
  rgb.copyTo(canvas(cv::Rect(x, y, view_width, view_height)));
  const auto pixel = [&](double px, double py) { return cv::Point(x + cvRound(px * scale), y + cvRound(py * scale)); };
  const auto in_image = [&](double px, double py) {
    return std::isfinite(px) && std::isfinite(py) && px >= 0 && py >= 0 && px < image.w && py < image.h;
  };
  if (visualization && camera < visualization->opt_flow_res->observations.size()) {
    for (const auto& observation : visualization->opt_flow_res->observations[camera]) {
      const auto p = observation.second.translation();
      if (in_image(p.x(), p.y())) cv::circle(canvas, pixel(p.x(), p.y()), 2, cv::Scalar(70, 255, 70), -1);
    }
  }
  if (visualization && camera < visualization->projections.size()) {
    for (const auto& projection : visualization->projections[camera]) {
      if (!in_image(projection[0], projection[1]) || !std::isfinite(projection[2])) continue;
      const cv::Point p = pixel(projection[0], projection[1]);
      if (projection[2] > 0) cv::circle(canvas, p, 5, depthColor(projection[2]), 2, cv::LINE_AA);
      else cv::rectangle(canvas, p - cv::Point(4, 4), p + cv::Point(4, 4), cv::Scalar(255, 160, 0), 1);
    }
  }
  text(canvas, "camera " + std::to_string(camera) + "  " + std::to_string(image.w) + "x" + std::to_string(image.h), left + 15, 22);
}

void T265Gui::drawTrajectory(cv::Mat& canvas, const cv::Rect& area, int second_axis) const {
  cv::rectangle(canvas, area, cv::Scalar(65, 65, 65), 1);
  text(canvas, second_axis == 1 ? "Trajectory XY" : "Trajectory XZ", area.x + 12, area.y + 22);
  if (trajectory.empty()) {
    text(canvas, "Waiting for estimated poses", area.x + 12, area.y + 60);
    return;
  }
  double min_x = trajectory.front().x(), max_x = min_x;
  double min_y = trajectory.front()[second_axis], max_y = min_y;
  for (const auto& p : trajectory) {
    min_x = std::min(min_x, p.x()); max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p[second_axis]); max_y = std::max(max_y, p[second_axis]);
  }
  const double extent = std::max({max_x - min_x, max_y - min_y, 0.2});
  const double scale = std::min(area.width - 40, area.height - 60) / (extent * 1.1);
  const double center_x = (min_x + max_x) / 2, center_y = (min_y + max_y) / 2;
  const auto pixel = [&](const Eigen::Vector3d& p) {
    return cv::Point(area.x + area.width / 2 + cvRound((p.x() - center_x) * scale),
                     area.y + (area.height + 20) / 2 - cvRound((p[second_axis] - center_y) * scale));
  };
  for (size_t i = 1; i < trajectory.size(); ++i) cv::line(canvas, pixel(trajectory[i - 1]), pixel(trajectory[i]), cv::Scalar(80, 210, 255), 1, cv::LINE_AA);
  cv::circle(canvas, pixel(trajectory.front()), 4, cv::Scalar(80, 255, 80), -1);
  cv::circle(canvas, pixel(trajectory.back()), 5, cv::Scalar(80, 80, 255), -1);
}

cv::Mat T265Gui::compose(const T265GuiStatus& status) const {
  cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(24, 24, 24));
  drawImage(canvas, 0);
  drawImage(canvas, 1);
  drawTrajectory(canvas, cv::Rect(10, image_height + 10, 395, 260), 1);
  drawTrajectory(canvas, cv::Rect(415, image_height + 10, 395, 260), 2);
  const int x = 825;
  int y = image_height + 30;
  const auto line = [&](const std::string& value) { text(canvas, value, x, y); y += 22; };
  line(status.mode + (status.capture_only ? "  CAPTURE ONLY" : ""));
  line("tracking: " + status.tracking);
  line("support: " + status.support);
  line(std::string("metric initialized: ") + (status.metric_initialized ? "yes" : "no"));
  std::ostringstream rates;
  rates << std::fixed << std::setprecision(1) << "fps input/state: " << status.input_fps << "/" << status.state_fps;
  line(rates.str());
  std::ostringstream lag;
  lag << std::fixed << std::setprecision(1) << "elapsed: " << status.elapsed << "s  lag: " << status.lag_ms << "ms";
  line(lag.str());
  line("images: " + std::to_string(status.images) + "  states: " + std::to_string(status.states));
  line("drops: " + std::to_string(status.dropped));
  line("landmarks finite/all: " + std::to_string(status.finite_landmarks) + "/" + std::to_string(status.landmarks));
  line("connected finite/infinite: " + std::to_string(status.connected_finite_landmarks) + "/" + std::to_string(status.connected_infinite_landmarks));
  if (state) {
    const auto& p = state->state.T_w_i.translation();
    std::ostringstream position;
    position << std::fixed << std::setprecision(2) << "xyz: " << p.x() << ", " << p.y() << ", " << p.z();
    line(position.str());
  }
  text(canvas, "green: tracks | blue square: infinity | depth: blue far, red near | Esc / Q / close: stop", 15, height - 12);
  return canvas;
}

bool T265Gui::draw(const T265GuiStatus& status) {
  if (!opened) return false;
  try {
    if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) < 0) return false;
  } catch (const cv::Exception&) {
    return false;
  }
  cv::imshow(window_name, compose(status));
  const int key = cv::waitKey(1) & 255;
  if (key == 27 || key == 'q' || key == 'Q') return false;
  try {
    return cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0;
  } catch (const cv::Exception&) {
    return false;
  }
}

}  // namespace granite
