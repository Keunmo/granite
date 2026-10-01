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

Copyright (c) 2019, Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
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

#include <stdexcept>
#include <opencv2/imgproc.hpp>
#include <granite/calibration/calibration.hpp>
#include <granite/io/dataset_io.h>

namespace granite {

inline void prepareT265Calibration(Calibration<double>& calib, int mono_camera,
                                   bool resize_half) {
  if (mono_camera < 0 || mono_camera > 2 || calib.intrinsics.size() != 2 ||
      calib.resolution.size() != 2 || calib.T_i_c.size() != 2) {
    throw std::invalid_argument("Expected two T265 cameras and camera index 0, 1 or 2");
  }
  calib.main_cam_idx = {0};
  if (mono_camera != 0) {
    const size_t selected = mono_camera - 1;
    const auto camera = calib.intrinsics[selected];
    const auto transform = calib.T_i_c[selected];
    const auto resolution = calib.resolution[selected];
    calib.intrinsics = {camera};
    calib.T_i_c = {transform};
    calib.resolution = {resolution};
    calib.stereo_pairs.clear();
  } else {
    calib.stereo_pairs = {{0, 1}};
  }
  if (!resize_half) return;
  for (size_t i = 0; i < calib.intrinsics.size(); ++i) {
    auto* camera = std::get_if<KannalaBrandtCamera4<double>>(&calib.intrinsics[i].variant);
    if (!camera) throw std::invalid_argument("Expected T265 Kannala-Brandt intrinsics");
    auto params = camera->getParam();
    params.head<2>() *= 0.5;
    // INTER_AREA maps the center of each 2x2 block to one output pixel.
    params.segment<2>(2) = (params.segment<2>(2).array() + 0.5) * 0.5 - 0.5;
    calib.intrinsics[i].variant = KannalaBrandtCamera4<double>(params);
    calib.resolution[i] /= 2;
  }
}

inline ManagedImage<PixelType>::Ptr copyT265Image(const cv::Mat& source,
                                               bool resize_half) {
  cv::Mat pixels;
  if (resize_half) {
    cv::resize(source, pixels, cv::Size(source.cols / 2, source.rows / 2),
               0, 0, cv::INTER_AREA);
  } else {
    pixels = source;
  }
  auto image = std::make_shared<ManagedImage<PixelType>>(pixels.cols, pixels.rows);
  for (int y = 0; y < pixels.rows; ++y) {
    convert_copy_gray_image(pixels.cols, 1, pixels.ptr<uint8_t>(y), image->RowPtr(y));
  }
  return image;
}

}  // namespace granite
