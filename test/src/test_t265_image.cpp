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

#include <gtest/gtest.h>
#include <granite/device/t265_image.h>

namespace {
granite::Calibration<double> calibration() {
  granite::Calibration<double> c;
  for (int i = 0; i < 2; ++i) {
    granite::KannalaBrandtCamera4<double>::VecN p;
    p << 280.0 + i, 282.0 + i, 421.2 + i, 398.7 + i,
         0.01, -0.002, 0.0003, -0.00004;
    granite::GenericCamera<double> camera;
    camera.variant = granite::KannalaBrandtCamera4<double>(p);
    c.intrinsics.push_back(camera);
    c.resolution.emplace_back(848, 800);
    c.T_i_c.emplace_back(Sophus::SO3d(), Eigen::Vector3d(0.06 * i, 0, 0));
  }
  c.stereo_pairs.emplace_back(0, 1);
  return c;
}
}

TEST(T265Image, HalfProjectionMatchesAreaResize) {
  const auto full = calibration();
  auto half = full;
  granite::prepareT265Calibration(half, 0, true);
  ASSERT_EQ(half.intrinsics.size(), 2u);
  EXPECT_EQ(half.resolution[0], Eigen::Vector2i(424, 400));
  EXPECT_EQ(half.stereo_pairs.size(), 1u);
  for (size_t i = 0; i < 2; ++i) {
    Eigen::Vector4d ray(0.3, -0.2, 0.8, 0);
    Eigen::Vector2d a, b;
    ASSERT_TRUE(full.intrinsics[i].project(ray, a));
    ASSERT_TRUE(half.intrinsics[i].project(ray, b));
    EXPECT_TRUE(b.isApprox((a.array() + 0.5).matrix() * 0.5 - Eigen::Vector2d::Constant(0.5), 1e-12));
    EXPECT_TRUE(full.intrinsics[i].getParam().tail(4).isApprox(half.intrinsics[i].getParam().tail(4)));
    EXPECT_TRUE(full.T_i_c[i].matrix().isApprox(half.T_i_c[i].matrix()));
  }
}

TEST(T265Image, MonoRightPreservesSelectedCameraAndImu) {
  auto c = calibration();
  auto original = c;
  granite::prepareT265Calibration(c, 2, false);
  ASSERT_EQ(c.intrinsics.size(), 1u);
  ASSERT_EQ(c.T_i_c.size(), 1u);
  ASSERT_EQ(c.resolution.size(), 1u);
  EXPECT_TRUE(c.stereo_pairs.empty());
  EXPECT_TRUE(c.intrinsics[0].getParam().isApprox(original.intrinsics[1].getParam()));
  EXPECT_TRUE(c.T_i_c[0].matrix().isApprox(original.T_i_c[1].matrix()));
  EXPECT_EQ(c.imu_update_rate, original.imu_update_rate);
}

TEST(T265Image, MonoLeftHalfAndInvalidCamera) {
  auto c = calibration();
  const auto original = c;
  granite::prepareT265Calibration(c, 1, true);
  ASSERT_EQ(c.intrinsics.size(), 1u);
  EXPECT_TRUE(c.stereo_pairs.empty());
  EXPECT_DOUBLE_EQ(c.intrinsics[0].getParam()[0], original.intrinsics[0].getParam()[0] / 2);
  EXPECT_THROW(granite::prepareT265Calibration(c, 3, false), std::invalid_argument);
}

TEST(T265Image, AreaResizeUsesPaddedRows) {
  uint8_t pixels[] = {0, 4, 8, 12, 99, 99, 4, 8, 12, 16, 99, 99,
                      8, 12, 16, 20, 99, 99, 12, 16, 20, 24, 99, 99};
  cv::Mat src(4, 4, CV_8UC1, pixels, 6);
  auto half = granite::copyT265Image(src, true);
  ASSERT_EQ(half->w, 2u);
  ASSERT_EQ(half->h, 2u);
  const int factor = sizeof(granite::PixelType) == 2 ? 256 : 1;
  EXPECT_EQ((*half)(0, 0), 4 * factor);
  EXPECT_EQ((*half)(1, 0), 12 * factor);
  EXPECT_EQ((*half)(0, 1), 12 * factor);
  EXPECT_EQ((*half)(1, 1), 20 * factor);
  auto full = granite::copyT265Image(src, false);
  EXPECT_EQ((*full)(0, 1), 4 * factor);
  EXPECT_EQ((*full)(3, 3), 24 * factor);
}
