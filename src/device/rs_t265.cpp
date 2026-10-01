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

#include <granite/device/rs_t265.h>
#include <granite/device/t265_image.h>

namespace granite {

RsT265Device::RsT265Device(bool manual_exposure, int skip_frames,
                           int webp_quality, double exposure_value,
                           int mono_camera, bool resize_half)
    : manual_exposure(manual_exposure), skip_frames(skip_frames),
      webp_quality(webp_quality), exposure_value(exposure_value),
      mono_camera(mono_camera), resize_half(resize_half), pipe(context) {
  if (skip_frames < 1 || mono_camera < 0 || mono_camera > 2 || exposure_value < 0) {
    throw std::invalid_argument("Invalid T265 capture options");
  }
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  bool found = false;
  for (auto device : context.query_devices()) {
    if (std::string(device.get_info(RS2_CAMERA_INFO_NAME)).find("T265") != std::string::npos) {
      config.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
      found = true;
      break;
    }
  }
  if (!found) throw std::runtime_error("No Intel RealSense T265 connected");
  config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
  config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
  // Keep firmware tracking disabled: Granite consumes raw images and IMU.
}

RsT265Device::~RsT265Device() {
  if (running) {
    capturing = false;
    try { pipe.stop(); } catch (const rs2::error&) {}
  }
}

std::string RsT265Device::errorMessage() const {
  std::lock_guard<std::mutex> lock(error_mutex);
  return error_message;
}

void RsT265Device::start() {
  auto callback = [&](const rs2::frame& frame) {
    if (!capturing.load()) return;
    try {

    if (auto fp = frame.as<rs2::motion_frame>()) {
      auto motion = frame.as<rs2::motion_frame>();

      if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
          motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
        RsIMUData d;
        d.timestamp = motion.get_timestamp();
        d.data << motion.get_motion_data().x, motion.get_motion_data().y,
            motion.get_motion_data().z;

        gyro_data_queue.emplace_back(d);
      } else if (motion &&
                 motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                 motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
        RsIMUData d;
        d.timestamp = motion.get_timestamp();
        d.data << motion.get_motion_data().x, motion.get_motion_data().y,
            motion.get_motion_data().z;

        if (!prev_accel_data.get()) {
          prev_accel_data.reset(new RsIMUData(d));
        } else {
          if (d.timestamp <= prev_accel_data->timestamp) return;

          while (!gyro_data_queue.empty() && gyro_data_queue.front().timestamp <
                                                 prev_accel_data->timestamp) {
            std::cout << "Skipping gyro data. Timestamp before the first accel "
                         "measurement.";
            gyro_data_queue.pop_front();
          }

          while (!gyro_data_queue.empty() &&
                 gyro_data_queue.front().timestamp < d.timestamp) {
            RsIMUData gyro_data = gyro_data_queue.front();
            gyro_data_queue.pop_front();

            double w0 = (d.timestamp - gyro_data.timestamp) /
                        (d.timestamp - prev_accel_data->timestamp);

            double w1 = (gyro_data.timestamp - prev_accel_data->timestamp) /
                        (d.timestamp - prev_accel_data->timestamp);

            Eigen::Vector3d accel_interpolated =
                w0 * prev_accel_data->data + w1 * d.data;

            granite::ImuData::Ptr data;
            data.reset(new granite::ImuData);
            data->t_ns = gyro_data.timestamp * 1e6;
            data->accel = accel_interpolated;
            data->gyro = gyro_data.data;

            if (imu_data_queue) {
              imu_data_queue->push(data);
              ++imu_samples;
            }
          }

          prev_accel_data.reset(new RsIMUData(d));
        }
      }
    } else if (auto fs = frame.as<rs2::frameset>()) {
      auto left = fs.get_fisheye_frame(1);
      auto right = fs.get_fisheye_frame(2);
      if ((mono_camera != 2 && !left) || (mono_camera != 1 && !right)) return;
      const auto reference = mono_camera == 2 ? right : left;
      last_image_t_ns = static_cast<int64_t>(reference.get_timestamp() * 1e6);
      ++image_frames;
      if (frame_counter++ % skip_frames.load() != 0) return;

      OpticalFlowInput::Ptr data(new OpticalFlowInput);
      data->t_ns = last_image_t_ns.load();
      const int first = mono_camera == 0 ? 1 : mono_camera;
      const int count = mono_camera == 0 ? 2 : 1;
      data->img_data.resize(count);
      for (int i = 0; i < count; ++i) {
        const auto vf = fs.get_fisheye_frame(first + i);
        if (vf.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
          data->img_data[i].exposure =
              vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-6;
        }
        cv::Mat pixels(vf.get_height(), vf.get_width(), CV_8UC1,
                       const_cast<void*>(vf.get_data()), vf.get_stride_in_bytes());
        data->img_data[i].img = copyT265Image(pixels, resize_half);
      }
      if (image_data_queue) {
        if (image_data_queue->try_push(data)) ++queued_frames;
        else ++dropped_frames;
      }

    } else if (auto pf = frame.as<rs2::pose_frame>()) {
      auto data = pf.get_pose_data();

      RsPoseData pdata;
      pdata.t_ns = pf.get_timestamp() * 1e6;

      Eigen::Vector3d trans(data.translation.x, data.translation.y,
                            data.translation.z);
      Eigen::Quaterniond quat(data.rotation.w, data.rotation.x, data.rotation.y,
                              data.rotation.z);

      pdata.data = Sophus::SE3d(quat, trans);

      if (pose_data_queue) pose_data_queue->push(pdata);
    }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(error_mutex);
      error_message = error.what();
      failed = true;
      capturing = false;
    }
  };

  profile = pipe.start(config, callback);
  running = true;
  sensor = profile.get_device().first<rs2::sensor>();
  if (manual_exposure) {
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    setExposure(exposure_value);
  }
  exportCalibration();
}

void RsT265Device::enableCapture() { capturing = true; }

void RsT265Device::stop(bool send_end_of_stream) {
  capturing = false;
  if (running) {
    pipe.stop();
    running = false;
  }
  if (!send_end_of_stream) return;
  if (image_data_queue) image_data_queue->push(nullptr);
  if (imu_data_queue) imu_data_queue->push(nullptr);
}

bool RsT265Device::setExposure(double exposure) {
  if (!manual_exposure) return false;

  sensor.set_option(rs2_option::RS2_OPTION_EXPOSURE, exposure * 1000);
  return true;
}

void RsT265Device::setSkipFrames(int skip) {
  if (skip < 1) throw std::invalid_argument("Frame stride must be positive");
  skip_frames = skip;
}

void RsT265Device::setWebpQuality(int quality) { webp_quality = quality; }

std::shared_ptr<granite::Calibration<double>> RsT265Device::exportCalibration() {
  using Scalar = double;

  if (calib.get()) return calib;

  calib.reset(new granite::Calibration<Scalar>);
  calib->imu_update_rate = IMU_RATE;
  calib->cam_update_rate = 30;

  auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL);
  auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
  auto cam0_stream = profile.get_stream(RS2_STREAM_FISHEYE, 1);
  auto cam1_stream = profile.get_stream(RS2_STREAM_FISHEYE, 2);

  // get gyro extrinsics
  if (auto gyro = gyro_stream.as<rs2::motion_stream_profile>()) {
    rs2_motion_device_intrinsic intrinsics = gyro.get_motion_intrinsics();

    Eigen::Matrix<Scalar, 12, 1> gyro_bias_full;
    gyro_bias_full << intrinsics.data[0][3], intrinsics.data[1][3],
        intrinsics.data[2][3], intrinsics.data[0][0] - 1.0,
        intrinsics.data[1][0], intrinsics.data[2][0], intrinsics.data[0][1],
        intrinsics.data[1][1] - 1.0, intrinsics.data[2][1],
        intrinsics.data[0][2], intrinsics.data[1][2],
        intrinsics.data[2][2] - 1.0;
    granite::CalibGyroBias<Scalar> gyro_bias;
    gyro_bias.getParam() = gyro_bias_full;
    calib->calib_gyro_bias = gyro_bias;

    // std::cout << "Gyro Bias\n" << gyro_bias_full << std::endl;

    calib->gyro_noise_std = Eigen::Vector3d(intrinsics.noise_variances[0],
                                            intrinsics.noise_variances[1],
                                            intrinsics.noise_variances[2])
                                .cwiseSqrt();

    calib->gyro_bias_std = Eigen::Vector3d(intrinsics.bias_variances[0],
                                           intrinsics.bias_variances[1],
                                           intrinsics.bias_variances[2])
                               .cwiseSqrt();

    // std::cout << "Gyro noise var: " << intrinsics.noise_variances[0]
    //          << " bias var: " << intrinsics.bias_variances[0] << std::endl;
  } else {
    std::abort();
  }

  // get accel extrinsics
  if (auto accel = accel_stream.as<rs2::motion_stream_profile>()) {
    rs2_motion_device_intrinsic intrinsics = accel.get_motion_intrinsics();
    Eigen::Matrix<Scalar, 9, 1> accel_bias_full;
    accel_bias_full << intrinsics.data[0][3], intrinsics.data[1][3],
        intrinsics.data[2][3], intrinsics.data[0][0] - 1.0,
        intrinsics.data[1][0], intrinsics.data[2][0],
        intrinsics.data[1][1] - 1.0, intrinsics.data[2][1],
        intrinsics.data[2][2] - 1.0;
    granite::CalibAccelBias<Scalar> accel_bias;
    accel_bias.getParam() = accel_bias_full;
    calib->calib_accel_bias = accel_bias;

    // std::cout << "Gyro Bias\n" << accel_bias_full << std::endl;

    calib->accel_noise_std = Eigen::Vector3d(intrinsics.noise_variances[0],
                                             intrinsics.noise_variances[1],
                                             intrinsics.noise_variances[2])
                                 .cwiseSqrt();

    calib->accel_bias_std = Eigen::Vector3d(intrinsics.bias_variances[0],
                                            intrinsics.bias_variances[1],
                                            intrinsics.bias_variances[2])
                                .cwiseSqrt();

    // std::cout << "Accel noise var: " << intrinsics.noise_variances[0]
    //          << " bias var: " << intrinsics.bias_variances[0] << std::endl;
  } else {
    std::abort();
  }

  // get camera ex-/intrinsics
  for (const auto& cam_stream : {cam0_stream, cam1_stream}) {
    if (auto cam = cam_stream.as<rs2::video_stream_profile>()) {
      // extrinsics
      rs2_extrinsics ex = cam.get_extrinsics_to(gyro_stream);
      Eigen::Matrix3f rot = Eigen::Map<Eigen::Matrix3f>(ex.rotation);
      Eigen::Vector3f trans = Eigen::Map<Eigen::Vector3f>(ex.translation);

      Eigen::Quaterniond q(rot.cast<double>());
      granite::Calibration<Scalar>::SE3 T_i_c(q, trans.cast<double>());

      // std::cout << "T_i_c\n" << T_i_c.matrix() << std::endl;

      calib->T_i_c.push_back(T_i_c);

      // get resolution
      Eigen::Vector2i resolution;
      resolution << cam.width(), cam.height();
      calib->resolution.push_back(resolution);

      // intrinsics
      rs2_intrinsics intrinsics = cam.get_intrinsics();
      granite::KannalaBrandtCamera4<Scalar>::VecN params;
      params << intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
          intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
          intrinsics.coeffs[3];

      // std::cout << "Camera intrinsics: " << params.transpose() << std::endl;

      granite::GenericCamera<Scalar> camera;
      granite::KannalaBrandtCamera4 kannala_brandt(params);
      camera.variant = kannala_brandt;

      calib->intrinsics.push_back(camera);
    } else {
      std::abort();
    }
  }

  prepareT265Calibration(*calib, mono_camera, resize_half);
  return calib;
}

}  // namespace granite
