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

#include <granite/utils/vio_config.h>

#include <fstream>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>

namespace granite {

VioConfig::VioConfig() {
  // optical_flow_type = "patch";
  optical_flow_type = "frame_to_frame";
  optical_flow_detection_grid_size = 50;
  optical_flow_max_recovered_dist2 = 0.09f;
  optical_flow_pattern = 51;
  optical_flow_max_iterations = 5;
  optical_flow_levels = 3;
  optical_flow_epipolar_error = 0.005;
  optical_flow_skip_frames = 1;

  vio_max_states = 3;
  vio_max_kfs = 7;
  vio_min_frames_after_kf = 5;
  vio_fraction_entropy_take_kf = 0.995;
  vio_new_kf_keypoints_thresh = 0.7;

  vio_debug = false;
  vio_obs_std_dev = 0.5;
  vio_obs_huber_thresh = 1.0;
  vio_no_motion_reg_weight = 1e-6;
  vio_min_triangulation_dist = 0.05;
  vio_max_inverse_distance = 5.0;
  vio_outlier_threshold = 3.0;
  vio_take_keyframe_iteration = 2;
  vio_filter_iteration = 4;
  vio_max_iterations = 7;

  vio_enforce_realtime = false;

  vio_use_lm = false;
  vio_lm_lambda_min = 1e-32;
  vio_lm_lambda_max = 1e2;

  vio_init_pose_weight = 1e8;
  vio_mono_init_max_ransac_iter = 20;
  vio_mono_init_min_parallax = 2.5;
  vio_init_ba_weight = 1e1;
  vio_init_bg_weight = 1e2;

  vio_mono_init_consistent_alignments = 2;
  vio_mono_init_consistency_scale = 0.03;
  vio_mono_init_consistency_gravity_deg = 1.5;

  vio_mono_reinit_max = 0;
  vio_mono_health_settle_s = 3.0;
  vio_mono_health_window_s = 2.0;
  vio_mono_health_rot_deg = 0.07;
  vio_mono_health_pos_m = 0.005;
  vio_mono_health_max_accel_bias = 2.0;
  vio_mono_health_max_gyro_bias = 0.1;

  vio_use_deterministic = false;
  vio_gauge_gate = false;
  vio_gauge_gate_m = 0.0;
  vio_gauge_gate_weight = 50.0;

  vio_mono_init_method = "vo_alignment";
  vio_mono_init_window_time = 2.0;
  vio_mono_init_excitation_accel_std = 1.0;
  vio_mono_init_max_still_disparity = 8.7;
  vio_mono_init_max_tracks = 50;
  vio_mono_init_num_poses = 6;
  vio_mono_init_min_rotation_deg = 10.0;
  vio_mono_init_refine_max_iterations = 50;
  vio_mono_init_pixel_sigma = 1.0;
  vio_mono_init_prior_sigma_gyro_bias = 0.05;
  vio_mono_init_prior_sigma_accel_bias = 0.10;
  vio_mono_init_covariance_scale_orientation = 10.0;
  vio_mono_init_covariance_scale_velocity = 1.0;
  vio_mono_init_covariance_scale_gyro_bias = 10.0;
  vio_mono_init_covariance_scale_accel_bias = 9.0;
  vio_mono_init_rest_accel_bias_sigma = 0.03;
  vio_mono_init_replay_window = false;
  vio_mono_init_rest_tilt_bias_sigma = 0.0;
  vio_mono_init_max_relative_speed_sigma = 0.05;

  mapper_obs_std_dev = 0.25;
  mapper_obs_huber_thresh = 1.5;
  mapper_detection_num_points = 800;
  mapper_num_frames_to_match = 30;
  mapper_frames_to_match_threshold = 0.04;
  mapper_min_matches = 20;
  mapper_ransac_threshold = 5e-5;
  mapper_min_track_length = 5;
  mapper_max_hamming_distance = 70;
  mapper_second_best_test_ratio = 1.2;
  mapper_bow_num_bits = 16;
  mapper_min_triangulation_dist = 0.07;
  mapper_no_factor_weights = false;
  mapper_use_factors = true;

  mapper_use_lm = false;
  mapper_lm_lambda_min = 1e-32;
  mapper_lm_lambda_max = 1e2;
}

void VioConfig::save(const std::string& filename) {
  std::ofstream os(filename);

  {
    cereal::JSONOutputArchive archive(os);
    archive(*this);
  }
  os.close();
}

void VioConfig::load(const std::string& filename) {
  std::ifstream is(filename);

  if (!is.is_open()) {
    std::cerr << "Can not open " << filename << std::endl;
  }

  {
    cereal::JSONInputArchive archive(is);
    archive(*this);
  }
  is.close();
}
}  // namespace granite

namespace cereal {

template <class Archive, class T>
void optional_nvp(Archive& ar, const char* name, T& value) {
  if constexpr (Archive::is_loading::value) {
    try {
      ar(cereal::make_nvp(name, value));
    } catch (const cereal::Exception&) {
    }
  } else {
    ar(cereal::make_nvp(name, value));
  }
}

template <class Archive>
void serialize(Archive& ar, granite::VioConfig& config) {
  ar(CEREAL_NVP(config.optical_flow_type));
  ar(CEREAL_NVP(config.optical_flow_detection_grid_size));
  ar(CEREAL_NVP(config.optical_flow_max_recovered_dist2));
  ar(CEREAL_NVP(config.optical_flow_pattern));
  ar(CEREAL_NVP(config.optical_flow_max_iterations));
  ar(CEREAL_NVP(config.optical_flow_epipolar_error));
  ar(CEREAL_NVP(config.optical_flow_levels));
  ar(CEREAL_NVP(config.optical_flow_skip_frames));

  ar(CEREAL_NVP(config.vio_max_states));
  ar(CEREAL_NVP(config.vio_max_kfs));
  ar(CEREAL_NVP(config.vio_min_frames_after_kf));
  ar(CEREAL_NVP(config.vio_fraction_entropy_take_kf));
  ar(CEREAL_NVP(config.vio_new_kf_keypoints_thresh));
  ar(CEREAL_NVP(config.vio_debug));
  ar(CEREAL_NVP(config.vio_max_iterations));
  ar(CEREAL_NVP(config.vio_outlier_threshold));
  ar(CEREAL_NVP(config.vio_take_keyframe_iteration));
  ar(CEREAL_NVP(config.vio_filter_iteration));

  ar(CEREAL_NVP(config.vio_obs_std_dev));
  ar(CEREAL_NVP(config.vio_obs_huber_thresh));
  ar(CEREAL_NVP(config.vio_no_motion_reg_weight));
  ar(CEREAL_NVP(config.vio_min_triangulation_dist));
  ar(CEREAL_NVP(config.vio_max_inverse_distance));

  ar(CEREAL_NVP(config.vio_enforce_realtime));

  ar(CEREAL_NVP(config.vio_use_lm));
  ar(CEREAL_NVP(config.vio_lm_lambda_min));
  ar(CEREAL_NVP(config.vio_lm_lambda_max));

  ar(CEREAL_NVP(config.vio_init_pose_weight));
  ar(CEREAL_NVP(config.vio_mono_init_max_ransac_iter));
  ar(CEREAL_NVP(config.vio_mono_init_min_parallax));
  ar(CEREAL_NVP(config.vio_init_ba_weight));
  ar(CEREAL_NVP(config.vio_init_bg_weight));
  // Newer options: configuration files without them keep the defaults.
  optional_nvp(ar, "config.vio_mono_init_consistent_alignments",
               config.vio_mono_init_consistent_alignments);
  optional_nvp(ar, "config.vio_mono_init_consistency_scale",
               config.vio_mono_init_consistency_scale);
  optional_nvp(ar, "config.vio_mono_init_consistency_gravity_deg",
               config.vio_mono_init_consistency_gravity_deg);
  optional_nvp(ar, "config.vio_mono_reinit_max", config.vio_mono_reinit_max);
  optional_nvp(ar, "config.vio_mono_health_settle_s", config.vio_mono_health_settle_s);
  optional_nvp(ar, "config.vio_mono_health_window_s", config.vio_mono_health_window_s);
  optional_nvp(ar, "config.vio_mono_health_rot_deg", config.vio_mono_health_rot_deg);
  optional_nvp(ar, "config.vio_mono_health_pos_m", config.vio_mono_health_pos_m);
  optional_nvp(ar, "config.vio_mono_health_max_accel_bias",
               config.vio_mono_health_max_accel_bias);
  optional_nvp(ar, "config.vio_mono_health_max_gyro_bias",
               config.vio_mono_health_max_gyro_bias);
  optional_nvp(ar, "config.vio_use_deterministic", config.vio_use_deterministic);
  optional_nvp(ar, "config.vio_gauge_gate", config.vio_gauge_gate);
  optional_nvp(ar, "config.vio_gauge_gate_m", config.vio_gauge_gate_m);
  optional_nvp(ar, "config.vio_gauge_gate_weight", config.vio_gauge_gate_weight);

  // Optional: configs written before these options keep their defaults
  optional_nvp(ar, "config.vio_mono_init_method", config.vio_mono_init_method);
  optional_nvp(ar, "config.vio_mono_init_window_time", config.vio_mono_init_window_time);
  optional_nvp(ar, "config.vio_mono_init_excitation_accel_std", config.vio_mono_init_excitation_accel_std);
  optional_nvp(ar, "config.vio_mono_init_max_still_disparity", config.vio_mono_init_max_still_disparity);
  optional_nvp(ar, "config.vio_mono_init_max_tracks", config.vio_mono_init_max_tracks);
  optional_nvp(ar, "config.vio_mono_init_num_poses", config.vio_mono_init_num_poses);
  optional_nvp(ar, "config.vio_mono_init_min_rotation_deg", config.vio_mono_init_min_rotation_deg);
  optional_nvp(ar, "config.vio_mono_init_refine_max_iterations", config.vio_mono_init_refine_max_iterations);
  optional_nvp(ar, "config.vio_mono_init_pixel_sigma", config.vio_mono_init_pixel_sigma);
  optional_nvp(ar, "config.vio_mono_init_prior_sigma_gyro_bias", config.vio_mono_init_prior_sigma_gyro_bias);
  optional_nvp(ar, "config.vio_mono_init_prior_sigma_accel_bias", config.vio_mono_init_prior_sigma_accel_bias);
  optional_nvp(ar, "config.vio_mono_init_covariance_scale_orientation", config.vio_mono_init_covariance_scale_orientation);
  optional_nvp(ar, "config.vio_mono_init_covariance_scale_velocity", config.vio_mono_init_covariance_scale_velocity);
  optional_nvp(ar, "config.vio_mono_init_covariance_scale_gyro_bias", config.vio_mono_init_covariance_scale_gyro_bias);
  optional_nvp(ar, "config.vio_mono_init_covariance_scale_accel_bias", config.vio_mono_init_covariance_scale_accel_bias);
  optional_nvp(ar, "config.vio_mono_init_rest_accel_bias_sigma", config.vio_mono_init_rest_accel_bias_sigma);
  optional_nvp(ar, "config.vio_mono_init_replay_window", config.vio_mono_init_replay_window);
  optional_nvp(ar, "config.vio_mono_init_rest_tilt_bias_sigma", config.vio_mono_init_rest_tilt_bias_sigma);
  optional_nvp(ar, "config.vio_mono_init_max_relative_speed_sigma", config.vio_mono_init_max_relative_speed_sigma);

  ar(CEREAL_NVP(config.mapper_obs_std_dev));
  ar(CEREAL_NVP(config.mapper_obs_huber_thresh));
  ar(CEREAL_NVP(config.mapper_detection_num_points));
  ar(CEREAL_NVP(config.mapper_num_frames_to_match));
  ar(CEREAL_NVP(config.mapper_frames_to_match_threshold));
  ar(CEREAL_NVP(config.mapper_min_matches));
  ar(CEREAL_NVP(config.mapper_ransac_threshold));
  ar(CEREAL_NVP(config.mapper_min_track_length));
  ar(CEREAL_NVP(config.mapper_max_hamming_distance));
  ar(CEREAL_NVP(config.mapper_second_best_test_ratio));
  ar(CEREAL_NVP(config.mapper_bow_num_bits));
  ar(CEREAL_NVP(config.mapper_min_triangulation_dist));
  ar(CEREAL_NVP(config.mapper_no_factor_weights));
  ar(CEREAL_NVP(config.mapper_use_factors));

  ar(CEREAL_NVP(config.mapper_use_lm));
  ar(CEREAL_NVP(config.mapper_lm_lambda_min));
  ar(CEREAL_NVP(config.mapper_lm_lambda_max));
}
}  // namespace cereal
