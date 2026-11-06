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

#include <string>

namespace granite {

struct VioConfig {
  VioConfig();
  void load(const std::string& filename);
  void save(const std::string& filename);

  std::string optical_flow_type;
  int optical_flow_detection_grid_size;
  float optical_flow_max_recovered_dist2;
  int optical_flow_pattern;
  int optical_flow_max_iterations;
  int optical_flow_levels;
  float optical_flow_epipolar_error;
  int optical_flow_skip_frames;

  int vio_max_states;
  int vio_max_kfs;
  int vio_min_frames_after_kf;
  double vio_fraction_entropy_take_kf;
  float vio_new_kf_keypoints_thresh;
  bool vio_debug;

  double vio_outlier_threshold;
  int vio_take_keyframe_iteration;
  int vio_filter_iteration;
  int vio_max_iterations;

  double vio_obs_std_dev;
  double vio_obs_huber_thresh;
  double vio_no_motion_reg_weight;
  double vio_min_triangulation_dist;
  double vio_max_inverse_distance;

  bool vio_enforce_realtime;

  bool vio_use_lm;
  double vio_lm_lambda_min;
  double vio_lm_lambda_max;

  double vio_init_pose_weight;
  int vio_mono_init_max_ransac_iter;
  double vio_mono_init_min_parallax;
  double vio_init_ba_weight;
  double vio_init_bg_weight;

  // Mono VIO: accept a visual-inertial alignment only after this many
  // successive alignments of the same bootstrap map agree (1 accepts the
  // first one). Agreement: scale ratio and gravity direction within the
  // tolerances below for every pair in the run.
  int vio_mono_init_consistent_alignments;
  double vio_mono_init_consistency_scale;
  double vio_mono_init_consistency_gravity_deg;

  // Mono VIO health monitor and re-initialization. After settle + window
  // seconds from an initialization, the metric estimator is unhealthy when,
  // over the last window, the rms correction of the previously published
  // state exceeds health_rot_deg in rotation while staying below health_pos_m
  // in translation. Then a new bootstrap runs beside it, and the new metric
  // estimator takes over with the published position and heading. At most
  // reinit_max re-initializations (0 disables them).
  int vio_mono_reinit_max;
  double vio_mono_health_settle_s;
  double vio_mono_health_window_s;
  double vio_mono_health_rot_deg;
  double vio_mono_health_pos_m;
  // Also unhealthy: a bias estimate above these norms [m/s^2], [rad/s].
  double vio_mono_health_max_accel_bias;
  double vio_mono_health_max_gyro_bias;

  // Sum the bundle adjustment linearization in a fixed order
  // (tbb::parallel_deterministic_reduce), so that the result does not depend
  // on the number of TBB threads or on their timing. About 7 % slower.
  bool vio_use_deterministic;

  // Gauge gate of the VIO bundle adjustment. Vision and IMU residuals do not
  // change when all poses and states move by the same translation. Only the
  // weak position information of the marginalization prior holds the window,
  // and that information is coupled to the other states. An almost undamped
  // LM step can then move the whole window by meters in one frame. When the
  // common translation of a step is more than vio_gauge_gate_m, the gate adds
  // vio_gauge_gate_weight to the common translation direction of the
  // normal equations and solves again. Off by default.
  bool vio_gauge_gate;
  double vio_gauge_gate_m;
  double vio_gauge_gate_weight;

  // Monocular VIO initialization: "vo_alignment" (bootstrap VO aligned to the
  // IMU) or "inertial" (the visual-inertial initializer ported from openvio)
  std::string vio_mono_init_method;
  double vio_mono_init_window_time;
  double vio_mono_init_excitation_accel_std;
  double vio_mono_init_max_still_disparity;
  int vio_mono_init_max_tracks;
  int vio_mono_init_num_poses;
  double vio_mono_init_min_rotation_deg;
  int vio_mono_init_refine_max_iterations;
  double vio_mono_init_pixel_sigma;
  double vio_mono_init_prior_sigma_gyro_bias;
  double vio_mono_init_prior_sigma_accel_bias;
  double vio_mono_init_covariance_scale_orientation;
  double vio_mono_init_covariance_scale_velocity;
  double vio_mono_init_covariance_scale_gyro_bias;
  double vio_mono_init_covariance_scale_accel_bias;
  double vio_mono_init_rest_accel_bias_sigma;
  bool vio_mono_init_replay_window;
  double vio_mono_init_rest_tilt_bias_sigma;
  double vio_mono_init_max_relative_speed_sigma;
  // Start at rest only when the mean pixel motion of the older half window is
  // at most this [px] (0: off). A constant velocity has no IMU excitation and
  // can move the pixels less than vio_mono_init_max_still_disparity. The
  // start at rest then sets the speed to zero. Not in openvio.
  double vio_mono_init_max_rest_disparity;
  // Data window of the moving start [s] (0: vio_mono_init_window_time). The
  // start at rest keeps vio_mono_init_window_time. Not in openvio.
  double vio_mono_init_moving_window_time;
  // Moving start: use the tracks of the largest parallax after the gyro
  // rotation (they span at least half the window) instead of the longest
  // tracks, which see the farthest points. Not in openvio.
  bool vio_mono_init_parallax_tracks;
  // Reject a moving start when fewer than this fraction of its tracks have a
  // point in front of every ray (0: only the fixed minimum of 8 points). Not
  // in openvio.
  double vio_mono_init_min_refined_fraction;
  // false: never use the moving start, wait for a start at rest (true: the
  // moving start of openvio when the platform moves).
  bool vio_mono_init_moving_start;

  double mapper_obs_std_dev;
  double mapper_obs_huber_thresh;
  int mapper_detection_num_points;
  double mapper_num_frames_to_match;
  double mapper_frames_to_match_threshold;
  double mapper_min_matches;
  double mapper_ransac_threshold;
  double mapper_min_track_length;
  double mapper_max_hamming_distance;
  double mapper_second_best_test_ratio;
  int mapper_bow_num_bits;
  double mapper_min_triangulation_dist;
  bool mapper_no_factor_weights;
  bool mapper_use_factors;

  bool mapper_use_lm;
  double mapper_lm_lambda_min;
  double mapper_lm_lambda_max;
};
}  // namespace granite
