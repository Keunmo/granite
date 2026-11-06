// SPDX-License-Identifier: MIT
// Monocular metric landmark lifecycle. Granite's original VO remains separate.
#include <granite/vi_estimator/keypoint_vio.h>
#include <opencv2/calib3d.hpp>
#include <Eigen/SVD>
#include <iomanip>
#include <iostream>

namespace granite {
void KeypointVioEstimator::updateMonoLandmarks(const OpticalFlowResult::Ptr& frame,
                                             bool create_new) {
  const TimeCamId current(frame->t_ns, 0);
  if (!create_new) recoverMonoPose(frame);
  // Optical-flow IDs identify continuous tracks. Do not resurrect a known
  // bad association as a fresh map point after its old host is marginalized.
  // Keep this rejection memory bounded by currently present track IDs.
  for (auto it=mono_rejected_tracks.begin(); it!=mono_rejected_tracks.end();) {
    if (!frame->observations[0].count(*it)) it=mono_rejected_tracks.erase(it);
    else ++it;
  }
  const double min_baseline2 = config.vio_min_triangulation_dist * config.vio_min_triangulation_dist;
  const double min_parallax = std::max(1e-3, 2 * config.vio_obs_std_dev /
                                      std::abs(calib.intrinsics[0].getParam()[0]));
  for (const auto& observation : frame->observations[0]) {
    const int id = observation.first;
    const bool exists = lmdb.landmarkExists(id);
    if (create_new && exists) continue;
    if (!exists && mono_rejected_tracks.count(id)) continue;
    if (exists && lmdb.getLandmark(id).id > 0) {
      // Gate gross finite-track mismatches against the propagated metric pose
      // before robust optimization; a frame of wrong matches must not drag
      // the entire inertial state toward a coherent image displacement.
      if (!create_new) {
        const auto& point=lmdb.getLandmark(id);
        Eigen::Vector4d ray=StereographicParam<double>::unproject(point.dir);
        ray[3]=point.id;
        const Sophus::SE3d current_from_host=
            (getPoseStateWithLin(current.frame_id).getPose()*calib.T_i_c[0]).inverse()*
            (getPoseStateWithLin(point.kf_id.frame_id).getPose()*calib.T_i_c[0]);
        Eigen::Vector2d pixel;
        if (!calib.intrinsics[0].project(current_from_host.matrix()*ray,pixel) ||
            (pixel-observation.second.translation().cast<double>()).norm() > config.vio_outlier_threshold)
          {
            mono_rejected_tracks.insert(id);
            if (lmdb.getObservationsOfLandmark(id).count(current)) lmdb.removeObservations(id,{current});
          }
        else mono_rejected_tracks.erase(id);
      }
      continue;
    }
    if (!exists && !create_new) continue;
    const TimeCamId host = exists ? lmdb.getLandmark(id).kf_id : current;
    const auto host_frame = prev_opt_flow_res.find(host.frame_id);
    if (host_frame == prev_opt_flow_res.end()) continue;
    const auto host_obs = host_frame->second->observations[0].find(id);
    if (host_obs == host_frame->second->observations[0].end()) continue;
    const Eigen::Vector2d host_pixel = host_obs->second.translation().cast<double>();
    Eigen::Vector4d host_ray;
    if (!calib.intrinsics[0].unproject(host_pixel, host_ray)) continue;
    host_ray.head<3>().normalize(); host_ray[3] = 0;
    const auto host_pose = getPoseStateWithLin(host.frame_id).getPose() * calib.T_i_c[0];
    if (exists) {
      const Sophus::SE3d current_from_host=
          (getPoseStateWithLin(current.frame_id).getPose()*calib.T_i_c[0]).inverse()*host_pose;
      Eigen::Vector4d ray;
      if (calib.intrinsics[0].unproject(observation.second.translation().cast<double>(),ray)) {
        const Eigen::Vector3d plane=current_from_host.translation().cross(current_from_host.so3()*host_ray.head<3>());
        // Depth-free epipolar consistency distinguishes valid translation
        // parallax from a wrong association. Keep that rejection across host
        // removal, while genuine near bearings remain eligible for promotion.
        if (current_from_host.translation().norm()>1e-4 && plane.norm()>1e-8 &&
            std::abs(plane.normalized().dot(ray.head<3>().normalized()))*
                std::abs(calib.intrinsics[0].getParam()[0]) > config.vio_outlier_threshold) {
          mono_rejected_tracks.insert(id);
          if (lmdb.getObservationsOfLandmark(id).count(current)) lmdb.removeObservations(id,{current});
          continue;
        }
        mono_rejected_tracks.erase(id);
      }
      Eigen::Vector2d infinity_pixel;
      // Keep a bearing at infinity while that model still explains the current
      // image. Small inertial rotation errors must not manufacture finite depth
      // from the translation baseline accumulated during an image outage.
      if(calib.intrinsics[0].project(current_from_host.matrix()*host_ray,infinity_pixel) &&
          (infinity_pixel-observation.second.translation().cast<double>()).norm()<=config.vio_obs_huber_thresh)
        continue;
    }

    std::map<TimeCamId, KeypointObservation> observations;
    std::vector<std::pair<double, Eigen::Vector4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Vector4d>>> candidates;
    double best_baseline = -1;
    Eigen::Vector4d best_point = host_ray;
    for (const auto& prior : prev_opt_flow_res) {
      const auto found = prior.second->observations[0].find(id);
      if (found == prior.second->observations[0].end()) continue;
      const TimeCamId target(prior.first, 0);
      const Eigen::Vector2d pixel = found->second.translation().cast<double>();
      observations.emplace(target, KeypointObservation{id, pixel});
      if (target == host) continue;
      Eigen::Vector4d ray;
      if (!calib.intrinsics[0].unproject(pixel, ray)) continue;
      ray.head<3>().normalize();
      const Sophus::SE3d relative = host_pose.inverse() *
          (getPoseStateWithLin(target.frame_id).getPose() * calib.T_i_c[0]);
      const double baseline = relative.translation().squaredNorm();
      const double parallax = host_ray.head<3>().cross(relative.so3() * ray.head<3>()).norm();
      if (baseline < min_baseline2 || parallax < min_parallax) continue;
      const Eigen::Vector4d point = triangulate(host_ray.head<3>(), ray.head<3>(), relative);
      if (!point.allFinite() || point[3] <= 0 || point[3] >= config.vio_max_inverse_distance) continue;
      // Check both views before selecting the largest supported metric baseline.
      Eigen::Vector2d reprojection;
      if (!calib.intrinsics[0].project(point, reprojection) ||
          (reprojection-host_pixel).norm() > config.vio_outlier_threshold) continue;
      const Eigen::Vector4d target_point = relative.inverse().matrix() * point;
      if (target_point.head<3>().dot(ray.head<3>()) <= 0 ||
          !calib.intrinsics[0].project(target_point, reprojection) ||
          (reprojection-pixel).norm() > config.vio_outlier_threshold) continue;
      candidates.emplace_back(baseline,point);
    }
    // Evaluate every candidate against the full retained history. A far
    // outlier must not hide a nearer, well-supported triangulation.
    for (const auto& candidate : candidates) {
      if (candidate.first <= best_baseline) continue;
      size_t valid = 0;
      for (const auto& target : observations) {
        const Sophus::SE3d target_from_host =
            (getPoseStateWithLin(target.first.frame_id).getPose()*calib.T_i_c[0]).inverse()*host_pose;
        Eigen::Vector2d pixel;
        if (calib.intrinsics[0].project(target_from_host.matrix()*candidate.second,pixel) &&
            (pixel-target.second.pos).norm() <= config.vio_outlier_threshold) ++valid;
      }
      // Bad observations cannot promote a rotation-only bearing to bogus depth.
      if (valid >= 2 && valid * 4 >= observations.size() * 3) {
        best_baseline=candidate.first; best_point=candidate.second;
      }
    }
    if (exists) {
      if (best_baseline < 0) {
        // A near bearing may cease to be an infinity observation before its
        // metric baseline is sufficient for depth. Do not force that parallax
        // into camera rotation/bias while awaiting triangulation.
        Eigen::Vector2d predicted;
        const Sophus::SE3d current_from_host =
            (getPoseStateWithLin(current.frame_id).getPose()*calib.T_i_c[0]).inverse()*host_pose;
        if (!calib.intrinsics[0].project(current_from_host.matrix()*host_ray,predicted) ||
            (predicted-observation.second.translation().cast<double>()).norm() > config.vio_obs_huber_thresh)
          if (lmdb.getObservationsOfLandmark(id).count(current))
            lmdb.removeObservations(id, {current});
      }
      if (best_baseline >= 0) {
        auto& point = lmdb.getLandmark(id);
        point.dir = StereographicParam<double>::project(best_point);
        point.id = best_point[3];
        // Historical views can establish depth even if the current match is
        // an outlier. Promotion must not bypass the finite observation gate.
        std::set<TimeCamId> rejected;
        const auto accepted=lmdb.getObservationsOfLandmark(id);
        for (const auto& target:accepted) {
          const Sophus::SE3d target_from_host=
              (getPoseStateWithLin(target.first.frame_id).getPose()*calib.T_i_c[0]).inverse()*host_pose;
          Eigen::Vector2d pixel;
          if (!calib.intrinsics[0].project(target_from_host.matrix()*best_point,pixel) ||
              (pixel-target.second.pos).norm() > config.vio_outlier_threshold)
            rejected.insert(target.first);
        }
        if (rejected.count(current)) mono_rejected_tracks.insert(id);
        if (!rejected.empty()) lmdb.removeObservations(id,rejected);
      }
    } else {
      KeypointPosition point;
      point.kf_id = host;
      point.dir = StereographicParam<double>::project(best_baseline >= 0 ? best_point : host_ray);
      point.id = best_baseline >= 0 ? best_point[3] : 0;
      const auto level = host_frame->second->pyramid_levels[0].find(id);
      point.weight = level == host_frame->second->pyramid_levels[0].end()
          ? 1.0 : std::ldexp(1.0, -level->second);
      lmdb.addLandmark(id, point);
      ++num_points_kf[host.frame_id];
      // A newly born point must satisfy the same observation gates as an
      // existing point; low baseline does not imply zero image parallax.
      const Eigen::Vector4d model=best_baseline>=0 ? best_point : host_ray;
      for (const auto& target : observations) {
        const Sophus::SE3d target_from_host=
            (getPoseStateWithLin(target.first.frame_id).getPose()*calib.T_i_c[0]).inverse()*host_pose;
        Eigen::Vector2d pixel;
        const double threshold=point.id>0 ? config.vio_outlier_threshold : config.vio_obs_huber_thresh;
        if (calib.intrinsics[0].project(target_from_host.matrix()*model,pixel) &&
            (pixel-target.second.pos).norm()<=threshold) lmdb.addObservation(target.first,target.second);
      }
    }
  }
}
}  // namespace granite

namespace granite {
void KeypointVioEstimator::updateMonoSupport(const OpticalFlowResult::Ptr& frame,
                                            std::map<FrameId, int>& connected) {
  connected.clear(); connected_finite = connected_infinite = 0;
  const TimeCamId current(frame->t_ns, 0);
  for (const auto& observation : frame->observations[0]) {
    if (!lmdb.landmarkExists(observation.first)) continue;
    const auto obs = lmdb.getObservationsOfLandmark(observation.first);
    if (obs.size() < 2 || !obs.count(current)) continue;
    const auto& point = lmdb.getLandmark(observation.first);
    ++connected[point.kf_id.frame_id];
    if (point.id > 1e-6) ++connected_finite;
    else ++connected_infinite;
  }
  mono_support = connected_finite >= 5 ? MonoVioSupport::METRIC :
      (connected_infinite >= 5 ? MonoVioSupport::BEARING_ONLY : MonoVioSupport::INERTIAL_ONLY);
  if (connected_finite+connected_infinite>=20 &&
      (connected_finite+connected_infinite)*10>=frame->observations[0].size()*7)
    mono_last_orientation_supported_t_ns=frame->t_ns;
  if (mono_support==MonoVioSupport::METRIC) mono_last_supported_t_ns=frame->t_ns;
  tracking_state = mono_support == MonoVioSupport::INERTIAL_ONLY
      ? TrackingState::LOST : TrackingState::TRACKING;
}

void KeypointVioEstimator::considerMonoKeyframe(const Eigen::MatrixXd& hessian,
                                              int pose_offset) {
  // Conditional visual information after eliminating landmark parameters.
  // Use rotation only when monocular translation lacks finite-depth support.
  // Exclude IMU/prior information: it must not hide lost visual coverage.
  const int dimension = connected_finite >= 5 ? 6 : 3;
  if (dimension != mono_entropy_dimension) {
    mono_entropy_dimension = dimension; mono_entropy_samples = 0;
  }
  const int offset = pose_offset + (dimension == 3 ? 3 : 0);
  const Eigen::MatrixXd block = hessian.block(offset,offset,dimension,dimension);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(0.5*(block+block.transpose()));
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()) {
    if (frames_after_kf > config.vio_min_frames_after_kf) take_kf = true;
    return;
  }
  mono_negative_entropy = eigen.eigenvalues().array().max(1e-12).log().sum();
  // Compare information ratios in log space (valid also for negative logdet).
  if (mono_entropy_samples && frames_after_kf > config.vio_min_frames_after_kf &&
      (mono_negative_entropy < mono_average_entropy +
           std::log(std::max(1e-12,config.vio_fraction_entropy_take_kf)) ||
       connected_finite + connected_infinite < 20)) take_kf = true;
  ++mono_entropy_samples;
  mono_average_entropy += (mono_negative_entropy-mono_average_entropy)/mono_entropy_samples;
}
}  // namespace granite

namespace granite {
void KeypointVioEstimator::recoverMonoPose(const OpticalFlowResult::Ptr& frame) {
  std::vector<cv::Point3d> world;
  std::vector<cv::Point2d> normalized;
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> pixels;
  const auto predicted=frame_states.at(frame->t_ns).getState();
  const Sophus::SE3d camera_from_world=(predicted.T_w_i*calib.T_i_c[0]).inverse();
  const double image_gap=mono_last_image_t_ns<0 ? 0 :
      std::max(0.0,(frame->t_ns-mono_last_image_t_ns)*1e-9);
  if(!frame->observations[0].empty()) mono_last_image_t_ns=frame->t_ns;
  // Recover rotation from retained infinity directions before triangulation.
  // Otherwise gyro drift during an image outage can become fictitious depth.
  struct Bearing { Eigen::Vector3d world, camera; Eigen::Vector2d pixel; };
  std::vector<Bearing,Eigen::aligned_allocator<Bearing>> bearings;
  size_t bad_bearings=0, finite_parallax=0, finite_matches=0;
  for(const auto& observation:frame->observations[0]) {
    if(!lmdb.landmarkExists(observation.first)) continue;
    const auto& point=lmdb.getLandmark(observation.first);
    if(point.id>1e-6) {++finite_matches; continue;}
    Eigen::Vector4d ray;
    const Eigen::Vector2d pixel=observation.second.translation().cast<double>();
    if(!calib.intrinsics[0].unproject(pixel,ray)) continue;
    const Eigen::Vector3d direction=(getPoseStateWithLin(point.kf_id.frame_id).getPose()*
        calib.T_i_c[0]).so3()*StereographicParam<double>::unproject(point.dir).head<3>();
    bearings.push_back({direction.normalized(),ray.head<3>().normalized(),pixel});
    const Eigen::Vector3d host_ray=StereographicParam<double>::unproject(point.dir).head<3>();
    const Sophus::SE3d host_from_current=
        (getPoseStateWithLin(point.kf_id.frame_id).getPose()*calib.T_i_c[0]).inverse()*
        predicted.T_w_i*calib.T_i_c[0];
    if(image_gap>0.1 && host_from_current.translation().norm()>1e-4) {
      const Eigen::Vector4d candidate=triangulate(host_ray,ray.head<3>().normalized(),host_from_current);
      Eigen::Vector2d current_pixel,host_pixel,expected_host;
      const Eigen::Vector4d in_current=host_from_current.inverse().matrix()*candidate;
      Eigen::Vector4d infinity_host;infinity_host<<host_ray,0;
      if(candidate.allFinite() && candidate[3]>1e-6 && candidate[3]<config.vio_max_inverse_distance &&
          in_current.head<3>().dot(ray.head<3>())>0 &&
          calib.intrinsics[0].project(candidate,host_pixel) &&
          calib.intrinsics[0].project(infinity_host,expected_host) &&
          calib.intrinsics[0].project(in_current,current_pixel) &&
          (host_pixel-expected_host).norm()<=config.vio_obs_huber_thresh &&
          (current_pixel-pixel).norm()<=config.vio_obs_huber_thresh) ++finite_parallax;
    }
    Eigen::Vector4d infinity; infinity<<direction,0;
    Eigen::Vector2d projected;
    if(!calib.intrinsics[0].project(camera_from_world.matrix()*infinity,projected) ||
        (projected-pixel).norm()>config.vio_obs_huber_thresh) ++bad_bearings;
  }
  const double orientation_gap=mono_last_orientation_supported_t_ns<0 ? 0 :
      std::max(0.0,(frame->t_ns-mono_last_orientation_supported_t_ns)*1e-9);
  // Prefer a positive-depth translation explanation even below the normal
  // depth-promotion baseline. A near plane can fit a spurious Wahba rotation.
  if(finite_matches<20 && bearings.size()>=20 && bad_bearings*3>=bearings.size() &&
      orientation_gap>0.1 && image_gap>0.1 && finite_parallax*5<bearings.size()*4) {
    auto fit=[&](const std::vector<size_t>& selected) -> Eigen::Matrix3d {
      Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
      for(size_t i:selected) covariance+=bearings[i].camera*bearings[i].world.transpose();
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,Eigen::ComputeFullU|Eigen::ComputeFullV);
      Eigen::Matrix3d sign=Eigen::Matrix3d::Identity();
      sign(2,2)=(svd.matrixU()*svd.matrixV().transpose()).determinant();
      return svd.matrixU()*sign*svd.matrixV().transpose();
    };
    auto score=[&](const Eigen::Matrix3d& rotation) {
      std::vector<size_t> accepted;
      for(size_t i=0;i<bearings.size();++i) {
        Eigen::Vector4d ray;ray<<rotation*bearings[i].world,0;
        Eigen::Vector2d pixel;
        if(calib.intrinsics[0].project(ray,pixel) &&
            (pixel-bearings[i].pixel).norm()<=config.vio_obs_huber_thresh) accepted.push_back(i);
      }
      return accepted;
    };
    std::vector<size_t> best;
    // Fixed seed keeps reacquisition deterministic without modifying global RNG.
    uint32_t random=0x91e10da5;
    for(size_t trial=0;trial<96;++trial) {
      std::vector<size_t> sample;
      while(sample.size()<3) {
        random=1664525u*random+1013904223u;
        const size_t i=random%bearings.size();
        if(std::find(sample.begin(),sample.end(),i)==sample.end()) sample.push_back(i);
      }
      const auto accepted=score(fit(sample));
      if(accepted.size()>best.size()) best=accepted;
    }
    if(best.size()>=20 && best.size()*5>=bearings.size()*4) {
      const Eigen::Matrix3d rotation=fit(best);
      const auto validated=score(rotation);
      const Sophus::SO3d orientation=Sophus::SO3d(rotation.transpose())*calib.T_i_c[0].so3().inverse();
      if(validated.size()>=20 && validated.size()*5>=bearings.size()*4 &&
          (predicted.T_w_i.so3().inverse()*orientation).log().norm()<=0.005+0.05*orientation_gap) {
        Sophus::SE3d pose=predicted.T_w_i;
        pose.so3()=orientation;
        frame_states.at(frame->t_ns)=PoseVelBiasStateWithLin(frame->t_ns,pose,predicted.vel_w_i,
            predicted.bias_gyro,predicted.bias_accel,false);
        return;
      }
    }
  }
  size_t inconsistent=0;
  for(const auto& observation:frame->observations[0]) {
    if(!lmdb.landmarkExists(observation.first)) continue;
    const auto& point=lmdb.getLandmark(observation.first);
    if(point.id<=1e-6) continue;
    Eigen::Vector4d ray;
    const Eigen::Vector2d pixel=observation.second.translation().cast<double>();
    if(!calib.intrinsics[0].unproject(pixel,ray) || ray.z()<=0.1) continue;
    const Eigen::Vector3d local=StereographicParam<double>::unproject(point.dir).head<3>()/point.id;
    const Eigen::Vector3d position=(getPoseStateWithLin(point.kf_id.frame_id).getPose()*calib.T_i_c[0])*local;
    world.emplace_back(position.x(),position.y(),position.z());
    normalized.emplace_back(ray.x()/ray.z(),ray.y()/ray.z()); pixels.push_back(pixel);
    Eigen::Vector4d homogeneous; homogeneous<<position,1;
    Eigen::Vector2d projected;
    if(!calib.intrinsics[0].project(camera_from_world.matrix()*homogeneous,projected) ||
        (projected-pixel).norm()>config.vio_outlier_threshold) ++inconsistent;
  }
  // Run geometric reacquisition only when ordinary inertial prediction gates
  // would reject a substantial fraction. Normal tracking incurs no RANSAC.
  if(world.size()<20 || inconsistent*3<world.size()) return;
  cv::Mat rvec,tvec,inliers;
  if(!cv::solvePnPRansac(world,normalized,cv::Mat::eye(3,3,CV_64F),cv::Mat(),rvec,tvec,false,
      100,config.vio_outlier_threshold/std::abs(calib.intrinsics[0].getParam()[0]),0.99,inliers,cv::SOLVEPNP_EPNP)) return;
  cv::Mat rotation; cv::Rodrigues(rvec,rotation);
  Eigen::Matrix3d r; Eigen::Vector3d t;
  for(int i=0;i<3;++i) {t[i]=tvec.at<double>(i);for(int j=0;j<3;++j)r(i,j)=rotation.at<double>(i,j);}
  const Sophus::SE3d recovered_camera_from_world(r,t);
  const Sophus::SE3d pose=recovered_camera_from_world.inverse()*calib.T_i_c[0].inverse();
  if(!pose.matrix().allFinite()) return;
  size_t validated=0;
  for(size_t i=0;i<world.size();++i) {
    Eigen::Vector4d point;point<<world[i].x,world[i].y,world[i].z,1;
    Eigen::Vector2d pixel;
    if(calib.intrinsics[0].project(recovered_camera_from_world.matrix()*point,pixel) &&
        (pixel-pixels[i]).norm()<=config.vio_outlier_threshold) ++validated;
  }
  if(validated<20 || validated*10<world.size()*7) return;
  const double gap=mono_last_supported_t_ns<0 ? 0 :
      std::max(0.0,(frame->t_ns-mono_last_supported_t_ns)*1e-9);
  // A coherent instantaneous bad flow field must not masquerade as recovery.
  // Permit growing inertial drift during an outage, preserving metric gauge.
  if((predicted.T_w_i.so3().inverse()*pose.so3()).log().norm()>0.05+0.05*gap ||
      (predicted.T_w_i.translation()-pose.translation()).norm()>0.1+0.5*gap*gap) return;
  frame_states.at(frame->t_ns)=PoseVelBiasStateWithLin(frame->t_ns,pose,predicted.vel_w_i,
      predicted.bias_gyro,predicted.bias_accel,false);
}
}  // namespace granite

namespace granite {
bool KeypointVioEstimator::gateGaugeStep(const AbsOrderMap& aom, const Eigen::MatrixXd& H,
                                         const Eigen::VectorXd& b,
                                         const Eigen::VectorXd& diagonal,
                                         Eigen::VectorXd& inc) {
  // Common translation of the step: the mean translation increment of all
  // poses and states (the increment is applied as -inc).
  Eigen::Vector3d common = Eigen::Vector3d::Zero();
  for (const auto& kv : aom.abs_order_map) common += inc.segment<3>(kv.second.first);
  const double n = static_cast<double>(aom.abs_order_map.size());
  common /= n;
  if (common.norm() <= config.vio_gauge_gate_m) return false;
  // Solve again with information w on the common translation N (orthonormal
  // columns N / sqrt(n)): H + w N N^T / n + diag(diagonal). The other
  // directions get their solution for the restricted common translation, so
  // this is not the same as removing the common part from inc. H itself does
  // not change.
  const double w = config.vio_gauge_gate_weight / n;
  Eigen::MatrixXd damped = H;
  damped.diagonal() += diagonal;
  for (const auto& i : aom.abs_order_map)
    for (const auto& j : aom.abs_order_map)
      damped.block<3, 3>(i.second.first, j.second.first).diagonal().array() += w;
  inc = damped.ldlt().solve(b);
  return true;
}
}  // namespace granite
