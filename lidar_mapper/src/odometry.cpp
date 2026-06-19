#include "lidar_mapper/odometry.hpp"
#include "lidar_mapper/deskew.hpp"
#include "lidar_mapper/registration.hpp"
#include "lidar_mapper/se3.hpp"

namespace lm {

Odometry::Odometry(const OdometryParams &p, const Eigen::Isometry3d &T0)
  : p_(p), T0_(T0) {
  map_.v_size       = p.v_map;
  map_.max_n        = p.max_points_per_voxel;
  map_.min_plane_n  = p.min_voxel_points;
  map_.min_planarity = p.min_planarity;
}

OdometryStep Odometry::process(Scan &scan, std::vector<PointXYZT> &map_cloud_sensor) {
  // 1. predict pose + velocity twist
  Eigen::Matrix<double, 6, 1> xi_vel = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Isometry3d T_pred;
  bool gap_reset = false;
  if (traj_.empty()) {
    T_pred = T0_;
  } else {
    const auto &last = traj_.back();
    const double gap_cur = scan.stamp - last.stamp;
    if (traj_.size() < 2 || gap_cur > p_.gap_reset_sec || gap_cur <= 0) {
      T_pred = last.T;
      gap_reset = (gap_cur > p_.gap_reset_sec);
    } else {
      const auto &prev = traj_[traj_.size() - 2];
      const double dt_prev = last.stamp - prev.stamp;
      if (dt_prev > 0 && dt_prev <= p_.gap_reset_sec) {
        xi_vel = Log(prev.T.inverse() * last.T) / dt_prev;
      }
      T_pred = last.T * Exp(xi_vel * gap_cur);
    }
  }

  // 2. deskew full-res in place (the caller's scan is mutated; it is discarded after this call)
  deskew(scan, xi_vel);

  // 3. downsample → map cloud (v_map) → registration cloud (1.5 * v_map)
  // map_cloud is written straight into the caller's output buffer (sensor frame).
  std::vector<PointXYZT> &map_cloud = map_cloud_sensor;
  std::vector<PointXYZT> reg_cloud;
  voxel_subsample(scan.points, p_.v_map, map_cloud);
  voxel_subsample(map_cloud, 1.5 * p_.v_map, reg_cloud);

  // 4. register
  Eigen::Isometry3d T = T_pred;
  IcpResult icp = register_p2pl(reg_cloud, map_, T, p_.max_corr_dist, p_.huber_delta, 20);

  // 5. insert map cloud into local map at T; crop every 50 scans
  for (const auto &pt : map_cloud) {
    map_.insert(T * Eigen::Vector3d(pt.x, pt.y, pt.z));
  }
  if (++scans_since_crop_ >= 50) {
    map_.crop(T.translation(), p_.crop_radius);
    scans_since_crop_ = 0;
  }

  // 6. map_cloud (sensor frame) is already in the caller's buffer; the caller transforms it by
  //    step.T and dedupes into the world output map (lm::WorldMap).

  OdometryStep step;
  step.stamp = scan.stamp;
  step.T = T;
  step.iters = icp.iters;
  step.matched = icp.matched;
  step.gap_reset = gap_reset;
  traj_.push_back(step);
  return step;
}

}  // namespace lm
