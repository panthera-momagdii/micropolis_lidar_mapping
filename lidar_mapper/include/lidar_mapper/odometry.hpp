#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include "lidar_mapper/voxel_map.hpp"
#include <Eigen/Geometry>
#include <vector>

namespace lm {

// Defaults mirror params.yaml (the single runtime source of truth; run_odometry overrides every
// field from it). Kept in sync so there is one set of numbers.
struct OdometryParams {
  double v_map = 0.5;
  double v_out = 0.2;
  double max_corr_dist = 2.0;
  double huber_delta = 0.5;
  int    max_points_per_voxel = 50;
  int    min_voxel_points = 6;
  double min_planarity = 0.1;
  double crop_radius = 100.0;
  double gap_reset_sec = 5.0;
};

struct OdometryStep {
  double stamp = 0;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  int  iters = 0;
  int  matched = 0;
  bool gap_reset = false;
};

class Odometry {
 public:
  Odometry(const OdometryParams &p, const Eigen::Isometry3d &T0);

  // returns step. map_cloud_sensor is filled with the deskewed, v_map-subsampled cloud in the
  // SENSOR frame (caller transforms by step.T and dedupes into the output map at v_out, e.g. via
  // lm::WorldMap).
  OdometryStep process(Scan &scan, std::vector<PointXYZT> &map_cloud_sensor);

  const std::vector<OdometryStep> &trajectory() const { return traj_; }
  const VoxelMap &local_map() const { return map_; }

 private:
  OdometryParams p_;
  VoxelMap       map_;
  Eigen::Isometry3d T0_;
  std::vector<OdometryStep> traj_;
  int scans_since_crop_ = 0;
};

}  // namespace lm
