#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include <Eigen/Core>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace lm {

struct Voxel {
  int n = 0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d M2   = Eigen::Matrix3d::Zero();  // welford cov accumulator
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double planarity = 0;
  bool has_plane = false;
};

struct VoxelMap {
  double v_size = 0.5;
  int    max_n  = 50;
  int    min_plane_n   = 6;     // min points before plane is usable
  double min_planarity = 0.1;   // below: point-to-mean fallback
  std::unordered_map<int64_t, Voxel> cells;

  // 21 bits per axis, biased; bag-spanning safe at v ≤ 1 m.
  static int64_t key(double x, double y, double z, double v);
  void   insert(const Eigen::Vector3d &p);
  void   neighbors(const Eigen::Vector3d &p, std::vector<const Voxel *> &out) const;
  void   crop(const Eigen::Vector3d &center, double radius);
};

// keep first point per voxel of size v.
void voxel_subsample(const std::vector<PointXYZT> &in, double v, std::vector<PointXYZT> &out);

}  // namespace lm
