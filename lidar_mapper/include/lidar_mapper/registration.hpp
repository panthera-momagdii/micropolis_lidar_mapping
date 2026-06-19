#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include "lidar_mapper/voxel_map.hpp"
#include <Eigen/Geometry>

namespace lm {

struct IcpResult {
  int  iters    = 0;
  int  matched  = 0;
  bool converged = false;
};

// in/out T. Source = sensor-frame points (already deskewed and downsampled).
IcpResult register_p2pl(const std::vector<PointXYZT> &src, const VoxelMap &map,
                        Eigen::Isometry3d &T,
                        double max_corr_dist, double huber_delta, int max_iters);

}  // namespace lm
