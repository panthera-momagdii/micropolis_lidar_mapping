#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include <Eigen/Core>

namespace lm {

// in-place; reference = sweep end. For each point: p' = Exp((t_i - t_end) * xi_vel) * p.
void deskew(Scan &scan, const Eigen::Matrix<double, 6, 1> &xi_vel);

}  // namespace lm
