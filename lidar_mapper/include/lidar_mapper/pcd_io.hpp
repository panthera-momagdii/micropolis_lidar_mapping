#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include <string>

namespace lm {

// binary pcd: fields x y z intensity, all f32.
bool write_pcd(const std::string &path, const std::vector<PointXYZT> &pts);

}  // namespace lm
