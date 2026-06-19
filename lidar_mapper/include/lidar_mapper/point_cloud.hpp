#pragma once
#include <cstdint>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lm {

struct PointXYZT {
  float x, y, z;
  float intensity;
  double t;  // absolute unix seconds
};

struct Scan {
  double stamp = 0;          // header stamp, sweep end
  double t_min = 0, t_max = 0;
  size_t raw = 0;
  size_t dropped_2nd = 0;
  size_t dropped_nonfinite = 0;
  std::vector<PointXYZT> points;
};

// false if x/y/z/timestamp fields are missing.
bool to_scan(const sensor_msgs::msg::PointCloud2 &m, Scan &out);

}  // namespace lm
