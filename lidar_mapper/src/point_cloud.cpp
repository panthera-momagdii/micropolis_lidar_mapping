#include "lidar_mapper/point_cloud.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace lm {
namespace {

int offset_of(const sensor_msgs::msg::PointCloud2 &m, const std::string &name) {
  for (const auto &f : m.fields) if (f.name == name) return static_cast<int>(f.offset);
  return -1;
}

int datatype_of(const sensor_msgs::msg::PointCloud2 &m, const std::string &name) {
  for (const auto &f : m.fields) if (f.name == name) return static_cast<int>(f.datatype);
  return -1;
}

}  // namespace

bool to_scan(const sensor_msgs::msg::PointCloud2 &m, Scan &out) {
  const int ox = offset_of(m, "x");
  const int oy = offset_of(m, "y");
  const int oz = offset_of(m, "z");
  const int ot = offset_of(m, "timestamp");
  const int oi = offset_of(m, "intensity");
  const int o2 = offset_of(m, "is_2nd_return");
  if (ox < 0 || oy < 0 || oz < 0 || ot < 0) return false;

  // we memcpy x/y/z + intensity as float32 and timestamp as float64 below; fail loud if a
  // memcpy'd field's layout differs. (intensity is optional: only checked when present.)
  using PF = sensor_msgs::msg::PointField;
  if (datatype_of(m, "x") != PF::FLOAT32 || datatype_of(m, "y") != PF::FLOAT32 ||
      datatype_of(m, "z") != PF::FLOAT32 || datatype_of(m, "timestamp") != PF::FLOAT64 ||
      (oi >= 0 && datatype_of(m, "intensity") != PF::FLOAT32)) {
    std::fprintf(stderr, "to_scan: unexpected field datatype (need x/y/z/intensity float32, "
                         "timestamp float64)\n");
    return false;
  }

  out.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
  out.points.clear();
  const size_t n = static_cast<size_t>(m.width) * m.height;
  out.raw = n;
  out.dropped_2nd = 0;
  out.dropped_nonfinite = 0;
  out.points.reserve(n);
  double tmin = 1e300, tmax = -1e300;
  const size_t step = m.point_step;
  const uint8_t *d = m.data.data();

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *p = d + i * step;
    if (o2 >= 0 && p[o2] != 0) { ++out.dropped_2nd; continue; }
    PointXYZT q;
    std::memcpy(&q.x, p + ox, 4);
    std::memcpy(&q.y, p + oy, 4);
    std::memcpy(&q.z, p + oz, 4);
    std::memcpy(&q.t, p + ot, 8);
    if (oi >= 0) std::memcpy(&q.intensity, p + oi, 4);
    else q.intensity = 0.0f;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
      ++out.dropped_nonfinite;
      continue;
    }
    if (q.t < tmin) tmin = q.t;
    if (q.t > tmax) tmax = q.t;
    out.points.push_back(q);
  }
  if (out.points.empty()) { tmin = out.stamp; tmax = out.stamp; }
  out.t_min = tmin;
  out.t_max = tmax;
  return true;
}

}  // namespace lm
