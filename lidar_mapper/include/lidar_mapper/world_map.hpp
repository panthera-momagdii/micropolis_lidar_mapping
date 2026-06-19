#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include "lidar_mapper/voxel_map.hpp"
#include <Eigen/Geometry>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace lm {

// Accumulates the deduplicated world-frame OUTPUT map: transform a sensor-frame cloud by the
// scan pose, voxel-key at v_out, keep the first point seen per voxel. This is the "transform
// -> voxel-key -> dedup" logic that was duplicated inline in run_odometry, regen_map, and
// Odometry::process.
//
// key_from_double selects which coordinate the voxel key is computed from — this captures a
// pre-existing difference between the two callers, kept so each tool's .pcd stays byte-identical:
//   false (run_odometry / Variant A): key from the stored float-truncated coord (Variant A's
//          odometry always rounded to float before keying).
//   true  (regen_map / Variant B): key from the full-double world coord before truncation.
// The point WRITTEN is the float coord in both cases; only the dedup voxel id differs at the
// rare sub-mm boundary where rounding crosses a v_out grid line.
struct WorldMap {
  double v_out = 0.2;
  bool key_from_double = false;
  std::unordered_set<int64_t> keys;
  std::vector<PointXYZT> pts;

  void reserve(size_t n) { keys.reserve(n); pts.reserve(n); }

  void add(const std::vector<PointXYZT> &cloud_sensor, const Eigen::Isometry3d &T) {
    for (const auto &p : cloud_sensor) {
      const Eigen::Vector3d w = T * Eigen::Vector3d(p.x, p.y, p.z);
      PointXYZT q;
      q.x = static_cast<float>(w.x());
      q.y = static_cast<float>(w.y());
      q.z = static_cast<float>(w.z());
      q.intensity = p.intensity;
      q.t = 0;
      const int64_t k = key_from_double ? VoxelMap::key(w.x(), w.y(), w.z(), v_out)
                                        : VoxelMap::key(q.x, q.y, q.z, v_out);
      if (keys.insert(k).second) pts.push_back(q);
    }
  }
};

}  // namespace lm
