#pragma once
#include "lidar_mapper/point_cloud.hpp"
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace lm {

struct GTPose {
  double stamp = 0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

struct BagStats {
  std::string name;
  double t_start = 0, t_end = 0;
  int n_scans = 0;
  int n_poses = 0;
  size_t cloud_min = 0, cloud_max = 0;
  double cloud_mean = 0;
  double sweep_span_min = 0, sweep_span_max = 0, sweep_span_mean = 0;
  double raw_sum = 0, dropped_2nd_sum = 0, dropped_nonfinite_sum = 0;
  double gt_path = 0;
  GTPose first_pose, last_pose;  // valid if n_poses > 0
};

struct BagGap {
  double dt_seconds = 0;
  double gt_distance_m = 0;
};

// Scan is passed by mutable reference so consumers can deskew in place (it is owned by the bag
// stream and discarded right after the callback returns — no copy needed).
using ScanCb = std::function<void(Scan &, int bag_idx)>;
using PoseCb = std::function<void(const GTPose &, int bag_idx)>;

std::vector<std::filesystem::path> list_mcaps(const std::filesystem::path &dir);

// Resolve bag inputs from CLI args. A single directory arg is globbed+sorted
// (back-compat); otherwise the given paths are returned in the EXACT order
// provided (manifest-driven session ordering). Missing files are dropped with
// a warning to stderr.
std::vector<std::filesystem::path> resolve_bags(const std::vector<std::string> &args);

std::vector<BagStats> run_bag_stream(const std::vector<std::filesystem::path> &bags,
                                     ScanCb on_scan, PoseCb on_pose);

std::vector<BagGap> compute_gaps(const std::vector<BagStats> &stats);

// reads exactly one GT pose (first /fixposition/odometry_enu) from a single bag.
bool first_gt_pose(const std::filesystem::path &bag, GTPose &out);

}  // namespace lm
