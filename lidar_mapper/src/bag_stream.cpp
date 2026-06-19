#include "lidar_mapper/bag_stream.hpp"
#include <algorithm>
#include <climits>
#include <cstdio>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lm {
namespace {
constexpr const char *kLidarTopic = "/iv_points_fusion";
constexpr const char *kGtTopic    = "/fixposition/odometry_enu";
}  // namespace

std::vector<std::filesystem::path> list_mcaps(const std::filesystem::path &dir) {
  std::vector<std::filesystem::path> out;
  for (const auto &e : std::filesystem::directory_iterator(dir)) {
    if (e.is_regular_file() && e.path().extension() == ".mcap") out.push_back(e.path());
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::filesystem::path> resolve_bags(const std::vector<std::string> &args) {
  if (args.size() == 1 && std::filesystem::is_directory(args[0])) return list_mcaps(args[0]);
  std::vector<std::filesystem::path> out;
  for (const auto &a : args) {
    std::filesystem::path p(a);
    if (!std::filesystem::exists(p)) {
      std::fprintf(stderr, "resolve_bags: WARNING missing bag '%s' (skipped)\n", a.c_str());
      continue;
    }
    out.push_back(p);  // explicit order preserved
  }
  return out;
}

std::vector<BagStats> run_bag_stream(const std::vector<std::filesystem::path> &bags,
                                     ScanCb on_scan, PoseCb on_pose) {
  std::vector<BagStats> stats;
  stats.reserve(bags.size());
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry>       od_ser;

  for (int i = 0; i < static_cast<int>(bags.size()); ++i) {
    BagStats s;
    s.name = bags[i].filename().string();
    s.cloud_min = SIZE_MAX;
    double cloud_sum = 0, sweep_sum = 0;
    double sweep_min = 1e9, sweep_max = 0;
    bool   have_prev_pose = false;
    Eigen::Vector3d prev_t = Eigen::Vector3d::Zero();

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions opts;
    opts.uri = bags[i].string();
    opts.storage_id = "mcap";
    reader.open(opts);

    while (reader.has_next()) {
      auto bm = reader.read_next();
      const double tsec = bm->recv_timestamp * 1e-9;
      if (s.n_scans + s.n_poses == 0) s.t_start = tsec;
      s.t_end = tsec;
      rclcpp::SerializedMessage smsg(*bm->serialized_data);

      if (bm->topic_name == kLidarTopic) {
        sensor_msgs::msg::PointCloud2 pc;
        pc_ser.deserialize_message(&smsg, &pc);
        Scan scan;
        if (!to_scan(pc, scan)) continue;
        s.n_scans++;
        const size_t n = scan.points.size();
        if (n < s.cloud_min) s.cloud_min = n;
        if (n > s.cloud_max) s.cloud_max = n;
        cloud_sum += static_cast<double>(n);
        const double span = scan.t_max - scan.t_min;
        if (span < sweep_min) sweep_min = span;
        if (span > sweep_max) sweep_max = span;
        sweep_sum += span;
        s.raw_sum               += static_cast<double>(scan.raw);
        s.dropped_2nd_sum       += static_cast<double>(scan.dropped_2nd);
        s.dropped_nonfinite_sum += static_cast<double>(scan.dropped_nonfinite);
        if (on_scan) on_scan(scan, i);
      } else if (bm->topic_name == kGtTopic) {
        nav_msgs::msg::Odometry od;
        od_ser.deserialize_message(&smsg, &od);
        GTPose p;
        p.stamp = od.header.stamp.sec + od.header.stamp.nanosec * 1e-9;
        p.t << od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z;
        p.q = Eigen::Quaterniond(od.pose.pose.orientation.w,
                                 od.pose.pose.orientation.x,
                                 od.pose.pose.orientation.y,
                                 od.pose.pose.orientation.z);
        s.n_poses++;
        if (s.n_poses == 1) s.first_pose = p;
        s.last_pose = p;
        if (have_prev_pose) s.gt_path += (p.t - prev_t).norm();
        prev_t = p.t;
        have_prev_pose = true;
        if (on_pose) on_pose(p, i);
      }
    }

    if (s.n_scans > 0) {
      s.cloud_mean      = cloud_sum / s.n_scans;
      s.sweep_span_min  = sweep_min;
      s.sweep_span_max  = sweep_max;
      s.sweep_span_mean = sweep_sum / s.n_scans;
    } else {
      s.cloud_min = 0;
    }
    stats.push_back(std::move(s));
  }
  return stats;
}

bool first_gt_pose(const std::filesystem::path &bag, GTPose &out) {
  rclcpp::Serialization<nav_msgs::msg::Odometry> od_ser;
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions opts;
  opts.uri = bag.string();
  opts.storage_id = "mcap";
  reader.open(opts);
  while (reader.has_next()) {
    auto bm = reader.read_next();
    if (bm->topic_name != kGtTopic) continue;
    rclcpp::SerializedMessage smsg(*bm->serialized_data);
    nav_msgs::msg::Odometry od;
    od_ser.deserialize_message(&smsg, &od);
    out.stamp = od.header.stamp.sec + od.header.stamp.nanosec * 1e-9;
    out.t << od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z;
    out.q = Eigen::Quaterniond(od.pose.pose.orientation.w, od.pose.pose.orientation.x,
                               od.pose.pose.orientation.y, od.pose.pose.orientation.z);
    return true;
  }
  return false;
}

std::vector<BagGap> compute_gaps(const std::vector<BagStats> &stats) {
  std::vector<BagGap> g;
  for (size_t i = 1; i < stats.size(); ++i) {
    BagGap b;
    b.dt_seconds    = stats[i].first_pose.stamp - stats[i - 1].last_pose.stamp;
    b.gt_distance_m = (stats[i].first_pose.t - stats[i - 1].last_pose.t).norm();
    g.push_back(b);
  }
  return g;
}

}  // namespace lm
