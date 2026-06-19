#include "gnss_fusion/pose_graph.hpp"
#include "lidar_mapper/tum_io.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <yaml-cpp/yaml.h>

using Iso = Eigen::Isometry3d;

// rigid position-only fit (Kabsch / Umeyama without scale): R,t s.t. enu ~ R*odom + t.
// uses GNSS positions only (no orientation). yields enu_from_odom to seed a near-converged chain.
static Iso kabsch(const std::vector<Eigen::Vector3d> &odom, const std::vector<Eigen::Vector3d> &enu) {
  Iso T = Iso::Identity();
  Eigen::Vector3d ca = Eigen::Vector3d::Zero(), cb = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < odom.size(); ++i) { ca += odom[i]; cb += enu[i]; }
  ca /= odom.size(); cb /= enu.size();
  if (odom.size() < 3) { T.translation() = cb - ca; return T; }  // too few points: translate only
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < odom.size(); ++i) H += (odom[i] - ca) * (enu[i] - cb).transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1 : 1;  // proper rotation
  const Eigen::Matrix3d R = svd.matrixV() * D * svd.matrixU().transpose();
  T.linear() = R;
  T.translation() = cb - R * ca;
  return T;
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cerr << "usage: run_fusion <bag_dir> <variantA_trajectory.tum> <params_fusion.yaml> <out_dir>\n";
    return 1;
  }
  const std::string bag_dir = argv[1], traj_path = argv[2], cfg_path = argv[3], out_dir = argv[4];

  YAML::Node cfg = YAML::LoadFile(cfg_path);
  const std::string gnss_topic = cfg["gnss_topic"].as<std::string>();
  const double w_trans = cfg["w_trans"].as<double>();
  const double w_rot = cfg["w_rot"].as<double>();
  const double gnss_sigma = cfg["gnss_sigma"].as<double>();
  const int gnss_every_n = cfg["gnss_every_n"].as<int>();
  const double gap_reset_sec = cfg["gap_reset_sec"].as<double>();
  const int max_iters = cfg["max_iters"].as<int>();

  // --- read Variant A poses (initial X, odometry frame) + LiDAR header stamps ---
  std::vector<double> stamp;
  std::vector<Iso> TA;
  if (!lm::TumReader::read(traj_path, stamp, TA)) {
    std::cerr << "cannot open " << traj_path << "\n"; return 1;
  }
  const int N = static_cast<int>(TA.size());
  if (N == 0) { std::cerr << "no poses in " << traj_path << "\n"; return 1; }

  // --- stream GNSS (configured topic only) from every bag; skip LiDAR clouds entirely ---
  std::vector<double> gt;
  std::vector<Eigen::Vector3d> gp;
  {
    std::vector<std::string> bags;
    for (const auto &e : std::filesystem::directory_iterator(bag_dir))
      if (e.path().extension() == ".mcap") bags.push_back(e.path().string());
    std::sort(bags.begin(), bags.end());
    if (bags.empty()) { std::cerr << "no .mcap in " << bag_dir << "\n"; return 1; }
    rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
    for (const auto &bag : bags) {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions opts;
      opts.uri = bag;
      opts.storage_id = "mcap";
      reader.open(opts);
      rosbag2_storage::StorageFilter filter;
      filter.topics = {gnss_topic};  // read only GNSS bytes off disk; never touch the cloud messages
      reader.set_filter(filter);
      while (reader.has_next()) {
        auto bm = reader.read_next();
        if (bm->topic_name != gnss_topic) continue;
        rclcpp::SerializedMessage smsg(*bm->serialized_data);
        nav_msgs::msg::Odometry od;
        ser.deserialize_message(&smsg, &od);
        gt.push_back(od.header.stamp.sec + od.header.stamp.nanosec * 1e-9);
        gp.emplace_back(od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z);
      }
    }
  }
  if (gt.empty()) { std::cerr << "no GNSS on " << gnss_topic << "\n"; return 1; }

  // --- associate each node to nearest-in-time GNSS (|dt| <= 0.05 s) ---
  std::vector<char> matched(N, 0);
  std::vector<Eigen::Vector3d> mp(N);
  for (int k = 0; k < N; ++k) {
    auto it = std::lower_bound(gt.begin(), gt.end(), stamp[k]);
    int idx = static_cast<int>(it - gt.begin());
    double best = 1e18;
    int bi = -1;
    for (int c : {idx - 1, idx})
      if (c >= 0 && c < static_cast<int>(gt.size())) {
        const double d = std::fabs(gt[c] - stamp[k]);
        if (d < best) { best = d; bi = c; }
      }
    if (bi >= 0 && best <= 0.05) { matched[k] = 1; mp[k] = gp[bi]; }
  }

  // --- sub-chains: a node-time gap > gap_reset_sec breaks the sequential odometry edges ---
  std::vector<std::pair<int, int>> chains;  // [start, end)
  int start = 0;
  for (int k = 1; k <= N; ++k)
    if (k == N || stamp[k] - stamp[k - 1] > gap_reset_sec) { chains.emplace_back(start, k); start = k; }

  // --- per-sub-chain Kabsch init: seed each chain into ENU from its own GNSS (no cross-link) ---
  // nodes are added in index order (chains partition [0,N) contiguously), so node index == k.
  gf::PoseGraph g;
  g.set_weights(w_trans, w_rot, 1.0 / (gnss_sigma * gnss_sigma));
  for (const auto &c : chains) {
    std::vector<Eigen::Vector3d> a, b;
    for (int k = c.first; k < c.second; ++k)
      if (matched[k]) { a.push_back(TA[k].translation()); b.push_back(mp[k]); }
    const Iso T_align = kabsch(a, b);  // enu_from_odom for this chain
    for (int k = c.first; k < c.second; ++k) g.add_node(T_align * TA[k]);
  }

  // --- factors: odometry between-edges within a chain + subsampled GNSS priors ---
  int odom_edges = 0, gnss_priors = 0;
  for (const auto &c : chains)
    for (int k = c.first + 1; k < c.second; ++k) {
      g.add_odom_edge(k - 1, k, TA[k - 1].inverse() * TA[k]);  // Z is frame-invariant (relative)
      ++odom_edges;
    }
  for (int k = 0; k < N; ++k)
    if (matched[k] && k % gnss_every_n == 0) { g.add_gnss_prior(k, mp[k]); ++gnss_priors; }

  std::cout << "nodes=" << N << "  sub-chains=" << chains.size()
            << "  odom_edges=" << odom_edges << "  gnss_priors=" << gnss_priors << "\n";

  // --- optimize ---
  const auto hist = g.optimize(max_iters);
  std::cout << std::fixed << std::setprecision(4);
  for (size_t it = 0; it < hist.size(); ++it) {
    const auto &s = hist[it];
    std::cout << "iter " << std::setw(2) << it
              << "  cost=" << std::setw(14) << s.cost
              << "  odom[m] mean=" << s.odom_mean << " max=" << s.odom_max
              << "  gnss[m] mean=" << s.gnss_mean << " max=" << s.gnss_max << "\n";
  }

  // --- write fused.tum (TUM: timestamp tx ty tz qx qy qz qw, LiDAR header stamps) ---
  std::filesystem::create_directories(out_dir);
  const std::string out = out_dir + "/fused.tum";
  std::ofstream of(out);
  if (!of) { std::cerr << "cannot open " << out << "\n"; return 1; }
  of << std::fixed << std::setprecision(9);
  const auto &Xout = g.poses();
  for (int k = 0; k < N; ++k) {
    const Eigen::Quaterniond q(Xout[k].linear());
    const Eigen::Vector3d t = Xout[k].translation();
    of << stamp[k] << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
       << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "wrote " << out << "\n";
  return 0;
}
