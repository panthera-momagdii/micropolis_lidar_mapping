#include "lidar_mapper/bag_stream.hpp"
#include "lidar_mapper/odometry.hpp"
#include "lidar_mapper/pcd_io.hpp"
#include "lidar_mapper/tum_io.hpp"
#include "lidar_mapper/world_map.hpp"
#include <chrono>
#include <iostream>
#include <yaml-cpp/yaml.h>

using namespace lm;

int main(int argc, char **argv) {
  // Manifest-driven orchestration: explicit ordered bag list + output dir.
  //   run_odometry <params.yaml> <out_dir> [--no-map] <bag1.mcap> [bag2.mcap ...]
  // A single directory in place of the bag list is globbed+sorted (back-compat).
  if (argc < 4) {
    std::cerr << "usage: run_odometry <params.yaml> <out_dir> [--no-map] "
                 "<bag1.mcap> [bag2.mcap ...]\n";
    return 1;
  }
  const std::string params_path = argv[1];
  const std::filesystem::path out_dir = argv[2];
  bool write_map = true;
  std::vector<std::string> bag_args;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--no-map") { write_map = false; continue; }
    bag_args.push_back(a);
  }
  std::filesystem::create_directories(out_dir);
  const std::string out_tum = (out_dir / "trajectory.tum").string();
  const std::string out_pcd = (out_dir / "map.pcd").string();

  YAML::Node cfg = YAML::LoadFile(params_path);
  OdometryParams p;
  p.v_map                = cfg["v_map"].as<double>();
  p.v_out                = cfg["v_out"].as<double>();
  p.max_corr_dist        = cfg["max_corr_dist"].as<double>();
  p.huber_delta          = cfg["huber_delta"].as<double>();
  p.max_points_per_voxel = cfg["max_points_per_voxel"].as<int>();
  p.min_voxel_points     = cfg["min_voxel_points"].as<int>();
  p.min_planarity        = cfg["min_planarity"].as<double>();
  p.crop_radius          = cfg["crop_radius"].as<double>();
  p.gap_reset_sec        = cfg["gap_reset_sec"].as<double>();
  const std::string t0_mode = cfg["t0_mode"].as<std::string>();

  const auto bags = resolve_bags(bag_args);
  if (bags.empty()) { std::cerr << "no input bags resolved\n"; return 1; }
  std::cout << "session: " << bags.size() << " bag(s), ordered:\n";
  for (const auto &b : bags) std::cout << "  " << b.filename().string() << "\n";

  Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();
  if (t0_mode == "gt_anchor") {
    GTPose g;
    if (!first_gt_pose(bags.front(), g)) {
      std::cerr << "gt_anchor: no GT pose in first bag\n";
      return 1;
    }
    T0.linear() = g.q.toRotationMatrix();
    T0.translation() = g.t;
    std::cout << "T0 anchored from GT @ " << g.stamp << "\n";
  }

  Odometry odo(p, T0);
  TumWriter tum;
  if (!tum.open(out_tum)) { std::cerr << "cannot open " << out_tum << "\n"; return 1; }

  WorldMap world;
  world.v_out = p.v_out;
  world.reserve(2'000'000);

  GTPose first_gt{}, last_gt{};
  bool have_first_gt = false;

  int n_scans = 0;
  double cum_dist = 0;
  Eigen::Vector3d last_pos = T0.translation();
  auto t_wall0 = std::chrono::steady_clock::now();

  std::vector<PointXYZT> map_cloud;  // sensor-frame v_map cloud from each scan
  auto on_scan = [&](Scan &sc, int) {
    const auto step = odo.process(sc, map_cloud);
    const Eigen::Quaterniond q(step.T.linear());
    tum.write(step.stamp, step.T.translation(), q);
    cum_dist += (step.T.translation() - last_pos).norm();
    last_pos = step.T.translation();
    if (write_map) world.add(map_cloud, step.T);
    if (++n_scans % 20 == 0) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_wall0).count();
      std::cout << "[" << n_scans << "] iters=" << step.iters
                << " matched=" << step.matched
                << " dist=" << cum_dist << " m"
                << " fps=" << (n_scans / elapsed)
                << (step.gap_reset ? " GAP_RESET" : "") << "\n";
    }
  };
  auto on_pose = [&](const GTPose &gp, int) {
    if (!have_first_gt) { first_gt = gp; have_first_gt = true; }
    last_gt = gp;
  };

  run_bag_stream(bags, on_scan, on_pose);

  const double wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_wall0).count();
  const auto &traj = odo.trajectory();
  std::cout << "\n--- done ---\n"
            << "scans              : " << traj.size() << "\n"
            << "cumulative distance: " << cum_dist << " m\n"
            << "wall time          : " << wall << " s   (fps "
            << (traj.size() / wall) << ")\n"
            << "local map voxels   : " << odo.local_map().cells.size() << "\n"
            << "output map points  : " << world.pts.size() << "\n";

  if (have_first_gt && !traj.empty()) {
    // crude endpoint displacement check: align starts then compare end positions.
    const Eigen::Vector3d gt_disp = last_gt.t - first_gt.t;
    const Eigen::Vector3d my_disp = traj.back().T.translation() - traj.front().T.translation();
    std::cout << "endpoint disp err  : " << (gt_disp - my_disp).norm() << " m"
              << "   (gt " << gt_disp.norm() << " m, mine " << my_disp.norm() << " m)\n";
  }

  if (write_map) {
    if (!write_pcd(out_pcd, world.pts)) { std::cerr << "cannot write " << out_pcd << "\n"; return 1; }
    std::cout << "wrote " << out_tum << " and " << out_pcd << "\n";
  } else {
    std::cout << "wrote " << out_tum << " (map skipped: --no-map)\n";
  }
  return 0;
}
