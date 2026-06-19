#include "lidar_mapper/bag_stream.hpp"
#include "lidar_mapper/deskew.hpp"
#include "lidar_mapper/pcd_io.hpp"
#include "lidar_mapper/se3.hpp"
#include "lidar_mapper/tum_io.hpp"
#include "lidar_mapper/voxel_map.hpp"
#include "lidar_mapper/world_map.hpp"
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace lm;
using Iso = Eigen::Isometry3d;

// Rebuild the ENU map from the fused poses. Identical to Variant A's output path
// (deskew -> v_map subsample -> transform -> dedup at v_out); ONLY the pose source changes.
int main(int argc, char **argv) {
  if (argc < 5) {
    std::cerr << "usage: regen_map <bag_dir> <fused.tum> <variantA_params.yaml> <out_dir>\n";
    return 1;
  }
  const std::string bag_dir = argv[1], fused_path = argv[2], cfg_path = argv[3], out_dir = argv[4];

  YAML::Node cfg = YAML::LoadFile(cfg_path);
  const double v_map = cfg["v_map"].as<double>();
  const double v_out = cfg["v_out"].as<double>();
  const double gap_reset_sec = cfg["gap_reset_sec"].as<double>();

  // --- read fused poses (ENU) + LiDAR header stamps ---
  std::vector<double> stamp;
  std::vector<Iso> X;
  if (!lm::TumReader::read(fused_path, stamp, X)) {
    std::cerr << "cannot open " << fused_path << "\n"; return 1;
  }
  if (X.empty()) { std::cerr << "no poses in " << fused_path << "\n"; return 1; }

  // confirm which poses drive the map and that they are ENU (not the odometry frame)
  {
    Eigen::Vector3d lo = X[0].translation(), hi = X[0].translation();
    for (const auto &T : X) { lo = lo.cwiseMin(T.translation()); hi = hi.cwiseMax(T.translation()); }
    std::cout << std::fixed << std::setprecision(2)
              << "fused poses read from " << fused_path << ": " << X.size()
              << " poses, footprint x[" << lo.x() << "," << hi.x() << "] y[" << lo.y() << ","
              << hi.y() << "] z[" << lo.z() << "," << hi.z() << "]\n";
  }

  const auto bags = list_mcaps(bag_dir);
  if (bags.empty()) { std::cerr << "no .mcap in " << bag_dir << "\n"; return 1; }

  WorldMap world;
  world.v_out = v_out;
  world.key_from_double = true;  // regen_map keys output voxels from the full-double world coord
  world.reserve(4'000'000);

  int prev = -1, placed = 0, missed = 0;
  auto on_scan = [&](Scan &sc, int) {
    // match scan to its fused node by nearest stamp (fused.tum stamps ARE the scan stamps)
    auto it = std::lower_bound(stamp.begin(), stamp.end(), sc.stamp);
    const int idx = static_cast<int>(it - stamp.begin());
    double best = 1e18;
    int bi = -1;
    for (int c : {idx - 1, idx})
      if (c >= 0 && c < static_cast<int>(stamp.size())) {
        const double d = std::fabs(stamp[c] - sc.stamp);
        if (d < best) { best = d; bi = c; }
      }
    if (bi < 0 || best > 1e-3) { ++missed; return; }

    // deskew velocity from consecutive fused poses; zero at a sub-chain start/gap (as Variant A did)
    Eigen::Matrix<double, 6, 1> xi_vel = Eigen::Matrix<double, 6, 1>::Zero();
    if (prev >= 0) {
      const double dt = stamp[bi] - stamp[prev];
      if (dt > 0 && dt <= gap_reset_sec) xi_vel = Log(X[prev].inverse() * X[bi]) / dt;
    }
    prev = bi;

    deskew(sc, xi_vel);  // in place; sc is owned by the bag stream and discarded after this call
    std::vector<PointXYZT> map_cloud;
    voxel_subsample(sc.points, v_map, map_cloud);
    world.add(map_cloud, X[bi]);
    ++placed;
  };

  run_bag_stream(bags, on_scan, nullptr);

  std::filesystem::create_directories(out_dir);
  const std::string out = out_dir + "/fused_map.pcd";
  if (!write_pcd(out, world.pts)) { std::cerr << "cannot write " << out << "\n"; return 1; }
  std::cout << "placed " << placed << " scans (missed " << missed << "), "
            << world.pts.size() << " points -> " << out << "\n";

  // --- sanity: prove the map is in ENU, not the odometry frame ---
  // footprint must match the fused poses above; ground-Z (5th pctile) must be flat along x
  // (a monotonic slope of several metres would mean points were accumulated at drifted poses).
  std::cout << std::fixed << std::setprecision(2);
  Eigen::Vector3d lo(world.pts[0].x, world.pts[0].y, world.pts[0].z), hi = lo;
  for (const auto &p : world.pts) {
    const Eigen::Vector3d v(p.x, p.y, p.z);
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  std::cout << "map footprint: x[" << lo.x() << "," << hi.x() << "] y[" << lo.y() << "," << hi.y()
            << "] z[" << lo.z() << "," << hi.z() << "]\n";
  const int NB = 8;
  double gmin = 1e18, gmax = -1e18;
  std::cout << "ground-Z by x-slice (5th pctile):\n";
  for (int b = 0; b < NB; ++b) {
    const double a = lo.x() + (hi.x() - lo.x()) * b / NB;
    const double c = lo.x() + (hi.x() - lo.x()) * (b + 1) / NB;
    std::vector<float> zs;
    for (const auto &p : world.pts)
      if (p.x >= a && p.x < c) zs.push_back(p.z);
    if (zs.size() < 50) continue;
    const size_t k = zs.size() / 20;  // 5th percentile = ground
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    gmin = std::min(gmin, static_cast<double>(zs[k]));
    gmax = std::max(gmax, static_cast<double>(zs[k]));
    std::cout << "  x in [" << static_cast<int>(a) << "," << static_cast<int>(c)
              << "] : ground_z=" << zs[k] << "  (n=" << zs.size() << ")\n";
  }
  std::cout << "ground-Z spread across slices = " << (gmax - gmin) << " m  "
            << ((gmax - gmin) < 3.0 ? "[FLAT - ENU OK]" : "[SLOPED - WRONG FRAME]") << "\n";
  return 0;
}
