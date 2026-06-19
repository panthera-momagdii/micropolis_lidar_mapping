#include "lidar_mapper/bag_stream.hpp"
#include "lidar_mapper/pcd_io.hpp"
#include <iomanip>
#include <iostream>

using namespace lm;

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: inspect_bags <bag_dir>\n";
    return 1;
  }
  const auto bags = list_mcaps(argv[1]);
  if (bags.empty()) {
    std::cerr << "no .mcap files found in " << argv[1] << "\n";
    return 1;
  }
  std::cout << "found " << bags.size() << " bag(s):\n";
  for (const auto &b : bags) std::cout << "  " << b.filename().string() << "\n";

  std::vector<PointXYZT> debug;
  int grabbed = 0;
  auto on_scan = [&](Scan &s, int) {
    if (grabbed < 20) {
      debug.insert(debug.end(), s.points.begin(), s.points.end());
      ++grabbed;
    }
  };

  const auto stats = run_bag_stream(bags, on_scan, nullptr);

  std::cout << std::fixed;
  for (const auto &s : stats) {
    std::cout << "\n== " << s.name << " ==\n"
              << "  duration   : " << std::setprecision(2) << (s.t_end - s.t_start) << " s\n"
              << "  scans      : " << s.n_scans << "\n"
              << "  gt poses   : " << s.n_poses << "\n"
              << "  cloud size : min=" << s.cloud_min
              << " mean=" << std::setprecision(0) << s.cloud_mean
              << " max=" << s.cloud_max << "\n";
    const double pct2 = s.raw_sum > 0 ? 100.0 * s.dropped_2nd_sum       / s.raw_sum : 0.0;
    const double pctn = s.raw_sum > 0 ? 100.0 * s.dropped_nonfinite_sum / s.raw_sum : 0.0;
    const double avg_drop = s.n_scans > 0
        ? (s.dropped_2nd_sum + s.dropped_nonfinite_sum) / s.n_scans : 0.0;
    std::cout << "  dropped    : 2nd-return " << std::setprecision(1) << pct2
              << "%  non-finite " << pctn
              << "%  (avg/scan: " << std::setprecision(0) << avg_drop << ")\n"
              << "  sweep span : " << std::setprecision(4)
              << s.sweep_span_min << " .. " << s.sweep_span_max
              << " (mean " << s.sweep_span_mean << ") s\n"
              << "  gt path    : " << std::setprecision(2) << s.gt_path << " m\n";
  }

  const auto gaps = compute_gaps(stats);
  if (!gaps.empty()) std::cout << "\n== inter-bag gaps ==\n";
  for (size_t i = 0; i < gaps.size(); ++i) {
    std::cout << "  " << stats[i].name << " -> " << stats[i + 1].name << ":  "
              << "dt=" << std::setprecision(1) << gaps[i].dt_seconds << " s, "
              << "gt_dist=" << std::setprecision(3) << gaps[i].gt_distance_m << " m\n";
  }

  if (!write_pcd("debug.pcd", debug)) {
    std::cerr << "failed to write debug.pcd\n";
    return 1;
  }
  std::cout << "\nwrote debug.pcd (" << debug.size()
            << " pts from first " << grabbed << " scans)\n";
  return 0;
}
