#include "lidar_mapper/bag_stream.hpp"
#include "lidar_mapper/tum_io.hpp"
#include <iostream>

using namespace lm;

int main(int argc, char **argv) {
  // export GNSS ENU ground-truth poses (FP_ENU0) as TUM, in explicit bag order.
  //   export_gt <out.tum> <bag1.mcap> [bag2.mcap ...]   (single dir => globbed)
  if (argc < 3) {
    std::cerr << "usage: export_gt <out.tum> <bag1.mcap> [bag2.mcap ...]\n";
    return 1;
  }
  const std::string out_tum = argv[1];
  std::vector<std::string> bag_args(argv + 2, argv + argc);
  const auto bags = resolve_bags(bag_args);
  if (bags.empty()) { std::cerr << "no input bags resolved\n"; return 1; }

  TumWriter tum;
  if (!tum.open(out_tum)) { std::cerr << "cannot open " << out_tum << "\n"; return 1; }

  int n = 0;
  auto on_pose = [&](const GTPose &g, int) { tum.write(g.stamp, g.t, g.q); ++n; };
  run_bag_stream(bags, nullptr, on_pose);

  std::cout << "wrote " << n << " GT poses to " << out_tum << "\n";
  return 0;
}
