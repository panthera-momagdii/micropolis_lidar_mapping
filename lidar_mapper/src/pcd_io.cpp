#include "lidar_mapper/pcd_io.hpp"
#include <fstream>

namespace lm {

bool write_pcd(const std::string &path, const std::vector<PointXYZT> &pts) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << "# .PCD v0.7 - Point Cloud Data file format\n"
       "VERSION 0.7\n"
       "FIELDS x y z intensity\n"
       "SIZE 4 4 4 4\n"
       "TYPE F F F F\n"
       "COUNT 1 1 1 1\n"
       "WIDTH "  << pts.size() << "\n"
       "HEIGHT 1\n"
       "VIEWPOINT 0 0 0 1 0 0 0\n"
       "POINTS "  << pts.size() << "\n"
       "DATA binary\n";
  for (const auto &p : pts) {
    f.write(reinterpret_cast<const char *>(&p.x), 4);
    f.write(reinterpret_cast<const char *>(&p.y), 4);
    f.write(reinterpret_cast<const char *>(&p.z), 4);
    f.write(reinterpret_cast<const char *>(&p.intensity), 4);
  }
  return static_cast<bool>(f);
}

}  // namespace lm
