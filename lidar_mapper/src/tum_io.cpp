#include "lidar_mapper/tum_io.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace lm {

bool TumWriter::open(const std::string &path, bool append) {
  f.open(path, append ? std::ios::app : std::ios::trunc);
  if (!f) return false;
  f << std::fixed << std::setprecision(9);
  return true;
}

void TumWriter::write(double t, const Eigen::Vector3d &p, const Eigen::Quaterniond &q) {
  f << t << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
    << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
}

bool TumReader::read(const std::string &path, std::vector<double> &stamps,
                     std::vector<Eigen::Isometry3d> &poses) {
  stamps.clear();
  poses.clear();
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  double t, x, y, z, qx, qy, qz, qw;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    T.translation() = Eigen::Vector3d(x, y, z);
    stamps.push_back(t);
    poses.push_back(T);
  }
  return true;
}

}  // namespace lm
