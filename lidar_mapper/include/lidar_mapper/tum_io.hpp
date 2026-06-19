#pragma once
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <vector>

namespace lm {

struct TumWriter {
  std::ofstream f;
  bool open(const std::string &path, bool append = false);
  void write(double t, const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
};

struct TumReader {
  // Read a TUM file (timestamp tx ty tz qx qy qz qw). Skips blank / '#' / malformed lines.
  // Quaternion is read in (x,y,z,w) order and normalized. Outputs are cleared first.
  // Returns false only if the file cannot be opened.
  static bool read(const std::string &path, std::vector<double> &stamps,
                   std::vector<Eigen::Isometry3d> &poses);
};

}  // namespace lm
