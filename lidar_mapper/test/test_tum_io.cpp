// Standalone round-trip test for lm::TumWriter / lm::TumReader (no ROS, plain g++ + Eigen).
//
// build & run (from this directory, lidar_mapper/test/):
//   g++ -std=c++17 -O2 -I../include -I/usr/include/eigen3 \
//       test_tum_io.cpp ../src/tum_io.cpp -o /tmp/test_tum_io && /tmp/test_tum_io
//
// Writes a handful of SE(3) poses via TumWriter, reads them back via TumReader, and checks the
// recovered timestamp / translation / rotation match to ~1e-9 (TumWriter emits 9 decimals).
#include "lidar_mapper/tum_io.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

int main() {
  using namespace lm;
  const std::vector<double> ts = {100.0, 100.1, 1234.567890123, 9999.999999999};
  const std::vector<Eigen::Vector3d> ps = {
      {0, 0, 0},
      {1.5, -2.25, 3.125},
      {123.456789, -987.654321, 42.0},
      {-0.000001, 0.001, 5.0}};
  std::vector<Eigen::Quaterniond> qs = {
      Eigen::Quaterniond::Identity(),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0, 0, 1))),
      Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(1, 1, 0).normalized())),
      Eigen::Quaterniond(Eigen::AngleAxisd(-2.0, Eigen::Vector3d(0.2, 0.5, -0.8).normalized()))};
  for (auto &q : qs) q.normalize();

  const char *path = "/tmp/test_tum_io_roundtrip.tum";
  {
    TumWriter w;
    if (!w.open(path)) { std::printf("FAIL: cannot open %s for write\n", path); return 1; }
    for (size_t i = 0; i < ts.size(); ++i) w.write(ts[i], ps[i], qs[i]);
  }

  std::vector<double> ts2;
  std::vector<Eigen::Isometry3d> Ts2;
  if (!TumReader::read(path, ts2, Ts2)) { std::printf("FAIL: cannot read %s\n", path); return 1; }
  if (ts2.size() != ts.size()) {
    std::printf("FAIL: count %zu != %zu\n", ts2.size(), ts.size());
    return 1;
  }

  double max_t = 0, max_p = 0, max_R = 0;
  for (size_t i = 0; i < ts.size(); ++i) {
    max_t = std::max(max_t, std::abs(ts2[i] - ts[i]));
    max_p = std::max(max_p, (Ts2[i].translation() - ps[i]).cwiseAbs().maxCoeff());
    const Eigen::Matrix3d dR = Ts2[i].linear() - qs[i].toRotationMatrix();
    max_R = std::max(max_R, dR.cwiseAbs().maxCoeff());
  }
  std::printf("round-trip: max |dt|=%.3e  max |dpos|=%.3e  max |dR|=%.3e\n", max_t, max_p, max_R);

  const double tol = 1e-8;  // TumWriter prints 9 decimals; quaternion->matrix amplifies ~2-3x
  if (max_t < tol && max_p < tol && max_R < tol) {
    std::printf("PASS (tol %.0e)\n", tol);
    return 0;
  }
  std::printf("FAIL: exceeds tol %.0e\n", tol);
  return 1;
}
