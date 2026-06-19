#include "lidar_mapper/deskew.hpp"
#include "lidar_mapper/se3.hpp"

namespace lm {

void deskew(Scan &scan, const Eigen::Matrix<double, 6, 1> &xi_vel) {
  if (xi_vel.squaredNorm() < 1e-20) return;
  const double t_end = scan.t_max;
  const int n = static_cast<int>(scan.points.size());
  // Pure map: each point reads its own t and writes its own x/y/z — no shared state, no ordering
  // dependence — so the result is bit-identical for any thread count. (OpenMP already links lm_core.)
#ifdef _OPENMP
  #pragma omp parallel for schedule(static)
#endif
  for (int i = 0; i < n; ++i) {
    auto &pt = scan.points[i];
    const double dt = pt.t - t_end;  // in [-sweep_span, 0]
    const Eigen::Vector3d w = Exp_apply_small(xi_vel * dt, Eigen::Vector3d(pt.x, pt.y, pt.z));
    pt.x = static_cast<float>(w.x());
    pt.y = static_cast<float>(w.y());
    pt.z = static_cast<float>(w.z());
  }
}

}  // namespace lm
