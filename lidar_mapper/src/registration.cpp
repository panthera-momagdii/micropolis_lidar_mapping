#include "lidar_mapper/registration.hpp"
#include "lidar_mapper/se3.hpp"
#include <Eigen/Cholesky>
#include <cmath>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace lm {

namespace {
inline double huber_w(double r, double d) {
  const double a = std::abs(r);
  return (a <= d) ? 1.0 : d / a;
}
}  // namespace

IcpResult register_p2pl(const std::vector<PointXYZT> &src, const VoxelMap &map,
                        Eigen::Isometry3d &T,
                        double max_corr_dist, double huber_delta, int max_iters) {
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  IcpResult res;
  const double d2max = max_corr_dist * max_corr_dist;
  const int n = static_cast<int>(src.size());

  // Per-point normal-equation contributions. The expensive per-point work (neighbor search +
  // Jacobian/products) is computed in parallel; the contributions are then SUMMED SERIALLY in
  // the original point order. This keeps H/g bit-identical to the serial loop — the scan-to-map
  // odometry is a chaotic integrator, so a reordered cross-thread sum (~1e-12 per element) would
  // drift the trajectory by cm over a session. Reducing in index order makes the OpenMP build
  // byte-identical to serial. (Threads write disjoint indices i — no race, no critical section.)
  std::vector<Mat6> Hc(n);
  std::vector<Vec6> gc(n);
  std::vector<char> mc(n);

  for (int it = 0; it < max_iters; ++it) {
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d t = T.translation();

    auto compute = [&](int i, std::vector<const Voxel *> &nbrs) {
      const PointXYZT &sp = src[i];
      const Eigen::Vector3d p(sp.x, sp.y, sp.z);
      const Eigen::Vector3d q = R * p + t;
      map.neighbors(q, nbrs);
      const Voxel *best = nullptr;
      double best_d2 = d2max;
      for (const auto *vp : nbrs) {
        const double d2 = (q - vp->mean).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best = vp; }
      }
      if (!best) { mc[i] = 0; return; }
      // right perturbation: dq/d(delta) = [R | -R*hat(p)]
      Eigen::Matrix<double, 3, 6> Jq;
      Jq.block<3, 3>(0, 0) = R;
      Jq.block<3, 3>(0, 3) = -R * hat(p);
      const Eigen::Vector3d delta = q - best->mean;
      if (best->planarity > map.min_planarity) {
        const double r = best->normal.dot(delta);
        const Eigen::Matrix<double, 1, 6> J = best->normal.transpose() * Jq;
        const double w = huber_w(r, huber_delta);
        Hc[i] = w * J.transpose() * J;
        gc[i] = -(w * J.transpose() * r);
      } else {
        const double rn = delta.norm();
        const double w = huber_w(rn, huber_delta);
        Hc[i] = w * Jq.transpose() * Jq;
        gc[i] = -(w * Jq.transpose() * delta);
      }
      mc[i] = 1;
    };

#ifdef _OPENMP
    // Parallelize the per-point loop; thread count from OMP_NUM_THREADS (default hardware
    // concurrency). Each thread gets its own neighbor scratch and writes disjoint Hc/gc/mc[i].
    #pragma omp parallel
    {
      std::vector<const Voxel *> nbrs;
      nbrs.reserve(27);
      #pragma omp for schedule(static)
      for (int i = 0; i < n; ++i) compute(i, nbrs);
    }
#else
    std::vector<const Voxel *> nbrs;
    nbrs.reserve(27);
    for (int i = 0; i < n; ++i) compute(i, nbrs);
#endif

    // serial reduction in index order == original serial accumulation order (bit-identical).
    Mat6 H = Mat6::Zero();
    Vec6 g = Vec6::Zero();
    int matched = 0;
    for (int i = 0; i < n; ++i) {
      if (!mc[i]) continue;
      H += Hc[i];
      g += gc[i];
      ++matched;
    }

    res.matched = matched;
    res.iters = it + 1;
    if (matched < 10) return res;
    const Vec6 dx = H.ldlt().solve(g);  // solve + update stay serial, once per iteration
    T = T * Exp(dx);
    if (dx.head<3>().norm() < 1e-4 && dx.tail<3>().norm() < 1e-5) {
      res.converged = true;
      return res;
    }
  }
  return res;
}

}  // namespace lm
