// Standalone finite-difference check of the pose-graph factor Jacobians (no ROS, plain g++ + Eigen).
//
// build & run (from this directory, gnss_fusion/test/):
//   g++ -std=c++17 -O2 -I../include -I../../lidar_mapper/include -I/usr/include/eigen3 \
//       test_pose_graph_jac.cpp ../src/pose_graph.cpp -o /tmp/test_pose_graph_jac \
//     && /tmp/test_pose_graph_jac
//
// Verifies the analytic Jacobians returned by gf::linearize_odom / gf::linearize_gnss against
// central finite differences of the residuals, using the same SE(3) right-perturbation convention
// (X <- X * Exp(d)) the solver uses.
//   - odometry edge: Ji = -A, Jj = I  (Gauss-Newton, exact at the operating point r=0, which is
//     where the solver linearizes; the test sets Z = Xi^-1 Xj so r = 0).
//   - GNSS prior:    J = [R | 0]       (exact for any pose).
#include "gnss_fusion/pose_graph.hpp"
#include "lidar_mapper/se3.hpp"
#include <Eigen/Geometry>
#include <cstdio>

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Iso = Eigen::Isometry3d;

static Iso expmap(double a, double b, double c, double d, double e, double f) {
  Vec6 xi; xi << a, b, c, d, e, f;
  return lm::Exp(xi);
}
static Vec6 basis(int k) { Vec6 v = Vec6::Zero(); v[k] = 1.0; return v; }

// residual of the odometry between-factor: r = Log(Z^-1 Xi^-1 Xj)
static Vec6 res_odom(const Iso &Xi, const Iso &Xj, const Iso &Z) {
  return lm::Log(Z.inverse() * (Xi.inverse() * Xj));
}

int main() {
  const double eps = 1e-6;
  double worst = 0.0;
  bool ok = true;

  // ---------- odometry edge ----------
  const Iso Xi = expmap(0.5, -0.3, 1.2, 0.10, -0.20, 0.05);
  const Iso Xj = Xi * expmap(0.2, 0.1, -0.15, 0.03, 0.04, -0.02);
  const Iso Z = Xi.inverse() * Xj;  // -> residual r = 0 at the linearization point
  const gf::OdomLin lin = gf::linearize_odom(Xi, Xj, Z);
  const Mat6 Ji = -lin.A;            // analytic d r / d di
  const Mat6 Jj = Mat6::Identity();  // analytic d r / d dj

  printf("odom residual norm at linearization point = %.3e (expect ~0)\n", lin.r.norm());

  Mat6 fdJi, fdJj;
  for (int k = 0; k < 6; ++k) {
    const Vec6 ek = basis(k);
    fdJi.col(k) = (res_odom(Xi * lm::Exp(eps * ek), Xj, Z) -
                   res_odom(Xi * lm::Exp(-eps * ek), Xj, Z)) / (2 * eps);
    fdJj.col(k) = (res_odom(Xi, Xj * lm::Exp(eps * ek), Z) -
                   res_odom(Xi, Xj * lm::Exp(-eps * ek), Z)) / (2 * eps);
  }
  const double eJi = (fdJi - Ji).cwiseAbs().maxCoeff();
  const double eJj = (fdJj - Jj).cwiseAbs().maxCoeff();
  printf("odom  max|FD-Ji|=%.3e  max|FD-Jj|=%.3e\n", eJi, eJj);
  worst = std::max({worst, eJi, eJj});

  // ---------- GNSS prior ----------
  const Iso Xg = expmap(1.0, 2.0, 3.0, 0.2, -0.1, 0.3);
  const Eigen::Vector3d p = Xg.translation() - Eigen::Vector3d(0.1, 0.2, -0.05);
  const gf::GnssLin gl = gf::linearize_gnss(Xg, p);
  Eigen::Matrix<double, 3, 6> Jg;
  Jg.leftCols<3>() = gl.R;                       // d r / d rho = R
  Jg.rightCols<3>() = Eigen::Matrix3d::Zero();   // d r / d phi = 0
  Eigen::Matrix<double, 3, 6> fdJg;
  for (int k = 0; k < 6; ++k) {
    const Vec6 ek = basis(k);
    const Eigen::Vector3d rp = (Xg * lm::Exp(eps * ek)).translation() - p;
    const Eigen::Vector3d rm = (Xg * lm::Exp(-eps * ek)).translation() - p;
    fdJg.col(k) = (rp - rm) / (2 * eps);
  }
  const double eJg = (fdJg - Jg).cwiseAbs().maxCoeff();
  printf("gnss  max|FD-J|=%.3e\n", eJg);
  worst = std::max(worst, eJg);

  const double tol = 1e-5;  // central-difference truncation with eps=1e-6
  ok = worst < tol;
  printf("%s (worst %.3e, tol %.0e)\n", ok ? "PASS" : "FAIL", worst, tol);
  return ok ? 0 : 1;
}
