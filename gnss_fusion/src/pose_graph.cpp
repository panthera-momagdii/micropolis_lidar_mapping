#include "gnss_fusion/pose_graph.hpp"
#include <algorithm>
#include <iostream>

namespace gf {
namespace {
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// SE(3) adjoint in the [rho,phi] (translation-first) convention: Exp(Adj(T) xi) = T Exp(xi) T^-1.
// Adj = [[R, [t]x R],[0, R]]. Used for the between-factor's d r/d delta_i.
Mat6 Adj(const Eigen::Isometry3d &T) {
  const Eigen::Matrix3d R = T.linear();
  Mat6 A = Mat6::Zero();
  A.topLeftCorner<3, 3>() = R;
  A.topRightCorner<3, 3>() = lm::hat(T.translation()) * R;
  A.bottomRightCorner<3, 3>() = R;
  return A;
}
}  // namespace

OdomLin linearize_odom(const Eigen::Isometry3d &Xi, const Eigen::Isometry3d &Xj,
                       const Eigen::Isometry3d &Z) {
  const Eigen::Isometry3d D = Xi.inverse() * Xj;
  OdomLin out;
  out.r = lm::Log(Z.inverse() * D);
  out.A = Adj(D.inverse());  // Ji = -A, Jj = I
  return out;
}

GnssLin linearize_gnss(const Eigen::Isometry3d &Xi, const Eigen::Vector3d &p) {
  GnssLin out;
  out.R = Xi.linear();
  out.r = Xi.translation() - p;
  return out;
}

IterStat PoseGraph::step() {
  const int N = static_cast<int>(X_.size());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(odom_.size() * 144 + gnss_.size() * 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * N);

  Vec6 wdiag;
  wdiag << w_trans_, w_trans_, w_trans_, w_rot_, w_rot_, w_rot_;
  const Mat6 W = wdiag.asDiagonal();

  IterStat s;
  auto add = [&](int bi, int bj, const Mat6 &B) {
    for (int a = 0; a < 6; ++a)
      for (int c = 0; c < 6; ++c) trips.emplace_back(6 * bi + a, 6 * bj + c, B(a, c));
  };

  // --- odometry between-factors ---
  // r = Log( Z^-1 * (Xi^-1 Xj) ).  Right-perturb Xj: Jj = d r/d dj = Jr^-1(r) ~ I (near-converged).
  // Right-perturb Xi: Xi^-1 -> Exp(-di) Xi^-1, so Z^-1 Xi^-1 Xj Exp(-Adj(D^-1) di) gives
  // Ji = -Adj(D^-1), D = Xi^-1 Xj.  Numerically Ji ~ -I (per-scan motion is tiny).
  // WHY THE EXACT ADJOINT, not the textbook Ji ~ -I shortcut: with position-only GNSS the
  // translation Jacobian d t/d phi = 0 (see below), so a uniform per-sub-chain rotation (di = [0,w])
  // spins every node in place -> node positions don't move -> GNSS sees nothing, and with Ji ~ -I,
  // Jj = +I the between-factor residual is also unchanged. That makes [0,w] an exact null vector,
  // H rank-deficient, and LDLT fail. The [t]x R block of Adj(D^-1) is precisely the coupling that
  // ties attitude to the GNSS-pinned positions, so the absolute attitude becomes observable per chain.
  for (const auto &e : odom_) {
    const OdomLin lin = linearize_odom(X_[e.i], X_[e.j], e.Z);
    const Vec6 &r = lin.r;
    const Mat6 &A = lin.A;  // Ji = -A,  Jj = I
    const Mat6 WA = W * A;
    add(e.i, e.i, A.transpose() * WA);   // Hii = Ji^T W Ji = A^T W A
    add(e.j, e.j, W);                    // Hjj = Jj^T W Jj = W
    add(e.i, e.j, -A.transpose() * W);   // Hij = Ji^T W Jj
    add(e.j, e.i, -WA);                  // Hji = Jj^T W Ji = -W A
    b.segment<6>(6 * e.i) += A.transpose() * (W * r);  // b_i = -Ji^T W r
    b.segment<6>(6 * e.j) += -(W * r);                 // b_j = -Jj^T W r
    s.cost += r.dot(W * r);
    const double rt = r.head<3>().norm();
    s.odom_mean += rt;
    s.odom_max = std::max(s.odom_max, rt);
  }

  // --- GNSS position priors (unary) ---
  // r = Xi.t - p.  Right-perturb: Xi.t -> Xi.t + R_i*(V(phi)*rho) ~ Xi.t + R_i*rho to 1st order,
  // so d r/d rho = R_i and d r/d phi = 0 (position is independent of small rotation to 1st order).
  // Hence H adds info*I3 to the rho block (R^T R = I) and b adds -info*R_i^T*r.
  for (const auto &g : gnss_) {
    const GnssLin lin = linearize_gnss(X_[g.i], g.p);
    const Eigen::Matrix3d &R = lin.R;
    const Eigen::Vector3d &r = lin.r;
    const Eigen::Vector3d br = -gnss_info_ * (R.transpose() * r);
    for (int k = 0; k < 3; ++k) {
      trips.emplace_back(6 * g.i + k, 6 * g.i + k, gnss_info_);
      b[6 * g.i + k] += br[k];
    }
    s.cost += gnss_info_ * r.squaredNorm();
    const double rn = r.norm();
    s.gnss_mean += rn;
    s.gnss_max = std::max(s.gnss_max, rn);
  }

  // --- assemble sparse H, solve H d = b, apply right-update Xi <- Xi*Exp(di) ---
  Eigen::SparseMatrix<double> H(6 * N, 6 * N);
  H.setFromTriplets(trips.begin(), trips.end());
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(H);
  if (solver.info() != Eigen::Success) {
    // singular H: do NOT apply a garbage step. report and abort this iteration.
    std::cerr << "error: LDLT factorization failed (singular H); aborting step\n";
    s.solve_ok = false;
    if (!odom_.empty()) s.odom_mean /= odom_.size();
    if (!gnss_.empty()) s.gnss_mean /= gnss_.size();
    return s;
  }
  const Eigen::VectorXd dx = solver.solve(b);

  for (int i = 0; i < N; ++i) {
    const Vec6 d = dx.segment<6>(6 * i);
    X_[i] = X_[i] * lm::Exp(d);
    s.dmax_trans = std::max(s.dmax_trans, d.head<3>().norm());
    s.dmax_rot = std::max(s.dmax_rot, d.tail<3>().norm());
  }
  if (!odom_.empty()) s.odom_mean /= odom_.size();
  if (!gnss_.empty()) s.gnss_mean /= gnss_.size();
  return s;
}

std::vector<IterStat> PoseGraph::optimize(int max_iters) {
  std::vector<IterStat> hist;
  for (int it = 0; it < max_iters; ++it) {
    const IterStat s = step();
    hist.push_back(s);
    if (!s.solve_ok) break;                                // factorization failed: stop
    if (s.dmax_trans < 1e-4 && s.dmax_rot < 1e-5) break;  // converged
  }
  return hist;
}

}  // namespace gf
