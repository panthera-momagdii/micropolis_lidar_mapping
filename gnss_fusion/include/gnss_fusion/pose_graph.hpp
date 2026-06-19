#pragma once
#include "lidar_mapper/se3.hpp"  // reuse Phase-2 SE(3) exactly: Exp/Log, [rho,phi] twist, T <- T*Exp(d)
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <cassert>
#include <vector>

namespace gf {

// between-factor measurement Z = (Ti^A)^-1 * Tj^A from Variant A; sequential within one sub-chain.
struct OdomEdge { int i, j; Eigen::Isometry3d Z; };

// unary GNSS prior: nearest-in-time GNSS ENU position for node i (position only, no orientation).
struct GnssPrior { int i; Eigen::Vector3d p; };

struct IterStat {
  double cost = 0;
  double odom_mean = 0, odom_max = 0;  // odometry-edge translation residual [m]
  double gnss_mean = 0, gnss_max = 0;  // GNSS position residual [m]
  double dmax_trans = 0, dmax_rot = 0;  // max applied step, for the stop test
  bool solve_ok = true;                 // false if the sparse LDLT factorization failed (step aborted)
};

// --- per-factor linearization (extracted from PoseGraph::step for testability; no virtuals) ---
// Odometry between-factor. Returns residual r = Log(Z^-1 Xi^-1 Xj) and A = Adj(D^-1), D = Xi^-1 Xj.
// The Gauss-Newton right-perturbation Jacobians (valid at the operating point r~0) are Ji = -A,
// Jj = I. step() forms the weighted normal-equation blocks (A^T W A, etc.) from these.
struct OdomLin {
  Eigen::Matrix<double, 6, 1> r;
  Eigen::Matrix<double, 6, 6> A;
};
OdomLin linearize_odom(const Eigen::Isometry3d &Xi, const Eigen::Isometry3d &Xj,
                       const Eigen::Isometry3d &Z);

// GNSS unary position prior. Returns residual r = Xi.t - p and R = Xi.linear(). The (exact)
// Jacobian is d r/d rho_i = R, d r/d phi_i = 0, i.e. J = [R | 0_3x3].
struct GnssLin {
  Eigen::Vector3d r;
  Eigen::Matrix3d R;
};
GnssLin linearize_gnss(const Eigen::Isometry3d &Xi, const Eigen::Vector3d &p);

// sparse Gauss-Newton over SE(3) nodes (world_from_body, ENU). Nodes/edges/priors are added
// through the API; X is optimized in place. The invariant — every edge/prior references a valid
// node index — is enforced at insertion (assert).
class PoseGraph {
 public:
  void set_weights(double w_trans, double w_rot, double gnss_info) {
    w_trans_ = w_trans; w_rot_ = w_rot; gnss_info_ = gnss_info;  // Omega_odom = diag(w_t*I3, w_r*I3)
  }

  // add a node with its initial pose; returns the node index.
  int add_node(const Eigen::Isometry3d &T) {
    X_.push_back(T);
    return static_cast<int>(X_.size()) - 1;
  }
  void add_odom_edge(int i, int j, const Eigen::Isometry3d &Z) {
    assert(i >= 0 && j >= 0 && i < static_cast<int>(X_.size()) && j < static_cast<int>(X_.size()));
    odom_.push_back({i, j, Z});
  }
  void add_gnss_prior(int i, const Eigen::Vector3d &p) {
    assert(i >= 0 && i < static_cast<int>(X_.size()));
    gnss_.push_back({i, p});
  }

  IterStat step();                          // one linearize -> sparse solve -> right-update
  std::vector<IterStat> optimize(int max_iters);
  const std::vector<Eigen::Isometry3d> &poses() const { return X_; }

 private:
  std::vector<Eigen::Isometry3d> X_;
  std::vector<OdomEdge> odom_;
  std::vector<GnssPrior> gnss_;
  double w_trans_ = 1, w_rot_ = 1;  // Omega_odom = diag(w_trans*I3, w_rot*I3)
  double gnss_info_ = 1;            // Omega_gnss = (1/sigma^2)*I3
};

}  // namespace gf
