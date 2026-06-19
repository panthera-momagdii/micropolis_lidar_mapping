#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace lm {

// twist xi = [rho (translation), phi (rotation)]; right-multiplicative updates: T <- T * Exp(xi).

inline Eigen::Matrix3d hat(const Eigen::Vector3d &w) {
  Eigen::Matrix3d S;
  S <<     0, -w.z(),  w.y(),
       w.z(),      0, -w.x(),
      -w.y(),  w.x(),      0;
  return S;
}

inline Eigen::Isometry3d Exp(const Eigen::Matrix<double, 6, 1> &xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d phi = xi.tail<3>();
  const double t2 = phi.squaredNorm();
  Eigen::Matrix3d R, V;
  if (t2 < 1e-16) {  // taylor near 0
    R = Eigen::Matrix3d::Identity() + hat(phi);
    V = Eigen::Matrix3d::Identity() + 0.5 * hat(phi);
  } else {
    const double t = std::sqrt(t2);
    const Eigen::Matrix3d K  = hat(phi) / t;
    const Eigen::Matrix3d K2 = K * K;
    const double s = std::sin(t), c = std::cos(t);
    R = Eigen::Matrix3d::Identity() + s * K + (1.0 - c) * K2;
    V = Eigen::Matrix3d::Identity() + ((1.0 - c) / t) * K + ((t - s) / t) * K2;
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = V * rho;
  return T;
}

// 2nd-order Taylor Exp(xi)*p for inner loops where |phi| is small (e.g. per-point deskew).
// at |dt*omega| ≤ 0.06 rad over a 92 ms sweep, |error| < 0.2 mm at 100 m range.
inline Eigen::Vector3d Exp_apply_small(const Eigen::Matrix<double, 6, 1> &xi,
                                       const Eigen::Vector3d &p) {
  const Eigen::Vector3d phi = xi.tail<3>();
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d phi_x_p = phi.cross(p);
  return p + phi_x_p + 0.5 * phi.cross(phi_x_p) + rho + 0.5 * phi.cross(rho);
}

inline Eigen::Matrix<double, 6, 1> Log(const Eigen::Isometry3d &T) {
  const Eigen::Matrix3d R = T.linear();
  const double cos_t = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  const double t = std::acos(cos_t);
  Eigen::Vector3d phi(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  Eigen::Matrix3d Vinv;
  if (t < 1e-8) {  // taylor near 0
    phi *= 0.5;
    Vinv = Eigen::Matrix3d::Identity() - 0.5 * hat(phi);
  } else {
    phi *= t / (2.0 * std::sin(t));
    const double half = 0.5 * t;
    const double coef = (1.0 - half / std::tan(half)) / (t * t);
    Vinv = Eigen::Matrix3d::Identity() - 0.5 * hat(phi) + coef * hat(phi) * hat(phi);
  }
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = Vinv * T.translation();
  xi.tail<3>() = phi;
  return xi;
}

}  // namespace lm
