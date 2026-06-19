#include "lidar_mapper/voxel_map.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <unordered_set>

namespace lm {

namespace {
constexpr int64_t kBias = 1LL << 20;
constexpr int64_t kMask = (1LL << 21) - 1;
}  // namespace

int64_t VoxelMap::key(double x, double y, double z, double v) {
  const int64_t i = static_cast<int64_t>(std::floor(x / v)) + kBias;
  const int64_t j = static_cast<int64_t>(std::floor(y / v)) + kBias;
  const int64_t k = static_cast<int64_t>(std::floor(z / v)) + kBias;
  return (i & kMask) | ((j & kMask) << 21) | ((k & kMask) << 42);
}

void VoxelMap::insert(const Eigen::Vector3d &p) {
  const int64_t k = key(p.x(), p.y(), p.z(), v_size);
  Voxel &v = cells[k];
  if (v.n >= max_n) return;
  // welford
  const Eigen::Vector3d d_old = p - v.mean;
  v.n += 1;
  v.mean += d_old / v.n;
  const Eigen::Vector3d d_new = p - v.mean;
  v.M2 += d_old * d_new.transpose();
  if (v.n >= min_plane_n) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(v.M2 / (v.n - 1));
    const auto &lam = es.eigenvalues();  // ascending: lam[0] ≤ lam[1] ≤ lam[2]
    v.normal    = es.eigenvectors().col(0);
    v.planarity = (lam[2] > 1e-12) ? (lam[1] - lam[0]) / lam[2] : 0.0;
    v.has_plane = true;
  }
}

void VoxelMap::neighbors(const Eigen::Vector3d &p, std::vector<const Voxel *> &out) const {
  out.clear();
  const int64_t ci = static_cast<int64_t>(std::floor(p.x() / v_size)) + kBias;
  const int64_t cj = static_cast<int64_t>(std::floor(p.y() / v_size)) + kBias;
  const int64_t ck = static_cast<int64_t>(std::floor(p.z() / v_size)) + kBias;
  for (int di = -1; di <= 1; ++di)
    for (int dj = -1; dj <= 1; ++dj)
      for (int dk = -1; dk <= 1; ++dk) {
        const int64_t k = ((ci + di) & kMask) | (((cj + dj) & kMask) << 21) | (((ck + dk) & kMask) << 42);
        auto it = cells.find(k);
        if (it != cells.end() && it->second.n >= min_plane_n) out.push_back(&it->second);
      }
}

void VoxelMap::crop(const Eigen::Vector3d &center, double radius) {
  const double r2 = radius * radius;
  for (auto it = cells.begin(); it != cells.end();) {
    if ((it->second.mean - center).squaredNorm() > r2) it = cells.erase(it);
    else ++it;
  }
}

void voxel_subsample(const std::vector<PointXYZT> &in, double v, std::vector<PointXYZT> &out) {
  out.clear();
  out.reserve(in.size() / 4);
  std::unordered_set<int64_t> seen;
  seen.reserve(in.size() / 4);
  for (const auto &p : in) {
    const int64_t k = VoxelMap::key(p.x, p.y, p.z, v);
    if (seen.insert(k).second) out.push_back(p);
  }
}

}  // namespace lm
