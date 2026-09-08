#include "dynasense/BpsMap.hpp"

#include <algorithm>
#include <cmath>

namespace dynasense {

// ---------------------------------------------------------------------------
// VoxelLruBuffer
// ---------------------------------------------------------------------------

void VoxelLruBuffer::setCapacity(std::size_t capacity) {
  capacity_ = capacity;
  while (order_.size() > capacity_) {
    index_.erase(order_.back());
    order_.pop_back();
  }
  index_.reserve(capacity_ + 1);
}

bool VoxelLruBuffer::insert(const VoxelKey& key) {
  if (capacity_ == 0) {
    return false;
  }

  const auto it = index_.find(key);
  if (it != index_.end()) {
    // Already buffered: refresh recency only, do not duplicate.
    order_.splice(order_.begin(), order_, it->second);
    return false;
  }

  order_.push_front(key);
  index_[key] = order_.begin();

  if (order_.size() > capacity_) {
    index_.erase(order_.back());
    order_.pop_back();
  }
  return true;
}

void VoxelLruBuffer::clear() {
  order_.clear();
  index_.clear();
}

// ---------------------------------------------------------------------------
// BpsMap
// ---------------------------------------------------------------------------

void BpsMap::configure(const BpsMapConfig& config) {
  config_ = config;
  if (config_.voxelSize <= 0.0) {
    config_.voxelSize = 0.05;
  }
  if (config_.maxQueryDistance <= 0.0) {
    config_.maxQueryDistance = 0.5;
  }
  // A disabled activation path gets capacity 0. VoxelLruBuffer::insert() bails
  // out on a zero capacity, so this is an exact off switch rather than a filter
  // applied somewhere downstream.
  contactBuffer_.setCapacity(config_.contactEnabled ? config_.contactCapacity : 0u);
  rayBuffer_.setCapacity(config_.rayEnabled ? config_.rayCapacity : 0u);
}

VoxelKey BpsMap::worldToVoxel(const Eigen::Vector3d& pointWorld) const {
  const double inv = 1.0 / config_.voxelSize;
  return VoxelKey{static_cast<int32_t>(std::floor(pointWorld.x() * inv)),
                  static_cast<int32_t>(std::floor(pointWorld.y() * inv)),
                  static_cast<int32_t>(std::floor(pointWorld.z() * inv))};
}

Eigen::Vector3d BpsMap::voxelCentre(const VoxelKey& key) const {
  const double s = config_.voxelSize;
  return Eigen::Vector3d((static_cast<double>(key.x) + 0.5) * s, (static_cast<double>(key.y) + 0.5) * s,
                         (static_cast<double>(key.z) + 0.5) * s);
}

bool BpsMap::accept(const Eigen::Vector3d& pointWorld) const {
  if (!pointWorld.allFinite()) {
    return false;
  }

  // Horizontal extent gate: reproduces the training grid's 8 x 8 m footprint,
  // which was anchored to the (nonexistent here) per-environment origin.
  const double dx = pointWorld.x() - basePositionWorld_.x();
  const double dy = pointWorld.y() - basePositionWorld_.y();
  if (std::abs(dx) > config_.maxHorizontalRange || std::abs(dy) > config_.maxHorizontalRange) {
    return false;
  }

  // Height gate is applied to the voxel *centre*, not the raw hit, so that a
  // hit just below the threshold cannot sneak a voxel in.
  const double centreZ = voxelCentre(worldToVoxel(pointWorld)).z();
  const double heightAboveGround = centreZ - config_.groundZ;
  return heightAboveGround >= config_.minHeightAboveGround && heightAboveGround < config_.maxHeightAboveGround;
}

bool BpsMap::addContactPoint(const Eigen::Vector3d& pointWorld) {
  if (!accept(pointWorld)) {
    return false;
  }
  contactBuffer_.insert(worldToVoxel(pointWorld));
  return true;
}

bool BpsMap::addRayPoint(const Eigen::Vector3d& pointWorld) {
  if (!accept(pointWorld)) {
    return false;
  }
  rayBuffer_.insert(worldToVoxel(pointWorld));
  return true;
}

void BpsMap::clear() {
  contactBuffer_.clear();
  rayBuffer_.clear();
}

void BpsMap::accumulateNearest(const VoxelLruBuffer& buffer, const Eigen::Vector3d& pointWorld, double& bestSqDist,
                               Eigen::Vector3d& bestOutside, Eigen::Vector3d& bestSign, bool& found) const {
  const double halfWidth = 0.5 * config_.voxelSize;

  for (const VoxelKey& key : buffer.entries()) {
    const Eigen::Vector3d centre = voxelCentre(key);
    const Eigen::Vector3d delta = pointWorld - centre;

    // Exterior distance to the axis-aligned cube (zero inside the cube).
    const Eigen::Vector3d outside = (delta.cwiseAbs() - Eigen::Vector3d::Constant(halfWidth)).cwiseMax(0.0);
    const double sqDist = outside.squaredNorm();

    if (!found || sqDist < bestSqDist) {
      found = true;
      bestSqDist = sqDist;
      bestOutside = outside;
      bestSign = Eigen::Vector3d(delta.x() < 0.0 ? -1.0 : 1.0, delta.y() < 0.0 ? -1.0 : 1.0,
                                 delta.z() < 0.0 ? -1.0 : 1.0);
    }
  }
}

Eigen::Vector3d BpsMap::unitGradient(const Eigen::Vector3d& pointWorld, const Eigen::Vector3d& centre) const {
  const double halfWidth = 0.5 * config_.voxelSize;
  const Eigen::Vector3d delta = pointWorld - centre;
  const Eigen::Vector3d outside = (delta.cwiseAbs() - Eigen::Vector3d::Constant(halfWidth)).cwiseMax(0.0);
  const Eigen::Vector3d sign(delta.x() < 0.0 ? -1.0 : 1.0, delta.y() < 0.0 ? -1.0 : 1.0,
                             delta.z() < 0.0 ? -1.0 : 1.0);

  Eigen::Vector3d dir = outside.cwiseProduct(sign);
  const double norm = dir.norm();
  if (norm > 0.0) {
    return dir / norm;
  }
  // Query point lies inside the cube: the gradient is undefined, so fall back to
  // the convention used in training.
  return Eigen::Vector3d::UnitZ();
}

BpsQueryResult BpsMap::query(const Eigen::Vector3d& pointWorld) const {
  BpsQueryResult result;
  result.distance = config_.maxQueryDistance;

  double bestSqDist = 0.0;
  Eigen::Vector3d bestOutside = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestSign = Eigen::Vector3d::Ones();
  bool found = false;

  // The nearest cube is taken across both buffers jointly.
  accumulateNearest(contactBuffer_, pointWorld, bestSqDist, bestOutside, bestSign, found);
  accumulateNearest(rayBuffer_, pointWorld, bestSqDist, bestOutside, bestSign, found);

  if (!found) {
    // No buffered cube: d saturates, gain is zero, and the actor value is
    // exactly zero. directionWorld stays at its zero default.
    //
    // unitDirectionWorld deliberately reproduces the training-time debug
    // artifact: the Play visualisation does not mask invalid buffer slots, so on
    // an empty map it takes the gradient against the zero-initialised slot 0,
    // and only falls back to +z when that happens to be degenerate. The
    // resulting arrow direction is meaningless - it is documented as such - but
    // reproducing it keeps this view identical to the training-time one.
    result.unitDirectionWorld = unitGradient(pointWorld, voxelCentre(VoxelKey{0, 0, 0}));
    return result;
  }
  result.valid = true;

  const double d = std::min(std::sqrt(bestSqDist), config_.maxQueryDistance);
  result.distance = d;

  // Gradient of the exterior-box SDF: normalize(outside * sign(p - c)), i.e. it
  // points from the cube toward the query point. This is verbatim the
  // expression the training sensor used, so the sign convention matches.
  Eigen::Vector3d dir = bestOutside.cwiseProduct(bestSign);
  const double norm = dir.norm();
  if (norm > 0.0) {
    dir /= norm;
  } else {
    dir = Eigen::Vector3d::UnitZ();
  }
  result.unitDirectionWorld = dir;

  const double gain = std::min(std::max((config_.maxQueryDistance - std::abs(d)) / config_.maxQueryDistance, 0.0), 1.0);
  result.directionWorld = gain * dir;
  return result;
}

void BpsMap::bufferedVoxelCentresUnion(std::vector<Eigen::Vector3d>& centres) const {
  centres.clear();
  centres.reserve(contactBuffer_.size() + rayBuffer_.size());

  // The two buffers dedup independently, so the same voxel can sit in both.
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  seen.reserve(contactBuffer_.size() + rayBuffer_.size() + 1);

  for (const VoxelLruBuffer* buffer : {&contactBuffer_, &rayBuffer_}) {
    for (const VoxelKey& key : buffer->entries()) {
      if (seen.insert(key).second) {
        centres.push_back(voxelCentre(key));
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Query-point layout
// ---------------------------------------------------------------------------

const std::vector<QueryPointSpec>& queryPointSpecs() {
  static const std::vector<QueryPointSpec> kSpecs = [] {
    std::vector<QueryPointSpec> specs;
    specs.reserve(kNumQueryPoints);

    // base: 6 points
    specs.push_back({QueryBody::Base, -1, {-0.4, 0.0, 0.0}});
    specs.push_back({QueryBody::Base, -1, {-0.2, 0.0, 0.0}});
    specs.push_back({QueryBody::Base, -1, {0.0, 0.0, 0.0}});
    specs.push_back({QueryBody::Base, -1, {0.2, 0.0, 0.0}});
    specs.push_back({QueryBody::Base, -1, {0.4, 0.0, 0.0}});
    specs.push_back({QueryBody::Base, -1, {-0.3, 0.0, 0.2}});

    // Per leg, in LF, LH, RF, RH order: THIGH 3, SHANK 1, FOOT 3.
    for (int leg = 0; leg < 4; ++leg) {
      specs.push_back({QueryBody::Thigh, leg, {0.0, 0.0, 0.0}});
      specs.push_back({QueryBody::Thigh, leg, {-0.1, 0.0, 0.0}});
      specs.push_back({QueryBody::Thigh, leg, {-0.1, 0.0, -0.15}});

      specs.push_back({QueryBody::Shank, leg, {0.0, 0.0, 0.0}});

      specs.push_back({QueryBody::Foot, leg, {0.0, 0.0, 0.0}});
      specs.push_back({QueryBody::Foot, leg, {0.0, 0.0, 0.1}});
      specs.push_back({QueryBody::Foot, leg, {0.0, 0.0, 0.2}});
    }

    return specs;
  }();

  return kSpecs;
}

}  // namespace dynasense
