// Contact-history basis-point-set (BPS) map.
//
// Deployment-side reimplementation of the IsaacLab
// `contact_history_bps` sensor described in
// DS_LOCORESET_DEPTHHEIGHTSCAN_CONTACTBPS_DEPLOYMENT_HANDOVER.md.
//
// Two independent deduplicating LRU buffers of 5 cm voxels are fed by
// (a) physical contacts on the THIGH/SHANK bodies and (b) external depth/lidar
// ray hits. Querying a point returns the exterior-box SDF gradient of the
// nearest buffered voxel, attenuated by distance.
//
// This header is deliberately free of ROS and Gazebo so the geometry can be
// reasoned about (and unit-tested) on its own.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace dynasense {

/// Integer coordinates of a voxel on the global lattice.
struct VoxelKey {
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const noexcept {
    // Three odd 64-bit primes; cheap and good enough for a few hundred entries.
    std::size_t h = static_cast<std::size_t>(static_cast<uint32_t>(k.x)) * 0x9E3779B97F4A7C15ull;
    h ^= static_cast<std::size_t>(static_cast<uint32_t>(k.y)) * 0xC2B2AE3D27D4EB4Full + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(static_cast<uint32_t>(k.z)) * 0x165667B19E3779F9ull + (h << 6) + (h >> 2);
    return h;
  }
};

/// Fixed-capacity, deduplicating, least-recently-used set of voxels.
///
/// Re-inserting a voxel that is already present refreshes its recency rather
/// than adding a duplicate. Once at capacity, the least recently touched voxel
/// is evicted to make room.
class VoxelLruBuffer {
 public:
  explicit VoxelLruBuffer(std::size_t capacity = 0) : capacity_(capacity) { index_.reserve(capacity + 1); }

  void setCapacity(std::size_t capacity);
  std::size_t capacity() const { return capacity_; }
  std::size_t size() const { return order_.size(); }
  bool empty() const { return order_.empty(); }

  /// Insert `key`, or refresh it if already buffered. Returns true if the voxel
  /// was not previously present.
  bool insert(const VoxelKey& key);

  void clear();

  /// Most-recent-first iteration over the buffered voxels.
  const std::list<VoxelKey>& entries() const { return order_; }

 private:
  std::size_t capacity_;
  std::list<VoxelKey> order_;  ///< front = most recent, back = least recent
  std::unordered_map<VoxelKey, std::list<VoxelKey>::iterator, VoxelKeyHash> index_;
};

/// Result of a single BPS query point.
struct BpsQueryResult {
  /// Gain-scaled exterior-box SDF gradient in the world frame. This is what the
  /// actor observation uses (after rotation into the yaw-only base frame).
  /// Exactly zero when no voxel is buffered.
  Eigen::Vector3d directionWorld{Eigen::Vector3d::Zero()};

  /// UNIT gradient in the world frame, without the distance gain. Debug
  /// visualisation only: the arrow view draws a fixed-length arrow along this,
  /// and encodes distance in colour instead of length.
  ///
  /// When no voxel is buffered this is NOT zero. It reproduces the training-time
  /// debug artifact described in the Play task: the visualisation reads the
  /// zero-initialised slot 0 instead of masking invalid slots, so the direction
  /// is a meaningless artifact (see BpsMap::query). `directionWorld` stays
  /// exactly zero regardless, so the actor is unaffected.
  Eigen::Vector3d unitDirectionWorld{Eigen::Vector3d::UnitZ()};

  double distance{0.5};  ///< capped exterior-box SDF distance
  bool valid{false};     ///< false when no voxel is buffered
};

/// Configuration of the voxel lattice and the query response.
struct BpsMapConfig {
  double voxelSize{0.05};          ///< edge length of a voxel [m]
  double maxQueryDistance{0.5};    ///< distance at which the response saturates to zero [m]

  /// The two independent voxel-activation paths can be disabled individually.
  /// A disabled path is given capacity 0, so it never retains a voxel and never
  /// contributes to a query. Disabling BOTH leaves the map permanently empty:
  /// every BPS observation value is then exactly zero and the policy is blind to
  /// obstacles. That is a deliberate debugging configuration, not a fault.
  bool contactEnabled{true};  ///< physical THIGH/SHANK contacts
  bool rayEnabled{true};      ///< external depth-camera and lidar hits

  std::size_t contactCapacity{200};
  std::size_t rayCapacity{500};

  /// Voxels are kept only while their centre sits in
  /// [groundZ + minHeightAboveGround, groundZ + maxHeightAboveGround).
  /// This reproduces the training grid's z extent of [-0.2, 1.8) combined with
  /// its "at least 0.05 m above the ground surface" filter.
  double groundZ{0.0};
  double minHeightAboveGround{0.05};
  double maxHeightAboveGround{1.8};

  /// Voxels farther than this from the robot base (horizontally) are ignored,
  /// reproducing the training grid's 8 x 8 m extent.
  double maxHorizontalRange{4.0};
};

/// The contact-history BPS map: two LRU voxel buffers plus the query operator.
class BpsMap {
 public:
  BpsMap() = default;
  explicit BpsMap(const BpsMapConfig& config) { configure(config); }

  void configure(const BpsMapConfig& config);
  const BpsMapConfig& config() const { return config_; }

  /// Update the robot position used for the horizontal range gate.
  void setBasePosition(const Eigen::Vector3d& baseWorld) { basePositionWorld_ = baseWorld; }

  /// Register a physical contact point (world frame). Returns false if the
  /// point was rejected by the height or range gates.
  bool addContactPoint(const Eigen::Vector3d& pointWorld);

  /// Register an external depth/lidar hit (world frame). Returns false if the
  /// point was rejected by the height or range gates.
  bool addRayPoint(const Eigen::Vector3d& pointWorld);

  /// Clear both buffers. Called on episode/policy reset.
  void clear();

  std::size_t numContactVoxels() const { return contactBuffer_.size(); }
  std::size_t numRayVoxels() const { return rayBuffer_.size(); }

  /// Nearest-voxel exterior-box SDF query for a point in the world frame.
  ///
  ///   outside = max(|p - c| - h, 0)              componentwise, h = voxelSize/2
  ///   d       = min(||outside||, maxQueryDistance)
  ///   dir     = normalize(outside * sign(p - c)) (0,0,1) when the norm is zero
  ///   gain    = clamp((maxQueryDistance - |d|) / maxQueryDistance, 0, 1)
  ///   value   = gain * dir
  ///
  /// With no buffered voxel the result is zero with d = maxQueryDistance.
  BpsQueryResult query(const Eigen::Vector3d& pointWorld) const;

  /// Centres (world frame) of the UNION of both buffers, deduplicated. This is
  /// what the activated-voxel cube view draws: contact- and ray-activated
  /// voxels are not distinguished.
  void bufferedVoxelCentresUnion(std::vector<Eigen::Vector3d>& centres) const;

  VoxelKey worldToVoxel(const Eigen::Vector3d& pointWorld) const;
  Eigen::Vector3d voxelCentre(const VoxelKey& key) const;

 private:
  bool accept(const Eigen::Vector3d& pointWorld) const;
  void accumulateNearest(const VoxelLruBuffer& buffer, const Eigen::Vector3d& pointWorld, double& bestSqDist,
                         Eigen::Vector3d& bestOutside, Eigen::Vector3d& bestSign, bool& found) const;

  /// normalize(max(|p-c|-h, 0) * sign(p-c)), falling back to +z when the norm
  /// is zero (i.e. the query point lies inside the cube).
  Eigen::Vector3d unitGradient(const Eigen::Vector3d& pointWorld, const Eigen::Vector3d& centre) const;

  BpsMapConfig config_{};
  VoxelLruBuffer contactBuffer_{200};
  VoxelLruBuffer rayBuffer_{500};
  Eigen::Vector3d basePositionWorld_{Eigen::Vector3d::Zero()};
};

// ---------------------------------------------------------------------------
// Query-point layout (observation block [49,151))
// ---------------------------------------------------------------------------

/// Number of BPS query points. (constexpr at namespace scope has internal
/// linkage, so this is header-safe without C++17 inline variables.)
constexpr int kNumQueryPoints = 34;

/// Bodies the query points are attached to, in insertion order.
enum class QueryBody { Base, Thigh, Shank, Foot };

/// One query point: which body it rides on and where it sits in that body's frame.
struct QueryPointSpec {
  QueryBody body;
  int leg;  ///< -1 for the base, otherwise 0..3 for LF, LH, RF, RH
  Eigen::Vector3d local;
};

/// The 34 query points from the handover, in the exact order the observation
/// expects: base (6), then LF, LH, RF, RH each contributing THIGH (3),
/// SHANK (1) and FOOT (3).
const std::vector<QueryPointSpec>& queryPointSpecs();

/// Leg names in query order.
inline const std::array<const char*, 4>& queryLegNames() {
  static const std::array<const char*, 4> kLegs{"LF", "LH", "RF", "RH"};
  return kLegs;
}

}  // namespace dynasense
