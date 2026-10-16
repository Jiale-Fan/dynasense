#pragma once

// Occupied-cell extraction from a wavemap HashedWaveletOctree around a point.
// ROS-free; built only when wavemap is available (C++17, see CMakeLists.txt).

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <wavemap/core/map/hashed_wavelet_octree.h>

namespace dynasense {

struct VoxelExtractionParams {
  double radius{1.0};              ///< [m] horizontal (xy) radius around the centre, judged on cell centres
  double zMin{0.05};               ///< [m] absolute z band [zMin, zMax) in the map frame, on cell centres
  double zMax{1.8};
  float occupancyThreshold{0.0f};  ///< keep cells with log-odds strictly above this (unobserved never kept)
  std::size_t maxVoxels{2000};     ///< keep the nearest (xy) ones; 0 = no cap
};

struct ExtractedVoxel {
  Eigen::Vector3f centre;  ///< cell centre in the map frame, (k + 0.5) * min_cell_width
  float logOdds{0.0f};
  float xyDistanceSq{0.0f};
};

struct ExtractionStats {
  std::size_t cellsVisited{0};
  std::size_t occupied{0};   ///< occupied cells found before the cap
  std::size_t truncated{0};  ///< dropped by maxVoxels
};

/// Fills `out` with the occupied min_cell_width cells whose centres lie within
/// `params.radius` of `centre` in xy and inside [zMin, zMax), nearest first.
/// Coarse (saturated) octree leaves are resolved per cell by the query
/// accelerator, so the result is always at min_cell_width.
void extractOccupiedVoxels(const wavemap::HashedWaveletOctree& map, const Eigen::Vector3d& centre,
                           const VoxelExtractionParams& params, std::vector<ExtractedVoxel>& out,
                           ExtractionStats* stats = nullptr);

}  // namespace dynasense
