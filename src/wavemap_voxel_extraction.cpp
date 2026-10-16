#include "wavemap_voxel_extraction.hpp"

#include <algorithm>
#include <cmath>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/utils/query/occupancy_classifier.h>
#include <wavemap/core/utils/query/query_accelerator.h>

namespace dynasense {

void extractOccupiedVoxels(const wavemap::HashedWaveletOctree& map, const Eigen::Vector3d& centre,
                           const VoxelExtractionParams& params, std::vector<ExtractedVoxel>& out,
                           ExtractionStats* stats) {
  out.clear();
  ExtractionStats local;
  const double w = static_cast<double>(map.getMinCellWidth());
  const double radius = params.radius;
  if (!(w > 0.0) || !(radius > 0.0) || !(params.zMax > params.zMin) || !centre.allFinite()) {
    if (stats) *stats = local;
    return;
  }
  const double radiusSq = radius * radius;

  // Cell k has its centre at (k + 0.5) w, so "centre >= lo" is k >= ceil(lo / w - 0.5),
  // "centre <= hi" is k <= floor(hi / w - 0.5) and "centre < hi" is k <= ceil(hi / w - 0.5) - 1.
  const auto firstAtLeast = [w](double lo) { return static_cast<int>(std::ceil(lo / w - 0.5)); };
  const auto lastAtMost = [w](double hi) { return static_cast<int>(std::floor(hi / w - 0.5)); };
  const auto lastBelow = [w](double hi) { return static_cast<int>(std::ceil(hi / w - 0.5)) - 1; };

  const int zFirst = firstAtLeast(params.zMin);
  const int zLast = lastBelow(params.zMax);
  const int yFirst = firstAtLeast(centre.y() - radius);
  const int yLast = lastAtMost(centre.y() + radius);

  wavemap::QueryAccelerator<wavemap::HashedWaveletOctree> accelerator(map);
  for (int iz = zFirst; iz <= zLast; ++iz) {
    for (int iy = yFirst; iy <= yLast; ++iy) {
      const double dy = (static_cast<double>(iy) + 0.5) * w - centre.y();
      const double rest = radiusSq - dy * dy;
      if (rest < 0.0) {
        continue;
      }
      const double halfWidth = std::sqrt(rest);
      const int xFirst = firstAtLeast(centre.x() - halfWidth);
      const int xLast = lastAtMost(centre.x() + halfWidth);
      for (int ix = xFirst; ix <= xLast; ++ix) {
        ++local.cellsVisited;
        const double dx = (static_cast<double>(ix) + 0.5) * w - centre.x();
        const double distSq = dx * dx + dy * dy;
        if (distSq > radiusSq) {
          continue;
        }
        const wavemap::Index3D index{ix, iy, iz};
        const wavemap::FloatingPoint value = accelerator.getCellValue(index);
        if (wavemap::OccupancyClassifier::isUnobserved(value) || !(value > params.occupancyThreshold)) {
          continue;
        }
        ExtractedVoxel voxel;
        voxel.centre = wavemap::convert::indexToCenterPoint(index, static_cast<wavemap::FloatingPoint>(w));
        voxel.logOdds = value;
        voxel.xyDistanceSq = static_cast<float>(distSq);
        out.push_back(voxel);
      }
    }
  }

  local.occupied = out.size();
  const auto nearer = [](const ExtractedVoxel& a, const ExtractedVoxel& b) { return a.xyDistanceSq < b.xyDistanceSq; };
  if (params.maxVoxels > 0 && out.size() > params.maxVoxels) {
    std::partial_sort(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(params.maxVoxels), out.end(), nearer);
    local.truncated = out.size() - params.maxVoxels;
    out.resize(params.maxVoxels);
  } else {
    std::sort(out.begin(), out.end(), nearer);
  }
  if (stats) *stats = local;
}

}  // namespace dynasense
