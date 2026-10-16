// gtest for dynasense::extractOccupiedVoxels (wavemap core, no ROS graph).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_voxel_extraction.hpp"

using dynasense::ExtractedVoxel;
using dynasense::ExtractionStats;
using dynasense::VoxelExtractionParams;
using dynasense::extractOccupiedVoxels;

namespace {

constexpr float kW = 0.05f;

std::shared_ptr<wavemap::HashedWaveletOctree> makeMap() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = kW;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 6;
  return std::make_shared<wavemap::HashedWaveletOctree>(config);
}

VoxelExtractionParams params(double radius, double zMin, double zMax, std::size_t maxVoxels = 0) {
  VoxelExtractionParams p;
  p.radius = radius;
  p.zMin = zMin;
  p.zMax = zMax;
  p.occupancyThreshold = 0.0f;
  p.maxVoxels = maxVoxels;
  return p;
}

double centre(int k) { return (k + 0.5) * static_cast<double>(kW); }

}  // namespace

TEST(WavemapVoxelExtraction, EmptyMap) {
  auto map = makeMap();
  std::vector<ExtractedVoxel> out;
  ExtractionStats stats;
  extractOccupiedVoxels(*map, {0, 0, 0}, params(1.0, -1.0, 1.0), out, &stats);
  EXPECT_TRUE(out.empty());
  EXPECT_GT(stats.cellsVisited, 0u);
}

TEST(WavemapVoxelExtraction, CellCentreAndValue) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{3, -2, 4}, 2.f);
  std::vector<ExtractedVoxel> out;
  extractOccupiedVoxels(*map, {0, 0, 0}, params(1.0, -1.0, 1.0), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].centre.x(), centre(3), 1e-6);
  EXPECT_NEAR(out[0].centre.y(), centre(-2), 1e-6);
  EXPECT_NEAR(out[0].centre.z(), centre(4), 1e-6);
  EXPECT_NEAR(out[0].logOdds, 2.f, 1e-3);
}

TEST(WavemapVoxelExtraction, RadiusIsHorizontalOnCentres) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{18, 0, 30}, 2.f);  // x centre 0.925, z 1.525: inside the 1 m circle
  map->setCellValue(wavemap::Index3D{20, 0, 0}, 2.f);   // x centre 1.025: outside
  std::vector<ExtractedVoxel> out;
  extractOccupiedVoxels(*map, {0, 0.025, 0}, params(1.0, -2.0, 2.0), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].centre.x(), centre(18), 1e-6);
}

TEST(WavemapVoxelExtraction, ZBandHalfOpenOnCentres) {
  auto map = makeMap();
  for (int k = 0; k < 8; ++k) map->setCellValue(wavemap::Index3D{0, 0, k}, 2.f);  // centres 0.025 .. 0.375
  std::vector<ExtractedVoxel> out;
  // Band [0.04, 0.26) judged on CENTRES: cell 0 ([0, 0.05), centre 0.025) and cell 5
  // ([0.25, 0.3), centre 0.275) overlap the band but their centres lie outside.
  extractOccupiedVoxels(*map, {0, 0, 0}, params(0.5, 0.04, 0.26), out);
  std::set<int> ks;
  for (const auto& v : out) ks.insert(static_cast<int>(std::lround(v.centre.z() / kW - 0.5)));
  EXPECT_EQ(ks, (std::set<int>{1, 2, 3, 4}));
}

TEST(WavemapVoxelExtraction, ThresholdFreeAndUnobserved) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{0, 0, 1}, 0.3f);
  map->setCellValue(wavemap::Index3D{1, 0, 1}, -1.f);
  map->setCellValue(wavemap::Index3D{2, 0, 1}, 2.f);
  std::vector<ExtractedVoxel> out;
  auto p = params(0.5, -1.0, 1.0);
  p.occupancyThreshold = 0.5f;
  extractOccupiedVoxels(*map, {0, 0, 0}, p, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].logOdds, 2.f, 1e-3);
  // A negative threshold keeps free cells, never unobserved ones.
  p.occupancyThreshold = -1.5f;
  extractOccupiedVoxels(*map, {0, 0, 0}, p, out);
  EXPECT_EQ(out.size(), 3u);
}

TEST(WavemapVoxelExtraction, UniformCoarseBlockMatchesBruteForce) {
  auto map = makeMap();
  map->getOrAllocateBlock(wavemap::Index3D{0, 0, 0}).getRootScale() = 2.f;  // whole block [0, 3.2)^3 occupied
  const Eigen::Vector3d base(0.2, 1.6, 0.5);
  std::vector<ExtractedVoxel> out;
  extractOccupiedVoxels(*map, base, params(0.5, 0.05, 0.3), out);
  std::size_t expected = 0;
  for (int ix = 0; ix < 64; ++ix)
    for (int iy = 0; iy < 64; ++iy)
      for (int iz = 0; iz < 64; ++iz) {
        const double dx = centre(ix) - base.x();
        const double dy = centre(iy) - base.y();
        const double z = centre(iz);
        if (dx * dx + dy * dy <= 0.25 && z >= 0.05 && z < 0.3) ++expected;
      }
  EXPECT_GT(expected, 0u);
  EXPECT_EQ(out.size(), expected);
}

TEST(WavemapVoxelExtraction, NearestFirstAndCap) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{10, 0, 2}, 2.f);
  map->setCellValue(wavemap::Index3D{2, 0, 2}, 2.f);
  map->setCellValue(wavemap::Index3D{5, 0, 2}, 2.f);
  std::vector<ExtractedVoxel> out;
  ExtractionStats stats;
  extractOccupiedVoxels(*map, {0, 0.025, 0}, params(1.0, 0.0, 1.0, 2), out, &stats);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].centre.x(), centre(2), 1e-6);
  EXPECT_NEAR(out[1].centre.x(), centre(5), 1e-6);
  EXPECT_EQ(stats.occupied, 3u);
  EXPECT_EQ(stats.truncated, 1u);
}

TEST(WavemapVoxelExtraction, NegativeCoordinates) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{-1, -1, -1}, 2.f);  // centre (-0.025, -0.025, -0.025)
  std::vector<ExtractedVoxel> out;
  extractOccupiedVoxels(*map, {-0.1, -0.1, 0}, params(0.3, -0.05, 0.0), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].centre.x(), -0.025, 1e-6);
  EXPECT_NEAR(out[0].centre.z(), -0.025, 1e-6);
}

TEST(WavemapVoxelExtraction, RosMessageRoundTrip) {
  auto map = makeMap();
  map->setCellValue(wavemap::Index3D{3, 4, 5}, 2.f);
  map->setCellValue(wavemap::Index3D{-7, 2, 3}, 3.f);
  map->threshold();
  wavemap_msgs::Map msg;
  ASSERT_TRUE(wavemap::convert::mapToRosMsg(*map, "odom", ros::Time(0), msg));
  wavemap::MapBase::Ptr received;
  ASSERT_TRUE(wavemap::convert::rosMsgToMap(msg, received));
  auto* octree = dynamic_cast<wavemap::HashedWaveletOctree*>(received.get());
  ASSERT_NE(octree, nullptr);
  std::vector<ExtractedVoxel> a, b;
  extractOccupiedVoxels(*map, {0, 0, 0}, params(1.0, -1.0, 1.0), a);
  extractOccupiedVoxels(*octree, {0, 0, 0}, params(1.0, -1.0, 1.0), b);
  ASSERT_EQ(a.size(), 2u);
  ASSERT_EQ(b.size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_NEAR((a[i].centre - b[i].centre).norm(), 0.f, 1e-6);
    EXPECT_NEAR(a[i].logOdds, b[i].logOdds, 1e-2);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
