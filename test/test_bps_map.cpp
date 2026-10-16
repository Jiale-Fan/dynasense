// gtest for BpsMap::replaceRayVoxels (ROS-free).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "dynasense/BpsMap.hpp"

using dynasense::BpsMap;
using dynasense::BpsMapConfig;

namespace {

BpsMap makeMap(std::size_t rayCapacity = 3, bool rayEnabled = true) {
  BpsMapConfig config;
  config.voxelSize = 0.05;
  config.rayCapacity = rayCapacity;
  config.rayEnabled = rayEnabled;
  config.contactCapacity = 10;
  config.groundZ = 0.0;
  config.minHeightAboveGround = 0.05;
  config.maxHeightAboveGround = 1.8;
  config.maxHorizontalRange = 4.0;
  return BpsMap(config);
}

std::vector<Eigen::Vector3d> centres(const BpsMap& map) {
  std::vector<Eigen::Vector3d> out;
  map.bufferedVoxelCentresUnion(out);
  return out;
}

bool contains(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& p) {
  for (const auto& q : points) {
    if ((q - p).norm() < 1e-9) return true;
  }
  return false;
}

}  // namespace

TEST(BpsMapReplace, ReplacesPreviousContent) {
  BpsMap map = makeMap();
  map.replaceRayVoxels({{0.125, 0.125, 0.525}, {0.225, 0.125, 0.525}});
  EXPECT_EQ(map.numRayVoxels(), 2u);
  const auto result = map.replaceRayVoxels({{1.025, 0.025, 0.325}});
  EXPECT_EQ(result.accepted, 1u);
  EXPECT_EQ(map.numRayVoxels(), 1u);
  EXPECT_TRUE(contains(centres(map), {1.025, 0.025, 0.325}));
  EXPECT_FALSE(contains(centres(map), {0.125, 0.125, 0.525}));
}

TEST(BpsMapReplace, EmptyInputClears) {
  BpsMap map = makeMap();
  map.replaceRayVoxels({{0.125, 0.125, 0.525}});
  map.replaceRayVoxels({});
  EXPECT_EQ(map.numRayVoxels(), 0u);
}

TEST(BpsMapReplace, CapacityKeepsFirstInInputOrder) {
  BpsMap map = makeMap(3);
  const std::vector<Eigen::Vector3d> points{
      {0.025, 0.025, 0.525}, {0.075, 0.025, 0.525}, {0.125, 0.025, 0.525}, {0.175, 0.025, 0.525}, {0.225, 0.025, 0.525}};
  const auto result = map.replaceRayVoxels(points);
  EXPECT_EQ(result.accepted, 3u);
  EXPECT_EQ(result.truncated, 2u);
  EXPECT_EQ(map.numRayVoxels(), 3u);
  const auto c = centres(map);
  EXPECT_TRUE(contains(c, points[0]));
  EXPECT_TRUE(contains(c, points[1]));
  EXPECT_TRUE(contains(c, points[2]));
  EXPECT_FALSE(contains(c, points[3]));
  EXPECT_FALSE(contains(c, points[4]));
}

TEST(BpsMapReplace, PointsInOneVoxelCountOnce) {
  BpsMap map = makeMap(3);
  const auto result = map.replaceRayVoxels({{0.01, 0.01, 0.51}, {0.04, 0.04, 0.54}, {0.125, 0.025, 0.525}});
  EXPECT_EQ(result.accepted, 3u);
  EXPECT_EQ(map.numRayVoxels(), 2u);
}

TEST(BpsMapReplace, GatesOptional) {
  BpsMap map = makeMap();
  // Voxel centre z 0.025 is below groundZ + 0.05; 5 m is beyond the 4 m range.
  const std::vector<Eigen::Vector3d> points{{0.025, 0.025, 0.02}, {5.0, 0.0, 0.5}};
  auto gated = map.replaceRayVoxels(points, true);
  EXPECT_EQ(gated.rejected, 2u);
  EXPECT_EQ(map.numRayVoxels(), 0u);
  auto ungated = map.replaceRayVoxels(points, false);
  EXPECT_EQ(ungated.rejected, 0u);
  EXPECT_EQ(map.numRayVoxels(), 2u);
  EXPECT_FALSE(map.passesGates(points[0]));
  EXPECT_TRUE(map.passesGates({0.5, 0.0, 0.5}));
}

TEST(BpsMapReplace, NonFiniteAlwaysRejected) {
  BpsMap map = makeMap();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto result = map.replaceRayVoxels({{nan, 0.0, 0.5}, {0.125, 0.025, 0.525}}, false);
  EXPECT_EQ(result.rejected, 1u);
  EXPECT_EQ(map.numRayVoxels(), 1u);
}

TEST(BpsMapReplace, ContactsUntouched) {
  BpsMap map = makeMap();
  ASSERT_TRUE(map.addContactPoint({0.3, 0.3, 0.6}));
  map.replaceRayVoxels({{0.125, 0.025, 0.525}});
  map.replaceRayVoxels({});
  EXPECT_EQ(map.numContactVoxels(), 1u);
  EXPECT_EQ(map.numRayVoxels(), 0u);
}

TEST(BpsMapReplace, DisabledRayPathStaysEmpty) {
  BpsMap map = makeMap(3, false);
  const auto result = map.replaceRayVoxels({{0.125, 0.025, 0.525}});
  EXPECT_EQ(result.accepted, 0u);
  EXPECT_EQ(result.truncated, 1u);
  EXPECT_EQ(map.numRayVoxels(), 0u);
}

TEST(BpsMapReplace, QuerySeesReplacedVoxels) {
  BpsMap map = makeMap();
  map.replaceRayVoxels({{0.025, 0.025, 0.525}});
  // Query 0.2 m beyond the cube's +x face: distance 0.2, direction +x.
  const auto result = map.query({0.05 + 0.2, 0.025, 0.525});
  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.distance, 0.2, 1e-9);
  EXPECT_GT(result.directionWorld.x(), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
