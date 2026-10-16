#!/usr/bin/env python3
"""dynasense_wavemap_bps_bridge: full-map request, extraction geometry, block removal."""

import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from wavemap_msgs.msg import HashedWaveletOctree, HashedWaveletOctreeBlock, Index3D, Map

W = 0.05
BASE = (0.2, 1.6, 0.5)
RADIUS, Z_MIN, Z_MAX = 0.5, 0.05, 0.3


def uniform_block(index, value):
    return HashedWaveletOctreeBlock(root_node_offset=Index3D(*index), root_node_scale_coefficient=value, nodes=[])


def make_map(allocated, blocks):
    octree = HashedWaveletOctree(min_cell_width=W, min_log_odds=-2.0, max_log_odds=4.0, tree_height=6,
                                 allocated_block_indices=[Index3D(*b) for b in allocated], blocks=blocks)
    msg = Map(hashed_wavelet_octree=[octree])
    msg.header.frame_id = "odom"
    msg.header.stamp = rospy.Time.now()
    return msg


def brute_force():
    """Cells of block (0,0,0) (cells 0..63 per axis, occupied) inside the cylinder and band."""
    out = []
    for ix in range(64):
        for iy in range(64):
            for iz in range(64):
                x, y, z = (ix + 0.5) * W, (iy + 0.5) * W, (iz + 0.5) * W
                d2 = (x - BASE[0]) ** 2 + (y - BASE[1]) ** 2
                if d2 <= RADIUS ** 2 and Z_MIN <= z < Z_MAX:
                    out.append((x, y, z))
    return out


class BridgeTest(unittest.TestCase):
    def test_bridge(self):
        lock = threading.Lock()
        requests = []
        clouds = []
        floor_clouds = []

        def on_request(_req):
            with lock:
                requests.append(rospy.Time.now())
            return EmptyResponse()

        def on_cloud(msg):
            with lock:
                clouds.append(msg)

        rospy.Service("/bridge_test/map_request_full", Empty, on_request)
        pub = rospy.Publisher("/bridge_test/map", Map, queue_size=1)
        rospy.Subscriber("/bridge_test/voxels", PointCloud2, on_cloud)

        def on_floor_cloud(msg):
            with lock:
                floor_clouds.append(msg)

        rospy.Subscriber("/bridge_test/voxels_floor", PointCloud2, on_floor_cloud)

        # Block (0,0,0) occupied (log-odds 2), block (-1,0,0) free (log-odds -1).
        full = make_map([(0, 0, 0), (-1, 0, 0)], [uniform_block((0, 0, 0), 2.0), uniform_block((-1, 0, 0), -1.0)])
        deadline = time.monotonic() + 30.0
        got = None
        while time.monotonic() < deadline and got is None:
            with lock:
                requested = bool(requests)
            if requested:
                pub.publish(full)
            rospy.sleep(0.2)
            with lock:
                got = next((c for c in reversed(clouds) if c.width > 0), None)
        self.assertIsNotNone(got, "no non-empty voxel cloud from the bridge")
        with lock:
            # one request per bridge (absolute band and floor-topic instances)
            self.assertEqual(len(requests), 2, "each bridge must request the full map exactly once")
            self.assertEqual(len(floor_clouds), 0, "the floor-relative bridge must wait for the floor height")
        self.assertEqual(got.header.frame_id, "odom")
        self.assertEqual([f.name for f in got.fields], ["x", "y", "z", "intensity"])

        points = list(point_cloud2.read_points(got, field_names=("x", "y", "z", "intensity")))
        expected = brute_force()
        self.assertEqual(len(points), len(expected))
        want = {tuple(round(v / W - 0.5) for v in p) for p in expected}
        have = {tuple(round(v / W - 0.5) for v in p[:3]) for p in points}
        self.assertEqual(have, want)
        for p in points:
            for v in p[:3]:  # every coordinate is a cell centre, (k + 0.5) * W
                k = v / W - 0.5
                self.assertAlmostEqual(k, round(k), places=3)
            self.assertAlmostEqual(p[3], 2.0, places=2)
        dists = [(p[0] - BASE[0]) ** 2 + (p[1] - BASE[1]) ** 2 for p in points]
        # Nearest first; equal distances may differ by float32 rounding of the coordinates.
        self.assertTrue(all(b >= a - 1e-6 for a, b in zip(dists, dists[1:])), "voxels must be ordered nearest first")

        # Floor-relative band: floor 0.0 gives the same band [0.05, 0.3) as the absolute bridge.
        floor_pub = rospy.Publisher("/bridge_test/floor_z", Float64, queue_size=1, latch=True)
        floor_pub.publish(Float64(0.0))
        deadline = time.monotonic() + 15.0
        floor_got = None
        while time.monotonic() < deadline and floor_got is None:
            pub.publish(full)
            rospy.sleep(0.2)
            with lock:
                floor_got = next((c for c in reversed(floor_clouds) if c.width > 0), None)
        self.assertIsNotNone(floor_got, "no voxels from the floor-relative bridge after the floor height")
        floor_points = list(point_cloud2.read_points(floor_got, field_names=("x", "y", "z")))
        self.assertEqual({tuple(round(v / W - 0.5) for v in p) for p in floor_points}, want)

        # The server drops block (0,0,0): the bridge must erase it and publish an empty cloud.
        removal = make_map([(-1, 0, 0)], [])
        deadline = time.monotonic() + 15.0
        empty = False
        with lock:
            clouds.clear()
        while time.monotonic() < deadline and not empty:
            pub.publish(removal)
            rospy.sleep(0.2)
            with lock:
                empty = any(c.width == 0 for c in clouds)
        self.assertTrue(empty, "after the block removal the bridge must publish an empty cloud")


if __name__ == "__main__":
    rospy.init_node("test_wavemap_bps_bridge")
    rostest.rosrun("dynasense", "wavemap_bps_bridge", BridgeTest)
