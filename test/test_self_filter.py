#!/usr/bin/env python3
"""Checks for dynasense_self_filter_node on a synthetic robot (no bag, no Gazebo).

Covers: exact Euclidean margin to box, cylinder and sphere primitives (including
box corners, where a per-axis test would be wrong), range clipping, NaN removal,
arbitrary input layouts, output layout, header preservation, keep_organized,
per-entry and enabled_inputs sensor selection, and the body markers.
"""

import math
import struct
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

SENSOR_FRAME = "sensor"
SENSOR_Z = 1.0  # base -> sensor translation (0, 0, 1)
MARGIN = 0.05

# (point in the base frame, kept with max_range 2.0?, kept with max_range 0?, why)
NAN = float("nan")
CASES = [
    ((0.0, 0.0, 0.0), False, False, "inside the box"),
    ((0.13, 0.0, 0.0), False, False, "3 cm off the +x face"),
    ((0.17, 0.0, 0.0), True, True, "7 cm off the +x face"),
    ((0.125, 0.125, 0.125), False, False, "corner, 0.025*sqrt(3) = 4.3 cm"),
    ((0.15, 0.15, 0.15), True, True, "corner, 0.05*sqrt(3) = 8.7 cm (per-axis would drop)"),
    ((0.5, 0.0, 0.09), False, False, "4 cm above the sphere"),
    ((0.5, 0.0, 0.12), True, True, "7 cm above the sphere"),
    ((0.08, 0.5, 0.0), False, False, "cylinder, radial 3 cm"),
    ((0.0, 0.5, 0.18), False, False, "cylinder, axial 3 cm"),
    ((0.0, 0.5, 0.22), True, True, "cylinder, axial 7 cm"),
    ((0.10, 0.5, 0.20), True, True, "cylinder rim, hypot(5, 5) = 7.1 cm"),
    ((0.0, 0.0, 3.5), False, True, "range 2.5 m from the sensor"),
    ((1.0, 0.0, 1.5), True, True, "range 1.1 m, far from the body"),
    ((NAN, NAN, NAN), False, False, "NaN"),
]


def sensor_points():
    return [(x, y, z - SENSOR_Z) if not math.isnan(x) else (x, y, z) for (x, y, z), _, _, _ in CASES]


def expected(organized_no_clip):
    return [(k0 if organized_no_clip else k1) for _, k1, k0, _ in CASES]


def cloud_xyz(stamp):
    header = Header(stamp=stamp, frame_id=SENSOR_FRAME)
    return point_cloud2.create_cloud_xyz32(header, sensor_points())


def cloud_lidar_like(stamp):
    """x y z intensity ring layout (point_step 18) with the same coordinates."""
    header = Header(stamp=stamp, frame_id=SENSOR_FRAME)
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
        PointField("ring", 16, PointField.UINT16, 1),
    ]
    points = [(x, y, z, 10.0, i % 16) for i, (x, y, z) in enumerate(sensor_points())]
    return point_cloud2.create_cloud(header, fields, points)


def read_xyz(cloud):
    fmt = ">f" if cloud.is_bigendian else "<f"
    offsets = {f.name: f.offset for f in cloud.fields}
    out = []
    for j in range(cloud.height):
        for i in range(cloud.width):
            base = j * cloud.row_step + i * cloud.point_step
            out.append(tuple(struct.unpack_from(fmt, cloud.data, base + offsets[n])[0] for n in ("x", "y", "z")))
    return out


class SelfFilterTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.clouds = {}
        self.markers = []
        self.subs = [
            rospy.Subscriber(topic, PointCloud2, lambda msg, key=key: self._store(key, msg))
            for key, topic in (("a", "/sf_test/a_filtered"), ("c", "/sf_test/c_self_filtered"),
                               ("o_c", "/sf_test/o_c"), ("o_a", "/sf_test/o_a"))
        ]
        self.subs.append(rospy.Subscriber("/sf_list/body_markers", MarkerArray, self._store_markers))
        self.pub_a = rospy.Publisher("/sf_test/a", PointCloud2, queue_size=1)
        self.pub_c = rospy.Publisher("/sf_test/c", PointCloud2, queue_size=1)

    def _store(self, key, msg):
        with self.lock:
            self.clouds[key] = msg

    def _store_markers(self, msg):
        with self.lock:
            self.markers = msg.markers

    def _publish_until(self, publisher, make_cloud, keys, timeout=30.0):
        stamps = set()
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            stamps.add(stamp)
            publisher.publish(make_cloud(stamp))
            time.sleep(0.1)
            with self.lock:
                got = {key: self.clouds.get(key) for key in keys}
            if all(c is not None and c.header.stamp in stamps for c in got.values()):
                return got
        self.fail("no matching cloud within %.0f s for %s" % (timeout, keys))

    def assertKept(self, cloud, keep_flags):
        want = [p for p, k in zip(sensor_points(), keep_flags) if k]
        got = read_xyz(cloud)
        self.assertEqual(len(got), len(want), "kept %d points, expected %d: %s" % (len(got), len(want), got))
        for g, w in zip(got, want):
            for gv, wv in zip(g, w):
                self.assertAlmostEqual(gv, wv, places=5)

    def test_margin_range_and_layout(self):
        got = self._publish_until(self.pub_a, cloud_xyz, ("a",))
        a = got["a"]
        self.assertEqual(a.header.frame_id, SENSOR_FRAME)
        self.assertEqual(a.height, 1)
        self.assertEqual([f.name for f in a.fields], ["x", "y", "z"])
        self.assertEqual([f.offset for f in a.fields], [0, 4, 8])
        self.assertTrue(all(f.datatype == PointField.FLOAT32 for f in a.fields))
        self.assertEqual(a.point_step, 16)
        self.assertEqual(a.row_step, 16 * a.width)
        self.assertEqual(len(a.data), a.row_step)
        self.assertTrue(a.is_dense)
        self.assertKept(a, expected(False))

        # A lidar-like layout (x y z intensity ring) gives the same xyz-only output.
        got = self._publish_until(self.pub_a, cloud_lidar_like, ("a",))
        self.assertEqual(got["a"].point_step, 16)
        self.assertKept(got["a"], expected(False))

    def test_default_output_topic_and_override_instance(self):
        got = self._publish_until(self.pub_c, cloud_xyz, ("c", "o_c"))
        self.assertKept(got["c"], expected(False))  # sf_list: default suffix, range clip on

        o = got["o_c"]  # sf_override: keep_organized, max_range 0
        self.assertEqual((o.height, o.width), (1, len(CASES)))
        self.assertFalse(o.is_dense)
        pts = read_xyz(o)
        for (p, keep), name in zip(zip(sensor_points(), expected(True)), [c[3] for c in CASES]):
            g = pts[sensor_points().index(p)] if not math.isnan(p[0]) else pts[-1]
            if keep:
                for gv, wv in zip(g, p):
                    self.assertAlmostEqual(gv, wv, places=5, msg=name)
            else:
                self.assertTrue(all(math.isnan(v) for v in g), "%s should be NaN, got %s" % (name, g))

    def test_sensor_selection_via_master(self):
        deadline = time.monotonic() + 20.0
        master = rosgraph.Master(rospy.get_name())
        while time.monotonic() < deadline:
            pubs, subs, _ = master.getSystemState()
            subs = {topic: set(nodes) for topic, nodes in subs}
            pubs = {topic: set(nodes) for topic, nodes in pubs}
            if "/sf_list" in subs.get("/sf_test/a", set()) and "/sf_override" in subs.get("/sf_test/c", set()):
                break
            time.sleep(0.5)
        self.assertIn("/sf_list", subs.get("/sf_test/a", set()))
        self.assertIn("/sf_list", subs.get("/sf_test/c", set()))
        self.assertNotIn("/sf_list", subs.get("/sf_test/b", set()), "disabled input must not be subscribed")
        self.assertNotIn("/sf_list", pubs.get("/sf_test/b_self_filtered", set()))
        self.assertIn("/sf_override", subs.get("/sf_test/c", set()))
        self.assertNotIn("/sf_override", subs.get("/sf_test/a", set()), "enabled_inputs must replace the flags")
        self.assertNotIn("/sf_override", pubs.get("/sf_test/o_a", set()))

    def test_body_markers(self):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with self.lock:
                markers = list(self.markers)
            if len(markers) == 3:
                break
            time.sleep(0.2)
        self.assertEqual(len(markers), 3)
        by_type = {m.type: m for m in markers}
        self.assertEqual(sorted(by_type), [1, 2, 3])  # CUBE, SPHERE, CYLINDER
        cube, sphere, cyl = by_type[1], by_type[2], by_type[3]
        self.assertEqual(cube.header.frame_id, "base")
        self.assertAlmostEqual(cube.scale.x, 2 * (0.1 + MARGIN), places=6)
        self.assertEqual(sphere.header.frame_id, "ball")
        self.assertAlmostEqual(sphere.scale.x, 2 * (0.05 + MARGIN), places=6)
        self.assertAlmostEqual(cyl.scale.x, 2 * (0.05 + MARGIN), places=6)
        self.assertAlmostEqual(cyl.scale.z, 2 * (0.15 + MARGIN), places=6)
        self.assertAlmostEqual(cyl.pose.position.y, 0.5, places=6)
        self.assertTrue(all(m.frame_locked for m in markers))


if __name__ == "__main__":
    rospy.init_node("test_self_filter")
    rostest.rosrun("dynasense", "self_filter", SelfFilterTest)
