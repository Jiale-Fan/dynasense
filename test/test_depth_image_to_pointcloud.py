#!/usr/bin/env python3
"""Synthetic-image checks for dynasense_depth_image_to_pointcloud_node (no bag, no Gazebo).

Covers: organized layout and header, row padding (step > width * 2), the
pinhole math, NaN for no-return / out-of-range / NaN pixels, 32FC1 input,
decimation, and adoption of a live camera_info.
"""

import math
import struct
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

FRAME = "depth2pc_test_optical_frame"
IMAGE_TOPIC = "/depth2pc_test/full/depth/image_rect_raw"
W, H = 8, 4
FX = FY = 100.0
CX, CY = 3.5, 1.5


def image_16uc1(stamp, step=20):
    """8x4 16UC1 with row padding (step 20 > 16 bytes). All 500 mm except
    (u=5, v=2) = 1000 mm, (0, 0) = 0 (no return), (7, 3) = 5000 mm (> depth_max)."""
    values = [[500] * W for _ in range(H)]
    values[2][5] = 1000
    values[0][0] = 0
    values[3][7] = 5000
    rows = []
    for row in values:
        packed = struct.pack("<%dH" % W, *row)
        rows.append(packed + b"\0" * (step - len(packed)))
    return _image(stamp, "16UC1", step, b"".join(rows))


def image_32fc1(stamp):
    """8x4 32FC1 metres: all 0.5 m except (5, 2) = 1.0 m and (2, 1) = NaN."""
    values = [[0.5] * W for _ in range(H)]
    values[2][5] = 1.0
    values[1][2] = float("nan")
    data = b"".join(struct.pack("<%df" % W, *row) for row in values)
    return _image(stamp, "32FC1", W * 4, data)


def _image(stamp, encoding, step, data):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME
    msg.height = H
    msg.width = W
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = step
    msg.data = data
    return msg


def camera_info(fx, fy, cx, cy):
    info = CameraInfo()
    info.header.frame_id = FRAME
    info.width, info.height = W, H
    info.distortion_model = "plumb_bob"
    info.D = [0.0] * 5
    info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def xyz(cloud, i, j):
    """Point (column i, row j) of an organized xyz float32 cloud."""
    offsets = {f.name: f.offset for f in cloud.fields}
    fmt = ">f" if cloud.is_bigendian else "<f"
    base = j * cloud.row_step + i * cloud.point_step
    return tuple(struct.unpack_from(fmt, cloud.data, base + offsets[name])[0] for name in ("x", "y", "z"))


class DepthImageToPointCloudTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.clouds = {}
        self.subscribers = [
            rospy.Subscriber("/depth2pc_test/%s/depth/points" % key, PointCloud2,
                             lambda msg, key=key: self._store(key, msg))
            for key in ("full", "dec", "live")
        ]
        self.images = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1)
        self.infos = rospy.Publisher("/depth2pc_test/live/depth/camera_info", CameraInfo, queue_size=1, latch=True)

    def _store(self, key, msg):
        with self.lock:
            self.clouds[key] = msg

    def _publish_until(self, make_image, keys, accept=None, timeout=25.0):
        """Publishes images until every key has a cloud for one of them (and
        `accept` holds); returns the clouds keyed by name."""
        stamps = set()
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            stamps.add(stamp)
            self.images.publish(make_image(stamp))
            time.sleep(0.1)
            with self.lock:
                got = {key: self.clouds.get(key) for key in keys}
            if all(cloud is not None and cloud.header.stamp in stamps for cloud in got.values()):
                if accept is None or accept(got):
                    return got
        self.fail("no matching cloud within %.0f s for %s" % (timeout, keys))

    def assertPoint(self, cloud, i, j, expected):
        x, y, z = xyz(cloud, i, j)
        for value, want, name in ((x, expected[0], "x"), (y, expected[1], "y"), (z, expected[2], "z")):
            self.assertAlmostEqual(value, want, places=5, msg="point (%d,%d) %s" % (i, j, name))

    def assertNaN(self, cloud, i, j):
        self.assertTrue(all(math.isnan(v) for v in xyz(cloud, i, j)), "point (%d,%d) should be NaN" % (i, j))

    def test_16uc1_layout_geometry_and_decimation(self):
        got = self._publish_until(image_16uc1, ("full", "dec"))
        full, dec = got["full"], got["dec"]

        self.assertEqual(full.header.frame_id, FRAME)
        self.assertEqual((full.height, full.width), (H, W))
        self.assertEqual(full.point_step, 16)
        self.assertEqual(full.row_step, 16 * W)
        self.assertFalse(full.is_dense)
        self.assertEqual([f.name for f in full.fields], ["x", "y", "z"])
        self.assertEqual([f.offset for f in full.fields], [0, 4, 8])
        self.assertTrue(all(f.datatype == 7 for f in full.fields))  # FLOAT32
        self.assertEqual(len(full.data), full.row_step * full.height)

        # z = 1.0 m at (5, 2): x = (5 - 3.5) / 100, y = (2 - 1.5) / 100
        self.assertPoint(full, 5, 2, (0.015, 0.005, 1.0))
        # z = 0.5 m at (1, 0): padding bytes must not shift the rows
        self.assertPoint(full, 1, 0, (-0.0125, -0.0075, 0.5))
        self.assertPoint(full, 7, 2, (0.0175, 0.0025, 0.5))
        self.assertNaN(full, 0, 0)  # 0 = no return
        self.assertNaN(full, 7, 3)  # 5 m > depth_max 3 m

        # decimation 2: ceil(8/2) x ceil(4/2), point (i, j) is source pixel (2i, 2j)
        self.assertEqual((dec.height, dec.width), (2, 4))
        self.assertEqual(dec.header.stamp, full.header.stamp)
        self.assertPoint(dec, 2, 1, (0.0025, 0.0025, 0.5))  # source (4, 2)
        self.assertNaN(dec, 0, 0)  # source (0, 0)

    def test_32fc1_metres_and_nan(self):
        full = self._publish_until(image_32fc1, ("full",))["full"]
        self.assertEqual((full.height, full.width), (H, W))
        self.assertPoint(full, 5, 2, (0.015, 0.005, 1.0))
        self.assertPoint(full, 0, 0, (-0.0175, -0.0075, 0.5))
        self.assertNaN(full, 2, 1)

    def test_live_camera_info_overrides_config(self):
        # Before the override the live instance uses fx = 100 like the others.
        before = self._publish_until(image_16uc1, ("live",))["live"]
        self.assertPoint(before, 5, 2, (0.015, 0.005, 1.0))

        self.infos.publish(camera_info(50.0, 50.0, CX, CY))

        def overridden(got):
            return abs(xyz(got["live"], 5, 2)[0] - 0.03) < 1e-5

        after = self._publish_until(image_16uc1, ("live",), accept=overridden)["live"]
        self.assertPoint(after, 5, 2, (0.03, 0.01, 1.0))  # (5 - 3.5) / 50, (2 - 1.5) / 50


if __name__ == "__main__":
    rospy.init_node("test_depth_image_to_pointcloud")
    rostest.rosrun("dynasense", "depth_image_to_pointcloud", DepthImageToPointCloudTest)
