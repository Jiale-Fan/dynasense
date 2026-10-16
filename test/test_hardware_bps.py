#!/usr/bin/env python3
"""dynasense_fk_ground_clearance_node and scripts/estimate_floor_z.py against static TF."""

import math
import subprocess
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

from dynasense.msg import GroundClearance

LEGS = ("LF", "LH", "RF", "RH")
# heights relative to base (base at z = 0.5 in odom)
OFFSETS = {"HIP": -0.05, "THIGH": -0.08, "SHANK": -0.25, "FOOT": -0.429}
BASE_Z = 0.5
BALL_R = 0.033
SOLE_BELOW_FOOT = BALL_R - BALL_R * math.cos(math.radians(15.0))  # upright foot


def transform(parent, child, xyz):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
    msg.transform.rotation.w = 1.0
    return msg


def tree(foot_override=None):
    transforms = [transform("odom", "base", (1.0, 2.0, BASE_Z))]
    for i, leg in enumerate(LEGS):
        for link, dz in OFFSETS.items():
            if link == "FOOT" and foot_override and leg in foot_override:
                dz = foot_override[leg]
            transforms.append(transform("base", "%s_%s" % (leg, link), (0.1 * i, 0.05 * i, dz)))
    return transforms


def expected(floor, max_range):
    values = [BASE_Z - floor]
    for _ in LEGS:
        for link in ("HIP", "THIGH", "SHANK", "FOOT"):
            values.append(BASE_Z + OFFSETS[link] - floor)
    return [min(max(v, 0.0), max_range) for v in values]


def estimate(*args):
    proc = subprocess.run(["rosrun", "dynasense", "estimate_floor_z.py", "--duration", "1.0"] + list(args),
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60, universal_newlines=True)
    return proc.returncode, proc.stdout.strip(), proc.stderr


class HardwareBpsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.broadcaster = tf2_ros.StaticTransformBroadcaster()
        cls.broadcaster.sendTransform(tree())
        cls.lock = threading.Lock()
        cls.msgs = {"fixed": [], "topic": []}
        for key in cls.msgs:
            rospy.Subscriber("/hw_test/clearance_" + key, GroundClearance,
                             lambda m, k=key: cls.store(k, m))

    @classmethod
    def store(cls, key, msg):
        with cls.lock:
            cls.msgs[key].append(msg)

    def wait_for(self, key, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.msgs[key]:
                    return self.msgs[key][-1]
            rospy.sleep(0.05)
        return None

    def test_1_fixed_floor(self):
        msg = self.wait_for("fixed")
        self.assertIsNotNone(msg, "no clearance message with a fixed floor")
        self.assertEqual(msg.header.frame_id, "odom")
        self.assertAlmostEqual(msg.max_range, 0.4, places=5)
        for got, want in zip(msg.clearance, expected(0.06, 0.4)):
            self.assertAlmostEqual(got, want, places=4)
        self.assertAlmostEqual(msg.clearance[0], 0.4, places=5)  # clipped

    def test_2_floor_from_topic(self):
        rospy.sleep(1.0)
        with self.lock:
            self.assertEqual(len(self.msgs["topic"]), 0, "must wait for the floor height")
        pub = rospy.Publisher("/hw_test/floor_z", Float64, queue_size=1, latch=True)
        pub.publish(Float64(0.0))
        msg = self.wait_for("topic")
        self.assertIsNotNone(msg, "no clearance message after the floor height arrived")
        for got, want in zip(msg.clearance, expected(0.0, 2.0)):
            self.assertAlmostEqual(got, want, places=4)

    def test_3_floor_estimate(self):
        rc, out, err = estimate()
        self.assertEqual(rc, 0, err)
        self.assertAlmostEqual(float(out), BASE_Z + OFFSETS["FOOT"] - SOLE_BELOW_FOOT, places=3)

    def test_4_floor_estimate_rejects_uneven_feet(self):
        self.broadcaster.sendTransform(tree({"LF": OFFSETS["FOOT"] + 0.1}))
        rospy.sleep(0.5)
        rc, out, err = estimate("--max-spread", "0.03")
        self.assertEqual(rc, 2, err)
        self.assertEqual(out, "")
        self.broadcaster.sendTransform(tree())


if __name__ == "__main__":
    rospy.init_node("test_hardware_bps")
    rostest.rosrun("dynasense", "hardware_bps", HardwareBpsTest)
