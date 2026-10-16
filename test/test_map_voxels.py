#!/usr/bin/env python3
"""dynasense_bps_node with ray_voxel_source: map_voxels, without Gazebo or wavemap."""

import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
import tf2_ros
from gazebo_msgs.msg import ContactsState, ContactState
from geometry_msgs.msg import TransformStamped, Vector3, Wrench
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from dynasense.msg import BpsState

VOXEL = 0.05
BASE = (5.0, -3.0, 0.5)  # odom, far from the origin


def transform(parent, child, xyz, yaw=0.0):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
    msg.transform.rotation.z = math.sin(yaw / 2.0)
    msg.transform.rotation.w = math.cos(yaw / 2.0)
    return msg


def centre(i, j, k):
    """Odom-frame centre of the voxel (i, j, k) relative to the base voxel."""
    return tuple((math.floor(b / VOXEL) + d + 0.5) * VOXEL for b, d in zip(BASE, (i, j, k)))


class MapVoxelsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.broadcaster = tf2_ros.StaticTransformBroadcaster()
        transforms = [transform("world", "odom", (1.0, 0.0, 0.0)), transform("odom", "base", BASE)]
        for leg in ("LF", "LH", "RF", "RH"):
            for body in ("THIGH", "SHANK", "FOOT"):
                transforms.append(transform("base", leg + "_" + body, (0.0, 0.0, 0.0)))
        cls.broadcaster.sendTransform(transforms)
        cls.lock = threading.Lock()
        cls.state = None
        cls.cubes = None
        cls.subs = [
            rospy.Subscriber("/map_voxel_test/state", BpsState, cls.on_state),
            rospy.Subscriber("/map_voxel_test/markers", MarkerArray, cls.on_markers),
        ]
        cls.voxels = rospy.Publisher("/map_voxel_test/voxels", PointCloud2, queue_size=1)
        cls.contacts = rospy.Publisher("/map_voxel_test/contacts/LF/thigh", ContactsState, queue_size=1)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and (cls.voxels.get_num_connections() == 0 or cls.state is None):
            rospy.sleep(0.05)

    @classmethod
    def on_state(cls, msg):
        with cls.lock:
            cls.state = msg

    @classmethod
    def on_markers(cls, msg):
        for m in msg.markers:
            if m.ns == "bps_activated_voxels":
                with cls.lock:
                    cls.cubes = [] if m.action == Marker.DELETE else [(p.x, p.y, p.z) for p in m.points]

    def publish_until(self, points, frame, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.voxels.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id=frame), points))
            rospy.sleep(0.1)
            with self.lock:
                if self.state is not None and predicate(self.state, self.cubes):
                    return True
        return False

    def assertCubes(self, expected):
        with self.lock:
            cubes = sorted(self.cubes or [])
        self.assertEqual(len(cubes), len(expected))
        for got, want in zip(cubes, sorted(expected)):
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b, places=5)

    def test_1_replacement_semantics(self):
        two = [centre(0, 0, 0), centre(2, 0, 0)]
        self.assertTrue(self.publish_until(two, "odom", lambda s, c: s.num_ray_voxels == 2 and c and len(c) == 2))
        self.assertCubes(two)

        one = [centre(0, 3, 1)]
        self.assertTrue(self.publish_until(one, "odom", lambda s, c: s.num_ray_voxels == 1 and c and len(c) == 1))
        self.assertCubes(one)

        five = [centre(i, 0, 0) for i in range(5)]  # nearest first; capacity 3
        self.assertTrue(self.publish_until(five, "odom", lambda s, c: s.num_ray_voxels == 3 and c and len(c) == 3))
        self.assertCubes(five[:3])

        # world -> odom is +1 m in x, so a world point maps to x - 1 in odom.
        target = centre(1, 1, 0)
        in_world = [(target[0] + 1.0, target[1], target[2])]
        self.assertTrue(self.publish_until(in_world, "world", lambda s, c: s.num_ray_voxels == 1 and c and len(c) == 1))
        self.assertCubes([target])

        self.assertTrue(self.publish_until([], "odom", lambda s, c: s.num_ray_voxels == 0))

    def test_2_sensor_topic_not_subscribed(self):
        _, subscribers, _ = rosgraph.Master("/test_map_voxels").getSystemState()
        subscribed = {topic for topic, _ in subscribers}
        self.assertIn("/map_voxel_test/voxels", subscribed)
        self.assertNotIn("/map_voxel_test/sensor_cloud", subscribed)

    def test_3_contacts_survive_and_stale_keeps(self):
        patch = ContactState()
        patch.collision1_name = "anymal::LF_THIGH::collision"
        patch.collision2_name = "terrain::link::collision"
        world = centre(-3, -3, 2)
        patch.contact_positions = [Vector3(world[0] + 1.0, world[1], world[2])]  # world coordinates
        wrench = Wrench()
        wrench.force.z = 10.0
        patch.total_wrench = wrench
        patch.wrenches = [wrench]
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self.contacts.publish(ContactsState(header=Header(stamp=rospy.Time.now(), frame_id="LF_THIGH"),
                                                states=[patch]))
            rospy.sleep(0.1)
            with self.lock:
                if self.state is not None and self.state.num_contact_voxels == 1:
                    break
        with self.lock:
            self.assertEqual(self.state.num_contact_voxels, 1)

        two = [centre(0, 0, 0), centre(0, 1, 0)]
        self.assertTrue(self.publish_until(two, "odom", lambda s, c: s.num_ray_voxels == 2))
        with self.lock:
            self.assertEqual(self.state.num_contact_voxels, 1)

        rospy.sleep(1.0)  # > map_voxel_timeout without a message: voxels are kept
        with self.lock:
            self.assertEqual(self.state.num_ray_voxels, 2)
            self.assertEqual(self.state.num_contact_voxels, 1)


if __name__ == "__main__":
    rospy.init_node("test_map_voxels")
    rostest.rosrun("dynasense", "map_voxels", MapVoxelsTest)
