#!/usr/bin/env python3
"""Exercise the production node across a nonidentity world/odom TF, without Gazebo."""

import math
import threading
import time
import unittest

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


def transform(parent, child, xyz, yaw=0.0):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
    msg.transform.rotation.z = math.sin(yaw / 2.0)
    msg.transform.rotation.w = math.cos(yaw / 2.0)
    return msg


class FrameAlignmentTest(unittest.TestCase):
    def test_contact_cloud_and_query_frames(self):
        # world <- odom is translated and rotated; base <- camera is identity.
        # base origin: world (1.5, -0.5, 0.7), odom (0.5, 0.5, 0.4).
        # Camera hit (0.37, 0.13, 0.22) corresponds to world (1.37, -0.13, 0.92).
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transforms = [
            transform("world", "odom", (2.0, -1.0, 0.3), math.pi / 2.0),
            transform("odom", "base", (0.5, 0.5, 0.4)),
            transform("base", "frame_test_camera", (0.0, 0.0, 0.0)),
        ]
        for leg in ("LF", "LH", "RF", "RH"):
            for body in ("THIGH", "SHANK", "FOOT"):
                transforms.append(transform("base", leg + "_" + body, (0.0, 0.0, 0.0)))
        broadcaster.sendTransform(transforms)

        states, markers = {}, {}
        lock = threading.Lock()

        def save(target, frame, msg):
            with lock:
                target[frame] = msg

        subscribers = []
        for frame in ("world", "odom"):
            subscribers.append(rospy.Subscriber(
                "/frame_test/" + frame + "/state", BpsState,
                lambda msg, frame=frame: save(states, frame, msg)))
            subscribers.append(rospy.Subscriber(
                "/frame_test/" + frame + "/markers", MarkerArray,
                lambda msg, frame=frame: save(markers, frame, msg)))
        contacts = rospy.Publisher("/frame_test/contacts/LF/thigh", ContactsState, queue_size=1)
        clouds = rospy.Publisher("/frame_test/cloud", PointCloud2, queue_size=1)

        patch = ContactState()
        patch.collision1_name = "anymal::LF_THIGH::collision"
        patch.collision2_name = "terrain::link::collision"
        patch.contact_positions = [Vector3(1.37, -0.13, 0.92)]
        wrench = Wrench()
        wrench.force.z = 10.0
        patch.total_wrench = wrench
        patch.wrenches = [wrench]

        deadline = time.monotonic() + 25.0
        ready = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            # Deliberately misleading header, as emitted by stock gazebo_ros_bumper.
            contacts.publish(ContactsState(header=Header(stamp=stamp, frame_id="LF_THIGH"), states=[patch]))
            clouds.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id="frame_test_camera"), [(0.37, 0.13, 0.22)]))
            with lock:
                ready = all(
                    frame in states and frame in markers
                    and states[frame].num_contact_voxels == 1
                    and states[frame].num_ray_voxels == 1
                    and any(m.ns == "bps_activated_voxels" and m.action == Marker.ADD
                            and len(m.points) == 1 for m in markers[frame].markers)
                    for frame in ("world", "odom"))
            if ready:
                break
            rospy.sleep(0.05)
        self.assertTrue(ready, "Both map frames must merge contact and cloud into the same single voxel")

        expected_centres = {"world": (1.375, -0.125, 0.925), "odom": (0.875, 0.625, 0.625)}
        # Query 2 is the base-local origin (see queryPointSpecs()).
        expected_queries = {"world": (1.5, -0.5, 0.7), "odom": (0.5, 0.5, 0.4)}
        with lock:
            for frame in ("world", "odom"):
                state = states[frame]
                self.assertEqual(state.header.frame_id, frame)
                query = state.query_points[2]
                for actual, expected in zip((query.x, query.y, query.z), expected_queries[frame]):
                    self.assertAlmostEqual(actual, expected, places=6)
                cube = next(m for m in markers[frame].markers if m.ns == "bps_activated_voxels")
                self.assertEqual(cube.header.frame_id, frame)
                self.assertEqual(len(cube.points), 1)
                point = cube.points[0]
                for actual, expected in zip((point.x, point.y, point.z), expected_centres[frame]):
                    self.assertAlmostEqual(actual, expected, places=6)
                self.assertTrue(all(m.header.frame_id == frame for m in markers[frame].markers))
            # A 90-degree frame rotation must not rotate the policy's base-frame encoding.
            for actual, expected in zip(states["world"].directions, states["odom"].directions):
                self.assertAlmostEqual(actual, expected, places=5)
            self.assertGreater(sum(abs(v) for v in states["world"].directions), 0.0)


if __name__ == "__main__":
    rospy.init_node("test_frame_alignment")
    rostest.rosrun("dynasense", "frame_alignment", FrameAlignmentTest)
