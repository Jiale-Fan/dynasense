#!/usr/bin/env python3

import math

import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from dynasense.msg import KneeVisMsg


class KneeContactForceVisualizer:
    """Visualize knee contact as arrow (force) or blue sphere (no contact)."""

    LEGS = ("LF", "LH", "RF", "RH")

    LEN_COEFF = 0.001  # [m/N]
    MIN_ARROW_LEN = 0.05
    MAX_ARROW_LEN = 0.3
    ARROW_SHAFT_DIAM = 0.05
    ARROW_HEAD_DIAM = 0.1
    ARROW_HEAD_LEN = 0.02
    SPHERE_DIAM = 0.03

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/dynasense/knee_vis")
        self.output_topic = rospy.get_param("~marker_topic", "/knee_eflesh/knee_contact_force_markers")
        self.target_frame = rospy.get_param("~target_frame", "odom")
        self.marker_lifetime = rospy.Duration(rospy.get_param("~marker_lifetime_sec", 0.2))

        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher(self.output_topic, MarkerArray, queue_size=20)
        self.sub = rospy.Subscriber(self.input_topic, KneeVisMsg, self._msg_cb, queue_size=50)
        rospy.loginfo("knee_contact_force_visualizer listening to %s", self.input_topic)

    def _msg_cb(self, msg):
        if msg.leg_id not in self.LEGS:
            rospy.logwarn_throttle(2.0, "knee_contact_force_visualizer: unknown leg_id '%s'", msg.leg_id)
            return

        marker = self._build_marker(msg)
        if marker is not None:
            self.pub.publish(MarkerArray(markers=[marker]))

    def _build_marker(self, msg):
        leg_frame = "{}_shank_fixed".format(msg.leg_id)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.ns = "knee_contact_vis"
        marker.id = self.LEGS.index(msg.leg_id)
        marker.lifetime = self.marker_lifetime
        marker.frame_locked = False
        marker.pose.orientation.w = 1.0

        origin_leg = Point(x=0.0, y=0.0, z=0.0)
        origin_target = self._transform_point(origin_leg, leg_frame, self.target_frame)
        if origin_target is None:
            return None

        if msg.magnitude <= 0.0:
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = origin_target
            marker.scale.x = self.SPHERE_DIAM
            marker.scale.y = self.SPHERE_DIAM
            marker.scale.z = self.SPHERE_DIAM
            marker.color.r = 0.1
            marker.color.g = 0.3
            marker.color.b = 1.0
            marker.color.a = 0.95
            return marker

        arrow_len = max(self.MIN_ARROW_LEN, min(self.MAX_ARROW_LEN, self.LEN_COEFF * msg.magnitude))
        dir_x = math.sin(msg.angle)
        dir_z = math.cos(msg.angle)
        end_leg = Point(x=arrow_len * dir_x, y=0.0, z=arrow_len * dir_z)
        end_target = self._transform_point(end_leg, leg_frame, self.target_frame)
        if end_target is None:
            return None

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [origin_target, end_target]
        marker.scale.x = self.ARROW_SHAFT_DIAM
        marker.scale.y = self.ARROW_HEAD_DIAM
        marker.scale.z = self.ARROW_HEAD_LEN
        marker.color.r = 0.95
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.95
        return marker

    def _transform_point(self, point, source_frame, target_frame):
        if source_frame == target_frame:
            return point

        stamped = PointStamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = source_frame
        stamped.point = point

        try:
            transformed = self.tf_listener.transformPoint(target_frame, stamped)
            return transformed.point
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(
                2.0,
                "knee_contact_force_visualizer: cannot transform point from %s to %s",
                source_frame,
                target_frame,
            )
            return None


def main():
    rospy.init_node("knee_contact_force_visualizer")
    KneeContactForceVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
