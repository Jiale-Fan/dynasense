#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class KneeContactForceVisualizer:
    """Visualize projected knee-contact force as arrow markers."""

    LEGS = ("LF", "RF", "LH", "RH")
    LEN_COEFF = 0.01  # [m/N] hardcoded scale from projected force to arrow length.
    ARROW_RADIUS = 0.008
    ARROW_HEAD_RADIUS = 0.015
    MIN_ARROW_LEN = 1e-4
    ARROW_DIR = (0.0, 1.0, 0.0)  # +Y axis in each <leg>_shank_fixed frame.

    def __init__(self):
        self.marker_topic = rospy.get_param("~marker_topic", "/knee_eflesh/knee_contact_force_markers")
        self.contact_topic_format = rospy.get_param("~contact_topic_format", "/contacts/{leg}/knee_cylinder")
        self.frame_fallback = rospy.get_param("~frame_fallback", "base")
        self.marker_lifetime = rospy.Duration(rospy.get_param("~marker_lifetime_sec", 0.2))

        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)

        for leg in self.LEGS:
            topic = self.contact_topic_format.format(leg=leg)
            rospy.Subscriber(topic, ContactsState, self._contact_cb, callback_args=leg, queue_size=1)
            rospy.loginfo("knee_contact_force_visualizer subscribed to %s", topic)

    def _contact_cb(self, msg, leg):
        marker = self._build_marker(msg, leg)
        self.pub.publish(MarkerArray(markers=[marker]))

    def _build_marker(self, msg, leg):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_fallback
        marker.ns = "knee_contact_force"
        marker.id = self.LEGS.index(leg)
        marker.type = Marker.ARROW
        marker.lifetime = self.marker_lifetime
        marker.frame_locked = False
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.ARROW_RADIUS
        marker.scale.y = self.ARROW_HEAD_RADIUS
        marker.scale.z = self.ARROW_HEAD_RADIUS
        marker.color.r = 0.95
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.95

        best_state = None
        best_proj_mag = -1.0
        for state in msg.states:
            if not state.contact_positions:
                continue
            projected_force = state.total_wrench.force.x * self.ARROW_DIR[0] + state.total_wrench.force.y * self.ARROW_DIR[1] + state.total_wrench.force.z * self.ARROW_DIR[2]
            projected_force_mag = abs(projected_force)
            if projected_force_mag > best_proj_mag:
                best_proj_mag = projected_force_mag
                best_state = state

        if best_state is None:
            marker.action = Marker.DELETE
            return marker

        contact_pos = best_state.contact_positions[0]
        projected_force = (
            best_state.total_wrench.force.x * self.ARROW_DIR[0]
            + best_state.total_wrench.force.y * self.ARROW_DIR[1]
            + best_state.total_wrench.force.z * self.ARROW_DIR[2]
        )

        direction_sign = 1.0 if projected_force >= 0.0 else -1.0
        arrow_len = max(self.MIN_ARROW_LEN, self.LEN_COEFF * abs(projected_force))

        start = Point(x=contact_pos.x, y=contact_pos.y, z=contact_pos.z)
        end = Point(
            x=start.x + direction_sign * arrow_len * self.ARROW_DIR[0],
            y=start.y + direction_sign * arrow_len * self.ARROW_DIR[1],
            z=start.z + direction_sign * arrow_len * self.ARROW_DIR[2],
        )

        marker.action = Marker.ADD
        marker.points = [start, end]
        return marker


def main():
    rospy.init_node("knee_contact_force_visualizer")
    KneeContactForceVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
