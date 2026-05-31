#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ContactsState

from dynasense.msg import KneeVisMsg


class KneeContactVisPublisher:
    """Convert Gazebo knee contact topics to compact visualization messages."""

    LEGS = ("LF", "RF", "LH", "RH")
    EPS = 1e-9

    def __init__(self):
        self.contact_topic_format = rospy.get_param("~contact_topic_format", "/contacts/{leg}/knee_cylinder")
        self.output_topic = rospy.get_param("~output_topic", "/knee_eflesh/knee_vis")

        self.pub = rospy.Publisher(self.output_topic, KneeVisMsg, queue_size=20)

        for leg in self.LEGS:
            topic = self.contact_topic_format.format(leg=leg)
            rospy.Subscriber(topic, ContactsState, self._contact_cb, callback_args=leg, queue_size=1)
            rospy.loginfo("knee_contact_vis_publisher subscribed to %s", topic)

    def _contact_cb(self, msg, leg):
        dxsum = 0.0
        dzsum = 0.0
        for state in msg.states:
            if not state.contact_positions:
                continue

            print(state.collision1_name)
            if "knee_cylinder_collision" in state.collision1_name:
                sign = 1
            else: 
                sign = -1

            px = state.contact_positions[0].x
            pz = state.contact_positions[0].z
            denom = math.hypot(px, pz)
            if denom < self.EPS:
                continue

            dx = -px / denom
            dz = -pz / denom
            normal_mag = sign*(state.total_wrench.force.x * dx + state.total_wrench.force.z * dz)
            dxsum += normal_mag * dx
            dzsum += normal_mag * dz

        magnitude = math.hypot(dxsum, dzsum)
        angle = math.atan2(dxsum, dzsum) if magnitude > self.EPS else 0.0

        vis_msg = KneeVisMsg()
        vis_msg.leg_id = leg
        vis_msg.angle = angle
        vis_msg.magnitude = magnitude
        self.pub.publish(vis_msg)


def main():
    rospy.init_node("knee_contact_vis_publisher")
    KneeContactVisPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
