#!/usr/bin/env python3
"""Re-latch the union of all /tf_static transforms (bag replay helper).

rosbag play merges a bag's latched /tf_static publishers (nine in the Sept 2026
ANYmal bags: anymal_tf_publisher, the six RealSense nodelet managers, compslam)
into ONE publisher, and a latched publisher only re-sends its LAST message to a
late subscriber. Anything that subscribes after the first milliseconds of
playback (a slow RViz start, tf_echo, a restarted node) therefore never sees
the depth-camera chains, and with `rosbag play --start N` the statics are not
played at all.

This node keeps the union of every static transform it sees (keyed by child
frame), optionally pre-loads them straight from the bag files, and republishes
the union latched on /tf_static whenever it grows.

Parameters
  ~bags   bag file path, or a list of paths, whose /tf_static messages are
          loaded at startup (index-based read; fast even for 17 GB bags)

Usage
  rosrun dynasense tf_static_aggregator.py _bags:=/path/to/file.bag
"""

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage


def copy_transform(source):
    """geometry_msgs/TransformStamped from any object with the same fields.

    Messages read from a bag are instances of dynamically generated classes;
    copying makes them comparable with, and serializable as, the installed type.
    """
    out = TransformStamped()
    out.header.seq = source.header.seq
    out.header.stamp = rospy.Time(source.header.stamp.secs, source.header.stamp.nsecs)
    out.header.frame_id = source.header.frame_id
    out.child_frame_id = source.child_frame_id
    t = source.transform.translation
    q = source.transform.rotation
    out.transform.translation = Vector3(t.x, t.y, t.z)
    out.transform.rotation = Quaternion(q.x, q.y, q.z, q.w)
    return out


class TfStaticAggregator:
    def __init__(self):
        self._transforms = {}  # child_frame_id -> TransformStamped
        self._pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)

        bags = rospy.get_param("~bags", [])
        if isinstance(bags, str):
            bags = [bags] if bags else []
        for path in bags:
            self._load_bag(path)
        if self._transforms:
            self._publish()

        self._sub = rospy.Subscriber("/tf_static", TFMessage, self._on_static, queue_size=100)

    def _load_bag(self, path):
        import rosbag

        try:
            with rosbag.Bag(path, "r") as bag:
                added = 0
                for _, msg, _ in bag.read_messages(topics=["/tf_static"]):
                    added += self._merge(msg.transforms)
            rospy.loginfo("[tf_static_aggregator] %s: %d static transform(s) loaded", path, added)
        except Exception as exc:  # keep running on the live topic
            rospy.logwarn("[tf_static_aggregator] cannot read /tf_static from %s: %s", path, exc)

    def _merge(self, transforms):
        added = 0
        for source in transforms:
            new = copy_transform(source)
            old = self._transforms.get(new.child_frame_id)
            if old is None or old.header.frame_id != new.header.frame_id or old.transform != new.transform:
                self._transforms[new.child_frame_id] = new
                added += 1
        return added

    def _on_static(self, msg):
        if msg._connection_header.get("callerid") == rospy.get_name():
            return  # our own republication
        if self._merge(msg.transforms):
            self._publish()

    def _publish(self):
        self._pub.publish(TFMessage(transforms=list(self._transforms.values())))
        rospy.loginfo("[tf_static_aggregator] re-latched %d static transform(s)", len(self._transforms))


if __name__ == "__main__":
    rospy.init_node("tf_static_aggregator")
    TfStaticAggregator()
    rospy.spin()
