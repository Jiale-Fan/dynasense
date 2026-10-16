#!/usr/bin/env python3
"""Checks the output of dynasense_wavemap_bps_bridge once per second.

  rosrun dynasense check_wavemap_voxels.py _radius:=1.0 _z_min:=-0.24 _z_max:=1.51
  rosrun dynasense check_wavemap_voxels.py _histogram:=true _duration:=20     # locate the floor

Per second: message rate and size, largest xy distance from the base vs the
radius, z range vs the band, whether every point is a cell centre, whether the
points are sorted nearest first, and how many voxels lie inside the robot base
box (self voxels). With _histogram:=true it also prints a z histogram (cell
size bins) over all received voxels; the floor shows up as the dominant peak.
"""

import collections
import math
import threading

import rospy
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

# Base box in the base frame for the self-voxel count (ANYmal D trunk + margin).
BASE_BOX = ((-0.55, 0.55), (-0.35, 0.35), (-0.6, 0.3))


class Checker:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/dynasense/wavemap_voxels")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.radius = float(rospy.get_param("~radius", 1.0))
        self.z_min = float(rospy.get_param("~z_min", -1e9))
        self.z_max = float(rospy.get_param("~z_max", 1e9))
        self.cell = float(rospy.get_param("~cell", 0.05))
        self.histogram = bool(rospy.get_param("~histogram", False))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.Lock()
        self.window = []
        self.z_counts = collections.Counter()
        self.total = 0
        rospy.Subscriber(self.topic, PointCloud2, self.on_cloud, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), self.report)

    def on_cloud(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        try:
            tf = self.tf_buffer.lookup_transform(msg.header.frame_id, self.base_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        max_xy, zs, lattice_bad, self_voxels, dists = 0.0, [], 0, 0, []
        for x, y, z in points:
            dx, dy = x - t.x, y - t.y
            d = math.hypot(dx, dy)
            dists.append(d)
            max_xy = max(max_xy, d)
            zs.append(z)
            for v in (x, y, z):
                k = v / self.cell - 0.5
                lattice_bad += abs(k - round(k)) > 1e-3
            bx, by, bz = c * dx + s * dy, -s * dx + c * dy, z - t.z  # yaw-only base frame
            self_voxels += (BASE_BOX[0][0] <= bx <= BASE_BOX[0][1] and BASE_BOX[1][0] <= by <= BASE_BOX[1][1]
                            and BASE_BOX[2][0] <= bz <= BASE_BOX[2][1])
        ordered = all(b >= a - 1e-4 for a, b in zip(dists, dists[1:]))
        with self.lock:
            self.window.append((len(points), max_xy, min(zs) if zs else None, max(zs) if zs else None,
                                lattice_bad, ordered, self_voxels))
            self.total += 1
            if self.histogram:
                for z in zs:
                    self.z_counts[int(math.floor(z / self.cell))] += 1

    def report(self, _event):
        with self.lock:
            window, self.window = self.window, []
            counts = dict(self.z_counts)
        if not window:
            rospy.loginfo("no message on %s", self.topic)
            return
        sizes = [w[0] for w in window]
        max_xy = max(w[1] for w in window)
        z_lo = min((w[2] for w in window if w[2] is not None), default=float("nan"))
        z_hi = max((w[3] for w in window if w[3] is not None), default=float("nan"))
        lattice_bad = sum(w[4] for w in window)
        ordered = all(w[5] for w in window)
        self_voxels = max(w[6] for w in window)
        radius_ok = max_xy <= self.radius + 1e-3
        band_ok = (math.isnan(z_lo) or (z_lo >= self.z_min - 1e-3 and z_hi < self.z_max + 1e-3))
        rospy.loginfo("%d msgs/s, %d..%d voxels | max xy %.3f m (radius %.2f) %s | z %.3f..%.3f (band %.2f..%.2f) %s | "
                      "lattice %s | order %s | self voxels %d",
                      len(window), min(sizes), max(sizes), max_xy, self.radius, "ok" if radius_ok else "OUTSIDE",
                      z_lo, z_hi, self.z_min, self.z_max, "ok" if band_ok else "OUTSIDE",
                      "ok" if lattice_bad == 0 else "%d off-centre" % lattice_bad, "ok" if ordered else "UNSORTED",
                      self_voxels)
        if self.histogram and counts:
            peak = max(counts, key=counts.get)
            lines = ["z histogram (%.0f cm bins, peak at %.3f m):" % (self.cell * 100, (peak + 0.5) * self.cell)]
            for k in sorted(counts):
                lines.append("  %7.3f  %s %d" % ((k + 0.5) * self.cell, "#" * min(60, counts[k] * 60 // counts[peak]),
                                                 counts[k]))
            rospy.loginfo("\n".join(lines))


def main():
    rospy.init_node("check_wavemap_voxels", anonymous=True)
    checker = Checker()
    if checker.duration > 0.0:
        rospy.sleep(checker.duration)
        rospy.signal_shutdown("duration reached")
    rospy.spin()


if __name__ == "__main__":
    main()
