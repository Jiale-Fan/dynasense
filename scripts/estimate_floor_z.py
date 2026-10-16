#!/usr/bin/env python3
"""Estimate the floor height from the four foot soles and print it.

  rosrun dynasense estimate_floor_z.py                 # prints e.g. -0.2912 on stdout
  rosrun dynasense estimate_floor_z.py --duration 3 --frame odom

The robot must stand still with all four feet on flat ground. The sole height of
each foot is the lowest point of its collision ball: the FOOT frame comes from
TF, i.e. forward kinematics of the measured joint angles on top of the state
estimator's odom -> base, and the ball (centre and radius from the ANYmal D foot
macro, foot_3_6_3_macro.urdf.xacro) is placed in it. The floor is the median
over the sampling window of the mean of the four soles.

Only the value goes to stdout, so scripts can capture it; everything else goes
to stderr. Exit codes: 0 ok, 1 no master or no TF, 2 feet not level (not standing on flat
ground), 3 too few samples.
"""

import argparse
import math
import statistics
import sys

import rosgraph
import rospy
import tf2_ros

FEET = ("LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT")
# Collision ball of the ANYmal D foot in the FOOT frame: y = +-r sin(-15 deg)
# (+ for left legs), z = r cos(-15 deg), radius r = 0.033 m.
BALL_R = 0.033
BALL_Y = BALL_R * math.sin(math.radians(-15.0))
BALL_Z = BALL_R * math.cos(math.radians(-15.0))


def rotate(q, v):
    """Rotate v by the unit quaternion q (geometry_msgs Quaternion)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w t + q_vec x t
    tx, ty, tz = 2.0 * (y * vz - z * vy), 2.0 * (z * vx - x * vz), 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def sole_z(transform, left):
    t, q = transform.transform.translation, transform.transform.rotation
    centre = rotate(q, (0.0, BALL_Y if left else -BALL_Y, BALL_Z))
    return t.z + centre[2] - BALL_R


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frame", default="odom", help="frame the floor height is expressed in")
    parser.add_argument("--duration", type=float, default=2.0, help="sampling window [s]")
    parser.add_argument("--rate", type=float, default=20.0, help="samples per second")
    parser.add_argument("--max-spread", type=float, default=0.03,
                        help="largest allowed height difference between the four soles [m]")
    parser.add_argument("--timeout", type=float, default=15.0, help="wait this long for TF before giving up [s]")
    args = parser.parse_args(rospy.myargv()[1:])

    if not rosgraph.is_master_online():
        print("[estimate_floor_z] no ROS master at %s" % rosgraph.get_master_uri(), file=sys.stderr, flush=True)
        return 1
    rospy.init_node("estimate_floor_z", anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    def log(msg):
        print("[estimate_floor_z] " + msg, file=sys.stderr, flush=True)

    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    while not rospy.is_shutdown():
        if all(buf.can_transform(args.frame, f, rospy.Time(0)) for f in FEET):
            break
        if rospy.Time.now() > deadline:
            log("no TF %s <- %s within %.0f s" % (args.frame, "/".join(FEET), args.timeout))
            return 1
        rospy.sleep(0.1)

    means, spreads = [], []
    period = rospy.Duration(1.0 / max(args.rate, 1.0))
    end = rospy.Time.now() + rospy.Duration(args.duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        try:
            soles = [sole_z(buf.lookup_transform(args.frame, f, rospy.Time(0)), f.startswith("L")) for f in FEET]
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.sleep(period)
            continue
        means.append(sum(soles) / 4.0)
        spreads.append(max(soles) - min(soles))
        rospy.sleep(period)

    if len(means) < 3:
        log("only %d samples in %.1f s" % (len(means), args.duration))
        return 3
    floor = statistics.median(means)
    spread = statistics.median(spreads)
    jitter = max(means) - min(means)
    log("%d samples: floor %.4f m in %s, soles spread %.4f m, drift over the window %.4f m"
        % (len(means), floor, args.frame, spread, jitter))
    if spread > args.max_spread:
        log("the four soles differ by %.3f m (> %.3f): stand on flat ground with all feet down" % (spread, args.max_spread))
        return 2
    print("%.4f" % floor, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
