#!/usr/bin/env python3

import random
import math

import rospy
from sensor_msgs.msg import LaserScan
from rl_controller.msg import FootToFSnapshot, FootToFSensor


class ToFPreprocessor:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "foot_tof")
        self.scale = float(rospy.get_param("~scale", 1000.0))  # meters -> mm
        self.noise_std = float(rospy.get_param("~noise_std", 0.001))  # meters
        self.drop_prob = float(rospy.get_param("~drop_prob", 0.20))
        self.missing_value = int(rospy.get_param("~missing_value", -1000))
        self.max_range = float(rospy.get_param("~max_range", 1.0))  # meter
        self.valid_status = int(rospy.get_param("~valid_status", 5))
        self.missing_status = int(rospy.get_param("~missing_status", 0))
        self.constant_vertical_offset = float(rospy.get_param("~constant_vertical_offset", 0.01))

        # Topic -> sensor name mapping (default matches example naming)
        self.topic_map = {
            "/dynasense/foot_tof/LF": "FL",
            "/dynasense/foot_tof/RF": "FR",
            "/dynasense/foot_tof/LH": "BL",
            "/dynasense/foot_tof/RH": "BR",
        }

        self.latest = {name: None for name in self.topic_map.values()}
        self.pub = rospy.Publisher("/foot_tof/snapshot_gazebo_source", FootToFSnapshot, queue_size=1)

        for topic, name in self.topic_map.items():
            rospy.Subscriber(topic, LaserScan, self._cb, callback_args=name, queue_size=1)

        self.seq = 0

    def _process_scan(self, scan):
        data = []
        status = []
        for r in scan.ranges:
            if r < 0:
                raise ValueError("[ToFPreprocessor] Received negative range from ToF sensor!!")
            # if not math.isfinite(r) or r <= 0.0 or random.random() < self.drop_prob:
            if random.random() < self.drop_prob:
                data.append(self.missing_value)
                status.append(self.missing_status)
                continue
            noisy = r + random.gauss(0.0, self.noise_std)

            # Apply constant vertical offset to simulate the ToF being above the ground
            noisy = noisy - self.constant_vertical_offset
            noisy = min(max(0.0, noisy), self.max_range)  # clamp to [0, max_range]

            data.append(int(round(noisy * self.scale)))
            status.append(self.valid_status)
        return data, status

    def _cb(self, msg, sensor_name):
        data, status = self._process_scan(msg)
        sensor = FootToFSensor()
        sensor.name = sensor_name
        sensor.data = data
        sensor.status = status
        sensor.seq = msg.header.seq
        self.latest[sensor_name] = sensor

        # Publish only when we have all four sensors at least once.
        if any(v is None for v in self.latest.values()):
            return

        snapshot = FootToFSnapshot()
        snapshot.header.seq = self.seq
        snapshot.header.stamp = rospy.Time.now()
        snapshot.header.frame_id = self.frame_id
        snapshot.names = ["BL", "BR", "FL", "FR"]
        snapshot.sensors = [self.latest[n] for n in snapshot.names]

        self.pub.publish(snapshot)
        self.seq += 1


def main():
    rospy.init_node("tof_preprocessor")
    ToFPreprocessor()
    rospy.spin()


if __name__ == "__main__":
    main()
