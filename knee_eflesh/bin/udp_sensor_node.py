#!/usr/bin/env python3
import socket
import time

import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from dynasense_udp_receiver_ros1.msg import KneeVisMsg


class UdpLatestPacketReceiver:
    def __init__(self, port):
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", self.port))
        self.sock.setblocking(False)

    def send_discovery(self):
        self.sock.sendto(b"DISCOVER", ("255.255.255.255", self.port))

    def poll_latest(self, packet_size, max_drain_packets):
        latest_data = None
        packets = 0

        for _ in range(int(max_drain_packets)):
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                break

            if len(data) == packet_size:
                latest_data = data
                packets += 1

        return latest_data, packets


class DynasenseUdpReceiverNode:
    def __init__(self):
        rospy.init_node("dynasense_udp_receiver", anonymous=False)

        self.mag_max = 1000.0

        self.udp_port = int(rospy.get_param("~udp_port", 4210))
        self.num_sensors = int(rospy.get_param("~num_sensors", 8))
        self.axes_per_sensor = int(rospy.get_param("~axes_per_sensor", 3))
        self.max_drain_packets = int(rospy.get_param("~max_drain_packets", 800))
        self.poll_period_s = float(rospy.get_param("~poll_period_s", 0.001))
        self.discovery_period_s = float(rospy.get_param("~discovery_period_s", 2.0))
        self.auto_tare_delay_s = float(rospy.get_param("~auto_tare_delay_s", 10.0))
        self.connection_timeout_s = float(rospy.get_param("~connection_timeout_s", 1.0))
        self.knee_vis_start_angle_deg = float(rospy.get_param("~knee_vis_start_angle_deg", 210.0))

        stream_order_raw = list(rospy.get_param("~stream_order", [1, 2, 3, 4, 5, 6, 7, 8]))

        self.stream_order = self._validate_stream_order(stream_order_raw)

        self.sensor_float_count = self.num_sensors * self.axes_per_sensor
        self.packet_float_count = self.sensor_float_count + 1  # Last float is battery voltage.
        self.packet_size = self.packet_float_count * 4

        self.receiver = UdpLatestPacketReceiver(self.udp_port)
        self.receiver.send_discovery()

        self.raw_flat_pub = rospy.Publisher("dynasense/raw_flat", Float32MultiArray, queue_size=10)
        self.ordered_vectors_pub = rospy.Publisher("dynasense/ordered_vectors", Float32MultiArray, queue_size=10)
        self.magnitudes_pub = rospy.Publisher("dynasense/magnitudes", Float32MultiArray, queue_size=10)
        self.battery_voltage_pub = rospy.Publisher("dynasense/battery_voltage", Float32, queue_size=10)
        self.knee_vis_pub = rospy.Publisher("dynasense/knee_vis", KneeVisMsg, queue_size=10)

        self.latest_packet = None
        self.baseline_offset = np.zeros((self.num_sensors, self.axes_per_sensor), dtype=np.float32)
        self.auto_tare_done = False
        self.start_time = time.perf_counter()
        self.last_packet_time = 0.0
        self.connection_active = False
        self.packets_since_report = 0
        self.last_rate_report_time = time.perf_counter()

        self.poll_timer = rospy.Timer(rospy.Duration(self.poll_period_s), self._on_poll_timer)
        self.discovery_timer = rospy.Timer(rospy.Duration(self.discovery_period_s), self._on_discovery_timer)
        self.report_timer = rospy.Timer(rospy.Duration(1.0), self._on_report_timer)

        rospy.loginfo(
            "Listening on UDP port %d for %d sensors x %d axes (+ battery voltage, packet size=%d bytes)",
            self.udp_port,
            self.num_sensors,
            self.axes_per_sensor,
            self.packet_size,
        )
        rospy.loginfo(
            "Using stream order (display->stream, 1-based): %s",
            ",".join(str(int(i) + 1) for i in self.stream_order),
        )
        rospy.loginfo("Auto-tare delay: %.2f s", self.auto_tare_delay_s)

    def _validate_stream_order(self, stream_order_raw):
        if len(stream_order_raw) != self.num_sensors:
            raise ValueError("stream_order must have {} entries".format(self.num_sensors))

        order = np.array([int(v) - 1 for v in stream_order_raw], dtype=np.int32)

        if np.any(order < 0) or np.any(order >= self.num_sensors):
            raise ValueError("stream_order indices must be in 1..{}".format(self.num_sensors))
        if len(np.unique(order)) != self.num_sensors:
            raise ValueError("stream_order indices must be unique")

        return order

    def _on_discovery_timer(self, _event):
        self.receiver.send_discovery()

    def _on_poll_timer(self, _event):
        latest_data, packet_count = self.receiver.poll_latest(self.packet_size, self.max_drain_packets)
        now = time.perf_counter()

        if packet_count <= 0 or latest_data is None:
            if self.connection_active and self.last_packet_time > 0.0:
                elapsed_without_packets = now - self.last_packet_time
                if elapsed_without_packets >= self.connection_timeout_s:
                    self.connection_active = False
                    self.auto_tare_done = False
                    self.start_time = now
                    self.baseline_offset.fill(0.0)
                    rospy.logwarn(
                        "Connection lost (no packets for %.2f s). Auto-tare will run after reconnect.",
                        elapsed_without_packets,
                    )
            return

        if not self.connection_active:
            self.connection_active = True
            self.auto_tare_done = False
            self.start_time = now
            rospy.loginfo("Connection restored. Re-arming auto-tare.")

        self.last_packet_time = now
        self.latest_packet = latest_data
        self.packets_since_report += packet_count
        self._publish_packet(latest_data)

    def _publish_packet(self, packet):
        vals = np.frombuffer(packet, dtype="<f4", count=self.packet_float_count)
        if vals.size != self.packet_float_count:
            return

        sensor_vals = vals[: self.sensor_float_count]
        battery_voltage = float(vals[-1])

        b_raw = sensor_vals.reshape(self.num_sensors, self.axes_per_sensor)
        b_ordered = b_raw[self.stream_order, :]

        if not self.auto_tare_done:
            elapsed_s = time.perf_counter() - self.start_time
            if elapsed_s >= self.auto_tare_delay_s:
                self.baseline_offset = b_ordered.copy()
                self.auto_tare_done = True
                rospy.loginfo("Auto-tare captured after %.2f s", elapsed_s)

        b_ordered = b_ordered - self.baseline_offset
        magnitudes = np.linalg.norm(b_ordered, axis=1)

        raw_msg = Float32MultiArray()
        raw_msg.data = vals.tolist()
        self.raw_flat_pub.publish(raw_msg)

        battery_msg = Float32()
        battery_msg.data = battery_voltage
        self.battery_voltage_pub.publish(battery_msg)

        ordered_msg = Float32MultiArray()
        ordered_msg.data = b_ordered.reshape(-1).tolist()
        self.ordered_vectors_pub.publish(ordered_msg)

        mag_msg = Float32MultiArray()
        mag_msg.data = magnitudes.tolist()
        self.magnitudes_pub.publish(mag_msg)

        knee_vis_msg = KneeVisMsg()
        knee_vis_msg.leg_id = "RF"
        angle_deg, magnitude = self._compute_knee_vis(magnitudes)
        knee_vis_msg.angle = float(angle_deg)
        knee_vis_msg.magnitude = float(magnitude)
        self.knee_vis_pub.publish(knee_vis_msg)

    def _compute_knee_vis(self, sensor_mag):
        if sensor_mag.size != self.num_sensors:
            return 0.0, 0.0

        angles_deg = self.knee_vis_start_angle_deg - np.arange(self.num_sensors, dtype=np.float32) * 30.0
        angles_rad = np.mod(np.deg2rad(angles_deg), 2.0 * np.pi)

        unit_x = -np.cos(angles_rad)
        unit_y = -np.sin(angles_rad)

        sum_dx = float(np.sum(unit_x * sensor_mag))
        sum_dy = float(np.sum(unit_y * sensor_mag))

        sum_arrow_len = float(np.hypot(sum_dx, sum_dy))
        angle_rad = 0.0
        if sum_arrow_len > 1e-9:
            angle_rad = float(np.mod(np.arctan2(sum_dy, sum_dx), 2.0 * np.pi))
        else:
            angle_rad = 0.0
        return angle_rad, sum_arrow_len

    def _on_report_timer(self, _event):
        now = time.perf_counter()
        elapsed = now - self.last_rate_report_time
        if elapsed <= 0.0:
            return

        rate_hz = self.packets_since_report / elapsed
        rospy.loginfo("Data rate: %.1f packets/s", rate_hz)

        self.last_rate_report_time = now
        self.packets_since_report = 0

    def spin(self):
        rospy.spin()


def main():
    node = DynasenseUdpReceiverNode()
    node.spin()


if __name__ == "__main__":
    main()
