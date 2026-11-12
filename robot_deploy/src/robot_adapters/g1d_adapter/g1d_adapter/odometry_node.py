"""Publish measured native G1-D DDS odometry as ROS Odometry; no motion API."""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .odometry import DdsOdometrySubscriber, OdometryConverter, populate_odometry


class G1DOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("g1d_odometry")
        self.declare_parameter("network_interface", Parameter.Type.STRING)
        for name, default in {
            "dds_domain_id": 0, "dds_topic": "rt/agv/odom", "odom_topic": "g1d/odom",
            "odom_frame": "odom", "base_frame": "base_link",
            "clock_offset_s": 0.0, "max_age_s": 0.5, "poll_rate_hz": 100.0,
        }.items():
            self.declare_parameter(name, default)
        network_interface = str(self.get_parameter("network_interface").value or "").strip()
        domain_id = int(self.get_parameter("dds_domain_id").value)
        self.dds_topic = str(self.get_parameter("dds_topic").value).strip()
        odom_topic = str(self.get_parameter("odom_topic").value).strip()
        poll_hz = float(self.get_parameter("poll_rate_hz").value)
        if not odom_topic or not math.isfinite(poll_hz) or poll_hz <= 0:
            raise ValueError("odom_topic must be set and poll_rate_hz must be finite and positive")
        self.converter = OdometryConverter(
            odom_frame=str(self.get_parameter("odom_frame").value).strip(),
            base_frame=str(self.get_parameter("base_frame").value).strip(),
            clock_offset_s=float(self.get_parameter("clock_offset_s").value),
            max_age_s=float(self.get_parameter("max_age_s").value),
        )
        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        # Use the actual resolved/remapped publisher name (also available on
        # Foxy). ROS /agv/odom maps to DDS rt/agv/odom: publishing back to the
        # source would create a self-feeding stream hiding sensor loss.
        if "rt" + self.publisher.topic_name == self.dds_topic:
            raise ValueError("ROS odom_topic must not resolve to the native DDS source topic")
        self.status_pub = self.create_publisher(String, "g1d/odometry_status", 1)
        self._last_stamp_ns = 0
        self._last_error = "waiting for native odometry"
        self._last_log_s = float("-inf")
        self._accepted = 0
        self._rejected = 0
        self._closed = False
        self._subscriber = DdsOdometrySubscriber(network_interface, domain_id, self.dds_topic)
        self.create_timer(1.0 / poll_hz, self._poll)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Read-only G1-D odometry: DDS {self.dds_topic} on {network_interface} -> {odom_topic}"
        )

    def _poll(self) -> None:
        message = self._subscriber.take()
        if message is None:
            return
        try:
            sample = self.converter.convert(message, now_ns=self.get_clock().now().nanoseconds)
            self.publisher.publish(populate_odometry(Odometry(), sample))
        except (AttributeError, ValueError, TypeError, OverflowError) as exc:
            self._rejected += 1
            self._last_error = str(exc)
            if time.monotonic() - self._last_log_s >= 5.0:
                self._last_log_s = time.monotonic()
                self.get_logger().warning(f"Native odometry rejected: {exc}")
            return
        self._last_stamp_ns = sample.stamp_ns
        self._last_error = ""
        self._accepted += 1

    def _publish_status(self) -> None:
        age_ns = self.get_clock().now().nanoseconds - self._last_stamp_ns
        connected = bool(self._last_stamp_ns and 0 <= age_ns <= self.converter.max_age_ns)
        self.status_pub.publish(String(data=json.dumps({
            "connected": connected, "dds_topic": self.dds_topic,
            "accepted": self._accepted, "rejected": self._rejected,
            "age_s": age_ns / 1e9 if self._last_stamp_ns else None,
            "timestamp_basis": "source_stamp_plus_offset",
            "error": self._last_error or ("" if connected else "native odometry is stale"),
        })))

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            self._subscriber.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = G1DOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
