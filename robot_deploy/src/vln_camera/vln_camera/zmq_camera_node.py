"""Publish an existing XRAW v2 JPEG stream as ROS CompressedImage messages."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .xraw import MAX_PACKET_BYTES, FrameClock, XrawSubscriber, parse_frame


class ZmqCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("zmq_camera")
        for name, default in {
            "endpoint": "", "camera_role": "head_fpv",
            "image_topic": "camera/color/image_raw/compressed", "frame_id": "head_camera",
            "clock_offset_s": 0.0, "capture_delay_s": 0.0, "max_frame_age_s": 0.5,
            "poll_rate_hz": 100.0, "max_packet_bytes": MAX_PACKET_BYTES,
        }.items():
            self.declare_parameter(name, default)
        endpoint = str(self.get_parameter("endpoint").value).strip()
        self.role = str(self.get_parameter("camera_role").value).strip()
        self.frame_id = str(self.get_parameter("frame_id").value).strip()
        image_topic = str(self.get_parameter("image_topic").value).strip()
        self.max_packet_bytes = int(self.get_parameter("max_packet_bytes").value)
        poll_hz = float(self.get_parameter("poll_rate_hz").value)
        if not self.role or len(self.role.encode("utf-8")) > 16 or "\0" in self.role:
            raise ValueError("camera_role must be 1..16 UTF-8 bytes without NUL")
        if not self.frame_id or not image_topic or not math.isfinite(poll_hz) or poll_hz <= 0:
            raise ValueError("frame_id and image_topic are required; poll_rate_hz must be positive")
        self.clock = FrameClock(
            clock_offset_s=float(self.get_parameter("clock_offset_s").value),
            capture_delay_s=float(self.get_parameter("capture_delay_s").value),
            max_frame_age_s=float(self.get_parameter("max_frame_age_s").value),
        )
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(CompressedImage, image_topic, qos)
        self.status_pub = self.create_publisher(String, "camera/status", 1)
        self._last_stamp_ns = 0
        self._last_error = "waiting for video stream"
        self._last_log_s = float("-inf")
        self._frames = 0
        self._rejected = 0
        self._closed = False
        self._subscriber = XrawSubscriber(endpoint, max_packet_bytes=self.max_packet_bytes)
        self.create_timer(1.0 / poll_hz, self._poll)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f"Subscribing to {endpoint}, role={self.role}, output={image_topic}")
        self.get_logger().warning(
            "XRAW time is stamped after capture/JPEG encoding, not at sensor exposure. "
            "Synchronize sender/ROS/odometry clocks; set measured clock_offset_s and "
            "capture_delay_s as needed. Receipt time is never substituted."
        )

    def _poll(self) -> None:
        try:
            packet = self._subscriber.receive()
            if packet is None:
                return
            frame = parse_frame(packet, expected_role=self.role, max_packet_bytes=self.max_packet_bytes)
            now_ns = self.get_clock().now().nanoseconds
            stamp_ns = self.clock.stamp(frame, now_ns)
            message = CompressedImage()
            message.header.stamp.sec = stamp_ns // 1_000_000_000
            message.header.stamp.nanosec = stamp_ns % 1_000_000_000
            message.header.frame_id = self.frame_id
            message.format = "jpeg"
            message.data = frame.jpeg
            self.publisher.publish(message)
            self._last_stamp_ns = stamp_ns
            self._last_error = ""
            self._frames += 1
        except Exception as exc:
            self._rejected += 1
            self._last_error = str(exc)
            if time.monotonic() - self._last_log_s >= 5.0:
                self._last_log_s = time.monotonic()
                self.get_logger().warning(f"Camera frame rejected: {exc}")

    def _publish_status(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        age_ns = now_ns - self._last_stamp_ns
        fresh = bool(self._last_stamp_ns and 0 <= age_ns <= self.clock.max_age_ns)
        self.status_pub.publish(String(data=json.dumps({
            "connected": fresh, "role": self.role, "frames": self._frames,
            "rejected": self._rejected, "timestamp_basis": "sender_post_encode_corrected",
            "frame_age_s": age_ns / 1e9 if self._last_stamp_ns else None,
            "error": self._last_error or ("" if fresh else "video stream is stale"),
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
        node = ZmqCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
