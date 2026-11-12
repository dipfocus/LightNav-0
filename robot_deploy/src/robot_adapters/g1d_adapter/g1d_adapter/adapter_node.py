"""Bridge the G1-D wheel base and measured ROS odometry to LightNav."""

from __future__ import annotations

import json
import math
import queue
import time
from dataclasses import fields

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from .safety import ControlConfig, fresh, validate_odometry
from .unitree_client import G1DClient, connect_rpc


class G1DAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("g1d_adapter")
        self.declare_parameter("network_interface", Parameter.Type.STRING)
        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("odom_topic", "g1d/odom")
        self.declare_parameter("native_odom_dds_topic", "")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)
        defaults = ControlConfig()
        for field in fields(ControlConfig):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self.config = ControlConfig(**{
            field.name: self.get_parameter(field.name).value for field in fields(ControlConfig)
        })
        self.network_interface = str(self.get_parameter("network_interface").value or "").strip()
        self.domain_id = int(self.get_parameter("dds_domain_id").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value).strip()
        self.odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        if not all((self.network_interface, self.odom_topic, self.odom_frame, self.base_frame)):
            raise ValueError("network_interface, odom_topic, odom_frame and base_frame must be set")
        if self.resolve_topic_name(self.odom_topic) == self.resolve_topic_name("odom"):
            raise ValueError("odom_topic input must differ from the adapter's odom output")
        native_odom_topic = str(self.get_parameter("native_odom_dds_topic").value).strip()
        if native_odom_topic and native_odom_topic == "rt" + self.resolve_topic_name("odom"):
            raise ValueError("native odometry DDS source must differ from the adapter's odom output")
        if self.odom_frame == self.base_frame:
            raise ValueError("odom_frame and base_frame must differ")

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.sent_pub = self.create_publisher(TwistStamped, "cmd_vel", 20)
        self.source_pub = self.create_publisher(String, "control/source", latched)
        self.event_pub = self.create_publisher(String, "robot/events", 20)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.get_parameter("publish_tf").value else None
        )
        self._incoming: queue.Queue = queue.Queue(maxsize=100)
        self._last_stamp_ns = 0
        self._last_source = None
        self._source_enabled_ns = 0
        self._last_error = ""
        self._closed = False
        rpc = connect_rpc(self.network_interface, self.domain_id, self.config.sdk_timeout_s)
        self._client = G1DClient(
            rpc, self.config,
            sent_callback=lambda value: self._enqueue("sent", value),
            error_callback=lambda value: self._enqueue("error", value),
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "web/cmd_vel",
                                 lambda msg: self._on_command("manual", msg), 1)
        self.create_subscription(TwistStamped, "mpc/cmd_vel",
                                 lambda msg: self._on_command("auto", msg), 1)
        self.create_service(SetBool, "control/set_manual",
                            lambda req, res: self._set_source("manual", req, res))
        self.create_service(SetBool, "control/set_auto",
                            lambda req, res: self._set_source("auto", req, res))
        self.create_service(Trigger, "control/stop", self._stop)
        self.create_service(Trigger, "robot/emergency_stop", self._emergency_stop)
        self.create_service(Trigger, "robot/reset_emergency_stop", self._reset_emergency_stop)
        for action in ("stand", "walk", "sit"):
            self.create_service(Trigger, f"robot/{action}", self._unsupported_posture)
        self.create_timer(0.05, self._process_incoming)
        self.create_timer(0.1, self._publish_status)
        self._publish_status()
        self.get_logger().info(
            f"G1-D adapter disabled; AGV interface={self.network_interface}, "
            f"measured odometry input={self.odom_topic}"
        )

    def _enqueue(self, kind, value) -> None:
        try:
            self._incoming.put_nowait((kind, value))
        except queue.Full:
            # Telemetry cannot delay the independent command watchdog.
            pass

    def _on_odometry(self, message: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        received_s = time.monotonic()
        try:
            stamp_ns = validate_odometry(
                message, now_ns=now_ns, timeout_s=self.config.state_timeout_s,
                odom_frame=self.odom_frame, base_frame=self.base_frame,
            )
        except (ValueError, TypeError, OverflowError) as exc:
            self._client.invalidate_odometry(str(exc))
            return
        if stamp_ns <= self._last_stamp_ns:
            # Duplicate/out-of-order messages must not refresh the watchdog.
            return
        self._last_stamp_ns = stamp_ns
        # Account for transport age rather than allowing another full timeout.
        self._client.receive_odometry(received_s - (now_ns - stamp_ns) / 1e9)
        self.odom_pub.publish(message)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            pose = message.pose.pose
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _on_command(self, source: str, message: TwistStamped) -> None:
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        twist = message.twist
        unsupported = (twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y)
        if (
            message.header.frame_id != self.base_frame
            or stamp_ns <= 0
            or not 0 <= stamp.nanosec < 1_000_000_000
            or not fresh(age_s, self.config.input_watchdog_s)
            or not all(math.isfinite(value) and value == 0.0 for value in unsupported)
        ):
            self._client.reject_command(source, "invalid, stale or unsupported velocity command")
            return
        if stamp_ns <= self._source_enabled_ns:
            return
        self._client.receive_command(
            source, float(twist.linear.x), float(twist.angular.z),
            received_s=time.monotonic() - age_s,
        )

    def _set_source(self, source, request, response):
        response.success, response.message = self._client.set_source(source if request.data else "disabled")
        if response.success:
            self._source_enabled_ns = self.get_clock().now().nanoseconds
        self._publish_status()
        return response

    def _stop(self, _request, response):
        response.success, response.message = self._client.stop()
        self._publish_status()
        return response

    def _emergency_stop(self, _request, response):
        response.success, response.message = self._client.stop(emergency=True)
        self._publish_status()
        return response

    def _reset_emergency_stop(self, _request, response):
        response.success, response.message = self._client.reset_emergency_stop()
        self._publish_status()
        return response

    def _unsupported_posture(self, _request, response):
        response.success = False
        response.message = "G1-D is a wheeled base; posture and lift actions are not supported"
        return response

    def _process_incoming(self) -> None:
        for _ in range(100):
            try:
                kind, value = self._incoming.get_nowait()
            except queue.Empty:
                break
            if kind == "sent":
                message = TwistStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = self.base_frame
                message.twist.linear.x, message.twist.angular.z = value
                self.sent_pub.publish(message)
            elif kind == "error" and value != self._last_error:
                self._last_error = value
                self.get_logger().error(value)
                self.event_pub.publish(String(data=json.dumps({"error": value})))

    def _publish_status(self) -> None:
        if not rclpy.ok():
            return
        state = self._client.snapshot()
        source = state["control_source"]
        if source != self._last_source:
            self._last_source = source
            self.source_pub.publish(String(data=source))
            self.event_pub.publish(String(data=json.dumps({"control_source": source})))
        status = DiagnosticStatus()
        status.name = "robot_adapter"
        status.hardware_id = "g1d"
        status.level = (
            DiagnosticStatus.ERROR if state["sdk_error"] or state["software_stop_latched"]
            else DiagnosticStatus.OK if state["connected"] else DiagnosticStatus.WARN
        )
        status.message = state["sdk_error"] or (
            "software stop latched" if state["software_stop_latched"]
            else "measured odometry available; AGV RPC checked on commands" if state["connected"]
            else "waiting for measured odometry"
        )
        values = {
            "adapter": "g1d", "robot_id": "g1d", **state,
            "battery": "", "imu": "UNKNOWN", "motor": "UNKNOWN",
            "network_interface": self.network_interface, "odom_topic": self.odom_topic,
        }
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            self._client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = G1DAdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
