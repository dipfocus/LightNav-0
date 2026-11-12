"""Read-only native G1-D DDS odometry, independent of ROS and motion RPCs.

The official SDK exposes rt/agv/odom as nav_msgs.msg.dds_.Odometry_.
Preserve its measured pose, twist and sampling time; never integrate commands.
"""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class OdometrySample:
    source_stamp_ns: int
    stamp_ns: int
    odom_frame: str
    base_frame: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]
    pose_covariance: tuple[float, ...]
    twist_covariance: tuple[float, ...]


def _vector(value) -> tuple[float, float, float]:
    return float(value.x), float(value.y), float(value.z)


class OdometryConverter:
    """Validate and snapshot native samples on the ROS publishing thread.

    G1-D's omitted child_frame_id is filled with the configured base_frame.
    Nonempty mismatched frames are always rejected; this is not a TF transform.
    """

    def __init__(
        self, *, odom_frame: str = "odom", base_frame: str = "base_link",
        clock_offset_s: float = 0.0, max_age_s: float = 0.5,
    ) -> None:
        if not odom_frame or not base_frame or odom_frame == base_frame:
            raise ValueError("odom_frame and base_frame must be nonempty and distinct")
        if not math.isfinite(clock_offset_s) or not math.isfinite(clock_offset_s * 1e9):
            raise ValueError("clock_offset_s must be finite")
        if not math.isfinite(max_age_s) or max_age_s <= 0 or not math.isfinite(max_age_s * 1e9):
            raise ValueError("max_age_s must be finite and positive")
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.offset_ns = round(clock_offset_s * 1e9)
        self.max_age_ns = round(max_age_s * 1e9)
        self.last_source_stamp_ns = 0

    def convert(self, message, *, now_ns: int) -> OdometrySample:
        if message.header.frame_id != self.odom_frame:
            raise ValueError(f"native odometry frame must be {self.odom_frame!r}; transform upstream")
        child = message.child_frame_id
        if child not in ("", self.base_frame):
            raise ValueError(
                f"native child_frame_id is {child!r}, expected {self.base_frame!r} or an omitted name"
            )
        stamp = message.header.stamp
        if not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int):
            raise ValueError("native odometry timestamp must contain integer seconds/nanoseconds")
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("invalid native odometry timestamp")
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if source_ns <= 0 or source_ns <= self.last_source_stamp_ns:
            raise ValueError("missing, duplicate or out-of-order native odometry timestamp")
        stamp_ns = source_ns + self.offset_ns
        if not 0 < stamp_ns < (2**31) * 1_000_000_000:
            raise ValueError("corrected odometry timestamp is outside the ROS Time range")
        if not 0 <= now_ns - stamp_ns <= self.max_age_ns:
            raise ValueError("native odometry is stale or future-dated; synchronize clocks or set clock_offset_s")
        pose = message.pose.pose
        twist = message.twist.twist
        position = _vector(pose.position)
        q = pose.orientation
        orientation = (float(q.x), float(q.y), float(q.z), float(q.w))
        linear, angular = _vector(twist.linear), _vector(twist.angular)
        pose_cov = tuple(float(value) for value in message.pose.covariance)
        twist_cov = tuple(float(value) for value in message.twist.covariance)
        if len(pose_cov) != 36 or len(twist_cov) != 36:
            raise ValueError("native odometry covariance must have 36 entries")
        if not all(math.isfinite(value) for value in (
            *position, *orientation, *linear, *angular, *pose_cov, *twist_cov,
        )):
            raise ValueError("native odometry contains non-finite values")
        if abs(math.hypot(*orientation) - 1.0) > 0.01:
            raise ValueError("native odometry quaternion must be normalized")
        self.last_source_stamp_ns = source_ns
        return OdometrySample(
            source_ns, stamp_ns, self.odom_frame, self.base_frame,
            position, orientation, linear, angular, pose_cov, twist_cov,
        )


def populate_odometry(message, sample: OdometrySample):
    """Copy a validated DDS snapshot into a fresh ROS nav_msgs/Odometry."""
    message.header.stamp.sec = sample.stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = sample.stamp_ns % 1_000_000_000
    message.header.frame_id = sample.odom_frame
    message.child_frame_id = sample.base_frame
    p, q = message.pose.pose.position, message.pose.pose.orientation
    p.x, p.y, p.z = sample.position
    q.x, q.y, q.z, q.w = sample.orientation
    linear, angular = message.twist.twist.linear, message.twist.twist.angular
    linear.x, linear.y, linear.z = sample.linear
    angular.x, angular.y, angular.z = sample.angular
    message.pose.covariance = list(sample.pose_covariance)
    message.twist.covariance = list(sample.twist_covariance)
    return message


class DdsOdometrySubscriber:
    """One latest-only SDK subscription. Creates no DDS publisher or RPC client."""

    def __init__(self, network_interface: str, domain_id: int = 0, topic: str = "rt/agv/odom") -> None:
        if not network_interface.strip() or domain_id < 0 or not topic.strip():
            raise ValueError("network_interface and DDS topic are required; domain_id must be nonnegative")
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        except ImportError as exc:
            raise RuntimeError("native odometry requires Unitree's official unitree_sdk2_python") from exc
        self._lock = threading.Lock()
        self._latest = None
        self._closed = False
        ChannelFactoryInitialize(domain_id, network_interface)
        self._subscription = ChannelSubscriber(topic, Odometry_)
        try:
            self._subscription.Init(self._receive, 1)
        except Exception:
            self.close()
            raise

    def _receive(self, message) -> None:
        with self._lock:
            if not self._closed:
                self._latest = copy.deepcopy(message)

    def take(self):
        with self._lock:
            message, self._latest = self._latest, None
            return message

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._latest = None
        # Close may join the SDK callback thread; never hold its lock here.
        self._subscription.Close()
