"""Transport-independent command arbitration for the G1-D wheel base."""

from __future__ import annotations

import math
from dataclasses import dataclass

MAX_LINEAR_SPEED = 1.5
MAX_ANGULAR_SPEED = 0.6
ZERO = (0.0, 0.0)


@dataclass(frozen=True)
class ControlConfig:
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.3
    input_watchdog_s: float = 0.35
    state_timeout_s: float = 0.5
    sdk_timeout_s: float = 0.1
    control_rate_hz: float = 20.0
    zero_frames: int = 5

    def __post_init__(self) -> None:
        for name in (
            "max_linear_speed", "max_angular_speed", "input_watchdog_s",
            "state_timeout_s", "sdk_timeout_s", "control_rate_hz",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_linear_speed > MAX_LINEAR_SPEED:
            raise ValueError("max_linear_speed exceeds G1-D limit of 1.5 m/s")
        if self.max_angular_speed > MAX_ANGULAR_SPEED:
            raise ValueError("max_angular_speed exceeds G1-D limit of 0.6 rad/s")
        if self.rpc_budget_s >= min(self.input_watchdog_s, self.state_timeout_s):
            raise ValueError("SDK discovery/response timeout budget must be shorter than both watchdogs")
        if isinstance(self.zero_frames, bool) or not isinstance(self.zero_frames, int) or self.zero_frames < 1:
            raise ValueError("zero_frames must be a positive integer")

    @property
    def rpc_budget_s(self) -> float:
        # SDK Write() can wait for discovery before waiting again for a reply.
        return 2.0 * self.sdk_timeout_s + 0.1


def fresh(age_s: float, timeout_s: float) -> bool:
    return math.isfinite(age_s) and 0.0 <= age_s <= timeout_s


class CommandGate:
    """Call under the client's lock; all ages use the monotonic clock.

    A watchdog or RPC fault disables control until an explicit re-enable.
    Stops discard commands and emit a finite zero burst, so an idle adapter
    does not continuously overwrite the robot's remote control.
    """

    def __init__(self, config: ControlConfig) -> None:
        self.config = config
        self.source = "disabled"
        self.emergency_stopped = False
        self.fault = ""
        self.odom_received_s = float("-inf")
        self.command_received_s = float("-inf")
        self.source_enabled_s = float("-inf")
        self.awaiting_command = True
        self.command = ZERO
        self.zero_remaining = 0

    def connected(self, now_s: float) -> bool:
        return fresh(now_s - self.odom_received_s, self.config.state_timeout_s)

    def mode(self, now_s: float) -> str:
        if self.emergency_stopped:
            return "STOPPED"
        if self.zero_remaining:
            return "STOPPING"
        if self.fault:
            return "FAULT"
        return "READY" if self.connected(now_s) else "UNKNOWN"

    def set_source(self, source: str, now_s: float) -> tuple[bool, str]:
        if source not in {"disabled", "manual", "auto"}:
            raise ValueError(f"invalid control source {source!r}")
        if source == "disabled":
            self.stop()
            return True, "control disabled; pending zero commands will finish"
        if self.emergency_stopped:
            return False, "software emergency stop is latched; reset it first"
        if self.zero_remaining:
            return False, "stop burst is pending; retry after it finishes"
        if not self.connected(now_s):
            return False, "fresh measured odometry is required"
        # Even re-selecting the same source invalidates the preceding command.
        self.source = source
        self.source_enabled_s = now_s
        self.command = ZERO
        self.command_received_s = now_s
        self.awaiting_command = True
        self.fault = ""
        return True, f"{source} control enabled"

    def receive_command(self, source: str, linear: float, angular: float, now_s: float) -> None:
        if source != self.source or self.source == "disabled":
            return
        if not all(math.isfinite(value) for value in (linear, angular, now_s)):
            self.stop("non-finite velocity command")
            return
        if now_s <= self.source_enabled_s or now_s < self.command_received_s:
            return
        self.command = (
            max(-self.config.max_linear_speed, min(self.config.max_linear_speed, linear)),
            max(-self.config.max_angular_speed, min(self.config.max_angular_speed, angular)),
        )
        self.command_received_s = now_s
        self.awaiting_command = False

    def stop(self, reason: str = "", *, force: bool = False) -> None:
        if force or self.source != "disabled":
            self.zero_remaining = max(self.zero_remaining, self.config.zero_frames)
        self.source = "disabled"
        self.command = ZERO
        self.command_received_s = float("-inf")
        if reason:
            self.fault = reason

    def emergency_stop(self) -> None:
        self.emergency_stopped = True
        self.stop(force=True)

    def reset_emergency_stop(self) -> tuple[bool, str]:
        if self.zero_remaining:
            return False, "stop burst is pending"
        self.emergency_stopped = False
        return True, "software stop latch reset; control remains disabled"

    def next_command(self, now_s: float) -> tuple[float, float] | None:
        if self.source != "disabled":
            if not self.connected(now_s):
                self.stop("odometry timed out")
            elif not self.awaiting_command and not fresh(
                now_s - self.command_received_s, self.config.input_watchdog_s
            ):
                self.stop("velocity command timed out")
            else:
                return self.command
        if self.zero_remaining:
            self.zero_remaining -= 1
            return ZERO
        return None


def validate_odometry(message, *, now_ns: int, timeout_s: float, odom_frame: str, base_frame: str) -> int:
    """Validate measured ROS odometry without importing ROS in CPU tests.

    Never integrate outgoing commands or replace a sensor's timestamp with
    receipt time. ROS and sensor clocks must be synchronized.
    """
    if message.header.frame_id != odom_frame or message.child_frame_id != base_frame:
        raise ValueError(f"odometry frames must be {odom_frame} -> {base_frame}")
    stamp = message.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if not 0 <= stamp.nanosec < 1_000_000_000 or stamp_ns <= 0:
        raise ValueError("odometry requires a valid nonzero timestamp")
    if not fresh((now_ns - stamp_ns) / 1e9, timeout_s):
        raise ValueError("odometry timestamp is stale or in the future")
    pose = message.pose.pose
    twist = message.twist.twist
    q = pose.orientation
    values = (
        pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
        *message.pose.covariance, *message.twist.covariance,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry contains non-finite values")
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if abs(norm - 1.0) > 0.01:
        raise ValueError("odometry quaternion must be normalized")
    return stamp_ns
