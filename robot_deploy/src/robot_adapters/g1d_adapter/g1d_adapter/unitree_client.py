"""G1-D AGV RPC bridge; SDK imports and connections are lazy.

Protocol: unitreerobotics/unitree_sdk2 include/unitree/robot/g1/agv/.
Only API 1001 is registered. This adapter never controls the lift or arms.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Callable

from .safety import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, CommandGate, ControlConfig

AGV_SERVICE = "agv"
AGV_VERSION = "1.0.0.1"
AGV_MOVE = 1001


class AgvRpc:
    def __init__(self, client) -> None:
        self.client = client
        # The spelling Verson is part of the official SDK2 Python interface.
        client._SetApiVerson(AGV_VERSION)
        client._RegistApi(AGV_MOVE, 0)

    def move(self, linear: float, angular: float) -> None:
        if not all(math.isfinite(value) for value in (linear, angular)):
            raise ValueError("non-finite G1-D velocity")
        if abs(linear) > MAX_LINEAR_SPEED or abs(angular) > MAX_ANGULAR_SPEED:
            raise ValueError("G1-D velocity exceeds mechanical limits")
        payload = json.dumps({"vx": linear, "vy": 0.0, "vyaw": angular}, allow_nan=False)
        result = self.client._Call(AGV_MOVE, payload)
        code = result[0] if isinstance(result, tuple) and result else result
        # None/booleans are not successful SDK acknowledgements.
        if isinstance(code, bool) or not isinstance(code, int) or code != 0:
            raise RuntimeError(f"G1-D Move failed: {result!r}")


def connect_rpc(network_interface: str, domain_id: int, timeout_s: float) -> AgvRpc:
    if not network_interface.strip() or domain_id < 0:
        raise ValueError("network_interface must be set and dds_domain_id nonnegative")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.rpc.client import Client
    except ImportError as exc:
        raise RuntimeError("g1d_adapter requires Unitree's official unitree_sdk2_python") from exc
    ChannelFactoryInitialize(domain_id, network_interface)
    client = Client(AGV_SERVICE, False)
    client.SetTimeout(timeout_s)
    return AgvRpc(client)


class G1DClient:
    """Serialize commands and stops on one watchdog-driven worker.

    ROS callbacks only update the gate. The SDK can wait separately for
    discovery and a response; the lock orders source changes after any in-flight call,
    so no previously selected movement can be sent after a completed stop
    request. No callbacks into ROS are made while holding the lock.
    """

    def __init__(
        self, rpc: AgvRpc, config: ControlConfig, *,
        sent_callback: Callable[[tuple[float, float]], None],
        error_callback: Callable[[str], None],
    ) -> None:
        self.rpc = rpc
        self.config = config
        self.gate = CommandGate(config)
        self._sent_callback = sent_callback
        self._error_callback = error_callback
        self._condition = threading.Condition()
        self._closed = False
        self._worker = threading.Thread(target=self._run, name="g1d-agv-command", daemon=True)
        self._worker.start()

    def receive_odometry(self, received_s: float) -> None:
        with self._condition:
            self.gate.odom_received_s = received_s

    def invalidate_odometry(self, reason: str) -> None:
        with self._condition:
            self.gate.odom_received_s = float("-inf")
            self.gate.stop(reason)
            self._condition.notify()

    def receive_command(
        self, source: str, linear: float, angular: float, *, received_s: float | None = None,
    ) -> None:
        with self._condition:
            if not self._closed:
                self.gate.receive_command(
                    source, linear, angular,
                    time.monotonic() if received_s is None else received_s,
                )

    def reject_command(self, source: str, reason: str) -> None:
        with self._condition:
            if self.gate.source == source:
                self.gate.stop(reason)
                self._condition.notify()

    def set_source(self, source: str) -> tuple[bool, str]:
        with self._condition:
            if self._closed:
                return False, "G1-D client is closed"
            result = self.gate.set_source(source, time.monotonic())
            self._condition.notify()
            return result

    def stop(self, *, emergency: bool = False) -> tuple[bool, str]:
        with self._condition:
            if self._closed:
                return False, "G1-D client is closed"
            if emergency:
                self.gate.emergency_stop()
            else:
                self.gate.stop(force=True)
            self._condition.notify()
        return True, "software stop queued; this is not a hardware emergency stop"

    def reset_emergency_stop(self) -> tuple[bool, str]:
        with self._condition:
            if self._closed:
                return False, "G1-D client is closed"
            return self.gate.reset_emergency_stop()

    def snapshot(self) -> dict:
        with self._condition:
            now_s = time.monotonic()
            return {
                "connected": self.gate.connected(now_s),
                "mode": self.gate.mode(now_s),
                "control_source": self.gate.source,
                "sdk_error": self.gate.fault,
                "software_stop_latched": self.gate.emergency_stopped,
            }

    def _run(self) -> None:
        while True:
            error = None
            command = None
            with self._condition:
                if self._closed and not self.gate.zero_remaining:
                    return
                command = self.gate.next_command(time.monotonic())
                if command is not None:
                    try:
                        self.rpc.move(*command)
                    except Exception as exc:
                        error = str(exc)
                        # A failure during a stop does not endlessly renew its burst.
                        self.gate.stop(error)
            if error is not None:
                self._error_callback(error)
            elif command is not None:
                self._sent_callback(command)
            with self._condition:
                self._condition.wait(timeout=1.0 / self.config.control_rate_hz)

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self.gate.stop()
            self._closed = True
            self._condition.notify()
        self._worker.join(timeout=(self.config.zero_frames + 1) * (
            self.config.rpc_budget_s + 1.0 / self.config.control_rate_hz
        ) + 1.0)
        if self._worker.is_alive():
            self._error_callback("G1-D RPC worker did not stop within its shutdown deadline")
