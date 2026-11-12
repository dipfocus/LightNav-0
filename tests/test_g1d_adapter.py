"""G1-D protocol and control regression tests; no ROS, SDK or hardware required."""

from __future__ import annotations

import importlib
import json
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def api(monkeypatch):
    package = Path(__file__).resolve().parents[1] / "robot_deploy/src/robot_adapters/g1d_adapter"
    monkeypatch.syspath_prepend(str(package))
    return SimpleNamespace(
        safety=importlib.import_module("g1d_adapter.safety"),
        transport=importlib.import_module("g1d_adapter.unitree_client"),
    )


@pytest.fixture
def gate(api):
    return api.safety.CommandGate(api.safety.ControlConfig())


def enable(gate, source="auto", now=10.0):
    gate.odom_received_s = now
    assert gate.set_source(source, now)[0]


def test_idle_adapter_does_not_send_commands_or_claim_readiness(gate):
    assert gate.source == "disabled"
    assert gate.mode(10.0) == "UNKNOWN"
    assert gate.next_command(10.0) is None
    assert not gate.set_source("auto", 10.0)[0]
    gate.receive_command("manual", 0.2, 0.1, 10.0)
    assert gate.next_command(10.0) is None
    gate.stop()
    assert gate.next_command(10.0) is None


def test_source_switch_discards_previous_source_and_previous_command(gate):
    enable(gate, "manual")
    gate.receive_command("manual", 0.2, -0.1, 10.01)
    assert gate.next_command(10.02) == (0.2, -0.1)
    gate.receive_command("auto", 0.3, 0.3, 10.02)
    assert gate.next_command(10.03) == (0.2, -0.1)
    assert gate.set_source("auto", 10.04)[0]
    assert gate.next_command(10.04) == (0.0, 0.0)
    gate.receive_command("manual", 0.3, 0.3, 10.05)
    assert gate.next_command(10.05) == (0.0, 0.0)
    gate.receive_command("auto", -0.1, 0.2, 10.06)
    assert gate.next_command(10.06) == (-0.1, 0.2)
    assert gate.set_source("auto", 10.07)[0]
    assert gate.next_command(10.07) == (0.0, 0.0)


def test_configured_speed_limits_apply_to_both_axes_and_signs(gate):
    enable(gate)
    gate.receive_command("auto", 20.0, -20.0, 10.01)
    assert gate.next_command(10.01) == (0.3, -0.3)
    gate.receive_command("auto", -20.0, 20.0, 10.02)
    assert gate.next_command(10.02) == (-0.3, 0.3)


def test_waiting_for_first_inference_holds_zero_without_command_timeout(gate):
    enable(gate)
    gate.odom_received_s = 20.0
    assert gate.next_command(20.0) == (0.0, 0.0)
    assert gate.source == "auto"
    gate.receive_command("auto", 0.2, 0.1, 20.01)
    assert gate.next_command(20.02) == (0.2, 0.1)
    gate.odom_received_s = 20.4
    assert gate.next_command(20.4) == (0.0, 0.0)
    assert gate.source == "disabled"
    assert "command timed out" in gate.fault


def test_odometry_watchdog_applies_before_first_command(gate):
    enable(gate)
    assert gate.next_command(10.6) == (0.0, 0.0)
    assert gate.source == "disabled"
    assert "odometry timed out" in gate.fault


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_command_disables_control_and_sends_only_zero(gate, value):
    enable(gate)
    gate.receive_command("auto", value, 0.0, 10.01)
    assert gate.source == "disabled"
    assert gate.mode(10.01) == "STOPPING"
    for _ in range(gate.config.zero_frames):
        assert gate.next_command(10.01) == (0.0, 0.0)
    assert gate.mode(10.01) == "FAULT"


@pytest.mark.parametrize("stale_input", ["command", "odometry"])
def test_watchdogs_stop_and_require_explicit_rearming(gate, stale_input):
    enable(gate)
    gate.receive_command("auto", 0.2, 0.1, 10.01)
    now = 10.6
    if stale_input == "command":
        gate.odom_received_s = now
    else:
        gate.receive_command("auto", 0.2, 0.1, now)
    assert gate.next_command(now) == (0.0, 0.0)
    assert gate.source == "disabled"
    assert stale_input in gate.fault
    gate.odom_received_s = now
    gate.receive_command("auto", 0.2, 0.1, now)
    assert not gate.set_source("auto", now)[0]
    for _ in range(gate.config.zero_frames - 1):
        assert gate.next_command(now) == (0.0, 0.0)
    assert gate.next_command(now) is None
    assert gate.source == "disabled"
    assert gate.set_source("auto", now)[0]
    assert gate.fault == ""
    assert gate.next_command(now) == (0.0, 0.0)


def test_future_feedback_and_future_commands_are_not_fresh(gate):
    gate.odom_received_s = 11.0
    assert not gate.connected(10.0)
    assert not gate.set_source("manual", 10.0)[0]
    enable(gate)
    gate.receive_command("auto", 0.2, 0.1, 11.0)
    assert gate.next_command(10.0) == (0.0, 0.0)
    assert gate.source == "disabled"


def test_repeated_disable_does_not_extend_stop_burst(gate):
    enable(gate)
    gate.receive_command("auto", 0.2, 0.1, 10.01)
    gate.stop()
    for _ in range(gate.config.zero_frames):
        assert gate.next_command(10.0) == (0.0, 0.0)
        assert gate.set_source("disabled", 10.0)[0]
    assert gate.next_command(10.0) is None


def test_command_queued_before_reenable_cannot_restart_motion(gate):
    enable(gate, "manual")
    gate.receive_command("manual", 0.2, 0.1, 10.01)
    gate.stop()
    for _ in range(gate.config.zero_frames):
        gate.next_command(10.02)
    assert gate.set_source("manual", 10.03)[0]
    # All three arrivals are still inside the watchdog window, but originate
    # before this control session. Equality is also rejected conservatively.
    for stamp in (10.01, 10.02, 10.03):
        gate.receive_command("manual", 0.2, 0.1, stamp)
        assert gate.next_command(10.04) == (0.0, 0.0)
    gate.receive_command("manual", -0.1, 0.2, 10.05)
    assert gate.next_command(10.06) == (-0.1, 0.2)


def test_delayed_command_does_not_replace_newer_command(gate):
    enable(gate)
    gate.receive_command("auto", 0.1, -0.2, 10.10)
    gate.receive_command("auto", 0.3, 0.3, 10.05)
    assert gate.next_command(10.11) == (0.1, -0.2)


def test_software_emergency_stop_requires_reset_and_separate_enable(gate):
    enable(gate)
    gate.emergency_stop()
    assert gate.mode(10.0) == "STOPPED"
    assert not gate.set_source("manual", 10.0)[0]
    assert not gate.reset_emergency_stop()[0]
    for _ in range(gate.config.zero_frames):
        assert gate.next_command(10.0) == (0.0, 0.0)
    assert not gate.set_source("auto", 10.0)[0]
    assert gate.reset_emergency_stop()[0]
    assert gate.source == "disabled"
    assert gate.next_command(10.0) is None
    assert gate.set_source("manual", 10.0)[0]


@pytest.mark.parametrize("kwargs", [
    {"max_linear_speed": 1.5001}, {"max_angular_speed": 0.6001},
    {"max_linear_speed": float("nan")}, {"control_rate_hz": float("inf")},
    {"state_timeout_s": 0.0}, {"input_watchdog_s": -1.0},
    {"sdk_timeout_s": 0.35}, {"zero_frames": 0},
    {"zero_frames": True}, {"zero_frames": 1.5},
])
def test_invalid_configuration_fails_before_hardware_connection(api, kwargs):
    with pytest.raises(ValueError):
        api.safety.ControlConfig(**kwargs)


class FakeSdkClient:
    def __init__(self, result=(0, "")):
        self.result = result
        self.calls = []

    def _SetApiVerson(self, version):
        self.calls.append(("version", version))

    def _RegistApi(self, api_id, priority):
        self.calls.append(("register", api_id, priority))

    def _Call(self, api_id, payload):
        self.calls.append(("call", api_id, json.loads(payload)))
        return self.result


def test_rpc_registers_only_chassis_move_and_uses_correct_axes(api):
    sdk = FakeSdkClient()
    rpc = api.transport.AgvRpc(sdk)
    rpc.move(0.25, -0.1)
    assert sdk.calls == [
        ("version", "1.0.0.1"),
        ("register", 1001, 0),
        ("call", 1001, {"vx": 0.25, "vy": 0.0, "vyaw": -0.1}),
    ]


def test_connection_uses_agv_service_without_a_go2_lease(api, monkeypatch):
    import sys

    calls = []

    class Client(FakeSdkClient):
        def __init__(self, service, lease):
            super().__init__()
            calls.append(("client", service, lease))

        def SetTimeout(self, timeout):
            calls.append(("timeout", timeout))

    channel = ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda domain, interface: calls.append(("channel", domain, interface))
    rpc_module = ModuleType("unitree_sdk2py.rpc.client")
    rpc_module.Client = Client
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.rpc.client", rpc_module)
    rpc = api.transport.connect_rpc("test0", 3, 0.2)
    assert calls == [("channel", 3, "test0"), ("client", "agv", False), ("timeout", 0.2)]
    assert rpc.client.calls == [("version", "1.0.0.1"), ("register", 1001, 0)]


@pytest.mark.parametrize("linear, angular", [
    (float("nan"), 0.0), (0.0, float("inf")),
    (1.5001, 0.0), (-1.5001, 0.0), (0.0, 0.6001), (0.0, -0.6001),
])
def test_rpc_rejects_invalid_commands_without_sending(api, linear, angular):
    sdk = FakeSdkClient()
    rpc = api.transport.AgvRpc(sdk)
    with pytest.raises(ValueError):
        rpc.move(linear, angular)
    assert len(sdk.calls) == 2


@pytest.mark.parametrize("result", [(7, "error"), -1, None, False, True, (), "0"])
def test_rpc_requires_an_explicit_success_result(api, result):
    rpc = api.transport.AgvRpc(FakeSdkClient(result))
    with pytest.raises(RuntimeError, match="Move failed"):
        rpc.move(0.1, 0.1)


def make_odometry():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=SimpleNamespace(sec=10, nanosec=0)),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.1, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.1),
            ),
            covariance=[0.0] * 36,
        ),
    )


def validate_odometry(api, message, now_ns=10_100_000_000):
    return api.safety.validate_odometry(
        message, now_ns=now_ns, timeout_s=0.5, odom_frame="odom", base_frame="base_link",
    )


def test_measured_odometry_preserves_original_sensor_stamp(api):
    message = make_odometry()
    assert validate_odometry(api, message) == 10_000_000_000
    assert message.header.stamp.sec == 10
    assert message.header.stamp.nanosec == 0


@pytest.mark.parametrize("now_ns", [9_900_000_000, 10_500_000_001])
def test_odometry_from_wrong_time_window_is_rejected(api, now_ns):
    with pytest.raises(ValueError, match="stale or in the future"):
        validate_odometry(api, make_odometry(), now_ns)


@pytest.mark.parametrize("mutation", [
    lambda m: setattr(m.header, "frame_id", "map"),
    lambda m: setattr(m, "child_frame_id", "camera"),
    lambda m: setattr(m, "child_frame_id", ""),
    lambda m: setattr(m.header.stamp, "sec", 0),
    lambda m: setattr(m.header.stamp, "nanosec", 1_000_000_000),
    lambda m: setattr(m.pose.pose.orientation, "w", 0.0),
    lambda m: setattr(m.pose.pose.position, "x", float("nan")),
    lambda m: setattr(m.twist.twist.linear, "x", float("inf")),
    lambda m: m.pose.covariance.__setitem__(0, float("nan")),
])
def test_invalid_measured_odometry_is_rejected(api, mutation):
    message = make_odometry()
    mutation(message)
    with pytest.raises(ValueError):
        validate_odometry(api, message)


class RecordingRpc:
    def __init__(self, fail_nonzero=False, fail_all=False):
        self.calls = []
        self.fail_nonzero = fail_nonzero
        self.fail_all = fail_all
        self.changed = threading.Condition()

    def move(self, linear, angular):
        with self.changed:
            self.calls.append((linear, angular))
            self.changed.notify_all()
        if self.fail_all or (self.fail_nonzero and (linear or angular)):
            raise RuntimeError("injected RPC failure")

    def wait_for(self, predicate):
        with self.changed:
            assert self.changed.wait_for(lambda: predicate(self.calls), timeout=2.0)


@pytest.fixture
def client_factory(api):
    clients = []

    def create(rpc):
        errors = []
        sent = []
        client = api.transport.G1DClient(
            rpc,
            api.safety.ControlConfig(
                control_rate_hz=100.0, state_timeout_s=5.0, input_watchdog_s=5.0,
            ),
            sent_callback=sent.append,
            error_callback=errors.append,
        )
        clients.append(client)
        return client, errors, sent

    yield create
    for client in clients:
        client.close()


def test_never_enabled_worker_closes_without_writing_to_chassis(client_factory):
    rpc = RecordingRpc()
    client, errors, sent = client_factory(rpc)
    client.close()
    assert rpc.calls == []
    assert errors == sent == []
    assert not client._worker.is_alive()


def test_shutdown_sends_bounded_zero_burst_and_rejects_future_commands(client_factory):
    rpc = RecordingRpc()
    client, errors, _ = client_factory(rpc)
    client.receive_odometry(time.monotonic())
    assert client.set_source("auto")[0]
    client.receive_command("auto", 0.2, 0.1)
    rpc.wait_for(lambda calls: (0.2, 0.1) in calls)
    client.close()
    final_nonzero = max(i for i, command in enumerate(rpc.calls) if command != (0.0, 0.0))
    assert rpc.calls[final_nonzero + 1:] == [(0.0, 0.0)] * client.config.zero_frames
    assert not client._worker.is_alive()
    assert not client.set_source("manual")[0]
    assert not client.stop()[0]
    before = list(rpc.calls)
    client.receive_command("auto", 0.3, 0.3)
    client.close()
    assert rpc.calls == before
    assert errors == []


def test_rpc_fault_disables_motion_until_explicit_rearm(client_factory):
    rpc = RecordingRpc(fail_nonzero=True)
    client, errors, _ = client_factory(rpc)
    client.receive_odometry(time.monotonic())
    assert client.set_source("auto")[0]
    client.receive_command("auto", 0.2, 0.1)
    rpc.wait_for(lambda calls: (0.2, 0.1) in calls and (
        len(calls) - calls.index((0.2, 0.1)) - 1 >= client.config.zero_frames
    ))
    assert client.snapshot()["control_source"] == "disabled"
    assert client.snapshot()["mode"] == "FAULT"
    client.receive_command("auto", 0.3, 0.3)
    assert client.gate.command == (0.0, 0.0)
    rpc.fail_nonzero = False
    client.receive_odometry(time.monotonic())
    assert client.set_source("manual")[0]
    client.receive_command("manual", -0.1, 0.2)
    rpc.wait_for(lambda calls: (-0.1, 0.2) in calls)
    client.close()
    nonzero = [command for command in rpc.calls if command != (0.0, 0.0)]
    assert nonzero[0] == (0.2, 0.1)
    assert all(command == (-0.1, 0.2) for command in nonzero[1:])
    assert errors == ["injected RPC failure"]


def test_failed_stop_burst_is_bounded_and_error_is_reported(client_factory):
    rpc = RecordingRpc(fail_all=True)
    client, errors, sent = client_factory(rpc)
    assert client.stop(emergency=True)[0]
    client.close()
    assert rpc.calls == [(0.0, 0.0)] * client.config.zero_frames
    assert len(errors) == client.config.zero_frames
    assert sent == []
    assert client.snapshot()["software_stop_latched"]
    assert not client._worker.is_alive()


def test_stop_serializes_with_inflight_move_and_discards_subsequent_input(client_factory):
    entered = threading.Event()
    release = threading.Event()
    stop_attempted = threading.Event()
    stop_finished = threading.Event()

    class BlockingRpc(RecordingRpc):
        def move(self, linear, angular):
            super().move(linear, angular)
            if linear or angular:
                entered.set()
                if not release.wait(timeout=2.0):
                    raise RuntimeError("test did not release blocked RPC")

    rpc = BlockingRpc()
    client, errors, _ = client_factory(rpc)
    client.receive_odometry(time.monotonic())
    assert client.set_source("manual")[0]
    client.receive_command("manual", 0.2, 0.1)
    assert entered.wait(timeout=2.0)

    def stop():
        stop_attempted.set()
        client.stop()
        stop_finished.set()

    stopper = threading.Thread(target=stop)
    stopper.start()
    try:
        assert stop_attempted.wait(timeout=2.0)
        # An in-flight move cannot be overtaken by a concurrently issued stop.
        assert not stop_finished.is_set()
    finally:
        release.set()
    assert stop_finished.wait(timeout=2.0)
    stopper.join(timeout=2.0)
    after_stop = len(rpc.calls)
    client.receive_command("manual", 0.3, 0.3)
    client.close()
    assert all(command == (0.0, 0.0) for command in rpc.calls[after_stop:])
    assert rpc.calls[-client.config.zero_frames:] == [(0.0, 0.0)] * client.config.zero_frames
    assert errors == []
