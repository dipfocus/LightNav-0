"""Native G1-D measured-odometry bridge tests without ROS, DDS or hardware."""

from __future__ import annotations

import builtins
import importlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "robot_deploy/src/robot_adapters/g1d_adapter"


@pytest.fixture
def api(monkeypatch):
    monkeypatch.syspath_prepend(str(PACKAGE))
    return SimpleNamespace(
        odometry=importlib.import_module("g1d_adapter.odometry"),
        safety=importlib.import_module("g1d_adapter.safety"),
    )


def vector(x=0.0, y=0.0, z=0.0, **extra):
    return SimpleNamespace(x=x, y=y, z=z, **extra)


def native_odometry(stamp_ns=10_000_000_000, *, frame="odom", child="base_link"):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
        ),
        child_frame_id=child,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(1.0, -2.0, 0.3),
                orientation=vector(0.0, 0.0, math.sin(0.4), w=math.cos(0.4)),
            ),
            covariance=[index / 1000.0 for index in range(36)],
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(0.1, -0.02, 0.03),
                angular=vector(-0.001, 0.002, -0.2),
            ),
            covariance=[index / 2000.0 for index in range(36)],
        ),
    )


@pytest.mark.parametrize("child", ["base_link", ""])
def test_preserves_measured_pose_twist_covariance_and_sampling_time(api, child):
    original = native_odometry(10_123_456_789, child=child)
    sample = api.odometry.OdometryConverter().convert(original, now_ns=10_200_000_000)
    assert sample.source_stamp_ns == sample.stamp_ns == 10_123_456_789
    assert (sample.odom_frame, sample.base_frame) == ("odom", "base_link")
    assert sample.position == (1.0, -2.0, 0.3)
    assert sample.orientation == (0.0, 0.0, math.sin(0.4), math.cos(0.4))
    assert sample.linear == (0.1, -0.02, 0.03)
    assert sample.angular == (-0.001, 0.002, -0.2)
    assert sample.pose_covariance == tuple(original.pose.covariance)
    assert sample.twist_covariance == tuple(original.twist.covariance)
    # A DDS implementation may reuse its message object after the callback.
    original.pose.pose.position.x = 99.0
    original.twist.twist.angular.z = 99.0
    original.pose.covariance[0] = 99.0
    assert sample.position[0] == 1.0
    assert sample.angular[2] == -0.2
    assert sample.pose_covariance[0] == 0.0


def test_stationary_measured_pose_is_not_replaced_by_command_integration(api):
    converter = api.odometry.OdometryConverter()
    first = native_odometry()
    first.twist.twist.linear = vector()
    first.twist.twist.angular = vector()
    second = native_odometry(10_100_000_000)
    second.twist.twist.linear = vector()
    second.twist.twist.angular = vector()
    a = converter.convert(first, now_ns=10_010_000_000)
    b = converter.convert(second, now_ns=10_110_000_000)
    assert a.position == b.position == (1.0, -2.0, 0.3)
    assert a.orientation == b.orientation
    assert a.linear == b.linear == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("base_frame", ["base_link", "robot_base"])
def test_empty_native_child_frame_uses_configured_body_frame(api, base_frame):
    original = native_odometry(child="")
    sample = api.odometry.OdometryConverter(base_frame=base_frame).convert(
        original, now_ns=10_010_000_000,
    )
    assert sample.base_frame == base_frame
    assert original.child_frame_id == ""


@pytest.mark.parametrize("frame, child", [
    ("map", "base_link"), ("", "base_link"), ("odom", "camera"),
    ("map", ""), ("", ""), ("odom", None), ("odom", " "),
])
def test_frame_mismatch_is_not_fixed_by_relabeling(api, frame, child):
    converter = api.odometry.OdometryConverter()
    with pytest.raises(ValueError, match="frame"):
        converter.convert(native_odometry(frame=frame, child=child), now_ns=10_010_000_000)


@pytest.mark.parametrize("offset_s, source_ns", [(2.0, 8_000_000_000), (-2.0, 12_000_000_000)])
def test_explicit_clock_offset_preserves_source_stamp_and_transport_age(api, offset_s, source_ns):
    converter = api.odometry.OdometryConverter(clock_offset_s=offset_s)
    original = native_odometry(source_ns)
    sample = converter.convert(original, now_ns=10_120_000_000)
    assert sample.source_stamp_ns == source_ns
    assert sample.stamp_ns == 10_000_000_000
    assert original.header.stamp.sec * 1_000_000_000 + original.header.stamp.nanosec == source_ns
    next_sample = converter.convert(native_odometry(source_ns + 60_000_000), now_ns=10_190_000_000)
    assert next_sample.stamp_ns - sample.stamp_ns == 60_000_000


@pytest.mark.parametrize("now_ns", [9_999_999_999, 10_500_000_001])
def test_future_and_stale_source_samples_are_not_restamped_to_now(api, now_ns):
    converter = api.odometry.OdometryConverter()
    with pytest.raises(ValueError, match="stale or future"):
        converter.convert(native_odometry(), now_ns=now_ns)
    assert converter.last_source_stamp_ns == 0


@pytest.mark.parametrize("stamp_ns", [10_000_000_000, 9_900_000_000, 1_000_000_000])
def test_duplicates_out_of_order_and_source_clock_restart_do_not_refresh_feedback(api, stamp_ns):
    converter = api.odometry.OdometryConverter()
    accepted = converter.convert(native_odometry(), now_ns=10_010_000_000)
    with pytest.raises(ValueError, match="duplicate or out-of-order"):
        converter.convert(native_odometry(stamp_ns), now_ns=10_020_000_000)
    assert converter.last_source_stamp_ns == accepted.source_stamp_ns
    # A later genuinely new sample still works after the rejected input.
    recovered = converter.convert(native_odometry(10_030_000_000), now_ns=10_040_000_000)
    assert recovered.stamp_ns == 10_030_000_000


def test_deliberate_bridge_restart_and_offset_can_admit_a_new_source_epoch(api):
    old = api.odometry.OdometryConverter()
    old.convert(native_odometry(), now_ns=10_010_000_000)
    reset_source = native_odometry(1_000_000_000)
    with pytest.raises(ValueError):
        old.convert(reset_source, now_ns=11_010_000_000)
    restarted = api.odometry.OdometryConverter(clock_offset_s=10.0)
    sample = restarted.convert(reset_source, now_ns=11_010_000_000)
    assert sample.source_stamp_ns == 1_000_000_000
    assert sample.stamp_ns == 11_000_000_000


@pytest.mark.parametrize("mutate", [
    lambda m: setattr(m.header.stamp, "sec", -1),
    lambda m: setattr(m.header.stamp, "sec", 10.0),
    lambda m: setattr(m.header.stamp, "nanosec", -1),
    lambda m: setattr(m.header.stamp, "nanosec", 1_000_000_000),
    lambda m: setattr(m.header.stamp, "nanosec", 0.5),
    lambda m: setattr(m.header.stamp, "sec", 0),
    lambda m: setattr(m.pose.pose.position, "x", float("nan")),
    lambda m: setattr(m.twist.twist.linear, "y", float("inf")),
    lambda m: setattr(m.twist.twist.angular, "z", float("-inf")),
    lambda m: setattr(m.pose.pose, "orientation", vector(w=0.0)),
    lambda m: setattr(m.pose.pose.orientation, "w", 2.0),
    lambda m: setattr(m.pose, "covariance", [0.0] * 35),
    lambda m: setattr(m.twist, "covariance", [0.0] * 37),
    lambda m: m.pose.covariance.__setitem__(0, float("nan")),
    lambda m: m.twist.covariance.__setitem__(35, float("inf")),
])
def test_bad_native_values_are_rejected_without_poisoning_sequence(api, mutate):
    converter = api.odometry.OdometryConverter()
    message = native_odometry()
    mutate(message)
    with pytest.raises(ValueError):
        converter.convert(message, now_ns=10_010_000_000)
    assert converter.last_source_stamp_ns == 0
    assert converter.convert(native_odometry(), now_ns=10_010_000_000).stamp_ns == 10_000_000_000


@pytest.mark.parametrize("kwargs", [
    {"odom_frame": ""}, {"base_frame": ""}, {"base_frame": "odom"},
    {"clock_offset_s": float("nan")}, {"clock_offset_s": float("inf")},
    {"max_age_s": 0.0}, {"max_age_s": -1.0}, {"max_age_s": float("inf")},
])
def test_invalid_converter_configuration_is_rejected(api, kwargs):
    with pytest.raises(ValueError):
        api.odometry.OdometryConverter(**kwargs)


@pytest.mark.parametrize("offset_s", [-10.0, float(2**31)])
def test_corrected_time_must_fit_ros_stamp_range(api, offset_s):
    with pytest.raises(ValueError, match="ROS Time range"):
        api.odometry.OdometryConverter(clock_offset_s=offset_s).convert(
            native_odometry(), now_ns=10_010_000_000,
        )


@pytest.mark.parametrize("base_frame", ["base_link", "robot_base"])
def test_populated_ros_message_passes_adapter_validation_and_keeps_sensor_age(api, base_frame):
    original = native_odometry(10_123_456_789, child="")
    sample = api.odometry.OdometryConverter(base_frame=base_frame).convert(
        original, now_ns=10_150_000_000,
    )
    output = native_odometry(1_000_000_000, frame="unused", child="unused")
    assert api.odometry.populate_odometry(output, sample) is output
    assert api.safety.validate_odometry(
        output, now_ns=10_200_000_000, timeout_s=0.5, odom_frame="odom", base_frame=base_frame,
    ) == 10_123_456_789
    assert output.child_frame_id == base_frame
    assert original.child_frame_id == ""
    assert original.header.stamp.sec == 10
    assert original.header.stamp.nanosec == 123_456_789
    assert output.header.stamp.sec == 10
    assert output.header.stamp.nanosec == 123_456_789
    assert output.pose.pose.position.x == 1.0
    assert output.twist.twist.linear.y == -0.02
    assert output.pose.covariance == list(sample.pose_covariance)
    assert output.twist.covariance == list(sample.twist_covariance)
    output.pose.covariance[0] = 999.0
    assert sample.pose_covariance[0] == 0.0
    with pytest.raises(ValueError, match="stale"):
        api.safety.validate_odometry(
            output, now_ns=10_700_000_000, timeout_s=0.5, odom_frame="odom", base_frame=base_frame,
        )


def test_module_import_does_not_import_ros_or_the_unitree_sdk(monkeypatch):
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        assert not name.startswith(("unitree_sdk2py", "rclpy", "nav_msgs"))
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    name = "_g1d_odometry_lazy_import_test"
    spec = importlib.util.spec_from_file_location(name, PACKAGE / "g1d_adapter/odometry.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    assert module.OdometryConverter
    assert module.DdsOdometrySubscriber


@pytest.fixture
def sdk(monkeypatch):
    calls = []
    subscriptions = []
    native_type = type("Odometry_", (), {})

    class Subscription:
        def __init__(self, topic, message_type):
            calls.append(("subscribe", topic, message_type))
            subscriptions.append(self)
            self.callback = None
            self.fail_init = False

        def Init(self, callback, depth):
            self.callback = callback
            calls.append(("init", depth))
            if self.fail_init:
                raise RuntimeError("injected subscription init failure")

        def Close(self):
            calls.append(("close",))
            # A callback during shutdown must neither deadlock nor refill the mailbox.
            self.callback(native_odometry())

    channel = ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda domain, interface: calls.append(("channel", domain, interface))
    channel.ChannelSubscriber = Subscription
    native = ModuleType("unitree_sdk2py.idl.nav_msgs.msg.dds_")
    native.Odometry_ = native_type
    monkeypatch.setitem(sys.modules, channel.__name__, channel)
    monkeypatch.setitem(sys.modules, native.__name__, native)
    # Any attempt to acquire a publisher or motion RPC fails the test immediately.
    def forbidden(*args, **kwargs):
        pytest.fail("read-only odometry tried to create a publisher or RPC client")

    channel.ChannelPublisher = forbidden
    rpc = ModuleType("unitree_sdk2py.rpc.client")
    rpc.Client = forbidden
    monkeypatch.setitem(sys.modules, rpc.__name__, rpc)
    return SimpleNamespace(calls=calls, subscriptions=subscriptions, native_type=native_type, cls=Subscription)


def test_dds_subscription_uses_native_type_latest_only_queue_and_no_motion_api(api, sdk):
    subscriber = api.odometry.DdsOdometrySubscriber("test0", 3)
    assert sdk.calls == [
        ("channel", 3, "test0"),
        ("subscribe", "rt/agv/odom", sdk.native_type),
        ("init", 1),
    ]
    callback = sdk.subscriptions[0].callback
    assert subscriber.take() is None
    callback(native_odometry(10_000_000_000))
    latest = native_odometry(10_060_000_000)
    callback(latest)
    latest.pose.pose.position.x = 99.0
    received = subscriber.take()
    assert received.header.stamp.nanosec == 60_000_000
    assert received.pose.pose.position.x == 1.0
    assert subscriber.take() is None
    callback(native_odometry(10_120_000_000))
    subscriber.close()
    subscriber.close()
    assert subscriber.take() is None
    callback(native_odometry(10_180_000_000))
    assert subscriber.take() is None
    assert sdk.calls.count(("close",)) == 1


def test_dds_init_failure_closes_the_partially_open_subscription(api, sdk, monkeypatch):
    original_init = sdk.cls.Init

    def failed_init(self, callback, depth):
        self.fail_init = True
        original_init(self, callback, depth)

    monkeypatch.setattr(sdk.cls, "Init", failed_init)
    with pytest.raises(RuntimeError, match="injected"):
        api.odometry.DdsOdometrySubscriber("test0")
    assert sdk.calls[-1] == ("close",)


@pytest.fixture
def node_factory(api, monkeypatch):
    state = SimpleNamespace(
        now_ns=10_100_000_000, incoming=None, output=[], status=[], warnings=[],
        parameters={"network_interface": "test0"},
        namespace="", remappings={}, publishers=[], subscriptions=[], closes=[],
    )

    def take():
        message, state.incoming = state.incoming, None
        return message

    class RosNode:
        # Deliberately no resolve_topic_name: the bridge also supports Foxy.
        def __init__(self, name):
            self.parameters = {}

        def declare_parameter(self, name, default):
            self.parameters[name] = state.parameters.get(name, default)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, message_type, topic, depth):
            resolved = topic if topic.startswith("/") else f"{state.namespace}/{topic}"
            resolved = state.remappings.get(resolved, resolved)
            publisher = SimpleNamespace(
                topic_name=resolved,
                publish=(
                    state.output.append if message_type is native_odometry
                    else lambda message: state.status.append(json.loads(message.data))
                ),
            )
            state.publishers.append(publisher)
            return publisher

        def create_timer(self, period, callback):
            return None

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=state.now_ns))

        def get_logger(self):
            return SimpleNamespace(info=lambda _: None, warning=state.warnings.append)

        def destroy_node(self):
            return True

    modules = {
        "rclpy": {},
        "rclpy.executors": {"ExternalShutdownException": RuntimeError},
        "rclpy.node": {"Node": RosNode},
        "rclpy.parameter": {"Parameter": SimpleNamespace(Type=SimpleNamespace(STRING=object()))},
        "nav_msgs.msg": {"Odometry": native_odometry},
        "std_msgs.msg": {"String": SimpleNamespace},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "g1d_adapter._odometry_node_test", PACKAGE / "g1d_adapter/odometry_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def subscriber(interface, domain, topic):
        state.subscriptions.append((interface, domain, topic))
        return SimpleNamespace(take=take, close=lambda: state.closes.append(topic))

    monkeypatch.setattr(module, "DdsOdometrySubscriber", subscriber)
    return module.G1DOdometryNode, state


@pytest.fixture
def node_bridge(node_factory):
    create_node, state = node_factory
    return create_node(), state


@pytest.mark.parametrize("namespace, output_topic, remapping", [
    ("", "/agv/odom", {}),
    ("/agv", "odom", {}),
    ("/nav", "g1d/odom", {"/nav/g1d/odom": "/agv/odom"}),
])
def test_node_rejects_resolved_or_remapped_source_loop_before_dds_connect(
    node_factory, namespace, output_topic, remapping,
):
    create_node, state = node_factory
    state.namespace = namespace
    state.parameters["odom_topic"] = output_topic
    state.remappings = remapping
    with pytest.raises(ValueError, match="must not resolve to the native DDS source"):
        create_node()
    assert state.publishers[0].topic_name == "/agv/odom"
    assert state.subscriptions == []
    assert state.output == []


def test_node_connects_configured_readonly_source_and_closes_it_once(node_factory):
    create_node, state = node_factory
    state.namespace = "/nav"
    state.parameters.update({
        "network_interface": " test1 ", "dds_domain_id": 3,
        "dds_topic": "rt/vendor/agv/odom", "odom_topic": "measured",
    })
    node = create_node()
    assert state.subscriptions == [("test1", 3, "rt/vendor/agv/odom")]
    assert node.publisher.topic_name == "/nav/measured"
    assert state.output == []
    node.destroy_node()
    node.destroy_node()
    assert state.closes == ["rt/vendor/agv/odom"]


def test_node_publishes_each_native_sample_once_and_does_not_mask_sensor_loss(node_bridge, api):
    node, state = node_bridge
    state.incoming = native_odometry(child="")
    node._poll()
    node._poll()
    assert len(state.output) == 1
    output = state.output[0]
    assert api.safety.validate_odometry(
        output, now_ns=state.now_ns, timeout_s=0.5, odom_frame="odom", base_frame="base_link",
    ) == 10_000_000_000
    node._publish_status()
    assert state.status[-1]["connected"]
    assert state.status[-1]["age_s"] == pytest.approx(0.1)
    state.now_ns = 10_600_000_000
    node._poll()
    node._publish_status()
    assert len(state.output) == 1
    assert not state.status[-1]["connected"]
    assert state.status[-1]["error"] == "native odometry is stale"
    assert state.status[-1]["accepted"] == 1
    assert output.header.stamp.sec == 10


def test_node_rejects_bad_and_repeated_samples_then_recovers_on_new_measurement(node_bridge):
    node, state = node_bridge
    state.incoming = native_odometry(frame="map", child="")
    node._poll()
    assert state.output == []
    assert node._rejected == 1
    assert state.warnings
    state.incoming = native_odometry(child="")
    node._poll()
    state.incoming = native_odometry(child="")
    node._poll()
    assert len(state.output) == 1
    assert node._rejected == 2
    state.incoming = native_odometry(10_050_000_000, child="")
    node._poll()
    node._publish_status()
    assert len(state.output) == 2
    assert state.status[-1]["accepted"] == 2
    assert state.status[-1]["error"] == ""
    assert state.output[-1].header.stamp.nanosec == 50_000_000
