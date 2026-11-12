"""Exercise the ROS bridge callbacks without ROS or a robot installed."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


def load_adapter(monkeypatch, *, node_class=object, parameter_class=object, qos=None):
    package_dir = Path(__file__).parents[1] / "robot_deploy/src/robot_adapters/g1d_adapter"
    monkeypatch.syspath_prepend(str(package_dir))
    modules = {
        "rclpy": {},
        "rclpy.executors": {"ExternalShutdownException": RuntimeError},
        "rclpy.node": {"Node": node_class},
        "rclpy.parameter": {"Parameter": parameter_class},
        "rclpy.qos": qos or dict.fromkeys(
            ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy", "qos_profile_sensor_data"), object
        ),
        "diagnostic_msgs.msg": dict.fromkeys(("DiagnosticArray", "DiagnosticStatus", "KeyValue"), object),
        "geometry_msgs.msg": dict.fromkeys(("TransformStamped", "TwistStamped"), object),
        "nav_msgs.msg": {"Odometry": object},
        "std_msgs.msg": {"String": object},
        "std_srvs.srv": dict.fromkeys(("SetBool", "Trigger"), object),
        "tf2_ros": {"TransformBroadcaster": object},
    }
    for name, attrs in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "g1d_adapter._node_test", package_dir / "g1d_adapter/adapter_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bridge(monkeypatch):
    module = load_adapter(monkeypatch)
    node = module.G1DAdapterNode.__new__(module.G1DAdapterNode)
    node.config = module.ControlConfig()
    node.odom_frame = "odom"
    node.base_frame = "base_link"
    node._last_stamp_ns = 0
    node._source_enabled_ns = 0
    node.tf_broadcaster = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    node._publish_status = lambda: None
    monkeypatch.setattr(module.time, "monotonic", lambda: 20.0)
    calls = SimpleNamespace(odom=[], invalid=[], commands=[], rejected=[], published=[])
    node._client = SimpleNamespace(
        receive_odometry=calls.odom.append,
        invalidate_odometry=calls.invalid.append,
        receive_command=lambda *args, **kwargs: calls.commands.append((args, kwargs)),
        reject_command=lambda *args: calls.rejected.append(args),
        set_source=lambda source: (True, source),
    )
    node.odom_pub = SimpleNamespace(publish=calls.published.append)
    return node, calls


@pytest.fixture
def adapter_factory(monkeypatch):
    """Run the complete constructor with ROS and hardware transports replaced."""
    calls = SimpleNamespace(
        parameters={"network_interface": "test0"}, namespace="", remapping={},
        publishers=[], published=[], rpc=[], clients=[],
    )

    class RosNode:
        def __init__(self, name):
            self.parameters = {}

        def declare_parameter(self, name, default):
            self.parameters[name] = calls.parameters.get(name, default)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def resolve_topic_name(self, topic):
            resolved = topic if topic.startswith("/") else f"{calls.namespace}/{topic}"
            return calls.remapping.get(resolved, resolved)

        def create_publisher(self, message_type, topic, qos):
            publisher = SimpleNamespace(
                topic_name=self.resolve_topic_name(topic), publish=calls.published.append,
            )
            calls.publishers.append(publisher)
            return publisher

        def create_subscription(self, *args):
            return None

        def create_service(self, *args):
            return None

        def create_timer(self, *args):
            return None

        def get_logger(self):
            return SimpleNamespace(info=lambda _: None)

    module = load_adapter(
        monkeypatch, node_class=RosNode,
        parameter_class=SimpleNamespace(Type=SimpleNamespace(STRING=object())),
        qos={
            "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=object()),
            "ReliabilityPolicy": SimpleNamespace(RELIABLE=object()),
            "QoSProfile": SimpleNamespace,
            "qos_profile_sensor_data": object(),
        },
    )
    module.rclpy.ok = lambda: False

    def connect_rpc(*args):
        calls.rpc.append(args)
        return object()

    def create_client(*args, **kwargs):
        calls.clients.append((args, kwargs))
        return SimpleNamespace()

    monkeypatch.setattr(module, "connect_rpc", connect_rpc)
    monkeypatch.setattr(module, "G1DClient", create_client)
    return module.G1DAdapterNode, calls


@pytest.mark.parametrize("namespace, remapping, native_topic", [
    ("", {}, "rt/odom"),
    ("/nav", {}, "rt/nav/odom"),
    ("", {"/odom": "/agv/odom"}, "rt/agv/odom"),
    ("/robot", {"/robot/odom": "/device/pose"}, "rt/device/pose"),
])
def test_adapter_rejects_native_feedback_loop_before_publishers_or_rpc(
    adapter_factory, namespace, remapping, native_topic,
):
    create_node, calls = adapter_factory
    calls.namespace = namespace
    calls.remapping = remapping
    calls.parameters["native_odom_dds_topic"] = native_topic
    with pytest.raises(ValueError, match="native odometry DDS source must differ"):
        create_node()
    assert calls.publishers == []
    assert calls.published == []
    assert calls.rpc == []
    assert calls.clients == []


@pytest.mark.parametrize("native_topic", [None, "", "rt/agv/odom"])
def test_adapter_allows_external_mode_and_a_distinct_native_source(adapter_factory, native_topic):
    create_node, calls = adapter_factory
    if native_topic is not None:
        calls.parameters["native_odom_dds_topic"] = native_topic
    node = create_node()
    assert node.odom_pub.topic_name == "/odom"
    assert calls.rpc == [("test0", 0, 0.1)]
    assert len(calls.clients) == 1
    assert calls.published == []


def vector(**kwargs):
    return SimpleNamespace(**{"x": 0.0, "y": 0.0, "z": 0.0, **kwargs})


def header(stamp_ns, frame="base_link"):
    return SimpleNamespace(
        frame_id=frame,
        stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
    )


def odom(stamp_ns=9_900_000_000):
    return SimpleNamespace(
        header=header(stamp_ns, "odom"), child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=vector(x=1.0), orientation=vector(w=1.0)),
            covariance=[0.0] * 36,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=vector(), angular=vector()), covariance=[0.0] * 36,
        ),
    )


def command(stamp_ns=9_900_000_000):
    return SimpleNamespace(
        header=header(stamp_ns),
        twist=SimpleNamespace(linear=vector(x=0.2), angular=vector(z=-0.1)),
    )


def test_odometry_preserves_sensor_stamp_and_does_not_refresh_repeated_data(bridge):
    node, calls = bridge
    message = odom()
    node._on_odometry(message)
    node._on_odometry(message)
    node._on_odometry(odom(9_800_000_000))
    assert calls.published == [message]
    assert message.header.stamp.nanosec == 900_000_000
    assert calls.odom == pytest.approx([19.9])
    assert calls.invalid == []


def test_invalid_odom_disables_gate_and_is_not_published(bridge):
    node, calls = bridge
    message = odom()
    message.child_frame_id = "camera"
    node._on_odometry(message)
    assert calls.invalid
    assert calls.published == []
    assert calls.odom == []


def test_commands_preserve_transport_age(bridge):
    node, calls = bridge
    node._on_command("auto", command())
    assert calls.commands[0][0] == ("auto", 0.2, -0.1)
    assert calls.commands[0][1]["received_s"] == pytest.approx(19.9)


@pytest.mark.parametrize("change", ["stale", "future", "frame", "lateral", "nan_lateral"])
def test_invalid_command_is_rejected(bridge, change):
    node, calls = bridge
    message = command()
    if change == "stale":
        message.header = header(9_000_000_000)
    elif change == "future":
        message.header = header(10_100_000_000)
    elif change == "frame":
        message.header.frame_id = "odom"
    else:
        message.twist.linear.y = 0.1 if change == "lateral" else float("nan")
    node._on_command("manual", message)
    assert calls.rejected[0][0] == "manual"
    assert calls.commands == []


def test_source_enable_drops_already_queued_commands(bridge):
    node, calls = bridge
    response = node._set_source("auto", SimpleNamespace(data=True), SimpleNamespace())
    assert response.success
    node._on_command("auto", command())
    assert calls.commands == []
    assert calls.rejected == []


def test_posture_service_reports_unsupported(bridge):
    node, _ = bridge
    response = node._unsupported_posture(None, SimpleNamespace())
    assert not response.success
    assert "not supported" in response.message
