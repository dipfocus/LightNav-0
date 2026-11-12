"""Check camera and odometry wiring without ROS or starting launch actions."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "robot_deploy/src/vln_bringup/launch/g1d.launch.py"
)


@pytest.fixture
def launch_module(monkeypatch):
    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    class DeclareLaunchArgument:
        def __init__(self, name, **kwargs):
            self.name = name
            self.options = kwargs

    class OpaqueFunction:
        def __init__(self, *, function):
            self.function = function

    class Node:
        def __init__(self, **kwargs):
            self.options = kwargs

    class ParameterValue:
        def __init__(self, value, *, value_type):
            self.value = value
            self.value_type = value_type

    exports = {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "DeclareLaunchArgument": DeclareLaunchArgument,
            "OpaqueFunction": OpaqueFunction,
        },
        "launch.substitutions": {"LaunchConfiguration": LaunchConfiguration},
        "launch_ros": {},
        "launch_ros.actions": {"Node": Node},
        "launch_ros.parameter_descriptions": {"ParameterValue": ParameterValue},
    }
    for name, attributes in exports.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("g1d_camera_test_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_nodes(module, **overrides):
    description = module.generate_launch_description()
    context = {
        item.name: item.options.get("default_value", "")
        for item in description
        if hasattr(item, "name")
    }
    context.update(network_interface="eth0", **overrides)
    action = next(item for item in description if hasattr(item, "function"))
    nodes = action.function(context)

    def resolve(value):
        if isinstance(value, module.ParameterValue):
            raw = resolve(value.value)
            if value.value_type is bool:
                assert raw in {"true", "false"}
                return raw == "true"
            return value.value_type(raw)
        if isinstance(value, module.LaunchConfiguration):
            return value.perform(context)
        return value

    return {
        node.options["executable"]: {
            name: resolve(value) for name, value in node.options["parameters"][0].items()
        }
        for node in nodes
    }


def test_default_zmq_camera_connects_head_camera_and_configures_both_consumers(launch_module):
    nodes = launch_nodes(launch_module)
    topic = "camera/color/image_raw/compressed"
    assert nodes["zmq_camera"]["endpoint"] == "tcp://192.168.100.78:5556"
    assert nodes["zmq_camera"]["camera_role"] == "head_fpv"
    assert nodes["zmq_camera"]["image_topic"] == topic
    for package in ("vln_client", "vln_web"):
        assert nodes[package]["image_topic"] == topic
        assert nodes[package]["image_transport"] == "compressed"


def test_explicit_ros_camera_preserves_existing_consumers(launch_module):
    nodes = launch_nodes(launch_module, camera_source="ros")
    assert "zmq_camera" not in nodes
    for package in ("vln_client", "vln_web"):
        assert nodes[package]["image_topic"] == "camera/color/image_raw"
        assert nodes[package]["image_transport"] == "raw"


def test_existing_compressed_ros_camera_does_not_start_zmq(launch_module):
    nodes = launch_nodes(
        launch_module, camera_source="ros", image_topic="/external/jpeg", image_transport="compressed"
    )
    assert "zmq_camera" not in nodes
    for package in ("vln_client", "vln_web"):
        assert nodes[package]["image_topic"] == "/external/jpeg"
        assert nodes[package]["image_transport"] == "compressed"


def test_zmq_endpoint_override_reaches_receiver(launch_module):
    nodes = launch_nodes(
        launch_module, camera_endpoint="tcp://camera-host:5557"
    )
    topic = "camera/color/image_raw/compressed"
    assert nodes["zmq_camera"]["endpoint"] == "tcp://camera-host:5557"
    assert nodes["zmq_camera"]["image_topic"] == topic
    for package in ("vln_client", "vln_web"):
        assert nodes[package]["image_topic"] == topic
        assert nodes[package]["image_transport"] == "compressed"


def test_zmq_topic_override_is_shared_even_with_raw_transport_argument(launch_module):
    nodes = launch_nodes(
        launch_module,
        camera_source="zmq",
        camera_endpoint="tcp://camera-host:5556",
        image_topic="/g1d/head/jpeg",
        image_transport="raw",
    )
    for package in ("zmq_camera", "vln_client", "vln_web"):
        assert nodes[package]["image_topic"] == "/g1d/head/jpeg"
    assert nodes["vln_client"]["image_transport"] == "compressed"
    assert nodes["vln_web"]["image_transport"] == "compressed"


@pytest.mark.parametrize("endpoint", ["", "   "])
def test_zmq_rejects_empty_endpoint_before_creating_nodes(launch_module, endpoint):
    with pytest.raises(ValueError, match="camera_endpoint is required"):
        launch_nodes(launch_module, camera_source="zmq", camera_endpoint=endpoint)


def test_unknown_camera_source_is_rejected(launch_module):
    with pytest.raises(ValueError, match="camera_source must be"):
        launch_nodes(launch_module, camera_source="http")


def test_native_odometry_is_default(launch_module):
    nodes = launch_nodes(launch_module)
    odometry = nodes["g1d_odometry"]
    assert odometry["dds_topic"] == "rt/agv/odom"
    assert odometry["clock_offset_s"] == 0.0
    assert odometry["max_age_s"] == 0.5
    assert nodes["g1d_adapter"]["native_odom_dds_topic"] == "rt/agv/odom"
    for executable in ("g1d_odometry", "g1d_adapter"):
        assert nodes[executable]["network_interface"] == "eth0"
        assert nodes[executable]["dds_domain_id"] == 0
        assert nodes[executable]["odom_topic"] == "g1d/odom"


def test_native_odometry_overrides_reach_bridge_and_adapter(launch_module):
    nodes = launch_nodes(
        launch_module,
        dds_domain_id="7",
        odom_topic="/localization/odom",
        odom_dds_topic="rt/custom/odom",
        odom_clock_offset_s="-0.05",
        odom_max_age_s="0.25",
    )
    odometry = nodes["g1d_odometry"]
    assert odometry["dds_topic"] == "rt/custom/odom"
    assert odometry["clock_offset_s"] == -0.05
    assert odometry["max_age_s"] == 0.25
    assert nodes["g1d_adapter"]["native_odom_dds_topic"] == "rt/custom/odom"
    for executable in ("g1d_odometry", "g1d_adapter"):
        assert nodes[executable]["dds_domain_id"] == 7
        assert nodes[executable]["odom_topic"] == "/localization/odom"


def test_ros_odometry_source_does_not_start_native_bridge(launch_module):
    nodes = launch_nodes(launch_module, odom_source="ros", odom_topic="/localization/odom")
    assert "g1d_odometry" not in nodes
    assert nodes["g1d_adapter"]["odom_topic"] == "/localization/odom"
    assert nodes["g1d_adapter"]["native_odom_dds_topic"] == ""
    assert {"vln_client", "vln_web", "vln_mpc"} <= nodes.keys()


def test_unknown_odometry_source_is_rejected(launch_module):
    with pytest.raises(ValueError, match="odom_source must be"):
        launch_nodes(launch_module, odom_source="fake")
