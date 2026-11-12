"""XRAW v2 camera bridge tests using synthetic JPEGs, without ROS or a robot."""

from __future__ import annotations

import importlib
import io
import json
import struct
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from PIL import Image

# Deliberately independent of the implementation's header constant: these
# fixtures represent the sender's documented network-byte-order format.
HEADER = struct.Struct(">4sIIIIQQQ16s16sQI")
WALL_NS = 1_800_000_000_000_000_000
MONOTONIC_NS = 42_000_000_000


@pytest.fixture
def camera(monkeypatch):
    package = Path(__file__).resolve().parents[1] / "robot_deploy/src/vln_camera"
    monkeypatch.syspath_prepend(str(package))
    return importlib.import_module("vln_camera.xraw")


@pytest.fixture
def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (8, 6), (20, 40, 80)).save(output, format="JPEG")
    return output.getvalue()


def packet(jpeg, **changes):
    fields = {
        "magic": b"XRAW",
        "version": 2,
        "width": 8,
        "height": 6,
        "channels": 3,
        "sequence": 7,
        "wall_ns": WALL_NS,
        "monotonic_ns": MONOTONIC_NS,
        "role": b"head_fpv",
        "encoding": b"jpeg",
        "capture_monotonic_ns": MONOTONIC_NS,
        "flags": 0,
    }
    fields.update(changes)
    return HEADER.pack(*fields.values()) + jpeg


def test_parse_actual_jpeg_and_exact_sender_header(camera, jpeg):
    assert HEADER.size == 88
    frame = camera.parse_frame(packet(jpeg))
    assert (frame.width, frame.height) == (8, 6)
    assert frame.sequence == 7
    assert frame.wall_ns == WALL_NS
    assert frame.monotonic_ns == MONOTONIC_NS
    assert frame.role == "head_fpv"
    assert frame.jpeg == jpeg


@pytest.mark.parametrize("length", [0, 4, 20, 87, 88, 89])
def test_truncated_packet_fails_before_publication(camera, jpeg, length):
    with pytest.raises(ValueError):
        camera.parse_frame(packet(jpeg)[:length])


@pytest.mark.parametrize("changes", [
    {"magic": b"NOPE"},
    {"version": 1},
    {"version": 3},
    {"width": 0},
    {"height": 0},
    {"width": 0xFFFFFFFF},
    {"width": 100_000, "height": 100_000},
    {"channels": 1},
    {"channels": 4},
    {"wall_ns": 0},
    {"monotonic_ns": 0},
    {"role": b"wrist_left"},
    {"role": b""},
    {"role": b"\xff"},
    {"encoding": b"rgb8"},
    {"encoding": b"png"},
    {"flags": 1},
])
def test_invalid_header_is_rejected(camera, jpeg, changes):
    with pytest.raises(ValueError):
        camera.parse_frame(packet(jpeg, **changes))


def test_expected_role_is_explicitly_configurable(camera, jpeg):
    frame = camera.parse_frame(packet(jpeg, role=b"wrist_left"), expected_role="wrist_left")
    assert frame.role == "wrist_left"


@pytest.mark.parametrize("payload", [b"", b"not a jpeg", b"\xff\xd8broken", b"broken\xff\xd9"])
def test_non_jpeg_or_truncated_payload_is_rejected(camera, payload):
    with pytest.raises(ValueError):
        camera.parse_frame(packet(payload))


def test_packet_limit_includes_header(camera, jpeg):
    encoded = packet(jpeg)
    assert camera.parse_frame(encoded, max_packet_bytes=len(encoded)).jpeg == jpeg
    with pytest.raises(ValueError):
        camera.parse_frame(encoded, max_packet_bytes=len(encoded) - 1)


def test_jpeg_dimensions_must_match_header(camera, jpeg):
    with pytest.raises(ValueError, match="dimensions"):
        camera.parse_frame(packet(jpeg, width=9))


def test_soi_and_eoi_without_jpeg_frame_header_are_insufficient(camera):
    with pytest.raises(ValueError):
        camera.parse_frame(packet(b"\xff\xd8\xff\xd9"))


def test_clock_preserves_sender_wall_timestamp(camera, jpeg):
    frame = camera.parse_frame(packet(jpeg))
    clock = camera.FrameClock()
    assert clock.stamp(frame, now_ns=WALL_NS + 100_000_000) == WALL_NS


def test_clock_offset_and_capture_delay_are_applied_before_age_validation(camera, jpeg):
    frame = camera.parse_frame(packet(jpeg))
    clock = camera.FrameClock(clock_offset_s=-2.0, capture_delay_s=0.1)
    capture_ns = WALL_NS - 2_100_000_000
    assert clock.stamp(frame, now_ns=capture_ns + 100_000_000) == capture_ns


@pytest.mark.parametrize("delta_ns", [-1, 500_000_001])
def test_stale_or_future_capture_is_rejected(camera, jpeg, delta_ns):
    frame = camera.parse_frame(packet(jpeg))
    with pytest.raises(ValueError):
        camera.FrameClock().stamp(frame, now_ns=WALL_NS + delta_ns)


def test_frame_at_age_limit_is_accepted(camera, jpeg):
    frame = camera.parse_frame(packet(jpeg))
    assert camera.FrameClock().stamp(frame, now_ns=WALL_NS + 500_000_000) == WALL_NS


def test_duplicate_and_out_of_order_frames_are_rejected(camera, jpeg):
    clock = camera.FrameClock()
    frame = camera.parse_frame(packet(jpeg))
    assert clock.stamp(frame, now_ns=WALL_NS + 100_000_000) == WALL_NS
    for wall_ns, sequence in ((WALL_NS, 7), (WALL_NS, 8), (WALL_NS - 1, 9)):
        repeated = camera.parse_frame(packet(jpeg, wall_ns=wall_ns, sequence=sequence))
        with pytest.raises(ValueError):
            clock.stamp(repeated, now_ns=WALL_NS + 100_000_000)


def test_sequence_restart_with_newer_capture_timestamp_is_accepted(camera, jpeg):
    clock = camera.FrameClock()
    before = camera.parse_frame(packet(jpeg, sequence=1000))
    after = camera.parse_frame(packet(
        jpeg, sequence=0, wall_ns=WALL_NS + 20_000_000, monotonic_ns=1_000_000,
        capture_monotonic_ns=1_000_000,
    ))
    assert clock.stamp(before, now_ns=WALL_NS + 100_000_000) == WALL_NS
    assert clock.stamp(after, now_ns=WALL_NS + 100_000_000) == WALL_NS + 20_000_000


def test_rejected_future_frame_does_not_poison_timestamp_order(camera, jpeg):
    clock = camera.FrameClock()
    future = camera.parse_frame(packet(jpeg, wall_ns=WALL_NS + 1_000_000_000))
    with pytest.raises(ValueError):
        clock.stamp(future, now_ns=WALL_NS)
    current = camera.parse_frame(packet(jpeg))
    assert clock.stamp(current, now_ns=WALL_NS + 100_000_000) == WALL_NS


@pytest.mark.parametrize("kwargs", [
    {"clock_offset_s": float("nan")},
    {"clock_offset_s": float("inf")},
    {"capture_delay_s": -0.1},
    {"capture_delay_s": float("nan")},
    {"max_frame_age_s": 0.0},
    {"max_frame_age_s": float("inf")},
])
def test_invalid_clock_configuration_is_rejected(camera, kwargs):
    with pytest.raises(ValueError):
        camera.FrameClock(**kwargs)


@pytest.fixture
def fake_zmq(monkeypatch):
    class Again(Exception):
        pass

    state = SimpleNamespace(calls=[], incoming=[], fail_connect=False)

    class Socket:
        def setsockopt(self, option, value):
            state.calls.append(("option", option, value))

        def connect(self, endpoint):
            state.calls.append(("connect", endpoint))
            if state.fail_connect:
                raise RuntimeError("injected connect failure")

        def recv(self, *, flags):
            state.calls.append(("recv", flags))
            if not state.incoming:
                raise Again()
            return state.incoming.pop(0)

        def close(self, *, linger):
            state.calls.append(("close", linger))

    class Context:
        def socket(self, kind):
            state.calls.append(("socket", kind))
            return Socket()

        def term(self):
            state.calls.append(("term",))

    module = ModuleType("zmq")
    for number, name in enumerate(("SUB", "LINGER", "CONFLATE", "MAXMSGSIZE", "SUBSCRIBE", "NOBLOCK")):
        setattr(module, name, number)
    module.Context = Context
    module.Again = Again
    state.module = module
    monkeypatch.setitem(sys.modules, "zmq", module)
    return state


def test_subscriber_is_nonblocking_conflated_and_bounded(camera, fake_zmq, jpeg):
    zmq = fake_zmq.module
    subscriber = camera.XrawSubscriber("tcp://192.0.2.1:6000", max_packet_bytes=1_000_000)
    assert fake_zmq.calls == [
        ("socket", zmq.SUB),
        ("option", zmq.LINGER, 0),
        ("option", zmq.CONFLATE, 1),
        ("option", zmq.MAXMSGSIZE, 1_000_000),
        ("option", zmq.SUBSCRIBE, b""),
        ("connect", "tcp://192.0.2.1:6000"),
    ]
    assert subscriber.receive() is None
    fake_zmq.incoming.append(packet(jpeg))
    assert subscriber.receive() == packet(jpeg)
    assert fake_zmq.calls[-1] == ("recv", zmq.NOBLOCK)
    subscriber.close()
    assert fake_zmq.calls[-2:] == [("close", 0), ("term",)]
    calls_after_close = list(fake_zmq.calls)
    subscriber.close()
    assert subscriber.receive() is None
    assert fake_zmq.calls == calls_after_close


def test_failed_connect_closes_owned_resources(camera, fake_zmq):
    fake_zmq.fail_connect = True
    with pytest.raises(RuntimeError, match="injected"):
        camera.XrawSubscriber("tcp://192.0.2.1:6000")
    assert fake_zmq.calls[-2:] == [("close", 0), ("term",)]


@pytest.mark.parametrize("endpoint", [
    "", "192.0.2.1:6000", "http://192.0.2.1:6000", "tcp://192.0.2.1",
    "tcp://0.0.0.0:6000", "tcp://*:6000", "tcp://[::]:6000",
    "tcp://192.0.2.1:0", "tcp://192.0.2.1:65536", "tcp://name:notaport",
    "tcp://user:password@host:6000", "tcp://host:6000/path", "tcp://host:6000?key=value",
])
def test_invalid_endpoint_fails_without_creating_socket(camera, fake_zmq, endpoint):
    with pytest.raises(ValueError):
        camera.XrawSubscriber(endpoint)
    assert fake_zmq.calls == []


@pytest.fixture
def bridge(camera, monkeypatch):
    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")

    class Node:
        def destroy_node(self):
            return True

    modules = {
        "rclpy": {},
        "rclpy.executors": {"ExternalShutdownException": RuntimeError},
        "rclpy.node": {"Node": Node},
        "rclpy.qos": dict.fromkeys(("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"), object),
        "sensor_msgs.msg": {"CompressedImage": Message},
        "std_msgs.msg": {"String": lambda **kw: SimpleNamespace(**kw)},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(camera.__file__).with_name("zmq_camera_node.py")
    spec = importlib.util.spec_from_file_location("vln_camera._node_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.ZmqCameraNode.__new__(module.ZmqCameraNode)
    node.role = "head_fpv"
    node.frame_id = "head_camera"
    node.max_packet_bytes = 8 * 1024 * 1024
    node.clock = camera.FrameClock()
    node._received_ns = node._last_stamp_ns = node._frames = node._rejected = 0
    node._last_error = "waiting for video stream"
    node._last_log_s = float("-inf")
    node._closed = False
    state = SimpleNamespace(
        now_ns=WALL_NS + 100_000_000, packets=[], published=[], statuses=[], warnings=[], closed=[],
    )
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=state.now_ns))
    node.get_logger = lambda: SimpleNamespace(warning=state.warnings.append)
    node.publisher = SimpleNamespace(publish=state.published.append)
    node.status_pub = SimpleNamespace(publish=state.statuses.append)
    node._subscriber = SimpleNamespace(
        receive=lambda: state.packets.pop(0) if state.packets else None,
        close=lambda: state.closed.append(True),
    )
    return node, state


def test_bridge_publishes_only_jpeg_with_corrected_sender_timestamp(bridge, camera, jpeg):
    node, state = bridge
    node.clock = camera.FrameClock(capture_delay_s=0.05)
    state.packets.append(packet(jpeg))
    node._poll()
    assert len(state.published) == 1
    image = state.published[0]
    assert image.data == jpeg
    assert image.format == "jpeg"
    assert image.header.frame_id == "head_camera"
    assert image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec == WALL_NS - 50_000_000
    assert node._frames == 1
    assert node._rejected == 0
    node._poll()
    assert len(state.published) == 1


def test_bridge_rejects_malformed_stale_and_duplicate_frames(bridge, jpeg):
    node, state = bridge
    state.packets.extend([
        b"invalid",
        packet(jpeg, wall_ns=WALL_NS - 1_000_000_000),
        packet(jpeg),
        packet(jpeg),
    ])
    for _ in range(4):
        node._poll()
    assert len(state.published) == 1
    assert node._frames == 1
    assert node._rejected == 3
    assert state.warnings


def test_bridge_status_expires_when_sender_stops(bridge, jpeg):
    node, state = bridge
    state.packets.append(packet(jpeg))
    node._poll()
    node._publish_status()
    current = json.loads(state.statuses[-1].data)
    assert current["connected"]
    assert current["timestamp_basis"] == "sender_post_encode_corrected"
    state.now_ns += 1_000_000_000
    node._poll()
    node._publish_status()
    stale = json.loads(state.statuses[-1].data)
    assert not stale["connected"]
    assert stale["frames"] == 1
    assert "stale" in stale["error"]


def test_bridge_destruction_closes_subscriber_once(bridge):
    node, state = bridge
    assert node.destroy_node()
    assert node.destroy_node()
    assert state.closed == [True]
