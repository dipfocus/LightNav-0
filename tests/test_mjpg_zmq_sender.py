"""Standalone camera sender interoperability, without cameras, ROS or OpenCV."""

from __future__ import annotations

import importlib
import importlib.util
import io
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from PIL import Image

PACKAGE = Path(__file__).resolve().parents[1] / "robot_deploy/src/vln_camera"
WALL_NS = 1_800_000_000_000_000_000
MONOTONIC_NS = 42_000_000_000


@pytest.fixture
def sender(monkeypatch):
    # Stubs are installed only for this fixture; no native camera or socket opens.
    monkeypatch.setitem(sys.modules, "cv2", ModuleType("cv2"))
    monkeypatch.setitem(sys.modules, "zmq", ModuleType("zmq"))
    spec = importlib.util.spec_from_file_location(
        "_test_mjpg_zmq_sender", PACKAGE / "scripts/mjpg_zmq_sender.py"
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def receiver(monkeypatch):
    monkeypatch.syspath_prepend(str(PACKAGE))
    return importlib.import_module("vln_camera.xraw")


@pytest.fixture
def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (8, 6), (20, 40, 80)).save(output, format="JPEG")
    return output.getvalue()


def test_sender_packet_is_accepted_by_receiver(sender, receiver, jpeg, monkeypatch):
    monkeypatch.setattr(sender, "time", SimpleNamespace(
        time_ns=lambda: WALL_NS, monotonic_ns=lambda: MONOTONIC_NS,
    ))
    packet = sender._xraw("wrist_left", "jpeg", 17, 8, 6, jpeg)
    frame = receiver.parse_frame(packet, expected_role="wrist_left")
    assert (frame.width, frame.height, frame.sequence, frame.role) == (8, 6, 17, "wrist_left")
    assert (frame.wall_ns, frame.monotonic_ns) == (WALL_NS, MONOTONIC_NS)
    assert frame.jpeg == jpeg


def test_raw_mode_is_not_accepted_by_jpeg_receiver(sender, receiver):
    packet = sender._xraw("head_fpv", "raw_bgr", 0, 8, 6, bytes(8 * 6 * 3))
    with pytest.raises(ValueError, match="only XRAW v2 jpeg"):
        receiver.parse_frame(packet)


def test_multiple_cameras_bind_by_role_not_argument_order(sender):
    specs = sender._parse_specs(
        ["head_fpv=/dev/video0", "wrist_left=/dev/video2"],
        ["wrist_left=tcp://0.0.0.0:5557", "head_fpv=tcp://0.0.0.0:5556"],
    )
    assert {spec.role: (spec.device, spec.endpoint) for spec in specs} == {
        "head_fpv": ("/dev/video0", "tcp://0.0.0.0:5556"),
        "wrist_left": ("/dev/video2", "tcp://0.0.0.0:5557"),
    }


@pytest.mark.parametrize("cameras, binds", [
    ([], []),
    (["head_fpv=/dev/video0"], []),
    (["head_fpv=/dev/video0"], ["wrist_left=tcp://0.0.0.0:5556"]),
    (["head_fpv"], ["head_fpv=tcp://0.0.0.0:5556"]),
    (["head_fpv="], ["head_fpv=tcp://0.0.0.0:5556"]),
])
def test_missing_or_mismatched_camera_configuration_is_rejected(sender, cameras, binds):
    with pytest.raises(ValueError):
        sender._parse_specs(cameras, binds)


@pytest.fixture
def worker(sender, jpeg, monkeypatch):
    state = SimpleNamespace(events=[], packets=[], failure=None, closed=False, released=False)
    stop = threading.Event()
    frame = object()

    class Capture:
        def __init__(self, device, backend):
            assert (device, backend) == ("/dev/video0", sender.cv2.CAP_V4L2)
            self.settings = {}

        def isOpened(self):
            return True

        def set(self, prop, value):
            self.settings[prop] = value

        def get(self, prop):
            return self.settings[prop]

        def read(self):
            state.events.append("read")
            return state.failure != "read", frame

        def release(self):
            state.released = True

    class Encoded:
        def tobytes(self):
            state.events.append("bytes")
            return jpeg

    def encode(extension, image, params):
        assert extension == ".jpg" and image is frame
        assert params == [sender.cv2.IMWRITE_JPEG_QUALITY, 80]
        state.events.append("encode")
        return state.failure != "encode", Encoded()

    class Socket:
        def bind(self, endpoint):
            assert endpoint == "tcp://0.0.0.0:5556"

        def send(self, packet):
            state.events.append("send")
            if state.failure == "send":
                raise RuntimeError("send failed")
            state.packets.append(packet)
            stop.set()

        def close(self, *, linger):
            assert linger == 0
            state.closed = True

    def create_socket(kind):
        assert kind == sender.zmq.PUB
        return Socket()

    def timestamp(event, value):
        state.events.append(event)
        return value

    for number, name in enumerate((
        "CAP_V4L2", "CAP_PROP_FOURCC", "CAP_PROP_FRAME_WIDTH", "CAP_PROP_FRAME_HEIGHT",
        "CAP_PROP_FPS", "IMWRITE_JPEG_QUALITY",
    )):
        monkeypatch.setattr(sender.cv2, name, number, raising=False)
    monkeypatch.setattr(sender.cv2, "VideoCapture", Capture, raising=False)
    monkeypatch.setattr(sender.cv2, "VideoWriter_fourcc", lambda *chars: "".join(chars), raising=False)
    monkeypatch.setattr(sender.cv2, "imencode", encode, raising=False)
    monkeypatch.setattr(sender.zmq, "PUB", 1, raising=False)
    monkeypatch.setattr(sender, "time", SimpleNamespace(
        time_ns=lambda: timestamp("wall", WALL_NS),
        monotonic_ns=lambda: timestamp("monotonic", MONOTONIC_NS),
    ))
    state.args = (
        sender.CameraSpec("head_fpv", "/dev/video0", "tcp://0.0.0.0:5556"),
        SimpleNamespace(width=8, height=6, fps=30, encoding="jpeg", jpeg_quality=80),
        SimpleNamespace(socket=create_socket), stop, sender.CameraStats(),
    )
    return state


def test_worker_stamps_after_encoding_and_publishes_compatible_jpeg(sender, receiver, worker, jpeg):
    sender._worker(*worker.args)
    assert worker.events == ["read", "encode", "bytes", "wall", "monotonic", "send"]
    assert len(worker.packets) == 1
    frame = receiver.parse_frame(worker.packets[0])
    assert frame.jpeg == jpeg
    assert (frame.sequence, frame.wall_ns, frame.monotonic_ns) == (0, WALL_NS, MONOTONIC_NS)
    stats = worker.args[-1]
    assert (stats.captured, stats.encoded, stats.sent) == (1, 1, 1)
    assert stats.bytes_sent == len(worker.packets[0])
    assert worker.closed and worker.released


@pytest.mark.parametrize("failure, message", [
    ("read", "camera read failed"),
    ("encode", "JPEG encode failed"),
    ("send", "send failed"),
])
def test_worker_releases_resources_on_frame_failure(sender, worker, failure, message):
    worker.failure = failure
    with pytest.raises(RuntimeError, match=message):
        sender._worker(*worker.args)
    assert worker.packets == []
    assert worker.args[-1].sent == 0
    assert worker.closed and worker.released
