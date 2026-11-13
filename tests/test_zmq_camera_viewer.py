"""Standalone preview behavior with synthetic JPEGs and no camera, GUI or ROS."""

from __future__ import annotations

import importlib.util
import io
import struct
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from PIL import Image

PACKAGE = Path(__file__).resolve().parents[1] / "robot_deploy/src/vln_camera"
SCRIPT = PACKAGE / "scripts/zmq_camera_viewer.py"
ENDPOINT = "tcp://192.0.2.1:5556"


@pytest.fixture
def viewer(monkeypatch):
    # Restore sys.path after the script's source-checkout bootstrap, too.
    monkeypatch.syspath_prepend(str(PACKAGE))
    spec = importlib.util.spec_from_file_location("_test_zmq_camera_viewer", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def packet():
    output = io.BytesIO()
    Image.new("RGB", (8, 6), (20, 40, 80)).save(output, format="JPEG")
    jpeg = output.getvalue()

    def build(role="head_fpv"):
        # Use the wire layout independently of the parser's header constant.
        return struct.pack(
            ">4sIIIIQQQ16s16sQI", b"XRAW", 2, 8, 6, 3, 7,
            1_800_000_000_000_000_000, 42_000_000_000,
            role.encode(), b"jpeg", 42_000_000_000, 0,
        ) + jpeg

    return build


@pytest.fixture
def runtime(viewer, monkeypatch):
    class GuiError(Exception):
        pass

    class ZmqError(Exception):
        pass

    state = SimpleNamespace(
        packets=[], keys=[], displayed=[], decoded=[], endpoints=[],
        window_created=False, window_destroyed=False, subscriber_closed=False,
        visible=1, failure=None, decode_result="jpeg", GuiError=GuiError, ZmqError=ZmqError,
    )

    def named_window(_name, _flags):
        if state.failure == "window":
            raise GuiError("GUI unavailable")
        state.window_created = True

    def decode(data, _flags):
        state.decoded.append(data.tobytes())
        if state.decode_result is None:
            return None
        if isinstance(state.decode_result, BaseException):
            raise state.decode_result
        if isinstance(state.decode_result, np.ndarray):
            return state.decode_result
        return np.asarray(Image.open(io.BytesIO(data.tobytes())).convert("RGB"))[:, :, ::-1]

    def show(_name, frame):
        if state.failure == "show":
            raise GuiError("display failed")
        state.displayed.append(frame)

    def destroy(_name):
        state.window_destroyed = True
        if state.failure == "destroy":
            raise GuiError("window was already closed")

    class Subscriber:
        def __init__(self, endpoint):
            state.endpoints.append(endpoint)
            if state.failure == "connect":
                raise ZmqError("connection failed")

        def receive(self):
            if state.failure == "receive":
                raise ZmqError("receive failed")
            if not state.packets:
                return None
            incoming = state.packets.pop(0)
            if isinstance(incoming, BaseException):
                raise incoming
            return incoming

        def close(self):
            state.subscriber_closed = True

    cv2 = ModuleType("cv2")
    cv2.error = GuiError
    cv2.WINDOW_NORMAL = cv2.WND_PROP_VISIBLE = cv2.IMREAD_COLOR = 1
    cv2.namedWindow = named_window
    cv2.imdecode = decode
    cv2.imshow = show
    cv2.waitKey = lambda _delay: state.keys.pop(0) if state.keys else ord("q")
    cv2.getWindowProperty = lambda _name, _property: state.visible
    cv2.destroyWindow = destroy
    zmq = ModuleType("zmq")
    zmq.ZMQError = ZmqError
    monkeypatch.setitem(sys.modules, "cv2", cv2)
    monkeypatch.setitem(sys.modules, "zmq", zmq)
    monkeypatch.setattr(viewer, "XrawSubscriber", Subscriber)
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("DISPLAY", ":test")
    return state


def test_displays_valid_jpeg_and_releases_resources(viewer, runtime, packet, capsys):
    runtime.packets = [packet()]
    assert viewer.main([ENDPOINT]) == 0
    assert runtime.endpoints == [ENDPOINT]
    assert len(runtime.displayed) == 1
    assert runtime.displayed[0].shape == (6, 8, 3)
    assert "First frame: head_fpv 8x6, seq=7" in capsys.readouterr().out
    assert runtime.subscriber_closed and runtime.window_destroyed


def test_wrong_role_is_rejected_then_expected_role_is_displayed(viewer, runtime, packet, capsys):
    runtime.packets = [packet(), packet("wrist_left")]
    runtime.keys = [-1, 27]
    assert viewer.main([ENDPOINT, "--camera-role", "wrist_left"]) == 0
    output = capsys.readouterr()
    assert "expected camera role 'wrist_left'" in output.err
    assert "First frame: wrist_left" in output.out
    assert len(runtime.decoded) == len(runtime.displayed) == 1


@pytest.mark.parametrize("result", [None, "wrong_shape", "decode_error"])
def test_decode_failures_are_not_displayed(viewer, runtime, packet, capsys, result):
    runtime.packets = [packet()]
    runtime.decode_result = {
        None: None,
        "wrong_shape": np.zeros((8, 6, 3), dtype=np.uint8),
        "decode_error": runtime.GuiError("corrupt JPEG"),
    }[result]
    assert viewer.main([ENDPOINT]) == 0
    assert not runtime.displayed
    assert "Rejected frame:" in capsys.readouterr().err
    assert runtime.subscriber_closed and runtime.window_destroyed


@pytest.mark.parametrize("failure", ["show", "receive"])
def test_runtime_failure_closes_subscriber_and_window(viewer, runtime, packet, capsys, failure):
    runtime.failure = failure
    runtime.packets = [packet()]
    assert viewer.main([ENDPOINT]) == 1
    assert "failed" in capsys.readouterr().err
    assert runtime.subscriber_closed and runtime.window_destroyed


def test_connect_failure_cleans_up_created_window(viewer, runtime, capsys):
    runtime.failure = "connect"
    assert viewer.main([ENDPOINT]) == 1
    assert runtime.window_destroyed
    assert "connection failed" in capsys.readouterr().err


def test_gui_startup_failure_does_not_subscribe(viewer, runtime, capsys):
    runtime.failure = "window"
    assert viewer.main([ENDPOINT]) == 1
    assert not runtime.endpoints
    assert "opencv-python-headless cannot display windows" in capsys.readouterr().err


def test_ctrl_c_releases_resources(viewer, runtime):
    runtime.packets = [KeyboardInterrupt()]
    assert viewer.main([ENDPOINT]) == 0
    assert runtime.subscriber_closed and runtime.window_destroyed


def test_manual_window_close_exits_even_if_destroy_reports_already_closed(viewer, runtime):
    runtime.visible = 0
    runtime.keys = [-1]
    runtime.failure = "destroy"
    assert viewer.main([ENDPOINT]) == 0
    assert runtime.subscriber_closed and runtime.window_destroyed


def test_headless_linux_fails_before_gui_or_network(viewer, runtime, monkeypatch, capsys):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    assert viewer.main([ENDPOINT]) == 1
    assert not runtime.window_created and not runtime.endpoints
    assert "No graphical display" in capsys.readouterr().err


def test_missing_dependency_has_installation_guidance(viewer, runtime, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "cv2", None)
    assert viewer.main([ENDPOINT]) == 1
    assert "python3 -m pip install opencv-python numpy pyzmq" in capsys.readouterr().err
    assert not runtime.window_created and not runtime.endpoints


def test_no_frame_warning_does_not_depend_on_sender_clock(viewer, runtime, monkeypatch, capsys):
    times = iter([0.0, 5.1])
    monkeypatch.setattr(viewer, "time", SimpleNamespace(monotonic=lambda: next(times)))
    assert viewer.main([ENDPOINT]) == 0
    assert "No valid frame for 5.1 seconds" in capsys.readouterr().out


def test_reports_average_displayed_fps(viewer, runtime, packet, monkeypatch, capsys):
    runtime.packets = [packet(), packet()]
    runtime.keys = [-1, ord("Q")]
    times = iter([0.0, 1.0, 1.0, 6.0, 6.0])
    monkeypatch.setattr(viewer, "time", SimpleNamespace(monotonic=lambda: next(times)))
    assert viewer.main([ENDPOINT]) == 0
    assert "Viewer average: 0.2 FPS; displayed=2 rejected=0" in capsys.readouterr().out


@pytest.mark.parametrize(("args", "code", "message"), [
    (["--help"], 0, "--camera-role"),
    (["tcp://0.0.0.0:5556"], 2, "concrete host address"),
    ([ENDPOINT, "--camera-role", ""], 2, "camera-role must be"),
])
def test_cli_validation_works_outside_checkout_without_site_packages(tmp_path, args, code, message):
    result = subprocess.run(
        [sys.executable, "-S", str(SCRIPT), *args], cwd=tmp_path,
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == code, result.stderr
    assert message in result.stdout + result.stderr
