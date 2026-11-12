"""CPU-only browser command tests for the G1-D wheel-base integration."""

import asyncio
import importlib.util
import queue
from pathlib import Path

import pytest

pytest.importorskip("aiohttp")
WEB_ROOT = Path(__file__).resolve().parents[1] / "robot_deploy/src/vln_web"
_spec = importlib.util.spec_from_file_location(
    "g1d_test_web_server", WEB_ROOT / "vln_web/web_server.py"
)
web_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(web_server)


class Logger:
    def info(self, _message):
        pass


class Browser:
    def __init__(self):
        self.messages = []

    async def send_json(self, message):
        self.messages.append(message)


@pytest.fixture
def server():
    server = web_server.WebServer(
        host="127.0.0.1",
        port=8088,
        web_dir=WEB_ROOT / "web",
        image_topic="camera/color/image_raw",
        manual_linear_limit=0.3,
        manual_angular_limit=0.3,
        manual_linear_accel=0.5,
        manual_angular_accel=0.5,
        commands=queue.Queue(),
        logger=Logger(),
    )
    server.update_robot_diagnostics(
        {"adapter": "g1d", "connected": True, "mode": "READY"}, "disabled"
    )
    server.update_server_url("ws://gpu-host:8050")
    server.update_mpc_status("IDLE")
    return server


@pytest.mark.parametrize(
    ("adapter", "mode", "connected", "expected"),
    [
        ("g1d", "READY", True, True),
        ("g1d", "READY", False, False),
        ("g1d", "WALK", True, False),
        ("g1d", "FAULT", True, False),
        ("g1d", "STOPPED", True, False),
        ("g1d", "STOPPING", True, False),
        ("go2", "WALK", True, True),
        ("tron", "WALK", True, True),
        ("go2", "READY", True, False),
        ("tron", "STAND", True, False),
    ],
)
def test_wheel_base_readiness_preserves_existing_adapter_modes(
    adapter, mode, connected, expected
):
    assert web_server.robot_motion_ready(
        {"adapter": adapter, "mode": mode, "connected": connected}
    ) is expected


def test_g1d_ready_can_start_navigation_without_posture_action(server):
    browser = Browser()
    asyncio.run(server._handle_browser_message(
        browser, "browser", {
            "type": "set_vln", "enabled": True,
            "instruction": "go to the door", "mode": "objnav",
        },
    ))
    assert server._commands.get_nowait() == (
        "set_vln", (True, "go to the door", "objnav"), "browser"
    )
    assert server._commands.get_nowait() == ("set_mpc", True, "browser")
    assert server._auto_controller is browser
    assert not browser.messages


@pytest.mark.parametrize("ready", [True, False])
def test_g1d_mpc_enable_requires_ready_feedback(server, ready):
    server._vln["enabled"] = True
    server._path["body_waypoints"] = [[0.2, 0.0, 0.0]]
    server._robot["mode"] = "READY" if ready else "STOPPED"
    browser = Browser()
    asyncio.run(server._handle_browser_message(
        browser, "browser", {"type": "set_mpc", "enabled": True}
    ))
    if ready:
        assert server._commands.get_nowait() == ("set_mpc", True, "browser")
        assert server._auto_controller is browser
    else:
        assert server._commands.empty()
        assert browser.messages[-1]["ok"] is False
        assert "requires READY" in browser.messages[-1]["message"]


@pytest.mark.parametrize("mode", ["FAULT", "STOPPED", "STOPPING", "UNKNOWN", "WALK"])
def test_g1d_fault_or_missing_odometry_blocks_navigation(server, mode):
    server.update_robot_diagnostics(
        {"adapter": "g1d", "connected": True, "mode": mode}, "disabled"
    )
    browser = Browser()
    asyncio.run(server._handle_browser_message(
        browser, "browser", {
            "type": "set_vln", "enabled": True,
            "instruction": "go to the door", "mode": "objnav",
        },
    ))
    assert server._commands.empty()
    assert browser.messages[-1]["ok"] is False
    assert "requires READY" in browser.messages[-1]["message"]


@pytest.mark.parametrize("action", ["stand", "walk", "sit", "toggle_policy"])
def test_g1d_rejects_unsupported_actions_even_from_control_owner(server, action):
    browser = Browser()
    server._controller = browser
    asyncio.run(server._handle_browser_message(
        browser, "browser", {"type": "robot_action", "action": action}
    ))
    assert server._commands.empty()
    assert browser.messages[-1]["ok"] is False
    assert "does not support" in browser.messages[-1]["message"]


def test_g1d_fault_allows_explicit_manual_recovery_after_stopping_auto(server):
    browser = Browser()
    server._auto_controller = browser
    server._robot["mode"] = "FAULT"
    asyncio.run(server._handle_browser_message(browser, "browser", {"type": "stop"}))
    assert server._commands.get_nowait() == ("set_mpc", False, "browser")
    assert server._commands.get_nowait() == ("stop", None, "browser")
    assert server._auto_controller is None
    # The adapter retains FAULT after its zero burst, until explicitly re-enabled.
    asyncio.run(server._handle_browser_message(
        browser, "browser", {"type": "acquire_control"}
    ))
    assert server._commands.get_nowait() == ("manual_control", True, "browser")
    assert server._controller is browser
    assert not browser.messages


@pytest.mark.parametrize("mode", ["STOPPED", "STOPPING", "UNKNOWN"])
def test_g1d_does_not_acquire_manual_control_during_stop(server, mode):
    server._robot["mode"] = mode
    browser = Browser()
    asyncio.run(server._handle_browser_message(
        browser, "browser", {"type": "acquire_control"}
    ))
    assert server._commands.empty()
    assert server._controller is None
    assert browser.messages[-1]["ok"] is False


@pytest.mark.parametrize("kind", ["set_vln", "set_mpc"])
def test_g1d_requires_manual_release_before_auto_control(server, kind):
    browser = Browser()
    server._controller = browser
    server._vln["enabled"] = True
    server._path["body_waypoints"] = [[0.2, 0.0, 0.0]]
    asyncio.run(server._handle_browser_message(
        browser, "browser", {
            "type": kind, "enabled": True,
            "instruction": "go to the door", "mode": "objnav",
        },
    ))
    assert server._commands.empty()
    assert server._controller is browser
    assert server._auto_controller is None
    assert "release manual control and wait for READY" in browser.messages[-1]["message"]


def test_g1d_must_stop_auto_before_taking_manual_control(server):
    browser = Browser()
    server._auto_controller = browser
    asyncio.run(server._handle_browser_message(
        browser, "browser", {"type": "acquire_control"}
    ))
    assert server._commands.empty()
    assert server._controller is None
    assert browser.messages[-1]["ok"] is False
