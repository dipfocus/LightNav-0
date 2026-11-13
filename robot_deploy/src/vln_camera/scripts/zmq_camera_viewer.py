#!/usr/bin/env python3
"""Preview the XRAW v2 JPEG stream from mjpg_zmq_sender.py without ROS."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Run directly from a checkout without installing the ROS package. When this
# script is installed under share/vln_camera/scripts, use the installed package.
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if (_PACKAGE_ROOT / "vln_camera" / "__init__.py").is_file():
    sys.path.insert(0, str(_PACKAGE_ROOT))

from vln_camera.xraw import XrawSubscriber, parse_frame, validate_endpoint  # noqa: E402

_WINDOW = "LightNav camera - press Q or Esc to exit"
_GUI_HELP = (
    "Use a graphical desktop and install opencv-python in a separate viewer "
    "environment; opencv-python-headless cannot display windows."
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("endpoint", help="sender address, e.g. tcp://192.168.100.78:5556")
    parser.add_argument("--camera-role", default="head_fpv", help="expected XRAW camera role (default: head_fpv)")
    args = parser.parse_args(argv)
    try:
        validate_endpoint(args.endpoint)
    except ValueError as exc:
        parser.error(str(exc))
    if not 1 <= len(args.camera_role.encode("utf-8")) <= 16:
        parser.error("camera-role must be 1..16 UTF-8 bytes")

    # Keep --help and argument validation usable without optional dependencies.
    try:
        import cv2
        import numpy as np
        import zmq
    except ImportError as exc:
        print(
            f"Viewer dependency unavailable: {exc}. "
            "Install with: python3 -m pip install opencv-python numpy pyzmq",
            file=sys.stderr,
        )
        return 1

    # Qt-backed OpenCV builds may abort the process, rather than raise an error,
    # if there is no display. Catch the common SSH/headless case before HighGUI.
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print(f"No graphical display is available. {_GUI_HELP}", file=sys.stderr)
        return 1

    subscriber = None
    window_open = False
    try:
        cv2.namedWindow(_WINDOW, cv2.WINDOW_NORMAL)
        window_open = True
        subscriber = XrawSubscriber(args.endpoint)
        last_frame = last_notice = started = time.monotonic()
        count = rejected = 0
        last_error = ""
        print(f"Waiting for {args.endpoint}; press Q or Esc in the window to exit.", flush=True)
        while True:
            packet = subscriber.receive()
            if packet is not None:
                try:
                    source = parse_frame(packet, expected_role=args.camera_role)
                    frame = cv2.imdecode(np.frombuffer(source.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None or frame.shape != (source.height, source.width, 3):
                        raise ValueError("Invalid JPEG or decoded dimensions differ from header")
                except (ValueError, cv2.error) as exc:
                    rejected += 1
                    last_error = str(exc)
                    if rejected == 1:
                        print(f"Rejected frame: {last_error}", file=sys.stderr, flush=True)
                else:
                    cv2.imshow(_WINDOW, frame)
                    last_frame = time.monotonic()
                    count += 1
                    last_error = ""
                    if count == 1:
                        started = last_frame
                        print(
                            f"First frame: {source.role} {source.width}x{source.height}, seq={source.sequence}",
                            flush=True,
                        )
            now = time.monotonic()
            if now - last_notice >= 5:
                if now - last_frame >= 5:
                    detail = f" Last error: {last_error}" if last_error else ""
                    print(
                        f"No valid frame for {now - last_frame:.1f} seconds; "
                        f"check sender, endpoint and network. rejected={rejected}.{detail}",
                        flush=True,
                    )
                else:
                    print(
                        f"Viewer average: {max(0, count - 1) / max(now - started, 0.001):.1f} FPS; "
                        f"displayed={count} rejected={rejected}",
                        flush=True,
                    )
                last_notice = now
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    except cv2.error as exc:
        print(f"OpenCV preview failed: {exc}. {_GUI_HELP}", file=sys.stderr)
        return 1
    except (ValueError, zmq.ZMQError) as exc:
        print(f"Camera preview failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if subscriber is not None:
            subscriber.close()
        if window_open:
            try:
                cv2.destroyWindow(_WINDOW)
            except cv2.error:
                # Closing the window manually may already have destroyed it.
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
