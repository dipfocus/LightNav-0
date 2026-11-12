"""Receive the single-part XRAW v2 packets emitted by mjpg_zmq_sender.py.

The sender stamps packets AFTER JPEG encoding, not at sensor exposure. The
wall timestamp can be corrected by a measured clock offset and capture delay;
the remote monotonic timestamp cannot be compared to the local ROS clock.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from urllib.parse import urlsplit

HEADER = struct.Struct(">4sIIIIQQQ16s16sQI")
MAX_PACKET_BYTES = 8 * 1024 * 1024
MAX_PIXELS = 4096 * 4096
SOF_MARKERS = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


@dataclass(frozen=True)
class XrawFrame:
    width: int
    height: int
    sequence: int
    wall_ns: int
    monotonic_ns: int
    role: str
    jpeg: bytes


def _text(value: bytes) -> str:
    text, _, padding = value.partition(b"\0")
    if not text or any(padding):
        raise ValueError("invalid XRAW text field")
    try:
        return text.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("invalid UTF-8 in XRAW header") from exc


def _jpeg_size(data: bytes) -> tuple[int, int]:
    """Inspect JPEG SOF dimensions without decoding pixels or allocating an image."""
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise ValueError("XRAW payload is not a complete JPEG")
    offset = 2
    while offset < len(data) - 2:
        if data[offset] != 0xFF:
            raise ValueError("invalid JPEG marker")
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in {0xDA, 0xD9}:
            break
        if marker == 0x01 or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(data):
            break
        length = int.from_bytes(data[offset:offset + 2], "big")
        if length < 2 or offset + length > len(data):
            raise ValueError("truncated JPEG segment")
        if marker in SOF_MARKERS:
            if length < 8:
                raise ValueError("invalid JPEG frame header")
            height = int.from_bytes(data[offset + 3:offset + 5], "big")
            width = int.from_bytes(data[offset + 5:offset + 7], "big")
            return width, height
        offset += length
    raise ValueError("JPEG dimensions are missing")


def parse_frame(
    packet: bytes, *, expected_role: str = "head_fpv", max_packet_bytes: int = MAX_PACKET_BYTES,
) -> XrawFrame:
    if not HEADER.size < len(packet) <= max_packet_bytes:
        raise ValueError("XRAW packet is truncated or exceeds max_packet_bytes")
    (magic, version, width, height, channels, sequence, wall_ns, mono_ns,
     role_bytes, encoding_bytes, _send_mono_ns, flags) = HEADER.unpack_from(packet)
    if magic != b"XRAW" or version != 2:
        raise ValueError("expected an XRAW v2 packet")
    if not width or not height or width * height > MAX_PIXELS or channels != 3:
        raise ValueError("invalid XRAW image dimensions or channel count")
    role, encoding = _text(role_bytes), _text(encoding_bytes)
    if role != expected_role:
        raise ValueError(f"expected camera role {expected_role!r}, received {role!r}")
    if encoding != "jpeg" or flags != 0:
        raise ValueError("only XRAW v2 jpeg encoding with flags=0 is supported")
    if wall_ns <= 0 or mono_ns <= 0:
        raise ValueError("XRAW timestamps must be positive")
    jpeg = packet[HEADER.size:]
    if _jpeg_size(jpeg) != (width, height):
        raise ValueError("JPEG dimensions do not match the XRAW header")
    return XrawFrame(width, height, sequence, wall_ns, mono_ns, role, jpeg)


class FrameClock:
    """Validate sender time; never relabel a buffered frame with receipt time."""

    def __init__(
        self, *, clock_offset_s: float = 0.0, capture_delay_s: float = 0.0,
        max_frame_age_s: float = 0.5,
    ) -> None:
        if not all(math.isfinite(v) for v in (clock_offset_s, capture_delay_s, max_frame_age_s)):
            raise ValueError("camera clock parameters must be finite")
        if capture_delay_s < 0.0 or max_frame_age_s <= 0.0:
            raise ValueError("capture_delay_s must be nonnegative and max_frame_age_s positive")
        self.adjustment_ns = round(clock_offset_s * 1e9) - round(capture_delay_s * 1e9)
        self.max_age_ns = round(max_frame_age_s * 1e9)
        self.last_wall_ns = 0

    def stamp(self, frame: XrawFrame, now_ns: int) -> int:
        stamp_ns = frame.wall_ns + self.adjustment_ns
        if stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= self.max_age_ns:
            raise ValueError("camera timestamp is stale or in the future; check clock synchronization")
        if frame.wall_ns <= self.last_wall_ns:
            raise ValueError("duplicate or out-of-order camera timestamp")
        self.last_wall_ns = frame.wall_ns
        # Sequence can reset when the sender restarts; newer timestamps still work.
        return stamp_ns


def validate_endpoint(endpoint: str) -> None:
    try:
        parts = urlsplit(endpoint)
        valid = (
            parts.scheme == "tcp" and parts.hostname and parts.port is not None
            and 1 <= parts.port <= 65535 and not parts.username and not parts.password
            and not parts.path and not parts.query and not parts.fragment
            and parts.hostname not in {"0.0.0.0", "*", "::"}
        )
    except ValueError:
        valid = False
    if not valid:
        raise ValueError("endpoint must be tcp://HOST:PORT with a concrete host address")


class XrawSubscriber:
    """Only subscribes to an existing sender; never opens or starts a camera.

    All socket operations belong to one thread. CONFLATE is appropriate here
    because the inspected sender uses one ZMQ part per complete header+JPEG.
    """

    def __init__(self, endpoint: str, *, max_packet_bytes: int = MAX_PACKET_BYTES) -> None:
        validate_endpoint(endpoint)
        if isinstance(max_packet_bytes, bool) or not isinstance(max_packet_bytes, int) or max_packet_bytes <= HEADER.size:
            raise ValueError("max_packet_bytes must be an integer larger than the XRAW header")
        import zmq

        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._closed = False
        try:
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.CONFLATE, 1)
            self._socket.setsockopt(zmq.MAXMSGSIZE, max_packet_bytes)
            self._socket.setsockopt(zmq.SUBSCRIBE, b"")
            self._socket.connect(endpoint)
        except Exception:
            self.close()
            raise

    def receive(self) -> bytes | None:
        if self._closed:
            return None
        try:
            return self._socket.recv(flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            return None

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._socket.close(linger=0)
            self._context.term()
