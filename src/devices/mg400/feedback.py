"""Dobot MG400 real-time feedback packets (port 30004): assembly and parsing.

Layout: guide section 4 ("Real-time Feedback"), 1440 bytes per packet, little-endian.
Only fields defined in that table are read, at their documented offsets; reserved areas
(e.g. 1038-1119) are not interpreted, and nothing from another model's or version's
layout is used. Values are reported, not judged: BrakeStatus is kept as the raw byte
because the guide's own example of it is inconsistent.
"""
import math
import struct
from typing import Dict, Iterator, List, NamedTuple, Tuple

PACKET_SIZE = 1440
TEST_VALUE = 0x0123456789ABCDEF
ROBOT_TYPES = {1: "MG400", 2: "M1 Pro", 4: "M1 Pro (RS485)"}

#: name -> (offset, struct format, count)
FIELDS: Dict[str, Tuple[int, str, int]] = {
    "message_size": (0, "<H", 1),
    "robot_mode": (24, "<Q", 1),
    "timestamp_ms": (32, "<Q", 1),
    "test_value": (48, "<Q", 1),
    "speed_scaling": (64, "<d", 1),
    "q_target": (192, "<6d", 6),
    "q_actual": (432, "<6d", 6),
    "qd_actual": (480, "<6d", 6),
    "tool_vector_actual": (624, "<6d", 6),
    "tcp_speed_actual": (672, "<6d", 6),
    "user_index": (1012, "<B", 1),
    "tool_index": (1013, "<B", 1),
    "run_queued_cmd": (1014, "<B", 1),
    "pause_cmd_flag": (1015, "<B", 1),
    "velocity_ratio": (1016, "<B", 1),
    "xyz_velocity_ratio": (1019, "<B", 1),
    "xyz_acceleration_ratio": (1021, "<B", 1),
    "brake_status_raw": (1025, "<B", 1),
    "enable_status": (1026, "<B", 1),
    "error_status": (1029, "<B", 1),
    "robot_type": (1031, "<B", 1),
    "load": (1168, "<d", 1),
    "center_x": (1176, "<d", 1),
    "center_y": (1184, "<d", 1),
    "center_z": (1192, "<d", 1),
}


class FeedbackError(ValueError):
    """A packet that is not a valid 1440-byte feedback packet"""


def parse(packet: bytes) -> dict:
    """One packet -> dict of the FIELDS (arrays as lists). Raises FeedbackError on a wrong
    size, MessageSize or TestValue. Non-finite doubles are kept and listed in
    ``nonfinite`` (nothing here decides anything from them)"""
    if len(packet) != PACKET_SIZE:
        raise FeedbackError(f"packet is {len(packet)} bytes, not {PACKET_SIZE}")
    out = {}
    for name, (offset, fmt, count) in FIELDS.items():
        values = struct.unpack_from(fmt, packet, offset)
        out[name] = list(values) if count > 1 else values[0]
    if out["message_size"] != PACKET_SIZE:
        raise FeedbackError(f"MessageSize {out['message_size']} != {PACKET_SIZE}")
    if out["test_value"] != TEST_VALUE:
        raise FeedbackError(f"TestValue {out['test_value']:#x} != {TEST_VALUE:#x}")
    nonfinite = []
    for name, value in out.items():
        for v in (value if isinstance(value, list) else [value]):
            if isinstance(v, float) and not math.isfinite(v):
                nonfinite.append(name)
                break
    out["nonfinite"] = nonfinite
    out["out_of_range"] = [name for name, ok in (
        ("robot_mode", 1 <= out["robot_mode"] <= 11),
        ("robot_type", out["robot_type"] in ROBOT_TYPES),
        ("enable_status", out["enable_status"] in (0, 1)),
        ("error_status", out["error_status"] in (0, 1)),
        ("user_index", out["user_index"] <= 9),
        ("tool_index", out["tool_index"] <= 9),
        ("run_queued_cmd", out["run_queued_cmd"] in (0, 1)),
        ("pause_cmd_flag", out["pause_cmd_flag"] in (0, 1)),
        ("velocity_ratio", out["velocity_ratio"] <= 100),
        ("xyz_velocity_ratio", out["xyz_velocity_ratio"] <= 100),
        ("xyz_acceleration_ratio", out["xyz_acceleration_ratio"] <= 100),
    ) if not ok]
    return out


def build(**fields) -> bytes:
    """A synthetic packet (tests and the fake controller): MessageSize and TestValue set,
    the given FIELDS written, everything else zero"""
    buf = bytearray(PACKET_SIZE)
    values = {"message_size": PACKET_SIZE, "test_value": TEST_VALUE}
    values.update(fields)
    for name, value in values.items():
        offset, fmt, count = FIELDS[name]
        args = list(value) if count > 1 else [value]
        struct.pack_into(fmt, buf, offset, *args)
    return bytes(buf)


class Chunk(NamedTuple):
    seq: int               # packet number in the stream (0-based)
    offset: int            # byte offset of the packet in the stream
    data: bytes


class Assembler:
    """Cuts the byte stream of port 30004 into 1440-byte packets. A read may hold part
    of a packet or several; :attr:`tail` is what is left (a truncated packet at close)"""

    def __init__(self):
        self._buf = bytearray()
        self._seq = 0
        self._offset = 0

    def feed(self, data: bytes) -> Iterator[Chunk]:
        self._buf += data
        while len(self._buf) >= PACKET_SIZE:
            chunk = bytes(self._buf[:PACKET_SIZE])
            del self._buf[:PACKET_SIZE]
            yield Chunk(self._seq, self._offset, chunk)
            self._seq += 1
            self._offset += PACKET_SIZE

    @property
    def tail(self) -> int:
        return len(self._buf)


def stats(values: List[float]) -> dict:
    """min / median / p99 / max of a list (empty -> {})"""
    if not values:
        return {}
    s = sorted(values)
    n = len(s)
    return {"n": n, "min": s[0], "median": s[n // 2], "p99": s[min(n - 1, int(0.99 * n))],
            "max": s[-1]}
