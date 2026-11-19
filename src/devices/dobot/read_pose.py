"""Read a Dobot Magician's pose without moving it, and record named positions.

Teaching aid for flows and the tube-balancing runner: hand-guide the tip
(hold the unlock button on the forearm and move the arm), release the button,
then read the pose here. Only read commands are sent (GetPose, GetEndEffector-
Params, GetDeviceSN, GetDeviceName); the arm never moves.

The port is opened with RTS and DTR held low *before* open(). On Dobots with a
CP2102N USB bridge (VID 10C4 PID EA60) an RTS transition resets the
controller, which forgets its end-effector offset and reads its joint angles
wrong for minutes afterwards; pyserial's default open toggles RTS.

Usage (from the repository root)::

    python -m src.devices.dobot.read_pose --robot 5             # port from config.yaml
    python -m src.devices.dobot.read_pose --port COM7 --watch   # print every 0.5 s (Ctrl+C ends)
    python -m src.devices.dobot.read_pose --robot 5 --save tube_a
    python -m src.devices.dobot.read_pose --list-ports          # which COM port is which device

``--save NAME`` appends the pose to ``--out`` (default dobot_positions.json,
gitignored) together with the end-effector offset in force: coordinates are
only comparable between readings taken with the same offset.
"""
import argparse
import json
import os
import struct
import sys
import time
from datetime import datetime
from typing import Callable, Dict, List, Optional

DEFAULT_OUT = "dobot_positions.json"

CMD_GET_DEVICE_SN = 0
CMD_GET_DEVICE_NAME = 1
CMD_GET_POSE = 10
CMD_GET_END_EFFECTOR_PARAMS = 60

#: USB bridges seen on Dobot Magicians, by "VID:PID".
BRIDGES = {
    "1A86:55D4": "CH9102",
    "1A86:7523": "CH340",
    "10C4:EA60": "CP2102N (an RTS change resets the controller)",
}


def frame(cmd_id: int, ctrl: int = 0x00, params: bytes = b"") -> bytes:
    """``AA AA len id ctrl params checksum`` with checksum = two's complement of the payload sum."""
    payload = bytes([cmd_id, ctrl]) + params
    return b"\xaa\xaa" + bytes([len(payload)]) + payload + bytes([(-sum(payload)) & 0xFF])


def parse_reply(buf: bytes, cmd_id: int) -> Optional[bytes]:
    """Params of the first well-formed reply to ``cmd_id`` in ``buf``, or None."""
    # Every AA AA is a candidate: a stray 0xAA just before a real header makes
    # the first candidate claim a bogus length, so an incomplete or invalid
    # candidate is skipped rather than waited for.
    i = 0
    while True:
        i = buf.find(b"\xaa\xaa", i)
        if i < 0 or len(buf) < i + 3:
            return None
        n = buf[i + 2]
        end = i + 3 + n
        if len(buf) >= end + 1:
            body = buf[i + 3:end]
            if n >= 2 and (sum(body) + buf[end]) & 0xFF == 0 and body[0] == cmd_id:
                return body[2:]
        i += 1


def open_quiet(port: str, timeout: float = 0.2):
    """Open ``port`` at 115200 8N1 without raising RTS or DTR."""
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = timeout
    ser.write_timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class DobotReader:
    """Read-only Dobot queries over an already open serial object."""

    def __init__(self, ser, timeout: float = 2.0, clock: Callable[[], float] = time.monotonic):
        self.ser = ser
        self.timeout = timeout
        self.clock = clock

    def query(self, cmd_id: int) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(frame(cmd_id))
        deadline = self.clock() + self.timeout
        buf = b""
        while self.clock() < deadline:
            buf += self.ser.read(256)
            params = parse_reply(buf, cmd_id)
            if params is not None:
                return params
        raise TimeoutError(f"no reply to Dobot command {cmd_id} within {self.timeout:.1f} s "
                           "(is the Dobot powered and is this its port?)")

    def pose(self) -> List[float]:
        """[x, y, z, r, j1, j2, j3, j4] (mm, degrees)."""
        return list(struct.unpack_from("<8f", self.query(CMD_GET_POSE), 0))

    def end_effector(self) -> List[float]:
        """[xBias, yBias, zBias] (mm)."""
        return list(struct.unpack_from("<3f", self.query(CMD_GET_END_EFFECTOR_PARAMS), 0))

    def text(self, cmd_id: int) -> str:
        return self.query(cmd_id).rstrip(b"\x00").decode("ascii", "replace")


def format_pose(pose: List[float], ee: List[float]) -> str:
    x, y, z, r, j1, j2, j3, j4 = pose
    return (f"x={x:8.2f}  y={y:8.2f}  z={z:8.2f}  r={r:7.2f}   "
            f"J1={j1:7.2f} J2={j2:7.2f} J3={j3:7.2f} J4={j4:7.2f}   "
            f"end effector offset=({ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f})")


def save_position(path: str, name: str, pose: List[float], ee: List[float], port: str) -> Dict:
    """Add or replace ``name`` in the JSON positions file; returns the stored record."""
    data = {"positions": {}}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        data.setdefault("positions", {})
    record = {"x": round(pose[0], 2), "y": round(pose[1], 2), "z": round(pose[2], 2),
              "r": round(pose[3], 2), "joints": [round(v, 2) for v in pose[4:8]],
              "end_effector_offset": [round(v, 2) for v in ee],
              "port": port, "saved_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    offsets = {tuple(p.get("end_effector_offset", [])) for p in data["positions"].values()}
    if offsets and tuple(record["end_effector_offset"]) not in offsets:
        print("WARNING: the end-effector offset differs from earlier saved positions; "
              "positions taught with different offsets do not match each other.")
    data["positions"][name] = record
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)
    return record


def bridge_of(port: str) -> str:
    try:
        from serial.tools import list_ports
    except ImportError:
        return "unknown"
    for p in list_ports.comports():
        if p.device.lower() == port.lower() and p.vid is not None:
            key = f"{p.vid:04X}:{p.pid:04X}"
            return f"{key} {BRIDGES.get(key, '(not a known Dobot bridge)')}"
    return "unknown"


def port_from_config(robot_id: int) -> str:
    from src import config as lab_config
    ports = lab_config.get_robot_ports().get(robot_id) or {}
    port = ports.get("dobot_port")
    if not port:
        raise SystemExit(f"config.yaml has no robots.{robot_id}.dobot_port; give --port")
    return port


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Read a Dobot's pose without moving it (read-only)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--robot", type=int, help="robot id in config.yaml")
    src.add_argument("--port", help="serial port, e.g. COM7")
    src.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    ap.add_argument("--watch", action="store_true", help="print the pose every 0.5 s until Ctrl+C")
    ap.add_argument("--save", metavar="NAME", help="record the pose under NAME in --out")
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"positions file (default {DEFAULT_OUT})")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, "|", p.description, "|", p.hwid)
        return 0
    if args.robot is None and args.port is None:
        print("give --robot N or --port COMx (or --list-ports)")
        return 2
    if args.watch and args.save:
        print("--watch ends with Ctrl+C, so it cannot be combined with --save; "
              "watch first, then run again with --save")
        return 2
    port = args.port or port_from_config(args.robot)
    print(f"port {port}, USB bridge {bridge_of(port)}")
    ser = open_quiet(port)
    try:
        reader = DobotReader(ser)
        print(f"device {reader.text(CMD_GET_DEVICE_NAME)!r}, serial {reader.text(CMD_GET_DEVICE_SN)!r}")
        while True:
            pose, ee = reader.pose(), reader.end_effector()
            print(format_pose(pose, ee))
            if not args.watch:
                break
            time.sleep(0.5)
        if args.save:
            rec = save_position(args.out, args.save, pose, ee, port)
            print(f"saved {args.save}: x={rec['x']} y={rec['y']} z={rec['z']} -> "
                  f"{os.path.abspath(args.out)}")
    except KeyboardInterrupt:
        print()
    except TimeoutError as e:
        print(f"error: {e}")
        return 1
    finally:
        ser.close()      # RTS/DTR were never raised, so closing does not toggle them
    return 0


if __name__ == "__main__":
    sys.exit(main())
