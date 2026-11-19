"""src/devices/dobot/read_pose.py: framing, reply parsing, read-only queries, saving."""
import json
import struct

import pytest

from src.devices.dobot import read_pose as rp


class FakeSerial:
    """Answers GetPose / GetEndEffectorParams / name / SN; records what was written."""

    def __init__(self, pose=(250.0, 0.0, 50.0, 0.0, 0.0, 30.0, 40.0, 0.0), ee=(59.7, 0.0, 0.0),
                 noise=b"", silent=False):
        self.pose, self.ee, self.noise, self.silent = pose, ee, noise, silent
        self.written = []
        self.pending = b""

    def reset_input_buffer(self):
        self.pending = b""

    def write(self, data):
        self.written.append(bytes(data))
        if self.silent:
            return
        cmd = data[3]
        params = {rp.CMD_GET_POSE: struct.pack("<8f", *self.pose),
                  rp.CMD_GET_END_EFFECTOR_PARAMS: struct.pack("<3f", *self.ee),
                  rp.CMD_GET_DEVICE_NAME: b"Dobot\x00",
                  rp.CMD_GET_DEVICE_SN: b"DT123\x00"}[cmd]
        self.pending = self.noise + rp.frame(cmd, 0x00, params)

    def read(self, n):
        out, self.pending = self.pending[:n], self.pending[n:]
        return out


def test_frame_checksum():
    f = rp.frame(10)
    assert f[:2] == b"\xaa\xaa" and f[2] == 2
    assert (sum(f[3:-1]) + f[-1]) & 0xFF == 0


def test_parse_skips_garbage_and_other_replies():
    other = rp.frame(60, 0, b"\x00" * 12)
    mine = rp.frame(10, 0, b"\x01\x02")
    bad = bytearray(rp.frame(10, 0, b"\x09\x09")); bad[-1] ^= 0xFF
    assert rp.parse_reply(b"\x00\xaa" + other + bytes(bad) + mine, 10) == b"\x01\x02"
    assert rp.parse_reply(mine[:-1], 10) is None


def test_reader_sends_only_read_commands():
    ser = FakeSerial(noise=b"\xaa\x05junk")
    r = rp.DobotReader(ser)
    assert r.pose()[:3] == pytest.approx([250.0, 0.0, 50.0])
    assert r.end_effector() == pytest.approx([59.7, 0.0, 0.0])
    assert r.text(rp.CMD_GET_DEVICE_NAME) == "Dobot"
    for pkt in ser.written:
        assert pkt[4] & 0x01 == 0, "ctrl rw bit set: that would be a write command"
        assert pkt[3] in (rp.CMD_GET_POSE, rp.CMD_GET_END_EFFECTOR_PARAMS,
                          rp.CMD_GET_DEVICE_NAME, rp.CMD_GET_DEVICE_SN)


def test_reader_times_out():
    t = [0.0]

    def clock():
        t[0] += 0.5
        return t[0]

    with pytest.raises(TimeoutError):
        rp.DobotReader(FakeSerial(silent=True), timeout=2.0, clock=clock).pose()


def test_save_position(tmp_path, capsys):
    path = str(tmp_path / "pos.json")
    pose = [250.123, -10.0, 40.0, 0.0, 1, 2, 3, 4]
    rec = rp.save_position(path, "tube_a", pose, [0.0, 0.0, 0.0], "COM7")
    assert rec["x"] == 250.12 and rec["end_effector_offset"] == [0.0, 0.0, 0.0]
    rp.save_position(path, "tube_b", pose, [59.7, 0.0, 0.0], "COM7")
    assert "differs" in capsys.readouterr().out
    data = json.load(open(path, encoding="utf-8"))
    assert set(data["positions"]) == {"tube_a", "tube_b"}


def test_watch_and_save_are_exclusive(capsys):
    assert rp.main(["--port", "COM0", "--watch", "--save", "x"]) == 2
