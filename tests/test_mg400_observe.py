"""Dobot MG400 observation tool: read-only commands, reply / feedback parsing, the tool's flow.

No hardware and no network beyond 127.0.0.1: the fake controller (src/devices/mg400/fake.py)
listens on loopback only. The tests check what is written on the wire, not only the results.
"""
import json
import os
import socket
import struct
import subprocess
import sys
import textwrap
import time

import pytest

from src.devices.mg400 import feedback, observe, protocol
from src.devices.mg400.fake import LOOPBACK, FakeMG400
from src.devices.mg400.protocol import MG400ProtocolError

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ----------------------------------------------------------------------
# Commands: built, never free text
# ----------------------------------------------------------------------
def test_the_six_commands_and_their_exact_text():
    assert protocol.robot_mode().text == "RobotMode()"
    assert protocol.get_error_id().text == "GetErrorID()"
    assert protocol.get_pose().text == "GetPose()"
    assert protocol.get_pose(base_frames=True).text == "GetPose(User=0,Tool=0)"
    assert protocol.get_angle().text == "GetAngle()"
    assert protocol.positive_solution([1, 2, 3, 4]).text == \
        "PositiveSolution(1.000000,2.000000,3.000000,4.000000,0,0)"
    assert protocol.inverse_solution([300, 0, 50, 10], [0, 10, 20, 0]).text == \
        ("InverseSolution(300.000000,0.000000,50.000000,10.000000,0,0,1,"
         "{0.000000,10.000000,20.000000,0.000000})")


@pytest.mark.parametrize("text", [
    "RobotMode()\n", "GetAngle(1)", "PositiveSolution(nan,2,3,4,0,0)",
    "PositiveSolution(1.000000,2.000000,3.000000,4.000000,1,0)",
    "InverseSolution(1.000000,2.000000,3.000000,4.000000,0,0,1,}1.000000,2.000000,3.000000,4.000000{)",
    "GetPose(User=1,Tool=0)", "GetPose(Tool=0,User=0)",
    "EnableRobot()", "DisableRobot()", "ClearError()", "EmergencyStop()", "MovL(1,2,3,4)",
    "ResetRobot()", "SpeedFactor(50)", "User(1)", "Tool(1)", "Continue()",
    "RobotMode()EnableRobot()", "RobotMode();EnableRobot()", "RobotMode()\nEnableRobot()",
    "GetPose(EnableRobot())", "GetPose(User=0,Tool=0", "PositiveSolution(1,2,3,4,0,0)) ",
    "InverseSolution(1,2,3,4,0,0,1,{1,2,{3},4})", "getangle() ", " GetAngle()", "",
])
def test_anything_else_is_refused_before_writing(text):
    with pytest.raises(ValueError):
        protocol.check_command(text)


@pytest.mark.parametrize("bad", [[1, 2, 3], [1, 2, 3, 4, 5], [1, 2, float("nan"), 4],
                                 [1, 2, float("inf"), 4], [1, True, 3, 4], [1, "2", 3, 4]])
def test_numbers_are_checked(bad):
    with pytest.raises(ValueError):
        protocol.positive_solution(bad)
    with pytest.raises(ValueError):
        protocol.inverse_solution([1, 2, 3, 4], bad)


def test_the_dashboard_writes_only_command_objects():
    class Sock:
        sent = []

        def sendall(self, data):
            Sock.sent.append(data)

    dash = observe.Dashboard(Sock(), observe.Recorder.__new__(observe.Recorder), "x")
    with pytest.raises(TypeError):
        dash.write("EnableRobot()")
    with pytest.raises(ValueError):
        dash.write(protocol.Command("EnableRobot", "EnableRobot()"))
    with pytest.raises(ValueError):
        dash.write(protocol.Command("RobotMode", "RobotMode();EnableRobot()"))
    with pytest.raises(ValueError):
        dash.write(protocol.Command("RobotMode", "GetAngle()"))       # name must match
    assert Sock.sent == []


def test_no_motion_port_anywhere_in_the_package():
    folder = os.path.join(REPO_ROOT, "src", "devices", "mg400")
    for name in os.listdir(folder):
        if name.endswith(".py"):
            with open(os.path.join(folder, name), encoding="utf-8") as f:
                assert "30003" not in f.read(), name


# ----------------------------------------------------------------------
# Replies
# ----------------------------------------------------------------------
BENCH_ERRORID = "0,{[\n\t[],\n\t[],\n\t[],\n\t[],\n\t[],\n\t[]\n]\n},GetErrorID();"


def test_replies_seen_on_the_bench_parse():
    r = protocol.parse_reply("0,{4},RobotMode();")
    assert (r.error_id, protocol.parse_robot_mode(r.values), r.echo) == (0, 4, "RobotMode()")
    r = protocol.parse_reply("0,{308.141606,0.384430,67.673470,18.632881},GetPose();")
    assert protocol.parse_numbers(r.values, 4)[0] == 308.141606
    r = protocol.parse_reply("0,{0.071847,10.278645,16.846972,18.555908,0.000000,0.000000},GetAngle();")
    assert len(protocol.parse_numbers(r.values)) == 6
    r = protocol.parse_reply(BENCH_ERRORID)
    errs = protocol.parse_error_ids(r.values)
    assert len(errs) == 6 and protocol.no_alarm(errs)


def test_alarm_is_judged_by_the_inner_lists_not_the_leading_errorid():
    r = protocol.parse_reply("0,{[[22],[],[],[],[],[]]},GetErrorID();")
    assert r.error_id == 0 and not protocol.no_alarm(protocol.parse_error_ids(r.values))
    assert not protocol.no_alarm(protocol.parse_error_ids("[[],[],[-2],[],[],[]]"))


def test_reader_handles_split_and_joined_replies():
    reader = protocol.ReplyReader()
    whole = (BENCH_ERRORID + "0,{4},RobotMode();").encode()
    out = []
    for i in range(0, len(whole), 3):
        out += reader.feed(whole[i:i + 3])
    assert [r.echo for r in out] == ["GetErrorID()", "RobotMode()"] and reader.pending == ""


@pytest.mark.parametrize("raw", ["0,{4},RobotMode()", "x,{4},RobotMode();", "0,4,RobotMode();",
                                 "0,{4,RobotMode();", "0,{4};"])
def test_malformed_replies_raise(raw):
    with pytest.raises(MG400ProtocolError):
        protocol.parse_reply(raw)


def test_echo_rules():
    cmd = protocol.get_pose(base_frames=True)
    assert protocol.check_echo(protocol.parse_reply("0,{1,2,3,4},GetPose();"), cmd) == \
        "echo without parameters"
    assert protocol.check_echo(protocol.parse_reply("0,{1,2,3,4},GetPose(User=0,Tool=0);"), cmd) is None
    with pytest.raises(MG400ProtocolError):
        protocol.check_echo(protocol.parse_reply("0,{4},RobotMode();"), cmd)
    with pytest.raises(MG400ProtocolError):
        protocol.check_echo(protocol.parse_reply("0,{1,2,3,4},GetPose(User=1,Tool=0);"), cmd)


def test_unknown_robot_mode_and_bad_numbers():
    with pytest.raises(MG400ProtocolError):
        protocol.parse_robot_mode("12")
    with pytest.raises(MG400ProtocolError):
        protocol.parse_numbers("1,nan,3")
    with pytest.raises(MG400ProtocolError):
        protocol.parse_numbers("1,2", count=4)


# ----------------------------------------------------------------------
# Feedback packets
# ----------------------------------------------------------------------
def test_feedback_offsets_follow_the_guide_table():
    pkt = feedback.build(robot_mode=4, timestamp_ms=1234567890123, speed_scaling=0.5,
                         q_actual=[1, 2, 3, 4, 5, 6], qd_actual=[0.1] * 6,
                         tool_vector_actual=[300, 1, 60, 18, 0, 0], user_index=0, tool_index=0,
                         run_queued_cmd=0, brake_status_raw=0x3c, enable_status=0, error_status=0,
                         robot_type=1, load=0.0, center_z=5.5)
    assert len(pkt) == 1440
    # spot checks straight from the guide's byte positions
    assert struct.unpack_from("<H", pkt, 0)[0] == 1440
    assert struct.unpack_from("<Q", pkt, 24)[0] == 4
    assert struct.unpack_from("<Q", pkt, 48)[0] == 0x0123456789ABCDEF
    assert struct.unpack_from("<6d", pkt, 432) == (1, 2, 3, 4, 5, 6)
    assert pkt[1031] == 1 and pkt[1025] == 0x3c
    assert struct.unpack_from("<d", pkt, 1192)[0] == 5.5
    p = feedback.parse(pkt)
    assert p["robot_mode"] == 4 and p["timestamp_ms"] == 1234567890123
    assert p["q_actual"] == [1, 2, 3, 4, 5, 6] and p["tool_vector_actual"][0] == 300
    assert p["robot_type"] == 1 and p["center_z"] == 5.5 and p["nonfinite"] == []


def test_feedback_rejects_wrong_packets():
    good = feedback.build()
    with pytest.raises(feedback.FeedbackError):
        feedback.parse(good[:-1])
    bad = bytearray(good)
    struct.pack_into("<Q", bad, 48, 0)
    with pytest.raises(feedback.FeedbackError):
        feedback.parse(bytes(bad))
    bad = bytearray(good)
    struct.pack_into("<H", bad, 0, 1439)
    with pytest.raises(feedback.FeedbackError):
        feedback.parse(bytes(bad))
    assert feedback.parse(feedback.build(load=float("nan")))["nonfinite"] == ["load"]


def test_assembler_cuts_a_stream_read_in_odd_pieces():
    stream = b"".join(feedback.build(timestamp_ms=i) for i in range(3)) + b"\x00" * 100
    asm = feedback.Assembler()
    chunks = []
    for i in range(0, len(stream), 997):
        chunks += list(asm.feed(stream[i:i + 997]))
    assert [c.seq for c in chunks] == [0, 1, 2] and [c.offset for c in chunks] == [0, 1440, 2880]
    assert [feedback.parse(c.data)["timestamp_ms"] for c in chunks] == [0, 1, 2]
    assert asm.tail == 100


def test_feedback_names_keep_indices_apart_from_frame_arrays():
    assert "user_index" in feedback.FIELDS and "user" not in feedback.FIELDS
    assert "collision_state" not in feedback.FIELDS        # reserved area in this guide


# ----------------------------------------------------------------------
# The tool against the fake controller
# ----------------------------------------------------------------------
def _run(fake, tmp_path, seconds=1.0):
    def connect(host, port, timeout):
        assert host == LOOPBACK
        return observe._connect(LOOPBACK, {observe.DASHBOARD_PORT: fake.dashboard_port,
                                           observe.FEEDBACK_PORT: fake.feedback_port}[port], timeout)
    return observe.observe(LOOPBACK, str(tmp_path), seconds, "1.6.0.0", connect=connect,
                           out=lambda *a: None)


def test_full_observation_sends_only_allowed_commands(tmp_path):
    with FakeMG400(split_replies=True) as fake:
        report = _run(fake, tmp_path)
        sent = list(fake.received)
        fb_written = fake.feedback_bytes_received
    assert report["complete"] is True
    assert {c.split("(")[0] for c in sent} <= set(protocol.ALLOWED)
    assert fb_written == 0                                  # the feedback port is never written
    assert report["two_connections"]["outstanding"]["second_answered_s"] >= 0
    assert report["feedback"]["packets"] > 5 and report["feedback"]["rejected"] == 0
    assert len(report["kinematics"]) == 1 + len(list(observe.kinematics_grid()))
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["complete"] is True
    assert (tmp_path / "feedback.bin").stat().st_size % 1440 == 0
    assert any(c["echo_note"] == "echo without parameters" for c in report["commands"])


@pytest.mark.parametrize("mode, errors, why, expected", [
    (5, None, "mode 5", []),                                     # seen in the feedback first
    (7, None, "mode 7", []),
    (4, [[22], [], [], [], [], []], "alarm", ["RobotMode()", "GetErrorID()"]),
])
def test_other_states_end_the_observation_before_any_compute_command(tmp_path, mode, errors, why,
                                                                     expected):
    with FakeMG400(mode=mode, errors=errors) as fake:
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and why in " ".join(report["notes"])
    assert sent == expected


def test_a_change_during_the_observation_stops_the_kinematics(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd.startswith("PositiveSolution"):
                fake.mode = 5                               # someone enabled it
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False
    assert sum(c.startswith("PositiveSolution") for c in sent) == 1
    assert not any(c.startswith("InverseSolution") for c in sent)
    assert "5" in " ".join(report["notes"])


def test_dry_run_connects_to_nothing(monkeypatch, capsys):
    monkeypatch.setattr(observe, "_connect", lambda *a: pytest.fail("connected"))
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("connected"))
    assert observe.main(["--dry-run"]) == 0
    assert "never written" in capsys.readouterr().out


def test_mock_never_touches_the_given_address(monkeypatch, tmp_path):
    real = socket.create_connection
    seen = []

    def guarded(addr, *a, **k):
        seen.append(addr[0])
        assert addr[0] == LOOPBACK
        return real(addr, *a, **k)

    monkeypatch.setattr(socket, "create_connection", guarded)
    real_lookup = socket.getaddrinfo

    def lookup(host, *a, **k):
        assert host == LOOPBACK, f"name lookup of {host!r}"
        return real_lookup(host, *a, **k)

    monkeypatch.setattr(socket, "getaddrinfo", lookup)
    assert observe.main(["192.0.2.1", "--mock", "--seconds", "1", "--out", str(tmp_path)]) == 0
    assert seen and set(seen) == {LOOPBACK}


def test_ctrl_c_in_a_separate_process_leaves_a_partial_record_and_sends_nothing_more(tmp_path):
    script = textwrap.dedent(f"""
        import os, signal, sys, threading, time
        sys.path.insert(0, {REPO_ROOT!r})
        from src.devices.mg400 import observe
        from src.devices.mg400.fake import FakeMG400, LOOPBACK
        fake = FakeMG400()
        def connect(host, port, timeout):
            return observe._connect(LOOPBACK, {{observe.DASHBOARD_PORT: fake.dashboard_port,
                                     observe.FEEDBACK_PORT: fake.feedback_port}}[port], timeout)
        def interrupt():
            time.sleep(1.0)
            print("SENT_BEFORE", len(fake.received), flush=True)
            if os.name == "nt":
                import _thread; _thread.interrupt_main()
            else:
                os.kill(os.getpid(), signal.SIGINT)
        threading.Thread(target=interrupt, daemon=True).start()
        try:
            observe.observe(LOOPBACK, {str(tmp_path)!r}, 30.0, None, connect=connect, out=lambda *a: None)
            print("NOT_INTERRUPTED")
        except KeyboardInterrupt:
            time.sleep(0.3)
            print("SENT_AFTER", len(fake.received), flush=True)
            sys.exit(130)
    """)
    p = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
    assert p.returncode == 130, p.stdout + p.stderr
    lines = dict(line.split() for line in p.stdout.split("\n") if line.startswith("SENT_"))
    assert lines["SENT_BEFORE"] == lines["SENT_AFTER"]
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["complete"] is False and "ended_utc" in report


def test_a_refused_command_ends_the_observation(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply
        fake.reply = lambda cmd: "-1,{},GetPose();" if cmd.startswith("GetPose") else orig(cmd)
        with pytest.raises(protocol.MG400CommandError):
            _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent[-1] == "GetPose()" and not any(c.startswith("PositiveSolution") for c in sent)
    assert json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))["complete"] is False


def test_a_state_reply_on_the_second_connection_ends_it_even_if_it_recovers(tmp_path):
    with FakeMG400() as fake:
        orig, calls = fake.reply, {"n": 0}

        def reply(cmd):
            if cmd == "RobotMode()":
                calls["n"] += 1
                if calls["n"] == 3:                         # dashboard-2's first RobotMode
                    return "0,{5},RobotMode();"
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and "RobotMode 5" in " ".join(report["notes"])
    assert not any(c.startswith(("PositiveSolution", "InverseSolution")) for c in sent)


def test_a_feedback_alarm_after_a_forward_solution_stops_the_inverse(tmp_path):
    with FakeMG400() as fake:
        orig_reply, orig_packet, flag = fake.reply, fake.packet, {"alarm": False}

        def reply(cmd):
            if cmd.startswith("PositiveSolution"):
                flag["alarm"] = True
                time.sleep(0.05)                             # let a packet with the alarm go out
            return orig_reply(cmd)

        def packet():
            pkt = bytearray(orig_packet())
            if flag["alarm"]:
                pkt[1029] = 1                                # ErrorStatus
            return bytes(pkt)

        fake.reply, fake.packet = reply, packet
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and "error_status 1" in " ".join(report["notes"])
    assert not any(c.startswith("InverseSolution") for c in sent)


def test_no_valid_feedback_ends_before_any_command(tmp_path):
    with FakeMG400() as fake:
        fake.packet = lambda: b"\x00" * 1440                 # wrong MessageSize / TestValue
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and sent == []
    assert "rejected" in " ".join(report["notes"])


def test_feedback_lost_during_the_kinematics_ends_it(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd.startswith("PositiveSolution"):
                fake.drop_feedback()
                time.sleep(0.05)
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and "closed" in " ".join(report["notes"])
    assert not any(c.startswith("InverseSolution") for c in sent)


def test_a_reply_timeout_is_recorded_with_the_command(tmp_path, monkeypatch):
    monkeypatch.setattr(observe, "TIMEOUT_S", 0.3)
    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd == "GetPose()":
                fake.hold_replies.set()
            return orig(cmd)

        fake.reply = reply
        try:
            with pytest.raises(OSError):
                _run(fake, tmp_path)
        finally:
            fake.hold_replies.clear()
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    last = report["commands"][-1]
    assert last["command"] == "GetPose(User=0,Tool=0)" and last["reply"] is None   # held after GetPose()
    assert "timed out" in last["error"]
    assert report["complete"] is False and report["notes"]


def test_a_wrong_echo_is_recorded_with_the_command(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply
        fake.reply = lambda cmd: "0,{1,2,3,4},GetAngle();" if cmd == "GetPose()" else orig(cmd)
        with pytest.raises(MG400ProtocolError):
            _run(fake, tmp_path)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    last = report["commands"][-1]
    assert last["command"] == "GetPose()" and last["reply"].startswith("0,{1,2,3,4},GetAngle()")
    assert "echoes" in last["error"]


@pytest.mark.parametrize("values", ["", "1,2,3", "1,2,3,4,5"])
def test_reply_shapes_are_checked(tmp_path, values):
    with FakeMG400() as fake:
        orig = fake.reply
        fake.reply = lambda cmd: ("0,{" + values + "}," + cmd + ";"
                                  if cmd.startswith("InverseSolution") else orig(cmd))
        with pytest.raises(MG400ProtocolError):
            _run(fake, tmp_path)


def test_an_empty_or_short_error_list_is_not_no_alarm():
    with pytest.raises(MG400ProtocolError):
        protocol.parse_error_ids("[]")
    with pytest.raises(MG400ProtocolError):
        protocol.parse_error_ids("[[],[]]")


def test_offsets_independently_of_the_field_table():
    # written at the guide's byte positions directly, not through feedback.FIELDS
    buf = bytearray(1440)
    struct.pack_into("<H", buf, 0, 1440)
    struct.pack_into("<Q", buf, 24, 4)
    struct.pack_into("<Q", buf, 32, 1700000000123)
    struct.pack_into("<Q", buf, 48, 0x0123456789ABCDEF)
    struct.pack_into("<d", buf, 64, 0.25)
    struct.pack_into("<6d", buf, 432, 1.5, 2.5, 3.5, 4.5, 0, 0)
    struct.pack_into("<6d", buf, 480, 0.1, 0.2, 0.3, 0.4, 0, 0)
    struct.pack_into("<6d", buf, 624, 300.0, 1.0, 60.0, 18.0, 0, 0)
    for off, val in ((1012, 0), (1013, 0), (1014, 1), (1015, 0), (1016, 50), (1019, 40), (1021, 30),
                     (1025, 0x3c), (1026, 1), (1029, 0), (1031, 1)):
        buf[off] = val
    for off, val in ((1168, 0.2), (1176, 1.0), (1184, 2.0), (1192, 3.0)):
        struct.pack_into("<d", buf, off, val)
    p = feedback.parse(bytes(buf))
    assert (p["robot_mode"], p["timestamp_ms"], p["speed_scaling"]) == (4, 1700000000123, 0.25)
    assert p["q_actual"][:4] == [1.5, 2.5, 3.5, 4.5] and p["qd_actual"][3] == 0.4
    assert p["tool_vector_actual"][:4] == [300.0, 1.0, 60.0, 18.0]
    assert (p["run_queued_cmd"], p["velocity_ratio"], p["xyz_velocity_ratio"],
            p["xyz_acceleration_ratio"]) == (1, 50, 40, 30)
    assert (p["brake_status_raw"], p["enable_status"], p["robot_type"]) == (0x3c, 1, 1)
    assert (p["load"], p["center_x"], p["center_y"], p["center_z"]) == (0.2, 1.0, 2.0, 3.0)
    assert p["out_of_range"] == []


def test_out_of_range_values_are_flagged():
    p = feedback.parse(feedback.build(robot_mode=42, robot_type=9, velocity_ratio=200))
    assert {"robot_mode", "robot_type", "velocity_ratio"} <= set(p["out_of_range"])


def test_ctrl_c_through_the_cli_returns_130(tmp_path):
    script = textwrap.dedent(f"""
        import sys, threading, time, _thread
        sys.path.insert(0, {REPO_ROOT!r})
        from src.devices.mg400 import observe
        threading.Thread(target=lambda: (time.sleep(1.5), _thread.interrupt_main()), daemon=True).start()
        sys.exit(observe.main(["--mock", "--seconds", "30", "--out", {str(tmp_path)!r}]))
    """)
    p = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
    assert p.returncode == 130, p.stdout + p.stderr
    assert "not an emergency stop" in p.stdout
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["complete"] is False and "KeyboardInterrupt" in " ".join(report["notes"])


def test_a_silent_feed_ends_before_any_command(tmp_path):
    with FakeMG400() as fake:
        fake.packet = lambda: b""                            # connected, but nothing arrives
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and sent == []
    assert "no valid feedback packet" in " ".join(report["notes"])


def test_a_malformed_reply_whose_values_are_not_used_still_ends_it(tmp_path):
    with FakeMG400() as fake:
        orig, calls = fake.reply, {"n": 0}

        def reply(cmd):
            if cmd == "GetPose()":
                calls["n"] += 1
                if calls["n"] == 3:                         # dashboard-2, two-connection test
                    return "0,{1,2,3},GetPose();"
            return orig(cmd)

        fake.reply = reply
        with pytest.raises(MG400ProtocolError):
            _run(fake, tmp_path)
        sent = list(fake.received)
    assert not any(c.startswith(("PositiveSolution", "InverseSolution")) for c in sent)


def test_number_formatting_refuses_non_finite_values_itself():
    for bad in (float("nan"), float("inf"), -float("inf")):
        with pytest.raises(ValueError):
            protocol._number(bad, "x")



def _alarm_on(fake, trigger):
    """After the reply to the first command for which trigger(cmd) is true, the feedback
    shows ErrorStatus 1"""
    orig_reply, orig_packet, flag = fake.reply, fake.packet, {"alarm": False}

    def reply(cmd):
        out = orig_reply(cmd)
        if not flag["alarm"] and trigger(cmd):
            flag["alarm"] = True
            time.sleep(0.06)                                 # packets with the alarm go out
        return out

    def packet():
        pkt = bytearray(orig_packet())
        if flag["alarm"]:
            pkt[1029] = 1
        return bytes(pkt)

    fake.reply, fake.packet = reply, packet


def test_a_feedback_fault_stops_the_very_next_send(tmp_path):
    with FakeMG400() as fake:
        _alarm_on(fake, lambda cmd: cmd == "RobotMode()")
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent == ["RobotMode()"] and report["complete"] is False
    unsent = report["commands"][-1]
    assert unsent["command"] == "GetErrorID()" and "not sent" in unsent["error"]


def test_a_feedback_fault_during_a_pose_query_stops_the_following_queries(tmp_path):
    with FakeMG400() as fake:
        _alarm_on(fake, lambda cmd: cmd == "GetPose()")
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent[-1] == "GetPose()" and "GetPose(User=0,Tool=0)" not in sent
    assert report["complete"] is False


def test_a_fault_seen_after_the_last_check_is_not_complete(tmp_path, monkeypatch):
    orig_stop = observe.FeedbackRecorder.stop

    def stop(self):
        self._fault("late alarm")                            # arrives just before the stop
        orig_stop(self)

    monkeypatch.setattr(observe.FeedbackRecorder, "stop", stop)
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    assert report["complete"] is False and "late alarm" in " ".join(report["notes"])


@pytest.mark.parametrize("number", ["9" * 400 + ".000000", "1000001.000000"])
def test_huge_numbers_are_refused_at_the_send_boundary(number):
    text = "PositiveSolution(" + ",".join([number, "0.000000", "0.000000", "0.000000"]) + ",0,0)"
    with pytest.raises(ValueError):
        protocol.check_command(text, "PositiveSolution")
    with pytest.raises(ValueError):
        protocol.positive_solution([2e6, 0, 0, 0])


def test_the_report_is_strict_json_even_with_non_finite_feedback(tmp_path):
    with FakeMG400() as fake:
        orig = fake.packet
        fake.packet = lambda: _with_nan(orig())
        report = _run(fake, tmp_path)
    text = (tmp_path / "report.json").read_text(encoding="utf-8")

    def refuse(name):
        raise ValueError(name)

    json.loads(text, parse_constant=refuse)                  # no NaN / Infinity in the file
    assert report["complete"] is False and "non-finite" in " ".join(report["notes"])


def _with_nan(pkt):
    buf = bytearray(pkt)
    struct.pack_into("<d", buf, 1168, float("nan"))         # Load
    return bytes(buf)


#: every parsed field at the guide's byte position (section 4), written here by hand
GUIDE_LAYOUT = {
    "message_size": (0, "<H"), "robot_mode": (24, "<Q"), "timestamp_ms": (32, "<Q"),
    "test_value": (48, "<Q"), "speed_scaling": (64, "<d"), "q_target": (192, "<6d"),
    "q_actual": (432, "<6d"), "qd_actual": (480, "<6d"), "tool_vector_actual": (624, "<6d"),
    "tcp_speed_actual": (672, "<6d"), "user_index": (1012, "<B"), "tool_index": (1013, "<B"),
    "run_queued_cmd": (1014, "<B"), "pause_cmd_flag": (1015, "<B"), "velocity_ratio": (1016, "<B"),
    "xyz_velocity_ratio": (1019, "<B"), "xyz_acceleration_ratio": (1021, "<B"),
    "brake_status_raw": (1025, "<B"), "enable_status": (1026, "<B"), "error_status": (1029, "<B"),
    "robot_type": (1031, "<B"), "load": (1168, "<d"), "center_x": (1176, "<d"),
    "center_y": (1184, "<d"), "center_z": (1192, "<d"),
}


def test_every_field_at_its_own_guide_position():
    assert set(GUIDE_LAYOUT) == set(feedback.FIELDS)
    buf = bytearray(1440)
    expected = {}
    for i, (name, (offset, fmt)) in enumerate(sorted(GUIDE_LAYOUT.items())):
        if name == "message_size":
            values = [1440]
        elif name == "test_value":
            values = [0x0123456789ABCDEF]
        elif fmt == "<B":
            # non-zero everywhere, so a field read from a reserved (zero) byte shows
            values = [1 if name in ("enable_status", "error_status", "run_queued_cmd",
                                    "pause_cmd_flag") else (1 + i % 8 if "index" in name else 7 + i)]
        elif fmt == "<6d":
            values = [100.0 * i + k + 0.5 for k in range(6)]
        elif fmt == "<Q":
            values = [4 if name == "robot_mode" else 1000 + i]
        else:
            values = [i + 0.25]
        struct.pack_into(fmt, buf, offset, *values)
        expected[name] = values if fmt == "<6d" else values[0]
    p = feedback.parse(bytes(buf))
    for name, value in expected.items():
        assert p[name] == value, name


def test_a_mode_change_seen_only_on_the_dashboard_after_a_forward_solution_stops_the_inverse(tmp_path):
    with FakeMG400() as fake:
        orig, seen = fake.reply, {"fk": False}

        def reply(cmd):
            if cmd.startswith("PositiveSolution"):
                seen["fk"] = True
            elif cmd == "RobotMode()" and seen["fk"]:
                return "0,{5},RobotMode();"                 # the feedback still says 4
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert report["complete"] is False and "RobotMode 5" in " ".join(report["notes"])
    assert not any(c.startswith("InverseSolution") for c in sent)



class _FailingFile:
    """A file whose write or close raises (after ``ok_writes`` writes)"""

    def __init__(self, real, fail_on, ok_writes=0):
        self._real, self._fail_on, self._ok = real, fail_on, ok_writes
        self.name = real.name

    def write(self, data):
        if self._fail_on == "write":
            if self._ok <= 0:
                raise OSError("disk full")
            self._ok -= 1
        return self._real.write(data)

    def close(self):
        self._real.close()
        if self._fail_on == "close":
            raise OSError("flush failed")


@pytest.mark.parametrize("fail_on, ok_writes", [("close", 0), ("write", 3)])
def test_a_feedback_file_failure_is_not_complete(tmp_path, monkeypatch, fail_on, ok_writes):
    real_open = open

    def fake_open(path, *a, **k):
        f = real_open(path, *a, **k)
        return _FailingFile(f, fail_on, ok_writes) if str(path).endswith("feedback.csv") else f

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    assert report["complete"] is False
    assert report["feedback"]["fault"] and ("flush failed" in report["feedback"]["fault"]
                                            or "disk full" in report["feedback"]["fault"])


@pytest.mark.parametrize("flag, offset", [("run_queued_cmd", 1014), ("pause_cmd_flag", 1015),
                                          ("enable_status", 1026), ("error_status", 1029)])
def test_each_flag_alone_at_its_guide_position(flag, offset):
    buf = bytearray(feedback.build())
    buf[offset] = 1
    p = feedback.parse(bytes(buf))
    flags = ("run_queued_cmd", "pause_cmd_flag", "enable_status", "error_status")
    assert {f: p[f] for f in flags} == {f: int(f == flag) for f in flags}


def test_a_write_failure_after_the_stop_is_still_a_fault(tmp_path, monkeypatch):
    # the write happens after the stop flag is set (as when a stop and a write overlap);
    # only a socket receive error after our own stop may be ignored, never a file error
    recorders = []
    orig_init = observe.FeedbackRecorder.__init__

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        recorders.append(self)

    real_open = open

    class StopThenFail:
        def __init__(self, real):
            self._real, self.name = real, real.name

        def write(self, data):
            if recorders and recorders[0].recv_times:      # after the first packet
                recorders[0]._halt.set()                   # the stop has begun
                raise OSError("disk full")
            return self._real.write(data)

        def close(self):
            self._real.close()

    def fake_open(path, *a, **k):
        f = real_open(path, *a, **k)
        return StopThenFail(f) if str(path).endswith("feedback.bin") else f

    monkeypatch.setattr(observe.FeedbackRecorder, "__init__", init)
    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    assert report["complete"] is False and "disk full" in (report["feedback"]["fault"] or "")
