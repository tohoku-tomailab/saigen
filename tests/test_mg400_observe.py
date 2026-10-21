"""Dobot MG400 observation tool: read-only commands, reply / feedback parsing, the tool's flow.

No hardware and no network beyond 127.0.0.1: the fake controller (src/devices/mg400/fake.py)
listens on loopback only. The tests check what is written on the wire, not only the results.
"""
import json
import os
import re
import socket
import struct
import subprocess
import sys
import textwrap
import time

import pytest

from src.devices.mg400 import fake as fake_module
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
    assert protocol.get_angle().text == "GetAngle()"
    assert protocol.positive_solution([1, 2, 3, 4]).text == \
        "PositiveSolution(1.000000,2.000000,3.000000,4.000000,0,0)"
    assert protocol.inverse_solution([300, 0, 50, 10]).text == \
        "InverseSolution(300.000000,0.000000,50.000000,10.000000,0,0)"


@pytest.mark.parametrize("text", [
    "RobotMode()\n", "GetAngle(1)", "PositiveSolution(nan,2,3,4,0,0)",
    "PositiveSolution(1.000000,2.000000,3.000000,4.000000,1,0)",
    "InverseSolution(1.000000,2.000000,3.000000,4.000000,0,0,1,}1.000000,2.000000,3.000000,4.000000{)",
    "GetPose(User=0,Tool=0)", "GetPose(User=1,Tool=0)", "GetPose(Tool=0,User=0)",
    "EnableRobot()", "DisableRobot()", "ClearError()", "EmergencyStop()", "MovL(1,2,3,4)",
    "ResetRobot()", "SpeedFactor(50)", "User(1)", "Tool(1)", "Continue()",
    "RobotMode()EnableRobot()", "RobotMode();EnableRobot()", "RobotMode()\nEnableRobot()",
    "GetPose(EnableRobot())", "GetPose(User=0,Tool=0", "PositiveSolution(1,2,3,4,0,0)) ",
    "InverseSolution(1,2,3,4,0,0,1,{1,2,{3},4})", "getangle() ", " GetAngle()", "",
    # the guide's isJointNear / JointNear form, refused by the bench controller (-20000)
    "InverseSolution(1.000000,2.000000,3.000000,4.000000,0,0,1,{1.000000,2.000000,3.000000,4.000000})",
    "InverseSolution(1.000000,2.000000,3.000000,4.000000,0,0,1)",
    "InverseSolution(1.000000,2.000000,3.000000,4.000000,0,1)",
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
        protocol.inverse_solution(bad)


def test_the_dashboard_writes_only_command_objects():
    class Sock:
        sent = []

        def sendall(self, data):
            Sock.sent.append(data)

    rec = observe.Recorder.__new__(observe.Recorder)       # no folder: nothing may be saved
    rec.report = {"commands": []}
    dash = observe.Dashboard(Sock(), rec, "x")
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
    cmd = protocol.inverse_solution([300, 0, 50, 10])
    assert protocol.check_echo(protocol.parse_reply("0,{0,10,20,0},InverseSolution();"), cmd) == \
        "echo without parameters"
    assert protocol.check_echo(protocol.parse_reply("0,{0,10,20,0},%s;" % cmd.text), cmd) is None
    with pytest.raises(MG400ProtocolError):
        protocol.check_echo(protocol.parse_reply("0,{4},RobotMode();"), cmd)
    with pytest.raises(MG400ProtocolError):
        protocol.check_echo(protocol.parse_reply(
            "0,{0,10,20,0},InverseSolution(1.000000,0.000000,50.000000,10.000000,0,0);"), cmd)


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
def _run(fake, tmp_path, seconds=1.0, wrap=None):
    def connect(host, port, timeout):
        assert host == LOOPBACK
        s = observe._connect(LOOPBACK, {observe.DASHBOARD_PORT: fake.dashboard_port,
                                        observe.FEEDBACK_PORT: fake.feedback_port}[port], timeout)
        return wrap(s) if wrap is not None and port == observe.DASHBOARD_PORT else s
    return observe.observe(LOOPBACK, str(tmp_path), seconds, "1.6.0.0", connect=connect,
                           out=lambda *a: None)


def test_full_observation_sends_only_allowed_commands(tmp_path):
    with FakeMG400(split_replies=True, echo_bare={"InverseSolution"}) as fake:
        orig_packet = fake.packet

        def packet():                    # distinct non-zero frame indices, taken from the packet
            pkt = bytearray(orig_packet())
            pkt[1012], pkt[1013] = 3, 5
            return bytes(pkt)

        fake.packet = packet
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
    assert not any(c.startswith("GetPose(User") for c in sent)       # refused by the bench arm
    compare = report["compare_mapping_unverified"]
    assert len(compare["tool_vector_actual_minus_GetPose_first4"]) == 4
    assert compare["packet_user_index"] == 3 and compare["packet_tool_index"] == 5


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
        orig, flipped = fake.reply, {"done": False}

        def reply(cmd):
            # dashboard-2's first RobotMode answers 5 once; later replies are 4 again
            if cmd == "RobotMode()" and fake.conn_number.n == 2 and not flipped["done"]:
                flipped["done"] = True
                return "0,{5},RobotMode();"
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        sent = list(fake.received_on)
    assert flipped["done"] and report["complete"] is False
    assert "RobotMode 5" in " ".join(report["notes"])
    assert sent[-1] == (2, "RobotMode()")                  # nothing after it, on either connection
    assert report["two_connections"]["result"] is None


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
            with pytest.raises(protocol.MG400ConnectionError):
                _run(fake, tmp_path)
        finally:
            fake.hold_replies.clear()
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    last = report["commands"][-1]
    assert last["command"] == "GetAngle()" and last["reply"] is None   # held after GetPose()
    assert "no reply within" in last["error"]
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


def test_the_fake_refuses_the_joint_near_form_as_the_bench_did():
    """the fake mirrors the bench reply; the tool itself can no longer build that text"""
    old = ("InverseSolution(300.000000,0.000000,50.000000,10.000000,0,0,1,"
           "{0.000000,10.000000,20.000000,0.000000})")
    with pytest.raises(ValueError):
        protocol.check_command(old)
    with FakeMG400() as fake:
        assert fake.reply(old) == f"-20000,{{}},{old};"
        assert fake.reply(protocol.inverse_solution([300, 0, 50, 10]).text).startswith("0,{")


def test_a_refused_inverse_solution_ends_the_run_and_nothing_follows(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply
        fake.reply = lambda cmd: (f"-20000,{{}},{cmd};" if cmd.startswith("InverseSolution")
                                  else orig(cmd))
        with pytest.raises(protocol.MG400CommandError) as info:
            _run(fake, tmp_path)
        sent = list(fake.received)
    assert "-20000" in str(info.value)
    assert sent[-1].startswith("InverseSolution") and \
        sum(c.startswith("InverseSolution") for c in sent) == 1
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["complete"] is False
    assert report["commands"][-1]["reply"].startswith("-20000,")


def test_only_the_six_parameter_inverse_form_is_sent_and_any_branch_is_recorded(tmp_path):
    angles = (45.0, 30.0, 60.0, -20.0, 0.0, 0.0)       # unlike any grid joint set
    with FakeMG400(angles=angles) as fake:
        report = _run(fake, tmp_path)
        sent = [c for c in fake.received if c.startswith("InverseSolution")]
    assert report["complete"] is True
    assert sent and all(re.fullmatch(r"InverseSolution\((-?\d+\.\d{6},){4}0,0\)", c) for c in sent)
    assert "nearest the arm's current joint angles" in report["kinematics_note"]
    pairs = report["kinematics"]
    assert len(pairs) == len(sent)
    assert all(p["inverse_solution"] == list(angles[:4]) for p in pairs)
    assert any(p["joints"] != list(angles[:4]) for p in pairs)


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
        orig = fake.reply

        def reply(cmd):
            if cmd == "GetPose()" and fake.conn_number.n == 2:      # dashboard-2
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


# ----------------------------------------------------------------------
# After the first bench run: GetPose(User=0,Tool=0) refused; a locked report.json
# ----------------------------------------------------------------------
def _until(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while not pred():
        assert time.monotonic() < end, "condition not reached"
        time.sleep(0.005)


def _capture_feedback(monkeypatch):
    recorders = []
    orig_init = observe.FeedbackRecorder.__init__

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        recorders.append(self)

    monkeypatch.setattr(observe.FeedbackRecorder, "__init__", init)
    return recorders


def _patch_replace(monkeypatch, fail):
    """os.replace for the tool: fail(call number, text of the tmp file) -> exception or None.
    Every call counts, retries included"""
    real = os.replace
    calls = []

    def replace(src, dst):
        calls.append(src)
        with open(src, encoding="utf-8") as f:
            text = f.read()
        e = fail(len(calls), text)
        if e is not None:
            raise e
        return real(src, dst)

    monkeypatch.setattr(observe.os, "replace", replace)
    monkeypatch.setattr(observe, "REPLACE_RETRY_S", (0.001,) * 5)
    return calls


def _main_with(monkeypatch, tmp_path, setup=None, cls=FakeMG400):
    """main() --mock with the fake it creates handed to setup() first"""
    made = []

    def factory(*a, **k):
        f = cls(*a, **k)
        if setup is not None:
            setup(f)
        made.append(f)
        return f

    monkeypatch.setattr(fake_module, "FakeMG400", factory)
    code = observe.main(["--mock", "--seconds", "1", "--out", str(tmp_path)])
    return code, made


def _lines(out, start):
    return [line for line in out.splitlines() if line.startswith(start)]


class _Wrapped:
    def __init__(self, sock):
        self._sock = sock

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_a_refused_command_reaches_the_cli_as_that_error(tmp_path, monkeypatch, capsys):
    def setup(f):
        orig = f.reply
        # synthetic, modelled on the bench reply -30001,{},GetPose(User=0,Tool=0);
        f.reply = lambda cmd: "-30001,{},GetAngle();" if cmd == "GetAngle()" else orig(cmd)

    code, made = _main_with(monkeypatch, tmp_path, setup)
    out = capsys.readouterr().out
    assert code == 1 and _lines(out, "error: GetAngle() -> ErrorID -30001")
    assert "record also failed" not in out and made[0].received[-1] == "GetAngle()"


def test_a_rename_refused_twice_then_allowed_loses_nothing(tmp_path, monkeypatch):
    _patch_replace(monkeypatch, lambda n, text: PermissionError("locked") if n in (2, 3) else None)
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    assert report["complete"] is True and "save_errors" not in report
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk == json.loads(json.dumps(observe._finite(report)))
    assert not (tmp_path / "report.json.tmp").exists()


def test_every_command_is_on_disk_before_its_bytes_leave(tmp_path):
    entries = [0]
    labels = iter(["dashboard-1", "dashboard-2"])

    class CheckedSend(_Wrapped):
        def __init__(self, sock):
            super().__init__(sock)
            self.label = next(labels)

        def sendall(self, data):
            entries[0] += 1
            report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
            last = report["commands"][-1]
            assert len(report["commands"]) == entries[0]
            assert last["connection"] == self.label and last["command"] == data.decode("ascii")
            assert "planned_utc" in last and "sent_utc" not in last     # this very entry
            return self._sock.sendall(data)

    with FakeMG400() as fake:
        report = _run(fake, tmp_path, wrap=CheckedSend)
        sent = list(fake.received)
    assert report["complete"] is True and entries[0] == len(sent) > 20


def test_a_feed_gap_during_a_save_retry_is_not_forgotten_when_the_feed_resumes(tmp_path, monkeypatch):
    recorders = _capture_feedback(monkeypatch)
    state = {}
    with FakeMG400() as fake:
        orig_packet = fake.packet

        def fail(n, text):
            # call 6 is the save before GetPose() is sent (1: start, 2-5: RobotMode, GetErrorID)
            if n == 6 and not state:
                fb = recorders[0]
                state["sent"] = len(fake.received)
                fake.packet = lambda: b""                        # the feed pauses ...
                _until(lambda: time.monotonic() - fb.last_valid_t > 0.6)
                have = len(fb.recv_times)
                fake.packet = orig_packet                        # ... and resumes
                _until(lambda: len(fb.recv_times) >= have + 3)
                return PermissionError("locked")                 # the retry then succeeds
            return None

        _patch_replace(monkeypatch, fail)
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert state["sent"] == 2 and len(sent) == 2
    assert report["complete"] is False and "feedback gap" in " ".join(report["notes"])


@pytest.mark.parametrize("offset, fmt, value, why", [(24, "<Q", 5, "mode 5"),
                                                     (1029, "<B", 1, "error_status 1")])
def test_a_feedback_fault_that_recovers_still_ends_it(tmp_path, monkeypatch, offset, fmt, value, why):
    recorders = _capture_feedback(monkeypatch)
    with FakeMG400() as fake:
        orig_reply, orig_packet = fake.reply, fake.packet
        bad = {"left": 0, "done": False}

        def packet():
            pkt = bytearray(orig_packet())
            if bad["left"] > 0:
                bad["left"] -= 1
                struct.pack_into(fmt, pkt, offset, value)
            return bytes(pkt)

        def reply(cmd):
            out = orig_reply(cmd)
            if cmd == "GetPose()" and not bad["done"]:
                bad["done"] = True
                fb = recorders[0]
                have = len(fb.recv_times)
                bad["left"] = 3                                  # three bad packets, then good
                _until(lambda: bad["left"] == 0 and len(fb.recv_times) >= have + 6)
            return out

        fake.reply, fake.packet = reply, packet
        report = _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent[-1] == "GetPose()" and report["complete"] is False
    assert why in " ".join(report["notes"])


_GOOD = feedback.build(robot_mode=4, robot_type=1)


def _feed_pair(tmp_path, recv_timeout):
    a, b = socket.socketpair()
    fb = observe.FeedbackRecorder(a, str(tmp_path), recv_timeout=recv_timeout)
    fb.start()
    return fb, b


def test_a_gap_seen_when_the_next_packet_arrives_is_a_fault(tmp_path):
    fb, b = _feed_pair(tmp_path, recv_timeout=5.0)       # the timeout side cannot fire
    try:
        for _ in range(3):
            b.sendall(_GOOD)
        _until(lambda: len(fb.recv_times) == 3)
        time.sleep(0.6)
        b.sendall(_GOOD)
        _until(lambda: len(fb.recv_times) == 4)
        assert fb.fault and fb.fault.startswith("feedback gap")
    finally:
        fb.stop()
        fb.join(timeout=2.0)
        b.close()


def test_a_gap_seen_while_waiting_for_a_packet_is_a_fault(tmp_path):
    fb, b = _feed_pair(tmp_path, recv_timeout=0.05)
    try:
        for _ in range(3):
            b.sendall(_GOOD)
        _until(lambda: fb.fault is not None)               # no further packet is sent
        assert fb.fault.startswith("feedback gap") and len(fb.recv_times) == 3
    finally:
        fb.stop()
        fb.join(timeout=2.0)
        b.close()


@pytest.mark.parametrize("error, attempts", [(PermissionError("locked"), 6), (OSError("busy"), 1)])
def test_a_rename_that_keeps_failing_ends_it_and_the_tmp_holds_the_latest(tmp_path, monkeypatch,
                                                                         error, attempts):
    state = {}
    with FakeMG400() as fake:
        def fail(n, text):
            if n >= 4:                                   # the save before GetErrorID is sent
                state.setdefault("sent", len(fake.received))
                return error
            return None

        calls = _patch_replace(monkeypatch, fail)
        with pytest.raises(observe.MG400RecordError) as info:
            _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent == ["RobotMode()"] and state["sent"] == 1
    assert len(calls) == 3 + 2 * attempts                # the failed save, then the final one
    tmp = json.loads((tmp_path / "report.json.tmp").read_text(encoding="utf-8"))
    assert len(tmp["save_errors"]) == 2 and "ended_utc" in tmp
    assert "report.json.tmp holds the latest" in info.value.recorder.describe()


def test_a_failed_first_save_opens_no_connection(tmp_path, monkeypatch, capsys):
    real_open = open

    def fake_open(path, *a, **k):
        if str(path).endswith("report.json.tmp"):
            raise OSError("denied")
        return real_open(path, *a, **k)

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("connected"))
    code, made = _main_with(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    assert code == 1 and made[0].received == []
    line = _lines(out, "record error: could not write report.json.tmp")
    assert line and "no save of this run was confirmed" in line[0]


def test_the_record_line_tells_the_state_after_the_last_save(tmp_path, monkeypatch, capsys):
    # one save in the middle fails, the final save succeeds: the line says report.json is
    # current (the state at the end), and the run still fails
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if n == 4 else None)
    code, made = _main_with(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    line = _lines(out, "record error: could not rename report.json.tmp to report.json")
    assert code == 1 and line and "OSError('busy')" in line[0] and "report.json is current" in line[0]
    assert made[0].received == ["RobotMode()"]


def test_an_ended_observation_whose_record_failed_on_the_way_is_not_a_success(tmp_path, monkeypatch,
                                                                            capsys):
    # pre-send save succeeds -> the guard ends it (feedback alarm) -> the save in the send's
    # error branch fails -> the final save succeeds
    once = {}

    def fail(n, text):
        if '"not sent' in text and not once:
            once["failed"] = True
            return OSError("busy")
        return None

    _patch_replace(monkeypatch, fail)
    recorders = _capture_feedback(monkeypatch)

    def setup(f):
        orig_reply, orig_packet, alarm = f.reply, f.packet, {"on": False}

        def reply(cmd):
            out = orig_reply(cmd)
            if cmd == "GetPose()" and not alarm["on"]:
                alarm["on"] = True
                _until(lambda: recorders[0].fault is not None)   # the receiver has the alarm
            return out

        def packet():
            pkt = bytearray(orig_packet())
            if alarm["on"]:
                pkt[1029] = 1
            return bytes(pkt)

        f.reply, f.packet = reply, packet

    code, made = _main_with(monkeypatch, tmp_path, setup)
    out = capsys.readouterr().out
    assert once and code == 1
    line = _lines(out, "record error:")
    assert line and "report.json is current" in line[0]
    assert made[0].received[-1] == "GetPose()"
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["complete"] is False and report["save_errors"]


class _TmpFile:
    def __init__(self, real, stage):
        self._real, self._stage = real, stage

    def write(self, s):
        return self._real.write(s)

    def flush(self):
        if self._stage == "flush":
            raise OSError("flush failed")
        return self._real.flush()

    def fileno(self):
        return self._real.fileno()

    def close(self):
        self._real.close()
        if self._stage == "close":
            raise OSError("close failed")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@pytest.mark.parametrize("stage", ["flush", "fsync", "close"])
def test_a_failed_tmp_write_sends_nothing_more_and_says_what_is_on_disk(tmp_path, monkeypatch, stage):
    real_open, real_fsync = open, os.fsync
    state = {"opens": 0, "sent": None}
    fakes = []

    def fake_open(path, *a, **k):
        f = real_open(path, *a, **k)
        if str(path).endswith("report.json.tmp"):
            state["opens"] += 1
            if state["opens"] >= 4:                      # the save before GetErrorID is sent
                if state["sent"] is None:
                    state["sent"] = len(fakes[0].received)
                if stage != "fsync":
                    return _TmpFile(f, stage)
        return f

    def fsync(fd):
        if stage == "fsync" and state["opens"] >= 4:
            raise OSError("fsync failed")
        return real_fsync(fd)

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    monkeypatch.setattr(observe.os, "fsync", fsync)
    with FakeMG400() as fake:
        fakes.append(fake)
        with pytest.raises(observe.MG400RecordError) as info:
            _run(fake, tmp_path)
        sent = list(fake.received)
    assert sent == ["RobotMode()"] and state["sent"] == 1
    assert f"{stage} failed" in str(info.value)
    assert "report.json holds the last successful save of this run (save 3)" in \
        info.value.recorder.describe()


class _PartialTmp(_TmpFile):
    """Writes the first chunks in full, then fails (the file holds the start of the JSON)"""

    def __init__(self, real):
        super().__init__(real, None)
        self._left = 20

    def write(self, s):
        if self._left <= 0:
            raise OSError("disk full")
        self._left -= 1
        return self._real.write(s)


@pytest.mark.parametrize("how", ["open", "partial"])
def test_a_failed_rewrite_after_a_failed_rename_is_not_called_the_latest_record(tmp_path,
                                                                                monkeypatch, how):
    # renames fail from the 4th save; the rewrite of the tmp file after each failed rename
    # fails too (opens 5 and 7), the plain writes succeed
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if n >= 4 else None)
    real_open = open
    opens = [0]

    def fake_open(path, *a, **k):
        if str(path).endswith("report.json.tmp"):
            opens[0] += 1
            if opens[0] in (5, 7):
                if how == "open":
                    raise OSError("denied")
                return _PartialTmp(real_open(path, *a, **k))
        return real_open(path, *a, **k)

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    with FakeMG400() as fake:
        with pytest.raises(observe.MG400RecordError) as info:
            _run(fake, tmp_path)
    assert opens[0] == 7
    assert info.value.recorder.describe().startswith(
        "report.json holds the last successful save of this run (save 3)")
    assert len(json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))["commands"]) == 1
    if how == "partial":                                 # what the message warns about
        text = (tmp_path / "report.json.tmp").read_text(encoding="utf-8")
        assert text.startswith('{\n  "tool": "src.devices.mg400.observe"') and len(text) > 40
        with pytest.raises(ValueError):
            json.loads(text)


def _fail_tmp_writes(monkeypatch, stage, armed):
    """From armed() on, every write of report.json.tmp fails at ``stage``"""
    real_open, real_fsync = open, os.fsync

    def fake_open(path, *a, **k):
        f = real_open(path, *a, **k)
        if str(path).endswith("report.json.tmp") and armed():
            if stage == "partial":
                return _PartialTmp(f)
            if stage in ("flush", "close"):
                return _TmpFile(f, stage)
        return f

    def fsync(fd):
        if stage == "fsync" and armed():
            raise OSError("fsync failed")
        return real_fsync(fd)

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    monkeypatch.setattr(observe.os, "fsync", fsync)


@pytest.mark.parametrize("stage", ["flush", "fsync", "close", "partial"])
@pytest.mark.parametrize("trigger", ["refusal", "send"])
def test_a_tmp_write_failure_on_an_error_path_keeps_the_original_error(tmp_path, monkeypatch,
                                                                       stage, trigger):
    armed = {"on": False}
    _fail_tmp_writes(monkeypatch, stage, lambda: armed["on"])
    count = [0]

    class BrokenSend(_Wrapped):
        def sendall(self, data):
            count[0] += 1
            if count[0] == 3:
                armed["on"] = True
                raise ConnectionResetError("reset by peer")
            return self._sock.sendall(data)

    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd == "GetAngle()":
                armed["on"] = True
                return "-30001,{},GetAngle();"
            return orig(cmd)

        if trigger == "refusal":
            fake.reply = reply
        expected = protocol.MG400CommandError if trigger == "refusal" else protocol.MG400ConnectionError
        with pytest.raises(expected) as info:
            _run(fake, tmp_path, wrap=BrokenSend if trigger == "send" else None)
        sent = list(fake.received)
    rec = info.value.mg400_recorder
    assert rec.failed and not rec.tmp_current and not rec.report_current
    assert sent[-1] == ("GetAngle()" if trigger == "refusal" else "GetErrorID()")


class _BreakableStdout:
    """Passes to the real stream until ``broken`` is set, then every write fails"""

    def __init__(self, real):
        self._real, self.broken = real, False

    def write(self, s):
        if self.broken:
            raise OSError("stdout disk full")
        return self._real.write(s)

    def flush(self):
        if self.broken:
            raise OSError("stdout disk full")
        return self._real.flush()


def test_a_failing_notice_does_not_turn_an_ended_observation_into_an_error(tmp_path):
    def out(text):
        if text.startswith("observation ended"):
            raise OSError("stdout disk full")

    with FakeMG400(mode=5) as fake:
        report = observe.observe(
            LOOPBACK, str(tmp_path), 1.0, None, out=out,
            connect=lambda h, p, t: observe._connect(
                LOOPBACK, {observe.DASHBOARD_PORT: fake.dashboard_port,
                           observe.FEEDBACK_PORT: fake.feedback_port}[p], t))
    assert report["complete"] is False and "mode 5" in " ".join(report["notes"])


@pytest.mark.parametrize("trigger, code", [("refusal", 1), ("ctrl-c", 130)])
def test_a_failing_stdout_keeps_the_exit_code(tmp_path, monkeypatch, trigger, code):
    stream = _BreakableStdout(sys.stdout)
    monkeypatch.setattr(sys, "stdout", stream)
    if trigger == "ctrl-c":
        orig_check = observe.FeedbackRecorder.check
        calls = [0]

        def check(self, now=None):
            calls[0] += 1
            if calls[0] == 6:
                stream.broken = True
                raise KeyboardInterrupt
            return orig_check(self, now)

        monkeypatch.setattr(observe.FeedbackRecorder, "check", check)
        setup = None
    else:
        def setup(f):
            orig = f.reply

            def reply(cmd):
                if cmd == "GetAngle()":
                    stream.broken = True
                    return "-30001,{},GetAngle();"
                return orig(cmd)

            f.reply = reply
    got, made = _main_with(monkeypatch, tmp_path, setup)
    assert stream.broken and got == code


@pytest.mark.parametrize("streams", ["stdout", "stderr", "stdout,stderr"])
@pytest.mark.parametrize("trigger, code", [("refusal", 1), ("ctrl-c", 130)])
def test_a_stream_that_cannot_be_flushed_keeps_the_process_exit_code(tmp_path, trigger, code,
                                                                     streams):
    # Python turns a failed flush of stdout / stderr at shutdown into exit code 120. Once the
    # streams break, opening any file fails too (as when out of file descriptors)
    script = textwrap.dedent(f"""
        import builtins, sys
        sys.path.insert(0, {REPO_ROOT!r})
        from src.devices.mg400 import fake as fake_module, observe

        class Breakable:
            broken = False
            def __init__(self, real):
                self.real = real
            def write(self, s):
                if Breakable.broken:
                    raise OSError("disk full")
                return self.real.write(s)
            def flush(self):
                if Breakable.broken:
                    raise OSError("disk full")
                return self.real.flush()

        for name in {streams!r}.split(","):
            setattr(sys, name, Breakable(getattr(sys, name)))
        real_open = builtins.open
        def failing_open(*a, **k):
            if Breakable.broken:
                raise OSError(24, "Too many open files")
            return real_open(*a, **k)
        builtins.open = failing_open

        Fake = fake_module.FakeMG400
        if {trigger!r} == "ctrl-c":
            orig_check = observe.FeedbackRecorder.check
            calls = [0]
            def check(self, now=None):
                calls[0] += 1
                if calls[0] == 6:
                    Breakable.broken = True
                    raise KeyboardInterrupt
                return orig_check(self, now)
            observe.FeedbackRecorder.check = check
            factory = Fake
        else:
            def factory(*a, **k):
                f = Fake(*a, **k)
                orig = f.reply
                def reply(cmd):
                    if cmd == "GetAngle()":
                        Breakable.broken = True
                        return "-30001,{{}},GetAngle();"
                    return orig(cmd)
                f.reply = reply
                return f
        fake_module.FakeMG400 = factory
        observe._exit(observe.main(["--mock", "--seconds", "1", "--out", {str(tmp_path)!r}]))
    """)
    p = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
    assert p.returncode == code, p.stdout + p.stderr


def test_a_failing_stderr_does_not_replace_the_original_error(tmp_path, monkeypatch):
    class BrokenStream:
        def write(self, s):
            raise OSError("stderr disk full")

        def flush(self):
            raise OSError("stderr disk full")

    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if "-30001" in text else None)
    with FakeMG400() as fake:
        orig = fake.reply
        fake.reply = lambda cmd: "-30001,{},GetAngle();" if cmd == "GetAngle()" else orig(cmd)
        monkeypatch.setattr(sys, "stderr", BrokenStream())
        try:
            with pytest.raises(protocol.MG400CommandError):
                _run(fake, tmp_path)
        finally:
            monkeypatch.undo()


def test_a_refusal_is_reported_even_when_its_record_cannot_be_saved(tmp_path, monkeypatch, capsys):
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if "-30001" in text else None)

    def setup(f):
        orig = f.reply
        f.reply = lambda cmd: "-30001,{},GetAngle();" if cmd == "GetAngle()" else orig(cmd)

    code, made = _main_with(monkeypatch, tmp_path, setup)
    out = capsys.readouterr().out
    line = _lines(out, "error: GetAngle() -> ErrorID -30001")
    assert code == 1 and line and "record also failed" in line[0]
    assert made[0].received[-1] == "GetAngle()"


def test_a_send_failure_is_reported_even_when_its_record_cannot_be_saved(tmp_path, monkeypatch):
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if '"send: ' in text else None)
    count = [0]

    class BrokenSend(_Wrapped):
        def sendall(self, data):
            count[0] += 1
            if count[0] == 3:
                raise ConnectionResetError("reset by peer")
            return self._sock.sendall(data)

    with FakeMG400() as fake:
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path, wrap=BrokenSend)
        sent = list(fake.received)
    assert len(sent) == 2 and info.value.mg400_recorder.failed


def test_ctrl_c_is_still_130_when_the_record_also_fails(tmp_path, monkeypatch, capsys):
    orig_check = observe.FeedbackRecorder.check
    calls = [0]

    def check(self, now=None):
        calls[0] += 1
        if calls[0] == 6:
            raise KeyboardInterrupt
        return orig_check(self, now)

    monkeypatch.setattr(observe.FeedbackRecorder, "check", check)
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if "KeyboardInterrupt" in text else None)
    code, made = _main_with(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    assert code == 130 and "not an emergency stop" in out and "record also failed" in out


def test_a_complete_run_whose_final_save_fails_is_not_a_success(tmp_path, monkeypatch, capsys):
    _patch_replace(monkeypatch, lambda n, text: OSError("busy") if '"ended_utc"' in text else None)
    code, made = _main_with(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    line = _lines(out, "record error:")
    assert code == 1 and line and "report.json.tmp holds the latest" in line[0]
    tmp = json.loads((tmp_path / "report.json.tmp").read_text(encoding="utf-8"))
    assert tmp["complete"] is True and tmp["save_errors"]


def test_a_file_error_is_never_called_a_connection_error(tmp_path, monkeypatch, capsys):
    real_open = open

    def fake_open(path, *a, **k):
        if str(path).endswith("feedback.bin"):
            raise OSError("disk full")
        return real_open(path, *a, **k)

    monkeypatch.setattr(observe, "open", fake_open, raising=False)
    code, made = _main_with(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    assert code == 1 and _lines(out, "error: disk full") and "connection error" not in out


def test_a_refused_connection_is_a_connection_error(tmp_path):
    s = socket.socket()
    s.bind((LOOPBACK, 0))
    port = s.getsockname()[1]
    s.close()                                            # nothing listens there now
    with pytest.raises(protocol.MG400ConnectionError):
        observe.observe(LOOPBACK, str(tmp_path), 1.0, None,
                        connect=lambda h, p, t: observe._connect(LOOPBACK, port, t),
                        out=lambda *a: None)


class _ClosingFake(FakeMG400):
    def _serve_dashboard(self, conn):
        conn.settimeout(2.0)
        try:
            conn.recv(4096)          # the first command is read, so the close is a clean end
        finally:
            conn.close()


class _ResettingFake(FakeMG400):
    def _serve_dashboard(self, conn):
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
        conn.close()


@pytest.mark.parametrize("cls, text", [(_ClosingFake, "connection error: dashboard-1: connection "
                                                      "closed by the arm"),
                                       (_ResettingFake, "connection error: dashboard-1")])
def test_a_connection_closed_or_reset_by_the_arm_is_a_connection_error(tmp_path, monkeypatch,
                                                                       capsys, cls, text):
    code, made = _main_with(monkeypatch, tmp_path, cls=cls)
    out = capsys.readouterr().out
    assert code == 1 and _lines(out, text), out


def test_the_onedrive_note_depends_on_the_output_folder(tmp_path, capsys):
    assert observe.main(["--mock", "--seconds", "1", "--out", str(tmp_path / "OneDrive" / "r")]) == 0
    assert observe.ONEDRIVE_NOTE in capsys.readouterr().out
    if "onedrive" not in str(tmp_path).lower():
        assert observe.main(["--mock", "--seconds", "1", "--out", str(tmp_path / "plain")]) == 0
        assert observe.ONEDRIVE_NOTE not in capsys.readouterr().out


# ----------------------------------------------------------------------
# After the second bench run: a second Dashboard connection may end the first
# ----------------------------------------------------------------------
@pytest.fixture(autouse=True)
def _short_waits(monkeypatch):
    monkeypatch.setattr(observe, "IDLE_PROBE_S", 0.05)
    monkeypatch.setattr(observe, "SECOND_OPEN_SETTLE_S", 0.02)


def _on(fake, number):
    return [cmd for n, cmd in fake.received_on if n == number]


def _ordered(fake, extra=None):
    """wrap for _run: the second Dashboard connect returns only after the single-client fake
    has closed the first one (the order of drop and probe is then fixed)"""
    count = [0]

    def wrap(sock):
        count[0] += 1
        if count[0] == 2 and fake.single_dashboard:
            assert fake.dropped.wait(5.0)
        return extra(sock, count[0]) if extra is not None else sock

    return wrap


class _ProbeFault(_Wrapped):
    """dashboard-1 only: once ``armed`` (set when dashboard-2 opens), the next sendall or recv
    raises ``error`` (or recv returns ``chunks`` one by one)"""

    def __init__(self, sock, armed, op, error=None, chunks=None):
        super().__init__(sock)
        self._armed, self._op, self._error, self._chunks = armed, op, error, chunks

    def sendall(self, data):
        if self._armed[0] and self._op == "send":
            self._armed[0] = False
            raise self._error
        return self._sock.sendall(data)

    def recv(self, n):
        if self._armed[0] and self._op == "recv":
            if self._chunks is not None:
                if self._chunks:
                    return self._chunks.pop(0)
                self._armed[0] = False
                return b""
            self._armed[0] = False
            raise self._error
        return self._sock.recv(n)


def _probe_fault(op, error=None, chunks=None):
    armed = [False]

    def extra(sock, number):
        if number == 1:
            return _ProbeFault(sock, armed, op, error, chunks)
        armed[0] = True                                     # dashboard-2 is open: probe next
        return sock

    return extra


def test_a_dropped_first_connection_is_recorded_and_the_run_completes_on_the_second(tmp_path):
    entries = [0]
    labels = iter(["dashboard-1", "dashboard-2"])

    class CheckedSend(_Wrapped):
        def __init__(self, sock):
            super().__init__(sock)
            self.label = next(labels)

        def sendall(self, data):
            entries[0] += 1
            report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
            last = report["commands"][-1]
            assert len(report["commands"]) == entries[0]
            assert last["connection"] == self.label and last["command"] == data.decode("ascii")
            assert "sent_utc" not in last
            assert self.label in [c["label"] for c in report["connections"]]
            return self._sock.sendall(data)

    with FakeMG400(single_dashboard=True) as fake:
        report = _run(fake, tmp_path, wrap=_ordered(fake, lambda s, n: CheckedSend(s)))
    assert report["complete"] is True
    two = report["two_connections"]
    assert two["outcome"].startswith("first connection's end detected") and two["result"] == two["outcome"]
    assert two["interleaved"] == two["outstanding"] == "not run: first connection ended"
    conns = {c["label"]: c for c in report["connections"]}
    assert conns["dashboard-1"]["end_seen"] in ("eof detected", "reset detected")
    assert conns["dashboard-1"]["opened_monotonic"] < conns["dashboard-2"]["opened_monotonic"] \
        < conns["dashboard-1"]["end_detected_monotonic"]
    probe = next(c for c in report["commands"] if c.get("tag") == "probe")
    assert two["probe"]["command_id"] == probe["id"] and probe["connection"] == "dashboard-1"
    after = report["commands"][report["commands"].index(probe) + 1:]
    assert after and all(c["connection"] == "dashboard-2" for c in after)
    assert "dashboard-2 GetPose()" in report["queries"]
    assert entries[0] == len(report["commands"])            # every command was tried after its save


def test_both_answered_is_set_only_when_the_stage_is_done(tmp_path):
    with FakeMG400() as fake:
        orig, seen = fake.reply, {"n": 0}

        def reply(cmd):
            if cmd == "GetAngle()" and fake.conn_number.n == 2:
                seen["n"] += 1
                if seen["n"] == 2:                          # the outstanding step's GetAngle
                    return "-30001,{},GetAngle();"
            return orig(cmd)

        fake.reply = reply
        with pytest.raises(protocol.MG400CommandError):
            _run(fake, tmp_path)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    two = report["two_connections"]
    assert two["outcome"] == "first still answered after the second opened"
    assert len(two["interleaved"]) == 8 and two["result"] is None


def test_the_default_fake_still_answers_on_both(tmp_path):
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    two = report["two_connections"]
    assert report["complete"] is True and two["result"] == "both answered"
    assert two["control"]["command_id"] and two["probe"]["elapsed_s"] >= observe.SECOND_OPEN_SETTLE_S


def test_a_drop_during_the_idle_wait_ends_it_before_a_second_connection(tmp_path, monkeypatch):
    real_wait = observe._wait
    with FakeMG400() as fake:
        def wait(fb, seconds):
            if seconds == observe.IDLE_PROBE_S:
                c = fake._dashboard_conns[0]
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            return real_wait(fb, seconds)

        monkeypatch.setattr(observe, "_wait", wait)
        with pytest.raises(protocol.MG400ConnectionError):
            _run(fake, tmp_path)
        conns = len(fake._dashboard_conns)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert conns == 1 and "dashboard-2" not in [c["label"] for c in report["connections"]]
    assert "before a second connection was opened" in " ".join(report["notes"])
    assert report["two_connections"]["control"]["command_id"]


def test_a_probe_timeout_is_not_taken_as_a_drop(tmp_path, monkeypatch):
    monkeypatch.setattr(observe, "TIMEOUT_S", 0.3)
    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd == "RobotMode()" and fake.conn_number.n == 1 and len(fake._dashboard_conns) == 2:
                time.sleep(0.6)
            return orig(cmd)

        fake.reply = reply
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path)
        on_2 = _on(fake, 2)
    assert info.value.kind == "timeout" and on_2 == []


def test_a_probe_answered_with_another_mode_ends_it(tmp_path):
    with FakeMG400() as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd == "RobotMode()" and fake.conn_number.n == 1 and len(fake._dashboard_conns) == 2:
                return "0,{5},RobotMode();"
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
        on_2 = _on(fake, 2)
    assert report["complete"] is False and "RobotMode 5" in " ".join(report["notes"]) and on_2 == []


def test_an_end_after_part_of_a_reply_is_not_a_drop(tmp_path):
    with FakeMG400() as fake:
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path, wrap=_ordered(fake, _probe_fault("recv", chunks=[b"0,{4"])))
        on_2 = _on(fake, 2)
    assert info.value.kind == "eof" and info.value.partial_reply and on_2 == []


@pytest.mark.parametrize("op, error, kind", [
    ("send", ConnectionAbortedError(10053, "aborted"), "aborted"),
    ("recv", ConnectionAbortedError(10053, "aborted"), "aborted"),
    ("send", BrokenPipeError(32, "broken pipe"), "broken pipe"),
    ("send", socket.timeout("timed out"), "timeout"),
    ("send", OSError(9, "bad file descriptor"), "other"),
])
def test_other_probe_errors_end_it(tmp_path, op, error, kind):
    with FakeMG400() as fake:
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path, wrap=_ordered(fake, _probe_fault(op, error)))
        on_2 = _on(fake, 2)
    assert info.value.kind == kind and on_2 == []


@pytest.mark.parametrize("op", ["send", "recv"])
def test_a_reset_of_the_probe_is_a_drop(tmp_path, op):
    with FakeMG400() as fake:
        report = _run(fake, tmp_path,
                      wrap=_ordered(fake, _probe_fault(op, ConnectionResetError(10054, "reset"))))
        on_2 = _on(fake, 2)
    assert report["complete"] is True and on_2
    assert report["two_connections"]["result"].startswith("first connection's end detected")
    conns = {c["label"]: c for c in report["connections"]}
    assert conns["dashboard-1"]["end_seen"] == "reset detected"


def test_a_drop_shaped_error_of_another_command_ends_it(tmp_path):
    # every tolerated condition holds except the command: the error names command 1
    err = protocol.MG400ConnectionError("dashboard-1: connection closed by the arm",
                                        label="dashboard-1", op="recv", kind="eof")
    err.command_id = 1
    with FakeMG400() as fake:
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path, wrap=_ordered(fake, _probe_fault("recv", err)))
        on_2 = _on(fake, 2)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    probe = next(c for c in report["commands"] if c.get("tag") == "probe")
    assert info.value is err and probe["id"] != 1 and on_2 == []


def _single_client_main(monkeypatch, tmp_path):
    """main() --mock with a single-client fake; the second Dashboard connect returns only
    after the fake has ended the first (fixed order of drop and probe)"""
    made = []
    real_open = observe._open
    opens = [0]

    def ordered_open(connect, host, port):
        sock = real_open(connect, host, port)
        if port == observe.DASHBOARD_PORT:
            opens[0] += 1
            if opens[0] == 2:
                assert made[0].dropped.wait(5.0)
        return sock

    monkeypatch.setattr(observe, "_open", ordered_open)
    code, fakes = _main_with(monkeypatch, tmp_path,
                             setup=lambda f: (setattr(f, "single_dashboard", True), made.append(f)))
    return code, fakes[0]


@pytest.mark.parametrize("marker, code_line", [
    # the save of the probe's error entry fails: the probe's connection error is the error
    ('"end_seen"', "connection error: dashboard-1: connection closed by the arm"),
    # the save of the drop record fails: the record error is the error
    ('"outcome": "first connection', "record error:"),
])
def test_a_drop_whose_record_cannot_be_saved_ends_it(tmp_path, monkeypatch, capsys, marker,
                                                    code_line):
    # that one save fails (not retried); the saves after it succeed
    once = {}

    def fail(n, text):
        if marker in text and '"tag": "probe"' in text and not once:
            once["failed"] = True
            return OSError("busy")
        return None

    _patch_replace(monkeypatch, fail)
    code, fake = _single_client_main(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    assert once and code == 1 and _on(fake, 2) == []
    line = _lines(out, code_line)
    assert line, out
    if code_line.startswith("connection error"):
        assert "record also failed" in line[0]
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["save_errors"] and report["complete"] is False


def test_a_single_client_run_through_the_cli_completes(tmp_path, monkeypatch, capsys):
    code, fake = _single_client_main(monkeypatch, tmp_path)
    out = capsys.readouterr().out
    assert code == 0 and "continuing on dashboard-2 only" in out and _on(fake, 2)


def test_a_control_probe_timeout_is_not_called_an_end(tmp_path, monkeypatch):
    monkeypatch.setattr(observe, "TIMEOUT_S", 0.3)
    with FakeMG400() as fake:
        orig, seen = fake.reply, {"n": 0}

        def reply(cmd):
            if cmd == "RobotMode()" and fake.conn_number.n == 1:
                seen["n"] += 1
                if seen["n"] == 2:                          # the control (1: the first check)
                    time.sleep(0.6)
            return orig(cmd)

        fake.reply = reply
        with pytest.raises(protocol.MG400ConnectionError) as info:
            _run(fake, tmp_path)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    notes = " ".join(report["notes"])
    assert info.value.kind == "timeout" and "failed (timeout)" in notes and "end detected" not in notes
    conns = {c["label"]: c for c in report["connections"]}
    assert "end_seen" not in conns["dashboard-1"] or conns["dashboard-1"]["end_seen"] == "closed by this tool"


def test_the_connection_records_say_what_was_seen(tmp_path):
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    conns = {c["label"]: c for c in report["connections"]}
    assert set(conns) == {"feedback", "dashboard-1", "dashboard-2"}
    for c in conns.values():
        assert c["opened_monotonic"] < c["end_detected_monotonic"]
        assert c["end_seen"] == "closed by this tool"
    probe = report["two_connections"]["probe"]
    assert probe["last_reply_monotonic_before"] is not None
    assert probe["elapsed_s"] >= observe.SECOND_OPEN_SETTLE_S


def test_the_feedback_end_is_recorded_when_the_receiver_saw_it(tmp_path):
    """not when the tool tore down: a Dashboard reply that comes later must not move it"""
    with FakeMG400() as fake:
        orig = fake.reply
        dropped_at = []

        def reply(cmd):
            if cmd.startswith("PositiveSolution") and not dropped_at:
                dropped_at.append(time.monotonic())
                fake.drop_feedback()
                time.sleep(0.6)
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path)
    conns = {c["label"]: c for c in report["connections"]}
    fb = conns["feedback"]
    assert fb["end_seen"] == "eof detected" and fb["fault"] == "feedback connection closed"
    assert fb["end_detected_monotonic"] - dropped_at[0] < 0.4
    assert conns["dashboard-1"]["end_seen"] == "closed by this tool"
    assert conns["dashboard-1"]["end_detected_monotonic"] - fb["end_detected_monotonic"] > 0.2


def test_the_feedback_close_is_recorded_at_the_first_stop(tmp_path, monkeypatch):
    """a slow wind-down after a complete run must not move the time the tool closed it"""
    orig = observe.FeedbackRecorder.join

    def slow_join(self, timeout=None):
        time.sleep(0.6)
        return orig(self, timeout)

    monkeypatch.setattr(observe.FeedbackRecorder, "join", slow_join)
    with FakeMG400() as fake:
        report = _run(fake, tmp_path)
    assert report["complete"] is True
    conns = {c["label"]: c for c in report["connections"]}
    fb = conns["feedback"]
    assert fb["end_seen"] == "closed by this tool" and "fault" not in fb
    assert conns["dashboard-1"]["end_detected_monotonic"] - fb["end_detected_monotonic"] > 0.4


def test_a_failed_connect_says_which_operation_and_what_was_seen(tmp_path):
    s = socket.socket()
    s.bind((LOOPBACK, 0))
    port = s.getsockname()[1]
    s.close()
    with pytest.raises(protocol.MG400ConnectionError) as info:
        observe.observe(LOOPBACK, str(tmp_path), 1.0, None,
                        connect=lambda h, p, t: observe._connect(LOOPBACK, port, t),
                        out=lambda *a: None)
    assert info.value.op == "connect" and info.value.kind in ("refused", "timeout")
    assert info.value.label == f"port {observe.FEEDBACK_PORT}"


def test_a_reset_at_connect_is_not_a_drop():
    err = protocol.MG400ConnectionError("x", label="dashboard-1", op="connect", kind="reset")
    err.command_id = 7
    rec = observe.Recorder.__new__(observe.Recorder)
    rec.failed = False
    assert not observe._tolerated_drop(err, 7, rec)
    err.op = "recv"
    assert observe._tolerated_drop(err, 7, rec)


def _feed_fault_then_recover(fake, recorders):
    """Three packets with mode 5, then good ones. Every packet from here on carries its own
    number as TimeStamp, so the wait is on what the receiver took: the three bad packets and
    then three good ones after the last bad one"""
    orig_packet = fake.packet
    state = {"n": 10 ** 15, "bad": []}       # above any real millisecond TimeStamp

    def packet():
        pkt = bytearray(orig_packet())
        state["n"] += 1
        struct.pack_into("<Q", pkt, 32, state["n"])
        if len(state["bad"]) < 3:
            state["bad"].append(state["n"])
            struct.pack_into("<Q", pkt, 24, 5)
        return bytes(pkt)

    fb = recorders[0]
    fake.packet = packet

    def recovered():
        with fb.lock:
            seen = list(fb.timestamps)
        if len(state["bad"]) < 3 or not set(state["bad"]) <= set(seen):
            return False
        return len([t for t in seen if t > state["bad"][-1]]) >= 3

    _until(recovered)
    with fb.lock:
        assert fb.latest["robot_mode"] == 4                  # the feed is good again


@pytest.mark.parametrize("where", ["first send on dashboard-2", "drop record"])
def test_a_feed_fault_during_a_save_after_the_drop_stops_the_next_send(tmp_path, monkeypatch, where):
    recorders = _capture_feedback(monkeypatch)
    state = {}
    with FakeMG400(single_dashboard=True) as fake:
        def fail(n, text):
            report = json.loads(text)
            if state or not report["commands"]:
                return None
            last = report["commands"][-1]
            hit = (where == "drop record" and '"outcome": "first connection' in text) or \
                  (where == "first send on dashboard-2" and last["connection"] == "dashboard-2"
                   and "sent_utc" not in last)
            if hit:
                state["hit"] = True
                _feed_fault_then_recover(fake, recorders)    # after _check_state's opening check
                return PermissionError("locked")             # one refused rename: the retry succeeds
            return None

        _patch_replace(monkeypatch, fail)
        report = _run(fake, tmp_path, wrap=_ordered(fake))
        on_2 = _on(fake, 2)
    assert state and on_2 == [] and report["complete"] is False
    assert "mode 5" in " ".join(report["notes"])


def test_the_second_connection_is_checked_before_its_queries(tmp_path):
    with FakeMG400(single_dashboard=True) as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd == "RobotMode()" and fake.conn_number.n == 2:
                return "0,{5},RobotMode();"
            return orig(cmd)

        fake.reply = reply
        report = _run(fake, tmp_path, wrap=_ordered(fake))
        on_2 = _on(fake, 2)
    assert on_2 == ["RobotMode()"] and report["complete"] is False


def test_the_second_connection_ending_later_ends_it(tmp_path):
    with FakeMG400(single_dashboard=True) as fake:
        orig = fake.reply

        def reply(cmd):
            if cmd.startswith("PositiveSolution") and fake.conn_number.n == 2:
                c = fake._dashboard_conns[1]
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            return orig(cmd)

        fake.reply = reply
        with pytest.raises(protocol.MG400ConnectionError):
            _run(fake, tmp_path, wrap=_ordered(fake))
        sent = list(fake.received)
    assert not any(c.startswith("InverseSolution") for c in sent)


@pytest.mark.parametrize("which", ["idle", "settle"])
def test_a_feed_fault_during_the_waits_stops_the_next_send(tmp_path, monkeypatch, which):
    real_wait = observe._wait
    with FakeMG400() as fake:
        alarm = {"on": False}
        orig_packet = fake.packet

        def packet():
            pkt = bytearray(orig_packet())
            if alarm["on"]:
                pkt[1029] = 1
            return bytes(pkt)

        fake.packet = packet
        target = {"idle": observe.IDLE_PROBE_S, "settle": observe.SECOND_OPEN_SETTLE_S}[which]

        def wait(fb, seconds):
            if seconds == target:
                alarm["on"] = True
                _until(lambda: fb.fault is not None)
            return real_wait(fb, seconds)

        monkeypatch.setattr(observe, "_wait", wait)
        report = _run(fake, tmp_path)
        on_1 = _on(fake, 1)
    assert report["complete"] is False
    assert on_1.count("RobotMode()") == (1 if which == "idle" else 2)


def test_the_dry_run_names_the_control_and_the_probe(capsys):
    assert observe.main(["--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "(control)" in out and "(probe)" in out and "end of stream or reset" in out
