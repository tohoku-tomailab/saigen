"""Dobot MG400 jog tool: the control-command gates, the sequence, the stop sequence.

No hardware and no network beyond 127.0.0.1: the fake controller listens on loopback only.
The tests check what is written on the wire, on which port, and in which order.
"""
import ast
import io
import json
import os
import signal
import socket
import threading
import time

import pytest

from src.devices.mg400 import feedback, jog, observe, protocol
from src.devices.mg400.fake import LOOPBACK, FakeMG400
from src.devices.mg400.protocol import MG400ProtocolError

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MG400_DIR = os.path.join(REPO_ROOT, "src", "devices", "mg400")


def connect_to(fake):
    ports = {jog.DASHBOARD_PORT: fake.dashboard_port, jog.FEEDBACK_PORT: fake.feedback_port,
             jog.MOTION_PORT: fake.motion_port}

    def connect(host, port, timeout):
        return jog._connect_fixed(LOOPBACK, ports[port], timeout)
    return connect


def run(fake, folder, **kw):
    kw.setdefault("confirm", lambda text: True)
    kw.setdefault("out", lambda *a, **k: None)
    return jog.jog(LOOPBACK, str(folder), connect=connect_to(fake), **kw)


def run_failing(fake, folder, exc=jog.MG400Error, **kw):
    with pytest.raises(exc) as info:
        run(fake, folder, **kw)
    return info.value.mg400_report


def names(cmds):
    return [c.split("(", 1)[0] for c in cmds]


def dashboard_names(fake):
    return names([c for n, c in fake.received_on])


def stop_tail(fake):
    """Dashboard command names from the stop sequence's ResetRobot on (the second ResetRobot
    of a run: the first one is step 5 of the normal sequence)"""
    dash = dashboard_names(fake)
    first = dash.index("ResetRobot")
    second = dash.index("ResetRobot", first + 1)
    return dash[second:]


# ----------------------------------------------------------------------
# Gates
# ----------------------------------------------------------------------
@pytest.mark.parametrize("text", ["EnableRobot()", "DisableRobot()", "ResetRobot()", "EmergencyStop()",
                                  "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=10)"])
def test_control_commands_need_the_control_flag(text):
    with pytest.raises(ValueError):
        protocol.check_command(text)
    with pytest.raises(ValueError):
        protocol.check_command(text, control=1)             # only the bool True
    assert protocol.check_command(text, control=True) == text.split("(")[0]


@pytest.mark.parametrize("text", [
    "EnableRobot(0.5)", "EnableRobot(0.5,0,0,5.5)", "DisableRobot(1)", "ClearError()", "SpeedFactor(10)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=0,AccJ=0)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=31,AccJ=31)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=100,AccJ=100)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=30)",      # must be equal
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10.0,AccJ=10)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=10,CP=50)",
    "JointMovJ(1.000000,2.000000,3.000000,4.000000,5.000000,SpeedJ=10,AccJ=10)",
    "JointMovJ(1,2,3,4,SpeedJ=10,AccJ=10)", "JointMovJ(nan,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=10)",
    "MovJ(1.000000,2.000000,3.000000,4.000000)", "MovL(1,2,3,4)", "RelJointMovJ(1,0,0,0)", "Sync()",
    "EnableRobot()DisableRobot()", "EnableRobot();", "JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=10)\n",
])
def test_other_forms_are_refused_even_with_the_flag(text):
    with pytest.raises(ValueError):
        protocol.check_command(text, control=True)


def test_the_builders():
    assert protocol.enable_robot().text == "EnableRobot()"
    assert protocol.disable_robot().text == "DisableRobot()"
    assert protocol.reset_robot().text == "ResetRobot()"
    assert protocol.emergency_stop().text == "EmergencyStop()"
    assert protocol.joint_mov_j([5.0718, 10.2786, 16.847, 18.5559], 10).text == \
        "JointMovJ(5.071800,10.278600,16.847000,18.555900,SpeedJ=10,AccJ=10)"
    assert protocol.joint_mov_j([0, 0, 0, 0], 30).name == "JointMovJ"
    assert protocol.enable_robot().control and not protocol.robot_mode().control
    for bad in (0, 31, 10.0, True, "10", None):
        with pytest.raises(ValueError):
            protocol.joint_mov_j([0, 0, 0, 0], bad)
    with pytest.raises(ValueError):
        protocol.joint_mov_j([0, 0, 0], 10)


class Sock:
    def __init__(self):
        self.sent = []

    def sendall(self, data):
        self.sent.append(data)

    def gettimeout(self):
        return 1.0

    def settimeout(self, t):
        pass


def bare_recorder():
    rec = observe.Recorder.__new__(observe.Recorder)
    rec.report = {"commands": []}
    return rec


def test_a_read_only_connection_refuses_a_control_command_object():
    sock = Sock()
    dash = observe.Dashboard(sock, bare_recorder(), "x")
    for cmd in (protocol.enable_robot(), protocol.disable_robot(), protocol.reset_robot(),
                protocol.emergency_stop(), protocol.joint_mov_j([0, 0, 0, 0], 10)):
        with pytest.raises(ValueError):
            dash.write(cmd)
    assert sock.sent == []


def test_names_restrict_each_connection():
    sock = Sock()
    rec = bare_recorder()
    dash = observe.Dashboard(sock, rec, "dashboard-1", control=True)
    dash.names = jog.DASHBOARD_NAMES
    motion = observe.Dashboard(sock, rec, "motion", control=True)
    motion.names = jog.MOTION_NAMES
    with pytest.raises(ValueError):
        dash.write(protocol.joint_mov_j([0, 0, 0, 0], 10))          # no motion on the Dashboard port
    with pytest.raises(ValueError):
        dash.write(protocol.emergency_stop())                       # only the stop sequence sends it
    with pytest.raises(ValueError):
        motion.write(protocol.robot_mode())
    with pytest.raises(ValueError):
        motion.write(protocol.enable_robot())
    assert sock.sent == []
    assert "EmergencyStop" not in jog.DASHBOARD_NAMES and "EmergencyStop" in jog.STOP_NAMES
    assert not set(protocol.MOTION) & set(jog.STOP_NAMES)


def test_observe_source_never_arms_a_connection():
    """AST: observe.py passes no control= keyword, assigns no .names, builds no control command"""
    with open(os.path.join(MG400_DIR, "observe.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    builders = {"enable_robot", "disable_robot", "reset_robot", "emergency_stop", "joint_mov_j"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            for kw in node.keywords:
                assert not (kw.arg == "control" and isinstance(kw.value, ast.Constant)
                            and kw.value.value is True), ast.dump(node)
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", None)
            assert name not in builders, ast.dump(node)
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Attribute) and t.attr == "names":
                    assert isinstance(t.value, ast.Name) and t.value.id == "self" and \
                        isinstance(node.value, ast.Constant) and node.value.value is None, ast.dump(node)
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            assert "30003" not in node.value and node.value not in protocol.CONTROL, node.value


def test_observe_mock_run_opens_only_read_only_connections(tmp_path, monkeypatch):
    fake = FakeMG400()
    ports = []
    made = []
    real = socket.create_connection

    def guarded(addr, *a, **k):
        ports.append(addr[1])
        return real(addr, *a, **k)

    class Recording(observe.Dashboard):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            made.append(self)
    monkeypatch.setattr(socket, "create_connection", guarded)
    monkeypatch.setattr(observe, "Dashboard", Recording)
    try:
        def connect(host, port, timeout):
            return observe._connect(LOOPBACK, {jog.DASHBOARD_PORT: fake.dashboard_port,
                                               jog.FEEDBACK_PORT: fake.feedback_port}[port], timeout)
        report = observe.observe(LOOPBACK, str(tmp_path), seconds=1.0, connect=connect, out=lambda *a: None)
    finally:
        fake.close()
    assert report["complete"] is True
    assert made and all(d.control is False and d.names is None for d in made)
    assert fake.motion_port not in ports and ports
    assert fake.motion_received == []
    assert set(names(fake.received)) <= set(protocol.ALLOWED)


def test_the_motion_port_is_named_only_by_the_jog_tool_and_the_fake():
    for name in os.listdir(MG400_DIR):
        if name.endswith(".py") and name not in ("jog.py", "fake.py"):
            with open(os.path.join(MG400_DIR, name), encoding="utf-8") as f:
                tree = ast.parse(f.read())
            for node in ast.walk(tree):
                if isinstance(node, ast.Constant):
                    assert node.value != 30003 and not (isinstance(node.value, str) and "30003" in node.value), name


# ----------------------------------------------------------------------
# Limits, the address, the prompt
# ----------------------------------------------------------------------
@pytest.mark.parametrize("j1,j4", [(10.5, 0), (0, -10.001), (0, 0), (float("nan"), 0), (True, 0), ("5", 0),
                                   (0.5, 0), (5, 0.1), (-0.99, 0)])
def test_offsets_outside_the_limits_are_refused(j1, j4):
    with pytest.raises(ValueError):
        jog.offsets_ok(j1, j4)


@pytest.mark.parametrize("j1,j4", [(1.0, 0), (0, -1.0), (10, 10), (-10, 0), (5, -2)])
def test_offsets_inside_the_limits(j1, j4):
    jog.offsets_ok(j1, j4)


def test_target_joints_and_their_limits():
    assert jog.target_joints([1.0, 2.0, 3.0, 4.0], 5.0, -2.5) == [6.0, 2.0, 3.0, 1.5]
    with pytest.raises(ValueError):
        jog.target_joints([148.0, 0, 0, 0], 5.0, 0)                 # target J1 153 > 150
    with pytest.raises(ValueError):
        jog.target_joints([151.0, 0, 0, 0], -5.0, 0)                # start outside
    with pytest.raises(ValueError):
        jog.target_joints([0, 0, 0, 168.0], 0, 5.0)
    with pytest.raises(ValueError):
        jog.target_joints([0, 0, 0], 5.0, 0)


def test_the_address_must_be_a_numeric_literal():
    assert jog.fixed_address(" 192.0.2.1 ") == "192.0.2.1"
    assert jog.fixed_address("::1") == "::1"
    for bad in ("arm.local", "192.0.2", "localhost", "", "192.0.2.1:29999", "http://192.0.2.1"):
        with pytest.raises(ValueError):
            jog.fixed_address(bad)


def test_connect_fixed_does_no_name_lookup(monkeypatch):
    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **k: pytest.fail("name lookup"))
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("create_connection"))
    srv = socket.socket()
    srv.bind((LOOPBACK, 0))
    srv.listen(1)
    try:
        s = jog._connect_fixed(LOOPBACK, srv.getsockname()[1], 1.0)
        assert s.gettimeout() == 1.0
        s.close()
        t0 = time.monotonic()
        with pytest.raises(OSError):
            jog._connect_fixed("192.0.2.1", 29999, 0.3)         # TEST-NET-1: unroutable, times out or is refused
        assert time.monotonic() - t0 < 2.0
    finally:
        srv.close()


def test_a_start_near_the_limit_ends_before_enabling(tmp_path):
    fake = FakeMG400(angles=(147.0, 10.0, 16.0, 18.0, 0.0, 0.0))
    try:
        report = run_failing(fake, tmp_path, exc=ValueError)
    finally:
        fake.close()
    assert report["armed"] is False
    assert names(fake.received) == ["RobotMode", "GetErrorID", "GetAngle", "GetPose"]
    assert fake.motion_received == []


class TtyStdin(io.StringIO):
    def isatty(self):
        return True


@pytest.mark.parametrize("stdin,expected", [
    (io.StringIO("\n"), False),                     # not a terminal
    (TtyStdin("\n"), True), (TtyStdin("\r\n"), True),
    (TtyStdin(""), False), (TtyStdin("y\n"), False), (TtyStdin(" \n"), False),
    (TtyStdin("\n\n"), True), (None, False),
])
def test_confirm_needs_a_terminal_and_an_empty_line(stdin, expected):
    lines = []
    assert jog.confirm_tty("plan", stdin=stdin, out=lines.append) is expected   # None: pytest's stdin, no tty
    assert lines[0] == "plan"


def test_without_a_terminal_nothing_is_enabled(tmp_path):
    fake = FakeMG400()
    try:
        report = run(fake, tmp_path, confirm=jog.confirm_tty)          # pytest's stdin is not a tty
    finally:
        fake.close()
    assert report["armed"] is False and report["complete"] is False
    assert names(fake.received) == ["RobotMode", "GetErrorID", "GetAngle", "GetPose", "PositiveSolution"]
    assert fake.motion_received == []
    assert "not confirmed" in " ".join(report["notes"])


def test_a_refusing_human_sends_nothing_more(tmp_path):
    fake = FakeMG400()
    try:
        report = run(fake, tmp_path, confirm=lambda text: False)
    finally:
        fake.close()
    assert report["armed"] is False
    assert not any(n in protocol.CONTROL for n in names(fake.received))
    assert fake.motion_received == []


# ----------------------------------------------------------------------
# The sequence
# ----------------------------------------------------------------------
def test_mock_run_complete_and_the_exact_commands(tmp_path):
    fake = FakeMG400(move_s=0.3)
    try:
        report = run(fake, tmp_path, j1=5.0, j4=-2.0, note="playback 30 deg/s, global 50 %")
    finally:
        fake.close()
    assert report["complete"] is True and report["armed"] is True and report.get("stop") is None
    assert report["note_typed"] == "playback 30 deg/s, global 50 %"
    start = report["jog"]["start_joints"]
    target = report["jog"]["target_joints"]
    assert target == [start[0] + 5.0, start[1], start[2], start[3] - 2.0]
    dash = dashboard_names(fake)
    assert dash[:10] == ["RobotMode", "GetErrorID", "GetAngle", "GetPose", "PositiveSolution",
                         "ResetRobot", "RobotMode", "GetErrorID", "GetAngle", "EnableRobot"]
    assert dash[-6:] == ["DisableRobot", "RobotMode", "RobotMode", "GetErrorID", "GetAngle", "GetPose"]
    assert dash.count("EnableRobot") == 1 and dash.count("DisableRobot") == 1 and dash.count("ResetRobot") == 1
    assert not {"EmergencyStop", "ClearError"} & set(dash)
    assert set(dash) <= set(jog.DASHBOARD_NAMES)
    assert fake.motion_received == [protocol.joint_mov_j(target, 10).text, protocol.joint_mov_j(start, 10).text]
    assert fake.moves == [target, start]
    assert [round(v, 4) for v in fake.angles[:4]] == [round(v, 4) for v in start]
    assert [leg["leg"] for leg in report["jog"]["legs"]] == ["out", "back"]
    assert all(leg["arrived_s"] >= jog.MIN_LEG_S for leg in report["jog"]["legs"])
    assert report["jog"]["final_minus_start_joints"] == [0.0, 0.0, 0.0, 0.0]
    assert report["jog"]["enable_s"] >= 0.15 and report["feedback"]["fault"] is None
    assert [p["phase"] for p in report["jog"]["phases"]] == \
        ["disabled", "enabling", "enabled", "moving", "enabled", "moving", "enabled", "disabling", "disabled"]
    assert report["io_thread_drained"] is True
    assert [c["label"] for c in report["connections"]] == ["feedback", "dashboard-1", "motion"]
    assert (tmp_path / "feedback.csv").exists()
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["complete"] is True and len(on_disk["commands"]) == len(report["commands"])
    assert all(c.get("reply") for c in on_disk["commands"])


def test_speed_ratio_reaches_the_wire_and_is_capped(tmp_path):
    fake = FakeMG400(move_s=0.2)
    try:
        run(fake, tmp_path, speed=30)
    finally:
        fake.close()
    assert all(c.endswith("SpeedJ=30,AccJ=30)") for c in fake.motion_received)
    with pytest.raises(ValueError):
        jog.jog(LOOPBACK, str(tmp_path / "x"), speed=31, connect=lambda *a: pytest.fail("connected"))
    with pytest.raises(ValueError):
        jog.jog(LOOPBACK, str(tmp_path / "y"), j1=11, connect=lambda *a: pytest.fail("connected"))
    with pytest.raises(ValueError):
        jog.jog(LOOPBACK, str(tmp_path / "z"), j1=0.5, connect=lambda *a: pytest.fail("connected"))


def test_the_arm_is_not_enabled_when_it_is_not_disabled_or_alarmed(tmp_path):
    for i, fake in enumerate((FakeMG400(mode=5), FakeMG400(errors=[[23], [], [], [], [], []]))):
        try:
            report = run_failing(fake, tmp_path / str(i), exc=observe.ObservationEnded)
        finally:
            fake.close()
        assert report["armed"] is False
        assert not any(n in protocol.CONTROL for n in names(fake.received))


def test_the_initial_reset_must_leave_the_arm_disabled_and_in_place(tmp_path):
    """A Reset answered with the arm now reporting mode 5: no Enable, stop sequence"""
    fake = FakeMG400()

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 1:
            fake.mode = 5                                  # someone else enabled it meanwhile
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.MG400Error)
    finally:
        fake.close()
    dash = dashboard_names(fake)
    assert "EnableRobot" not in dash and fake.motion_received == []
    assert report["stop"]["stop_state"] == "reset_disable" and fake.mode == 4


def test_a_start_that_changed_before_the_reset_is_a_fault(tmp_path):
    fake = FakeMG400()

    def confirm(text):
        fake.angles[0] += 0.5                              # the arm was moved while the plan was shown
        return True
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, confirm=confirm)
    finally:
        fake.close()
    assert "start joints changed" in " ".join(report["notes"])
    assert "EnableRobot" not in dashboard_names(fake) and fake.motion_received == []


def test_an_enable_that_moves_the_arm_is_a_fault(tmp_path):
    fake = FakeMG400()
    original = fake._enable_later

    def enable_and_sag():
        original()
        with fake._lock:
            fake.angles[1] += 0.5
    fake._enable_later = enable_and_sag
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    assert "enabling moved" in " ".join(report["notes"])
    assert fake.motion_received == []
    assert report["stop"]["stop_state"] == "reset_disable"


# ----------------------------------------------------------------------
# The stop sequence
# ----------------------------------------------------------------------
def test_stuck_running_reset_then_disable_no_emergency_stop(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["disabled"] is True and stop["last_mode"] == 4
    assert stop["enable_result"] == "acked" and stop["hold_s"] == jog.STOP_HOLD_S
    tail = stop_tail(fake)
    assert tail[:3] == ["ResetRobot", "DisableRobot", "RobotMode"]
    assert "EmergencyStop" not in tail
    assert len(fake.motion_received) == 1                 # the second leg was never sent
    assert fake.mode == 4
    assert jog.STOP_HOLD_S <= stop["duration_s"] < jog.STOP_HOLD_S + 1.0
    assert "did not arrive" in stop["reason"]
    assert stop["guidance"] == jog.GUIDANCE["reset_disable"]
    assert stop["dashboard-1"] == "reused (clean)"
    assert "_t0" not in stop and "deadline" not in stop
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable"


def test_alarm_with_the_arm_still_no_emergency_stop(tmp_path):
    fake = FakeMG400(move_s=0.5, alarm_on_move=True)
    try:
        report = run_failing(fake, tmp_path, exc=jog.MG400Error)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "alarm_still" and stop["last_mode"] == 9
    tail = stop_tail(fake)
    assert tail[:2] == ["ResetRobot", "DisableRobot"] and "EmergencyStop" not in tail
    assert len(stop["angles"]) >= 3
    assert jog.STOP_POLL_S <= stop["duration_s"] < jog.STOP_POLL_S + 1.5
    assert "9" in stop["reason"]
    assert "clear the alarm" in stop["guidance"]


def test_alarm_with_the_arm_moving_emergency_stop(tmp_path):
    """Mode 9 is not by itself a reason to skip EmergencyStop: here neither Reset nor Disable
    stops the (hypothetical) motion under the alarm"""
    fake = FakeMG400(move_s=0.5, alarm_on_move=True, alarm_keeps_moving=True, reset_fails=True,
                     disable_fails=True)
    try:
        report = run_failing(fake, tmp_path, exc=jog.MG400Error)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "emergency_stop"
    tail = stop_tail(fake)
    assert tail.count("EmergencyStop") == 1 and tail.index("EmergencyStop") > tail.index("DisableRobot")
    j1 = [a[0] for t, a in stop["angles"]]
    assert max(j1) - min(j1) > jog.STILL_DEG                  # the record shows the motion that decided it
    assert "emergency-stop button" in stop["guidance"]


def test_disable_without_effect_emergency_stop(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True, disable_fails=True)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "emergency_stop" and stop["disabled"] is False
    tail = stop_tail(fake)
    assert tail[:2] == ["ResetRobot", "DisableRobot"]
    assert tail.count("EmergencyStop") == 1 and tail.index("EmergencyStop") > tail.index("DisableRobot")
    assert fake.mode == 9 and stop["last_mode"] == 9
    assert jog.STOP_POLL_S + jog.STOP_ESTOP_POLL_S <= stop["duration_s"] < jog.STOP_TOTAL_S
    assert stop["deadline_hit"] is False


def test_reset_and_disable_without_effect_while_running_emergency_stop(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True, disable_fails=True, reset_fails=True)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "emergency_stop"
    assert 7 in [m for t, m in stop["modes"]] and 4 not in [m for t, m in stop["modes"]]
    assert "EmergencyStop" in stop_tail(fake) and fake.mode == 9


@pytest.mark.parametrize("flips,expected", [(1, "reset_disable"), (2, "emergency_stop")])
def test_a_disabled_reading_that_does_not_hold_disables_again_then_emergency_stops(tmp_path, monkeypatch,
                                                                                    flips, expected):
    """Mode 4 read once, then 5 again (a late Enable): a second DisableRobot; if that does not
    hold either, the last resort"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    polls_after_disable = []
    flipped = []

    def on_command(cmd):
        if cmd.startswith("RobotMode") and "DisableRobot()" in fake.received:
            polls_after_disable.append(1)
            # the first three RobotMode polls after each Disable answer 4 (about 0.6 s into the
            # hold); the fourth finds the arm enabled again
            if len(polls_after_disable) % 4 == 0 and len(flipped) < flips and fake.mode == 4:
                flipped.append(1)
                fake.mode = 5
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["disable_attempts"] == 2 and len(flipped) == flips
    assert stop_tail(fake).count("DisableRobot") == 2
    assert stop["stop_state"] == expected
    if expected == "emergency_stop":
        assert stop["hold_broken"] == 5 and "EmergencyStop" in stop_tail(fake)
    else:
        assert "hold_broken" not in stop and "EmergencyStop" not in stop_tail(fake)


def test_an_enable_not_shown_complete_ends_in_emergency_stop(tmp_path, monkeypatch):
    """Enable acknowledged but RobotMode 5 never read: a mode 4 that holds now proves nothing
    about an Enable that may still take effect, so the last resort is used anyway"""
    monkeypatch.setattr(jog, "ENABLE_DEADLINE_S", 0.5)
    fake = FakeMG400(enable_s=None)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["enable_result"] == "acked" and stop["enable_completed"] is False and stop["enable_known"] is False
    assert stop["disabled"] is True and stop["stop_state"] == "emergency_stop"
    assert "Enable" in stop["why_emergency_stop"]
    tail = stop_tail(fake)
    assert tail[:2] == ["ResetRobot", "DisableRobot"] and tail.count("EmergencyStop") == 1
    assert fake.motion_received == [] and fake.mode == 9
    assert "not reported within" in stop["reason"]


def test_a_late_enable_cannot_slip_past_the_stop(tmp_path, monkeypatch):
    """The Enable completes 2.8 s after it was sent: after Reset, Disable and the hold, the arm
    would become enabled with the tool gone. The EmergencyStop sent because the Enable's
    outcome was unknown leaves it in alarm instead"""
    monkeypatch.setattr(jog, "ENABLE_DEADLINE_S", 0.5)
    fake = FakeMG400(enable_s=2.8)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        time.sleep(3.0)                                    # let the late enable fire
        final_mode = fake.mode
        fake.close()
    assert report["stop"]["stop_state"] == "emergency_stop"
    assert final_mode == 9                                 # not 5


def test_an_enable_whose_reply_was_lost_is_unresolved(tmp_path):
    fake = FakeMG400()
    fake.ack_lost.add("EnableRobot")                      # executed, connection closed instead of a reply
    try:
        report = run_failing(fake, tmp_path, exc=protocol.MG400ConnectionError)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["enable_result"] == "unresolved" and stop["enable_known"] is False
    assert stop["disabled"] is True and stop["stop_state"] == "emergency_stop"
    assert stop["dashboard-1"].startswith("not reused") and stop["new_connections"] == 1
    assert fake.mode == 9 and fake.motion_received == []


def test_a_hold_that_replies_stop_showing_is_unconfirmed(tmp_path, monkeypatch):
    """First RobotMode after the Disable: 4; afterwards RobotMode is refused while GetAngle
    keeps answering: the hold is not shown, so EmergencyStop follows within hold + grace"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    real_reply = fake.reply
    after = []

    def reply(cmd):
        if cmd.startswith("RobotMode") and "DisableRobot()" in fake.received:
            after.append(1)
            if len(after) > 1:
                return f"-1,{{}},{cmd};"
        return real_reply(cmd)
    fake.reply = reply
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["hold_unconfirmed"] is True and stop["disabled"] is False
    assert stop["stop_state"] == "emergency_stop" and "EmergencyStop" in stop_tail(fake)
    assert stop["duration_s"] < jog.STOP_HOLD_S + jog.STOP_HOLD_GRACE_S + jog.STOP_ESTOP_POLL_S + 2.0
    assert stop["deadline_hit"] is False


def test_a_hold_shown_only_by_a_late_reply_is_unconfirmed(tmp_path, monkeypatch):
    """After the first 4, RobotMode is refused for 1.3 s; the next RobotMode is answered 4 but
    only after 0.9 s (within the 1 s socket timeout), at about 2.3 s after the first 4: past
    hold + grace, so it must not confirm the hold (review 3 M1)"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    real_reply = fake.reply
    first4 = {}

    def reply(cmd):
        if cmd.startswith("RobotMode") and "DisableRobot()" in fake.received:
            if "t" not in first4:
                first4["t"] = time.monotonic()
                return real_reply(cmd)                            # 4
            if time.monotonic() - first4["t"] < 1.3:
                return f"-1,{{}},{cmd};"
            time.sleep(0.9)
            return real_reply(cmd)                                # a 4, read too late
        return real_reply(cmd)
    fake.reply = reply
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["disabled"] is False and stop["hold_unconfirmed"] is True
    assert stop["stop_state"] == "emergency_stop" and "EmergencyStop" in stop_tail(fake)


def test_an_io_thread_that_cannot_start_enables_nothing_and_defers_saves(tmp_path, monkeypatch):
    """No thread: a synchronous save could block the main thread between the stop commands, so
    the run ends at the armed point (Enable never sent), saves and notices wait for the end,
    and the stop sequence runs without them. Saves after arming here take 1 s each: with
    synchronous saves the stop (Reset, Disable, polls) would take several seconds"""
    monkeypatch.setattr(jog.BackgroundIO, "start", lambda self: (_ for _ in ()).throw(RuntimeError("no thread")))
    real_write = observe.Recorder._write_tmp

    def slow_write(self, tmp):
        if self.report.get("armed"):
            time.sleep(1.0)
        return real_write(self, tmp)
    monkeypatch.setattr(observe.Recorder, "_write_tmp", slow_write)
    fake = FakeMG400()
    out = []
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, out=out.append)
    finally:
        fake.close()
    assert report["jog"]["io_thread"].startswith("not started") and "deferred" in report["jog"]["io_thread"]
    assert "io_thread_drained" not in report
    assert "EnableRobot" not in names(fake.received) and report["armed"] is True
    stop = report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["enable_result"] == "not_sent" and fake.mode == 4
    assert stop["duration_s"] < 3.0                       # no 1 s save between the stop commands
    assert any(line.startswith("stop sequence: reset_disable") for line in out)   # said at the end
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]


def test_a_refused_enable_is_resolved(tmp_path):
    fake = FakeMG400()
    real_reply = fake.reply

    def reply(cmd):
        if cmd.startswith("EnableRobot"):
            return f"-1,{{}},{cmd};"
        return real_reply(cmd)
    fake.reply = reply
    try:
        report = run_failing(fake, tmp_path, exc=protocol.MG400CommandError)
    finally:
        fake.close()
    assert report["stop"]["enable_result"] == "refused" and report["stop"]["enable_known"] is True
    assert report["stop"]["stop_state"] == "reset_disable" and "EmergencyStop" not in stop_tail(fake)


def test_refused_motion_stops_and_the_arm_never_moved(tmp_path):
    fake = FakeMG400(refuse_motion=True)
    try:
        report = run_failing(fake, tmp_path, exc=protocol.MG400CommandError)
    finally:
        fake.close()
    assert fake.moves == [] and len(fake.motion_received) == 1
    assert report["stop"]["stop_state"] == "reset_disable"
    assert fake.mode == 4


def test_a_refused_reset_in_the_stop_sequence_does_not_stop_the_sequence(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    real_reply = fake.reply

    def reply(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            return f"-1,{{}},{cmd};"
        return real_reply(cmd)
    fake.reply = reply
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert [s["outcome"][:7] for s in stop["steps"] if s.get("command") == "ResetRobot()"] == ["refused"]
    assert stop["stop_state"] == "reset_disable" and fake.mode == 4


def test_feedback_lost_during_the_move_stops(tmp_path):
    fake = FakeMG400(move_s=1.0)
    dropped = []

    def poll(run_):
        if run_.phase == "moving" and fake.motion_received and not dropped:
            dropped.append(time.monotonic())           # the first JointMovJ is out: the feed ends
            fake.drop_feedback()
    try:
        report = run_failing(fake, tmp_path, exc=observe.ObservationEnded, hooks={"poll": poll})
    finally:
        fake.close()
    assert report["stop"]["stop_state"] == "reset_disable"
    assert len(fake.motion_received) == 1 and fake.mode == 4
    assert "feedback" in report["stop"]["reason"]


def test_feedback_mode_outside_the_phase_is_a_fault(tmp_path):
    """The arm starts running on its own after the enable: fault (reply or packet), stop"""
    fake = FakeMG400()
    original = fake._enable_later

    def enable_then_run():
        original()
        time.sleep(0.15)
        fake.mode = 7
    fake._enable_later = enable_then_run
    try:
        report = run_failing(fake, tmp_path, exc=jog.MG400Error)
    finally:
        fake.close()
    assert fake.motion_received == []
    assert report["stop"]["stop_state"] == "reset_disable"      # Reset: 7 -> 5, Disable: -> 4
    assert "7" in report["stop"]["reason"]


def test_connection_lost_at_reset_goes_to_a_new_connection(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            fake.drop_on.add("ResetRobot")             # the stop sequence's Reset: connection cut, no reply
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    stop = report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["new_connections"] == 1
    on = [(n, c.split("(")[0]) for n, c in fake.received_on if c.split("(")[0] in protocol.STOP]
    assert on[-3:] == [(1, "ResetRobot"), (2, "ResetRobot"), (2, "DisableRobot")]
    assert [s.get("connection") for s in stop["steps"] if s.get("command") == "DisableRobot()"] == ["dashboard-stop-1"]
    assert [c["label"] for c in report["connections"]] == ["feedback", "dashboard-1", "motion", "dashboard-stop-1"]


def test_no_connection_at_all_is_unconfirmed(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "STOP_POLL_S", 0.5)
    fake = FakeMG400(move_s=0.2, stuck_running=True)

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            fake.drop_on.add("ResetRobot")
    fake.on_command = on_command
    base = connect_to(fake)
    dash_connects = []

    def connect(host, port, timeout):
        if port == jog.DASHBOARD_PORT:
            dash_connects.append(timeout)
            if len(dash_connects) > 1:
                raise ConnectionRefusedError("no more")
        return base(host, port, timeout)
    try:
        with pytest.raises(jog.JogFault) as info:
            jog.jog(LOOPBACK, str(tmp_path), connect=connect, confirm=lambda t: True, out=lambda *a: None)
    finally:
        fake.close()
    stop = info.value.mg400_report["stop"]
    assert stop["stop_state"] == "unconfirmed"              # EmergencyStop could not be sent either
    assert stop["new_connections"] == jog.STOP_NEW_CONNECTIONS
    assert all(t <= jog.STOP_TIMEOUT_S for t in dash_connects[1:])
    outcomes = [s.get("outcome", "") for s in stop["steps"]]
    assert any(o.startswith("not sent") for o in outcomes)
    assert stop["duration_s"] < jog.STOP_TOTAL_S
    assert "emergency-stop button" in stop["guidance"]


def test_the_stop_sequence_ends_by_its_total_deadline(tmp_path, monkeypatch):
    """Every reply trickles in slowly: the per-reply deadline and the total deadline hold"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "STOP_TOTAL_S", 3.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    real_send = socket.socket.sendall
    slow = {"on": False}

    def trickle(sock, data):
        if slow["on"] and data.startswith(b"0,{"):
            for ch in data:                                 # one byte every 0.4 s
                real_send(sock, bytes([ch]))
                time.sleep(0.4)
            return None
        return real_send(sock, data)

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            slow["on"] = True
    fake.on_command = on_command
    monkeypatch.setattr(socket.socket, "sendall", trickle)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        slow["on"] = False
        fake.close()
    stop = report["stop"]
    assert stop["duration_s"] < 3.0 + 0.5                   # at most one trickle interval past the deadline
    assert stop["deadline_hit"] is True and stop["stop_state"] == "unconfirmed"
    assert not any(s.get("outcome") == "ok" for s in stop["steps"] if "command" in s)
    assert "emergency-stop button" in stop["guidance"]


def test_reply_deadline_bounds_a_trickling_reply(tmp_path):
    a, b = socket.socketpair()
    a.settimeout(0.5)
    dash = observe.Dashboard(a, observe.Recorder(str(tmp_path)), "x")
    halt = threading.Event()

    def trickle():
        for ch in b"0,{4},RobotMode();":
            if halt.is_set():
                return
            b.sendall(bytes([ch]))
            time.sleep(0.12)
    t = threading.Thread(target=trickle, daemon=True)
    t.start()
    try:
        t0 = time.monotonic()
        with pytest.raises(protocol.MG400ConnectionError) as e:
            dash.query(protocol.robot_mode())
        assert 0.45 < time.monotonic() - t0 < 1.0 and e.value.kind == "timeout" and e.value.partial_reply
        assert not dash.clean
    finally:
        halt.set()
        t.join(2.0)                                   # the sender ends before its socket is closed
        a.close()
        b.close()


class LateRecv:
    """Stands in for a dashboard socket: recv returns the bytes, but only after a pause that
    carries the read past the deadline the caller set (the socket timeout is not what ends it)"""

    def __init__(self, sock, pause_s):
        self._sock = sock
        self.pause_s = pause_s

    def recv(self, n):
        data = self._sock.recv(n)
        time.sleep(self.pause_s)
        return data

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_a_reply_read_after_the_deadline_is_not_a_success(tmp_path):
    # the deadline is still ahead when the read starts, so the socket is read; the reply is
    # complete only after it has passed. Timed on the slow socket, not on the clock's
    # resolution: a deadline already in the past would be refused before the recv.
    a, b = socket.socketpair()
    a.settimeout(1.0)
    dash = observe.Dashboard(LateRecv(a, 0.3), observe.Recorder(str(tmp_path)), "x")
    b.sendall(b"0,{4},RobotMode();")
    time.sleep(0.05)
    with pytest.raises(protocol.MG400ConnectionError) as e:
        dash.query(protocol.robot_mode(), deadline=time.monotonic() + 0.1)
    assert e.value.kind == "timeout" and "after the deadline" in str(e.value)
    assert dash.clean                                 # the reply itself was consumed
    a.close()
    b.close()


class SlowSend:
    """Stands in for dashboard-1's socket in the stop sequence: a DisableRobot send takes 0.3 s"""

    def __init__(self, sock):
        self._sock = sock

    def sendall(self, data):
        if data.startswith(b"DisableRobot"):
            time.sleep(0.3)
        return self._sock.sendall(data)

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_a_slow_send_cannot_push_a_reply_past_the_total_deadline(tmp_path, monkeypatch):
    """Total 1.0 s. Reset answered after 0.4 s (ok at 0.4). Disable: 0.6 s left, the send takes
    0.3 s and the fake answers 0.4 s later, at 1.1 s: within the socket timeout set at the send
    (0.6 s), past the total deadline. It must not count as ok"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "STOP_TOTAL_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    state = {}

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            time.sleep(0.4)
        if cmd.startswith("DisableRobot"):
            time.sleep(0.4)

    def poll(run_):
        if run_.phase == "moving" and fake.motion_received and "proxy" not in state:
            state["proxy"] = True
            run_.dash._sock = SlowSend(run_.dash._sock)
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    stop = report["stop"]
    disable_steps = [s for s in stop["steps"] if s.get("command") == "DisableRobot()"]
    assert disable_steps and all(s["outcome"] != "ok" for s in disable_steps)
    # the read itself ended at the deadline (it was passed down), not the stop sequence's
    # own re-check of a reply that came back late
    assert disable_steps[0]["outcome"].startswith("connection failed")
    assert stop["disabled"] is False and stop["stop_state"] == "unconfirmed"
    assert stop["deadline_hit"] is True and stop["duration_s"] < 1.0 + 0.5


def test_ctrl_c_after_arming_runs_the_stop_sequence(tmp_path):
    fake = FakeMG400(move_s=1.0)
    fired = []

    def poll(run_):
        if run_.phase == "moving" and fake.motion_received and not fired:
            fired.append(1)                            # during the first leg
            raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["armed"] is True
    assert report["stop"]["stop_state"] == "reset_disable" and "KeyboardInterrupt" in report["stop"]["reason"]
    assert fake.mode == 4 and len(fake.motion_received) == 1


def test_ctrl_c_right_before_a_send_is_the_same_stop(tmp_path):
    fake = FakeMG400(move_s=1.0)
    fired = []

    def before_send(run_):
        if run_.phase == "moving" and not fired:
            fired.append(1)
            raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"before_send": before_send})
    finally:
        fake.close()
    stop = info.value.mg400_report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["dashboard-1"] == "reused (clean)"
    not_sent = [c for c in info.value.mg400_report["commands"] if "sent_utc" not in c]
    assert len(not_sent) == 1 and not_sent[0]["error"].startswith("not sent")


class InterruptingRecv:
    """Stands in for dashboard-1's socket: the first recv raises KeyboardInterrupt, as a real
    Ctrl+C delivered during a blocking receive does; everything else goes to the real socket"""

    def __init__(self, sock):
        self._sock = sock
        self.fired = False

    def recv(self, n):
        if not self.fired:
            self.fired = True
            time.sleep(0.05)                             # the command is on its way to the fake
            raise KeyboardInterrupt
        return self._sock.recv(n)

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_ctrl_c_while_waiting_for_a_reply_leaves_a_dirty_connection(tmp_path):
    fake = FakeMG400(move_s=1.0)
    state = {}

    def poll(run_):
        if run_.phase == "moving" and fake.motion_received and "armed" not in state:
            state["armed"] = True
            run_.dash._sock = InterruptingRecv(run_.dash._sock)
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    stop = info.value.mg400_report["stop"]
    assert stop["dashboard-1"].startswith("not reused: outstanding 1")
    assert stop["new_connections"] == 1 and stop["stop_state"] == "reset_disable"
    assert fake.mode == 4
    interrupted = [c for c in info.value.mg400_report["commands"]
                   if "sent_utc" in c and c.get("reply") is None and c["connection"] == "dashboard-1"]
    assert len(interrupted) == 1                          # recorded as sent, no reply: may have been executed


class InterruptingSend:
    """Stands in for the motion socket: sendall delivers the bytes, then raises KeyboardInterrupt
    (a Ctrl+C arriving during the send). Everything else goes to the real socket"""

    def __init__(self, sock):
        self._sock = sock
        self.fired = False

    def sendall(self, data):
        self._sock.sendall(data)
        if not self.fired:
            self.fired = True
            raise KeyboardInterrupt

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_ctrl_c_during_a_send_counts_as_sent(tmp_path):
    fake = FakeMG400(move_s=0.3)
    state = {}

    def poll(run_):
        if run_.phase == "moving" and "armed" not in state:
            state["armed"] = True
            run_.motion._sock = InterruptingSend(run_.motion._sock)
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    sent = [c for c in report["commands"] if c["connection"] == "motion"]
    assert len(sent) == 1 and "sent_utc" in sent[0] and sent[0].get("reply") is None
    assert sent[0]["error"].startswith("send:")           # sent, interrupted: may have been executed
    assert len(fake.motion_received) == 1                 # it was
    assert report["stop"]["stop_state"] == "reset_disable" and fake.mode == 4
    assert report["stop"]["dashboard-1"] == "reused (clean)"   # the dirty one is the motion connection


def test_a_failure_while_arming_runs_the_stop_sequence(tmp_path):
    """The motion connection cannot be opened: armed, nothing enabled, the stop sequence still runs"""
    fake = FakeMG400()
    base = connect_to(fake)

    def connect(host, port, timeout):
        if port == jog.MOTION_PORT:
            raise ConnectionRefusedError("no motion port")
        return base(host, port, timeout)
    with pytest.raises(protocol.MG400ConnectionError) as info:
        try:
            jog.jog(LOOPBACK, str(tmp_path), connect=connect, confirm=lambda t: True, out=lambda *a: None)
        finally:
            fake.close()
    report = info.value.mg400_report
    assert report["armed"] is True and report["disabled_confirmed"] is False
    assert report["stop"]["stop_state"] == "reset_disable" and report["stop"]["enable_result"] == "not_sent"
    assert "EnableRobot" not in names(fake.received)


def test_ctrl_c_before_arming_sends_nothing(tmp_path):
    fake = FakeMG400()

    def confirm(text):
        raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, confirm=confirm)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["armed"] is False and report.get("stop") is None
    assert not any(n in protocol.CONTROL for n in names(fake.received))


def test_sigint_is_ignored_during_the_stop_sequence_and_restored(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    seen = {}
    before = signal.getsignal(signal.SIGINT)

    def on_command(cmd):
        if cmd.startswith("DisableRobot"):
            h = signal.getsignal(signal.SIGINT)
            seen["during"] = (getattr(h, "__self__", None).__class__, getattr(h, "__self__", None).phase)
    fake.on_command = on_command
    try:
        run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    assert seen["during"] == (jog._Interrupts, "stop")        # the run's handler, in its stop phase at that moment
    assert signal.getsignal(signal.SIGINT) is before


def test_a_blocked_save_does_not_delay_the_stop(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "IO_DRAIN_S", 0.5)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    real_replace = os.replace
    armed = []

    def slow_replace(src, dst):
        if armed:
            time.sleep(2.0)
        return real_replace(src, dst)
    monkeypatch.setattr(os, "replace", slow_replace)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"after_arm": lambda r: armed.append(1)})
    finally:
        armed.clear()
        fake.close()
    sent = {c["command"]: c["sent_monotonic"] for c in report["commands"] if "sent_monotonic" in c}
    assert sent["DisableRobot()"] - sent[fake.motion_received[0]] < 1.0 + 1.0
    assert report["stop"]["stop_state"] == "reset_disable"
    assert report["io_thread_drained"] is False            # the queue was still full of slow saves


def test_a_blocked_stdout_does_not_delay_the_stop(tmp_path, monkeypatch):
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "IO_DRAIN_S", 0.5)
    fake = FakeMG400(move_s=0.2, stuck_running=True)

    def out(text):
        if text.startswith("enabled"):
            time.sleep(3.0)                               # printed between EnableRobot and the first JointMovJ
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, out=out)
    finally:
        fake.close()
    sent = {c["command"]: c["sent_monotonic"] for c in report["commands"] if "sent_monotonic" in c}
    assert sent[fake.motion_received[0]] - sent["EnableRobot()"] < 0.2 + jog.SETTLE_S + 1.0
    assert sent["DisableRobot()"] - sent[fake.motion_received[0]] < 1.0 + 1.0


def test_a_failing_save_after_arming_is_a_fault_and_the_stop_still_goes(tmp_path, monkeypatch):
    fake = FakeMG400(move_s=1.0)
    real_replace = os.replace
    armed = []

    def failing_replace(src, dst):
        if armed:
            raise PermissionError("locked")
        return real_replace(src, dst)
    monkeypatch.setattr(os, "replace", failing_replace)
    monkeypatch.setattr(observe, "REPLACE_RETRY_S", (0.01,))
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"after_arm": lambda r: armed.append(1)})
    finally:
        fake.close()
    assert "record save failed" in " ".join(report["notes"])
    # depending on when the I/O thread failed, the fault came before or after the Enable
    assert report["stop"]["stop_state"] in ("reset_disable", "emergency_stop")
    assert report["stop"]["disabled"] is True
    assert (tmp_path / "report.json.tmp").exists()
    assert any(s.get("command") == "DisableRobot()" and s.get("outcome") == "ok" for s in report["stop"]["steps"])


def test_the_main_thread_saves_and_notes_nothing_synchronously_while_armed(tmp_path, monkeypatch):
    fake = FakeMG400(move_s=0.2)
    calls = []
    real_note = observe.Recorder.note

    def note(self, text):
        calls.append(("note", threading.current_thread().name, self.report.get("armed"), self.report.get("stop")))
        return real_note(self, text)
    monkeypatch.setattr(observe.Recorder, "note", note)
    try:
        run(fake, tmp_path)
    finally:
        fake.close()
    assert calls == []                                      # a complete run notes nothing at all


def test_the_stop_sequence_is_not_run_after_a_confirmed_disable(tmp_path):
    fake = FakeMG400(move_s=0.2)
    count = []

    def on_command(cmd):
        if cmd.startswith("GetErrorID"):
            count.append(1)
            if len(count) == 3:                          # the final check's GetErrorID: connection cut
                fake.drop_on.add("GetErrorID")
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=protocol.MG400ConnectionError)
    finally:
        fake.close()
    assert report.get("stop") is None
    assert dashboard_names(fake).count("ResetRobot") == 1 and "EmergencyStop" not in names(fake.received)
    assert fake.mode == 4


# ----------------------------------------------------------------------
# Phase judge, arrival criterion, stillness, echo
# ----------------------------------------------------------------------
class StubFeed:
    def __init__(self):
        self.lock = threading.Lock()
        self.allowed_modes = {4}

    def check(self):
        pass


def make_run(tmp_path):
    rec = observe.Recorder(str(tmp_path))
    return jog.JogRun(rec, StubFeed(), LOOPBACK, None, 0, 0, lambda *a: None)


def test_phase_sets_with_the_grace_for_late_packets(tmp_path, monkeypatch):
    r = make_run(tmp_path)
    r.set_phase("enabling")
    assert r.fb.allowed_modes == {4, 5}
    r.set_phase("enabled")
    assert r.fb.allowed_modes == {4, 5}                 # packets of mode 4 still allowed ...
    assert r.fb.narrow[1] == {5} and abs(r.fb.narrow[0] - (r.phase_t + jog.PHASE_GRACE_S)) < 1e-6
    r.judge(5)
    for bad in (4, 7, 9):                               # ... but not replies
        with pytest.raises(jog.JogFault):
            r.judge(bad)
    r.set_phase("moving")
    assert r.fb.allowed_modes == {5, 7} and r.fb.narrow[1] == {5, 7}
    r.judge(7)
    with pytest.raises(jog.JogFault):
        r.judge(9)
    r.set_phase("stopping")
    for mode in range(1, 12):
        r.judge(mode)
    assert jog.PHASES["disabled"] == {4} and jog.PHASES["disabling"] == {4, 5}


def test_feedback_recorder_judges_with_the_allowed_set(tmp_path):
    a, b = socket.socketpair()
    fb = observe.FeedbackRecorder(a, str(tmp_path), recv_timeout=0.05)
    fb.allowed_modes = {5, 7}
    fb.start()
    try:
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=1, robot_type=1))
        time.sleep(0.2)
        assert fb.fault is None
        with fb.lock:
            fb.allowed_modes = {5}
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=2, robot_type=1))
        time.sleep(0.2)
        assert fb.fault and "mode 7" in fb.fault
    finally:
        fb.stop()
        b.close()


def test_feedback_recorder_narrows_in_its_own_thread_after_the_grace(tmp_path):
    """No check() from the main thread: a packet of the old state after the grace is a fault"""
    a, b = socket.socketpair()
    fb = observe.FeedbackRecorder(a, str(tmp_path), recv_timeout=0.05)
    fb.allowed_modes = {5, 7}
    fb.narrow = (time.monotonic() + 0.3, {5})
    fb.start()
    try:
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=1, robot_type=1))     # within the grace
        time.sleep(0.15)
        assert fb.fault is None
        time.sleep(0.25)
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=2, robot_type=1))     # after it
        time.sleep(0.2)
        assert fb.fault and "mode 7" in fb.fault
    finally:
        fb.stop()
        b.close()


def test_a_phase_change_during_the_write_does_not_rejudge_the_packet(tmp_path):
    """A mode-7 packet received after the grace of `enabled` ({5}); while the recorder writes
    it, the main thread moves on to `moving` ({5, 7}). The packet is judged by the set that
    applied when it was received: a fault"""
    a, b = socket.socketpair()
    fb = observe.FeedbackRecorder(a, str(tmp_path), recv_timeout=0.05)
    fb.allowed_modes = {5}
    fb.narrow = (time.monotonic() - 1.0, {5})          # the grace is long over

    class ChangingWriter:
        def __init__(self, inner):
            self._inner = inner

        def writerow(self, row):
            with fb.lock:                               # the phase change lands mid-write
                fb.allowed_modes = {5, 7}
                fb.narrow = (time.monotonic() - 1.0, {5, 7})
            return self._inner.writerow(row)
    fb._pw = ChangingWriter(fb._pw)
    fb.start()
    try:
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=1, robot_type=1))
        time.sleep(0.3)
        assert fb.fault and "mode 7" in fb.fault
    finally:
        fb.stop()
        b.close()


def test_arrival_criterion(tmp_path, monkeypatch):
    r = make_run(tmp_path)
    monkeypatch.setattr(jog, "POLL_S", 0.0)
    target, start = [5.0, 10.0, 16.0, 18.0], [0.0, 10.0, 16.0, 18.0]
    r.check = lambda: None

    def feed(modes, angles):
        mi, ai = iter(modes), iter(angles)
        r.mode = lambda: next(mi)
        r.angles = lambda: next(ai)
    # 7, origin, 7, target (mode was 7: not yet), 5, target -> arrived at the 6th poll
    feed([7, 7, 5], [[2.0, 10.0, 16.0, 18.0], target, target])
    assert r.wait_arrival(target, start, time.monotonic() - 1.0)["polls"] == 6
    # a GetAngle at the target right after a RobotMode 7 does not finish the leg
    feed([7, 5], [target, target])
    assert r.wait_arrival(target, start, time.monotonic() - 1.0)["polls"] == 4
    # idle at the origin (the move has not started): never an arrival
    r.mode = lambda: 5
    r.angles = lambda: list(start)
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 0.2)
    with pytest.raises(jog.JogFault):
        r.wait_arrival(target, start, time.monotonic())
    # within 0.2 deg of the target but still at the origin is impossible: offsets are >= 1 deg
    assert jog.MIN_OFFSET_DEG > 2 * jog.ARRIVE_TOL_DEG
    # before MIN_LEG_S nothing finishes the leg
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 20.0)
    r.angles = lambda: list(target)
    t0 = time.monotonic()
    r.wait_arrival(target, start, t0)
    assert time.monotonic() - t0 >= jog.MIN_LEG_S
    # J2 drift is a fault of its own (not the deadline)
    r.angles = lambda: [5.0, 11.5, 16.0, 18.0]
    with pytest.raises(jog.JogFault, match="drifted"):
        r.wait_arrival(target, start, time.monotonic() - 1.0)
    # the deadline is counted from the reply and checked before an arrival is accepted
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 0.2)
    r.mode = lambda: 5
    r.angles = lambda: list(target)
    with pytest.raises(jog.JogFault, match="did not arrive"):
        r.wait_arrival(target, start, time.monotonic() - 0.3)
    slow = iter([5])

    def late_mode():
        time.sleep(0.3)
        return next(slow)
    r.mode = late_mode
    with pytest.raises(jog.JogFault, match="did not arrive"):
        r.wait_arrival(target, start, time.monotonic())            # the reply itself came too late
    # a GetAngle at the target whose reply arrives after the deadline is not an arrival either
    r.mode = lambda: 5

    def late_angles():
        time.sleep(0.3)
        return list(target)
    r.angles = late_angles
    with pytest.raises(jog.JogFault, match="did not arrive"):
        r.wait_arrival(target, start, time.monotonic() - 0.05)


def test_wait_mode_rejects_a_late_reply(tmp_path):
    r = make_run(tmp_path)
    r.check = lambda: None

    def late_mode():
        time.sleep(0.3)
        return 5
    r.mode = late_mode
    with pytest.raises(jog.JogFault, match="not reported within"):
        r.wait_mode(5, 0.2, "enabling")
    r.mode = lambda: 5
    assert r.wait_mode(5, 0.2, "enabling") < 0.1


def test_arrival_is_timed_from_the_reply_not_the_send(tmp_path):
    """The fake answers the JointMovJ 0.5 s late: the leg still needs MIN_LEG_S after the reply"""
    fake = FakeMG400(move_s=0.1)
    real = fake.motion_reply

    def slow_motion_reply(cmd):
        time.sleep(0.5)
        return real(cmd)
    fake.motion_reply = slow_motion_reply
    try:
        report = run(fake, tmp_path)
    finally:
        fake.close()
    for leg in report["jog"]["legs"]:
        # from the reply: the 0.1 s move plus a poll or two, never less than MIN_LEG_S; counted
        # from the send it would be at least 0.5 s more
        assert leg["reply_s"] >= 0.5 and jog.MIN_LEG_S <= leg["arrived_s"] < 0.75


def test_stillness_rule():
    now = 10.0
    a = [1.0, 2.0, 3.0, 4.0]
    assert jog._still([(now - 0.6, a), (now - 0.3, a), (now - 0.05, a)], now)
    assert not jog._still([(now - 0.6, a), (now - 0.05, a)], now)                        # fewer than three
    assert not jog._still([(now - 0.4, a), (now - 0.2, a), (now - 0.05, a)], now)       # span too short
    assert not jog._still([(now - 1.5, a), (now - 1.2, a), (now - 0.9, a)], now)        # latest too old
    moved = [1.0, 2.0, 3.0, 4.06]
    assert not jog._still([(now - 0.6, a), (now - 0.3, a), (now - 0.05, moved)], now)
    assert jog._still([(now - 5.0, moved), (now - 0.6, a), (now - 0.3, a), (now - 0.05, a)], now)  # old ones ignored


def test_joint_mov_j_echo_forms():
    cmd = protocol.joint_mov_j([1, 2, 3, 4], 10)
    full = protocol.parse_reply("0,{},JointMovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=10,AccJ=10);")
    assert protocol.check_echo(full, cmd) is None
    short = protocol.parse_reply("0,{},JointMovJ(1.000000,2.000000,3.000000,4.000000);")
    assert protocol.check_echo(short, cmd) == "echo without optional parameters"
    for bad in ("0,{},JointMovJ();", "0,{},JointMovJ(1.000000,2.000000,3.000000,5.000000);",
                "0,{},MovJ(1.000000,2.000000,3.000000,4.000000);",
                "0,{},JointMovJ(1.000000,2.000000,3.000000);"):
        with pytest.raises(MG400ProtocolError):
            protocol.check_echo(protocol.parse_reply(bad), cmd)
    # the prefix rule is for motion commands only; the bare echo stays accepted for the others
    with pytest.raises(MG400ProtocolError):
        protocol.check_echo(protocol.parse_reply("0,{},PositiveSolution(1.000000,2.000000,3.000000,4.000000);"),
                            protocol.positive_solution([1, 2, 3, 4]))
    assert protocol.check_echo(protocol.parse_reply("0,{},PositiveSolution();"),
                               protocol.positive_solution([1, 2, 3, 4])) == "echo without parameters"


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
def test_dry_run_connects_to_nothing(monkeypatch, capsys):
    monkeypatch.setattr(jog, "_connect_fixed", lambda *a: pytest.fail("connected"))
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("connected"))
    assert jog.main(["--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "EmergencyStop() unless (disabled" in out and "Enable's outcome is known" in out
    assert "30003" in out and "ResetRobot(); RobotMode()" in out


def test_cli_mock_run_and_its_connections(monkeypatch, tmp_path, capsys):
    seen = []
    real_connect = socket.socket.connect

    def guarded(sock, addr):
        seen.append(addr)
        assert addr[0] == LOOPBACK
        return real_connect(sock, addr)
    monkeypatch.setattr(socket.socket, "connect", guarded)
    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **k: pytest.fail("name lookup"))
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("create_connection"))
    assert jog.main(["192.0.2.1", "--mock", "--out", str(tmp_path)]) == 0
    assert len(seen) == 3 and {a[0] for a in seen} == {LOOPBACK}
    out = capsys.readouterr().out
    assert "prompt is skipped" in out and "complete: True" in out and "192.0.2.1" not in out


@pytest.mark.parametrize("argv", [["192.0.2.1"], ["--mock", "--j1", "11"], ["--mock", "--j1", "0"],
                                  ["--mock", "--j1", "0.5"], ["--mock", "--speed", "0"],
                                  ["--mock", "--speed", "31"], [],
                                  ["arm.local", "--controller-version", "1"],
                                  ["192.0.2.1:29999", "--controller-version", "1"]])
def test_cli_refuses_bad_arguments_before_connecting(argv, monkeypatch):
    monkeypatch.setattr(jog, "_connect_fixed", lambda *a: pytest.fail("connected"))
    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **k: pytest.fail("name lookup"))
    with pytest.raises(SystemExit) as e:
        jog.main(argv)
    assert e.value.code == 2


def test_cli_record_error_before_arming_is_exit_2(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(os, "replace", lambda src, dst: (_ for _ in ()).throw(PermissionError("locked")))
    monkeypatch.setattr(observe, "REPLACE_RETRY_S", (0.01,))
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 2
    assert "record error" in capsys.readouterr().out


def test_cli_exit_code_after_a_stop_sequence(monkeypatch, tmp_path, capsys):
    import src.devices.mg400.fake as fake_module
    fakes = {}

    class Stuck(FakeMG400):
        def __init__(self, *a, **k):
            k.update(move_s=0.2, stuck_running=True)
            super().__init__(*a, **k)
            fakes["f"] = self
    monkeypatch.setattr(fake_module, "FakeMG400", Stuck)
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "stop sequence: reset_disable" in out and "Check the arm" in out
    assert fakes["f"].mode == 4


# ----------------------------------------------------------------------
# Review 4: the receive-time snapshot, the deadline after the checks, the Enable's send
# ----------------------------------------------------------------------
class DataFlag:
    """Stands in for the feedback socket: remembers that a recv returned data"""

    def __init__(self, sock, state):
        self._sock = sock
        self._state = state

    def recv(self, n):
        data = self._sock.recv(n)
        if data:
            self._state["got_data"] = True
        return data

    def __getattr__(self, name):
        return getattr(self._sock, name)


def test_the_receive_time_and_the_allowed_set_are_taken_together(tmp_path):
    """A phase change that lands between the receiver's recv and its lock must not pair an
    earlier receive time with the later set: the time is stamped under the lock, after it"""
    a, b = socket.socketpair()
    state = {"got_data": False, "changed_at": None}
    fb = observe.FeedbackRecorder(DataFlag(a, state), str(tmp_path))
    real = fb.lock

    class ChangeBeforeLock:
        def __enter__(self):
            if (state["got_data"] and state["changed_at"] is None
                    and threading.current_thread().name == "mg400-feedback"):
                with real:                                 # the phase change: enabled -> moving
                    fb.allowed_modes = {5, 7}
                    fb.narrow = (time.monotonic() + 0.3, {7})
                time.sleep(0.05)                           # a clock tick, on any clock
                state["changed_at"] = time.monotonic()
            return real.__enter__()

        def __exit__(self, *exc):
            return real.__exit__(*exc)
    fb.lock = ChangeBeforeLock()
    fb.start()
    try:
        b.sendall(feedback.build(robot_mode=7, timestamp_ms=1, robot_type=1))
        deadline = time.monotonic() + 3.0
        times = []
        while time.monotonic() < deadline and not times:
            time.sleep(0.02)
            with real:
                times = list(fb.recv_times)
        assert times and state["changed_at"] is not None
        assert times[0] >= state["changed_at"]            # stamped after the change, not before
    finally:
        fb.stop()
        fb.join(timeout=2.0)
        a.close()
        b.close()


def test_a_reply_checked_past_the_deadline_is_a_timeout(tmp_path, monkeypatch):
    """The reply is parsed before the deadline; the echo check carries it past: not a success"""
    real_check = protocol.check_echo

    def slow_check(reply, command):
        time.sleep(0.3)
        return real_check(reply, command)
    monkeypatch.setattr(observe.protocol, "check_echo", slow_check)
    a, b = socket.socketpair()
    a.settimeout(1.0)
    dash = observe.Dashboard(a, observe.Recorder(str(tmp_path)), "x")
    b.sendall(b"0,{4},RobotMode();")
    time.sleep(0.05)
    with pytest.raises(protocol.MG400ConnectionError) as e:
        dash.query(protocol.robot_mode(), deadline=time.monotonic() + 0.1)
    assert e.value.kind == "timeout" and "after the deadline" in str(e.value)
    assert dash.clean
    a.close()
    b.close()


def test_a_reply_adopted_after_the_stop_deadline_is_not_ok(tmp_path, monkeypatch):
    """Total 1.0 s. Reset answered at 0.4 s. The Disable's reply comes back from a query that
    ignored the deadline (a read that let it through), 0.7 s later: the stop sequence itself
    must refuse to count it as ok, and nothing is confirmed"""
    monkeypatch.setattr(jog, "MOVE_DEADLINE_S", 1.0)
    monkeypatch.setattr(jog, "STOP_TOTAL_S", 1.0)
    fake = FakeMG400(move_s=0.2, stuck_running=True)
    state = {}

    def on_command(cmd):
        if cmd.startswith("ResetRobot") and fake.received.count("ResetRobot()") == 2:
            time.sleep(0.4)

    def poll(run_):
        if run_.phase == "moving" and fake.motion_received and "wrapped" not in state:
            state["wrapped"] = True
            orig = run_.dash.query

            def leaky_query(command, tag=None, deadline=None):
                reply = orig(command, tag)                 # the deadline dropped on purpose
                if command.name == "DisableRobot":
                    time.sleep(0.7)
                return reply
            run_.dash.query = leaky_query
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    stop = report["stop"]
    disable_steps = [s for s in stop["steps"] if s.get("command") == "DisableRobot()"]
    assert disable_steps and disable_steps[0]["outcome"] == "reply after the stop deadline"
    assert all(s["outcome"] != "ok" for s in disable_steps)
    assert stop["disabled"] is False and stop["stop_state"] == "unconfirmed"
    assert stop["deadline_hit"] is True and stop["duration_s"] < 1.0 + 1.0


def test_ctrl_c_in_the_guard_of_the_enable_leaves_it_not_sent_and_known(tmp_path):
    """Nothing left for the Enable: its outcome is known (not_sent), so a disabled that holds
    ends the stop sequence without EmergencyStop"""
    fake = FakeMG400()
    fired = []

    def before_send(run_):
        if run_.phase == "enabling" and not fired:
            fired.append(1)
            raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"before_send": before_send})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert "EnableRobot" not in names(fake.received)
    enable = [c for c in report["commands"] if c["command"] == "EnableRobot()"]
    assert len(enable) == 1 and "sent_utc" not in enable[0] and enable[0]["error"].startswith("not sent")
    stop = report["stop"]
    assert stop["enable_result"] == "not_sent" and stop["enable_known"] is True
    assert stop["stop_state"] == "reset_disable" and "EmergencyStop" not in stop_tail(fake)
    assert fake.mode == 4


def test_ctrl_c_during_the_enable_send_is_unresolved(tmp_path):
    """The Enable's bytes left (the fake executed it) and the reply was never read: its outcome
    is not known, so the stop sequence ends in EmergencyStop whatever the polls show"""
    fake = FakeMG400()
    state = {}

    def poll(run_):
        if run_.phase == "enabling" and "wrapped" not in state:
            state["wrapped"] = True
            run_.dash._sock = InterruptingSend(run_.dash._sock)
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    enable = [c for c in report["commands"] if c["command"] == "EnableRobot()"]
    assert len(enable) == 1 and "sent_utc" in enable[0] and enable[0]["error"].startswith("send:")
    assert "EnableRobot" in names(fake.received)
    stop = report["stop"]
    assert stop["enable_result"] == "unresolved" and stop["enable_known"] is False
    assert stop["stop_state"] == "emergency_stop" and "EmergencyStop" in stop_tail(fake)
    assert stop["dashboard-1"].startswith("not reused")   # one request without its reply


# ----------------------------------------------------------------------
# Review 5: the Enable's record decides, Ctrl+C while the I/O thread starts, a broken
# stdout at the end
# ----------------------------------------------------------------------
def test_the_stop_sequence_reads_the_enable_from_the_record_not_the_attribute(tmp_path):
    """The attribute says not_sent while the record shows the Enable's bytes left (sent_utc):
    the window of a Ctrl+C landing between write() returning and the next statement, put
    into the run by hand once the Enable was read. The stop sequence must follow the
    record and end in EmergencyStop"""
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            run_.enable_result, run_.enable_completed = "not_sent", False
            raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    enable = [c for c in report["commands"] if c["command"] == "EnableRobot()"]
    assert len(enable) == 1 and "sent_utc" in enable[0] and "error" not in enable[0]
    assert fired and "EnableRobot" in names(fake.received)
    stop = report["stop"]
    assert stop["enable_result"] == "unresolved" and stop["enable_known"] is False
    assert stop["stop_state"] == "emergency_stop" and "EmergencyStop" in stop_tail(fake)


def test_ctrl_c_while_the_io_thread_starts_enables_nothing_and_defers_saves(tmp_path, monkeypatch):
    """A Ctrl+C that lands in the thread's start is not a RuntimeError: the saves must still be
    deferred (1 s saves after arming here), the Enable never sent and the stop sequence quick"""
    monkeypatch.setattr(jog.BackgroundIO, "start", lambda self: (_ for _ in ()).throw(KeyboardInterrupt()))
    real_write = observe.Recorder._write_tmp

    def slow_write(self, tmp):
        if self.report.get("armed"):
            time.sleep(1.0)
        return real_write(self, tmp)
    monkeypatch.setattr(observe.Recorder, "_write_tmp", slow_write)
    fake = FakeMG400()
    out = []
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, out=out.append)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert "deferred" in report["jog"]["io_thread"] and "io_thread_drained" not in report
    assert "EnableRobot" not in names(fake.received) and report["armed"] is True
    stop = report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["enable_result"] == "not_sent" and fake.mode == 4
    assert stop["duration_s"] < 3.0                       # no 1 s save between the stop commands
    assert any(line.startswith("stop sequence: reset_disable") for line in out)
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]


def test_a_broken_stdout_at_the_end_does_not_cost_the_record(tmp_path, monkeypatch):
    """The deferred notices are printed at the end; stdout is gone (BrokenPipeError from the
    first of them on): the final save still happens, the exception still carries the report
    with its stop section, and the failure is recorded (the "jog ended" line goes through
    _say, which swallows the failure)"""
    monkeypatch.setattr(jog.BackgroundIO, "start", lambda self: (_ for _ in ()).throw(RuntimeError("no thread")))
    fake = FakeMG400()
    printed, broken = [], []

    def out(text, *a, **k):
        if broken or str(text).startswith("stop sequence:"):
            broken.append(text)
            raise BrokenPipeError(32, "Broken pipe")
        printed.append(text)
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, out=out)
    finally:
        fake.close()
    assert report["stop"]["stop_state"] == "reset_disable" and report["armed"] is True
    assert len(broken) == 2                                # the stop notice, then "jog ended"
    assert not any(str(t).startswith("stop sequence:") for t in printed)
    errors = report["notice_errors"]
    assert len(errors) == 1 and "BrokenPipeError" in errors[0]["error"]
    assert errors[0]["unprinted"] == ["stop sequence: reset_disable (last RobotMode 4). %s" % report["stop"]["guidance"]]
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]
    assert len(on_disk["notice_errors"]) == 1


# ----------------------------------------------------------------------
# Review 6: the stop sequence without the thread wherever the fault landed, the clean-up
# under its own guards, a thread that started without its flag
# ----------------------------------------------------------------------
def _slow_saves_after_arming(monkeypatch, seconds=1.0):
    real_write = observe.Recorder._write_tmp

    def slow_write(self, tmp):
        if self.report.get("armed"):
            time.sleep(seconds)
        return real_write(self, tmp)
    monkeypatch.setattr(observe.Recorder, "_write_tmp", slow_write)


def test_ctrl_c_between_the_armed_point_and_the_thread_leaves_no_save_in_the_stop(tmp_path, monkeypatch):
    """The interrupt lands in arm() after armed = True and before the thread is started (here:
    in the _utc() of armed_utc): the stop sequence itself must make the saves no-ops, or the
    1 s saves would sit between its commands"""
    _slow_saves_after_arming(monkeypatch)
    real_utc = jog._utc
    confirmed, fired = [], []

    def utc():
        if confirmed and not fired:
            fired.append(1)
            raise KeyboardInterrupt
        return real_utc()
    monkeypatch.setattr(jog, "_utc", utc)
    fake = FakeMG400()
    out = []
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, out=out.append, confirm=lambda text: confirmed.append(1) or True)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert fired and report["armed"] is True and "armed_utc" not in report["jog"]
    assert "io_thread" not in report["jog"] and "io_thread_drained" not in report
    assert "EnableRobot" not in names(fake.received)
    stop = report["stop"]
    assert stop["stop_state"] == "reset_disable" and stop["enable_result"] == "not_sent" and fake.mode == 4
    assert stop["duration_s"] < 3.0                       # no 1 s save between the stop commands
    assert any(line.startswith("stop sequence: reset_disable") for line in out)
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]


class Interrupt(BaseException):
    """Stands in for KeyboardInterrupt in the clean-up tests: the code treats every
    BaseException alike, and a real KeyboardInterrupt escaping a mutant would abort pytest
    itself (exit 2) instead of failing the test"""


def test_a_ctrl_c_in_the_clean_up_does_not_cost_the_record(tmp_path, monkeypatch):
    """The join of the I/O thread is interrupted after the stop sequence ran: the connections
    are still closed, the feedback summary and ended_utc still written, the report still on
    the exception, and the interruption recorded"""
    monkeypatch.setattr(jog.BackgroundIO, "stop", lambda self, timeout: (_ for _ in ()).throw(Interrupt()))
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            raise jog.JogFault("injected")
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    assert report["stop"]["stop_state"] == "reset_disable" and report["armed"] is True
    assert report["cleanup_errors"] == ["finish: Interrupt()"]
    assert report["ended_utc"] and report["feedback"]
    assert all("end_detected_utc" in c for c in report["connections"] if c["label"] != "feedback")   # closed
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["ended_utc"] and on_disk["cleanup_errors"] == report["cleanup_errors"]


def test_a_ctrl_c_in_the_clean_up_of_a_complete_run_is_the_runs_error(tmp_path, monkeypatch):
    """The run completed (disabled confirmed); the interrupt lands in the thread's join: no stop
    sequence (nothing more is sent), the report is complete and on the exception"""
    monkeypatch.setattr(jog.BackgroundIO, "stop", lambda self, timeout: (_ for _ in ()).throw(Interrupt()))
    fake = FakeMG400()
    try:
        with pytest.raises(Interrupt) as info:
            run(fake, tmp_path)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["complete"] is True and report["disabled_confirmed"] is True and "stop" not in report
    assert report["cleanup_errors"] == ["finish: Interrupt()"] and report["ended_utc"]
    assert names(fake.received)[-1] == "GetPose"           # the final read; nothing after it
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["complete"] is True and on_disk["cleanup_errors"]


def test_a_thread_started_without_its_flag_is_recorded_and_drained(tmp_path, monkeypatch):
    """start() returned (the thread runs) and the interrupt landed before io_started was set:
    the record says so, nothing is enabled, and finish() still drains the live thread"""
    real_start = jog.BackgroundIO.start
    seen = {}

    def start_then_interrupt(self):
        seen["io"] = self
        real_start(self)
        raise KeyboardInterrupt
    monkeypatch.setattr(jog.BackgroundIO, "start", start_then_interrupt)
    fake = FakeMG400()
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["jog"]["io_thread"].startswith("start interrupted, thread running") and "deferred" in report["jog"]["io_thread"]
    assert report["io_thread_drained"] is True
    assert "EnableRobot" not in names(fake.received)
    assert report["stop"]["stop_state"] == "reset_disable" and fake.mode == 4
    assert not seen["io"].is_alive()                     # this run's thread, ended by finish()


# ----------------------------------------------------------------------
# Review 7: the stop sequence decides the save method, every clean-up step guarded, no
# return from inside the try
# ----------------------------------------------------------------------
def test_the_stop_sequence_fixes_the_save_method_itself(tmp_path, monkeypatch):
    """The synchronous save is back in place while io_started is true (the window between the
    flag and the queued save, put into the run by hand once enabled): the stop sequence must
    set the queued save itself, or the 1 s saves would sit between its commands"""
    _slow_saves_after_arming(monkeypatch)
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            run_.rec.save = run_._sync_save
            raise KeyboardInterrupt
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    stop = report["stop"]
    assert fired and stop["stop_state"] == "reset_disable" and fake.mode == 4
    assert stop["duration_s"] < 3.0                       # the saves went to the queue
    assert "io_thread_drained" in report                  # the thread was drained (the 1 s saves may outlast the 5 s)
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]


def test_a_complete_run_queues_its_saves(tmp_path, monkeypatch):
    """After the armed point no save is written by the main thread until the run has ended
    (the final save, after ended_utc, is synchronous on purpose)"""
    real_write = observe.Recorder._write_tmp
    writers = []

    def recording_write(self, tmp):
        if self.report.get("armed") and "ended_utc" not in self.report:
            writers.append(threading.current_thread().name)
        return real_write(self, tmp)
    monkeypatch.setattr(observe.Recorder, "_write_tmp", recording_write)
    fake = FakeMG400()
    try:
        report = run(fake, tmp_path)
    finally:
        fake.close()
    assert report["complete"] is True and writers
    assert set(writers) == {"mg400-jog-io"}


def test_an_interrupt_in_the_end_notice_does_not_cost_the_record(tmp_path, monkeypatch):
    """rec.note() of the "jog ended" line is interrupted: the feedback summary, ended_utc and
    the final save still happen, the report is on the exception, the failure recorded"""
    real_note = observe.Recorder.note

    def note(self, text):
        if text.startswith("jog ended"):
            raise Interrupt
        return real_note(self, text)
    monkeypatch.setattr(observe.Recorder, "note", note)
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            raise jog.JogFault("injected")
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    assert report["stop"]["stop_state"] == "reset_disable"
    assert report["cleanup_errors"] == ["notices: Interrupt()"]
    assert report["ended_utc"] and report["feedback"]
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["ended_utc"] and on_disk["cleanup_errors"] == ["notices: Interrupt()"]


def test_an_interrupt_in_the_final_save_still_hands_the_report_over(tmp_path, monkeypatch):
    """save_during() is interrupted: nothing more can reach the disk, but the exception still
    carries the report with its stop section and the failure"""
    monkeypatch.setattr(observe.Recorder, "save_during", lambda self: (_ for _ in ()).throw(Interrupt()))
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            raise jog.JogFault("injected")
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    assert report["stop"]["stop_state"] == "reset_disable" and report["ended_utc"]
    assert report["cleanup_errors"] == ["final_save: Interrupt()"]


def test_a_declined_run_whose_clean_up_is_interrupted_raises(tmp_path, monkeypatch):
    """The human did not confirm; the close of the Dashboard connection is interrupted: the
    interrupt is raised with the report (a return from inside the try would have hidden it
    and the CLI would exit 2 instead of 130)"""
    def close(self):
        raise Interrupt
    monkeypatch.setattr(observe.Dashboard, "close", close)
    fake = FakeMG400()
    try:
        with pytest.raises(Interrupt) as info:
            run(fake, tmp_path, confirm=lambda text: False)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["armed"] is False and "stop" not in report
    assert any(n.startswith("not confirmed") for n in report["notes"])
    assert report["cleanup_errors"] == ["close: Interrupt()"] and report["ended_utc"]
    assert "EnableRobot" not in names(fake.received) and "ResetRobot" not in names(fake.received)
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["cleanup_errors"] == ["close: Interrupt()"]


def test_a_declined_run_returns_the_report(tmp_path):
    fake = FakeMG400()
    try:
        report = run(fake, tmp_path, confirm=lambda text: False)
    finally:
        fake.close()
    assert report["armed"] is False and "stop" not in report and report["ended_utc"]
    assert any(n.startswith("not confirmed") for n in report["notes"])
    assert "EnableRobot" not in names(fake.received)


def test_a_ctrl_c_without_a_report_tells_the_human_to_check_the_arm(tmp_path, monkeypatch, capsys):
    """An interrupt that reaches the CLI without mg400_report (a KeyboardInterrupt from code
    that has no report, or one landing on the raise after the clean-up): the message must
    not claim that nothing was sent"""
    monkeypatch.setattr(jog, "jog", lambda *a, **k: (_ for _ in ()).throw(KeyboardInterrupt()))
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 130
    out = capsys.readouterr().out
    assert "check the arm, then the record" in out and "nothing was enabled" not in out


# ----------------------------------------------------------------------
# Review 8: the clean-up loop resumes, the final save stands on its own, background I/O
# errors are a fault
# ----------------------------------------------------------------------
def _fault_once_enabled():
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            raise jog.JogFault("injected")
    return poll


def test_an_interrupt_between_clean_up_steps_still_runs_the_rest(tmp_path):
    """The interrupt lands between two steps (the hook, outside the step's own guard): the
    loop is re-entered and every step still runs"""
    fake = FakeMG400()
    seen = []

    def between(name):
        seen.append(name)
        if name == "notices" and seen.count("notices") == 1:
            raise Interrupt
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault,
                             hooks={"poll": _fault_once_enabled(), "between_cleanup_steps": between})
    finally:
        fake.close()
    assert report["cleanup_errors"] == ["clean-up: Interrupt()"]
    assert seen.count("notices") == 2 and seen[-1] == "final_save"      # resumed at the same step
    assert any(n.startswith("jog ended") for n in report["notes"])
    assert report["stop"]["stop_state"] == "reset_disable" and report["feedback"] and report["ended_utc"]
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["ended_utc"] and on_disk["cleanup_errors"] == ["clean-up: Interrupt()"]


def test_repeated_interrupts_between_clean_up_steps_end_the_clean_up(tmp_path):
    """An interrupt at every attempt: the clean-up gives up after CLEANUP_ATTEMPTS, the
    original error still carries the report, and every attempt is recorded"""
    fake = FakeMG400()
    seen = []

    def between(name):
        seen.append(name)
        raise Interrupt
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault,
                             hooks={"poll": _fault_once_enabled(), "between_cleanup_steps": between})
    finally:
        fake.close()
    assert report["cleanup_errors"] == ["clean-up: Interrupt()"] * jog.CLEANUP_ATTEMPTS
    assert seen == ["plan"] * jog.CLEANUP_ATTEMPTS                      # never got past the preparation
    assert "ended_utc" not in report and report["stop"]["stop_state"] == "reset_disable"


def test_the_final_save_does_not_depend_on_finish(tmp_path, monkeypatch):
    """No I/O thread (saves were no-ops), and finish() is interrupted before it restores the
    synchronous save: the final save must restore it itself, or the record on disk would
    stop at the armed point"""
    monkeypatch.setattr(jog.BackgroundIO, "start", lambda self: (_ for _ in ()).throw(RuntimeError("no thread")))

    def finish(self):
        raise Interrupt
    monkeypatch.setattr(jog.JogRun, "finish", finish)
    fake = FakeMG400()
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault)
    finally:
        fake.close()
    assert report["cleanup_errors"] == ["finish: Interrupt()"] and report["ended_utc"]
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["armed"] is True and on_disk["stop"]["stop_state"] == "reset_disable" and on_disk["ended_utc"]


def test_a_background_io_error_at_the_end_is_a_fault(tmp_path):
    """The last notice ("disabled ...") fails on the I/O thread: the arm's sequence completed,
    but the run does not end as a success"""
    fake = FakeMG400()

    def out(text, *a, **k):
        if str(text).startswith("disabled ("):
            raise BrokenPipeError(32, "Broken pipe")
    try:
        with pytest.raises(jog.JogFault) as info:
            run(fake, tmp_path, out=out)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert "background I/O reported errors" in str(info.value) and "BrokenPipeError" in str(info.value)
    assert report["complete"] is True and report["disabled_confirmed"] is True and "stop" not in report
    assert report["io_thread_errors"] and report["io_thread_drained"] is True
    assert names(fake.received)[-1] == "GetPose"                       # nothing sent after the final read
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["io_thread_errors"] and on_disk["ended_utc"]


def test_cli_exit_1_on_a_background_io_error(monkeypatch, tmp_path, capsys):
    """stdout breaks at the last notice (written by the I/O thread): exit 1, the message
    names the background error and says the arm had already reported disabled"""
    import sys

    class BreakingStdout:
        def __init__(self, inner):
            self.inner = inner

        def write(self, s):
            if s.startswith("disabled ("):
                raise BrokenPipeError(32, "Broken pipe")
            return self.inner.write(s)

        def __getattr__(self, name):
            return getattr(self.inner, name)
    monkeypatch.setattr(sys, "stdout", BreakingStdout(sys.stdout))
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "background I/O reported errors" in out and "already reported disabled" in out


# ----------------------------------------------------------------------
# Review 9: the run's error and a recorded clean-up error, the drain that did not finish,
# SIGINT held during the clean-up
# ----------------------------------------------------------------------
def test_an_interrupt_in_the_clean_up_preparation_is_kept(tmp_path):
    """A complete run (no error of its own); the interrupt lands in the preparation of the
    clean-up (the "plan" hook): the re-entry must not reset the recorded error to None"""
    fake = FakeMG400()
    seen = []

    def between(name):
        seen.append(name)
        if name == "plan" and seen.count("plan") == 1:
            raise Interrupt
    try:
        with pytest.raises(Interrupt) as info:
            run(fake, tmp_path, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["complete"] is True and report["cleanup_errors"] == ["clean-up: Interrupt()"]
    assert seen.count("plan") == 2 and seen[-1] == "final_save" and report["ended_utc"]
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["complete"] is True and on_disk["cleanup_errors"] == ["clean-up: Interrupt()"]


def test_an_unfinished_drain_is_a_fault_and_late_errors_count(tmp_path, monkeypatch):
    """The I/O thread is still busy when the drain gives up (0.2 s here): that alone is a
    fault, and an error the thread reports after the drain is read live, not from the copy
    finish() took"""
    monkeypatch.setattr(jog, "IO_DRAIN_S", 0.2)
    go = threading.Event()

    def out(text, *a, **k):
        if str(text).startswith("disabled ("):
            go.wait(5.0)                                    # outlives the drain
            raise BrokenPipeError(32, "Broken pipe")        # after finish() copied the list

    def between(name):
        if name == "background_errors":
            go.set()
            time.sleep(0.3)                                 # the thread fails before the check
    fake = FakeMG400()
    try:
        with pytest.raises(jog.JogFault) as info:
            run(fake, tmp_path, out=out, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert "did not finish within 0.2 s" in str(info.value) and "BrokenPipeError" in str(info.value)
    assert report["io_thread_drained"] is False and report["io_thread_errors"]
    assert report["complete"] is True and report["ended_utc"]


def test_sigint_is_held_during_the_clean_up_and_restored(tmp_path):
    """From the first statement of jog()'s finally to the end of the clean-up the interrupt
    phase is "cleanup" (a first Ctrl+C deferred, a second one ends the clean-up; the hook
    runs inside it); the handler is back afterwards"""
    before = signal.getsignal(signal.SIGINT)
    seen = {}

    def between(name):
        h = signal.getsignal(signal.SIGINT)
        seen.setdefault(name, (getattr(h, "__self__", None).__class__, getattr(h, "__self__", None).phase))
    fake = FakeMG400()
    try:
        report = run(fake, tmp_path, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    assert report["complete"] is True
    assert set(seen) >= {"plan", "finish", "notices", "final_save"}
    assert all(v == (jog._Interrupts, "cleanup") for v in seen.values())    # the phase at that moment
    assert signal.getsignal(signal.SIGINT) is before


# ----------------------------------------------------------------------
# Review 10: Ctrl+C as a real SIGINT in every phase (taken at a check, ignored during the
# stop, deferred then escaping in the clean-up), the unknown handler, the late error
# ----------------------------------------------------------------------
def test_ctrl_c_during_the_run_is_taken_before_the_next_send(tmp_path):
    """A real SIGINT while the arm moves: the handler only sets a flag (nothing raised where
    it landed), the next guard() raises KeyboardInterrupt before a send, the stop sequence
    runs"""
    fake = FakeMG400(move_s=0.6)
    state = {}

    def poll(run_):
        if run_.phase == "moving" and "raised" not in state:
            state["raised"] = True
            signal.raise_signal(signal.SIGINT)              # the handler runs here, in the main thread
            state["after"] = run_.ints.requested             # reached: nothing was raised in the hook
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"poll": poll})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert state.get("after") is True
    assert report["stop"]["stop_state"] in ("reset_disable", "emergency_stop") and report["ended_utc"]
    assert report["ctrl_c"]["count"] == 1 and report["ctrl_c"]["raised_at_a_check"] is True
    assert report["ctrl_c"]["phase"] == "done" and fake.mode == 4


def test_a_second_ctrl_c_during_the_stop_is_ignored(tmp_path):
    """A fault starts the stop sequence; a real SIGINT arrives while the fake handles the
    stop's DisableRobot: counted, ignored, the sequence completes and the fault is kept"""
    fake = FakeMG400()
    fired = []

    def poll(run_):
        if run_.phase == "enabled" and not fired:
            fired.append(1)
            raise jog.JogFault("injected")

    def on_command(cmd):
        if cmd.startswith("DisableRobot") and len(fired) == 1:
            fired.append(2)
            signal.raise_signal(signal.SIGINT)
    fake.on_command = on_command
    try:
        report = run_failing(fake, tmp_path, exc=jog.JogFault, hooks={"poll": poll})
    finally:
        fake.close()
    stop = report["stop"]
    assert fired == [1, 2] and stop["stop_state"] == "reset_disable" and "sequence_error" not in stop
    assert report["ctrl_c"]["count"] == 1 and report["ctrl_c"]["raised_at_a_check"] is False
    assert report["ended_utc"] and fake.mode == 4


def test_the_first_ctrl_c_in_the_clean_up_is_deferred(tmp_path):
    """A complete run; a real SIGINT between two clean-up steps: every step still runs, the
    record is complete, and the run ends with KeyboardInterrupt afterwards"""
    fake = FakeMG400()
    seen = []

    def between(name):
        seen.append(name)
        if name == "notices" and seen.count("notices") == 1:
            signal.raise_signal(signal.SIGINT)
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["complete"] is True and report["ended_utc"] and "cleanup_errors" not in report
    assert report["ctrl_c"]["cleanup_count"] == 1 and report["ctrl_c"]["escaped"] is False
    assert seen[-1] == "final_save" and "cleanup_cut_short" not in report
    on_disk = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert on_disk["complete"] is True and on_disk["ended_utc"] and on_disk["ctrl_c"]["cleanup_count"] == 1


def test_a_second_ctrl_c_in_the_clean_up_ends_it(tmp_path):
    """Two real SIGINTs between clean-up steps: the second raises, the clean-up stops where it
    is, the exception still carries the report, which says the record may be incomplete"""
    before = signal.getsignal(signal.SIGINT)
    fake = FakeMG400()
    seen = []

    def between(name):
        seen.append(name)
        if name == "notices" and seen.count("notices") == 1:
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGINT)              # raises KeyboardInterrupt here
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["cleanup_cut_short"] is True and report["complete"] is True
    assert report["cleanup_errors"] == ["clean-up: KeyboardInterrupt()"]
    assert "ended_utc" not in report and seen[-1] == "notices"       # nothing after the second Ctrl+C
    assert signal.getsignal(signal.SIGINT) is before                 # restored


def test_cli_says_when_a_second_ctrl_c_cut_the_clean_up(monkeypatch, tmp_path, capsys):
    def interrupted(*a, **k):
        e = KeyboardInterrupt()
        e.mg400_report = {"armed": True, "disabled_confirmed": True, "complete": True, "cleanup_cut_short": True}
        raise e
    monkeypatch.setattr(jog, "jog", interrupted)
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 130
    out = capsys.readouterr().out
    assert "may be incomplete" in out and "already reported disabled" in out


def test_an_unknown_sigint_handler_refuses_to_run(tmp_path, monkeypatch):
    """getsignal() None means a handler not set from Python, which the tool could not
    restore: it refuses before any connection is opened"""
    monkeypatch.setattr(signal, "getsignal", lambda signum: None)
    fake = FakeMG400()
    try:
        with pytest.raises(ValueError) as info:
            run(fake, tmp_path)
    finally:
        fake.close()
    assert "SIGINT handler" in str(info.value) and fake.received == []
    assert not (tmp_path / "report.json").exists()


def test_an_error_after_the_background_check_is_not_listed_but_the_drain_fault_stands(tmp_path, monkeypatch):
    """The thread fails after background_errors looked at its list: that error is not in
    io_thread_errors (the list is what the check saw), but the unfinished drain was already a
    fault, so the run still ends with exit 1"""
    monkeypatch.setattr(jog, "IO_DRAIN_S", 0.2)
    go = threading.Event()

    def out(text, *a, **k):
        if str(text).startswith("disabled ("):
            go.wait(5.0)
            raise BrokenPipeError(32, "Broken pipe")

    def between(name):
        if name == "final_save":                            # after background_errors
            go.set()
            time.sleep(0.3)
    fake = FakeMG400()
    try:
        with pytest.raises(jog.JogFault) as info:
            run(fake, tmp_path, out=out, hooks={"between_cleanup_steps": between})
    finally:
        fake.close()
    report = info.value.mg400_report
    assert "did not finish within 0.2 s" in str(info.value) and "BrokenPipeError" not in str(info.value)
    assert report["io_thread_drained"] is False and "io_thread_errors" not in report
    assert report["complete"] is True and report["ended_utc"]


# ----------------------------------------------------------------------
# Review 11: the Ctrl+C taken before the next send, the armed point as one store, the
# handler's install / restore, the escape through a deferred notice, after the clean-up
# ----------------------------------------------------------------------
def test_ctrl_c_is_taken_before_the_next_send(tmp_path):
    """A real SIGINT while the fake answers the RobotMode that follows the initial Reset (four
    queries go out in a row there, with no poll between): the next guard() takes it, so no
    second command goes out before the stop's ResetRobot"""
    fake = FakeMG400()
    fired = []

    def on_command(cmd):
        if cmd.startswith("RobotMode") and fake.received.count("ResetRobot()") == 1 and not fired:
            fired.append(len(fake.received))                 # the index after this RobotMode
            signal.raise_signal(signal.SIGINT)
    fake.on_command = on_command
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert fired and fake.received[fired[0]] == "ResetRobot()"          # the stop's, straight after
    assert "EnableRobot" not in names(fake.received)
    assert report["stop"]["stop_state"] == "reset_disable" and report["ctrl_c"]["raised_at_a_check"] is True


def test_a_second_ctrl_c_in_a_deferred_notice_still_ends_the_clean_up(tmp_path, monkeypatch):
    """No I/O thread: the stop notice is printed by finish(); two real SIGINTs land while it is
    printed. The second must not be swallowed as a notice failure: the clean-up ends there"""
    monkeypatch.setattr(jog.BackgroundIO, "start", lambda self: (_ for _ in ()).throw(RuntimeError("no thread")))

    def out(text, *a, **k):
        if str(text).startswith("stop sequence:"):
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGINT)              # raises KeyboardInterrupt here
    fake = FakeMG400()
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path, out=out)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["cleanup_cut_short"] is True and report["cleanup_errors"] == ["finish: KeyboardInterrupt()"]
    assert "notice_errors" not in report and "ended_utc" not in report
    assert report["stop"]["stop_state"] == "reset_disable" and fake.mode == 4


def test_ctrl_c_before_the_armed_point_sends_nothing(tmp_path, monkeypatch):
    """A KeyboardInterrupt on arm()'s first statement (before the armed point is stored): no
    control command was sent, so no stop sequence runs and the record says armed false"""
    monkeypatch.setattr(jog.JogRun, "arm", lambda self: (_ for _ in ()).throw(KeyboardInterrupt()))
    fake = FakeMG400()
    try:
        with pytest.raises(KeyboardInterrupt) as info:
            run(fake, tmp_path)
    finally:
        fake.close()
    report = info.value.mg400_report
    assert report["armed"] is False and "stop" not in report and report["ended_utc"]
    assert not {"ResetRobot", "DisableRobot", "EnableRobot", "EmergencyStop"} & set(names(fake.received))


def test_an_interrupt_landing_in_the_install_leaves_no_handler_behind(tmp_path, monkeypatch):
    """signal.signal installs the handler and the interrupt lands right after it (before the
    flag would have been set): the restore must still run"""
    before = signal.getsignal(signal.SIGINT)
    real_signal = signal.signal

    def installing_then_interrupted(signum, handler):
        result = real_signal(signum, handler)
        if isinstance(getattr(handler, "__self__", None), jog._Interrupts):
            raise KeyboardInterrupt
        return result
    monkeypatch.setattr(signal, "signal", installing_then_interrupted)
    fake = FakeMG400()
    try:
        with pytest.raises(KeyboardInterrupt):
            run(fake, tmp_path)
    finally:
        fake.close()
    assert signal.getsignal(signal.SIGINT) is before and fake.received == []


def test_an_interrupt_landing_in_the_restore_is_retried(tmp_path, monkeypatch):
    before = signal.getsignal(signal.SIGINT)
    real_signal = signal.signal
    attempts = []

    def interrupted_once(signum, handler):
        if handler is before and not attempts:
            attempts.append(1)
            raise KeyboardInterrupt                         # lands in the restore
        return real_signal(signum, handler)
    monkeypatch.setattr(signal, "signal", interrupted_once)
    fake = FakeMG400()
    try:
        with pytest.raises(KeyboardInterrupt) as info:          # the Ctrl+C is not lost: the run ends with it
            run(fake, tmp_path)
    finally:
        fake.close()
    report = info.value.mg400_report                            # restored, retried, and still reported
    assert report["complete"] is True and attempts == [1] and report["ctrl_c"]["count"] == 1
    assert signal.getsignal(signal.SIGINT) is before


def test_a_ctrl_c_after_the_clean_up_is_counted_for_the_end():
    """Once the clean-up is done the handler only counts (it never raises again), and the
    count is what jog() turns into KeyboardInterrupt after restoring the handler, so a late
    Ctrl+C can neither end as exit 0 nor land in the restore"""
    ints = jog._Interrupts()
    ints.phase = "cleanup"
    ints.handler(signal.SIGINT, None)
    assert ints.cleanup_count == 1 and ints.unanswered() is True and ints.escaped is False
    ints.phase = "done"
    ints.handler(signal.SIGINT, None)
    assert ints.count == 2 and ints.cleanup_count == 1 and ints.escaped is False and ints.unanswered() is True


# ----------------------------------------------------------------------
# Review 13: the escape raises once; the CLI answers a Ctrl+C in its own last statements
# ----------------------------------------------------------------------
def test_the_escape_raises_once():
    """The second Ctrl+C of the clean-up raises (the escape); any further one only counts, so
    nothing of the handler's own can land in jog()'s last statements or in the restore"""
    ints = jog._Interrupts()
    ints.phase = "cleanup"
    ints.handler(signal.SIGINT, None)
    assert ints.cleanup_count == 1 and ints.escaped is False
    with pytest.raises(KeyboardInterrupt):
        ints.handler(signal.SIGINT, None)
    assert ints.escaped is True and ints.cleanup_count == 2
    try:
        ints.handler(signal.SIGINT, None)                   # a third one: counted, never raised
        ints.handler(signal.SIGINT, None)
    except KeyboardInterrupt:                               # not let out: it would abort pytest itself
        pytest.fail("the escape raised again")
    assert ints.count == 4 and ints.cleanup_count == 4 and ints.escaped is True
    assert ints.unanswered() is False                       # the run already ended with the escape


def test_cli_answers_a_ctrl_c_in_its_own_last_statements(monkeypatch, tmp_path, capsys):
    """jog() has restored the handler and raised; the Ctrl+C lands while main() prints its
    message (here: in _record_also): main()'s outer handler answers it with exit 130"""
    def interrupted(*a, **k):
        e = KeyboardInterrupt()
        e.mg400_report = {"armed": False}
        raise e
    monkeypatch.setattr(jog, "jog", interrupted)
    monkeypatch.setattr(jog, "_record_also", lambda e: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        code = jog.main(["--mock", "--out", str(tmp_path)])
    except KeyboardInterrupt:                               # not let out: it would abort pytest itself
        pytest.fail("the Ctrl+C in the CLI's last statements was not answered")
    assert code == 130
    out = capsys.readouterr().out
    assert "check the arm, then the record" in out and str(tmp_path) in out     # the folder named, not "above"


# ---------------------------------------------------------------------------
# After the bench of 2026-10-30: a reused --out folder overwrote four records.
# The CLI refuses a folder that already holds files, before anything is opened.
# ---------------------------------------------------------------------------


def test_main_refuses_an_out_folder_that_holds_files(tmp_path, capsys, monkeypatch):
    (tmp_path / "report.json").write_text("{}", encoding="utf-8")
    opened = []
    monkeypatch.setattr(jog, "jog", lambda *a, **k: opened.append(a) or {})
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 2
    assert opened == []                                        # jog() never ran: nothing opened
    assert (tmp_path / "report.json").read_text(encoding="utf-8") == "{}"   # the record untouched
    out = capsys.readouterr().out
    assert "already holds files" in out and str(tmp_path) in out
    assert "Before jogging" not in out                         # refused before the preconditions


def test_main_accepts_an_empty_existing_out_folder(tmp_path):
    (tmp_path / "empty").mkdir()
    assert jog.main(["--mock", "--out", str(tmp_path / "empty")]) == 0
    assert (tmp_path / "empty" / "report.json").exists()


def test_main_treats_an_unreadable_out_folder_as_a_record_error(tmp_path, capsys, monkeypatch):
    import os as _os
    real = _os.listdir

    def denied(path):
        if _os.path.abspath(path) == _os.path.abspath(str(tmp_path)):
            raise PermissionError("denied")
        return real(path)
    monkeypatch.setattr(jog.os, "listdir", denied)
    opened = []
    monkeypatch.setattr(jog, "jog", lambda *a, **k: opened.append(a) or {})
    assert jog.main(["--mock", "--out", str(tmp_path)]) == 2
    assert opened == []
    assert "cannot be checked" in capsys.readouterr().out
