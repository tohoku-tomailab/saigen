"""MG400Robot (type: mg400) on the fake controller: the gates, the six validation steps, the
arrival, the stop latch, the record. No hardware; the fake listens on 127.0.0.1 only.

The mock robot is the real wrapper on the fake (one code path), so these tests exercise the
class a real run uses; only the connection target differs.
"""
import asyncio
import json
import math
import os
import threading
import time

import pytest

from src.devices.mg400 import control, protocol
from src.devices.mg400.control import StopRequested
from src.devices.mg400.fake import LOOPBACK, FakeMG400
from src.devices.safety import mg400_robot as mr
from src.devices.safety.mg400_robot import MG400Robot, MockMG400Robot
from src.devices.safety.validators import WorkspaceValidator, WorkspaceViolationError

BOX = dict(x_min=350.0, x_max=450.0, y_min=-50.0, y_max=50.0, z_min=170.0, z_max=270.0,
           r_min=350.0, r_max=450.0)
OPTS = dict(home_position=[400.0, 0.0, 220.0],
            joint_limits=[[-160, 160], [-25, 85], [-25, 105], [-180, 180]],
            speed_ratio=10, max_move_mm=50.0)
#: the fake's start pose, fk of its default joints (x, y, z, r)
START = FakeMG400.fk([0.071847, 10.278645, 16.846972, 18.555908])


_ROBOTS = []


@pytest.fixture(autouse=True)
def _clean_up_leftover_robots():
    """A test that fails before its cleanup() must not leave the non-daemon control worker
    alive (it would keep the pytest process from ending) nor the fake's sockets open"""
    yield
    while _ROBOTS:
        r = _ROBOTS.pop()
        try:
            if r._ctl is not None:
                r.emergency_stop()
                asyncio.run(r.cleanup())
        except BaseException:  # noqa: BLE001 - best effort at teardown
            pass
        fake = getattr(r, "fake", None)
        if fake is not None:
            fake.close()


def make(tmp_path, box=None, **opts):
    kw = dict(OPTS)
    kw.update(opts)
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**(box or BOX)),
                       record_folder=str(tmp_path / "mg400_5"), **kw)
    _ROBOTS.append(r)
    return r


def run(coro):
    return asyncio.run(coro)


def names(cmds):
    return [c.split("(", 1)[0] for c in cmds]


def real_on_fake(tmp_path, fake, confirmed, **opts):
    """The real class (not the mock subclass) connected to ``fake``: the confirmation path"""
    ports = {control.DASHBOARD_PORT: fake.dashboard_port, control.FEEDBACK_PORT: fake.feedback_port,
             control.MOTION_PORT: fake.motion_port}

    def connect(host, port, timeout):
        return control._connect_fixed(LOOPBACK, ports[port], timeout)

    kw = dict(OPTS)
    kw.update(opts)
    r = MG400Robot(LOOPBACK, workspace_validator=WorkspaceValidator(**BOX),
                   controller_factory=lambda address, folder: control.Controller(LOOPBACK, folder, connect=connect),
                   record_folder=str(tmp_path / "rec"), confirmed=confirmed, **kw)
    _ROBOTS.append(r)
    return r


# ----------------------------------------------------------------------
# options
# ----------------------------------------------------------------------
@pytest.mark.parametrize("bad", [0, 11, 30, 10.0, True, "10", None])
def test_speed_ratio_is_1_to_10_in_v1(bad):
    with pytest.raises(ValueError):
        mr.validate_speed_ratio(bad)
    assert mr.validate_speed_ratio(10) == 10 and mr.MAX_SPEED_RATIO_V1 < protocol.MAX_SPEED_RATIO


@pytest.mark.parametrize("bad", [0.5, 50.1, 100, float("nan"), "50", None])
def test_max_move_mm_is_1_to_50(bad):
    with pytest.raises(ValueError):
        mr.validate_max_move_mm(bad)


@pytest.mark.parametrize("bad", [None, [1, 2], [1, 2, "x"], [float("inf"), 0, 0], "400,0,220"])
def test_home_position_is_required_xyz(bad):
    with pytest.raises(ValueError):
        mr.parse_home_position(bad)


@pytest.mark.parametrize("bad", [None, [[-160, 160]] * 3, [[-160, 160], [-25, 85], [-25, 105]],
                                 [[-170, 160], [-25, 85], [-25, 105], [-180, 180]],
                                 [[-160, 160], [-25, 90], [-25, 105], [-180, 180]],
                                 [[-160, 160], [-25, 85], [-25, 105], [-181, 180]],
                                 [[10, 10], [-25, 85], [-25, 105], [-180, 180]],
                                 [[-160, 160], [-25, 85], [-25, 105], [-180, float("nan")]]])
def test_joint_limits_only_tighten_the_published_ranges(bad):
    with pytest.raises(ValueError):
        mr.parse_joint_limits(bad)
    assert mr.parse_joint_limits(OPTS["joint_limits"])[1] == [-25.0, 85.0]


def test_segment_radius_range():
    # a chord of the circle r = 400 dips below both endpoints (the review's counter-example)
    lo, hi = mr.segment_radius_range([400, -30, 0], [400, 30, 0])
    assert lo == pytest.approx(400.0) and hi == pytest.approx(math.hypot(400, 30))
    lo, hi = mr.segment_radius_range([398.5, 0, 0], [395.0, 50.0, 0])
    assert lo < 398.0 < min(398.5, math.hypot(395, 50)) and hi == pytest.approx(398.5)
    # a pure z move: the point's radius, both
    assert mr.segment_radius_range([300, 400, 10], [300, 400, 50]) == (500.0, 500.0)
    # a segment through the z axis: minimum 0
    lo, hi = mr.segment_radius_range([-10, 0, 0], [10, 0, 0])
    assert lo == 0.0 and hi == 10.0


def test_plan_samples_every_5_mm_with_the_ends():
    s = mr.plan_samples([0, 0, 0], [0, 0, 50])
    assert len(s) == 11 and s[0] == [0, 0, 0] and s[-1] == [0, 0, 50]
    assert mr.plan_samples([0, 0, 0], [0, 0, 1]) == [[0, 0, 0], [0, 0, 1]]
    assert len(mr.plan_samples([0, 0, 0], [0, 0, 12])) == 4            # 0, 4, 8, 12


# ----------------------------------------------------------------------
# the mock run and the record
# ----------------------------------------------------------------------
def test_mock_run_end_to_end_and_the_record(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        assert r.get_current_position() == pytest.approx(START[:3], abs=1e-3)
        await r.go_home()
        await r.move_z(5.0)
        await r.move_xyz(405.0, 10.0, 225.0)
        await r.go_home()
        await r.cleanup()
    run(body())
    fake = r.fake
    assert names(fake.movl_received) == ["MovL"] * 4
    first = fake.movl_received[0]
    assert first.startswith("MovL(400.000000,0.000000,220.000000,") and first.endswith(",User=0,Tool=0,SpeedL=10,AccL=10)")
    # every MovL carries the R of the first PositiveSolution (never changed by a flow)
    assert {c.split(",")[3] for c in fake.movl_received} == {f"{START[3]:.6f}"}
    dash = names([c for n, c in fake.received_on])
    assert dash.count("ResetRobot") == 1 and dash.count("EnableRobot") == 1 and dash.count("DisableRobot") == 1
    assert "EmergencyStop" not in dash and "JointMovJ" not in names(fake.motion_received)
    assert not r.ended_abnormally
    rep = r.run_report()
    assert rep["complete"] and rep["disabled_confirmed"] and not rep["stopped"] and rep["failed_reason"] is None
    folder = tmp_path / "mg400_5"
    report = json.load(open(folder / "report.json", encoding="utf-8"))
    assert report["tool"] == "src.devices.safety.mg400_robot" and report["complete"] is True
    assert report["flow"]["home_position"] == [400.0, 0.0, 220.0] and report["flow"]["speed_ratio"] == 10
    assert len(report["control"]["moves"]) == 4 and report["control"]["moves"][0]["target"][:3] == [400.0, 0.0, 220.0]
    assert "jog" not in report and report["first_packet"]["robot_mode"] == 4
    assert (folder / "feedback.csv").exists() and report["feedback"]["packets"] > 10
    # GetPose() is recorded next to every pose and never used: the fake's GetPose differs from fk
    assert all("get_pose" in p for p in report["control"]["poses"] if "after" not in p)


def test_go_home_within_1_mm_sends_nothing(tmp_path):
    r = make(tmp_path, home_position=[round(START[0], 3), round(START[1], 3), round(START[2], 3)])

    async def body():
        await r.initialize()
        await r.go_home()
        await r.cleanup()
    run(body())
    assert r.fake.movl_received == []


def test_a_held_record_folder_is_refused_before_anything_connects(tmp_path):
    folder = tmp_path / "mg400_5"
    folder.mkdir()
    (folder / "report.json").write_text("{}", encoding="utf-8")
    r = make(tmp_path)
    with pytest.raises(control.MG400RecordError):
        run(r.initialize())
    assert r.fake.received == [] and r.fake.feedback_bytes_received == 0
    run(r.cleanup())


# ----------------------------------------------------------------------
# gates before any control command
# ----------------------------------------------------------------------
def test_without_a_validator_nothing_connects(tmp_path):
    r = MockMG400Robot(workspace_validator=None, record_folder=str(tmp_path / "a"), **OPTS)
    with pytest.raises(RuntimeError):
        run(r.initialize())
    assert r.fake.received == []
    run(r.cleanup())


def test_without_the_confirmation_no_control_command_is_sent(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        r = real_on_fake(tmp_path, fake, confirmed=False)
        with pytest.raises(RuntimeError) as info:
            run(r.initialize())
        assert "確認" in str(info.value)
        dash = names(fake.received)
        assert set(dash) <= {"RobotMode", "GetErrorID", "GetAngle", "PositiveSolution", "GetPose"}
        assert fake.motion_received == []
        # the confirmation given, the same object connects and enables (a new record folder:
        # the one of the refused attempt holds the record of that attempt)
        r.confirm()
        r.record_folder = str(tmp_path / "rec2")
        run(r.initialize())
        assert names(fake.received).count("EnableRobot") == 1
        run(r.cleanup())


def test_alarm_at_connect_refuses_and_sends_no_control_command(tmp_path):
    r = make(tmp_path)
    r.fake.errors = [[67], [], [], [], [], []]
    with pytest.raises(control.MG400Error):
        run(r.initialize())
    assert "EnableRobot" not in names(r.fake.received) and "ResetRobot" not in names(r.fake.received)
    run(r.cleanup())


def test_error_id_minus_1_names_the_remote_control_mode(tmp_path):
    r = make(tmp_path)
    orig = r.fake.reply
    r.fake.reply = lambda cmd: f"-1,{{}},{cmd};" if cmd.startswith("RobotMode") else orig(cmd)
    with pytest.raises(control.MG400Error) as info:
        run(r.initialize())
    assert "TCP/IP" in str(info.value)
    assert names(r.fake.received) == ["RobotMode"]
    run(r.cleanup())


def test_start_pose_outside_the_box_refuses_before_control(tmp_path):
    box = dict(BOX, x_min=410.0)                       # the fake starts at x ~ 398.7
    r = make(tmp_path, box=box, home_position=[420.0, 0.0, 220.0])
    with pytest.raises(WorkspaceViolationError):
        run(r.initialize())
    assert "ResetRobot" not in names(r.fake.received)
    run(r.cleanup())


def test_a_stop_before_connecting_is_a_latch_initialize_sees_first(tmp_path):
    r = make(tmp_path)
    r.emergency_stop()                                  # the GUI's Stop before the connections exist
    assert r.ended_abnormally
    with pytest.raises(RuntimeError):
        run(r.initialize())
    assert r.fake.received == [] and r.fake.feedback_bytes_received == 0
    run(r.cleanup())


# ----------------------------------------------------------------------
# the six validation steps, each refusing before any MovL
# ----------------------------------------------------------------------
_N = [0]


def _refused(tmp_path, coro_fn, text, **kw):
    _N[0] += 1
    r = make(tmp_path / f"case{_N[0]}", **kw)         # one record folder per attempt

    async def body():
        await r.initialize()
        with pytest.raises(WorkspaceViolationError) as info:
            await coro_fn(r)
        assert text in str(info.value)
        await r.cleanup()
    run(body())
    assert r.fake.movl_received == [], r.fake.movl_received
    assert not r.ended_abnormally
    return r


def test_step2_target_outside_the_box(tmp_path):
    _refused(tmp_path, lambda r: r.move_xyz(400.0, 0.0, 290.0), "可動域外")
    _refused(tmp_path, lambda r: r.move_z(60.0), "可動域外")


def test_step3_minimum_move_length(tmp_path):
    x, y, z = START[:3]
    _refused(tmp_path, lambda r: r.move_xyz(x + 0.5, y, z), "最小")


def test_step4_maximum_move_length(tmp_path):
    _refused(tmp_path, lambda r: r.move_z(20.0), "max_move_mm", max_move_mm=10.0)


def test_step5_the_segment_radius_is_checked_analytically(tmp_path):
    # endpoints at r ~ 398.7 and ~ 398.2 pass r_min 398.0; the chord between them dips to ~397.7
    box = dict(BOX, r_min=398.0)
    r = _refused(tmp_path, lambda r: r.move_xyz(395.0, 49.0, START[2]), "r_min", box=box)
    assert math.hypot(395.0, 49.0) > 398.0 and math.hypot(START[0], START[1]) > 398.0


def test_step6_a_sample_outside_the_joint_limits(tmp_path):
    # J2 at the start is 10.28 deg; 20 mm lower it is about 12.5 deg (the fake's planar model)
    _refused(tmp_path, lambda r: r.move_z(-20.0), "J2",
             joint_limits=[[-160, 160], [10.0, 10.5], [-25, 105], [-180, 180]])


def test_step6_a_refused_inverse_solution_names_the_sample(tmp_path):
    r = make(tmp_path)
    orig = r.fake.ik
    r.fake.ik = lambda x, y, z, rr=None: None if z > START[2] + 2.5 else orig(x, y, z, rr)

    async def body():
        await r.initialize()
        with pytest.raises(WorkspaceViolationError) as info:
            await r.move_z(5.0)
        assert "InverseSolution" in str(info.value) and "サンプル 2/2" in str(info.value)
        await r.cleanup()
    run(body())
    assert r.fake.movl_received == [] and not r.ended_abnormally


def test_step6_a_joint_jump_between_samples(tmp_path):
    r = make(tmp_path)
    orig = r.fake.ik

    def jumpy(x, y, z, rr=None):
        j = orig(x, y, z, rr)
        if j is not None and z > START[2] + 10:
            j = [j[0] + 6.0, j[1], j[2], j[3]]
        return j
    r.fake.ik = jumpy

    async def body():
        await r.initialize()
        with pytest.raises(WorkspaceViolationError) as info:
            await r.move_z(20.0)
        assert "関節角" in str(info.value)
        await r.cleanup()
    run(body())
    assert r.fake.movl_received == []


def test_validation_uses_the_measured_start_pose_not_the_last_target(tmp_path):
    """The arm drifts between two moves (the fake's joints are changed behind the wrapper's
    back): the next move is judged from the measured pose, and a start outside the box refuses"""
    r = make(tmp_path)

    async def body():
        await r.initialize()
        await r.move_z(5.0)
        with r.fake._lock:
            r.fake.angles[:4] = r.fake.ik(460.0, 0.0, 225.0)        # x outside the box
            moved = list(r.fake.angles[:4])
        # the fake jumps at once (a real arm cannot): wait for a packet that shows the new
        # joints, or the joint-tracking check would compare GetAngle with an older packet
        fb = r._ctl.fb
        end = time.monotonic() + 5
        while time.monotonic() < end:
            with fb.lock:
                latest = (fb.latest or {}).get("q_actual") or []
            if latest[:4] and max(abs(a - b) for a, b in zip(latest[:4], moved)) < 1e-6:
                break
            time.sleep(0.01)
        # move_xyz measures nothing before _move_to: the start must come from the measurement
        # there, not from the pose remembered after the last move (which is inside the box)
        with pytest.raises(WorkspaceViolationError) as info:
            await r.move_xyz(400.0, 0.0, 220.0)
        assert "開始姿勢" in str(info.value)
        await r.cleanup()
    run(body())
    assert len(r.fake.movl_received) == 1


# ----------------------------------------------------------------------
# arrival and its faults
# ----------------------------------------------------------------------
def test_ack_then_no_motion_ends_in_the_deadline_fault_never_in_arrival(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "MOVE_DEADLINE_S", 0.6)
    monkeypatch.setattr(control, "MOVE_SPEED_FLOOR_MM_S", 1000.0)
    r = make(tmp_path)
    r.fake.ack_no_motion = True
    x, y, z = START[:3]
    d = 1.0 / math.sqrt(3)                                   # a 1.0 mm diagonal move

    async def body():
        await r.initialize()
        with pytest.raises(control.JogFault) as info:
            await r.move_xyz(x + d, y + d, z + d)
        assert "did not arrive" in str(info.value)
        await r.cleanup()
    run(body())
    assert len(r.fake.movl_received) == 1
    dash = names([c for n, c in r.fake.received_on])
    assert dash.count("ResetRobot") == 2 and dash.count("DisableRobot") >= 1      # the stop sequence ran
    rep = r.run_report()
    assert r.ended_abnormally and rep["stopped"] and rep["autonomous"]
    assert rep["stop_state"] == "reset_disable" and "did not arrive" in rep["stop_reason"]


def test_an_alarm_during_the_move_runs_the_stop_sequence(tmp_path):
    r = make(tmp_path)
    r.fake.alarm_on_move = True

    async def body():
        await r.initialize()
        with pytest.raises(control.MG400Error):
            await r.move_z(5.0)
        await r.cleanup()
    run(body())
    rep = r.run_report()
    assert rep["stopped"] and rep["stop_state"] in ("alarm_still", "emergency_stop")
    assert rep["failed_reason"] and r.ended_abnormally
    assert "DisableRobot" in names([c for n, c in r.fake.received_on])


def test_a_move_the_controller_refuses_is_a_fault(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        r.fake.refuse_motion = True
        with pytest.raises(protocol.MG400CommandError):
            await r.move_z(5.0)
        await r.cleanup()
    run(body())
    assert r.run_report()["stopped"] and r.run_report()["stop_state"] == "reset_disable"


# ----------------------------------------------------------------------
# the stop latch
# ----------------------------------------------------------------------
def test_emergency_stop_returns_at_once_and_the_worker_stops_the_arm(tmp_path):
    r = make(tmp_path)
    r.fake.move_s = 3.0
    took = {}

    async def body():
        await r.initialize()
        task = asyncio.ensure_future(r.move_z(5.0))
        await asyncio.sleep(0.4)                          # the MovL was sent, the arm "moves"
        assert r.fake.movl_received and r.fake.mode == 7
        t0 = time.monotonic()
        await asyncio.get_running_loop().run_in_executor(None, r.emergency_stop)
        took["estop"] = time.monotonic() - t0
        with pytest.raises(control.MG400Error):          # StopRequested (a JogFault if the poll saw mode 4 first)
            await task
        took["stopped_after"] = time.monotonic() - t0
        await r.cleanup()
    run(body())
    assert took["estop"] < 0.2, took
    assert took["stopped_after"] < 5.0, took
    rep = r.run_report()
    assert rep["stopped"] and not rep["autonomous"] and rep["stop_reason"] == "emergency_stop"
    assert rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True
    assert r.fake.mode == 4 and rep["failed_reason"] is None


def test_a_cancel_during_a_move_sets_the_latch(tmp_path):
    r = make(tmp_path)
    r.fake.move_s = 3.0

    async def body():
        await r.initialize()
        task = asyncio.ensure_future(r.move_z(5.0))
        await asyncio.sleep(0.4)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await r.cleanup()
    run(body())
    rep = r.run_report()
    assert rep["stopped"] and rep["stop_state"] == "reset_disable" and "中断" in rep["stop_reason"]


def test_after_a_stop_every_move_is_refused_and_cleanup_sends_no_motion(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        r.emergency_stop()
        await asyncio.sleep(0.5)
        with pytest.raises(StopRequested):
            await r.move_z(5.0)
        with pytest.raises(StopRequested):
            await r.go_home()
        await r.cleanup()
    run(body())
    assert r.fake.motion_received == []
    dash = names([c for n, c in r.fake.received_on])
    assert dash.count("DisableRobot") == 1 and "EmergencyStop" not in dash
    assert r.run_report()["stop"]["disabled"] is True


def test_a_feedback_fault_while_idle_stops_the_arm_and_fails_the_run(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        r.fake.drop_feedback()                            # the flow is in a wait: nothing is requested
        await asyncio.sleep(1.0)
        rep = r.run_report()
        assert rep["stopped"] and rep["autonomous"] and "feedback" in rep["stop_reason"]
        with pytest.raises(StopRequested):
            await r.move_z(5.0)
        await r.cleanup()
    run(body())
    rep = r.run_report()
    assert rep["failed_reason"] and "feedback" in rep["failed_reason"]
    assert "DisableRobot" in names([c for n, c in r.fake.received_on]) and r.fake.mode == 4
    assert r.ended_abnormally


def test_cleanup_after_a_normal_run_disables_and_confirms(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        await r.move_z(5.0)
        await r.cleanup()
    run(body())
    rep = r.run_report()
    assert rep["disabled_confirmed"] and rep["complete"] and rep["stop"] is None
    assert not rep["worker_alive"]
    report = json.load(open(tmp_path / "mg400_5" / "report.json", encoding="utf-8"))
    assert report["disabled_confirmed"] is True and "stop" not in report


def test_a_disable_that_is_not_confirmed_runs_the_stop_sequence(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "DISABLE_DEADLINE_S", 0.5)
    r = make(tmp_path)
    r.fake.disable_fails = True

    async def body():
        await r.initialize()
        await r.move_z(5.0)
        await r.cleanup()
    run(body())
    rep = r.run_report()
    assert rep["stopped"] and rep["stop"] is not None
    # disable_fails: the arm never shows mode 4, so the sequence ends in EmergencyStop
    assert rep["stop_state"] == "emergency_stop" and rep["failed_reason"]
    assert "EmergencyStop" in names([c for n, c in r.fake.received_on])


def test_set_current_position_as_home_is_refused(tmp_path):
    r = make(tmp_path)
    with pytest.raises(RuntimeError):
        r.set_current_position_as_home()
    assert r.home_is_within_limits()
    r2 = make(tmp_path / "b", home_position=[500.0, 0.0, 220.0])
    assert not r2.home_is_within_limits()
    run(r.cleanup())
    run(r2.cleanup())


def test_double_initialize_is_refused(tmp_path):
    r = make(tmp_path)

    async def body():
        await r.initialize()
        with pytest.raises(RuntimeError):
            await r.initialize()
        await r.cleanup()
    run(body())
    assert names(r.fake.received).count("EnableRobot") == 1


def test_mock_start_pose_seeds_the_fake(tmp_path):
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**BOX), start_pose=[420.0, 10.0, 230.0, 18.0],
                       record_folder=str(tmp_path / "c"), **OPTS)
    assert r.fake.fk(r.fake.angles[:4]) == pytest.approx([420.0, 10.0, 230.0, 18.0], abs=1e-6)
    with pytest.raises(ValueError):
        MockMG400Robot(workspace_validator=WorkspaceValidator(**BOX), start_pose=[900.0, 0.0, 230.0],
                       record_folder=str(tmp_path / "d"), **OPTS)
    run(r.cleanup())


# ----------------------------------------------------------------------
# the MovL command form (protocol)
# ----------------------------------------------------------------------
def test_mov_l_form_its_speed_cap_and_its_echo_rule():
    cmd = protocol.mov_l([400.0, 0.0, 220.0, 18.5], 10)
    assert cmd.text == "MovL(400.000000,0.000000,220.000000,18.500000,User=0,Tool=0,SpeedL=10,AccL=10)"
    assert "MovL" in protocol.CONTROL and "MovL" in protocol.MOTION
    for bad in (0, 31, 10.0, True, "10"):
        with pytest.raises(ValueError):
            protocol.mov_l([400.0, 0.0, 220.0, 18.5], bad)
    with pytest.raises(ValueError):
        protocol.mov_l([400.0, 0.0, 220.0], 10)
    with pytest.raises(ValueError):                              # a control command needs the flag
        protocol.check_command(cmd.text, "MovL")
    for text in ("MovL(400.000000,0.000000,220.000000,18.500000,User=0,Tool=0,SpeedL=31,AccL=31)",
                 "MovL(400.000000,0.000000,220.000000,18.500000,User=0,Tool=0,SpeedL=10,AccL=20)",
                 "MovL(400.000000,0.000000,220.000000,18.500000,User=1,Tool=0,SpeedL=10,AccL=10)",
                 "MovL(400.000000,0.000000,220.000000,18.500000,SpeedL=10,AccL=10)",
                 "MovL(400.000000,0.000000,220.000000,18.500000)",
                 "MovL(400,0,220,18.5,User=0,Tool=0,SpeedL=10,AccL=10)"):
        with pytest.raises(ValueError):
            protocol.check_command(text, "MovL", control=True)
    # the echo may leave the optional parameters out, never the target
    full = protocol.parse_reply("0,{},MovL(400.000000,0.000000,220.000000,18.500000,User=0,Tool=0,SpeedL=10,AccL=10);")
    assert protocol.check_echo(full, cmd) is None
    short = protocol.parse_reply("0,{},MovL(400.000000,0.000000,220.000000,18.500000);")
    assert protocol.check_echo(short, cmd) == "echo without optional parameters"
    frames = protocol.parse_reply("0,{},MovL(400.000000,0.000000,220.000000,18.500000,User=0,Tool=0);")
    assert protocol.check_echo(frames, cmd) == "echo without optional parameters"
    for echo in ("MovL();", "MovL(400.000000,0.000000,220.000000);",
                 "MovL(400.000000,0.000000,221.000000,18.500000);"):
        with pytest.raises(protocol.MG400ProtocolError):
            protocol.check_echo(protocol.parse_reply("0,{}," + echo), cmd)


# ----------------------------------------------------------------------
# F6: a cancel at every await of initialize(), and a cancelled cleanup wait
# ----------------------------------------------------------------------
@pytest.mark.parametrize("after", ["GetErrorID", "GetPose", "ResetRobot", "EnableRobot"])
def test_a_cancel_during_initialize_is_a_latch_and_the_cleanup_still_finishes(tmp_path, after):
    """The task is cancelled while initialize() awaits the worker (open, pose, take_control or
    enable, by the last command the fake has seen): the latch is set, the stop sequence runs
    when control was taken (nothing to stop before), the connection is closed by the robot's
    own cleanup, no motion was ever sent"""
    r = make(tmp_path)
    fake = r.fake
    fake.enable_s = 0.8

    async def body():
        task = asyncio.ensure_future(r.initialize())
        end = time.monotonic() + 10
        while time.monotonic() < end:
            if after in names(fake.received):          # seen (the poll's granularity is coarse on Windows)
                break
            await asyncio.sleep(0.002)
        else:
            pytest.fail(f"{after} was never sent")
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    run(body())
    rep = r.run_report()
    assert rep["stopped"] and "中断" in rep["stop_reason"] and not rep["complete"]
    assert r._ctl is None and r.ended_abnormally and not r._initialized
    if rep["armed"]:
        # the jog tool's rule: a stop while the Enable's outcome is unknown ends in EmergencyStop
        if rep["stop"]["enable_known"]:
            assert rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True and fake.mode == 4
        else:
            assert rep["stop_state"] == "emergency_stop" and fake.mode == 9 and "EmergencyStop" in names(fake.received)
            assert "EmergencyStop" in rep["failed_reason"] or "emergency_stop" in rep["failed_reason"]
    else:
        assert rep["stop"] is None and fake.mode == 4
    assert fake.motion_received == []
    assert json.load(open(os.path.join(r.record_folder, "report.json"), encoding="utf-8"))["stop_reason"] == rep["stop_reason"]


def test_a_cancelled_cleanup_wait_goes_on_waiting_and_the_report_is_final(tmp_path):
    """The cleanup's wait is cancelled (the GUI Stop, a second cancel) while the shutdown queues
    behind a long move: the cancel becomes a stop, the wait goes on until the shutdown and the
    worker have ended, the report is final (not a snapshot of a running stop), the controller
    is released, and only then the CancelledError is re-raised"""
    r = make(tmp_path)
    r.fake.move_s = 5.0
    seen = {}

    async def body():
        await r.initialize()
        move = asyncio.ensure_future(r.move_z(5.0))                   # the worker is busy with a long MovL
        await asyncio.sleep(0.4)
        task = asyncio.ensure_future(r.cleanup())                     # the shutdown queues behind the move
        await asyncio.sleep(0.2)
        ctl = r._ctl
        task.cancel()
        await asyncio.sleep(0.2)
        seen["latch"] = ctl.latch.is_set()                            # the cancel became a stop at once
        seen["waiting"] = not task.done()                             # ... and the wait goes on
        task.cancel()                                                 # another cancel changes nothing
        with pytest.raises(asyncio.CancelledError):
            await task
        with pytest.raises(control.MG400Error):
            await move
    run(body())
    assert seen == {"latch": True, "waiting": True}
    assert r._ctl is None and r._shutdown_thread is None              # the shutdown finished inside the cleanup
    rep = r.run_report()
    assert not rep["worker_alive"] and rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"]
    assert rep["failed_reason"] is None and "cleanup_incomplete" not in rep
    assert r.fake.mode == 4 and len(r.fake.motion_received) == 1        # the one MovL, stopped by the sequence


def test_a_keyboard_interrupt_of_the_cleanup_wait_is_reported_as_unfinished(tmp_path, monkeypatch):
    """A KeyboardInterrupt from outside SAIGEN's handlers (run_flow's own guard never raises one)
    ends the wait at once: the latch is set,
    the controller is kept (a stop still reaches it), the report says the end could not be
    confirmed, and the next cleanup() waits for the same shutdown and clears that"""
    r = make(tmp_path)
    r.fake.move_s = 3.0
    real_join = MG400Robot._join_shutdown
    calls = []

    async def interrupted_join(self, ctl, th, timeout):
        calls.append(timeout)
        if len(calls) == 1:
            raise KeyboardInterrupt
        await real_join(self, ctl, th, timeout)

    monkeypatch.setattr(MG400Robot, "_join_shutdown", interrupted_join)

    async def body():
        await r.initialize()
        move = asyncio.ensure_future(r.move_z(5.0))
        await asyncio.sleep(0.4)
        with pytest.raises(KeyboardInterrupt):
            await r.cleanup()
        assert r._ctl is not None and r._ctl.latch.is_set()
        rep = r.run_report()
        assert rep["failed_reason"] and "確認できませんでした" in rep["failed_reason"]
        assert rep["cleanup_incomplete"]
        with pytest.raises(control.MG400Error):
            await move
        await r.cleanup()                                             # the same shutdown, waited for to its end
    run(body())
    assert r._ctl is None and len(calls) >= 2
    rep = r.run_report()
    assert "cleanup_incomplete" not in rep and rep["stop_state"] == "reset_disable" and rep["failed_reason"] is None


def test_an_interruption_outside_the_protected_wait_still_reports_the_end_unconfirmed(tmp_path, monkeypatch):
    """A KeyboardInterrupt raised by the wait loop's own check (not inside the awaited join):
    the latch is set, the report carries cleanup_incomplete from the cleanup's entry, the
    controller is kept; the next cleanup() confirms the end and clears it"""
    r = make(tmp_path)
    r.fake.move_s = 3.0
    real_done = MG400Robot._shutdown_done
    calls = []

    def interrupted_done(ctl, th):
        calls.append(1)
        if len(calls) == 1:
            raise KeyboardInterrupt
        return real_done(ctl, th)
    monkeypatch.setattr(MG400Robot, "_shutdown_done", staticmethod(interrupted_done))

    async def body():
        await r.initialize()
        move = asyncio.ensure_future(r.move_z(5.0))
        await asyncio.sleep(0.4)
        with pytest.raises(KeyboardInterrupt):
            await r.cleanup()
        assert r._ctl is not None and r._ctl.latch.is_set()
        rep = r.run_report()
        assert rep["cleanup_incomplete"] and "確認できませんでした" in rep["failed_reason"]
        with pytest.raises(control.MG400Error):
            await move
        await r.cleanup()
    run(body())
    assert r._ctl is None and "cleanup_incomplete" not in r.run_report()


def test_a_worker_that_outlives_the_shutdown_bound_keeps_the_controller_and_fails_the_report(tmp_path, monkeypatch):
    """The shutdown helper returns after its own bound while the worker still runs (here: a long
    move ahead of the shutdown request): the controller is not detached, a stop still reaches the
    worker, the report says the end was not confirmed; a later cleanup() sees the real end"""
    r = make(tmp_path)
    r.fake.move_s = 4.0
    monkeypatch.setattr(mr, "CLEANUP_JOIN_S", 0.2)
    monkeypatch.setattr(mr, "CLEANUP_MARGIN_S", 0.3)

    async def body():
        await r.initialize()
        move = asyncio.ensure_future(r.move_z(5.0))
        await asyncio.sleep(0.4)
        await r.cleanup()                                             # returns after about 0.5 s
        assert r._ctl is not None and r.run_report()["worker_alive"]
        rep = r.run_report()
        assert "確認できませんでした" in rep["failed_reason"] and rep["cleanup_incomplete"]
        r.emergency_stop()
        assert r._ctl.latch.is_set()                                  # the stop reaches the live worker
        with pytest.raises(control.MG400Error):
            await move
        monkeypatch.setattr(mr, "CLEANUP_JOIN_S", 30.0)
        monkeypatch.setattr(mr, "CLEANUP_MARGIN_S", 5.0)
        await r.cleanup()
    run(body())
    assert r._ctl is None
    rep = r.run_report()
    assert not rep["worker_alive"] and "cleanup_incomplete" not in rep and rep["stop_state"] == "reset_disable"
