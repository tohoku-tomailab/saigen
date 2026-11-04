"""Joint tracking at rest for the MG400 flow robot type (the tracking plan, after stage C1's
jog_07: J2's measured angle 2.06 deg below its target while ``GetAngle()`` answered the target).

``control.Controller`` compares the feedback's ``q_actual`` with ``q_target`` and with the
GetAngle value about to be used, at the Enable, at every enabled pose read, before every MovL,
at arrival, and between requests. No hardware; the fake listens on 127.0.0.1 only.
"""
import asyncio
import math
import queue
import threading
import time

import pytest

from src.devices.mg400 import control
from src.devices.mg400.control import Controller, JogFault, StopRequested, TrackingFault
from src.devices.mg400.fake import LOOPBACK, FakeMG400
from src.devices.safety.mg400_robot import MockMG400Robot
from src.devices.safety.validators import WorkspaceValidator

_STARTED = []
_ROBOTS = []


@pytest.fixture(autouse=True)
def _shut_down_leftovers():
    yield
    while _STARTED:
        ctl = _STARTED.pop()
        if ctl._thread.is_alive():
            ctl.request_stop("test teardown")
            ctl.shutdown(30)
    while _ROBOTS:
        r = _ROBOTS.pop()
        try:
            if r._ctl is not None:
                r.emergency_stop()
                asyncio.run(r.cleanup())
        except BaseException:  # noqa: BLE001 - best effort at teardown
            pass
        r.fake.close()


def make(fake, folder, **kw):
    ports = {control.DASHBOARD_PORT: fake.dashboard_port, control.FEEDBACK_PORT: fake.feedback_port,
             control.MOTION_PORT: fake.motion_port}

    def connect(host, port, timeout):
        return control._connect_fixed(LOOPBACK, ports[port], timeout)
    ctl = Controller(LOOPBACK, str(folder), connect=connect, **kw)
    ctl.start()
    _STARTED.append(ctl)
    return ctl


def armed(fake, folder, **kw):
    ctl = make(fake, folder, **kw)
    ctl.open().result(5)
    pose = ctl.pose().result(5)
    ctl.take_control().result(5)
    ctl.enable().result(10)
    return ctl, pose


def wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def published(fb, pred, timeout=5.0):
    """Wait until the reader has published a packet for which ``pred(packet)`` holds: a knob
    set on the fake is seen by the next check only once such a packet exists"""
    def seen():
        with fb.lock:
            pkt = fb.latest
        return pkt is not None and pred(pkt)
    assert wait_for(seen, timeout), "the fake's change never reached the reader"


def sagged(joint, deg):
    return lambda pkt: abs(pkt["q_actual"][joint - 1] - pkt["q_target"][joint - 1] - deg) < 1e-6


def no_idle(ctl):
    """Switch the idle watch off, so a check inside a request is the one under test"""
    ctl._idle_track = lambda: None


def tracking(ctl):
    return ctl.run.report["tracking"]


# ----------------------------------------------------------------------
# one copy, judged on its own (each condition isolated)
# ----------------------------------------------------------------------
def snap(qa=(0.0, 0.0, 0.0, 0.0), qt=(0.0, 0.0, 0.0, 0.0), mode=5, age=0.0, fault=None,
         error_status=0, nonfinite=(), out_of_range=(), packet=True):
    p = None
    if packet:
        p = {"q_actual": list(qa), "q_target": list(qt), "mode": mode, "error_status": error_status,
             "nonfinite": list(nonfinite), "out_of_range": list(out_of_range)}
    return {"packet": p, "t": time.monotonic() - age, "fault": fault}


@pytest.fixture
def judge(tmp_path):
    ctl = Controller(LOOPBACK, str(tmp_path / "j"))       # never started: no I/O
    return lambda s, ref=None: ctl._track_judge(s, ref, "test")


@pytest.mark.parametrize("bad, text", [
    (dict(fault="feedback gap"), "feedback fault"),
    (dict(packet=False), "no feedback packet"),
    (dict(age=control.FEEDBACK_MAX_AGE_S + 0.05), "stale"),
    (dict(error_status=1), "error_status"),
    (dict(nonfinite=["tcp_speed_actual"]), "non-finite"),          # another field; joints finite
    (dict(out_of_range=["speed_scaling"]), "out of range"),
    (dict(qa=(0.0, float("nan"), 0.0, 0.0)), "not finite"),         # the flag cleared
    (dict(qt=(0.0, 0.0, float("inf"), 0.0)), "not finite"),
    (dict(qa=(0.0, 0.0, 0.0)), "fewer than 4"),
    (dict(qt=(0.0, 0.0)), "fewer than 4"),
])
def test_an_invalid_copy_fails_closed(judge, bad, text):
    with pytest.raises(TrackingFault) as info:
        judge(snap(**bad))
    assert text in str(info.value)


def test_a_short_or_non_finite_reference_fails_closed(judge):
    with pytest.raises(TrackingFault):
        judge(snap(), ref=[0.0, 0.0, 0.0])
    with pytest.raises(TrackingFault):
        judge(snap(), ref=[0.0, float("nan"), 0.0, 0.0])


@pytest.mark.parametrize("joint", [1, 2, 3, 4])
@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_the_bound_is_inclusive_on_every_joint_both_signs(judge, joint, sign):
    at = [0.0] * 4
    at[joint - 1] = sign * 0.2                                 # literal: a widened bound fails here
    rec = judge(snap(qa=at))
    assert not rec["mismatch"] and rec["worst_joint"] == joint
    over = [0.0] * 4
    over[joint - 1] = sign * 0.200001
    rec = judge(snap(qa=over))
    assert rec["mismatch"] and rec["worst_joint"] == joint


def test_the_reference_alone_can_fail(judge):
    """q_actual equal to q_target, but the GetAngle value used is 0.5 deg off on J2"""
    rec = judge(snap(qa=(1.0, 2.0, 3.0, 4.0), qt=(1.0, 2.0, 3.0, 4.0)), ref=[1.0, 2.5, 3.0, 4.0])
    assert rec["mismatch"] and rec["worst_joint"] == 2 and max(map(abs, rec["d_t"])) == 0.0
    assert rec["d_r"][1] == pytest.approx(-0.5)


# ----------------------------------------------------------------------
# a: the Enable
# ----------------------------------------------------------------------
def test_a_joint_sunk_at_the_enable_fails_initialize_and_sends_no_move(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        fake.sag = {2: -2.0}                                   # jog_07: J2 2 deg below its target
        ctl = make(fake, tmp_path / "a")
        ctl.open().result(5)
        ctl.pose().result(5)
        ctl.take_control().result(5)
        with pytest.raises(TrackingFault) as info:
            ctl.enable().result(10)
        assert "enable: J2" in str(info.value)
        assert wait_for(lambda: ctl.stop_report is not None)
        assert ctl.stop_report["stop_state"] == "reset_disable"
        assert tracking(ctl)[-1]["what"] == "enable" and tracking(ctl)[-1]["outcome"] == "mismatch"
        summary = ctl.shutdown(10)
        assert "TrackingFault" in summary["failed_reason"]
        assert fake.motion_received == [] and fake.movl_received == []


def test_a_healthy_enable_records_its_check(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        rec = [r for r in tracking(ctl) if r["what"] == "enable"]
        assert len(rec) == 1 and rec[0]["outcome"] == "ok" and rec[0]["mode"] == 5
        assert rec[0]["received_monotonic"] > rec[0]["t_b"]
        assert rec[0]["ref_angles"] is not None and rec[0]["phase"] == "enabled"
        ctl.shutdown(10)


# ----------------------------------------------------------------------
# b: every enabled pose read
# ----------------------------------------------------------------------
def test_a_pose_read_with_a_sunk_joint_faults_and_sends_nothing_more(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        fake.sag = {3: 0.5}
        published(ctl.fb, sagged(3, 0.5))
        with pytest.raises(TrackingFault) as info:
            ctl.pose().result(5)
        assert "pose: J3" in str(info.value)
        assert wait_for(lambda: ctl.stop_report is not None)
        assert fake.movl_received == []
        ctl.shutdown(10)


def test_a_pose_read_whose_getangle_differs_from_the_measurement_faults(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        fake.getangle_offset = {1: 0.3}                        # q_target = q_actual; GetAngle off
        with pytest.raises(TrackingFault) as info:
            ctl.pose().result(5)
        assert "pose: J1" in str(info.value) and "GetAngle" in str(info.value)
        ctl.shutdown(10)


def test_a_stale_copy_at_the_judgement_faults(tmp_path):
    """The worker pauses after copying the packet and before the judgement; the reader goes on
    publishing newer packets, but the copy judged is the old one"""
    with FakeMG400(ik_model=True) as fake:
        once = []

        def copied(ctl, s):
            if ctl.run is not None and ctl.run.enable_completed and not once:
                once.append(1)
                time.sleep(control.FEEDBACK_MAX_AGE_S + 0.1)
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        ctl.hooks["track_copied"] = copied
        with pytest.raises(TrackingFault) as info:
            ctl.pose().result(5)
        assert "stale" in str(info.value)
        ctl.shutdown(10)


def test_a_stop_during_the_tracking_wait_ends_it_and_runs_one_stop_sequence(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        with ctl.fb.lock:                                      # no resting packet: the wait goes on
            ctl.fb.allowed_modes = {5, 7}
            ctl.fb.narrow = None
        fake.feedback_mode = 7
        published(ctl.fb, lambda p: p["robot_mode"] == 7)
        ctl.hooks["track_copied"] = lambda c, s: c.request_stop("operator")
        with pytest.raises(StopRequested):
            ctl.pose().result(5)
        assert wait_for(lambda: ctl.stop_report is not None)
        summary = ctl.shutdown(10)
        assert summary["stop_reason"] == "operator" and not summary["autonomous"]
        d = [c.split("(", 1)[0] for n, c in fake.received_on]
        assert d.count("ResetRobot") == 2                      # take_control's and the stop's


def test_no_resting_packet_within_the_wait_is_a_fault(tmp_path, monkeypatch):
    """A stream of fresh mode-7 packets never satisfies a resting check"""
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        with ctl.fb.lock:                                      # keep the reader's own mode check quiet
            ctl.fb.allowed_modes = {5, 7}
            ctl.fb.narrow = None
        fake.feedback_mode = 7
        published(ctl.fb, lambda p: p["robot_mode"] == 7)
        with pytest.raises(TrackingFault) as info:
            ctl.pose().result(5)
        assert "no resting feedback packet" in str(info.value) and "last RobotMode 7" in str(info.value)
        ctl.shutdown(10)


# ----------------------------------------------------------------------
# c: the pre-send gate
# ----------------------------------------------------------------------
def test_the_gate_before_movl_faults_and_no_movl_is_sent(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)

        def before_track(c):
            fake.sag = {2: -1.0}
            published(c.fb, sagged(2, -1.0))
        ctl.hooks["before_track"] = before_track
        with pytest.raises(TrackingFault) as info:
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(10)
        assert "before MovL: J2" in str(info.value)
        assert fake.movl_received == []
        rec = tracking(ctl)[-1]
        assert rec["what"] == "before MovL" and rec["phase"] == "enabled"   # mode 7 not yet allowed
        ctl.shutdown(10)


def test_consecutive_moves_each_pass_the_gate(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        p = ctl.pose().result(5)
        ctl.move_l([p[0], p[1], p[2] + 3.0, p[3]], 10, 3.0).result(20)
        ctl.move_l([p[0], p[1], p[2], p[3]], 10, 3.0).result(20)      # at once: within the grace
        whats = [r["what"] for r in tracking(ctl)]
        assert whats.count("before MovL") == 2 and whats.count("arrival") == 2
        assert all(r["outcome"] == "ok" for r in tracking(ctl))
        second = [r for r in tracking(ctl) if r["what"] == "before MovL"][1]
        assert second["t_b"] - second["phase_t"] < control.PHASE_GRACE_S
        assert second["received_monotonic"] > second["t_b"]
        ctl.shutdown(10)


def test_mode_7_at_the_gate_within_the_grace_sends_no_movl(tmp_path, monkeypatch):
    """Inside the phase grace the reader still allows mode 7: the gate itself refuses"""
    monkeypatch.setattr(control, "TRACK_WAIT_S", 0.1)          # the gate's budget ends inside the grace
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        p = ctl.pose().result(5)
        ctl.move_l([p[0], p[1], p[2] + 3.0, p[3]], 10, 3.0).result(20)

        def before_track(c):
            fake.feedback_mode = 7
            published(c.fb, lambda pkt: pkt["robot_mode"] == 7)
        ctl.hooks["before_track"] = before_track
        with pytest.raises(TrackingFault) as info:
            ctl.move_l([p[0], p[1], p[2], p[3]], 10, 3.0).result(20)
        assert str(info.value).startswith("before MovL: no resting feedback packet")
        rec = tracking(ctl)[-1]
        assert rec["what"] == "before MovL" and rec["outcome"] == "timeout"
        assert rec["t_b"] - rec["phase_t"] < control.PHASE_GRACE_S
        assert len(fake.movl_received) == 1
        ctl.shutdown(10)


def test_mode_7_at_the_gate_after_the_grace_sends_no_movl(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        p = ctl.pose().result(5)
        ctl.move_l([p[0], p[1], p[2] + 3.0, p[3]], 10, 3.0).result(20)

        def before_track(c):
            fake.feedback_mode = 7
            published(c.fb, lambda pkt: pkt["robot_mode"] == 7)
            time.sleep(control.PHASE_GRACE_S + 0.1)
        ctl.hooks["before_track"] = before_track
        with pytest.raises(Exception):
            ctl.move_l([p[0], p[1], p[2], p[3]], 10, 3.0).result(20)
        assert len(fake.movl_received) == 1
        ctl.shutdown(10)


def test_evidence_first_seen_after_the_wait_budget_never_counts(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "TRACK_WAIT_S", 0.2)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        once = []

        def copied(c, s):
            if not once:
                once.append(1)
                time.sleep(0.3)                                # past the budget, not yet stale
                s.update(c._track_copy())                      # fresh, eligible, first seen past the budget
                assert s["packet"]["mode"] == 5
        ctl.hooks["track_copied"] = copied
        with pytest.raises(TrackingFault) as info:
            ctl.pose().result(5)
        assert "no resting feedback packet" in str(info.value)
        assert tracking(ctl)[-1]["outcome"] == "timeout"
        ctl.shutdown(10)


class _StubRun:
    def __init__(self):
        self.report, self.phase, self.phase_t, self.checks = {"tracking": []}, "enabled", 0.0, 0

    def check(self):
        self.checks += 1


class _StubInts:
    def poll(self):
        pass


def _bare(tmp_path, t):
    """A never-started controller whose only copy was received at ``t`` (no I/O)"""
    ctl = Controller(LOOPBACK, str(tmp_path / "b"))
    ctl.run, ctl.ints = _StubRun(), _StubInts()
    ctl._track_copy = lambda: {"t": t, "fault": None,
                               "packet": {"q_actual": [0.0] * 4, "q_target": [0.0] * 4, "mode": 5,
                                          "error_status": 0, "nonfinite": [], "out_of_range": []}}
    return ctl


def test_a_packet_received_exactly_at_the_boundary_is_not_used(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "TRACK_WAIT_S", 0.05)
    t_b = time.monotonic()
    ctl = _bare(tmp_path, t_b)
    with pytest.raises(TrackingFault, match="no resting feedback packet"):
        ctl._track_wait(t_b, None, "test")
    assert ctl.run.report["tracking"][-1]["outcome"] == "timeout"
    ctl2 = _bare(tmp_path, t_b + 1e-6)
    assert ctl2._track_wait(t_b, None, "test")["outcome"] == "ok"


def test_every_failing_exit_of_the_wait_is_recorded(tmp_path, monkeypatch):
    t_b = time.monotonic()

    class Late(Exception):
        pass
    ctl = _bare(tmp_path, t_b - 1.0)                           # never eligible
    with pytest.raises(Late):
        ctl._track_wait(t_b, None, "arrival", deadline=time.monotonic() - 0.01, on_deadline=lambda: Late("late"))
    assert ctl.run.report["tracking"][-1]["outcome"] == "deadline"
    assert "late" in ctl.run.report["tracking"][-1]["error"]

    ctl = _bare(tmp_path, t_b - 1.0)

    def check():
        raise control.JogFault("feedback fault: gap")
    ctl.run.check = check
    with pytest.raises(control.JogFault):
        ctl._track_wait(t_b, None, "pose")
    rec = ctl.run.report["tracking"][-1]
    assert rec["outcome"] == "stopped" and "feedback fault" in rec["error"]

    ctl = _bare(tmp_path, t_b - 10.0)                          # stale: the copy is invalid
    with pytest.raises(TrackingFault, match="stale"):
        ctl._track_wait(t_b, None, "pose")
    assert ctl.run.report["tracking"][-1]["outcome"] == "invalid"


# ----------------------------------------------------------------------
# d: arrival, under the move's deadline
# ----------------------------------------------------------------------
def test_healthy_settling_longer_than_the_wait_is_accepted(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        fake.settle = (2, 0.6, 1.2)                            # above the bound for about 0.8 s
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        result = ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(30)
        assert result["tracking_deg"] <= control.TRACK_TOL_DEG
        arrival = [r for r in tracking(ctl) if r["what"] == "arrival"]
        assert arrival[0]["outcome"] == "mismatch" and arrival[-1]["outcome"] == "ok"
        assert not ctl.stopped
        ctl.shutdown(10)


def test_a_persistent_mismatch_at_arrival_ends_in_the_deadline_fault_with_it(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "MOVE_DEADLINE_S", 2.0)
    monkeypatch.setattr(control, "MOVE_SPEED_FLOOR_MM_S", 1000.0)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)

        def sag_when_moving():
            wait_for(lambda: fake.moves, 5)
            fake.sag = {2: -2.0}
        threading.Thread(target=sag_when_moving, daemon=True).start()
        with pytest.raises(JogFault) as info:
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(30)
        assert "did not arrive" in str(info.value) and "last tracking mismatch: arrival: J2" in str(info.value)
        assert wait_for(lambda: ctl.stop_report is not None)
        ctl.shutdown(10)


def test_the_deadline_inside_a_later_wait_keeps_the_last_mismatch(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "MOVE_DEADLINE_S", 2.0)
    monkeypatch.setattr(control, "MOVE_SPEED_FLOOR_MM_S", 1000.0)
    monkeypatch.setattr(control, "TRACK_WAIT_S", 30.0)        # the move deadline comes first
    with FakeMG400(ik_model=True) as fake:
        fake.settle = (1, 1.0, 100.0)                          # a mismatch at the first arrival check
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)

        def copied(c, s):
            # after the first mismatched arrival check, no packet is resting any more (mode 7 is
            # allowed while moving): the next wait runs into the move's deadline
            if any(r["what"] == "arrival" and r["mismatch"] for r in c.run.report["tracking"]):
                fake.feedback_mode = 7
        ctl.hooks["track_copied"] = copied
        with pytest.raises(JogFault) as info:
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(30)
        assert "did not arrive" in str(info.value) and "last tracking mismatch: arrival: J1" in str(info.value)
        ctl.shutdown(10)


def test_success_is_never_accepted_after_the_deadline(tmp_path, monkeypatch):
    """Matching evidence is found before the deadline; the worker pauses past it before the
    final acceptance: the move fails"""
    monkeypatch.setattr(control, "MOVE_DEADLINE_S", 1.5)
    monkeypatch.setattr(control, "MOVE_SPEED_FLOOR_MM_S", 1000.0)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        ctl.hooks["before_accept"] = lambda c: time.sleep(1.6)
        with pytest.raises(JogFault) as info:
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(30)
        assert "did not arrive" in str(info.value)
        ctl.shutdown(10)


# ----------------------------------------------------------------------
# e: the idle watch
# ----------------------------------------------------------------------
def test_the_idle_watch_stops_a_joint_that_sinks_between_requests(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.sag = {2: -2.0}
        published(ctl.fb, sagged(2, -2.0))
        t0 = time.monotonic()
        assert wait_for(lambda: ctl.stopped, 3.0)
        assert time.monotonic() - t0 < 0.4                    # one tick (0.1 s) and the check
        assert ctl.autonomous and "idle: J2" in ctl.stop_reason
        assert wait_for(lambda: ctl.stop_report is not None)
        assert ctl.stop_report["stop_state"] == "reset_disable"
        with pytest.raises(StopRequested):
            ctl.pose().result(5)
        assert fake.movl_received == []
        ctl.shutdown(10)


def test_an_idle_fault_with_a_request_already_dequeued_rejects_it(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        last = {"item": None, "calls": 0}
        orig_get = ctl._q.get

        def get(*a, **k):
            last["calls"] += 1
            try:
                item = orig_get(*a, **k)
            except queue.Empty:
                last["item"] = None
                raise
            last["item"] = item
            return item
        ctl._q.get = get
        fired = []

        def tick(c):
            if last["item"] is not None and last["item"][0] == "pose" and not fired:
                fired.append(1)
                fake.sag = {4: 1.0}
                published(c.fb, sagged(4, 1.0))
        ctl.hooks["tick"] = tick
        assert wait_for(lambda: last["calls"] > 0, 2.0)        # the worker now waits in the wrapped get
        with pytest.raises(StopRequested):
            ctl.pose().result(10)
        assert fired and ctl.autonomous and "idle: J4" in ctl.stop_reason
        summary = ctl.shutdown(10)
        d = [c.split("(", 1)[0] for n, c in fake.received_on]
        assert d.count("ResetRobot") == 2 and summary["stop_state"] == "reset_disable"


def test_a_throttled_idle_sample_still_updates_the_maxima(tmp_path, monkeypatch):
    monkeypatch.setattr(control, "TRACK_IDLE_RECORD_S", 1000.0)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        assert wait_for(lambda: any(r["what"] == "idle" for r in tracking(ctl)), 3.0)
        fake.sag = {2: 0.15}                                   # within the bound
        assert wait_for(lambda: ctl.run.report["tracking_max"].get("J2", {}).get("deg", 0) > 0.14, 3.0)
        assert ctl.run.report["tracking_max"]["J2"]["what"] == "idle"
        assert sum(r["what"] == "idle" for r in tracking(ctl)) == 1
        assert not ctl.stopped
        ctl.shutdown(10)


# ----------------------------------------------------------------------
# the wrapper's pose consumers
# ----------------------------------------------------------------------
BOX = dict(x_min=350.0, x_max=450.0, y_min=-50.0, y_max=50.0, z_min=170.0, z_max=270.0,
           r_min=350.0, r_max=450.0)
START = FakeMG400.fk([0.071847, 10.278645, 16.846972, 18.555908])


def robot(tmp_path, **opts):
    kw = dict(home_position=[400.0, 0.0, 220.0],
              joint_limits=[[-160, 160], [-25, 85], [-25, 105], [-180, 180]],
              speed_ratio=10, max_move_mm=50.0)
    kw.update(opts)
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**BOX),
                       record_folder=str(tmp_path / "mg400_5"), **kw)
    _ROBOTS.append(r)
    return r


@pytest.mark.parametrize("knob", ["sag", "getangle_offset"])
def test_move_z_start_pose_is_checked(tmp_path, knob):
    r = robot(tmp_path)

    async def body():
        await r.initialize()
        r._ctl._idle_track = lambda: None
        setattr(r.fake, knob, {2: 0.5})
        if knob == "sag":
            published(r._ctl.fb, sagged(2, 0.5))
        with pytest.raises(TrackingFault) as info:
            await r.move_z(5.0)
        assert str(info.value).startswith("pose:")
        await r.cleanup()
    asyncio.run(body())
    assert r.fake.movl_received == []


@pytest.mark.parametrize("knob", ["sag", "getangle_offset"])
def test_already_at_home_is_decided_on_a_checked_pose(tmp_path, knob):
    r = robot(tmp_path, home_position=[round(START[0], 3), round(START[1], 3), round(START[2], 3)])

    async def body():
        await r.initialize()
        r._ctl._idle_track = lambda: None
        setattr(r.fake, knob, {3: -0.5})
        if knob == "sag":
            published(r._ctl.fb, sagged(3, -0.5))
        with pytest.raises(TrackingFault) as info:
            await r.go_home()
        assert str(info.value).startswith("pose:")
        await r.cleanup()
    asyncio.run(body())
    assert r.fake.movl_received == []
    assert r.run_report()["failed_reason"]


def test_the_mock_flow_path_is_unchanged_without_sag(tmp_path):
    r = robot(tmp_path)

    async def body():
        await r.initialize()
        await r.move_z(5.0)
        await r.move_z(-5.0)
        await r.go_home()
        await r.cleanup()
    asyncio.run(body())
    report = r.run_report()
    assert not report.get("failed_reason")
    assert len(r.fake.movl_received) == 3


def test_arrival_still_needs_the_dashboards_robotmode_5(tmp_path, monkeypatch):
    """The feedback shows RobotMode 5 at the target, but the controller's Dashboard still answers
    7 (the move not finished): the resting packet alone is not an arrival"""
    monkeypatch.setattr(control, "MOVE_DEADLINE_S", 2.0)
    monkeypatch.setattr(control, "MOVE_SPEED_FLOOR_MM_S", 1000.0)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        fake.stuck_running = True
        fake.feedback_mode = 5
        with pytest.raises(JogFault) as info:
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(30)
        assert "did not arrive" in str(info.value)
        ctl.shutdown(10)


# ----------------------------------------------------------------------
# the real feedback reader's interleavings
# ----------------------------------------------------------------------
class _Gate:
    """A csv writer stand-in that blocks the reader thread on its next row while ``hold`` is set;
    with ``then`` given, the row after the released one blocks again until ``then`` is set"""

    def __init__(self, inner, then=None):
        self.inner, self.then = inner, then
        self.hold, self.release, self.blocked = threading.Event(), threading.Event(), threading.Event()
        self.rows = []

    def writerow(self, row):
        if self.hold.is_set() and not self.release.is_set():
            self.rows.append(row)
            self.blocked.set()
            self.release.wait(10)
            return self.inner.writerow(row)
        if self.then is not None and self.release.is_set() and len(self.rows) == 1:
            self.rows.append(row)
            self.then.wait(10)
        return self.inner.writerow(row)


def test_a_reader_blocked_in_its_file_writes_leaves_no_usable_packet(tmp_path):
    """No gap fault can be latched while the reader is blocked; the idle watch judges the copy's
    own age and stops the arm"""
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        gate = _Gate(ctl.fb._pw)
        ctl.fb._pw = gate
        gate.hold.set()
        try:
            assert gate.blocked.wait(2)
            assert wait_for(lambda: ctl.stopped, 2.0)
            with ctl.fb.lock:
                assert ctl.fb.fault is None                   # the reader itself never noticed
            assert ctl.autonomous and "stale" in ctl.stop_reason
        finally:
            gate.release.set()
        ctl.shutdown(10)


def test_a_packet_received_before_the_boundary_but_published_after_it_is_not_used(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        no_idle(ctl)
        then = threading.Event()                               # the next packet waits for this
        gate = _Gate(ctl.fb._tw, then)                         # written after the receive time is taken
        ctl.fb._tw = gate
        copies, seen = [], threading.Event()
        holder = {}

        def copied(c, s):
            copies.append((s["t"], gate.release.is_set()))
            if gate.release.is_set() and "t" in holder and abs(s["t"] - holder["t"]) < 1e-6:
                seen.set()                                     # the delayed packet, published, copied
        ctl.hooks["track_copied"] = copied
        gate.hold.set()
        assert gate.blocked.wait(2)
        held_t = holder["t"] = float(gate.rows[0][2])
        fut = ctl.pose()
        time.sleep(0.1)                                        # the check's boundary is later than held_t
        # while only packets received at or before the boundary exist, the check does not pass
        assert not fut.done() and copies and all(t <= held_t + 1e-6 for t, _ in copies)
        gate.release.set()                                     # the delayed packet is published now
        assert seen.wait(2)                                    # copied with its own receive time
        time.sleep(0.05)
        assert not fut.done()                                  # and not accepted
        then.set()                                             # fresh packets from here on
        fut.result(5)
        rec = [r for r in tracking(ctl) if r["what"] == "pose"][-1]
        assert rec["received_monotonic"] > rec["t_b"] >= held_t
        ctl.shutdown(10)


def test_an_error_packet_is_refused_before_the_reader_latches_its_fault(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        release = threading.Event()
        original = ctl.fb._fault

        def slow_fault(text):
            if "error_status" in text:
                release.wait(10)
            original(text)
        ctl.fb._fault = slow_fault
        fake.feedback_error_status = 1
        try:
            assert wait_for(lambda: ctl.stopped, 2.0)
            with ctl.fb.lock:
                assert ctl.fb.fault is None                   # still held back in the reader
            assert ctl.autonomous and "error_status" in ctl.stop_reason
        finally:
            fake.feedback_error_status = None
            release.set()
        ctl.shutdown(10)


def test_the_idle_watch_ignores_a_fresh_packet_that_is_not_resting(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        with ctl.fb.lock:                                      # keep the reader's own mode check quiet
            ctl.fb.allowed_modes = {5, 7}
            ctl.fb.narrow = None
        fake.feedback_mode = 7
        fake.sag = {2: -2.0}
        published(ctl.fb, lambda p: p["robot_mode"] == 7 and sagged(2, -2.0)(p))
        time.sleep(0.6)
        assert not ctl.stopped
        fake.feedback_mode = None
        fake.sag = {}
        published(ctl.fb, lambda p: p["robot_mode"] == 5 and sagged(2, 0.0)(p))
        ctl.shutdown(10)
