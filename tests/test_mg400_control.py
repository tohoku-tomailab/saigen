"""The MG400 control worker (``control.Controller``): one thread owns the I/O; the stop latch
is taken at the guards, on a reply and on every wake-up of the bounded queue wait; the stop
sequence has its own deadline. No hardware; the fake listens on 127.0.0.1 only.
"""
import threading
import time

import pytest

from src.devices.mg400 import control, protocol
from src.devices.mg400.control import Controller, StopRequested
from src.devices.mg400.fake import LOOPBACK, FakeMG400

HOME = [400.0, 0.0, 220.0]
_STARTED = []


@pytest.fixture(autouse=True)
def _shut_down_leftover_controllers():
    """A test that fails before its shutdown() must not leave the non-daemon worker alive
    (it would keep the pytest process from ending)"""
    yield
    while _STARTED:
        ctl = _STARTED.pop()
        if ctl._thread.is_alive():
            ctl.request_stop("test teardown")
            ctl.shutdown(30)


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
    """open, pose, take_control, enable: the arm enabled under the worker"""
    ctl = make(fake, folder, **kw)
    ctl.open().result(5)
    pose = ctl.pose().result(5)
    ctl.take_control().result(5)
    ctl.enable().result(10)
    return ctl, pose


def names(cmds):
    return [c.split("(", 1)[0] for c in cmds]


def dash_names(fake):
    return names([c for n, c in fake.received_on])


def wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def test_requests_run_in_order_on_one_thread_and_the_normal_end(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        seen = []
        ctl, pose = armed(fake, tmp_path / "a", hooks={"before_send": lambda run: seen.append(threading.current_thread().name)})
        r = ctl.move_l([pose[0], pose[1], pose[2] + 5.0, pose[3]], 10, 5.0).result(10)
        assert r["arrived_s"] > 0 and abs(r["pose"][2] - (pose[2] + 5.0)) <= control.ARRIVE_TOL_MM
        joints = ctl.inverse(HOME + [pose[3]]).result(5)
        assert len(joints) == 4
        summary = ctl.shutdown(10)
        assert set(seen) == {"mg400-control"}                     # every send from the worker
        assert summary["complete"] and summary["disabled_confirmed"] and not summary["stopped"]
        assert not summary["worker_alive"] and summary["failed_reason"] is None
        d = dash_names(fake)
        assert d.count("ResetRobot") == 1 and d.count("EnableRobot") == 1 and d.count("DisableRobot") == 1
        assert names(fake.motion_received) == ["MovL"]


def test_an_idle_worker_takes_the_latch_within_the_tick(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        t0 = time.monotonic()
        ctl.request_stop("test")                                    # nothing queued, nobody waiting
        assert wait_for(lambda: ctl.stopped, 2.0)
        taken = time.monotonic() - t0
        assert wait_for(lambda: ctl.stop_report is not None, 5.0)
        assert ctl.stop_report["stop_state"] == "reset_disable" and ctl.stop_report["disabled"] is True
        assert taken < 1.0, taken
        # the stop sequence's steps, from the latch: ResetRobot, RobotMode polls, DisableRobot,
        # confirmation polls; no EmergencyStop (the arm showed itself disabled)
        d = dash_names(fake)
        tail = d[d.index("ResetRobot", d.index("EnableRobot")):]
        assert tail[0] == "ResetRobot" and "DisableRobot" in tail and "EmergencyStop" not in tail
        assert set(tail[1:tail.index("DisableRobot")]) <= {"RobotMode"}
        with pytest.raises(StopRequested):
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(5)
        summary = ctl.shutdown(10)
        assert summary["stopped"] and summary["stop_reason"] == "test" and not summary["autonomous"]
        assert fake.motion_received == []


def test_a_feedback_fault_with_the_queue_empty_starts_the_stop_itself(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.drop_feedback()
        assert wait_for(lambda: ctl.stopped, 2.0)
        assert ctl.autonomous and "feedback" in ctl.stop_reason
        summary = ctl.shutdown(10)
        assert summary["stop_state"] == "reset_disable" and "feedback" in summary["failed_reason"]


def test_a_stop_set_right_after_the_guard_is_taken_when_the_reply_returns(tmp_path):
    """The latch is set after guard() passed for a MovL: the command is written (it is in the
    record and the fake received it), the stop is taken at the next poll, and the stop
    sequence runs"""
    with FakeMG400(ik_model=True) as fake:
        fake.move_s = 2.0
        box = {}

        def before_send(run):
            if run.phase == "moving" and "MovL" not in box:
                box["MovL"] = True
                ctl.request_stop("after the guard")
        ctl, pose = armed(fake, tmp_path / "a", hooks={"before_send": before_send})
        with pytest.raises(control.MG400Error):                      # StopRequested at the next guard
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(10)
        assert names(fake.motion_received) == ["MovL"]
        summary = ctl.shutdown(10)
        report_cmds = [c["command"] for c in ctl.rec.report["commands"]]
        assert any(c.startswith("MovL(") for c in report_cmds)
        assert summary["stopped"] and summary["stop_state"] == "reset_disable"
        assert fake.mode == 4


def test_a_stop_while_waiting_for_a_reply(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.hold_replies.set()                                      # the next RobotMode reply waits
        fut = ctl.pose()
        time.sleep(0.3)
        ctl.request_stop("while waiting")
        time.sleep(0.3)
        fake.hold_replies.clear()
        with pytest.raises(control.MG400Error):
            fut.result(10)
        summary = ctl.shutdown(10)
        assert summary["stopped"] and summary["stop_state"] == "reset_disable"


def test_request_stop_is_idempotent_and_concurrent_callers_get_one_sequence(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        results = {}

        def stopper(i):
            ctl.request_stop(f"caller {i}")

        def closer():
            results["summary"] = ctl.shutdown(15)
        threads = [threading.Thread(target=stopper, args=(i,)) for i in range(3)] + [threading.Thread(target=closer)]
        for th in threads:
            th.start()
        for th in threads:
            th.join(20)
        summary = results["summary"]
        assert summary["stop_reason"].startswith("caller ")
        d = dash_names(fake)
        assert d.count("ResetRobot") == 2 and d.count("DisableRobot") == 1       # step 5 + one sequence
        assert summary["stop_state"] == "reset_disable"


def test_the_stop_deadline_is_counted_in_the_worker(tmp_path, monkeypatch):
    """The arm answers nothing during the stop sequence and no caller waits: the sequence still
    ends by its own deadline (STOP_TOTAL_S) with 'unconfirmed'"""
    monkeypatch.setattr(control, "STOP_TOTAL_S", 2.0)
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.hold_replies.set()
        t0 = time.monotonic()
        ctl.request_stop("deadline")
        assert wait_for(lambda: ctl.stop_report is not None, 6.0)
        took = time.monotonic() - t0
        assert took < 2.0 + control.STOP_TIMEOUT_S + 1.0, took
        assert ctl.stop_report["stop_state"] == "unconfirmed" and ctl.stop_report["deadline_hit"]
        fake.hold_replies.clear()
        summary = ctl.shutdown(10)
        assert "unconfirmed" in summary["failed_reason"]


def test_a_soft_error_does_not_stop(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        with pytest.raises(protocol.MG400CommandError):
            ctl.inverse([900.0, 0.0, 220.0, pose[3]]).result(5)        # unreachable in the fake's model
        assert not ctl.stopped
        ctl.pose().result(5)                                          # the connection is still clean
        summary = ctl.shutdown(10)
        assert summary["complete"] and not summary["stopped"]


def test_a_fault_in_a_request_after_control_runs_the_stop_sequence(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.refuse_motion = True
        with pytest.raises(protocol.MG400CommandError):
            ctl.move_l([pose[0], pose[1], pose[2] + 5, pose[3]], 10, 5.0).result(10)
        assert ctl.stopped and ctl.autonomous
        summary = ctl.shutdown(10)
        assert summary["stop_state"] == "reset_disable"


def test_a_stop_before_control_sends_no_control_command(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl = make(fake, tmp_path / "a")
        ctl.open().result(5)
        ctl.pose().result(5)
        ctl.request_stop("before control")
        with pytest.raises(StopRequested):
            ctl.take_control().result(5)
        summary = ctl.shutdown(10)
        assert summary["stopped"] and summary["stop"] is None and not summary["armed"]
        assert not (set(names(fake.received)) & set(protocol.CONTROL))
        assert any("nothing to stop" in n for n in ctl.rec.report["notes"])


def test_requests_after_close_are_answered(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl = make(fake, tmp_path / "a")
        ctl.open().result(5)
        ctl.shutdown(10)
        with pytest.raises(StopRequested):
            ctl.pose().result(5)
        assert not ctl._thread.is_alive()


def test_shutdown_disables_when_control_was_taken(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        summary = ctl.shutdown(10)
        assert summary["disabled_confirmed"] and fake.mode == 4
        assert dash_names(fake)[-4:] == ["RobotMode", "GetErrorID", "GetAngle", "GetPose"]   # the final queries


def test_shutdown_decides_the_disable_on_the_worker_after_the_queued_requests(tmp_path):
    """take_control and enable are queued but not awaited when shutdown is called: the caller's
    view is 'not armed', the worker's decision (after those requests) is a DisableRobot"""
    with FakeMG400(ik_model=True) as fake:
        ctl = make(fake, tmp_path / "a")
        ctl.open().result(5)
        ctl.pose().result(5)
        ctl.take_control()
        ctl.enable()
        summary = ctl.shutdown(20)
        assert summary["disabled_confirmed"] and summary["complete"] and not summary["stopped"]
        assert fake.mode == 4
        d = dash_names(fake)
        assert d.count("EnableRobot") == 1 and d.count("DisableRobot") == 1 and "EmergencyStop" not in d


def test_a_stop_set_during_the_shutdowns_disable_is_taken(tmp_path):
    box = {}

    def before_send(run):
        if run.phase == "disabling" and "late" not in box:
            box["late"] = True
            ctl.request_stop("late")                                  # right before the DisableRobot goes out
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a", hooks={"before_send": before_send})
        summary = ctl.shutdown(20)
        assert summary["stopped"] and summary["stop_reason"] == "late" and not summary["autonomous"]
        assert summary["stop_state"] == "reset_disable" and fake.mode == 4


def test_cancelled_futures_are_tolerated_and_the_close_answers_the_rest(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.hold_replies.set()
        busy = ctl.pose()                                             # the worker waits for this reply
        f1 = ctl.pose()
        f2 = ctl.pose()
        f1.cancel()
        ctl.request_stop("test")
        fake.hold_replies.clear()
        with pytest.raises(control.MG400Error):
            busy.result(10)
        with pytest.raises(StopRequested):
            f2.result(10)
        assert f1.cancelled()
        summary = ctl.shutdown(10)
        assert not summary["worker_alive"] and summary["stop_state"] == "reset_disable"
        f3 = ctl.pose()
        with pytest.raises(StopRequested):
            f3.result(1)
        assert ctl.shutdown(5)["worker_alive"] is False               # idempotent after the close


def test_a_cancelled_shutdown_future_still_shuts_down_cleanly(tmp_path):
    """The owner of the shutdown request cancels its future while it waits in the queue: the
    worker still carries out the shutdown (DisableRobot, close, record) and the answer to the
    cancelled future is not an error of the worker"""
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        fake.hold_replies.set()
        busy = ctl.pose()                                             # the worker waits for this reply
        fut = ctl.submit("shutdown", lambda: None)
        assert fut.cancel()
        fake.hold_replies.clear()
        busy.result(10)
        assert ctl.wait_closed(15)
        summary = ctl.summary()
        assert ctl.loop_error is None and summary["failed_reason"] is None
        assert summary["disabled_confirmed"] and summary["complete"] and summary["record_saved"]
        assert "DisableRobot" in dash_names(fake) and fake.mode == 4


def test_a_failed_final_record_write_fails_the_report(tmp_path):
    """The arm ended disabled and confirmed, but the device record could not be written: the
    summary keeps the arm's state and still names the failure (the run is marked failed)"""
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")

        def refuse():
            raise control.MG400RecordError("disk refused the final save (test)")
        real_disable = ctl._disable

        def disable_then_refuse_saves():
            out = real_disable()                                      # its own saves go through
            ctl.run._sync_save = refuse                               # only the final save fails
            return out
        ctl._disable = disable_then_refuse_saves
        summary = ctl.shutdown(10)
        assert summary["disabled_confirmed"] and not summary["worker_alive"]
        assert summary["record_saved"] is False
        assert "could not be written" in summary["failed_reason"] and "test" in summary["failed_reason"]


def test_a_stop_set_after_the_shutdowns_disable_is_still_recorded(tmp_path):
    """A stop requested after the normal end's DisableRobot was confirmed (no send left for the
    guard to take it) is taken by the close: there is nothing to stop, and the summary says the
    run was stopped, with the reason, instead of reporting a plain normal end"""
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a")
        real_disable = ctl._disable

        def disable_then_stop():
            out = real_disable()
            ctl.request_stop("late, after the disable")
            return out
        ctl._disable = disable_then_stop
        summary = ctl.shutdown(20)
        assert summary["disabled_confirmed"] and summary["stopped"]
        assert summary["stop_reason"] == "late, after the disable" and summary["stop_state"] is None
        assert dash_names(fake).count("ResetRobot") == 1 and fake.mode == 4  # no stop sequence added


def test_a_fault_after_the_stop_adds_no_second_stop_sequence(tmp_path):
    """The stop sequence runs once: a fault the worker sees after it (here injected at the next
    wake-up) does not send ResetRobot / DisableRobot again"""
    box = {}

    def tick(c):
        if c.stopped and "fault" not in box:
            box["fault"] = True
            box["before"] = dash_names(fake).count("ResetRobot")
            c._fault(RuntimeError("a fault after the stop (test)"))
            box["after"] = dash_names(fake).count("ResetRobot")
    with FakeMG400(ik_model=True) as fake:
        ctl, pose = armed(fake, tmp_path / "a", hooks={"tick": tick})
        ctl.request_stop("first")
        assert wait_for(lambda: "after" in box, 20)
        assert box["before"] == box["after"] == 2              # take_control's ResetRobot and the stop's
        summary = ctl.shutdown(20)
        assert summary["stop_state"] == "reset_disable" and summary["stop_reason"] == "first"


def test_a_feedback_fault_before_control_is_still_an_autonomous_stop(tmp_path):
    """Before the arm is armed the joint-tracking watch does not run: the feedback fault itself
    must set the latch (an autonomous stop with nothing to stop)"""
    with FakeMG400(ik_model=True) as fake:
        ctl = make(fake, tmp_path / "a")
        ctl.open().result(5)
        ctl.pose().result(5)
        fake.drop_feedback()
        assert wait_for(lambda: ctl.stopped, 2.0)
        assert ctl.autonomous and "feedback" in ctl.stop_reason
        summary = ctl.shutdown(10)
        assert "feedback" in summary["failed_reason"]
        assert "EnableRobot" not in dash_names(fake)
