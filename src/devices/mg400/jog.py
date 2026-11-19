"""Jog a Dobot MG400: enable, one small J1 / J4 rotation and back, disable (human-run tool).

This is the first tool in this repository that changes the arm's state. It sends, on the
Dashboard port 29999, ``ResetRobot()`` (clear the planned queue), ``EnableRobot()`` and
``DisableRobot()`` (and, only in its stop sequence, ``ResetRobot()`` and
``EmergencyStop()``), and on the motion port 30003 exactly two
``JointMovJ(j1,j2,j3,j4,SpeedJ=s,AccJ=s)``: to the start joints plus an offset of 1..10 deg
on J1 and / or J4 (J2 and J3 never change: the swept volume is an arc at the arm's current
radius), and back to the start. The speed / acceleration ratio ``s`` is 1..30 % of the
controller's playback setting x its global ratio (guide 3.1): the tool bounds the ratio in
the command, not the speed itself, which depends on settings the tool does not read.

The arm is enabled only after the human pressed Enter on a terminal (``--mock`` skips the
prompt: the fake is in-process and no address can be typed in). Whatever goes wrong after
that point (an unexpected RobotMode, an alarm, a deadline, a lost connection, Ctrl+C)
starts the stop sequence: ``ResetRobot()``, ``DisableRobot()``, polls, and
``EmergencyStop()`` unless the arm has shown itself disabled (mode 4 held) or in alarm
and still, AND the Enable's outcome is known (never sent, refused, or RobotMode 5 read
after it). Once the arm has reported disabled again (the RobotMode 4 after the final
``DisableRobot()``), a later fault or Ctrl+C adds no stop sequence (only the final read-only
queries follow). Ctrl+C is therefore **not** an emergency stop:
it is an orderly stop bounded by 15 s that ignores a second Ctrl+C while it runs. The
emergency stop is the button on the controller; the human must have it within reach.

Usage (from the repository root)::

    python -m src.devices.mg400.jog --dry-run              # list the commands, connect to nothing
    python -m src.devices.mg400.jog --mock [--j1 5 --j4 0 --speed 10]
    python -m src.devices.mg400.jog <arm IP> --controller-version V [--j1 5 --j4 0 --speed 10]

Records go to ``logs/mg400_jog/<UTC time>/`` (gitignored), in the form of the observation
tool's records plus a ``jog`` and a ``stop`` section in ``report.json``.
"""
import argparse
import datetime
import os
import signal
import sys
import threading
import time
from typing import Callable, List, Optional

from . import protocol
from . import control
from .observe import (DASHBOARD_PORT, FEEDBACK_PORT, ONEDRIVE_NOTE, FeedbackRecorder, MG400RecordError,
                      ObservationEnded, Recorder, _exit, _record_also, _say, _utc)
from .protocol import Command, MG400CommandError, MG400ConnectionError, MG400Error, MG400ProtocolError
from .control import (  # noqa: F401 - the jog tool's names, re-exported for its tests and the harness
    MOTION_PORT, MAX_OFFSET_DEG, MIN_OFFSET_DEG, J1_LIMIT_DEG, J4_LIMIT_DEG, DEFAULT_SPEED,
    ARRIVE_TOL_DEG, DRIFT_LIMIT_DEG, STILL_DEG, ENABLE_DEADLINE_S, DISABLE_DEADLINE_S,
    MOVE_DEADLINE_S, MIN_LEG_S, SETTLE_S, DWELL_S, POLL_S, PHASE_GRACE_S, STOP_TIMEOUT_S,
    STOP_POLL_S, STOP_HOLD_S, STOP_HOLD_GRACE_S, STOP_STILL_SPAN_S, STOP_STILL_RECENT_S,
    STOP_ESTOP_POLL_S, STOP_NEW_CONNECTIONS, STOP_TOTAL_S, IO_DRAIN_S, PHASES, DASHBOARD_NAMES,
    MOTION_NAMES, STOP_NAMES, GUIDANCE, _NoInterrupts, JogFault, fixed_address, _connect_fixed,
    _open, BackgroundIO, confirm_tty, offsets_ok, target_joints, _max_diff, _still, JogRun,
    CLEANUP_ATTEMPTS, _Cleanup)

LOG_ROOT = os.path.join("logs", "mg400_jog")

PRECONDITIONS = """Before jogging (the arm WILL move: a rotation of J1 / J4 and back):
  - DobotStudio Pro is closed, and no other PC or program sends commands to the arm.
  - The arm is disabled (RobotMode 4) with no alarm; otherwise the tool stops before enabling.
  - Nothing is mounted on the flange. The whole arm and its cables swing through the arc:
    10 deg at a 300 mm radius is about 52 mm at the flange. Check that the arc is clear.
  - Note the controller's playback joint speed / acceleration and the global speed ratio
    (DobotStudio Pro) and pass them with --note: the tool bounds only the ratio in the
    command (SpeedJ = AccJ <= 30 %), not the resulting speed.
  - The emergency-stop button is within your reach. Ctrl+C is NOT an emergency stop: after
    the Enter it starts an orderly stop (ResetRobot, DisableRobot, then EmergencyStop unless
    the arm shows itself disabled, or in alarm and still, AND the Enable's outcome is known),
    bounded by 15 s; once the arm has reported disabled again (step 8) a later fault or Ctrl+C
    adds no stop sequence (only the final read-only queries follow)."""


class _Interrupts:
    """Ctrl+C in jog(), main thread (reviews 9 M3, 10 M1-M3): one handler for the whole run,
    whose effect depends on the phase.

    - ``prompt`` (until the armed point): raises KeyboardInterrupt at once, as Python's
      default handler does; nothing is armed, so nothing is lost.
    - ``run`` (armed): only sets ``requested``; the main thread raises KeyboardInterrupt
      itself at its next ``poll()`` (in guard(), before every send: within POLL_S normally,
      the send and reply timeouts of the command in progress, about 2 x TIMEOUT_S, at most),
      so the interrupt never lands inside a send, a record update, or the handlers that
      start the stop sequence, and no second command goes out in between. ``poll()`` raises
      once and moves the phase to ``stop``.
    - ``stop`` (the stop sequence and what follows it, up to the clean-up): further Ctrl+C
      are counted and ignored.
    - ``cleanup``: the first Ctrl+C is deferred (the run ends with KeyboardInterrupt once the
      record is written); the second ends the clean-up where it is (``escaped``: the record
      may be incomplete, the exception still carries the report). This is the way out of a
      clean-up stuck on a disk or a terminal.
    - ``done`` (the clean-up is over, the record written): only counts. jog() then restores
      the previous handler and ends a run that has no other error with KeyboardInterrupt
      when a Ctrl+C was seen but never raised (``unanswered``: review 12 M1, M3).

    Off the main thread nothing is installed and poll() never raises (Python delivers SIGINT
    to the main thread only). A SIGINT handler that was not set from Python (getsignal()
    None) is refused before anything opens: it could not be restored afterwards"""

    def __init__(self):
        self.phase = "prompt"
        self.count = 0                  # Ctrl+C seen after the armed point
        self.cleanup_count = 0
        self.requested = False
        self.raised = False
        self.escaped = False
        self.previous = None
        self.installed = False

    def install(self):
        if threading.current_thread() is not threading.main_thread():
            return
        previous = signal.getsignal(signal.SIGINT)
        if previous is None:            # not set from Python: it could not be restored (review 10 M3)
            raise ValueError("the SIGINT handler is not one set from Python: refusing to run")
        self.previous = previous
        self.installed = True           # before the call, so that restore() runs whatever lands in between (review 11 M4)
        signal.signal(signal.SIGINT, self.handler)

    def restore(self):
        while self.installed:           # a Ctrl+C landing in the restore itself retries it (review 11 M4)
            try:
                signal.signal(signal.SIGINT, self.previous)
                self.installed = False
            except KeyboardInterrupt:       # from the handler just restored: counted, the caller ends with 130
                self.count += 1

    def handler(self, signum, frame):
        if self.phase == "prompt":
            raise KeyboardInterrupt
        self.count += 1
        if self.phase == "run":
            self.requested = True
        elif self.phase == "cleanup":
            self.cleanup_count += 1
            if self.cleanup_count >= 2 and not self.escaped:
                self.escaped = True
                raise KeyboardInterrupt     # once: from here the handler only counts (review 13 M1)

    def poll(self):
        """The main thread, between its sends and polls: a Ctrl+C of the run phase takes
        effect here, once"""
        if self.phase == "run" and self.requested:
            self.phase = "stop"
            self.raised = True
            raise KeyboardInterrupt

    def unanswered(self) -> bool:
        """A Ctrl+C that was seen but never produced a KeyboardInterrupt (flagged after the
        last guard(), deferred in or after the clean-up, landed in the restore): jog() ends a
        run that has no other error with one, after the handler is restored"""
        return self.count > 0 and not self.raised and not self.escaped

    def summary(self) -> dict:
        return {"count": self.count, "cleanup_count": self.cleanup_count, "escaped": self.escaped,
                "raised_at_a_check": self.raised, "phase": self.phase}


def jog(host: str, folder: str, j1: float = 5.0, j4: float = 0.0, speed: int = DEFAULT_SPEED,
        controller_version: Optional[str] = None, note: Optional[str] = None, connect=_connect_fixed,
        dashboard_port: int = DASHBOARD_PORT, feedback_port: int = FEEDBACK_PORT,
        motion_port: int = MOTION_PORT, out=print, confirm: Callable[[str], bool] = confirm_tty,
        hooks: Optional[dict] = None) -> dict:
    """Run the jog. Returns the report (also written to ``folder``). Exceptions carry
    ``mg400_report`` (``report["armed"]`` says whether control commands may have been sent)"""
    offsets_ok(j1, j4)
    protocol.joint_mov_j([0, 0, 0, 0], speed)             # the speed ratio is checked before anything opens
    ints = _Interrupts()
    ctx: dict = {}
    error: Optional[BaseException] = None
    report: Optional[dict] = None
    try:
        ints.install()                  # before anything opens; an unknown SIGINT handler is refused
        report = _jog(host, folder, j1, j4, speed, controller_version, note, connect, dashboard_port,
                      feedback_port, motion_port, out, confirm, hooks, ints, ctx)
    except BaseException as e:  # noqa: BLE001 - raised again below, once the handler is restored
        error = e
    ints.restore()                      # from here a Ctrl+C is Python's own again (the record is written)
    rec = ctx.get("rec")
    if rec is not None and ints.count:
        rec.report["ctrl_c"] = ints.summary()   # in memory: the final save may predate the last one
        if error is None and ints.unanswered():
            error = KeyboardInterrupt()         # seen after the last guard(), in or after the clean-up
        if error is not None and not hasattr(error, "mg400_report"):
            error.mg400_report = rec.report     # type: ignore[attr-defined]
            error.mg400_recorder = rec          # type: ignore[attr-defined]
    if error is not None:
        raise error
    return report


def _jog(host: str, folder: str, j1: float, j4: float, speed: int, controller_version: Optional[str],
         note: Optional[str], connect, dashboard_port: int, feedback_port: int, motion_port: int, out,
         confirm: Callable[[str], bool], hooks: Optional[dict], ints: _Interrupts, ctx: dict) -> dict:
    """jog() proper, with the Ctrl+C handler installed; ``ctx["rec"]`` hands the recorder back"""
    rec = Recorder(folder)
    ctx["rec"] = rec
    rec.report.update(tool="src.devices.mg400.jog", controller_version_typed=controller_version,
                      note_typed=note, armed=False)
    fb: Optional[FeedbackRecorder] = None
    fb_conn: Optional[dict] = None
    run: Optional[JogRun] = None
    ended = None
    error: Optional[BaseException] = None

    # the clean-up (reviews 6 M2, 7 M2, 8 M1-M3), defined here where nothing is armed yet:
    # the finally below is a single call, and everything it does runs under the guards of
    # _Cleanup. The functions read the variables above as they are at the end of the run
    def notices():
        if ended:
            rec.note(ended)
            _say(ended, out)             # swallows a failing stdout: the save below comes first

    def background_errors():
        # a notice or save that failed on the I/O thread (its list read live: the thread may
        # have outlived the drain), a drain that did not finish, or a deferred notice that
        # could not be printed, is an I/O fault (J6): the run does not end with exit 0
        # (reviews 8 M3, 9 M2)
        errs: List[str] = []
        if run is not None:
            errs += list(run.io.errors)
            if errs:
                rec.report["io_thread_errors"] = list(errs)
            if rec.report.get("io_thread_drained") is False:
                errs.append(f"the I/O thread did not finish within {control.IO_DRAIN_S} s: its queue may "
                            "still hold saves or notices")
        errs += [n["error"] for n in rec.report.get("notice_errors") or []]
        if errs and cleanup.error is None:
            cleanup.error = JogFault("the run ended, but the background I/O reported errors: "
                                     + "; ".join(errs))

    def feedback_summary():
        fb.stop()
        fb.join(timeout=2.0)
        rec.report["feedback"] = fb.summary()
        if fb_conn is not None:
            with fb.lock:
                fault, end = fb.fault, fb.end or fb.closed
            fb_conn.update(end)
            if fault:
                fb_conn["fault"] = fault

    def final_save():
        if run is not None:
            rec.save = run._sync_save   # type: ignore[assignment]  # not left to finish() (review 8 M2)
        if ints.count:
            rec.report["ctrl_c"] = ints.summary()
        rec.report["ended_utc"] = control._utc()
        rec.save_during()

    def plan() -> List[Callable[[], None]]:
        steps: List[Callable[[], None]] = []
        if run is not None:
            steps.append(run.finish)
            steps.extend(d.close for d in run.conns)
        steps.append(notices)
        if fb is not None:
            steps.append(feedback_summary)
        steps.append(background_errors)
        steps.append(final_save)
        return steps

    def done():
        # the clean-up is over and the record written: from here the handler only counts, and
        # jog() turns a Ctrl+C that was never raised into KeyboardInterrupt once the handler is
        # restored (reviews 11 M5, 12 M1 / M3)
        ints.phase = "done"

    cleanup = _Cleanup(rec, plan, lambda: error, (hooks or {}).get("between_cleanup_steps"),
                       escape=lambda: ints.escaped, on_done=done)
    try:
        fb = FeedbackRecorder(_open(connect, host, feedback_port), folder)
        fb_conn = {"label": "feedback", "port": feedback_port, "opened_utc": control._utc(),
                   "opened_monotonic": round(time.monotonic(), 6)}
        rec.report.setdefault("connections", []).append(fb_conn)
        fb.start()
        fb.wait_first()
        run = JogRun(rec, fb, host, connect, dashboard_port, motion_port, out, hooks, ints)
        text = run.prepare(j1, j4, speed)
        out("state: disabled, no alarm")
        if not confirm(text):
            rec.note("not confirmed by the human: nothing was enabled")
            pass                        # no return here: the clean-up below decides the outcome (review 7 M3)
        else:
            try:
                run.arm()               # the armed point is its second statement (J4): from there, J5
                run.move(speed)
            except BaseException as e:
                if run.armed and not run.disabled_confirmed:    # nothing to stop before the armed point (review 11 M3)
                    stop = run.stop_sequence(f"{type(e).__name__}: {e}")
                    run.say("stop sequence: %s (last RobotMode %s). %s"
                            % (stop["stop_state"], stop["last_mode"], stop["guidance"]))
                raise
            finally:
                rec.report["disabled_confirmed"] = run.disabled_confirmed
            fb.stop()
            fb.join(timeout=2.0)
            with fb.lock:
                fault = fb.fault
            if fault:
                raise JogFault(fault)
            rec.report["complete"] = True
    except (JogFault, ObservationEnded) as e:
        ended = f"jog ended: {e}"
        error = e
    except BaseException as e:
        ended = f"stopped by {type(e).__name__}: {e}"
        error = e
    finally:
        ints.phase = "cleanup"          # a Ctrl+C from here is deferred; a second one ends the clean-up (J7)
        cleanup.run()                   # puts the report on the error it leaves behind, then done()
    if cleanup.error is not None:
        raise cleanup.error
    if rec.failed:
        raise MG400RecordError(rec.first_error, recorder=rec)
    return rec.report                   # a declined prompt too: the notes say so, nothing was enabled


def dry_run(out=print):
    out("feedback port %d: read only, never written" % FEEDBACK_PORT)
    out("limits: --j1, --j4 each 0 or %g..%g deg in size, not both 0 (J2, J3 never change); "
        "--speed 1..%d (default %d); start and target J1 within +-%g, J4 within +-%g"
        % (MIN_OFFSET_DEG, MAX_OFFSET_DEG, protocol.MAX_SPEED_RATIO, DEFAULT_SPEED, J1_LIMIT_DEG, J4_LIMIT_DEG))
    for line in ("RobotMode()   (end unless 4)", "GetErrorID()  (end on any alarm)",
                 "GetAngle() -> start joints; GetPose()", "PositiveSolution(target,0,0)  (compute only)",
                 "human Enter on a terminal (empty line); otherwise nothing more is sent",
                 "open the motion port %d" % MOTION_PORT,
                 "ResetRobot(); RobotMode() (must be 4); GetErrorID(); GetAngle() (start unchanged)",
                 "EnableRobot(); RobotMode() every %g s until 5 (deadline %g s); wait %g s; GetAngle()"
                 % (POLL_S, ENABLE_DEADLINE_S, SETTLE_S),
                 "JointMovJ(target,SpeedJ=s,AccJ=s) on the motion port; RobotMode() / GetAngle() "
                 "alternating every %g s until 5 then within %g deg (deadline %g s); dwell %g s"
                 % (POLL_S, ARRIVE_TOL_DEG, MOVE_DEADLINE_S, DWELL_S),
                 "JointMovJ(start,...) the same way",
                 "DisableRobot(); RobotMode() until 4 (deadline %g s)" % DISABLE_DEADLINE_S,
                 "RobotMode() GetErrorID() GetAngle() GetPose()",
                 "on any fault or Ctrl+C after the Enter (bounded by %g s): ResetRobot() DisableRobot(), "
                 "RobotMode() / GetAngle() polls; disabled = 4 held %g s; EmergencyStop() unless (disabled "
                 "or (RobotMode 9 and joints still within %g deg)) and the Enable's outcome is known "
                 "(never sent, refused, or RobotMode 5 read after it)" % (STOP_TOTAL_S, STOP_HOLD_S, STILL_DEG),
                 "once the arm has reported disabled again after the final DisableRobot(), a later fault or Ctrl+C "
                 "adds no stop sequence (only the final read-only queries follow)"):
        out("  " + line)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Jog a Dobot MG400: enable, a small J1 / J4 rotation "
                                             "and back, disable (human-run; the arm moves).")
    ap.add_argument("address", nargs="?", help="the arm's IP address (a numeric literal; not used with --mock)")
    ap.add_argument("--mock", action="store_true", help="fake controller on 127.0.0.1 only")
    ap.add_argument("--dry-run", action="store_true", help="list the commands; connect to nothing")
    ap.add_argument("--j1", type=float, default=5.0, help="J1 offset in deg (0 or 1..10 in size)")
    ap.add_argument("--j4", type=float, default=0.0, help="J4 offset in deg (0 or 1..10 in size)")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help="SpeedJ / AccJ ratio 1..%d" % protocol.MAX_SPEED_RATIO)
    ap.add_argument("--controller-version", default=None,
                    help="as shown in DobotStudio Pro (required with a real address)")
    ap.add_argument("--note", default=None,
                    help="recorded as typed: the controller's playback speed / acceleration and global ratio")
    ap.add_argument("--out", default=None, help="run folder (default logs/mg400_jog/<UTC time>)")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.dry_run:
        dry_run()
        return 0
    try:
        return _main(ap, args)
    except KeyboardInterrupt:  # in the CLI's own last statements, after jog() restored the handler
        _say("\nCtrl+C after the run had ended: check the arm, then the record (%s)."
             % (args.out or os.path.join(LOG_ROOT, "<UTC time>")))
        return 130


def _main(ap, args) -> int:
    """main() proper; a KeyboardInterrupt that escapes it is one that landed in its own
    statements (jog() had restored the handler), and main() answers it"""
    try:
        offsets_ok(args.j1, args.j4)
        protocol.joint_mov_j([0, 0, 0, 0], args.speed)
    except ValueError as e:
        ap.error(str(e))
    if not args.mock and not args.address:
        ap.error("give the arm's IP address, or --mock / --dry-run")
    if not args.mock and not args.controller_version:
        ap.error("--controller-version is required with a real address")
    address = None
    if not args.mock:
        try:
            address = fixed_address(args.address)
        except ValueError:
            ap.error("the address must be a numeric IP literal (no host names)")
    folder = args.out or os.path.join(
        LOG_ROOT, datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ("_mock" if args.mock else ""))
    # a record is never overwritten (2026-10-30: four bench records were lost to a reused
    # --out); checked before anything is opened, so nothing reaches the arm
    try:
        held = os.path.isdir(folder) and bool(os.listdir(folder))
    except OSError as e:
        _say("the record folder %s cannot be checked: %s" % (folder, e))
        return 2
    if held:
        _say("the record folder %s already holds files: give a new --out (a run never "
             "overwrites a record)" % folder)
        return 2
    print(PRECONDITIONS)
    if "onedrive" in os.path.abspath(folder).lower():
        print(ONEDRIVE_NOTE)
    fake = None
    report = None
    try:
        if args.mock:
            from .fake import LOOPBACK, FakeMG400
            fake = FakeMG400()
            ports = {DASHBOARD_PORT: fake.dashboard_port, FEEDBACK_PORT: fake.feedback_port,
                     MOTION_PORT: fake.motion_port}

            def connect(host, port, timeout):
                # never the address given on the command line, never a name lookup of it
                return _connect_fixed(LOOPBACK, ports[port], timeout)

            def confirm(text):
                print(text)
                print("--mock: the prompt is skipped (the fake controller is in this process)")
                return True
            report = jog(LOOPBACK, folder, args.j1, args.j4, args.speed, args.controller_version,
                         args.note, connect=connect, confirm=confirm)
        else:
            report = jog(address, folder, args.j1, args.j4, args.speed, args.controller_version, args.note)
    except KeyboardInterrupt as e:
        rep = getattr(e, "mg400_report", None) or {}
        stop = rep.get("stop") or {}
        if rep.get("cleanup_cut_short"):
            _say("\nA second Ctrl+C ended the clean-up: the record in %s may be incomplete." % folder)
        if stop:
            _say("\nCtrl+C after the arm was armed: the stop sequence ran: %s. %s Record in %s%s"
                 % (stop.get("stop_state"), stop.get("guidance", ""), folder, _record_also(e)))
        elif rep.get("disabled_confirmed"):
            _say("\nCtrl+C after the arm had already reported disabled; no stop sequence was needed. "
                 "Check the arm. Record in %s%s" % (folder, _record_also(e)))
        elif rep.get("armed"):
            _say("\nCtrl+C after the arm was armed, and no stop sequence ran: check the arm. "
                 "Record in %s%s" % (folder, _record_also(e)))
        elif rep:
            _say("\nCtrl+C before the arm was armed: no control command was sent (only the read-only "
                 "checks). Record in %s%s" % (folder, _record_also(e)))
        else:
            _say("\nCtrl+C before the run could report its state: check the arm, then the record "
                 "in %s" % folder)
        return 130
    except MG400RecordError as e:
        rec = e.recorder
        armed = bool(rec and rec.report.get("armed"))
        _say("record error: %s; %s (folder %s)%s"
             % (rec.first_error if rec else e, rec.describe() if rec else "", folder,
                "" if not armed else "; the arm was armed: check it"))
        return 1 if armed else 2
    except (MG400Error, OSError, ValueError) as e:
        rep = getattr(e, "mg400_report", None) or {}
        stop = rep.get("stop") or {}
        _say("%s: %s (record in %s)%s" % ("error" if rep.get("armed") else "ended before enabling",
                                          e, folder, _record_also(e)))
        if stop:
            _say("stop sequence: %s, last RobotMode %s. %s"
                 % (stop.get("stop_state"), stop.get("last_mode"), stop.get("guidance", "")))
        elif rep.get("disabled_confirmed"):
            _say("the arm had already reported disabled; check it anyway")
        elif rep.get("armed"):
            _say("no stop sequence ran; check the arm")
        return 1 if rep.get("armed") else 2
    finally:
        if fake is not None:
            fake.close()
    if not report.get("armed"):
        _say("not enabled; record: %s" % folder)
        return 2
    _say("record: %s (complete: %s)" % (folder, report.get("complete")))
    return 0 if report.get("complete") else 1


if __name__ == "__main__":
    _exit(main())
