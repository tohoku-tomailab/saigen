"""The MG400 control core shared by the jog tool and (stage two of the flow plan) the robot
wrapper: connections, the feedback gate, enable / disable with their polls, the stop sequence,
the background I/O thread, the record and the clean-up step list.

Extracted from ``jog.py`` on 2026-10-30 with the behaviour unchanged (flow plan F9, stage one):
the jog tests and the mutation harness are the regression. Nothing here installs a signal
handler: the owner of the run (the jog CLI, later the session) hands in the interrupt object
whose ``poll()`` is called before every send; by default a ``_NoInterrupts`` that never raises.
"""
import ipaddress
import os
import queue
import socket
import sys
import threading
import time
from typing import Callable, List, Optional

from . import protocol
from .observe import (DASHBOARD_PORT, FEEDBACK_PORT, TIMEOUT_S, Dashboard, FeedbackRecorder,
                      MG400RecordError, ObservationEnded, Recorder, _kind, _utc)
from .protocol import Command, MG400CommandError, MG400ConnectionError, MG400Error, MG400ProtocolError

MOTION_PORT = 30003

MAX_OFFSET_DEG = 10.0           # per joint, J1 and J4 only
MIN_OFFSET_DEG = 1.0            # a non-zero offset must be at least this: 5x the arrival tolerance,
#                                 so a GetAngle at the target cannot also be at the leg's origin
J1_LIMIT_DEG = 150.0            # datasheet +-160 with margin, start and target
J4_LIMIT_DEG = 170.0
DEFAULT_SPEED = 10              # % (protocol.MAX_SPEED_RATIO caps it at 30)
ARRIVE_TOL_DEG = 0.2            # every joint within this of the leg's target
DRIFT_LIMIT_DEG = 1.0           # J2 / J3 further than this from the start: fault
STILL_DEG = 0.05                # stop sequence: joints "still" if the GetAngle range is within this
ENABLE_DEADLINE_S = 10.0        # the guide gives no enabling time; an Enable not shown complete ends in EmergencyStop
DISABLE_DEADLINE_S = 5.0
MOVE_DEADLINE_S = 20.0
MIN_LEG_S = 0.3                 # no arrival before this after the JointMovJ reply
SETTLE_S = 0.5                  # after enabling, before the first move
DWELL_S = 1.0                   # after each arrival
POLL_S = 0.1
PHASE_GRACE_S = 0.3             # feedback packets of the previous phase's modes are allowed this long
STOP_TIMEOUT_S = 1.0            # socket timeouts in the stop sequence (per reply, whole reply)
STOP_POLL_S = 3.0               # confirmation polls after DisableRobot
STOP_HOLD_S = 1.0               # mode 4 must be read again this long after the first 4, with no other mode between
STOP_HOLD_GRACE_S = 1.0         # ... and shown by replies within hold + this, or the hold is unconfirmed
STOP_STILL_SPAN_S = 0.5         # alarm: GetAngle replies spanning at least this, all within STILL_DEG
STOP_STILL_RECENT_S = 0.6       # ... and the latest one this recent
STOP_ESTOP_POLL_S = 2.0         # RobotMode polls after EmergencyStop
STOP_NEW_CONNECTIONS = 2        # at most this many new Dashboard connections in the stop sequence
STOP_TOTAL_S = 15.0             # the whole stop sequence, monotonic
IO_DRAIN_S = 5.0

#: RobotMode values a reply or a packet may show in each phase; anything else is a fault
PHASES = {"disabled": {4}, "enabling": {4, 5}, "enabled": {5}, "moving": {5, 7},
          "disabling": {4, 5}, "stopping": set(range(1, 12))}
DASHBOARD_NAMES = frozenset(protocol.ALLOWED + ("ResetRobot", "EnableRobot", "DisableRobot"))
MOTION_NAMES = frozenset(protocol.MOTION)
STOP_NAMES = frozenset(protocol.STOP + ("RobotMode", "GetAngle", "GetErrorID"))

GUIDANCE = {
    "reset_disable": "the arm reported disabled (RobotMode 4 held). Check the arm.",
    "alarm_still": "the arm is in alarm (RobotMode 9) and the joint angles it reported while polled "
                   "stayed within the stop threshold. Check the arm, then clear the alarm in DobotStudio Pro.",
    "emergency_stop": "EmergencyStop() was sent and acknowledged: the arm should be powered off "
                      "and in alarm. If it still moves, press the emergency-stop button.",
    "unconfirmed": "the tool could NOT confirm a stop. Press the emergency-stop button unless the "
                   "arm is clearly at rest.",
}


class _NoInterrupts:
    """Stands in for the owner's interrupt object when none is given: never raises, counts
    nothing; the phases are still written so the stop sequence can mark its own"""
    phase = "prompt"
    requested = False
    count = 0

    def poll(self):
        pass


class JogFault(MG400Error):
    """The arm is not in the state this phase expects; the stop sequence runs"""


def fixed_address(text: str) -> str:
    """The arm's address as a numeric IP literal (no name lookup, ever); raises ValueError"""
    return str(ipaddress.ip_address(text.strip()))


def _connect_fixed(ip: str, port: int, timeout: float) -> socket.socket:
    """connect() to a numeric address with the timeout set first: no name lookup, and the
    connect itself is bounded"""
    family = socket.AF_INET6 if ":" in ip else socket.AF_INET
    s = socket.socket(family, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
    except OSError:
        s.close()
        raise
    return s


def _open(connect, host: str, port: int, timeout: float = TIMEOUT_S) -> socket.socket:
    try:
        return connect(host, port, timeout)
    except MG400Error:
        raise
    except OSError as e:
        raise MG400ConnectionError(f"connecting to port {port}: {e}", label=f"port {port}",
                                   op="connect", kind=_kind(e)) from e


class BackgroundIO(threading.Thread):
    """Runs saves and notices after the armed point, so a stuck disk or a blocked stdout
    cannot keep the main thread from sending the stop commands. The queue is unbounded:
    ``submit`` never waits"""

    def __init__(self):
        super().__init__(name="mg400-jog-io", daemon=True)
        self.q: "queue.Queue" = queue.Queue()
        self.errors: List[str] = []

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            try:
                item()
            except BaseException as e:  # noqa: BLE001 - kept for the main thread's next check
                self.errors.append(repr(e))

    def submit(self, fn: Callable[[], None]):
        self.q.put(fn)

    def stop(self, timeout: float) -> bool:
        """Drain and end; True if everything queued before this call was run"""
        self.q.put(None)
        self.join(timeout)
        return not self.is_alive()


def confirm_tty(text: str, stdin=None, out=print) -> bool:
    """The human's Enter: standard input must be a terminal and the line empty. Anything
    else (EOF, other text, a pipe, no stdin) means no"""
    stdin = sys.stdin if stdin is None else stdin
    out(text)
    try:
        tty = stdin is not None and stdin.isatty()
    except (AttributeError, ValueError):
        tty = False
    if not tty:
        out("standard input is not a terminal: the arm is not enabled")
        return False
    out("Press Enter (an empty line) to enable and move the arm; anything else ends the run: ")
    try:
        line = stdin.readline()
    except (EOFError, OSError, ValueError):
        return False
    return line in ("\n", "\r\n")


def offsets_ok(j1: float, j4: float):
    for name, v in (("--j1", j1), ("--j4", j4)):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or abs(v) > MAX_OFFSET_DEG:
            raise ValueError(f"{name} must be a number within +-{MAX_OFFSET_DEG:g} deg: {v!r}")
        if v != 0 and abs(v) < MIN_OFFSET_DEG:
            raise ValueError(f"{name} must be 0 or at least {MIN_OFFSET_DEG:g} deg in size "
                             f"(the arrival tolerance is {ARRIVE_TOL_DEG:g} deg): {v!r}")
    if j1 == 0 and j4 == 0:
        raise ValueError("--j1 and --j4 are both 0: nothing to move")


def target_joints(start: List[float], j1: float, j4: float) -> List[float]:
    """start + (j1, 0, 0, j4), both inside the limits; raises ValueError"""
    offsets_ok(j1, j4)
    if len(start) < 4:
        raise ValueError(f"GetAngle gave {len(start)} values")
    target = [start[0] + j1, start[1], start[2], start[3] + j4]
    for label, joints in (("start", start), ("target", target)):
        if abs(joints[0]) > J1_LIMIT_DEG:
            raise ValueError(f"{label} J1 {joints[0]:.3f} outside +-{J1_LIMIT_DEG:g} deg")
        if abs(joints[3]) > J4_LIMIT_DEG:
            raise ValueError(f"{label} J4 {joints[3]:.3f} outside +-{J4_LIMIT_DEG:g} deg")
    return target


def _max_diff(a: List[float], b: List[float]) -> float:
    return max(abs(x - y) for x, y in zip(a[:4], b[:4]))


def _still(samples: List[tuple], now: float) -> bool:
    """``samples`` = [(monotonic, angles)]: at least three, spanning STOP_STILL_SPAN_S, the
    latest within STOP_STILL_RECENT_S of ``now``, every joint's range within STILL_DEG"""
    recent = [s for s in samples if now - s[0] <= STOP_STILL_SPAN_S + STOP_STILL_RECENT_S]
    if len(recent) < 3 or now - recent[-1][0] > STOP_STILL_RECENT_S:
        return False
    if recent[-1][0] - recent[0][0] < STOP_STILL_SPAN_S:
        return False
    for j in range(4):
        values = [s[1][j] for s in recent]
        if max(values) - min(values) > STILL_DEG:
            return False
    return True


class JogRun:
    """The state of one run: connections, phase, record; the sequence and the stop sequence"""

    def __init__(self, rec: Recorder, fb: FeedbackRecorder, host: str, connect, dashboard_port: int,
                 motion_port: int, out, hooks: Optional[dict] = None, ints=None):
        self.rec, self.fb, self.host, self.connect = rec, fb, host, connect
        self.ints = ints or _NoInterrupts()  # the owner's handler, or one that never raises
        self.dashboard_port, self.motion_port = dashboard_port, motion_port
        self._out = out
        self.hooks = hooks or {}
        self.io = BackgroundIO()
        self.io_started = False          # the thread runs saves and notices after the armed point
        self.io_deferred = False         # it could not start: saves and notices wait for the end of the run
        self._deferred_out: List[str] = []
        self._sync_save = rec.save
        self._async_save = lambda: self.io.submit(self._sync_save)   # queued; the thread writes
        self.dash: Optional[Dashboard] = None
        self.motion: Optional[Dashboard] = None
        self.conns: List[Dashboard] = []
        self.phase = "disabled"
        self.phase_t = time.monotonic()
        self.enable_result = "not_sent"  # not_sent / acked / refused; unresolved (sent, no reply read)
                                         # is derived from the record by the stop sequence
        self.enable_completed = False    # RobotMode 5 was read after the Enable
        self.disabled_confirmed = False  # step 9's RobotMode 4 was read
        self.report = {"phases": [], "legs": []}
        rec.report["jog"] = self.report
        rec.report["armed"] = False

    @property
    def armed(self) -> bool:
        """The human's Enter was given and arm() reached its armed point: control commands may
        have followed. One store, in the record (review 11 M3)"""
        return bool(self.rec.report.get("armed"))

    # -- helpers ---------------------------------------------------------------
    def say(self, text: str):
        if self.armed and self.io_started:
            self.io.submit(lambda: self._out(text))
        elif self.armed:
            self._deferred_out.append(text)      # no thread: printed by finish(), never between stop commands
        else:
            self._out(text)

    def set_phase(self, name: str):
        """Replies are judged by the new phase at once; packets may still show the previous
        phase's modes for PHASE_GRACE_S (the feed lags the Dashboard). The receiving thread
        itself applies the narrower set from ``phase_t + PHASE_GRACE_S`` on (``fb.narrow``)"""
        previous = self.phase
        self.phase, self.phase_t = name, time.monotonic()
        with self.fb.lock:
            self.fb.allowed_modes = set(PHASES[previous]) | set(PHASES[name])
            self.fb.narrow = (self.phase_t + PHASE_GRACE_S, set(PHASES[name]))
        self.report["phases"].append({"phase": name, "utc": _utc(),
                                      "monotonic": round(self.phase_t, 6)})

    def judge(self, mode: int):
        if mode not in PHASES[self.phase]:
            raise JogFault(f"RobotMode {mode} ({protocol.ROBOT_MODES.get(mode, '?')}) during {self.phase}")

    def check(self):
        """Everything that must hold between commands: the feed, the record, the I/O thread (a
        Ctrl+C of the run phase is taken by guard(), before every send: J6)"""
        hook = self.hooks.get("poll")
        if hook is not None:
            hook(self)
        self.fb.check()
        if self.rec.failed:
            raise JogFault(f"record save failed: {self.rec.first_error}")
        if self.io.errors:
            raise JogFault(f"background I/O failed: {self.io.errors[0]}")

    def guard(self):
        """Before every send on dashboard-1 / motion: a Ctrl+C of the run phase is taken here,
        never where it landed (J6), so no two commands go out between it and the stop"""
        self.ints.poll()
        self.fb.check()
        hook = self.hooks.get("before_send")
        if hook is not None:
            hook(self)

    def mode(self) -> int:
        return protocol.parse_robot_mode(self.dash.query(protocol.robot_mode()).values)

    def angles(self) -> List[float]:
        return protocol.parse_numbers(self.dash.query(protocol.get_angle()).values, (4, 6))[:4]

    def wait_mode(self, target: int, deadline_s: float, what: str) -> float:
        """Polls RobotMode until ``target``; the deadline is checked before every poll and
        again before a reply is accepted, so a late reply never counts"""
        t0 = time.monotonic()
        while True:
            self.check()
            if time.monotonic() - t0 > deadline_s:
                raise JogFault(f"{what}: RobotMode {target} not reported within {deadline_s:g} s")
            mode = self.mode()
            if time.monotonic() - t0 > deadline_s:
                raise JogFault(f"{what}: RobotMode {target} not reported within {deadline_s:g} s")
            if mode == target:
                return round(time.monotonic() - t0, 4)
            time.sleep(POLL_S)

    def wait_arrival(self, target: List[float], start: List[float], t_reply: float) -> dict:
        """Polls alternating RobotMode / GetAngle; arrived when a RobotMode 5 and a following
        GetAngle within ARRIVE_TOL_DEG of the target were read, no 7 between them, and at
        least MIN_LEG_S after the JointMovJ **reply** (``t_reply``, monotonic). The target is
        at least MIN_OFFSET_DEG from the leg's origin, so such a GetAngle also shows that the
        arm left the origin. The deadline MOVE_DEADLINE_S (from the reply) is checked before
        every poll and again before an arrival is accepted"""
        idle_seen = False
        polls = 0
        while True:
            self.check()
            if time.monotonic() - t_reply > MOVE_DEADLINE_S:
                raise JogFault(f"the arm did not arrive within {MOVE_DEADLINE_S:g} s")
            if polls % 2 == 0:
                idle_seen = self.mode() == 5
            else:
                angles = self.angles()
                if max(abs(angles[1] - start[1]), abs(angles[2] - start[2])) > DRIFT_LIMIT_DEG:
                    raise JogFault(f"J2 / J3 drifted from the start: {angles[1:3]} vs {start[1:3]}")
                now = time.monotonic()
                if now - t_reply > MOVE_DEADLINE_S:
                    raise JogFault(f"the arm did not arrive within {MOVE_DEADLINE_S:g} s")
                if idle_seen and now - t_reply >= MIN_LEG_S and _max_diff(angles, target) <= ARRIVE_TOL_DEG:
                    return {"arrived_s": round(now - t_reply, 4), "angles": angles, "polls": polls + 1}
            polls += 1
            time.sleep(POLL_S)

    def dwell(self, seconds: float, start: List[float]):
        end = time.monotonic() + seconds
        polls = 0
        while time.monotonic() < end:
            self.check()
            if polls % 2 == 0:
                self.mode()
            else:
                angles = self.angles()
                if max(abs(angles[1] - start[1]), abs(angles[2] - start[2])) > DRIFT_LIMIT_DEG:
                    raise JogFault(f"J2 / J3 drifted from the start: {angles[1:3]} vs {start[1:3]}")
            polls += 1
            time.sleep(min(POLL_S, max(0.0, end - time.monotonic())))
        self.check()

    # -- the sequence ------------------------------------------------------------
    def prepare(self, j1: float, j4: float, speed: int) -> str:
        """Steps 1 and 2: state, start joints, target; returns the text for the prompt"""
        self.dash = Dashboard(_open(self.connect, self.host, self.dashboard_port), self.rec,
                              "dashboard-1", self.guard, port=self.dashboard_port)
        self.dash.names = DASHBOARD_NAMES
        self.conns.append(self.dash)
        self.fb.check()
        self.dash.query(protocol.robot_mode())          # default judge: end unless 4
        self.dash.query(protocol.get_error_id())        # end on any alarm
        self.fb.check()
        start = self.angles()
        pose = protocol.parse_numbers(self.dash.query(protocol.get_pose()).values, 4)
        target = target_joints(start, j1, j4)
        fk = protocol.parse_numbers(
            self.dash.query(protocol.positive_solution(target)).values, 4)
        self.report.update(start_joints=start, start_pose=pose, target_joints=target,
                           target_positive_solution=fk, offsets={"j1": j1, "j4": j4},
                           speed_ratio=speed, limits={"max_offset_deg": MAX_OFFSET_DEG,
                                                      "min_offset_deg": MIN_OFFSET_DEG,
                                                      "arrive_tol_deg": ARRIVE_TOL_DEG,
                                                      "drift_limit_deg": DRIFT_LIMIT_DEG})
        self.rec.save()
        return ("plan: start joints %s\n      target joints %s (J1 %+g, J4 %+g deg), then back\n"
                "      JointMovJ at SpeedJ=AccJ=%d %% of the controller's playback setting x global "
                "ratio (not read by this tool)\n"
                "      deadlines: enable %g s, move %g s per leg, disable %g s\n"
                "      after the Enter: ResetRobot (clear the queue), re-check, EnableRobot, the two "
                "moves, DisableRobot\n"
                "      on any fault or Ctrl+C after the Enter: ResetRobot, DisableRobot, polls, then "
                "EmergencyStop unless the arm showed itself disabled (or in alarm with its joints "
                "still) AND the Enable's outcome is known (never sent, refused, or RobotMode 5 read);\n"
                "      once the arm has reported disabled again after the final DisableRobot, a later fault or "
                "Ctrl+C adds no stop sequence (only the final read-only queries follow)"
                % ([round(v, 3) for v in start], [round(v, 3) for v in target], j1, j4, speed,
                   ENABLE_DEADLINE_S, MOVE_DEADLINE_S, DISABLE_DEADLINE_S))

    def arm(self):
        """Step 4: the armed point. From here the stop sequence runs on any exception"""
        self.ints.phase = "run"          # first: from here a Ctrl+C is only flagged, taken at the next guard()
        self.rec.report["armed"] = True  # the armed point proper (armed reads this): from here J5 runs on any exception
        self.report["armed_utc"] = _utc()
        # saves are queued from here (a queue nobody runs yet loses nothing: every save
        # writes the same report, and the final save is synchronous). The stop sequence
        # fixes the save method itself when it starts (queued if the thread runs, no-op
        # otherwise: reviews 6 M1, 7 M1), so wherever in arm() start() fails or a Ctrl+C
        # lands (review 5 M2), no synchronous save sits between the stop commands; nothing
        # is enabled, notices wait for the end of the run and the stop sequence runs
        self.rec.save = self._async_save  # type: ignore[assignment]
        try:
            self.io.start()
            self.io_started = True
        except BaseException as e:          # any failure, Ctrl+C included
            self.io_deferred = True
            state = "start interrupted, thread running" if self.io.is_alive() else "not started"
            self.report["io_thread"] = (f"{state}: {e!r}; nothing enabled, saves and notices "
                                        "deferred to the end of the run")
            if isinstance(e, Exception):
                raise JogFault(f"the background I/O thread could not start ({e!r}): nothing enabled")
            raise
        self.dash.control = True
        self.dash.judge_mode = self.judge
        self.motion = Dashboard(_open(self.connect, self.host, self.motion_port), self.rec, "motion",
                                self.guard, port=self.motion_port, control=True)
        self.motion.names = MOTION_NAMES
        self.motion.judge_mode = self.judge
        self.conns.append(self.motion)
        hook = self.hooks.get("after_arm")
        if hook is not None:
            hook(self)

    def move(self, speed: int):
        """Steps 5 to 9"""
        start = self.report["start_joints"]
        target = self.report["target_joints"]
        # 5. clear whatever queue another client may have left, then re-check everything
        self.set_phase("disabled")
        self.check()
        self.dash.query(protocol.reset_robot())
        if self.mode() != 4:
            raise JogFault("not disabled after ResetRobot")
        self.dash.query(protocol.get_error_id())
        after_reset = self.angles()
        self.report["angles_after_reset"] = after_reset
        if _max_diff(after_reset, start) > ARRIVE_TOL_DEG:
            raise JogFault(f"the start joints changed since the plan was shown: {after_reset} vs {start}")
        # 6. enable. enable_result stays not_sent until the reply is read (acked / refused);
        # whether the bytes left is the record's word (sent_utc on the command entry, stamped
        # before the sendall), which the stop sequence reads itself (_enable_sent): a failure
        # or Ctrl+C anywhere from the guard to the read then counts as unresolved only if the
        # record says so (review 4 M4, review 5 M1)
        self.set_phase("enabling")
        self.check()
        cmd = protocol.enable_robot()
        entry = self.dash.write(cmd)
        try:
            self.dash.read(cmd, entry)
        except MG400CommandError:
            self.enable_result = "refused"
            raise
        self.enable_result = "acked"
        self.report["enable_s"] = self.wait_mode(5, ENABLE_DEADLINE_S, "enabling")
        self.enable_completed = True
        self.set_phase("enabled")
        self.say("enabled (RobotMode 5 after %.2f s)" % self.report["enable_s"])
        self.dwell(SETTLE_S, start)
        after_enable = self.angles()
        self.report["angles_after_enable"] = after_enable
        if _max_diff(after_enable, start) > ARRIVE_TOL_DEG:
            raise JogFault(f"enabling moved the arm: {after_enable} vs start {start}")
        # 7. the two legs
        for leg, goal in (("out", target), ("back", start)):
            self.set_phase("moving")
            self.check()
            cmd = protocol.joint_mov_j(goal, speed)
            entry = self.motion.write(cmd)
            self.motion.read(cmd, entry)                   # ErrorID != 0 raises MG400CommandError
            t_reply = entry["sent_monotonic"] + entry["reply_s"]
            result = self.wait_arrival(goal, start, t_reply)
            self.set_phase("enabled")
            result.update(leg=leg, command_id=entry["id"], target=goal,
                          reply_s=entry["reply_s"])
            self.report["legs"].append(result)
            self.say("leg %s: arrived after %.2f s, joints %s" % (leg, result["arrived_s"],
                                                                  [round(v, 3) for v in result["angles"]]))
            self.dwell(DWELL_S, start)
        # 8. disable
        self.set_phase("disabling")
        self.check()
        self.dash.query(protocol.disable_robot())
        self.report["disable_s"] = self.wait_mode(4, DISABLE_DEADLINE_S, "disabling")
        self.disabled_confirmed = True
        # 9. final state
        self.set_phase("disabled")
        self.say("disabled (RobotMode 4 after %.2f s)" % self.report["disable_s"])
        self.dash.query(protocol.robot_mode())
        self.dash.query(protocol.get_error_id())
        final = self.angles()
        self.report["final_joints"] = final
        self.report["final_pose"] = protocol.parse_numbers(self.dash.query(protocol.get_pose()).values, 4)
        self.report["final_minus_start_joints"] = [round(a - b, 6) for a, b in zip(final, start)]

    # -- the stop sequence --------------------------------------------------------
    def _stop_conn(self, dash: Dashboard) -> Dashboard:
        dash.control = True
        dash.strict_record = False
        dash.guard = None
        dash.judge_mode = lambda mode: None
        dash.names = STOP_NAMES
        dash.set_timeout(STOP_TIMEOUT_S)
        return dash

    def _new_stop_conn(self, stop: dict) -> Optional[Dashboard]:
        if stop["new_connections"] >= STOP_NEW_CONNECTIONS:
            return None
        if stop["deadline"] - time.monotonic() <= 0:
            stop["steps"].append({"connect": "skipped", "outcome": "stop deadline", "utc": _utc()})
            return None
        stop["new_connections"] += 1
        label = f"dashboard-stop-{stop['new_connections']}"
        try:
            sock = self.connect(self.host, self.dashboard_port,
                                min(STOP_TIMEOUT_S, stop["deadline"] - time.monotonic()))
        except (OSError, MG400Error) as e:
            stop["steps"].append({"connect": label, "outcome": f"failed: {e!r}", "utc": _utc()})
            return None
        dash = Dashboard(sock, self.rec, label, None, port=self.dashboard_port, control=True,
                         strict_record=False)
        self.conns.append(dash)
        return self._stop_conn(dash)

    def _stop_send(self, stop: dict, command: Command):
        """One command on the current stop connection; on a connection or protocol failure
        once more on a new connection. A refused command (ErrorID != 0) is recorded and the
        sequence goes on. Returns the Reply, or None"""
        for attempt in range(2):
            if stop["deadline"] - time.monotonic() <= 0:
                stop["steps"].append({"command": command.text, "outcome": "not sent: stop deadline",
                                      "utc": _utc()})
                return None
            dash = self._sconn
            if dash is None:
                dash = self._sconn = self._new_stop_conn(stop)
                if dash is None:
                    stop["steps"].append({"command": command.text, "outcome": "not sent: no connection",
                                          "utc": _utc()})
                    return None
            remaining = stop["deadline"] - time.monotonic()
            if remaining < 0.05:
                stop["steps"].append({"command": command.text, "outcome": "not sent: stop deadline",
                                      "utc": _utc()})
                return None
            dash.set_timeout(min(STOP_TIMEOUT_S, remaining))     # bounds the send
            step = {"command": command.text, "connection": dash.label, "utc": _utc()}
            stop["steps"].append(step)
            try:
                reply = dash.query(command, deadline=stop["deadline"])   # bounds the whole reply too
                if time.monotonic() > stop["deadline"]:
                    # read and checked, but past the deadline: not adopted as a success (review 4 M3)
                    step["outcome"] = "reply after the stop deadline"
                    return None
                step["outcome"] = "ok"
                step["values"] = reply.values
                return reply
            except MG400CommandError as e:
                step["outcome"] = f"refused: {e}"
                return None
            except (MG400ConnectionError, MG400ProtocolError) as e:
                step["outcome"] = f"connection failed: {e}"
                self._sconn = None                       # dirty or gone: never used again
                dash.close()
                if attempt == 1:
                    return None
            except BaseException as e:  # noqa: BLE001 - the sequence goes on
                step["outcome"] = f"error: {e!r}"
                self._sconn = None
                dash.close()
                return None

    def _stop_mode(self, stop: dict) -> Optional[int]:
        reply = self._stop_send(stop, protocol.robot_mode())
        if reply is None:
            return None
        try:
            mode = protocol.parse_robot_mode(reply.values)
        except MG400ProtocolError:
            return None
        stop["modes"].append((round(time.monotonic(), 4), mode))
        stop["last_mode"] = mode
        return mode

    def _stop_angles(self, stop: dict):
        reply = self._stop_send(stop, protocol.get_angle())
        if reply is None:
            return None
        try:
            angles = protocol.parse_numbers(reply.values, (4, 6))[:4]
        except MG400ProtocolError:
            return None
        stop["angles"].append((time.monotonic(), angles))
        return angles

    def _sleep_until(self, t: float, stop: dict):
        time.sleep(max(0.0, min(t, stop["deadline"]) - time.monotonic()))

    def _confirm_disabled(self, stop: dict) -> bool:
        """Polls alternating RobotMode / GetAngle for STOP_POLL_S; when a 4 is read, it must
        hold: a RobotMode reply of 4 read at least STOP_HOLD_S after the first one, with no
        other mode read between. The hold is shown by replies, not by elapsed time: if no
        such reply arrives within hold + STOP_HOLD_GRACE_S the hold is unconfirmed. Returns
        True if disabled was shown; False with stop["hold_broken"] (a 4 followed by another
        mode), stop["hold_unconfirmed"] (no replies to show it) or neither (no 4 in time)"""
        hold = STOP_HOLD_S
        stop["hold_s"] = hold
        end = min(time.monotonic() + STOP_POLL_S, stop["deadline"])
        first_4 = None
        polls = 0
        while time.monotonic() < stop["deadline"]:
            if polls % 2 == 0:
                mode = self._stop_mode(stop)
                t = time.monotonic()
                # the deadlines first: a reply that arrives after them counts for nothing
                if first_4 is not None and t - first_4 > hold + STOP_HOLD_GRACE_S:
                    stop["hold_unconfirmed"] = True
                    return False
                if first_4 is None and t >= end:
                    return False
                if mode == 4:
                    if first_4 is None:
                        first_4 = t
                        stop["disabled_seen_s"] = round(first_4 - stop["_t0"], 4)
                    elif t - first_4 >= hold:
                        return True
                elif mode is not None and first_4 is not None:
                    stop["hold_broken"] = mode
                    return False
            else:
                self._stop_angles(stop)
            polls += 1
            now = time.monotonic()
            if first_4 is None and now >= end:
                return False
            if first_4 is not None and now - first_4 > hold + STOP_HOLD_GRACE_S:
                stop["hold_unconfirmed"] = True          # replies stopped coming: not shown
                return False
            self._sleep_until(now + POLL_S, stop)
        return False

    def _enable_sent(self) -> bool:
        """The record says the Enable's bytes may have left: ``sent_utc`` is stamped on its
        command entry before the sendall (observe.Dashboard.write)"""
        text = protocol.enable_robot().text
        return any(c.get("command") == text and "sent_utc" in c for c in self.rec.report["commands"])

    def stop_sequence(self, reason: str) -> dict:
        """ResetRobot, DisableRobot, confirmation polls; EmergencyStop unless the arm showed
        itself disabled (mode 4 held) or in alarm and still, and the Enable's outcome is
        known. Bounded by STOP_TOTAL_S. Never raises; a further Ctrl+C is counted and ignored
        while it runs and until the clean-up starts (interrupt phase "stop", J5)"""
        self.ints.phase = "stop"                 # first: further Ctrl+C are counted and ignored (J5)
        t0 = time.monotonic()
        # the save method is decided here, whatever arm() got to before the fault (reviews 6
        # M1, 7 M1): queued when the thread runs, otherwise a no-op (no thread runs the saves:
        # none between the stop commands; saves and notices wait for finish())
        if self.io_started and self.io.is_alive():
            self.rec.save = self._async_save  # type: ignore[assignment]
        else:
            self.io_deferred = True
            self.rec.save = lambda: None  # type: ignore[assignment]
        # the Enable's outcome is known when it was never sent, was refused, or RobotMode 5 was
        # read after it; otherwise it may still take effect later and a mode 4 that holds now
        # proves nothing: the sequence then ends in EmergencyStop whatever the polls show.
        # "never sent" is the record's word, not the attribute's (review 5 M1): the attribute
        # is not_sent until the reply was read, the record has sent_utc from before the sendall
        if self.enable_result == "not_sent" and self._enable_sent():
            self.enable_result = "unresolved"
        enable_known = self.enable_result in ("not_sent", "refused") or self.enable_completed
        stop = {"reason": reason, "trigger_utc": _utc(), "enable_result": self.enable_result,
                "enable_completed": self.enable_completed, "enable_known": enable_known,
                "steps": [], "new_connections": 0, "stop_state": None, "last_mode": None,
                "disabled": False, "modes": [], "angles": [], "disable_attempts": 0,
                "_t0": t0, "deadline": t0 + STOP_TOTAL_S}
        self._sconn: Optional[Dashboard] = None          # the connection the stop commands use
        self.rec.report["stop"] = stop
        try:
            self.set_phase("stopping")
            if self.dash is not None:
                if self.dash.clean:
                    self._sconn = self._stop_conn(self.dash)
                    stop["dashboard-1"] = "reused (clean)"
                else:
                    stop["dashboard-1"] = ("not reused: outstanding %d, pending %r, partial %r, end %s"
                                           % (self.dash.outstanding, bool(self.dash._pending),
                                              bool(self.dash._reader.pending),
                                              self.dash.conn.get("end_seen")))
                    self.dash.close()
            self._stop_send(stop, protocol.reset_robot())
            while True:
                stop["disable_attempts"] += 1
                reply = self._stop_send(stop, protocol.disable_robot())
                if reply is not None and "disable_reply_s" not in stop:
                    stop["disable_reply_s"] = round(time.monotonic() - t0, 4)
                if self._confirm_disabled(stop):
                    stop["disabled"] = True
                    if enable_known:
                        stop["stop_state"] = "reset_disable"
                    break
                if "hold_broken" in stop and stop["disable_attempts"] < 2:
                    stop.pop("hold_broken")
                    continue                                 # once more, then the last resort
                break
            if stop["stop_state"] is None:
                now = time.monotonic()
                if stop["disabled"]:
                    stop["why_emergency_stop"] = ("the Enable's outcome is not known (%s, completed %s): "
                                                  "a mode 4 that holds now may not last"
                                                  % (self.enable_result, self.enable_completed))
                elif stop["last_mode"] == 9 and enable_known and _still(stop["angles"], now):
                    stop["stop_state"] = "alarm_still"
                else:
                    stop["why_emergency_stop"] = ("disabled not shown (last RobotMode %s, hold broken %s, "
                                                  "hold unconfirmed %s, still %s, enable known %s)"
                                                  % (stop["last_mode"], stop.get("hold_broken"),
                                                     stop.get("hold_unconfirmed", False),
                                                     _still(stop["angles"], now), enable_known))
                if stop["stop_state"] is None:
                    reply = self._stop_send(stop, protocol.emergency_stop())
                    stop["stop_state"] = "emergency_stop" if reply is not None else "unconfirmed"
                    end = min(time.monotonic() + STOP_ESTOP_POLL_S, stop["deadline"])
                    while time.monotonic() < end:
                        self._stop_mode(stop)
                        self._sleep_until(time.monotonic() + POLL_S, stop)
        except BaseException as e:  # noqa: BLE001 - recorded; the original exception is raised by the caller
            stop["sequence_error"] = repr(e)
        finally:
            if stop["stop_state"] is None:
                stop["stop_state"] = "unconfirmed"
            stop["guidance"] = GUIDANCE[stop["stop_state"]]
            stop["ended_utc"] = _utc()
            stop["duration_s"] = round(time.monotonic() - t0, 4)
            stop["deadline_hit"] = time.monotonic() >= stop["deadline"]
            stop["angles"] = [(round(t - t0, 4), a) for t, a in stop["angles"]]
            stop["modes"] = [(round(t - t0, 4), m) for t, m in stop["modes"]]
            for key in ("_t0", "deadline"):
                stop.pop(key, None)
            self._sconn = None
        return stop

    def finish(self):
        """Back to synchronous saves; drain the I/O thread"""
        if self.armed:
            self.rec.save = self._sync_save                        # type: ignore[assignment]
            if self.io_started or self.io.is_alive():          # alive: start() returned, the flag did not follow
                self.rec.report["io_thread_drained"] = self.io.stop(IO_DRAIN_S)
            for i, text in enumerate(self._deferred_out):          # the run is over: blocking is harmless now
                try:
                    self._out(text)
                except KeyboardInterrupt:
                    raise                       # the second Ctrl+C of the clean-up: not a notice failure (review 11 M2)
                except BaseException as e:  # noqa: BLE001 - a broken stdout must not cost the record (review 5 M3)
                    self.rec.report.setdefault("notice_errors", []).append(
                        {"error": repr(e), "unprinted": self._deferred_out[i:]})
                    break
            self._deferred_out = []
            if self.io.errors:
                self.rec.report["io_thread_errors"] = list(self.io.errors)


CLEANUP_ATTEMPTS = 3


class _Cleanup:
    """The clean-up of jog(): steps run in order, each under its own guard, and the loop
    itself re-entered (CLEANUP_ATTEMPTS attempts in all) when a BaseException lands between
    two steps, resuming at the step it was at (review 8 M1). A first Ctrl+C is deferred by
    the run's _Interrupts (phase "cleanup"), so what the loop guards against is an exception
    injected by a test, a SystemExit, or the KeyboardInterrupt of a second Ctrl+C, which
    ``escape`` reports: the loop then stops where it is and re-raises it with the report
    (review 10 M1). ``plan`` builds the step list and ``initial`` reads the run's error,
    both inside the guard; the run's error comes first and a recorded clean-up error is
    kept when there is none (review 9 M1). At the end the report is put on ``error``,
    which the caller raises"""

    def __init__(self, rec: Recorder, plan: Callable[[], List[Callable[[], None]]],
                 initial: Callable[[], Optional[BaseException]], hook=None,
                 escape: Callable[[], bool] = lambda: False, on_done: Callable[[], None] = lambda: None):
        self.rec, self.plan, self.initial, self.hook, self.escape = rec, plan, initial, hook, escape
        self.on_done = on_done
        self.bailed = False
        self.steps: Optional[List[Callable[[], None]]] = None
        self.next = 0
        self.error: Optional[BaseException] = None
        self.errors: List[str] = []

    def record(self, name: str, e: BaseException):
        self.errors.append(f"{name}: {e!r}")
        self.rec.report["cleanup_errors"] = self.errors
        if self.error is None:
            self.error = e

    def _attach(self, e: BaseException):
        e.mg400_report = self.rec.report       # type: ignore[attr-defined]
        e.mg400_recorder = self.rec            # type: ignore[attr-defined]

    def _bail(self, e: BaseException):
        """A second Ctrl+C: the clean-up ends here, the record may be incomplete"""
        if self.escape():
            self.bailed = True
            self.rec.report["cleanup_cut_short"] = True
            self._attach(e)
            raise e

    def run(self):
        for _ in range(CLEANUP_ATTEMPTS):
            try:
                init = self.initial()               # first of all, so that nothing below loses it
                if init is not None:                # the run's own error comes first; a recorded
                    self.error = init               # clean-up error is kept when there is none
                if self.hook is not None:
                    self.hook("plan")               # tests: an interrupt in the preparation
                if self.steps is None:
                    self.steps = self.plan()
                while self.next < len(self.steps):
                    fn = self.steps[self.next]
                    if self.hook is not None:
                        self.hook(fn.__name__)      # tests: an interrupt between two steps
                    self.next += 1
                    try:
                        fn()
                    except BaseException as e:  # noqa: BLE001 - recorded; the loop goes on
                        self.record(fn.__name__, e)
                        self._bail(e)
                break
            except BaseException as e:  # noqa: BLE001 - between two steps: re-entered
                if self.bailed:             # the step guard already recorded and re-raised it
                    raise
                self.record("clean-up", e)
                self._bail(e)
        if self.error is not None:                  # the report travels with the exception
            self._attach(self.error)
        self.on_done()                              # the caller's last word (a deferred Ctrl+C, J7)
