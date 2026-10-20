"""Observe a Dobot MG400 over TCP/IP without changing its state (human-run tool).

Sends only the six read-only commands of :mod:`protocol` (RobotMode, GetErrorID, GetPose,
GetAngle, PositiveSolution, InverseSolution) on the Dashboard port 29999, and only reads
the feedback port 30004. The motion port is never opened. It records what the plan to
integrate the MG400 needs: replies and their form, whether two Dashboard connections are
served at once, the feedback packets (raw and parsed), their timing, and the controller's
own forward / inverse kinematics on a grid of joint sets.

The observation ends, sending nothing more, as soon as any reply or any feedback packet
shows a mode other than 4 (disabled) or an alarm, when the feedback stops or never starts,
or on any protocol error. That is the arm's state when read; the tool does not guarantee
it afterwards.

It is the arm, so the real run is the human's (CLAUDE.md rule 1), although nothing it
sends moves the arm. Ctrl+C ends the observation: it sends nothing to the arm, it is **not**
an emergency stop, and the arm's state afterwards is not guaranteed by this tool.

Usage (from the repository root)::

    python -m src.devices.mg400.observe --dry-run            # list the commands, connect to nothing
    python -m src.devices.mg400.observe --mock               # fake controller on 127.0.0.1
    python -m src.devices.mg400.observe <arm address> [--seconds 10] [--controller-version V]

Records go to ``logs/mg400_observe/<UTC time>/`` (gitignored): ``report.json``,
``feedback.bin`` (raw packets back to back), ``feedback_times.csv``, ``feedback.csv``.
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import socket
import sys
import threading
import time
from typing import Callable, List, Optional

from . import feedback, protocol
from .protocol import Command, MG400Error, MG400ProtocolError

DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004
TIMEOUT_S = 3.0
FEEDBACK_FIRST_S = 2.0          # the first valid packet must arrive within this
FEEDBACK_MAX_AGE_S = 0.5        # the last valid packet must be younger than this
LOG_ROOT = os.path.join("logs", "mg400_observe")
GUIDE = "TCP_IP Remote Control Interface Guide (4axis) 2024-04-19"

#: joint sets for the kinematics grid (deg), inside the datasheet limits with margin
#: (J1 +-160, J2 -25..85, J3 -25..105)
GRID_J1 = (-30.0, 0.0, 30.0)
GRID_J2 = (0.0, 30.0)
GRID_J3 = (0.0, 30.0)
GRID_J4 = (0.0,)

#: reply lengths accepted per command (GetAngle: the guide shows 4 and 6, the bench gave 6)
LENGTHS = {"GetPose": (4,), "GetAngle": (4, 6), "PositiveSolution": (4,), "InverseSolution": (4,)}

PRECONDITIONS = """Before observing (nothing is moved, but it is the arm):
  - DobotStudio Pro is disconnected from the arm; nobody else is controlling it.
  - The arm should be disabled (RobotMode 4) with no alarm; otherwise the tool stops.
  - Ctrl+C ends the observation only. It sends nothing and is NOT an emergency stop."""

Connector = Callable[[str, int, float], socket.socket]


def _connect(host: str, port: int, timeout: float) -> socket.socket:
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    return s


def _finite(value):
    """Copy for JSON: non-finite floats become None (the raw packets keep the bytes)"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def _utc() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _source_id() -> str:
    """sha256 of this package's .py files (which code made the record)"""
    h = hashlib.sha256()
    folder = os.path.dirname(os.path.abspath(__file__))
    for name in sorted(os.listdir(folder)):
        if name.endswith(".py"):
            with open(os.path.join(folder, name), "rb") as f:
                h.update(name.encode() + b"\0" + f.read())
    return h.hexdigest()[:16]


class ObservationEnded(MG400Error):
    """The arm is not in the state the observation requires; nothing more is sent"""


class Recorder:
    """The run folder; ``report.json`` is rewritten after every command (``complete`` false
    until the end), so an interrupted run leaves a partial record"""

    def __init__(self, folder: str):
        os.makedirs(folder, exist_ok=True)
        self.folder = folder
        self.report = {"tool": "src.devices.mg400.observe", "source_sha256_16": _source_id(),
                       "guide": GUIDE, "complete": False, "started_utc": _utc(),
                       "commands": [], "notes": []}
        self._lock = threading.Lock()
        self.save()

    def save(self):
        with self._lock:
            tmp = os.path.join(self.folder, "report.json.tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(_finite(self.report), f, ensure_ascii=False, indent=2, allow_nan=False)
            os.replace(tmp, os.path.join(self.folder, "report.json"))

    def note(self, text: str):
        with self._lock:
            self.report["notes"].append(text)


def judge_mode(mode: int):
    if mode != protocol.MODE_DISABLED:
        raise ObservationEnded(f"RobotMode {mode} ({protocol.ROBOT_MODES.get(mode, '?')}), "
                               "not 4 (DISABLED)")


def judge_errors(errors: list):
    if not protocol.no_alarm(errors):
        raise ObservationEnded(f"alarm reported by GetErrorID: {errors}")


class Dashboard:
    """One Dashboard connection. Writes only commands that pass :func:`protocol.check_command`
    in their exact form; every RobotMode / GetErrorID reply on any connection is judged at once"""

    def __init__(self, sock: socket.socket, recorder: Recorder, label: str,
                 guard: Optional[Callable[[], None]] = None):
        self._sock = sock
        self._guard = guard            # called right before every send; raises to stop it
        self._reader = protocol.ReplyReader()
        self._pending: List[protocol.Reply] = []
        self._rec = recorder
        self.label = label

    def write(self, command: Command) -> dict:
        if not isinstance(command, Command):
            raise TypeError("only protocol.Command objects can be sent")
        protocol.check_command(command.text, command.name)
        entry = {"connection": self.label, "command": command.text, "planned_utc": _utc(),
                 "reply": None}
        self._rec.report["commands"].append(entry)       # recorded before the bytes leave
        self._rec.save()
        try:
            if self._guard is not None:
                self._guard()
            entry["sent_utc"], entry["sent_monotonic"] = _utc(), time.monotonic()
            self._sock.sendall(command.text.encode("ascii"))
        except BaseException as e:
            entry["error"] = f"not sent: {e!r}" if "sent_utc" not in entry else f"send: {e!r}"
            self._rec.save()
            raise
        return entry

    def read(self, command: Command, entry: dict) -> protocol.Reply:
        try:
            while not self._pending:
                data = self._sock.recv(4096)
                if not data:
                    raise MG400ProtocolError(f"{self.label}: connection closed")
                self._pending.extend(self._reader.feed(data))
            reply = self._pending.pop(0)
            entry.update(reply=reply.raw, reply_utc=_utc(),
                         reply_s=round(time.monotonic() - entry["sent_monotonic"], 4))
            entry["echo_note"] = protocol.check_echo(reply, command)
            if reply.error_id != 0:
                raise protocol.MG400CommandError(f"{command.text} -> ErrorID {reply.error_id}")
            if command.name == "RobotMode":
                judge_mode(protocol.parse_robot_mode(reply.values))
            elif command.name == "GetErrorID":
                judge_errors(protocol.parse_error_ids(reply.values))
            elif command.name in LENGTHS:
                protocol.parse_numbers(reply.values, LENGTHS[command.name])
            return reply
        except BaseException as e:
            entry.setdefault("error", repr(e))
            if self._reader.pending:
                entry["partial_reply"] = self._reader.pending
            raise
        finally:
            self._rec.save()

    def query(self, command: Command) -> protocol.Reply:
        return self.read(command, self.write(command))

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


class FeedbackRecorder(threading.Thread):
    """Reads port 30004 (never writes), saves every packet raw with its receive times and
    parsed fields, and keeps the first fault (mode other than 4, alarm, out-of-range value,
    rejected packet, read error, connection closed) until the end"""

    COLUMNS = ["seq", "recv_monotonic_s", "recv_unix_ms", "timestamp_minus_recv_ms"] + \
        list(feedback.FIELDS) + ["nonfinite", "out_of_range"]

    def __init__(self, sock: socket.socket, folder: str):
        super().__init__(name="mg400-feedback", daemon=True)
        self._sock = sock
        self._sock.settimeout(0.2)
        self._halt = threading.Event()
        self._asm = feedback.Assembler()
        self._bin = open(os.path.join(folder, "feedback.bin"), "wb")
        self._times = open(os.path.join(folder, "feedback_times.csv"), "w", newline="", encoding="utf-8")
        self._parsed = open(os.path.join(folder, "feedback.csv"), "w", newline="", encoding="utf-8")
        self._tw = csv.writer(self._times)
        self._tw.writerow(["seq", "byte_offset", "recv_monotonic_s", "recv_unix_ms", "rejected"])
        self._pw = csv.writer(self._parsed)
        self._pw.writerow(self.COLUMNS)
        self.lock = threading.Lock()
        self.latest: Optional[dict] = None
        self.last_valid_t: Optional[float] = None
        self.recv_times: List[float] = []
        self.timestamps: List[int] = []
        self.clock_diffs: List[int] = []
        self.rejected: List[str] = []
        self.fault: Optional[str] = None
        self.tail = 0
        self.started_t = time.monotonic()

    def _fault(self, text: str):
        with self.lock:
            if self.fault is None:
                self.fault = text

    def run(self):
        try:
            while not self._halt.is_set():
                try:
                    data = self._sock.recv(65536)
                except socket.timeout:
                    continue
                except OSError as e:
                    # our own stop closes the socket: only then is a receive error expected
                    if not self._halt.is_set():
                        self._fault(f"feedback read failed: {e}")
                    return
                if not data:
                    if not self._halt.is_set():
                        self._fault("feedback connection closed")
                    return
                t = time.monotonic()
                wall = int(time.time() * 1000)
                for chunk in self._asm.feed(data):
                    self._bin.write(chunk.data)
                    try:
                        pkt = feedback.parse(chunk.data)
                        reason = ""
                    except feedback.FeedbackError as e:
                        pkt, reason = None, str(e)
                    self._tw.writerow([chunk.seq, chunk.offset, f"{t:.6f}", wall, reason])
                    if pkt is None:
                        with self.lock:
                            self.rejected.append(reason)
                        self._fault(f"feedback packet {chunk.seq} rejected: {reason}")
                        continue
                    diff = pkt["timestamp_ms"] - wall
                    self._pw.writerow([chunk.seq, f"{t:.6f}", wall, diff] +
                                      [json.dumps(pkt[k]) if isinstance(pkt[k], list) else pkt[k]
                                       for k in feedback.FIELDS] +
                                      [";".join(pkt["nonfinite"]), ";".join(pkt["out_of_range"])])
                    with self.lock:
                        self.latest = pkt
                        self.last_valid_t = t
                        self.recv_times.append(t)
                        self.timestamps.append(pkt["timestamp_ms"])
                        self.clock_diffs.append(diff)
                    if pkt["robot_mode"] != protocol.MODE_DISABLED or pkt["error_status"]:
                        self._fault(f"feedback packet {chunk.seq}: mode {pkt['robot_mode']}, "
                                    f"error_status {pkt['error_status']}")
                    elif pkt["out_of_range"] or pkt["nonfinite"]:
                        self._fault(f"feedback packet {chunk.seq}: out of range "
                                    f"{pkt['out_of_range']}, non-finite {pkt['nonfinite']}")
        except BaseException as e:  # noqa: BLE001 - a recording failure is a fault, stopped or not
            self._fault(f"feedback recording failed: {e!r}")
        finally:
            self.tail = self._asm.tail
            for f in (self._bin, self._times, self._parsed):
                try:
                    f.close()
                except BaseException as e:  # noqa: BLE001 - flush / close failure: the record is incomplete
                    self._fault(f"closing {os.path.basename(f.name)} failed: {e!r}")

    def check(self, now: Optional[float] = None):
        """Raise ObservationEnded unless the feed is valid, fresh and has shown no fault"""
        now = time.monotonic() if now is None else now
        with self.lock:
            fault, last = self.fault, self.last_valid_t
        if fault:
            raise ObservationEnded(fault)
        if last is None:
            raise ObservationEnded("no valid feedback packet received")
        if now - last > FEEDBACK_MAX_AGE_S:
            raise ObservationEnded(f"feedback stale: last valid packet {now - last:.2f} s ago")

    def wait_first(self, timeout: float = FEEDBACK_FIRST_S):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if self.last_valid_t is not None or self.fault:
                    break
            time.sleep(0.01)
        self.check()

    def stop(self):
        self._halt.set()
        try:
            self._sock.close()
        except OSError:
            pass

    def summary(self) -> dict:
        with self.lock:
            gaps = [(b - a) * 1000 for a, b in zip(self.recv_times, self.recv_times[1:])]
            steps = [b - a for a, b in zip(self.timestamps, self.timestamps[1:])]
            return {"packets": len(self.recv_times), "rejected": len(self.rejected),
                    "reject_reasons": sorted(set(self.rejected)), "fault": self.fault,
                    "span_s": round(self.recv_times[-1] - self.recv_times[0], 3)
                    if len(self.recv_times) > 1 else 0.0,
                    "inter_arrival_ms": feedback.stats(gaps),
                    "timestamp_step_ms": feedback.stats(steps),
                    "timestamp_minus_pc_receive_ms": feedback.stats(self.clock_diffs),
                    "note_clock": "PC clock not assumed synchronised: not a latency",
                    "truncated_tail_bytes": self.tail,
                    "last_packet": dict(self.latest or {})}


def kinematics_grid():
    for j1 in GRID_J1:
        for j2 in GRID_J2:
            for j3 in GRID_J3:
                for j4 in GRID_J4:
                    yield [j1, j2, j3, j4]


def _check_state(dash: Dashboard, fb: FeedbackRecorder):
    """Feedback first (no command if it already shows a fault), then RobotMode (judged
    before GetErrorID is sent), then GetErrorID; all judged inside Dashboard.read"""
    fb.check()
    dash.query(protocol.robot_mode())
    dash.query(protocol.get_error_id())
    fb.check()


def observe(host: str, folder: str, seconds: float = 10.0, controller_version: Optional[str] = None,
            connect: Connector = _connect, dashboard_port: int = DASHBOARD_PORT,
            feedback_port: int = FEEDBACK_PORT, out=print) -> dict:
    """Run the observation. Returns the report (also written to ``folder``)"""
    rec = Recorder(folder)
    rec.report["controller_version_typed"] = controller_version
    dashes: List[Dashboard] = []
    fb: Optional[FeedbackRecorder] = None
    ended = None
    try:
        fb = FeedbackRecorder(connect(host, feedback_port, TIMEOUT_S), folder)
        fb.start()
        fb.wait_first()
        dash = Dashboard(connect(host, dashboard_port, TIMEOUT_S), rec, "dashboard-1", fb.check)
        dashes.append(dash)
        _check_state(dash, fb)
        out("state: disabled, no alarm")

        # 1. queries
        r = {}
        r["GetPose()"] = protocol.parse_numbers(dash.query(protocol.get_pose()).values, 4)
        r["GetPose(User=0,Tool=0)"] = protocol.parse_numbers(
            dash.query(protocol.get_pose(base_frames=True)).values, 4)
        r["GetAngle()"] = protocol.parse_numbers(dash.query(protocol.get_angle()).values, (4, 6))
        rec.report["queries"] = r
        rec.save()

        # 2. a second Dashboard connection (every RobotMode / GetErrorID reply is judged)
        dash2 = Dashboard(connect(host, dashboard_port, TIMEOUT_S), rec, "dashboard-2", fb.check)
        dashes.append(dash2)
        two = {"interleaved": [], "outstanding": None}
        for cmd in (protocol.robot_mode(), protocol.get_error_id(), protocol.get_pose(),
                    protocol.get_angle()):
            for d in (dash, dash2):
                fb.check()
                d.query(cmd)
                two["interleaved"].append({"connection": d.label, "command": cmd.text, "ok": True})
        fb.check()
        c1 = protocol.robot_mode()
        e1 = dash.write(c1)                      # left outstanding while dashboard-2 answers
        t2 = time.monotonic()
        dash2.query(protocol.get_angle())
        answered = time.monotonic() - t2
        dash.read(c1, e1)
        two["outstanding"] = {"second_answered_s": round(answered, 4)}
        rec.report["two_connections"] = two
        rec.save()
        out("two Dashboard connections: both answered")

        # 3. feedback window
        out(f"recording feedback for {seconds:g} s ...")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            fb.check()
            time.sleep(0.05)
        _check_state(dash, fb)
        r["GetAngle() after window"] = protocol.parse_numbers(
            dash.query(protocol.get_angle()).values, (4, 6))
        r["GetPose(User=0,Tool=0) after window"] = protocol.parse_numbers(
            dash.query(protocol.get_pose(base_frames=True)).values, 4)
        with fb.lock:
            last = dict(fb.latest)
        rec.report["compare_mapping_unverified"] = {
            "q_actual_minus_GetAngle_first4": [
                round(a - b, 6) for a, b in zip(last["q_actual"][:4], r["GetAngle() after window"][:4])],
            "tool_vector_actual_minus_GetPose00_first4": [
                round(a - b, 6) for a, b in zip(last["tool_vector_actual"][:4],
                                                r["GetPose(User=0,Tool=0) after window"][:4])]}
        rec.save()

        # 4. kinematics (compute only); the state is checked before every compute command
        pairs = []
        current = r["GetAngle()"][:4]
        for joints in [current] + list(kinematics_grid()):
            _check_state(dash, fb)
            fk = protocol.parse_numbers(dash.query(protocol.positive_solution(joints)).values, 4)
            _check_state(dash, fb)
            ik = protocol.parse_numbers(dash.query(protocol.inverse_solution(fk, joints)).values, 4)
            pairs.append({"joints": joints, "positive_solution": fk, "inverse_solution": ik})
            rec.report["kinematics"] = pairs
            rec.save()
        _check_state(dash, fb)
        out(f"kinematics: {len(pairs)} pairs")
        fb.stop()
        fb.join(timeout=2.0)
        if fb.is_alive():
            raise ObservationEnded("the feedback recorder did not stop")
        with fb.lock:
            fault = fb.fault
        if fault:
            raise ObservationEnded(fault)
        rec.report["complete"] = True
        return rec.report
    except ObservationEnded as e:
        ended = f"observation ended: {e}"
        out(ended)
        return rec.report
    except BaseException as e:
        ended = f"stopped by {type(e).__name__}: {e}"
        raise
    finally:
        if ended:
            rec.note(ended)
        for d in dashes:
            d.close()
        if fb is not None:
            fb.stop()
            fb.join(timeout=2.0)
            rec.report["feedback"] = fb.summary()       # the final summary, whole recording
        rec.report["ended_utc"] = _utc()
        rec.save()


def dry_run(out=print):
    """The command templates in order; connects to nothing"""
    out("feedback port %d: read only, never written; first valid packet within %g s"
        % (FEEDBACK_PORT, FEEDBACK_FIRST_S))
    for line in ("RobotMode()   (end unless 4)", "GetErrorID()  (end on any alarm)",
                 "GetPose()", "GetPose(User=0,Tool=0)", "GetAngle()",
                 "second connection: RobotMode() GetErrorID() GetPose() GetAngle() on both, interleaved",
                 "RobotMode() on 1 left outstanding while GetAngle() on 2",
                 "wait --seconds, recording feedback (end on any fault)",
                 "RobotMode() GetErrorID() GetAngle() GetPose(User=0,Tool=0)",
                 "for the current joints and %d grid joint sets: RobotMode() GetErrorID() "
                 "PositiveSolution(j1,j2,j3,j4,0,0) RobotMode() GetErrorID() "
                 "InverseSolution(x,y,z,r,0,0,1,{j1,j2,j3,j4})" % len(list(kinematics_grid())),
                 "RobotMode() GetErrorID()"):
        out("  " + line)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Observe a Dobot MG400 without changing its state "
                                             "(human-run; read-only commands).")
    ap.add_argument("address", nargs="?", help="the arm's address (not used with --mock)")
    ap.add_argument("--mock", action="store_true", help="fake controller on 127.0.0.1 only")
    ap.add_argument("--dry-run", action="store_true", help="list the commands; connect to nothing")
    ap.add_argument("--seconds", type=float, default=10.0, help="feedback window (1..120 s)")
    ap.add_argument("--controller-version", default=None, help="as shown in DobotStudio Pro")
    ap.add_argument("--out", default=None, help="run folder (default logs/mg400_observe/<UTC time>)")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if not 1.0 <= args.seconds <= 120.0:
        ap.error("--seconds must be 1..120")
    if args.dry_run:
        dry_run()
        return 0
    if not args.mock and not args.address:
        ap.error("give the arm's address, or --mock / --dry-run")
    folder = args.out or os.path.join(
        LOG_ROOT, datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ("_mock" if args.mock else ""))
    print(PRECONDITIONS)
    fake = None
    try:
        if args.mock:
            from .fake import LOOPBACK, FakeMG400
            fake = FakeMG400()

            def connect(host, port, timeout):
                # never the address given on the command line, never a name lookup of it
                return _connect(LOOPBACK, {DASHBOARD_PORT: fake.dashboard_port,
                                           FEEDBACK_PORT: fake.feedback_port}[port], timeout)
            report = observe(LOOPBACK, folder, args.seconds, args.controller_version, connect=connect)
        else:
            report = observe(args.address, folder, args.seconds, args.controller_version)
    except KeyboardInterrupt:
        print("\nCtrl+C: observation ended. Nothing was sent to the arm; this is not an "
              "emergency stop. Partial record in %s" % folder)
        return 130
    except MG400Error as e:
        print("error: %s (record in %s)" % (e, folder))
        return 1
    except OSError as e:
        print("connection error: %s (record in %s)" % (e, folder))
        return 1
    finally:
        if fake is not None:
            fake.close()
    print("record: %s (complete: %s)" % (folder, report.get("complete")))
    return 0 if report.get("complete") else 2


if __name__ == "__main__":
    sys.exit(main())
