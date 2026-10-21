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
from .protocol import Command, MG400ConnectionError, MG400Error

DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004
TIMEOUT_S = 3.0
FEEDBACK_FIRST_S = 2.0          # the first valid packet must arrive within this
FEEDBACK_MAX_AGE_S = 0.5        # the last valid packet must be younger than this
IDLE_PROBE_S = 1.0              # dashboard-1 idle before the control probe
SECOND_OPEN_SETTLE_S = 0.2      # wait after opening dashboard-2, before probing dashboard-1
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

ONEDRIVE_NOTE = ("note: the record folder may be synced by OneDrive; if saving fails, "
                 "use --out <folder outside OneDrive>")

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


class MG400RecordError(MG400Error):
    """The record could not be saved. ``recorder.describe()``, called when the message is
    printed, says what is on disk at that moment"""

    def __init__(self, text: str, recorder: Optional["Recorder"] = None):
        super().__init__(text)
        self.recorder = recorder


#: waits before retrying a refused rename (a sync client or a scanner holding report.json)
REPLACE_RETRY_S = (0.05, 0.1, 0.2, 0.4, 0.8)


class Recorder:
    """The run folder; ``report.json`` is rewritten after every command (``complete`` false
    until the end), so an interrupted run leaves a partial record. A save writes
    ``report.json.tmp`` and renames it; what is on disk after a failure is kept in
    ``report_current`` / ``tmp_current`` / ``confirmed_saves`` and told by :meth:`describe`"""

    def __init__(self, folder: str):
        os.makedirs(folder, exist_ok=True)
        self.folder = folder
        self.report = {"tool": "src.devices.mg400.observe", "source_sha256_16": _source_id(),
                       "guide": GUIDE, "complete": False, "started_utc": _utc(),
                       "commands": [], "notes": []}
        self._lock = threading.Lock()
        self.confirmed_saves = 0        # renames that succeeded in this run
        self.report_current = False     # report.json holds the latest record
        self.tmp_current = False        # report.json.tmp holds the latest record
        self.failed = False             # sticky: some save of this run failed
        self.first_error: Optional[str] = None
        self.save()                     # before any connection: a failure opens no socket

    def _write_tmp(self, tmp: str):
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(_finite(self.report), f, ensure_ascii=False, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())

    def _fail(self, text: str):
        self.failed = True
        if self.first_error is None:
            self.first_error = text
        raise MG400RecordError(text, recorder=self)

    def save(self):
        """Raises MG400RecordError (nothing else) when the record could not be saved"""
        with self._lock:
            tmp = os.path.join(self.folder, "report.json.tmp")
            try:
                self._write_tmp(tmp)
            except Exception as e:  # noqa: BLE001 - open / write / flush / fsync / close
                self.report_current = self.tmp_current = False
                self._fail(f"could not write report.json.tmp: {e!r}")
            err = None
            for delay in REPLACE_RETRY_S + (None,):
                try:
                    os.replace(tmp, os.path.join(self.folder, "report.json"))
                    self.confirmed_saves += 1
                    self.report_current, self.tmp_current = True, False
                    return
                except PermissionError as e:
                    err = e
                    if delay is None:
                        break
                    time.sleep(delay)
                except OSError as e:          # not a lock: retrying does not help
                    err = e
                    break
            self.report_current = False
            self.report.setdefault("save_errors", []).append(
                {"utc": _utc(), "error": f"renaming report.json.tmp to report.json: {err!r}"})
            try:
                self._write_tmp(tmp)          # again, now with the error in it
                self.tmp_current = True
            except Exception:  # noqa: BLE001
                self.tmp_current = False
            self._fail(f"could not rename report.json.tmp to report.json: {err!r}")

    def save_during(self):
        """For paths where an exception is already on its way: a save failure is kept and
        printed, never raised, so it cannot replace the original exception"""
        try:
            self.save()
        except MG400RecordError as e:
            try:
                print(f"record error while stopping: {e}", file=sys.stderr)
            except Exception:  # noqa: BLE001 - not even this may replace the original
                pass

    def describe(self) -> str:
        """What is on disk now (call it after the last save)"""
        if self.report_current:
            return "report.json is current"
        if self.tmp_current:
            return "report.json.tmp holds the latest record (report.json is older or missing)"
        if self.confirmed_saves:
            return (f"report.json holds the last successful save of this run (save "
                    f"{self.confirmed_saves}), not the latest; report.json.tmp may be incomplete")
        return "no save of this run was confirmed"

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


def _kind(e: OSError) -> str:
    """What a socket error says, by its type (not a cause)"""
    return ("timeout" if isinstance(e, socket.timeout) else
            "refused" if isinstance(e, ConnectionRefusedError) else
            "reset" if isinstance(e, ConnectionResetError) else
            "aborted" if isinstance(e, ConnectionAbortedError) else
            "broken pipe" if isinstance(e, BrokenPipeError) else "other")


def _attribute(e: BaseException, entry: dict):
    """Name the command an exception belongs to; the first command named keeps it"""
    if getattr(e, "command_id", None) is None:
        e.command_id = entry["id"]


class Dashboard:
    """One Dashboard connection. Writes only commands that pass :func:`protocol.check_command`
    in their exact form; every RobotMode / GetErrorID reply on any connection is judged at once"""

    def __init__(self, sock: socket.socket, recorder: Recorder, label: str,
                 guard: Optional[Callable[[], None]] = None, port: Optional[int] = None):
        self._sock = sock
        self._guard = guard            # called right before every send; raises to stop it
        self._reader = protocol.ReplyReader()
        self._pending: List[protocol.Reply] = []
        self._rec = recorder
        self.label = label
        self.last_reply: Optional[tuple] = None           # (utc, monotonic) of the last reply
        # the connection's record; on disk with the next save, before anything is sent on it
        self.conn = {"label": label, "port": port, "opened_utc": _utc(),
                     "opened_monotonic": round(time.monotonic(), 6)}
        recorder.report.setdefault("connections", []).append(self.conn)

    def _ended(self, kind: str, error: BaseException):
        if "end_seen" not in self.conn:
            self.conn.update(end_seen=f"{kind} detected", end_detected_utc=_utc(),
                             end_detected_monotonic=round(time.monotonic(), 6), error=repr(error))

    def _connection_error(self, op: str, e: OSError) -> MG400ConnectionError:
        kind = _kind(e)
        if kind == "timeout":
            text = (f"{self.label}: no reply within {self._sock.gettimeout():g} s" if op == "recv"
                    else f"{self.label}: send not done within {self._sock.gettimeout():g} s")
        else:
            text = f"{self.label}: {op} failed ({kind}): {e}"
        err = MG400ConnectionError(text, label=self.label, op=op, kind=kind,
                                   partial_reply=bool(self._reader.pending))
        if kind != "timeout":
            self._ended(kind, err)
        return err

    def write(self, command: Command, tag: Optional[str] = None) -> dict:
        if not isinstance(command, Command):
            raise TypeError("only protocol.Command objects can be sent")
        protocol.check_command(command.text, command.name)
        entry = {"id": len(self._rec.report["commands"]) + 1, "connection": self.label,
                 "command": command.text, "planned_utc": _utc(), "reply": None}
        if tag is not None:
            entry["tag"] = tag
        self._rec.report["commands"].append(entry)       # recorded before the bytes leave
        try:
            self._rec.save()
        except BaseException as e:
            _attribute(e, entry)
            raise
        try:
            if self._guard is not None:
                self._guard()
            entry["sent_utc"], entry["sent_monotonic"] = _utc(), time.monotonic()
            try:
                self._sock.sendall(command.text.encode("ascii"))
            except OSError as e:
                raise self._connection_error("send", e) from e
        except BaseException as e:
            _attribute(e, entry)
            entry["error"] = f"not sent: {e!r}" if "sent_utc" not in entry else f"send: {e!r}"
            self._rec.save_during()
            raise
        return entry

    def _recv(self) -> bytes:
        try:
            data = self._sock.recv(4096)
        except OSError as e:
            raise self._connection_error("recv", e) from e
        if not data:
            err = MG400ConnectionError(f"{self.label}: connection closed by the arm",
                                       label=self.label, op="recv", kind="eof",
                                       partial_reply=bool(self._reader.pending))
            self._ended("eof", err)
            raise err
        return data

    def read(self, command: Command, entry: dict) -> protocol.Reply:
        try:
            while not self._pending:
                self._pending.extend(self._reader.feed(self._recv()))
            reply = self._pending.pop(0)
            now = time.monotonic()
            entry.update(reply=reply.raw, reply_utc=_utc(),
                         reply_s=round(now - entry["sent_monotonic"], 4))
            self.last_reply = (entry["reply_utc"], now)
            entry["echo_note"] = protocol.check_echo(reply, command)
            if reply.error_id != 0:
                raise protocol.MG400CommandError(f"{command.text} -> ErrorID {reply.error_id}")
            if command.name == "RobotMode":
                judge_mode(protocol.parse_robot_mode(reply.values))
            elif command.name == "GetErrorID":
                judge_errors(protocol.parse_error_ids(reply.values))
            elif command.name in LENGTHS:
                protocol.parse_numbers(reply.values, LENGTHS[command.name])
        except BaseException as e:
            _attribute(e, entry)
            entry.setdefault("error", repr(e))
            if self._reader.pending:
                entry["partial_reply"] = self._reader.pending
            self._rec.save_during()
            raise
        try:
            self._rec.save()
        except BaseException as e:
            _attribute(e, entry)
            raise
        return reply

    def query(self, command: Command, tag: Optional[str] = None) -> protocol.Reply:
        return self.read(command, self.write(command, tag))

    def close(self):
        if "end_seen" not in self.conn:
            self.conn.update(end_seen="closed by this tool", end_detected_utc=_utc(),
                             end_detected_monotonic=round(time.monotonic(), 6))
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

    def __init__(self, sock: socket.socket, folder: str, recv_timeout: float = 0.2):
        super().__init__(name="mg400-feedback", daemon=True)
        self._sock = sock
        self._sock.settimeout(recv_timeout)
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
        self.end: Optional[dict] = None     # the end of the connection, when this thread saw it
        self.closed: Optional[dict] = None  # or when this tool first closed it
        self.tail = 0
        self.started_t = time.monotonic()

    def _fault(self, text: str):
        with self.lock:
            if self.fault is None:
                self.fault = text

    def _ended(self, seen: str):
        with self.lock:
            self.end = {"end_seen": seen, "end_detected_utc": _utc(),
                        "end_detected_monotonic": round(time.monotonic(), 6)}

    def _check_gap(self, now: float):
        """A gap is latched as a fault here, in the receiving thread: a feed that pauses and
        resumes between two of check()'s looks still ends the observation"""
        with self.lock:
            last = self.last_valid_t
        if last is not None and now - last > FEEDBACK_MAX_AGE_S and not self._halt.is_set():
            self._fault(f"feedback gap: no valid packet for {now - last:.2f} s")

    def run(self):
        try:
            while not self._halt.is_set():
                try:
                    data = self._sock.recv(65536)
                except socket.timeout:
                    self._check_gap(time.monotonic())
                    continue
                except OSError as e:
                    # our own stop closes the socket: only then is a receive error expected
                    if not self._halt.is_set():
                        self._ended(f"{_kind(e)} detected")
                        self._fault(f"feedback read failed: {e}")
                    return
                if not data:
                    if not self._halt.is_set():
                        self._ended("eof detected")
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
                    self._check_gap(t)          # before this packet renews last_valid_t
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
        with self.lock:
            if self.closed is None:
                self.closed = {"end_seen": "closed by this tool", "end_detected_utc": _utc(),
                               "end_detected_monotonic": round(time.monotonic(), 6)}
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


def _wait(fb: FeedbackRecorder, seconds: float):
    """Wait, checking the feedback every 50 ms (a fault ends the observation)"""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        fb.check()
        time.sleep(min(0.05, max(0.0, end - time.monotonic())))
    fb.check()


def _tolerated_drop(e: BaseException, probe_id: Optional[int], rec: Recorder) -> bool:
    """The one connection error the observation continues after: the probe's own, on
    dashboard-1, an end of stream or a reset by the peer with no reply bytes, and no save of
    the run has failed. Anything else ends the observation"""
    return (isinstance(e, MG400ConnectionError) and probe_id is not None
            and getattr(e, "command_id", None) == probe_id and e.label == "dashboard-1"
            and not e.partial_reply
            and ((e.kind == "eof" and e.op == "recv")
                 or (e.kind == "reset" and e.op in ("send", "recv")))
            and not rec.failed)


def _probe_timing(rec: Recorder, tag: str, before: Optional[tuple]) -> dict:
    """The tagged entry's id and the time from the connection's last reply to its send,
    copied now so later replies cannot change it"""
    entry = next((c for c in rec.report["commands"] if c.get("tag") == tag), None)
    out = {"command_id": entry["id"] if entry else None,
           "last_reply_utc_before": before[0] if before else None,
           "last_reply_monotonic_before": round(before[1], 6) if before else None}
    if entry and before and "sent_monotonic" in entry:
        out["elapsed_s"] = round(entry["sent_monotonic"] - before[1], 4)
    return out


def _two_connections(rec, fb, dash, dashes, connect, host, port):
    """Control on dashboard-1 after an idle wait, open dashboard-2, probe dashboard-1.
    Returns (the connection to continue on, whether dashboard-1's end was detected)"""
    two = {"control_idle_s": IDLE_PROBE_S, "settle_s": SECOND_OPEN_SETTLE_S, "result": None}
    rec.report["two_connections"] = two
    rec.save()

    # control: does dashboard-1 survive an idle wait with no second connection?
    _wait(fb, IDLE_PROBE_S)
    before = dash.last_reply
    try:
        dash.query(protocol.robot_mode(), tag="control")
    except MG400ConnectionError as e:
        if e.kind in ("eof", "reset"):
            rec.note("dashboard-1 end detected by the control probe after the idle wait, "
                     "before a second connection was opened")
        else:                              # not an end of the connection: say only what it was
            rec.note(f"the control probe on dashboard-1 failed ({e.kind}) after the idle wait, "
                     "before a second connection was opened")
        raise
    finally:
        two["control"] = _probe_timing(rec, "control", before)
    rec.save()

    dash2 = Dashboard(_open(connect, host, port), rec, "dashboard-2", fb.check, port=port)
    dashes.append(dash2)
    two["second_opened_utc"] = dash2.conn["opened_utc"]
    rec.save()
    _wait(fb, SECOND_OPEN_SETTLE_S)

    before = dash.last_reply
    try:
        dash.query(protocol.robot_mode(), tag="probe")
    except BaseException as e:
        two["probe"] = _probe_timing(rec, "probe", before)
        if not _tolerated_drop(e, two["probe"]["command_id"], rec):
            raise
        dash.close()                       # never used again; its end_seen stays as detected
        two["outcome"] = "first connection's end detected after the second opened (this run)"
        two["interleaved"] = two["outstanding"] = "not run: first connection ended"
        two["result"] = two["outcome"]
        rec.save()                         # the drop record must be on disk, or it ends here
        return dash2, True
    two["probe"] = _probe_timing(rec, "probe", before)
    two["outcome"] = "first still answered after the second opened"
    two["interleaved"] = []
    rec.save()

    for cmd in (protocol.robot_mode(), protocol.get_error_id(), protocol.get_pose(),
                protocol.get_angle()):
        for d in (dash, dash2):
            fb.check()
            d.query(cmd)
            two["interleaved"].append({"connection": d.label, "command": cmd.text, "ok": True})
    rec.save()
    fb.check()
    c1 = protocol.robot_mode()
    e1 = dash.write(c1)                      # left outstanding while dashboard-2 answers
    t2 = time.monotonic()
    dash2.query(protocol.get_angle())
    answered = time.monotonic() - t2
    dash.read(c1, e1)
    two["outstanding"] = {"second_answered_s": round(answered, 4)}
    two["result"] = "both answered"
    rec.save()
    return dash, False


def observe(host: str, folder: str, seconds: float = 10.0, controller_version: Optional[str] = None,
            connect: Connector = _connect, dashboard_port: int = DASHBOARD_PORT,
            feedback_port: int = FEEDBACK_PORT, out=print) -> dict:
    """Run the observation. Returns the report (also written to ``folder``). Any save failure
    of the run ends in MG400RecordError, unless another exception is already on its way
    (that one is raised, with the recorder as ``mg400_recorder``)"""
    rec = Recorder(folder)          # MG400RecordError(recorder=rec) here: nothing opened
    rec.report["controller_version_typed"] = controller_version
    dashes: List[Dashboard] = []
    fb: Optional[FeedbackRecorder] = None
    fb_conn: Optional[dict] = None
    ended = None
    error: Optional[BaseException] = None
    try:
        fb = FeedbackRecorder(_open(connect, host, feedback_port), folder)
        fb_conn = {"label": "feedback", "port": feedback_port, "opened_utc": _utc(),
                   "opened_monotonic": round(time.monotonic(), 6)}
        rec.report.setdefault("connections", []).append(fb_conn)
        fb.start()
        fb.wait_first()
        dash = Dashboard(_open(connect, host, dashboard_port), rec, "dashboard-1", fb.check,
                         port=dashboard_port)
        dashes.append(dash)
        _check_state(dash, fb)
        out("state: disabled, no alarm")

        # 1. queries
        r = {}
        r["GetPose()"] = protocol.parse_numbers(dash.query(protocol.get_pose()).values, 4)
        r["GetAngle()"] = protocol.parse_numbers(dash.query(protocol.get_angle()).values, (4, 6))
        rec.report["queries"] = r
        rec.save()

        # 2. a second Dashboard connection (every RobotMode / GetErrorID reply is judged)
        dash, dropped = _two_connections(rec, fb, dash, dashes, connect, host, dashboard_port)
        if dropped:
            out("second Dashboard connection: the first one's end was detected; "
                "continuing on dashboard-2 only")
            _check_state(dash, fb)
            r["dashboard-2 GetPose()"] = protocol.parse_numbers(
                dash.query(protocol.get_pose()).values, 4)
            r["dashboard-2 GetAngle()"] = protocol.parse_numbers(
                dash.query(protocol.get_angle()).values, (4, 6))
            rec.save()
        else:
            out("two Dashboard connections: both answered")

        # 3. feedback window
        out(f"recording feedback for {seconds:g} s ...")
        _wait(fb, seconds)
        _check_state(dash, fb)
        r["GetAngle() after window"] = protocol.parse_numbers(
            dash.query(protocol.get_angle()).values, (4, 6))
        r["GetPose() after window"] = protocol.parse_numbers(
            dash.query(protocol.get_pose()).values, 4)
        with fb.lock:
            last = dict(fb.latest)
        # the packet and the replies are not one snapshot; the frames are the packet's
        rec.report["compare_mapping_unverified"] = {
            "q_actual_minus_GetAngle_first4": [
                round(a - b, 6) for a, b in zip(last["q_actual"][:4], r["GetAngle() after window"][:4])],
            "tool_vector_actual_minus_GetPose_first4": [
                round(a - b, 6) for a, b in zip(last["tool_vector_actual"][:4],
                                                r["GetPose() after window"][:4])],
            "packet_user_index": last["user_index"], "packet_tool_index": last["tool_index"]}
        rec.save()

        # 4. kinematics (compute only); the state is checked before every compute command
        pairs = []
        rec.report["kinematics_note"] = (
            "inverse_solution: six-parameter form, the solution nearest the arm's current joint "
            "angles (not the joints of the pair); a different branch is not an error")
        current = r["GetAngle()"][:4]
        for joints in [current] + list(kinematics_grid()):
            _check_state(dash, fb)
            fk = protocol.parse_numbers(dash.query(protocol.positive_solution(joints)).values, 4)
            _check_state(dash, fb)
            ik = protocol.parse_numbers(dash.query(protocol.inverse_solution(fk)).values, 4)
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
    except ObservationEnded as e:
        ended = f"observation ended: {e}"
        _say(ended, out)
    except BaseException as e:
        ended = f"stopped by {type(e).__name__}: {e}"
        error = e
    finally:
        if ended:
            rec.note(ended)
        for d in dashes:
            d.close()
        if fb is not None:
            fb.stop()
            fb.join(timeout=2.0)
            rec.report["feedback"] = fb.summary()       # the final summary, whole recording
            if fb_conn is not None:
                with fb.lock:
                    fault, end = fb.fault, fb.end or fb.closed
                fb_conn.update(end)         # the receiver's time when it saw the end, or the first stop()
                if fault:
                    fb_conn["fault"] = fault
        rec.report["ended_utc"] = _utc()
        rec.save_during()
    if error is not None:
        error.mg400_recorder = rec
        raise error
    if rec.failed:
        raise MG400RecordError(rec.first_error, recorder=rec)
    return rec.report


def _open(connect: Connector, host: str, port: int) -> socket.socket:
    try:
        return connect(host, port, TIMEOUT_S)
    except MG400Error:
        raise
    except OSError as e:
        raise MG400ConnectionError(f"connecting to {host}:{port}: {e}", label=f"port {port}",
                                   op="connect", kind=_kind(e)) from e


def dry_run(out=print):
    """The command templates in order; connects to nothing"""
    out("feedback port %d: read only, never written; first valid packet within %g s"
        % (FEEDBACK_PORT, FEEDBACK_FIRST_S))
    for line in ("RobotMode()   (end unless 4)", "GetErrorID()  (end on any alarm)",
                 "GetPose()", "GetAngle()",
                 "wait %g s (dashboard-1 idle), then RobotMode() on 1 (control)" % IDLE_PROBE_S,
                 "open dashboard-2, wait %g s, then RobotMode() on 1 (probe)" % SECOND_OPEN_SETTLE_S,
                 "if 1 answered: RobotMode() GetErrorID() GetPose() GetAngle() on both, "
                 "interleaved; RobotMode() on 1 left outstanding while GetAngle() on 2",
                 "if 1's end was detected (end of stream or reset, nothing else): close 1; "
                 "RobotMode() GetErrorID() GetPose() GetAngle() on 2, and 2 from here on",
                 "wait --seconds, recording feedback (end on any fault)",
                 "RobotMode() GetErrorID() GetAngle() GetPose()",
                 "for the current joints and %d grid joint sets: RobotMode() GetErrorID() "
                 "PositiveSolution(j1,j2,j3,j4,0,0) RobotMode() GetErrorID() "
                 "InverseSolution(x,y,z,r,0,0)" % len(list(kinematics_grid())),
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


def _say(text: str, out=print):
    """A notice on a path whose outcome is already decided: a failing stdout must not
    replace the error or the exit code"""
    try:
        out(text)
    except Exception:  # noqa: BLE001
        pass


def _record_also(e: BaseException) -> str:
    """The disk state, read now (after every save), when the record also failed"""
    rec = getattr(e, "recorder", None) or getattr(e, "mg400_recorder", None)
    if rec is None or not rec.failed:
        return ""
    return " (record also failed: %s; %s)" % (rec.first_error, rec.describe())


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
    if "onedrive" in os.path.abspath(folder).lower():
        print(ONEDRIVE_NOTE)
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
    except KeyboardInterrupt as e:
        _say("\nCtrl+C: observation ended. Nothing was sent to the arm; this is not an "
             "emergency stop. Partial record in %s%s" % (folder, _record_also(e)))
        return 130
    except MG400RecordError as e:
        rec = e.recorder
        _say("record error: %s; %s (folder %s)"
             % (rec.first_error if rec else e, rec.describe() if rec else "", folder))
        return 1
    except MG400ConnectionError as e:
        _say("connection error: %s (record in %s)%s" % (e, folder, _record_also(e)))
        return 1
    except (MG400Error, OSError) as e:      # an OSError here is not a connection error
        _say("error: %s (record in %s)%s" % (e, folder, _record_also(e)))
        return 1
    finally:
        if fake is not None:
            fake.close()
    _say("record: %s (complete: %s)" % (folder, report.get("complete")))
    return 0 if report.get("complete") else 2


def _exit(code: int):
    """Exit with ``code`` even if stdout / stderr cannot be flushed: Python would otherwise
    turn a failed flush at shutdown into exit code 120"""
    for name in ("stdout", "stderr"):
        try:
            getattr(sys, name).flush()
        except Exception:  # noqa: BLE001
            setattr(sys, name, None)    # the shutdown flush skips None; nothing needs opening
    sys.exit(code)


if __name__ == "__main__":
    _exit(main())
