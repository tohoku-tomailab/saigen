"""MaxArmClient against the real firmware Server running in-process (loopback).

The fake serial port feeds every written byte into saigen_maxarm.Server and
returns whatever the server writes, so the PC codec and the firmware are
exercised together. A small simulated arm interpolates moves in real time.
No serial port is opened.
"""
import math
import threading
import time

import pytest

from src.devices.maxarm import (
    MaxArmClient,
    MaxArmCommandError,
    MaxArmError,
    MaxArmMoveError,
    MaxArmProtocolError,
    MaxArmResetDetected,
    MaxArmTimeout,
)
from src.devices.maxarm import protocol as P
from src.devices.maxarm.client import MaxArmLinkFault, MaxArmStatus, limits_to_args
from src.devices.maxarm.firmware import saigen_maxarm as fw


def _now_ms():
    return int(time.monotonic() * 1000)


class SimArm:
    def __init__(self):
        self.start = self.target = (0.0, -162.8, 212.0)
        self.t0 = _now_ms()
        self.dur = 1
        self.offset = (0.0, 0.0, 0.0)
        self.moves = []

    def current(self):
        frac = min(1.0, max(0.0, (_now_ms() - self.t0) / float(self.dur)))
        return tuple(s + (t - s) * frac for s, t in zip(self.start, self.target))

    def set_position(self, pos, ms):
        self.moves.append((tuple(pos), ms))
        self.start = self.current()
        self.target = tuple(float(v) for v in pos)
        self.t0 = _now_ms()
        self.dur = max(1, ms)
        return True

    def freeze(self):
        self.start = self.target = self.current()
        self.dur = 1

    def read_position(self):
        c = self.current()
        return tuple(int(round(v + o)) for v, o in zip(c, self.offset))


class SimBus:
    def __init__(self, arm):
        self.arm = arm
        self.stops = 0
        self.stop_failures = 0         # the next N stop() calls raise (servo not stopped)

    def stop(self, i):
        self.stops += 1
        if self.stop_failures > 0:
            self.stop_failures -= 1
            raise OSError("uart")
        self.arm.freeze()

    def load(self, i):
        pass

    def unload(self, i):
        pass

    def get_position(self, i):
        return 500

    def get_vin(self, i):
        return 12000


class SimNozzle:
    def __init__(self):
        self.on_calls = 0

    def on(self):
        self.on_calls += 1

    def off(self):
        pass


class LoopbackSerial:
    """serial.Serial stand-in wired to an in-process firmware Server."""

    instances = []                 # every port's ticker is stopped after each test

    def __init__(self, banner=b"", silent=False):
        LoopbackSerial.instances.append(self)
        self.arm = SimArm()
        self.bus = SimBus(self.arm)
        self.nozzle = SimNozzle()
        self.log = []                  # ("rts"/"dtr"/"open"/"close", value, is_open)
        self.requests = []             # request bodies as received
        self.is_open = False
        self.banner = banner
        self.silent = silent
        self.before_reply = None       # hook(body) -> list of extra raw lines to emit first
        self.port = self.baudrate = self.bytesize = self.parity = self.stopbits = None
        self.timeout = 0.05
        self.write_timeout = None
        self._rts = self._dtr = None
        self._rx = bytearray()
        self._cv = threading.Condition()
        self._pending = bytearray()
        self.server = fw.Server(self.arm, self.bus, self.nozzle, self._server_write,
                                _now_ms, lambda a, b: a - b, sys_version="sim")
        self._alive = True
        self._ticker = threading.Thread(target=self._tick_loop, daemon=True)
        self._ticker.start()

    # --- device side
    def _server_write(self, text):
        self._rx += text.encode("ascii")

    def _tick_loop(self):
        while self._alive:
            with self._cv:
                self.server.tick()
                self._cv.notify_all()
            time.sleep(0.005)

    def emit(self, raw):
        with self._cv:
            self._rx += raw
            self._cv.notify_all()

    # --- pyserial API
    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, v):
        self.log.append(("rts", v, self.is_open))
        self._rts = v

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, v):
        self.log.append(("dtr", v, self.is_open))
        self._dtr = v

    def open(self):
        self.log.append(("open", None, self.is_open))
        self.is_open = True
        self.emit(self.banner)

    def close(self):
        self.log.append(("close", None, self.is_open))
        self.is_open = False
        self._alive = False

    @property
    def in_waiting(self):
        return len(self._rx)

    def write(self, data):
        assert self.is_open
        with self._cv:
            for b in data:
                if b == 10:
                    body = bytes(self._pending).decode()
                    self._pending = bytearray()
                    self.requests.append(body.split("*")[0])
                    if self.silent:
                        continue
                    if self.before_reply:
                        for extra in self.before_reply(body) or ():
                            self._rx += extra
                    self.server.feed(b)
                else:
                    self._pending.append(b)
                    if not self.silent:
                        self.server.feed(b)
            self._cv.notify_all()
        return len(data)

    def read(self, n=1):
        deadline = time.monotonic() + (self.timeout or 0)
        with self._cv:
            while not self._rx and time.monotonic() < deadline:
                self._cv.wait(deadline - time.monotonic())
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    # --- helpers
    def commands(self):
        return [r.split(" ")[1] for r in self.requests if " " in r]

    def stop_ticker(self):
        self._alive = False
        self._ticker.join(1.0)


@pytest.fixture(autouse=True)
def _stop_loopback_tickers():
    """Ports a test never closed would keep ticking (every 5 ms) for the rest of the
    run; dozens of them starve later tests of CPU and make timing tests flaky."""
    yield
    while _CLIENTS:
        c = _CLIENTS.pop()
        c._hb_stop.set()                  # an unclosed client's heartbeat thread ends too
        c._join_heartbeat()
    while LoopbackSerial.instances:
        LoopbackSerial.instances.pop().stop_ticker()


_CLIENTS = []


def make_client(port=None, **kw):
    port = port or LoopbackSerial()
    kw.setdefault("heartbeat_period", 0.05)
    c = MaxArmClient("COMX", serial_factory=lambda: port, **kw)
    _CLIENTS.append(c)
    return c, port


LIMITS = {"xmin": -250, "xmax": 250, "ymin": -280, "ymax": -60, "zmin": 40, "zmax": 225,
          "rmin": 60, "rmax": 260}

WAIT = 10.0          # generous upper bound for anything that waits on a thread


def _wait_for(pred, timeout=WAIT):
    """Poll ``pred`` until true; the timeout is only a bound, never the expectation."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def _always_moving(port):
    """STATUS keeps reporting moving=1 (a move that never finishes)."""
    real = port.server.commands["STATUS"]
    port.server.commands["STATUS"] = lambda args: real(args).replace("moving=0", "moving=1")


@pytest.fixture
def client():
    c, port = make_client()
    c.connect(limits=LIMITS, watchdog_ms=1000)
    yield c, port
    c.close()


# ----------------------------------------------------------------------
# connect / close
# ----------------------------------------------------------------------
def test_connect_happy_path_sets_lines_low_before_open(client):
    c, port = client
    assert c.connected
    assert port.log[:3] == [("rts", False, False), ("dtr", False, False), ("open", None, False)]
    assert c.info["fw"] == "saigen-maxarm" and c.info["proto"] == "1"
    assert c.info["ver"] == fw.FIRMWARE_VERSION
    cmds = port.commands()
    assert cmds[:4] == ["PING", "INFO", "LIMITS", "WATCHDOG"]
    assert "CLEAR" not in cmds, "the stop latch is the safety wrapper's decision"
    assert port.server.limits == (-250, 250, -280, -60, 40, 225, 60, 260)
    assert port.server.wd_ms == 1000
    assert port.bus.stops == 0 and port.arm.moves == []


def test_close_sends_bye_then_rts_then_dtr_then_close():
    c, port = make_client()
    c.connect()
    c.close()
    assert port.commands()[-1] == "BYE"
    tail = [e[0] for e in port.log[-3:]]
    assert tail == ["rts", "dtr", "close"]
    assert port.log[-3][1] is False and port.log[-2][1] is False
    assert not c.connected
    assert port.server.wd_ms == 0


@pytest.mark.parametrize("banner", [
    b"ets Jun  8 2016 00:22:57\r\n\r\nrst:0x1 (POWERON_RESET),boot:0x13\r\n",
    b"MicroPython v1.19.1 on 2022-06-18; ESP32 module with ESP32\r\n",
])
def test_reset_banner_is_detected(banner):
    c, port = make_client(LoopbackSerial(banner=banner))
    with pytest.raises(MaxArmResetDetected):
        c.connect()
    assert not c.connected
    assert port.log[-1][0] == "close"
    assert port.requests == [], "nothing is sent to a board that just reset"


def test_wrong_firmware_is_refused(monkeypatch):
    monkeypatch.setattr(fw, "FIRMWARE_NAME", "someone-else")
    c, port = make_client()
    with pytest.raises(MaxArmProtocolError, match="unexpected firmware"):
        c.connect()
    assert not c.connected and port.log[-1][0] == "close"


def test_silent_device_times_out_after_three_pings():
    c, port = make_client(LoopbackSerial(silent=True), timeout=0.1)
    with pytest.raises(MaxArmTimeout, match="PING"):
        c.connect()
    assert port.commands() == ["PING", "PING", "PING"]
    assert port.log[-1][0] == "close"


def test_friendly_repl_is_reported():
    port = LoopbackSerial(silent=True)

    def repl(body):
        return [body.encode() + b"\r\n", b"SyntaxError: invalid syntax\r\n", b">>> "]
    port.silent = False
    port.before_reply = repl
    port.server.handle_line = lambda line: None       # the REPL does not answer
    c, _ = make_client(port, timeout=0.2)
    with pytest.raises(MaxArmProtocolError, match="REPL"):
        c.connect()


# ----------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------
def test_status_ping_pump_torque(client):
    c, port = client
    assert c.ping() is True
    st = c.status()
    assert st.position == [0.0, -163.0, 212.0]
    assert st.pulses == (500, 500, 500) and st.vin_mv == 12000
    assert (st.moving, st.stopped, st.pump) == (False, False, False)
    c.pump(True)
    assert c.status().pump is True and port.nozzle.on_calls == 1
    c.torque(False)
    c.torque(True)
    c.set_limits(None)
    assert port.server.limits is None
    c.set_watchdog(0)
    c.heartbeat()


def test_move_xyz_and_wait_completes(client):
    c, port = client
    t0 = time.monotonic()
    st = c.move_xyz_and_wait(0, -200, 150, 300)
    assert time.monotonic() - t0 >= 0.25
    assert (st.x, st.y, st.z) == (0, -200, 150) and not st.moving
    assert port.arm.moves == [((0, -200, 150), 300)]


def test_move_xyz_and_wait_times_out(client):
    c, port = client
    _always_moving(port)
    t0 = time.monotonic()
    with pytest.raises(MaxArmTimeout):
        c.move_xyz_and_wait(0, -200, 150, 200, timeout_margin=0)
    assert time.monotonic() - t0 >= 0.2
    assert c.motion_uncertain


def test_move_xyz_and_wait_tolerance_error(client):
    c, port = client
    port.arm.offset = (0.0, 0.0, -15.0)       # the arm sags 15 mm
    with pytest.raises(MaxArmMoveError) as ei:
        c.move_xyz_and_wait(0, -200, 150, 200, tolerance_mm=10.0)
    assert ei.value.status is not None and ei.value.status.z == 135


def test_device_side_errors_raise_command_error(client):
    c, _port = client
    with pytest.raises(MaxArmCommandError) as ei:
        c.move_xyz(300, -200, 150, 500)            # outside the device LIMITS box
    assert ei.value.code == "LIMIT"
    with pytest.raises(ValueError):
        c.move_xyz(0, -200, 150, 50)
    with pytest.raises(ValueError):
        c.move_xyz(float("nan"), -200, 150, 500)


def test_stop_from_another_thread_interrupts_a_move(client):
    c, port = client
    result = {}

    def worker():
        try:
            c.move_xyz_and_wait(0, -250, 100, 3000)
        except BaseException as e:     # noqa: BLE001
            result["exc"] = e
        result["t"] = time.monotonic()

    th = threading.Thread(target=worker)
    th.start()
    assert _wait_for(lambda: port.arm.moves), "the MOVE reached the device"
    time.sleep(0.1)
    t_stop = time.monotonic()
    assert c.force_stop() is True
    th.join(WAIT)
    assert not th.is_alive()
    assert isinstance(result["exc"], MaxArmMoveError)
    assert result["t"] - t_stop < 1.0, "the waiting move must return promptly after STOP"
    assert port.bus.stops == 3
    st = c.status()
    assert st.stopped and not st.moving
    assert st.y > -250 + 5, "the arm halted part-way"
    with pytest.raises(MaxArmCommandError) as ei:
        c.move_xyz(0, -200, 150, 500)
    assert ei.value.code == "STOPPED"
    c.clear()
    c.move_xyz_and_wait(0, -200, 150, 200)


def test_stop_latched_on_device_ends_the_wait(client):
    c, port = client
    result = {}

    def worker():
        try:
            c.move_xyz_and_wait(0, -250, 100, 3000)
        except BaseException as e:     # noqa: BLE001
            result["exc"] = e

    th = threading.Thread(target=worker)
    th.start()
    assert _wait_for(lambda: port.arm.moves), "the MOVE reached the device"
    with port._cv:                        # e.g. the device watchdog fired
        port.server.safe_stop()
    th.join(WAIT)
    assert not th.is_alive()
    assert isinstance(result.get("exc"), MaxArmMoveError)


def test_force_stop_returns_false_without_ack():
    c, port = make_client(timeout=0.1)
    c.connect()
    port.silent = True
    assert c.force_stop() is False
    c.close()


def test_events_are_tolerated_mid_transaction(client):
    c, port = client

    def evt(body):
        if " STATUS" in body:
            return [P.encode_line("EVT NOTE text=hello")]
        return []
    port.before_reply = evt
    st = c.status()
    assert st.x == 0
    assert any(e.code == "NOTE" for e in c.events)


def test_stale_seq_and_garbage_are_dropped(client):
    c, port = client

    def stale(body):
        if " PING" in body:
            return [P.encode_line("999 OK pong=0"), b"\x00\xffnoise\r\n", b"1 OK*00\n"]
        return []
    port.before_reply = stale
    assert c.ping() is True


def test_ready_event_after_connect_means_reset(client):
    c, port = client
    port.before_reply = lambda body: [P.encode_line("EVT READY fw=saigen-maxarm proto=1 ver=0.1.0")]
    with pytest.raises(MaxArmResetDetected):
        c.ping()
    assert not c.connected


def test_sequence_numbers_wrap():
    c, port = make_client()
    c.connect()
    c._seq = P.SEQ_MAX - 1
    c.ping()
    c.ping()
    assert port.requests[-2].split(" ")[0] == str(P.SEQ_MAX)
    assert port.requests[-1].split(" ")[0] == "1"
    c.close()


# ----------------------------------------------------------------------
# Heartbeat and device watchdog
# ----------------------------------------------------------------------
def test_heartbeat_only_when_idle():
    # period 0.3 s against pings every ~10 ms: an HB in the busy phase would need a
    # 300 ms stall of this thread, so the check is robust under a loaded test run
    c, port = make_client(heartbeat_period=0.3)
    c.connect(limits=LIMITS, watchdog_ms=2000)
    assert _wait_for(lambda: port.commands().count("HB") >= 2), "HB while idle"
    n_before = len(port.requests)
    t_end = time.monotonic() + 0.4
    while time.monotonic() < t_end:
        c.ping()
        time.sleep(0.01)
    busy = port.commands()[n_before:]
    assert busy and "HB" not in busy
    assert port.server.stopped is False
    c.close()


def test_device_watchdog_stops_the_arm_when_the_host_goes_silent():
    c, port = make_client(heartbeat_period=0.05, timeout=0.1)
    c.connect(watchdog_ms=200)
    c._hb_stop.set()                              # simulate a hung host process
    c._join_heartbeat()
    assert _wait_for(lambda: port.server.stopped is True and port.server.wd_ms == 0)
    with pytest.raises(MaxArmCommandError):
        c.move_xyz(0, -200, 150, 500)
    assert any(e.code == "STOPPED" and e.fields.get("reason") == "watchdog" for e in c.events)
    # the watchdog disabled itself: this connection is finished (P4)
    assert c.link_fault and "watchdog" in c.link_fault
    n = len(port.requests)
    with pytest.raises(MaxArmLinkFault):
        c.clear()
    with pytest.raises(MaxArmLinkFault):
        c.move_xyz(0, -200, 150, 500)
    assert len(port.requests) == n, "refused commands are never sent"
    assert c.force_stop() is True, "STOP is always allowed"
    c.close()
    assert "BYE" not in port.commands()[n:]


# ----------------------------------------------------------------------
# limits_to_args
# ----------------------------------------------------------------------
def test_limits_to_args_forms():
    flat = {"xmin": -1, "xmax": 1, "ymin": -2, "ymax": 2, "zmin": 3, "zmax": 4}
    assert limits_to_args(flat) == [-1, 1, -2, 2, 3, 4]
    assert limits_to_args({**flat, "rmin": 60}) == [-1, 1, -2, 2, 3, 4, 60, 100000]
    nested = {"x": {"min": -1, "max": 1}, "y": {"min": -2, "max": 2}, "z": {"min": 3, "max": 4},
              "joint1": {"min": -90, "max": 90}, "r": {"max": 260}}
    assert limits_to_args(nested) == [-1, 1, -2, 2, 3, 4, 0, 260]
    assert limits_to_args({"x_min": -1, "x_max": 1, "y_min": -2, "y_max": 2,
                           "z_min": 3, "z_max": 4}) == [-1, 1, -2, 2, 3, 4]
    with pytest.raises(ValueError):
        limits_to_args({"xmin": 0})
    with pytest.raises(ValueError):
        limits_to_args({**flat, "zmax": float("inf")})


# ----------------------------------------------------------------------
# Review fixes (codex_review_maxarm_device.md D1, D3, D7, D8, D10, D12; P4)
# ----------------------------------------------------------------------
def _drop_replies(port, match=None):
    """Device -> host direction dead (all replies, or those containing ``match``)."""
    orig = port.server.write

    def write(text):
        if match is None or match in text:
            return
        orig(text)
    port.server.write = write


def test_d1_lost_replies_latch_a_link_fault_and_stop_feeding_the_watchdog():
    c, port = make_client(heartbeat_period=0.05, timeout=0.1)
    c.connect(limits=LIMITS, watchdog_ms=300)
    _drop_replies(port)                                 # requests still reach the device
    assert _wait_for(lambda: c.link_fault is not None)
    # the link fault is latched before its STOP is written: wait for the STOP itself
    assert _wait_for(lambda: "STOP" in port.commands() and port.server.stopped)
    assert "heartbeat" in c.link_fault
    assert c.connected is False and c.fault_reason == c.link_fault
    cmds = port.commands()
    last_stop = len(cmds) - 1 - cmds[::-1].index("STOP")
    assert cmds[last_stop - 2:last_stop] == ["HB", "HB"], "two failed HB, then one STOP"
    assert port.server.stopped is True, "the STOP reached the device"
    assert _wait_for(lambda: port.server.wd_ms == 0), "the device watchdog fired (no longer fed)"
    c._join_heartbeat()                                 # the worker has ended ...
    assert "HB" not in port.commands()[last_stop:], "no HB after the link fault"
    with pytest.raises(MaxArmLinkFault):
        c.status()
    with pytest.raises(MaxArmLinkFault):
        c.move_xyz(0, -200, 150, 500)
    assert port.commands()[-1] == "STOP"
    c.close()
    assert "BYE" not in port.commands()[last_stop:]


def test_d1_a_single_failed_heartbeat_is_tolerated():
    # timeout 0.3 s: only the deliberately dropped reply may fail, not a slow one
    c, port = make_client(heartbeat_period=0.05, timeout=0.3)
    c.connect(watchdog_ms=1000)
    orig = port.server.write
    dropped = []

    def drop_one_hb(text):
        if not dropped and " OK wd=" in text:
            dropped.append(text)
            return
        orig(text)
    port.server.write = drop_one_hb
    assert _wait_for(lambda: dropped and port.commands().count("HB") >= 4)
    assert c.link_fault is None and c.ping() is True
    c.close()


def test_p4_watchdog_must_outlast_heartbeat_and_timeout():
    port = LoopbackSerial()
    c = MaxArmClient("COMX", serial_factory=lambda: port)        # hb 0.25 s, timeout 0.5 s
    assert c.min_watchdog_ms() == 1000
    for wd in (100, 999, 60001, 1.5, True):
        with pytest.raises(ValueError):
            c.connect(watchdog_ms=wd)
    assert port.log == [], "nothing is opened for a bad watchdog"
    c.connect(watchdog_ms=1000)
    assert port.server.wd_ms == 1000
    with pytest.raises(ValueError):
        c.set_watchdog(500)
    c.close()


def test_p4_clear_rearms_and_verifies_the_watchdog(client):
    c, port = client
    c.stop()
    port.server.wd_ms = 0                     # e.g. a watchdog that fired, event lost
    c.clear()
    cmds = port.commands()
    assert cmds[-2:] == ["CLEAR", "WATCHDOG"]
    assert port.server.wd_ms == 1000 and not port.server.stopped
    c.move_xyz_and_wait(0, -200, 150, 200)


def test_p4_unconfirmed_rearm_is_a_link_fault(client):
    c, port = client
    c.stop()
    port.server.commands["WATCHDOG"] = lambda args: "wd=0"
    with pytest.raises(MaxArmLinkFault):
        c.clear()
    assert c.link_fault and "WATCHDOG" in c.link_fault
    assert port.commands()[-1] == "STOP"
    with pytest.raises(MaxArmLinkFault):
        c.move_xyz(0, -200, 150, 500)


def _drive_heartbeat_loop(c):
    """Run the heartbeat loop of the current connection in a thread and wait for it.

    The client's own heartbeat thread is stopped first, so exactly this loop
    sends HB; the loop returns on its own once it latched a fault.
    """
    c._hb_stop.set()
    c._join_heartbeat()
    c._last_tx = time.monotonic() - 10.0      # idle for long: the first pass sends HB
    th = threading.Thread(target=c._heartbeat_loop, args=(c._conn_gen, threading.Event()),
                          daemon=True)
    th.start()
    th.join(WAIT)
    assert not th.is_alive(), "the heartbeat loop must end after a link fault"


def test_p4_wd0_in_a_heartbeat_reply_is_a_link_fault():
    # deterministic: the heartbeat loop is driven explicitly (it used to race the
    # assertion on the STOP that follows the latched fault under full-suite load)
    c, port = make_client(heartbeat_period=0.05)
    c.connect(watchdog_ms=1000)
    with port._cv:
        port.server.wd_ms = 0                # watchdog disarmed behind our back
    _drive_heartbeat_loop(c)
    assert c.link_fault and "wd=0" in c.link_fault
    assert port.commands()[-2:] == ["HB", "STOP"], "one HB, then one STOP, then no HB"
    with pytest.raises(MaxArmLinkFault):
        c.ping()
    c.close()


def test_p4_wd0_in_an_explicit_heartbeat_reply_raises_after_stop():
    c, port = make_client(heartbeat_period=None)
    c.connect(watchdog_ms=1000)
    with port._cv:
        port.server.wd_ms = 0
    with pytest.raises(MaxArmLinkFault, match="wd=0"):
        c.heartbeat()
    assert port.commands()[-2:] == ["HB", "STOP"], "STOP is sent before the error is raised"
    c.close()


def test_d8_reset_seen_by_the_heartbeat_blocks_every_command_but_stop():
    ports = []

    def factory():
        ports.append(LoopbackSerial())
        return ports[-1]
    c = MaxArmClient("COMX", serial_factory=factory, heartbeat_period=0.05)
    c.connect(limits=LIMITS, watchdog_ms=1000)
    old = ports[0]
    old.emit(P.encode_line("EVT READY fw=saigen-maxarm proto=1 ver=0.1.0"))
    with old._cv:
        old.server.limits = None                          # what a real reboot does
        old.server.wd_ms = 0
    # the heartbeat thread records the reason and then drops `connected`: wait for both
    assert _wait_for(lambda: c.fault_reason is not None and not c.connected)
    assert c.connected is False and "EVT READY" in c.fault_reason
    n = len(old.requests)
    for fn in (lambda: c.move_xyz(80, -200, 150, 300), c.status, c.clear,
               lambda: c.pump(True), c.ping):
        with pytest.raises(MaxArmError):
            fn()
    assert len(old.requests) == n and old.arm.moves == [], "nothing reached the rebooted board"
    assert c.force_stop() is True, "STOP is always allowed"
    # the only way forward: connect() again -> old port closed first, full handshake
    c.connect(limits=LIMITS, watchdog_ms=1000)
    assert old.log[-1][0] == "close" and not old.is_open
    new = ports[1]
    assert new.commands()[:4] == ["PING", "INFO", "LIMITS", "WATCHDOG"]
    assert new.server.limits is not None and new.server.wd_ms == 1000
    assert c.fault_reason is None and c.connected
    c.close()


def test_d8_firmware_fault_event_blocks_commands(client):
    c, port = client
    port.before_reply = lambda body: [P.encode_line("EVT FAULT type=OSError msg=x")]
    with pytest.raises(MaxArmProtocolError):
        c.ping()
    port.before_reply = None
    with pytest.raises(MaxArmError):
        c.status()
    assert c.fault_reason and "FAULT" in c.fault_reason


def test_d8_connect_closes_a_port_left_open():
    ports = []

    def factory():
        ports.append(LoopbackSerial())
        return ports[-1]
    c = MaxArmClient("COMX", serial_factory=factory, heartbeat_period=0.05)
    c.connect(watchdog_ms=1000)
    c._set_fault("test")                                # e.g. reset detected
    c.connect(watchdog_ms=1000)
    assert [e[0] for e in ports[0].log[-3:]] == ["rts", "dtr", "close"]
    assert ports[1].is_open and c.connected
    c.close()


def test_d10_stale_heartbeat_worker_cannot_use_a_new_port():
    ports = []

    def factory():
        ports.append(LoopbackSerial())
        return ports[-1]
    c = MaxArmClient("COMX", serial_factory=factory, heartbeat_period=0.05)
    c.connect(watchdog_ms=1000)
    old_gen, old_thread, old_event = c._conn_gen, c._hb_thread, c._hb_stop
    c._set_fault("test")
    c.connect(watchdog_ms=1000)
    assert old_event.is_set() and c._hb_stop is not old_event
    assert not old_thread.is_alive()
    n = len(ports[1].requests)
    with pytest.raises(MaxArmError, match="古い接続"):
        c._transact(P.CMD_HB, _gen=old_gen)
    assert len(ports[1].requests) == n
    c.close()
    with pytest.raises(MaxArmError):
        c.ping()


def test_d10_close_waits_for_an_active_transaction():
    c, port = make_client()
    c.connect()
    c._lock.acquire()
    th = threading.Thread(target=c.close)
    th.start()
    time.sleep(0.1)
    assert th.is_alive() and c.ser is not None, "the port is not pulled from under a transaction"
    with pytest.raises(MaxArmError):
        c._gate(P.CMD_PING, None)                         # closing: new commands refused
    c._lock.release()
    th.join(2.0)
    assert not th.is_alive() and c.ser is None


def test_d3_close_after_an_unconfirmed_move_sends_stop_before_bye():
    c, port = make_client(timeout=0.2)
    c.connect(limits=LIMITS)
    _drop_replies(port, match="ms=")                     # MOVE executes, its ack is lost
    with pytest.raises(MaxArmTimeout):
        c.move_xyz_and_wait(80, -200, 150, 3000)
    assert c.motion_uncertain and port.server._moving()
    port.server.write = port._server_write
    c.close()
    assert port.commands()[-2:] == ["STOP", "BYE"]
    assert not port.server._moving() and port.server.stopped
    assert port.nozzle.on_calls == 0


def test_d3_close_skips_bye_when_stop_is_not_acknowledged():
    c, port = make_client(timeout=0.1)
    c.connect(limits=LIMITS, watchdog_ms=1000)
    c.move_xyz(80, -200, 150, 3000)                      # open loop: not confirmed
    assert c.motion_uncertain
    port.silent = True
    c.close()
    assert port.commands()[-1] == "STOP"
    assert port.server.wd_ms == 1000, "watchdog stays armed so the device stops the arm"


def test_d3_confirmed_move_or_acknowledged_stop_clears_the_uncertainty(client):
    c, port = client
    c.move_xyz_and_wait(0, -200, 150, 200)
    assert not c.motion_uncertain
    c.move_xyz(0, -210, 150, 2000)
    assert c.motion_uncertain
    assert c.force_stop() is True
    assert not c.motion_uncertain


GOOD_STATUS = dict(x="0", y="-200", z="150", p1="1", p2="2", p3="3", moving="0",
                   stopped="0", pump="1", vin="-1")


@pytest.mark.parametrize("key,value", [
    ("x", "nan"), ("y", "inf"), ("z", "-inf"), ("x", "1e999"), ("moving", "2"),
    ("stopped", "x"), ("pump", ""), ("moving", "true"), ("p1", "1.5"), ("vin", "nan"),
])
def test_d12_status_parsing_is_strict(key, value):
    with pytest.raises(MaxArmProtocolError):
        MaxArmStatus.from_fields(dict(GOOD_STATUS, **{key: value}))
    with pytest.raises(MaxArmProtocolError):
        MaxArmStatus.from_fields({k: v for k, v in GOOD_STATUS.items() if k != key})
    st = MaxArmStatus.from_fields(GOOD_STATUS)
    assert (st.moving, st.stopped, st.pump, st.vin_mv) == (False, False, True, None)


@pytest.mark.parametrize("tol", [float("nan"), float("inf"), 0, -1.0])
def test_d12_tolerance_must_be_finite_and_positive(client, tol):
    c, port = client
    n = len(port.requests)
    with pytest.raises(ValueError):
        c.move_xyz_and_wait(0, -200, 150, 200, tolerance_mm=tol)
    assert len(port.requests) == n, "no MOVE is sent"


def test_d12_nan_readback_fails_the_move(client):
    c, port = client
    real = port.server.commands["STATUS"]

    def nan_status(args):
        return real(args).replace("x=0 ", "x=nan ", 1)
    port.server.commands["STATUS"] = nan_status
    with pytest.raises(MaxArmProtocolError):
        c.move_xyz_and_wait(0, -200, 150, 200)
    assert c.motion_uncertain, "an invalid STATUS leaves the motion uncertain"


def test_pump_busy_window_is_reported(client):
    c, port = client
    c.pump(True)
    c.pump(False)
    with pytest.raises(MaxArmCommandError) as ei:
        c.pump(True)
    assert ei.value.code == "BUSY"
    assert port.nozzle.on_calls == 1


# ----------------------------------------------------------------------
# Round-2 review fixes (codex_review_maxarm_device2.md 2, 7, 10, 16)
# ----------------------------------------------------------------------
def _reset_before_watchdog(port, with_banner):
    """The board reboots between LIMITS and WATCHDOG of the handshake."""
    state = {"done": False}

    def hook(body):
        if " WATCHDOG " in body and not state["done"]:
            state["done"] = True
            port.server.limits = None             # a reboot forgets LIMITS
            port.server.wd_ms = 0
            extra = []
            if with_banner:
                extra.append(b"ets Jun  8 2016 00:22:57\r\n")
                extra.append(b"rst:0x1 (POWERON_RESET),boot:0x13\r\n")
            extra.append(fw.frame("EVT READY fw=saigen-maxarm proto=1 ver=0.1.0").encode())
            return extra
        return ()
    port.before_reply = hook


@pytest.mark.parametrize("with_banner", [True, False])
def test_r2_reset_during_the_handshake_fails_connect(with_banner):
    port = LoopbackSerial()
    _reset_before_watchdog(port, with_banner)
    c, _ = make_client(port)
    with pytest.raises(MaxArmResetDetected):
        c.connect(limits=LIMITS, watchdog_ms=1000)
    assert not c.connected and not port.is_open
    assert c.fault_reason
    if not with_banner:
        assert "EVT READY" in c.fault_reason and "handshake" in c.fault_reason


def test_r2_silent_limits_loss_during_the_handshake_is_caught_by_the_info_recheck():
    port = LoopbackSerial()

    def hook(body):                     # configuration lost without any event at all
        if " WATCHDOG " in body:
            port.server.limits = None
        return ()
    port.before_reply = hook
    c, _ = make_client(port)
    with pytest.raises(MaxArmProtocolError, match="not confirmed"):
        c.connect(limits=LIMITS, watchdog_ms=1000)
    assert not c.connected and not port.is_open
    assert port.commands()[:5] == ["PING", "INFO", "LIMITS", "WATCHDOG", "INFO"]


def test_r2_silent_watchdog_loss_during_the_handshake_fails_connect():
    port = LoopbackSerial()

    def hook(body):
        if " INFO" in body and "WATCHDOG" in port.commands():
            port.server.wd_ms = 0            # reset after WATCHDOG, before the re-check
        return ()
    port.before_reply = hook
    c, _ = make_client(port)
    with pytest.raises((MaxArmProtocolError, MaxArmLinkFault)):
        c.connect(limits=LIMITS, watchdog_ms=1000)
    assert not c.connected and not port.is_open


def test_r2_boot_announcement_before_connect_is_drained_not_a_reset():
    banner = fw.frame("EVT READY fw=saigen-maxarm proto=1 ver=0.1.0").encode()
    c, port = make_client(LoopbackSerial(banner=banner))
    c.connect(limits=LIMITS, watchdog_ms=1000)
    assert c.connected and c.fault_reason is None
    assert c.info["limits"] == "1" and c.info["wd"] == "1000"
    c.close()


def test_r2_info_after_connect_reflects_the_device_configuration(client):
    c, port = client
    assert port.commands()[:5] == ["PING", "INFO", "LIMITS", "WATCHDOG", "INFO"]
    assert (c.info["limits"], c.info["wd"], c.info["fw"]) == ("1", "1000", "saigen-maxarm")


@pytest.mark.parametrize("kw", [
    {"timeout": math.inf}, {"timeout": math.nan}, {"timeout": 0}, {"timeout": -1},
    {"timeout": True}, {"connect_timeout": math.inf}, {"connect_timeout": math.nan},
    {"connect_timeout": 0}, {"heartbeat_period": math.inf}, {"heartbeat_period": math.nan},
    {"heartbeat_period": 0}, {"heartbeat_period": -0.1}, {"timeout": "fast"},
])
def test_r16_non_finite_or_non_positive_timing_is_refused_at_construction(kw):
    port = LoopbackSerial()
    with pytest.raises(ValueError):
        MaxArmClient("COMX", serial_factory=lambda: port, **kw)
    assert port.log == [], "nothing is opened"


def test_r16_heartbeat_none_disables_the_thread():
    c, port = make_client(heartbeat_period=None)
    assert c.min_watchdog_ms() == 100
    c.connect(watchdog_ms=0)
    assert c._hb_thread is None
    c.close()


@pytest.mark.parametrize("margin", [-5, -0.001, math.nan, math.inf])
def test_r16_timeout_margin_must_be_finite_and_non_negative(client, margin):
    c, port = client
    n = len(port.requests)
    with pytest.raises(ValueError):
        c.move_xyz_and_wait(50, -200, 150, 1000, timeout_margin=margin)
    assert len(port.requests) == n and port.arm.moves == [], "no MOVE is sent"
    assert not c.motion_uncertain


def _fire_watchdog_with_failing_stops(c, port, failures):
    port.bus.stop_failures = failures
    c._hb_stop.set()                             # host goes silent: the device watchdog fires
    c._join_heartbeat()
    assert _wait_for(lambda: port.server.wd_ms == 0)


def test_r7_unconfirmed_watchdog_stop_is_retried_once_after_the_transaction():
    c, port = make_client(timeout=0.2)
    c.connect(limits=LIMITS, watchdog_ms=300)
    c.move_xyz(50, -200, 150, 3000)
    _fire_watchdog_with_failing_stops(c, port, failures=3)
    assert port.arm.current() != port.arm.target, "the servo stops raised: still moving"
    n = len(port.requests)
    c.status()                                   # reads EVT STOPPED ... servo_ok=0
    evt = [e for e in c.events if e.code == "STOPPED"][-1]
    assert evt.fields["servo_ok"] == "0" and c.link_fault
    assert port.commands()[n:] == ["STATUS", "STOP"], "exactly one STOP retry, after the lock"
    assert port.server.stopped and port.arm.current() == port.arm.target, "the retry froze it"
    assert not c.motion_uncertain
    with pytest.raises(MaxArmLinkFault):
        c.status()
    c.close()


def test_r7_confirmed_watchdog_stop_is_not_retried():
    c, port = make_client(timeout=0.2)
    c.connect(limits=LIMITS, watchdog_ms=300)
    _fire_watchdog_with_failing_stops(c, port, failures=0)
    n = len(port.requests)
    assert c.force_stop() is True                 # reads EVT STOPPED ... servo_ok=1
    assert [e for e in c.events if e.code == "STOPPED"][-1].fields["servo_ok"] == "1"
    assert port.commands()[n:] == ["STOP"]
    with pytest.raises(MaxArmLinkFault):
        c.ping()
    assert port.commands()[n:] == ["STOP"], "no retry, and the link fault refuses PING"
    c.close()


def test_r7_stopped_event_without_servo_ok_counts_as_unconfirmed(client):
    c, port = client
    fired = []

    def evt(body):
        if " PING" in body and not fired:
            fired.append(1)
            return [P.encode_line("EVT STOPPED reason=other")]
        return []
    port.before_reply = evt
    n = len(port.requests)
    assert c.ping() is True
    assert port.commands()[n:] == ["PING", "STOP"]


class _RecordingLock:
    """threading.Lock stand-in that records the client's watchdog at every release."""

    def __init__(self, client):
        self._lock = threading.Lock()
        self.client = client
        self.at_release = []

    def acquire(self, *a, **kw):
        return self._lock.acquire(*a, **kw)

    def release(self):
        self.at_release.append(self.client._watchdog_ms)
        self._lock.release()

    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, *exc):
        self.release()


def test_r10_watchdog_expectation_is_updated_under_the_transport_lock():
    c, port = make_client(heartbeat_period=None)
    c.connect(limits=LIMITS, watchdog_ms=1000)
    rec = c._lock = _RecordingLock(c)
    c.set_watchdog(2000)
    assert rec.at_release[-1] == 2000, "updated before the WATCHDOG transaction released the lock"
    c.set_watchdog(0)
    assert rec.at_release[-1] == 0 and port.server.wd_ms == 0
    port.server.commands["WATCHDOG"] = lambda args: "wd=5"
    with pytest.raises(MaxArmProtocolError):
        c.set_watchdog(1500)
    assert c._watchdog_ms == 0, "an unverified echo never becomes the expectation"
    c.close()
