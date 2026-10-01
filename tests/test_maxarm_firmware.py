"""SAIGEN MaxArm firmware (src/devices/maxarm/firmware) on CPython with fake hardware.

saigen_maxarm.py is written in the CPython/MicroPython common subset, so the
server logic is tested here unchanged. main.py is run with stub Hiwonder
modules. No serial port is opened.
"""
import ast
import io
import os
import runpy
import sys
import types

import pytest

from src.devices.maxarm import protocol as P
from src.devices.maxarm.firmware import saigen_maxarm as fw

FW_DIR = os.path.dirname(fw.__file__)


# ----------------------------------------------------------------------
# Fakes
# ----------------------------------------------------------------------
class Clock:
    def __init__(self):
        self.t = 1000

    def ticks_ms(self):
        return self.t

    @staticmethod
    def ticks_diff(a, b):
        return a - b

    @staticmethod
    def ticks_add(a, b):
        return a + b


class FakeBus:
    def __init__(self):
        self.calls = []
        self.pulses = {1: 500, 2: 510, 3: 520}
        self.vin = 11800
        self.fail_read = set()
        self.stop_raises = set()

    def stop(self, i):
        self.calls.append(("stop", i))
        if i in self.stop_raises:
            raise OSError("uart")

    def load(self, i):
        self.calls.append(("load", i))

    def unload(self, i):
        self.calls.append(("unload", i))

    def run(self, i, p, ms):
        self.calls.append(("run", i, p, ms))

    def get_position(self, i):
        return False if i in self.fail_read else self.pulses[i]

    def get_vin(self, i):
        return self.vin


class FakeArm:
    ORIGIN = (0, -162.8, 212)

    def __init__(self):
        self.moves = []
        self.result = True
        self.pos = (0, -163, 212)
        self.raise_on_read = None

    def set_position(self, pos, ms):
        self.moves.append((tuple(pos), ms))
        if self.result is True:
            self.pos = tuple(int(round(v)) for v in pos)
        return self.result

    def read_position(self):
        if self.raise_on_read:
            raise self.raise_on_read
        return self.pos

    def go_home(self, duration=2000):
        self.moves.append(("home", duration))


class FakeNozzle:
    def __init__(self):
        self.calls = []

    def on(self):
        self.calls.append("on")

    def off(self):
        self.calls.append("off")


def make_server(**kw):
    out = []
    clock = Clock()
    arm, bus, noz = FakeArm(), FakeBus(), FakeNozzle()
    srv = fw.Server(arm, bus, noz, out.append, clock.ticks_ms, clock.ticks_diff,
                    sys_version=kw.get("sys_version", "3.4.0; MicroPython v1.19.1 on 2022-06-18"))
    return srv, out, clock, arm, bus, noz


def req(seq, body):
    """Request line as the server receives it (no terminator)."""
    return P.encode_line("%d %s" % (seq, body)).decode().rstrip("\n")


def call(srv, seq, body):
    """handle_line + parse the reply with the PC codec."""
    reply = srv.handle_line(req(seq, body))
    return P.parse_reply(reply)


def feed_bytes(srv, data):
    for b in data:
        srv.feed(b)


def replies(out):
    return [P.parse_reply(P.decode_line(line)) for line in out]


# ----------------------------------------------------------------------
# Boot
# ----------------------------------------------------------------------
def test_construction_and_banner_never_move_or_touch_torque():
    srv, out, _clock, arm, bus, noz = make_server()
    srv.announce()
    assert arm.moves == [] and bus.calls == [] and noz.calls == []
    (evt,) = replies(out)
    assert evt.kind == "EVT" and evt.code == "READY"
    assert evt.fields == {"fw": "saigen-maxarm", "proto": "1", "ver": fw.FIRMWARE_VERSION}
    assert srv.wd_ms == 0 and srv.stopped is False and srv.limits is None


def test_constants_match_protocol_module():
    assert fw.FIRMWARE_NAME == P.FIRMWARE_NAME
    assert fw.PROTOCOL_VERSION == P.PROTOCOL_VERSION
    assert fw.MAX_LINE == P.MAX_LINE
    assert fw.SEQ_MAX == P.SEQ_MAX
    assert (fw.MOVE_MS_MIN, fw.MOVE_MS_MAX) == (P.MOVE_MS_MIN, P.MOVE_MS_MAX)


# ----------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------
def test_ping_info_hb_bye():
    srv, *_ = make_server()
    r = call(srv, 1, "PING")
    assert (r.seq, r.kind, r.fields) == (1, "OK", {"pong": "1"})
    info = call(srv, 2, "INFO").fields
    assert info["fw"] == "saigen-maxarm" and info["proto"] == "1"
    assert info["ver"] == fw.FIRMWARE_VERSION
    assert info["mp"] == "3.4.0;_MicroPython_v1.19.1_on_2022-06-18"
    assert (info["limits"], info["wd"], info["stopped"]) == ("0", "0", "0")
    assert call(srv, 3, "WATCHDOG 800").fields == {"wd": "800"}
    assert call(srv, 4, "HB").fields == {"wd": "800"}
    assert call(srv, 5, "BYE").fields == {"bye": "1"}
    assert srv.wd_ms == 0


def test_status_fields():
    srv, _o, _c, arm, bus, _n = make_server()
    st = call(srv, 1, "STATUS").fields
    assert st == {"x": "0", "y": "-163", "z": "212", "p1": "500", "p2": "510", "p3": "520",
                  "moving": "0", "stopped": "0", "pump": "0", "vin": "11800"}
    bus.vin = False
    assert call(srv, 2, "STATUS").fields["vin"] == "-1"


def test_status_servo_read_failures():
    srv, _o, _c, arm, bus, _n = make_server()
    bus.fail_read.add(2)
    r = call(srv, 1, "STATUS")
    assert (r.kind, r.code) == ("ERR", "SERVO")
    bus.fail_read.clear()
    arm.pos = False
    r = call(srv, 2, "STATUS")
    assert (r.kind, r.code) == ("ERR", "SERVO")


def test_move_accepts_and_sets_moving_window():
    srv, _o, clock, arm, _b, _n = make_server()
    r = call(srv, 7, "MOVE 10.5 -200 150 1500")
    assert r.kind == "OK" and r.fields == {"x": "10.5", "y": "-200", "z": "150", "ms": "1500"}
    assert arm.moves == [((10.5, -200, 150), 1500)]
    assert call(srv, 8, "STATUS").fields["moving"] == "1"
    clock.t += 1499
    assert call(srv, 9, "STATUS").fields["moving"] == "1"
    clock.t += 1
    assert call(srv, 10, "STATUS").fields["moving"] == "0"


@pytest.mark.parametrize("args", [
    "1 2 3", "1 2 3 4 5", "a -200 150 1000", "1e3 -200 150 1000", "nan -200 150 1000",
    "inf -200 150 1000", "+5 -200 150 1000", "1_0 -200 150 1000", ". -200 150 1000",
    "0 -200 150 99", "0 -200 150 30001", "0 -200 150 1500.0", "0 -200 150 -1",
    "0x10 -200 150 1000", "1.2.3 -200 150 1000",
])
def test_move_badarg(args):
    srv, _o, _c, arm, _b, _n = make_server()
    r = call(srv, 1, "MOVE " + args)
    assert (r.kind, r.code) == ("ERR", "BADARG")
    assert arm.moves == []


@pytest.mark.parametrize("body", [
    "PUMP", "PUMP 2", "PUMP on", "TORQUE", "TORQUE 1 1", "WATCHDOG", "WATCHDOG 50",
    "WATCHDOG 60001", "WATCHDOG x", "PING 1", "STOP now", "LIMITS 1 2 3 4 5",
    "LIMITS 1 2 3 4 5 6 7", "LIMITS 2 1 3 4 5 6", "LIMITS 1 2 3 4 5 6 -1 7",
    "LIMITS 1 2 3 4 5 6 8 7", "LIMITS a 2 3 4 5 6", "LIMITS off",
])
def test_other_badarg(body):
    srv, *_ = make_server()
    r = call(srv, 1, body)
    assert (r.kind, r.code) == ("ERR", "BADARG"), body


def test_empty_token_is_badarg():
    srv, *_ = make_server()
    r = call(srv, 3, "MOVE 1  2 3 1000")
    assert (r.seq, r.kind, r.code) == (3, "ERR", "BADARG")


@pytest.mark.parametrize("cmd", ["ping", "FOO", "MOVEX"])
def test_unknown_command(cmd):
    srv, *_ = make_server()
    r = call(srv, 4, cmd)
    assert (r.seq, r.kind, r.code) == (4, "ERR", "BADCMD")


def test_bad_sequence_numbers():
    srv, *_ = make_server()
    for body in ("0 PING", "1000000 PING", "-1 PING", "x PING"):
        reply = srv.handle_line(P.encode_line(body).decode().rstrip("\n"))
        r = P.parse_reply(reply)
        assert (r.seq, r.kind, r.code) == (0, "ERR", "BADARG"), body


def test_stop_latch_blocks_move_until_clear():
    srv, _o, clock, arm, bus, noz = make_server()
    assert call(srv, 1, "PUMP 1").fields == {"pump": "1"}
    assert call(srv, 2, "MOVE 0 -200 150 2000").kind == "OK"
    r = call(srv, 3, "STOP")
    assert r.fields == {"stopped": "1"}
    assert [c for c in bus.calls if c[0] == "stop"] == [("stop", 1), ("stop", 2), ("stop", 3)]
    st = call(srv, 4, "STATUS").fields
    assert st["moving"] == "0" and st["stopped"] == "1" and st["pump"] == "1"
    assert noz.calls == ["on"], "STOP must leave the pump unchanged"
    r = call(srv, 5, "MOVE 0 -210 150 1000")
    assert (r.kind, r.code) == ("ERR", "STOPPED")
    assert len(arm.moves) == 1
    assert call(srv, 6, "INFO").fields["stopped"] == "1"
    assert call(srv, 7, "CLEAR").fields == {"stopped": "0"}
    assert call(srv, 8, "MOVE 0 -210 150 1000").kind == "OK"


def test_stop_is_best_effort_when_a_servo_raises():
    srv, _o, _c, _a, bus, _n = make_server()
    bus.stop_raises.add(1)
    r = call(srv, 1, "STOP")
    assert (r.kind, r.code) == ("ERR", "SERVO")
    assert [c[1] for c in bus.calls if c[0] == "stop"] == [1, 2, 3]
    assert srv.stopped is True


def test_pump_and_torque():
    srv, _o, _c, _a, bus, noz = make_server()
    assert call(srv, 1, "PUMP 1").fields == {"pump": "1"}
    assert call(srv, 2, "PUMP 0").fields == {"pump": "0"}
    assert noz.calls == ["on", "off"]
    assert call(srv, 3, "TORQUE 0").fields == {"torque": "0"}
    assert call(srv, 4, "TORQUE 1").fields == {"torque": "1"}
    assert bus.calls == [("unload", 1), ("unload", 2), ("unload", 3),
                         ("load", 1), ("load", 2), ("load", 3)]


def test_limits_box_and_radius():
    srv, _o, _c, arm, _b, _n = make_server()
    r = call(srv, 1, "LIMITS -250 250 -280 -60 40 225 60 260")
    assert r.fields == {"limits": "1", "xmin": "-250", "xmax": "250", "ymin": "-280",
                        "ymax": "-60", "zmin": "40", "zmax": "225", "rmin": "60", "rmax": "260"}
    assert call(srv, 2, "INFO").fields["limits"] == "1"
    for body in ("MOVE 260 -200 150 1000", "MOVE 0 -50 150 1000", "MOVE 0 -200 30 1000",
                 "MOVE 200 -250 150 1000"):
        r = call(srv, 3, body)
        assert (r.kind, r.code) == ("ERR", "LIMIT"), body
    assert arm.moves == []
    assert call(srv, 4, "MOVE 0 -200 150 1000").kind == "OK"
    assert call(srv, 5, "LIMITS -10 10 -210 -190 100 200").fields["limits"] == "1"
    assert "rmin" not in call(srv, 6, "LIMITS -10 10 -210 -190 100 200").fields
    assert (call(srv, 7, "MOVE 20 -200 150 1000").code) == "LIMIT"
    assert call(srv, 8, "LIMITS OFF").fields == {"limits": "0"}
    assert call(srv, 9, "MOVE 20 -200 150 1000").kind == "OK"


def test_unreachable():
    srv, _o, _c, arm, _b, _n = make_server()
    for body in ("MOVE 0 -200 226 1000", "MOVE 30 -30 100 1000"):
        r = call(srv, 1, body)
        assert (r.kind, r.code) == ("ERR", "UNREACHABLE"), body
    assert arm.moves == [], "hard envelope is checked before set_position"
    for result in (False, None):
        arm.result = result
        r = call(srv, 2, "MOVE 0 -280 20 1000")
        assert (r.kind, r.code) == ("ERR", "UNREACHABLE")
    assert srv._moving() is False


def test_internal_error_on_raising_arm():
    srv, _o, _c, arm, _b, _n = make_server()
    arm.raise_on_read = ValueError("boom *\nbad")
    r = call(srv, 1, "STATUS")
    assert (r.kind, r.code) == ("ERR", "INTERNAL")
    assert "ValueError" in r.text and "boom" in r.text


def test_keyboard_interrupt_is_never_swallowed():
    srv, _o, _c, arm, _b, _n = make_server()
    arm.raise_on_read = KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt):
        srv.handle_line(req(1, "STATUS"))


# ----------------------------------------------------------------------
# Framing through feed()
# ----------------------------------------------------------------------
def test_feed_assembles_lines_with_crlf_and_blank_lines():
    srv, out, *_ = make_server()
    feed_bytes(srv, b"\n\r\n" + P.build_request(1, "PING").replace(b"\n", b"\r\n")
               + P.build_request(2, "PING"))
    rs = replies(out)
    assert [(r.seq, r.kind) for r in rs] == [(1, "OK"), (2, "OK")]
    assert all(line.endswith("\n") for line in out)


def test_checksum_errors():
    srv, out, *_ = make_server()
    good = P.build_request(5, "PING").decode()
    bad = good[:-3] + ("00" if not good.endswith("*00\n") else "01") + "\n"
    feed_bytes(srv, bad.encode())
    feed_bytes(srv, b"6 PING\n")
    feed_bytes(srv, b"6 PING*4\n")
    feed_bytes(srv, b"garbage*ZZ\n")
    rs = replies(out)
    assert [(r.seq, r.code) for r in rs] == [(5, "CHECKSUM"), (6, "CHECKSUM"),
                                            (6, "CHECKSUM"), (0, "CHECKSUM")]


def test_lowercase_checksum_accepted_like_protocol_module():
    srv, *_ = make_server()
    body = "1 PING"
    line = body + "*" + P.checksum(body).lower()
    assert P.decode_line(line) == body
    assert P.parse_reply(srv.handle_line(line)).kind == "OK"


def test_non_ascii_byte_rejects_the_line():
    srv, out, *_ = make_server()
    feed_bytes(srv, b"1 PI\xffNG*00\n")
    feed_bytes(srv, b"2 PI\x03NG*00\n")
    feed_bytes(srv, P.build_request(3, "PING"))
    rs = replies(out)
    assert [(r.seq, r.code) for r in rs] == [(0, "CHECKSUM"), (0, "CHECKSUM"), (3, None)]


def test_toolong_discards_buffer_and_recovers():
    srv, out, _c, arm, _b, _n = make_server()
    feed_bytes(srv, b"1 MOVE " + b"9" * 300 + b"\n")
    feed_bytes(srv, P.build_request(2, "PING"))
    rs = replies(out)
    assert [(r.seq, r.kind, r.code) for r in rs] == [(0, "ERR", "TOOLONG"), (2, "OK", None)]
    assert arm.moves == []
    assert len(srv.buf) == fw.MAX_LINE, "line buffer is preallocated and never grows"


def test_line_of_exactly_max_line_bytes_is_accepted():
    srv, out, *_ = make_server()
    body = "1 PING"
    pad = fw.MAX_LINE - 3 - len(body)
    # a 200-byte line: the server parses it (BADARG for the extra token, not TOOLONG)
    body = body + " " + "x" * (pad - 1)
    line = body + "*" + P.checksum(body)
    assert len(line) == fw.MAX_LINE
    feed_bytes(srv, (line + "\n").encode())
    (r,) = replies(out)
    assert (r.seq, r.code) == (1, "BADARG")


def test_replies_never_exceed_max_line():
    srv, out, _c, arm, _b, _n = make_server()
    arm.raise_on_read = RuntimeError("x" * 500)
    feed_bytes(srv, P.build_request(1, "STATUS"))
    assert len(out[0].rstrip("\n")) <= fw.MAX_LINE
    P.decode_line(out[0])


# ----------------------------------------------------------------------
# Watchdog
# ----------------------------------------------------------------------
def test_watchdog_fires_after_silence_and_disables_itself():
    srv, out, clock, arm, bus, noz = make_server()
    feed_bytes(srv, P.build_request(1, "PUMP", "1"))
    feed_bytes(srv, P.build_request(2, "WATCHDOG", 500))
    clock.t += 499
    srv.tick()
    assert srv.stopped is False
    clock.t += 1
    srv.tick()
    assert srv.stopped is True and srv.wd_ms == 0
    assert [c for c in bus.calls if c[0] == "stop"] == [("stop", 1), ("stop", 2), ("stop", 3)]
    assert noz.calls == ["on"], "watchdog leaves the pump unchanged (STOP behaviour)"
    evt = replies(out)[-1]
    assert (evt.kind, evt.code, evt.fields["reason"]) == ("EVT", "STOPPED", "watchdog")
    n = len(out)
    clock.t += 10000
    srv.tick()
    assert len(out) == n, "the watchdog fires once, then stays disabled until re-armed"
    feed_bytes(srv, P.build_request(3, "INFO"))
    assert replies(out)[-1].fields["wd"] == "0"


def test_valid_lines_feed_the_watchdog_but_corrupt_ones_do_not():
    srv, out, clock, *_ = make_server()
    feed_bytes(srv, P.build_request(1, "WATCHDOG", 500))
    for _ in range(5):
        clock.t += 400
        feed_bytes(srv, P.build_request(2, "HB"))
        srv.tick()
    clock.t += 400
    feed_bytes(srv, P.build_request(3, "NOPE"))      # bad command, but a valid line
    srv.tick()
    assert srv.stopped is False
    clock.t += 400
    feed_bytes(srv, b"4 HB*00\n")                     # corrupt line: does not feed
    clock.t += 100
    srv.tick()
    assert srv.stopped is True


def test_bye_disables_watchdog():
    srv, out, clock, *_ = make_server()
    feed_bytes(srv, P.build_request(1, "WATCHDOG", 300))
    feed_bytes(srv, P.build_request(2, "BYE"))
    clock.t += 5000
    srv.tick()
    assert srv.stopped is False


# ----------------------------------------------------------------------
# MicroPython compatibility of the sources
# ----------------------------------------------------------------------
def _source(name):
    with open(os.path.join(FW_DIR, name), encoding="utf-8") as f:
        return f.read()


@pytest.mark.parametrize("name", ["saigen_maxarm.py", "main.py"])
def test_firmware_sources_use_the_micropython_subset(name):
    src = _source(name)
    tree = ast.parse(src)
    for node in ast.walk(tree):
        assert not isinstance(node, ast.JoinedStr), "f-strings are not allowed"
        assert not isinstance(node, (ast.AnnAssign,)), "annotations are not allowed"
        if isinstance(node, (ast.FunctionDef,)):
            assert node.returns is None and all(a.annotation is None for a in node.args.args)
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module]
            for n in names:
                assert n.split(".")[0] not in ("typing", "dataclasses", "enum", "logging",
                                               "threading", "abc"), n
        if isinstance(node, ast.ExceptHandler):
            assert node.type is not None, "bare except would swallow KeyboardInterrupt"
            if isinstance(node.type, ast.Name) and node.type.id == "BaseException":
                assert name == "main.py"
                assert any(isinstance(n, ast.Raise) and n.exc is None for n in node.body), \
                    "a BaseException handler must re-raise"
    assert not any(isinstance(n, ast.Attribute) and n.attr == "monotonic" for n in ast.walk(tree))
    assert "kbd_intr" not in src
    assert src.isascii()


def test_saigen_maxarm_never_calls_go_home():
    tree = ast.parse(_source("saigen_maxarm.py") + _source("main.py"))
    called = {n.func.attr for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert "go_home" not in called
    assert "run" not in called and "set_position_relatively" not in called


# ----------------------------------------------------------------------
# main.py launcher with stub device modules
# ----------------------------------------------------------------------
class _PWM:
    def __init__(self):
        self.duties = [512]

    def duty(self, v):
        self.duties.append(v)


class _StubNozzle(FakeNozzle):
    def __init__(self):
        super().__init__()
        self.pump_f, self.pump_b, self.valve_f, self.valve_b = _PWM(), _PWM(), _PWM(), _PWM()


class _Stdin:
    def __init__(self, data):
        self.buffer = io.BytesIO(data)


def _run_main(monkeypatch, data, idle_polls=0, old_build=False):
    made = {}

    def bus_factory():
        made["bus"] = FakeBus()
        return made["bus"]

    def arm_factory(bus):
        made["arm"] = FakeArm()
        return made["arm"]

    class StubSuctionNozzle(_StubNozzle):
        """A class (main.py allocates it with __new__ before calling __init__)."""

        def __new__(cls):
            made["nozzle"] = obj = object.__new__(cls)
            return obj

        def __init__(self):
            _StubNozzle.__init__(self)        # looked up at call time: monkeypatchable

    clock = Clock()
    stdin = _Stdin(data)
    remaining = [len(data)]
    idle = [idle_polls]

    class Poll:
        def register(self, *a):
            pass

        def poll(self, ms):
            if remaining[0] > 0:
                remaining[0] -= 1
                return [(stdin, 1)]
            if idle[0] <= 0:
                raise KeyboardInterrupt()
            idle[0] -= 1
            clock.t += ms
            return []

    utime = types.ModuleType("utime")
    utime.ticks_ms = clock.ticks_ms
    utime.ticks_diff = clock.ticks_diff
    utime.ticks_add = clock.ticks_add
    uselect = types.ModuleType("uselect")
    uselect.poll = Poll
    uselect.POLLIN = 1
    mods = {"utime": utime, "uselect": uselect,
            "BusServo": types.SimpleNamespace(BusServo=bus_factory),
            "espmax": types.SimpleNamespace(ESPMax=arm_factory),
            "SuctionNozzle": types.SimpleNamespace(SuctionNozzle=StubSuctionNozzle)}
    if old_build:
        # a MicroPython build without Class.__new__: the attribute lookup raises
        class NoClassNew(type):
            def __getattribute__(cls, name):
                if name == "__new__":
                    raise AttributeError("type object has no attribute '__new__'")
                return type.__getattribute__(cls, name)

        class OldSuctionNozzle(_StubNozzle, metaclass=NoClassNew):
            def __init__(self):
                made["nozzle"] = self
                _StubNozzle.__init__(self)
        mods["SuctionNozzle"] = types.SimpleNamespace(SuctionNozzle=OldSuctionNozzle)
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.delitem(sys.modules, "saigen_maxarm", raising=False)
    monkeypatch.syspath_prepend(FW_DIR)
    out = io.StringIO()
    monkeypatch.setattr(sys, "stdin", stdin)
    monkeypatch.setattr(sys, "stdout", out)
    exc = None
    try:
        runpy.run_path(os.path.join(FW_DIR, "main.py"), run_name="__main__")
    except BaseException as e:      # noqa: BLE001 - we want to see what escapes
        exc = e
    monkeypatch.setattr(sys, "stdout", sys.__stdout__)
    return made, out.getvalue(), exc


def test_main_boots_without_motion_serves_and_faults_cleanly(monkeypatch):
    data = P.build_request(1, "PING") + P.build_request(2, "STATUS")
    made, text, exc = _run_main(monkeypatch, data)
    assert isinstance(exc, KeyboardInterrupt), "Ctrl-C must propagate to the REPL"
    lines = [ln for ln in text.splitlines() if ln]
    rs = [P.parse_reply(P.decode_line(ln)) for ln in lines]
    assert (rs[0].kind, rs[0].code) == ("EVT", "READY")
    assert [(r.seq, r.kind) for r in rs[1:3]] == [(1, "OK"), (2, "OK")]
    assert (rs[-1].kind, rs[-1].code) == ("EVT", "FAULT")
    assert rs[-1].fields["type"] == "KeyboardInterrupt"
    noz = made["nozzle"]
    for pwm in (noz.pump_f, noz.pump_b, noz.valve_f, noz.valve_b):
        assert pwm.duties[-1] == 0, "pump/valve PWM must be zeroed at boot"
    assert made["arm"].moves == [], "boot must never move the arm"
    bus_calls = made["bus"].calls
    assert not any(c[0] in ("load", "unload", "run") for c in bus_calls), "no torque change at boot"
    assert [c for c in bus_calls if c[0] == "stop"] == [("stop", 1), ("stop", 2), ("stop", 3)], \
        "the fault path sends STOP behaviour"


def test_main_watchdog_runs_in_the_loop(monkeypatch):
    made, text, exc = _run_main(monkeypatch, P.build_request(1, "WATCHDOG", 200), idle_polls=15)
    assert isinstance(exc, KeyboardInterrupt)
    rs = [P.parse_reply(P.decode_line(ln)) for ln in text.splitlines() if ln]
    assert [(r.kind, r.code) for r in rs] == [("EVT", "READY"), ("OK", None),
                                             ("EVT", "STOPPED"), ("EVT", "FAULT")]
    assert made["arm"].moves == []


# ----------------------------------------------------------------------
# Review fixes (codex_review_maxarm_device.md D4, D5, D6, D11; PUMP busy window)
# ----------------------------------------------------------------------
def _stops(bus):
    return [c for c in bus.calls if c[0] == "stop"]


def test_d4_failed_set_position_halts_partially_commanded_servos_without_latch():
    srv, _o, _c, arm, bus, _n = make_server()

    def partial(pos, ms):                 # vendor set_position: bare except -> False
        bus.calls.append(("run", 1, 600, ms))
        return False
    arm.set_position = partial
    r = call(srv, 1, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "UNREACHABLE")
    assert _stops(bus) == [("stop", 1), ("stop", 2), ("stop", 3)], "servo 1 must be halted"
    assert srv.stopped is False, "UNREACHABLE is not a fault: no latch"
    assert call(srv, 2, "STATUS").fields["moving"] == "0"
    arm.set_position = FakeArm.set_position.__get__(arm)
    assert call(srv, 3, "MOVE 50 -200 150 1000").kind == "OK"


@pytest.mark.parametrize("body,setup", [
    ("MOVE 50 -200 150 1000", "arm"),
    ("TORQUE 0", "bus"),
    ("PUMP 1", "nozzle"),
])
def test_d4_actuator_exception_stops_and_latches(body, setup):
    srv, _o, _c, arm, bus, noz = make_server()
    call(srv, 1, "PUMP 0")
    noz.calls.clear()
    if setup == "arm":
        def boom(pos, ms):
            raise OSError("uart")
        arm.set_position = boom
    elif setup == "bus":
        def boom(i):
            bus.calls.append(("unload", i))
            raise OSError("uart")
        bus.unload = boom
    else:
        def boom():
            raise OSError("pwm")
        noz.on = boom
        srv.pump_busy = False                          # not inside the PUMP 0 window
    r = call(srv, 2, body)
    assert (r.kind, r.code) == ("ERR", "INTERNAL")
    assert "stop latch set" in r.text
    assert srv.stopped is True and _stops(bus) == [("stop", 1), ("stop", 2), ("stop", 3)]
    assert noz.calls == [], "the fault path never touches the pump"
    r = call(srv, 3, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "STOPPED")


def test_d4_non_actuator_exception_does_not_latch():
    srv, _o, _c, arm, bus, _n = make_server()
    arm.raise_on_read = ValueError("boom")
    assert call(srv, 1, "STATUS").code == "INTERNAL"
    assert srv.stopped is False and _stops(bus) == []


def test_pump_on_inside_the_vent_window_is_busy():
    srv, _o, clock, _a, _b, noz = make_server()
    assert call(srv, 1, "PUMP 1").kind == "OK"
    assert call(srv, 2, "PUMP 0").kind == "OK"
    clock.t += fw.PUMP_OFF_BUSY_MS - 1
    r = call(srv, 3, "PUMP 1")
    assert (r.kind, r.code) == ("ERR", "BUSY")
    assert noz.calls == ["on", "off"], "PUMP 1 inside the window is not executed"
    assert call(srv, 4, "STATUS").fields["pump"] == "0"
    assert call(srv, 5, "PUMP 0").kind == "OK", "PUMP 0 is always accepted"
    clock.t += fw.PUMP_OFF_BUSY_MS
    assert call(srv, 6, "PUMP 1").fields == {"pump": "1"}
    assert noz.calls == ["on", "off", "off", "on"]
    assert P.ERR_BUSY in P.ERROR_CODES


def test_d6_late_line_after_expiry_fires_the_watchdog_first():
    srv, out, clock, arm, bus, _n = make_server()
    call(srv, 1, "WATCHDOG 1000")
    clock.t += 1015                                   # expired; the loop has not ticked yet
    r = call(srv, 2, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "STOPPED")
    assert arm.moves == [] and srv.wd_ms == 0 and srv.stopped is True
    evt = replies(out)[-1]
    assert (evt.code, evt.fields["reason"]) == ("STOPPED", "watchdog")
    srv2, out2, clock2, *_ = make_server()
    call(srv2, 1, "WATCHDOG 1000")
    clock2.t += 1000
    assert call(srv2, 2, "HB").fields == {"wd": "0"}, "a late HB cannot revive the watchdog"


PERIOD = 1 << 30                  # MicroPython ESP32 ticks period (TICKS_MAX + 1)


class ModClock:
    """MicroPython-style modular tick clock."""

    def __init__(self, t):
        self.t = t

    def ticks_ms(self):
        return self.t % PERIOD

    @staticmethod
    def ticks_diff(a, b):
        return ((a - b + PERIOD // 2) & (PERIOD - 1)) - PERIOD // 2

    @staticmethod
    def ticks_add(a, b):
        return (a + b) % PERIOD


def _mod_server(t):
    clock = ModClock(t)
    arm, bus, noz = FakeArm(), FakeBus(), FakeNozzle()
    srv = fw.Server(arm, bus, noz, lambda s: None, clock.ticks_ms, clock.ticks_diff,
                    ticks_add=clock.ticks_add)
    return srv, clock


def test_d11_no_phantom_motion_after_half_a_tick_period():
    srv, clock = _mod_server(1000)
    assert not srv._moving()
    clock.t += PERIOD // 2 + 1                         # ~6.2 days without a MOVE
    assert not srv._moving()
    call(srv, 1, "MOVE 50 -200 150 1000")
    clock.t += 1000
    srv.tick()                                         # the loop expires the deadline
    assert srv.motion_active is False
    clock.t += PERIOD // 2 + 1
    assert not srv._moving()
    assert call(srv, 2, "STATUS").fields["moving"] == "0"


def test_d11_deadline_wraps_with_ticks_add():
    srv, clock = _mod_server(PERIOD - 10)
    call(srv, 1, "MOVE 50 -200 150 1500")
    assert 0 <= srv.move_until < PERIOD, "deadline stays a valid tick value"
    assert srv.move_until == 1490
    clock.t += 1499
    assert srv._moving()
    clock.t += 1
    assert not srv._moving()


def test_d11_pump_window_wraps_too():
    srv, clock = _mod_server(PERIOD - 5)
    call(srv, 1, "PUMP 0")
    assert 0 <= srv.pump_off_until < PERIOD
    clock.t += 100
    assert call(srv, 2, "PUMP 1").code == "BUSY"
    clock.t += fw.PUMP_OFF_BUSY_MS
    assert call(srv, 3, "PUMP 1").kind == "OK"


def _is_tick_call(node):
    return (isinstance(node, ast.Call) and
            ((isinstance(node.func, ast.Attribute) and node.func.attr == "ticks_ms") or
             (isinstance(node.func, ast.Name) and node.func.id == "ticks_ms")))


def _is_deadline(node):
    return isinstance(node, ast.Attribute) and node.attr in (
        "move_until", "last_rx", "pump_off_until")


@pytest.mark.parametrize("name", ["saigen_maxarm.py", "main.py"])
def test_tick_arithmetic_only_through_ticks_add_and_ticks_diff(name):
    tree = ast.parse(_source(name))
    for node in ast.walk(tree):
        if isinstance(node, (ast.BinOp, ast.AugAssign)) and isinstance(node.op, (ast.Add, ast.Sub)):
            operands = [node.left, node.right] if isinstance(node, ast.BinOp) else [node.target, node.value]
            for o in operands:
                assert not _is_tick_call(o) and not _is_deadline(o), \
                    "%s line %d: use ticks_add/ticks_diff for tick values" % (name, node.lineno)
        if isinstance(node, ast.Compare):
            for o in [node.left] + node.comparators:
                assert not _is_tick_call(o) and not _is_deadline(o), \
                    "%s line %d: compare ticks with ticks_diff" % (name, node.lineno)
    if name == "main.py":
        assert "ticks_add=time.ticks_add" in _source(name)


def test_d5_pwm_zeroing_failure_prevents_ready(monkeypatch):
    class BadPWM(_PWM):
        def duty(self, v):
            raise OSError("pwm")
    orig = _StubNozzle.__init__

    def init(self):
        orig(self)
        self.pump_f = BadPWM()
    monkeypatch.setattr(_StubNozzle, "__init__", init)
    made, text, exc = _run_main(monkeypatch, P.build_request(1, "PING"))
    rs = [P.parse_reply(P.decode_line(ln)) for ln in text.splitlines() if ln]
    assert isinstance(exc, RuntimeError) and "pump_f" in str(exc)
    assert [(r.kind, r.code) for r in rs] == [("EVT", "FAULT")], "no READY, no command served"
    noz = made["nozzle"]
    for pwm in (noz.pump_b, noz.valve_f, noz.valve_b):
        assert pwm.duties[-1] == 0
    assert [c for c in made["bus"].calls if c[0] == "stop"] == [("stop", 1), ("stop", 2), ("stop", 3)]
    assert made["arm"].moves == []


def test_d5_missing_pwm_output_prevents_ready(monkeypatch):
    orig = _StubNozzle.__init__

    def init(self):
        orig(self)
        del self.valve_b
    monkeypatch.setattr(_StubNozzle, "__init__", init)
    made, text, exc = _run_main(monkeypatch, P.build_request(1, "PING"))
    assert isinstance(exc, RuntimeError) and "valve_b:missing" in str(exc)
    assert "EVT READY" not in text and "EVT FAULT" in text


def test_d5_fault_after_ready_leaves_the_pump_alone(monkeypatch):
    data = P.build_request(1, "PUMP", "1")
    made, text, exc = _run_main(monkeypatch, data)
    assert isinstance(exc, KeyboardInterrupt)
    noz = made["nozzle"]
    assert noz.calls == ["on"], "the runtime fault path must not switch the pump off"
    assert all(pwm.duties == [512, 0] for pwm in (noz.pump_f, noz.pump_b, noz.valve_f, noz.valve_b))


# ----------------------------------------------------------------------
# Round-2 review fixes (codex_review_maxarm_device2.md 6, 8, 15)
# ----------------------------------------------------------------------
@pytest.mark.parametrize("raising", [{1, 2, 3}, {2}])
def test_r6_failed_move_with_a_raising_stop_latches_and_never_claims_halted(raising):
    srv, _o, _c, arm, bus, _n = make_server()
    arm.result = False
    bus.stop_raises = set(raising)
    r = call(srv, 1, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "SERVO")
    assert "stop latch set" in r.text and "halted" not in r.text
    assert srv.stopped is True
    assert [c[1] for c in _stops(bus)] == [1, 2, 3], "every servo still gets its stop"
    bus.stop_raises = set()
    arm.result = True
    r = call(srv, 2, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "STOPPED"), "no further MOVE until CLEAR"
    assert call(srv, 3, "STOP").kind == "OK", "STOP is always accepted"


def test_r6_failed_move_with_clean_stops_is_still_unreachable_without_latch():
    srv, _o, _c, arm, bus, _n = make_server()
    arm.result = None
    r = call(srv, 1, "MOVE 50 -200 150 1000")
    assert (r.kind, r.code) == ("ERR", "UNREACHABLE") and "servos halted" in r.text
    assert srv.stopped is False and srv._halt_servos() is True


class VentingNozzle:
    """Vendor semantics (SuctionNozzle.py): off() vents in a thread that clears
    nozzle_st only at its end; on() is a silent no-op while nozzle_st is True."""

    def __init__(self):
        self.nozzle_st = False
        self.pump_running = False
        self.on_calls = 0

    def on(self):
        self.on_calls += 1
        if not self.nozzle_st:
            self.pump_running = True
            self.nozzle_st = True

    def off(self):
        self.pump_running = False               # the vent thread has not finished yet


def test_r15_late_vent_thread_makes_pump_on_busy_not_silently_ignored():
    srv, _o, clock, _a, _b, _n = make_server()
    noz = srv.nozzle = VentingNozzle()
    assert call(srv, 1, "PUMP 1").kind == "OK" and noz.pump_running
    assert call(srv, 2, "PUMP 0").kind == "OK"
    clock.t += fw.PUMP_OFF_BUSY_MS                # window over, vent thread still running
    r = call(srv, 3, "PUMP 1")
    assert (r.kind, r.code) == ("ERR", "BUSY") and "nozzle_st" in r.text
    assert srv.pump is False and noz.on_calls == 1, "on() is not even attempted"
    assert call(srv, 4, "STATUS").fields["pump"] == "0"
    assert call(srv, 5, "PUMP 0").kind == "OK", "PUMP 0 is always accepted"
    clock.t += fw.PUMP_OFF_BUSY_MS
    noz.nozzle_st = False                         # vent thread finished
    assert call(srv, 6, "PUMP 1").fields == {"pump": "1"} and noz.pump_running


def test_r15_pump_on_while_already_on_is_not_busy():
    srv, *_ = make_server()
    noz = srv.nozzle = VentingNozzle()
    assert call(srv, 1, "PUMP 1").kind == "OK"
    assert noz.nozzle_st is True
    assert call(srv, 2, "PUMP 1").fields == {"pump": "1"}, "our own pump flag is on"


def test_r15_nozzle_without_nozzle_st_uses_the_time_window_only():
    srv, _o, clock, _a, _b, noz = make_server()
    assert not hasattr(noz, "nozzle_st")
    call(srv, 1, "PUMP 0")
    clock.t += fw.PUMP_OFF_BUSY_MS
    assert call(srv, 2, "PUMP 1").kind == "OK"


@pytest.mark.parametrize("n_created", [0, 1, 2, 4])
def test_r8_nozzle_constructor_failure_zeroes_the_pwm_it_created(monkeypatch, n_created):
    created = []

    def init(self):
        FakeNozzle.__init__(self)
        for name in ("pump_f", "pump_b", "valve_f", "valve_b")[:n_created]:
            pwm = _PWM()                      # MicroPython PWM(Pin) starts at duty 512
            setattr(self, name, pwm)
            created.append(pwm)
        raise ValueError("out of PWM channels")
    monkeypatch.setattr(_StubNozzle, "__init__", init)
    made, text, exc = _run_main(monkeypatch, P.build_request(1, "PING"))
    assert isinstance(exc, ValueError)
    assert "EVT FAULT" in text and "EVT READY" not in text
    assert made["nozzle"] is not None, "the half-built nozzle was reachable by cleanup"
    assert [p.duties[-1] for p in created] == [0] * n_created, "every created output zeroed"
    assert [c for c in made["bus"].calls if c[0] == "stop"] == [("stop", 1), ("stop", 2),
                                                                 ("stop", 3)]
    assert made["arm"].moves == []


def test_r8_main_allocates_the_nozzle_before_initialising_it():
    src = _source("main.py")
    assert "SuctionNozzle.__new__(SuctionNozzle)" in src
    assert src.index('hw["nozzle"] = noz') < src.index("SuctionNozzle.__init__(noz)")


def test_main_falls_back_to_object_new_on_a_build_without_class_new(monkeypatch):
    made, text, exc = _run_main(monkeypatch, P.build_request(1, "PING"), old_build=True)
    assert isinstance(exc, KeyboardInterrupt)
    rs = [P.parse_reply(P.decode_line(ln)) for ln in text.splitlines() if ln]
    assert (rs[0].kind, rs[0].code) == ("EVT", "READY")
    assert (rs[1].seq, rs[1].kind) == (1, "OK")
    noz = made["nozzle"]
    assert type(noz).__name__ == "OldSuctionNozzle"
    for pwm in (noz.pump_f, noz.pump_b, noz.valve_f, noz.valve_b):
        assert pwm.duties[-1] == 0
    assert made["arm"].moves == []
