"""saigen-maxarm wire codec (src/devices/maxarm/protocol.py) and its agreement with
the firmware implementation (src/devices/maxarm/firmware/saigen_maxarm.py)."""
import random

import pytest

from src.devices.maxarm import protocol as P
from src.devices.maxarm.firmware import saigen_maxarm as fw


# ----------------------------------------------------------------------
# Checksum and framing
# ----------------------------------------------------------------------
@pytest.mark.parametrize("text,expected", [
    ("", "00"),
    ("A", "41"),
    ("AB", "03"),
    # a published NMEA sentence ($...*76): same XOR algorithm
    ("GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,", "76"),
    ("1 PING", "%02X" % (ord("1") ^ ord(" ") ^ ord("P") ^ ord("I") ^ ord("N") ^ ord("G"))),
])
def test_checksum_vectors(text, expected):
    assert P.checksum(text) == expected
    assert fw.checksum(text) == expected


def test_checksum_identical_on_random_ascii():
    rng = random.Random(1234)
    alphabet = "".join(chr(c) for c in range(0x20, 0x7F) if chr(c) != "*")
    for _ in range(500):
        s = "".join(rng.choice(alphabet) for _ in range(rng.randint(0, 150)))
        assert P.checksum(s) == fw.checksum(s)
        assert P.encode_line(s).decode() == fw.frame(s)


def test_encode_decode_roundtrip():
    raw = P.encode_line("12 OK x=1 y=2")
    assert raw.endswith(b"\n") and raw.count(b"*") == 1
    assert P.decode_line(raw) == "12 OK x=1 y=2"
    assert P.decode_line(raw.replace(b"\n", b"\r\n")) == "12 OK x=1 y=2"
    assert P.decode_line(raw.decode()) == "12 OK x=1 y=2"


@pytest.mark.parametrize("bad", [
    "has*star", "new\nline", "tab\there", "café",
])
def test_encode_rejects_forbidden_characters(bad):
    with pytest.raises(ValueError):
        P.encode_line(bad)


def test_encode_rejects_too_long():
    P.encode_line("x" * (P.MAX_LINE - 3))
    with pytest.raises(ValueError):
        P.encode_line("x" * (P.MAX_LINE - 2))


@pytest.mark.parametrize("raw", [
    b"1 OK", b"1 OK*", b"1 OK*4", b"1 OK*123", b"1 OK*ZZ", b"1 OK*00",
    b"1 O\xffK*00", "1 OK x=é*00",
])
def test_decode_rejects_bad_lines(raw):
    with pytest.raises(P.MaxArmProtocolError):
        P.decode_line(raw)


# ----------------------------------------------------------------------
# Requests and replies
# ----------------------------------------------------------------------
def test_build_request_formats_numbers_without_exponent():
    assert P.build_request(1, "PING") == P.encode_line("1 PING")
    assert P.build_request(2, "MOVE", 10, -200.5, 1e-7, 1500) == P.encode_line("2 MOVE 10 -200.5 0 1500")
    assert P.build_request(3, "MOVE", 1e6, 0.1234, -0.0004, 100) == \
        P.encode_line("3 MOVE 1000000 0.123 0 100")
    assert P.build_request(4, "LIMITS", "OFF") == P.encode_line("4 LIMITS OFF")


@pytest.mark.parametrize("seq", [0, -1, 1000000, 1.0, True, "1"])
def test_build_request_rejects_bad_seq(seq):
    with pytest.raises(ValueError):
        P.build_request(seq, "PING")


@pytest.mark.parametrize("arg", [float("nan"), float("inf"), True, "two words", ""])
def test_build_request_rejects_bad_args(arg):
    with pytest.raises(ValueError):
        P.build_request(1, "MOVE", arg)


def test_parse_kv():
    assert P.parse_kv(["x=1", "y=-2.5", "noeq", "k=a=b", "=v"]) == {"x": "1", "y": "-2.5", "k": "a=b"}


def test_parse_reply_kinds():
    ok = P.parse_reply("7 OK x=1 moving=0")
    assert ok == P.Reply(7, "OK", None, {"x": "1", "moving": "0"}, "x=1 moving=0") and ok.ok
    assert P.parse_reply("8 OK") == P.Reply(8, "OK", None, {}, "")
    err = P.parse_reply("9 ERR LIMIT target outside the device box")
    assert (err.seq, err.kind, err.code, err.text) == (9, "ERR", "LIMIT", "target outside the device box")
    assert not err.ok and err.fields == {}
    evt = P.parse_reply("EVT STOPPED reason=watchdog")
    assert (evt.seq, evt.kind, evt.code, evt.fields) == (None, "EVT", "STOPPED", {"reason": "watchdog"})


@pytest.mark.parametrize("line", ["", "x OK", "1", "1 WHAT", "1 ERR", "EVT", "1234567 OK"])
def test_parse_reply_rejects(line):
    with pytest.raises(P.MaxArmProtocolError):
        P.parse_reply(line)


def test_command_and_error_constants_match_firmware():
    srv = _server()
    assert set(srv.commands) == P.COMMANDS
    src = open(fw.__file__, encoding="utf-8").read()
    for code in P.ERROR_CODES:
        assert '"%s"' % code in src or "ERR %s" % code in src, code


# ----------------------------------------------------------------------
# Cross-compatibility: protocol.py <-> firmware Server
# ----------------------------------------------------------------------
class _Arm:
    def __init__(self):
        self.pos = (0, -163, 212)

    def set_position(self, pos, ms):
        self.pos = tuple(pos)
        return True

    def read_position(self):
        return self.pos


class _Bus:
    def stop(self, i):
        pass

    def load(self, i):
        pass

    def unload(self, i):
        pass

    def get_position(self, i):
        return 500

    def get_vin(self, i):
        return 12000


class _Nozzle:
    def on(self):
        pass

    def off(self):
        pass


def _server(out=None):
    t = [0]
    return fw.Server(_Arm(), _Bus(), _Nozzle(), (out if out is not None else []).append,
                     lambda: t[0], lambda a, b: a - b, sys_version="MicroPython v1.19 *weird*")


REQUESTS = [
    ("PING",), ("INFO",), ("STATUS",), ("MOVE", 12.25, -200, 150.5, 1500), ("STOP",),
    ("CLEAR",), ("PUMP", "1"), ("PUMP", "0"), ("TORQUE", "0"), ("TORQUE", "1"),
    ("LIMITS", -250, 250, -280, -60, 40, 225, 60, 260), ("LIMITS", -250, 250, -280, -60, 40, 225),
    ("LIMITS", "OFF"), ("WATCHDOG", 1000), ("HB",), ("BYE",), ("NOPE",), ("MOVE", 1, 2),
]


def test_requests_from_protocol_are_accepted_and_replies_parse():
    out = []
    srv = _server(out)
    for seq, req in enumerate(REQUESTS, start=1):
        for b in P.build_request(seq, *req):
            srv.feed(b)
        assert len(out) == seq, req
        reply = P.parse_reply(P.decode_line(out[-1]))
        assert reply.seq == seq
        if req[0] in ("NOPE",):
            assert (reply.kind, reply.code) == ("ERR", "BADCMD")
        elif req == ("MOVE", 1, 2):
            assert (reply.kind, reply.code) == ("ERR", "BADARG")
        else:
            assert reply.kind == "OK", (req, out[-1])
    assert all(len(line.rstrip("\n")) <= P.MAX_LINE for line in out)


def test_every_firmware_line_is_valid_for_the_pc_codec():
    out = []
    srv = _server(out)
    srv.announce()
    srv.handle_line("garbage")
    for b in b"x" * 300 + b"\n" + b"1 PING*00\n" + b"\xfe\n":
        srv.feed(b)
    srv.wd_ms = 100
    srv.last_rx = -1000
    srv.tick()
    assert len(out) == 5
    kinds = [P.parse_reply(P.decode_line(line)) for line in out]
    assert [(r.kind, r.code) for r in kinds] == [
        ("EVT", "READY"), ("ERR", "TOOLONG"), ("ERR", "CHECKSUM"), ("ERR", "CHECKSUM"),
        ("EVT", "STOPPED")]
    info = srv.handle_line(P.encode_line("2 INFO").decode().rstrip("\n"))
    P.decode_line(fw.frame(info))
    assert P.parse_reply(info).fields["mp"] == "MicroPython_v1.19_?weird?"


def test_number_grammar_agrees():
    for v in (0, 1, -1, 123456, 0.5, -0.5, 12.345, -250.0, 1e-9, 99999.999):
        tok = P.format_number(v)
        parsed = fw.parse_num(tok)
        assert parsed is not None, tok
        assert abs(parsed - float(tok)) < 1e-9
    for tok in ("1e3", "nan", "inf", "+1", "1_0", "", "-", ".", "1.2.3", " 1"):
        assert fw.parse_num(tok) is None, tok
