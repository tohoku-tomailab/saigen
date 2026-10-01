"""Wire codec of the ``saigen-maxarm`` line protocol (proto 1), PC side.

The same framing is implemented, byte for byte, by the device firmware in
``firmware/saigen_maxarm.py`` (which must also run on MicroPython, so it cannot
import this module). ``tests/test_maxarm_protocol.py`` checks that the two agree.

Frame grammar (both directions, ASCII only, ``\n`` terminated, ``\r`` ignored)::

    line     = body "*" XX "\n"          XX = XOR of all bytes of body, 2 hex digits
    request  = seq " " CMD [" " arg]*     seq = 1..999999 (decimal, chosen by the PC)
    reply    = seq " OK" [" " key=value]*  |  seq " ERR " CODE [" " free text]
    event    = "EVT " NAME [" " key=value]*

A line (body plus ``*XX``, without the terminator) is at most ``MAX_LINE`` bytes.
Numbers are written in plain decimal notation (no exponent), so that the tiny
parser in the firmware accepts exactly what ``format_number`` produces.
"""

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Union

FIRMWARE_NAME = "saigen-maxarm"
PROTOCOL_VERSION = 1
MAX_LINE = 200
SEQ_MAX = 999999

# Commands
CMD_PING = "PING"
CMD_INFO = "INFO"
CMD_STATUS = "STATUS"
CMD_MOVE = "MOVE"
CMD_STOP = "STOP"
CMD_CLEAR = "CLEAR"
CMD_PUMP = "PUMP"
CMD_TORQUE = "TORQUE"
CMD_LIMITS = "LIMITS"
CMD_WATCHDOG = "WATCHDOG"
CMD_HB = "HB"
CMD_BYE = "BYE"
COMMANDS = frozenset({
    CMD_PING, CMD_INFO, CMD_STATUS, CMD_MOVE, CMD_STOP, CMD_CLEAR, CMD_PUMP,
    CMD_TORQUE, CMD_LIMITS, CMD_WATCHDOG, CMD_HB, CMD_BYE,
})

# Error codes
ERR_BADCMD = "BADCMD"
ERR_BADARG = "BADARG"
ERR_CHECKSUM = "CHECKSUM"
ERR_TOOLONG = "TOOLONG"
ERR_STOPPED = "STOPPED"
ERR_LIMIT = "LIMIT"
ERR_UNREACHABLE = "UNREACHABLE"
ERR_SERVO = "SERVO"
ERR_INTERNAL = "INTERNAL"
ERR_BUSY = "BUSY"
ERROR_CODES = frozenset({
    ERR_BADCMD, ERR_BADARG, ERR_CHECKSUM, ERR_TOOLONG, ERR_STOPPED, ERR_LIMIT,
    ERR_UNREACHABLE, ERR_SERVO, ERR_INTERNAL, ERR_BUSY,
})

# Reply kinds
KIND_OK = "OK"
KIND_ERR = "ERR"
KIND_EVT = "EVT"

# MOVE duration range (ms), enforced by the firmware
MOVE_MS_MIN = 100
MOVE_MS_MAX = 30000


class MaxArmError(RuntimeError):
    """Base class of every MaxArm driver error."""


class MaxArmProtocolError(MaxArmError):
    """Malformed or unexpected data on the wire (bad checksum, wrong firmware, ...)."""


def checksum(text: str) -> str:
    """NMEA-style checksum: XOR of all bytes of ``text`` as two upper-case hex digits."""
    c = 0
    for b in text.encode("ascii"):
        c ^= b
    return "%02X" % c


def _check_body(text: str) -> None:
    if not isinstance(text, str):
        raise TypeError("line body must be str, got %r" % type(text).__name__)
    for ch in text:
        o = ord(ch)
        if o < 0x20 or o > 0x7E or ch == "*":
            raise ValueError("line body contains a forbidden character: %r" % ch)


def encode_line(text: str) -> bytes:
    """Frame ``text`` as ``text*XX\n`` (ASCII). Raises ValueError if too long."""
    _check_body(text)
    line = "%s*%s" % (text, checksum(text))
    if len(line) > MAX_LINE:
        raise ValueError("line too long (%d > %d bytes)" % (len(line), MAX_LINE))
    return (line + "\n").encode("ascii")


def decode_line(raw: Union[bytes, bytearray, str]) -> str:
    """Verify and strip the ``*XX`` checksum; return the body.

    Trailing ``\r``/``\n`` are stripped. Raises ``MaxArmProtocolError`` for
    non-ASCII data, a missing or malformed ``*XX`` or a checksum mismatch.
    """
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = bytes(raw).decode("ascii")
        except UnicodeDecodeError:
            raise MaxArmProtocolError("non-ASCII line: %r" % (bytes(raw)[:80],))
    line = raw.replace("\r", "").rstrip("\n")
    star = line.rfind("*")
    if star < 0 or len(line) - star != 3:
        raise MaxArmProtocolError("missing checksum: %r" % line[:80])
    body, given = line[:star], line[star + 1:]
    try:
        _check_body(body)
        value = int(given, 16) if all(c in "0123456789abcdefABCDEF" for c in given) else -1
    except ValueError:
        raise MaxArmProtocolError("bad characters in line: %r" % line[:80])
    if value != int(checksum(body), 16):
        raise MaxArmProtocolError("checksum mismatch: %r" % line[:80])
    return body


def format_number(value) -> str:
    """Format a number the way the firmware parser accepts it (no exponent).

    Integers as ``str(int)``; floats with up to 3 decimals, trailing zeros removed.
    Raises ValueError for bool, NaN and infinities.
    """
    if isinstance(value, bool):
        raise ValueError("bool is not a number here: %r" % value)
    if isinstance(value, int):
        return str(value)
    v = float(value)
    if v != v or v in (float("inf"), float("-inf")):
        raise ValueError("number must be finite, got %r" % value)
    s = ("%.3f" % v).rstrip("0").rstrip(".")
    if s in ("-0", ""):
        s = "0"
    return s


def build_request(seq: int, cmd: str, *args) -> bytes:
    """Build a framed request line: ``<seq> <CMD> [arg ...]*XX\n``."""
    if isinstance(seq, bool) or not isinstance(seq, int) or not 1 <= seq <= SEQ_MAX:
        raise ValueError("seq must be an int in 1..%d, got %r" % (SEQ_MAX, seq))
    if not cmd or " " in cmd:
        raise ValueError("bad command name %r" % cmd)
    parts = [str(seq), cmd]
    for a in args:
        if isinstance(a, str):
            if not a or " " in a:
                raise ValueError("string argument must be one non-empty token: %r" % a)
            parts.append(a)
        else:
            parts.append(format_number(a))
    return encode_line(" ".join(parts))


def parse_kv(tokens: Iterable[str]) -> Dict[str, str]:
    """``["x=1", "y=2"]`` -> ``{"x": "1", "y": "2"}``. Tokens without ``=`` are ignored."""
    out = {}
    for tok in tokens:
        eq = tok.find("=")
        if eq > 0:
            out[tok[:eq]] = tok[eq + 1:]
    return out


@dataclass(frozen=True)
class Reply:
    """A decoded reply or event line.

    ``seq`` is None for events; ``code`` is the error code (ERR) or the event
    name (EVT) and None for OK; ``text`` is everything after the kind/code.
    """
    seq: Optional[int]
    kind: str
    code: Optional[str]
    fields: Dict[str, str] = field(default_factory=dict)
    text: str = ""

    @property
    def ok(self) -> bool:
        return self.kind == KIND_OK


def parse_reply(line: str) -> Reply:
    """Parse a reply/event body (checksum already stripped by ``decode_line``)."""
    tokens = line.split(" ")
    if not tokens or not tokens[0]:
        raise MaxArmProtocolError("empty line")
    if tokens[0] == KIND_EVT:
        if len(tokens) < 2 or not tokens[1]:
            raise MaxArmProtocolError("event without a name: %r" % line)
        return Reply(None, KIND_EVT, tokens[1], parse_kv(tokens[2:]), " ".join(tokens[2:]))
    if not tokens[0].isdigit() or len(tokens[0]) > 6:
        raise MaxArmProtocolError("bad sequence number in %r" % line)
    seq = int(tokens[0])
    if len(tokens) < 2:
        raise MaxArmProtocolError("reply without a kind: %r" % line)
    kind = tokens[1]
    if kind == KIND_OK:
        return Reply(seq, KIND_OK, None, parse_kv(tokens[2:]), " ".join(tokens[2:]))
    if kind == KIND_ERR:
        if len(tokens) < 3 or not tokens[2]:
            raise MaxArmProtocolError("ERR reply without a code: %r" % line)
        return Reply(seq, KIND_ERR, tokens[2], {}, " ".join(tokens[3:]))
    raise MaxArmProtocolError("unknown reply kind %r in %r" % (kind, line))
