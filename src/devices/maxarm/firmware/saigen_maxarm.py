# saigen_maxarm.py -- SAIGEN command server for the Hiwonder MaxArm (ESP32).
#
# Runs unchanged on the Hiwonder MicroPython build (ESP32) and on CPython 3.11
# (the unit tests run it on the PC). Therefore: no f-strings, no typing, no
# dataclasses, no time.monotonic (the clock is injected; deadlines only through the
# injected ticks_add/ticks_diff, never plain + or -), %-formatting only,
# a preallocated line buffer, and only `except Exception` (never BaseException),
# so that Ctrl-C (KeyboardInterrupt) always reaches the launcher and the REPL.
#
# Wire protocol "saigen-maxarm" proto 1: see ../README.md. The framing below must
# stay byte-for-byte identical to ../protocol.py (tests/test_maxarm_protocol.py).
#
# The server never moves the arm on its own: the only call that commands motion is
# arm.set_position() inside MOVE. STOP and the watchdog only send SERVO_MOVE_STOP.
# It never changes the suction on its own either: only PUMP does (the host's safety
# wrapper decides what happens to a held object after a fault).
#
# Fault handling of actuator commands (MOVE, TORQUE, PUMP): an unexpected exception
# sends STOP behaviour and sets the stop latch, then ERR INTERNAL. A MOVE whose
# set_position() does not return True (the vendor library swallows its own errors
# and may already have commanded some servos) gets SERVO_MOVE_STOP on all three
# servos and ERR UNREACHABLE (no latch) -- unless one of those stops raised: then
# the stop latch is set and the reply is ERR SERVO (motion may not have stopped).

FIRMWARE_NAME = "saigen-maxarm"
PROTOCOL_VERSION = 1
FIRMWARE_VERSION = "0.1.0"

MAX_LINE = 200
SEQ_MAX = 999999
MOVE_MS_MIN = 100
MOVE_MS_MAX = 30000
WATCHDOG_MS_MIN = 100
WATCHDOG_MS_MAX = 60000
# SuctionNozzle.off() vents the valve in a thread for 1000 ms and ignores on() until
# it is done; PUMP 1 inside this window would be acknowledged but not executed.
# PUMP 1 is also BUSY while the vendor's own flag `nozzle_st` is still True after
# PUMP 0 (vent thread late), so the window is a minimum, not proof of completion.
PUMP_OFF_BUSY_MS = 1100
ACTUATOR_COMMANDS = ("MOVE", "TORQUE", "PUMP")
# Hard kinematic envelope of espmax.ESPMax.set_position (z is silently clamped to
# 225 there and hypot(x, y) < 50 is refused): reject instead of moving elsewhere.
Z_HARD_MAX = 225
R_HARD_MIN = 50

_HEX = "0123456789ABCDEF"


def checksum(text):
    """XOR of all characters of text, as two upper-case hex digits."""
    c = 0
    for ch in text:
        c ^= ord(ch)
    return _HEX[c >> 4] + _HEX[c & 15]


def frame(body):
    """body -> 'body*XX\\n' (body must be printable ASCII without '*')."""
    return body + "*" + checksum(body) + "\n"


def _hexval(s):
    v = 0
    for ch in s:
        i = _HEX.find(ch.upper())
        if i < 0:
            return -1
        v = v * 16 + i
    return v


def _is_digits(s):
    if not s:
        return False
    for ch in s:
        if ch < "0" or ch > "9":
            return False
    return True


def parse_int(tok):
    """Strict decimal integer ('-' allowed, no '+', no '_', no spaces) or None."""
    s = tok[1:] if tok[:1] == "-" else tok
    if not _is_digits(s) or len(s) > 9:
        return None
    return int(tok)


def parse_num(tok):
    """Strict decimal number: [-]digits[.digits] or [-].digits -> int/float, else None."""
    s = tok[1:] if tok[:1] == "-" else tok
    if not s or len(s) > 16:
        return None
    dot = s.find(".")
    if dot < 0:
        return parse_int(tok)
    a = s[:dot]
    b = s[dot + 1:]
    if not (a or b):
        return None
    if (a and not _is_digits(a)) or (b and not _is_digits(b)):
        return None
    v = float(tok)
    if not (v - v == 0):          # NaN / inf guard (cannot happen with this grammar)
        return None
    return v


def fmt_num(v):
    """Echo format: integers as int, others with one decimal."""
    if isinstance(v, int):
        return str(v)
    s = "%.1f" % v
    if s.endswith(".0"):
        s = s[:-2]
    if s == "-0":
        s = "0"
    return s


def sanitize(text, limit=120):
    """Make free text safe for a reply: printable ASCII, no '*', one line."""
    out = []
    n = 0
    for ch in str(text):
        o = ord(ch)
        if o < 0x20:
            ch = " "
        elif o > 0x7E or ch == "*":
            ch = "?"
        out.append(ch)
        n += 1
        if n >= limit:
            break
    return "".join(out)


def token_safe(text, limit=80):
    """Like sanitize, but also turns spaces into '_' (one token)."""
    return sanitize(text, limit).replace(" ", "_")


class _CmdError(Exception):
    # super().__init__, not Exception.__init__: on MicroPython 1.12 the native
    # Exception type has no __init__ attribute (bench, 2026-09-24: every ERR reply
    # became "ERR INTERNAL AttributeError" instead of the intended code).
    def __init__(self, code, text=""):
        super().__init__(code)
        self.code = code
        self.text = text


def _nargs(args, n):
    if len(args) != n:
        raise _CmdError("BADARG", "expected %d argument(s), got %d" % (n, len(args)))


def _plain_ticks_add(a, b):
    return a + b


def _flag(args):
    _nargs(args, 1)
    if args[0] == "1":
        return True
    if args[0] == "0":
        return False
    raise _CmdError("BADARG", "expected 0 or 1")


class Server:
    """Protocol state machine. Hardware objects and the clock are injected.

    arm       : espmax.ESPMax-like (set_position, read_position)
    bus_servo : BusServo.BusServo-like (stop, load, unload, get_position, get_vin)
    nozzle    : SuctionNozzle.SuctionNozzle-like (on, off)
    write     : callable(str) that sends text to the host
    ticks_ms, ticks_diff, ticks_add : time.ticks_ms / ticks_diff / ticks_add
        semantics (ticks_add defaults to plain addition, for unbounded test clocks)
    """

    def __init__(self, arm, bus_servo, nozzle, write, ticks_ms, ticks_diff,
                 version=FIRMWARE_VERSION, sys_version="", ticks_add=None):
        self.arm = arm
        self.bus = bus_servo
        self.nozzle = nozzle
        self.write = write
        self.ticks_ms = ticks_ms
        self.ticks_diff = ticks_diff
        self.ticks_add = ticks_add if ticks_add is not None else _plain_ticks_add
        self.version = version
        self.sys_version = sys_version
        self.buf = bytearray(MAX_LINE)
        self.n = 0
        self.overflow = False
        self.bad_byte = False
        self.stopped = False
        self.wd_ms = 0
        self.limits = None           # (xmin, xmax, ymin, ymax, zmin, zmax, rmin, rmax or None)
        self.pump = False
        self.torque = True
        now = ticks_ms()
        self.move_until = now
        self.motion_active = False   # move_until is only compared while this is True
        self.pump_off_until = now
        self.pump_busy = False       # pump_off_until is only compared while this is True
        self.last_rx = now
        self.commands = {
            "PING": self.cmd_ping,
            "INFO": self.cmd_info,
            "STATUS": self.cmd_status,
            "MOVE": self.cmd_move,
            "STOP": self.cmd_stop,
            "CLEAR": self.cmd_clear,
            "PUMP": self.cmd_pump,
            "TORQUE": self.cmd_torque,
            "LIMITS": self.cmd_limits,
            "WATCHDOG": self.cmd_watchdog,
            "HB": self.cmd_hb,
            "BYE": self.cmd_bye,
        }

    # ------------------------------------------------------------------ I/O
    def send(self, body):
        if "*" in body:                                  # never emit a second '*'
            body = body.replace("*", "?")
        if len(body) > MAX_LINE - 3:
            body = body[:MAX_LINE - 3]
        self.write(frame(body))

    def announce(self):
        """The one-time boot banner (the launcher calls it once)."""
        self.send("EVT READY fw=%s proto=%d ver=%s"
                  % (FIRMWARE_NAME, PROTOCOL_VERSION, token_safe(self.version)))

    def feed(self, byte):
        """Feed one received byte (int 0..255): line assembly and dispatch."""
        if byte == 10:                                   # '\n' ends the line
            n = self.n
            over = self.overflow
            bad = self.bad_byte
            self.n = 0
            self.overflow = False
            self.bad_byte = False
            if over:
                self.send("0 ERR TOOLONG line longer than %d bytes" % MAX_LINE)
                return
            if bad:
                self.send("0 ERR CHECKSUM non-ASCII byte in line")
                return
            if n == 0:
                return                                   # blank line: ignore
            reply = self.handle_line(bytes(memoryview(self.buf)[:n]).decode())
            if reply is not None:
                self.send(reply)
            return
        if byte == 13:                                   # '\r' tolerated and stripped
            return
        if self.overflow:
            return
        if byte < 0x20 or byte > 0x7E:
            self.bad_byte = True
            return
        if self.n >= MAX_LINE:
            self.overflow = True
            self.n = 0
            return
        self.buf[self.n] = byte
        self.n += 1

    def tick(self):
        """Call on every loop pass: watchdog check; expire old deadlines."""
        self._moving()
        self._pump_busy()
        if self.wd_ms > 0 and self.ticks_diff(self.ticks_ms(), self.last_rx) >= self.wd_ms:
            wd = self.wd_ms
            self.wd_ms = 0                               # disabled until re-armed
            ok = self.safe_stop()
            self.send("EVT STOPPED reason=watchdog wd=%d servo_ok=%d" % (wd, 1 if ok else 0))

    def safe_stop(self):
        """STOP behaviour: servos halt where they are, latch set, pump unchanged.

        Best effort: every servo gets its stop even if an earlier one raised.
        Returns True when all three stops were sent without an exception.
        """
        ok = True
        for i in (1, 2, 3):
            try:
                self.bus.stop(i)
            except Exception:
                ok = False
        self.move_until = self.ticks_ms()
        self.motion_active = False
        self.stopped = True
        return ok

    def _halt_servos(self):
        """SERVO_MOVE_STOP to all three servos (best effort), no latch.

        Returns True when all three stops were sent without an exception.
        """
        ok = True
        for i in (1, 2, 3):
            try:
                self.bus.stop(i)
            except Exception:
                ok = False
        self.move_until = self.ticks_ms()
        self.motion_active = False
        return ok

    # ------------------------------------------------------------ dispatch
    def handle_line(self, line):
        """One received line (no terminator, with '*XX') -> reply body, or None."""
        self.tick()                  # an already-expired watchdog fires before this line counts
        line = line.replace("\r", "")
        if not line:
            return None
        star = line.rfind("*")
        sp = line.find(" ")
        if sp > 0:
            head = line[:sp]
        elif star > 0:
            head = line[:star]
        else:
            head = line
        seq = parse_int(head)
        if seq is None or seq < 1 or seq > SEQ_MAX:
            seq = None
        if star < 0 or len(line) - star != 3:
            return "%d ERR CHECKSUM missing checksum" % (seq or 0)
        body = line[:star]
        if _hexval(line[star + 1:]) != _hexval(checksum(body)):
            return "%d ERR CHECKSUM mismatch" % (seq or 0)
        if seq is None:
            return "0 ERR BADARG bad sequence number"
        self.last_rx = self.ticks_ms()                   # valid line: feed the watchdog
        tokens = body.split(" ")
        if len(tokens) < 2 or not tokens[1]:
            return "%d ERR BADCMD missing command" % seq
        for t in tokens:
            if not t:
                return "%d ERR BADARG empty token" % seq
        fn = self.commands.get(tokens[1])
        if fn is None:
            return "%d ERR BADCMD %s" % (seq, sanitize(tokens[1], 20))
        try:
            payload = fn(tokens[2:])
        except _CmdError as e:
            if e.text:
                return "%d ERR %s %s" % (seq, e.code, sanitize(e.text))
            return "%d ERR %s" % (seq, e.code)
        except Exception as e:
            if tokens[1] in ACTUATOR_COMMANDS:
                # partially applied actuator command: halt and latch (pump unchanged)
                ok = self.safe_stop()
                return "%d ERR INTERNAL %s: %s (stop latch set%s)" % (
                    seq, type(e).__name__, sanitize(e, 80), "" if ok else ", servo stop raised")
            return "%d ERR INTERNAL %s: %s" % (seq, type(e).__name__, sanitize(e))
        if payload:
            return "%d OK %s" % (seq, payload)
        return "%d OK" % seq

    # ------------------------------------------------------------ helpers
    def _moving(self):
        if not self.motion_active:
            return False
        if self.ticks_diff(self.move_until, self.ticks_ms()) > 0:
            return True
        self.motion_active = False   # deadline passed: never compare the old value again
        return False

    def _pump_busy(self):
        if not self.pump_busy:
            return False
        if self.ticks_diff(self.pump_off_until, self.ticks_ms()) > 0:
            return True
        self.pump_busy = False
        return False

    def _limits_text(self):
        lim = self.limits
        if lim is None:
            return "limits=0"
        s = "limits=1 xmin=%s xmax=%s ymin=%s ymax=%s zmin=%s zmax=%s" % (
            fmt_num(lim[0]), fmt_num(lim[1]), fmt_num(lim[2]), fmt_num(lim[3]),
            fmt_num(lim[4]), fmt_num(lim[5]))
        if lim[7] is not None:
            s += " rmin=%s rmax=%s" % (fmt_num(lim[6]), fmt_num(lim[7]))
        return s

    # ------------------------------------------------------------ commands
    def cmd_ping(self, args):
        _nargs(args, 0)
        return "pong=1"

    def cmd_info(self, args):
        _nargs(args, 0)
        return "fw=%s proto=%d ver=%s mp=%s limits=%d wd=%d stopped=%d" % (
            FIRMWARE_NAME, PROTOCOL_VERSION, token_safe(self.version),
            token_safe(self.sys_version) or "unknown",
            0 if self.limits is None else 1, self.wd_ms, 1 if self.stopped else 0)

    def cmd_status(self, args):
        _nargs(args, 0)
        pos = self.arm.read_position()
        if not pos:
            raise _CmdError("SERVO", "read_position failed")
        pulses = []
        for i in (1, 2, 3):
            p = self.bus.get_position(i)
            if p is False or p is None:
                raise _CmdError("SERVO", "servo %d position read failed" % i)
            pulses.append(int(p))
        vin = self.bus.get_vin(1)
        if vin is False or vin is None:
            vin = -1
        return "x=%s y=%s z=%s p1=%d p2=%d p3=%d moving=%d stopped=%d pump=%d vin=%d" % (
            fmt_num(pos[0]), fmt_num(pos[1]), fmt_num(pos[2]),
            pulses[0], pulses[1], pulses[2],
            1 if self._moving() else 0, 1 if self.stopped else 0,
            1 if self.pump else 0, int(vin))

    def cmd_move(self, args):
        _nargs(args, 4)
        x = parse_num(args[0])
        y = parse_num(args[1])
        z = parse_num(args[2])
        ms = parse_int(args[3])
        if x is None or y is None or z is None:
            raise _CmdError("BADARG", "x y z must be decimal numbers")
        if ms is None or ms < MOVE_MS_MIN or ms > MOVE_MS_MAX:
            raise _CmdError("BADARG", "ms must be an integer %d..%d" % (MOVE_MS_MIN, MOVE_MS_MAX))
        if self.stopped:
            raise _CmdError("STOPPED", "stop latch set; send CLEAR")
        r = (x * x + y * y) ** 0.5
        lim = self.limits
        if lim is not None:
            if not (lim[0] <= x <= lim[1] and lim[2] <= y <= lim[3] and lim[4] <= z <= lim[5]):
                raise _CmdError("LIMIT", "target outside the device box")
            if lim[7] is not None and not (lim[6] <= r <= lim[7]):
                raise _CmdError("LIMIT", "target radius %s outside %s..%s"
                                % (fmt_num(r), fmt_num(lim[6]), fmt_num(lim[7])))
        if z > Z_HARD_MAX or r < R_HARD_MIN:
            raise _CmdError("UNREACHABLE", "z > %d or radius < %d" % (Z_HARD_MAX, R_HARD_MIN))
        if self.arm.set_position((x, y, z), ms) is not True:
            # the vendor set_position may have commanded some servos before failing
            if not self._halt_servos():
                # a stop raised: the arm may still be moving -> latch, never "halted"
                self.stopped = True
                raise _CmdError("SERVO", "set_position failed and a servo stop raised "
                                "(stop latch set)")
            raise _CmdError("UNREACHABLE", "no IK solution (servos halted)")
        self.move_until = self.ticks_add(self.ticks_ms(), ms)
        self.motion_active = True
        return "x=%s y=%s z=%s ms=%d" % (fmt_num(x), fmt_num(y), fmt_num(z), ms)

    def cmd_stop(self, args):
        _nargs(args, 0)
        if not self.safe_stop():
            raise _CmdError("SERVO", "stop latch set but a servo stop raised")
        return "stopped=1"

    def cmd_clear(self, args):
        _nargs(args, 0)
        self.stopped = False
        return "stopped=0"

    def cmd_pump(self, args):
        on = _flag(args)
        if on:
            if self._pump_busy():
                raise _CmdError("BUSY", "valve still venting after PUMP 0; retry in %d ms"
                                % PUMP_OFF_BUSY_MS)
            # the vendor on() is a silent no-op while nozzle_st is True; after our own
            # PUMP 0 that means its vent thread has not finished yet
            if not self.pump and getattr(self.nozzle, "nozzle_st", False):
                raise _CmdError("BUSY", "valve still venting after PUMP 0 (nozzle_st=1); retry")
            self.nozzle.on()
        else:
            self.nozzle.off()
            self.pump_off_until = self.ticks_add(self.ticks_ms(), PUMP_OFF_BUSY_MS)
            self.pump_busy = True
        self.pump = on
        return "pump=%d" % (1 if on else 0)

    def cmd_torque(self, args):
        on = _flag(args)
        for i in (1, 2, 3):
            if on:
                self.bus.load(i)
            else:
                self.bus.unload(i)
        self.torque = on
        return "torque=%d" % (1 if on else 0)

    def cmd_limits(self, args):
        if len(args) == 1 and args[0] == "OFF":
            self.limits = None
            return "limits=0"
        if len(args) not in (6, 8):
            raise _CmdError("BADARG", "LIMITS xmin xmax ymin ymax zmin zmax [rmin rmax] | OFF")
        vals = []
        for a in args:
            v = parse_num(a)
            if v is None:
                raise _CmdError("BADARG", "limits must be decimal numbers")
            vals.append(v)
        for i in range(0, len(vals), 2):
            if vals[i] > vals[i + 1]:
                raise _CmdError("BADARG", "min > max in pair %d" % (i // 2 + 1))
        if len(vals) == 8:
            if vals[6] < 0:
                raise _CmdError("BADARG", "rmin must be >= 0")
        else:
            vals.append(0)
            vals.append(None)
        self.limits = tuple(vals)
        return self._limits_text()

    def cmd_watchdog(self, args):
        _nargs(args, 1)
        ms = parse_int(args[0])
        if ms is None or not (ms == 0 or WATCHDOG_MS_MIN <= ms <= WATCHDOG_MS_MAX):
            raise _CmdError("BADARG", "ms must be 0 or %d..%d" % (WATCHDOG_MS_MIN, WATCHDOG_MS_MAX))
        self.wd_ms = ms
        self.last_rx = self.ticks_ms()
        return "wd=%d" % ms

    def cmd_hb(self, args):
        _nargs(args, 0)
        return "wd=%d" % self.wd_ms

    def cmd_bye(self, args):
        _nargs(args, 0)
        self.wd_ms = 0
        return "bye=1"
