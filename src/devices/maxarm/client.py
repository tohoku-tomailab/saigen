"""PC-side client of the SAIGEN MaxArm firmware (``saigen-maxarm`` proto 1).

The arm runs ``firmware/main.py`` + ``firmware/saigen_maxarm.py`` (installed with
``deploy_firmware.py`` by a human). This class speaks the checksummed line
protocol described in ``README.md`` over the CH340 USB serial port.

Reset safety: the ESP32 auto-reset circuit acts on DTR/RTS transitions, and a
reset re-runs ``main.py``. ``connect`` therefore sets ``rts=False`` and
``dtr=False`` on the *unopened* port before ``open()``, listens briefly for reset
banners (``rst:``, ``ets ``, ``MicroPython v``, ``boot:``) and refuses to go on if
it sees one (``MaxArmResetDetected``); ``close`` drops RTS first, then DTR.

Thread model: one lock per transaction (write request, read until the reply with
the same sequence number). Long operations (``move_xyz_and_wait``) never hold the
lock while sleeping, so ``stop()`` / ``force_stop()`` from another thread get the
port within one short transaction. A heartbeat thread sends ``HB`` whenever no
other command was sent for ``heartbeat_period`` seconds, so the firmware
watchdog (``WATCHDOG``) stops the arm if this process dies or the link breaks.
``watchdog_ms`` must be at least ``1000*(2*heartbeat_period + timeout)``.

Fault latches (only a new ``connect()`` clears them; STOP is always allowed):

- ``fault_reason`` -- reset banner, ``EVT READY`` after connect or during the
  handshake (after the pre-PING drain), ``EVT FAULT`` or a MicroPython REPL
  prompt: ``connected`` becomes False and every command except STOP is refused
  (the device lost LIMITS and WATCHDOG). ``connect`` also re-reads INFO at the
  end of the handshake and requires ``limits=1`` (when limits were sent) and
  ``wd=<watchdog_ms>``, so a silent reset mid-handshake fails the connect.
- ``link_fault`` -- two consecutive failed heartbeats, a reply reporting a
  watchdog other than the one armed (``wd=0`` after it fired),
  ``EVT STOPPED reason=watchdog``, or an unconfirmed WATCHDOG re-arm after
  CLEAR: one best-effort STOP is sent, the heartbeat stops (so the device
  watchdog fires), and every command except STOP raises ``MaxArmLinkFault``.

An ``EVT STOPPED`` whose ``servo_ok`` is not ``1`` (a servo stop raised on the
device, or the field is missing) makes the client send one more best-effort
STOP as soon as the transaction that read the event has released the lock.

The client never changes suction on its own; the safety wrapper decides.

Example::

    from src.devices.maxarm import MaxArmClient

    arm = MaxArmClient("COM11")
    info = arm.connect(limits={"xmin": -250, "xmax": 250, "ymin": -280, "ymax": -60,
                               "zmin": 40, "zmax": 225}, watchdog_ms=1000)
    st = arm.status()
    if st.stopped:
        arm.clear()
    arm.move_xyz_and_wait(0, -200, 150, 1500)
    arm.close()
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import protocol as P
from .protocol import MaxArmError, MaxArmProtocolError, Reply

logger = logging.getLogger(__name__)

RESET_MARKERS = ("rst:", "ets ", "MicroPython v", "boot:")
REPL_MARKERS = (">>> ", "raw REPL")
MAX_EVENTS = 200
WATCHDOG_MS_MIN = 100          # firmware range for WATCHDOG (0 = disabled)
WATCHDOG_MS_MAX = 60000


class MaxArmCommandError(MaxArmError):
    """The firmware answered ``ERR <code> <text>``."""

    def __init__(self, code: str, text: str = ""):
        self.code = code
        self.text = text
        super().__init__("MaxArm ERR %s%s" % (code, (": " + text) if text else ""))


class MaxArmTimeout(MaxArmError):
    """No (matching) reply in time, or a move did not finish in time."""


class MaxArmResetDetected(MaxArmError):
    """The board printed a reset/boot banner: it rebooted (main.py re-ran)."""


class MaxArmLinkFault(MaxArmError):
    """The link is no longer trusted (heartbeat/watchdog failure); reconnect needed."""


class MaxArmMoveError(MaxArmError):
    """A move finished away from its target, or was interrupted by STOP."""

    def __init__(self, message: str, status: "Optional[MaxArmStatus]" = None):
        self.status = status
        super().__init__(message)


@dataclass(frozen=True)
class MaxArmStatus:
    x: float
    y: float
    z: float
    pulses: Tuple[int, int, int]
    moving: bool
    stopped: bool
    pump: bool
    vin_mv: Optional[int]

    @property
    def position(self) -> List[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_fields(cls, f: Dict[str, str]) -> "MaxArmStatus":
        """Strict: flags must be exactly "0"/"1", coordinates finite, pulses/vin integers."""
        try:
            coords = [float(f[k]) for k in ("x", "y", "z")]
            if not all(math.isfinite(v) for v in coords):
                raise ValueError("non-finite coordinate")
            flags = []
            for k in ("moving", "stopped", "pump"):
                if f[k] not in ("0", "1"):
                    raise ValueError("%s=%r is not 0/1" % (k, f[k]))
                flags.append(f[k] == "1")
            pulses = tuple(int(f[k]) for k in ("p1", "p2", "p3"))
            vin = int(f["vin"])
            return cls(
                x=coords[0], y=coords[1], z=coords[2], pulses=pulses,
                moving=flags[0], stopped=flags[1], pump=flags[2],
                vin_mv=None if vin < 0 else vin,
            )
        except (KeyError, ValueError, TypeError) as e:
            raise MaxArmProtocolError("malformed STATUS payload %r: %s" % (f, e))


_LIMIT_ALIASES = {
    "xmin": ("xmin", "x_min"), "xmax": ("xmax", "x_max"),
    "ymin": ("ymin", "y_min"), "ymax": ("ymax", "y_max"),
    "zmin": ("zmin", "z_min"), "zmax": ("zmax", "z_max"),
    "rmin": ("rmin", "r_min"), "rmax": ("rmax", "r_max"),
}


def limits_to_args(limits: Dict) -> list:
    """Limits dict -> LIMITS arguments.

    Accepts flat keys ``xmin..zmax`` (or ``x_min..z_max``) plus optional
    ``rmin``/``rmax``, or the nested ``WorkspaceValidator.get_limits()`` form
    ``{"x": {"min":..,"max":..}, ..., "r": {...}}``. If only one radius bound is
    given, the other becomes 0 / 100000 (no constraint).
    """
    flat = {}
    for axis in ("x", "y", "z", "r"):
        sub = limits.get(axis)
        if isinstance(sub, dict):
            for b in ("min", "max"):
                if sub.get(b) is not None:
                    flat[axis + b] = sub[b]
    for key, aliases in _LIMIT_ALIASES.items():
        for a in aliases:
            if limits.get(a) is not None:
                flat[key] = limits[a]
                break
    missing = [k for k in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax") if k not in flat]
    if missing:
        raise ValueError("limits missing %s" % ", ".join(missing))
    args = [float(flat[k]) for k in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")]
    if "rmin" in flat or "rmax" in flat:
        args.append(float(flat.get("rmin", 0.0)))
        args.append(float(flat.get("rmax", 100000.0)))
    for v in args:
        if not math.isfinite(v):
            raise ValueError("limits must be finite: %r" % (limits,))
    return args


def _finite_positive(v) -> bool:
    if isinstance(v, bool):
        return False
    try:
        f = float(v)
    except (TypeError, ValueError):
        return False
    return math.isfinite(f) and f > 0


class MaxArmClient:
    """Serial client of the SAIGEN MaxArm firmware. See the module docstring."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5,
                 connect_timeout: float = 3.0, serial_factory: Optional[Callable] = None,
                 heartbeat_period: float = 0.25):
        """
        Args:
            port: COM port of the CH340 (e.g. "COM11"); from config.yaml.
            timeout: per-transaction reply timeout (seconds).
            connect_timeout: overall budget for the connect handshake (seconds).
            serial_factory: callable returning an *unopened* serial.Serial-like
                object (tests inject a fake); default ``serial.Serial``.
            heartbeat_period: HB is sent when no command was sent for this long.
                None disables the heartbeat thread (then use ``watchdog_ms=0``
                or feed the watchdog yourself).

        Raises ValueError unless ``timeout`` and ``connect_timeout`` are finite
        and > 0 and ``heartbeat_period`` is None or finite and > 0 (an infinite
        timeout would hold the transport lock forever and block STOP).
        """
        for name, v in (("timeout", timeout), ("connect_timeout", connect_timeout)):
            if not _finite_positive(v):
                raise ValueError("%s must be finite and > 0, got %r" % (name, v))
        if heartbeat_period is not None and not _finite_positive(heartbeat_period):
            raise ValueError("heartbeat_period must be None or finite and > 0, got %r"
                             % (heartbeat_period,))
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_timeout = connect_timeout
        self.serial_factory = serial_factory
        self.heartbeat_period = heartbeat_period
        self.ser = None
        self.info: Dict[str, str] = {}
        self.events: List[Reply] = []
        self.connected = False
        self.last_error: Optional[BaseException] = None
        self._lock = threading.Lock()
        self._seq = 0
        self._rx = bytearray()
        self._last_tx = 0.0
        self._stop_requested = threading.Event()
        self._hb_stop = threading.Event()        # replaced by a fresh event per connection
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_failures = 0
        self._conn_gen = 0                       # connection generation (old workers bail out)
        self._handshaking = False
        self._closing = False
        self._fault: Optional[str] = None        # reset / firmware fault / REPL: reconnect needed
        self._link_fault: Optional[str] = None   # heartbeat or watchdog failure: reconnect needed
        self._watchdog_ms = 0                    # watchdog armed by this connection (0 = none)
        self._motion_uncertain = False           # a MOVE was sent and not confirmed complete
        self._pending_stop: Optional[str] = None # unconfirmed device stop: STOP after the lock

    # ------------------------------------------------------------------
    # State visible to the safety wrapper
    # ------------------------------------------------------------------
    @property
    def link_fault(self) -> Optional[str]:
        """Reason of a latched link fault (None if none).

        Set after two consecutive failed heartbeats, when a reply reports a
        watchdog other than the one this connection armed (``wd=0`` after the
        device watchdog fired), on ``EVT STOPPED reason=watchdog``, or when a
        WATCHDOG re-arm is not confirmed. While set, every command except STOP
        is refused with ``MaxArmLinkFault``; only a new ``connect()`` clears it.
        """
        return self._link_fault

    @property
    def fault_reason(self) -> Optional[str]:
        """Why the client stopped accepting commands (reset/fault or link fault), or None."""
        return self._fault or self._link_fault

    @property
    def motion_uncertain(self) -> bool:
        """True while a MOVE was sent but its completion was not confirmed."""
        return self._motion_uncertain

    def min_watchdog_ms(self) -> int:
        """Smallest WATCHDOG the heartbeat can keep fed: 1000*(2*heartbeat_period + timeout)."""
        if self.heartbeat_period is None or self.heartbeat_period <= 0:
            return WATCHDOG_MS_MIN
        need = int(math.ceil(1000.0 * (2.0 * self.heartbeat_period + self.timeout) - 1e-6))
        return max(WATCHDOG_MS_MIN, need)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def _new_serial(self):
        if self.serial_factory is not None:
            return self.serial_factory()
        import serial  # lazy: tests never need pyserial
        return serial.Serial()

    def _check_watchdog_ms(self, watchdog_ms) -> int:
        if isinstance(watchdog_ms, bool) or int(watchdog_ms) != watchdog_ms:
            raise ValueError("watchdog_ms must be an integer, got %r" % (watchdog_ms,))
        wd = int(watchdog_ms)
        if wd == 0:
            logger.warning("MaxArm (%s): ウォッチドッグ無効 (watchdog_ms=0) で接続します", self.port)
            return 0
        if not WATCHDOG_MS_MIN <= wd <= WATCHDOG_MS_MAX:
            raise ValueError("watchdog_ms must be 0 or %d..%d, got %d"
                             % (WATCHDOG_MS_MIN, WATCHDOG_MS_MAX, wd))
        need = self.min_watchdog_ms()
        if wd < need:
            raise ValueError(
                "watchdog_ms=%d は短すぎます: ハートビート周期 %.3f s と応答タイムアウト %.3f s では "
                "1000*(2*heartbeat_period + timeout) = %d ms 以上が必要です"
                % (wd, self.heartbeat_period, self.timeout, need))
        return wd

    def connect(self, limits: Optional[Dict] = None, watchdog_ms: int = 1000) -> Dict[str, str]:
        """Open the port without resetting the board and perform the handshake.

        The stop latch is NOT cleared here (the safety wrapper decides).
        Returns the INFO fields. Raises ValueError (before opening anything) if
        ``watchdog_ms`` is shorter than the heartbeat can sustain
        (``min_watchdog_ms()``). A previously open port is closed first; after a
        reset, firmware fault or link fault this is the only way forward, and it
        redoes LIMITS and WATCHDOG.
        """
        wd = self._check_watchdog_ms(watchdog_ms)
        limit_args = limits_to_args(limits) if limits is not None else None
        if self.connected and self.ser is not None and not self.fault_reason:
            return self.info
        self._shutdown_previous()
        ser = self._new_serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.bytesize = 8
        ser.parity = "N"
        ser.stopbits = 1
        ser.timeout = 0.05
        ser.write_timeout = 0.5
        ser.rts = False            # set on the UNOPENED port: open() must not pulse EN
        ser.dtr = False
        ser.open()
        with self._lock:
            self._conn_gen += 1
            self.ser = ser
            self._rx = bytearray()
            self._fault = None
            self._link_fault = None
            self._watchdog_ms = 0
            self._motion_uncertain = False
            self._pending_stop = None
            self._closing = False
            self._hb_failures = 0
            self._hb_stop = threading.Event()
        self._handshaking = True
        try:
            self._handshake(limit_args, wd)
        except BaseException:
            self._handshaking = False
            self._hb_stop.set()
            self._close_port()
            raise
        self._handshaking = False
        self.connected = True
        self._start_heartbeat()
        logger.info("✓ MaxArm 接続 (%s): fw=%s proto=%s ver=%s mp=%s",
                    self.port, self.info.get("fw"), self.info.get("proto"),
                    self.info.get("ver"), self.info.get("mp"))
        return self.info

    def _shutdown_previous(self):
        """Before (re)connecting: stop the old heartbeat and close any open port."""
        self.connected = False
        self._hb_stop.set()
        self._join_heartbeat()
        if self.ser is not None:
            logger.warning("MaxArm (%s): 開いたままのポートを閉じてから再接続します (%s)",
                           self.port, self.fault_reason or "再接続")
            self._close_port()

    def _handshake(self, limit_args, watchdog_ms):
        t_end = time.monotonic() + self.connect_timeout
        listen_end = time.monotonic() + 0.3
        banner = bytearray()
        while time.monotonic() < listen_end:
            banner += self.ser.read(256)
        text = bytes(banner).decode("ascii", "replace")
        for m in RESET_MARKERS:
            if m in text:
                raise MaxArmResetDetected(
                    "MaxArm (%s) がリセットされました（起動メッセージ %r を受信）。"
                    "工場出荷の main.py ならアームが原点へ動いた可能性があります: %r"
                    % (self.port, m, text[:200]))
        if banner:
            logger.debug("MaxArm 接続直後の受信: %r", text[:200])
        self._rx = bytearray()     # drop whatever was pending before our first request
        last_exc: Optional[BaseException] = None
        for _ in range(3):
            if time.monotonic() > t_end:
                break
            try:
                self._transact(P.CMD_PING, timeout=0.5)
                last_exc = None
                break
            except MaxArmTimeout as e:
                last_exc = e
        if last_exc is not None:
            raise MaxArmTimeout(
                "MaxArm (%s) が PING に応答しません。SAIGEN ファームウェアが入っていますか"
                "（工場出荷のプログラムのままでは通信できません）: %s" % (self.port, last_exc))
        info = self._transact(P.CMD_INFO).fields
        if info.get("fw") != P.FIRMWARE_NAME or info.get("proto") != str(P.PROTOCOL_VERSION):
            raise MaxArmProtocolError("unexpected firmware: %r" % (info,))
        self.info = dict(info)
        if limit_args is not None:
            r = self._transact(P.CMD_LIMITS, *limit_args)
            if r.fields.get("limits") != "1":
                raise MaxArmProtocolError("LIMITS not confirmed: %r" % (r.fields,))
        self._arm_watchdog(watchdog_ms)
        # Read the configuration back before declaring the connection ready: a reset
        # in the middle of the handshake (announced or not) loses LIMITS and WATCHDOG.
        final = self._transact(P.CMD_INFO).fields
        want_wd = str(int(watchdog_ms))
        if final.get("wd") != want_wd or (limit_args is not None and final.get("limits") != "1"):
            raise MaxArmProtocolError(
                "MaxArm (%s): device configuration not confirmed after the handshake "
                "(want wd=%s%s): %r" % (self.port, want_wd,
                                         "" if limit_args is None else " limits=1", final))
        self.info = dict(final)
        self.info["wd"] = want_wd
        if limit_args is not None:
            self.info["limits"] = "1"

    def _arm_watchdog(self, watchdog_ms: int) -> None:
        """WATCHDOG <ms> and verify the echo (``wd=<ms>``).

        ``_watchdog_ms`` is updated by ``_transact`` under the transport lock as soon
        as the verified reply arrives, so a heartbeat can never compare the new
        device value against the old expectation.
        """
        r = self._transact(P.CMD_WATCHDOG, int(watchdog_ms))
        if r.fields.get("wd") != str(int(watchdog_ms)):
            raise MaxArmProtocolError("WATCHDOG %d not confirmed: %r" % (watchdog_ms, r.fields))

    def close(self) -> None:
        """Shut down: STOP if a move is unconfirmed, BYE, stop the heartbeat, close the port.

        New commands (except STOP) are refused from the start of close(). If a
        MOVE was sent and not confirmed complete (``motion_uncertain``), STOP is
        sent first; if that STOP is not acknowledged, BYE is skipped so that the
        device watchdog stays armed and stops the arm. BYE is also skipped after
        a reset, firmware fault or link fault. Suction is never changed here.
        """
        was_connected = self.connected and self.ser is not None
        self._closing = True
        self.connected = False
        self._hb_stop.set()
        send_bye = was_connected and not self.fault_reason
        if self.ser is not None and self._motion_uncertain:
            logger.warning("MaxArm (%s): 移動の完了が未確認のまま切断します。先に STOP を送ります",
                           self.port)
            if not self.force_stop():
                logger.error("MaxArm (%s): STOP が確認できないため BYE を送りません"
                             "（ファームウェアのウォッチドッグが有効なまま残ります）", self.port)
                send_bye = False
        if send_bye:
            try:
                self._transact(P.CMD_BYE)
            except Exception as e:
                logger.warning("MaxArm BYE 失敗（無視して切断）: %s", e)
        self._join_heartbeat()
        self._close_port()
        self._closing = False

    def _close_port(self):
        with self._lock:
            ser, self.ser = self.ser, None
            self._conn_gen += 1          # any worker of the old connection is now stale
        if ser is None:
            return
        for step in ("rts", "dtr"):
            try:
                setattr(ser, step, False)
            except Exception as e:
                logger.debug("MaxArm %s=False 失敗: %s", step, e)
        try:
            ser.close()
        except Exception as e:
            logger.debug("MaxArm close 失敗: %s", e)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------
    # Faults
    # ------------------------------------------------------------------
    def _set_fault(self, reason: str) -> None:
        """Reset / firmware fault / REPL: refuse everything but STOP until connect()."""
        if self._fault is None:
            self._fault = reason
            logger.error("MaxArm (%s): %s（再接続するまで STOP 以外のコマンドを拒否します）",
                         self.port, reason)
        self.connected = False
        self._hb_stop.set()

    def _mark_link_fault(self, reason: str) -> bool:
        """Latch a link fault without any I/O. True if it was newly set."""
        if self._link_fault is not None:
            return False
        self._link_fault = reason
        self.connected = False           # connected <=> non-STOP commands are accepted
        self._hb_stop.set()
        self._stop_requested.set()       # a waiting move ends at once
        logger.error("MaxArm (%s): 通信異常 (%s)。ハートビートを止め、再接続するまで STOP 以外の"
                     "コマンドを拒否します", self.port, reason)
        return True

    def _enter_link_fault(self, reason: str, send_stop: bool = True) -> None:
        """Latch a link fault and send one best-effort STOP (call without the lock held)."""
        if self._mark_link_fault(reason) and send_stop and self.ser is not None:
            self.force_stop()

    def _gate(self, cmd: str, gen: Optional[int]) -> None:
        """Raise unless ``cmd`` may be sent now (called with the lock held)."""
        if self.ser is None:
            raise MaxArmError("MaxArm (%s) は接続されていません" % self.port)
        if gen is not None and gen != self._conn_gen:
            raise MaxArmError("MaxArm (%s): 古い接続からの送信を破棄" % self.port)
        if cmd == P.CMD_STOP:
            return                                   # STOP is always allowed
        if self._fault is not None:
            raise MaxArmError("MaxArm (%s): %s。再接続 (connect) が必要です" % (self.port, self._fault))
        if self._link_fault is not None:
            raise MaxArmLinkFault("MaxArm (%s): link fault (%s)。再接続 (connect) が必要です"
                                  % (self.port, self._link_fault))
        if self._handshaking:
            return
        if self._closing and cmd == P.CMD_BYE:
            return
        if not self.connected:
            raise MaxArmError("MaxArm (%s) は接続されていません" % self.port)

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = self._seq + 1 if self._seq < P.SEQ_MAX else 1
        return self._seq

    def _readline(self, deadline: float, ser=None) -> bytes:
        ser = self.ser if ser is None else ser
        while True:
            nl = self._rx.find(b"\n")
            if nl >= 0:
                line = bytes(self._rx[:nl])
                del self._rx[:nl + 1]
                return line
            if time.monotonic() >= deadline:
                if self._rx:                      # e.g. a ">>> " prompt without newline
                    self._check_banner(bytes(self._rx))
                raise MaxArmTimeout("no reply within the timeout")
            waiting = getattr(ser, "in_waiting", 0) or 1
            chunk = ser.read(min(max(waiting, 1), 512))
            if chunk:
                self._rx += chunk
                if len(self._rx) > 4096:          # garbage without newlines
                    self._check_banner(bytes(self._rx))
                    del self._rx[:-256]

    def _check_banner(self, raw: bytes):
        text = raw.decode("ascii", "replace")
        for m in RESET_MARKERS:
            if m in text:
                self._set_fault("リセットを検出 (%r)" % m)
                raise MaxArmResetDetected(
                    "MaxArm (%s) がリセットされました（%r を受信）: %r" % (self.port, m, text[:200]))
        for m in REPL_MARKERS:
            if m in text:
                self._set_fault("MicroPython REPL を検出")
                raise MaxArmProtocolError(
                    "MaxArm (%s) は MicroPython REPL に落ちています（サーバー停止）: %r"
                    % (self.port, text[:200]))

    def _handle_event(self, reply: Reply):
        self.events.append(reply)
        if len(self.events) > MAX_EVENTS:
            del self.events[:-MAX_EVENTS]
        if reply.code == "STOPPED":
            logger.warning("MaxArm イベント: %s %s", reply.code, reply.text)
            if reply.fields.get("reason") == "watchdog":
                # the device stopped the arm and disabled its watchdog (wd=0): moving on
                # would run without protection, so this connection is finished
                self._mark_link_fault("device watchdog fired (EVT STOPPED reason=watchdog)")
            if reply.fields.get("servo_ok") != "1":
                # the device could not confirm every servo stop (or did not say):
                # one more best-effort STOP once the current transaction is over
                logger.error("MaxArm (%s): デバイス側の停止が未確認です (servo_ok=%s)。"
                             "STOP を再送します", self.port, reply.fields.get("servo_ok"))
                self._pending_stop = "EVT STOPPED servo_ok=%s" % reply.fields.get("servo_ok")
        elif reply.code == "READY" and (self.connected or self._closing or self._handshaking):
            # The boot announcement printed before connect() is dropped with the
            # pre-PING drain in _handshake; any READY after that is a reset.
            self._set_fault("ファームウェアが再起動しました (EVT READY%s)"
                            % (", handshake" if self._handshaking else ""))
            raise MaxArmResetDetected(
                "MaxArm (%s) のファームウェアが再起動しました (EVT READY)" % self.port)
        elif reply.code == "FAULT":
            self._set_fault("ファームウェア異常終了 (EVT FAULT %s)" % reply.text)
            raise MaxArmProtocolError("MaxArm ファームウェア異常終了: %s" % reply.text)
        else:
            logger.info("MaxArm イベント: %s %s", reply.code, reply.text)

    def _transact(self, cmd: str, *args, timeout: Optional[float] = None,
                  _gen: Optional[int] = None) -> Reply:
        """One request/reply exchange under the lock. Raises on ERR.

        Refuses everything but STOP unless the connection is ready (or being
        set up), and while a reset/firmware fault or a link fault is latched.
        """
        timeout = self.timeout if timeout is None else timeout
        try:
            return self._transact_locked(cmd, args, timeout, _gen)
        finally:
            if self._pending_stop is not None:
                reason, self._pending_stop = self._pending_stop, None
                # a STOP transaction that read the event already was the retry
                if cmd != P.CMD_STOP and self.ser is not None:
                    logger.warning("MaxArm (%s): 未確認の停止 (%s) のため STOP を送ります",
                                   self.port, reason)
                    self.force_stop()

    def _transact_locked(self, cmd: str, args: tuple, timeout: float,
                         _gen: Optional[int]) -> Reply:
        wd_fault = None
        with self._lock:
            self._gate(cmd, _gen)
            ser = self.ser
            seq = self._next_seq()
            ser.write(P.build_request(seq, cmd, *args))
            self._last_tx = time.monotonic()
            deadline = self._last_tx + timeout
            while True:
                try:
                    raw = self._readline(deadline, ser)
                except MaxArmTimeout:
                    raise MaxArmTimeout("MaxArm (%s): %s に %.1f 秒以内の応答なし"
                                        % (self.port, cmd, timeout))
                if not raw.strip(b"\r"):
                    continue
                try:
                    reply = P.parse_reply(P.decode_line(raw))
                except MaxArmProtocolError as e:
                    self._check_banner(raw)
                    logger.warning("MaxArm 受信行を破棄: %s", e)
                    continue
                if reply.kind == P.KIND_EVT:
                    self._handle_event(reply)
                    continue
                if reply.seq == seq or (reply.seq == 0 and reply.kind == P.KIND_ERR):
                    break
                logger.debug("MaxArm: 古い応答を破棄 (seq %s, 待機中 %d): %s",
                             reply.seq, seq, raw[:80])
            if (cmd == P.CMD_WATCHDOG and reply.kind == P.KIND_OK and args
                    and reply.fields.get("wd") == str(int(args[0]))):
                self._watchdog_ms = int(args[0])          # verified: update under the lock
            if (reply.kind == P.KIND_OK and self._watchdog_ms > 0 and "wd" in reply.fields
                    and cmd not in (P.CMD_WATCHDOG, P.CMD_BYE)
                    and reply.fields["wd"] != str(self._watchdog_ms)):
                wd_fault = "device reports wd=%s, armed %d" % (reply.fields["wd"], self._watchdog_ms)
        if wd_fault is not None:
            self._enter_link_fault(wd_fault)
            raise MaxArmLinkFault("MaxArm (%s): link fault (%s)" % (self.port, wd_fault))
        if reply.kind == P.KIND_ERR:
            raise MaxArmCommandError(reply.code or "?", reply.text)
        return reply

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def ping(self) -> bool:
        return self._transact(P.CMD_PING).fields.get("pong") == "1"

    def get_info(self) -> Dict[str, str]:
        self.info = dict(self._transact(P.CMD_INFO).fields)
        return self.info

    def status(self) -> MaxArmStatus:
        return MaxArmStatus.from_fields(self._transact(P.CMD_STATUS).fields)

    def move_xyz(self, x: float, y: float, z: float, ms: int) -> None:
        """Send MOVE; returns when the firmware accepted it (open loop).

        ``motion_uncertain`` becomes True just before the MOVE is sent and stays
        True until ``move_xyz_and_wait`` confirms the arrival or a STOP is
        acknowledged.
        """
        ms = int(ms)
        if not P.MOVE_MS_MIN <= ms <= P.MOVE_MS_MAX:
            raise ValueError("ms must be %d..%d, got %d" % (P.MOVE_MS_MIN, P.MOVE_MS_MAX, ms))
        for v in (x, y, z):
            if not math.isfinite(float(v)):
                raise ValueError("coordinates must be finite: %r" % ((x, y, z),))
        self._motion_uncertain = True
        self._transact(P.CMD_MOVE, float(x), float(y), float(z), ms)

    def move_xyz_and_wait(self, x: float, y: float, z: float, ms: int,
                          tolerance_mm: float = 10.0, timeout_margin: float = 2.0,
                          poll_interval: float = 0.1) -> MaxArmStatus:
        """MOVE, then poll STATUS until ``moving == 0`` and check the read-back.

        Raises MaxArmTimeout if the move is not over after ``ms/1000 + margin``;
        MaxArmMoveError if STOP interrupted it or the read-back is further than
        ``tolerance_mm`` from the target on any axis; MaxArmLinkFault if a link
        fault ended the wait. The lock is never held while sleeping, so stop()
        from another thread goes through at once. ``tolerance_mm`` must be
        finite and > 0, ``timeout_margin`` finite and >= 0 (ValueError, nothing
        sent). On any failure after the MOVE was sent, ``motion_uncertain``
        stays True; the client never sends STOP on its own here (the safety
        wrapper does, and close() does).
        """
        tol = float(tolerance_mm)
        if not (math.isfinite(tol) and tol > 0):
            raise ValueError("tolerance_mm must be finite and > 0, got %r" % (tolerance_mm,))
        margin = float(timeout_margin)
        poll = float(poll_interval)
        if not (math.isfinite(margin) and margin >= 0) or not (math.isfinite(poll) and poll > 0):
            raise ValueError("timeout_margin must be finite and >= 0 and poll_interval finite "
                             "and > 0, got %r / %r" % (timeout_margin, poll_interval))
        self._stop_requested.clear()
        self.move_xyz(x, y, z, ms)
        deadline = time.monotonic() + ms / 1000.0 + margin
        while True:
            if self._stop_requested.wait(poll):
                if self._link_fault is not None:
                    raise MaxArmLinkFault("MaxArm の移動は通信異常で中断されました (%s)"
                                          % self._link_fault)
                st = None
                try:
                    st = self.status()
                except MaxArmError as e:
                    logger.warning("MaxArm 停止後の状態読み出し失敗: %s", e)
                raise MaxArmMoveError("MaxArm の移動は STOP で中断されました", st)
            st = self.status()
            if st.stopped:
                raise MaxArmMoveError("MaxArm の移動は停止ラッチにより中断されました", st)
            if not st.moving:
                break
            if time.monotonic() > deadline:
                raise MaxArmTimeout("MaxArm の移動が %.1f 秒以内に終わりません"
                                    % (ms / 1000.0 + margin))
        err = max(abs(st.x - x), abs(st.y - y), abs(st.z - z))
        if not err <= tol:                      # also catches NaN
            raise MaxArmMoveError(
                "MaxArm 到達位置のずれが許容値を超えました: 目標 (%.1f, %.1f, %.1f), "
                "読み出し (%.1f, %.1f, %.1f), 最大 %.1f mm > %.1f mm"
                % (x, y, z, st.x, st.y, st.z, err, tol), st)
        self._motion_uncertain = False
        return st

    def stop(self) -> None:
        """STOP (servos halt where they are, stop latch set). Thread-safe, always allowed."""
        self._stop_requested.set()
        self._transact(P.CMD_STOP)
        self._motion_uncertain = False

    def force_stop(self) -> bool:
        """Best-effort STOP for the safety wrapper: True if acknowledged. Always allowed."""
        self._stop_requested.set()
        try:
            self._transact(P.CMD_STOP)
            self._motion_uncertain = False
            logger.warning("MaxArm (%s): STOP 送信・応答確認", self.port)
            return True
        except Exception as e:
            logger.error("MaxArm (%s): STOP が確認できません: %s", self.port, e)
            return False

    def clear(self) -> None:
        """CLEAR the stop latch, then re-arm and verify this connection's watchdog.

        The device watchdog disables itself after it fires; re-sending WATCHDOG
        after every CLEAR makes sure no move runs unprotected. If the re-arm is
        not confirmed, a link fault is latched (one STOP is sent) and
        MaxArmLinkFault is raised.
        """
        self._transact(P.CMD_CLEAR)
        if self._watchdog_ms > 0:
            wd = self._watchdog_ms
            try:
                self._arm_watchdog(wd)
            except MaxArmLinkFault:
                raise
            except Exception as e:
                self._enter_link_fault("WATCHDOG %d re-arm after CLEAR failed: %s" % (wd, e))
                raise MaxArmLinkFault("MaxArm (%s): CLEAR 後のウォッチドッグ再設定に失敗: %s"
                                      % (self.port, e))

    def pump(self, on: bool) -> None:
        """PUMP 1/0. The firmware answers ``ERR BUSY`` to PUMP 1 within ~1.1 s of PUMP 0."""
        self._transact(P.CMD_PUMP, "1" if on else "0")

    def torque(self, on: bool) -> None:
        self._transact(P.CMD_TORQUE, "1" if on else "0")

    def set_limits(self, limits: Optional[Dict]) -> None:
        if limits is None:
            self._transact(P.CMD_LIMITS, "OFF")
        else:
            self._transact(P.CMD_LIMITS, *limits_to_args(limits))

    def set_watchdog(self, ms: int) -> None:
        """Change this connection's watchdog (0 disables it; checked like connect)."""
        self._arm_watchdog(self._check_watchdog_ms(ms))

    def heartbeat(self) -> None:
        self._transact(P.CMD_HB)

    # ------------------------------------------------------------------
    # Heartbeat thread
    # ------------------------------------------------------------------
    def _start_heartbeat(self):
        if self.heartbeat_period is None or self.heartbeat_period <= 0:
            return
        self._last_tx = time.monotonic()
        self._hb_failures = 0
        gen, stop_event = self._conn_gen, self._hb_stop
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, args=(gen, stop_event),
                                           name="maxarm-heartbeat-%s" % self.port, daemon=True)
        self._hb_thread.start()

    def _join_heartbeat(self):
        t, self._hb_thread = self._hb_thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=self.timeout + (self.heartbeat_period or 0) + 1.0)
            if t.is_alive():
                logger.warning("MaxArm (%s): ハートビートスレッドが終了しません"
                               "（古い接続として無効化済み）", self.port)

    def _heartbeat_loop(self, gen: int, stop_event: threading.Event):
        """HB when idle. Two consecutive failures latch a link fault: one STOP is
        sent (best effort) and no further HB, so the device watchdog fires."""
        period = self.heartbeat_period
        while not stop_event.wait(period / 4.0):
            if gen != self._conn_gen or not self.connected or self.fault_reason:
                return
            if time.monotonic() - self._last_tx < period:
                continue
            try:
                self._transact(P.CMD_HB, _gen=gen)
                if self._hb_failures:
                    logger.info("MaxArm (%s): ハートビート復旧", self.port)
                self._hb_failures = 0
            except MaxArmLinkFault as e:
                self.last_error = e
                return
            except Exception as e:
                if stop_event.is_set() or gen != self._conn_gen:
                    return
                self.last_error = e
                if self._fault is not None or not self.connected:
                    return                        # reset / firmware fault: already latched
                self._hb_failures += 1
                if self._hb_failures < 2:
                    logger.warning("MaxArm (%s): ハートビート失敗 (1 回目): %s", self.port, e)
                    continue
                self._enter_link_fault("heartbeat failed %d times in a row: %s"
                                       % (self._hb_failures, e))
                return
