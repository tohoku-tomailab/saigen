"""deploy_firmware --catch-boot: interrupt the factory main.py during its boot delay.

Hiwonder's serial-slave main.py sleeps 10 s, then takes over the USB UART (9600 bps)
and homes the arm. catch_boot() sends Ctrl-C continuously and must reach the
friendly REPL (">>> ") as soon as the board boots, i.e. before go_home. No hardware.
"""
import pytest

from src.devices.maxarm import deploy_firmware as df


class BootingBoard:
    """Fake serial: silent until 'powered on'; then prints a boot banner and, if a
    Ctrl-C arrives within the boot delay, the KeyboardInterrupt + '>>> ' prompt.
    After the delay the factory program owns the UART and ignores everything."""

    def __init__(self, boot_at=0.5, delay=10.0, banner=b"ets Jun  8 2016\r\nrst:0x1 (POWERON_RESET)\r\nMicroPython v1.19.1\r\n"):
        self.t = 0.0
        self.boot_at = boot_at
        self.delay = delay
        self.banner = banner
        self.rx = b""
        self.writes = []
        self.booted = False
        self.interrupted = False
        self.rts = False
        self.dtr = False

    # clock / sleep injected into catch_boot
    def clock(self):
        return self.t

    def sleep(self, s):
        self.t += s
        if not self.booted and self.t >= self.boot_at:
            self.booted = True
            self.boot_t = self.t
            self.rx += self.banner

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def write(self, data):
        self.writes.append(data)
        if self.booted and not self.interrupted and b"\x03" in data:
            if self.t < self.boot_t + self.delay:
                self.interrupted = True
                self.rx += b"\r\nTraceback (most recent call last):\r\n  File \"main.py\"\r\nKeyboardInterrupt: \r\n>>> "
            # after the delay the UART belongs to the 9600-bps slave loop: silence


def test_catch_boot_interrupts_main_py_before_go_home():
    board = BootingBoard(boot_at=2.0)
    out = df.catch_boot(board, wait=60.0, sleep=board.sleep, clock=board.clock, log=lambda m: None)
    assert board.interrupted
    assert out.endswith(b">>> ")
    # interrupted well inside the 10 s delay: the arm never moved
    assert board.t < board.boot_t + 1.0
    assert all(w == b"\x03" for w in board.writes)


def test_catch_boot_works_when_boot_happened_before_listening():
    """Banner already gone (user power-cycled early): Ctrl-C still lands in the sleep."""
    board = BootingBoard(boot_at=0.0, banner=b"")
    board.sleep(0.0)                      # booted, no banner queued
    out = df.catch_boot(board, wait=30.0, sleep=board.sleep, clock=board.clock, log=lambda m: None)
    assert board.interrupted and out.endswith(b">>> ")


def test_catch_boot_reports_when_the_factory_program_takes_over():
    """Boot seen but the prompt never comes (delay expired): clear error, arm may have homed."""
    board = BootingBoard(boot_at=0.5, delay=0.0)   # slave loop takes over immediately
    with pytest.raises(df.ReplError) as e:
        df.catch_boot(board, wait=60.0, window=3.0, sleep=board.sleep, clock=board.clock,
                      log=lambda m: None)
    assert "taken over" in str(e.value)
    assert board.t <= 0.5 + 3.0 + 0.2


def test_catch_boot_times_out_without_any_boot():
    board = BootingBoard(boot_at=1e9)
    with pytest.raises(df.ReplError) as e:
        df.catch_boot(board, wait=5.0, sleep=board.sleep, clock=board.clock, log=lambda m: None)
    assert "power-cycled" in str(e.value)


def test_pulse_rts_reset_keeps_dtr_low_and_ends_with_rts_low():
    class Ser:
        def __init__(self):
            self.events = []
        def __setattr__(self, k, v):
            if k in ("rts", "dtr"):
                self.__dict__.setdefault("events", []).append((k, v))
            object.__setattr__(self, k, v)
    s = Ser()
    df.pulse_rts_reset(s, sleep=lambda t: None)
    assert s.events == [("dtr", False), ("rts", True), ("rts", False)]
    assert s.rts is False and s.dtr is False


def test_cli_rts_reset_requires_catch_boot(capsys):
    with pytest.raises(SystemExit):
        df.main(["--port", "COMX", "--list", "--rts-reset"])
    assert "need --catch-boot" in capsys.readouterr().err


def test_wait_for_replug_returns_the_reopened_port():
    """Port present -> gone (cable out) -> back: the port opened after the gap is returned."""
    t = {"now": 0.0}
    opened = []

    class Ser:
        def __init__(self):
            self.port = None
            self.closed = False
        def open(self):
            # present until t=1, absent 1..3, present again from t=3
            if 1.0 <= t["now"] < 3.0:
                raise OSError("no such port")
            opened.append(t["now"])
        def close(self):
            self.closed = True

    def sleep(s):
        t["now"] += s

    ser = df.wait_for_replug("COMX", serial_factory=Ser, wait=30.0, sleep=sleep,
                             clock=lambda: t["now"], log=lambda m: None)
    assert ser.closed is False and opened[-1] >= 3.0
    assert ser.rts is False and ser.dtr is False


def test_wait_for_replug_times_out_if_the_cable_is_never_pulled():
    t = {"now": 0.0}

    class Ser:
        def __init__(self):
            self.port = None
        def open(self):
            pass
        def close(self):
            pass

    with pytest.raises(df.ReplError):
        df.wait_for_replug("COMX", serial_factory=Ser, wait=5.0,
                           sleep=lambda s: t.__setitem__("now", t["now"] + s),
                           clock=lambda: t["now"], log=lambda m: None)


def test_cli_replug_and_rts_reset_are_exclusive(capsys):
    with pytest.raises(SystemExit):
        df.main(["--port", "COMX", "--list", "--catch-boot", "--replug", "--rts-reset"])
    assert "exclusive" in capsys.readouterr().err
