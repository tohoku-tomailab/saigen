"""Back up, install and restore the SAIGEN firmware on a Hiwonder MaxArm (human-run tool).

**Run by a human only** (CLAUDE.md rule 1). Installing firmware and any board
reset can move the arm: Hiwonder's factory ``main.py`` calls ``arm.go_home()``
at every boot.

Usage (from the repository root)::

    python -m src.devices.maxarm.deploy_firmware --port COM11 --list
    python -m src.devices.maxarm.deploy_firmware --port COM11 --backup backups/maxarm_2026-09-23
    python -m src.devices.maxarm.deploy_firmware --port COM11 --check
    python -m src.devices.maxarm.deploy_firmware --port COM11 --install --backup backups/maxarm_2026-09-23
    python -m src.devices.maxarm.deploy_firmware --port COM11 --restore backups/maxarm_2026-09-23

What it does:

- opens the port with ``rts=False`` / ``dtr=False`` set before ``open()`` (no
  auto-reset), interrupts the running program with Ctrl-C and enters the raw
  REPL (mpremote-style sequence, up to 5 attempts);
- ``--check`` runs ``check_device``: on the board itself it confirms the
  MicroPython features the firmware relies on (``ticks_add``, ``select.poll`` on
  stdin and a stdin round trip, ``sys.stdin.buffer``, the vendor modules,
  allocating a ``SuctionNozzle`` without running its ``__init__``, and that
  ``saigen_maxarm.py`` imports / ``main.py`` compiles on that build). Nothing
  moves and no PWM duty changes. ``--install`` runs the same checks first and
  refuses when a required one fails (``--force`` overrides);
- ``--backup DIR`` downloads every file of the device filesystem into DIR and
  writes ``DIR/device_info.json`` (os.uname, sys.version, file list, sha256).
  It refuses a DIR that is not empty (a complete backup, an interrupted one or
  anything else) unless ``--overwrite-backup`` is given, so the factory backup
  cannot be overwritten by accident; ``--backup`` cannot be combined with
  ``--restore``;
- ``--install`` uploads ``firmware/saigen_maxarm.py`` and ``firmware/main.py``
  (write to ``*.tmp``, read back and compare, then ``path -> path.old``,
  ``path.tmp -> path``, remove ``path.old``). It refuses unless
  ``--backup DIR`` holds a backup of the current ``main.py`` (``--force`` overrides).
  The factory ``main.py`` is also kept on the device as ``main_factory.py``;
- ``--restore DIR`` uploads the backed-up files again (creating missing
  directories, ``/main.py`` last) and removes the SAIGEN files that were not in
  the backup;
- it never soft-resets the board unless ``--reset`` is given; without it the
  board is left at the MicroPython REPL (arm holds its pose) and the new
  ``main.py`` runs at the next power-on.
"""

import argparse
import ast
import hashlib
import json
import os
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

FIRMWARE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")
FIRMWARE_FILES = ("saigen_maxarm.py", "main.py")      # upload order: library first
RAW_REPL_BANNER = b"raw REPL; CTRL-B to exit\r\n>"
SAIGEN_MARKER = b"saigen_maxarm"
CHUNK = 256            # bytes of file data per exec (repr() of the chunk is <= 4x)
WRITE_BLOCK = 256      # bytes per serial write in exec_raw (stdin ring buffer ~260 B)

RESET_WARNING = (
    "警告: 工場出荷時の main.py が入った状態でボードがリセットされると"
    "（電源の再投入、DTR/RTS の変化、Ctrl-D ソフトリセット）、アームは原点 (go_home) へ動きます。"
    "アームの可動範囲に物や手がないことを確認してから行ってください。\n"
    "WARNING: if the board resets while Hiwonder's factory main.py is installed "
    "(power cycle, DTR/RTS change, Ctrl-D soft reset), the arm MOVES to its home pose "
    "(go_home). Clear the arm's reach before doing so."
)


class ReplError(RuntimeError):
    """Raw REPL handshake failed or device-side code raised."""


# ----------------------------------------------------------------------
# Serial / raw REPL primitives (pure functions over a serial-like object)
# ----------------------------------------------------------------------
def open_serial(port: str, baudrate: int = 115200, serial_factory: Optional[Callable] = None):
    """Open ``port`` without toggling DTR/RTS (both set False before open())."""
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial
    ser = serial_factory()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = 0.05
    ser.write_timeout = 2.0
    ser.rts = False
    ser.dtr = False
    ser.open()
    return ser


def close_serial(ser) -> None:
    """Close with rts=False then dtr=False (same order as mpremote)."""
    for name in ("rts", "dtr"):
        try:
            setattr(ser, name, False)
        except Exception:
            pass
    ser.close()


def drain(ser, max_time: float = 2.0, clock: Callable[[], float] = time.monotonic) -> bytes:
    """Read and return everything pending (stops after a short silence or ``max_time``)."""
    out = b""
    end = clock() + max_time
    while clock() < end:
        n = getattr(ser, "in_waiting", 0)
        chunk = ser.read(n if n else 1)
        if not chunk:
            break
        out += chunk
    return out


def read_until(ser, ending: bytes, timeout: float = 10.0,
               clock: Callable[[], float] = time.monotonic) -> bytes:
    """Read until ``ending`` is received; raise ReplError on timeout."""
    data = b""
    start = clock()
    deadline = start + timeout
    hard = start + 6 * timeout            # a device that keeps talking cannot hold us forever
    while not data.endswith(ending):
        chunk = ser.read(1)
        now = clock()
        if chunk:
            data += chunk
            deadline = min(hard, max(deadline, now + 0.5))   # still talking: wait a bit more
        if now > deadline:
            raise ReplError("timeout waiting for %r; got %r" % (ending, data[-200:]))
    return data


def enter_raw_repl(ser, retries: int = 5, timeout: float = 2.0,
                   sleep: Optional[Callable[[float], None]] = None,
                   clock: Callable[[], float] = time.monotonic) -> bytes:
    """Interrupt the running program and enter the raw REPL.

    Per attempt: ``\\r\\x03\\x03`` (two Ctrl-C), short pause, drain, ``\\r\\x01``
    (Ctrl-A), wait for ``raw REPL; CTRL-B to exit\\r\\n>``. Never sends Ctrl-D
    (a soft reset). Returns the drained output of the interrupted program
    (e.g. the SAIGEN firmware's ``EVT FAULT type=KeyboardInterrupt``).
    """
    sleep = sleep or time.sleep
    drained = b""
    last = None
    for _ in range(max(1, retries)):
        ser.write(b"\r\x03\x03")
        sleep(0.1)
        drained += drain(ser)
        ser.write(b"\r\x01")
        try:
            read_until(ser, RAW_REPL_BANNER, timeout=timeout, clock=clock)
            return drained
        except ReplError as e:
            last = e
            sleep(0.2)
    raise ReplError(
        "raw REPL に入れませんでした / could not enter the raw REPL after %d attempts (%s). "
        "工場出荷のプログラムが USB の UART を 9600 bps のシリアル通信用に占有している可能性があります "
        "(README.md 'Raw REPL not reachable' を参照) / the factory program may have taken over the "
        "USB UART (see README.md)." % (retries, last))


BOOT_MARKERS = (b"rst:", b"ets ", b"MicroPython v", b"boot:")
FRIENDLY_PROMPT = b">>> "


def catch_boot(ser, wait: float = 120.0, window: float = 9.0,
               sleep: Optional[Callable[[float], None]] = None,
               clock: Callable[[], float] = time.monotonic,
               log: Callable[[str], None] = print) -> bytes:
    """Interrupt the factory ``main.py`` during its boot delay, before it moves the arm.

    Hiwonder's serial-slave ``main.py`` starts with ``time.sleep_ms(10000)`` and only
    then re-configures the USB UART (9600 bps) and homes the arm. During those 10 s
    a Ctrl-C at 115200 lands in the sleep and drops the board to the friendly REPL,
    so nothing moves. The operator power-cycles the arm (or ``--rts-reset`` pulses
    RTS); this function sends Ctrl-C every 0.1 s and returns as soon as the
    ``>>> `` prompt is seen. Raises ReplError if no prompt appears within ``wait``
    seconds, or within ``window`` seconds after boot output was seen (then the
    factory program has probably taken the UART, and ``go_home`` has run).
    """
    sleep = sleep or time.sleep
    buf = b""
    start = clock()
    boot_seen = None
    while True:
        now = clock()
        if boot_seen is not None and now > boot_seen + window:
            raise ReplError(
                "boot output was seen but no REPL prompt followed within %.0f s; the factory "
                "program has probably taken over the UART (and moved the arm to home). "
                "Power-cycle again and retry. / 起動メッセージ後 %.0f 秒以内に REPL プロンプトが"
                "出ませんでした。工場出荷プログラムが UART を占有した（ホーミング済み）可能性が"
                "あります。電源を入れ直してもう一度試してください。got %r"
                % (window, window, buf[-200:]))
        if now > start + wait:
            raise ReplError("no boot / no REPL prompt within %.0f s (got %r). Did the arm "
                            "get power-cycled? / %.0f 秒以内に起動が検出できませんでした。"
                            "電源を入れ直しましたか？" % (wait, buf[-200:], wait))
        ser.write(b"\x03")
        sleep(0.1)
        n = getattr(ser, "in_waiting", 0)
        chunk = ser.read(n if n else 1)
        if chunk:
            buf += chunk
            if boot_seen is None and any(m in buf for m in BOOT_MARKERS):
                boot_seen = clock()
                log("boot output detected; interrupting main.py ... / 起動を検出、main.py を中断中...")
            if FRIENDLY_PROMPT in buf[-64:]:
                log("friendly REPL reached before the factory program took over "
                    "/ 工場出荷プログラムより先に REPL に入りました")
                return buf


def wait_for_replug(port: str, baudrate: int = 115200, wait: float = 120.0,
                    serial_factory: Optional[Callable] = None,
                    sleep: Optional[Callable[[float], None]] = None,
                    clock: Callable[[], float] = time.monotonic,
                    log: Callable[[str], None] = print):
    """Wait until ``port`` cannot be opened (cable unplugged) and then can be opened
    again (re-enumerated after re-plugging), and return the freshly opened port.

    The ESP32 core board is USB-powered, so re-plugging the cable is the reset the
    operator can do by hand; the board boots the moment the port re-appears, well
    inside the factory program's 10 s delay. Raises ReplError on timeout.
    """
    sleep = sleep or time.sleep
    start = clock()
    gone = False
    while clock() < start + wait:
        try:
            ser = open_serial(port, baudrate, serial_factory=serial_factory)
        except Exception:                       # absent, or held by us a moment ago
            if not gone:
                gone = True
                log("%s disappeared; waiting for it to come back ... / %s が消えました。再接続を待っています..."
                    % (port, port))
            sleep(0.2)
            continue
        if gone:
            log("%s is back; catching the boot ... / %s が戻りました。起動を捕まえます..." % (port, port))
            return ser
        close_serial(ser)                       # still plugged in: keep polling
        sleep(0.2)
    raise ReplError("%s did not disappear and come back within %.0f s (unplug and re-plug the "
                    "USB-C cable while this tool runs) / %.0f 秒以内に %s の抜き差しが検出できません"
                    "でした（ツール実行中に USB-C を抜き差ししてください）" % (port, wait, wait, port))


def pulse_rts_reset(ser, sleep: Optional[Callable[[float], None]] = None) -> None:
    """Pulse RTS (DTR held low) to reset the ESP32 through the auto-reset circuit.

    Only works on boards that wire RTS/DTR to EN/IO0; the caller must be ready
    to catch the boot immediately (see ``catch_boot``).
    """
    sleep = sleep or time.sleep
    ser.dtr = False
    ser.rts = True
    sleep(0.2)
    ser.rts = False


def exit_raw_repl(ser) -> None:
    """Ctrl-B: back to the friendly REPL (does not reset, does not run main.py)."""
    ser.write(b"\r\x02")


def exec_raw(ser, code: str, timeout: float = 10.0,
             sleep: Optional[Callable[[float], None]] = None,
             clock: Callable[[], float] = time.monotonic,
             after_ok: bytes = b"") -> Tuple[bytes, bytes]:
    """Execute ``code`` in the raw REPL; return (stdout, stderr).

    Sends the code in WRITE_BLOCK-byte writes with a 10 ms pause (the device's
    stdin ring buffer is ~260 bytes), then Ctrl-D; expects ``OK``, stdout,
    ``\\x04``, stderr, ``\\x04``, ``>``. ``after_ok`` is written once the
    device has accepted the code, i.e. while it runs: input for a snippet that
    reads ``sys.stdin`` (the ``--check`` stdin round trip).
    """
    sleep = sleep or time.sleep
    data = code.encode("utf-8")
    for i in range(0, len(data), WRITE_BLOCK):
        ser.write(data[i:i + WRITE_BLOCK])
        sleep(0.01)
    ser.write(b"\x04")
    ok = read_until(ser, b"OK", timeout=timeout, clock=clock)
    if not ok.endswith(b"OK"):
        raise ReplError("raw REPL did not accept the code: %r" % ok)
    if after_ok:
        sleep(0.05)
        ser.write(after_ok)
    out = read_until(ser, b"\x04", timeout=timeout, clock=clock)[:-1]
    err = read_until(ser, b"\x04", timeout=timeout, clock=clock)[:-1]
    read_until(ser, b">", timeout=timeout, clock=clock)
    return out, err


def exec_checked(ser, code: str, **kw) -> bytes:
    out, err = exec_raw(ser, code, **kw)
    if err:
        raise ReplError("device error: %s" % err.decode("utf-8", "replace").strip())
    return out


def chunks(data: bytes, size: int = CHUNK) -> List[bytes]:
    """Split ``data`` into ``size``-byte pieces (an empty file gives no chunk)."""
    return [data[i:i + size] for i in range(0, len(data), size)]


# ----------------------------------------------------------------------
# Device-side code snippets (MicroPython)
# ----------------------------------------------------------------------
_LIST_CODE = (
    "import os\n"
    "def _w(d):\n"
    "    for e in os.ilistdir(d):\n"
    "        p = (d if d.endswith('/') else d + '/') + e[0]\n"
    "        if e[1] & 0x4000:\n"
    "            _w(p)\n"
    "        else:\n"
    "            print(repr((p, os.stat(p)[6])))\n"
    "_w('/')\n"
)

_READ_CODE = (
    "try:\n"
    "    import ubinascii as _h\n"
    "except ImportError:\n"
    "    import binascii as _h\n"
    "_f = open(%r, 'rb')\n"
    "while True:\n"
    "    _b = _f.read(256)\n"
    "    if not _b:\n"
    "        break\n"
    "    print(_h.hexlify(_b).decode())\n"
    "_f.close()\n"
)

_INFO_CODE = (
    "import os, sys\n"
    "print(repr((tuple(os.uname()), sys.version, sys.platform)))\n"
)


def list_files(ser, **kw) -> List[Tuple[str, int]]:
    """[(absolute path, size), ...] of every file on the device filesystem."""
    out = exec_checked(ser, _LIST_CODE, **kw)
    files = []
    for line in out.decode("utf-8", "replace").splitlines():
        line = line.strip()
        if line:
            path, size = ast.literal_eval(line)
            files.append((str(path), int(size)))
    return files


def read_file(ser, path: str, **kw) -> bytes:
    out = exec_checked(ser, _READ_CODE % path, **kw)
    return b"".join(bytes.fromhex(line.strip()) for line in out.decode("ascii").splitlines()
                    if line.strip())


_MKDIRS_CODE = (
    "import os\n"
    "_d = ''\n"
    "for _p in %r:\n"
    "    _d = _d + '/' + _p\n"
    "    try:\n"
    "        os.stat(_d)\n"
    "    except OSError as _e:\n"
    "        if _e.args[0] != 2:\n"
    "            raise\n"
    "        os.mkdir(_d)\n"
)

# Replace ``path`` by ``path.tmp`` without a moment in which no copy exists:
# path -> path.old, path.tmp -> path, remove path.old. Only ENOENT (errno 2) is
# treated as "absent"; every other filesystem error propagates.
_REPLACE_CODE = (
    "import os\n"
    "def _ex(p):\n"
    "    try:\n"
    "        os.stat(p)\n"
    "        return True\n"
    "    except OSError as e:\n"
    "        if e.args[0] != 2:\n"
    "            raise\n"
    "        return False\n"
    "_path, _tmp, _old = %r, %r, %r\n"
    "if _ex(_old):\n"
    "    if not _ex(_path):\n"
    "        raise OSError(17, 'only the previous version ' + _old + ' exists; recover it first')\n"
    "    os.remove(_old)\n"
    "_had = _ex(_path)\n"
    "if _had:\n"
    "    os.rename(_path, _old)\n"
    "os.rename(_tmp, _path)\n"
    "if _had:\n"
    "    os.remove(_old)\n"
)


def recovery_instructions(path: str) -> str:
    old, tmp = path + ".old", path + ".tmp"
    return (
        "RECOVERY / 復旧手順: the replacement of %(p)s was interrupted. At the MicroPython "
        "REPL (e.g. Thonny, or mpremote with the arm's reach clear) check the files:\n"
        "  import os; print(os.listdir('%(d)s'))\n"
        "- %(p)s missing and %(o)s present: os.rename('%(o)s', '%(p)s') restores the previous "
        "version;\n"
        "- %(p)s present and %(o)s present: %(p)s is the new, verified version; "
        "os.remove('%(o)s');\n"
        "- %(t)s left over: os.remove('%(t)s').\n"
        "%(p)s が見つからず %(o)s がある場合は os.rename('%(o)s', '%(p)s') で元に戻せます。"
        % {"p": path, "o": old, "t": tmp, "d": path.rsplit("/", 1)[0] or "/"})


def ensure_parent_dirs(ser, path: str, **kw) -> None:
    """Create the missing parent directories of ``path`` on the device."""
    parts = [p for p in path.split("/")[1:-1] if p]
    if parts:
        exec_checked(ser, _MKDIRS_CODE % (parts,), **kw)


def write_file(ser, path: str, data: bytes, verify: bool = True, **kw) -> None:
    """Upload ``data`` to ``path``: write ``path.tmp`` chunk by chunk, read it
    back and compare, then replace ``path`` (``path -> path.old``,
    ``path.tmp -> path``, remove ``path.old``). Missing parent directories are
    created. If the replacement step fails or is interrupted, the recovery
    instructions are printed and the error is re-raised."""
    tmp = path + ".tmp"
    ensure_parent_dirs(ser, path, **kw)
    exec_checked(ser, "_f = open(%r, 'wb')\n" % tmp, **kw)
    try:
        for piece in chunks(data):
            exec_checked(ser, "_f.write(%r)\n" % piece, **kw)
    finally:
        exec_checked(ser, "_f.close()\n", **kw)
    if verify:
        back = read_file(ser, tmp, **kw)
        if back != data:
            exec_raw(ser, "import os\nos.remove(%r)\n" % tmp, **kw)
            raise ReplError("verification failed for %s (%d bytes written, %d read back)"
                            % (path, len(data), len(back)))
    try:
        exec_checked(ser, _REPLACE_CODE % (path, tmp, path + ".old"), **kw)
    except BaseException:
        print(recovery_instructions(path), file=sys.stderr)
        raise


def remove_file(ser, path: str, **kw) -> None:
    exec_checked(ser, "import os\nos.remove(%r)\n" % path, **kw)


def device_info(ser, **kw) -> Dict:
    out = exec_checked(ser, _INFO_CODE, **kw)
    uname, version, platform = ast.literal_eval(out.decode("utf-8", "replace").strip())
    return {"uname": list(uname), "sys_version": version, "platform": platform}


# ----------------------------------------------------------------------
# --check: confirm on the device what the firmware relies on (no motion)
# ----------------------------------------------------------------------
# Everything below touches no servo, no torque and no PWM duty: it imports the
# vendor modules (the factory main.py has already imported them at boot; a
# repeated import is a cache hit), allocates a SuctionNozzle WITHOUT running its
# __init__, polls stdin, and imports the SAIGEN library from a temporary file.
_CHECK_CODE = (
    "import sys\n"
    "r = {}\n"
    "try:\n"
    "    import utime as time\n"
    "except ImportError:\n"
    "    import time\n"
    "for n in ('ticks_ms', 'ticks_diff', 'ticks_add'):\n"
    "    r[n] = hasattr(time, n)\n"
    "try:\n"
    "    r['ticks_add_ok'] = time.ticks_diff(time.ticks_add(time.ticks_ms(), 1000), time.ticks_ms()) > 900\n"
    "except Exception as e:\n"
    "    r['ticks_add_ok'] = repr(e)\n"
    "try:\n"
    "    import uselect as select\n"
    "except ImportError:\n"
    "    import select\n"
    "_in = getattr(sys, 'stdin', None)\n"
    "try:\n"
    "    p = select.poll()\n"
    "    p.register(_in, select.POLLIN)\n"
    "    p.poll(0)\n"
    "    p.unregister(_in)\n"
    "    r['poll_stdin'] = True\n"
    "except Exception as e:\n"
    "    r['poll_stdin'] = repr(e)\n"
    "r['stdin_buffer'] = hasattr(_in, 'buffer')\n"
    "r['stdout_write'] = hasattr(getattr(sys, 'stdout', None), 'write')\n"
    "try:\n"
    "    import _thread\n"
    "    r['thread'] = True\n"
    "except ImportError:\n"
    "    r['thread'] = False\n"
    "try:\n"
    "    from SuctionNozzle import SuctionNozzle\n"
    "    r['nozzle_import'] = True\n"
    "    try:\n"
    "        r['cls_new'] = isinstance(SuctionNozzle.__new__(SuctionNozzle), SuctionNozzle)\n"
    "    except Exception as e:\n"
    "        r['cls_new'] = repr(e)\n"
    "    try:\n"
    "        r['object_new'] = isinstance(object.__new__(SuctionNozzle), SuctionNozzle)\n"
    "    except Exception as e:\n"
    "        r['object_new'] = repr(e)\n"
    "except Exception as e:\n"
    "    r['nozzle_import'] = repr(e)\n"
    "for m in ('BusServo', 'espmax'):\n"
    "    try:\n"
    "        __import__(m)\n"
    "        r[m + '_import'] = True\n"
    "    except Exception as e:\n"
    "        r[m + '_import'] = repr(e)\n"
    "try:\n"
    "    r['mv_decode'] = bytes(memoryview(bytearray(b'abc def'))[:3]).decode() == 'abc'\n"
    "except Exception as e:\n"
    "    r['mv_decode'] = repr(e)\n"
    "try:\n"
    "    r['fmt'] = '%.1f' % 1.25\n"
    "except Exception as e:\n"
    "    r['fmt'] = repr(e)\n"
    "try:\n"
    "    import gc\n"
    "    gc.collect()\n"
    "    r['mem_free'] = gc.mem_free()\n"
    "except Exception:\n"
    "    r['mem_free'] = -1\n"
    "r['modules'] = sorted(getattr(sys, 'modules', {}).keys())\n"
    "print(repr(r))\n"
)

# The host writes one byte ('x') after the device accepted this code; the snippet
# must see it through poll() and read(1), exactly as main.py's serve loop does.
_STDIN_CODE = (
    "import sys\n"
    "try:\n"
    "    import utime as time\n"
    "except ImportError:\n"
    "    import time\n"
    "try:\n"
    "    import uselect as select\n"
    "except ImportError:\n"
    "    import select\n"
    "rd = getattr(sys.stdin, 'buffer', sys.stdin)\n"
    "p = select.poll()\n"
    "p.register(sys.stdin, select.POLLIN)\n"
    "t0 = time.ticks_ms()\n"
    "ev = p.poll(%d)\n"
    "dt = time.ticks_diff(time.ticks_ms(), t0)\n"
    "c = rd.read(1) if ev else None\n"
    "p.unregister(sys.stdin)\n"
    "print(repr((bool(ev), dt, c)))\n"
)
STDIN_PROBE_BYTE = b"x"
STDIN_PROBE_WAIT_MS = 3000

CHECK_MODULE = "saigen_check_tmp"        # /saigen_check_tmp.py, removed afterwards

# Import the SAIGEN library the way boot will (compiled from the file, not from a
# string, so the RAM use is the real one); then forget the module again.
_IMPORT_CODE = (
    "import sys, gc\n"
    "gc.collect()\n"
    "_m0 = gc.mem_free()\n"
    "__import__(%r)\n"
    "gc.collect()\n"
    "_m1 = gc.mem_free()\n"
    "try:\n"
    "    del sys.modules[%r]\n"
    "except Exception:\n"
    "    pass\n"
    "gc.collect()\n"
    "print(repr((_m0, _m1)))\n"
)

_COMPILE_CODE = (
    "import gc\n"
    "gc.collect()\n"
    "compile(open(%r).read(), %r, 'exec')\n"
    "gc.collect()\n"
    "print('ok')\n"
)

# Run the command server on the device against dummy arm/bus/nozzle objects: every
# reply path (OK, each ERR code, the watchdog event) executes on the real
# interpreter. Nothing is imported from the vendor modules and no hardware is
# touched. (Bench 2026-09-24: on MicroPython 1.12 every ERR reply turned into
# "ERR INTERNAL AttributeError" because of Exception.__init__ -- exactly what
# this catches before an install.)
_SELFTEST_CODE = (
    "M = __import__(%r)\n"
    "class Arm:\n"
    "    def __init__(self):\n"
    "        self.pos = (0, -163, 212)\n"
    "        self.result = True\n"
    "    def set_position(self, p, ms):\n"
    "        if not self.result:\n"
    "            return False\n"
    "        self.pos = p\n"
    "        return True\n"
    "    def read_position(self):\n"
    "        return self.pos\n"
    "    def read_angles(self):\n"
    "        return [500, 500, 500]\n"
    "class Bus:\n"
    "    def stop(self, i):\n"
    "        pass\n"
    "    def get_position(self, i):\n"
    "        return 500\n"
    "    def get_vin(self, i):\n"
    "        return 7400\n"
    "    def load(self, i):\n"
    "        pass\n"
    "    def unload(self, i):\n"
    "        pass\n"
    "class Noz:\n"
    "    def __init__(self):\n"
    "        self.nozzle_st = False\n"
    "    def on(self):\n"
    "        self.nozzle_st = True\n"
    "    def off(self):\n"
    "        self.nozzle_st = False\n"
    "out = []\n"
    "t = [0]\n"
    "arm = Arm()\n"
    "s = M.Server(arm, Bus(), Noz(), out.append, lambda: t[0], lambda a, b: a - b,\n"
    "             ticks_add=lambda a, b: a + b)\n"
    "def go(body):\n"
    "    return s.handle_line(M.frame(body).strip())\n"
    "r = []\n"
    "r.append(go('1 PING'))\n"
    "r.append(go('2 INFO'))\n"
    "r.append(go('3 LIMITS -250 250 -280 -60 40 225 60 260'))\n"
    "r.append(go('4 MOVE 0 -300 100 1000'))\n"
    "r.append(go('5 MOVE 50 -200 150 1000'))\n"
    "t[0] += 2000\n"
    "r.append(go('6 STATUS'))\n"
    "arm.result = False\n"
    "r.append(go('7 MOVE 60 -200 150 1000'))\n"
    "arm.result = True\n"
    "r.append(go('8 MOVE x y z 1000'))\n"
    "r.append(go('9 STOP'))\n"
    "r.append(go('10 MOVE 50 -200 150 1000'))\n"
    "r.append(go('11 CLEAR'))\n"
    "r.append(go('12 PUMP 1'))\n"
    "r.append(go('13 PUMP 0'))\n"
    "r.append(go('14 WATCHDOG 500'))\n"
    "t[0] += 600\n"
    "s.tick()\n"
    "r.append(go('15 NOSUCH'))\n"
    "r.append(s.handle_line('16 PING*00'))\n"
    "print(repr((r, out)))\n"
)
SELFTEST_EXPECTED = (
    "1 OK pong=1", "2 OK fw=saigen-maxarm proto=1", "3 OK limits=1", "4 ERR LIMIT",
    "5 OK x=50 y=-200 z=150 ms=1000", "6 OK x=50 y=-200 z=150", "7 ERR UNREACHABLE",
    "8 ERR BADARG", "9 OK stopped=1", "10 ERR STOPPED", "11 OK", "12 OK pump=1",
    "13 OK pump=0", "14 OK wd=500", "15 ERR BADCMD", "16 ERR CHECKSUM",
)


def selftest_mismatches(replies: List[str], events: List[str]) -> List[str]:
    """Which self-test replies do not start with the expected text (plus the watchdog event)."""
    bad = []
    for i, exp in enumerate(SELFTEST_EXPECTED):
        got = replies[i] if i < len(replies) else None
        if not isinstance(got, str) or not got.startswith(exp):
            bad.append("%s -> %r" % (exp, got))
    if not any("EVT STOPPED reason=watchdog" in e for e in events):
        bad.append("no EVT STOPPED reason=watchdog after the deadline")
    return bad


def _check_result(name: str, required: bool, ok, detail: str = "") -> Dict:
    return {"name": name, "required": required, "ok": bool(ok), "detail": str(detail)}


def check_device(ser, firmware_dir: str = FIRMWARE_DIR, log: Callable[[str], None] = print,
                 **kw) -> List[Dict]:
    """Confirm on the device the MicroPython features the firmware relies on.

    Returns one dict per check: ``name``, ``required`` (install is refused when a
    required check fails), ``ok``, ``detail``. Moves nothing, changes no torque
    and no PWM duty (see the comment above ``_CHECK_CODE``). Vendor build
    differences show up here instead of as a silent boot failure after install.
    """
    results = []

    def add(name, required, ok, detail=""):
        results.append(_check_result(name, required, ok, detail))
        log("  %-4s %-18s %s" % ("PASS" if ok else ("FAIL" if required else "warn"), name, detail))

    r = ast.literal_eval(exec_checked(ser, _CHECK_CODE, **kw).decode("utf-8", "replace").strip())
    ticks = [r.get(k) for k in ("ticks_ms", "ticks_diff", "ticks_add")]
    add("ticks", True, all(ticks) and r.get("ticks_add_ok") is True,
        "ticks_ms/ticks_diff/ticks_add=%s add_ok=%r" % ("/".join("y" if t else "n" for t in ticks),
                                                        r.get("ticks_add_ok")))
    add("poll_stdin", True, r.get("poll_stdin") is True,
        "select.poll().register(sys.stdin) %s" % ("ok" if r.get("poll_stdin") is True
                                                  else r.get("poll_stdin")))
    add("stdin_buffer", False, r.get("stdin_buffer"),
        "sys.stdin.buffer %s" % ("present" if r.get("stdin_buffer") else
                                 "absent (main.py falls back to sys.stdin)"))
    add("stdout_write", True, r.get("stdout_write"), "sys.stdout.write")
    add("vendor_imports", True, r.get("BusServo_import") is True and r.get("espmax_import") is True
        and r.get("nozzle_import") is True,
        "BusServo=%r espmax=%r SuctionNozzle=%r" % (r.get("BusServo_import"), r.get("espmax_import"),
                                                    r.get("nozzle_import")))
    cls_new, obj_new = r.get("cls_new"), r.get("object_new")
    add("nozzle_alloc", True, cls_new is True or obj_new is True,
        "SuctionNozzle.__new__=%r object.__new__=%r%s" % (
            cls_new, obj_new,
            " (main.py uses the object.__new__ fallback)" if cls_new is not True and obj_new is True
            else ""))
    add("memoryview_decode", True, r.get("mv_decode") is True, "bytes(memoryview(buf)[:n]).decode()")
    add("percent_format", True, isinstance(r.get("fmt"), str) and r["fmt"].startswith("1."),
        "'%%.1f' %% 1.25 -> %r" % r.get("fmt"))
    add("thread", False, r.get("thread"), "_thread (used by the vendor nozzle.off())")
    add("mem_free", False, r.get("mem_free", -1) > 40000, "%s bytes free before import"
        % r.get("mem_free"))
    log("  info modules already imported: %s" % ", ".join(r.get("modules", [])))

    # stdin round trip: poll -> read(1) while the host sends one byte
    consumed = False
    try:
        out = exec_checked(ser, _STDIN_CODE % STDIN_PROBE_WAIT_MS, after_ok=STDIN_PROBE_BYTE, **kw)
        ev, dt, c = ast.literal_eval(out.decode("utf-8", "replace").strip())
        got = c in (STDIN_PROBE_BYTE, STDIN_PROBE_BYTE.decode(), STDIN_PROBE_BYTE[0])
        consumed = bool(ev)
        add("stdin_roundtrip", True, ev and got, "readable=%r after %s ms, read(1)=%r" % (ev, dt, c))
    except ReplError as e:
        add("stdin_roundtrip", True, False, "no reply (%s)" % str(e)[:120])
    if not consumed:
        # the probe byte is still in the device's input buffer and would become the
        # first byte of the next snippet: Ctrl-C / Ctrl-A discard it
        enter_raw_repl(ser, sleep=kw.get("sleep"), clock=kw.get("clock", time.monotonic))

    # the SAIGEN library imports from a file, like at boot; main.py only compiles
    tmp = "/%s.py" % CHECK_MODULE
    with open(os.path.join(firmware_dir, "saigen_maxarm.py"), "rb") as f:
        lib = f.read()
    write_file(ser, tmp, lib, verify=False, **kw)
    try:
        try:
            out = exec_checked(ser, _IMPORT_CODE % (CHECK_MODULE, CHECK_MODULE), **kw)
            m0, m1 = ast.literal_eval(out.decode("utf-8", "replace").strip())
            add("firmware_import", True, True, "saigen_maxarm.py imports; %s -> %s bytes free"
                % (m0, m1))
        except ReplError as e:
            add("firmware_import", True, False, str(e)[:160])
        try:
            kw_long = dict(kw)
            kw_long.setdefault("timeout", 30.0)
            out = exec_checked(ser, _SELFTEST_CODE % CHECK_MODULE, **kw_long)
            replies, events = ast.literal_eval(out.decode("utf-8", "replace").strip())
            bad = selftest_mismatches(list(replies), list(events))
            add("firmware_selftest", True, not bad,
                "%d command paths on the device interpreter" % len(SELFTEST_EXPECTED) if not bad
                else "; ".join(bad)[:400])
        except (ReplError, ValueError, SyntaxError) as e:
            add("firmware_selftest", True, False, str(e)[:300])
        with open(os.path.join(firmware_dir, "main.py"), "rb") as f:
            main_src = f.read()
        exec_checked(ser, "_f = open(%r, 'wb')\n" % tmp, **kw)
        try:
            for piece in chunks(main_src):
                exec_checked(ser, "_f.write(%r)\n" % piece, **kw)
        finally:
            exec_checked(ser, "_f.close()\n", **kw)
        try:
            exec_checked(ser, _COMPILE_CODE % (tmp, "main.py"), **kw)
            add("main_compile", True, True, "main.py compiles")
        except ReplError as e:
            add("main_compile", True, False, str(e)[:160])
    finally:
        try:
            remove_file(ser, tmp, **kw)
        except ReplError:
            log("  warn could not remove %s (harmless; delete it at the REPL)" % tmp)
    return results


def checks_passed(results: List[Dict]) -> bool:
    return all(r["ok"] for r in results if r["required"])


def failed_checks(results: List[Dict]) -> List[str]:
    return [r["name"] for r in results if r["required"] and not r["ok"]]


# ----------------------------------------------------------------------
# High-level operations
# ----------------------------------------------------------------------
def _local_path(root: str, device_path: str) -> str:
    rel = device_path.lstrip("/")
    parts = [p for p in rel.split("/") if p not in ("", ".", "..")]
    return os.path.join(root, *parts)


def backup_dir_in_use(backup_dir: str) -> Optional[str]:
    """Why ``backup_dir`` must not be written without --overwrite-backup, or None.

    Any existing non-empty directory counts: a complete backup (device_info.json),
    an interrupted one (files without device_info.json) or unrelated files. A
    path that exists but is not a directory is refused as well.
    """
    if not os.path.exists(backup_dir):
        return None
    if not os.path.isdir(backup_dir):
        return "%s exists and is not a directory" % backup_dir
    entries = os.listdir(backup_dir)
    if not entries:
        return None
    if "device_info.json" in entries:
        return "%s already holds a backup (device_info.json)" % backup_dir
    return ("%s is not empty (%d entries, e.g. %s; possibly an interrupted backup)"
            % (backup_dir, len(entries), sorted(entries)[0]))


def _refuse_backup_dir(reason: str):
    raise SystemExit(
        "REFUSED: %s. Choose a new, empty directory, or pass --overwrite-backup to write "
        "into it.\n"
        "中止: バックアップ先が空ではありません（工場出荷プログラムの唯一の控えかもしれません）。"
        "空の新しいディレクトリを指定するか --overwrite-backup を付けてください。" % reason)


def backup(ser, backup_dir: str, log: Callable[[str], None] = print,
           overwrite: bool = False, **kw) -> Dict:
    """Download every device file into ``backup_dir`` and write device_info.json.

    Refuses (SystemExit) any non-empty ``backup_dir`` (see ``backup_dir_in_use``)
    unless ``overwrite`` is True: the first backup is the only copy of the
    factory program, and an interrupted backup must not be silently mixed with
    a later one.
    """
    reason = backup_dir_in_use(backup_dir)
    if reason and not overwrite:
        _refuse_backup_dir(reason)
    os.makedirs(backup_dir, exist_ok=True)
    info = device_info(ser, **kw)
    files = []
    for path, size in list_files(ser, **kw):
        data = read_file(ser, path, **kw)
        if len(data) != size:
            raise ReplError("size mismatch for %s: listed %d, read %d" % (path, size, len(data)))
        local = _local_path(backup_dir, path)
        os.makedirs(os.path.dirname(local), exist_ok=True)
        with open(local, "wb") as f:
            f.write(data)
        files.append({"path": path, "size": size, "sha256": hashlib.sha256(data).hexdigest()})
        log("  backed up %s (%d bytes)" % (path, size))
    info["files"] = files
    info["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    info["main_py_is_saigen"] = False
    if any(f["path"] == "/main.py" for f in files):
        with open(_local_path(backup_dir, "/main.py"), "rb") as f:
            info["main_py_is_saigen"] = SAIGEN_MARKER in f.read()
    with open(os.path.join(backup_dir, "device_info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    if info["main_py_is_saigen"]:
        log("NOTE: the device main.py is already the SAIGEN launcher; this backup does not "
            "contain the factory program (use the first backup, or main_factory.py).")
    return info


def check_backup(backup_dir: Optional[str]) -> Tuple[bool, str]:
    """Is there a usable backup of the current main.py in ``backup_dir``?"""
    if not backup_dir:
        return False, "no --backup DIR given"
    main = os.path.join(backup_dir, "main.py")
    if not os.path.isfile(main):
        return False, "%s not found (run --backup DIR first)" % main
    if not os.path.isfile(os.path.join(backup_dir, "device_info.json")):
        return False, "device_info.json missing in %s (not a deploy_firmware backup)" % backup_dir
    return True, "backup found: %s" % main


def install(ser, backup_dir: Optional[str], force: bool = False,
            firmware_dir: str = FIRMWARE_DIR, log: Callable[[str], None] = print, **kw) -> None:
    ok, msg = check_backup(backup_dir)
    if not ok and not force:
        raise SystemExit("REFUSED: %s. Take a backup first (--backup DIR) or pass --force.\n"
                         "中止: main.py のバックアップがありません。先に --backup DIR を実行するか "
                         "--force を付けてください。" % msg)
    if ok:
        with open(os.path.join(backup_dir, "main.py"), "rb") as f:
            backed_main = f.read()
        current = read_file(ser, "/main.py", **kw) if any(
            p == "/main.py" for p, _ in list_files(ser, **kw)) else None
        if current is not None and SAIGEN_MARKER not in current and current != backed_main:
            ok, msg = False, ("the device main.py differs from %s/main.py (the backup is not of "
                              "the current program)" % backup_dir)
            if not force:
                raise SystemExit("REFUSED: %s. Take a new backup or pass --force.\n"
                                 "中止: バックアップが現在の main.py と一致しません。" % msg)
    log(msg if ok else "WARNING: installing without a matching backup (--force): %s" % msg)
    # keep the factory program on the device as main_factory.py (only once)
    exec_checked(ser, (
        "import os\n"
        "try:\n"
        "    os.stat('/main_factory.py')\n"
        "except OSError:\n"
        "    try:\n"
        "        _d = open('/main.py', 'rb').read()\n"
        "    except OSError:\n"
        "        _d = None\n"
        "    if _d is not None and b'saigen_maxarm' not in _d:\n"
        "        _f = open('/main_factory.py', 'wb')\n"
        "        _f.write(_d)\n"
        "        _f.close()\n"
        "        print('kept /main_factory.py')\n"), **kw)
    for name in FIRMWARE_FILES:
        with open(os.path.join(firmware_dir, name), "rb") as f:
            data = f.read()
        write_file(ser, "/" + name, data, **kw)
        log("  installed /%s (%d bytes, verified)" % (name, len(data)))


def restore(ser, backup_dir: str, log: Callable[[str], None] = print, **kw) -> None:
    info_path = os.path.join(backup_dir, "device_info.json")
    with open(info_path, encoding="utf-8") as f:
        info = json.load(f)
    backed = {e["path"] for e in info.get("files", [])}
    if "/main.py" not in backed:
        raise SystemExit("REFUSED: the backup has no /main.py")
    if info.get("main_py_is_saigen"):
        log("WARNING: this backup's main.py is the SAIGEN launcher, not the factory program.")
    for entry in info["files"]:
        local = _local_path(backup_dir, entry["path"])
        with open(local, "rb") as f:
            data = f.read()
        if hashlib.sha256(data).hexdigest() != entry.get("sha256"):
            raise SystemExit("REFUSED: %s changed since the backup (sha256 mismatch)" % local)
    on_device = dict(list_files(ser, **kw))
    # startup files last: a failure part-way leaves the current main.py/boot.py in charge
    order = {"/main.py": 2, "/boot.py": 1}
    for entry in sorted(info["files"], key=lambda e: order.get(e["path"], 0)):
        with open(_local_path(backup_dir, entry["path"]), "rb") as f:
            data = f.read()
        if on_device.get(entry["path"]) == len(data) and read_file(ser, entry["path"], **kw) == data:
            log("  unchanged %s" % entry["path"])
            continue
        write_file(ser, entry["path"], data, **kw)
        log("  restored %s" % entry["path"])
    for path in on_device:
        if path in ("/saigen_maxarm.py",) and path not in backed:
            remove_file(ser, path, **kw)
            log("  removed %s" % path)


def soft_reset(ser, listen: float = 3.0, sleep: Optional[Callable[[float], None]] = None) -> bytes:
    """Leave the raw REPL and soft-reset in the friendly REPL (runs boot.py AND main.py)."""
    sleep = sleep or time.sleep
    exit_raw_repl(ser)
    sleep(0.2)
    drain(ser)
    ser.write(b"\x04")
    sleep(listen)
    return drain(ser)


def _confirm(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() in ("yes", "y")
    except EOFError:
        return False


def main(argv=None, serial_factory: Optional[Callable] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Back up / install / restore the SAIGEN MaxArm firmware (human-run).")
    ap.add_argument("--port", required=True, help="serial port of the MaxArm, e.g. COM11")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list files on the device")
    ap.add_argument("--check", action="store_true",
                    help="confirm on the device the MicroPython features the firmware relies on "
                         "(moves nothing; --install runs these checks first and refuses on a "
                         "failure unless --force)")
    ap.add_argument("--backup", metavar="DIR", help="download all device files into DIR "
                    "(with --install: the backup that must already exist)")
    ap.add_argument("--install", action="store_true", help="upload the SAIGEN firmware")
    ap.add_argument("--restore", metavar="DIR", help="upload a backup taken with --backup")
    ap.add_argument("--force", action="store_true", help="install without a backup")
    ap.add_argument("--overwrite-backup", action="store_true",
                    help="allow --backup DIR to write into a non-empty DIR "
                    "(e.g. to replace an existing backup)")
    ap.add_argument("--reset", action="store_true",
                    help="soft-reset afterwards so the new main.py runs (see the warning)")
    ap.add_argument("--yes", action="store_true", help="do not ask before a reset")
    ap.add_argument("--catch-boot", action="store_true",
                    help="the factory program occupies the USB UART: wait for you to power-cycle "
                         "the arm and interrupt main.py during its 10 s boot delay (before it "
                         "homes the arm), then continue")
    ap.add_argument("--rts-reset", action="store_true",
                    help="with --catch-boot: pulse RTS to reset the board instead of a manual "
                         "power cycle (works only if RTS is wired to EN)")
    ap.add_argument("--replug", action="store_true",
                    help="with --catch-boot: the ESP32 is USB-powered, so the arm's power switch "
                         "does not reboot it; wait for the port to disappear and come back "
                         "(you unplug and re-plug the USB-C cable), then catch the boot")
    ap.add_argument("--boot-wait", type=float, default=120.0,
                    help="--catch-boot: seconds to wait for the boot (default 120)")
    args = ap.parse_args(argv)
    if (args.rts_reset or args.replug) and not args.catch_boot:
        ap.error("--rts-reset / --replug need --catch-boot")
    if args.rts_reset and args.replug:
        ap.error("--rts-reset and --replug are exclusive")
    if not (args.list or args.check or args.backup or args.install or args.restore):
        ap.error("nothing to do: give --list, --check, --backup DIR, --install or --restore DIR")
    if args.install and args.restore:
        ap.error("--install and --restore are exclusive")
    if args.backup and args.restore:
        ap.error("--backup and --restore are exclusive (a backup into the restore source "
                 "would overwrite it before it is restored)")

    if args.backup and not args.install and not args.overwrite_backup:
        reason = backup_dir_in_use(args.backup)
        if reason:
            _refuse_backup_dir(reason)                   # before the port is opened

    print(RESET_WARNING)
    print()
    if args.catch_boot and args.replug:
        print("UNPLUG the MaxArm's USB-C cable now, wait 3 s, and plug it back in. Waiting up to "
              "%.0f s for %s to disappear and come back; Ctrl-C is then sent continuously so "
              "main.py is interrupted during its boot delay and the arm does not move.\n"
              "いま MaxArm の USB-C ケーブルを抜き、3 秒待ってから挿し直してください。%s が消えて再び現れる"
              "のを最大 %.0f 秒待ち、起動待ち時間中に main.py を中断します（アームは動きません）。"
              % (args.boot_wait, args.port, args.port, args.boot_wait))
        ser = wait_for_replug(args.port, args.baud, wait=args.boot_wait,
                              serial_factory=serial_factory)
    else:
        ser = open_serial(args.port, args.baud, serial_factory=serial_factory)
    try:
        if args.catch_boot:
            if args.replug:
                pass                                   # the board is booting right now
            elif args.rts_reset:
                print("pulsing RTS to reset the board ... / RTS でリセットします...")
                pulse_rts_reset(ser)
            else:
                print("POWER-CYCLE THE ARM NOW (switch off, wait 3 s, switch on). Waiting up to "
                      "%.0f s; Ctrl-C is sent continuously so main.py is interrupted during its "
                      "boot delay and the arm does not move.\n"
                      "いまアームの電源を入れ直してください（OFF → 3 秒待つ → ON）。最大 %.0f 秒待ちます。"
                      "起動待ち時間中に main.py を中断するので、アームは動きません。"
                      % (args.boot_wait, args.boot_wait))
            catch_boot(ser, wait=args.boot_wait)
        drained = enter_raw_repl(ser)
        if drained.strip():
            print("interrupted program said: %r" % drained[-300:])
        if args.list:
            for path, size in list_files(ser):
                print("%8d  %s" % (size, path))
        if args.check or args.install:
            print("device checks (nothing moves) / 実機チェック（アームは動きません）:")
            results = check_device(ser)
            failed = failed_checks(results)
            if failed:
                msg = ("required device checks failed: %s. The firmware would not start on this "
                       "MicroPython build; report the lines above.\n"
                       "実機チェックに失敗しました: %s。この MicroPython ではファームウェアが起動"
                       "しません。上の結果を報告してください。" % (", ".join(failed), ", ".join(failed)))
                if args.install and not args.force:
                    raise SystemExit("REFUSED: " + msg)
                print(("WARNING (--force): " if args.install else "") + msg)
            else:
                print("all required device checks passed / 必須チェックはすべて合格")
        if args.backup and not args.install:
            backup(ser, args.backup, overwrite=args.overwrite_backup)
            print("backup written to %s" % args.backup)
        if args.install:
            install(ser, args.backup, force=args.force)
        if args.restore:
            restore(ser, args.restore)
        if args.reset:
            if args.restore:
                print("RESTORED FACTORY PROGRAM: the reset WILL MOVE THE ARM (go_home). "
                      "/ 復元した工場出荷プログラムはリセットでアームを動かします。")
            if not args.yes and not _confirm("Soft-reset the board now? type 'yes' / "
                                             "今すぐソフトリセットしますか? 'yes' と入力: "):
                print("not reset / リセットしませんでした")
                exit_raw_repl(ser)
                return 0
            out = soft_reset(ser)
            print("board output after reset: %r" % out[-500:])
        else:
            exit_raw_repl(ser)
            if args.install or args.restore:
                print("The board was NOT reset; it waits at the MicroPython REPL and the arm "
                      "holds its pose. The new main.py runs at the next power-on.\n"
                      "ボードはリセットしていません（REPL で待機、アームは姿勢を保持）。"
                      "新しい main.py は次の電源投入で起動します。")
                if args.restore:
                    print("The restored factory main.py MOVES THE ARM at power-on: power-cycle "
                          "only with the arm's reach clear. / 復元した工場出荷の main.py は電源投入時に"
                          "アームを動かします。周囲を空けてから電源を入れ直してください。")
                else:
                    print("Power-cycle the arm to start the SAIGEN firmware (it does not move "
                          "the arm at boot). / SAIGEN ファームウェアは起動時にアームを動かしません。"
                          "電源を入れ直すと起動します。")
    finally:
        close_serial(ser)
    return 0


if __name__ == "__main__":
    sys.exit(main())
