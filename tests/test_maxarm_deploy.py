"""deploy_firmware.py raw-REPL helpers against a fake MicroPython board.

The fake board implements the friendly/raw REPL control characters and runs
the device-side snippets with CPython against an in-memory filesystem, so the
backup / install / restore logic is exercised end to end. No serial port is
opened and nothing is ever reset for real.
"""
import binascii
import builtins
import io
import json
import os
import traceback
import types

import pytest

from src.devices.maxarm import deploy_firmware as D

FACTORY_MAIN = b"import time\nfrom espmax import ESPMax\narm = ESPMax()\narm.go_home()\n"
FACTORY_BOOT = b"# boot.py\n"


class _File:
    def __init__(self, fs, path, mode):
        self.fs, self.path, self.mode = fs, path, mode
        if "r" in mode:
            if path not in fs.files:
                raise OSError(2, "ENOENT")
            self.buf = io.BytesIO(fs.files[path])
        else:
            parent = path.rsplit("/", 1)[0]
            if fs.strict_dirs and parent and not fs.is_dir(parent):
                raise OSError(2, "ENOENT")
            self.buf = io.BytesIO()

    def read(self, n=-1):
        return self.buf.read(n)

    def write(self, data):
        if self.fs.corrupt_writes:
            data = data[:-1] + b"!" if data else data
        return self.buf.write(data)

    def close(self):
        if "w" in self.mode:
            self.fs.files[self.path] = self.buf.getvalue()


class FakeFS:
    def __init__(self, files):
        self.files = dict(files)
        self.corrupt_writes = False
        self.strict_dirs = False       # True: directories must exist (see mkdir)
        self.dirs = set()

    def is_dir(self, p):
        return p in self.dirs or any(f.startswith(p + "/") for f in self.files)

    def module_os(self):
        fs = self

        class OS:
            @staticmethod
            def ilistdir(d):
                d = d.rstrip("/") + "/"
                seen = set()
                for p in sorted(fs.files):
                    if p.startswith(d):
                        rest = p[len(d):]
                        name = rest.split("/")[0]
                        if name in seen:
                            continue
                        seen.add(name)
                        yield (name, 0x4000 if "/" in rest else 0x8000, 0)

            @staticmethod
            def stat(p):
                if p not in fs.files:
                    if fs.is_dir(p):
                        return (0x4000, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                    raise OSError(2, "ENOENT")
                return (0x8000, 0, 0, 0, 0, 0, len(fs.files[p]), 0, 0, 0)

            @staticmethod
            def mkdir(p):
                if p in fs.files or fs.is_dir(p):
                    raise OSError(17, "EEXIST")
                parent = p.rsplit("/", 1)[0]
                if parent and not fs.is_dir(parent):
                    raise OSError(2, "ENOENT")
                fs.dirs.add(p)

            @staticmethod
            def remove(p):
                if p not in fs.files:
                    raise OSError(2, "ENOENT")
                del fs.files[p]

            @staticmethod
            def rename(a, b):
                if b in fs.files:
                    raise OSError(17, "EEXIST")
                fs.files[b] = fs.files.pop(a)

            @staticmethod
            def uname():
                return ("esp32", "esp32", "1.19.1", "v1.19.1 on 2022-06-18", "ESP32 module")
        return OS


class FakeBoard:
    """Serial-like fake MicroPython board (REPL control characters + exec)."""

    def __init__(self, files=None, responsive=True, running_output=b""):
        self.fs = FakeFS(files if files is not None else
                         {"/boot.py": FACTORY_BOOT, "/main.py": FACTORY_MAIN})
        self.responsive = responsive
        self.mode = "running"
        self.running_output = running_output
        self.rx = bytearray()
        self.code = bytearray()
        self.writes = []
        self.soft_resets = []            # "friendly" (runs main.py) or "raw"
        self.log = []
        self.is_open = False
        self.port = self.baudrate = self.timeout = self.write_timeout = None
        self._rts = self._dtr = None
        self.globals = {}
        # --check knobs (see _stub_modules): what the fake MicroPython build offers
        self.no_ticks_add = False        # utime without ticks_add (very old build)
        self.no_class_new = False        # Class.__new__ raises AttributeError, object.__new__ works
        self.stdin_dead = False          # poll() never reports stdin readable
        self.stdin_byte = b"x"           # what stdin.buffer.read(1) returns
        self.swallow = 0                 # host bytes consumed by a snippet's stdin read
        self.ticks = 0
        self.imported = []               # modules imported from the fake filesystem

    # pyserial-ish attributes
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

    def close(self):
        self.log.append(("close", None, self.is_open))
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data):
        self.writes.append(bytes(data))
        if not self.responsive:
            return len(data)
        for b in data:
            self._byte(b)
        return len(data)

    # REPL state machine
    def _byte(self, b):
        if self.mode == "running":
            if b == 0x03:
                self.rx += self.running_output + b"Traceback...\r\nKeyboardInterrupt: \r\n>>> "
                self.mode = "friendly"
            return
        if self.mode == "friendly":
            if b == 0x01:
                self.mode = "raw"
                self.code = bytearray()
                self.rx += D.RAW_REPL_BANNER
            elif b == 0x04:
                self.soft_resets.append("friendly")
                self.rx += b"MPY: soft reboot\r\n"
                self.mode = "running"
            elif b in (0x0D, 0x03):
                self.rx += b"\r\n>>> "
            return
        # raw REPL
        if b == 0x02:
            self.mode = "friendly"
            self.rx += b"\r\nMicroPython v1.19.1 on 2022-06-18; ESP32 module\r\n>>> "
        elif b == 0x03:
            self.code = bytearray()
        elif b == 0x01:
            self.code = bytearray()
            self.rx += D.RAW_REPL_BANNER
        elif self.swallow:
            # a byte the host sent while the previous snippet ran and read stdin:
            # on the real board it was consumed there, never part of the next code
            self.swallow -= 1
        elif b == 0x04:
            if not self.code.strip():
                self.soft_resets.append("raw")
                self.rx += b"OK\x04\x04>"
                return
            out, err = self._exec(bytes(self.code).decode())
            self.code = bytearray()
            self.rx += b"OK" + out + b"\x04" + err + b"\x04>"
        else:
            self.code.append(b)

    def _stub_modules(self):
        """The MicroPython-side modules the --check snippets look at (all stubs)."""
        board = self

        class _Buffer:
            @staticmethod
            def read(n=1):
                board.swallow += 1          # the host's byte arrives after this exec
                return board.stdin_byte

        class _Stdin:
            buffer = _Buffer()

        class _Stdout:
            @staticmethod
            def write(s):
                return len(s)

        sys_mod = types.SimpleNamespace(version="3.4.0; MicroPython v1.19.1", platform="esp32",
                                        stdin=_Stdin(), stdout=_Stdout(), modules={})

        def ticks_ms():
            board.ticks += 1
            return board.ticks
        utime = types.SimpleNamespace(ticks_ms=ticks_ms, ticks_diff=lambda a, b: a - b)
        if not self.no_ticks_add:
            utime.ticks_add = lambda a, b: a + b

        class Poll:
            def __init__(self):
                self.regs = []

            def register(self, obj, mask):
                self.regs.append(obj)

            def unregister(self, obj):
                self.regs.remove(obj)

            def poll(self, ms=-1):
                if board.stdin_dead or not self.regs:
                    board.ticks += max(ms, 0)
                    return []
                return [(self.regs[0], 1)]
        uselect = types.SimpleNamespace(poll=Poll, POLLIN=1)
        gc = types.SimpleNamespace(collect=lambda: None, mem_free=lambda: 100000)

        class NoClassNew(type):
            def __getattribute__(cls, name):
                if name == "__new__" and board.no_class_new:
                    raise AttributeError("type object has no attribute '__new__'")
                return type.__getattribute__(cls, name)

        class SuctionNozzle(metaclass=NoClassNew):
            def __init__(self):
                raise AssertionError("--check must never run SuctionNozzle.__init__")
        return {"os": self.fs.module_os(), "ubinascii": binascii, "sys": sys_mod,
                "utime": utime, "uselect": uselect, "gc": gc, "_thread": types.SimpleNamespace(),
                "SuctionNozzle": types.SimpleNamespace(SuctionNozzle=SuctionNozzle),
                "BusServo": types.SimpleNamespace(), "espmax": types.SimpleNamespace()}

    def _exec(self, code):
        stdout = io.StringIO()
        fs = self.fs
        real_import = builtins.__import__
        mods = self._stub_modules()
        b = dict(vars(builtins))

        def imp(name, *a, **k):
            if name in mods:
                return mods[name]
            if name == "binascii":
                raise ImportError(name)
            if "/" + name + ".py" in fs.files:          # import from the device filesystem
                mod = types.ModuleType(name)
                mod.__dict__["__builtins__"] = b
                exec(fs.files["/" + name + ".py"].decode().replace("\r", ""), mod.__dict__)
                mods[name] = mods["sys"].modules[name] = mod
                self.imported.append(name)
                return mod
            return real_import(name, *a, **k)

        b["__import__"] = imp
        b["open"] = lambda p, m="r": _File(fs, p, m)
        b["print"] = lambda *a, **k: builtins.print(*a, file=stdout, **k)
        self.globals["__builtins__"] = b
        try:
            exec(code.replace("\r", ""), self.globals)
            return stdout.getvalue().replace("\n", "\r\n").encode(), b""
        except Exception:
            return stdout.getvalue().encode(), traceback.format_exc().encode()

    # helpers
    def all_written(self):
        return b"".join(self.writes)


def _raw(board):
    D.enter_raw_repl(board, timeout=0.2, sleep=lambda s: None)
    return board


# ----------------------------------------------------------------------
# Raw REPL primitives
# ----------------------------------------------------------------------
def test_open_serial_sets_lines_low_before_open():
    board = FakeBoard()
    ser = D.open_serial("COMX", serial_factory=lambda: board)
    assert ser is board
    assert board.log == [("rts", False, False), ("dtr", False, False), ("open", None, False)]
    D.close_serial(board)
    assert [e[0] for e in board.log[-3:]] == ["rts", "dtr", "close"]


def test_enter_raw_repl_byte_sequence_interrupts_and_never_resets():
    board = FakeBoard(running_output=b"EVT FAULT type=KeyboardInterrupt msg=*00\n")
    drained = D.enter_raw_repl(board, sleep=lambda s: None)
    assert board.writes[:2] == [b"\r\x03\x03", b"\r\x01"]
    assert b"EVT FAULT" in drained
    assert board.mode == "raw"
    assert b"\x04" not in board.all_written(), "entering the raw REPL must not soft-reset"


def test_enter_raw_repl_retries_five_times_then_fails():
    board = FakeBoard(responsive=False)
    with pytest.raises(D.ReplError, match="raw REPL"):
        D.enter_raw_repl(board, timeout=0.02, sleep=lambda s: None)
    assert board.writes == [b"\r\x03\x03", b"\r\x01"] * 5


def test_exec_raw_protocol_and_errors():
    board = _raw(FakeBoard())
    n0 = len(board.writes)
    out, err = D.exec_raw(board, "print(1+1)\n", sleep=lambda s: None)
    assert (out, err) == (b"2\r\n", b"")
    assert board.writes[n0:] == [b"print(1+1)\n", b"\x04"]
    with pytest.raises(D.ReplError, match="ZeroDivisionError"):
        D.exec_checked(board, "1/0\n", sleep=lambda s: None)


def test_exec_raw_writes_in_small_blocks():
    board = _raw(FakeBoard())
    n0 = len(board.writes)
    code = "x = 1\n" * 200                    # 1200 bytes
    D.exec_raw(board, code, sleep=lambda s: None)
    blocks = board.writes[n0:-1]
    assert b"".join(blocks) == code.encode()
    assert max(len(b) for b in blocks) <= D.WRITE_BLOCK
    assert board.writes[-1] == b"\x04"


def test_chunks():
    assert D.chunks(b"") == []
    assert D.chunks(b"abc", 2) == [b"ab", b"c"]
    data = bytes(range(256)) * 3
    parts = D.chunks(data)
    assert b"".join(parts) == data and all(len(p) <= D.CHUNK for p in parts)
    assert len(parts) == 3


def test_write_and_read_file_roundtrip_binary():
    board = _raw(FakeBoard())
    data = bytes(range(256)) * 4 + b"'\"\\\r\n"
    D.write_file(board, "/x.bin", data, sleep=lambda s: None)
    assert board.fs.files["/x.bin"] == data
    assert "/x.bin.tmp" not in board.fs.files
    assert D.read_file(board, "/x.bin", sleep=lambda s: None) == data


def test_write_file_verification_failure_keeps_the_original():
    board = _raw(FakeBoard())
    board.fs.corrupt_writes = True
    with pytest.raises(D.ReplError, match="verification failed"):
        D.write_file(board, "/main.py", b"new content", sleep=lambda s: None)
    assert board.fs.files["/main.py"] == FACTORY_MAIN
    assert "/main.py.tmp" not in board.fs.files


def test_list_files_and_device_info():
    board = _raw(FakeBoard({"/boot.py": b"1", "/main.py": b"22", "/lib/a.py": b"333"}))
    assert sorted(D.list_files(board)) == [("/boot.py", 1), ("/lib/a.py", 3), ("/main.py", 2)]
    info = D.device_info(board)
    assert info["uname"][0] == "esp32" and "MicroPython" in info["sys_version"]


# ----------------------------------------------------------------------
# High-level operations through main()
# ----------------------------------------------------------------------
def _main(board, *argv):
    return D.main(["--port", "COMX", *argv], serial_factory=lambda: board)


def test_backup_downloads_everything(tmp_path):
    board = FakeBoard({"/boot.py": FACTORY_BOOT, "/main.py": FACTORY_MAIN, "/data/g.rob": b"\x00\x01"})
    assert _main(board, "--backup", str(tmp_path)) == 0
    assert (tmp_path / "main.py").read_bytes() == FACTORY_MAIN
    assert (tmp_path / "data" / "g.rob").read_bytes() == b"\x00\x01"
    info = json.loads((tmp_path / "device_info.json").read_text(encoding="utf-8"))
    assert {f["path"] for f in info["files"]} == {"/boot.py", "/main.py", "/data/g.rob"}
    assert info["main_py_is_saigen"] is False and info["uname"][0] == "esp32"
    assert board.fs.files["/main.py"] == FACTORY_MAIN, "backup never writes to the device"
    assert board.soft_resets == []
    assert [e[0] for e in board.log[-3:]] == ["rts", "dtr", "close"]


def test_install_refuses_without_backup(tmp_path, capsys):
    board = FakeBoard()
    before = dict(board.fs.files)
    with pytest.raises(SystemExit, match="REFUSED"):
        _main(board, "--install")
    with pytest.raises(SystemExit, match="REFUSED"):
        _main(FakeBoard(), "--install", "--backup", str(tmp_path / "empty"))
    assert board.fs.files == before
    assert board.soft_resets == []
    assert not board.is_open


def test_install_refuses_a_backup_of_another_program(tmp_path):
    _main(FakeBoard(), "--backup", str(tmp_path))
    board = FakeBoard({"/boot.py": FACTORY_BOOT, "/main.py": b"print('different')\n"})
    with pytest.raises(SystemExit, match="REFUSED"):
        _main(board, "--install", "--backup", str(tmp_path))
    assert board.fs.files["/main.py"] == b"print('different')\n"


def test_backup_install_restore_cycle_without_reset(tmp_path, capsys):
    board = FakeBoard()
    _main(board, "--backup", str(tmp_path))
    board.mode = "running"
    assert _main(board, "--install", "--backup", str(tmp_path)) == 0
    files = board.fs.files
    with open(os.path.join(D.FIRMWARE_DIR, "main.py"), "rb") as f:
        assert files["/main.py"] == f.read()
    with open(os.path.join(D.FIRMWARE_DIR, "saigen_maxarm.py"), "rb") as f:
        assert files["/saigen_maxarm.py"] == f.read()
    assert files["/main_factory.py"] == FACTORY_MAIN
    assert files["/boot.py"] == FACTORY_BOOT
    assert board.soft_resets == [], "no reset without --reset"
    assert board.writes[-1] == b"\r\x02", "leaves the raw REPL with Ctrl-B"
    out = capsys.readouterr().out
    assert "go_home" in out and "原点" in out
    assert "NOT reset" in out

    # a second install keeps the first main_factory.py
    board.mode = "running"
    _main(board, "--install", "--backup", str(tmp_path))
    assert board.fs.files["/main_factory.py"] == FACTORY_MAIN

    board.mode = "running"
    assert _main(board, "--restore", str(tmp_path)) == 0
    assert board.fs.files["/main.py"] == FACTORY_MAIN
    assert "/saigen_maxarm.py" not in board.fs.files
    assert board.soft_resets == []
    assert "MOVES THE ARM" in capsys.readouterr().out


def test_reset_only_with_flag_and_confirmation(tmp_path, monkeypatch):
    board = FakeBoard()
    _main(board, "--backup", str(tmp_path))
    board.mode = "running"
    monkeypatch.setattr("builtins.input", lambda prompt="": "no")
    _main(board, "--install", "--backup", str(tmp_path), "--reset")
    assert board.soft_resets == []
    board.mode = "running"
    _main(board, "--install", "--backup", str(tmp_path), "--reset", "--yes")
    assert board.soft_resets == ["friendly"], "--reset soft-resets in the friendly REPL (runs main.py)"


def test_restore_refuses_modified_backup(tmp_path):
    board = FakeBoard()
    _main(board, "--backup", str(tmp_path))
    (tmp_path / "main.py").write_bytes(b"tampered")
    board.mode = "running"
    with pytest.raises(SystemExit, match="sha256"):
        _main(board, "--restore", str(tmp_path))
    assert board.fs.files["/main.py"] == FACTORY_MAIN


def test_list(capsys):
    board = FakeBoard()
    assert _main(board, "--list") == 0
    out = capsys.readouterr().out
    assert "/main.py" in out and "/boot.py" in out


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(D.time, "sleep", lambda s: None)


# ----------------------------------------------------------------------
# Review fixes (codex_review_maxarm_device.md D9, D13, D14)
# ----------------------------------------------------------------------
def _with_rename(monkeypatch, board, fail):
    """Make os.rename on the fake device fail for the call numbers in ``fail``."""
    OS = board.fs.module_os()
    calls = []

    def rename(a, b):
        calls.append((a, b))
        if len(calls) in fail:
            raise OSError(28, "ENOSPC")
        OS.rename(a, b)
    monkeypatch.setattr(board.fs, "module_os",
                        lambda: type("O", (OS,), {"rename": staticmethod(rename)}))
    return calls


def test_d9_replace_never_leaves_the_device_without_a_copy(monkeypatch):
    board = _raw(FakeBoard())
    calls = _with_rename(monkeypatch, board, fail={1})
    with pytest.raises(D.ReplError, match="ENOSPC|28"):
        D.write_file(board, "/main.py", b"new")
    assert board.fs.files["/main.py"] == FACTORY_MAIN, "the installed file is untouched"
    assert calls == [("/main.py", "/main.py.old")]


def test_d9_interrupted_commit_keeps_the_previous_version_and_explains(monkeypatch, capsys):
    board = _raw(FakeBoard())
    _with_rename(monkeypatch, board, fail={2})         # path -> .old ok, tmp -> path fails
    with pytest.raises(D.ReplError):
        D.write_file(board, "/main.py", b"new")
    files = board.fs.files
    assert "/main.py" not in files and files["/main.py.old"] == FACTORY_MAIN
    assert files["/main.py.tmp"] == b"new"
    err = capsys.readouterr().err
    assert "RECOVERY" in err and "os.rename('/main.py.old', '/main.py')" in err
    # a second attempt refuses to destroy the only remaining copy
    monkeypatch.undo()
    monkeypatch.setattr(D.time, "sleep", lambda s: None)
    with pytest.raises(D.ReplError, match="only the previous version"):
        D.write_file(board, "/main.py", b"new")
    assert board.fs.files["/main.py.old"] == FACTORY_MAIN


def test_d9_successful_replace_leaves_no_old_file_and_removes_a_stale_one():
    board = _raw(FakeBoard())
    board.fs.files["/main.py.old"] = b"stale"          # left by an earlier interrupted update
    D.write_file(board, "/main.py", b"new")
    assert board.fs.files["/main.py"] == b"new"
    assert not any(p.endswith((".old", ".tmp")) for p in board.fs.files)


def test_d9_only_enoent_counts_as_absent(monkeypatch):
    board = _raw(FakeBoard())
    OS = board.fs.module_os()

    def stat(p):
        if p == "/main.py.old":
            raise OSError(5, "EIO")
        return OS.stat(p)
    monkeypatch.setattr(board.fs, "module_os",
                        lambda: type("O", (OS,), {"stat": staticmethod(stat)}))
    with pytest.raises(D.ReplError, match="EIO|5"):
        D.write_file(board, "/main.py", b"new")
    assert board.fs.files["/main.py"] == FACTORY_MAIN


def test_d13_backup_refuses_to_overwrite_an_existing_backup(tmp_path):
    board = FakeBoard()
    _main(board, "--backup", str(tmp_path))
    board.mode = "running"
    _main(board, "--install", "--backup", str(tmp_path))
    board.mode = "running"
    n_open = sum(1 for e in board.log if e[0] == "open")
    with pytest.raises(SystemExit, match="already holds a backup"):
        _main(board, "--backup", str(tmp_path))
    assert sum(1 for e in board.log if e[0] == "open") == n_open, "refused before opening"
    assert (tmp_path / "main.py").read_bytes() == FACTORY_MAIN, "factory copy kept"
    with pytest.raises(SystemExit, match="already holds a backup"):
        D.backup(_raw(board), str(tmp_path))
    board.mode = "running"
    _main(board, "--backup", str(tmp_path), "--overwrite-backup")
    assert b"saigen_maxarm" in (tmp_path / "main.py").read_bytes()


def test_d13_backup_and_restore_together_are_rejected(tmp_path):
    board = FakeBoard()
    _main(board, "--backup", str(tmp_path))
    board.mode = "running"
    _main(board, "--install", "--backup", str(tmp_path))
    board.mode = "running"
    n_log = len(board.log)
    with pytest.raises(SystemExit):
        _main(board, "--backup", str(tmp_path / "new"), "--restore", str(tmp_path))
    assert len(board.log) == n_log, "the port is never opened"
    assert (tmp_path / "main.py").read_bytes() == FACTORY_MAIN


def test_d14_restore_recreates_directories_and_writes_main_last(tmp_path, monkeypatch):
    board = FakeBoard({"/boot.py": FACTORY_BOOT, "/main.py": FACTORY_MAIN,
                       "/lib/sub/espmax.py": b"lib", "/z.py": b"z"})
    _main(board, "--backup", str(tmp_path))
    fs = board.fs
    fs.strict_dirs = True
    for p in list(fs.files):                          # e.g. a reformatted filesystem
        del fs.files[p]
    fs.files["/main.py"] = b"# saigen_maxarm launcher"
    fs.files["/saigen_maxarm.py"] = b"# server"
    order = []
    real = D.write_file

    def spy(ser, path, data, **kw):
        order.append(path)
        return real(ser, path, data, **kw)
    monkeypatch.setattr(D, "write_file", spy)
    board.mode = "running"
    assert _main(board, "--restore", str(tmp_path)) == 0
    assert fs.files["/lib/sub/espmax.py"] == b"lib"
    assert {"/lib", "/lib/sub"} <= fs.dirs
    assert fs.files["/main.py"] == FACTORY_MAIN and "/saigen_maxarm.py" not in fs.files
    assert order[-1] == "/main.py" and order[-2] == "/boot.py"


def test_d14_failed_restore_leaves_the_installed_main_in_charge(tmp_path, monkeypatch):
    board = FakeBoard({"/boot.py": FACTORY_BOOT, "/main.py": FACTORY_MAIN, "/a.py": b"a"})
    _main(board, "--backup", str(tmp_path))
    board.fs.files["/main.py"] = b"# saigen_maxarm launcher"
    board.fs.files["/a.py"] = b"changed"
    real = D.write_file

    def failing(ser, path, data, **kw):
        if path == "/a.py":
            raise D.ReplError("disk full")
        return real(ser, path, data, **kw)
    monkeypatch.setattr(D, "write_file", failing)
    board.mode = "running"
    with pytest.raises(D.ReplError):
        _main(board, "--restore", str(tmp_path))
    assert board.fs.files["/main.py"] == b"# saigen_maxarm launcher", "main.py not yet replaced"


# ----------------------------------------------------------------------
# Round-2 review fix (codex_review_maxarm_device2.md 14): any non-empty DIR
# ----------------------------------------------------------------------
def _opens(board):
    return sum(1 for e in board.log if e[0] == "open")


def test_r14_partial_backup_dir_is_refused_before_opening(tmp_path):
    (tmp_path / "main.py").write_bytes(FACTORY_MAIN)       # interrupted first backup
    board = FakeBoard({"/boot.py": FACTORY_BOOT, "/main.py": b"# saigen_maxarm"})
    with pytest.raises(SystemExit, match="not empty"):
        _main(board, "--backup", str(tmp_path))
    assert _opens(board) == 0, "refused before the port is opened"
    assert (tmp_path / "main.py").read_bytes() == FACTORY_MAIN, "the partial copy is kept"
    with pytest.raises(SystemExit, match="not empty"):
        D.backup(_raw(board), str(tmp_path))
    assert (tmp_path / "main.py").read_bytes() == FACTORY_MAIN
    assert not (tmp_path / "device_info.json").exists()


def test_r14_unrelated_files_and_non_directories_are_refused(tmp_path):
    (tmp_path / "notes.txt").write_text("keep me")
    with pytest.raises(SystemExit, match="REFUSED"):
        _main(FakeBoard(), "--backup", str(tmp_path))
    target = tmp_path / "a_file"
    target.write_text("x")
    assert "not a directory" in D.backup_dir_in_use(str(target))
    with pytest.raises(SystemExit, match="not a directory"):
        _main(FakeBoard(), "--backup", str(target))


def test_r14_empty_or_missing_dir_is_accepted_and_overwrite_is_explicit(tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    assert D.backup_dir_in_use(str(empty)) is None
    assert D.backup_dir_in_use(str(tmp_path / "missing")) is None
    board = FakeBoard()
    assert _main(board, "--backup", str(empty)) == 0
    assert (empty / "device_info.json").exists()
    partial = tmp_path / "partial"
    partial.mkdir()
    (partial / "boot.py").write_bytes(b"old")
    board.mode = "running"
    assert _main(board, "--backup", str(partial), "--overwrite-backup") == 0
    assert (partial / "boot.py").read_bytes() == FACTORY_BOOT
    assert (partial / "device_info.json").exists()


# ----------------------------------------------------------------------
# --check: device-side feature checks before an install (nothing moves)
# ----------------------------------------------------------------------
def _check(board, **kw):
    lines = []
    results = D.check_device(_raw(board), log=lines.append, sleep=lambda s: None, **kw)
    return results, lines


def test_check_passes_on_a_capable_build_and_leaves_no_trace():
    board = FakeBoard()
    before = dict(board.fs.files)
    results, lines = _check(board)
    assert D.checks_passed(results) and D.failed_checks(results) == []
    names = [r["name"] for r in results]
    for n in ("ticks", "poll_stdin", "stdin_roundtrip", "nozzle_alloc", "vendor_imports",
              "firmware_import", "main_compile", "memoryview_decode"):
        assert n in names and next(r for r in results if r["name"] == n)["ok"], n
    assert board.imported == [D.CHECK_MODULE] * 2, "imported from the temp file (import + self-test)"
    assert board.fs.files == before, "temporary files removed, nothing else written"
    assert board.swallow == 0, "the stdin probe byte was consumed by the probe"
    # the board is still usable afterwards (the probe byte did not pollute the raw REPL)
    assert D.list_files(board, sleep=lambda s: None) == D.list_files(board, sleep=lambda s: None)
    assert any("modules already imported" in ln for ln in lines)


def test_check_reports_a_missing_ticks_add_as_a_required_failure():
    board = FakeBoard()
    board.no_ticks_add = True
    results, lines = _check(board)
    assert D.failed_checks(results) == ["ticks"]
    assert any(ln.startswith("  FAIL ticks") for ln in lines)


def test_check_accepts_object_new_when_the_class_has_no_new(capsys):
    board = FakeBoard()
    board.no_class_new = True
    results, _ = _check(board)
    r = next(r for r in results if r["name"] == "nozzle_alloc")
    assert r["ok"] and "object.__new__ fallback" in r["detail"]


def test_check_stdin_roundtrip_fails_when_stdin_never_becomes_readable():
    board = FakeBoard()
    board.stdin_dead = True
    results, _ = _check(board)
    assert D.failed_checks(results) == ["stdin_roundtrip"]


def test_install_runs_the_checks_first_and_refuses_on_a_failure(tmp_path, monkeypatch):
    board = FakeBoard()
    bdir = str(tmp_path / "b")
    D.backup(_raw(board), bdir, log=lambda m: None, sleep=lambda s: None)
    board.no_ticks_add = True
    monkeypatch.setattr(D, "open_serial", lambda *a, **k: board)
    with pytest.raises(SystemExit) as e:
        D.main(["--port", "COMX", "--install", "--backup", bdir])
    assert "REFUSED" in str(e.value) and "ticks" in str(e.value)
    assert "/saigen_maxarm.py" not in board.fs.files, "nothing installed"
    # --force installs anyway (with a warning), --check alone just reports
    assert D.main(["--port", "COMX", "--install", "--backup", bdir, "--force"]) == 0
    assert "/saigen_maxarm.py" in board.fs.files
    board.no_ticks_add = False
    assert D.main(["--port", "COMX", "--check"]) == 0


def test_check_cli_reports_success(capsys, monkeypatch):
    board = FakeBoard()
    monkeypatch.setattr(D, "open_serial", lambda *a, **k: board)
    assert D.main(["--port", "COMX", "--check"]) == 0
    out = capsys.readouterr().out
    assert "all required device checks passed" in out
    assert "PASS stdin_roundtrip" in out
    assert board.soft_resets == []


def test_exec_raw_after_ok_is_written_once_the_code_was_accepted():
    board = _raw(FakeBoard())
    code = "import sys\nprint(7 if sys.stdin.buffer.read(1) == b'x' else 0)\n"
    out, err = D.exec_raw(board, code, sleep=lambda s: None, after_ok=b"x")
    assert (out.strip(), err) == (b"7", b"")
    assert board.writes[-2:] == [b"\x04", b"x"], "the byte follows Ctrl-D, never precedes it"


def test_check_runs_the_command_server_on_the_device_interpreter():
    board = FakeBoard()
    results, _ = _check(board)
    r = next(r for r in results if r["name"] == "firmware_selftest")
    assert r["ok"], r["detail"]
    assert board.imported.count(D.CHECK_MODULE) >= 1


def test_check_selftest_catches_a_firmware_that_misreports_an_error(tmp_path):
    """A library whose LIMIT rejection comes back under another code fails the self-test."""
    fw = tmp_path / "fw"
    fw.mkdir()
    with open(os.path.join(D.FIRMWARE_DIR, "saigen_maxarm.py"), encoding="utf-8") as f:
        src = f.read()
    broken = src.replace('raise _CmdError("LIMIT", "target outside the device box")',
                         'raise _CmdError("BADARG", "target outside the device box")')
    assert broken != src
    (fw / "saigen_maxarm.py").write_text(broken, encoding="utf-8")
    (fw / "main.py").write_bytes(open(os.path.join(D.FIRMWARE_DIR, "main.py"), "rb").read())
    results, _ = _check(FakeBoard(), firmware_dir=str(fw))
    assert D.failed_checks(results) == ["firmware_selftest"]
    detail = next(r for r in results if r["name"] == "firmware_selftest")["detail"]
    assert "4 ERR LIMIT" in detail


def test_selftest_mismatches_reports_missing_replies_and_events():
    ok = list(D.SELFTEST_EXPECTED)
    assert D.selftest_mismatches(ok, ["EVT STOPPED reason=watchdog wd=500 servo_ok=1*00\n"]) == []
    bad = D.selftest_mismatches(ok[:-1], [])
    assert any("16 ERR CHECKSUM" in b for b in bad) and any("watchdog" in b for b in bad)
