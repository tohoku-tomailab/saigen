"""UM22 microscope focus motor (autofocus / go-to position / step) and the
microscope_focus action. No hardware: a fake MCU simulates the motor."""
import asyncio
import csv
import os

import pytest

from src.devices.microscope import um22_serial
from src.devices.microscope.um22_serial import UM22SerialController
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import MICROSCOPE_ACTIONS, SHARED_DEVICE_ACTIONS, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.schema import ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_FLOW = os.path.join(REPO_ROOT, "examples", "microscope", "product_closeup.json")


class FakeMotorPort:
    """Simulates the UM22 MCU's focus motor: every read of the position advances
    the motor a bit towards its target, the status busy bit follows."""

    def __init__(self, pos=1568, af_target=1400, af_hunts=False):
        self.is_open = True
        self.sent = []
        self.pos = pos
        self.target = pos
        self.af_target = af_target
        self.af_hunts = af_hunts          # True: autofocus never settles
        self.mode = 0
        self.led_off = True
        self.tgt_hi = 0
        self.tgt_lo = 0
        self._rx = b""
        self.rts = False
        self.dtr = False

    def _tick(self):
        if self.af_hunts and self.mode == 2:
            self.pos = (self.pos + 25) % 2000
            return
        if self.pos != self.target:
            step = 25 if self.target > self.pos else -25
            if abs(self.target - self.pos) <= 25:
                self.pos = self.target
            else:
                self.pos += step

    def _busy(self):
        return self.pos != self.target or (self.af_hunts and self.mode == 2)

    def reset_input_buffer(self):
        self._rx = b""

    def write(self, data):
        cmd = data.decode()
        assert cmd.endswith("\r")
        cmd = cmd[:-1]
        self.sent.append(cmd)
        if cmd.startswith("R00"):
            addr = int(cmd[3:5], 16)
            if addr == 0x09:
                self._tick()
                v = self.pos >> 8
            elif addr == 0x0A:
                v = self.pos & 0xFF
            elif addr == 0x00:
                v = (0x03 if self.led_off else 0x01) | (0x04 if self._busy() else 0)
            elif addr == 0x04:
                v = self.mode
            else:
                v = 0
            self._rx += ("@%02X\r" % v).encode()
        elif cmd == "W0155":                       # single-shot AF
            self.mode = 2
            if not self.af_hunts:
                self.target = self.af_target
        elif cmd == "W0154":                       # manual mode: stop hunting
            self.mode = 0
            self.target = self.pos
        elif cmd.startswith("W18"):
            self.tgt_hi = int(cmd[3:5], 16)
        elif cmd.startswith("W19"):
            self.tgt_lo = int(cmd[3:5], 16)
        elif cmd == "W016E":                       # go to position
            self.target = (self.tgt_hi << 8) | self.tgt_lo
        elif cmd == "W0152":                       # step in (press): position value decreases
            self.target = self.pos - 14
        elif cmd == "W0153":                       # step out (press): position value increases
            self.target = self.pos + 14

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def close(self):
        self.is_open = False


def _scope(port):
    s = UM22SerialController("COMX", reply_wait=0.01)
    s.ser = port
    return s


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(um22_serial.time, "sleep", lambda s: None)


def test_focus_frames():
    assert UM22SerialController.frame_write(1, 64, 21) == "W0155"    # single AF
    assert UM22SerialController.frame_write(1, 64, 20) == "W0154"    # manual
    assert UM22SerialController.frame_write(1, 64, 46) == "W016E"    # go to position
    assert UM22SerialController.frame_write(0x18, 0, 0x06) == "W1806"
    assert UM22SerialController.frame_write(0x19, 0, 0x20) == "W1920"
    assert UM22SerialController.frame_write(1, 128, 18) == "W0192"   # release step-in


def test_motor_position_and_busy_flag():
    port = FakeMotorPort(pos=1568)
    s = _scope(port)
    assert s.get_motor_position() == 1568
    assert s.motor_is_busy() is False
    port.target = 1600
    assert s.motor_is_busy() is True


def test_autofocus_converges_and_reports_position():
    port = FakeMotorPort(pos=1568, af_target=1400)
    s = _scope(port)
    pos, ok = s.autofocus(timeout_s=30)
    assert ok is True
    assert pos == 1400
    assert "W0155" in port.sent
    assert "W0154" not in port.sent, "no manual fallback when AF converged"


def test_autofocus_timeout_falls_back_to_manual(monkeypatch):
    port = FakeMotorPort(pos=1568, af_hunts=True)
    s = _scope(port)
    # a fake clock that advances 1 ms per call: transact() still gets its reply
    # (the fake port answers at once), while wait_motor() runs into its timeout
    clock = [0.0]

    def fake_monotonic():
        clock[0] += 0.001
        return clock[0]

    monkeypatch.setattr(um22_serial.time, "monotonic", fake_monotonic)
    pos, ok = s.autofocus(timeout_s=0.2)
    assert ok is False
    assert "W0154" in port.sent, "hunting AF must be stopped by switching to manual"
    assert isinstance(pos, int)


def test_goto_position_writes_bytes_and_waits():
    port = FakeMotorPort(pos=1755)
    s = _scope(port)
    pos, reached = s.goto_position(1568, timeout_s=30)
    assert (pos, reached) == (1568, True)
    assert port.sent[:3] == ["W1806", "W1920", "W016E"]
    with pytest.raises(ValueError):
        s.goto_position(70000)


def test_step_focus_presses_and_releases():
    port = FakeMotorPort(pos=1568)
    s = _scope(port)
    pos, completed = s.step_focus("out", steps=2)
    assert completed is True
    assert pos == 1568 + 14, "each step lands where the fake motor moves per press"
    assert port.sent.count("W0153") == 2 and port.sent.count("W0193") == 2
    with pytest.raises(ValueError):
        s.step_focus("sideways")


def test_set_focus_mode_rejects_unknown():
    s = _scope(FakeMotorPort())
    with pytest.raises(ValueError):
        s.set_focus_mode("magic")


# ----------------------------------------------------------------------
# SharedDevices / mock / schema / executor / logger / CLI
# ----------------------------------------------------------------------
class FakeCamera:
    def __init__(self, camera_index, resolution=None):
        pass

    def connect(self):
        return True

    def disconnect(self):
        pass


class FakeSerialCtl:
    def __init__(self, port):
        self.calls = []

    def connect(self):
        return True

    def get_model(self):
        return "UM22TW0100"

    def disconnect(self):
        pass

    def autofocus(self, timeout_s):
        self.calls.append(("af", timeout_s))
        return 1400, True

    def goto_position(self, position, timeout_s):
        self.calls.append(("goto", position, timeout_s))
        return position, True

    def step_focus(self, direction, steps, timeout_s=60.0):
        self.calls.append(("step", direction, steps))
        return 1582, True


def test_shared_devices_focus_dispatch(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeCamera)
    monkeypatch.setattr(pkg, "UM22SerialController", FakeSerialCtl)
    shared = SharedDevices(use_microscope=True, use_microscope_serial=True, microscope_index=2, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    ctl = shared.microscope_serial
    assert asyncio.run(shared.focus_microscope("auto", timeout=30)) == {"focus_position": 1400, "focus_converged": True}
    assert asyncio.run(shared.focus_microscope("position", position=1568)) == {"focus_position": 1568, "focus_converged": True}
    assert asyncio.run(shared.focus_microscope("step", direction="in", steps=3))["focus_position"] == 1582
    assert ctl.calls == [("af", 30), ("goto", 1568, 60.0), ("step", "in", 3)]
    with pytest.raises(ValueError):
        asyncio.run(shared.focus_microscope("position"))
    with pytest.raises(ValueError):
        asyncio.run(shared.focus_microscope("magic"))


def test_shared_devices_focus_requires_port(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeCamera)
    shared = SharedDevices(use_microscope=True, microscope_index=2)
    assert asyncio.run(shared.initialize()) is True
    with pytest.raises(RuntimeError):
        asyncio.run(shared.focus_microscope("auto"))


def test_mock_focus():
    mock = MockSharedDevices(use_microscope=True)
    assert asyncio.run(mock.focus_microscope("auto"))["focus_position"] == 1400
    assert asyncio.run(mock.focus_microscope("position", position=900))["focus_position"] == 900
    assert asyncio.run(mock.focus_microscope("step", direction="out", steps=2))["focus_position"] == 900 + 28


def test_schema_focus_defaults_and_bounds():
    wf = ExperimentWorkflow(name="m", description="d", steps=[{"action": "microscope_focus"}])
    st = wf.steps[0]
    assert (st.mode, st.position, st.direction, st.steps, st.timeout) == ("auto", None, "in", 1, 60.0)
    with pytest.raises(Exception):
        ExperimentWorkflow(name="m", description="d", steps=[{"action": "microscope_focus", "mode": "sideways"}])
    with pytest.raises(Exception):
        ExperimentWorkflow(name="m", description="d", steps=[{"action": "microscope_focus", "position": 70000}])


def test_executor_routes_focus_and_returns_position():
    assert "microscope_focus" in SHARED_DEVICE_ACTIONS and "microscope_focus" in MICROSCOPE_ACTIONS
    mock = MockSharedDevices(use_microscope=True)
    result = asyncio.run(execute_step({"action": "microscope_focus", "mode": "position", "position": 1234},
                                      robots={}, shared_devices=mock))
    assert result == {"focus_position": 1234, "focus_converged": True}


def test_logger_writes_focus_position_column(tmp_path):
    src = tmp_path / "flow.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "d", str(src), base_dir=str(tmp_path / "logs"))
    exp.record_step(1, 1, "microscope_focus", None, None, "ok", 0.5,
                    result={"focus_position": 1400, "focus_converged": True})
    exp.finalize("completed", None)
    with open(exp.csv_path, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["focus_position"] == "1400"
    assert "focus_position" in rows[0]


def test_plan_resources_focus_needs_microscope():
    ids, picus, scale, camera, microscope, serial, dmm = run_flow.plan_resources([{"action": "microscope_focus"}])
    assert (ids, scale, camera, microscope) == ([], False, False, True)


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def test_example_flow_mock_run_records_focus(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE_FLOW, "--validate-only"]) == 0
    assert run_flow.main([EXAMPLE_FLOW, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    focus = [r for r in rows if r["action"] == "microscope_focus"]
    assert len(focus) == 1 and focus[0]["status"] == "ok" and focus[0]["focus_position"] == "1400"
