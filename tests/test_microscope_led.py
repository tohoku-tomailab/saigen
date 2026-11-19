"""UM22 microscope serial control (LED) and the microscope_led action. No hardware.

The 400-CAM106 (Vitiny UM22) exposes a CP210x serial port for its control MCU;
the protocol was taken from the vendor viewer and verified on the bench on
2026-09-22 (see src/devices/microscope/README.md).
"""
import asyncio
import csv
import os
import sys

import pytest

from src import config as lab_config
from src.devices.microscope import um22_serial
from src.devices.microscope.um22_serial import UM22SerialController
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import MICROSCOPE_ACTIONS, SHARED_DEVICE_ACTIONS, execute_step
from src.flow.schema import ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_FLOW = os.path.join(REPO_ROOT, "examples", "microscope", "product_closeup.json")


# ----------------------------------------------------------------------
# Fake MCU behind a fake serial port
# ----------------------------------------------------------------------
class FakeUM22Port:
    """Answers the CR-terminated R/S/W commands the way the real MCU does."""

    def __init__(self):
        self.is_open = True
        self.sent = []
        self.regs = {0x00: 0x01, 0x06: 12, 0x10: 0, 0x2F: 1, 0x2E: 3, 0x2D: 0}
        self.model = "UM22TW0100"
        self._rx = b""
        self.rts = False
        self.dtr = False

    def reset_input_buffer(self):
        self._rx = b""

    def write(self, data):
        cmd = data.decode()
        assert cmd.endswith("\r"), "commands must be CR-terminated"
        cmd = cmd[:-1]
        self.sent.append(cmd)
        if cmd.startswith("R00"):
            v = self.regs.get(int(cmd[3:5], 16), 0)
            self._rx += ("@%02X\r" % v).encode()
        elif cmd.startswith("S00"):
            idx = 0x2C - int(cmd[3:5], 16)
            ch = self.model[idx] if 0 <= idx < len(self.model) else "?"
            self._rx += ("@%s\r" % ch).encode()
        elif cmd == "W0159":                 # LED toggle flips status bit 1
            self.regs[0x00] ^= 0x02
        elif cmd.startswith("W06"):          # LED level write
            self.regs[0x06] = int(cmd[3:5], 16)

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def close(self):
        self.is_open = False


def _scope(port=None):
    s = UM22SerialController("COMX", reply_wait=0.01)
    s.ser = port or FakeUM22Port()
    return s


@pytest.fixture(autouse=True)
def _no_serial_sleep(monkeypatch):
    monkeypatch.setattr(um22_serial.time, "sleep", lambda s: None)


def test_frames_match_the_vendor_protocol():
    assert UM22SerialController.frame_read(0x06) == "R0006"
    assert UM22SerialController.frame_read_string(0x2C) == "S002C"
    assert UM22SerialController.frame_write(1, 64, 25) == "W0159"      # LED toggle
    assert UM22SerialController.frame_write(6, 0, 12) == "W060C"       # LED level 12
    assert UM22SerialController.frame_write(1, 64, 15) == "W014F"      # remote control on


def test_parse_value():
    assert UM22SerialController.parse_value("@0C\r") == 12
    assert UM22SerialController.parse_value("@01") == 1
    assert UM22SerialController.parse_value("") is None
    assert UM22SerialController.parse_value("start boot\r\n") is None


def test_reads_model_firmware_and_led_state():
    s = _scope()
    assert s.get_model() == "UM22TW0100"
    assert s.get_firmware() == "01.03.00"
    assert s.get_led_level() == 12
    assert s.get_led_mode() == 0
    assert s.led_is_on() is True


def test_set_led_toggles_only_when_needed():
    port = FakeUM22Port()
    s = _scope(port)
    assert s.set_led(True) is True
    assert "W0159" not in port.sent, "already on: no toggle must be sent"
    assert s.set_led(False) is False
    assert port.sent.count("W0159") == 1
    assert s.led_is_on() is False
    assert s.set_led(False) is False
    assert port.sent.count("W0159") == 1, "already off: no second toggle"
    assert s.set_led(True) is True
    assert port.sent.count("W0159") == 2


def test_set_led_with_level_writes_level_first():
    port = FakeUM22Port()
    s = _scope(port)
    s.set_led(True, level=4)
    assert "W0604" in port.sent
    assert port.regs[0x06] == 4
    assert s.get_led_level() == 4
    with pytest.raises(ValueError):
        s.set_led_level(300)


def test_set_led_raises_when_status_unreadable():
    class Silent(FakeUM22Port):
        def write(self, data):
            self.sent.append(data.decode())      # never answers

    s = _scope(Silent())
    with pytest.raises(ConnectionError):
        s.set_led(False)


def test_serial_methods_require_connection():
    s = UM22SerialController("COMX")
    assert not s.is_connected
    with pytest.raises(ConnectionError):
        s.get_status()


def test_connect_opens_with_rts_and_dtr_released(monkeypatch):
    """Asserting RTS resets the MCU, so the port must be opened with RTS/DTR released."""
    created = {}

    class FakeSerialModule:
        class Serial(FakeUM22Port):
            def __init__(self):
                super().__init__()
                self.is_open = False
                self.opened_with = None
                created["port"] = self

            def open(self):
                self.opened_with = (self.rts, self.dtr, self.baudrate, self.stopbits,
                                    self.bytesize, self.parity)
                self.is_open = True

    monkeypatch.setitem(sys.modules, "serial", FakeSerialModule)
    s = UM22SerialController("COMX", reply_wait=0.01)
    assert s.connect() is True
    assert created["port"].opened_with == (False, False, 115200, 2, 8, "N")
    s.disconnect()
    assert not created["port"].is_open


# ----------------------------------------------------------------------
# microscope_led through SharedDevices / mock / schema / executor / CLI
# ----------------------------------------------------------------------
class FakeCamera:
    def __init__(self, camera_index, resolution=None):
        self.camera_index = camera_index

    def connect(self):
        return True

    def disconnect(self):
        pass


class FakeSerialCtl:
    instances = []

    def __init__(self, port):
        self.port = port
        self.calls = []
        FakeSerialCtl.instances.append(self)

    def connect(self):
        return True

    def get_model(self):
        return "UM22TW0100"

    def set_led(self, on, level=None):
        self.calls.append((on, level))
        return on

    def disconnect(self):
        self.calls.append("closed")


def test_shared_devices_open_serial_only_when_port_configured(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeCamera)
    monkeypatch.setattr(pkg, "UM22SerialController", FakeSerialCtl)
    FakeSerialCtl.instances.clear()

    shared = SharedDevices(use_microscope=True, microscope_index=2, microscope_port="COM77")   # camera only
    assert asyncio.run(shared.initialize()) is True
    assert FakeSerialCtl.instances == []
    with pytest.raises(RuntimeError):
        asyncio.run(shared.set_microscope_led(False))

    shared = SharedDevices(use_microscope_serial=True, microscope_index=2, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    ctl = FakeSerialCtl.instances[0]
    assert ctl.port == "COM77"
    assert asyncio.run(shared.set_microscope_led(False)) is False
    assert asyncio.run(shared.set_microscope_led(True, level=8)) is True
    assert ctl.calls[:2] == [(False, None), (True, 8)]
    asyncio.run(shared.cleanup())
    assert ctl.calls[-1] == "closed"


def test_mock_shared_devices_led():
    mock = MockSharedDevices(use_microscope=True)
    assert asyncio.run(mock.set_microscope_led(False)) is False
    assert mock.microscope_led_on is False
    assert asyncio.run(mock.set_microscope_led(True, level=5)) is True
    assert mock.microscope_led_level == 5


def test_schema_microscope_led_defaults_and_bounds():
    wf = ExperimentWorkflow(name="m", description="d", steps=[{"action": "microscope_led"}])
    assert wf.steps[0].on is True and wf.steps[0].level is None
    with pytest.raises(Exception):
        ExperimentWorkflow(name="m", description="d",
                           steps=[{"action": "microscope_led", "level": 999}])


def test_executor_routes_microscope_led():
    assert "microscope_led" in SHARED_DEVICE_ACTIONS and "microscope_led" in MICROSCOPE_ACTIONS
    mock = MockSharedDevices(use_microscope=True)
    result = asyncio.run(execute_step({"action": "microscope_led", "on": False},
                                      robots={}, shared_devices=mock))
    assert result is None
    assert mock.microscope_led_on is False


def test_plan_resources_led_needs_microscope():
    plan = run_flow.plan_resources([{"action": "microscope_led", "on": False}])
    assert (plan.robot_ids, plan.needs_scale, plan.needs_camera, plan.needs_microscope,
            plan.needs_microscope_serial) == ([], False, False, False, True)


def test_resolve_ports_microscope_port(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.delenv("MICROSCOPE_PORT", raising=False)
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["microscope_port"] == lab_config.get_shared_devices()["microscope_port"]
    monkeypatch.setenv("MICROSCOPE_PORT", "COM55")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["microscope_port"] == "COM55"
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--microscope-port", "COM56"]))
    assert shared["microscope_port"] == "COM56"


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def test_example_flow_validates():
    assert run_flow.main([EXAMPLE_FLOW, "--validate-only"]) == 0


def test_mock_run_with_led_steps_completes(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE_FLOW, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    led = [r for r in rows if r["action"] == "microscope_led"]
    assert len(led) == 2 and all(r["status"] == "ok" for r in led)
    micro = [r for r in rows if r["action"] == "capture_microscope"]
    assert len(micro) == 2 and all("_microscope_" in r["image_path"] for r in micro)
