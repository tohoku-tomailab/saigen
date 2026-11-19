"""Fixes from the Codex review of the microscope support (2026-09-23):
communication loss is not "motor stopped", motor stop on errors and
cancellation, transactional initialisation, camera/serial planned separately,
schema requires position for mode=position, focus_converged is persisted,
an explicit empty --microscope-port disables serial control."""
import asyncio
import csv
import os

import pytest

from src.devices.microscope import um22_serial
from src.devices.microscope.um22_serial import UM22SerialController
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import ExperimentWorkflow
from tests.test_microscope_focus import FakeMotorPort


def _scope(port):
    s = UM22SerialController("COMX", reply_wait=0.01)
    s.ser = port
    return s


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(um22_serial.time, "sleep", lambda s: None)


# ----------------------------------------------------------------------
# #2 lost communication is never reported as a stopped motor
# ----------------------------------------------------------------------
class DeadPort(FakeMotorPort):
    """Stops answering after `alive_polls` reads."""

    def __init__(self, alive_polls=2):
        super().__init__()
        self.alive_polls = alive_polls
        self.reads = 0

    def write(self, data):
        cmd = data.decode()[:-1]
        self.sent.append(cmd)
        if cmd.startswith("R00"):
            self.reads += 1
            if self.reads > self.alive_polls:
                return                          # silence
        super().write(data)


def test_wait_motor_raises_on_lost_communication():
    s = _scope(DeadPort(alive_polls=2))
    with pytest.raises(ConnectionError):
        s.wait_motor(timeout_s=30)


def test_wait_motor_does_not_count_missing_reads_as_settled():
    class Flaky(FakeMotorPort):
        """Every other status read is lost; the motor is actually moving."""
        def __init__(self):
            super().__init__(pos=1000)
            self.target = 1600
            self.n = 0

        def write(self, data):
            cmd = data.decode()[:-1]
            if cmd == "R0000":
                self.n += 1
                if self.n % 4 == 0:
                    self.sent.append(cmd)
                    return                      # every 4th status reply is lost
            super().write(data)

    s = _scope(Flaky())
    pos, stopped = s.wait_motor(timeout_s=5)
    assert stopped is True and pos == 1600, "settles only once the motor really reached its target"


def test_autofocus_stops_motor_when_communication_breaks():
    port = DeadPort(alive_polls=3)
    s = _scope(port)
    with pytest.raises(ConnectionError):
        s.autofocus(timeout_s=30)
    assert port.sent[-1] == "W0154", "manual mode must be sent on the error path"


# ----------------------------------------------------------------------
# #1 cooperative stop from another thread
# ----------------------------------------------------------------------
def test_request_stop_ends_hunting_autofocus():
    port = FakeMotorPort(pos=1568, af_hunts=True)
    s = _scope(port)
    s.request_stop()                            # requested before the wait starts polling
    pos, ok = s.autofocus(timeout_s=30)
    assert ok is False
    assert "W0154" in port.sent
    assert not s._stop_requested.is_set(), "flag is cleared for the next operation"


# ----------------------------------------------------------------------
# #3 / #4 step: release in finally, timeout honoured, completion reported
# ----------------------------------------------------------------------
def test_step_release_is_sent_even_if_hold_raises(monkeypatch):
    port = FakeMotorPort()
    s = _scope(port)
    calls = {"n": 0}

    def boom(_):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("interrupted while the button is held")

    monkeypatch.setattr(um22_serial.time, "sleep", boom)
    with pytest.raises(RuntimeError):
        s.step_focus("out", steps=1)
    assert port.sent[-1] == "W0193", "the release command must follow the press"


def test_step_returns_completion_and_stops_at_timeout(monkeypatch):
    port = FakeMotorPort(pos=1568)
    s = _scope(port)
    pos, completed = s.step_focus("out", steps=2, timeout_s=30)
    assert completed is True and pos == 1568 + 14
    clock = [0.0]

    def fake_monotonic():
        clock[0] += 0.5
        return clock[0]

    monkeypatch.setattr(um22_serial.time, "monotonic", fake_monotonic)
    port2 = FakeMotorPort(pos=1568)
    pos, completed = _scope(port2).step_focus("out", steps=50, timeout_s=1.0)
    assert completed is False, "timeout cuts the press loop short and reports it"
    assert port2.sent.count("W0153") < 50


# ----------------------------------------------------------------------
# #6 / #8 SharedDevices: transactional init, camera and serial separate
# ----------------------------------------------------------------------
class FakeCamera:
    instances = []

    def __init__(self, camera_index, resolution=None):
        self.open = False
        self.ok = True
        FakeCamera.instances.append(self)

    def connect(self):
        self.open = self.ok
        return self.ok

    def disconnect(self):
        self.open = False


class FakeSerial:
    instances = []
    ok = True

    def __init__(self, port):
        self.port = port
        self.open = False
        self.stopped = False
        FakeSerial.instances.append(self)

    def connect(self):
        self.open = FakeSerial.ok
        return FakeSerial.ok

    def get_model(self):
        return "UM22TW0100"

    def disconnect(self):
        self.open = False

    def request_stop(self):
        self.stopped = True

    def set_led(self, on, level=None):
        return on

    def autofocus(self, timeout_s):
        # time.sleep is patched to a no-op by the autouse fixture, so wait on an
        # Event instead: the fake motor "moves" for up to one second
        import threading
        ev = threading.Event()
        for _ in range(50):
            if self.stopped:
                return 1500, False
            ev.wait(0.02)
        return 1600, True


@pytest.fixture
def fakes(monkeypatch):
    import src.devices.microscope as pkg
    monkeypatch.setattr(pkg, "MicroscopeController", FakeCamera)
    monkeypatch.setattr(pkg, "UM22SerialController", FakeSerial)
    FakeCamera.instances.clear()
    FakeSerial.instances.clear()
    FakeSerial.ok = True
    yield


def test_led_only_opens_serial_not_camera(fakes):
    shared = SharedDevices(use_microscope_serial=True, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    assert FakeCamera.instances == [] and len(FakeSerial.instances) == 1
    assert asyncio.run(shared.set_microscope_led(False)) is False
    with pytest.raises(RuntimeError):
        asyncio.run(shared.capture_microscope())


def test_capture_only_opens_camera_not_serial(fakes):
    shared = SharedDevices(use_microscope=True, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    assert len(FakeCamera.instances) == 1 and FakeSerial.instances == []
    with pytest.raises(RuntimeError):
        asyncio.run(shared.set_microscope_led(False))


def test_serial_requested_without_port_fails_before_opening_anything(fakes):
    shared = SharedDevices(use_microscope_serial=True)
    assert asyncio.run(shared.initialize()) is False
    assert FakeSerial.instances == []


def test_serial_failure_releases_the_camera(fakes):
    FakeSerial.ok = False
    shared = SharedDevices(use_microscope=True, use_microscope_serial=True, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is False
    assert shared.microscope is not None and shared.microscope_serial is None
    asyncio.run(shared.cleanup())
    assert FakeCamera.instances[0].open is False, "camera handle released by cleanup"


def test_camera_failure_leaves_no_handle(fakes):
    class BadCamera(FakeCamera):
        def __init__(self, camera_index, resolution=None):
            super().__init__(camera_index, resolution)
            self.ok = False

    import src.devices.microscope as pkg
    pkg.MicroscopeController = BadCamera
    shared = SharedDevices(use_microscope=True)
    assert asyncio.run(shared.initialize()) is False
    assert shared.microscope is None


def test_autofocus_needs_the_camera(fakes):
    shared = SharedDevices(use_microscope_serial=True, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    with pytest.raises(RuntimeError):
        asyncio.run(shared.focus_microscope("auto"))


# ----------------------------------------------------------------------
# #1 cancelling a focus step stops the worker before returning
# ----------------------------------------------------------------------
def test_cancelled_focus_requests_stop_and_waits_for_worker(fakes):
    shared = SharedDevices(use_microscope=True, use_microscope_serial=True, microscope_port="COM77")
    assert asyncio.run(shared.initialize()) is True
    ctl = shared.microscope_serial

    async def main():
        task = asyncio.create_task(shared.focus_microscope("auto", timeout=10))
        await asyncio.sleep(0.1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        return ctl.stopped

    assert asyncio.run(main()) is True


# ----------------------------------------------------------------------
# #9 schema, #5 logger, #13 CLI, planner
# ----------------------------------------------------------------------
def test_schema_requires_position_for_position_mode():
    with pytest.raises(Exception):
        ExperimentWorkflow(name="m", description="d",
                           steps=[{"action": "microscope_focus", "mode": "position"}])
    wf = ExperimentWorkflow(name="m", description="d",
                            steps=[{"action": "microscope_focus", "mode": "position", "position": 1600}])
    assert wf.steps[0].position == 1600


def test_validate_only_rejects_position_mode_without_position(tmp_path):
    import json
    flow = tmp_path / "f.json"
    flow.write_text(json.dumps({"name": "x", "description": "y",
                                "steps": [{"action": "microscope_focus", "mode": "position"}]}), encoding="utf-8")
    assert run_flow.main([str(flow), "--validate-only"]) != 0


def test_logger_persists_focus_converged(tmp_path):
    src = tmp_path / "flow.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "d", str(src), base_dir=str(tmp_path / "logs"))
    exp.record_step(1, 2, "microscope_focus", None, None, "ok", 1.0,
                    result={"focus_position": 1500, "focus_converged": False})
    exp.record_step(2, 2, "microscope_focus", None, None, "ok", 1.0,
                    result={"focus_position": 1600, "focus_converged": True})
    exp.finalize("completed", None)
    with open(exp.csv_path, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["focus_converged"] for r in rows] == ["False", "True"]
    assert "not converged" in open(os.path.join(exp.dir, "summary.md"), encoding="utf-8").read()


def test_empty_cli_port_disables_serial_control(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.setenv("MICROSCOPE_PORT", "COM55")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--microscope-port", ""]))
    assert shared["microscope_port"] == ""
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["microscope_port"] == "COM55"


def test_plan_resources_splits_camera_and_serial():
    plan = run_flow.plan_resources
    assert plan([{"action": "capture_microscope"}])[4:6] == (True, False)
    assert plan([{"action": "microscope_led", "on": False}])[4:6] == (False, True)
    assert plan([{"action": "microscope_focus"}])[4:6] == (True, True)


def test_mock_session_passes_serial_flag():
    session = ExperimentSession(mock=True, robot_ports={}, shared_config={}, workspace_validator=None)

    async def main():
        shared = await session.add_shared(use_microscope_serial=True)
        return shared

    shared = asyncio.run(main())
    assert isinstance(shared, MockSharedDevices)
    assert shared.use_microscope_serial is True and shared.use_microscope is False
