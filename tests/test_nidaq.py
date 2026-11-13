"""NI DAQ (USB-6351) driver, output policy and the daq_* flow actions. No hardware.

``nidaqmx`` is replaced by a fake module that records what a task was configured
with and what was written, so these tests run where neither the package nor the
NI-DAQmx driver is installed (CI).
"""
import asyncio
import csv
import inspect
import os
import sys
import types

import pytest
from pydantic import ValidationError

from src.devices.nidaq import DaqOutputPolicy, DaqPolicyError, NIDaq
from src.devices.nidaq import policy as daq_policy
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import DAQ_ACTIONS, SHARED_DEVICE_ACTIONS, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_FLOW = os.path.join(REPO_ROOT, "examples", "daq", "read_voltage.json")


# ----------------------------------------------------------------------
# Fake nidaqmx
# ----------------------------------------------------------------------
class Bench:
    """What the fake device saw."""

    def __init__(self, devices=("Dev1",)):
        self.devices = list(devices)
        self.inputs = {}          # physical channel -> value returned by read()
        self.outputs = {}         # physical channel -> last value written
        self.events = []          # ("write", phys, value) / ("close", phys)
        self.ai_configs = []      # kwargs of add_ai_voltage_chan
        self.timings = []
        self.fail_write = set()   # physical channels whose write raises
        self.open_tasks = 0


def install_fake_nidaqmx(monkeypatch, bench):
    class _Channels:
        def __init__(self, task, kind):
            self.task, self.kind = task, kind

        def _add(self, phys, **kwargs):
            self.task.kind, self.task.phys = self.kind, phys
            if self.kind == "ai":
                bench.ai_configs.append({"phys": phys, **kwargs})

        add_ai_voltage_chan = add_ao_voltage_chan = add_do_chan = add_di_chan = _add

    class _Timing:
        def cfg_samp_clk_timing(self, rate, sample_mode=None, samps_per_chan=None):
            bench.timings.append((rate, sample_mode, samps_per_chan))

    class Task:
        def __init__(self):
            bench.open_tasks += 1
            self.kind = self.phys = None
            self.ai_channels = _Channels(self, "ai")
            self.ao_channels = _Channels(self, "ao")
            self.do_channels = _Channels(self, "do")
            self.di_channels = _Channels(self, "di")
            self.timing = _Timing()

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()
            return False

        def read(self, number_of_samples_per_channel=None, timeout=10.0):
            value = bench.inputs.get(self.phys, 0.0)
            if number_of_samples_per_channel is None:
                return value
            return [value] * number_of_samples_per_channel if not isinstance(value, list) else value

        def write(self, data, auto_start=None):
            if self.phys in bench.fail_write:
                raise RuntimeError("write failed")
            assert auto_start is True
            bench.outputs[self.phys] = data
            bench.events.append(("write", self.phys, data))

        def close(self):
            bench.open_tasks -= 1
            bench.events.append(("close", self.phys))

    class _Device:
        def __init__(self, name):
            self.name, self.product_type = name, "USB-6351"

    class _Devices(list):
        pass

    class _System:
        @staticmethod
        def local():
            return types.SimpleNamespace(devices=_Devices(_Device(n) for n in bench.devices))

    constants = types.ModuleType("nidaqmx.constants")
    constants.TerminalConfiguration = types.SimpleNamespace(
        DEFAULT="T_DEFAULT", RSE="T_RSE", NRSE="T_NRSE", DIFF="T_DIFF")
    constants.AcquisitionType = types.SimpleNamespace(FINITE="FINITE")
    constants.LineGrouping = types.SimpleNamespace(CHAN_PER_LINE="CHAN_PER_LINE")
    system = types.ModuleType("nidaqmx.system")
    system.System = _System
    root = types.ModuleType("nidaqmx")
    root.Task, root.constants, root.system = Task, constants, system
    monkeypatch.setitem(sys.modules, "nidaqmx", root)
    monkeypatch.setitem(sys.modules, "nidaqmx.constants", constants)
    monkeypatch.setitem(sys.modules, "nidaqmx.system", system)
    return bench


@pytest.fixture
def bench(monkeypatch):
    return install_fake_nidaqmx(monkeypatch, Bench())


# ----------------------------------------------------------------------
# Output policy
# ----------------------------------------------------------------------
def test_policy_refuses_outputs_by_default():
    policy = DaqOutputPolicy.from_config({})
    with pytest.raises(DaqPolicyError, match="daq_ao_channels"):
        policy.check_analog_output("ao0", 1.0)
    with pytest.raises(DaqPolicyError, match="daq_do_lines"):
        policy.check_digital_output("port0/line0")


def test_policy_allows_listed_channels_inside_the_span():
    policy = DaqOutputPolicy.from_config({"daq_ao_channels": ["AO0"], "daq_do_lines": ["Port0/Line3"],
                                          "daq_ao_min_v": 0, "daq_ao_max_v": 5})
    assert policy.check_analog_output(" ao0 ", 5) == ("ao0", 5.0)
    assert policy.check_digital_output("port0/line3") == "port0/line3"
    for volts in (-0.01, 5.01, float("nan")):
        with pytest.raises(DaqPolicyError, match="outside the configured span"):
            policy.check_analog_output("ao0", volts)
    with pytest.raises(DaqPolicyError, match="not enabled"):
        policy.check_analog_output("ao1", 1.0)
    with pytest.raises(DaqPolicyError, match="not enabled"):
        policy.check_digital_output("port0/line4")


@pytest.mark.parametrize("config", [
    {"daq_ao_max_v": None},                         # YAML "daq_ao_max_v:" is not "no limit"
    {"daq_ao_min_v": 3, "daq_ao_max_v": 1},
    {"daq_ao_max_v": 12},
    {"daq_ao_min_v": -11},
    {"daq_ao_channels": ["Dev1/ao0"]},              # device name does not belong here
    {"daq_do_lines": ["line0"]},
])
def test_policy_rejects_bad_configuration(config):
    with pytest.raises(DaqPolicyError):
        DaqOutputPolicy.from_config(config)


@pytest.mark.parametrize("fn,bad", [
    (daq_policy.normalize_ai, "ao0"), (daq_policy.normalize_ai, "Dev1/ai0"),
    (daq_policy.normalize_ao, "ai0"), (daq_policy.normalize_ao, "ao0:1"),
    (daq_policy.normalize_line, "port0"), (daq_policy.normalize_line, "port0/line0:7"),
])
def test_channel_names_are_strict(fn, bad):
    with pytest.raises(DaqPolicyError):
        fn(bad)


# ----------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------
def test_connect_checks_the_device_name(bench):
    daq = NIDaq(device="Dev1")
    assert daq.connect() is True and daq.product_type == "USB-6351"
    assert NIDaq(device="Dev9").connect() is False
    assert bench.events == [] and bench.outputs == {}      # connecting drives nothing


def test_connect_reports_a_missing_driver(monkeypatch):
    monkeypatch.setitem(sys.modules, "nidaqmx", None)      # import nidaqmx -> ImportError
    assert NIDaq(device="Dev1").connect() is False


def test_read_analog_single_sample(bench):
    bench.inputs["Dev1/ai3"] = 2.5
    daq = NIDaq(device="Dev1")
    assert daq.read_analog("ai3", samples=1, v_range=5.0, terminal="rse") == 2.5
    assert bench.ai_configs == [{"phys": "Dev1/ai3", "terminal_config": "T_RSE",
                                 "min_val": -5.0, "max_val": 5.0}]
    assert bench.timings == []                              # on demand, no sample clock
    assert bench.open_tasks == 0


def test_read_analog_averages_clocked_samples(bench):
    bench.inputs["Dev1/ai0"] = [1.0, 2.0, 3.0, 6.0]
    daq = NIDaq(device="Dev1")
    assert daq.read_analog("ai0", samples=4, rate_hz=2000.0, terminal="diff") == 3.0
    assert bench.timings == [(2000.0, "FINITE", 4)]
    assert bench.ai_configs[0]["terminal_config"] == "T_DIFF"
    assert bench.open_tasks == 0


def test_read_analog_rejects_bad_arguments_and_short_reads(bench):
    daq = NIDaq(device="Dev1")
    for kwargs in ({"samples": 0}, {"terminal": "floating"}, {"v_range": 0}, {"v_range": 20}):
        with pytest.raises(ValueError):
            daq.read_analog("ai0", **kwargs)
    bench.inputs["Dev1/ai0"] = [1.0, 2.0]
    with pytest.raises(RuntimeError, match="asked for 4 samples, got 2"):
        daq.read_analog("ai0", samples=4)


def test_write_analog_holds_one_task_per_channel(bench):
    daq = NIDaq(device="Dev1")
    assert daq.write_analog("ao0", 1.5) == 1.5
    daq.write_analog("ao0", 2.0)
    assert bench.outputs == {"Dev1/ao0": 2.0}
    assert bench.open_tasks == 1                            # held open, so the value is held
    with pytest.raises(ValueError, match="outside the device span"):
        daq.write_analog("ao0", 10.5)
    assert bench.outputs == {"Dev1/ao0": 2.0}


def test_digital_write_then_read_of_the_same_line_is_refused(bench):
    bench.inputs["Dev1/port0/line1"] = True
    daq = NIDaq(device="Dev1")
    assert daq.read_digital("port0/line1") is True
    assert daq.write_digital("port0/line0", True) is True
    assert bench.outputs["Dev1/port0/line0"] is True
    with pytest.raises(RuntimeError, match="driven as an output"):
        daq.read_digital("port0/line0")


def test_close_returns_outputs_to_safe_state_before_releasing_them(bench):
    daq = NIDaq(device="Dev1")
    daq.write_analog("ao1", 4.0)
    daq.write_digital("port0/line2", True)
    bench.events.clear()
    assert daq.close() == []
    assert bench.outputs == {"Dev1/ao1": 0.0, "Dev1/port0/line2": False}
    kinds = [e[0] for e in bench.events]
    assert kinds == ["write", "write", "close", "close"]    # reset first, then release
    assert bench.open_tasks == 0


def test_close_attempts_every_output_and_reports_the_ones_it_could_not_reset(bench):
    daq = NIDaq(device="Dev1")
    daq.write_analog("ao0", 3.0)
    daq.write_digital("port0/line0", True)
    bench.fail_write.add("Dev1/ao0")
    assert daq.close() == ["ao0"]
    assert bench.outputs["Dev1/port0/line0"] is False       # the other one still went low
    assert bench.open_tasks == 0


# ----------------------------------------------------------------------
# SharedDevices (real wrapper over the fake device)
# ----------------------------------------------------------------------
def _shared(**config):
    shared = SharedDevices(use_daq=True, **config)
    assert asyncio.run(shared.initialize()) is True
    return shared


def test_shared_devices_needs_a_device_name(bench):
    assert asyncio.run(SharedDevices(use_daq=True).initialize()) is False
    assert asyncio.run(SharedDevices(use_daq=True, daq_device="Dev9").initialize()) is False
    assert asyncio.run(SharedDevices(use_daq=True, daq_device="Dev1",
                                     daq_ao_max_v=None).initialize()) is False


def test_shared_devices_without_daq_raises():
    shared = SharedDevices()
    with pytest.raises(RuntimeError, match="DAQ"):
        asyncio.run(shared.daq_read_analog("ai0"))


def test_shared_devices_reads(bench):
    bench.inputs["Dev1/ai0"] = 0.75
    bench.inputs["Dev1/port1/line0"] = True
    shared = _shared(daq_device="Dev1")
    assert asyncio.run(shared.daq_read_analog("ai0", samples=10)) == 0.75
    assert asyncio.run(shared.daq_read_digital("port1/line0")) is True


def test_shared_devices_refuses_outputs_not_in_config(bench):
    shared = _shared(daq_device="Dev1", daq_ao_channels=["ao0"], daq_ao_max_v=5.0)
    with pytest.raises(DaqPolicyError):
        asyncio.run(shared.daq_write_analog("ao1", 1.0))
    with pytest.raises(DaqPolicyError):
        asyncio.run(shared.daq_write_analog("ao0", 6.0))
    with pytest.raises(DaqPolicyError):
        asyncio.run(shared.daq_write_digital("port0/line0", True))
    assert bench.outputs == {} and bench.open_tasks == 0    # nothing reached the device


def test_shared_devices_writes_and_cleanup_resets(bench):
    shared = _shared(daq_device="Dev1", daq_ao_channels=["ao0"], daq_do_lines=["port0/line0"])
    assert asyncio.run(shared.daq_write_analog("ao0", 2.5)) == 2.5
    assert asyncio.run(shared.daq_write_digital("port0/line0", True)) is True
    assert bench.outputs == {"Dev1/ao0": 2.5, "Dev1/port0/line0": True}
    asyncio.run(shared.cleanup())
    assert bench.outputs == {"Dev1/ao0": 0.0, "Dev1/port0/line0": False}
    assert bench.open_tasks == 0


# ----------------------------------------------------------------------
# Mock parity
# ----------------------------------------------------------------------
DAQ_METHODS = ("daq_read_analog", "daq_write_analog", "daq_read_digital", "daq_write_digital")


@pytest.mark.parametrize("name", DAQ_METHODS)
def test_mock_has_the_same_signatures(name):
    assert (inspect.signature(getattr(SharedDevices, name))
            == inspect.signature(getattr(MockSharedDevices, name)))


def test_mock_applies_the_same_output_policy():
    mock = MockSharedDevices(use_daq=True, daq_ao_channels=["ao0"], daq_do_lines=["port0/line0"])
    assert asyncio.run(mock.daq_read_analog("ai0")) == MockSharedDevices.MOCK_VOLTAGE
    assert asyncio.run(mock.daq_read_digital("port0/line5")) is False
    assert asyncio.run(mock.daq_write_analog("ao0", 2.0)) == 2.0
    assert asyncio.run(mock.daq_write_digital("port0/line0", True)) is True
    with pytest.raises(DaqPolicyError):
        asyncio.run(mock.daq_write_analog("ao0", 5.5))
    with pytest.raises(DaqPolicyError):
        asyncio.run(mock.daq_write_analog("ao1", 1.0))
    with pytest.raises(DaqPolicyError):
        asyncio.run(mock.daq_write_digital("port0/line1", True))
    with pytest.raises(RuntimeError, match="driven as an output"):
        asyncio.run(mock.daq_read_digital("port0/line0"))
    with pytest.raises(DaqPolicyError):
        asyncio.run(mock.daq_read_analog("ao0"))
    asyncio.run(mock.cleanup())
    assert mock.daq_analog_outputs == {"ao0": 0.0}
    assert mock.daq_digital_outputs == {"port0/line0": False}


def test_mock_without_daq_raises():
    with pytest.raises(RuntimeError, match="DAQ"):
        asyncio.run(MockSharedDevices().daq_read_analog("ai0"))


# ----------------------------------------------------------------------
# Schema
# ----------------------------------------------------------------------
def _wf(step):
    return ExperimentWorkflow(name="d", description="", steps=[step])


def test_schema_defaults():
    step = _wf({"action": "daq_read_analog", "channel": "ai0"}).steps[0]
    assert (step.samples, step.rate_hz, step.v_range, step.terminal) == (100, 1000.0, 10.0, "default")
    assert _wf({"action": "daq_read_analog", "channel": "ai15", "v_range": 5}).steps[0].v_range == 5


@pytest.mark.parametrize("step", [
    {"action": "daq_read_analog"},                                         # channel required
    {"action": "daq_read_analog", "channel": "Dev1/ai0"},
    {"action": "daq_read_analog", "channel": "ai0", "v_range": 3.0},
    {"action": "daq_read_analog", "channel": "ai0", "terminal": "floating"},
    {"action": "daq_read_analog", "channel": "ai0", "samples": 0},
    {"action": "daq_read_analog", "channel": "ai0", "samples": 100000, "rate_hz": 10.0},   # > 60 s
    {"action": "daq_read_analog", "channel": "ai0", "robot_id": 1},        # shared: no robot_id
    {"action": "daq_write_analog", "channel": "ao0"},                      # voltage required
    {"action": "daq_write_analog", "channel": "ao0", "voltage": 10.5},
    {"action": "daq_write_analog", "channel": "ai0", "voltage": 1.0},
    {"action": "daq_read_digital", "line": "port0"},
    {"action": "daq_write_digital", "line": "port0/line0"},                # state required
    {"action": "daq_write_digital", "line": "port0/line0:3", "state": True},
])
def test_schema_rejects(step):
    with pytest.raises(ValidationError):
        _wf(step)


# ----------------------------------------------------------------------
# Executor, logger, resource planning, CLI
# ----------------------------------------------------------------------
def test_executor_routes_daq_actions():
    assert DAQ_ACTIONS <= SHARED_DEVICE_ACTIONS
    mock = MockSharedDevices(use_daq=True, daq_ao_channels=["ao0"], daq_do_lines=["port0/line0"])

    def run(step):
        return asyncio.run(execute_step(step, {}, mock))

    assert run({"action": "daq_read_analog", "channel": "ai2"}) == {
        "voltage": MockSharedDevices.MOCK_VOLTAGE, "daq_channel": "ai2"}
    assert run({"action": "daq_write_analog", "channel": "ao0", "voltage": 1.5}) == {
        "voltage": 1.5, "daq_channel": "ao0"}
    assert run({"action": "daq_read_digital", "line": "port0/line1"}) == {
        "digital_state": False, "daq_channel": "port0/line1"}
    assert run({"action": "daq_write_digital", "line": "port0/line0", "state": True}) == {
        "digital_state": True, "daq_channel": "port0/line0"}
    # the pydantic model form reaches the executor too
    model = _wf({"action": "daq_write_analog", "channel": "ao0", "voltage": 0.0}).steps[0]
    assert run(model) == {"voltage": 0.0, "daq_channel": "ao0"}


def _rows(run_dir):
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def test_logger_writes_daq_columns(tmp_path):
    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    log = ExperimentLogger("d", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    log.record_step(1, 3, "daq_read_analog", None, None, "ok", 0.1,
                    result={"voltage": 1.2345678, "daq_channel": "ai0"})
    log.record_step(2, 3, "daq_read_digital", None, None, "ok", 0.1,
                    result={"digital_state": False, "daq_channel": "port0/line0"})
    log.record_step(3, 3, "measure_weight", None, None, "ok", 0.1, result={"weight": 1.0})
    log.finalize("completed")
    rows = _rows(log.dir)
    assert (rows[0]["daq_channel"], rows[0]["voltage_v"], rows[0]["digital_state"]) == ("ai0", "1.234568", "")
    assert (rows[1]["daq_channel"], rows[1]["voltage_v"], rows[1]["digital_state"]) == ("port0/line0", "", "0")
    assert (rows[2]["daq_channel"], rows[2]["voltage_v"], rows[2]["digital_state"]) == ("", "", "")
    with open(os.path.join(log.dir, "summary.md"), encoding="utf-8") as f:
        summary = f.read()
    assert "ai0 +1.23457 V" in summary and "port0/line0 Low" in summary


def test_plan_daq_and_no_robot_for_a_daq_only_flow():
    steps = [{"action": "daq_read_analog", "channel": "ai0"}]
    assert run_flow.plan_daq(steps) is True
    assert run_flow.plan_daq([{"action": "measure_weight"}]) is False
    plan = run_flow.plan_resources(steps)
    assert len(plan) == 6 and plan[0] == [] and not any(plan[2:])


def test_resolve_ports_daq_device(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.setenv("DAQ_DEVICE", "Dev7")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["daq_device"] == "Dev7"
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--daq-device", "Dev8"]))
    assert shared["daq_device"] == "Dev8"


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def test_example_flow_validates():
    assert run_flow.main([EXAMPLE_FLOW, "--validate-only"]) == 0


def test_example_flow_mock_run_records_the_readings(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE_FLOW, "--mock"]) == 0
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    analog = [r for r in rows if r["action"] == "daq_read_analog"]
    assert len(analog) == 5
    assert all(r["status"] == "ok" and r["voltage_v"] == "1.234000" and r["daq_channel"] == "ai0"
               for r in analog)
    digital = [r for r in rows if r["action"] == "daq_read_digital"]
    assert len(digital) == 1 and digital[0]["digital_state"] == "0"
    assert all(r["robot_id"] == "" for r in rows)           # no robot is involved


def _write_flow(tmp_path, steps):
    import json
    path = tmp_path / "out.json"
    path.write_text(json.dumps({"name": "daq_out", "description": "", "steps": steps}),
                    encoding="utf-8")
    return str(path)


OUTPUT_STEPS = [
    {"action": "daq_write_analog", "channel": "ao0", "voltage": 2.5},
    {"action": "daq_write_digital", "line": "port0/line0", "state": True},
]


def test_mock_run_refuses_outputs_the_config_does_not_enable(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    shared = dict(run_flow.lab_config.get_shared_devices(), daq_ao_channels=[], daq_do_lines=[])
    monkeypatch.setattr(run_flow.lab_config, "get_shared_devices", lambda: dict(shared))
    assert run_flow.main([_write_flow(tmp_path, OUTPUT_STEPS), "--mock"]) == 1
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert rows[0]["status"] == "error" and "daq_ao_channels" in rows[0]["error"]


def test_mock_run_drives_outputs_the_config_enables(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    shared = dict(run_flow.lab_config.get_shared_devices(), daq_ao_channels=["ao0"],
                  daq_do_lines=["port0/line0"], daq_ao_min_v=0.0, daq_ao_max_v=5.0)
    monkeypatch.setattr(run_flow.lab_config, "get_shared_devices", lambda: dict(shared))
    assert run_flow.main([_write_flow(tmp_path, OUTPUT_STEPS), "--mock"]) == 0
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert (rows[0]["daq_channel"], rows[0]["voltage_v"]) == ("ao0", "2.500000")
    assert (rows[1]["daq_channel"], rows[1]["digital_state"]) == ("port0/line0", "1")


# ----------------------------------------------------------------------
# Session wiring
# ----------------------------------------------------------------------
def test_session_passes_the_daq_config_to_the_mock():
    config = {"scale_port": "", "camera_index": 0, "daq_device": "Dev1",
              "daq_ao_channels": ["ao1"], "daq_ao_max_v": 3.0}

    async def go():
        session = ExperimentSession(mock=True, shared_config=config)
        shared = await session.add_shared(use_daq=True)
        return shared

    shared = asyncio.run(go())
    assert shared.use_daq is True
    assert shared.daq_policy.ao_channels == {"ao1"} and shared.daq_policy.ao_max_v == 3.0


def test_session_keeps_working_with_a_factory_that_does_not_know_the_daq():
    seen = {}

    def factory(*, use_scale, use_camera, config, use_microscope=False,
                use_microscope_serial=False):
        seen["called"] = True
        return MockSharedDevices(use_scale=use_scale)

    async def go():
        session = ExperimentSession(mock=True, shared_factory=factory,
                                    shared_config={"scale_port": "", "camera_index": 0})
        await session.add_shared(use_scale=True)

    asyncio.run(go())
    assert seen == {"called": True}


def test_real_cleanup_through_the_session_resets_outputs(bench):
    config = {"scale_port": "", "camera_index": 0, "daq_device": "Dev1",
              "daq_do_lines": ["port0/line0"]}

    async def go():
        session = ExperimentSession(mock=False, shared_config=config)

        async def body():
            shared = await session.add_shared(use_daq=True)
            await shared.daq_write_digital("port0/line0", True)
            raise RuntimeError("step failed")

        await session.run(body)

    with pytest.raises(RuntimeError, match="step failed"):
        asyncio.run(go())
    assert bench.outputs == {"Dev1/port0/line0": False}     # low again although the run failed
    assert bench.open_tasks == 0
