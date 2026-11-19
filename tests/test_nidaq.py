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

from src.devices.nidaq import (DAQ_ACTION_NAMES, DaqOutputPolicy, DaqPolicyError, DaqService,
                               DaqSignals, MockNIDaq, NIDaq)
from src.devices.nidaq.service import waveform_period
from src.devices.nidaq import policy as daq_policy
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import (DAQ_ACTIONS, DAQ_DATA_ACTIONS, SHARED_DEVICE_ACTIONS,
                               execute_step)
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
        self.events = []          # ("write", phys, value) / ("close", phys) / ("start"|"stop", phys)
        self.ai_configs = []      # kwargs of add_ai_voltage_chan
        self.timings = []
        self.fail_write = set()   # physical channels whose write raises
        self.open_tasks = 0
        self.triggers = []        # (source, edge) of cfg_dig_edge_start_trig
        self.stream_chunks = []   # what successive READ_ALL_AVAILABLE reads return (per channel)
        self.counters = []        # {"kind": "count"|"freq"|"pulse", "phys":.., ...}
        self.counter_value = 0    # what a counter-input task reads
        self.read_error = None    # raised by the next read()
        self.pulses_done = True
        self.coerced_rate = None  # the sample clock the device reports (None = as asked)
        self.fail_start = set()   # physical channels whose task.start() raises
        self.clock_sources = []   # source of each cfg_samp_clk_timing (None = onboard)
        self.ai_chans = {}        # physical channel -> the channel object handed out
        self.change_detection = []  # (rising, falling, mode) of cfg_change_detection_timing
        self.product_type = "USB-6351"  # the model the fake device reports


class FakeDaqError(Exception):
    def __init__(self, code):
        super().__init__(f"DAQmx error {code}")
        self.error_code = code


def install_fake_nidaqmx(monkeypatch, bench):
    class _Channels:
        def __init__(self, task, kind):
            self.task, self.kind = task, kind

        def _add(self, phys, **kwargs):
            self.task.kind, self.task.phys = self.kind, phys
            self.task.chans.append(phys)
            chan = types.SimpleNamespace()
            if self.kind == "ai":
                bench.ai_configs.append({"phys": phys, **kwargs})
                bench.ai_chans[phys] = chan
            if self.kind.startswith(("ci", "co")):
                self.task.counter = {"kind": self.kind, "phys": phys, "chan": chan, **kwargs}
                bench.counters.append(self.task.counter)
            return chan

        add_ai_voltage_chan = add_ao_voltage_chan = add_do_chan = add_di_chan = _add
        add_ci_count_edges_chan = add_ci_freq_chan = add_co_pulse_chan_freq = _add

        def add_ai_thrmcpl_chan(self, phys, **kwargs):
            return self._add(phys, made="thermocouple", **kwargs)

        def add_ai_rtd_chan(self, phys, **kwargs):
            return self._add(phys, made="rtd", **kwargs)

    class _Timing:
        def __init__(self, task):
            self.task = task

        def cfg_samp_clk_timing(self, rate, source=None, sample_mode=None, samps_per_chan=None):
            bench.timings.append((rate, sample_mode, samps_per_chan))
            bench.clock_sources.append(source)
            self.samp_clk_rate = bench.coerced_rate or rate

        def cfg_change_detection_timing(self, rising_edge_chan="", falling_edge_chan="",
                                        sample_mode=None, samps_per_chan=None):
            bench.change_detection.append((rising_edge_chan, falling_edge_chan, sample_mode))

        def cfg_implicit_timing(self, sample_mode=None, samps_per_chan=None):
            self.task.counter["mode"] = sample_mode
            self.task.counter["count"] = samps_per_chan

    class _StartTrigger:
        def cfg_dig_edge_start_trig(self, trigger_source, trigger_edge=None):
            bench.triggers.append((trigger_source, trigger_edge))

        def cfg_anlg_edge_start_trig(self, trigger_source, trigger_slope=None, trigger_level=0.0):
            bench.triggers.append(("anlg_start", trigger_source, trigger_slope, trigger_level))

    class _ReferenceTrigger:
        def cfg_dig_edge_ref_trig(self, trigger_source, pretrigger_samples, trigger_edge=None):
            bench.triggers.append(("dig_ref", trigger_source, trigger_edge, pretrigger_samples))

        def cfg_anlg_edge_ref_trig(self, trigger_source, pretrigger_samples, trigger_slope=None,
                                   trigger_level=0.0):
            bench.triggers.append(("anlg_ref", trigger_source, trigger_slope, trigger_level,
                                   pretrigger_samples))

    class Task:
        def __init__(self):
            bench.open_tasks += 1
            self.kind = self.phys = self.counter = None
            self.chans = []
            self.ai_channels = _Channels(self, "ai")
            self.ao_channels = _Channels(self, "ao")
            self.do_channels = _Channels(self, "do")
            self.di_channels = _Channels(self, "di")
            self.ci_channels = _Channels(self, "ci_count")
            self.co_channels = _Channels(self, "co")
            self.timing = _Timing(self)
            self.triggers = types.SimpleNamespace(start_trigger=_StartTrigger(),
                                                  reference_trigger=_ReferenceTrigger())
            # the counter-input kinds must be told apart from add_ci_count_edges_chan
            self.ci_channels.add_ci_freq_chan = _Channels(self, "ci_freq")._add
            self.ci_channels.add_ci_pulse_width_chan = _Channels(self, "ci_width")._add
            self.ci_channels.add_ci_period_chan = _Channels(self, "ci_period")._add
            self.ci_channels.add_ci_two_edge_sep_chan = _Channels(self, "ci_sep")._add
            self.ci_channels.add_ci_ang_encoder_chan = _Channels(self, "ci_encoder")._add

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()
            return False

        def start(self):
            if self.phys in bench.fail_start:
                raise RuntimeError("start failed")
            bench.events.append(("start", self.phys))

        def stop(self):
            bench.events.append(("stop", self.phys))

        def is_task_done(self):
            return bench.pulses_done

        def wait_until_done(self, timeout=10.0):
            pass

        def read(self, number_of_samples_per_channel=None, timeout=10.0):
            if bench.read_error is not None:
                error, bench.read_error = bench.read_error, None
                raise error
            if self.kind.startswith("ci"):
                return bench.counter_value
            if number_of_samples_per_channel == -1:              # READ_ALL_AVAILABLE
                chunk = bench.stream_chunks.pop(0) if bench.stream_chunks else [[] for _ in self.chans]
                return chunk[0] if len(self.chans) == 1 else chunk
            if len(self.chans) > 1:
                return [self._column(phys, number_of_samples_per_channel) for phys in self.chans]
            value = bench.inputs.get(self.phys, 0.0)
            if number_of_samples_per_channel is None:
                return value
            return self._column(self.phys, number_of_samples_per_channel)

        @staticmethod
        def _column(phys, n):
            value = bench.inputs.get(phys, 0.0)
            return list(value) if isinstance(value, list) else [value] * n

        def write(self, data, auto_start=None):
            if self.phys in bench.fail_write:
                raise RuntimeError("write failed")
            assert auto_start is (False if isinstance(data, list) else True)
            bench.outputs[self.phys] = data
            bench.events.append(("write", self.phys, data))

        def close(self):
            bench.open_tasks -= 1
            bench.events.append(("close", self.phys))

    class _Device:
        def __init__(self, name):
            self.name, self.product_type = name, bench.product_type

    class _Devices(list):
        pass

    class _System:
        @staticmethod
        def local():
            return types.SimpleNamespace(devices=_Devices(_Device(n) for n in bench.devices))

    constants = types.ModuleType("nidaqmx.constants")
    constants.TerminalConfiguration = types.SimpleNamespace(
        DEFAULT="T_DEFAULT", RSE="T_RSE", NRSE="T_NRSE", DIFF="T_DIFF")
    constants.AcquisitionType = types.SimpleNamespace(FINITE="FINITE", CONTINUOUS="CONTINUOUS")
    constants.LineGrouping = types.SimpleNamespace(CHAN_PER_LINE="CHAN_PER_LINE")
    constants.Edge = types.SimpleNamespace(RISING="RISING", FALLING="FALLING")
    constants.Level = types.SimpleNamespace(LOW="LOW", HIGH="HIGH")
    constants.Slope = types.SimpleNamespace(RISING="RISING", FALLING="FALLING")
    constants.TemperatureUnits = types.SimpleNamespace(DEG_C="DEG_C")
    constants.ThermocoupleType = types.SimpleNamespace(
        **{t: f"TC_{t}" for t in "BEJKNRST"})
    constants.CJCSource = types.SimpleNamespace(CONSTANT_USER_VALUE="CJC_CONST")
    constants.RTDType = types.SimpleNamespace(
        **{t: t for t in ("PT_3750", "PT_3851", "PT_3911", "PT_3916", "PT_3920", "PT_3928")})
    constants.ResistanceConfiguration = types.SimpleNamespace(
        TWO_WIRE="TWO_WIRE", THREE_WIRE="THREE_WIRE", FOUR_WIRE="FOUR_WIRE")
    constants.ExcitationSource = types.SimpleNamespace(EXTERNAL="EXTERNAL")
    constants.EncoderType = types.SimpleNamespace(X_1="X_1", X_2="X_2", X_4="X_4")
    constants.AngleUnits = types.SimpleNamespace(DEGREES="DEGREES")
    constants.READ_ALL_AVAILABLE = -1
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
    # v_range / terminal are left to the signal's wiring (config.yaml) unless the step sets them
    assert (step.samples, step.rate_hz, step.v_range, step.terminal) == (100, 1000.0, None, None)
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
    assert plan.robot_ids == [] and plan.needs_daq is True and plan.needs_shared is True
    assert plan.shared_kwargs()["use_daq"] is True and "DAQ" in plan.shared_labels()
    assert not (plan.needs_scale or plan.needs_camera or plan.needs_dmm or plan.needs_scope)
    other = run_flow.plan_resources([{"action": "measure_weight"}])
    assert other.needs_daq is False and "use_daq" not in other.shared_kwargs()


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


# ======================================================================
# Extended functions: named signals, waveforms, logging, counters
# ======================================================================
SIGNALS = {
    "photodiode": {"channel": "ai2", "terminal": "diff", "v_range": 1, "scale": 2.0,
                   "offset": 0.5, "unit": "mW"},
    "lamp": {"channel": "ao0"},
    "valve": "port0/line3",                     # shorthand: just the channel
    "flow_meter": {"channel": "pfi4"},
    "clock": {"channel": "ctr1"},
}
ENABLED = {"daq_signals": SIGNALS, "daq_ao_channels": ["ao0"], "daq_do_lines": ["port0/line3"],
           "daq_co_counters": ["ctr1"], "daq_ao_min_v": 0.0, "daq_ao_max_v": 5.0,
           "daq_co_max_hz": 500.0}


# ----------------------------------------------------------------------
# Signals and the extended policy
# ----------------------------------------------------------------------
def test_signals_resolve_names_and_physical_channels():
    signals = DaqSignals.from_config({"daq_signals": SIGNALS})
    pd = signals.resolve("Photodiode", "ai")
    assert (pd.label, pd.channel, pd.terminal, pd.v_range, pd.unit) == (
        "photodiode:ai2", "ai2", "diff", 1.0, "mW")
    assert pd.scaled and pd.value(1.0) == 2.5
    raw = signals.resolve("ai7", "ai")
    assert (raw.label, raw.channel, raw.scaled, raw.terminal, raw.v_range) == (
        "ai7", "ai7", False, None, None)
    assert signals.resolve("valve", "line").channel == "port0/line3"
    assert signals.resolve("clock", ("ao", "line", "counter")).kind == "counter"
    with pytest.raises(DaqPolicyError, match="is a analog input|analog input; this step needs"):
        signals.resolve("photodiode", "ao")
    with pytest.raises(DaqPolicyError, match="this step needs analog output"):
        signals.resolve("ai0", "ao")
    with pytest.raises(DaqPolicyError, match="daq_signals"):
        signals.resolve("pump", "line")


@pytest.mark.parametrize("signals", [
    {"ai0": {"channel": "ai1"}},                        # a name shaped like a channel
    {"Bad Name": {"channel": "ai0"}},
    {"x": {"channel": "Dev1/ai0"}},
    {"x": {}},
    {"x": {"channel": "ai0", "gain": 2}},               # unknown key
    {"x": {"channel": "ao0", "scale": 2.0}},            # conversion only on analog inputs
    {"x": {"channel": "ai0", "scale": 0}},
    {"x": {"channel": "ai0", "v_range": 3}},
    {"x": {"channel": "ai0", "terminal": "floating"}},
    ["ai0"],
])
def test_signals_reject_bad_configuration(signals):
    with pytest.raises(DaqPolicyError):
        DaqSignals.from_config({"daq_signals": signals})


def test_check_flow_name_without_configuration():
    assert daq_policy.check_flow_name("AI3", "ai") == "ai3"
    assert daq_policy.check_flow_name("photodiode", "ai") == "photodiode"
    assert daq_policy.check_flow_name("ctr0", ("ao", "line", "counter")) == "ctr0"
    for bad, kinds in (("ao0", "ai"), ("Dev1/ai0", "ai"), ("port0", "line"), ("ai99x", "ai"),
                       ("my signal", "ai"), ("pfi0", "counter"), ("", "ai")):
        with pytest.raises(ValueError):
            daq_policy.check_flow_name(bad, kinds)


def test_policy_checks_both_ends_of_a_waveform_and_counter_outputs():
    policy = DaqOutputPolicy.from_config(ENABLED)
    assert policy.check_analog_waveform("ao0", 2.5, 2.5) == ("ao0", 2.5, 2.5)
    for offset, amplitude in ((2.5, 2.6), (0.5, 1.0), (4.5, 1.0), (1.0, -0.1)):
        with pytest.raises(DaqPolicyError):
            policy.check_analog_waveform("ao0", offset, amplitude)
    with pytest.raises(DaqPolicyError, match="not enabled"):
        policy.check_analog_waveform("ao1", 1.0, 0.5)
    assert policy.check_counter_output("CTR1", 500, 0.25) == ("ctr1", 500.0, 0.25)
    with pytest.raises(DaqPolicyError, match="daq_co_counters"):
        policy.check_counter_output("ctr0", 10, 0.5)
    with pytest.raises(DaqPolicyError, match="daq_co_max_hz"):
        policy.check_counter_output("ctr1", 501, 0.5)
    for duty in (0.0, 1.0, float("nan")):
        with pytest.raises(DaqPolicyError, match="duty_cycle"):
            policy.check_counter_output("ctr1", 10, duty)
    for config in ({"daq_co_max_hz": None}, {"daq_co_max_hz": 0}, {"daq_co_max_hz": 2e6},
                   {"daq_co_counters": ["pfi12"]}):
        with pytest.raises(DaqPolicyError):
            DaqOutputPolicy.from_config(config)
    # nothing is enabled by default
    with pytest.raises(DaqPolicyError):
        DaqOutputPolicy.from_config({}).check_counter_output("ctr0", 1, 0.5)


# ----------------------------------------------------------------------
# Driver: waveform acquisition, streaming
# ----------------------------------------------------------------------
def test_acquire_reads_all_channels_on_one_clock(bench):
    bench.inputs["Dev1/ai0"] = [1.0, 2.0, 3.0]
    bench.inputs["Dev1/ai5"] = [4.0, 5.0, 6.0]
    daq = NIDaq(device="Dev1")
    data = daq.acquire([("ai0", 10.0, "rse"), ("ai5", 1.0, "diff")], rate_hz=5000.0, samples=3)
    assert data == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert bench.timings == [(5000.0, "FINITE", 3)]
    assert [(c["phys"], c["terminal_config"], c["max_val"]) for c in bench.ai_configs] == [
        ("Dev1/ai0", "T_RSE", 10.0), ("Dev1/ai5", "T_DIFF", 1.0)]
    assert bench.triggers == [] and bench.open_tasks == 0
    assert daq.acquire([("ai0", 10.0, "rse")], 1000.0, 3) == [[1.0, 2.0, 3.0]]


def test_acquire_with_a_start_trigger_and_its_timeout(bench):
    daq = NIDaq(device="Dev1")
    daq.acquire([("ai0", 10.0, "default")], 1000.0, 4, trigger="pfi3", trigger_edge="falling")
    assert bench.triggers == [("/Dev1/PFI3", "FALLING")]
    bench.read_error = FakeDaqError(-200284)
    with pytest.raises(TimeoutError, match="no rising edge on pfi3"):
        daq.acquire([("ai0", 10.0, "default")], 1000.0, 4, trigger="pfi3", timeout_s=0.1)
    bench.read_error = FakeDaqError(-50103)                 # any other error is passed on
    with pytest.raises(FakeDaqError):
        daq.acquire([("ai0", 10.0, "default")], 1000.0, 4, trigger="pfi3")
    assert bench.open_tasks == 0


@pytest.mark.parametrize("specs,rate,samples", [
    ([("ai0", 10.0, "rse")], 1000.0, 1),                    # at least 2 samples
    ([("ai0", 10.0, "rse")], 1_300_000.0, 10),              # single-channel limit
    ([("ai0", 10.0, "rse"), ("ai1", 10.0, "rse")], 600_000.0, 10),   # aggregate limit
    ([("ai0", 10.0, "rse"), ("ai0", 10.0, "rse")], 1000.0, 10),      # listed twice
    ([("ai0", 20.0, "rse")], 1000.0, 10),
    ([("ai0", 10.0, "floating")], 1000.0, 10),
    ([("ao0", 10.0, "rse")], 1000.0, 10),
    ([], 1000.0, 10),
])
def test_acquire_rejects_what_the_device_cannot_do(bench, specs, rate, samples):
    with pytest.raises(ValueError):
        NIDaq(device="Dev1").acquire(specs, rate, samples)
    assert bench.open_tasks == 0


def test_stream_blocks_other_analog_input_until_stopped(bench):
    daq = NIDaq(device="Dev1")
    daq.start_stream([("ai0", 10.0, "rse"), ("ai1", 10.0, "rse")], 200.0)
    assert bench.timings == [(200.0, "CONTINUOUS", 2000)] and bench.open_tasks == 1
    bench.stream_chunks = [[[1.0, 2.0], [3.0, 4.0]]]
    assert daq.read_stream() == [[1.0, 2.0], [3.0, 4.0]]
    assert daq.read_stream() == [[], []]
    for call in (lambda: daq.read_analog("ai2"), lambda: daq.acquire([("ai2", 10.0, "rse")], 100.0, 2),
                 lambda: daq.start_stream([("ai2", 10.0, "rse")], 100.0)):
        with pytest.raises(RuntimeError, match="busy with a continuous acquisition"):
            call()
    daq.stop_stream()
    assert bench.open_tasks == 0
    with pytest.raises(RuntimeError, match="no continuous acquisition"):
        daq.read_stream()
    daq.start_stream([("ai3", 10.0, "rse")], 100.0)         # one channel: a flat list comes back
    bench.stream_chunks = [[[7.0, 8.0]]]
    assert daq.read_stream() == [[7.0, 8.0]]
    assert daq.close() == [] and bench.open_tasks == 0      # close() stops a running stream


# ----------------------------------------------------------------------
# Driver: waveform output, counters
# ----------------------------------------------------------------------
def test_waveform_output_is_replaced_by_dc_and_reset_on_close(bench):
    daq = NIDaq(device="Dev1")
    daq.write_analog("ao0", 1.0)
    daq.start_waveform("ao0", [0.0, 1.0, 2.0, 1.0], rate_hz=400.0)
    assert bench.outputs["Dev1/ao0"] == [0.0, 1.0, 2.0, 1.0]
    assert bench.timings[-1] == (400.0, "CONTINUOUS", 4)
    assert bench.open_tasks == 1                            # the DC task was released first
    assert daq.stop_output("ao0") is True
    assert bench.outputs["Dev1/ao0"] == 0.0 and bench.open_tasks == 1
    daq.start_waveform("ao1", [1.0, 2.0], rate_hz=100.0)
    assert daq.close() == []
    assert bench.outputs == {"Dev1/ao0": 0.0, "Dev1/ao1": 0.0} and bench.open_tasks == 0
    for period, rate in (([1.0], 100.0), ([0.0, 10.5], 100.0), ([0.0, 1.0], 3e6), ([0.0, 1.0], 0)):
        with pytest.raises(ValueError):
            daq.start_waveform("ao0", period, rate)
    assert bench.open_tasks == 0


def test_count_edges_and_frequency_use_a_free_counter(bench):
    bench.counter_value = 1234
    daq = NIDaq(device="Dev1")
    slept = []
    assert daq.count_edges("pfi2", 0.5, edge="falling", sleep=slept.append) == 1234
    counter = bench.counters[-1]
    assert (counter["kind"], counter["phys"], counter["edge"]) == ("ci_count", "Dev1/ctr3", "FALLING")
    assert counter["chan"].ci_count_edges_term == "/Dev1/PFI2" and slept == [0.5]
    bench.counter_value = 99.5
    assert daq.measure_frequency("100khztimebase", min_hz=10, max_hz=1e6, counter="ctr0") == 99.5
    counter = bench.counters[-1]
    assert (counter["kind"], counter["phys"], counter["min_val"], counter["max_val"]) == (
        "ci_freq", "Dev1/ctr0", 10.0, 1e6)
    assert counter["chan"].ci_freq_term == "/Dev1/100kHzTimebase"
    bench.read_error = FakeDaqError(-200474)
    with pytest.raises(TimeoutError, match="no signal on pfi0"):
        daq.measure_frequency("pfi0", timeout_s=0.1)
    for call in (lambda: daq.count_edges("pfi0", 0), lambda: daq.count_edges("ai0", 1.0),
                 lambda: daq.count_edges("pfi0", 1.0, edge="both"),
                 lambda: daq.measure_frequency("pfi0", min_hz=10, max_hz=5)):
        with pytest.raises(ValueError):
            call()
    assert bench.open_tasks == 0


def test_pulse_trains_and_the_counters_they_occupy(bench):
    daq = NIDaq(device="Dev1")
    daq.start_pulses("ctr3", 100.0, duty_cycle=0.25)
    counter = bench.counters[-1]
    assert (counter["kind"], counter["phys"], counter["freq"], counter["duty_cycle"],
            counter["idle_state"], counter["mode"]) == ("co", "Dev1/ctr3", 100.0, 0.25, "LOW",
                                                        "CONTINUOUS")
    daq.start_pulses("ctr0", 10.0, count=5)
    assert (bench.counters[-1]["mode"], bench.counters[-1]["count"]) == ("FINITE", 5)
    bench.pulses_done = False
    assert daq.pulses_done("ctr0") is False and daq.pulses_done("ctr1") is True
    # a measurement takes the highest counter that is not generating
    bench.counter_value = 7
    assert daq.count_edges("pfi0", 0.1, sleep=lambda s: None) == 7
    assert bench.counters[-1]["phys"] == "Dev1/ctr2"
    with pytest.raises(RuntimeError, match="generating a pulse train"):
        daq.count_edges("pfi0", 0.1, counter="ctr3", sleep=lambda s: None)
    assert daq.stop_output("ctr3") is True and daq.stop_output("ctr3") is False
    assert bench.open_tasks == 1
    assert daq.close() == [] and bench.open_tasks == 0      # the other train is stopped too
    for kwargs in ({"frequency_hz": 0}, {"frequency_hz": 10, "duty_cycle": 1.0},
                   {"frequency_hz": 10, "count": 0}):
        with pytest.raises(ValueError):
            daq.start_pulses("ctr0", **kwargs)
    with pytest.raises(ValueError):
        daq.stop_output("ai0")
    assert daq.stop_output("ao0") is False and daq.stop_output("port0/line0") is False
    assert bench.open_tasks == 0                            # an undriven output is not touched


def test_mock_device_has_the_methods_of_the_real_one():
    public = [n for n, v in vars(NIDaq).items() if callable(v) and not n.startswith("_")]
    assert len(public) > 15
    for name in public:
        real = inspect.signature(getattr(NIDaq, name))
        mock = inspect.signature(getattr(MockNIDaq, name))
        assert list(real.parameters) == list(mock.parameters), name


# ----------------------------------------------------------------------
# DaqService on the mock device
# ----------------------------------------------------------------------
def _service(config=None, mock=True):
    backend = MockNIDaq()
    return DaqService(backend, ENABLED if config is None else config, mock=mock), backend


def _go(service, action, **params):
    return asyncio.run(service.run(action, **params))


def _csv(path):
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.reader(f))


def test_service_reads_a_named_signal_with_its_wiring_and_conversion(monkeypatch):
    service, backend = _service()
    calls = []
    monkeypatch.setattr(backend, "read_analog", lambda *a: calls.append(a) or 1.0)
    assert _go(service, "daq_read_analog", channel="photodiode") == {
        "voltage": 1.0, "daq_channel": "photodiode:ai2", "daq_value": 2.5, "daq_unit": "mW"}
    assert calls == [("ai2", 100, 1000.0, 1.0, "diff")]     # the signal's range and terminal
    _go(service, "daq_read_analog", channel="photodiode", v_range=5.0, terminal="rse", samples=3)
    assert calls[-1] == ("ai2", 3, 1000.0, 5.0, "rse")      # the step's own values win
    assert _go(service, "daq_read_analog", channel="ai0") == {"voltage": 1.0, "daq_channel": "ai0"}
    assert calls[-1] == ("ai0", 100, 1000.0, 10.0, "default")
    with pytest.raises(DaqPolicyError):
        _go(service, "daq_read_analog", channel="lamp")     # an output is not an input
    with pytest.raises(ValueError, match="unknown DAQ action"):
        _go(service, "daq_reset_device")


def test_service_acquire_writes_a_csv(tmp_path):
    service, _ = _service()
    path = str(tmp_path / "sub" / "wave.csv")
    result = _go(service, "daq_acquire", channels=["photodiode", "ai0"], rate_hz=100.0,
                 duration_s=0.05, file_path=path)
    assert result == {"daq_channel": "photodiode:ai2+ai0", "data_path": path}
    rows = _csv(path)
    assert rows[0] == ["time_s", "photodiode:ai2_V", "photodiode:ai2_mW", "ai0_V"]
    assert len(rows) == 6 and rows[1] == ["0.000000", "1.234000", "2.968", "1.234000"]
    assert rows[-1][0] == "0.040000"
    single = _go(service, "daq_acquire", channels=["photodiode"], rate_hz=100.0, duration_s=0.05,
                 file_path=str(tmp_path / "one.csv"), trigger="flow_meter")
    assert single["voltage"] == pytest.approx(1.234) and single["daq_unit"] == "mW"
    assert single["daq_value"] == pytest.approx(2.968)
    for kwargs in ({"channels": []}, {"channels": ["ai0", "AI0"]},
                   {"channels": ["ai0"], "rate_hz": 10.0, "duration_s": 0.01},
                   {"channels": ["ai0"], "rate_hz": 1e6, "duration_s": 3.0},
                   {"channels": ["ai0"], "trigger_edge": "both"},
                   {"channels": ["ai0"], "trigger": "valve"}):
        with pytest.raises(ValueError):
            _go(service, "daq_acquire", file_path=str(tmp_path / "x.csv"), **kwargs)


def test_service_logging_runs_in_the_background_and_blocks_other_input(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "log.csv")

    async def scenario():
        started = await service.run("daq_start_logging", channels=["photodiode"], rate_hz=1000.0,
                                    file_path=path)
        with pytest.raises(RuntimeError, match="already running"):
            await service.run("daq_start_logging", channels=["ai0"], file_path=path)
        with pytest.raises(RuntimeError, match="busy with a continuous acquisition"):
            await service.run("daq_read_analog", channel="ai0")
        await service.run("daq_count_edges", terminal="pfi0", duration_s=0.5)   # counters are free
        await asyncio.sleep(0.3)
        return started, await service.run("daq_stop_logging")

    started, stopped = asyncio.run(scenario())
    assert started == {"daq_channel": "photodiode:ai2", "data_path": path}
    assert stopped["data_path"] == path and stopped["daq_unit"] == "samples"
    rows = _csv(path)
    assert rows[0] == ["time_s", "photodiode:ai2_V", "photodiode:ai2_mW"]
    assert stopped["daq_value"] == len(rows) - 1 >= 100
    assert rows[1][0] == "0.000000" and rows[2][0] == "0.001000"
    assert backend._stream is None
    with pytest.raises(RuntimeError, match="no logging session"):
        _go(service, "daq_stop_logging")
    assert _go(service, "daq_read_analog", channel="ai0")["voltage"] == 1.234   # free again


def test_service_close_stops_a_recording_nobody_stopped(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "log.csv")
    _go(service, "daq_start_logging", channels=["ai0"], rate_hz=100.0, file_path=path)
    assert service.close() == []
    assert backend._stream is None and len(_csv(path)) >= 2     # header and at least one sample


def test_service_reports_a_recording_that_broke(tmp_path, monkeypatch):
    service, backend = _service()

    def broken(stream=None):
        raise RuntimeError("device unplugged")

    async def scenario():
        await service.run("daq_start_logging", channels=["ai0"], file_path=str(tmp_path / "l.csv"))
        monkeypatch.setattr(backend, "read_stream", broken)
        await asyncio.sleep(0.5)
        await service.run("daq_stop_logging")

    with pytest.raises(RuntimeError, match="device unplugged"):
        asyncio.run(scenario())


def test_service_waveform_for_a_duration_returns_to_zero():
    service, backend = _service()
    seen = {}
    original = backend.stop_output

    def spy(name):
        seen["while_running"] = dict(backend.waveforms)
        return original(name)

    backend.stop_output = spy
    result = _go(service, "daq_generate_waveform", channel="lamp", waveform="square",
                 frequency_hz=50.0, amplitude_v=1.0, offset_v=2.0, duration_s=5.0)
    assert result == {"daq_channel": "lamp:ao0", "daq_value": 50.0, "daq_unit": "Hz"}
    wave = seen["while_running"]["ao0"]
    assert wave["rate_hz"] == 50.0 * 1000 and len(wave["points"]) == 1000
    assert (min(wave["points"]), max(wave["points"])) == (1.0, 3.0)
    assert backend.waveforms == {} and backend.analog_outputs == {"ao0": 0.0}


def test_service_waveform_without_a_duration_continues_until_stopped():
    service, backend = _service()
    _go(service, "daq_generate_waveform", channel="ao0", frequency_hz=20000.0, amplitude_v=2.5,
        offset_v=2.5)
    wave = backend.waveforms["ao0"]
    assert len(wave["points"]) == 100 and wave["rate_hz"] == 2_000_000.0   # fewer points when fast
    assert min(wave["points"]) >= 0.0 and max(wave["points"]) <= 5.0
    assert _go(service, "daq_stop_output", channel="lamp") == {"daq_channel": "lamp:ao0",
                                                              "voltage": 0.0}
    assert backend.waveforms == {} and backend.analog_outputs["ao0"] == 0.0
    _go(service, "daq_generate_waveform", channel="ao0", frequency_hz=1.0, amplitude_v=1.0,
        offset_v=1.0)
    assert service.close() == [] and backend.waveforms == {}


@pytest.mark.parametrize("params", [
    {"channel": "ao1", "frequency_hz": 10.0, "amplitude_v": 1.0, "offset_v": 1.0},   # not enabled
    {"channel": "ao0", "frequency_hz": 10.0, "amplitude_v": 1.0, "offset_v": 0.5},   # dips below 0 V
    {"channel": "ao0", "frequency_hz": 10.0, "amplitude_v": 3.0, "offset_v": 2.5},   # above 5 V
    {"channel": "valve", "frequency_hz": 10.0, "amplitude_v": 1.0, "offset_v": 1.0},  # not an AO
])
def test_service_refuses_waveforms_the_config_does_not_allow(params):
    service, backend = _service()
    with pytest.raises(DaqPolicyError):
        _go(service, "daq_generate_waveform", **params)
    assert backend.waveforms == {} and backend.analog_outputs == {}


def test_waveform_shapes():
    for name in ("sine", "square", "triangle", "sawtooth"):
        points = waveform_period(name, 2.0, 1.0, 1000)
        assert len(points) == 1000 and min(points) >= -1.0 - 1e-9 and max(points) <= 3.0 + 1e-9
        assert max(points) - min(points) > 3.9
    assert waveform_period("sine", 2.0, 1.0, 4) == pytest.approx([1.0, 3.0, 1.0, -1.0])
    assert waveform_period("triangle", 1.0, 0.0, 4) == pytest.approx([0.0, 1.0, 0.0, -1.0])
    with pytest.raises(ValueError):
        waveform_period("noise", 1.0, 0.0, 10)
    with pytest.raises(ValueError, match="too high"):
        _go(_service()[0], "daq_generate_waveform", channel="ao0", frequency_hz=150000.0,
            amplitude_v=1.0, offset_v=1.0)


def test_service_digital_pulse_and_waits():
    service, backend = _service()
    states = []
    original = backend.write_digital
    backend.write_digital = lambda name, state: states.append((name, state)) or original(name, state)
    assert _go(service, "daq_pulse_digital", line="valve", duration_s=3.0) == {
        "digital_state": False, "daq_channel": "valve:port0/line3"}
    assert states == [("port0/line3", True), ("port0/line3", False)]
    _go(service, "daq_pulse_digital", line="port0/line3", duration_s=0.1, state=False)
    assert states[2:] == [("port0/line3", False), ("port0/line3", True)]
    with pytest.raises(DaqPolicyError):
        _go(service, "daq_pulse_digital", line="port0/line0", duration_s=0.1)
    with pytest.raises(ValueError):
        _go(service, "daq_pulse_digital", line="valve", duration_s=0)
    # in a mock run a wait is taken as met (the mock line reads low)
    assert _go(service, "daq_wait_digital", line="port0/line1", state=True, timeout_s=0.1) == {
        "digital_state": True, "daq_channel": "port0/line1"}
    assert _go(service, "daq_wait_analog", channel="ai0", condition="below", threshold=0.0,
               timeout_s=0.1)["voltage"] == 1.234


def test_service_waits_time_out_when_the_condition_never_comes():
    service, backend = _service(mock=False)                 # real timing, simulated device
    assert _go(service, "daq_wait_digital", line="port0/line1", state=False)["digital_state"] is False
    with pytest.raises(TimeoutError, match="did not go high within 0.1 s"):
        _go(service, "daq_wait_digital", line="port0/line1", state=True, timeout_s=0.1)
    met = _go(service, "daq_wait_analog", channel="photodiode", condition="above", threshold=2.9)
    assert met["daq_value"] == pytest.approx(2.968)         # compared in the signal's unit
    with pytest.raises(TimeoutError, match="below 2 mW"):
        _go(service, "daq_wait_analog", channel="photodiode", condition="below", threshold=2.0,
            timeout_s=0.1)
    with pytest.raises(ValueError):
        _go(service, "daq_wait_analog", channel="ai0", condition="equal", threshold=1.0)
    readings = iter([False, False, True])
    backend.read_digital = lambda name: next(readings)
    assert _go(service, "daq_wait_digital", line="port0/line1", timeout_s=5.0)["digital_state"] is True


def test_service_counters():
    service, backend = _service()
    assert _go(service, "daq_count_edges", terminal="flow_meter", duration_s=2.0) == {
        "daq_channel": "flow_meter:pfi4", "daq_value": 2000, "daq_unit": "count"}
    assert _go(service, "daq_measure_frequency", terminal="pfi0", counter="clock") == {
        "daq_channel": "pfi0", "daq_value": 1000.0, "daq_unit": "Hz"}
    with pytest.raises(DaqPolicyError):
        _go(service, "daq_count_edges", terminal="valve")
    # finite and timed trains stop themselves; a continuous one runs until stopped
    assert _go(service, "daq_generate_pulses", counter="clock", frequency_hz=200.0, count=50) == {
        "daq_channel": "clock:ctr1", "daq_value": 200.0, "daq_unit": "Hz"}
    assert backend.pulse_trains == {}
    _go(service, "daq_generate_pulses", counter="ctr1", frequency_hz=10.0, duration_s=2.0)
    assert backend.pulse_trains == {}
    _go(service, "daq_generate_pulses", counter="ctr1", frequency_hz=10.0, duty_cycle=0.2)
    assert backend.pulse_trains == {"ctr1": {"frequency_hz": 10.0, "duty_cycle": 0.2, "count": None}}
    with pytest.raises(RuntimeError, match="generating a pulse train"):
        _go(service, "daq_count_edges", terminal="pfi0", counter="ctr1")
    assert _go(service, "daq_stop_output", channel="clock") == {"daq_channel": "clock:ctr1"}
    assert backend.pulse_trains == {}
    for params in ({"counter": "ctr0", "frequency_hz": 10.0},                 # not enabled
                   {"counter": "ctr1", "frequency_hz": 501.0},                # above daq_co_max_hz
                   {"counter": "ctr1", "frequency_hz": 10.0, "duty_cycle": 1.0}):
        with pytest.raises(DaqPolicyError):
            _go(service, "daq_generate_pulses", **params)
    with pytest.raises(ValueError, match="not both"):
        _go(service, "daq_generate_pulses", counter="ctr1", frequency_hz=10.0, count=5,
            duration_s=1.0)
    assert backend.pulse_trains == {}


def test_service_stop_output_leaves_undriven_outputs_alone():
    service, backend = _service()
    # nothing was sent, so no level is put on record
    assert _go(service, "daq_stop_output", channel="valve") == {"daq_channel": "valve:port0/line3"}
    assert _go(service, "daq_stop_output", channel="lamp") == {"daq_channel": "lamp:ao0"}
    assert backend.digital_outputs == {} and backend.analog_outputs == {}
    with pytest.raises(DaqPolicyError):
        _go(service, "daq_stop_output", channel="photodiode")


def test_service_on_the_real_driver_holds_outputs_and_resets_them(bench):
    """The same service over NIDaq (fake nidaqmx): what reaches the device."""
    shared = _shared(daq_device="Dev1", **ENABLED)

    async def scenario():
        await shared.daq_step("daq_generate_waveform", channel="lamp", frequency_hz=10.0,
                              amplitude_v=1.0, offset_v=1.0)
        await shared.daq_step("daq_generate_pulses", counter="clock", frequency_hz=100.0)
        await shared.daq_step("daq_write_digital", line="valve", state=True)
        with pytest.raises(DaqPolicyError):
            await shared.daq_step("daq_generate_pulses", counter="ctr0", frequency_hz=100.0)
        assert bench.open_tasks == 3
        await shared.cleanup()

    asyncio.run(scenario())
    assert bench.outputs == {"Dev1/ao0": 0.0, "Dev1/port0/line3": False}
    assert ("stop", "Dev1/ctr1") in bench.events and bench.open_tasks == 0


def test_shared_devices_and_mock_expose_the_same_entry_point():
    assert (inspect.signature(SharedDevices.daq_step)
            == inspect.signature(MockSharedDevices.daq_step))
    mock = MockSharedDevices(use_daq=True, **ENABLED)
    assert asyncio.run(mock.daq_step("daq_write_analog", channel="lamp", voltage=2.0)) == {
        "voltage": 2.0, "daq_channel": "lamp:ao0"}
    assert mock.daq_analog_outputs == {"ao0": 2.0}
    asyncio.run(mock.cleanup())
    assert mock.daq_analog_outputs == {"ao0": 0.0}
    with pytest.raises(RuntimeError, match="DAQ"):
        asyncio.run(MockSharedDevices().daq_step("daq_read_analog", channel="ai0"))
    with pytest.raises(DaqPolicyError):
        MockSharedDevices(use_daq=True, daq_signals={"ai0": "ai1"})


# ----------------------------------------------------------------------
# Schema, executor, logger and runner for the extended steps
# ----------------------------------------------------------------------
EVERY_STEP = [
    {"action": "daq_read_analog", "channel": "photodiode"},
    {"action": "daq_acquire", "channels": ["photodiode", "ai0"], "rate_hz": 100.0, "duration_s": 0.05,
     "trigger": "flow_meter"},
    {"action": "daq_start_logging", "channels": ["ai1"], "rate_hz": 100.0},
    {"action": "daq_count_edges", "terminal": "flow_meter", "duration_s": 0.5},
    {"action": "daq_measure_frequency", "terminal": "pfi0"},
    {"action": "daq_stop_logging"},
    {"action": "daq_wait_analog", "channel": "photodiode", "threshold": 0.1, "condition": "below"},
    {"action": "daq_write_analog", "channel": "lamp", "voltage": 2.5},
    {"action": "daq_generate_waveform", "channel": "lamp", "frequency_hz": 10.0, "amplitude_v": 1.0,
     "offset_v": 1.0, "duration_s": 0.5},
    {"action": "daq_write_digital", "line": "valve", "state": True},
    {"action": "daq_pulse_digital", "line": "valve", "duration_s": 0.2},
    {"action": "daq_wait_digital", "line": "port0/line1"},
    {"action": "daq_read_digital", "line": "port0/line2"},
    {"action": "daq_generate_pulses", "counter": "clock", "frequency_hz": 100.0, "count": 10},
    {"action": "daq_stop_output", "channel": "lamp"},
    {"action": "daq_stimulate_acquire", "channel": "lamp", "frequency_hz": 10.0,
     "amplitude_v": 1.0, "offset_v": 1.0, "channels": ["photodiode"], "duration_s": 0.1},
    {"action": "daq_acquire_digital", "lines": ["port0/line1"], "duration_s": 0.01},
    {"action": "daq_write_digital_pattern", "lines": ["valve"], "pattern": ["1", "0", "1"],
     "rate_hz": 100.0, "record": ["port0/line1"]},
    {"action": "daq_wait_digital_change", "lines": ["port0/line1"]},
    {"action": "daq_measure_pulse_width", "terminal": "flow_meter"},
    {"action": "daq_measure_period", "terminal": "pfi0"},
    {"action": "daq_measure_edge_separation", "first": "pfi0", "second": "pfi1"},
    {"action": "daq_start_encoder", "a": "pfi8", "b": "pfi10", "pulses_per_rev": 360},
    {"action": "daq_read_encoder"},
    {"action": "daq_stop_encoder"},
]


def test_every_daq_action_has_a_schema_and_is_exercised_here():
    assert {s["action"] for s in EVERY_STEP} == DAQ_ACTIONS == set(DAQ_ACTION_NAMES)
    workflow = ExperimentWorkflow(name="all", description="", steps=EVERY_STEP)
    assert len(workflow.steps) == len(EVERY_STEP)
    assert workflow.steps[0].v_range is None and workflow.steps[0].terminal is None


@pytest.mark.parametrize("step", [
    {"action": "daq_acquire", "channels": []},
    {"action": "daq_acquire", "channels": ["ai0", "ai0"]},
    {"action": "daq_acquire", "channels": ["ai0", "ai1"], "rate_hz": 600000.0, "duration_s": 0.1},
    {"action": "daq_acquire", "channels": ["ai0"], "rate_hz": 1000000.0, "duration_s": 3.0},
    {"action": "daq_acquire", "channels": ["ai0"], "rate_hz": 1.0, "duration_s": 0.5},
    {"action": "daq_acquire", "channels": ["ao0"]},
    {"action": "daq_acquire", "channels": ["ai0"], "trigger": "ai1"},     # no trigger_level
    {"action": "daq_acquire", "channels": ["ai0"], "trigger_edge": "both"},
    {"action": "daq_start_logging", "channels": ["ai0"], "rate_hz": 20000.0},
    {"action": "daq_stop_logging", "channel": "ai0"},
    {"action": "daq_wait_analog", "channel": "ai0"},                          # threshold required
    {"action": "daq_wait_analog", "channel": "ai0", "threshold": 1.0, "condition": "equal"},
    {"action": "daq_generate_waveform", "channel": "ao0", "frequency_hz": 10.0},   # amplitude
    {"action": "daq_generate_waveform", "channel": "ao0", "frequency_hz": 0, "amplitude_v": 1.0},
    {"action": "daq_generate_waveform", "channel": "ao0", "frequency_hz": 10.0, "amplitude_v": 1.0,
     "waveform": "noise"},
    {"action": "daq_generate_waveform", "channel": "port0/line0", "frequency_hz": 10.0,
     "amplitude_v": 1.0},
    {"action": "daq_stop_output", "channel": "ai0"},
    {"action": "daq_stop_output", "channel": "pfi0"},
    {"action": "daq_pulse_digital", "line": "port0/line0"},                   # duration required
    {"action": "daq_pulse_digital", "line": "port0/line0", "duration_s": 0},
    {"action": "daq_wait_digital", "line": "ctr0"},
    {"action": "daq_count_edges", "terminal": "port0/line0"},
    {"action": "daq_count_edges", "terminal": "pfi0", "counter": "pfi1"},
    {"action": "daq_measure_frequency", "terminal": "pfi0", "min_hz": 100.0, "max_hz": 10.0},
    {"action": "daq_generate_pulses", "counter": "ctr0"},                     # frequency required
    {"action": "daq_generate_pulses", "counter": "ctr0", "frequency_hz": 10.0, "duty_cycle": 1.0},
    {"action": "daq_generate_pulses", "counter": "ctr0", "frequency_hz": 10.0, "count": 5,
     "duration_s": 1.0},
    {"action": "daq_generate_pulses", "counter": "ao0", "frequency_hz": 10.0},
    {"action": "daq_read_analog", "channel": "my signal"},
    {"action": "daq_read_analog", "channel": "ai0", "robot_id": 1},
])
def test_schema_rejects_extended(step):
    with pytest.raises(ValidationError):
        _wf(step)


def test_executor_runs_every_daq_action_on_the_mock(tmp_path):
    mock = MockSharedDevices(use_daq=True, **ENABLED)

    async def scenario():
        results = []
        for i, step in enumerate(EVERY_STEP):
            if step["action"] in DAQ_DATA_ACTIONS:
                step = {**step, "file_path": str(tmp_path / f"{i}.csv")}
            results.append(await execute_step(step, {}, mock))
        await mock.cleanup()
        return results

    results = asyncio.run(scenario())
    assert all(isinstance(r, dict) and r.get("daq_channel") for r in results)
    assert results[0] == {"voltage": 1.234, "daq_channel": "photodiode:ai2",
                          "daq_value": pytest.approx(2.968), "daq_unit": "mW"}
    assert os.path.exists(results[1]["data_path"]) and os.path.exists(results[5]["data_path"])
    assert mock.daq_analog_outputs == {"ao0": 0.0} and mock.daq.pulse_trains == {}


def test_executor_passes_only_what_the_step_states():
    """A pydantic step must not push its defaults over the signal's wiring."""
    mock = MockSharedDevices(use_daq=True, **ENABLED)
    seen = []
    mock.daq.read_analog = lambda *a: seen.append(a) or 1.0
    model = _wf({"action": "daq_read_analog", "channel": "photodiode"}).steps[0]
    asyncio.run(execute_step(model, {}, mock))
    asyncio.run(execute_step(model.model_dump(), {}, mock))             # None values are dropped
    asyncio.run(execute_step({"action": "daq_read_analog", "channel": "photodiode",
                              "terminal": "rse", "_iteration": 2}, {}, mock))
    assert seen == [("ai2", 100, 1000.0, 1.0, "diff"), ("ai2", 100, 1000.0, 1.0, "diff"),
                    ("ai2", 100, 1000.0, 1.0, "rse")]


def test_logger_writes_the_value_and_data_columns(tmp_path):
    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    log = ExperimentLogger("d", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    data_path = log.next_data_path(2, tag="acquire")
    assert os.path.dirname(data_path) == os.path.join(log.dir, "data") and os.path.isdir(
        os.path.dirname(data_path))
    assert os.path.basename(data_path).startswith("step002_acquire_")
    log.record_step(1, 4, "daq_read_analog", None, None, "ok", 0.1,
                    result={"voltage": 1.0, "daq_channel": "photodiode:ai2", "daq_value": 2.5,
                            "daq_unit": "mW"})
    log.record_step(2, 4, "daq_acquire", None, None, "ok", 0.1,
                    result={"daq_channel": "ai0+ai1", "data_path": data_path})
    log.record_step(3, 4, "daq_count_edges", None, None, "ok", 0.1,
                    result={"daq_channel": "pfi0", "daq_value": 1234, "daq_unit": "count"})
    log.record_step(4, 4, "daq_stop_logging", None, None, "ok", 0.1,
                    result={"daq_channel": "ai0", "data_path": data_path, "daq_value": 500,
                            "daq_unit": "samples"})
    log.finalize("completed")
    rows = _rows(log.dir)
    assert (rows[0]["voltage_v"], rows[0]["daq_value"], rows[0]["daq_unit"]) == ("1.000000", "2.5", "mW")
    assert (rows[1]["data_path"], rows[1]["daq_value"]) == (data_path, "")
    assert (rows[2]["daq_value"], rows[2]["daq_unit"]) == ("1234", "count")
    assert (rows[3]["daq_value"], rows[3]["daq_unit"]) == ("500", "samples")
    with open(os.path.join(log.dir, "summary.md"), encoding="utf-8") as f:
        summary = f.read()
    assert "photodiode:ai2 +1.00000 V = 2.5 mW" in summary and "pfi0 1234 count" in summary
    assert summary.count("📈 データ") == 1                 # the stop row repeats the same file
    with open(os.path.join(log.dir, "metadata.json"), encoding="utf-8") as f:
        import json
        assert json.load(f)["output_files"]["data_dir"] == "data"


def test_input_only_example_mock_run_keeps_its_data_in_the_run_folder(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "acquire_and_count.json")
    assert run_flow.main([flow, "--validate-only"]) == 0
    assert run_flow.main([flow, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    rows = _rows(run_dir)
    assert [r["action"] for r in rows] == ["daq_acquire", "daq_start_logging", "daq_count_edges",
                                           "daq_measure_frequency", "daq_read_digital",
                                           "daq_stop_logging"]
    assert all(r["status"] == "ok" and r["robot_id"] == "" for r in rows)
    for row in (rows[0], rows[1]):
        assert os.path.dirname(row["data_path"]) == os.path.join(run_dir, "data")
        assert os.path.exists(row["data_path"])
    assert rows[5]["data_path"] == rows[1]["data_path"] and int(rows[5]["daq_value"]) >= 1
    assert len(_csv(rows[0]["data_path"])) == 5001          # header + 0.5 s at 10 kHz
    assert (rows[2]["daq_value"], rows[2]["daq_unit"]) == ("1000", "count")


def test_template_flow_needs_the_wiring_from_the_configuration(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "stimulus_response_template.json")
    assert run_flow.main([flow, "--validate-only"]) == 0

    def use(extra):
        shared = dict(run_flow.lab_config.get_shared_devices(), daq_signals={},
                      daq_ao_channels=[], daq_do_lines=[], daq_co_counters=[])
        shared.update(extra)
        monkeypatch.setattr(run_flow.lab_config, "get_shared_devices", lambda: dict(shared))

    use({})                                                 # as shipped: the names mean nothing
    assert run_flow.main([flow, "--mock"]) == 1
    assert "daq_signals" in _rows(_latest_run_dir(str(tmp_path / "logs")))[0]["error"]

    wiring = {"daq_signals": {"lamp": "ao0", "excitation": "ao1", "ready": "port0/line1",
                              "photodiode": {"channel": "ai0", "scale": 2.0, "unit": "mW"},
                              "valve": "port0/line0", "stirrer_clock": "ctr0"}}
    use(wiring)                                             # named but not enabled: still refused
    assert run_flow.main([flow, "--mock"]) == 1
    assert "daq_ao_channels" in _rows(_latest_run_dir(str(tmp_path / "logs")))[0]["error"]

    use({**wiring, "daq_ao_channels": ["ao0", "ao1"], "daq_do_lines": ["port0/line0"],
         "daq_co_counters": ["ctr0"], "daq_ao_max_v": 5.0, "daq_co_max_hz": 1000.0})
    assert run_flow.main([flow, "--mock"]) == 0
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert all(r["status"] == "ok" for r in rows) and len(rows) == 9
    assert rows[3]["daq_channel"] == "photodiode:ai0" and rows[3]["daq_unit"] == "mW"


def test_session_passes_signals_and_counter_settings_to_the_mock():
    config = {"scale_port": "", "camera_index": 0, "daq_device": "Dev1", **ENABLED}

    async def go():
        session = ExperimentSession(mock=True, shared_config=config)
        return await session.add_shared(use_daq=True)

    shared = asyncio.run(go())
    assert shared.daq_service.signals.names() == sorted(SIGNALS)
    assert shared.daq_policy.co_counters == {"ctr1"} and shared.daq_policy.co_max_hz == 500.0


def test_gui_palette_and_agent_prompt_cover_every_daq_action():
    import ast
    with open(os.path.join(REPO_ROOT, "src", "gui", "app.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    values = {t.id: n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    config = ast.literal_eval(values["ACTION_CONFIG"])
    categories = ast.literal_eval(values["CATEGORIES"])
    assert set(categories["daq"][1]) == DAQ_ACTIONS
    for action in DAQ_ACTIONS:
        # a block dropped from the palette is a valid step as it stands
        _wf({"action": action, **config[action]["defaults"]})
    from src.agent import llm_service
    for action in DAQ_ACTIONS:
        assert f'"action": "{action}"' in llm_service.SYSTEM_PROMPT


# ----------------------------------------------------------------------
# Review fixes: shutdown races, failed transitions, shared pins, timing
# ----------------------------------------------------------------------
@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_no_output_can_be_driven_after_close(bench, make):
    """A worker thread that outlives a cancelled step must not drive anything behind close()."""
    daq = make()
    daq.write_analog("ao0", 1.0)
    assert daq.close() == []
    for call in (lambda: daq.write_analog("ao0", 1.0),
                 lambda: daq.write_digital("port0/line0", True),
                 lambda: daq.start_waveform("ao0", [0.0, 1.0], 100.0),
                 lambda: daq.start_pulses("ctr0", 10.0)):
        with pytest.raises(RuntimeError, match="closed"):
            call()
    if make is NIDaq:
        assert daq.stop_output("ao0") is False              # harmless, sends nothing
    assert daq.connect() is False                           # one object serves one run
    with pytest.raises(RuntimeError, match="closed"):
        daq.start_stream([("ai0", 10.0, "default")], 100.0)


def test_close_waits_for_an_output_call_still_running_in_another_thread(bench):
    import threading
    daq = NIDaq()
    daq.write_analog("ao0", 1.0)
    entered, release, order = threading.Event(), threading.Event(), []
    task = daq._ao_tasks["ao0"]
    original = task.write

    def slow_write(data, auto_start=None):
        if data == 2.0:
            entered.set()
            release.wait(5.0)
            order.append("write 2.0 done")
        original(data, auto_start=auto_start)

    task.write = slow_write
    worker = threading.Thread(target=daq.write_analog, args=("ao0", 2.0))
    worker.start()
    assert entered.wait(5.0)
    closer = threading.Thread(target=lambda: order.append(("closed", daq.close())))
    closer.start()
    closer.join(0.2)
    assert closer.is_alive()                                # close() is waiting for the write
    release.set()
    worker.join(5.0)
    closer.join(5.0)
    assert order == ["write 2.0 done", ("closed", [])]
    assert bench.outputs["Dev1/ao0"] == 0.0                 # the reset came last


def test_a_failed_waveform_start_does_not_make_close_forget_the_channel(bench):
    daq = NIDaq()
    daq.write_analog("ao0", 3.0)
    bench.fail_start.add("Dev1/ao0")
    with pytest.raises(RuntimeError, match="start failed"):
        daq.start_waveform("ao0", [0.0, 1.0], 100.0)        # the DC task is gone, 3 V is held
    assert "ao0" not in daq._ao_tasks and bench.outputs["Dev1/ao0"] != 0.0
    assert daq.stop_output("ao0") is True                   # still known as driven
    assert bench.outputs["Dev1/ao0"] == 0.0
    daq.write_analog("ao0", 3.0)
    with pytest.raises(RuntimeError, match="start failed"):
        daq.start_waveform("ao0", [0.0, 1.0], 100.0)
    assert daq.close() == [] and bench.outputs["Dev1/ao0"] == 0.0 and bench.open_tasks == 0


def test_an_interrupted_pulse_start_leaves_no_task_behind(bench):
    daq = NIDaq()
    bench.fail_start.add("Dev1/ctr0")
    with pytest.raises(RuntimeError, match="start failed"):
        daq.start_pulses("ctr0", 10.0)
    assert daq._co_tasks == {} and bench.open_tasks == 0


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_only_one_waveform_at_a_time(bench, make):
    daq = make()
    daq.start_waveform("ao0", [0.0, 1.0], 100.0)
    with pytest.raises(RuntimeError, match="ao0 is generating a waveform"):
        daq.start_waveform("ao1", [0.0, 1.0], 100.0)
    daq.start_waveform("ao0", [0.0, 2.0], 100.0)            # replacing the same channel is fine
    daq.write_analog("ao0", 0.0)                            # ends the waveform
    daq.start_waveform("ao1", [0.0, 1.0], 100.0)
    assert daq.close() == []


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_a_counter_output_pin_is_not_also_used_as_a_digital_line(bench, make):
    daq = make()
    daq.start_pulses("ctr0", 10.0)                          # comes out on PFI12 = port2/line4
    with pytest.raises(RuntimeError, match="output pin of ctr0"):
        daq.write_digital("port2/line4", True)
    with pytest.raises(RuntimeError, match="output pin of ctr0"):
        daq.read_digital("port2/line4")
    daq.read_digital("port2/line5")
    daq.stop_output("ctr0")
    daq.write_digital("port2/line4", True)
    with pytest.raises(RuntimeError, match="driven as a digital line"):
        daq.start_pulses("ctr0", 10.0)
    daq.start_pulses("ctr1", 10.0)
    assert daq.close() == []


def test_count_edges_follows_the_32_bit_counter_through_a_wrap(bench, monkeypatch):
    daq = NIDaq()
    readings = iter([4_000_000_000, 3_705_032_704, 3_410_065_408])   # +4e9 each, modulo 2**32
    slept = []
    import nidaqmx
    monkeypatch.setattr(nidaqmx.Task, "read", lambda self, **kw: next(readings))
    assert daq.count_edges("20mhztimebase", 25.0, sleep=slept.append) == 12_000_000_000
    assert slept == [10.0, 10.0, 5.0]


def test_the_time_axis_uses_the_rate_the_device_really_set(bench, tmp_path):
    bench.coerced_rate = 3000.03
    bench.inputs["Dev1/ai0"] = [0.0, 1.0, 2.0]
    service = DaqService(NIDaq(), ENABLED)
    path = str(tmp_path / "w.csv")
    _go(service, "daq_acquire", channels=["ai0"], rate_hz=3000.0, duration_s=0.001, file_path=path)
    rows = _csv(path)
    assert [r[0] for r in rows[1:]] == ["0.000000", "0.000333", "0.000667"]
    assert float(rows[2][0]) == pytest.approx(1 / 3000.03, abs=1e-6)
    # the frequency on record is the one generated
    bench.coerced_rate = 990.0
    assert _go(service, "daq_generate_waveform", channel="ao0", frequency_hz=1.0,
               amplitude_v=1.0, offset_v=1.0)["daq_value"] == pytest.approx(0.99)
    assert service.close() == []


def test_fast_waveforms_keep_distinct_times(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "fast.csv")
    _go(service, "daq_acquire", channels=["ai0"], rate_hz=1.25e6, duration_s=0.00001, file_path=path)
    times = [r[0] for r in _csv(path)[1:]]
    assert len(times) == len(set(times)) and times[1] == "0.000000800"


def test_an_interrupted_low_pulse_ends_low_not_high():
    service, backend = _service()

    async def interrupted():
        async def boom(seconds):
            raise asyncio.CancelledError()
        service._sleep = boom
        await service.run("daq_pulse_digital", line="valve", duration_s=1.0, state=False)

    backend.write_digital("port0/line3", True)              # the line idles high before the pulse
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(interrupted())
    assert backend.digital_outputs == {"port0/line3": False}


def test_service_close_resets_outputs_before_and_whatever_the_recording_does(tmp_path):
    service, backend = _service()
    _go(service, "daq_start_logging", channels=["ai0"], file_path=str(tmp_path / "l.csv"))
    _go(service, "daq_write_analog", channel="lamp", voltage=2.0)
    seen = {}

    def broken_finish():
        seen["lamp at the time"] = backend.analog_outputs["ao0"]
        service._logging.stop.set()
        raise RuntimeError("join failed")

    service._finish_logging = broken_finish
    assert service.close() == []
    assert seen == {"lamp at the time": 0.0} and backend.analog_outputs == {"ao0": 0.0}


def test_a_logging_thread_that_cannot_start_releases_the_inputs(tmp_path, monkeypatch):
    import threading
    service, backend = _service()
    monkeypatch.setattr(threading.Thread, "start",
                        lambda self: (_ for _ in ()).throw(RuntimeError("can't start new thread")))
    with pytest.raises(RuntimeError, match="can't start"):
        _go(service, "daq_start_logging", channels=["ai0"], file_path=str(tmp_path / "l.csv"))
    monkeypatch.undo()
    assert service._logging is None
    assert _go(service, "daq_read_analog", channel="ai0")["voltage"] == 1.234   # inputs are free
    assert service.close() == []


def test_non_finite_calibration_and_thresholds_are_refused():
    for spec in ({"channel": "ai0", "offset": float("inf")}, {"channel": "ai0", "scale": float("-inf")}):
        with pytest.raises(DaqPolicyError, match="finite"):
            DaqSignals.from_config({"daq_signals": {"sensor": spec}})
    service, backend = _service(mock=False)
    with pytest.raises(ValueError, match="finite"):
        _go(service, "daq_wait_analog", channel="ai0", threshold=float("nan"))
    backend.read_analog = lambda *a: float("inf")
    with pytest.raises(RuntimeError, match="not a finite number"):
        _go(service, "daq_wait_analog", channel="ai0", threshold=1.0, timeout_s=0.1)
    with pytest.raises(RuntimeError, match="not a finite number"):
        _go(service, "daq_read_analog", channel="ai0")        # not recorded as a measurement


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_a_reader_of_an_earlier_stream_is_refused_the_next_ones_samples(bench, make):
    daq = make()
    first = daq.start_stream([("ai0", 10.0, "default")], 100.0)
    daq.read_stream(first)
    daq.stop_stream()
    with pytest.raises(RuntimeError, match="no continuous acquisition"):
        daq.read_stream(first)
    second = daq.start_stream([("ai1", 10.0, "default")], 100.0)
    assert second != first
    with pytest.raises(RuntimeError, match="another one is running"):
        daq.read_stream(first)                              # a stalled logger waking up late
    daq.read_stream(second)
    daq.read_stream()                                       # no number: the current stream
    assert daq.close() == []


def test_loopback_check_flow_runs_only_with_its_outputs_enabled(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "output_loopback_check.json")
    assert run_flow.main([flow, "--validate-only"]) == 0

    def use(extra):
        shared = dict(run_flow.lab_config.get_shared_devices(), daq_signals={},
                      daq_ao_channels=[], daq_do_lines=[], daq_co_counters=[])
        shared.update(extra)
        monkeypatch.setattr(run_flow.lab_config, "get_shared_devices", lambda: dict(shared))

    use({})                                                 # as shipped: refused at the first step
    assert run_flow.main([flow, "--mock"]) == 1
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert len(rows) == 1 and "daq_ao_channels" in rows[0]["error"]

    use({"daq_ao_channels": ["ao0", "ao1"], "daq_do_lines": ["port0/line0"],
         "daq_co_counters": ["ctr0"], "daq_ao_max_v": 5.0, "daq_co_max_hz": 1000.0})
    assert run_flow.main([flow, "--mock"]) == 0
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert len(rows) == 30 and all(r["status"] == "ok" for r in rows)


def test_timing_example_runs_on_the_mock_with_nothing_enabled(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "timing_and_triggers.json")
    assert run_flow.main([flow, "--validate-only"]) == 0
    assert run_flow.main([flow, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    rows = _rows(run_dir)
    assert len(rows) == 9 and all(r["status"] == "ok" for r in rows)
    assert [(r["daq_value"], r["daq_unit"]) for r in rows[:3]] == [
        ("0.001", "s"), ("0.0005", "s"), ("0.00025", "s")]
    for row in rows[3:6]:                                   # two waveforms and a digital one
        assert os.path.dirname(row["data_path"]) == os.path.join(run_dir, "data")
        assert os.path.exists(row["data_path"])
    assert _csv(rows[4]["data_path"])[1][0] == "-0.200000"     # pre-trigger samples
    assert (float(rows[7]["daq_value"]), rows[7]["daq_unit"]) == (90.0, "deg")


@pytest.mark.parametrize("name", ["read_voltage.json", "acquire_and_count.json",
                                  "timing_and_triggers.json"])
def test_input_only_examples_open_no_robot(name):
    """A robot step (even ``wait``) makes a real run connect to arm 1."""
    import json
    with open(os.path.join(REPO_ROOT, "examples", "daq", name), encoding="utf-8") as f:
        steps = json.load(f)["steps"]
    assert all(step["action"].startswith(("daq_", "loop_")) for step in steps)


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_shutdown_outputs_closes_admission_but_leaves_the_inputs_running(bench, make):
    """The end of a run: outputs off and refused at once, the recording still readable."""
    daq = make()
    daq.start_stream([("ai0", 10.0, "default")], 100.0)
    daq.write_analog("ao0", 2.0)
    daq.start_pulses("ctr0", 10.0)
    assert daq.shutdown_outputs() == []
    with pytest.raises(RuntimeError, match="closed"):
        daq.write_analog("ao0", 1.0)                        # a late worker drives nothing
    daq.read_stream()                                       # the recording can be wound up
    assert daq.close() == []


def test_service_close_closes_the_device_even_when_the_wind_up_is_interrupted(tmp_path):
    service, backend = _service()
    _go(service, "daq_write_analog", channel="lamp", voltage=2.0)

    def interrupted():
        raise KeyboardInterrupt()

    service._finish_logging = interrupted
    with pytest.raises(KeyboardInterrupt):
        service.close()
    assert backend._closed and backend.analog_outputs == {"ao0": 0.0}


def test_real_close_resets_the_outputs_before_it_stops_the_stream(bench):
    daq = NIDaq()
    daq.start_stream([("ai0", 10.0, "default")], 100.0)
    daq.write_analog("ao0", 2.0)
    del bench.events[:]
    assert daq.close() == []
    assert bench.events.index(("write", "Dev1/ao0", 0.0)) < bench.events.index(("stop", "Dev1/ai0"))

# ----------------------------------------------------------------------
# Sensors, triggers, clocks, timed digital I/O, time measurements, encoder
# ----------------------------------------------------------------------
SENSOR_SIGNALS = {
    "furnace": {"channel": "ai4", "sensor": "thermocouple", "tc_type": "k", "cjc_c": 23.5,
                "max_c": 400},
    "bath": {"channel": "ai5", "sensor": "rtd", "excitation_a": 0.001, "terminal": "diff"},
}


def test_sensor_signals_are_parsed_with_their_wiring_facts():
    signals = DaqSignals.from_config({"daq_signals": SENSOR_SIGNALS})
    furnace = signals.resolve("furnace", "ai")
    assert furnace.sensor == {"type": "thermocouple", "min_c": 0.0, "max_c": 400.0,
                              "tc_type": "K", "cjc_c": 23.5}
    assert furnace.unit == "degC" and furnace.scaled and furnace.raw_unit == "degC"
    bath = signals.resolve("bath", "ai")
    assert bath.sensor == {"type": "rtd", "min_c": 0.0, "max_c": 200.0, "rtd_type": "pt3851",
                           "r0_ohm": 100.0, "excitation_a": 0.001, "wires": 4}
    assert signals.resolve("ai0", "ai").sensor is None


@pytest.mark.parametrize("spec, message", [
    ({"channel": "ai0", "sensor": "bridge"}, "sensor must be one of"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "K"}, "cjc_c"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "X", "cjc_c": 25}, "tc_type"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "K", "cjc_c": 25, "v_range": 1}, "v_range"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "K", "cjc_c": 25, "wires": 4}, "do not apply"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "K", "cjc_c": 25, "min_c": 5, "max_c": 5},
     "min_c"),
    ({"channel": "ai0", "sensor": "rtd"}, "excitation_a"),
    ({"channel": "ai0", "sensor": "rtd", "excitation_a": 0.5}, "excitation_a"),
    ({"channel": "ai0", "sensor": "rtd", "excitation_a": 0.001, "wires": 5}, "wires"),
    ({"channel": "ai0", "tc_type": "K"}, "need sensor"),
    ({"channel": "ao0", "sensor": "thermocouple"}, "unknown or unusable"),
])
def test_sensor_signals_that_are_wrong_are_refused(spec, message):
    with pytest.raises(DaqPolicyError, match=message):
        DaqSignals.from_config({"daq_signals": {"probe": spec}})


def test_real_driver_opens_sensor_channels_with_the_configured_facts(bench):
    signals = DaqSignals.from_config({"daq_signals": SENSOR_SIGNALS})
    daq = NIDaq()
    bench.inputs["Dev1/ai4"] = 180.0
    furnace, bath = signals.resolve("furnace", "ai"), signals.resolve("bath", "ai")
    assert daq.read_analog("ai4", 1, 1000.0, 10.0, "default", furnace.sensor) == 180.0
    daq.acquire([("ai0", 1.0, "rse"), ("ai5", 10.0, "diff", bath.sensor)], 100.0, 4)
    kinds = [(c["phys"], c.get("made")) for c in bench.ai_configs]
    assert kinds == [("Dev1/ai4", "thermocouple"), ("Dev1/ai0", None), ("Dev1/ai5", "rtd")]
    tc, _, rtd = bench.ai_configs
    assert (tc["thermocouple_type"], tc["cjc_source"], tc["cjc_val"], tc["max_val"]) == (
        "TC_K", "CJC_CONST", 23.5, 400.0)
    assert (rtd["rtd_type"], rtd["resistance_config"], rtd["current_excit_source"],
            rtd["current_excit_val"], rtd["r_0"]) == ("PT_3851", "FOUR_WIRE", "EXTERNAL", 0.001, 100.0)
    assert bench.ai_chans["Dev1/ai5"].ai_term_cfg == "T_DIFF"
    assert not hasattr(bench.ai_chans["Dev1/ai4"], "ai_term_cfg")


def test_service_records_a_sensor_reading_in_its_unit_without_a_voltage(tmp_path):
    service, backend = _service({"daq_signals": SENSOR_SIGNALS})
    assert _go(service, "daq_read_analog", channel="furnace") == {
        "daq_channel": "furnace:ai4", "daq_value": 25.0, "daq_unit": "degC"}
    with pytest.raises(ValueError, match="v_range does not apply"):
        _go(service, "daq_read_analog", channel="furnace", v_range=1.0)
    path = str(tmp_path / "t.csv")
    result = _go(service, "daq_acquire", channels=["ai0", "furnace"], rate_hz=100.0,
                 duration_s=0.03, file_path=path)
    assert "voltage" not in result
    assert _csv(path)[:2] == [["time_s", "ai0_V", "furnace:ai4_degC"], ["0.000000", "1.234000", "25"]]
    one = _go(service, "daq_acquire", channels=["bath"], rate_hz=100.0, duration_s=0.03,
              file_path=str(tmp_path / "b.csv"))
    assert (one["daq_value"], one["daq_unit"]) == (25.0, "degC") and "voltage" not in one
    waited = _go(service, "daq_wait_analog", channel="furnace", threshold=20.0)
    assert waited == {"daq_channel": "furnace:ai4", "daq_value": 25.0, "daq_unit": "degC"}
    _go(service, "daq_start_logging", channels=["furnace"], file_path=str(tmp_path / "l.csv"))
    _go(service, "daq_stop_logging")
    assert _csv(str(tmp_path / "l.csv"))[0] == ["time_s", "furnace:ai4_degC"]


def test_real_acquire_configures_triggers_pretrigger_and_external_clock(bench):
    daq = NIDaq()
    spec = [("ai0", 10.0, "default"), ("ai1", 10.0, "default")]
    daq.acquire(spec, 1000.0, 10, "ai0", "falling", 1.0, trigger_level=2.5)
    assert bench.triggers[-1] == ("anlg_start", "Dev1/ai0", "FALLING", 2.5)
    daq.acquire(spec, 1000.0, 10, "ai0", "rising", 1.0, trigger_level=-1.0, pretrigger=4)
    assert bench.triggers[-1] == ("anlg_ref", "Dev1/ai0", "RISING", -1.0, 4)
    daq.acquire(spec, 1000.0, 10, "pfi2", "rising", 1.0, pretrigger=4)
    assert bench.triggers[-1] == ("dig_ref", "/Dev1/PFI2", "RISING", 4)
    daq.acquire(spec, 1000.0, 10, clock="pfi3")
    assert bench.clock_sources[-1] == "/Dev1/PFI3" and daq.ai_rate_hz == 1000.0
    for kwargs, message in (({"trigger": "ai1", "trigger_level": 1.0}, "first channel"),
                            ({"pretrigger": 4}, "need a trigger"),
                            ({"trigger": "pfi0", "pretrigger": 9}, "at least 2 samples"),
                            ({"trigger_level": 1.0}, "needs the analog input")):
        with pytest.raises(ValueError, match=message):
            daq.acquire(spec, 1000.0, 10, **kwargs)
    bench.read_error = FakeDaqError(-200281)                # the reference trigger never came
    with pytest.raises(TimeoutError, match="no rising edge on pfi2"):
        daq.acquire(spec, 1000.0, 10, "pfi2", "rising", 0.1, pretrigger=4)
    bench.read_error = FakeDaqError(-200284)
    with pytest.raises(TimeoutError, match="clock on pfi3"):
        daq.acquire(spec, 1000.0, 10, clock="pfi3")


def test_service_acquire_with_analog_trigger_and_pretrigger_time_axis(tmp_path):
    service, backend = _service()
    seen = {}
    original = backend.acquire
    backend.acquire = lambda *a, **k: seen.update(args=a, kwargs=k) or original(*a, **k)
    path = str(tmp_path / "p.csv")
    _go(service, "daq_acquire", channels=["ai0", "ai1"], rate_hz=100.0, duration_s=0.1,
        trigger="ai0", trigger_level=1.5, pretrigger_s=0.04, clock="flow_meter", file_path=path)
    assert seen["args"][3] == "ai0"
    assert seen["kwargs"] == {"trigger_level": 1.5, "pretrigger": 4, "clock": "pfi4"}
    times = [row[0] for row in _csv(path)[1:]]
    assert times[0] == "-0.040000" and times[4] == "0.000000" and times[-1] == "0.050000"
    for params, message in (
            ({"trigger": "ai1", "trigger_level": 1.0}, "first of channels"),
            ({"trigger_level": 1.0}, "needs trigger"),
            ({"trigger": "ai0", "trigger_level": 11.0}, "outside the input range"),
            ({"pretrigger_s": 0.04}, "needs a trigger"),
            ({"trigger": "pfi0", "pretrigger_s": 0.1}, "at least 2 samples")):
        with pytest.raises(ValueError, match=message):
            _go(service, "daq_acquire", channels=["ai0", "ai1"], rate_hz=100.0, duration_s=0.1,
                **params)
    with pytest.raises(DaqPolicyError):                     # a PFI is not an analog trigger
        _go(service, "daq_acquire", channels=["ai0"], trigger="pfi0", trigger_level=1.0)


def test_real_stimulate_acquire_arms_the_inputs_then_starts_and_resets_the_output(bench):
    daq = NIDaq()
    bench.inputs["Dev1/ai0"] = [0.1, 0.2, 0.3]
    data, rate = daq.stimulate_acquire("ao0", [0.0, 1.0, 2.0, 1.0], 400.0,
                                       [("ai0", 10.0, "default")], 100.0, 3)
    assert data == [[0.1, 0.2, 0.3]] and rate == 400.0
    assert bench.triggers[-1] == ("/Dev1/ao/StartTrigger", "RISING")
    starts = [e for e in bench.events if e[0] == "start"]
    assert starts[:2] == [("start", "Dev1/ai0"), ("start", "Dev1/ao0")]   # armed, then the output
    assert bench.outputs["Dev1/ao0"] == 0.0 and daq._ao_waveforms == set()
    bench.read_error = RuntimeError("acquisition failed")
    with pytest.raises(RuntimeError, match="acquisition failed"):
        daq.stimulate_acquire("ao0", [0.0, 1.0], 200.0, [("ai0", 10.0, "default")], 100.0, 3)
    assert bench.outputs["Dev1/ao0"] == 0.0                 # reset although the read failed
    assert daq.close() == [] and bench.open_tasks == 0


def test_service_stimulate_acquire_checks_the_policy_and_saves_stimulus_and_response(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "s.csv")
    result = _go(service, "daq_stimulate_acquire", channel="lamp", waveform="square",
                 frequency_hz=10.0, amplitude_v=1.0, offset_v=2.0, channels=["photodiode"],
                 rate_hz=100.0, duration_s=0.2, file_path=path)
    assert result == {"daq_channel": "lamp:ao0>photodiode:ai2", "data_path": path,
                      "daq_value": 10.0, "daq_unit": "Hz"}
    rows = _csv(path)
    assert rows[0] == ["time_s", "photodiode:ai2_V", "photodiode:ai2_mW", "lamp:ao0_set_V"]
    assert [r[3] for r in rows[1:11]] == ["1.000000"] * 5 + ["3.000000"] * 5   # one 10 Hz cycle
    assert backend.stimuli == [{"channel": "ao0", "points": 1000, "rate_hz": 10000.0,
                                "inputs": ["ai2"]}]
    assert backend.analog_outputs == {"ao0": 0.0} and backend.waveforms == {}
    for params in ({"channel": "ao1"}, {"offset_v": 4.5}, {"amplitude_v": 3.0}):
        with pytest.raises(DaqPolicyError):
            _go(service, "daq_stimulate_acquire", **{
                "channel": "lamp", "frequency_hz": 10.0, "amplitude_v": 1.0, "offset_v": 2.0,
                "channels": ["ai0"], **params})


def test_real_timed_digital_input_and_change_detection(bench):
    daq = NIDaq()
    bench.inputs["Dev1/port0/line1"] = [True, False, True]
    assert daq.acquire_digital(["port0/line0", "port0/line1"], 1000.0, 3) == [
        [False, False, False], [True, False, True]]
    daq.acquire_digital(["port0/line0"], 1000.0, 3, clock="pfi1")
    assert bench.clock_sources[-1] == "/Dev1/PFI1"
    bench.inputs["Dev1/port0/line2"] = True
    assert daq.wait_digital_change(["port0/line2"], "rising", 1.0) == [True]
    assert bench.change_detection[-1] == ("Dev1/port0/line2", "", "CONTINUOUS")
    daq.wait_digital_change(["port0/line2", "port0/line3"], "both", 1.0)
    assert bench.change_detection[-1][:2] == ("Dev1/port0/line2,Dev1/port0/line3",) * 2
    bench.read_error = FakeDaqError(-200284)
    with pytest.raises(TimeoutError, match="no falling change on port0/line2 within 0.5 s"):
        daq.wait_digital_change(["port0/line2"], "falling", 0.5)
    for call in (lambda: daq.acquire_digital(["port1/line0"], 1000.0, 3),
                 lambda: daq.wait_digital_change(["port2/line0"]),
                 lambda: daq.acquire_digital(["port0/line0", "port0/line0"], 1000.0, 3)):
        with pytest.raises(DaqPolicyError):
            call()
    daq.write_digital("port0/line4", True)
    with pytest.raises(RuntimeError, match="being driven as an output"):
        daq.acquire_digital(["port0/line4"], 1000.0, 3)
    assert bench.open_tasks == 1 and daq.close() == []


def test_real_digital_pattern_plays_records_and_leaves_the_lines_low(bench):
    daq = NIDaq()
    daq.write_digital("port0/line0", True)                  # a static task on a pattern line
    bench.inputs["Dev1/port0/line2"] = [False, True, True]
    recorded = daq.write_digital_pattern(["port0/line0", "port0/line1"],
                                         [[1, 0], [0, 1], [1, 1]], 10.0, record=["port0/line2"])
    assert recorded == [[False, True, True]]
    assert ("write", "Dev1/port0/line1", [[True, False, True], [False, True, True]]) in bench.events
    assert bench.clock_sources[-2:] == ["/Dev1/do/SampleClock", None]     # the response, then the pattern
    assert bench.outputs["Dev1/port0/line0"] is False and bench.outputs["Dev1/port0/line1"] is False
    assert sorted(daq._do_tasks) == ["port0/line0", "port0/line1"] and daq._pattern_task is None
    assert daq.write_digital_pattern(["port0/line0"], [[1], [0]], 10.0) is None
    for lines, rows, kwargs in ((["port0/line0"], [[1]], {}),
                                (["port0/line0"], [[1, 0], [0, 1]], {}),
                                (["port0/line0"], [[1], [0]], {"record": ["port0/line0"]})):
        with pytest.raises(ValueError):
            daq.write_digital_pattern(lines, rows, 10.0, **kwargs)
    with pytest.raises(DaqPolicyError):
        daq.write_digital_pattern(["port1/line0"], [[1], [0]], 10.0)
    bench.fail_start.add("Dev1/port0/line3")
    with pytest.raises(RuntimeError, match="start failed"):
        daq.write_digital_pattern(["port0/line3"], [[1], [0]], 10.0)
    assert bench.outputs["Dev1/port0/line3"] is False       # left low although the start failed
    assert daq.close() == [] and bench.open_tasks == 0
    with pytest.raises(RuntimeError, match="closed"):
        daq.write_digital_pattern(["port0/line0"], [[1], [0]], 10.0)


def test_close_ends_a_pattern_that_is_still_playing(bench):
    import threading
    daq = NIDaq()
    playing, release = threading.Event(), threading.Event()
    import nidaqmx

    def wait_until_done(self, timeout=10.0):
        playing.set()
        release.wait(5.0)

    nidaqmx.Task.wait_until_done = wait_until_done
    worker = threading.Thread(target=daq.write_digital_pattern,
                              args=(["port0/line0"], [[1], [1]], 10.0))
    worker.start()
    assert playing.wait(5.0)
    with pytest.raises(RuntimeError, match="playing a digital pattern"):
        daq.write_digital("port0/line0", True)
    assert daq.close() == []                                # does not wait for the pattern
    assert bench.outputs["Dev1/port0/line0"] is False
    release.set()
    worker.join(5.0)
    assert not worker.is_alive() and bench.open_tasks == 0


def test_service_digital_pattern_needs_every_line_enabled_and_saves_the_response(tmp_path):
    config = {**ENABLED, "daq_do_lines": ["port0/line3", "port0/line4"]}
    service, backend = _service(config)
    path = str(tmp_path / "pattern.csv")
    result = _go(service, "daq_write_digital_pattern", lines=["valve", "port0/line4"],
                 pattern=["10", "01", "00"], rate_hz=10.0, record=["port0/line1"], file_path=path)
    assert result == {"daq_channel": "valve:port0/line3+port0/line4>port0/line1", "data_path": path}
    assert _csv(path) == [["time_s", "valve:port0/line3_set", "port0/line4_set", "port0/line1"],
                          ["0.000000", "1", "0", "0"], ["0.100000", "0", "1", "0"],
                          ["0.200000", "0", "0", "0"]]
    assert backend.patterns[0]["rows"] == [[True, False], [False, True], [False, False]]
    assert backend.digital_outputs == {"port0/line3": False, "port0/line4": False}
    assert _go(service, "daq_write_digital_pattern", lines=["valve"], pattern=["1", "0"],
               rate_hz=10.0) == {"daq_channel": "valve:port0/line3"}
    with pytest.raises(DaqPolicyError, match="daq_do_lines"):   # one line not enabled: nothing plays
        _go(service, "daq_write_digital_pattern", lines=["valve", "port0/line5"],
            pattern=["10", "01"], rate_hz=10.0)
    for pattern, rate in ((["1"], 10.0), (["12", "00"], 10.0), (["1", "0"], 0.01)):
        with pytest.raises(ValueError):
            _go(service, "daq_write_digital_pattern", lines=["valve"], pattern=pattern, rate_hz=rate)
    assert len(backend.patterns) == 2


def test_service_digital_acquisition_and_change_wait(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "d.csv")
    result = _go(service, "daq_acquire_digital", lines=["port0/line1", "port0/line2"],
                 rate_hz=100.0, duration_s=0.03, clock="flow_meter", file_path=path)
    assert result == {"daq_channel": "port0/line1+port0/line2", "data_path": path}
    assert _csv(path)[:2] == [["time_s", "port0/line1", "port0/line2"], ["0.000000", "0", "0"]]
    assert _go(service, "daq_wait_digital_change", lines=["port0/line1"], edge="rising") == {
        "daq_channel": "port0/line1", "digital_state": True}
    assert _go(service, "daq_wait_digital_change", lines=["port0/line1", "port0/line2"]) == {
        "daq_channel": "port0/line1+port0/line2"}
    with pytest.raises(DaqPolicyError, match="only port0"):
        _go(service, "daq_acquire_digital", lines=["port1/line0"])


def test_real_time_measurements_set_their_terminals_and_time_out(bench):
    daq = NIDaq()
    bench.counter_value = 2.5e-4
    assert daq.measure_pulse_width("pfi1", 1e-6, 0.1, "low") == 2.5e-4
    width = bench.counters[-1]
    assert (width["kind"], width["starting_edge"], width["chan"].ci_pulse_width_term) == (
        "ci_width", "FALLING", "/Dev1/PFI1")
    assert daq.measure_period("pfi2", edge="falling") == 2.5e-4
    assert bench.counters[-1]["chan"].ci_period_term == "/Dev1/PFI2"
    assert daq.measure_edge_separation("pfi3", "pfi4", second_edge="falling", counter="ctr1") == 2.5e-4
    gap = bench.counters[-1]
    assert (gap["phys"], gap["first_edge"], gap["second_edge"]) == ("Dev1/ctr1", "RISING", "FALLING")
    assert (gap["chan"].ci_two_edge_sep_first_term, gap["chan"].ci_two_edge_sep_second_term) == (
        "/Dev1/PFI3", "/Dev1/PFI4")
    bench.read_error = FakeDaqError(-200474)
    with pytest.raises(TimeoutError, match="no pulse on pfi1 within 0.5 s"):
        daq.measure_pulse_width("pfi1", timeout_s=0.5)
    with pytest.raises(ValueError):
        daq.measure_period("pfi1", min_s=1.0, max_s=0.1)
    with pytest.raises(ValueError):
        daq.measure_pulse_width("pfi1", level="up")
    assert bench.open_tasks == 0


def test_real_encoder_holds_a_counter_until_it_is_stopped(bench):
    daq = NIDaq()
    assert daq.start_encoder("pfi8", "pfi10", 360, "x2") == "ctr3"
    encoder = bench.counters[-1]
    assert (encoder["kind"], encoder["decoding_type"], encoder["pulses_per_rev"]) == (
        "ci_encoder", "X_2", 360)
    assert (encoder["chan"].ci_encoder_a_input_term, encoder["chan"].ci_encoder_b_input_term) == (
        "/Dev1/PFI8", "/Dev1/PFI10")
    bench.counter_value = -45.5
    assert daq.read_encoder() == -45.5
    with pytest.raises(RuntimeError, match="already being followed"):
        daq.start_encoder("pfi8", "pfi10", 360)
    with pytest.raises(RuntimeError, match="following the encoder"):
        daq.count_edges("pfi0", 0.1, counter="ctr3", sleep=lambda s: None)
    with pytest.raises(RuntimeError, match="following the encoder"):
        daq.start_pulses("ctr3", 10.0)
    daq.count_edges("pfi0", 0.1, sleep=lambda s: None)      # an automatic choice avoids it
    assert bench.counters[-1]["phys"] == "Dev1/ctr2"
    assert daq.stop_encoder() is True and daq.stop_encoder() is False
    with pytest.raises(RuntimeError, match="no encoder"):
        daq.read_encoder()
    for args in (("pfi8", "pfi8", 360), ("pfi8", "pfi10", 0), ("pfi8", "pfi10", 360, "x8")):
        with pytest.raises(ValueError):
            daq.start_encoder(*args)
    daq.start_encoder("pfi8", "pfi10", 360)
    assert daq.close() == [] and bench.open_tasks == 0      # close() releases it


def test_service_time_measurements_and_encoder():
    service, backend = _service()
    assert _go(service, "daq_measure_pulse_width", terminal="flow_meter") == {
        "daq_channel": "flow_meter:pfi4", "daq_value": 0.0005, "daq_unit": "s"}
    assert _go(service, "daq_measure_period", terminal="pfi0")["daq_value"] == 0.001
    assert _go(service, "daq_measure_edge_separation", first="pfi0", second="flow_meter") == {
        "daq_channel": "pfi0>flow_meter:pfi4", "daq_value": 0.00025, "daq_unit": "s"}
    with pytest.raises(RuntimeError, match="no encoder"):
        _go(service, "daq_read_encoder")
    assert _go(service, "daq_start_encoder", a="pfi8", b="pfi10", pulses_per_rev=1000) == {
        "daq_channel": "pfi8/pfi10", "daq_value": 0.0, "daq_unit": "deg"}
    assert _go(service, "daq_read_encoder") == {
        "daq_channel": "pfi8/pfi10", "daq_value": 90.0, "daq_unit": "deg"}
    assert _go(service, "daq_stop_encoder")["daq_value"] == 90.0
    with pytest.raises(RuntimeError, match="no encoder"):
        _go(service, "daq_stop_encoder")
    _go(service, "daq_start_encoder", a="pfi8", b="pfi10", pulses_per_rev=1000)
    assert service.close() == [] and backend.encoder is None


@pytest.mark.parametrize("step", [
    {"action": "daq_acquire", "channels": ["ai0"], "trigger_level": 1.0},
    {"action": "daq_acquire", "channels": ["ai0", "ai1"], "trigger": "ai1", "trigger_level": 1.0},
    {"action": "daq_acquire", "channels": ["ai0"], "trigger": "ai0", "trigger_level": 11.0},
    {"action": "daq_acquire", "channels": ["ai0"], "pretrigger_s": 0.1},
    {"action": "daq_acquire", "channels": ["ai0"], "trigger": "pfi0", "pretrigger_s": 1.0},
    {"action": "daq_acquire", "channels": ["ai0"], "clock": "ai1"},
    {"action": "daq_stimulate_acquire", "channel": "ai0", "frequency_hz": 10.0, "amplitude_v": 1.0,
     "channels": ["ai1"]},
    {"action": "daq_stimulate_acquire", "channel": "ao0", "frequency_hz": 10.0, "amplitude_v": 1.0,
     "channels": []},
    {"action": "daq_stimulate_acquire", "channel": "ao0", "frequency_hz": 10.0, "channels": ["ai0"]},
    {"action": "daq_acquire_digital", "lines": ["port0/line0", "port0/line0"]},
    {"action": "daq_acquire_digital", "lines": ["pfi0"]},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["1"], "rate_hz": 10.0},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["10", "01"],
     "rate_hz": 10.0},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["1", "x"],
     "rate_hz": 10.0},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["1", "0"],
     "rate_hz": 0.01},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["1", "0"],
     "rate_hz": 10.0, "record": ["port0/line0"]},
    {"action": "daq_write_digital_pattern", "lines": ["port0/line0"], "pattern": ["1", "0"]},
    {"action": "daq_wait_digital_change", "lines": ["port0/line0"], "edge": "high"},
    {"action": "daq_wait_digital_change", "lines": []},
    {"action": "daq_measure_pulse_width", "terminal": "pfi0", "level": "up"},
    {"action": "daq_measure_period", "terminal": "pfi0", "min_s": 1.0, "max_s": 0.5},
    {"action": "daq_measure_edge_separation", "first": "pfi0"},
    {"action": "daq_start_encoder", "a": "pfi8", "b": "pfi8", "pulses_per_rev": 360},
    {"action": "daq_start_encoder", "a": "pfi8", "b": "pfi10"},
    {"action": "daq_start_encoder", "a": "pfi8", "b": "pfi10", "pulses_per_rev": 360, "decoding": "x8"},
    {"action": "daq_read_encoder", "counter": "ctr0"},
])
def test_schema_rejects_new_steps(step):
    with pytest.raises(ValidationError):
        _wf(step)


def test_schema_accepts_an_analog_trigger_on_the_first_channel():
    step = _wf({"action": "daq_acquire", "channels": ["photodiode", "ai1"], "trigger": "photodiode",
                "trigger_level": 0.5, "pretrigger_s": 0.2, "clock": "pfi1"}).steps[0]
    assert (step.trigger_level, step.pretrigger_s, step.clock) == (0.5, 0.2, "pfi1")


def test_a_failed_pattern_setup_still_gets_a_previously_high_line_low(bench):
    daq = NIDaq()
    daq.write_digital("port0/line0", True)                  # a valve held open
    import nidaqmx
    original = nidaqmx.Task.write

    def write(self, data, auto_start=None):
        if isinstance(data, list):
            raise RuntimeError("buffer refused")            # the buffered write of the pattern
        return original(self, data, auto_start=auto_start)

    nidaqmx.Task.write = write
    with pytest.raises(RuntimeError, match="buffer refused"):
        daq.write_digital_pattern(["port0/line0", "port0/line1"], [[1, 0], [0, 1]], 10.0)
    assert bench.outputs["Dev1/port0/line0"] is False and bench.outputs["Dev1/port0/line1"] is False
    assert daq._do_dirty == set() and daq.close() == []


def test_pattern_lines_that_cannot_be_set_low_are_retried_and_reported(bench):
    daq = NIDaq()
    daq.write_digital("port0/line0", False)                 # known low before the pattern
    bench.fail_write.add("Dev1/port0/line0")
    original_write = None
    import nidaqmx
    original_write = nidaqmx.Task.write

    def write(self, data, auto_start=None):                 # the pattern itself may be written
        if isinstance(data, list):
            bench.events.append(("write", self.phys, data))
            return None
        return original_write(self, data, auto_start=auto_start)

    nidaqmx.Task.write = write
    with pytest.raises(RuntimeError, match="did not end cleanly"):
        daq.write_digital_pattern(["port0/line0", "port0/line1"], [[1, 1], [1, 1]], 10.0)
    assert bench.outputs["Dev1/port0/line1"] is False       # the other line was still set low
    assert daq._do_dirty == {"port0/line0"}                 # not forgotten
    assert daq.close() == ["port0/line0"]                   # and reported, not passed as low


@pytest.mark.parametrize("name, kinds", [
    ("ai16", "ai"), ("ao2", "ao"), ("port0/line8", "line"), ("port3/line0", "line"),
    ("port0/line99", "line"), ("ctr4", "counter"), ("pfi16", "terminal")])
def test_channels_the_device_does_not_have_are_refused_everywhere(name, kinds):
    from src.devices.nidaq.policy import check_flow_name
    with pytest.raises(ValueError):
        check_flow_name(name, kinds)
    with pytest.raises(DaqPolicyError):
        DaqSignals.from_config({}).resolve(name, kinds)
    with pytest.raises(DaqPolicyError):
        DaqSignals.from_config({"daq_signals": {"thing": name}})


def test_pattern_csv_uses_the_clock_the_device_really_set(bench, tmp_path):
    bench.coerced_rate = 8.0
    service = DaqService(NIDaq(), {**ENABLED, "daq_do_lines": ["port0/line3"]})
    path = str(tmp_path / "p.csv")
    _go(service, "daq_write_digital_pattern", lines=["valve"], pattern=["1", "0", "1"],
        rate_hz=10.0, record=["port0/line1"], file_path=path)
    assert [row[0] for row in _csv(path)[1:]] == ["0.000000", "0.125000", "0.250000"]
    assert service.close() == []


def test_stimulus_column_does_not_slip_at_exact_sample_boundaries(tmp_path):
    service, backend = _service()
    path = str(tmp_path / "s.csv")
    _go(service, "daq_stimulate_acquire", channel="lamp", waveform="sawtooth", frequency_hz=1.0,
        amplitude_v=1.0, offset_v=1.0, channels=["ai0"], rate_hz=1000.0, duration_s=1.5,
        file_path=path)
    rows = _csv(path)[1:]
    from src.devices.nidaq.service import waveform_period
    period = waveform_period("sawtooth", 1.0, 1.0, 1000)
    assert all(float(row[-1]) == pytest.approx(period[i % 1000], abs=1e-6)
               for i, row in enumerate(rows))


# ----------------------------------------------------------------------
# Device profiles: one table entry per model (USB-6351, USB-6002)
# ----------------------------------------------------------------------
from src.devices.nidaq import profiles as daq_profiles  # noqa: E402
from src.devices.nidaq import probe as daq_probe  # noqa: E402

U6002 = {"daq_model": "USB-6002"}
U6002_OUT = {"daq_model": "USB-6002", "daq_ao_channels": ["ao0", "ao1"],
             "daq_do_lines": ["port0/line0"], "daq_ao_min_v": 0.0, "daq_ao_max_v": 5.0}


def _service_6002(config=None):
    backend = MockNIDaq(model="USB-6002")
    return DaqService(backend, U6002 if config is None else config, mock=True), backend


@pytest.mark.parametrize("name", ["USB-6002", "usb6002", "NI USB-6002", " usb-6002 "])
def test_profile_lookup_accepts_the_usual_spellings(name):
    assert daq_profiles.get_profile(name) is daq_profiles.USB_6002


def test_profile_default_and_unknown_model():
    assert daq_profiles.get_profile(None) is daq_profiles.USB_6351
    assert daq_profiles.get_profile("") is daq_profiles.USB_6351
    with pytest.raises(daq_profiles.DaqProfileError, match="USB-6002, USB-6351"):
        daq_profiles.get_profile("USB-6999")


@pytest.mark.parametrize("name, kind_6351, kind_6002", [
    ("ai7", "ai", "ai"), ("ai8", "ai", None), ("ai16", None, None),
    ("ao1", "ao", "ao"), ("ao2", None, None),
    ("port0/line7", "line", "line"), ("port1/line3", "line", "line"),
    ("port1/line4", "line", None), ("port2/line0", "line", "line"),
    ("port2/line1", "line", None), ("port3/line0", None, None),
    ("ctr0", "counter", "counter"), ("ctr1", "counter", None),
    ("pfi1", "terminal", "terminal"), ("pfi2", "terminal", None),
    ("100khztimebase", "terminal", None),
])
def test_profiles_know_which_channels_each_model_has(name, kind_6351, kind_6002):
    assert daq_profiles.USB_6351.kind_of(name) == kind_6351
    assert daq_profiles.USB_6002.kind_of(name) == kind_6002
    # the schema, which does not know the model, accepts what some model has
    assert daq_policy.physical_kind(name) == (kind_6351 or kind_6002)


def test_every_feature_and_action_in_the_tables_is_known():
    assert set(daq_profiles.ACTION_FEATURES) <= set(DAQ_ACTION_NAMES)
    assert {a for a, _ in daq_profiles.OPTION_FEATURES} <= set(DAQ_ACTION_NAMES)
    used = set(daq_profiles.ACTION_FEATURES.values()) | set(daq_profiles.OPTION_FEATURES.values())
    assert used | {daq_profiles.THERMOCOUPLE, daq_profiles.RTD} == set(daq_profiles.FEATURE_LABEL)
    assert daq_profiles.USB_6351.features == frozenset(daq_profiles.FEATURE_LABEL)
    for step in DAQ_ACTION_NAMES:                       # the first model runs every step
        daq_profiles.USB_6351.check_step(step, {})


def test_policy_of_the_6002_refuses_what_it_does_not_have():
    p = daq_profiles.USB_6002
    DaqOutputPolicy.from_config({"daq_ao_channels": ["ao0", "ao1"],
                                 "daq_do_lines": ["port1/line3", "port2/line0"]}, p)
    with pytest.raises(DaqPolicyError, match="USB-6002 has no digital line port1/line4"):
        DaqOutputPolicy.from_config({"daq_do_lines": ["port1/line4"]}, p)
    with pytest.raises(DaqPolicyError, match="cannot output pulse trains"):
        DaqOutputPolicy.from_config({"daq_co_counters": ["ctr0"]}, p)
    with pytest.raises(DaqPolicyError, match="USB-6002 has no counter ctr1"):
        DaqOutputPolicy.from_config({"daq_co_counters": ["ctr1"]}, p)
    # the unused frequency limit of a config written for another model does not matter
    assert DaqOutputPolicy.from_config({"daq_co_max_hz": 1000.0}, p).co_counters == frozenset()
    with pytest.raises(DaqPolicyError, match="no hardware-timed"):
        DaqOutputPolicy.from_config({"daq_do_lines": ["port0/line0"]}, p).check_digital_pattern(
            ["port0/line0"])


@pytest.mark.parametrize("spec, message", [
    ({"channel": "ai8"}, "USB-6002 has no analog input ai8"),
    ({"channel": "pfi2"}, "USB-6002 has no terminal pfi2"),
    ({"channel": "100khztimebase"}, "USB-6002 has no terminal"),
    ({"channel": "ai0", "terminal": "nrse"}, "terminal must be one of"),
    ({"channel": "ai4", "terminal": "diff"}, "cannot be read differentially"),
    ({"channel": "ai0", "v_range": 5}, "v_range must be one of"),
    ({"channel": "ai0", "sensor": "thermocouple", "tc_type": "K", "cjc_c": 25.0},
     "does not support thermocouple"),
    ({"channel": "ai0", "sensor": "rtd", "excitation_a": 0.001}, "does not support RTD"),
])
def test_signals_of_the_6002_are_checked_against_its_profile(spec, message):
    with pytest.raises(DaqPolicyError, match=message):
        DaqSignals({"probe": spec}, daq_profiles.USB_6002)
    if spec["channel"] != "ai4":
        DaqSignals({"probe": spec})                         # fine on the USB-6351


def test_signals_of_the_6002_that_it_has():
    signals = DaqSignals({"sensor_v": {"channel": "ai3", "terminal": "diff", "v_range": 10,
                                       "scale": 2.0, "unit": "bar"},
                          "valve": "port1/line3", "flow": "pfi1"}, daq_profiles.USB_6002)
    assert signals.resolve("sensor_v", "ai").channel == "ai3"
    assert signals.resolve("ai7", "ai").channel == "ai7"
    with pytest.raises(DaqPolicyError, match="USB-6002 has no analog input ai12"):
        signals.resolve("ai12", "ai")


def test_differential_pairs_are_checked_on_the_6351_too():
    service, _ = _service()
    with pytest.raises(DaqPolicyError, match="ai8 cannot be read differentially"):
        _go(service, "daq_read_analog", channel="ai8", terminal="diff")
    assert _go(service, "daq_read_analog", channel="ai7", terminal="diff")["voltage"] == 1.234


@pytest.mark.parametrize("action, params", [
    ("daq_measure_frequency", {"terminal": "pfi0"}),
    ("daq_measure_period", {"terminal": "pfi0"}),
    ("daq_measure_pulse_width", {"terminal": "pfi0"}),
    ("daq_measure_edge_separation", {"first": "pfi0", "second": "pfi1"}),
    ("daq_start_encoder", {"a": "pfi0", "b": "pfi1", "pulses_per_rev": 360}),
    ("daq_read_encoder", {}),
    ("daq_stop_encoder", {}),
    ("daq_generate_pulses", {"counter": "ctr0", "frequency_hz": 10.0}),
    ("daq_acquire_digital", {"lines": ["port0/line0"], "rate_hz": 1000.0, "duration_s": 0.1}),
    ("daq_write_digital_pattern", {"lines": ["port0/line0"], "pattern": ["1", "0"],
                                   "rate_hz": 10.0}),
    ("daq_wait_digital_change", {"lines": ["port0/line0"]}),
    ("daq_acquire", {"channels": ["ai0"], "trigger": "ai0", "trigger_level": 1.0}),
    ("daq_acquire", {"channels": ["ai0"], "trigger": "pfi0", "pretrigger_s": 0.1}),
    ("daq_acquire", {"channels": ["ai0"], "clock": "pfi0"}),
])
def test_the_6002_refuses_steps_it_cannot_run_before_touching_the_device(action, params):
    service, backend = _service_6002(U6002_OUT)
    called = []
    for name in [n for n in dir(backend) if not n.startswith("_") and callable(getattr(backend, n))]:
        original = getattr(backend, name)
        setattr(backend, name, lambda *a, _n=name, _f=original, **k: (called.append(_n), _f(*a, **k))[1])
    with pytest.raises(DaqPolicyError, match="the USB-6002 does not support"):
        _go(service, action, **params)
    assert called == []
    assert service.unsupported_steps([{"action": action, **params}])[0].startswith("step 1: ")


def test_the_6002_runs_the_steps_it_has(tmp_path):
    service, backend = _service_6002(U6002_OUT)
    assert service.profile is daq_profiles.USB_6002 and backend.product_type == "USB-6002 (mock)"
    assert _go(service, "daq_read_analog", channel="ai7", terminal="rse")["voltage"] == 1.234
    _go(service, "daq_write_analog", channel="ao1", voltage=2.0)
    _go(service, "daq_write_digital", line="port0/line0", state=True)
    _go(service, "daq_pulse_digital", line="port0/line0", duration_s=0.01)
    assert _go(service, "daq_read_digital", line="port2/line0")["digital_state"] in (True, False)
    path = str(tmp_path / "wave.csv")
    _go(service, "daq_acquire", channels=["ai0", "ai1"], rate_hz=25000.0, duration_s=0.01,
        trigger="pfi1", file_path=path)
    assert len(_csv(path)) == 251
    _go(service, "daq_count_edges", terminal="pfi0", duration_s=0.01)
    _go(service, "daq_generate_waveform", channel="ao0", waveform="sine", frequency_hz=10.0,
        amplitude_v=1.0, offset_v=2.0)
    assert backend.waveforms["ao0"]["rate_hz"] == 5000.0    # 500 points per cycle, not 1000
    _go(service, "daq_stop_output", channel="ao0")
    _go(service, "daq_stimulate_acquire", channel="ao0", waveform="triangle", frequency_hz=10.0,
        amplitude_v=1.0, offset_v=2.0, channels=["ai0"], rate_hz=5000.0, duration_s=0.1,
        file_path=str(tmp_path / "stim.csv"))
    assert service.close() == []


@pytest.mark.parametrize("params, message", [
    ({"channels": ["ai0"], "rate_hz": 60000.0, "duration_s": 0.01}, "limit of the USB-6002"),
    ({"channels": ["ai0", "ai1"], "rate_hz": 30000.0, "duration_s": 0.01}, "limit of the USB-6002"),
    ({"channels": ["ai0"], "v_range": 5.0}, "v_range must be one of"),
    ({"channels": ["ai0"], "terminal": "nrse"}, "terminal must be one of"),
    ({"channels": ["ai4"], "terminal": "diff"}, "cannot be read differentially"),
    ({"channels": ["ai8"]}, "USB-6002 has no analog input ai8"),
    ({"channels": ["ai0"], "trigger": "pfi5"}, "USB-6002 has no terminal pfi5"),
])
def test_the_6002_limits_apply_to_an_acquisition(params, message, tmp_path):
    service, _ = _service_6002()
    with pytest.raises(ValueError, match=message):
        _go(service, "daq_acquire", file_path=str(tmp_path / "x.csv"), **params)


def test_the_6002_waveform_frequency_is_bounded_by_its_update_rate():
    service, _ = _service_6002(U6002_OUT)
    with pytest.raises(ValueError, match="too high"):
        _go(service, "daq_generate_waveform", channel="ao0", waveform="sine",
            frequency_hz=500.0, amplitude_v=1.0, offset_v=2.0)     # 10 points per cycle
    _go(service, "daq_generate_waveform", channel="ao0", waveform="sine",
        frequency_hz=250.0, amplitude_v=1.0, offset_v=2.0)         # 20 points per cycle


def test_real_driver_takes_the_profile_from_the_device(bench):
    bench.product_type = "USB-6002"
    daq = NIDaq("Dev1")
    assert daq.profile is daq_profiles.USB_6351             # until the device has been asked
    assert daq.connect() is True and daq.profile is daq_profiles.USB_6002
    with pytest.raises(ValueError, match="limit of the USB-6002"):
        daq.acquire([("ai0", 10.0, "rse")], 60000.0, 10)
    with pytest.raises(ValueError, match="rate_hz"):
        daq.start_waveform("ao0", [0.0, 1.0], 6000.0)
    with pytest.raises(ValueError, match="the device has"):
        daq.count_edges("pfi0", 0.01, counter="ctr1")
    assert bench.outputs == {}
    daq.close()


def test_real_driver_refuses_a_model_without_a_profile(bench, caplog):
    bench.product_type = "USB-6999"
    daq = NIDaq("Dev1")
    assert daq.connect() is False
    assert "has no profile" in caplog.text and "profiles.py" in caplog.text


def _shared_real(config):
    async def go():
        shared = SharedDevices(use_daq=True, daq_device="Dev1", **config)
        ok = await shared.initialize()
        return shared, ok
    return asyncio.run(go())


def test_real_session_stops_when_config_names_another_model(bench, caplog):
    bench.product_type = "USB-6351"
    shared, ok = _shared_real({"daq_model": "USB-6002"})
    assert ok is False and "means USB-6002, but the device is a USB-6351" in caplog.text
    assert bench.outputs == {} and bench.open_tasks == 0


def test_real_session_on_a_6002_refuses_what_the_model_lacks(bench):
    bench.product_type = "USB-6002"
    shared, ok = _shared_real({"daq_model": "USB-6002"})
    assert ok is True and shared.daq_service.profile is daq_profiles.USB_6002

    async def go():
        with pytest.raises(DaqPolicyError, match="USB-6002 does not support"):
            await shared.daq_step("daq_measure_period", terminal="pfi0")
        with pytest.raises(DaqPolicyError, match="daq_ao_channels"):
            await shared.daq_step("daq_write_analog", channel="ao0", voltage=1.0)
        await shared.cleanup()
    asyncio.run(go())
    assert bench.outputs == {} and bench.counters == []


def test_a_config_for_the_6351_is_refused_by_a_6002(bench, caplog):
    bench.product_type = "USB-6002"
    shared, ok = _shared_real({"daq_model": "USB-6002", "daq_co_counters": ["ctr0"]})
    assert ok is False and "cannot output pulse trains" in caplog.text


def test_a_blank_daq_model_means_the_default_model_on_the_bench_too(bench, caplog):
    """--mock imitates a USB-6351 when daq_model is empty; a real USB-6002 must then not
    start, or a flow could pass --mock and be refused on the bench."""
    bench.product_type = "USB-6002"
    shared, ok = _shared_real({})
    assert ok is False
    assert "set daq_model: USB-6002 in config.yaml" in caplog.text
    assert bench.outputs == {} and bench.open_tasks == 0


def test_real_driver_refuses_a_device_without_a_product_type(bench, caplog):
    bench.product_type = " "
    assert NIDaq("Dev1").connect() is False and "no product type" in caplog.text


@pytest.mark.parametrize("name", ["ai00", "ai07", "pfi00", "pfi01", "port0/line00",
                                  "port2/line04", "ctr00", "ao00"])
def test_padded_channel_numbers_are_not_names(name):
    """DAQmx refuses them, and "port2/line04" would slip past the pin-conflict table."""
    assert daq_policy.physical_kind(name) is None
    assert daq_profiles.USB_6351.kind_of(name) is None
    with pytest.raises(ValueError):
        daq_policy.check_flow_name(name, daq_policy.KINDS)
    with pytest.raises(DaqPolicyError):
        DaqSignals({"x": name})
    if name.startswith("port"):
        with pytest.raises(DaqPolicyError):
            DaqOutputPolicy.from_config({"daq_do_lines": [name]})


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_an_averaged_reading_respects_the_model_rate_limit(bench, make):
    daq = make("Dev1") if make is NIDaq else make(model="USB-6002")
    if make is NIDaq:
        bench.product_type = "USB-6002"
        assert daq.connect()
    with pytest.raises(ValueError, match="limit of the USB-6002"):
        daq.read_analog("ai0", samples=100, rate_hz=60000.0, terminal="rse")
    daq.read_analog("ai0", samples=1, rate_hz=60000.0, terminal="rse")   # no clock is used
    daq.read_analog("ai0", samples=100, rate_hz=50000.0, terminal="rse")
    daq.close()


@pytest.mark.parametrize("model, pfi, line", [("USB-6002", "pfi0", "port2/line0"),
                                              ("USB-6002", "pfi1", "port1/line1"),
                                              ("USB-6351", "pfi0", "port1/line0"),
                                              ("USB-6351", "pfi12", "port2/line4")])
@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_a_pfi_is_no_input_while_its_pin_is_driven_as_a_digital_output(bench, make, model,
                                                                      pfi, line):
    bench.product_type = model
    daq = make("Dev1") if make is NIDaq else make(model=model)
    assert daq.connect()
    daq.count_edges(pfi, 0.001)                             # free pin: fine
    daq.write_digital(line, True)
    with pytest.raises(RuntimeError, match=f"{pfi} is the pin of {line}"):
        daq.count_edges(pfi, 0.001)
    with pytest.raises(RuntimeError, match=f"{pfi} is the pin of {line}"):
        daq.acquire([("ai0", 10.0, "rse")], 1000.0, 10, trigger=pfi)
    if make is NIDaq:
        assert bench.outputs[f"Dev1/{line}"] is True         # the output was not disturbed
    daq.stop_output(line)
    assert daq.close() == []


@pytest.mark.parametrize("make", [NIDaq, MockNIDaq])
def test_a_line_is_not_driven_while_the_encoder_listens_on_its_pin(bench, make):
    daq = make("Dev1") if make is NIDaq else make()
    assert daq.connect()
    daq.start_encoder("pfi8", "pfi10", 360)
    with pytest.raises(RuntimeError, match="encoder"):
        daq.write_digital("port2/line0", True)               # pfi8
    daq.write_digital("port2/line1", True)                   # pfi9: not an encoder pin
    daq.stop_encoder()
    daq.write_digital("port2/line0", True)
    assert daq.close() == []


def test_profiles_are_checked_for_consistency():
    import dataclasses
    base = daq_profiles.USB_6351
    with pytest.raises(ValueError, match="counter_output_lines"):
        dataclasses.replace(base, counter_output_lines={"ctr0": "port2/line4"})
    with pytest.raises(ValueError, match="pfi_lines"):
        dataclasses.replace(base, pfi_lines={})
    with pytest.raises(ValueError, match="not a digital line"):
        dataclasses.replace(daq_profiles.USB_6002, pfi_lines={"pfi0": "port2/line0",
                                                             "pfi1": "port1/line7"})
    with pytest.raises(ValueError, match="timed_port"):
        dataclasses.replace(daq_profiles.USB_6002, timed_port="port0")


def _use_config(monkeypatch, extra):
    shared = dict(run_flow.lab_config.get_shared_devices(), daq_signals={}, daq_model="",
                  daq_ao_channels=[], daq_do_lines=[], daq_co_counters=[])
    shared.update(extra)
    monkeypatch.setattr(run_flow.lab_config, "get_shared_devices", lambda: dict(shared))


def test_6002_input_example_runs_as_a_6002_and_as_a_6351(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "usb6002_inputs.json")
    assert run_flow.main([flow, "--validate-only"]) == 0
    for model in ("USB-6002", ""):
        _use_config(monkeypatch, {"daq_model": model})
        assert run_flow.main([flow, "--mock"]) == 0
        rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
        assert len(rows) == 9 and all(r["status"] == "ok" for r in rows)
    with open(flow, encoding="utf-8") as f:
        import json
        assert all(s["action"].startswith("daq_") for s in json.load(f)["steps"])


def test_a_flow_the_model_cannot_run_is_stopped_before_its_first_step(tmp_path, monkeypatch, caplog):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "timing_and_triggers.json")
    _use_config(monkeypatch, {"daq_model": "USB-6002"})
    assert run_flow.main([flow, "--mock"]) == 1
    assert _rows(_latest_run_dir(str(tmp_path / "logs"))) == []     # nothing was executed
    assert "step 1: daq_measure_period: the USB-6002 does not support" in caplog.text
    assert "step 5: daq_acquire: the USB-6002 does not support pre-trigger" in caplog.text


def test_6002_loopback_check_runs_only_with_its_outputs_enabled(tmp_path, monkeypatch):
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    flow = os.path.join(REPO_ROOT, "examples", "daq", "usb6002_loopback_check.json")
    assert run_flow.main([flow, "--validate-only"]) == 0
    _use_config(monkeypatch, {"daq_model": "USB-6002"})        # as shipped: refused at step 1
    assert run_flow.main([flow, "--mock"]) == 1
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert len(rows) == 1 and "daq_ao_channels" in rows[0]["error"]
    _use_config(monkeypatch, U6002_OUT)
    assert run_flow.main([flow, "--mock"]) == 0
    rows = _rows(_latest_run_dir(str(tmp_path / "logs")))
    assert len(rows) == 28 and all(r["status"] == "ok" for r in rows)


def test_probe_compares_a_device_report_with_its_profile():
    info = {"model": "USB-6002", "ai_channels": 8, "v_ranges": [10.0],
            "ai_max_rate_single_hz": 50000.0, "ai_max_rate_aggregate_hz": 50000.0,
            "ao_channels": 2, "ao_max_rate_hz": 5000.0, "port_lines": (8, 4, 1),
            "counters": 1, "pfi": 2, "timed_ports": []}
    assert daq_probe.compare(info)[0].startswith("the compared figures agree with the USB-6002")
    assert "counters: device reports 2" in daq_probe.compare({**info, "counters": 2})[0]
    assert "no profile for USB-6999" in daq_probe.compare({**info, "model": "USB-6999"})[0]
