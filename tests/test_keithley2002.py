"""Keithley 2002 DMM: driver SCPI, SharedDevices wrapper, flow wiring, logging.

No hardware and no PyVISA: the driver takes an injected resource manager, so
these tests also prove that the mock / CI path never imports pyvisa.
"""
import asyncio
import csv
import inspect
import json
import os
import threading
import time

import pytest
from pydantic import ValidationError

import src.devices.keithley2002 as k2002_pkg
from src.devices.keithley2002.keithley2002 import (
    FUNCTIONS, Keithley2002, Keithley2002Error, parse_error, parse_reading,
)
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import SHARED_DEVICE_ACTIONS, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import ActionMeasureDMM, ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(REPO_ROOT, "examples", "keithley2002", "measure_dc_voltage.json")
IDN_2002 = "KEITHLEY INSTRUMENTS INC.,MODEL 2002,1234567,A06  /A01"


# ----------------------------------------------------------------------
# Fakes: a VISA resource that records the bus traffic
# ----------------------------------------------------------------------
class FakeResource:
    def __init__(self, readings=(), errors=(), idn=IDN_2002):
        self.readings = list(readings)
        self.errors = list(errors)          # :SYST:ERR? replies before the queue is empty
        self.idn = idn
        self.log = []                       # ("w", cmd) / ("q", cmd) / "clear" / "gtl" / "close"
        self.closed = False
        self.io_after_close = False

    def write(self, command):
        self._check_open(command)
        self.log.append(("w", command))

    def _check_open(self, command):
        if self.closed:
            self.io_after_close = True
            raise AssertionError(f"I/O after close: {command!r}")

    def query(self, command):
        self._check_open(command)
        self.log.append(("q", command))
        if command == ":SYST:LSYN:STAT?":
            return "0"
        if command == ":SYST:ERR?":
            return self.errors.pop(0) if self.errors else '0,"No error"'
        if command == "*IDN?":
            return self.idn
        if command == ":READ?":
            return self.readings.pop(0) if self.readings else "+1.000000000E+00"
        if command.endswith(":RANG?"):
            return "+2.000000E+01"
        if command.endswith(":NPLC?"):
            return "+1.000000E+01"
        raise AssertionError(f"unexpected query {command!r}")

    def clear(self):
        self.log.append("clear")

    def control_ren(self, mode):
        self.log.append("gtl")

    def close(self):
        self.log.append("close")
        self.closed = True

    def writes(self):
        return [c for kind, c in (e for e in self.log if isinstance(e, tuple)) if kind == "w"]


class FakeRM:
    def __init__(self, resource):
        self.resource = resource
        self.opened = []

    def open_resource(self, name):
        self.opened.append(name)
        return self.resource


def make_driver(resource=None):
    resource = resource or FakeResource()
    return Keithley2002("GPIB0::16::INSTR", resource_manager=FakeRM(resource)), resource


# ----------------------------------------------------------------------
# 1. Driver: parsing
# ----------------------------------------------------------------------
@pytest.mark.parametrize("reply, expected", [
    ("+1.23456789E+00", 1.23456789),
    ("-1.2000000E-08\n", -1.2e-08),
    ("+0.000000000E+00", 0.0),
    # front-panel elements left on: take the reading, ignore units and the rest
    ("+1.23E+00VDC,+12.3SECS,+00000RDNG#", 1.23),
    ("+9.9E37", None),                     # overflow
    ("-9.9E37", None),
    ("NAN", None),
    ("", None),
    ("garbage", None),
])
def test_parse_reading(reply, expected):
    assert parse_reading(reply) == expected


def test_parse_error():
    assert parse_error('0,"No error"')[0] == 0
    assert parse_error('-113,"Undefined header"') == (-113, '-113,"Undefined header"')
    with pytest.raises(Keithley2002Error):
        parse_error("timeout?")


def test_identify_accepts_2002_and_rejects_other_models():
    dmm, _ = make_driver()
    assert dmm.identify() == IDN_2002
    other, _ = make_driver(FakeResource(idn="KEITHLEY INSTRUMENTS INC.,MODEL 2000,1,A01"))
    with pytest.raises(Keithley2002Error):
        other.identify()


# ----------------------------------------------------------------------
# 2. Driver: the SCPI sequence of configure()
# ----------------------------------------------------------------------
def test_configure_dc_voltage_autorange():
    dmm, res = make_driver()
    settings = dmm.configure("dc_voltage", nplc=10)
    assert res.writes() == [
        ":ABOR", ":CONF:VOLTage:DC", ":SENS:VOLTage:DC:RANG:AUTO ON",
        ":SENS:VOLTage:DC:NPLC 10", ":FORM:DATA ASC", ":FORM:ELEM READ", ":FORM:EXP HPR",
    ]
    assert ("q", ":SYST:ERR?") in res.log
    assert settings == {"function": "dc_voltage", "unit": "V", "auto_range": True}
    assert dmm.read_settings("dc_voltage") == {"range": 20.0, "nplc": 10.0}


def test_configure_gives_a_fresh_single_reading_whatever_state_was_left():
    """Stateful fake of the parts of the 2002 the driver relies on: front panel left
    in continuous initiation, infinite trigger count and filter on. :CONF resets
    these (manual p.3-42); only then does :READ? return a fresh single conversion."""
    class Stateful(FakeResource):
        def __init__(self):
            super().__init__()
            self.state = {"continuous": True, "trigger_count": "INF", "filter": True}

        def write(self, command):
            super().write(command)
            if command.startswith(":CONF:"):
                self.state = {"continuous": False, "trigger_count": 1, "filter": False}

        def query(self, command):
            if command == ":READ?":
                self._check_open(command)
                self.log.append(("q", command))
                fresh = self.state == {"continuous": False, "trigger_count": 1, "filter": False}
                return "+1.000000000E+00" if fresh else "+5.000000000E+00"   # stale / filtered
            return super().query(command)

    dmm, res = make_driver(Stateful())
    dmm.configure("dc_voltage")
    assert dmm.read() == 1.0


def test_read_settings_does_not_swallow_io_errors():
    """A timed-out query may leave its reply on the bus; carrying on would pair the
    next query with the wrong reply (a range read as a measurement)."""
    class Timeout(FakeResource):
        def query(self, command):
            if command.endswith(":RANG?"):
                raise TimeoutError("VI_ERROR_TMO")
            return super().query(command)

    dmm, _ = make_driver(Timeout())
    with pytest.raises(TimeoutError):
        dmm.read_settings("dc_voltage")


def test_line_sync_reads_the_global_state():
    dmm, _ = make_driver()
    assert dmm.line_sync() is False


@pytest.mark.parametrize("function", ["ac_voltage", "ac_current"])
def test_configure_sends_nplc_for_ac_functions_too(function):
    """2002 manual Table 3-10: NPLC 0.01-50 also for ACV and ACI."""
    dmm, res = make_driver()
    dmm.configure(function, nplc=2)
    scpi = FUNCTIONS[function][0]
    assert f":SENS:{scpi}:NPLC 2" in res.writes()


def test_ac_voltage_forces_volt_units():
    dmm, res = make_driver()
    dmm.configure("ac_voltage")
    assert ":UNIT:VOLT:AC V" in res.writes()
    dmm2, res2 = make_driver()
    dmm2.configure("dc_voltage")
    assert ":UNIT:VOLT:AC V" not in res2.writes()


def test_configure_fixed_range():
    dmm, res = make_driver()
    dmm.configure("resistance_4w", range_=2000.0)
    assert ":SENS:FRESistance:RANG 2000" in res.writes()
    assert not any("RANG:AUTO" in c for c in res.writes())


def test_conf_comes_first_so_trigger_state_is_known():
    """:CONF puts all three trigger layers on IMMediate, count 1, delay 0 (manual p.3-42)."""
    dmm, res = make_driver()
    dmm.configure("dc_current")
    assert res.writes()[:2] == [":ABOR", ":CONF:CURRent:DC"]


def test_instrument_errors_after_configure_raise():
    res = FakeResource(errors=['-222,"Parameter data out of range"', '+510,"Reading buffer data lost"'])
    dmm, _ = make_driver(res)
    with pytest.raises(Keithley2002Error, match="-222.*\\+510"):
        dmm.configure("dc_voltage")


@pytest.mark.parametrize("kwargs", [
    {"function": "temperature"}, {"nplc": 0.001}, {"nplc": 60},
    {"function": "ac_voltage", "range_": 1000}, {"range_": 0},
])
def test_bad_arguments_are_rejected_before_any_bus_traffic(kwargs):
    dmm, res = make_driver()
    with pytest.raises(ValueError):
        dmm.configure(**kwargs)
    assert res.log == []


def test_read_returns_none_on_overflow():
    dmm, _ = make_driver(FakeResource(readings=["+9.9E37"]))
    assert dmm.read() is None


def test_constructor_closes_the_handle_when_setup_fails():
    class Broken(FakeResource):
        def __setattr__(self, name, value):
            if name == "timeout":
                raise RuntimeError("bad session")
            super().__setattr__(name, value)

    res = Broken()
    with pytest.raises(RuntimeError):
        Keithley2002("GPIB0::16::INSTR", resource_manager=FakeRM(res))
    assert res.closed


def test_empty_resource_is_rejected():
    with pytest.raises(ValueError):
        Keithley2002("", resource_manager=FakeRM(FakeResource()))


def test_local_sends_gtl_not_an_scpi_string():
    dmm, res = make_driver()
    dmm.local()
    assert "gtl" in res.log and res.writes() == []


# ----------------------------------------------------------------------
# 3. SharedDevices: initialise, measure, clean up
# ----------------------------------------------------------------------
def _patch_driver(monkeypatch, resource):
    monkeypatch.setattr(
        k2002_pkg, "Keithley2002",
        lambda resource_name, timeout_ms=10000: Keithley2002(
            resource_name, timeout_ms=timeout_ms, resource_manager=FakeRM(resource)),
    )


def _shared(**configs):
    return SharedDevices(use_dmm=True, dmm_resource="GPIB0::16::INSTR", **configs)


def test_initialize_without_resource_fails_without_opening(monkeypatch):
    opened = []
    monkeypatch.setattr(k2002_pkg, "Keithley2002", lambda *a, **k: opened.append(a))
    shared = SharedDevices(use_dmm=True)
    assert asyncio.run(shared.initialize()) is False
    assert opened == [] and shared.dmm is None


def test_initialize_checks_idn_and_clears_status(monkeypatch):
    res = FakeResource()
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize()) is True
    assert res.log[:3] == ["clear", ("w", "*CLS"), ("q", "*IDN?")]
    assert shared.dmm is not None


def test_initialize_closes_the_handle_on_a_wrong_instrument(monkeypatch):
    res = FakeResource(idn="KEITHLEY INSTRUMENTS INC.,MODEL 2000,1,A01")
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize()) is False
    assert res.closed and shared.dmm is None


def test_measure_dmm_returns_median_and_settings(monkeypatch):
    res = FakeResource(readings=["+1.0E+00", "+9.9E37", "+3.0E+00", "+2.0E+00"])
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        return await shared.measure_dmm("dc_voltage", samples=4, nplc=10)

    result = asyncio.run(main())
    assert result == {"dmm_value": 2.0, "dmm_unit": "V", "dmm_function": "dc_voltage",
                      "dmm_range": 20.0, "dmm_nplc": 10.0, "dmm_valid_samples": 3}


def test_measure_dmm_even_count_median_is_the_mean_of_the_middle_pair(monkeypatch):
    _patch_driver(monkeypatch, FakeResource(readings=["+1.0E+00", "+2.0E+00"]))
    shared = _shared()

    async def main():
        await shared.initialize()
        return await shared.measure_dmm(samples=2)

    assert asyncio.run(main())["dmm_value"] == 1.5


def test_measure_dmm_raises_when_no_reading_is_valid(monkeypatch):
    _patch_driver(monkeypatch, FakeResource(readings=["+9.9E37"] * 3))
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.measure_dmm(samples=3)

    with pytest.raises(ValueError):
        asyncio.run(main())


def test_dmm_range_is_the_one_used_by_the_samples(monkeypatch):
    """S2: on autorange the instrument picks the range while reading; the range
    read back before the first reading is not the one the samples used."""
    class Autorange(FakeResource):
        def __init__(self):
            super().__init__()
            self.range = "+1.000000E+03"

        def query(self, command):
            if command == ":READ?":
                self.range = "+2.000000E+00"
            if command.endswith(":RANG?"):
                self._check_open(command)
                self.log.append(("q", command))
                return self.range
            return super().query(command)

    res = Autorange()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        return await shared.measure_dmm(samples=2)

    assert asyncio.run(main())["dmm_range"] == 2.0
    queries = [c for kind, c in (e for e in res.log if isinstance(e, tuple)) if kind == "q"]
    assert queries.index(":SENS:VOLTage:DC:RANG?") > max(
        i for i, c in enumerate(queries) if c == ":READ?")


def test_io_error_in_the_readback_fails_the_step(monkeypatch):
    class Timeout(FakeResource):
        def query(self, command):
            if command.endswith(":NPLC?"):
                raise TimeoutError("VI_ERROR_TMO")
            return super().query(command)

    _patch_driver(monkeypatch, Timeout())
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.measure_dmm()

    with pytest.raises(TimeoutError):
        asyncio.run(main())


def test_wait_after_dmm(monkeypatch):
    # record the requested pause instead of timing it (Windows timer granularity ~15 ms)
    _patch_driver(monkeypatch, FakeResource())
    shared = _shared(wait_after_dmm=0.3)
    pauses = []

    async def fake_sleep(seconds):
        pauses.append(seconds)

    async def main():
        await shared.initialize()
        monkeypatch.setattr(asyncio, "sleep", fake_sleep)
        await shared.measure_dmm(samples=1)

    asyncio.run(main())
    assert pauses == [0.3]


def test_measure_dmm_requires_initialisation():
    with pytest.raises(RuntimeError):
        asyncio.run(SharedDevices().measure_dmm())


def test_cleanup_sends_gtl_then_closes(monkeypatch):
    res = FakeResource()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()

    asyncio.run(main())
    assert res.log[-2:] == ["gtl", "close"] and shared.dmm is None


def test_cleanup_closes_even_when_gtl_fails(monkeypatch):
    class NoGtl(FakeResource):
        def control_ren(self, mode):
            raise RuntimeError("bus error")

    res = NoGtl()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()

    asyncio.run(main())
    assert res.closed


def test_cancel_does_not_wait_for_the_read_and_cleanup_waits_before_closing(monkeypatch):
    """Ctrl+C / Stop: the step is cancelled at once (the session can emergency-stop
    the robots), the worker skips the remaining samples, and the DMM is not closed
    under a read that is still on the bus."""
    reading = threading.Event()
    release = threading.Event()

    class Slow(FakeResource):
        def query(self, command):
            if command == ":READ?":
                reading.set()
                release.wait(5)
            return super().query(command)

    res = Slow()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=5))
        await asyncio.to_thread(reading.wait, 5)
        task.cancel()
        t0 = time.monotonic()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert time.monotonic() - t0 < 1.0
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.3)
        assert not res.closed, "closed while a read was in flight"
        release.set()
        await cleanup

    asyncio.run(main())
    assert res.log.count(("q", ":READ?")) == 1, "remaining samples must be skipped"
    assert res.log[-2:] == ["gtl", "close"]


# ----------------------------------------------------------------------
# 4. Mock parity
# ----------------------------------------------------------------------
def test_mock_measure_dmm_has_the_real_signature():
    assert (inspect.signature(MockSharedDevices.measure_dmm)
            == inspect.signature(SharedDevices.measure_dmm))


def _opened_mock():
    mock = MockSharedDevices(use_dmm=True)
    asyncio.run(mock.initialize())
    return mock


def test_mock_measure_dmm_returns_a_value_and_checks_like_the_driver():
    mock = _opened_mock()
    result = asyncio.run(mock.measure_dmm("resistance_2w", samples=2, nplc=5, range_=1e3))
    assert result["dmm_value"] == MockSharedDevices.MOCK_DMM_VALUE
    assert result["dmm_unit"] == "Ohm" and result["dmm_valid_samples"] == 2
    with pytest.raises(ValueError):
        asyncio.run(mock.measure_dmm("ac_current", range_=5.0))


@pytest.mark.parametrize("make", [
    lambda: MockSharedDevices(),                               # DMM not requested
    lambda: MockSharedDevices(use_dmm=True),                   # requested, not initialised
    lambda: SharedDevices(),
    lambda: SharedDevices(use_dmm=True, dmm_resource="GPIB0::16::INSTR"),
])
def test_mock_and_real_refuse_to_measure_before_initialize(make):
    with pytest.raises(RuntimeError):
        asyncio.run(make().measure_dmm())


def test_mock_and_real_refuse_to_measure_after_cleanup(monkeypatch):
    mock = _opened_mock()
    asyncio.run(mock.cleanup())
    with pytest.raises(RuntimeError):
        asyncio.run(mock.measure_dmm())
    _patch_driver(monkeypatch, FakeResource())
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()
        await shared.measure_dmm()

    with pytest.raises(RuntimeError):
        asyncio.run(main())


def test_mock_and_real_fail_alike_without_samples(monkeypatch):
    with pytest.raises(ValueError):
        asyncio.run(_opened_mock().measure_dmm(samples=0))
    _patch_driver(monkeypatch, FakeResource())
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.measure_dmm(samples=0)

    with pytest.raises(ValueError):
        asyncio.run(main())


# ----------------------------------------------------------------------
# 5. Schema and executor
# ----------------------------------------------------------------------
def _validate(step):
    return ExperimentWorkflow(name="t", steps=[{"action": "measure_dmm", **step}]).steps[0]


def test_schema_defaults_and_bounds():
    step = _validate({})
    assert isinstance(step, ActionMeasureDMM)
    assert (step.function, step.samples, step.nplc, step.range) == ("dc_voltage", 3, 1.0, None)
    for bad in ({"function": "temperature"}, {"samples": 0}, {"samples": 11},
                {"nplc": 0.001}, {"nplc": 51}, {"range": 0}, {"range": -1},
                {"function": "resistance_4w", "range": 3e6}, {"robot_id": 1}):
        with pytest.raises(ValidationError):
            _validate(bad)


def test_schema_and_driver_share_the_function_table():
    from typing import get_args
    assert set(get_args(ActionMeasureDMM.model_fields["function"].annotation)) == set(FUNCTIONS)


class RecordingShared:
    def __init__(self):
        self.calls = []

    async def measure_dmm(self, **kwargs):
        self.calls.append(kwargs)
        return {"dmm_value": 0.5, "dmm_unit": "A", "dmm_function": kwargs["function"]}


@pytest.mark.parametrize("as_model", [False, True])
def test_executor_passes_every_parameter(as_model):
    raw = {"action": "measure_dmm", "function": "dc_current", "samples": 7, "nplc": 0.5,
           "range": 0.2}
    step = _validate({k: v for k, v in raw.items() if k != "action"}) if as_model else raw
    shared = RecordingShared()
    result = asyncio.run(execute_step(step, robots={}, shared_devices=shared))
    assert shared.calls == [{"function": "dc_current", "samples": 7, "nplc": 0.5, "range_": 0.2}]
    assert result["dmm_value"] == 0.5
    assert "measure_dmm" in SHARED_DEVICE_ACTIONS


# ----------------------------------------------------------------------
# 6. Runner wiring
# ----------------------------------------------------------------------
def test_plan_resources_needs_dmm_and_no_robot():
    plan = run_flow.plan_resources([{"action": "measure_dmm"}])
    assert plan.robot_ids == [] and plan.needs_dmm and plan.needs_shared
    assert not (plan.needs_scale or plan.needs_camera or plan.needs_microscope
                or plan.needs_microscope_serial or plan.needs_lockin)
    assert run_flow.plan_resources([{"action": "measure_weight"}]).needs_dmm is False


def test_dmm_resource_priority(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.delenv("DMM_RESOURCE", raising=False)
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["dmm_resource"] == run_flow.lab_config.get_shared_devices()["dmm_resource"]
    monkeypatch.setenv("DMM_RESOURCE", "GPIB0::22::INSTR")
    assert run_flow.resolve_ports(parser.parse_args(["x.json"]))[1]["dmm_resource"] == "GPIB0::22::INSTR"
    args = parser.parse_args(["x.json", "--dmm-resource", "GPIB1::5::INSTR"])
    assert run_flow.resolve_ports(args)[1]["dmm_resource"] == "GPIB1::5::INSTR"


def test_csv_runner_parser_works_with_resolve_ports(monkeypatch):
    """run_csv has its own parser without --dmm-resource; resolve_ports must cope."""
    from src.flow.csv_runner import run_csv
    monkeypatch.setenv("DMM_RESOURCE", "GPIB0::9::INSTR")
    _, shared = run_flow.resolve_ports(run_csv.build_parser().parse_args(["--mock"]))
    assert shared["dmm_resource"] == "GPIB0::9::INSTR"


def test_session_factories_forward_dmm_settings():
    from src.flow.experiment_session import default_shared_factory
    shared = default_shared_factory(use_scale=False, use_camera=False, use_dmm=True,
                                    config={"scale_port": "", "camera_index": 0,
                                            "dmm_resource": "GPIB0::3::INSTR",
                                            "dmm_timeout_ms": 2500, "wait_after_dmm": 1.5})
    assert shared.use_dmm
    assert shared.device_configs["dmm_resource"] == "GPIB0::3::INSTR"
    assert shared._dmm_timeout_ms() == 2500 and shared.wait_after_dmm == 1.5


def test_gui_function_change_drops_the_fixed_range():
    """S3: 20 V kept as 20 A would make the step invalid and read-only in the GUI."""
    from src.gui import runner as gui_runner
    step = {"action": "measure_dmm", "function": "dc_voltage", "range": 20.0}
    gui_runner.change_dmm_function(step, "dc_current")
    assert step == {"action": "measure_dmm", "function": "dc_current"}
    _validate({k: v for k, v in step.items() if k != "action"})
    same = {"action": "measure_dmm", "function": "dc_current", "range": 0.2}
    assert gui_runner.change_dmm_function(same, "dc_current")["range"] == 0.2


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def test_example_flow_validates_and_mock_runs_without_a_robot(tmp_path, monkeypatch):
    assert run_flow.main([EXAMPLE, "--validate-only"]) == 0
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "completed"
    assert meta["robots_used"] == [] and meta["dmm_measurements"] == 3
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    assert [r["dmm_value"] for r in rows] == ["1.234"] * 3
    assert {r["dmm_unit"] for r in rows} == {"V"}
    assert rows[1]["dmm_range"] == "20.0" and rows[1]["dmm_nplc"] == "10.0"
    assert "DMM 測定値" in open(os.path.join(run_dir, "summary.md"), encoding="utf-8").read()


def test_gui_runner_mock_run_records_dmm(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE, encoding="utf-8"))
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path), source_path=EXAMPLE)
    runner.start()
    assert runner.join(60)
    assert runner.status == "completed", runner.error
    with open(os.path.join(runner.log_dir, "measurements.csv"), encoding="utf-8-sig") as f:
        assert all(r["dmm_value"] == "1.234" for r in csv.DictReader(f))


# ----------------------------------------------------------------------
# 7. Logging keeps the instrument's precision
# ----------------------------------------------------------------------
@pytest.mark.parametrize("value, text", [
    (1.234567891, "1.234567891"), (0.0, "0.0"), (-2.5e-09, "-2.5e-09"),
])
def test_logger_keeps_full_precision_zero_and_negative(tmp_path, value, text):
    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    exp.record_step(1, 1, "measure_dmm", None, None, "ok", 0.1,
                    result={"dmm_value": value, "dmm_unit": "V", "dmm_function": "dc_voltage"})
    exp.finalize("completed")
    with open(exp.csv_path, encoding="utf-8-sig") as f:
        row = next(csv.DictReader(f))
    assert row["dmm_value"] == text
    assert any(e["kind"] == "dmm" for e in exp.events)


# ----------------------------------------------------------------------
# 8. Fixes that came with the DMM
# ----------------------------------------------------------------------
class _Shared:
    def __init__(self, outcome):
        self.outcome = outcome
        self.cleaned = 0

    async def initialize(self):
        if isinstance(self.outcome, BaseException):
            raise self.outcome
        return self.outcome

    async def cleanup(self):
        self.cleaned += 1


@pytest.mark.parametrize("outcome", [False, RuntimeError("boom")])
def test_add_shared_closes_a_partially_initialised_device(outcome):
    """A failed initialize() must not leave ports open: the device is only
    registered in session.shared on success, so cleanup() would miss it."""
    made = []

    def factory(**kwargs):
        made.append(_Shared(outcome))
        return made[-1]

    session = ExperimentSession(mock=True, shared_factory=factory, robot_ports={},
                                shared_config={}, workspace_validator=object())
    with pytest.raises(BaseException):
        asyncio.run(session.add_shared(use_scale=True))
    assert made[0].cleaned == 1 and session.shared is None


def test_add_shared_defers_closing_an_interrupted_initialisation_to_run():
    """Waiting for the device here would delay run()'s emergency stop; it is
    registered instead and closed by run()'s cleanup()."""
    made = []

    def factory(**kwargs):
        made.append(_Shared(asyncio.CancelledError()))
        return made[-1]

    session = ExperimentSession(mock=True, shared_factory=factory, robot_ports={},
                                shared_config={}, workspace_validator=object())
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(session.add_shared(use_scale=True))
    assert made[0].cleaned == 0 and session.shared is made[0]
    asyncio.run(session.cleanup())
    assert made[0].cleaned == 1


def _record_robot_safety(monkeypatch):
    """Record emergency stops and go-home moves of every MockLabRobot."""
    from src.devices.safety.mock_robot import MockLabRobot
    calls = []
    original_estop = MockLabRobot.emergency_stop

    def estop(self):
        calls.append("emergency_stop")
        return original_estop(self)

    async def go_home(self):
        calls.append("go_home")

    monkeypatch.setattr(MockLabRobot, "emergency_stop", estop)
    monkeypatch.setattr(MockLabRobot, "go_home", go_home)
    return calls


async def _wait_until(predicate, timeout=5.0):
    t0 = time.monotonic()
    while not predicate():
        assert time.monotonic() - t0 < timeout, "condition not reached"
        await asyncio.sleep(0.01)


def test_ctrl_c_while_closing_a_failed_device_stops_in_place(monkeypatch):
    """M1: initialize() returns False, Ctrl+C arrives while its ports are being
    closed. That is an interruption (emergency stop, no recovery move), not an
    ordinary error (which would send the arms home)."""
    calls = _record_robot_safety(monkeypatch)
    closing = asyncio.Event()

    class SlowToClose:
        def __init__(self):
            self.calls = 0

        async def initialize(self):
            return False

        async def cleanup(self):
            self.calls += 1
            if self.calls == 1:              # the first close hangs until cancelled
                closing.set()
                await asyncio.Event().wait()

    session = ExperimentSession(mock=True, shared_factory=lambda **k: SlowToClose(),
                                robot_ports={1: {"type": "dobot"}}, shared_config={})

    async def body():
        await session.add_robot(1)
        await session.add_shared(use_scale=True)

    async def main():
        task = asyncio.create_task(session.run(body))
        await asyncio.wait_for(closing.wait(), 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())
    assert "emergency_stop" in calls and "go_home" not in calls
    assert session.status == "aborted"


def test_ctrl_c_during_dmm_initialisation_stops_robots_first(monkeypatch):
    """M2/M3: Ctrl+C while the DMM worker waits for *IDN?. The robots are
    emergency-stopped without waiting for the GPIB reply; the handle the worker
    opened is closed once its I/O is done, never under it, and never leaked."""
    from src.flow.experiment_session import default_shared_factory
    calls = _record_robot_safety(monkeypatch)
    in_idn = threading.Event()
    release = threading.Event()

    class SlowIdn(FakeResource):
        def query(self, command):
            if command == "*IDN?":
                in_idn.set()
                release.wait(10)
            return super().query(command)

    res = SlowIdn()
    _patch_driver(monkeypatch, res)
    session = ExperimentSession(
        mock=True, shared_factory=default_shared_factory, robot_ports={1: {"type": "dobot"}},
        shared_config={"scale_port": "", "camera_index": 0, "dmm_resource": "GPIB0::16::INSTR"})

    async def body():
        await session.add_robot(1)
        await session.add_shared(use_dmm=True)

    async def main():
        task = asyncio.create_task(session.run(body))
        await asyncio.to_thread(in_idn.wait, 5)
        task.cancel()
        await _wait_until(lambda: "emergency_stop" in calls)
        assert not release.is_set(), "the e-stop must not wait for the instrument"
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())
    assert "go_home" not in calls and session.status == "aborted"
    assert res.closed and not res.io_after_close
    assert res.log[-2:] == ["gtl", "close"]


def test_cleanup_that_gives_up_waiting_leaves_closing_to_the_worker(monkeypatch):
    """M3: cleanup() must not break the lock when the wait runs out. It returns
    without touching the handle; the worker closes it after its read."""
    reading = threading.Event()
    release = threading.Event()

    class Slow(FakeResource):
        def query(self, command):
            if command == ":READ?":
                reading.set()
                release.wait(10)
            return super().query(command)

    res = Slow()
    _patch_driver(monkeypatch, res)
    shared = _shared()
    shared.dmm_close_wait_s = 0.1

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=5))
        await asyncio.to_thread(reading.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await shared.cleanup()               # gives up after 0.1 s
        assert not res.closed, "closed while a read was in flight"
        release.set()
        await _wait_until(lambda: res.closed)

    asyncio.run(main())
    assert not res.io_after_close
    assert res.log.count(("q", ":READ?")) == 1
    assert res.log[-2:] == ["gtl", "close"] and shared.dmm is None


def _blocking(on_command):
    """FakeResource whose ``on_command`` (write or query) blocks until released."""
    entered, release = threading.Event(), threading.Event()

    class Blocking(FakeResource):
        def write(self, command):
            if command == on_command:
                entered.set()
                release.wait(10)
            super().write(command)

        def query(self, command):
            if command == on_command:
                entered.set()
                release.wait(10)
            return super().query(command)

    return Blocking(), entered, release


def _sent(res):
    return [c for kind, c in (e for e in res.log if isinstance(e, tuple))]


def test_cancel_during_the_last_read_sends_no_readback(monkeypatch):
    """The abort is checked at every I/O boundary, not only before each READ: a
    cancel during the final READ must not be followed by RANG? / NPLC?."""
    res, entered, release = _blocking(":READ?")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=1))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        release.set()
        await shared.cleanup()

    asyncio.run(main())
    after_read = _sent(res)[_sent(res).index(":READ?") + 1:]
    assert not any(c.endswith(("RANG?", "NPLC?")) for c in after_read), after_read
    assert res.log[-2:] == ["gtl", "close"]


def test_cancel_during_configure_stops_at_the_next_command(monkeypatch):
    res, entered, release = _blocking(":SENS:VOLTage:DC:RANG:AUTO ON")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm())
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        release.set()
        await shared.cleanup()

    asyncio.run(main())
    sent = _sent(res)
    assert sent[-1] == ":SENS:VOLTage:DC:RANG:AUTO ON", sent      # nothing after it
    assert ":READ?" not in sent


def test_a_close_failure_in_the_worker_is_logged_and_retried(monkeypatch, caplog):
    """cleanup() gave up waiting and left the close to the worker; the worker's
    close fails. The failure must be logged and the handle kept for a retry."""
    class FailsOnce(FakeResource):
        def __init__(self):
            super().__init__()
            self.close_attempts = 0

        def close(self):
            self.close_attempts += 1
            if self.close_attempts == 1:
                raise RuntimeError("VI_ERROR_CONN_LOST")
            super().close()

    res = FailsOnce()
    entered, release = threading.Event(), threading.Event()
    original_query = res.query

    def query(command):
        if command == ":READ?":
            entered.set()
            release.wait(10)
        return original_query(command)

    res.query = query
    _patch_driver(monkeypatch, res)
    shared = _shared()
    shared.dmm_close_wait_s = 0.1

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=3))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await shared.cleanup()               # gives up; the worker will close
        release.set()
        await _wait_until(lambda: res.close_attempts == 1)
        await asyncio.sleep(0.05)
        assert shared.dmm is not None, "the handle must be kept after a failed close"
        await shared.cleanup()               # retry

    with caplog.at_level("ERROR"):
        asyncio.run(main())
    assert any("閉じられません" in r.getMessage() for r in caplog.records)
    assert res.closed and shared.dmm is None


def test_a_close_failure_in_cleanup_is_reported(monkeypatch, caplog):
    class NeverCloses(FakeResource):
        def close(self):
            raise RuntimeError("VI_ERROR_CONN_LOST")

    _patch_driver(monkeypatch, NeverCloses())
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()

    with caplog.at_level("ERROR"):
        asyncio.run(main())
    messages = [r.getMessage() for r in caplog.records]
    assert any("切断エラー" in m for m in messages) and shared.dmm is not None


def test_cleanup_during_initialisation_does_not_leak_the_handle(monkeypatch):
    in_idn = threading.Event()
    release = threading.Event()

    class SlowIdn(FakeResource):
        def query(self, command):
            if command == "*IDN?":
                in_idn.set()
                release.wait(10)
            return super().query(command)

    res = SlowIdn()
    _patch_driver(monkeypatch, res)
    shared = _shared()
    shared.dmm_close_wait_s = 0.1

    async def main():
        init = asyncio.create_task(shared.initialize())
        await asyncio.to_thread(in_idn.wait, 5)
        init.cancel()
        with pytest.raises(asyncio.CancelledError):
            await init
        await shared.cleanup()               # the worker still holds the lock
        release.set()
        await _wait_until(lambda: res.closed)

    asyncio.run(main())
    assert shared.dmm is None and not res.io_after_close


def test_balance_is_closed_when_the_dmm_fails_to_open(monkeypatch):
    """End to end on the real SharedDevices: balance opens, DMM fails -> balance closed."""
    closed = []

    class FakeBalance:
        def __init__(self, port):
            pass

        def get_weight(self):
            return 1.0

        def close(self):
            closed.append("balance")

    import src.devices.scale.BCE8221 as bce
    monkeypatch.setattr(bce, "SerialBalance", FakeBalance)
    session = ExperimentSession(mock=False, robot_ports={}, workspace_validator=object(),
                                shared_config={"scale_port": "COMX", "camera_index": 0,
                                               "dmm_resource": ""})
    with pytest.raises(RuntimeError):
        asyncio.run(session.add_shared(use_scale=True, use_dmm=True))
    assert closed == ["balance"]


def test_gui_settings_keep_config_keys_without_a_sidebar_field(monkeypatch):
    """The GUI rebuilt the shared settings from the sidebar only, so microscope and
    DMM settings from config.yaml never reached the devices."""
    from src import config as lab_config
    from src.gui import runner as gui_runner
    monkeypatch.setattr(lab_config, "get_shared_devices", lambda: {
        "scale_port": "COM8", "camera_index": 0, "microscope_index": 2,
        "microscope_port": "COM10", "dmm_resource": "GPIB0::16::INSTR", "dmm_timeout_ms": 20000})
    settings = gui_runner.default_device_settings()
    settings["shared"]["scale_port"] = "COM_SIDEBAR"
    settings["shared"]["camera_index"] = 3
    _, shared = gui_runner.settings_to_session_args(settings)
    assert shared["scale_port"] == "COM_SIDEBAR" and shared["camera_index"] == 3
    assert shared["microscope_port"] == "COM10"
    assert shared["dmm_resource"] == "GPIB0::16::INSTR" and shared["dmm_timeout_ms"] == 20000
