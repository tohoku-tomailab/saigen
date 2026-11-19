"""Agilent 1000 series oscilloscope: driver, SharedDevices wrapper, flow wiring, logging.

No hardware and no PyVISA: the driver takes an injected resource manager. The
fake answers by command from a settings table in the reply forms of the
programmer's guide (``Y-T``, ``1.000e000``, ``5.000e001%`` ...), records the bus
traffic, and can change a setting after a given measurement query, return
bounds / sentinels / malformed replies, or block on one command.
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

import src.devices.dso1000 as dso_pkg
import src.devices.keithley2002 as k2002_pkg
import src.devices.li5640 as li_pkg
from src.devices.dso1000.dso1000 import (
    DSO1000, DSO1000Error, ScopeStepError, item_unit, parse_measurement, parse_model,
    parse_on_off, parse_timebase_format, parse_units,
)
from src.devices.keithley2002.keithley2002 import Keithley2002
from src.devices.li5640.li5640 import LI5640
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import SHARED_DEVICE_ACTIONS, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import ActionMeasureScope, ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(REPO_ROOT, "examples", "dso1000", "read_scope.json")
IDN = "Agilent Technologies,DSO1014A,CN00000000,00.04.02.01.00"
RESOURCE = "USB0::0x0957::0x0588::CN00000000::INSTR"

GLOBAL = {
    ":TIMebase:FORMat?": "Y-T", ":TIMebase:MODE?": "MAIN", ":TIMebase:SCALe?": "1.000e-003",
    ":TIMebase:OFFSet?": "0.000e000", ":ACQuire:TYPE?": "NORMAL", ":ACQuire:AVERages?": "16",
    ":TRIGger:MODE?": "EDGE", ":TRIGger:COUPling?": "DC", ":TRIGger:HOLDoff?": "1.000e-007",
    ":TRIGger:SENSitivity?": "5.000e-001", ":TRIGger:HFREject?": "0",
    ":TRIGger:EDGE:SOURce?": "CH1", ":TRIGger:EDGE:LEVel?": "0.000e000",
    ":TRIGger:EDGE:SLOPe?": "POSITIVE", ":TRIGger:EDGE:SWEep?": "AUTO",
    ":MEASure?": "UnLocked", ":KEY:LOCK?": "ENABLE", ":MEASure:SOURce?": "CHANnel1",
    ":TRIGger:STATus?": "RUN", ":ACQuire:SRATe?": "1.000e008",
}
CHANNEL = {"DISPlay": "1", "SCALe": "1.000e000", "OFFSet": "0.000e000", "PROBe": "10X",
           "COUPling": "DC", "BWLimit": "0", "FILTer": "0", "INVert": "0", "VERNier": "0",
           "UNITs": "VOLTs"}
MEASURED = {"VPP": "2.000e000", "VMAX": "1.000e000", "VMIN": "-1.000e000",
            "VAMPlitude": "2.000e000", "VTOP": "1.000e000", "VBASe": "-1.000e000",
            "VAVerage": "1.000e-003", "VRMS": "7.070e-001", "FREQuency": "1.000e003",
            "PERiod": "1.000e-003", "RISetime": "<2.000e-009", "FALLtime": "<2.000e-009",
            "PWIDth": "5.000e-004", "NWIDth": "5.000e-004", "PDUTycycle": "5.000e001%",
            "NDUTycycle": "5.000e001%", "OVERshoot": "1.000e000%", "PREShoot": "1.000e000%"}


class FakeScope:
    """A VISA resource that behaves like a DSO1014A and records the bus traffic."""

    def __init__(self, idn=IDN, errors=(), measured=None, **settings):
        self.idn = idn
        self.errors = list(errors)
        self.settings = dict(GLOBAL)
        for n in range(1, 5):
            for k, v in CHANNEL.items():
                self.settings[f":CHANnel{n}:{k}?"] = v
        self.settings.update(settings)
        self.measured = {k: [v] for k, v in MEASURED.items()}
        for k, v in (measured or {}).items():
            self.measured[k] = list(v) if isinstance(v, (list, tuple)) else [v]
        self.after_measure = {}             # {n: {command: reply}} applied after the n-th :MEASure
        self.n_measure = 0
        self.log = []
        self.closed = False
        self.io_after_close = False
        self.timeout = None
        self.read_termination = self.write_termination = "unset"

    def _check_open(self, command):
        if self.closed:
            self.io_after_close = True
            raise AssertionError(f"I/O after close: {command!r}")

    def write(self, command):
        self._check_open(command)
        self.log.append(("w", command))
        if command != "*CLS":
            raise AssertionError(f"unexpected write {command!r}")

    def query(self, command):
        self._check_open(command)
        self.log.append(("q", command))
        if command == "*IDN?":
            return self.idn + "\n"
        if command == ":SYSTem:ERRor?":
            return self.errors.pop(0) if self.errors else '0,"No error"'
        if command.startswith(":MEASure:") and command.endswith(tuple(f"CHANnel{n}" for n in range(1, 5))):
            header = command[len(":MEASure:"):].split("?")[0]
            self.n_measure += 1
            if header not in self.measured:
                raise AssertionError(f"unknown measurement header {header!r}")
            seq = self.measured[header]
            reply = seq.pop(0) if len(seq) > 1 else seq[0]
            self.settings.update(self.after_measure.get(self.n_measure, {}))
            return reply
        if command in self.settings:
            return self.settings[command]
        raise AssertionError(f"unexpected query {command!r}")

    def clear(self):
        self.log.append("clear")

    def control_ren(self, mode):
        self.log.append(f"ren:{int(mode)}")
        raise AssertionError("no REN / GTL over USB (it hung 120 s on the bench)")

    def close(self):
        self.log.append("close")
        self.closed = True

    def sent(self):
        return [c for kind, c in (e for e in self.log if isinstance(e, tuple))]

    def writes(self):
        return [c for kind, c in (e for e in self.log if isinstance(e, tuple)) if kind == "w"]


class FakeRM:
    def __init__(self, resource):
        self.resource = resource

    def open_resource(self, name):
        return self.resource


def make_driver(res=None):
    res = res or FakeScope()
    return DSO1000(RESOURCE, resource_manager=FakeRM(res)), res


def _patch_driver(monkeypatch, res):
    monkeypatch.setattr(dso_pkg, "DSO1000",
                        lambda name, timeout_ms=5000: DSO1000(name, timeout_ms=timeout_ms,
                                                              resource_manager=FakeRM(res)))


def _shared(**configs):
    return SharedDevices(use_scope=True, scope_resource=RESOURCE, **configs)


def _measure(shared, **kwargs):
    async def main():
        assert await shared.initialize()
        try:
            return await shared.measure_scope(**kwargs)
        finally:
            await shared.cleanup()
    return asyncio.run(main())


RELEASE_TAIL = ["clear", "close"]       # no GTL: it hung 120 s over USB on the bench


# ----------------------------------------------------------------------
# 1. Pure functions
# ----------------------------------------------------------------------
@pytest.mark.parametrize("reply, item, expected", [
    ("2.000e000", "vpp", (2.0, None)),
    (" -1.500e-003\r", "vmin", (-1.5e-3, None)),
    ("5.000e001%", "pdutycycle", (50.0, None)),          # not multiplied by 100
    ("-2.000e000%", "preshoot", (-2.0, None)),           # preshoot may be negative
    ("5.000e001", "ndutycycle", (50.0, None)),           # without the % sign too
    ("5.000e001%", "vpp", (None, "unparsable")),          # % only for the four % items
    ("%5.000e001", "overshoot", (None, "unparsable")),    # only a trailing %
    ("<2.000e-009", "risetime", (None, "bound")),
    (">1.000e003", "frequency", (None, "bound")),
    ("9.9E+37", "frequency", (None, "no_measurement")),
    ("-9.9e37", "vpp", (None, "no_measurement")),
    ("NaN", "vpp", (None, "non_finite")),
    ("inf", "vpp", (None, "non_finite")),
    ("****", "vpp", (None, "unparsable")),
    ("", "vpp", (None, "unparsable")),
])
def test_parse_measurement(reply, item, expected):
    assert parse_measurement(reply, item) == expected


@pytest.mark.parametrize("reply, expected", [("Y-T", "YT"), ("X-Y", "XY"), ("ROLL", "ROLL"),
                                             ("YT\n", "YT"), ("xy", "XY")])
def test_timebase_format_accepts_the_reply_form(reply, expected):
    assert parse_timebase_format(reply) == expected
    with pytest.raises(DSO1000Error):
        parse_timebase_format("SCAN")


@pytest.mark.parametrize("model, channels", [("DSO1002A", 2), ("DSO1004A", 4), ("DSO1012A", 2),
                                             ("DSO1014A", 4), ("DSO1022A", 2), ("DSO1024A", 4)])
def test_model_table(model, channels):
    assert parse_model(f"Agilent Technologies,{model},X,1") == (model, channels)


@pytest.mark.parametrize("idn", ["Agilent Technologies,DSO1052B,X,1", "RIGOL,DS1102E,X,1",
                                 "Agilent Technologies", "DSO1014A,X,1"])
def test_unsupported_models_are_refused(idn):
    with pytest.raises(DSO1000Error):
        parse_model(idn)


def test_small_parsers():
    assert parse_on_off("1", "x") == 1 and parse_on_off("OFF", "x") == 0
    with pytest.raises(DSO1000Error):
        parse_on_off("2", "x")
    assert [parse_units(u, "x") for u in ("VOLTs", "AMPeres", "WATTs", "UNKNown")] == \
        ["V", "A", "W", ""]
    assert item_unit("vpp", "A") == "A" and item_unit("frequency", "A") == "Hz"
    assert item_unit("risetime", "V") == "s" and item_unit("overshoot", "V") == "%"


# ----------------------------------------------------------------------
# 2. Driver
# ----------------------------------------------------------------------
def test_constructor_uses_lf_and_closes_on_setup_failure():
    _, res = make_driver()
    assert res.read_termination == "\n" and res.write_termination == "\n"

    class Broken(FakeScope):
        def __setattr__(self, name, value):
            if name == "timeout" and value is not None:
                raise RuntimeError("bad attribute")
            super().__setattr__(name, value)

    broken = Broken()
    with pytest.raises(RuntimeError):
        DSO1000(RESOURCE, resource_manager=FakeRM(broken))
    assert broken.closed


def test_identify_learns_model_and_channels():
    dso, _ = make_driver(FakeScope(idn="Agilent Technologies,DSO1012A,X,1"))
    dso.identify()
    assert (dso.model, dso.channels) == ("DSO1012A", 2)


def test_error_queue_of_thirty_is_read_to_the_end():
    dso, res = make_driver(FakeScope(errors=[f'63,"Undefined header {i}"' for i in range(30)]))
    with pytest.raises(DSO1000Error, match="Undefined header 29"):
        dso.check_errors()
    assert res.sent().count(":SYSTem:ERRor?") == 31


def test_error_queue_that_never_empties_is_an_error():
    dso, res = make_driver(FakeScope(errors=['1,"Same setting"'] * 40))
    with pytest.raises(DSO1000Error, match="空になりません"):
        dso.check_errors()
    assert res.sent().count(":SYSTem:ERRor?") == 31


def test_snapshot_only_queries_and_reads_edge_settings_only_for_edge():
    dso, res = make_driver()
    snap = dso.snapshot(2)
    assert res.writes() == []
    assert snap["tb_format"] == "YT" and snap["ch_probe"] == "10X" and snap["ch_units"] == "V"
    assert snap["edge_source"] == "CH1" and snap["meas_lock"] == "UNLOCKED"
    assert snap["key_lock"] == "ENAB"
    assert all(":CHANnel2:" in c for c in res.sent() if ":CHANnel" in c)

    dso, res = make_driver(FakeScope(**{":TRIGger:MODE?": "PULSE"}))
    snap = dso.snapshot(1)
    assert snap["trig_mode"] == "PULSE" and "edge_source" not in snap
    assert not any(":TRIGger:EDGE" in c for c in res.sent())


@pytest.mark.parametrize("command, bad", [(":CHANnel1:PROBe?", "3X"), (":CHANnel1:SCALe?", "0"),
                                          (":TRIGger:HOLDoff?", "-1"), (":TRIGger:HOLDoff?", "2.0"),
                                          (":TRIGger:SENSitivity?", "99"),
                                          (":TRIGger:SENSitivity?", "0.05"),
                                          (":TIMebase:SCALe?", "100"),
                                          (":CHANnel1:UNITs?", "VOLTS_GARBAGE"),
                                          (":TIMebase:MODE?", "ZOOM"), (":ACQuire:AVERages?", "3"),
                                          (":TRIGger:EDGE:LEVel?", "nan"),
                                          (":CHANnel1:DISPlay?", "maybe")])
def test_snapshot_rejects_unexpected_settings(command, bad):
    dso, _ = make_driver(FakeScope(**{command: bad}))
    with pytest.raises(DSO1000Error):
        dso.snapshot(1)


def test_release_goes_on_after_failures(caplog):
    class Flaky(FakeScope):
        def clear(self):
            self.log.append("clear failed")
            raise RuntimeError("VI_ERROR_TMO")

    dso, res = make_driver(Flaky())
    dso.abort_check = lambda: True                 # release is not gated by the abort
    assert dso.release()
    assert res.log == ["clear failed", "close"]
    assert "デバイスクリア" in caplog.text


@pytest.mark.parametrize("reply, expected", [("ENABLE", "ENAB"), ("ENAB", "ENAB"),
                                             ("DISABLE", "DIS"), ("DIS", "DIS")])
def test_key_lock_accepts_the_bench_and_the_guide_forms(reply, expected):
    dso, _ = make_driver(FakeScope(**{":KEY:LOCK?": reply}))
    assert dso.snapshot(1)["key_lock"] == expected


# ----------------------------------------------------------------------
# 3. SharedDevices: initialisation
# ----------------------------------------------------------------------
def test_initialize_without_resource_fails_without_opening(monkeypatch):
    opened = []
    monkeypatch.setattr(dso_pkg, "DSO1000", lambda *a, **k: opened.append(a))
    assert asyncio.run(SharedDevices(use_scope=True).initialize()) is False
    assert opened == []


def test_initialize_sends_only_clear_cls_and_queries(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize())
    assert res.log[0] == "clear" and res.sent() == ["*CLS", "*IDN?", ":SYSTem:ERRor?"]
    asyncio.run(shared.cleanup())
    assert res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL


@pytest.mark.parametrize("res", [FakeScope(idn="Agilent Technologies,DSO1052B,X,1"),
                                 FakeScope(errors=['63,"Undefined header"'])])
def test_failed_initialisation_closes(monkeypatch, res):
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize()) is False
    assert res.closed and shared.scope is None and res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL


# ----------------------------------------------------------------------
# 4. SharedDevices: measurement
# ----------------------------------------------------------------------
def test_measure_returns_medians_units_counts_and_settings(monkeypatch):
    res = FakeScope(measured={"VPP": ["2.0e000", "2.2e000", "1.8e000"],
                              "FREQuency": ["1.000e003"] * 3,
                              "PDUTycycle": ["4.9e001%", "5.0e001%", "5.1e001%"]})
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), channel=1, items=["vpp", "frequency", "pdutycycle"],
                      samples=3, interval=0.0)
    assert result["scope_values"] == {"vpp": 2.0, "frequency": 1000.0, "pdutycycle": 50.0}
    assert result["scope_units"] == {"vpp": "V", "frequency": "Hz", "pdutycycle": "%"}
    assert result["scope_valid"] == {"vpp": 3, "frequency": 3, "pdutycycle": 3}
    assert result["scope_invalid"] == {} and result["scope_channel"] == 1
    assert result["scope_settings"]["tb_format"] == "YT"
    assert result["scope_raw"]["pdutycycle"] == ["4.9e001%", "5.0e001%", "5.1e001%"]
    # only queries, every measurement names its channel, the error queue is checked after
    assert res.writes() == ["*CLS"]
    measures = [c for c in res.sent() if c.startswith(":MEASure:") and "CHANnel" in c]
    assert len(measures) == 9 and all(c.endswith("? CHANnel1") for c in measures)
    assert res.sent()[-1] == ":SYSTem:ERRor?" and res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL


def test_current_probe_units_follow_the_channel(monkeypatch):
    _patch_driver(monkeypatch, FakeScope(**{":CHANnel2:UNITs?": "AMPeres"}))
    result = _measure(_shared(), channel=2, items=["vpp", "period"], samples=1)
    assert result["scope_units"] == {"vpp": "A", "period": "s"}


def test_invalid_readings_are_counted_per_item(monkeypatch):
    res = FakeScope(measured={"VPP": ["2.0e000", "9.9E+37", "NaN", "2.4e000"],
                              "RISetime": ["<2.0e-009", "3.0e-009", "<2.0e-009", "5.0e-009"]})
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), items=["vpp", "risetime"], samples=4, interval=0.0)
    assert result["scope_values"] == {"vpp": 2.2, "risetime": 4.0e-9}
    assert result["scope_valid"] == {"vpp": 2, "risetime": 2}
    assert result["scope_invalid"] == {"vpp": {"no_measurement": 1, "non_finite": 1},
                                       "risetime": {"bound": 2}}


def test_an_item_without_any_valid_reading_fails_the_step(monkeypatch):
    _patch_driver(monkeypatch, FakeScope(measured={"RISetime": "<2.0e-009"}))
    with pytest.raises(ScopeStepError) as info:
        _measure(_shared(), items=["vpp", "risetime"], samples=2, interval=0.0)
    diag = info.value.diagnostics
    assert diag["values"] == {"vpp": 2.0, "risetime": None}
    assert diag["valid"] == {"vpp": 2, "risetime": 0}
    assert diag["invalid"] == {"risetime": {"bound": 2}}
    assert diag["raw"]["risetime"] == ["<2.0e-009"] * 2


@pytest.mark.parametrize("setting, match", [
    ({":TIMebase:FORMat?": "X-Y"}, "X-Y"),
    ({":TIMebase:MODE?": "DELAYED"}, "DELAYED"),
    ({":TRIGger:MODE?": "PULSE"}, "エッジ"),
    ({":CHANnel1:DISPlay?": "0"}, "表示"),
])
def test_unsupported_states_fail_before_any_measurement(monkeypatch, setting, match):
    res = FakeScope(**setting)
    _patch_driver(monkeypatch, res)
    with pytest.raises(ScopeStepError, match=match):
        _measure(_shared(), items=["vpp"], samples=1)
    assert not any(c.startswith(":MEASure:VPP") for c in res.sent())
    if ":TIMebase:FORMat?" in setting:           # X-Y: nothing else is asked
        assert ":TIMebase:OFFSet?" not in res.sent()


def test_a_channel_the_model_lacks_fails_before_any_query(monkeypatch):
    res = FakeScope(idn="Agilent Technologies,DSO1012A,X,1")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        assert await shared.initialize()
        n = len(res.log)
        with pytest.raises(ScopeStepError, match="CH3"):
            await shared.measure_scope(channel=3, items=["vpp"])
        assert res.log[n:] == []
        await shared.cleanup()

    asyncio.run(main())


def test_a_setting_change_fails_with_before_after_and_diff(monkeypatch):
    res = FakeScope()
    res.after_measure = {1: {":CHANnel1:SCALe?": "5.000e-001"}}
    _patch_driver(monkeypatch, res)
    with pytest.raises(ScopeStepError) as info:
        _measure(_shared(), items=["vpp"], samples=2, interval=0.0)
    assert info.value.diagnostics["settings_diff"] == {"ch_scale": [1.0, 0.5]}


def test_volatile_settings_are_recorded_but_not_compared(monkeypatch):
    res = FakeScope()
    res.after_measure = {1: {":MEASure:SOURce?": "CHANnel2", ":TRIGger:STATus?": "T'D",
                             ":ACQuire:SRATe?": "5.000e007"}}
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), items=["vpp"], samples=2, interval=0.0)
    assert result["scope_settings"]["meas_source"] == "CHANnel1"


def test_an_instrument_error_after_the_reads_fails_the_step(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        assert await shared.initialize()
        res.errors = ['45,"Measure already selected"']
        with pytest.raises(DSO1000Error, match="Measure already selected"):
            await shared.measure_scope(items=["vpp"], samples=1)
        await shared.cleanup()

    asyncio.run(main())


def test_io_error_fails_the_step(monkeypatch):
    class Lost(FakeScope):
        def query(self, command):
            if command == ":MEASure:FREQuency? CHANnel1":
                self.log.append(("q", command))
                raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    res = Lost()
    _patch_driver(monkeypatch, res)
    with pytest.raises(RuntimeError, match="VI_ERROR_TMO") as info:
        _measure(_shared(), items=["vpp", "frequency"], samples=2, interval=0.0)
    assert not isinstance(info.value, ScopeStepError)
    after = res.sent()[res.sent().index(":MEASure:FREQuency? CHANnel1") + 1:]
    assert after == [] and res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL      # no read of a late reply


def test_stopped_scope_warns_but_measures(monkeypatch, caplog):
    _patch_driver(monkeypatch, FakeScope(**{":TRIGger:STATus?": "STOP"}))
    result = _measure(_shared(), items=["vpp"], samples=1)
    assert result["scope_values"]["vpp"] == 2.0 and "STOP" in caplog.text


def test_wait_after_scope_comes_after_the_measurement(monkeypatch):
    _patch_driver(monkeypatch, FakeScope())
    slept = []
    real_sleep = asyncio.sleep

    async def fake_sleep(seconds, *a, **k):
        slept.append(seconds)
        await real_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", fake_sleep)
    _measure(_shared(wait_after_scope=1.5), items=["vpp"], samples=1)
    assert 1.5 in slept


def test_bad_arguments_and_uninitialised():
    with pytest.raises(RuntimeError):
        asyncio.run(_shared().measure_scope())


# ----------------------------------------------------------------------
# 5. Cancellation and clean-up
# ----------------------------------------------------------------------
def _blocking(on_command, cls=FakeScope, **kwargs):
    entered, release = threading.Event(), threading.Event()

    class Blocking(cls):
        def query(self, command):
            if command == on_command:
                entered.set()
                release.wait(10)
            return super().query(command)

    return Blocking(**kwargs), entered, release


async def _wait_until(predicate, timeout=5.0):
    t0 = time.monotonic()
    while not predicate():
        assert time.monotonic() - t0 < timeout, "condition not reached"
        await asyncio.sleep(0.01)


def test_cancel_during_a_measurement_query_sends_nothing_more(monkeypatch):
    res, entered, release = _blocking(":MEASure:VPP? CHANnel1")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_scope(items=["vpp", "frequency"], samples=3))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        t0 = time.monotonic()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert time.monotonic() - t0 < 1.0
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.3)
        assert not res.closed, "closed while a query was in flight"
        release.set()
        await cleanup

    asyncio.run(main())
    after = res.log[res.log.index(("q", ":MEASure:VPP? CHANnel1")) + 1:]
    assert after == RELEASE_TAIL and not res.io_after_close


def test_cancel_during_the_interval_stops_at_once(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_scope(items=["vpp"], samples=3, interval=30.0))
        await _wait_until(lambda: ":MEASure:VPP? CHANnel1" in res.sent())
        await asyncio.sleep(0.2)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        t0 = time.monotonic()
        await shared.cleanup()
        assert time.monotonic() - t0 < 2.0

    asyncio.run(main())
    assert res.sent().count(":MEASure:VPP? CHANnel1") == 1 and res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL


def test_cleanup_that_gives_up_waiting_leaves_the_release_to_the_worker(monkeypatch):
    res, entered, release = _blocking(":MEASure:VPP? CHANnel1")
    _patch_driver(monkeypatch, res)
    shared = _shared()
    shared.scope_close_wait_s = 0.1

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_scope(items=["vpp"], samples=2))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await shared.cleanup()
        assert not res.closed
        release.set()
        await _wait_until(lambda: res.closed)

    asyncio.run(main())
    assert res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL and shared.scope is None and not res.io_after_close


def test_a_close_failure_is_logged_and_retried(monkeypatch, caplog):
    class FailsOnce(FakeScope):
        def __init__(self):
            super().__init__()
            self.attempts = 0

        def close(self):
            self.attempts += 1
            if self.attempts == 1:
                raise RuntimeError("VI_ERROR_CONN_LOST")
            super().close()

    res = FailsOnce()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()
        assert shared.scope is not None and not res.closed
        await shared.cleanup()

    asyncio.run(main())
    assert "閉じられません" in caplog.text and res.closed and shared.scope is None


def test_recancel_during_cleanup_still_releases_the_other_instruments(monkeypatch):
    """DMM, lock-in and scope open; Ctrl+C again while cleanup() waits for the DMM.
    The LI5640 and the scope are still released and the cancellation propagates."""
    from tests.test_keithley2002 import FakeResource, FakeRM as DmmRM
    from tests.test_li5640 import FakeLI, FakeRM as LiRM
    dmm_reading, dmm_release = threading.Event(), threading.Event()

    class SlowDmm(FakeResource):
        def query(self, command):
            if command == ":READ?":
                dmm_reading.set()
                dmm_release.wait(10)
            return super().query(command)

    dmm_res, li_res, scope_res = SlowDmm(), FakeLI(), FakeScope()
    monkeypatch.setattr(k2002_pkg, "Keithley2002", lambda name, timeout_ms=10000: Keithley2002(
        name, timeout_ms=timeout_ms, resource_manager=DmmRM(dmm_res)))
    monkeypatch.setattr(li_pkg, "LI5640", lambda name, timeout_ms=5000: LI5640(
        name, timeout_ms=timeout_ms, resource_manager=LiRM(li_res)))
    _patch_driver(monkeypatch, scope_res)
    shared = SharedDevices(use_dmm=True, dmm_resource="GPIB0::16::INSTR", use_lockin=True,
                           lockin_resource="GPIB0::7::INSTR", use_scope=True,
                           scope_resource=RESOURCE)
    shared.dmm_close_wait_s = 10.0

    async def main():
        assert await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=3))
        await asyncio.to_thread(dmm_reading.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.2)
        cleanup.cancel()
        with pytest.raises(asyncio.CancelledError):
            await cleanup
        assert li_res.closed and scope_res.closed and scope_res.log[-len(RELEASE_TAIL):] == RELEASE_TAIL
        dmm_release.set()
        await _wait_until(lambda: dmm_res.closed)

    asyncio.run(main())
    assert not scope_res.io_after_close


def test_cancelled_initialisation_sends_no_further_query(monkeypatch):
    res, entered, release = _blocking("*IDN?")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        init = asyncio.create_task(shared.initialize())
        await asyncio.to_thread(entered.wait, 5)
        init.cancel()
        with pytest.raises(asyncio.CancelledError):
            await init
        release.set()
        await asyncio.sleep(0.3)
        after_idn = res.log[res.log.index(("q", "*IDN?")) + 1:]
        assert not any(isinstance(e, tuple) for e in after_idn), after_idn
        await shared.cleanup()

    asyncio.run(main())
    assert res.closed and shared.scope is None


def test_ctrl_c_during_scope_initialisation_stops_robots_before_release(monkeypatch):
    from src.devices.safety.mock_robot import MockLabRobot
    from src.flow.experiment_session import default_shared_factory
    events = []
    original_estop = MockLabRobot.emergency_stop

    def estop(self):
        events.append("emergency_stop")
        return original_estop(self)

    async def go_home(self):
        events.append("go_home")

    monkeypatch.setattr(MockLabRobot, "emergency_stop", estop)
    monkeypatch.setattr(MockLabRobot, "go_home", go_home)

    class Recording(FakeScope):
        def close(self):
            events.append("scope_close")
            super().close()

    res, entered, release = _blocking("*IDN?", cls=Recording)
    _patch_driver(monkeypatch, res)
    session = ExperimentSession(
        mock=True, shared_factory=default_shared_factory, robot_ports={1: {"type": "dobot"}},
        shared_config={"scale_port": "", "camera_index": 0, "scope_resource": RESOURCE})

    async def body():
        await session.add_robot(1)
        await session.add_shared(use_scope=True)

    async def main():
        task = asyncio.create_task(session.run(body))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        await _wait_until(lambda: "emergency_stop" in events)
        assert not release.is_set()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())
    assert events.index("emergency_stop") < events.index("scope_close")
    assert "go_home" not in events and session.status == "aborted"


# ----------------------------------------------------------------------
# 6. Mock parity
# ----------------------------------------------------------------------
def test_mock_has_the_real_signature():
    real = inspect.signature(SharedDevices.measure_scope)
    mock = inspect.signature(MockSharedDevices.measure_scope)
    assert list(real.parameters) == list(mock.parameters)
    assert all(real.parameters[n].default == mock.parameters[n].default for n in real.parameters)


def test_mock_lifecycle_and_result_keys(monkeypatch):
    mock = MockSharedDevices(use_scope=True)
    with pytest.raises(RuntimeError):
        asyncio.run(mock.measure_scope())
    asyncio.run(mock.initialize())
    for bad in ({"items": []}, {"items": ["vpp", "vpp"]}, {"items": ["volts"]}, {"samples": 0}):
        with pytest.raises(ValueError):
            asyncio.run(mock.measure_scope(**bad))
    mock_result = asyncio.run(mock.measure_scope(items=["vpp", "pdutycycle"]))
    assert mock_result["scope_units"] == {"vpp": "V", "pdutycycle": "%"}
    asyncio.run(mock.cleanup())
    with pytest.raises(RuntimeError):
        asyncio.run(mock.measure_scope())

    _patch_driver(monkeypatch, FakeScope())
    real_result = _measure(_shared(), items=["vpp"], samples=1)
    assert set(mock_result) == set(real_result)


# ----------------------------------------------------------------------
# 7. Schema, executor, runner wiring
# ----------------------------------------------------------------------
def _validate(step):
    return ExperimentWorkflow.model_validate({"name": "t", "steps": [step]}).steps[0]


def test_schema_defaults_and_bounds():
    step = _validate({"action": "measure_scope"})
    assert isinstance(step, ActionMeasureScope)
    assert (step.channel, step.items, step.samples, step.interval) == (1, ["vpp", "frequency"], 3, 0.5)
    for bad in ({"channel": 0}, {"channel": 5}, {"items": []}, {"items": ["vpp", "vpp"]},
                {"items": ["volts"]}, {"items": ["vpp"] * 11}, {"samples": 0}, {"samples": 21},
                {"interval": -1}, {"interval": 61}, {"robot_id": 1}):
        with pytest.raises(ValidationError):
            _validate({"action": "measure_scope", **bad})


class RecordingShared:
    def __init__(self):
        self.calls = []

    async def measure_scope(self, **kwargs):
        self.calls.append(kwargs)
        return {"scope_channel": kwargs["channel"], "scope_values": {"vpp": 1.0},
                "scope_units": {"vpp": "V"}}


@pytest.mark.parametrize("as_model", [False, True])
def test_executor_passes_every_parameter(as_model):
    step = {"action": "measure_scope", "channel": 2, "items": ["vrms", "period"],
            "samples": 4, "interval": 0.25}
    shared = RecordingShared()
    asyncio.run(execute_step(_validate(step) if as_model else step, {}, shared))
    assert shared.calls == [{"channel": 2, "items": ["vrms", "period"], "samples": 4,
                             "interval": 0.25}]
    assert "measure_scope" in SHARED_DEVICE_ACTIONS


def test_plan_resources_needs_scope_and_no_robot():
    plan = run_flow.plan_resources([{"action": "measure_scope"}])
    assert plan.robot_ids == [] and plan.needs_scope and plan.needs_shared
    assert plan.shared_kwargs()["use_scope"] is True and "オシロ" in plan.shared_labels()


def test_scope_resource_priority(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.setattr(run_flow.lab_config, "get_shared_devices",
                        lambda: {"scope_resource": "USB0::A::INSTR"})
    monkeypatch.delenv("SCOPE_RESOURCE", raising=False)
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["scope_resource"] == "USB0::A::INSTR"
    monkeypatch.setenv("SCOPE_RESOURCE", "USB0::B::INSTR")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["scope_resource"] == "USB0::B::INSTR"
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--scope-resource",
                                                          "USB0::C::INSTR"]))
    assert shared["scope_resource"] == "USB0::C::INSTR"


def test_session_factories_forward_scope_settings():
    from src.flow.experiment_session import default_shared_factory, mock_shared_factory
    config = {"scale_port": "", "camera_index": 0, "scope_resource": RESOURCE,
              "scope_timeout_ms": 2345, "wait_after_scope": 0.5}
    shared = default_shared_factory(use_scale=False, use_camera=False, config=config,
                                    use_scope=True)
    assert shared.use_scope and shared.device_configs["scope_resource"] == RESOURCE
    assert shared._scope_timeout_ms() == 2345 and shared.wait_after_scope == 0.5
    assert mock_shared_factory(use_scale=False, use_camera=False, config=config,
                               use_scope=True).use_scope


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs
    return max(runs, key=os.path.getmtime)


def _rows(run_dir):
    with open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def test_example_flow_validates_and_mock_runs_without_a_robot(tmp_path, monkeypatch):
    assert run_flow.main([EXAMPLE, "--validate-only"]) == 0
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE, "--mock"]) == 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "completed" and meta["robots_used"] == []
    assert meta["scope_measurements"] == 2
    assert [r["step_index"] for r in meta["scope_readings"]] == [1, 2]
    assert meta["scope_readings"][0]["raw"]["vpp"] == ["2.0"] * 3
    rows = _rows(run_dir)
    assert json.loads(rows[0]["scope_values"]) == {"vpp": 2.0, "vaverage": 0.0, "frequency": 1000.0}
    assert json.loads(rows[1]["scope_units"]) == {"pdutycycle": "%", "risetime": "s"}
    assert rows[0]["scope_channel"] == "1"
    assert "オシロ測定値" in open(os.path.join(run_dir, "summary.md"), encoding="utf-8").read()


def test_gui_runner_mock_run_records_scope(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE, encoding="utf-8"))
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path), source_path=EXAMPLE)
    runner.start()
    assert runner.join(60)
    assert runner.status == "completed", runner.error
    assert all(r["scope_values"] for r in _rows(runner.log_dir))


DIAG = {"settings_diff": {"ch_scale": [1.0, 0.5]}, "values": {"vpp": 2.0}}


async def _failing_measure(self, channel=1, items=("vpp", "frequency"), samples=3, interval=0.5):
    raise ScopeStepError("オシロスコープの設定が測定中に変わりました: ['ch_scale']", DIAG)


def test_run_flow_records_diagnostics_of_a_failed_step(tmp_path, monkeypatch):
    monkeypatch.setattr(MockSharedDevices, "measure_scope", _failing_measure)
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE, "--mock"]) != 0
    run_dir = _latest_run_dir(str(tmp_path / "logs"))
    failed = [r for r in _rows(run_dir) if r["status"] == "error"]
    assert json.loads(failed[0]["diagnostics"]) == DIAG
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["step_diagnostics"][0]["diagnostics"] == DIAG


def test_logger_keeps_full_precision(tmp_path):
    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    exp.record_step(1, 1, "measure_scope", None, None, "ok", 0.1, result={
        "scope_channel": 3, "scope_values": {"vpp": 1.234567e-3, "risetime": None},
        "scope_units": {"vpp": "V", "risetime": "s"}, "scope_valid": {"vpp": 2, "risetime": 0},
        "scope_invalid": {"risetime": {"bound": 2}}, "scope_settings": {"tb_scale": 1e-3}})
    exp.finalize("completed")
    row = _rows(exp.dir)[0]
    assert json.loads(row["scope_values"]) == {"vpp": 1.234567e-3, "risetime": None}
    assert json.loads(row["scope_invalid"]) == {"risetime": {"bound": 2}}
    assert any(e["kind"] == "scope" for e in exp.events)
    assert "| 3 | vpp | 0.001234567 | V | 2 |" in open(exp.summary_path, encoding="utf-8").read()


# ----------------------------------------------------------------------
# 8. Review and bench fixes
# ----------------------------------------------------------------------
def test_every_item_maps_to_a_header_the_guide_lists():
    from src.devices.dso1000 import ITEMS
    assert {h for h, _ in ITEMS.values()} == set(MEASURED)
    _patch_driver_res = FakeScope()
    dso, res = make_driver(_patch_driver_res)
    for item in ITEMS:
        dso.measure(item, 1)                       # the fake refuses unknown headers


def test_bench_reply_forms_parse():
    """Replies seen on a DSO1014A (firmware 00.04.04)."""
    dso, _ = make_driver(FakeScope(**{":TIMebase:SCALe?": "1.000e-06",
                                      ":TIMebase:OFFSet?": "-2.925e-03",
                                      ":TRIGger:SENSitivity?": "3.00e-01",
                                      ":TRIGger:EDGE:LEVel?": "-6.00e-03",
                                      ":CHANnel1:PROBe?": "1X", ":KEY:LOCK?": "ENABLE"}))
    snap = dso.snapshot(1)
    assert snap["tb_scale"] == 1e-6 and snap["trig_sensitivity"] == 0.3 and snap["key_lock"] == "ENAB"


def test_settings_after_the_reads_are_kept_on_success(monkeypatch, tmp_path):
    res = FakeScope()
    res.after_measure = {1: {":MEASure:SOURce?": "CHANnel2", ":TRIGger:STATus?": "STOP",
                             ":ACQuire:SRATe?": "5.000e007"}}
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), items=["vpp"], samples=2, interval=0.0)
    assert result["scope_settings"]["meas_source"] == "CHANnel1"
    after = result["scope_settings_after"]
    assert (after["meas_source"], after["status"], after["srate"]) == ("CHANnel2", "STOP", 5e7)

    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    exp.record_step(1, 1, "measure_scope", None, None, "ok", 0.1, result=result)
    exp.finalize("completed")
    meta = json.load(open(exp.metadata_path, encoding="utf-8"))
    assert meta["scope_readings"][0]["settings_after"]["status"] == "STOP"


def test_io_error_keeps_what_was_read_in_diagnostics(monkeypatch):
    class Lost(FakeScope):
        def query(self, command):
            if command == ":MEASure:FREQuency? CHANnel1":
                self.log.append(("q", command))
                raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    _patch_driver(monkeypatch, Lost())
    with pytest.raises(RuntimeError) as info:
        _measure(_shared(), items=["vpp", "frequency"], samples=1)
    diag = info.value.diagnostics
    assert diag["raw"] == {"vpp": ["2.000e000"], "frequency": []}
    assert diag["settings_before"]["ch_scale"] == 1.0 and diag["error_type"] == "RuntimeError"
    assert "settings_after" not in diag


def test_instrument_error_keeps_both_snapshots_in_diagnostics(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        assert await shared.initialize()
        res.errors = ['45,"Measure already selected"']
        with pytest.raises(DSO1000Error) as info:
            await shared.measure_scope(items=["vpp"], samples=1)
        await shared.cleanup()
        return info.value.diagnostics

    diag = asyncio.run(main())
    assert diag["raw"] == {"vpp": ["2.000e000"]} and "settings_after" in diag


def test_wait_after_scope_is_after_the_last_query(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    order = []
    real_sleep = asyncio.sleep

    async def fake_sleep(seconds, *a, **k):
        if seconds == 1.5:
            order.append(("sleep", len(res.sent())))
        await real_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", fake_sleep)
    shared = _shared(wait_after_scope=1.5)

    async def main():
        assert await shared.initialize()
        await shared.measure_scope(items=["vpp"], samples=2, interval=0.0)
        order.append(("done", len(res.sent())))
        await shared.cleanup()

    asyncio.run(main())
    assert order[0][0] == "sleep" and order[0][1] == order[1][1]   # nothing sent after the sleep
    last_measure = max(i for i, c in enumerate(res.sent()) if c.startswith(":MEASure:VPP"))
    assert last_measure < order[0][1]


def test_cancel_during_the_last_query_sends_no_snapshot(monkeypatch):
    res, entered, release = _blocking(":MEASure:VPP? CHANnel1")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_scope(items=["vpp"], samples=1))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        release.set()
        # the worker must stop on its own: wait for it before cleanup() raises the closing flag
        assert await asyncio.to_thread(shared._scope_lock.acquire, True, 5)
        shared._scope_lock.release()
        assert not shared._scope_closing.is_set()
        before_cleanup = list(res.log)
        await shared.cleanup()
        return before_cleanup

    before_cleanup = asyncio.run(main())
    tail = before_cleanup[before_cleanup.index(("q", ":MEASure:VPP? CHANnel1")) + 1:]
    assert tail == [], tail                                  # nothing after the cancelled query
    after = res.log[res.log.index(("q", ":MEASure:VPP? CHANnel1")) + 1:]
    assert after == RELEASE_TAIL, after


def test_no_gtl_is_sent(monkeypatch):
    res = FakeScope()
    _patch_driver(monkeypatch, res)
    _measure(_shared(), items=["vpp"], samples=1)
    assert not any(str(e).startswith("ren:") for e in res.log)


@pytest.mark.parametrize("failing, key", [(":CHANnel1:OFFSet?", "settings_before"),
                                          (":TRIGger:HOLDoff?", "settings_after")])
def test_io_error_inside_a_snapshot_keeps_the_settings_read_so_far(monkeypatch, failing, key):
    class Lost(FakeScope):
        calls = 0

        def query(self, command):
            if command == failing:
                Lost.calls += 1
                if key == "settings_before" or Lost.calls == 2:
                    self.log.append(("q", command))
                    raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    _patch_driver(monkeypatch, Lost())
    with pytest.raises(RuntimeError) as info:
        _measure(_shared(), items=["vpp"], samples=1)
    partial = info.value.diagnostics[key]
    assert partial["ch_scale"] == 1.0
    if key == "settings_before":
        assert "ch_offset" not in partial and "raw" not in info.value.diagnostics
    else:
        assert info.value.diagnostics["raw"] == {"vpp": ["2.000e000"]}
        assert "trig_holdoff" not in partial and "trig_coupling" in partial


def test_io_error_while_reading_the_error_queue_keeps_the_errors_read(monkeypatch):
    class Lost(FakeScope):
        def query(self, command):
            if command == ":SYSTem:ERRor?" and not self.errors and getattr(self, "armed", False):
                self.log.append(("q", command))
                raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    res = Lost()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        assert await shared.initialize()
        res.errors, res.armed = ['-113,"Undefined header"'], True
        with pytest.raises(RuntimeError) as info:
            await shared.measure_scope(items=["vpp"], samples=1)
        await shared.cleanup()
        return info.value.diagnostics

    diag = asyncio.run(main())
    assert diag["errors_after"] == ['-113,"Undefined header"']


def test_bench_measurement_replies_on_a_real_signal():
    """Replies of a DSO1014A (firmware 00.04.04) to a signal on CH1: no % on duty cycle."""
    from src.devices.dso1000.dso1000 import parse_measurement
    assert parse_measurement("8.880e-001", "vpp") == (0.888, None)
    assert parse_measurement("3.048e002", "frequency") == (304.8, None)
    assert parse_measurement("5.000e001", "pdutycycle") == (50.0, None)
    assert parse_measurement("5.121e001", "ndutycycle") == (51.21, None)
    assert parse_measurement("0.000e000", "overshoot") == (0.0, None)
    assert parse_measurement("9.090e-003", "preshoot") == (0.00909, None)
    assert parse_measurement("-4.320e-001", "vmin") == (-0.432, None)

    dso, _ = make_driver(FakeScope(**{":TRIGger:STATus?": "T'D"}))
    assert dso.snapshot(1)["status"] == "T'D"
