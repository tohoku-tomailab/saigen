"""NF LI5640 lock-in amplifier: driver, SharedDevices wrapper, flow wiring, logging.

No hardware and no PyVISA: the driver takes an injected resource manager. The
fake instrument answers by header from a settings table, so a test can change a
panel setting in the middle of a step, return over-level / UNLOCK states,
malformed replies, or block on one command to test cancellation.
"""
import asyncio
import csv
import inspect
import itertools
import json
import os
import threading
import time

import pytest
from pydantic import ValidationError

import src.devices.keithley2002 as k2002_pkg
import src.devices.li5640 as li_pkg
from src.devices.keithley2002.keithley2002 import Keithley2002
from src.devices.li5640.li5640 import (
    LI5640, LI5640Error, LockinStepError, describe, format_si, invalid_reason, parse_dout,
    parse_output_type, parse_warning, sensitivity_value, theta_median, time_constant_value,
)
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.executor import SHARED_DEVICE_ACTIONS, execute_step
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import ActionMeasureLockin, ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(REPO_ROOT, "examples", "li5640", "read_lockin.json")
IDN = "NF CORPORATION,LI5640,1234567,1.00"
RESOURCE = "GPIB0::7::INSTR"

#: a valid panel state: X / Y display, voltage input A, 100 mV, 300 ms, 24 dB/oct
PANEL = {
    "ddef1": "0", "ddef2": "0", "isrc": "0", "norm": "0", "vstd": "1.0000E+00",
    "istd": "1.0000E-06", "rat": "0", "kfac": "1.0000", "ofso1": "0", "ofso2": "0",
    "offs1": "0.00", "offs2": "0.00", "oexp1": "0", "oexp2": "0", "phas": "0.00",
    "icpl": "0", "ignd": "0", "vsen": "23", "isen": "26", "tcon": "9", "slop": "3",
    "sync": "0", "nois": "1", "ilin": "0", "ifrq": "0", "ithr": "0", "drsv": "2",
    "rsrc": "0", "redg": "0", "harm": "1", "opcr": "0", "ostr": "0",
}


# ----------------------------------------------------------------------
# Fakes
# ----------------------------------------------------------------------
class FakeLI:
    """A VISA resource that behaves like an LI5640 and records the bus traffic."""

    def __init__(self, douts=(), warnings=(), errors=(), idn=IDN, otyp="1,2", **panel):
        self.panel = {**PANEL, **panel}
        self.douts = list(douts)            # DOUT? replies in order (then a default)
        self.warnings = list(warnings)      # WRCR? replies in order (then "0")
        self.errors = list(errors)          # EROR? replies before the queue is empty
        self.idn = idn
        self.otyp = otyp
        self.after_dout = {}                # {n: {key: value}} panel change after the n-th DOUT?
        self.n_dout = 0
        self.log = []                       # ("w", cmd) / ("q", cmd) / "clear" / "gtl" / "close"
        self.closed = False
        self.io_after_close = False
        self.timeout = None
        self.read_termination = "unset"
        self.write_termination = "unset"

    def _check_open(self, command):
        if self.closed:
            self.io_after_close = True
            raise AssertionError(f"I/O after close: {command!r}")

    def write(self, command):
        self._check_open(command)
        self.log.append(("w", command))
        if command.startswith("OTYP "):
            self.otyp = command[5:].replace(" ", "")
        elif command != "*CLS":
            raise AssertionError(f"unexpected write {command!r}")

    def query(self, command):
        self._check_open(command)
        self.log.append(("q", command))
        if command == "*IDN?":
            return self.idn + "\r\n"
        if command == "EROR?":
            return self.errors.pop(0) if self.errors else '0,"No error"'
        if command == "OTYP?":
            return self.otyp
        if command == "DOUT?":
            self.n_dout += 1
            reply = self.douts.pop(0) if self.douts else "+1.0000E-03,+2.0000E-05,+1.0000E+03,0"
            self.panel.update(self.after_dout.get(self.n_dout, {}))
            return reply
        if command == "WRCR?":
            return self.warnings.pop(0) if self.warnings else "0"
        key = command.replace("?", "").replace(" ", "").lower()
        if key in self.panel:
            return self.panel[key]
        raise AssertionError(f"unexpected query {command!r}")

    def clear(self):
        self.log.append("clear")

    def control_ren(self, mode):
        # only the GTL addressed to this instrument (VI_GPIB_REN_ADDRESS_GTL = 6) counts;
        # deasserting REN for the whole bus would send every instrument to local
        self.log.append("gtl" if int(mode) == 6 else f"ren:{int(mode)}")

    def close(self):
        self.log.append("close")
        self.closed = True

    def writes(self):
        return [c for kind, c in (e for e in self.log if isinstance(e, tuple)) if kind == "w"]

    def sent(self):
        return [c for kind, c in (e for e in self.log if isinstance(e, tuple))]


class FakeRM:
    def __init__(self, resource):
        self.resource = resource

    def open_resource(self, name):
        return self.resource


def make_driver(res=None):
    res = res or FakeLI()
    return LI5640(RESOURCE, resource_manager=FakeRM(res)), res


def _patch_driver(monkeypatch, res):
    monkeypatch.setattr(
        li_pkg, "LI5640",
        lambda name, timeout_ms=5000: LI5640(name, timeout_ms=timeout_ms,
                                             resource_manager=FakeRM(res)))


def _shared(**configs):
    return SharedDevices(use_lockin=True, lockin_resource=RESOURCE, **configs)


def _measure(shared, **kwargs):
    async def main():
        assert await shared.initialize()
        try:
            return await shared.measure_lockin(**kwargs)
        finally:
            await shared.cleanup()
    return asyncio.run(main())


RELEASE_TAIL = ["clear", ("w", "OTYP 1,2"), ("q", "OTYP?"), "gtl", "close"]


# ----------------------------------------------------------------------
# 1. Pure functions
# ----------------------------------------------------------------------
@pytest.mark.parametrize("code, current, value", [
    (0, False, 2e-9), (8, False, 1e-6), (23, False, 0.1), (26, False, 1.0),
    (1, True, 5e-15), (26, True, 1e-6), (0, True, None), (27, False, None), (-1, False, None),
])
def test_sensitivity_table(code, current, value):
    got = sensitivity_value(code, current)
    assert got == pytest.approx(value) if value is not None else got is None


@pytest.mark.parametrize("code, seconds", [(0, 1e-5), (1, 3e-5), (9, 0.3), (10, 1.0),
                                           (19, 3e4), (20, None)])
def test_time_constant_table(code, seconds):
    got = time_constant_value(code)
    assert got == pytest.approx(seconds) if seconds is not None else got is None


def test_format_si():
    assert format_si(1e-7, "V") == "100 nV"
    assert format_si(0.3, "s") == "300 ms"
    assert format_si(5e-15, "A") == "5 fA"
    assert format_si(None, "V") == ""


@pytest.mark.parametrize("reply, expected", [
    ("+1.2345E-06,-0.7890E-06,+1.0000E+03,0", (1.2345e-06, -0.789e-06, 1000.0, 0)),
    (" 1E-3 , 2E-5 , 1000 , 12 \r\n", (1e-3, 2e-5, 1000.0, 12)),
    ("1E-3,2E-5,1000", None),                      # wrong number of fields
    ("1E-3,2E-5,1000,0,5", None),
    ("NAN,2E-5,1000,0", None),                     # non-finite values are not readings
    ("1E-3,INF,1000,0", None),
    ("1E-3,2E-5,-inf,0", None),
    ("1E-3,2E-5,1000,1.5", None),                  # OVERLEVEL must be an integer
    ("1E-3,2E-5,1000,64", None),                   # ... within its six bits
    ("1E-3,2E-5,1000,-1", None),
    ("garbage", None),
])
def test_parse_dout(reply, expected):
    got = parse_dout(reply)
    if expected is None:
        assert got is None
    else:
        assert (got["data1"], got["data2"], got["freq"], got["overlevel"]) == expected


@pytest.mark.parametrize("reply, expected", [("0", 0), ("3", 3), ("4", None), ("-1", None),
                                             ("1.0", None), ("x", None), ("", None)])
def test_parse_warning(reply, expected):
    assert parse_warning(reply) == expected


def test_parse_output_type():
    assert parse_output_type(" 1, 2 ") == "1,2"
    for bad in ("", "6", "1,2,3,4,5,0,1", "a"):
        with pytest.raises(LI5640Error):
            parse_output_type(bad)


def _snap(**overrides):
    s = {"ddef1": 0, "ddef2": 0, "isrc": 0, "norm": 0, "rat": 0}
    s.update(overrides)
    return s


@pytest.mark.parametrize("snap, expected", [
    # plain X / Y, voltage and current input
    (_snap(), ("X", "plain", "V", "Y", "plain", "V")),
    (_snap(isrc=1), ("X", "plain", "V", "Y", "plain", "V")),
    (_snap(isrc=2), ("X", "plain", "A", "Y", "plain", "A")),
    (_snap(isrc=3, ddef1=1, ddef2=1), ("R", "plain", "A", "THETA", "plain", "deg")),
    # NOISE forces DATA2 to AUX IN1 whatever DDEF? 2 says (4-34); its unit is per √Hz (4-35)
    (_snap(ddef1=2, ddef2=1), ("NOISE", "plain", "V/√Hz", "AUX1", "plain", "V")),
    (_snap(ddef1=2, ddef2=3, isrc=2), ("NOISE", "plain", "A/√Hz", "AUX1", "plain", "V")),
    # normalize applies to X / R (4-32), not to NOISE / AUX; Y is not asserted
    (_snap(norm=1), ("X", "dB", "dB", "Y", "normalize?", "")),
    (_snap(norm=2, ddef1=1, ddef2=1), ("R", "%", "%", "THETA", "plain", "deg")),
    (_snap(norm=1, ddef1=2), ("NOISE", "plain", "V/√Hz", "AUX1", "plain", "V")),
    (_snap(norm=1, ddef1=3, ddef2=3), ("AUX1", "plain", "V", "AUX2", "plain", "V")),
    # ratio applies to X / Y / R and wins over normalize (4-33)
    (_snap(rat=1), ("X", "ratio", "", "Y", "ratio", "")),
    (_snap(rat=1, norm=1, ddef1=1, ddef2=1), ("R", "ratio", "", "THETA", "plain", "deg")),
    (_snap(rat=1, ddef1=2), ("NOISE", "plain", "V/√Hz", "AUX1", "plain", "V")),
])
def test_describe_follows_the_manual(snap, expected):
    d = describe(snap)
    assert (d["data1_kind"], d["data1_mode"], d["data1_unit"],
            d["data2_kind"], d["data2_mode"], d["data2_unit"]) == expected


@pytest.mark.parametrize("over, warn, mode1, mode2, reason", [
    (0, 0, "plain", "plain", None),
    (1, 0, "plain", "plain", "overlevel_input"),
    (2, 0, "plain", "plain", "overlevel_input"),
    (4, 0, "plain", "plain", "overlevel_display"),
    (8, 0, "plain", "plain", "overlevel_display"),
    (16, 0, "plain", "plain", None),               # ratio over-level while ratio is off
    (16, 0, "ratio", "plain", "overlevel_display"),
    (32, 0, "plain", "ratio", "overlevel_display"),
    (4, 0, "ratio", "plain", None),                # in ratio mode the ratio bit counts
    (0, 1, "plain", "plain", "unlock"),
    (0, 2, "plain", "plain", "unlock"),
    (0, None, "plain", "plain", "status_unparsable"),
])
def test_invalid_reason(over, warn, mode1, mode2, reason):
    desc = {"data1_mode": mode1, "data2_mode": mode2}
    sample = {"data1": 0.0, "data2": 0.0, "freq": 1.0, "overlevel": over}
    assert invalid_reason(sample, warn, desc) == reason
    assert invalid_reason(None, 0, desc) == "unparsable"


@pytest.mark.parametrize("values, expected", [
    ([10.0], 10.0),
    ([10.0] * 5, 10.0),                            # a steady phase is not "spread out"
    ([180.0, -180.0], -180.0),                     # the same angle written twice
    ([179.0, 179.0, -179.0], 179.0),
    ([5.0, 5.0, -5.0, -5.0], 0.0),
    ([179.0, -179.0, 178.0], 179.0),               # straddles ±180
    ([170.0, -170.0], -180.0),                     # even count: mean of the middle pair, wrapped
    ([-10.0, 10.0, 0.0, 5.0], 2.5),
    ([0.0, 179.0, -179.0], None),                  # no arc narrower than 180° holds them
    ([0.0, 180.0], None),                          # opposite angles
    ([0.0, 120.0, -120.0], None),                  # wide scatter
])
def test_theta_median_is_order_independent(values, expected):
    for perm in itertools.permutations(values):
        got = theta_median(list(perm))
        assert got == pytest.approx(expected) if expected is not None else got is None, perm


# ----------------------------------------------------------------------
# 2. Driver
# ----------------------------------------------------------------------
def test_constructor_reads_on_eoi_and_closes_on_setup_failure():
    lia, res = make_driver()
    assert res.read_termination is None and res.write_termination == "\n"

    class Broken(FakeLI):
        def __setattr__(self, name, value):
            if name == "read_termination" and value is None:   # the driver's setting
                raise RuntimeError("bad attribute")
            super().__setattr__(name, value)

    broken = Broken()
    with pytest.raises(RuntimeError):
        LI5640(RESOURCE, resource_manager=FakeRM(broken))
    assert broken.closed


@pytest.mark.parametrize("idn, ok", [
    ("NF CORPORATION,LI5640,1234567,1.00", True),
    ("LI5640,1234567,1.00", True),                 # the manual's example omits the maker
    ("NF CORPORATION,LI5650,1,1.00", False),
    ("KEITHLEY INSTRUMENTS INC.,MODEL 2002,1,A09", False),
])
def test_identify(idn, ok):
    lia, _ = make_driver(FakeLI(idn=idn))
    if ok:
        assert lia.identify() == idn
    else:
        with pytest.raises(LI5640Error):
            lia.identify()


def test_error_queue_of_twenty_is_read_to_the_end():
    lia, res = make_driver(FakeLI(errors=[f'-113,"Undefined header {i}"' for i in range(20)]))
    with pytest.raises(LI5640Error, match="Undefined header 19"):
        lia.check_errors()
    assert res.sent().count("EROR?") == 21          # 20 entries + the terminating 0


def test_error_queue_that_never_empties_is_an_error():
    lia, res = make_driver(FakeLI(errors=['-350,"Queue overflow"'] * 30))
    with pytest.raises(LI5640Error, match="空になりません"):
        lia.check_errors()
    assert res.sent().count("EROR?") == 21


def test_snapshot_rejects_out_of_range_and_unparsable_settings():
    for key, bad, isrc in (("ddef1", "7", "0"), ("tcon", "abc", "0"), ("phas", "nan", "0"),
                           ("kfac", "12.0", "0"), ("kfac", "0", "0"), ("vstd", "2.0", "0"),
                           ("vstd", "0", "0"), ("istd", "2E-06", "2"), ("isen", "0", "2")):
        lia, _ = make_driver(FakeLI(isrc=isrc, **{key: bad}))
        with pytest.raises(LI5640Error):
            lia.snapshot()


def test_snapshot_reads_the_current_settings_only_for_the_selected_input():
    _, res = make_driver(FakeLI(isrc="2"))
    lia = LI5640(RESOURCE, resource_manager=FakeRM(res))
    snap = lia.snapshot()
    assert "isen" in snap and "istd" in snap and "vsen" not in snap and "vstd" not in snap
    assert res.writes() == []                       # a snapshot never writes


def test_release_restores_otyp_even_when_aborted():
    lia, res = make_driver(FakeLI(otyp="1,2,3,5"))
    lia.abort_check = lambda: True                  # cleanup is not gated by the abort
    assert lia.release("1,2")
    assert res.log == RELEASE_TAIL


def test_release_goes_on_to_gtl_and_close_when_the_restore_fails(caplog):
    class NoOtyp(FakeLI):
        def write(self, command):
            if command.startswith("OTYP"):
                raise RuntimeError("VI_ERROR_TMO")
            super().write(command)

    lia, res = make_driver(NoOtyp(otyp="1,2,3,5"))
    assert lia.release("1,2")
    # the write may have reached the instrument: it is still read back, then local and close
    assert res.log == ["clear", ("q", "OTYP?"), "gtl", "close"]
    assert "OTYP を戻せませんでした" in caplog.text


def test_release_goes_on_when_the_read_back_fails(caplog):
    class NoReadBack(FakeLI):
        def query(self, command):
            if command == "OTYP?":
                raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    lia, res = make_driver(NoReadBack())
    assert lia.release("1,2")
    assert res.log == ["clear", ("w", "OTYP 1,2"), "gtl", "close"]
    assert "読み返し失敗" in caplog.text


def test_release_reports_a_restore_that_does_not_read_back(caplog):
    class Sticky(FakeLI):
        def query(self, command):
            reply = super().query(command)
            return "1,2,3,5" if command == "OTYP?" else reply

    lia, res = make_driver(Sticky())
    lia.release("1,2")
    assert "OTYP を戻せませんでした" in caplog.text and res.closed


# ----------------------------------------------------------------------
# 3. SharedDevices: initialisation
# ----------------------------------------------------------------------
def test_initialize_without_resource_fails_without_opening(monkeypatch):
    opened = []
    monkeypatch.setattr(li_pkg, "LI5640", lambda *a, **k: opened.append(a))
    shared = SharedDevices(use_lockin=True)
    assert asyncio.run(shared.initialize()) is False
    assert opened == [] and shared.lockin is None


def test_initialize_checks_and_remembers_otyp_without_writing_settings(monkeypatch):
    res = FakeLI(otyp="1,2,3")
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize())
    assert res.log[0] == "clear"
    assert res.sent() == ["*CLS", "*IDN?", "EROR?", "OSTR?", "OTYP?"]
    assert res.writes() == ["*CLS"] and shared._lockin_saved_otyp == "1,2,3"
    asyncio.run(shared.cleanup())
    assert res.log[-5:] == ["clear", ("w", "OTYP 1,2,3"), ("q", "OTYP?"), "gtl", "close"]


@pytest.mark.parametrize("res", [FakeLI(ostr="1"), FakeLI(idn="NF CORPORATION,LI5650,1,1"),
                                 FakeLI(errors=['-113,"Undefined header"'])])
def test_failed_initialisation_closes_without_restoring_an_unread_otyp(monkeypatch, res):
    _patch_driver(monkeypatch, res)
    shared = _shared()
    assert asyncio.run(shared.initialize()) is False
    assert res.closed and shared.lockin is None
    assert not any(c.startswith("OTYP") for c in res.sent())    # never read, never written
    assert res.log[-2:] == ["gtl", "close"]


# ----------------------------------------------------------------------
# 4. SharedDevices: measurement
# ----------------------------------------------------------------------
def test_measure_returns_medians_kinds_units_and_settings(monkeypatch):
    res = FakeLI(douts=["1.0E-03,2.0E-05,1000.0,0", "3.0E-03,1.0E-05,1001.0,0",
                        "2.0E-03,3.0E-05,999.0,0"])
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), samples=3)
    assert result["lockin_data1"] == 2.0e-03 and result["lockin_data2"] == 2.0e-05
    assert result["lockin_freq"] == 1000.0
    assert (result["lockin_data1_kind"], result["lockin_data1_unit"]) == ("X", "V")
    assert (result["lockin_data2_kind"], result["lockin_data2_unit"]) == ("Y", "V")
    assert result["lockin_sensitivity"] == "100 mV"
    assert result["lockin_time_constant"] == "300 ms"
    assert result["lockin_valid_samples"] == 3 and result["lockin_invalid"] == {}
    assert result["lockin_settings"]["tcon"] == 9 and result["lockin_settings"]["slop"] == 3
    # the only setting ever written is OTYP, and it is put back
    assert set(res.writes()) == {"*CLS", "OTYP 1,2,3,5", "OTYP 1,2"}
    assert res.log[-5:] == RELEASE_TAIL and res.otyp == "1,2"
    assert res.sent().count("DOUT?") == 3 and res.sent().count("WRCR?") == 3


def test_invalid_samples_are_dropped_and_counted(monkeypatch):
    res = FakeLI(douts=["1.0E-03,2.0E-05,1000,1", "5.0E-03,2.0E-05,1000,0", "NAN,1,1,0",
                        "7.0E-03,2.0E-05,1000,0", "9.0E-03,2.0E-05,1000,8"],
                 warnings=["0", "0", "0", "1", "0"])
    _patch_driver(monkeypatch, res)
    result = _measure(_shared(), samples=5)
    assert result["lockin_valid_samples"] == 1 and result["lockin_data1"] == 5.0e-03
    assert result["lockin_invalid"] == {"overlevel_input": 1, "unparsable": 1, "unlock": 1,
                                        "overlevel_display": 1}


def test_no_valid_sample_fails_with_diagnostics(monkeypatch):
    res = FakeLI(warnings=["1", "1"])
    _patch_driver(monkeypatch, res)
    with pytest.raises(LockinStepError) as info:
        _measure(_shared(), samples=2)
    assert info.value.diagnostics["invalid"] == {"unlock": 2}
    assert info.value.diagnostics["settings_before"]["vsen"] == 23
    assert res.log[-5:] == RELEASE_TAIL


def test_a_panel_change_during_the_step_fails_it_with_before_after_and_diff(monkeypatch):
    res = FakeLI()
    res.after_dout = {1: {"offs1": "12.50"}}        # offset amount changed, OFFSET ON unchanged
    _patch_driver(monkeypatch, res)
    with pytest.raises(LockinStepError) as info:
        _measure(_shared(), samples=3)
    diag = info.value.diagnostics
    assert diag["settings_diff"] == {"offs1": [0.0, 12.5]}
    assert diag["invalid"] == {}
    assert diag["settings_before"]["offs1"] == 0.0 and diag["settings_after"]["offs1"] == 12.5


def test_diagnostics_of_a_panel_change_keep_the_discarded_readings(monkeypatch):
    res = FakeLI(warnings=["0", "1", "0"])
    res.after_dout = {2: {"phas": "45.00"}}
    _patch_driver(monkeypatch, res)
    with pytest.raises(LockinStepError) as info:
        _measure(_shared(), samples=3)
    diag = info.value.diagnostics
    assert diag["settings_diff"] == {"phas": [0.0, 45.0]} and diag["invalid"] == {"unlock": 1}
    assert diag["display"]["data1_kind"] == "X"


def test_auto_setup_running_fails_the_step(monkeypatch):
    _patch_driver(monkeypatch, FakeLI(opcr="256"))
    with pytest.raises(LockinStepError, match="自動設定"):
        _measure(_shared(), samples=1)


def test_theta_uses_the_arc_median_and_ambiguous_phase_fails(monkeypatch):
    _patch_driver(monkeypatch, FakeLI(ddef1="1", ddef2="1", douts=[
        "1E-3,179.0,1000,0", "1E-3,-179.0,1000,0", "1E-3,178.0,1000,0"]))
    result = _measure(_shared(), samples=3)
    assert result["lockin_data2_kind"] == "THETA" and result["lockin_data2"] == 179.0
    assert result["lockin_data2_unit"] == "deg"

    _patch_driver(monkeypatch, FakeLI(ddef1="1", ddef2="1", douts=[
        "1E-3,0.0,1000,0", "1E-3,179.0,1000,0", "1E-3,-179.0,1000,0"]))
    with pytest.raises(LockinStepError, match="180°"):
        _measure(_shared(), samples=3)


def test_noise_display_reads_aux1_on_data2_and_labels_per_root_hz(monkeypatch):
    _patch_driver(monkeypatch, FakeLI(ddef1="2", ddef2="1", vsen="5"))
    result = _measure(_shared(), samples=1)
    assert (result["lockin_data1_kind"], result["lockin_data1_unit"]) == ("NOISE", "V/√Hz")
    assert (result["lockin_data2_kind"], result["lockin_data2_unit"]) == ("AUX1", "V")
    assert result["lockin_sensitivity"] == "100 nV/√Hz"


def test_normalize_and_ratio_are_named_in_the_kind(monkeypatch):
    _patch_driver(monkeypatch, FakeLI(norm="1"))
    result = _measure(_shared(), samples=1)
    assert (result["lockin_data1_kind"], result["lockin_data1_unit"]) == ("X[dB]", "dB")
    assert (result["lockin_data2_kind"], result["lockin_data2_unit"]) == ("Y[normalize?]", "")


def test_io_error_fails_the_step_and_cleanup_still_restores(monkeypatch):
    class Lost(FakeLI):
        def query(self, command):
            if command == "TCON?":
                raise RuntimeError("VI_ERROR_TMO")
            return super().query(command)

    res = Lost()
    _patch_driver(monkeypatch, res)
    with pytest.raises(RuntimeError, match="VI_ERROR_TMO") as info:
        _measure(_shared(), samples=3)
    assert not isinstance(info.value, LockinStepError)
    assert "DOUT?" not in res.sent()
    assert res.log[-5:] == RELEASE_TAIL


def test_otyp_set_timeout_is_followed_by_clear_and_restore(monkeypatch):
    """S3: a write that timed out may have left part of a command in the input
    buffer; the release clears the device before it writes the saved OTYP."""
    class TimesOut(FakeLI):
        def write(self, command):
            if command == "OTYP 1,2,3,5":
                self.log.append(("w", "OTYP 1,2,3,5 (partial)"))
                raise RuntimeError("VI_ERROR_TMO")
            super().write(command)

    res = TimesOut()
    _patch_driver(monkeypatch, res)
    with pytest.raises(RuntimeError, match="VI_ERROR_TMO"):
        _measure(_shared(), samples=1)
    tail = res.log[res.log.index(("w", "OTYP 1,2,3,5 (partial)")) + 1:]
    assert tail == RELEASE_TAIL


def test_wait_after_lockin_comes_after_the_measurement(monkeypatch):
    _patch_driver(monkeypatch, FakeLI())
    slept = []
    real_sleep = asyncio.sleep

    async def fake_sleep(seconds, *a, **k):
        slept.append(seconds)
        await real_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", fake_sleep)
    _measure(_shared(wait_after_lockin=2.5), samples=1)
    assert 2.5 in slept


def test_measure_requires_initialisation():
    with pytest.raises(RuntimeError):
        asyncio.run(_shared().measure_lockin())


# ----------------------------------------------------------------------
# 5. Cancellation and clean-up
# ----------------------------------------------------------------------
def _blocking(on_command, cls=FakeLI, **kwargs):
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


def test_cancel_during_dout_sends_nothing_more_and_cleanup_waits(monkeypatch):
    res, entered, release = _blocking("DOUT?")
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_lockin(samples=5))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        t0 = time.monotonic()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert time.monotonic() - t0 < 1.0
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.3)
        assert not res.closed, "closed while DOUT? was in flight"
        release.set()
        await cleanup

    asyncio.run(main())
    after = res.log[res.log.index(("q", "DOUT?")) + 1:]
    assert after == RELEASE_TAIL, after              # no WRCR?, no snapshot, then release
    assert not res.io_after_close


def test_cancel_during_the_interval_stops_at_once(monkeypatch):
    res = FakeLI()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_lockin(samples=3, interval=30.0))
        await _wait_until(lambda: res.sent().count("WRCR?") == 1)
        await asyncio.sleep(0.2)                    # now inside the 30 s interval
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        t0 = time.monotonic()
        await shared.cleanup()
        assert time.monotonic() - t0 < 2.0, "the interval must not delay the clean-up"

    asyncio.run(main())
    assert res.sent().count("DOUT?") == 1 and res.log[-5:] == RELEASE_TAIL


def test_cleanup_that_gives_up_waiting_leaves_the_release_to_the_worker(monkeypatch):
    res, entered, release = _blocking("DOUT?")
    _patch_driver(monkeypatch, res)
    shared = _shared()
    shared.lockin_close_wait_s = 0.1

    async def main():
        await shared.initialize()
        task = asyncio.create_task(shared.measure_lockin(samples=2))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await shared.cleanup()                      # gives up after 0.1 s
        assert not res.closed
        release.set()
        await _wait_until(lambda: res.closed)

    asyncio.run(main())
    assert res.log[-5:] == RELEASE_TAIL and shared.lockin is None and not res.io_after_close


def test_a_close_failure_is_logged_and_retried_with_the_restore(monkeypatch, caplog):
    class FailsOnce(FakeLI):
        def __init__(self):
            super().__init__()
            self.attempts = 0

        def close(self):
            self.attempts += 1
            if self.attempts == 1:
                self.log.append("close failed")
                raise RuntimeError("VI_ERROR_CONN_LOST")
            super().close()

    res = FailsOnce()
    _patch_driver(monkeypatch, res)
    shared = _shared()

    async def main():
        await shared.initialize()
        await shared.cleanup()
        assert shared.lockin is not None and not res.closed
        await shared.cleanup()

    asyncio.run(main())
    assert "閉じられません" in caplog.text
    assert res.closed and shared.lockin is None
    assert res.sent().count("OTYP 1,2") == 2         # the retry restores again


def test_recancel_during_cleanup_still_releases_the_other_instrument(monkeypatch):
    """S1: DMM and lock-in both open; Ctrl+C again while cleanup() waits for the
    DMM. The lock-in is still restored, sent to local and closed, and the
    cancellation still propagates."""
    dmm_reading, dmm_release = threading.Event(), threading.Event()

    from tests.test_keithley2002 import FakeResource, FakeRM as DmmRM

    class SlowDmm(FakeResource):
        def query(self, command):
            if command == ":READ?":
                dmm_reading.set()
                dmm_release.wait(10)
            return super().query(command)

    dmm_res = SlowDmm()
    monkeypatch.setattr(k2002_pkg, "Keithley2002", lambda name, timeout_ms=10000: Keithley2002(
        name, timeout_ms=timeout_ms, resource_manager=DmmRM(dmm_res)))
    li_res = FakeLI()
    _patch_driver(monkeypatch, li_res)
    shared = SharedDevices(use_dmm=True, dmm_resource="GPIB0::16::INSTR",
                           use_lockin=True, lockin_resource=RESOURCE)
    shared.dmm_close_wait_s = 10.0

    async def main():
        assert await shared.initialize()
        task = asyncio.create_task(shared.measure_dmm(samples=3))
        await asyncio.to_thread(dmm_reading.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.2)                    # waiting for the DMM's read
        cleanup.cancel()                            # second Ctrl+C
        with pytest.raises(asyncio.CancelledError):
            await cleanup
        assert li_res.closed and li_res.log[-5:] == RELEASE_TAIL
        dmm_release.set()
        await _wait_until(lambda: dmm_res.closed)

    asyncio.run(main())
    assert not dmm_res.io_after_close and not li_res.io_after_close


def test_cancelled_initialisation_sends_no_further_query_while_another_device_closes(
        monkeypatch):
    """Init cancelled while the LI5640 waits for *IDN?; cleanup() is then held up
    by the DMM's release. The LI5640 worker must not go on with EROR? / OSTR? /
    OTYP? once *IDN? returns."""
    from tests.test_keithley2002 import FakeResource, FakeRM as DmmRM
    gtl_entered, gtl_release = threading.Event(), threading.Event()

    class SlowGtl(FakeResource):
        def control_ren(self, mode):
            gtl_entered.set()
            gtl_release.wait(10)
            super().control_ren(mode)

    dmm_res = SlowGtl()
    monkeypatch.setattr(k2002_pkg, "Keithley2002", lambda name, timeout_ms=10000: Keithley2002(
        name, timeout_ms=timeout_ms, resource_manager=DmmRM(dmm_res)))
    li_res, idn_entered, idn_release = _blocking("*IDN?")
    _patch_driver(monkeypatch, li_res)
    shared = SharedDevices(use_dmm=True, dmm_resource="GPIB0::16::INSTR",
                           use_lockin=True, lockin_resource=RESOURCE)

    async def main():
        init = asyncio.create_task(shared.initialize())
        await asyncio.to_thread(idn_entered.wait, 5)
        init.cancel()
        with pytest.raises(asyncio.CancelledError):
            await init
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.to_thread(gtl_entered.wait, 5)       # DMM release is held up
        idn_release.set()
        await asyncio.sleep(0.3)
        after_idn = li_res.log[li_res.log.index(("q", "*IDN?")) + 1:]
        assert not any(isinstance(e, tuple) for e in after_idn), after_idn
        gtl_release.set()
        await cleanup

    asyncio.run(main())
    assert li_res.closed and dmm_res.closed and shared.lockin is None
    assert not any(c.startswith("OTYP") for c in li_res.sent())


def test_ctrl_c_during_lockin_initialisation_stops_robots_before_release(monkeypatch):
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

    class Recording(FakeLI):
        def close(self):
            events.append("lockin_close")
            super().close()

    res, entered, release = _blocking("*IDN?", cls=Recording)
    _patch_driver(monkeypatch, res)
    session = ExperimentSession(
        mock=True, shared_factory=default_shared_factory, robot_ports={1: {"type": "dobot"}},
        shared_config={"scale_port": "", "camera_index": 0, "lockin_resource": RESOURCE})

    async def body():
        await session.add_robot(1)
        await session.add_shared(use_lockin=True)

    async def main():
        task = asyncio.create_task(session.run(body))
        await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        await _wait_until(lambda: "emergency_stop" in events)
        assert not release.is_set(), "the e-stop must not wait for the instrument"
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())
    assert events.index("emergency_stop") < events.index("lockin_close")
    assert "go_home" not in events and session.status == "aborted"
    assert not any(c.startswith("OTYP") for c in res.sent())    # OTYP was never read
    assert res.closed and not res.io_after_close


# ----------------------------------------------------------------------
# 6. Mock parity
# ----------------------------------------------------------------------
def test_mock_has_the_real_signature():
    real = inspect.signature(SharedDevices.measure_lockin)
    mock = inspect.signature(MockSharedDevices.measure_lockin)
    assert list(real.parameters) == list(mock.parameters)
    assert all(real.parameters[n].default == mock.parameters[n].default for n in real.parameters)


def test_mock_lifecycle_and_result_keys(monkeypatch):
    mock = MockSharedDevices(use_lockin=True)
    with pytest.raises(RuntimeError):
        asyncio.run(mock.measure_lockin())
    asyncio.run(mock.initialize())
    with pytest.raises(ValueError):
        asyncio.run(mock.measure_lockin(samples=0))
    mock_result = asyncio.run(mock.measure_lockin(samples=2))
    asyncio.run(mock.cleanup())
    with pytest.raises(RuntimeError):
        asyncio.run(mock.measure_lockin())

    _patch_driver(monkeypatch, FakeLI())
    real_result = _measure(_shared(), samples=1)
    assert set(mock_result) == set(real_result)


# ----------------------------------------------------------------------
# 7. Schema, executor, runner wiring
# ----------------------------------------------------------------------
def _validate(step):
    return ExperimentWorkflow.model_validate({"name": "t", "steps": [step]}).steps[0]


def test_schema_defaults_and_bounds():
    step = _validate({"action": "measure_lockin"})
    assert isinstance(step, ActionMeasureLockin) and (step.samples, step.interval) == (5, 0.0)
    for bad in ({"samples": 0}, {"samples": 101}, {"interval": -1}, {"interval": 61},
                {"robot_id": 1}, {"sensitivity": 23}):
        with pytest.raises(ValidationError):
            _validate({"action": "measure_lockin", **bad})


class RecordingShared:
    def __init__(self):
        self.calls = []

    async def measure_lockin(self, **kwargs):
        self.calls.append(kwargs)
        return {"lockin_data1": 1.0, "lockin_data1_kind": "X", "lockin_data1_unit": "V",
                "lockin_data2": 2.0, "lockin_data2_kind": "Y", "lockin_data2_unit": "V"}


@pytest.mark.parametrize("as_model", [False, True])
def test_executor_passes_every_parameter(as_model):
    step = {"action": "measure_lockin", "samples": 7, "interval": 0.5}
    shared = RecordingShared()
    asyncio.run(execute_step(_validate(step) if as_model else step, {}, shared))
    assert shared.calls == [{"samples": 7, "interval": 0.5}]
    assert "measure_lockin" in SHARED_DEVICE_ACTIONS


def test_plan_resources_needs_lockin_and_no_robot():
    plan = run_flow.plan_resources([{"action": "measure_lockin"}])
    assert plan.robot_ids == [] and plan.needs_lockin and plan.needs_shared
    assert not plan.needs_dmm and plan.shared_kwargs()["use_lockin"] is True
    assert "ロックイン" in plan.shared_labels()


def test_lockin_resource_priority(monkeypatch):
    parser = run_flow.build_parser()
    monkeypatch.setattr(run_flow.lab_config, "get_shared_devices",
                        lambda: {"lockin_resource": "GPIB0::1::INSTR"})
    monkeypatch.delenv("LOCKIN_RESOURCE", raising=False)
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["lockin_resource"] == "GPIB0::1::INSTR"
    monkeypatch.setenv("LOCKIN_RESOURCE", "GPIB0::3::INSTR")
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert shared["lockin_resource"] == "GPIB0::3::INSTR"
    _, shared = run_flow.resolve_ports(parser.parse_args(["x.json", "--lockin-resource",
                                                          "GPIB0::5::INSTR"]))
    assert shared["lockin_resource"] == "GPIB0::5::INSTR"


def test_session_factories_forward_lockin_settings():
    from src.flow.experiment_session import default_shared_factory, mock_shared_factory
    config = {"scale_port": "", "camera_index": 0, "lockin_resource": RESOURCE,
              "lockin_timeout_ms": 1234, "wait_after_lockin": 0.5}
    shared = default_shared_factory(use_scale=False, use_camera=False, config=config,
                                    use_lockin=True)
    assert shared.use_lockin and shared.device_configs["lockin_resource"] == RESOURCE
    assert shared._lockin_timeout_ms() == 1234 and shared.wait_after_lockin == 0.5
    assert mock_shared_factory(use_scale=False, use_camera=False, config=config,
                               use_lockin=True).use_lockin


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
    assert meta["lockin_measurements"] == 2 and meta["step_diagnostics"] == []
    assert [s["step_index"] for s in meta["lockin_settings"]] == [1, 2]
    assert meta["lockin_settings"][0]["settings"] == {"mock": True}
    rows = _rows(run_dir)
    assert [r["lockin_data1"] for r in rows] == ["0.001"] * 2
    assert {r["lockin_data2_kind"] for r in rows} == {"Y"} and rows[0]["lockin_freq"] == "1000.0"
    assert json.loads(rows[0]["lockin_settings"]) == {"mock": True}
    assert "ロックイン測定値" in open(os.path.join(run_dir, "summary.md"), encoding="utf-8").read()


def test_gui_runner_mock_run_records_lockin(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE, encoding="utf-8"))
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path), source_path=EXAMPLE)
    runner.start()
    assert runner.join(60)
    assert runner.status == "completed", runner.error
    assert all(r["lockin_data1"] == "0.001" for r in _rows(runner.log_dir))


# ----------------------------------------------------------------------
# 8. Failed-step diagnostics reach the run folder from every runner (S2)
# ----------------------------------------------------------------------
DIAG = {"settings_before": {"offs1": 0.0}, "settings_after": {"offs1": 12.5},
        "settings_diff": {"offs1": [0.0, 12.5]}, "invalid": {"unlock": 1}}


async def _failing_measure(self, samples=5, interval=0.0):
    raise LockinStepError("LI5640 の設定が測定中に変わりました: ['offs1']", DIAG)


def _assert_diagnostics_recorded(run_dir):
    failed = [r for r in _rows(run_dir) if r["status"] == "error"]
    assert failed and json.loads(failed[0]["diagnostics"]) == DIAG
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["step_diagnostics"][0]["diagnostics"] == DIAG
    summary = open(os.path.join(run_dir, "summary.md"), encoding="utf-8").read()
    assert "設定の変化: offs1" in summary


def test_run_flow_records_diagnostics_of_a_failed_step(tmp_path, monkeypatch):
    monkeypatch.setattr(MockSharedDevices, "measure_lockin", _failing_measure)
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_flow.main([EXAMPLE, "--mock"]) != 0
    _assert_diagnostics_recorded(_latest_run_dir(str(tmp_path / "logs")))


def test_gui_runner_records_diagnostics_of_a_failed_step(tmp_path, monkeypatch):
    from src.gui import runner as gui_runner
    monkeypatch.setattr(MockSharedDevices, "measure_lockin", _failing_measure)
    flow = json.load(open(EXAMPLE, encoding="utf-8"))
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path), source_path=EXAMPLE)
    runner.start()
    assert runner.join(60)
    assert runner.status != "completed"
    _assert_diagnostics_recorded(runner.log_dir)


def test_csv_runner_records_diagnostics_of_a_failed_step(tmp_path, monkeypatch):
    from src.flow.csv_runner import run_csv

    async def failing_execute(step, robots, shared, logger=None):
        raise LockinStepError("LI5640 の設定が測定中に変わりました: ['offs1']", DIAG)

    monkeypatch.setattr(run_csv, "execute_step", failing_execute)
    monkeypatch.setattr(run_csv, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_csv.main(["--mock"]) != 0
    _assert_diagnostics_recorded(_latest_run_dir(str(tmp_path / "logs")))


def test_logger_full_precision_and_event(tmp_path):
    src = tmp_path / "f.json"
    src.write_text("{}", encoding="utf-8")
    exp = ExperimentLogger("t", "", str(src), base_dir=str(tmp_path / "logs"), mode="mock")
    exp.record_step(1, 1, "measure_lockin", None, None, "ok", 0.1, result={
        "lockin_data1": 1.2345678901e-07, "lockin_data1_kind": "R", "lockin_data1_unit": "V",
        "lockin_data2": -179.5, "lockin_data2_kind": "THETA", "lockin_data2_unit": "deg",
        "lockin_freq": 1000.25, "lockin_valid_samples": 5, "lockin_invalid": {}})
    exp.finalize("completed")
    row = _rows(exp.dir)[0]
    assert row["lockin_data1"] == "1.2345678901e-07" and row["lockin_data2"] == "-179.5"
    assert row["lockin_invalid"] == "" and row["diagnostics"] == ""
    assert any(e["kind"] == "lockin" for e in exp.events)
