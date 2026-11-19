"""NF LI5640 internal oscillator: set_lockin_oscillator and the restore at the end of a run.

No hardware and no PyVISA. The fake instrument (OscLI) extends the read-only fake of
tests/test_li5640.py with FREQ / AMPL writes, and can refuse, ignore or time out on
any of them. The tests check the bus traffic, not only the exceptions: which command
was sent, in which order, and that nothing depending on a failed command followed.
"""
import asyncio
import csv
import json
import os
import subprocess
import sys
import textwrap
import threading
import time
from decimal import Decimal

import pytest

import src.devices.li5640 as li_pkg
from src.devices.li5640 import oscillator as osc
from src.devices.li5640.li5640 import LI5640, LI5640Error, LockinStepError, OscillatorWriteError
from src.devices.safety.mock_robot import MockSharedDevices
from src.devices.safety.shared_devices import SharedDevices
from src.flow import run_flow
from src.flow.experiment_session import ExperimentSession, OutputRestoreError
from tests.test_li5640 import RESOURCE, FakeLI, FakeRM

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(REPO_ROOT, "examples", "li5640", "oscillator", "osc_sweep_scope.json")
LIMITS = {"lockin_osc_max_vrms": 0.3, "lockin_osc_min_hz": 10.0, "lockin_osc_max_hz": 1000.0}
D = Decimal


# ----------------------------------------------------------------------
# Fakes
# ----------------------------------------------------------------------
class OscLI(FakeLI):
    """FakeLI that accepts FREQ / AMPL. ``faults`` maps a command prefix to a behaviour:

    - "raise": the write raises (not applied)
    - "raise_after": applied, then the write itself raises (the bus timed out late)
    - "timeout_after": applied, then the read-back raises
    - "ignore": the write is accepted but not applied (read-back shows the old value)
    - "neighbour": the instrument applies a neighbouring value (one count off)
    - "error": applied, and the error queue gets an entry
    """

    def __init__(self, freq="1.0000E+02", ampl="0.300,1", faults=None, **kw):
        super().__init__(**kw)
        self.panel["freq"], self.panel["ampl"] = freq, ampl
        self.faults = dict(faults or {})
        self.fail_next_readback = None

    def write(self, command):
        head = command.split(" ", 1)[0]
        if head not in ("FREQ", "AMPL"):
            return super().write(command)
        self._check_open(command)
        self.log.append(("w", command))
        fault = self.faults.pop(head, None)
        if fault == "raise":
            raise RuntimeError("VI_ERROR_TMO (write)")
        if fault == "ignore":
            return
        value = command.split(" ", 1)[1]
        if head == "FREQ":
            f = D(value)
            if fault == "neighbour":
                f += D("0.01")
            self.panel["freq"] = f"{float(f):.4E}"
        else:
            self.panel["ampl"] = value
        if fault == "error":
            self.errors.append('-222,"Data out of range"')
        if fault == "timeout_after":
            self.fail_next_readback = head
        if fault == "raise_after":
            raise RuntimeError("VI_ERROR_TMO (write, applied)")

    def query(self, command):
        if self.fail_next_readback and command == f"{self.fail_next_readback}?":
            self.fail_next_readback = None
            self._check_open(command)
            self.log.append(("q", command))
            raise RuntimeError("VI_ERROR_TMO (read)")
        return super().query(command)

    def osc_writes(self):
        return [c for c in self.writes() if c.split(" ", 1)[0] in ("FREQ", "AMPL")]


def make(res=None, **kw):
    res = res or OscLI(**kw)
    return LI5640(RESOURCE, resource_manager=FakeRM(res)), res


def state(f, x, r):
    return osc.OscState(D(str(f)), D(str(x)), r)


def _patch(monkeypatch, res):
    monkeypatch.setattr(li_pkg, "LI5640", lambda name, timeout_ms=5000: LI5640(
        name, timeout_ms=timeout_ms, resource_manager=FakeRM(res), lock_wait_s=0.2))


def _plan(*amplitudes, freqs=(100.0,)):
    steps = [{"action": osc.ACTION, "frequency": f, "amplitude": a}
             for f in freqs for a in amplitudes]
    return osc.plan_from_steps(steps)


def _shared(plan=None, **configs):
    return SharedDevices(use_lockin=True, lockin_resource=RESOURCE, lockin_osc_plan=plan,
                         **{**LIMITS, **configs})


def _run(coro):
    return asyncio.run(coro)


# ----------------------------------------------------------------------
# 1. Pure rules
# ----------------------------------------------------------------------
@pytest.mark.parametrize("value, ok", [
    ("100", True), ("100.01", True), ("1.9999", True), ("2", True), ("2.0001", False),
    ("199.99", True), ("200.05", False), ("19999", True), ("20001", False), ("105000", True),
    ("105001", False), ("0.0005", True), ("0.00005", False), ("12345.6", False),
])
def test_frequency_grid(value, ok):
    assert osc.frequency_representable(D(value)) is ok


def test_frequency_format_round_trips():
    for v in ("100", "1.9999", "105000", "0.0005", "123.45"):
        assert D(osc.format_frequency(D(v))) == D(v)


@pytest.mark.parametrize("x, ranges", [
    ("0.0123", (0,)), ("0.300", (1, 2)), ("0.05", (0, 1, 2)), ("1.5", (2,)), ("0.0005", (0,)),
    ("0", (0, 1, 2)), ("0.501", ()), ("5.01", ()),
])
def test_amplitude_ranges(x, ranges):
    assert osc.fitting_ranges([D(x)]) == ranges


def test_amplitude_format_and_reply():
    assert osc.format_amplitude(D("0.3"), 1) == "0.300,1"
    assert osc.format_amplitude(D("0.0123"), 0) == "0.0123,0"
    assert osc.parse_amplitude_reply("0.300,2") == (D("0.300"), 2)
    for bad in ("0.300", "0.3,3", "x,1", "0.6,1", "-0.1,1", "nan,1"):
        with pytest.raises(ValueError):
            osc.parse_amplitude_reply(bad)


def test_limits_default_disabled_and_bad_values_refused():
    assert not osc.parse_limits({}).enabled
    assert osc.parse_limits(LIMITS).enabled
    for bad in ({"lockin_osc_max_vrms": None}, {"lockin_osc_max_vrms": True},
                {"lockin_osc_max_vrms": "0.3"}, {"lockin_osc_max_vrms": float("nan")},
                {"lockin_osc_max_vrms": float("inf")}, {"lockin_osc_max_vrms": -0.1},
                {"lockin_osc_max_vrms": 6.0}, {"lockin_osc_min_hz": 0.0},
                {"lockin_osc_min_hz": 500.0, "lockin_osc_max_hz": 100.0},
                {"lockin_osc_max_hz": 200000.0}):
        with pytest.raises(ValueError):
            osc.parse_limits({**LIMITS, **bad})


def _errors(steps, config):
    return osc.preflight_errors(steps, config)


def test_preflight_refuses_when_disabled_even_for_zero_volts():
    steps = [{"action": osc.ACTION, "frequency": 100.0, "amplitude": 0.0}]
    assert _errors(steps, {}) and "無効" in _errors(steps, {})[0]
    assert _errors(steps, LIMITS) == []
    assert _errors([{"action": "measure_lockin"}], {}) == []


def test_preflight_names_each_problem():
    steps = [{"action": osc.ACTION, "frequency": 100.0, "amplitude": 0.31},
             {"action": osc.ACTION, "frequency": 5.0, "amplitude": 0.1},
             {"action": osc.ACTION, "frequency": 100.005, "amplitude": 0.1}]
    text = " ".join(_errors(steps, LIMITS))
    assert "ステップ 1" in text and "ステップ 2" in text and "ステップ 3" in text


def test_preflight_refuses_amplitudes_without_a_common_range():
    steps = [{"action": osc.ACTION, "frequency": 100.0, "amplitude": 0.0123},
             {"action": osc.ACTION, "frequency": 100.0, "amplitude": 0.300}]
    assert any("共通のレンジ" in e for e in _errors(steps, LIMITS))


def test_plan_carries_first_frequency_and_common_ranges():
    plan = osc.plan_from_steps([{"action": "measure_lockin"},
                                {"action": osc.ACTION, "frequency": 50.0, "amplitude": 0.01},
                                {"action": osc.ACTION, "frequency": 200.0, "amplitude": 0.03}])
    assert plan.first_frequency == D("50") and plan.ranges == (0, 1, 2)
    assert osc.plan_from_steps([{"action": "measure_lockin"}]) is None


# ----------------------------------------------------------------------
# 2. Driver: one command at a time
# ----------------------------------------------------------------------
def test_lower_amplitude_writes_ampl_first_higher_writes_freq_first():
    lia, res = make()
    lia.apply_oscillator(state(100, "0.3", 1), state(200, "0.1", 1))
    assert res.osc_writes() == ["AMPL 0.100,1", "FREQ 2.0000E+02"]
    lia.close()
    lia2, res2 = make(OscLI(freq="2.0000E+02", ampl="0.100,1"))
    lia2.apply_oscillator(state(200, "0.1", 1), state(100, "0.3", 1))
    assert res2.osc_writes() == ["FREQ 1.0000E+02", "AMPL 0.300,1"]


def test_each_write_is_read_back_and_error_checked_before_the_next():
    lia, res = make()
    lia.apply_oscillator(state(100, "0.3", 1), state(200, "0.1", 1))
    assert res.sent() == ["AMPL 0.100,1", "AMPL?", "EROR?", "FREQ 2.0000E+02", "FREQ?", "EROR?"]


def test_equal_values_send_nothing():
    lia, res = make()
    lia.apply_oscillator(state(100, "0.3", 1), state(100, "0.3", 1))
    lia.apply_oscillator(state(100, "0.3", 1), state(100, "0.2", 1))
    assert res.osc_writes() == ["AMPL 0.200,1"]


@pytest.mark.parametrize("fault", ["raise", "raise_after", "timeout_after", "ignore", "neighbour",
                                   "error"])
def test_a_failed_first_command_stops_the_dependent_second(fault):
    # lowering the amplitude: AMPL first. Its failure must not be followed by FREQ
    first = "AMPL" if fault != "neighbour" else "FREQ"
    current, target = ((state(100, "0.3", 1), state(200, "0.1", 1)) if first == "AMPL"
                       else (state(100, "0.1", 1), state(200, "0.3", 1)))
    lia, res = make(OscLI(ampl=f"{current.ampl},1", faults={first: fault}))
    with pytest.raises(Exception) as info:
        lia.apply_oscillator(current, target)
    assert not isinstance(info.value, AssertionError)
    assert len(res.osc_writes()) == 1 and res.osc_writes()[0].startswith(first)


def test_values_that_would_be_rounded_are_never_sent():
    lia, res = make()
    for target in (state(100, "0.3006", 1), state("100.006", "0.3", 1)):
        with pytest.raises(OscillatorWriteError, match="丸めずに"):
            lia.apply_oscillator(state(200, "0.1", 1), target)
    report = lia.restore_oscillator(state(100, "0.3006", 1))     # 0.301 V would exceed 0.3006
    assert report["state"] == "failed" and res.osc_writes() == []


def test_the_range_is_never_switched():
    lia, res = make()
    with pytest.raises(OscillatorWriteError):
        lia.apply_oscillator(state(100, "0.3", 1), state(100, "0.3", 2))
    assert res.osc_writes() == []


def test_restore_reads_first_and_writes_only_what_differs():
    lia, res = make(OscLI(freq="2.0000E+02", ampl="0.100,1"))
    report = lia.restore_oscillator(state(100, "0.3", 1))
    assert report["state"] == "verified" and report["read_before"]["freq_hz"] == 200.0
    assert res.osc_writes() == ["FREQ 1.0000E+02", "AMPL 0.300,1"]
    res.log.clear()
    assert lia.restore_oscillator(state(100, "0.3", 1))["state"] == "verified"
    assert res.osc_writes() == []                               # already there


def test_restore_keeps_a_non_minimal_saved_range():
    lia, res = make(OscLI(ampl="0.10,2"))
    assert lia.restore_oscillator(state(100, "0.30", 2))["state"] == "verified"
    assert res.osc_writes() == ["AMPL 0.30,2"]


@pytest.mark.parametrize("case, expected", [("unreadable", "unverified"), ("range", "failed")])
def test_restore_writes_nothing_when_it_cannot_start_safely(case, expected):
    res = OscLI(ampl="0.100,2" if case == "range" else "garbage")
    lia, _ = make(res)
    report = lia.restore_oscillator(state(100, "0.3", 1))
    assert report["state"] == expected and res.osc_writes() == []
    if case == "range":
        assert "接続時と違います" in report["errors"][0]       # the restore's own check


def test_restore_separates_pre_existing_errors_and_stops_on_its_own():
    lia, res = make(OscLI(ampl="0.100,1", errors=['-113,"Undefined header"']))
    report = lia.restore_oscillator(state(100, "0.3", 1))
    assert report["state"] == "verified" and report["pre_existing_errors"] == ['-113,"Undefined header"']
    lia.close()
    lia2, res2 = make(OscLI(freq="2.0000E+02", ampl="0.100,1", faults={"FREQ": "error"}))
    report = lia2.restore_oscillator(state(100, "0.3", 1))
    assert report["state"] == "failed" and res2.osc_writes() == ["FREQ 1.0000E+02"]


def test_restore_never_starts_when_the_error_queue_does_not_empty():
    lia, res = make(OscLI(ampl="0.100,1", errors=['-350,"Queue overflow"'] * 30))
    assert lia.restore_oscillator(state(100, "0.3", 1))["state"] == "failed"
    assert res.osc_writes() == []


def test_release_order_and_result_survive_a_close_failure():
    class BadClose(OscLI):
        def close(self):
            self.log.append("close")
            raise RuntimeError("close failed")

    lia, res = make(BadClose(ampl="0.100,1"))
    with pytest.raises(RuntimeError):
        lia.release("1,2", state(100, "0.3", 1))
    order = [e if isinstance(e, str) else e[1] for e in res.log]
    assert order.index("clear") < order.index("AMPL 0.300,1") < order.index("OTYP 1,2") \
        < order.index("gtl") < order.index("close")
    assert lia.last_release["oscillator"]["state"] == "verified"
    assert lia.last_release["closed"] is False


# ----------------------------------------------------------------------
# 3. One open handle per instrument
# ----------------------------------------------------------------------
def test_a_second_open_of_the_same_instrument_waits_then_fails():
    first, res = make()
    with pytest.raises(LI5640Error, match="終了処理"):
        LI5640("GPIB::7::INSTR", resource_manager=FakeRM(res), lock_wait_s=0.1)
    first.close()
    LI5640("GPIB::7::INSTR", resource_manager=FakeRM(res), lock_wait_s=0.1).close()


def test_the_lock_is_released_only_by_a_successful_close():
    class Flaky(OscLI):
        fail = True

        def close(self):
            if Flaky.fail:
                Flaky.fail = False
                raise RuntimeError("close failed")
            super().close()

    res = Flaky()
    lia = LI5640(RESOURCE, resource_manager=FakeRM(res))
    with pytest.raises(RuntimeError):
        lia.close()
    with pytest.raises(LI5640Error):
        LI5640(RESOURCE, resource_manager=FakeRM(res), lock_wait_s=0.05)
    lia.close()
    lia.close()                                     # a second close does not release twice
    LI5640(RESOURCE, resource_manager=FakeRM(OscLI()), lock_wait_s=0.05).close()


def test_open_failures_release_the_lock_unless_a_handle_may_remain():
    class NoOpen(FakeRM):
        def open_resource(self, name):
            raise RuntimeError("VI_ERROR_RSRC_NFOUND")

    with pytest.raises(RuntimeError):
        LI5640(RESOURCE, resource_manager=NoOpen(OscLI()))
    LI5640(RESOURCE, resource_manager=FakeRM(OscLI()), lock_wait_s=0.05).close()

    class BadSetup(OscLI):
        close_fails = False

        def __setattr__(self, name, value):
            if name == "read_termination" and value is None:
                raise RuntimeError("bad attribute")
            super().__setattr__(name, value)

        def close(self):
            if self.close_fails:
                raise RuntimeError("close failed")
            super().close()

    with pytest.raises(RuntimeError):
        LI5640(RESOURCE, resource_manager=FakeRM(BadSetup()))
    LI5640(RESOURCE, resource_manager=FakeRM(OscLI()), lock_wait_s=0.05).close()
    stuck = BadSetup()
    stuck.close_fails = True
    with pytest.raises(LI5640Error, match="再起動"):
        LI5640(RESOURCE, resource_manager=FakeRM(stuck))
    with pytest.raises(LI5640Error, match="閉じられなかった"):   # a handle may remain: keep the lock
        LI5640(RESOURCE, resource_manager=FakeRM(OscLI()), lock_wait_s=0.05)


# ----------------------------------------------------------------------
# 4. SharedDevices
# ----------------------------------------------------------------------
def _life(shared, body):
    async def main():
        assert await shared.initialize()
        try:
            return await body()
        finally:
            await shared.cleanup()
    return _run(main())


def test_a_read_only_flow_writes_no_oscillator_command(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = SharedDevices(use_lockin=True, lockin_resource=RESOURCE)
    result = _life(shared, lambda: shared.measure_lockin(samples=1))
    assert res.osc_writes() == [] and shared.output_restore_report() is None
    assert result["lockin_settings"]["osc_freq"] == 100.0
    assert result["lockin_settings"]["osc_ampl"] == 0.3


def test_set_then_restore_at_cleanup(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1, freqs=(200.0,)))
    result = _life(shared, lambda: shared.set_lockin_oscillator(200.0, 0.1))
    assert (result["lockin_osc_freq_hz"], result["lockin_osc_ampl_vrms"], result["lockin_osc_range"]) \
        == (200.0, 0.1, 1)
    assert result["lockin_osc_before"] == {"freq_hz": 100.0, "ampl_vrms": 0.3, "range": 1}
    assert result["lockin_osc_rsrc"] == 0
    assert res.osc_writes() == ["AMPL 0.100,1", "FREQ 2.0000E+02", "FREQ 1.0000E+02", "AMPL 0.300,1"]
    report = shared.output_restore_report()
    assert report["oscillator"]["state"] == "verified" and report["closed"] is True


@pytest.mark.parametrize("configs, panel, match", [
    ({"lockin_osc_max_vrms": 0.0}, {}, "無効"),
    ({}, {"ampl": "0.400,1"}, "超えています"),          # restore would write 0.4 V back
    ({}, {"freq": "5.0000E+00"}, "外です"),
    ({}, {"ampl": "0.30,2"}, "レンジ"),                  # plan needs range 0 or 1
    ({"lockin_osc_max_vrms": 0.3006}, {"ampl": "0.3006,1"}, "丸めずに"),  # restore would send 0.301
    ({"lockin_osc_max_hz": 100.006}, {"freq": "100.006"}, "丸めずに"),    # restore would send 100.01
])
def test_initialisation_refuses_and_writes_nothing(monkeypatch, configs, panel, match, caplog):
    res = OscLI(**panel)
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.012), **configs) if "ampl" in panel and panel["ampl"].endswith(",2") \
        else _shared(_plan(0.1), **configs)

    async def main():
        ok = await shared.initialize()
        await shared.cleanup()
        return ok

    assert not _run(main())
    assert res.osc_writes() == [] and match in caplog.text


def test_wrong_requests_are_refused_before_any_write(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def body():
        for bad in ((100.0, 0.31), (5.0, 0.1), (100.005, 0.1), (100.0, 0.0123)):
            with pytest.raises(ValueError):
                await shared.set_lockin_oscillator(*bad)
        return None

    _life(shared, body)
    assert res.osc_writes() == []
    assert shared.output_restore_report()["oscillator"]["state"] == "not_needed"
    assert res.sent().count("AMPL?") == 1          # read once on connect; no restore was run


def test_without_a_plan_the_wrapper_refuses(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = SharedDevices(use_lockin=True, lockin_resource=RESOURCE, **LIMITS)

    async def body():
        with pytest.raises(RuntimeError):
            await shared.set_lockin_oscillator(100.0, 0.1)

    _life(shared, body)
    assert res.osc_writes() == []


def test_a_failed_set_makes_the_state_unknown_and_cleanup_restores_from_what_it_reads(monkeypatch):
    res = OscLI(faults={"AMPL": "timeout_after"})
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1, freqs=(200.0,)))

    async def body():
        with pytest.raises(LockinStepError) as info:
            await shared.set_lockin_oscillator(200.0, 0.1)
        assert info.value.diagnostics["dirty"] is True
        assert info.value.diagnostics["writes"] == ["AMPL 0.100,1"]
        with pytest.raises(LockinStepError, match="不明"):
            await shared.set_lockin_oscillator(200.0, 0.1)

    _life(shared, body)
    assert res.osc_writes() == ["AMPL 0.100,1", "AMPL 0.300,1"]   # restore: FREQ unchanged
    assert shared.output_restore_report()["oscillator"]["state"] == "verified"


def test_a_write_that_raises_after_applying_still_marks_the_state_changed(monkeypatch):
    res = OscLI(faults={"AMPL": "raise_after"})
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1, freqs=(200.0,)))

    async def body():
        with pytest.raises(LockinStepError) as info:
            await shared.set_lockin_oscillator(200.0, 0.1)
        assert info.value.diagnostics["dirty"] is True
        assert info.value.diagnostics["writes"] == ["AMPL 0.100,1"]

    _life(shared, body)
    assert res.osc_writes() == ["AMPL 0.100,1", "AMPL 0.300,1"]   # no FREQ; restore wrote AMPL back
    assert shared.output_restore_report()["oscillator"]["state"] == "verified"


def test_a_close_failure_without_writes_is_still_reported(monkeypatch):
    class NoClose(OscLI):
        def close(self):
            raise RuntimeError("close failed")

    res = NoClose()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.3))
    _life(shared, lambda: shared.set_lockin_oscillator(100.0, 0.3))      # equal: nothing written
    assert res.osc_writes() == []
    assert shared.output_restore_report() == {"oscillator": {"state": "not_needed"}, "closed": False}


def test_a_panel_change_between_steps_is_not_overwritten(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1, 0.2))

    async def body():
        await shared.set_lockin_oscillator(100.0, 0.1)
        res.panel["ampl"] = "0.250,1"                          # someone turned the knob
        with pytest.raises(LockinStepError, match="違います"):
            await shared.set_lockin_oscillator(100.0, 0.2)

    _life(shared, body)
    assert res.osc_writes() == ["AMPL 0.100,1", "AMPL 0.300,1"]


def test_errors_already_queued_stop_the_step_before_writing(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def body():
        res.errors.append('-113,"Undefined header"')
        with pytest.raises(LockinStepError, match="エラーキュー"):
            await shared.set_lockin_oscillator(100.0, 0.1)

    _life(shared, body)
    assert res.osc_writes() == []


def test_measure_fails_when_the_oscillator_changes_during_the_step(monkeypatch):
    res = OscLI()
    res.after_dout = {1: {"ampl": "0.200,1"}}
    _patch(monkeypatch, res)
    shared = SharedDevices(use_lockin=True, lockin_resource=RESOURCE)
    with pytest.raises(LockinStepError) as info:
        _life(shared, lambda: shared.measure_lockin(samples=1))
    assert "osc_ampl" in info.value.diagnostics["settings_diff"]


def test_cancel_during_settle_keeps_a_fixed_snapshot_and_cleanup_restores(monkeypatch):
    res = OscLI()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def main():
        assert await shared.initialize()
        task = asyncio.create_task(shared.set_lockin_oscillator(100.0, 0.1, settle=30.0))
        while "AMPL 0.100,1" not in res.osc_writes():
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.2)
        task.cancel()
        with pytest.raises(asyncio.CancelledError) as info:
            await task
        snap = info.value.diagnostics
        frozen = json.dumps(snap, sort_keys=True)
        await shared.cleanup()
        assert json.dumps(snap, sort_keys=True) == frozen       # not shared with the worker
        return snap

    t0 = time.monotonic()
    snap = _run(main())
    assert time.monotonic() - t0 < 5
    assert snap["dirty"] is True and snap["writes"] == ["AMPL 0.100,1"]
    assert 0.15 <= snap["settled_s"] < 5                    # elapsed at the cancel, not 0
    assert res.osc_writes()[-1] == "AMPL 0.300,1"
    assert shared.output_restore_report()["oscillator"]["state"] == "verified"


def test_a_worker_mid_io_at_cleanup_restores_exactly_once(monkeypatch):
    entered, release = threading.Event(), threading.Event()

    class Blocking(OscLI):
        def query(self, command):
            if command == "DOUT?" and not release.is_set():
                entered.set()
                release.wait(10)
            return super().query(command)

    res = Blocking()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def main():
        assert await shared.initialize()
        await shared.set_lockin_oscillator(100.0, 0.1)
        task = asyncio.create_task(shared.measure_lockin(samples=1))
        await asyncio.to_thread(entered.wait, 5)
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.3)
        release.set()
        with pytest.raises(Exception):
            await task
        await cleanup

    _run(main())
    assert res.osc_writes().count("AMPL 0.300,1") == 1
    assert shared.output_restore_report()["oscillator"]["state"] == "verified"


def test_cancelling_the_wait_never_withdraws_the_release_and_final_values_win(monkeypatch):
    gate = threading.Event()

    class SlowRestore(OscLI):
        def query(self, command):
            if command == "EROR?" and any(c == "AMPL 0.100,1" for c in self.osc_writes()) \
                    and not gate.is_set() and self.closing:
                gate.wait(10)
            return super().query(command)

    res = SlowRestore()
    res.closing = False
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def main():
        assert await shared.initialize()
        await shared.set_lockin_oscillator(100.0, 0.1)
        res.closing = True
        cleanup = asyncio.create_task(shared.cleanup())
        await asyncio.sleep(0.3)
        cleanup.cancel()                                # second Ctrl+C while waiting
        with pytest.raises(asyncio.CancelledError):
            await cleanup
        provisional = shared.output_restore_report()
        gate.set()                                      # the release thread goes on
        for _ in range(200):
            if shared.output_restore_report()["closed"] is True:
                break
            await asyncio.sleep(0.02)
        return provisional

    provisional = _run(main())
    assert provisional["oscillator"]["state"] == "unverified" and provisional["closed"] == "pending"
    final = shared.output_restore_report()
    assert final["oscillator"]["state"] == "verified" and final["closed"] is True
    assert res.osc_writes()[-1] == "AMPL 0.300,1" and res.closed


def test_a_close_failure_is_reported_and_the_restore_is_not_repeated(monkeypatch):
    class Flaky(OscLI):
        fails = 1

        def close(self):
            if Flaky.fails:
                Flaky.fails -= 1
                self.log.append("close")
                raise RuntimeError("close failed")
            super().close()

    res = Flaky()
    _patch(monkeypatch, res)
    shared = _shared(_plan(0.1))

    async def main():
        assert await shared.initialize()
        await shared.set_lockin_oscillator(100.0, 0.1)
        await shared.cleanup()
        first = shared.output_restore_report()
        await shared.cleanup()                          # retry closes; no second restore
        return first

    first = _run(main())
    assert first["closed"] is False and first["oscillator"]["state"] == "verified"
    assert shared.output_restore_report()["closed"] is True
    assert res.osc_writes().count("AMPL 0.300,1") == 1
    # the report is the one restore's, not overwritten by the retry
    assert shared.output_restore_report()["oscillator"]["read_before"]["ampl_vrms"] == 0.1


def test_mock_follows_the_same_rules_and_starts_from_a_state_that_fits_the_flow():
    plan = osc.plan_from_steps([{"action": osc.ACTION, "frequency": 50.0, "amplitude": 0.01},
                                {"action": osc.ACTION, "frequency": 200.0, "amplitude": 0.03}])
    mock = MockSharedDevices(use_lockin=True, lockin_osc_plan=plan, **LIMITS)

    async def main():
        assert await mock.initialize()
        r = await mock.set_lockin_oscillator(50.0, 0.01)
        assert r["lockin_osc_before"] == {"freq_hz": 50.0, "ampl_vrms": 0.0, "range": 0}
        with pytest.raises(ValueError):
            await mock.set_lockin_oscillator(50.0, 0.31)
        await mock.cleanup()

    _run(main())
    assert mock.output_restore_report()["oscillator"]["state"] == "verified"
    assert not _run(MockSharedDevices(use_lockin=True, lockin_osc_plan=plan).initialize())


def test_mock_sends_nothing_for_an_equal_request():
    plan = osc.plan_from_steps([{"action": osc.ACTION, "frequency": 50.0, "amplitude": 0.0}])
    mock = MockSharedDevices(use_lockin=True, lockin_osc_plan=plan, **LIMITS)

    async def main():
        assert await mock.initialize()
        await mock.set_lockin_oscillator(50.0, 0.0)          # the mock's starting state
        await mock.cleanup()

    _run(main())
    assert mock.output_restore_report() == {"oscillator": {"state": "not_needed"}, "closed": True}


# ----------------------------------------------------------------------
# 5. Session: the restore result decides the status
# ----------------------------------------------------------------------
class ReportingShared:
    def __init__(self, report, cleanup_raises=None):
        self.report = report
        self.cleanup_raises = cleanup_raises

    async def initialize(self):
        return True

    async def cleanup(self):
        if self.cleanup_raises is not None:
            raise self.cleanup_raises

    def output_restore_report(self):
        return self.report


FAILED = {"oscillator": {"state": "failed", "errors": ["x"]}, "closed": True}
OK = {"oscillator": {"state": "verified"}, "closed": True}


def _session(shared):
    session = ExperimentSession(mock=True, shared_factory=lambda **kw: shared, shared_config={})
    session.shared = shared
    return session


def test_completed_body_with_a_failed_restore_raises_and_fails():
    session = _session(ReportingShared(FAILED))

    async def body():
        return 1

    with pytest.raises(OutputRestoreError, match="OSC OUT"):
        _run(session.run(body))
    assert session.status == "failed"


def test_completed_body_with_a_good_restore_returns():
    session = _session(ReportingShared(OK))

    async def body():
        return 7

    assert _run(session.run(body)) == 7 and session.status == "completed"


def test_an_exception_in_flight_is_kept_and_gets_the_note():
    session = _session(ReportingShared(FAILED))

    async def body():
        raise ValueError("step failed")

    with pytest.raises(ValueError):
        _run(session.run(body))
    assert session.status == "failed" and "step failed" in session.error and "OSC OUT" in session.error


def test_cleanup_interrupted_after_a_completed_body_is_aborted_and_reported():
    session = _session(ReportingShared(FAILED, cleanup_raises=asyncio.CancelledError()))

    async def body():
        return 1

    with pytest.raises(asyncio.CancelledError):
        _run(session.run(body))
    assert session.status == "aborted" and "OSC OUT" in session.error


@pytest.mark.parametrize("closed, fails", [(False, True), (None, False), (True, False)])
def test_a_failed_close_counts_even_when_nothing_was_written(closed, fails):
    session = _session(ReportingShared({"oscillator": {"state": "not_needed"}, "closed": closed}))

    async def body():
        return 1

    if fails:
        with pytest.raises(OutputRestoreError):
            _run(session.run(body))
        assert session.status == "failed"
    else:
        assert _run(session.run(body)) == 1 and session.status == "completed"


def test_closed_other_than_true_counts_as_a_failure():
    session = _session(ReportingShared({"oscillator": {"state": "verified"}, "closed": "pending"}))

    async def body():
        return 1

    with pytest.raises(OutputRestoreError):
        _run(session.run(body))


# ----------------------------------------------------------------------
# 6. Runners: preflight, metadata, exit codes, aborted steps
# ----------------------------------------------------------------------
def _flow(tmp_path, steps):
    path = tmp_path / "flow.json"
    path.write_text(json.dumps({"name": "osc", "steps": steps}), encoding="utf-8")
    return str(path)


def _cli(monkeypatch, tmp_path, flow, limits=LIMITS, extra=()):
    from src import config as lab_config
    real = lab_config.get_shared_devices
    monkeypatch.setattr(lab_config, "get_shared_devices", lambda: {**real(), **limits})
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    return run_flow.main([flow, *extra])


def _latest_meta(tmp_path):
    runs = []
    for root, dirs, files in os.walk(tmp_path / "logs"):
        if "metadata.json" in files:
            runs.append(root)
    run = max(runs, key=os.path.getmtime)
    with open(os.path.join(run, "metadata.json"), encoding="utf-8") as f:
        meta = json.load(f)
    with open(os.path.join(run, "measurements.csv"), encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    return meta, rows


STEPS = [{"action": osc.ACTION, "frequency": 200.0, "amplitude": 0.1, "settle": 0.0},
         {"action": "measure_lockin", "samples": 1}]


def test_cli_refuses_by_default_and_runs_the_mock_when_enabled(monkeypatch, tmp_path):
    flow = _flow(tmp_path, STEPS)
    assert _cli(monkeypatch, tmp_path, flow, limits={"lockin_osc_max_vrms": 0.0}, extra=["--validate-only"]) == 3
    assert _cli(monkeypatch, tmp_path, flow, extra=["--validate-only"]) == 0
    assert _cli(monkeypatch, tmp_path, flow, extra=["--mock"]) == 0
    meta, rows = _latest_meta(tmp_path)
    assert meta["status"] == "completed"
    assert meta["lockin_oscillator_restore"]["oscillator"]["state"] == "verified"
    assert meta["lockin_oscillator"][0]["after"] == {"freq_hz": 200.0, "ampl_vrms": 0.1, "range": 1}
    assert rows[0]["lockin_osc_freq_hz"] == "200.0" and rows[0]["lockin_osc_range"] == "1"


def test_cli_exit_code_is_1_when_the_restore_failed(monkeypatch, tmp_path):
    monkeypatch.setattr(MockSharedDevices, "output_restore_report", lambda self: FAILED)
    assert _cli(monkeypatch, tmp_path, _flow(tmp_path, STEPS), extra=["--mock"]) == 1
    meta, _ = _latest_meta(tmp_path)
    assert meta["status"] == "failed" and meta["lockin_oscillator_restore"] == FAILED


def test_cli_records_an_interrupted_step_as_aborted_with_its_snapshot(monkeypatch, tmp_path):
    async def interrupted(self, frequency, amplitude, settle=0.0):
        e = asyncio.CancelledError()
        e.diagnostics = {"dirty": True, "writes": ["AMPL 0.100,1"]}
        raise e

    monkeypatch.setattr(MockSharedDevices, "set_lockin_oscillator", interrupted)
    assert _cli(monkeypatch, tmp_path, _flow(tmp_path, STEPS), extra=["--mock"]) == 130
    meta, rows = _latest_meta(tmp_path)
    assert meta["aborted_steps"] == 1 and rows[0]["status"] == "aborted"
    assert json.loads(rows[0]["diagnostics"])["writes"] == ["AMPL 0.100,1"]
    assert meta["step_diagnostics"][0]["diagnostics"]["dirty"] is True


def _gui_config():
    from src import config as lab_config
    return {**lab_config.get_shared_devices(), **LIMITS}


def test_gui_runner_uses_the_same_preflight_and_reports_the_restore(monkeypatch, tmp_path):
    from src.gui.runner import FlowRunner
    data = {"name": "osc", "steps": STEPS}
    with pytest.raises(run_flow.PreflightError):
        FlowRunner(data, mock=True, shared_config={}, logs_dir=str(tmp_path / "logs"))
    monkeypatch.setattr(MockSharedDevices, "output_restore_report", lambda self: FAILED)
    runner = FlowRunner(data, mock=True, shared_config=_gui_config(),
                        logs_dir=str(tmp_path / "logs"))
    runner.start()
    assert runner.join(60)
    assert runner.status == "failed" and "OSC OUT" in runner.error
    meta, _ = _latest_meta(tmp_path)
    assert meta["status"] == "failed" and meta["lockin_oscillator_restore"] == FAILED


def test_gui_runner_records_an_interrupted_step_as_aborted(monkeypatch, tmp_path):
    from src.gui.runner import FlowRunner

    async def interrupted(self, frequency, amplitude, settle=0.0):
        e = asyncio.CancelledError()
        e.diagnostics = {"dirty": False}
        raise e

    monkeypatch.setattr(MockSharedDevices, "set_lockin_oscillator", interrupted)
    runner = FlowRunner({"name": "osc", "steps": STEPS}, mock=True, shared_config=_gui_config(),
                        logs_dir=str(tmp_path / "logs"))
    runner.start()
    assert runner.join(60)
    meta, rows = _latest_meta(tmp_path)
    assert meta["aborted_steps"] == 1 and rows[0]["status"] == "aborted"


def test_csv_runner_records_a_failed_restore(monkeypatch, tmp_path):
    from src.flow.csv_runner import run_csv
    monkeypatch.setattr(MockSharedDevices, "output_restore_report", lambda self: FAILED)
    monkeypatch.setattr(run_csv, "LOGS_DIR", str(tmp_path / "logs"))
    assert run_csv.main(["--mock"]) != 0
    meta, _ = _latest_meta(tmp_path)
    assert meta["status"] == "failed" and meta["lockin_oscillator_restore"] == FAILED


def test_example_flow_is_refused_by_default_and_mock_runs_when_enabled(monkeypatch, tmp_path):
    assert _cli(monkeypatch, tmp_path, EXAMPLE, limits={"lockin_osc_max_vrms": 0.0}, extra=["--validate-only"]) == 3
    assert _cli(monkeypatch, tmp_path, EXAMPLE, extra=["--mock"]) == 0
    meta, rows = _latest_meta(tmp_path)
    assert meta["robots_used"] == [] and len(meta["lockin_oscillator"]) == 6
    assert meta["lockin_oscillator_restore"]["oscillator"]["state"] == "verified"


# ----------------------------------------------------------------------
# 7. A real SIGINT in a separate process (fake instrument, real SharedDevices)
# ----------------------------------------------------------------------
CHILD = textwrap.dedent('''
    import json, os, signal, sys, threading, time
    sys.path.insert(0, sys.argv[1])
    import src.devices.li5640 as li_pkg
    from src import config as lab_config
    from src.devices.li5640.li5640 import LI5640
    from src.flow import run_flow
    from tests.test_li5640 import RESOURCE, FakeRM
    from tests.test_li5640_oscillator import LIMITS, OscLI

    res = OscLI()
    li_pkg.LI5640 = lambda name, timeout_ms=5000: LI5640(name, timeout_ms=timeout_ms,
                                                         resource_manager=FakeRM(res))
    real = lab_config.get_shared_devices
    lab_config.get_shared_devices = lambda: {**real(), **LIMITS}
    run_flow.LOGS_DIR = sys.argv[3]

    def interrupt():
        while "AMPL 0.100,1" not in res.osc_writes():
            time.sleep(0.01)
        time.sleep(0.3)                           # inside the settle wait
        signal.raise_signal(signal.SIGINT)

    threading.Thread(target=interrupt, daemon=True).start()
    code = run_flow.main([sys.argv[2], "--no-record", "--lockin-resource", RESOURCE])
    with open(os.path.join(sys.argv[3], "fake_log.json"), "w") as f:
        json.dump({"code": code, "writes": res.osc_writes(), "closed": res.closed}, f)
''')


def test_sigint_during_settle_restores_and_exits_130(tmp_path):
    flow = _flow(tmp_path, [{"action": osc.ACTION, "frequency": 100.0, "amplitude": 0.1,
                             "settle": 30.0}])
    logs = tmp_path / "logs"
    logs.mkdir()
    script = tmp_path / "child.py"
    script.write_text(CHILD, encoding="utf-8")
    proc = subprocess.run([sys.executable, str(script), REPO_ROOT, flow, str(logs)],
                          capture_output=True, text=True, timeout=60, cwd=REPO_ROOT)
    out = json.loads((logs / "fake_log.json").read_text())
    assert out["code"] == 130, proc.stderr[-2000:]
    assert out["writes"] == ["AMPL 0.100,1", "AMPL 0.300,1"] and out["closed"]
    meta, rows = _latest_meta(tmp_path)
    assert meta["status"] == "aborted" and rows[0]["status"] == "aborted"
    assert meta["lockin_oscillator_restore"]["oscillator"]["state"] == "verified"
