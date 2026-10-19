"""Tube-balancing runner (src/flow/balance_runner): planner, sheet, preflight, mock runs.

No hardware: the runs use MockMaxArmRobot through ExperimentSession plus the
runner's SimulatedBench (mock pipette and balance that share one bench).
"""
import asyncio
import csv
import os

import pytest

from src.flow.balance_runner import planner
from src.flow.balance_runner import run_balance as rb

WORKSPACE = {"x_min": -250.0, "x_max": 250.0, "y_min": -280.0, "y_max": 0.0,
             "z_min": 40.0, "z_max": 225.0, "r_min": 60.0, "r_max": 260.0}
ROBOT_PORTS = {4: {"type": "maxarm", "maxarm_port": "", "workspace": dict(WORKSPACE),
                   "move_ms": 1500, "home_position": [0.0, -163.0, 200.0]}}

SHEET = {
    "travel_z": "185", "reservoir_x": "-100", "reservoir_y": "-170",
    "reservoir_aspirate_z": "120", "reservoir_min_z": "110",
    "tube_a_x": "60", "tube_a_y": "-185", "tube_b_x": "95", "tube_b_y": "-185",
    "dispense_z": "160", "blow_out_after_dispense": "0", "settle_s": "0",
}


def params(**over):
    raw = dict(SHEET)
    raw.update({k: str(v) for k, v in over.items()})
    return rb.parse_params(raw)


async def _no_sleep(_s):
    await asyncio.sleep(0)


def run_mock(p, tmp_path, **bench_kw):
    bench = rb.SimulatedBench(p, **bench_kw)
    sheet = tmp_path / "balance.csv"
    sheet.write_text("parameter,value,note\n", encoding="utf-8")
    results = asyncio.run(rb.run_balancing(
        p, str(sheet), mock=True, robot_ports=ROBOT_PORTS, shared_config={},
        bench=bench, sleep=_no_sleep, logs_dir=str(tmp_path / "logs")))
    return results, bench


# ----------------------------------------------------------------- planner
class TestPlanner:
    def test_split_volume(self):
        assert planner.split_volume(20, 10, 0.5) == [10.0, 10.0]
        assert planner.split_volume(10.3, 10, 0.5) == [5.15, 5.15]
        assert planner.split_volume(0.5, 10, 0.5) == [0.5]
        parts = planner.split_volume(21.07, 10, 0.5)
        assert len(parts) == 3 and abs(sum(parts) - 21.07) < 1e-9
        assert all(0.5 <= v <= 10 for v in parts)
        with pytest.raises(ValueError):
            planner.split_volume(0.4, 10, 0.5)

    def test_bulk_stops_short_by_the_reserve(self):
        assert planner.bulk_volume_for_b(20.0, 1.0, 1.0, 0.5) == pytest.approx(19.0)
        assert planner.bulk_volume_for_b(1.2, 1.0, 1.0, 0.5) == 0.0   # 0.2 mL < minimum

    def kw(self, allow=False):
        return dict(g_per_mL=1.0, tolerance_g=0.1, min_ml=0.5, max_ml=10.0, allow_extra_in_a=allow)

    def test_done_within_tolerance(self):
        assert planner.next_action(30.0, 29.92, **self.kw()).kind == "done"
        assert planner.next_action(30.0, 30.1, **self.kw()).kind == "done"

    def test_lighter_b_gets_the_gap(self):
        d = planner.next_action(30.0, 28.8, **self.kw())
        assert (d.kind, d.tube, d.volume_ml) == ("dispense", "B", 1.2)

    def test_gap_capped_at_max(self):
        d = planner.next_action(40.0, 15.0, **self.kw())
        assert (d.tube, d.volume_ml) == ("B", 10.0)

    def test_small_gap_fails_without_permission(self):
        d = planner.next_action(30.0, 29.7, **self.kw())
        assert d.kind == "fail" and "less than the pipette minimum" in d.reason

    def test_small_gap_widened_through_a_when_allowed(self):
        d = planner.next_action(30.0, 29.7, **self.kw(allow=True))
        assert (d.kind, d.tube, d.volume_ml) == ("dispense", "A", 0.5)
        # after A got 0.5 g, B is 0.8 g short: a normal dispense into B
        d = planner.next_action(30.5, 29.7, **self.kw(allow=True))
        assert (d.tube, d.volume_ml) == ("B", 0.8)

    def test_heavier_b_never_touches_a_without_permission(self):
        d = planner.next_action(30.0, 31.0, **self.kw())
        assert d.kind == "fail" and "heavier" in d.reason
        d = planner.next_action(30.0, 31.0, **self.kw(allow=True))
        assert (d.tube, d.volume_ml) == ("A", 1.0)

    def test_density_conversion(self):
        d = planner.next_action(30.0, 28.8, g_per_mL=0.8, tolerance_g=0.1, min_ml=0.5,
                                max_ml=10, allow_extra_in_a=False)
        assert d.volume_ml == pytest.approx(1.5)

    def test_check_density(self):
        assert planner.check_density(0.98, 1.0, 15) is None
        assert "off the expected" in planner.check_density(0.70, 1.0, 15)


# ----------------------------------------------------------------- sheet
class TestSheet:
    def test_defaults(self):
        p = params()
        assert p["volume_a_mL"] == 20.0 and p["tolerance_g"] == 0.1 and p["robot_id"] == 4
        assert p["allow_extra_in_a"] is False and p["blow_out_after_dispense"] is False

    def test_min_z_defaults_to_aspirate_z(self):
        raw = dict(SHEET)
        del raw["reservoir_min_z"]
        assert rb.parse_params(raw)["reservoir_min_z"] == 120.0

    @pytest.mark.parametrize("over, fragment", [
        ({"tube_a_x": ""}, "tube_a_x: required"),
        ({"tolerance_g": "0"}, "below the minimum"),
        ({"max_aspirate_mL": "12"}, "above the maximum"),
        ({"dispense_z": "190"}, "must be below travel_z"),
        ({"tube_b_x": "62", "tube_b_y": "-186"}, "apart"),
        ({"reservoir_min_z": "130"}, "reservoir_min_z"),
        ({"volume_a_mL": "48"}, "larger than max_tube_volume_mL"),
        ({"allow_extra_in_a": "maybe"}, "yes/no"),
        ({"max_top_ups": "2.5"}, "whole number"),
        ({"bogus": "1"}, "unknown parameter"),
    ])
    def test_rejected(self, over, fragment):
        with pytest.raises(ValueError, match=fragment):
            params(**over)

    def test_example_sheet_parses(self):
        p = rb.load_params_from_csv(rb.EXAMPLE_CSV_PATH)
        assert p["volume_a_mL"] == 20.0

    def test_duplicate_parameter(self, tmp_path):
        f = tmp_path / "s.csv"
        f.write_text("parameter,value,note\ntravel_z,185,\ntravel_z,180,\n", encoding="utf-8")
        with pytest.raises(ValueError, match="twice"):
            rb.load_params_from_csv(str(f))


# ----------------------------------------------------------------- preflight
class TestPreflight:
    def test_example_targets_pass(self):
        validator, problems = rb.preflight(params(), ROBOT_PORTS)
        assert validator is not None and problems == []

    def test_unreachable_target_is_reported(self):
        # 220 mm is inside the box but above the highest reachable z there
        _, problems = rb.preflight(params(travel_z=220), ROBOT_PORTS)
        assert problems and any("travel height" in p for p in problems)

    def test_outside_box_is_reported(self):
        _, problems = rb.preflight(params(reservoir_min_z=30), ROBOT_PORTS)
        assert any("aspiration (lowest)" in p for p in problems)

    def test_non_maxarm_robot_is_refused(self):
        _, problems = rb.preflight(params(robot_id=1), {1: {"dobot_port": "COM3"}})
        assert problems and "not a MaxArm" in problems[0]

    def test_run_refuses_before_opening_anything(self, tmp_path):
        with pytest.raises(rb.run_flow.PreflightError):
            run_mock(params(travel_z=220), tmp_path)
        assert not (tmp_path / "logs").exists()

    def test_real_run_needs_a_pipette_port(self, tmp_path):
        with pytest.raises(ValueError, match="pipette port"):
            asyncio.run(rb.run_balancing(params(), "x.csv", mock=False, robot_ports=ROBOT_PORTS,
                                         shared_config={}, logs_dir=str(tmp_path)))

    def test_pipette_port_resolution(self):
        p = params()
        ports = {4: {**ROBOT_PORTS[4], "picus2_address": "COM9"}}
        assert rb.resolve_pipette_port(p, ports, None) == "COM9"
        assert rb.resolve_pipette_port(params(pipette_port="COM4"), ports, None) == "COM4"
        assert rb.resolve_pipette_port(p, ports, "COM5") == "COM5"


# ----------------------------------------------------------------- mock runs
class TestMockRun:
    def test_balances_within_tolerance(self, tmp_path):
        results, bench = run_mock(params(), tmp_path)
        assert results["status"] == "completed" and results["balanced"]
        # the bench's true masses, not the runner's bookkeeping
        assert abs(bench.gross("A") - bench.gross("B")) <= 0.1 + 0.01
        assert results["liquid_into_A_mL"] == 20.0
        assert results["top_ups"] >= 1
        log_dir = results["log_dir"]
        with open(os.path.join(log_dir, "balance_log.csv"), encoding="utf-8-sig") as f:
            rows = list(csv.DictReader(f))
        assert [r["tube"] for r in rows[:2]] == ["A", "A"]
        assert all(float(r["commanded_mL"]) >= 0.5 for r in rows)
        assert os.path.exists(os.path.join(log_dir, "metadata.json"))

    def test_low_pipette_output_is_learnt(self, tmp_path):
        # the pipette delivers 8 % less than commanded; the top-up must still converge
        results, bench = run_mock(params(), tmp_path, bias=0.92)
        assert results["balanced"]
        assert abs(bench.gross("A") - bench.gross("B")) <= 0.11
        assert results["effective_g_per_mL"] == pytest.approx(0.92, abs=0.01)

    def test_density_check_stops_before_b(self, tmp_path):
        results, bench = run_mock(params(), tmp_path, bias=0.70)
        assert results["status"] == "failed" and not results["balanced"]
        assert "off the expected" in results["error"]
        assert bench.gross("B") == pytest.approx(13.22 + 1.83)   # B untouched

    def test_tube_capacity_checked_before_any_liquid(self, tmp_path):
        results, bench = run_mock(params(max_tube_volume_mL=21), tmp_path, sample_g=8.0)
        assert not results["balanced"] and "nothing has been dispensed" in results["error"]
        assert bench.gross("A") == pytest.approx(13.31 + 8.0)

    def test_heavier_b_fails_without_touching_a_again(self, tmp_path):
        # B's sample is much heavier than A's: refused before any liquid moves
        results, bench = run_mock(params(volume_a_mL=1), tmp_path, sample_b_g=8.0)
        assert not results["balanced"] and "already heavier" in results["error"]
        assert bench.gross("A") == pytest.approx(13.31 + 2.47)

    @pytest.mark.parametrize("sample_a, sample_b", [(2.47, 1.83), (1.0, 4.5), (3.0, 3.0)])
    def test_both_tubes_hold_samples(self, tmp_path, sample_a, sample_b):
        results, bench = run_mock(params(), tmp_path, sample_g=sample_a, sample_b_g=sample_b)
        assert results["balanced"]
        assert abs(bench.gross("A") - bench.gross("B")) <= 0.11
        assert results["tube_b_start_g"] == pytest.approx(13.22 + sample_b)
        assert results["liquid_into_A_mL"] == 20.0

    def test_reservoir_running_low_stops(self, tmp_path):
        results, _ = run_mock(params(reservoir_drop_mm_per_mL=0.6), tmp_path)
        assert not results["balanced"] and "running low" in results["error"]

    def test_extra_in_a_closes_small_gaps(self, tmp_path):
        results, bench = run_mock(params(tolerance_g=0.02, allow_extra_in_a=1, max_top_ups=20),
                                  tmp_path, noise_g=0.03, seed=7)
        assert results["balanced"]
        assert abs(bench.gross("A") - bench.gross("B")) <= 0.03

    def test_results_csv(self, tmp_path):
        results, _ = run_mock(params(), tmp_path)
        sheet = tmp_path / "balance.csv"
        out = rb.write_results_csv(str(sheet), params(), results)
        with open(out, encoding="utf-8-sig") as f:
            table = dict(csv.reader(f))
        assert table["balanced"] == "True" and table["volume_a_mL"] == "20.0"
