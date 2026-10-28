"""
run_balance.py - balance two 50 mL centrifuge tubes with a robot arm and a pipette

The arm is ``robot_id`` in config.yaml: a Dobot Magician with its Picus 2
(``picus2_address`` of that robot), or a MaxArm (``type: maxarm``) carrying a
Picus 2 in an adapter. Both tubes stand open in a rack on the balance for the
whole run; the operator places them, the arm never lifts a tube.

    python -m src.flow.balance_runner.run_balance --validate-only   # check the sheet, open nothing
    python -m src.flow.balance_runner.run_balance --mock            # simulated arm, pipette and balance
    python -m src.flow.balance_runner.run_balance                   # real run (operator at the terminal)

Sequence:

1. The arm goes to its ``home_position``.
2. Operator: empty rack on the balance -> tare. Operator: tube B (with its
   sample) -> its gross mass. Operator: tube A (with its sample) -> its gross mass.
3. Tube A receives ``volume_a_mL`` from the reservoir, in portions of at most
   ``max_aspirate_mL``. The mass that arrived per commanded mL is the run's own
   effective density (liquid density times pipette bias); it must agree with
   ``liquid_density_g_per_mL`` within ``density_check_pct``.
4. Tube B receives the volume that brings it to ``top_up_reserve_mL`` short of A,
   then weighed top-ups close the gap until |A - B| <= ``tolerance_g``
   (``planner.next_action``). Every top-up is at least the pipette minimum
   (0.5 mL), which is why the bulk stops short of the target.
5. The arm goes home and the result is written to ``results.csv`` beside the
   sheet and to the run folder ``logs/<date>/tube_balancing_<time>/``
   (``balance_log.csv`` lists every dispense with the masses it left behind).

The balance is read only with the tip lifted to ``travel_z`` and after
``settle_s``. Every absolute target (reservoir, both tubes, at working and
travel height, and the lowest aspiration height) is checked against the robot's
workspace (a MaxArm's also against its inverse kinematics) before any device is opened.

With ``single_position=1`` the balance holds one tube at a time, in one place
(``tube_a_x``, ``tube_a_y``; ``tube_b_*`` are not used): empty holder (tare),
tube A with its cap, the arm fills A; then (arm parked at home) the operator
swaps A for B with its cap, B is weighed and the arm fills B until it matches A.
B is weighed only after A is filled, so the checks on B (not heavier than A,
not needing more than ``max_tube_volume_mL``) come after A has its liquid.
Liquid is only ever dispensed into the tube that is on the balance, so
``allow_extra_in_a`` is not available in this mode.

Exit codes: 0 balanced, 1 not balanced / run failed, 2 bad sheet, 3 workspace
violation, 130 interrupted (Ctrl+C = emergency stop, the arm halts where it is).
"""
import argparse
import asyncio
import csv
import logging
import math
import os
import random
import sys
import time
from datetime import datetime
from typing import Awaitable, Callable, Dict, List, Optional, Tuple

_REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from src import config as lab_config
from src.devices.safety.lab_robot import LabRobot
from src.devices.safety.mock_robot import MockLabRobot, MockSharedDevices
from src.devices.safety.validators import ValidationError
from src.flow import run_flow
from src.flow.balance_runner import planner
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession, robot_type, validator_for_robot

logger = logging.getLogger("run_balance")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_CSV_PATH = os.path.join(_THIS_DIR, "balance.csv")
EXAMPLE_CSV_PATH = os.path.join(_THIS_DIR, "balance.example.csv")
LOGS_DIR = os.path.join(_REPO_ROOT, "logs")

WORKFLOW_NAME = "tube_balancing"
WORKFLOW_DESCRIPTION = (
    "Centrifuge-tube balancing: tube A gets a fixed volume, tube B the same liquid "
    "until both tubes weigh the same (robot arm + pipette, both tubes in a rack on the balance)"
)

#: Robot types the runner can drive.
SUPPORTED_TYPES = ("dobot", "maxarm")

#: Smallest / largest volume the pipette handles in one stroke (same as LabRobot).
MIN_ML = LabRobot.DEFAULT_MIN_PIPETTE_VOLUME
MAX_ML = LabRobot.DEFAULT_MAX_PIPETTE_VOLUME
#: Blow-out after each dispense (same duration as the CSV runner).
BLOW_OUT_DELAY_MS = 3000
BLOW_OUT_SPEED = 1
#: Pause with the tip in the liquid after aspirating, and after dispensing, s.
AFTER_ASPIRATE_S = 1.0
AFTER_DISPENSE_S = 0.5
#: A dispense that delivers less than this % of the expected mass stops the run
#: (sheet ``min_delivery_pct``; this is its default).
MIN_DELIVERY_PCT = 50.0
#: The balance must read within this of zero right after taring, g.
TARE_CHECK_G = 0.05
#: A 50 mL tube alone weighs ~13 g; less than this means "no tube there", g.
MIN_TUBE_G = 1.0
#: Tubes closer than this in x/y are the same position (a typo in the sheet), mm.
MIN_TUBE_SEPARATION_MM = 10.0


class BalanceError(RuntimeError):
    """The run cannot reach (or safely continue towards) a balanced pair."""


# ======================================================================
# Sheet
# ======================================================================
REQUIRED = object()

#: name -> (type, min, max, default). min/max None = unchecked here (coordinates
#: are checked by the robot's workspace validator instead).
PARAM_SPECS: Dict[str, tuple] = {
    "robot_id":                 (int,   1,    9,     4),
    "volume_a_mL":              (float, MIN_ML, 50.0, 20.0),
    "tolerance_g":              (float, 0.01, 5.0,   0.1),
    "liquid_density_g_per_mL":  (float, 0.5,  3.0,   1.0),
    "density_check_pct":        (float, 1.0,  50.0,  15.0),
    "min_delivery_pct":         (float, 10.0, 100.0, MIN_DELIVERY_PCT),
    "travel_z":                 (float, None, None,  REQUIRED),
    "reservoir_x":              (float, None, None,  REQUIRED),
    "reservoir_y":              (float, None, None,  REQUIRED),
    "reservoir_aspirate_z":     (float, None, None,  REQUIRED),
    "reservoir_drop_mm_per_mL": (float, 0.0,  10.0,  0.0),
    "reservoir_min_z":          (float, None, None,  None),     # default: reservoir_aspirate_z
    "tube_a_x":                 (float, None, None,  REQUIRED),
    "tube_a_y":                 (float, None, None,  REQUIRED),
    "single_position":          (bool,  None, None,  False),
    "tube_b_x":                 (float, None, None,  None),     # required unless single_position
    "tube_b_y":                 (float, None, None,  None),
    "dispense_z":               (float, None, None,  REQUIRED),
    "aspirate_speed":           (int,   1,    9,     3),
    "dispense_speed":           (int,   1,    9,     3),
    "max_aspirate_mL":          (float, MIN_ML, MAX_ML, MAX_ML),
    "air_gap_mL":               (float, 0.0,  2.0,   0.0),      # 0 = no air gap
    "blow_out_after_dispense":  (bool,  None, None,  True),
    "top_up_reserve_mL":        (float, MIN_ML, 5.0,  1.0),
    "max_top_ups":              (int,   1,    20,    6),
    "max_tube_volume_mL":       (float, 1.0,  50.0,  45.0),
    "allow_extra_in_a":         (bool,  None, None,  False),
    "settle_s":                 (float, 0.0,  60.0,  3.0),
    "stabilization_count":      (int,   1,    10,    5),
    "pipette_port":             (str,   None, None,  ""),
}

_TRUE_WORDS = {"true", "yes", "y", "1", "on"}
_FALSE_WORDS = {"false", "no", "n", "0", "off"}


def _convert(name, typ, raw):
    text = str(raw).strip()
    if typ is str:
        return text
    if typ is bool:
        low = text.lower()
        if low in _TRUE_WORDS:
            return True
        if low in _FALSE_WORDS:
            return False
        raise ValueError(f"{name}: '{text}' is not a yes/no value (use 1 or 0)")
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{name}: '{text}' is not a number") from None
    if not math.isfinite(value):
        raise ValueError(f"{name}: '{text}' is not a finite number")
    if typ is int:
        if value != int(value):
            raise ValueError(f"{name}: '{text}' must be a whole number")
        return int(value)
    return value


def parse_params(raw: Dict[str, str]) -> dict:
    """Convert and check the sheet values (``parameter -> text``).

    Raises:
        ValueError: unknown, missing, malformed or out-of-range values, or
            values that contradict each other. The message lists every problem.
    """
    errors: List[str] = []
    unknown = sorted(set(raw) - set(PARAM_SPECS))
    if unknown:
        errors.append("unknown parameter(s): " + ", ".join(unknown))
    params = {}
    for name, (typ, lo, hi, default) in PARAM_SPECS.items():
        text = raw.get(name)
        if text is None or str(text).strip() == "":
            if default is REQUIRED:
                errors.append(f"{name}: required")
            else:
                params[name] = default
            continue
        try:
            value = _convert(name, typ, text)
        except ValueError as e:
            errors.append(str(e))
            continue
        if lo is not None and value < lo:
            errors.append(f"{name}: {value} is below the minimum {lo}")
        elif hi is not None and value > hi:
            errors.append(f"{name}: {value} is above the maximum {hi}")
        params[name] = value

    if not errors:
        if params["single_position"]:
            # one place on the balance: B stands where A stood
            params["tube_b_x"], params["tube_b_y"] = params["tube_a_x"], params["tube_a_y"]
            if params["allow_extra_in_a"]:
                errors.append("allow_extra_in_a needs both tubes on the balance; "
                              "it is not available with single_position=1")
        else:
            errors += [f"{k}: required (unless single_position=1)"
                       for k in ("tube_b_x", "tube_b_y") if params[k] is None]
    if not errors:
        if params["reservoir_min_z"] is None:
            params["reservoir_min_z"] = params["reservoir_aspirate_z"]
        tz = params["travel_z"]
        for key in ("reservoir_aspirate_z", "dispense_z"):
            if params[key] >= tz:
                errors.append(f"{key} ({params[key]}) must be below travel_z ({tz})")
        if params["reservoir_min_z"] > params["reservoir_aspirate_z"]:
            errors.append("reservoir_min_z must not be above reservoir_aspirate_z")
        sep = math.hypot(params["tube_a_x"] - params["tube_b_x"],
                         params["tube_a_y"] - params["tube_b_y"])
        if not params["single_position"] and sep < MIN_TUBE_SEPARATION_MM:
            errors.append(f"tube A and tube B are only {sep:.1f} mm apart "
                          f"(at least {MIN_TUBE_SEPARATION_MM} mm expected)")
        if params["volume_a_mL"] > params["max_tube_volume_mL"]:
            errors.append("volume_a_mL is larger than max_tube_volume_mL")
        air = params["air_gap_mL"]
        if 0 < air < MIN_ML:
            errors.append(f"air_gap_mL must be 0 (off) or at least the pipette minimum of {MIN_ML} mL")
        if params["max_aspirate_mL"] + air > MAX_ML + 1e-9:
            errors.append(f"max_aspirate_mL + air_gap_mL must not exceed the pipette's {MAX_ML} mL "
                          f"(lower max_aspirate_mL to {MAX_ML - air:g})")
    if errors:
        raise ValueError("balance sheet: " + "; ".join(errors))
    return params


def default_csv_path() -> str:
    return CONTROL_CSV_PATH if os.path.exists(CONTROL_CSV_PATH) else EXAMPLE_CSV_PATH


def load_params_from_csv(path: str) -> dict:
    """Read a ``parameter,value,note`` sheet (UTF-8, BOM allowed)."""
    raw: Dict[str, str] = {}
    with open(path, encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header or [h.strip().lower() for h in header[:2]] != ["parameter", "value"]:
            raise ValueError(f"{path}: the first row must be 'parameter,value,note'")
        for row in reader:
            if not row or not row[0].strip() or row[0].strip().startswith("#"):
                continue
            name = row[0].strip()
            if name in raw:
                raise ValueError(f"{path}: parameter '{name}' appears twice")
            raw[name] = row[1] if len(row) > 1 else ""
    return parse_params(raw)


# ======================================================================
# Workspace preflight
# ======================================================================
def motion_targets(params: dict) -> List[Tuple[str, float, float, float]]:
    """Every absolute (x, y, z) the run can command, with a label."""
    p = params
    tz = p["travel_z"]
    targets = [
        ("reservoir, travel height", p["reservoir_x"], p["reservoir_y"], tz),
        ("reservoir, aspiration (full)", p["reservoir_x"], p["reservoir_y"], p["reservoir_aspirate_z"]),
        ("reservoir, aspiration (lowest)", p["reservoir_x"], p["reservoir_y"], p["reservoir_min_z"]),
    ]
    if p.get("single_position"):
        return targets + [
            ("tube (A, then B), travel height", p["tube_a_x"], p["tube_a_y"], tz),
            ("tube (A, then B), dispense", p["tube_a_x"], p["tube_a_y"], p["dispense_z"]),
        ]
    return targets + [
        ("tube A, travel height", p["tube_a_x"], p["tube_a_y"], tz),
        ("tube A, dispense", p["tube_a_x"], p["tube_a_y"], p["dispense_z"]),
        ("tube B, travel height", p["tube_b_x"], p["tube_b_y"], tz),
        ("tube B, dispense", p["tube_b_x"], p["tube_b_y"], p["dispense_z"]),
    ]


def robot_ports_for(params: dict, robot_ports: dict) -> dict:
    rid = params["robot_id"]
    return robot_ports.get(rid) or robot_ports.get(str(rid)) or {}


def preflight(params: dict, robot_ports: dict) -> Tuple[object, List[str]]:
    """Check the robot type and every target; open no device.

    A Dobot's targets are checked against its workspace box (the same check its
    ``move_xyz`` does); a MaxArm's also against the arm's inverse kinematics.

    Returns:
        (validator, problems). ``problems`` is empty when the run may start.
    """
    rid = params["robot_id"]
    ports = robot_ports_for(params, robot_ports)
    rtype = robot_type(ports)
    if rtype not in SUPPORTED_TYPES:
        return None, [f"robot {rid} is of type '{rtype}'; the runner drives "
                      + " or ".join(SUPPORTED_TYPES)]
    try:
        validator = validator_for_robot(rid, robot_ports=robot_ports)
    except lab_config.ConfigError as e:
        return None, [str(e)]
    problems = []
    for label, x, y, z in motion_targets(params):
        try:
            validator.validate_xyz(x, y, z)
        except ValidationError as e:
            problems.append(f"{label} ({x:.1f}, {y:.1f}, {z:.1f}): {e}")
    return validator, problems


# ======================================================================
# Simulation for --mock and the tests
# ======================================================================
class SimulatedBench:
    """A balance with a rack, two tubes and a reservoir, driven by the mock arm.

    Liquid that the mock pipette dispenses lands in the tube whose (x, y) is
    within ``hit_radius_mm`` of the arm; anywhere else is an error, so a wrong
    coordinate in the sheet fails the mock run instead of passing silently.
    The pipette delivers ``density * bias`` g per mL plus Gaussian noise; the
    balance rounds to ``readability``.
    """

    def __init__(self, params: dict, *, rack_g=48.30, tube_a_g=13.31, tube_b_g=13.22,
                 sample_g=2.47, sample_b_g=1.83, density=None, bias=0.985, noise_g=0.01,
                 readability=0.01, hit_radius_mm=8.0, seed=1):
        self.params = params
        self.rack_g, self.tube_a_g, self.tube_b_g = rack_g, tube_a_g, tube_b_g
        self.sample_g, self.sample_b_g = sample_g, sample_b_g
        self.density = params["liquid_density_g_per_mL"] if density is None else density
        self.bias, self.noise_g, self.readability = bias, noise_g, readability
        self.hit_radius_mm = hit_radius_mm
        self.rng = random.Random(seed)
        self.on_pan: Dict[str, float] = {}
        self.off_pan: Dict[str, float] = {}    # tubes set aside by the operator
        self.tare_g = 0.0
        self.arm = None

    # operator
    def place(self, key: str):
        if key == "rack":
            self.on_pan["rack"] = self.rack_g
        elif key == "tube_b":
            self.on_pan["B"] = self.tube_b_g + self.sample_b_g
        elif key in ("tube_a", "tube_a_single"):
            self.on_pan["A"] = self.tube_a_g + self.sample_g
        elif key == "swap_to_b":
            self.off_pan["A"] = self.on_pan.pop("A")
            self.on_pan["B"] = self.tube_b_g + self.sample_b_g

    # balance
    def reading(self) -> float:
        total = sum(self.on_pan.values()) - self.tare_g
        return round(total / self.readability) * self.readability

    def tare(self):
        self.tare_g = sum(self.on_pan.values())

    # pipette
    def _near(self, x, y) -> bool:
        ax, ay, _ = self.arm.get_current_position()[:3]
        return math.hypot(ax - x, ay - y) <= self.hit_radius_mm

    def check_aspirate(self):
        p = self.params
        if not self._near(p["reservoir_x"], p["reservoir_y"]):
            raise RuntimeError("[MOCK] aspirating with the tip not in the reservoir")

    def deliver(self, volume_ml: float):
        p = self.params
        # with single_position both tubes share one place: the one on the pan gets it
        near = [tube for tube, x, y in (("A", p["tube_a_x"], p["tube_a_y"]),
                                        ("B", p["tube_b_x"], p["tube_b_y"])) if self._near(x, y)]
        if not near:
            raise RuntimeError("[MOCK] dispensing with the tip above no tube")
        present = [tube for tube in near if tube in self.on_pan]
        if not present:
            raise RuntimeError(f"[MOCK] dispensing into tube {near[0]}, which is not there")
        tube = present[0]
        self.on_pan[tube] += volume_ml * self.density * self.bias + self.rng.gauss(0, self.noise_g)

    def gross(self, tube: str) -> float:
        return self.on_pan[tube] if tube in self.on_pan else self.off_pan[tube]


class BenchPipette(MockLabRobot):
    """MockLabRobot's pipette (same volume rules) that moves liquid on the bench."""

    def __init__(self, bench: SimulatedBench):
        super().__init__(use_picus2=True)
        self.bench = bench
        self.air_ml = 0.0    # air taken in at travel height (air gap); weighs nothing

    async def aspirate(self, volume, speed=5):
        z = self.bench.arm.get_current_position()[2]
        if abs(z - self.bench.params["travel_z"]) <= 0.5:
            result = await super().aspirate(volume, speed)
            self.air_ml += volume
            return result
        self.bench.check_aspirate()
        return await super().aspirate(volume, speed)

    async def dispense(self, volume, speed=5):
        result = await super().dispense(volume, speed)
        air = min(self.air_ml, volume)
        self.air_ml -= air
        self.bench.deliver(volume - air)
        return result

    async def blow_out(self, *args, **kwargs):
        self.air_ml = 0.0
        return await super().blow_out(*args, **kwargs)


class BenchScale(MockSharedDevices):
    """MockSharedDevices whose balance reads the bench."""

    def __init__(self, bench: SimulatedBench):
        super().__init__(use_scale=True)
        self.bench = bench

    async def measure_weight(self, stabilization_count: int = 3) -> float:
        await asyncio.sleep(0)
        w = self.bench.reading()
        logger.info(f"[MOCK] balance: {w:.2f} g")
        return w

    async def tare_scale(self, delay: float = 1.0):
        await asyncio.sleep(0)
        self.bench.tare()
        logger.info("[MOCK] balance tared")


# ======================================================================
# Operator prompts
# ======================================================================
PROMPTS = {
    "rack": "Put the EMPTY rack on the balance (no tubes, nothing else touching the pan), "
            "then press Enter to tare.",
    "tube_b": "Put tube B (with its sample, cap off) into the B position of the rack. If "
              "the caps go into the centrifuge, lay B's cap beside it on the rack. Press Enter.",
    "tube_a": "Put tube A (with its sample, cap off) into the A position of the rack, "
              "with its cap the same way. Press Enter.",
    # single_position=1: one tube on the balance at a time
    "tube_a_single": "Put tube A (with its sample, cap off) in the holder on the balance. If "
                     "the caps go into the centrifuge, lay A's cap beside it on the holder. "
                     "Press Enter.",
    "swap_to_b": "Tube A is done. Take tube A and its cap off the balance and set them aside. "
                 "Put tube B (with its sample, cap off) in the same holder, with its cap the "
                 "same way. Press Enter.",
}

Operator = Callable[[str, str], Awaitable[None]]


async def terminal_operator(key: str, message: str) -> None:
    """Ask the operator at the terminal. Blocking on purpose: Ctrl+C reaches the
    run directly and triggers the emergency stop."""
    input(f"\n>>> {message}\n")


# ======================================================================
# The run
# ======================================================================
class BalanceRun:
    """One balancing run on initialised devices (arm, pipette, balance)."""

    def __init__(self, params: dict, arm, pipette, scale, operator: Operator,
                 exp_logger: Optional[ExperimentLogger] = None, sleep=asyncio.sleep):
        self.p = params
        self.arm, self.pipette, self.scale = arm, pipette, scale
        self.operator = operator
        self.exp_logger = exp_logger
        self.sleep = sleep
        self.rid = params["robot_id"]
        self.step = 0
        self.reading = 0.0
        self.mass = {"A": 0.0, "B": 0.0}       # gross masses (tube + contents)
        self.start_b = 0.0
        self.start_a = 0.0
        self.liquid_ml = {"A": 0.0, "B": 0.0}  # commanded liquid per tube
        self.consumed_ml = 0.0
        self.delivered_g = 0.0
        self.commanded_ml = 0.0
        self.g_per_mL = params["liquid_density_g_per_mL"]
        self.dispenses: List[dict] = []
        self.top_ups = 0
        self.single = bool(params.get("single_position"))
        self.zero = 0.0               # balance reading right after taring the empty holder
        self.on_balance = None        # single_position: the tube now on the balance

    # -------------------------------------------------------------- records
    def _record(self, action, t0, result=None, status="ok", error=None, robot=True):
        self.step += 1
        if self.exp_logger is not None:
            self.exp_logger.record_step(self.step, "", action, self.rid if robot else None, None,
                                        status, time.monotonic() - t0, result=result, error=error)

    async def _do(self, action, coro, robot=True):
        t0 = time.monotonic()
        try:
            result = await coro
        except Exception as e:
            self._record(action, t0, status="error", error=str(e), robot=robot)
            raise
        self._record(action, t0, result=result if isinstance(result, dict) else None, robot=robot)
        return result

    # -------------------------------------------------------------- motion
    async def goto(self, x, y, z):
        """Rise to travel_z where we are, cross at travel_z, then descend to z."""
        tz = self.p["travel_z"]
        cx, cy, cz = self.arm.get_current_position()[:3]
        if cz < tz - 0.5:
            await self._do("move_xyz", self.arm.move_xyz(cx, cy, tz))
            cz = tz
        if math.hypot(cx - x, cy - y) > 0.5 or abs(cz - tz) > 0.5:
            await self._do("move_xyz", self.arm.move_xyz(x, y, tz))
        if abs(z - tz) > 0.5:
            await self._do("move_xyz", self.arm.move_xyz(x, y, z))

    def _tube_xy(self, tube):
        if tube == "A":
            return self.p["tube_a_x"], self.p["tube_a_y"]
        return self.p["tube_b_x"], self.p["tube_b_y"]

    # -------------------------------------------------------------- balance
    async def measure(self, what: str) -> float:
        await self.sleep(self.p["settle_s"])
        t0 = time.monotonic()
        w = await self.scale.measure_weight(self.p["stabilization_count"])
        self._record("measure_weight", t0, result={"weight": w}, robot=False)
        logger.info(f"balance {w:.2f} g ({what})")
        return w

    # -------------------------------------------------------------- liquid
    async def transfer(self, tube: str, volume_ml: float):
        """Reservoir -> tube, in portions; weigh after each portion."""
        p = self.p
        if self.single and tube != self.on_balance:
            raise BalanceError(f"tube {tube} is not on the balance (tube {self.on_balance} is)")
        tx, ty = self._tube_xy(tube)
        for v in planner.split_volume(volume_ml, p["max_aspirate_mL"], MIN_ML):
            if self.liquid_ml[tube] + v > p["max_tube_volume_mL"] + 1e-9:
                raise BalanceError(
                    f"tube {tube} would hold {self.liquid_ml[tube] + v:.2f} mL of added liquid, "
                    f"more than max_tube_volume_mL={p['max_tube_volume_mL']}")
            z_asp = p["reservoir_aspirate_z"] - p["reservoir_drop_mm_per_mL"] * (self.consumed_ml + v)
            if z_asp < p["reservoir_min_z"] - 1e-6:
                raise BalanceError(
                    f"the reservoir is running low: aspirating {v:.2f} mL would need the tip at "
                    f"z={z_asp:.1f}, below reservoir_min_z={p['reservoir_min_z']}")
            logger.info(f"--- {v:.2f} mL -> tube {tube}")
            await self.goto(p["reservoir_x"], p["reservoir_y"], z_asp)
            await self._do("aspirate", self.pipette.aspirate(v, p["aspirate_speed"]))
            await self.sleep(AFTER_ASPIRATE_S)
            self.consumed_ml += v
            air = p["air_gap_mL"]
            if air > 0:
                # tip out of the liquid first, then air below the liquid against drips
                await self._do("move_xyz", self.arm.move_xyz(
                    p["reservoir_x"], p["reservoir_y"], p["travel_z"]))
                await self._do("aspirate", self.pipette.aspirate(air, p["aspirate_speed"]))
            await self.goto(tx, ty, p["dispense_z"])
            await self._do("dispense", self.pipette.dispense(v + air, p["dispense_speed"]))
            if p["blow_out_after_dispense"]:
                await self._do("blow_out", self.pipette.blow_out(
                    go_home=True, speed=BLOW_OUT_SPEED, delay_ms=BLOW_OUT_DELAY_MS))
            await self.sleep(AFTER_DISPENSE_S)
            await self._do("move_xyz", self.arm.move_xyz(tx, ty, p["travel_z"]))

            w = await self.measure(f"after {v:.2f} mL into {tube}")
            delta = w - self.reading
            self.reading = w
            self.mass[tube] += delta
            self.liquid_ml[tube] += v
            self.delivered_g += delta
            self.commanded_ml += v
            self.dispenses.append({
                "n": len(self.dispenses) + 1, "tube": tube, "commanded_mL": round(v, 2),
                "arrived_g": round(delta, 3), "balance_g": round(w, 3),
                "mass_A_g": round(self.mass["A"], 3), "mass_B_g": round(self.mass["B"], 3),
                "diff_A_minus_B_g": round(self.mass["A"] - self.mass["B"], 3),
            })
            logger.info(f"    arrived {delta:.3f} g; A {self.mass['A']:.2f} g, "
                        f"B {self.mass['B']:.2f} g, A-B {self.mass['A'] - self.mass['B']:+.3f} g")
            # checked before the next dispense: a short one would also skew the
            # effective density that converts the top-ups, and overfill B
            expected = v * p["liquid_density_g_per_mL"]
            if delta < p["min_delivery_pct"] / 100.0 * expected:
                raise BalanceError(
                    f"only {delta:.2f} g arrived for {v:.2f} mL (expected ~{expected:.2f} g, "
                    f"{delta / expected * 100:.0f} % < min_delivery_pct={p['min_delivery_pct']:g} %): "
                    "check the tip, the reservoir level and the tube position. "
                    f"Now A {self.mass['A']:.2f} g, B {self.mass['B']:.2f} g")

    def _update_density(self):
        self.g_per_mL = planner.effective_density(self.delivered_g, self.commanded_ml)
        logger.info(f"effective density so far: {self.g_per_mL:.4f} g per commanded mL")
        reason = planner.check_density(self.g_per_mL, self.p["liquid_density_g_per_mL"],
                                       self.p["density_check_pct"])
        if reason:
            raise BalanceError(reason)

    # -------------------------------------------------------------- sequence
    async def setup_tubes(self):
        p = self.p
        await self.operator("rack", PROMPTS["rack"])
        t0 = time.monotonic()
        await self.scale.tare_scale(1.0)
        self._record("tare_scale", t0, robot=False)
        zero = await self.measure("after tare")
        if abs(zero) > TARE_CHECK_G:
            raise BalanceError(f"the balance reads {zero:.2f} g right after taring; check the pan")
        self.reading = self.zero = zero

        if self.single:
            # B is weighed after A is filled (swap_to_b)
            await self.operator("tube_a_single", PROMPTS["tube_a_single"])
            w = await self.measure("holder + tube A")
            self.start_a = self.mass["A"] = w - self.zero
            self.reading = w
            self.on_balance = "A"
            if self.start_a < MIN_TUBE_G:
                raise BalanceError(f"tube A weighs {self.start_a:.2f} g: is it in the holder?")
            logger.info(f"tube A {self.start_a:.2f} g (with its sample)")
            return

        await self.operator("tube_b", PROMPTS["tube_b"])
        w = await self.measure("rack + tube B")
        self.start_b = self.mass["B"] = w - self.reading
        self.reading = w
        if self.start_b < MIN_TUBE_G:
            raise BalanceError(f"tube B weighs {self.start_b:.2f} g: is it in the rack?")

        await self.operator("tube_a", PROMPTS["tube_a"])
        w = await self.measure("rack + tube B + tube A")
        self.start_a = self.mass["A"] = w - self.reading
        self.reading = w
        if self.start_a < MIN_TUBE_G:
            raise BalanceError(f"tube A weighs {self.start_a:.2f} g: is it in the rack?")
        logger.info(f"tube A {self.start_a:.2f} g, tube B {self.start_b:.2f} g (each with its sample)")

        rho = p["liquid_density_g_per_mL"]
        need_b_ml = (self.start_a + p["volume_a_mL"] * rho - self.start_b) / rho
        if need_b_ml < 0:
            raise BalanceError(
                f"tube B ({self.start_b:.2f} g with its sample) is already heavier than tube A "
                f"will be after {p['volume_a_mL']} mL ({self.start_a + p['volume_a_mL'] * rho:.2f} g); "
                "nothing has been dispensed")
        if need_b_ml > p["max_tube_volume_mL"]:
            raise BalanceError(
                f"tube B would need ~{need_b_ml:.1f} mL, more than max_tube_volume_mL="
                f"{p['max_tube_volume_mL']}; nothing has been dispensed")
        logger.info(f"tube B will need about {need_b_ml:.1f} mL")

    async def swap_to_b(self):
        """single_position: A is done; park the arm, have A swapped for B, weigh B."""
        p = self.p
        await self._do("go_home", self.arm.go_home())
        await self.operator("swap_to_b", PROMPTS["swap_to_b"])
        w = await self.measure("holder + tube B")
        self.start_b = self.mass["B"] = w - self.zero
        self.reading = w
        self.on_balance = "B"
        logger.info(f"tube A {self.mass['A']:.2f} g (filled), tube B {self.start_b:.2f} g "
                    "(with its sample)")
        nothing_yet = "tube A is filled; nothing has been put into B"
        if self.start_b < MIN_TUBE_G:
            raise BalanceError(f"tube B weighs {self.start_b:.2f} g: is it in the holder? "
                               f"({nothing_yet})")
        gap_g = self.mass["A"] - self.start_b
        if gap_g < MIN_ML * self.g_per_mL:
            # also what a tube A left on the balance looks like
            raise BalanceError(
                f"the balance reads {self.start_b:.2f} g for tube B, only {gap_g:+.2f} g from "
                f"tube A ({self.mass['A']:.2f} g): is tube A still on the balance? Otherwise B "
                f"is too heavy for the pipette minimum of {MIN_ML} mL to close the gap "
                f"({nothing_yet})")
        need_b_ml = gap_g / self.g_per_mL
        if need_b_ml > p["max_tube_volume_mL"]:
            raise BalanceError(
                f"tube B would need ~{need_b_ml:.1f} mL, more than max_tube_volume_mL="
                f"{p['max_tube_volume_mL']} ({nothing_yet})")
        logger.info(f"tube B will need about {need_b_ml:.1f} mL")

    async def balance(self) -> planner.Decision:
        p = self.p
        await self.transfer("A", p["volume_a_mL"])
        self._update_density()
        if self.single:
            await self.swap_to_b()

        bulk = planner.bulk_volume_for_b(self.mass["A"] - self.mass["B"], self.g_per_mL,
                                         p["top_up_reserve_mL"], MIN_ML)
        if bulk > 0:
            await self.transfer("B", bulk)
            self._update_density()

        while True:
            d = planner.next_action(self.mass["A"], self.mass["B"], g_per_mL=self.g_per_mL,
                                    tolerance_g=p["tolerance_g"], min_ml=MIN_ML,
                                    max_ml=p["max_aspirate_mL"],
                                    allow_extra_in_a=p["allow_extra_in_a"])
            if d.kind != "dispense":
                return d
            if self.top_ups >= p["max_top_ups"]:
                return planner.Decision("fail", reason=(
                    f"still {self.mass['A'] - self.mass['B']:+.3f} g apart after "
                    f"{self.top_ups} top-ups (max_top_ups)"))
            self.top_ups += 1
            if d.tube == "A":
                logger.warning(f"adding {d.volume_ml:.2f} mL to tube A (allow_extra_in_a=1)")
            await self.transfer(d.tube, d.volume_ml)

    async def run(self) -> dict:
        logger.info("=== tube balancing started ===")
        await self._do("go_home", self.arm.go_home())
        await self.setup_tubes()
        try:
            decision = await self.balance()
        except BalanceError:
            # A checked stop at a weighing point (tip lifted): park the arm so
            # the operator can reach the tubes, then report.
            await self._do("go_home", self.arm.go_home())
            raise
        await self._do("go_home", self.arm.go_home())
        if decision.kind == "fail":
            raise BalanceError(decision.reason)
        diff = self.mass["A"] - self.mass["B"]
        logger.info(f"=== balanced: A {self.mass['A']:.2f} g, B {self.mass['B']:.2f} g, "
                    f"A-B {diff:+.3f} g (tolerance {self.p['tolerance_g']} g) ===")
        return self.summary()

    def summary(self) -> dict:
        return {
            "tube_a_start_g": round(self.start_a, 3),
            "tube_b_start_g": round(self.start_b, 3),
            "tube_a_final_g": round(self.mass["A"], 3),
            "tube_b_final_g": round(self.mass["B"], 3),
            "diff_A_minus_B_g": round(self.mass["A"] - self.mass["B"], 3),
            "liquid_into_A_mL": round(self.liquid_ml["A"], 2),
            "liquid_into_B_mL": round(self.liquid_ml["B"], 2),
            "effective_g_per_mL": round(self.g_per_mL, 4),
            "dispenses": len(self.dispenses),
            "top_ups": self.top_ups,
        }


def write_balance_log(path: str, rows: List[dict]) -> None:
    fields = ["n", "tube", "commanded_mL", "arrived_g", "balance_g",
              "mass_A_g", "mass_B_g", "diff_A_minus_B_g"]
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


RESULT_KEYS = ("status", "balanced", "error", "tube_a_start_g", "tube_b_start_g",
               "tube_a_final_g", "tube_b_final_g", "diff_A_minus_B_g", "liquid_into_A_mL",
               "liquid_into_B_mL", "effective_g_per_mL", "dispenses", "top_ups", "log_dir")


def write_results_csv(input_csv_path: str, params: dict, results: dict) -> str:
    """results.csv beside the sheet (timestamped fallback if the file is locked)."""
    out_dir = os.path.dirname(os.path.abspath(input_csv_path))
    out_path = os.path.join(out_dir, "results.csv")
    rows = [("last_run_at", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))]
    rows += [(k, results.get(k)) for k in RESULT_KEYS]
    rows += [(k, params.get(k)) for k in PARAM_SPECS]

    def _save(path):
        with open(path, "w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["key", "value"])
            writer.writerows(rows)

    try:
        _save(out_path)
        return out_path
    except PermissionError:
        fallback = os.path.join(out_dir, f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        logger.warning(f"cannot write {out_path} (open elsewhere?); saving {fallback}")
        _save(fallback)
        return fallback


def resolve_pipette_port(params: dict, robot_ports: dict, cli_port: Optional[str]) -> str:
    """--pipette-port > sheet pipette_port > robots.<id>.picus2_address in config.yaml."""
    if cli_port:
        return cli_port
    if params.get("pipette_port"):
        return params["pipette_port"]
    return robot_ports_for(params, robot_ports).get("picus2_address") or ""


async def run_balancing(params: dict, csv_path: str, *, mock: bool, robot_ports: dict,
                        shared_config: dict, pipette_port: str = "",
                        operator: Optional[Operator] = None,
                        bench: Optional[SimulatedBench] = None,
                        sleep=asyncio.sleep, logs_dir: str = LOGS_DIR) -> dict:
    """Open the devices, run, close. Returns the summary plus status/log_dir."""
    validator, problems = preflight(params, robot_ports)
    if problems:
        raise run_flow.PreflightError(None, "; ".join(problems))
    rid = params["robot_id"]
    rtype = robot_type(robot_ports_for(params, robot_ports))
    if not mock and not pipette_port:
        raise ValueError(f"no pipette port: give --pipette-port, the sheet's pipette_port, or "
                         f"robots.{rid}.picus2_address in config.yaml")
    if not mock and rtype == "dobot" and not robot_ports_for(params, robot_ports).get("dobot_port"):
        raise ValueError(f"robot {rid} has no dobot_port in config.yaml (robots.{rid})")
    if rtype == "dobot" and pipette_port:
        # the Dobot's own LabRobot opens its pipette from robots.<id>.picus2_address
        robot_ports = {**robot_ports, rid: {**robot_ports_for(params, robot_ports),
                                            "picus2_address": pipette_port}}

    if mock and bench is None:
        bench = SimulatedBench(params)
    if mock and operator is None:
        async def operator(key, message):  # the mock operator places things on the bench
            logger.info(f"[MOCK operator] {message}")
            bench.place(key)

    exp_logger = ExperimentLogger(WORKFLOW_NAME, WORKFLOW_DESCRIPTION, csv_path,
                                  base_dir=logs_dir, mode="mock" if mock else "real")
    exp_logger.record_resources([rid], [rid], robot_types={rid: rtype})
    logger.info(f"experiment log folder: {exp_logger.dir}")

    session_kwargs = {}
    if mock:
        session_kwargs["shared_factory"] = lambda **_kw: BenchScale(bench)
    session = ExperimentSession(mock=mock, robot_ports=robot_ports, shared_config=shared_config,
                                robot_validators={rid: validator}, **session_kwargs)
    holder = {"pipette": None, "run": None}

    async def body():
        if mock:
            arm = await session.add_robot(rid)
            bench.arm = arm
            pipette = BenchPipette(bench)
        elif rtype == "dobot":
            # the Dobot's LabRobot carries the pipette; the session closes both
            arm = pipette = await session.add_robot(rid, use_picus2=True)
        else:
            arm = await session.add_robot(rid)
            pipette = LabRobot(use_dobot=False, use_picus2=True, picus2_address=pipette_port,
                               picus2_connection_type="usb")
        if pipette is not arm:
            holder["pipette"] = pipette
            if not await pipette.initialize():
                raise RuntimeError("the pipette did not initialise")
        scale = await session.add_shared(use_scale=True)
        run = BalanceRun(params, arm, pipette, scale, operator or terminal_operator,
                         exp_logger=exp_logger, sleep=sleep)
        holder["run"] = run
        return await run.run()

    results: dict = {}
    try:
        results = await session.run(body) or {}
    except (KeyboardInterrupt, asyncio.CancelledError):
        raise
    except Exception as e:  # noqa: BLE001 - reported through the return value
        logger.error(f"run failed: {e}")
        results["error"] = str(e)
    finally:
        if holder["pipette"] is not None:
            try:
                await holder["pipette"].cleanup()
            except BaseException as e:  # noqa: BLE001 - keep closing
                logger.error(f"pipette disconnect: {e!r}")
        run = holder["run"]
        if run is not None:
            results = {**run.summary(), **results}
            try:
                write_balance_log(os.path.join(exp_logger.dir, "balance_log.csv"), run.dispenses)
            except Exception as e:  # noqa: BLE001
                logger.warning(f"could not write balance_log.csv: {e}")
        exp_logger.finalize(session.status, session.error)
        logger.info(f"experiment log saved: {exp_logger.dir}")

    results["status"] = session.status
    results["balanced"] = session.status == "completed"
    results["log_dir"] = exp_logger.dir
    return results


# ======================================================================
# CLI
# ======================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Balance two centrifuge tubes with a robot arm and a pipette")
    p.add_argument("--csv", default=None,
                   help="parameter sheet (default: balance.csv here, else balance.example.csv)")
    p.add_argument("--mock", action="store_true", help="simulated arm, pipette and balance")
    p.add_argument("--validate-only", action="store_true",
                   help="check the sheet and the workspace, open no device")
    p.add_argument("--pipette-port", default=None,
                   help="Picus 2 port (default: sheet pipette_port, then robots.<id>.picus2_address)")
    p.add_argument("--maxarm-port", default=None, help="MaxArm port (default: config.yaml)")
    p.add_argument("--dobot-port", default=None, help="Dobot port (default: config.yaml)")
    p.add_argument("--scale-port", default=None, help="balance port (default: SCALE_PORT / config.yaml)")
    p.add_argument("-v", "--verbose", action="store_true")
    p.set_defaults(camera_index=None)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    csv_path = args.csv or default_csv_path()
    if not os.path.exists(csv_path):
        logger.error(f"file not found: {csv_path}")
        return 2
    logger.info(f"parameter sheet: {csv_path}")
    try:
        params = load_params_from_csv(csv_path)
    except ValueError as e:
        logger.error(str(e))
        return 2

    rid = params["robot_id"]
    if args.maxarm_port:
        setattr(args, f"robot{rid}_maxarm", args.maxarm_port)
    if args.dobot_port:
        setattr(args, f"robot{rid}_dobot", args.dobot_port)
    try:
        robot_ports, shared_config = run_flow.resolve_ports(args)
    except lab_config.ConfigError as e:
        logger.error(str(e))
        return 2

    _, problems = preflight(params, robot_ports)
    for label, x, y, z in motion_targets(params):
        logger.info(f"target {label}: ({x:.1f}, {y:.1f}, {z:.1f})")
    if problems:
        for prob in problems:
            logger.error(f"workspace: {prob}")
        return run_flow.PREFLIGHT_EXIT_CODE
    logger.info("workspace check passed for every target")
    if args.validate_only:
        return 0

    pipette_port = resolve_pipette_port(params, robot_ports, args.pipette_port)
    if not args.mock and not pipette_port:
        logger.error(f"no pipette port: give --pipette-port, the sheet's pipette_port, or "
                     f"robots.{rid}.picus2_address in config.yaml")
        return 2

    try:
        results = asyncio.run(run_balancing(params, csv_path, mock=args.mock,
                                            robot_ports=robot_ports, shared_config=shared_config,
                                            pipette_port=pipette_port))
    except run_flow.PreflightError as e:
        logger.error(str(e))
        return run_flow.PREFLIGHT_EXIT_CODE
    except KeyboardInterrupt:
        logger.warning("interrupted (emergency stop and cleanup done)")
        return 130

    saved = write_results_csv(csv_path, params, results)
    logger.info(f"results written to {saved}")
    if results.get("balanced"):
        logger.info(f"BALANCED: A-B = {results.get('diff_A_minus_B_g'):+.3f} g")
        return 0
    logger.error(f"NOT BALANCED: {results.get('error')}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
