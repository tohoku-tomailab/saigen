"""
run_csv.py - run the fixed two-solution mixing experiment from a CSV file

This is the "spreadsheet input" path: the operator opens `control.csv` in Excel /
LibreOffice / Numbers, types the volumes, Z descents, angles and pipette speeds,
saves, and runs

    python -m src.flow.csv_runner.run_csv --mock       # dry run, no hardware
    python -m src.flow.csv_runner.run_csv              # real run

This is the path that produced the ZIF-8 batches reported in the paper. The step
sequence is fixed - it reproduces the lab runner (`excel_runner/run_csv.py`, commit
4584333) move for move, and `examples/zif8/zif8_two_solution_mixing_speed*.json` are
its JSON equivalents; only the numbers come from the CSV. No JSON is written by hand.
The generated steps are validated with the same Pydantic schema
(`ExperimentWorkflow`) and executed through the same `execute_step` / `ExperimentSession`
/ `ExperimentLogger` path as `run_flow.py`, so a CSV run is checked by the workspace
validator and leaves behind exactly the same `logs/<date>/<name>_<timestamp>/` folder.

Input CSV format (`parameter,value,note`, see `control.example.csv`):

    parameter,value,note
    robot1_volume_mL,5.0,Robot 1 aspirate/dispense volume (0 skips Robot 1; otherwise 0.5-10 mL)
    ...

Results are also written back next to the input CSV as `results.csv`, so the operator
can read the dispensed masses in the same spreadsheet program.
"""
import argparse
import asyncio
import csv
import json
import logging
import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

_REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from dotenv import load_dotenv

from src.devices.safety.validators import default_workspace_validator
from src.flow import run_flow
from src.flow.accuracy_logger import DispenseAccuracyLogger
from src.flow.executor import execute_step, expand_loops, SHARED_DEVICE_ACTIONS
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import PIPETTE_MIN_VOLUME_ML, ExperimentWorkflow

load_dotenv()
logger = logging.getLogger("run_csv")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_CSV_PATH = os.path.join(_THIS_DIR, "control.csv")
EXAMPLE_CSV_PATH = os.path.join(_THIS_DIR, "control.example.csv")
LOGS_DIR = os.path.join(_REPO_ROOT, "logs")

WORKFLOW_NAME = "two-solution mixing (CSV input)"
WORKFLOW_DESCRIPTION = (
    "Fixed two-solution mixing sequence with the parameters entered in control.csv: "
    "each robot in turn aspirates its solution, swings over the vial on the balance and "
    "lowers into it; the balance is tared at that lowered position, the solution is "
    "dispensed and the mass read before the tip rises and the arm returns home. This is "
    "the sequence used for the ZIF-8 experiments reported in the paper."
)

# Number of balance readings averaged per measurement (fixed, not a CSV parameter).
# Same value as DEFAULT_STABILIZATION_COUNT in the lab version of this runner.
DEFAULT_STABILIZATION_COUNT = 3
# Wait after taring before dispensing, seconds (fixed; `tare_scale(delay=1.0)`).
TARE_DELAY_S = 1.0
# Aspiration speed used when the CSV has no `aspirate_speed` row.
# The lab version of this runner had no such row at all: aspiration was the
# constant `ASPIRATE_SPEED = 1` in the code, and that constant was in force for
# the ZIF-8 batches reported in the paper. (The note column of the lab's sheet
# claimed "aspiration is always 5"; that note was stale and never affected the
# runs.) The default is therefore 1, so a sheet that omits the row reproduces
# the historical behaviour; a sheet that carries the row overrides it.
ASPIRATE_SPEED_DEFAULT = 1
# Duration of the optional blow-out after dispensing, milliseconds (fixed; same
# value as BLOW_OUT_DELAY_MS in the lab version of this runner).
BLOW_OUT_DELAY_MS = 3000


# ======================================================================
# CSV parameters
# ======================================================================
_TRUE_WORDS = {"true", "yes", "y", "1", "on"}
_FALSE_WORDS = {"false", "no", "n", "0", "off"}


def parse_bool(value) -> bool:
    """Read a spreadsheet cell as a boolean (TRUE/yes/1/on vs FALSE/no/0/off)."""
    text = str(value).strip().lower()
    if text in _TRUE_WORDS:
        return True
    if text in _FALSE_WORDS:
        return False
    raise ValueError(f"not a boolean: {value!r}")


parse_bool.__name__ = "boolean"

# Ranges mirror the constraints in schema.py; a parameter with a "default" may be
# omitted from the CSV (older control.csv files predate `aspirate_speed` and
# `capture_photo`). A spec without "min"/"max" is not range-checked.
PARAM_SPECS: Dict[str, dict] = {
    "robot1_volume_mL": {
        "type": float, "min": 0.0, "max": 10.0, "nonzero_min": PIPETTE_MIN_VOLUME_ML,
        "note": "Robot 1 aspirate/dispense volume (0 skips Robot 1, otherwise 0.5-10 mL)",
    },
    "robot2_volume_mL": {
        "type": float, "min": 0.0, "max": 10.0, "nonzero_min": PIPETTE_MIN_VOLUME_ML,
        "note": "Robot 2 aspirate/dispense volume (0 skips Robot 2, otherwise 0.5-10 mL)",
    },
    "aspirate_z_descent_mm": {
        "type": float, "min": 0.1, "max": 200.0,
        "note": "Z descent onto the source vial before aspirating (0.1-200 mm)",
    },
    "dispense_z_descent_mm": {
        "type": float, "min": 0.1, "max": 200.0,
        "note": "Z descent onto the vial on the balance before dispensing (0.1-200 mm)",
    },
    "aspirate_speed": {
        "type": int, "min": 1, "max": 9, "default": ASPIRATE_SPEED_DEFAULT,
        "note": "Pipette speed used for aspiration, both robots (1-9; 1 slowest). "
                "Optional; 1 is used if this row is missing, which is the value "
                "the paper's ZIF-8 runs used",
    },
    "robot1_dispense_speed": {
        "type": int, "min": 1, "max": 9,
        "note": "Robot 1 pipette speed for dispensing (1-9; 1 slowest, 9 fastest)",
    },
    "robot2_dispense_speed": {
        "type": int, "min": 1, "max": 9,
        "note": "Robot 2 pipette speed for dispensing (1-9; 1 slowest, 9 fastest)",
    },
    "robot1_angle_deg": {
        "type": float, "min": -360.0, "max": 360.0,
        "note": "Robot 1 base rotation from the source vial to the balance (-360 to 360 deg)",
    },
    "robot2_angle_deg": {
        "type": float, "min": -360.0, "max": 360.0,
        "note": "Robot 2 base rotation from the source vial to the balance (-360 to 360 deg)",
    },
    "robot1_dispense_radial_mm": {
        "type": float, "min": -100.0, "max": 100.0, "default": 0.0,
        "note": "Robot 1 radial offset of the dispense position (mm), applied after "
                "the rotation and before the Z descent; negative = towards the base. "
                "Optional; 0 (no offset, the paper's sequence) if this row is missing",
    },
    "robot2_dispense_radial_mm": {
        "type": float, "min": -100.0, "max": 100.0, "default": 0.0,
        "note": "Robot 2 radial offset of the dispense position (mm), applied after "
                "the rotation and before the Z descent; negative = towards the base. "
                "Optional; 0 (no offset, the paper's sequence) if this row is missing",
    },
    "blow_out_after_dispense": {
        "type": parse_bool, "default": False,
        "note": "TRUE to blow out the tip after dispensing, before the mass is read "
                "(same speed as the dispense). Optional; FALSE (the paper's "
                "sequence) if this row is missing",
    },
    "capture_photo": {
        "type": parse_bool, "default": False,
        "note": "TRUE to photograph the vial once at the end of the run "
                "(needs a camera). Optional; FALSE - the sequence the paper's "
                "ZIF-8 runs used - if this row is missing",
    },
}


def default_csv_path() -> str:
    """The CSV used when `--csv` is not given: the lab's own control.csv if it
    exists (it is gitignored), otherwise the published example."""
    return CONTROL_CSV_PATH if os.path.exists(CONTROL_CSV_PATH) else EXAMPLE_CSV_PATH


def load_params_from_csv(csv_path: str) -> dict:
    """Read the CSV and convert/range-check every value in PARAM_SPECS.

    Rows whose `parameter` cell is empty or starts with `#` are comments. Every
    other row must name a parameter in PARAM_SPECS exactly once: an unknown name
    (usually a typo such as `robot1_volume_ml`, which would otherwise leave the
    real row at its default) and a name given twice are both errors, reported
    with their line number in the file (the header is line 1).

    Raises:
        ValueError: with every problem found, so the operator can fix the whole
            spreadsheet in one pass instead of one error per run.
    """
    raw, first_line, errors = {}, {}, []
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        if "parameter" not in fields or "value" not in fields:
            raise ValueError(
                f"{csv_path}: the CSV needs 'parameter' and 'value' columns "
                f"(found: {fields})"
            )
        for row in reader:
            line = reader.line_num
            key = (row.get("parameter") or "").strip()
            if not key or key.startswith("#"):
                continue
            if key not in PARAM_SPECS:
                errors.append(f"line {line}: unknown parameter '{key}' "
                              f"(known: {', '.join(PARAM_SPECS)})")
                continue
            if key in raw:
                errors.append(f"line {line}: '{key}' is given twice "
                              f"(first on line {first_line[key]})")
                continue
            raw[key] = row.get("value")
            first_line[key] = line

    params = {}
    for name, spec in PARAM_SPECS.items():
        if name not in raw or raw[name] in (None, ""):
            if "default" in spec:
                params[name] = spec["default"]
                logger.warning(
                    f"'{name}' is not in {os.path.basename(csv_path)}; "
                    f"using the default {spec['default']}"
                )
            else:
                errors.append(f"'{name}' is missing from the CSV")
            continue
        try:
            value = spec["type"](str(raw[name]).strip())
        except (TypeError, ValueError):
            errors.append(
                f"'{name}' cannot be read as {spec['type'].__name__}: {raw[name]!r}"
            )
            continue
        if "min" in spec and not (spec["min"] <= value <= spec["max"]):
            errors.append(
                f"'{name}'={value} is out of range ({spec['min']} to {spec['max']})"
            )
            continue
        if "nonzero_min" in spec and value != 0 and value < spec["nonzero_min"]:
            # 0 means "skip this robot"; any other volume must be one the real
            # pipette wrapper accepts (the mock does not check the minimum).
            errors.append(
                f"'{name}'={value} is below the pipette minimum "
                f"{spec['nonzero_min']} mL (use 0 to skip the robot)"
            )
            continue
        params[name] = value

    if errors:
        raise ValueError(
            "CSV parameter validation failed:\n  - " + "\n  - ".join(errors)
        )
    return params


# ======================================================================
# CSV -> step list
# ======================================================================
def build_steps(params: dict) -> Tuple[List[dict], Dict[int, int]]:
    """Turn the CSV parameters into the fixed two-solution mixing step list.

    The ten steps per robot are, in order:

        1  move_z(-aspirate_z_descent_mm)      descend onto the source vial
        2  aspirate(volume, aspirate_speed)
        3  move_z(+aspirate_z_descent_mm)      rise
        4  rotate_relative(angle, "low")       swing over the vial on the balance
        4a move_radial(robotN_dispense_radial_mm)   only if the offset is not 0
        5  move_z(-dispense_z_descent_mm)      descend into the vial
        6  tare_scale(delay=1.0)               tare with the tip already lowered
        7  dispense(volume, robotN_dispense_speed)
        7a blow_out(go_home, speed, 3000 ms)   only if blow_out_after_dispense
        8  measure_weight(stabilization_count) read the mass before rising
        9  move_z(+dispense_z_descent_mm)      rise
       10  go_home()                           the home move also undoes the rotation

    Robot 1 runs its steps first, then Robot 2; a robot whose volume is 0 is
    skipped entirely. Without 4a and 7a this is exactly the sequence of the lab
    runner (`excel_runner/run_csv.py`, commit 4584333) that produced the paper's
    ZIF-8 batches: the tare happens immediately before the dispense at the lowered
    position, the balance is read before the tip rises, there is no rotation back
    (`go_home` returns the arm) and no photograph. Steps 4a and 7a are the two
    additions the lab runner gained later (commit fc68096): a radial offset of the
    dispense position, for fixtures where the vial on the balance is not exactly on
    the rotation arc, and a blow-out of the tip before the mass is read. Both are
    off by default. Setting `capture_photo` to TRUE appends a single
    `capture_and_save` after both robots, also an addition relative to the
    historical sequence.

    Returns:
        (steps, weight_step_owner) where weight_step_owner maps the 1-based index
        of each `measure_weight` step to the robot whose dispense it measures.
    """
    za = params["aspirate_z_descent_mm"]
    zd = params["dispense_z_descent_mm"]
    asp_speed = params["aspirate_speed"]

    steps: List[dict] = []
    weight_step_owner: Dict[int, int] = {}

    for rid in (1, 2):
        volume = params[f"robot{rid}_volume_mL"]
        if volume == 0:
            logger.info(f"Robot {rid}: volume is 0 - skipped")
            continue
        angle = params[f"robot{rid}_angle_deg"]
        dispense_speed = params[f"robot{rid}_dispense_speed"]
        radial = params.get(f"robot{rid}_dispense_radial_mm", 0.0)
        blow_out = params.get("blow_out_after_dispense", False)
        steps += [
            {"action": "move_z", "robot_id": rid, "distance": -za},
            {"action": "aspirate", "robot_id": rid, "volume": volume, "speed": asp_speed},
            {"action": "move_z", "robot_id": rid, "distance": za},
            {"action": "rotate_relative", "robot_id": rid, "angle": angle, "speed": "low"},
        ]
        if radial != 0:
            steps.append({"action": "move_radial", "robot_id": rid, "distance": radial})
        steps += [
            {"action": "move_z", "robot_id": rid, "distance": -zd},
            {"action": "tare_scale", "delay": TARE_DELAY_S},
            {"action": "dispense", "robot_id": rid, "volume": volume, "speed": dispense_speed},
        ]
        if blow_out:
            steps.append({"action": "blow_out", "robot_id": rid, "go_home": True,
                          "speed": dispense_speed, "delay_ms": BLOW_OUT_DELAY_MS})
        steps.append({"action": "measure_weight",
                      "stabilization_count": DEFAULT_STABILIZATION_COUNT})
        # 1-based index of the measure_weight step just appended
        weight_step_owner[len(steps)] = rid
        steps += [
            {"action": "move_z", "robot_id": rid, "distance": zd},
            {"action": "go_home", "robot_id": rid},
        ]

    if steps and params.get("capture_photo"):
        steps.append({"action": "capture_and_save", "file_path": ""})

    return steps, weight_step_owner


def build_workflow(params: dict) -> Tuple[ExperimentWorkflow, Dict[int, int]]:
    """Build the step list and validate it with the same schema the JSON flows use."""
    steps, weight_step_owner = build_steps(params)
    workflow = ExperimentWorkflow(
        name=WORKFLOW_NAME, description=WORKFLOW_DESCRIPTION, steps=steps
    )
    return workflow, weight_step_owner


# ======================================================================
# results.csv (written next to the input CSV, for the spreadsheet user)
# ======================================================================
def write_results_csv(input_csv_path: str, params: dict, results: dict) -> str:
    """Write results.csv beside the input CSV; fall back to a timestamped name if
    the file is open in a spreadsheet program (Windows locks it)."""
    out_dir = os.path.dirname(os.path.abspath(input_csv_path))
    out_path = os.path.join(out_dir, "results.csv")

    rows = [("last_run_at", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))]
    rows += [(name, params.get(name)) for name in PARAM_SPECS]
    rows += [
        ("robot1_weight_g", results.get("robot1_weight_g")),
        ("robot2_weight_g", results.get("robot2_weight_g")),
        ("log_dir", results.get("log_dir")),
    ]

    def _save(path):
        with open(path, "w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["key", "value"])
            writer.writerows(rows)

    try:
        _save(out_path)
        return out_path
    except PermissionError:
        fallback = os.path.join(
            out_dir, f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        logger.warning(f"cannot write {out_path} (open elsewhere?); saving {fallback}")
        _save(fallback)
        return fallback


# ======================================================================
# Execution
# ======================================================================
async def run_two_solution_mixing(
    params: dict,
    csv_path: str,
    *,
    mock: bool = False,
    record: bool = False,
    liquid_density: float = 1.0,
    robot_ports: Optional[dict] = None,
    shared_config: Optional[dict] = None,
) -> dict:
    """Run the fixed sequence built from `params`.

    The workspace preflight runs before the run folder is created or any device
    is opened; a violation raises `run_flow.PreflightError`. (The fixed CSV
    sequence uses only relative moves, which are reported as unverified and
    checked move by move at run time.)

    Returns:
        {"robot1_weight_g": float|None, "robot2_weight_g": float|None,
         "log_dir": str, "status": str}
    """
    workflow, weight_step_owner = build_workflow(params)
    steps = expand_loops([s.model_dump() for s in workflow.steps])
    results = {"robot1_weight_g": None, "robot2_weight_g": None}

    # Same preflight as run_flow (per-robot workspace + robot-type capabilities);
    # raises PreflightError before the run folder or any device exists.
    validator = default_workspace_validator()
    report = run_flow.require_preflight(steps, validator, log=logger, robot_ports=robot_ports)

    (robot_ids, picus2_robots, needs_scale, needs_camera,
     needs_microscope, needs_microscope_serial) = run_flow.plan_resources(steps)

    exp_logger = ExperimentLogger(
        workflow.name, workflow.description, csv_path, base_dir=LOGS_DIR,
        mode="mock" if mock else "real",
    )
    exp_logger.record_resources(robot_ids, picus2_robots, robot_types=report.robot_types)
    logger.info(f"experiment log folder: {exp_logger.dir}")
    results["log_dir"] = exp_logger.dir

    # Keep the exact steps that were run next to the CSV they came from.
    with open(os.path.join(exp_logger.dir, "generated_flow.json"), "w",
              encoding="utf-8") as f:
        json.dump(workflow.model_dump(), f, ensure_ascii=False, indent=2)

    accuracy_logger = DispenseAccuracyLogger(
        os.path.join(exp_logger.dir, "dispense_accuracy.csv"), density=liquid_density
    )

    if not steps:
        logger.warning("both volumes are 0 - nothing to run")
        exp_logger.finalize("completed", None)
        return {**results, "status": "completed"}

    logger.info(
        f"robots: {robot_ids} / aspirate speed {params['aspirate_speed']} / "
        f"Z aspirate {params['aspirate_z_descent_mm']} mm, "
        f"Z dispense {params['dispense_z_descent_mm']} mm"
    )

    video_recorder = None
    recording = None
    if record:
        if run_flow.ensure_dashboard_running():
            recording = run_flow.start_csv_recording(workflow.name)
            exp_logger.record_recording(recording.as_metadata())
            save_dir = recording.save_dir or exp_logger.dir
            try:
                from src.monitoring.camera.video_recorder import VideoRecorder
                video_recorder = VideoRecorder(
                    output_path=os.path.join(
                        save_dir, f"{os.path.basename(exp_logger.dir)}.mp4"),
                    camera_index=run_flow.lab_config.get_video_camera_index(),
                )
                if not video_recorder.start():
                    logger.warning("video recording failed to start (CSV recording continues)")
                    video_recorder = None
            except ImportError as e:
                logger.warning(f"video recording unavailable (OpenCV missing?): {e}")
                video_recorder = None
        else:
            logger.warning("dashboard not running - recording skipped")

    session = ExperimentSession(
        mock=mock,
        robot_ports=robot_ports,
        shared_config=shared_config,
        workspace_validator=validator,
        robot_validators=report.validators_by_robot,
    )

    async def body():
        for rid in robot_ids:
            await session.add_robot(rid, use_picus2=(rid in picus2_robots))
        if needs_scale or needs_camera or needs_microscope or needs_microscope_serial:
            await session.add_shared(use_scale=needs_scale, use_camera=needs_camera,
                                     use_microscope=needs_microscope,
                                     use_microscope_serial=needs_microscope_serial)

        logger.info("=== two-solution mixing started ===")
        total = len(steps)
        for i, step in enumerate(steps, 1):
            action = step.get("action")
            rid = None if action in SHARED_DEVICE_ACTIONS else step.get("robot_id", 1)
            label = "shared" if rid is None else f"Robot {rid}"
            logger.info(f"[{i}/{total}] {label}: {action}")

            t_start = time.monotonic()
            try:
                result = await execute_step(step, session.robots, session.shared, logger)
                exp_logger.record_step(i, total, action, rid, None, "ok",
                                       time.monotonic() - t_start, result=result)
            except Exception as step_err:
                exp_logger.record_step(i, total, action, rid, None, "error",
                                       time.monotonic() - t_start, error=str(step_err))
                raise

            if action == "dispense":
                accuracy_logger.record_dispense(
                    volume=step.get("volume"), speed=step.get("speed"), step_index=i
                )
            elif action == "measure_weight":
                weight = result.get("weight") if isinstance(result, dict) else result
                owner = weight_step_owner.get(i)
                if owner is not None:
                    results[f"robot{owner}_weight_g"] = weight
                err = accuracy_logger.record_weight(weight)
                if err is not None:
                    logger.info(f"  dispensing error: {err:+.4f} g")

        logger.info("=== two-solution mixing finished ===")

    try:
        await session.run(body)
    except (KeyboardInterrupt, asyncio.CancelledError):
        raise
    except Exception as e:  # noqa: BLE001 - reported through the return value
        logger.error(f"run failed: {e}")
    finally:
        if video_recorder is not None:
            try:
                video_recorder.stop()
            except Exception as e:  # noqa: BLE001
                logger.warning(f"error stopping the video recording: {e}")
        if recording is not None:
            try:
                # only a recording this run started (/api/start -> 200) is stopped
                run_flow.stop_csv_recording(recording)
            except Exception as e:  # noqa: BLE001
                logger.warning(f"error stopping the CSV recording: {e}")
        try:
            accuracy_logger.finalize(logger)
        except Exception as e:  # noqa: BLE001
            logger.warning(f"error finalising the accuracy CSV: {e}")
        exp_logger.finalize(session.status, session.error)
        logger.info(f"experiment log saved: {exp_logger.dir}")

    results["status"] = session.status
    for rid in (1, 2):
        w = results[f"robot{rid}_weight_g"]
        logger.info(
            f"Robot {rid} dispensed mass: {w:.3f} g" if isinstance(w, (int, float))
            else f"Robot {rid} dispensed mass: - (not run)"
        )
    return results


# ======================================================================
# CLI
# ======================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Run the fixed two-solution mixing experiment from a CSV file"
    )
    p.add_argument("--csv", default=None,
                   help="input CSV (default: control.csv next to this script, "
                        "or control.example.csv if it does not exist)")
    p.add_argument("--mock", action="store_true",
                   help="run without hardware (log only; recording always off)")
    p.add_argument("--validate-only", action="store_true",
                   help="read the CSV, build and validate the step list and run the "
                        "workspace preflight, but open no device")
    for rid in (1, 2, 3):
        p.add_argument(f"--robot{rid}-dobot", default=None,
                       help=f"Robot {rid} Dobot port "
                            f"(default: ROBOT{rid}_DOBOT_PORT / config.yaml)")
        p.add_argument(f"--robot{rid}-picus2", default=None,
                       help=f"Robot {rid} Picus 2 port "
                            f"(default: ROBOT{rid}_PICUS2_PORT / config.yaml)")
    p.add_argument("--scale-port", default=None,
                   help="balance port (default: SCALE_PORT / config.yaml)")
    record = p.add_mutually_exclusive_group()
    record.add_argument("--record", dest="record", action="store_true", default=None,
                        help="record sensor CSV and video (default for real runs)")
    record.add_argument("--no-record", dest="record", action="store_false",
                        help="do not record")
    p.add_argument("--liquid-density", type=float, default=1.0,
                   help="liquid density in g/mL for the accuracy CSV (default: 1.0 = water)")
    p.add_argument("-v", "--verbose", action="store_true")
    # Only used when the sheet sets capture_photo=TRUE; otherwise the CSV
    # sequence takes no photographs and no camera is opened.
    p.add_argument("--camera-index", type=int, default=None,
                   help="camera index for capture_photo (default: CAMERA_INDEX / config.yaml)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    csv_path = args.csv or default_csv_path()
    if not os.path.exists(csv_path):
        logger.error(f"file not found: {csv_path}")
        return 2
    logger.info(f"input CSV: {csv_path}")

    # Parameters are validated (and the step list built) before any device is
    # created, so a typo in the spreadsheet never reaches the hardware.
    try:
        params = load_params_from_csv(csv_path)
        workflow, _ = build_workflow(params)
    except ValueError as e:
        logger.error(str(e))
        return 2
    except Exception as e:  # pydantic.ValidationError etc.
        logger.error(f"the step list built from the CSV is invalid: {e}")
        return 2

    logger.info(f"steps built from the CSV: {len(workflow.steps)}")

    # Workspace preflight, same as run_flow: before any device is opened, and
    # with --validate-only too. Exit code 3 on a violation.
    steps = expand_loops([s.model_dump() for s in workflow.steps])
    robot_ports, shared_config = run_flow.resolve_ports(args)
    if not run_flow.run_preflight(steps, log=logger, robot_ports=robot_ports).ok:
        return run_flow.PREFLIGHT_EXIT_CODE
    if args.validate_only:
        logger.info("validation only (--validate-only)")
        return 0

    record = (not args.mock) and (True if args.record is None else args.record)

    try:
        results = asyncio.run(run_two_solution_mixing(
            params, csv_path,
            mock=args.mock,
            record=record,
            liquid_density=args.liquid_density,
            robot_ports=robot_ports,
            shared_config=shared_config,
        ))
    except run_flow.PreflightError as e:
        logger.error(str(e))
        return run_flow.PREFLIGHT_EXIT_CODE
    except KeyboardInterrupt:
        # The emergency stop and cleanup already ran inside ExperimentSession.
        logger.warning("interrupted (emergency stop and cleanup done)")
        return 130

    saved = write_results_csv(csv_path, params, results)
    logger.info(f"results written to {saved}")
    return 0 if results.get("status") == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
