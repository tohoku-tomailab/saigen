# Experimental flow format

The AI agent outputs experimental procedures as a JSON "experimental flow" — an
intermediate representation connecting the natural-language experimental objective
with the control code actually executed. Constraining the LLM output to structured
JSON suppresses generation variability and enables mechanical validation before
execution.

The schema is defined with Pydantic in [`src/flow/schema.py`](../src/flow/schema.py)
and executed by [`src/flow/executor.py`](../src/flow/executor.py). Run
`python src/flow/schema.py` to print the machine-readable JSON Schema.

## Top level

```json
{
  "name": "human-readable name",
  "description": "what the flow does",
  "steps": [ { "action": "...", ... }, ... ]
}
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `name` | string | yes | |
| `description` | string | no | default `""` |
| `steps` | array | yes | executed in order; each element is one of the actions below |

Every step has an `action` field. Robot actions take `robot_id` (1–4, default 1);
shared-device actions (balance, camera) do not. Unknown fields (e.g. a misspelled
`robotid`), out-of-range values and non-finite numbers (`NaN`, `Infinity`) are
rejected at validation time.

### Robot types

Each robot in `config.yaml` has a `type`. Robots 1–3 are Dobot Magicians
(`type: dobot`, the default when the entry has no `type`, so existing
`config.yaml` files keep working). Robot 4 is an optional Hiwonder MaxArm
(`type: maxarm`) with a suction nozzle and no pipette. The type decides which
robot actions a robot accepts:

| Type | Accepted robot actions |
|---|---|
| `dobot` | `move_xyz`, `move_z`, `move_radial`, `rotate`, `rotate_relative`, `go_home`, `move_slider`, `move_conveyer`, `wait`, `aspirate`, `dispense`, `blow_out` |
| `maxarm` | `move_xyz`, `move_z`, `go_home`, `suction`, `wait` |

The preflight rejects an action the robot's type does not accept (for example
`aspirate` on the MaxArm, or `suction` on a Dobot) before any device is opened,
exactly like a workspace violation (exit code 3 / `PreflightError`). The
executor refuses the same actions at run time as a second line of defence.

A robot can have its own `workspace` block under `robots.<id>` in
`config.yaml`. For a Dobot it is merged over the global `workspace` for that
robot only. A MaxArm (`type: maxarm`, at any robot id) **must** have one, and it
is used on its own, never merged over the global Dobot box (the frames differ):
`x_min` … `z_max` are all required, `r_min` / `r_max` (bounds on the horizontal
radius `hypot(x, y)`) are optional. A missing or incomplete MaxArm block, or any
workspace key left without a value (null), stops `--validate-only`, `--mock` and
real runs with a configuration error naming the robot, before any device is
opened. For the MaxArm the validator additionally runs the arm's own inverse
kinematics on the PC (`src/devices/maxarm/kinematics.py`): a `move_xyz` target
inside the box can still be unreachable (the wrist joint would leave 0..240°,
which happens near the top of the workspace), and such a step is rejected at
`--validate-only` with the highest reachable z at that (x, y). See
`docs/setup.md` §9.

## Actions

### Robot arm (Dobot Magician) — `robot_id` required

| `action` | Parameters | Description |
|---|---|---|
| `move_xyz` | `x`, `y`, `z` (mm) | Absolute move in the arm's Cartesian frame |
| `move_z` | `distance` (mm) | Relative vertical move; positive = up |
| `move_radial` | `distance` (mm) | Relative move along the radius from the base; positive = outward |
| `rotate` | `angle` (deg) | Absolute base (joint 1) rotation |
| `rotate_relative` | `angle` (deg), `speed` (`"low"`/`"normal"`/`"high"`, default `"low"`) | Relative base rotation; positive = counter-clockwise seen from above (toward +Y) |
| `go_home` | — | Return to the home position captured at start-up |
| `move_slider` | `position` (0–1000 mm) | Linear rail (optional accessory) |
| `move_conveyer` | `index` (0/1), `speed` (0–200 mm/s), `duration` (0–300 s) | Conveyor belt (optional accessory) |

### Electric pipette (Sartorius Picus 2) — `robot_id` required

| `action` | Parameters | Description |
|---|---|---|
| `aspirate` | `volume` (0.5 ≤ v ≤ 10 mL), `speed` (1–9, default 5) | Aspirate; the wrapper refuses to exceed 10 mL total |
| `dispense` | `volume` (0.5 ≤ v ≤ 10 mL), `speed` (1–9, default 5) | Dispense; refuses to exceed the held volume |
| `blow_out` | `go_home` (bool, default true), `speed` (1–9, default 1), `delay_ms` (int, default 3000) | Expel residual liquid |

Speed 1 is slowest, 9 fastest (about 1–11 mL/s for a 10 mL tip, from the manufacturer's timing table).
The lower bound of 0.5 mL is the real wrapper's minimum operating volume
(`LabRobot.min_pipette_volume`); the schema enforces it so that a flow the mock
accepts is not rejected on hardware.

### Suction nozzle (Hiwonder MaxArm) — `robot_id` of a `maxarm` robot

| `action` | Parameters | Description |
|---|---|---|
| `suction` | `on` (bool, default true) | `true` switches the pump on (the object is held); `false` releases it (the firmware opens the release valve for about 1 s, so wait before lifting). Only a robot with `type: maxarm` accepts it |

MaxArm coordinates are in the arm's own frame (mm, origin on the base yaw axis,
z = 0 at the base plane, the arm faces −y at home); `move_xyz` / `move_z` /
`go_home` behave as on the Dobot. After an error the session stops the MaxArm
where it is and does **not** send it home (it has no verified safe return path);
the Dobots are still sent home as before. See
[`examples/maxarm/pick_and_place.json`](../examples/maxarm/pick_and_place.json).

### Shared devices — no `robot_id`

| `action` | Parameters | Description |
|---|---|---|
| `tare_scale` | `delay` (0.1–10 s, default 1.0) | Tare the electronic balance |
| `measure_weight` | `stabilization_count` (1–10, default 3) | Read the balance; median of N readings, logged in grams |
| `capture_and_save` | `file_path` (string, `""` = auto-generate) | Capture a webcam image. If no image is saved the step fails (and the run stops) rather than being recorded as `ok`; the same holds for `capture_microscope` |
| `capture_microscope` | `file_path` (string, `""` = auto-generate) | Capture an image with the USB digital microscope (UVC, e.g. Sanwa Supply 400-CAM106); saved into the run's `images/` folder with `_microscope` in the name. Opens only the microscope camera; `microscope_led` opens only its serial port; `microscope_focus` opens both |
| `microscope_led` | `on` (bool, default true), `level` (0–255, optional) | Switch the microscope's LED ring on or off and optionally set its brightness (factory level 12). Needs `microscope_port` in `config.yaml` (the UM22's CP210x serial port) |
| `microscope_focus` | `mode` (`auto` / `position` / `step`, default `auto`), `position` (0–65535, for `position`), `direction` (`in`/`out`) and `steps` (1–100, for `step`), `timeout` (1–300 s, default 60) | Focus the microscope: single-shot autofocus, move the lens motor to a recorded position, or step it. Waits until the motor stops and writes the lens position and whether it converged to the `focus_position` / `focus_converged` columns of `measurements.csv`. Autofocus needs a textured target and a non-saturating LED level; on a blank field it hunts until `timeout` and then falls back to manual mode. For reproducibility, record the position once and replay it with `mode: "position"`. Needs `microscope_port` |

### Utility and control flow

| `action` | Parameters | Description |
|---|---|---|
| `wait` | `seconds` (≥ 0) | Sleep |
| `loop_start` | `loop_id` (string), `count` (1–1000) | Begin a loop |
| `loop_end` | `loop_id` | End the loop with the same `loop_id` |

Loops are expanded into a flat list before execution (`expand_loops`). Nesting is not
supported; a `loop_start` without a matching `loop_end` (or vice versa), or a
`loop_end` with a different `loop_id` inside a loop body, is an error.

## Validation rules

Applied before any device moves:

1. **Schema** — every step must match exactly one action model; types and ranges
   above are enforced by Pydantic.
2. **Loop structure** — `loop_id`s must pair up and must not nest.
3. **Workspace and capability preflight** (`--validate-only` and before every
   run, no device opened) — every absolute target (`move_xyz`, `rotate`) is
   checked against the robot's workspace (for a Dobot, `robots.<id>.workspace`
   merged over the global `workspace` section of `config.yaml`, or the global
   section alone when the robot has none; for a MaxArm, its own required
   `robots.<id>.workspace` alone), and every robot action is checked against the
   robot's type (see *Robot types*). Relative moves (`move_z`,
   `rotate_relative`, `move_radial`) depend on the start pose and are listed as
   "unverified until run"; rule 5 checks them. The same preflight
   (`run_flow.run_preflight` / `require_preflight`) guards all three runners: the
   CLI (`run_flow.py`, exit code 3), the CSV runner (`run_csv.py`, exit code 3)
   and the GUI (`src/gui/runner.py`, which refuses to start the run and shows the
   offending steps and coordinates).
4. **Pipette volume tracking** (run time, in the safety wrapper) — cumulative
   `aspirate` may not exceed 10 mL; `dispense` may not exceed the volume held.
5. **Workspace limits** (run time, move by move) — `move_xyz` / `move_z` / `rotate*` targets are
   checked against the XYZ box and joint-1 range of the robot's workspace (as in
   rule 3); a violation raises before the command is sent, aborting the run.
   The same check runs in `--mock` mode, so a flow can be screened for out-of-range
   moves without hardware.

Conventions the agent's system prompt also enforces: Robot 3 (rail robot) carries
no pipette; each loop body and the whole flow end with `go_home`.

## Full example — ZIF-8 two-solution mixing

This is `examples/zif8/zif8_two_solution_mixing_speed5.json` (solution 2 dispensed at
pipette speed 5, nominal ≈1.45 s for 5 mL). The `speed1` and `speed9` variants differ
only in the `speed` of the second `dispense`.

The three files are the **JSON equivalents of the CSV-runner sequence** — the fixed
ten-step-per-robot sequence in [`docs/csv-runner.md`](csv-runner.md) that the paper's
ZIF-8 batches were actually run with — plus one closing `capture_and_save`, which the
CSV runs did not take. They are not the files that were executed for the paper; see
`examples/zif8/README.md`.

Running a flow leaves a record of the run in
`logs/<YYYY-MM-DD>/<flow name>_<timestamp>/`: `run.log`, `measurements.csv` (one row
per step, with the weights read, the nominal pipette duration of each aspirate/dispense
step from the Picus 2 timing table, and the photographs taken), `summary.md`,
`metadata.json`, `images/` and a copy of the flow JSON. `metadata.json` also
records the type of every robot used (`robot_types`, e.g. `{"4": "maxarm"}`).
The execution mode
(`mock` / `real`) is recorded in `metadata.json` (`mode`), in a `mode` column of
`measurements.csv` and in `summary.md`. Runs started within the same second get
distinct folders (`_2`, `_3`, … suffix). When sensor recording is on, a run stops
only a dashboard recording it started itself (`/api/start` → 200); a recording that
was already running (409) is left running, and `metadata.json` →
`sensor_recording.started_by_this_run` says which. See
[`src/flow/README.md`](../src/flow/README.md).

```json
{
  "name": "ZIF-8 two-solution mixing (solution 2 dispensed at pipette speed 5)",
  "description": "… (see the file)",
  "steps": [
    {"action": "move_z", "robot_id": 1, "distance": -0.1},
    {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 1},
    {"action": "move_z", "robot_id": 1, "distance": 0.1},
    {"action": "rotate_relative", "robot_id": 1, "angle": 90.0, "speed": "low"},
    {"action": "move_z", "robot_id": 1, "distance": -120.0},
    {"action": "tare_scale", "delay": 1.0},
    {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 9},
    {"action": "measure_weight", "stabilization_count": 3},
    {"action": "move_z", "robot_id": 1, "distance": 120.0},
    {"action": "go_home", "robot_id": 1},

    {"action": "move_z", "robot_id": 2, "distance": -0.1},
    {"action": "aspirate", "robot_id": 2, "volume": 5.0, "speed": 1},
    {"action": "move_z", "robot_id": 2, "distance": 0.1},
    {"action": "rotate_relative", "robot_id": 2, "angle": -90.0, "speed": "low"},
    {"action": "move_z", "robot_id": 2, "distance": -120.0},
    {"action": "tare_scale", "delay": 1.0},
    {"action": "dispense", "robot_id": 2, "volume": 5.0, "speed": 5},
    {"action": "measure_weight", "stabilization_count": 3},
    {"action": "move_z", "robot_id": 2, "distance": 120.0},
    {"action": "go_home", "robot_id": 2},

    {"action": "capture_and_save", "file_path": ""}
  ]
}
```

Note the order inside each robot's block: the balance is tared only after the tip is
down in the vial, the mass is read before the tip rises again, and `go_home` — not a
second `rotate_relative` — brings the arm back.

Distances and angles are specific to the fixture geometry; see
`examples/zif8/README.md` for how to adapt them.

## Presets

Flows that have been checked on hardware are stored as *presets* in
`src/agent/presets/*.json`. A preset wraps a flow in `{"label", "description",
"template"}` and is offered in the GUI; the agent can also be asked to change only
the parameters of a preset (the step order is kept), which is the recommended way to
derive a new condition from a validated procedure.
