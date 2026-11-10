# Guidance for AI coding agents working in this repository

This file is read by Claude Code (`CLAUDE.md`) and by other agents (`AGENTS.md`,
an identical copy). It tells an agent what SAIGEN is, how to verify changes, and
which parts must not be changed on the agent's own judgement. The human-facing
introduction is `README.md`; read it first.

## What this repository is

SAIGEN (Standardized Automation Infrastructure for Generalizable Experiment
reproductioN) is the control software of a small automated materials-synthesis
cell: Dobot Magician robot arms with Picus 2 electronic pipettes, an electronic
balance, IKA hot-plate stirrers, webcams and a Raspberry Pi sensor logger.
Experiments are described as JSON "flows" (or as a parameter spreadsheet) and
executed step by step through a safety wrapper that checks every robot move
against configured workspace limits before the hardware is commanded.

The typical reason a user hands this repository to an agent is to adapt it to
their own hardware: different ports, a different number of robots, no balance,
a different vial layout, a new experiment. Most of that adaptation belongs in
configuration and flow files, not in the driver code.

## Where things live

| Path | Role |
|---|---|
| `config.example.yaml` -> `config.yaml` | Serial ports per robot, shared devices, robot workspace limits. `config.yaml` is gitignored. |
| `.env.example` -> `.env` | API keys for flow generation, optional per-run port overrides, monitoring settings. Never committed. |
| `src/flow/` | JSON flow schema, validator, executor and the CLI runner `run_flow.py`. |
| `src/flow/csv_runner/` | Spreadsheet-driven runner for the two-solution mixing experiment (`run_csv.py`). |
| `src/devices/safety/` | `LabRobot` (real), `MockLabRobot` (no hardware), shared devices and the workspace validators. **All flows run through this layer.** |
| `src/devices/{dobot,picus2,scale,ika,webcam}/` | Per-instrument drivers. |
| `src/devices/keithley2002/` | Keithley 2002 digital multimeter over GPIB (PyVISA, optional, `measure_dmm`). Its self-check `python -m src.devices.keithley2002.keithley2002 <resource>` talks to the instrument: the human's step. |
| `src/devices/li5640/` | NF LI5640 lock-in amplifier over GPIB (PyVISA, optional, read-only, `measure_lockin`). Its front-panel settings are never changed; the only setting written is the read-out selection `OTYP`, restored on close. Exception, off by default: `set_lockin_oscillator` writes the internal oscillator (OSC OUT) only when `lockin_osc_max_vrms` in `config.yaml` is above 0; it never switches the amplitude range and restores the starting values on close (a restore that fails or cannot be checked marks the run failed). Never raise those limits on your own: they protect what is wired to OSC OUT (rule 2 applies to them as to workspace limits). Its self-check `python -m src.devices.li5640.li5640 <resource>` talks to the instrument: the human's step. |
| `src/devices/dso1000/` | Agilent 1000 series oscilloscope (DSO1014A etc.) over USB (PyVISA, optional, read-only, `measure_scope`). Sends queries only (plus a device clear on open and close, and `*CLS` on open); the set-up stays on the front panel. Its self-check `python -m src.devices.dso1000.dso1000 <resource>` talks to the instrument: the human's step. |
| `src/devices/mg400/` | Dobot MG400 over TCP/IP: robot type `mg400` (optional robot 5; `move_xyz`, `move_z`, `go_home`, `wait` only, every move one `MovL` checked against the robot's own box, the segment's radius and the controller's `InverseSolution` within `joint_limits`; at rest, at its checks (the Enable, every enabled pose read, before every `MovL`, arrival, between requests; sampled, not continuous), a joint whose measured angle in the feedback (`q_actual`) is more than 0.2 deg from its target, or (except between requests, where only the target is compared) from the `GetAngle()` value about to be used, stops the run (at arrival the arm may settle until the move deadline; no Cartesian accuracy beyond the arrival check is claimed); wrapped by `MG400Robot` / `MockMG400Robot` in `src/devices/safety/mg400_robot.py`, which runs on `control.Controller`, the one-thread worker of `control.py`; a real run asks the human for Enter before any device is opened, and the GUI's runner refuses a real MG400 run without the ids its confirmation dialog approved) and two human-run tools. `python -m src.devices.mg400.observe <address>` is **read-only**: six commands (RobotMode, GetErrorID, GetPose, GetAngle, PositiveSolution, InverseSolution; built, never free text), the feedback port read, the motion port never opened. `python -m src.devices.mg400.jog <IP> --controller-version V` **moves the arm**: after the human's Enter on a terminal it sends ResetRobot, EnableRobot, two `JointMovJ` (J1 / J4 only, 1..10 deg, SpeedJ = AccJ at most 30 % of the controller's own setting) and DisableRobot; its stop sequence sends ResetRobot, DisableRobot and then EmergencyStop unless the arm showed itself disabled (or in alarm with its joints still) and the Enable's outcome is known; once the arm has reported disabled again after its DisableRobot, a later fault or Ctrl+C adds no stop sequence (only the final read-only queries follow). Both tools and every flow on the arm are the arm, so the real run is the human's (only `--mock`, `--dry-run` and `--validate-only` are the agent's). Do not add other commands that enable, move or stop the arm, widen the jog limits, raise `speed_ratio` above 10 or `max_move_mm` above 50, or arm a connection in `observe.py`, without a reviewed plan. |
| `src/devices/maxarm/` | Hiwonder MaxArm (optional robot 4, `type: maxarm`): serial client, protocol and the MicroPython firmware with its human-run installer `deploy_firmware.py`. Wrapped by `MaxArmRobot` / `MockMaxArmRobot` in `src/devices/safety/`. |
| `src/agent/` | LLM-based generation of flows from natural language, plus preset flows. |
| `src/gui/` | Streamlit GUI. `src/voice/` voice input. |
| `src/monitoring/` | Sensor dashboard (FastAPI) and Raspberry Pi agent; own `config.example.yaml`. |
| `src/imaging/` | Appearance-imaging system (light + camera), separate README. |
| `detection/` | YOLOv8 object detection. **AGPL-3.0**, isolated on purpose; never import it from `src/`. |
| `examples/zif8/` | Example flows of the paper's ZIF-8 demonstration. |
| `examples/microscope/` | Example flow using the USB digital microscope (`capture_microscope`). |
| `examples/maxarm/` | Suction pick-and-place flow for the MaxArm (robot 4). |
| `examples/mg400/` | `z_hop.json` for the MG400 (robot 5) and `config.mg400.yaml`, a complete `config.yaml` with the MG400 entry (no address; its numbers are not measurements). The CI runs the flow against it. |
| `examples/keithley2002/` | DMM-only flow for the Keithley 2002 (`measure_dmm`). |
| `examples/li5640/` | Lock-in-only flow for the NF LI5640 (`measure_lockin`); `oscillator/` holds an oscillator sweep read on the oscilloscope (`set_lockin_oscillator`, refused unless `config.yaml` enables it). |
| `examples/dso1000/` | Oscilloscope-only flow for the Agilent 1000 series (`measure_scope`). |
| `docs/` | Bill of materials, setup guide, flow format reference, spreadsheet guide, guide to adding an instrument, MaxArm commissioning notes (`maxarm-commissioning.md`: what the bench taught, numbers, reachable envelope, recovery), Keithley 2002 commissioning notes (`keithley2002-commissioning.md`: order of work, what the firmware accepted, open observations, interruption rules), NF LI5640 commissioning notes (`li5640-commissioning.md`: read-only scope, what the firmware answered, defects found by review, open points, the internal-oscillator bench test), Agilent DSO1014A commissioning notes (`dso1000-commissioning.md`: reply forms, GTL hang over USB, defects found by review, open points), Dobot MG400 notes (`mg400-commissioning.md`: replies seen, the refused GetPose form, the record under OneDrive, the dropped first Dashboard connection, the order of work for the jog tool and the flow bench stages C1 to D, open points, what Ctrl+C does in each tool and in a flow). |
| `tests/`, `src/monitoring/tests/`, `src/imaging/tests/` | Unit tests; none need hardware. |

## How to verify a change (no hardware needed)

```bash
pip install -r requirements.txt            # or the lighter set in .github/workflows/ci.yml
python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q
python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --validate-only
python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --mock
python -m src.flow.csv_runner.run_csv --mock
```

`--validate-only` checks a flow against the schema and the loop structure, and
checks every absolute target (`move_xyz`, `rotate`) against the workspace limits,
without opening any device. Relative moves (`move_z`, `rotate_relative`,
`move_radial`) depend on the start pose, so `--validate-only` lists them as
"unverified until run"; they are checked move by move at `--mock` and at run
time. `--mock` executes the flow end to end against simulated devices, with the
same validation. Run both after any change to a flow, the schema, the executor
or `config.yaml`. A `config.yaml` that exists but cannot be parsed is an error,
never a silent fallback to the built-in defaults. The CI workflow in
`.github/workflows/ci.yml` runs the same steps and must stay green.

## Rules

1. **Never start a real run on your own.** Anything without `--mock` (or
   without *Mock* in the GUI) moves a robot arm and dispenses liquid; this
   includes the homing tool `src/devices/dobot/home_dobot.py`, whose sweep is
   decided by the firmware and cannot be checked by the validator, and the
   MaxArm firmware installer `src/devices/maxarm/deploy_firmware.py`: installing
   or restoring firmware, and any reset of the ESP32 (including opening its
   serial port with DTR/RTS toggling), can make the factory program move the
   arm, and the MaxArm jog/teach tool `src/devices/maxarm/teach.py` (every
   command moves the arm; only `--mock` is safe). Run
   `--validate-only`, then `--mock`, and stop there. A real run happens only
   when the human explicitly asks for it in the current conversation, after the
   mock run succeeded, and it is the human who presses Enter. Ctrl+C is the
   emergency stop: the arm halts where it is. Exception, the MG400 tools: in
   `observe.py` Ctrl+C sends nothing; in `jog.py` it is taken at the tool's next
   send (within 0.1 s normally, about 6 s at most) and starts an orderly stop
   (ResetRobot, DisableRobot, then EmergencyStop unless the arm showed itself
   disabled, or in alarm with its joints still, and the Enable's outcome is
   known) that takes up to 15 s, unless the arm has already reported disabled
   again after its DisableRobot, in which case no stop sequence is added; the
   record is then written (normally a few seconds; one more Ctrl+C is deferred
   to its end, a second one ends it at once). In a flow on an MG400 robot
   (`type: mg400`), Ctrl+C and the GUI Stop reach the control worker as the
   same latch (taken at its next send, within about 0.1 s when idle) and start
   the same orderly stop; the session never sends an MG400 home after an error,
   and a stop that ends in EmergencyStop or unconfirmed marks the run failed. In
   the terminal the first Ctrl+C reaches the worker through a separate thread
   (also while the run is busy in another step), and every later Ctrl+C is
   deferred until the run has finished its stop, disconnect and record (the
   MG400's own disconnect wait is bounded by about 25 s; a step stuck in another
   device is not, and then the human ends the process, `docs/setup.md` §13). The MG400's emergency stop is the button on its controller, which the human
   must have within reach.
2. **Do not weaken the safety layer.** Do not edit `src/devices/safety/`, the
   workspace validators, or the `workspace` limits in `config.yaml` in order to
   make a failing move pass. A move rejected by the validator is the safety
   system doing its job; report it to the human with the offending coordinates
   and let them decide, after measuring their cell. Only tighten limits on your
   own.
3. **Adapt through configuration and flows, not drivers.** Ports, robot count,
   presence of a pipette or balance, vial positions, volumes, speeds and
   step order are all expressible in `config.yaml`, `.env`, JSON flows and the
   CSV sheet. Change those first. Edit the drivers in `src/devices/` only for
   a genuine defect or a new instrument, and add a test when you do. Keeping
   the shared code unchanged is what makes runs comparable between labs.
4. **Keep secrets and lab-specific values out of the tree.** API keys go in
   `.env`; ports and limits in `config.yaml`; both are gitignored. Never write
   real keys, IP addresses of lab machines, or Bluetooth addresses into tracked
   files, examples or documentation.
5. **Respect the licence boundary.** `detection/` depends on AGPL-3.0
   Ultralytics. Nothing under `src/`, `examples/` or `tests/` may import it.
6. **Do not invent measurements.** If a user asks for data the repository does
   not contain (the paper's raw batch logs, camera images, sensor CSVs), say so.
   `examples/zif8/` contains flows and CSV templates, not the published
   measurements, except where a README says otherwise.
7. **Write tests for what you change** and run the full suite before reporting
   done. Tests must keep working without hardware; use `MockLabRobot` and
   `MockSharedDevices` from `src/devices/safety/`.

## Typical tasks and where to start

- *"Set this up for my hardware"*: copy `config.example.yaml` to `config.yaml`
  and `.env.example` to `.env`; ask the user for the ports (on Windows they are
  in Device Manager, on Linux/macOS `ls /dev/tty*`); then `--validate-only`
  and `--mock` an example flow. Only the robots and devices a flow's steps
  actually reference are opened, so entries for hardware the user does not
  have can stay in `config.yaml` (it is deep-merged over built-in defaults, so
  deleting an entry does not remove it).
- *"Write a flow for experiment X"*: read `docs/experimental-flow.md`, start
  from the nearest file in `examples/zif8/`, validate, mock-run, then hand the
  file to the user for a real run. `src/agent/` can also draft flows from
  natural language if an API key is configured.
- *"The robot hit something / a move was rejected"*: see rule 2. Collect the
  coordinates from `logs/<date>/<flow>_<time>/run.log` and report.
- *"Use the MaxArm"*: robot 4 in `config.example.yaml` is a `type: maxarm`
  entry with an empty `maxarm_port` and a conservative example `workspace`
  in the MaxArm's own frame. Ask the user for the port; start from
  `examples/maxarm/pick_and_place.json`; `--validate-only`, then `--mock`.
  The MaxArm accepts only `move_xyz`, `move_z`, `go_home`, `suction` and
  `wait`, and the preflight rejects other actions on it. Its validator also
  runs the arm's inverse kinematics (`src/devices/maxarm/kinematics.py`): a
  target inside the box can still be unreachable (near the top of the
  workspace the wrist angle drops below 0°), and the rejection names the
  highest reachable z at that (x, y). Do not loosen that check; lower the
  target. Set `home_position` to a reachable pose. Firmware backup and
  installation (`docs/setup.md` §9) are the human's job.
- *"Use the MG400"*: copy `examples/mg400/config.mg400.yaml` to `config.yaml`
  (robot 5, `type: mg400`): every key of the entry is required, nothing is
  inherited, and its numbers are not measurements. Ask the user for the
  address (chat only, never a tracked file) and for the rest pose and clear
  volume measured in DobotStudio Pro; tighten `workspace` and set
  `home_position` to the rest pose; for `--mock` at that pose set
  `shared_devices.mock_start_pose` (the rest pose, x, y, z, r) and
  `mock_kinematics: {5: fitted_planar_v1}` (a fitted approximation, still a
  fake). Start from `examples/mg400/z_hop.json`;
  `--validate-only`, then `--mock`, and stop there: the real run asks the
  human for Enter on the terminal (`docs/setup.md` §13), and the bench stages
  C1 to D (`docs/mg400-commissioning.md` §2) run only when the human asks.
  The MG400 accepts only `move_xyz`, `move_z`, `go_home` and `wait`; a move
  refused by its six validation steps is reported with the step and the
  value, never worked around (rule 2: `joint_limits` only tighten,
  `speed_ratio` stays at most 10, `max_move_mm` at most 50).
- *"Add a new instrument"*: follow `docs/adding-an-instrument.md` step by
  step: a driver package under `src/devices/`, a mock counterpart, registration
  with `SharedDevices` / `MockSharedDevices`, the flow schema and executor, the
  run-log column, configuration keys, tests, documentation and, if the step
  should be available from the GUI, the block palette in `src/gui/app.py`.
