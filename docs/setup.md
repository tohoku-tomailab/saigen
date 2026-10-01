# Setup guide

From an empty control PC to the first automated dispense. Windows 11 is the
reference platform (the pipette and balance drivers use COM ports); the flow
executor, agent, GUI and voice input also run on macOS/Linux with `/dev/tty*`
device names.

## 1. Hardware

See [`bom.md`](bom.md) for the full list. The minimum for the ZIF-8 demonstration:

- 2 × robot arm (Dobot Magician), USB
- 2 × electric pipette (Sartorius Picus 2, 10 mL), USB (Bluetooth is also supported)
- 1 × electronic balance (Sartorius BCE822i) with RS-232C/USB serial
- 2 × hot-plate stirrer (IKA RET control-visc, or a cheaper IKA plate with the same
  NAMUR serial interface such as the IKA Plate (RCT digital); see `bom.md`) — optional, RS-232C/USB
- 1 × USB webcam
- 1 × USB digital microscope (Sanwa Supply 400-CAM106 or any UVC microscope) — optional,
  used by the `capture_microscope` step
- 1 × Hiwonder MaxArm with suction nozzle — optional robot 4 for pick-and-place,
  USB (see §9); not needed for the ZIF-8 demonstration
- 3D-printed fixtures from `cad/` (pipette holders on the arm tips, flask holders,
  balance splash guard)

Assembly: mount one pipette on each arm, place the receiving vial on the balance
between the arms, put each solution flask on its stirrer within reach of the
corresponding arm. **One arm + one pipette per solution**, so tips are never
exchanged.

Connect everything to the control PC and note the port of each device
(Windows: Device Manager → *Ports (COM & LPT)*; unplug/replug to identify).

## 2. Software

```bash
git clone https://github.com/YusukeHashimotoLab/saigen.git
cd saigen
python -m venv venv
# Windows
venv\Scripts\activate
# macOS / Linux
source venv/bin/activate
pip install -r requirements.txt
```

Python 3.11–3.13. Optional extras:

- `ffmpeg` on PATH for voice input (`winget install ffmpeg` / `brew install ffmpeg`).
- A CUDA GPU speeds up whisper; CPU works with the int8 model.

## 3. Configuration (`config.yaml` and `.env`)

Two files, with different jobs: `config.yaml` holds the ports and the workspace
limits of your cell; `.env` holds API keys and per-run overrides. Both are
git-ignored.

```bash
cp config.example.yaml config.yaml
cp .env.example .env
```

Edit `config.yaml`:

| Section | Purpose |
|---|---|
| `robots` | `dobot_port` (and `picus2_address`, if that robot has a pipette) per `robot_id`. Robot 4 is an optional MaxArm (`type: maxarm`, `maxarm_port`, its own `workspace`, see §9); a robot without `type` is a Dobot |
| `shared_devices` | `scale_port`, `camera_index`, `microscope_index`, `microscope_port` and `microscope_resolution` of the balance, the photo camera and the digital microscope (camera index for photos, serial port for the LED and focus, capture size; 4K by default, lower it when the image volume gets heavy) |
| `workspace` | X/Y/Z and Joint-1 limits every move is checked against before it is commanded |

If `config.yaml` is absent the tracked `config.example.yaml` is used, and if that is
absent too the built-in defaults in `src/config.py` apply. Sensor-dashboard ports and
the video-recording camera index live in `src/monitoring/config.yaml` (section 8).

Edit `.env`:

| Variable | Purpose |
|---|---|
| `LLM_MODE` | `gemini` (default) or `openai_compatible` |
| `GEMINI_API_KEY` | required for `gemini`; free key from Google AI Studio |
| `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` | for any OpenAI-compatible endpoint (local inference server etc.) |
| `ROBOT1_DOBOT_PORT`, `ROBOT1_PICUS2_PORT`, `ROBOT2_*`, `ROBOT3_DOBOT_PORT`, `ROBOT4_MAXARM_PORT` | optional per-run override of the `config.yaml` robot ports |
| `SCALE_PORT`, `CAMERA_INDEX`, `MICROSCOPE_INDEX`, `MICROSCOPE_PORT` | optional per-run override of the shared devices |
| `SENSOR_SERVER_URL` | dashboard URL used to start/stop recording (optional) |

`.env` is git-ignored. No key is read from anywhere else. For device ports the
resolution order is: command-line flag > `.env` > `config.yaml` > built-in defaults.

## 4. Check the devices one by one

Each driver has a small self-test; run them with the arm clear of obstacles.

```bash
python -m src.devices.dobot.pydobot_controller          # lists serial ports, connects, reads position
python src/devices/picus2/test_picus2_minimal.py         # edit the port inside the file first
python -m src.devices.scale.scale_controller             # reads the balance
```

The Dobot Magician only knows its joint angles precisely after a **homing** run
(the firmware drives joint 1 to its end stop and re-zeroes the encoders). Do it
once after every power-on, one arm at a time, with the area around the arm clear:

```bash
python -m src.devices.dobot.home_dobot --robot 1 --mock   # rehearse the procedure, no hardware
python -m src.devices.dobot.home_dobot --robot 1          # real: asks for Enter before the arm moves
python -m src.devices.dobot.home_dobot --robot 2
```

The port comes from `config.yaml` (or `.env` / `--port`). `--target` chooses
where the arm returns after homing: `current` (default, the pose it started
from), `zero` (base angle 0°, same radius and height) or `config`
(`DobotConfig.HOME_SETTINGS`). The return pose is checked against the
`workspace` limits before anything is sent, and the tool refuses if it is
outside. The homing sweep itself is decided by the firmware and cannot be
validated, so keep the whole reach clear. Ctrl+C during the sweep is the same
emergency stop as in a flow: the queue is force-stopped and the arm halts.

For the arm, confirm the `workspace` section of `config.yaml` matches your fixtures
before running any flow. Absolute targets (`move_xyz`, `rotate`) are checked
against those limits by a preflight before any device is opened, both with
`--validate-only` and at the start of every run. Relative moves (`move_z`,
`rotate_relative`, `move_radial`) depend on where the arm starts, so they are
checked move by move, **before** each command is sent, in `--mock` mode and in
a real run; a violation aborts the run. If `config.yaml` exists but cannot be
read (invalid YAML, PyYAML missing), every command refuses to start instead of
falling back to the wider built-in defaults.

## 5. First run without hardware

```bash
python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --validate-only
python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --mock
```

`--validate-only` checks the JSON against the schema (unknown keys, NaN and
out-of-range values are rejected), expands loops and checks every absolute target
(`move_xyz`, `rotate`) against the workspace limits, without opening any device; a
violation exits non-zero with the step number, coordinates and limits. Relative
moves are reported as "unverified until run".
`--mock` runs the whole flow against simulated robots and logs every action; the
workspace limits are enforced move by move, including relative moves, and
recording is always off. A mock run produces the
same `logs/<date>/<flow name>_<timestamp>/` folder as a real one, so you can inspect
`run.log`, `measurements.csv`, `summary.md` and `metadata.json` before touching
hardware.

The mock enforces the same pipette limits as the real wrapper (10 mL capacity,
0.5 mL minimum, no dispensing more than is held, `blow_out` empties the tip), so a
flow that dispenses before it aspirates fails in the mock exactly as it would on
the bench.

**Set the mock start pose to the pose you will start the real run from.** A real
run takes the arm's *current* pose as home and validates every relative move
(`move_z`, `move_radial`, `rotate_relative`) from there. The mock cannot know where
your arm is, so it starts from a fixed pose, X=250, Y=0, Z=150, R=0 by default, and
validates relative moves from that pose. If the real start pose is different, a
mock run can pass a move that the real run rejects, or the other way round.
Read the pose off the arm (DobotStudio, or the start of a previous `run.log`) and
set it either in `config.yaml`:

```yaml
shared_devices:
  mock_start_pose: "200,0,50,0"      # x, y, z, r (mm, mm, mm, degrees)
  # or per robot:
  # mock_start_pose: {1: "200,0,50,0", 2: "0,200,50,90"}
```

or for one run with the environment variable `MOCK_START_POSE=200,0,50,0` (it takes
precedence over `config.yaml`). Joint 1 is derived from X and Y, as on the arm. An
unparsable value stops the mock run instead of silently falling back to the default.

Run the test suite too — it needs no hardware:

```bash
pytest tests/
```

## 6. First run with hardware

0. Home each arm once after power-on (`python -m src.devices.dobot.home_dobot
   --robot N`, see §4). Without it the joint angles the firmware reports can be
   off, and vertical `move_z` moves come out slanted.
1. Move each arm by hand (or with the driver self-test) to the pose you want as
   *home*: pipette tip above the flask, clear of everything. The runner captures the
   current pose as home at start-up; `go_home` returns there.
2. Check the reach: run the position-check presets (`src/agent/presets/wdc_2_dispense_robot1.json`
   etc.) — they move without aspirating.
3. Run the flow:

```bash
python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json \
    --robot1-dobot COM12 --robot1-picus2 COM4 \
    --robot2-dobot COM7  --robot2-picus2 COM5 \
    --scale-port COM9 --camera-index 1
```

Flags override `.env`, which overrides `config.yaml`. Only the robots referenced by
`robot_id` in the flow are initialised.

Press Ctrl+C to stop: this is an **emergency stop** — the Dobot command queue is
force-stopped so the move in progress halts immediately, the queue is discarded, the
conveyor motors and the IKA heater/stirrer are switched off, and no further motion
(not even going home) is commanded before the devices are disconnected.

Every run writes `logs/<YYYY-MM-DD>/<flow name>_<timestamp>/` containing `run.log`,
`measurements.csv` (one row per step, with the weights and image paths),
`summary.md`, `metadata.json`, `images/` and a copy of the flow JSON. Real runs also
record sensor CSV and overhead video by default; pass `--no-record` to skip that.

For the two-solution mixing experiment the parameters can be entered in a spreadsheet
instead of a JSON flow:

```bash
cp src/flow/csv_runner/control.example.csv src/flow/csv_runner/control.csv
python -m src.flow.csv_runner.run_csv --mock     # then without --mock
```

Same validation, same safety frame, same `logs/<date>/…` folder; the dispensed masses
are also written back to `results.csv` next to the sheet. See [`csv-runner.md`](csv-runner.md).

## 7. GUI and the AI agent

```bash
streamlit run src/gui/app.py
```

The browser opens at `http://localhost:8501`. In the sidebar set the ports and
toggle *Mock* on for a dry run. Then either

- pick a preset and press *Run*, or
- describe the experiment in natural language (typed, or spoken with the microphone
  button), let the agent produce the JSON flow, review and edit it in the step
  editor, and press *Run*.

The ports shown in the sidebar are read from `config.yaml` (see §3); edits there apply
to the current browser session only. Pressing *Run* first validates the flow against
`src/flow/schema.py` — a flow that fails is refused and the offending steps are listed,
so nothing is sent to the instruments. The run then goes through the same
`ExperimentSession` as the CLI and produces the same `logs/<date>/<flow>_<timestamp>/`
folder (`run.log`, `measurements.csv`, `dispense_accuracy.csv`, `metadata.json`,
`summary.md`, `images/`), whose path is shown under the progress bar. *Stop* cancels the
run, emergency-stops every robot and disconnects them. In Real mode, if the monitoring
dashboard is running, sensor recording is started and stopped around the run and its
`/api/is_safe` check gates each step. For the statistics of a dispensing run:

```bash
python -m src.gui.analyze_dispense_log logs/<date>/<flow>_<timestamp>
```

## 8. Monitoring (optional)

```bash
pip install -r src/monitoring/requirements.txt
cp src/monitoring/config.example.yaml src/monitoring/config.yaml
python src/monitoring/dashboard/launch_sensor_dashboard.py
```

opens the dashboard at `http://localhost:8000`, receives sensor data from Raspberry Pi
agents over TCP and records CSV/video into `experiment_data/`. Installing the Pi agent
and the network settings are described in `src/monitoring/README.md`.

## 9. MaxArm (Hiwonder) robot (optional)

Robot 4 can be a Hiwonder MaxArm: an ESP32 running MicroPython, three bus
servos and a suction nozzle, for suction pick-and-place (no pipette). It
accepts only `move_xyz`, `move_z`, `go_home`, `suction` and `wait`; the
preflight rejects anything else on robot 4 before any device is opened.

**USB and the reset hazard.** The arm connects over a CH340 USB serial adapter
(115200 baud; Windows: *USB-SERIAL CH340* in Device Manager, Linux
`/dev/ttyUSB*`). Toggling DTR/RTS on that port can reset the ESP32, and the
factory program calls `go_home()` at every boot, so **a reset moves the arm**.
The saigen driver opens the port with DTR/RTS held low and refuses to continue
if it sees a boot banner. Do not open the port with other serial terminals or
tools that toggle DTR/RTS while the arm is in a fixture.

**Firmware.** The driver talks to a small command server (protocol
`saigen-maxarm`) that replaces the factory `main.py`; it never moves the arm at
boot and stops the servos when the PC goes silent (watchdog). Installing it is
a **human** step, with the arm clear of everything:

```bash
python -m src.devices.maxarm.deploy_firmware --port COM11 --list
python -m src.devices.maxarm.deploy_firmware --port COM11 --backup backups/maxarm   # keep this folder
python -m src.devices.maxarm.deploy_firmware --port COM11 --check                   # build compatible?
python -m src.devices.maxarm.deploy_firmware --port COM11 --backup backups/maxarm --install
# then power-cycle the arm with the reach clear (or pass --reset knowingly)
```

`--check` confirms on the board, without moving anything, that Hiwonder's
MicroPython build offers what the firmware needs (`--install` runs it first and
refuses on a failure). Set `home_position` for the MaxArm: the pose the arm
rests in after Hiwonder's factory homing (z ≈ 222 mm) is one its own inverse
kinematics cannot command, so without a configured home `go_home` is refused.
The MaxArm's validator includes that kinematics, and reports the highest
reachable z at the target's (x, y) when it rejects a move. To find the
coordinates of your vials and rests, jog the arm with
`python -m src.devices.maxarm.teach` (human-run; every command moves the arm,
`--mock` to practise) and `save NAME` each pose: they are written to
`maxarm_positions.json` (gitignored) for pasting into flows.
`--install` refuses unless a backup of the current `main.py` exists;
`--restore backups/maxarm` puts the factory files back. AI agents must not run
this tool (see `CLAUDE.md`, rule 1). Details, the wire protocol and the
restore-to-factory notes are in [`src/devices/maxarm/README.md`](../src/devices/maxarm/README.md);
the order of work that proved safe, the numbers measured on a real arm and the
recovery steps are in [`maxarm-commissioning.md`](maxarm-commissioning.md).

**Configuration.** Set the port in `config.yaml` (or `--robot4-maxarm COM11` /
`ROBOT4_MAXARM_PORT`):

```yaml
robots:
  4:
    type: maxarm
    maxarm_port: COM11
    workspace:            # MaxArm frame, replaces the global limits for robot 4 only
      x_min: -250.0
      x_max: 250.0
      y_min: -280.0
      y_max: -60.0
      z_min: 40.0
      z_max: 225.0
      r_min: 60.0         # horizontal radius hypot(x, y)
      r_max: 260.0
    move_ms: 1500         # duration of one move (100..30000 ms)
    # home_position: [0.0, -160.0, 180.0]   # optional, see "Home" below
```

**The workspace is required and self-contained.** A robot with `type: maxarm`
(at any robot id, not only 4) is checked only against its own
`robots.<id>.workspace`; nothing is inherited from the global Dobot
`workspace`, whose frame is different. `x_min`, `x_max`, `y_min`, `y_max`,
`z_min` and `z_max` must all be set; `r_min` / `r_max` are optional. If the
block is missing or incomplete, `--validate-only`, `--mock`, the GUI and a real
run all stop with a `ConfigError` naming the robot id, before any device is
opened. A key written without a value (`z_min:` in YAML, i.e. null) is an
error too, never "no limit": this holds for every workspace key, for the
MaxArm and for a Dobot's own `robots.<id>.workspace`.

**Home.** Without `home_position`, the pose the arm is in when the session
connects becomes its home (as for the Dobots). With `home_position` set, that
configured point is kept as the home and must lie inside the workspace; if it
does not, the session disconnects the arm and refuses to start. The session
never sends a MaxArm home by itself after an error; `go_home` runs only as a
flow step.

With `maxarm_port` empty, a real run that uses robot 4 refuses to start with a
message naming `--robot4-maxarm` / `ROBOT4_MAXARM_PORT`; flows that do not use
robot 4 are not affected. Other keys under `robots.4` (for example
`pose_settle_tolerance`, `move_tolerance_mm`, `watchdog_ms`) are passed to the
driver as options.

**Workspace calibration.** The example box is a conservative guess inside the
arm's reach, not a measured cell. The frame is the arm's own: millimetres,
origin on the base yaw axis, z = 0 at the base plane, the arm faces −y at home.
To measure your cell: with the firmware installed and the arm clear, read the
pose at each fixture (pick points, place points, the lowest safe height above
the table) from the driver's `status()` (or, for calibration only, with torque
off and the arm moved by hand); take the smallest box and radius range that
contains them plus a margin, and set `robots.4.workspace` to it. Then
`--validate-only` and `--mock` your flow before any real run. Only tighten the
limits; a move the validator rejects is the safety layer doing its job.

**Emergency stop and suction.** Ctrl+C (or Stop in the GUI) sends STOP: the
servos halt where they are and the firmware latches the stop. The session does
not send the MaxArm home after an error. When the run ends normally, the
driver turns the pump off before disconnecting (`pump_off_on_cleanup`, default
true). After an emergency stop, or when the last motion failed and the pose is
unknown, it **keeps the suction on** so a held part is not dropped from
wherever the arm stopped, and logs a warning: remove the part by hand and
switch the pump off manually. Set `pump_off_after_estop: true` under
`robots.<id>` if your lab prefers the pump to be switched off in that case too.
The MaxArm moves open loop and has no hardware emergency stop: put a physical
switch on its 12 V servo supply within reach.

**Stop latch at connect.** If the firmware still holds a stop latch from a
previous session, the driver clears it when it connects (as the Dobot driver
re-arms on connect) and logs this as a warning with the arm's status; it never
clears the latch while the arm reports that it is moving.

**Relative moves.** `move_z` is relative to the last *commanded* target, not to
the read-back pose, so small servo errors do not accumulate into the target.
The read-back must agree with the target within `move_tolerance_mm` (a finite
number greater than 0); tighten it if your task needs a closer match.

**Watchdog.** The firmware stops the servos when it hears nothing from the PC
for `watchdog_ms` (default 1000 ms), and the watchdog then disables itself
until the next connect or stop-clear; the driver re-arms it after every clear
and treats a reply reporting a disabled watchdog as a link fault (no further
motion until a new connect). A heartbeat keeps the watchdog fed, so
`watchdog_ms` must be at least 1000 × (2 × heartbeat period + reply timeout)
(1000 ms with the defaults of 0.25 s and 0.5 s); a shorter value is refused at
connect. Details are in [`src/devices/maxarm/README.md`](../src/devices/maxarm/README.md).

The example flow is [`examples/maxarm/pick_and_place.json`](../examples/maxarm/pick_and_place.json).

## Troubleshooting

- **`GEMINI_API_KEY が設定されていません`** — the key is missing from `.env`, or the
  process was started from another directory (`.env` is loaded from the current
  working directory).
- **Pipette does not connect** — the Picus 2 must be switched on and in remote mode;
  the wrapper retries three times. On Bluetooth, pass the MAC address instead of a
  COM port and set `picus2_connection_type="bluetooth"`.
- **`WorkspaceViolationError`** — the target is outside the configured box; adjust the
  flow, or the `workspace` section of `config.yaml`. The message names the axis, the
  target value and the limit it broke.
- **Camera index** — enumeration order differs between machines; try 0, 1, 2. On
  Windows, `python -m src.devices.microscope.microscope_controller` prints the
  DirectShow device names in index order, so the webcam and the microscope can be
  told apart (the 400-CAM106 shows up as "UVC Video Device", VID `EB1A`).
- **Microscope image is a single flat colour** — the lens cap is on, the tip is
  pressed against the sample, or the LED ring is off; the driver logs a warning
  in that case but still saves the frame.
- **Microscope image has a strong green cast** — after power-up the 400-CAM106
  applies no white-balance gains until the host writes the UVC white-balance
  control once. `MicroscopeController.connect()` does that (5000 K, through
  DirectShow on Windows), so use the saigen driver rather than a plain OpenCV
  capture; the vendor viewer's *Connect* button has the same effect.
- **Microscope LED does nothing** — `microscope_led` needs `microscope_port`: the
  400-CAM106 exposes a second device, a Silicon Labs CP210x serial port (VID
  `10C4`, S/N `0001`), on the same USB cable. Set that COM port in `config.yaml`.
  Do not run the vendor's UM Viewer during an experiment: on *Connect* it writes
  to every COM port on the PC, including the robots, pipettes and balance.
