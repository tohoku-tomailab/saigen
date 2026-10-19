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
git clone https://github.com/tohoku-tomailab/saigen.git
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

On Windows, keep the virtual environment on a short path (e.g. `C:\saigen\venv`).
Without long-path support enabled, `pip install streamlit` (version not
recorded; October 2026) into a venv several directories deep failed with
`WinError 206` on a file deep inside the package and left a package that could
not be imported.

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
| `shared_devices` | `scale_port`, `camera_index`, `microscope_index`, `microscope_port` and `microscope_resolution` of the balance, the photo camera and the digital microscope (camera index for photos, serial port for the LED and focus, capture size; 4K by default, lower it when the image volume gets heavy); `dmm_resource` and `dmm_timeout_ms` of the optional Keithley 2002 DMM (§10); `lockin_resource` and `lockin_timeout_ms` of the optional NF LI5640 lock-in amplifier (§11); `scope_resource` and `scope_timeout_ms` of the optional Agilent 1000 series oscilloscope (§12) |
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
| `SCALE_PORT`, `CAMERA_INDEX`, `MICROSCOPE_INDEX`, `MICROSCOPE_PORT`, `DMM_RESOURCE`, `LOCKIN_RESOURCE`, `SCOPE_RESOURCE` | optional per-run override of the shared devices |
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

## 10. Keithley 2002 digital multimeter (optional)

The Keithley 2002 speaks only GPIB (IEEE-488). SAIGEN talks to it through
PyVISA, which needs a VISA library from the maker of the GPIB interface.

1. Install the driver of the GPIB interface. For National Instruments
   adapters, **NI-488.2 with NI-VISA** (select NI-VISA in the installer if it
   is offered separately); this is the configuration tested on hardware (NI
   GPIB-USB-HS+, see the commissioning notes). Keysight adapters use the
   **Keysight IO Libraries Suite**, which provides a VISA library as well, but
   that combination has not been tried. (PyVISA-py can also reach GPIB through
   `gpib-ctypes` and the vendor's GPIB driver; that path is not covered here.)
2. `pip install pyvisa` (listed in `requirements.txt`; the mock and the tests do
   not need it).
3. Find the address: on the 2002, *MENU > GPIB* shows it (16 from the factory,
   but do not assume it). On the PC, NI MAX lists the instrument, or list the
   GPIB resources only (the default query also returns the serial ports, and
   some devices on this cell react when a serial port is opened):

   ```bash
   python -c "import pyvisa; print(pyvisa.ResourceManager().list_resources('GPIB?*::INSTR'))"
   ```

4. Put the resource string in `config.yaml`:

   ```yaml
   shared_devices:
     dmm_resource: "GPIB0::16::INSTR"
     dmm_timeout_ms: 10000
     wait_after_dmm: 0.0      # optional pause after each measure_dmm step (s)
   ```

5. Check the link. This talks to the instrument (`*IDN?`, line sync, one
   DC-voltage reading, then back to front-panel control) and moves nothing.
   Steps 3 (the listing), 5 and the real run in step 6 are the human's. An
   agent runs them only when the human explicitly asks for it in the current
   conversation, the real run only after the `--mock` run succeeded, and it is
   the human who starts the real run (presses Enter) (CLAUDE.md rule 1):

   ```bash
   python -m src.devices.keithley2002.keithley2002 GPIB0::16::INSTR
   ```

6. `--validate-only` and `--mock` the example
   [`examples/keithley2002/measure_dc_voltage.json`](../examples/keithley2002/measure_dc_voltage.json),
   then run it for real.

Every `measure_dmm` step sends `:CONFigure:<function>` first, which puts the
trigger model in single-shot mode (all three layers immediate, count 1, delay
0), disables math, the buffer and scanning, sets the acquisition method to
normal and returns the autozero controls and that function's own settings
(filter, REL, digits) to their `*RST` values (2002 manual p. 3-42); then the
range, NPLC and output format. It does not send `*RST`; settings outside that
list, such as the GPIB address, the display and line synchronization, stay as
they are. The line-synchronization state is written to `run.log` when the DMM
is opened (a snapshot, not a per-measurement record). The range and NPLC are read back after the last sample and
logged (on autorange, the range the last sample used). At the end of the run
the instrument is returned to front-panel control (GTL).

Ctrl+C during a DMM step stops the step at once (robots in the flow are
emergency-stopped without waiting for the instrument). The I/O already on the
bus is left to finish (each VISA read and write has the `dmm_timeout_ms`
timeout), no further SCPI command is sent, and then the instrument is returned
to front-panel control (GTL) and closed. A failed GTL is logged as a warning and
the close goes ahead; a failed close is logged as an error and the handle is
kept, so the next clean-up can try again.

A failing DMM step is an ordinary step error: the run stops and, as for any
other error, Dobot arms in the same flow are sent home. Current ranges go
through the instrument's fuse; check the lead configuration before measuring
current.

What commissioning the first unit taught (order of work, what the firmware
accepted, an unexplained line-sync observation, things not to do) is in
[`keithley2002-commissioning.md`](keithley2002-commissioning.md).

## 11. NF LI5640 lock-in amplifier (optional, read-only)

SAIGEN reads the LI5640 over GPIB; it does not set it up. Sensitivity, time
constant, slope, reference, input, filters, dynamic reserve and the DATA1 /
DATA2 display (with offset, expand, normalize and ratio) are set on the front
panel. The only setting SAIGEN writes is the read-out selection `OTYP`, which it
reads when it opens the instrument and writes back, and reads back, when it
closes it. The one exception, off by default, is the internal oscillator
(`set_lockin_oscillator`, at the end of this section). The driver README
([`src/devices/li5640/README.md`](../src/devices/li5640/README.md)) lists every
command.

The GPIB interface driver and PyVISA are the same as in §10. On the instrument
and the PC:

1. On the LI5640, *SHIFT + SPECIAL* (`intFc`) must show `GPib`: the LI5640
   uses either GPIB or RS-232, not both (manual 5-3).
2. Read the address with *SHIFT + ADDRESS* (0 to 30, manual 5-4) and make it
   different from every other instrument on the same GPIB bus; check each
   instrument's own panel. Two instruments at one address corrupt the bus. The
   transmit terminator can stay as it is.
3. Continuous data output must be off. While it runs the LI5640 answers no
   other query (manual 5-68, error -440); SAIGEN checks `OSTR?` when it opens
   the instrument and refuses to go on if it is running, without stopping it.
4. List the GPIB resources only (as in §10 step 3) and put the resource in
   `config.yaml`:

   ```yaml
   shared_devices:
     lockin_resource: "GPIB0::<address>::INSTR"
     lockin_timeout_ms: 5000
     wait_after_lockin: 0.0   # optional pause AFTER each measure_lockin step (s)
   ```

5. Check the link. This talks to the instrument (`*IDN?`, the error queue,
   `OSTR?`, `OTYP?`, the panel settings, one `DOUT?`, then `OTYP` restored and
   front-panel control) and moves nothing. As in §10, the listing, this check
   and the real run are the human's (CLAUDE.md rule 1):

   ```bash
   python -m src.devices.li5640.li5640 GPIB0::<address>::INSTR
   ```

6. `--validate-only` and `--mock` the example
   [`examples/li5640/read_lockin.json`](../examples/li5640/read_lockin.json),
   then run it for real; for the first check add `--no-record` (no sensor CSV
   or video recording; the run folder is still written).

Side effects on the instrument besides `OTYP`: a device clear addressed to it
(clears its input and output buffers; the setup does not change, manual 5-78),
`*CLS` (clears its event registers and error queue; not restored) and the
REMOTE state until the end of the run (the front panel is locked; SAIGEN sends
GTL to this instrument only, never REN off, IFC or DCL to the bus).

Each `measure_lockin` step reads the panel settings before and after its
readings and fails if they differ (a change made and undone within the step is
not detected), fails while an automatic setup is running, discards readings
taken with an over-level or UNLOCK, and fails if none is left. The settings
decide what DATA1 and DATA2 mean (X, R, θ, noise, AUX; normalize and ratio) and
their unit. **SAIGEN does not wait for the output to settle**: after a change on
the panel, wait before the step (manual Table 4-3: at 24 dB/oct about 10 time
constants to 99 %; noise density about 170 time constants times the smoothing
factor). `interval` only spaces the readings, and `wait_after_lockin` is a
pause after the step.

Ctrl+C during a lock-in step behaves as in §10: the robots are stopped first,
the I/O on the bus is left to finish, nothing more is sent, and then the
instrument is cleared, `OTYP` restored, sent to local and closed; each of these
is tried even if the one before failed. If a second Ctrl+C arrives while
SAIGEN waits for one GPIB instrument, it still releases the other before it
stops.

**Setting the internal oscillator (optional, off by default).** A flow step
`set_lockin_oscillator` can set the oscillator frequency and the OSC OUT
amplitude (Vrms). It is refused before anything opens unless `config.yaml`
allows it:

```yaml
shared_devices:
  lockin_osc_max_vrms: 0.3     # 0 = disabled (default)
  lockin_osc_min_hz: 10.0
  lockin_osc_max_hz: 1000.0
```

Choose the limits for what is wired to OSC OUT. They limit the **set values**
only: not the actual output, not how long it stays on, not the load. After a
failure the run may home robots or close other instruments before it restores
the oscillator, so keep a sample on OSC OUT only if it tolerates the maximum for
an unlimited time. SAIGEN never switches the amplitude range: set it on the
panel (50 mV, 500 mV or 5 V) so that it holds every amplitude of the flow. Each
command is read back before the next is sent. When the run ends, fails or is
stopped with Ctrl+C, SAIGEN reads the oscillator and writes back the values it
found when it opened the instrument (Ctrl+C does **not** switch OSC OUT off, and
the restore may raise the amplitude to its earlier value). The result is in
`metadata.json` (`lockin_oscillator_restore`); if it could not restore or check
it, a completed run is marked failed (exit code 1) and a run stopped with Ctrl+C
stays aborted (exit code 130); both get a note: check OSC OUT on the front
panel. Details and every rule are in the driver README. The example
[`examples/li5640/oscillator/osc_sweep_scope.json`](../examples/li5640/oscillator/osc_sweep_scope.json)
sweeps the amplitude and frequency and reads OSC OUT on the oscilloscope of §12.

What commissioning the first unit taught (what firmware 1.60 answered, the
defects review found before the bench, open points, things not to do) is in
[`li5640-commissioning.md`](li5640-commissioning.md).

## 12. Agilent 1000 series oscilloscope (optional, read-only)

SAIGEN reads the automatic measurements (Vpp, mean, RMS, frequency, rise time,
duty cycle ...) of an Agilent DSO1002A / 1004A / 1012A / 1014A / 1022A / 1024A
over USB. It sends queries only, apart from a device clear when it opens and
closes the instrument and `*CLS` when it opens it (`*CLS` also clears errors left in the instrument's queue
by front-panel use): vertical scale, time base, trigger and acquisition are set
on the front panel. The driver README
([`src/devices/dso1000/README.md`](../src/devices/dso1000/README.md)) lists
every command.

1. Connect the rear **USB DEVICE** port (the square connector) to the PC. The
   VISA library of §10 (NI-VISA) also drives USB instruments (USBTMC); no
   GPIB adapter is involved.
2. List USB resources only and put the one of the oscilloscope
   (`USB0::0x0957::...::INSTR`, vendor 0x0957 = Agilent) in `config.yaml`. It
   contains the instrument's serial number, so it stays in `config.yaml`:

   ```bash
   python -c "import pyvisa; print(pyvisa.ResourceManager().list_resources('USB?*::INSTR'))"
   ```

   ```yaml
   shared_devices:
     scope_resource: "USB0::0x0957::<product id>::<serial>::INSTR"
     scope_timeout_ms: 5000
     wait_after_scope: 0.0    # optional pause AFTER each measure_scope step (s)
   ```

3. On the front panel: show the channel to be measured, use the **Y-T** (or
   ROLL) time base with the main (not zoomed) window and an **edge** trigger.
   Other states make the step fail rather than record values whose meaning
   SAIGEN cannot check.
4. Check the link (the human's step, CLAUDE.md rule 1; queries only, after a
   device clear and `*CLS`):

   ```bash
   python -m src.devices.dso1000.dso1000 USB0::0x0957::<product id>::<serial>::INSTR 1
   ```

5. `--validate-only` and `--mock` the example
   [`examples/dso1000/read_scope.json`](../examples/dso1000/read_scope.json),
   then run it for real; for the first check add `--no-record`.

Each `measure_scope` step reads the settings before and after its reads and
fails if they differ (a change made and undone within the step is not
detected), if any requested item has no valid reading, or if the instrument
reports an error. `samples` is the number of reads, not of acquisitions: the
oscilloscope does not acquire while it answers, and nothing guarantees a new
acquisition within `interval`. Ctrl+C behaves as in §10 and §11: robots first,
the query on the bus is left to finish, nothing more is sent, then device clear
and close. No GTL is sent: over USB it hung for 120 s and then failed on the
DSO1014A that was tested. Between query sessions the front panel stayed usable,
but it was once found locked after the session in which GTL hung (cause not
determined; it was released from the front panel). On the bench all 18 items
were read on a signal (duty cycle came back without the `%` the guide shows;
both forms are accepted); accuracy against a known reference was not checked.

What commissioning the first unit taught (what firmware 00.04.04 answered, the
defects found on the bench and by review, open points, things not to do) is in
[`dso1000-commissioning.md`](dso1000-commissioning.md).

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
- **`Keithley 2002 初期化エラー`** — `dmm_resource` is empty or wrong, the VISA
  library is not installed (`pyvisa` then cannot find a backend), or another
  program (NI MAX's interactive window, KickStart) holds the instrument. The
  check in §10 step 5 shows which. A `*IDN?` that is not a 2002 is refused.
- **`機器がエラーを報告しました: -113,"Undefined header"`** — the instrument
  rejected a command; the message lists the errors from its queue. Old firmware
  may lack `:FORMat:EXPonent`; report the firmware revision (the last field of
  `*IDN?`).
- **DMM step times out** — one reading takes longer than `dmm_timeout_ms` (high
  NPLC with autorange, AC functions); raise it in `config.yaml`.
- **`LI5640 初期化エラー`** — `lockin_resource` is empty or wrong, the interface
  is set to RS-232 on the panel, two instruments share the address, continuous
  data output is running (`連続データ出力（OSTR 1）が動いています`), or the
  `*IDN?` is not an LI5640. The check in §11 step 5 shows which.
- **`LI5640 の設定が測定中に変わりました`** — a panel setting differed before and
  after the readings (someone used the panel, or an automatic function ran).
  The settings before and after and the difference are in the `diagnostics`
  column of `measurements.csv` and under `step_diagnostics` in `metadata.json`.
- **`LI5640 から有効なサンプルを取得できませんでした`** — every reading had an
  over-level or UNLOCK; the reasons are counted in the message and in
  `diagnostics`. Check the sensitivity, the reference and the lamps on the panel.
- **`オシロスコープ初期化エラー`** — `scope_resource` is empty or wrong, the cable
  is in the front USB host port instead of the rear USB DEVICE port, or the
  model is not one of the six of the series (`対応していない機種です`).
- **`有効な値を取得できなかった項目があります`** — the oscilloscope could not
  make that measurement on the displayed waveform (for example a rise time
  faster than it can resolve comes back as `<value`, a frequency needs a full
  cycle on screen). The reasons per item are in `diagnostics`; adjust the time
  base or the vertical scale on the panel.
- **Microscope LED does nothing** — `microscope_led` needs `microscope_port`: the
  400-CAM106 exposes a second device, a Silicon Labs CP210x serial port (VID
  `10C4`, S/N `0001`), on the same USB cable. Set that COM port in `config.yaml`.
  Do not run the vendor's UM Viewer during an experiment: on *Connect* it writes
  to every COM port on the PC, including the robots, pipettes and balance.
