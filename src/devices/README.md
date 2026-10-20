# `src/devices` — instrument drivers and safety wrapper

Two layers:

- **Drivers** (`dobot/`, `maxarm/`, `picus2/`, `ika/`, `scale/`, `webcam/`, `microscope/`, `keithley2002/`, `li5640/`, `dso1000/`): thin, device-specific
  classes that speak the instrument's protocol. They know nothing about flows or
  robots. (A few carry older stand-alone helpers, such as `run_temperature_experiment`
  in the IKA driver; the executor does not call them.)
- **Safety wrapper** (`safety/`): the only layer the flow executor and the AI agent
  ever call. It adds workspace limits, pipette volume tracking, retries, settle
  times and logging on top of the drivers.

The AI agent never generates driver-level code; it only chooses *what* to do, and the
wrapper decides *how*.

## Drivers, by what they do

Three kinds of entry in the tables: **flow** (a flow action and a `config.yaml` key
exist; opened only when a flow uses it), **library only** (a driver you can call
from your own script; no flow action) and **observation only**. Interface, library
and main class are as in the code; the *Used from* column names the flow actions,
the configuration keys, the safety-wrapper class and an example flow.

### Robot arms

| Directory | Instrument | Interface | Library | Main class | Used from |
|---|---|---|---|---|---|
| `dobot/` | Dobot Magician robot arm (+ optional rail / conveyor) | USB serial | `pydobot` (+ small patch in `pydobot_patch.py`) | `PyDobotController` | **flow**: `move_xyz`, `move_z`, `move_radial`, `rotate`, `rotate_relative`, `go_home`, `move_slider`, `move_conveyer`; `robots.<id>.dobot_port`; `LabRobot`; `examples/zif8/` |
| `maxarm/` | Hiwonder MaxArm robot arm with suction nozzle (optional robot 4) | USB serial (CH340), line protocol `saigen-maxarm` to the repository's MicroPython firmware | `pyserial` | `MaxArmClient` | **flow**: `move_xyz`, `move_z`, `go_home`, `suction`, `wait`; `robots.<id>.type: maxarm`, `maxarm_port`, own `workspace`; `MaxArmRobot`; `examples/maxarm/` |
| `mg400/` | Dobot MG400 | TCP/IP: Dashboard 29999 (read-only commands), feedback 30004 (read) | none (sockets) | `observe` tool | **observation only**: `python -m src.devices.mg400.observe <address>`; not a robot type, no flow can use it |

### Liquid handling

| Directory | Instrument | Interface | Library | Main class | Used from |
|---|---|---|---|---|---|
| `picus2/` | Sartorius Picus 2 electric pipette (10 mL) | USB serial or Bluetooth LE | `pyserial` / `bleak` | `Picus2Controller` | **flow**: `aspirate`, `dispense`, `blow_out`; `robots.<id>.picus2_address`; `LabRobot` (volume tracking); `examples/zif8/` |

### Heating and stirring

| Directory | Instrument | Interface | Library | Main class | Used from |
|---|---|---|---|---|---|
| `ika/` | IKA RET control-visc hot-plate stirrer | RS-232C | `pyserial` | `IKAController` | **library only**: no flow action and no `config.yaml` key; `LabRobot(use_ika=True, ika_port=...)` opens it and switches heater and stirrer off on `emergency_stop()` / `cleanup()` |

### Measurement instruments

| Directory | Instrument | Interface | Library | Main class | Used from |
|---|---|---|---|---|---|
| `scale/` | Sartorius BCE822i (`BCE8221.py`: the paper's cell, and the only balance the flows use) and A&D EK-610i (`scale_controller.py`; `Ek610i.py` is an older minimal driver, not used by anything here) | RS-232C | `pyserial` | `SerialBalance`, `ScaleController` | **flow** (Sartorius): `measure_weight`, `tare_scale`; `shared_devices.scale_port`; `SharedDevices`; `examples/zif8/`. A&D: library only |
| `keithley2002/` | Keithley 2002 8.5-digit digital multimeter (optional) | GPIB (IEEE-488) through a VISA library, SCPI | `pyvisa` | `Keithley2002` | **flow**: `measure_dmm`; `shared_devices.dmm_resource`, `dmm_timeout_ms`, `wait_after_dmm`; `SharedDevices`; `examples/keithley2002/` |
| `li5640/` | NF LI5640 digital lock-in amplifier (optional, read-only: settings stay on the front panel; exception, off by default: the internal oscillator, `set_lockin_oscillator`) | GPIB (IEEE-488) through a VISA library, NF command set | `pyvisa` | `LI5640` | **flow**: `measure_lockin`, `set_lockin_oscillator`; `shared_devices.lockin_resource`, `lockin_timeout_ms`, `wait_after_lockin`, `lockin_osc_*`; `SharedDevices`; `examples/li5640/` |
| `dso1000/` | Agilent 1000 series digital oscilloscope (DSO1002A-DSO1024A; optional, read-only automatic measurements) | USB (USBTMC) through a VISA library, SCPI-like command set | `pyvisa` | `DSO1000` | **flow**: `measure_scope`; `shared_devices.scope_resource`, `scope_timeout_ms`, `wait_after_scope`; `SharedDevices`; `examples/dso1000/` |

### Cameras and microscopy

| Directory | Instrument | Interface | Library | Main class | Used from |
|---|---|---|---|---|---|
| `webcam/` | USB webcam for process photos | UVC | `opencv-python` | `WebcamController` | **flow**: `capture_and_save`; `shared_devices.camera_index`; `SharedDevices` |
| `microscope/` | USB digital microscope for close-up photos (Sanwa Supply 400-CAM106 = Vitiny UM22 in the paper's cell) | UVC (image) + USB serial via CP210x (LED, focus motor, status) | `opencv-python`, `pyserial` | `MicroscopeController` (subclass of `WebcamController`), `UM22SerialController` | **flow**: `capture_microscope`, `microscope_led`, `microscope_focus`; `shared_devices.microscope_index`, `microscope_port`, `microscope_resolution`; `SharedDevices`; `examples/microscope/` |

Each driver directory has its own README with the command set and a usage example;
the safety wrapper is described below. Device addresses (COM port, Bluetooth MAC) are
passed in by the caller: the defaults built into the drivers (`COM3`, `COM4`, ...) are
placeholders, and the ports actually used come from `config.yaml` and `.env`.

## Safety wrapper (`safety/`)

| File | Class | Role |
|---|---|---|
| `lab_robot.py` | `LabRobot` | One robot = one arm + optionally one pipette (+ stirrer). Async API used by the executor: `move_xyz`, `move_z`, `move_radial`, `rotate`, `rotate_relative`, `go_home`, `aspirate`, `dispense`, `blow_out`, `move_slider`, `move_conveyer`. Tracks the current pose and the volume held in the tip. |
| `maxarm_robot.py` | `MaxArmRobot` | Robot of `type: maxarm` (robot 4 in `config.example.yaml`): `move_xyz`, `move_z`, `go_home`, `suction`, `wait` only (`SUPPORTED_ACTIONS`; the executor and the preflight refuse anything else). Validates every target against the robot's own workspace (`robots.<id>.workspace`, with optional radius limits `r_min` / `r_max`) before the command is sent; cancellation or `emergency_stop()` sends STOP and marks the pose unknown. No automatic go-home after an error; `cleanup()` switches the pump off and never moves the arm. `MockMaxArmRobot` (`mock_maxarm.py`) is its no-hardware counterpart. The runners pick the class from the robot's `type` |
| `shared_devices.py` | `SharedDevices` | Balance, camera, microscope, DMM, lock-in amplifier and oscilloscope, shared by all robots: `tare_scale`, `measure_weight`, `capture_and_save`, `capture_microscope`, `set_microscope_led`, `focus_microscope`, `measure_dmm`, `measure_lockin`, `set_lockin_oscillator`, `measure_scope`. VISA I/O (GPIB, USB) runs on a worker thread under a lock per instrument; a cancelled step returns at once and `cleanup()` waits for the I/O on the bus before it releases the instrument (the LI5640: clear, the oscillator restored if a write was attempted, `OTYP` restored, GTL, close). A second cancellation during `cleanup()` does not skip the other instrument. |
| `mock_robot.py` | `MockLabRobot`, `MockSharedDevices` | Same interface as `LabRobot` / `SharedDevices` without hardware; used by `--mock` and the GUI's Mock mode. The mock enforces the same workspace limits and the same pipette-volume limits (same exception types), keeps a coherent simulated pose (rotations move X/Y, XYZ moves update joint 1, radial moves follow the current radius) and returns realistic weights, so mock runs produce the same records as real ones. Its start pose is `start_pose=(x, y, z, r)`; the runners take it from `shared_devices.mock_start_pose` in `config.yaml` or `MOCK_START_POSE` (see `docs/setup.md`, section 5). |
| `validators/` | `WorkspaceValidator` | XYZ box and joint-1 angle limits (and an optional horizontal radius range), checked before every move. The limits come from the `workspace` section of `config.yaml` in the repository root (`default_workspace_validator()`); a robot with its own `robots.<id>.workspace` block (the MaxArm) gets that block merged over it (`workspace_validator_from_config`). Edit that file to match your fixtures. |

Construction:

```python
from src.devices.safety.lab_robot import LabRobot
from src.devices.safety.shared_devices import SharedDevices

async with LabRobot(use_dobot=True, use_picus2=True,
                    dobot_port="COM12", picus2_address="COM4",
                    picus2_connection_type="usb") as robot:
    robot.set_current_position_as_home()
    await robot.move_z(-50)
    await robot.aspirate(volume=5.0, speed=5)
    await robot.move_z(50)
    await robot.go_home()

async with SharedDevices(use_scale=True, scale_port="COM9") as shared:
    await shared.tare_scale()
    grams = await shared.measure_weight(stabilization_count=3)
```

Safety behaviour built into the wrapper:

- `aspirate` refuses to exceed 10 mL in the tip; `dispense` refuses to exceed the
  held volume.
- Every target pose is validated against the workspace limits from `config.yaml`;
  a violation raises `WorkspaceViolationError` before the command is sent.
- `initialize()` raises if any device fails to come up (and disconnects whatever did
  connect) rather than returning a status that a caller could ignore and then drive a
  disconnected arm.
- Every blocking driver call (pydobot moves with `wait=True`, the conveyor's timed
  run) runs in a worker thread, so the event loop stays responsive. If the awaiting
  task is cancelled (Ctrl+C, GUI Stop) while a move is in flight, the wrapper sends
  `force_stop()` at once, waits (at most `stop_wait_timeout`, default 10 s) for the
  driver call to return, and re-raises; the first Ctrl+C halts the arm mid-move.
- After a cancelled or failed move, or an emergency stop, the pose is treated as
  unknown: before the next move the wrapper re-reads it from the arm and requires
  it to be stable (`pose_settle_*`), otherwise the move is refused. A move that was
  interrupted by `emergency_stop()` from another thread raises instead of letting
  the flow continue, and no automatic go-home is attempted after an emergency stop.
- After a cancelled or failed `aspirate` / `dispense` / `blow_out` the held volume
  is unknown (`pipette_volume_known` is False; `pipette_volume` keeps the last
  confirmed value): further `aspirate` / `dispense` calls raise `RuntimeError` until
  `blow_out()` succeeds or `reset_pipette_volume()` is called.
- `emergency_stop()` force-stops the Dobot command queue (the move in progress halts
  immediately, the queue is discarded, the conveyor motors stop) and switches the IKA
  heater/stirrer off. `run_flow.py` calls it on Ctrl+C, and commands no further
  motion afterwards. The Picus 2 driver has no stop command: a stroke in progress
  finishes on the pipette, and the held volume is marked unknown.
- Configurable settle times after moves, pipette actions and balance readings
  (`wait_after_*` keyword arguments).
- Pipette connection is retried five times with back-off.
- `cleanup()` (also on `async with` exit or on error) switches the IKA heater and
  stirrer off (best effort) and then disconnects every device.

## Not included

The vendor DLL-based Dobot driver and the powder-dispenser driver used in other
experiments are not part of this repository; only the `pydobot`-based arm driver is
shipped. `LabRobot(use_powder_dispenser=True)` therefore fails in `initialize()`: the
missing module is logged and `RuntimeError` is raised, after the devices that did
connect have been disconnected.
