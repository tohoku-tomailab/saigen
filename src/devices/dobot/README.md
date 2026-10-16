# Dobot robot-arm control module

Driver for the Dobot Magician through the `pydobot` library (serial communication). Each
instance owns its own serial connection, so several Dobots can be controlled at the same
time.

## Files

- `home_dobot.py` — homing CLI (`python -m src.devices.dobot.home_dobot --robot N`)

| File | Role |
|---|---|
| `pydobot_controller.py` | `PyDobotController` — the driver itself |
| `pydobot_patch.py` | Patch replacing pydobot's communication layer (reply-frame validation, timeout on completion waits, abort on emergency stop) |
| `dobot_config.py` | Speed presets and working positions (`DobotConfig`) |

## Usage

```python
from src.devices.dobot import PyDobotController

# Connect by port name (Windows: 'COM12', Linux/macOS: '/dev/ttyUSB0', etc.)
robot = PyDobotController(port_name='COM12', homing=False)

robot.move_XYZ_abs(200, 0, 100)   # move to absolute coordinates
robot.move_Z(-50)                 # relative Z move
robot.move_XY(20, 30)             # relative XY move
robot.move_angle(45)              # base (joint 1) angle
print(robot.get_current_position())

robot.disconnect()
```

List the available serial ports:

```python
PyDobotController.list_available_ports()
```

## Main methods

| Method | Description |
|---|---|
| `move_XYZ_abs(x, y, z, r=None)` | Move to absolute coordinates |
| `move_Z(z_offset)` / `move_XY(dx, dy)` | Relative moves |
| `move_angle(angle)` | Absolute joint-1 angle |
| `move_to_initial_pos()` | Go to the initial position |
| `pickup(...)` / `place(...)` | Pick and place |
| `set_gripper(enabled, on)` / `set_suction_cup(enabled, on)` | End effectors |
| `set_speed_preset(name)` | Switch to a speed in `DobotConfig.SPEED_PRESETS` (both Cartesian and joint) |
| `set_home_params(x, y, z, r)` | Record the home position |
| `home(x, y, z, r, timeout_s)` | Firmware homing (SetHOMEParams / SetHOMECmd). Waits for completion; returns to the start position if no destination is given |
| `move_slider(pos)` | Linear rail (0–1000 mm) |
| `move_conveyer(index, speed, time_seconds)` | Conveyor belt |
| `get_current_position()` | `[x, y, z, r, joint1, joint2, joint3, joint4]` |
| `force_stop()` | Emergency stop (force-stop and clear the queue, stop the conveyor). Also releases any thread waiting for completion |
| `disconnect()` | Disconnect |

## Notes

- Before automatic operation, manually check the range of motion and confirm that the
  surrounding area is clear.
- With `homing=True` the arm moves during the homing motion. Make sure there are no
  obstacles. Normally this is done one arm at a time with the CLI after power-on:
  `python -m src.devices.dobot.home_dobot --robot 1` (`--mock` only walks through the
  procedure; on the real arm nothing moves until you press Enter). The return position is
  validated in advance against the workspace in config.yaml.
- The experiment safety wrapper (`src/devices/safety/lab_robot.py`) uses this class
  internally and adds range-of-motion checks and wait times. Flows call the arm through the
  wrapper.
- If a motor makes an unusual noise, call `force_stop()` immediately (`disconnect()` only
  closes the connection and does not stop the arm). Confirm the arm has stopped before
  disconnecting and checking for physical interference.
- Communication handling (`pydobot_patch.py`): a reply is used only after its header
  `AA AA`, length, checksum and command ID have been verified, and the receive buffer is
  discarded before sending. If no matching reply arrives within 2 s, `DobotReplyError` (a
  subclass of `ConnectionError`) is raised. Waiting for a queued move to complete times out
  after 60 s by default with `DobotWaitTimeout` (change with
  `PyDobotController(move_timeout_s=...)`; the arm is not stopped, so the caller must call
  `force_stop()`). After `force_stop()`, a move being waited on returns immediately with
  `DobotMoveAborted`. The stop flag is cleared by the next move command.

## Dependencies

- `pydobot` (Copyright 2017 Luis Mesas, MIT License. `pydobot_patch.py` is derived code that replaces this library's `_send_command` / `_send_message` / `_read_message` / `_get_queued_cmd_current_index` / `__init__`. The full text is in `THIRD_PARTY_NOTICES.md`)
- `pyserial`

The older driver based on Dobot's own DLL (DobotDll) is not included in this repository
for licensing reasons.
