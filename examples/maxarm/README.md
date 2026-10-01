# Hiwonder MaxArm pick-and-place example

`pick_and_place.json` drives the optional **robot 4**, a Hiwonder MaxArm with
its suction nozzle: it goes home, moves above a pick point, lowers, switches
suction on, lifts, moves above a place point, lowers, releases, lifts and goes
home again. Every step uses `robot_id: 4`.

```bash
python -m src.flow.run_flow examples/maxarm/pick_and_place.json --validate-only
python -m src.flow.run_flow examples/maxarm/pick_and_place.json --mock
```

## What the flow assumes

- `config.yaml` (or `config.example.yaml`) has a robot 4 entry with
  `type: maxarm`. A robot without `type` is a Dobot Magician. Robot 4's
  `workspace` block replaces the global (Dobot) limits for robot 4 only, so its
  `move_xyz` targets are checked against the MaxArm box and robots 1-3 keep the
  global one.
- Coordinates are in the **MaxArm's own frame**: millimetres, origin on the
  base yaw axis, z = 0 at the base plane, and the arm faces -y at home. They
  are not Dobot coordinates.
- The pick point (120, -180) and the place point (-120, -180) are at z 70 mm
  after the 80 mm descent. They lie inside the example box
  (x -250..250, y -280..-60, z 40..225, horizontal radius 60..260 mm), which is a
  conservative starting box, **not a measured cell**. Measure where your objects
  really are and tighten the workspace before a real run.
- The home position is wherever the arm is when the run starts (like the
  Dobots). The mock starts at the MaxArm origin (0, -162.8, 212).
- The MaxArm supports only `move_xyz`, `move_z`, `go_home`, `suction` and
  `wait`. The preflight rejects anything else on robot 4 (for example `aspirate`
  or `rotate_relative`) before any device is opened, and it rejects `suction` on
  a Dobot.
- After `suction` with `"on": false` the firmware opens the release valve for
  about one second, hence the one-second wait before lifting.

## Before a real run

A real run needs the saigen-maxarm firmware on the arm and `maxarm_port` set
(in `config.yaml`, with `--robot4-maxarm COM11`, or `ROBOT4_MAXARM_PORT` in
`.env`). The firmware installation, the reset warning (opening the USB port can
reset the ESP32, and the factory program moves the arm at boot) and the
workspace calibration are described in `docs/setup.md`, section "MaxArm
(Hiwonder) robot", and in `src/devices/maxarm/README.md`. The firmware
installer `src/devices/maxarm/deploy_firmware.py` is run by a human, never by an
agent.

The MaxArm moves open loop and has no hardware emergency stop; a physical switch
on its 12 V servo supply is recommended. After an error the session stops the
MaxArm where it is and does not send it home automatically.
