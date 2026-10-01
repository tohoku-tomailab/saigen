# Commissioning a Hiwonder MaxArm: what the bench taught us

Field notes from bringing a MaxArm into the SAIGEN cell (September 2026). They
complement the reference material in [`src/devices/maxarm/README.md`](../src/devices/maxarm/README.md)
(protocol, driver, firmware) and the procedure in [`setup.md` §9](setup.md#9-maxarm-hiwonder-robot-optional).
Everything here was observed on one arm (MicroPython 1.12 build, 2020-10-20);
another unit may differ in details, so treat the numbers as expectations to
re-check, not as specifications.

## 1. Order of work that worked

1. **Identify the port.** The controller shows up as a CH340 USB-serial device
   (VID 1A86, PID 7523). Close any other program that opens serial ports first;
   a microscope viewer on this PC silently held the MaxArm's port and every
   open failed with "PermissionError" until it was closed (found with
   Sysinternals `handle64`).
2. **Get to the REPL without a move.** The factory `main.py` sleeps 10 s, then
   takes the USB UART over at 9600 bps and homes the arm. Ctrl-C during the
   10 s window reaches the REPL and nothing moves:
   `deploy_firmware --catch-boot --rts-reset --list`. An RTS pulse (DTR low,
   RTS high 0.2 s, low) resets this board; the arm's 12 V switch does **not**,
   because the ESP32 is USB-powered.
3. **Back up** (`--backup DIR`), keep the directory (gitignored `backups/`).
4. **`--check`**, then **`--install`** (which re-runs the checks). On this build
   two checks mattered: `Class.__new__` does not exist (only `object.__new__`)
   and `Exception.__init__` does not exist (`super().__init__` works). The
   self-test that runs the command server on the device caught the second one
   only after it had produced an `ERR INTERNAL` on the bench; it is now part of
   `--check`, so run `--check` before every install, including after firmware
   edits.
5. **RTS reset, expect `EVT READY`** within about 3 s. A plain open of the
   port (RTS/DTR low) does not reset the board.
6. **Read before moving.** `STATUS` gives the pose, the pulses and the servo
   supply voltage. **Do not move while `vin` is below ~6 V**: with the 12 V
   switch off the servos still answer reads (USB leaks ~3.8 V onto the bus)
   but cannot move. 12.3 V is normal.
7. **First move: small, downwards, through the safety layer.** The pose the
   arm rests in after the factory homing (read back as z ≈ 222 mm) is one the
   arm's own inverse kinematics refuses (see §3), so "10 mm up" fails and
   "10 mm down" may fail too: go to a height the kinematics accept
   (`kinematics.highest_reachable_z` or the `top` command of `teach.py`).
8. **Set `home_position`** in `config.yaml` to a reachable pose before running
   flows; without it the resting pose becomes home and `go_home` is refused.
9. **Teach positions** with `python -m src.devices.maxarm.teach` (jog, `save`),
   write flows, `--validate-only`, `--mock`, real run.

## 2. Numbers measured on this arm

| Quantity | Observed |
|---|---|
| Boot to `EVT READY` after an RTS reset | 3.1–3.2 s |
| `MaxArmClient.connect` handshake (PING, INFO, LIMITS, WATCHDOG, INFO) | 0.64–0.67 s |
| `STATUS` round trip (three servo position reads + one voltage read) | 125–160 ms |
| Read-back noise of a resting pose | ±1–2 mm per axis, occasionally 3 mm |
| Read-back error after a move (1.5 s move, 0.3 s wait, settle check) | 0–3 mm per axis; y read 5 mm off once after a 70° base rotation |
| `move_xyz` wall time as seen by the flow (1500 ms move + settle + 0.3 s) | 2.0–2.2 s |
| Base rotation of 70–90° at r = 185 mm, z = 199 mm, in one 1.5 s move | fine, no visible overshoot |
| Sideways 20 mm, down/up 10 mm | fine |
| Pump on/off, `ERR BUSY` for a re-on within 1.1 s of off, accepted after 1.5 s | as designed |
| Servo supply during pump operation | stays at 12.3 V |
| Free heap on MicroPython 1.12 before / after importing the firmware | ~90 kB / ~79 kB |

## 3. The reachable envelope is not the box

The arm solves its inverse kinematics in a compiled module. The link lengths
that reproduce it to four decimals are L0 = 84.4, L1 = 8.14, L2 = 128.4,
L3 = 138.0, L4 = 16.8 mm (not the values printed in Hiwonder's `espmax.py`,
which the compiled module does not use). It refuses a target when any joint
angle leaves 0..240°, when `hypot(x, y) < 50`, or when the planar distance
exceeds L2 + L3; the SAIGEN firmware also refuses `z > 225`. Near the top of
the workspace the wrist angle passes through 0°, so:

- at (x, y) = (5, −185) the highest reachable z is 210 mm; at (120, −180) it
  is 201 mm; at (0, −100) it is 196 mm (table in `tests/test_maxarm_kinematics.py`
  and `kinematics.highest_reachable_z`);
- the resting pose after the factory `go_home` (z ≈ 222) is outside the domain
  although the arm physically stands there;
- the example workspace box (y −280..−60) limits the base to about ±71° at
  r = 185 mm, while the mechanism turns ±120° from straight ahead. Widening
  the box is the operator's decision after checking the cell.

`MaxArmWorkspaceValidator` runs this model before every move, so an
unreachable target is refused at `--validate-only` with the highest reachable
z at that (x, y); nothing is sent to the arm.

## 4. Things not to do

- **Do not hand-teach with the servos unloaded.** A bus servo that is loaded
  again jumps to its last commanded target, not to where it was pushed. Jog
  instead (`teach.py`).
- **Do not start a real move from a shell that cannot send Ctrl-C** (an
  agent's tool shell, a background job): there is no way to interrupt. Use a
  terminal, or a physical switch in the 12 V servo supply (recommended for
  every cell; the arm sags when cut and a held object drops).
- **Do not rely on the resting pose.** After a power-up the arm may be
  anywhere it was left; read `STATUS` first.
- **Do not skip `--check` after editing the firmware.** MicroPython 1.12 lacks
  parts of CPython that tests on the PC cannot miss; the device self-test can.
- **Do not open the port with pyserial defaults**: DTR/RTS are asserted on
  open and reset the board (with the factory program that means a homing
  move). Every tool here sets `rts = dtr = False` before `open()`.

## 5. Recovery

- **Firmware fault (`EVT FAULT`), board at the REPL:** RTS reset restarts the
  SAIGEN launcher; it never moves the arm at boot.
- **Back to the factory program:** `deploy_firmware --restore backups/<dir>`;
  the next reset then homes the arm (clear its reach). `/main_factory.py` on
  the device is a second copy of the factory `main.py`.
- **Stop latch set (`stopped=1` in `STATUS`):** the safety wrapper clears it
  at `initialize()` only when the arm is not moving, and logs a warning.
- **`ERR BUSY` on `PUMP 1`:** the valve is venting after `PUMP 0`; retry after
  1.1 s (the flow executor's `wait` step does this in a flow).
