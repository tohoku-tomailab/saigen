# `src/devices/maxarm` — Hiwonder MaxArm (SAIGEN firmware + PC client)

The Hiwonder MaxArm is a small 3-axis arm (three HTS-35H bus servos, suction
nozzle with pump and valve, nozzle rotation servo) driven by an ESP32 running
Hiwonder's MicroPython build. SAIGEN controls it by replacing the factory
`main.py` with a small, repository-versioned command server that speaks a
checksummed line protocol over the USB serial port. Hiwonder's own libraries
(`espmax`, `BusServo`, `SuctionNozzle`), which are already on the device, do the
kinematics and the servo bus; they are **not** redistributed in this repository.

The flow executor never uses this package directly: it goes through
`src.devices.safety.MaxArmRobot`, which validates every target against the
workspace limits in `config.yaml` before a command is sent.

| File | Role |
|---|---|
| `firmware/saigen_maxarm.py` | Command server (runs on the ESP32; also runs on CPython for the tests) |
| `firmware/main.py` | Device launcher: builds the Hiwonder objects, zeroes the pump/valve PWM, serves the protocol |
| `protocol.py` | PC-side codec (checksum, framing, reply parsing), byte-compatible with the firmware |
| `client.py` | `MaxArmClient` (pyserial): connect without reset, transactions, heartbeat, STOP from any thread |
| `deploy_firmware.py` | **Human-run** backup / install / restore tool (raw REPL) |

> **警告 / WARNING — the arm can move when the board resets.**
> 工場出荷の `main.py` は起動のたびに `arm.go_home()` を実行します。電源投入、USB の
> DTR/RTS の変化（ESP32 の自動リセット回路）、Ctrl-D ソフトリセットのいずれでも
> アームが原点へ動きます。SAIGEN ファームウェアは起動時にアームを動かしませんが、
> インストール前・復元後は工場出荷のプログラムが動きます。
> Hiwonder's factory `main.py` calls `arm.go_home()` at every boot — power-on,
> a DTR/RTS transition on the USB port (the ESP32 auto-reset circuit) or a Ctrl-D
> soft reset. The SAIGEN firmware never moves the arm at boot, but before it is
> installed and after a restore, the factory program is in charge.

## USB / wiring facts

- USB-serial bridge: CH340 (VID 1A86, PID 7523); on the lab PC it enumerates as a
  COM port (e.g. `COM11`). 115200 8N1 for the SAIGEN firmware and the REPL.
- The ESP32's EN/IO0 are wired to RTS/DTR. pyserial asserts both lines when a port
  is opened with the defaults, which resets the board. `MaxArmClient` and
  `deploy_firmware.py` set `rts=False` and `dtr=False` **on the unopened port**
  before `open()`, and on close drop RTS first, then DTR.
- Power: 12 V 5 A for the servos. Servos hold their last target after an ESP32
  reset or Ctrl-C; the pump is off after a reset.
- Coordinate frame (Hiwonder `espmax`): millimetres, origin on the base yaw axis,
  z = 0 at the base plane, the arm faces −y at home; home/ORIGIN is
  (0, −162.8, 212). `set_position` refuses hypot(x, y) < 50 and clamps z to 225
  (the SAIGEN firmware rejects both instead of moving elsewhere).

## Wire protocol `saigen-maxarm`, proto 1

Text lines, `\n` terminated (`\r` tolerated and stripped). At most 200 bytes per
line (body plus `*XX`, without the terminator); longer lines get `ERR TOOLONG`
and are discarded. Every line in both directions ends with `*XX`, where XX is
two upper-case hex digits of the XOR of all bytes before the `*` (NMEA style;
lower-case hex is accepted on input). Tokens are separated by single spaces.
Only printable ASCII (0x20–0x7E) is valid, so Ctrl-C (0x03) can never be part of
a frame and always reaches MicroPython as KeyboardInterrupt.

```
Request:  <seq> <CMD> [arg ...]*XX          seq = 1..999999, chosen by the PC
Reply:    <seq> OK [key=value ...]*XX   or  <seq> ERR <CODE> [free text]*XX
Event:    EVT <NAME> [key=value ...]*XX     unsolicited; the PC tolerates them anywhere
```

Numbers are plain decimals: `[-]digits[.digits]` (no exponent, no `+`, no `_`).

| CMD | args | OK payload | notes |
|---|---|---|---|
| PING | - | `pong=1` | |
| INFO | - | `fw=saigen-maxarm proto=1 ver=<semver> mp=<sys.version, spaces -> _> limits=<0/1> wd=<ms> stopped=<0/1>` | |
| STATUS | - | `x= y= z= p1= p2= p3= moving=<0/1> stopped=<0/1> pump=<0/1> vin=<mV or -1>` | x, y, z from `arm.read_position()` (ints, mm); p1..p3 servo pulses; a failed servo read -> `ERR SERVO`. `moving` = 1 while now < move_until (open loop). |
| MOVE | x y z ms | `x= y= z= ms=` (the accepted target) | ms integer 100..30000. `ERR STOPPED` while the stop latch is set; `ERR LIMIT` if device limits are set and violated; `ERR UNREACHABLE` if z > 225 or hypot(x, y) < 50 (nothing sent), or if `set_position` does not return True — the vendor library swallows its own errors after possibly commanding some servos, so all three get SERVO_MOVE_STOP first (no latch). If one of those stops raises, the arm may still be moving: the stop latch is set and the reply is `ERR SERVO ... (stop latch set)` instead. On success move_until = ticks_add(now, ms). Acknowledged immediately. |
| STOP | - | `stopped=1` | `bus_servo.stop(i)` for i = 1..3 (SERVO_MOVE_STOP), move_until = now, stop latch = 1. Pump unchanged. Always accepted (`ERR SERVO` if a servo stop raised; the latch is still set). |
| CLEAR | - | `stopped=0` | clears the stop latch |
| PUMP | 0/1 | `pump=<0/1>` | `nozzle.on()` / `nozzle.off()`. `off()` vents the valve for 1 s in a thread and the vendor `on()` is silently ignored until it is done, so `PUMP 1` within 1100 ms after `PUMP 0` is answered `ERR BUSY` (not executed; retry later). After the window `PUMP 1` is still `ERR BUSY` while the vendor flag `nozzle.nozzle_st` is True and the firmware's own pump flag is off (vent thread late): the window is a minimum, `nozzle_st` is the completion signal. `PUMP 0` is always accepted. |
| TORQUE | 0/1 | `torque=<0/1>` | `bus_servo.unload(i)` / `load(i)` for all 3 (0 = arm goes limp; calibration only) |
| LIMITS | xmin xmax ymin ymax zmin zmax [rmin rmax] | `limits=1 xmin=.. .. [rmin=.. rmax=..]` | device-side box (+ optional radius) re-checked on every MOVE. `LIMITS OFF` -> `limits=0`. Not persisted. |
| WATCHDOG | ms | `wd=<ms>` | 0 = disabled, otherwise 100..60000. When no complete valid line has arrived for `ms`: STOP behaviour + `EVT STOPPED reason=watchdog`, then the watchdog disables itself (wd=0) until re-armed. Any valid line (checksum and seq OK, even an unknown command) feeds it; a line that arrives after the deadline is checked against the watchdog *before* it counts, so a late HB cannot revive an expired watchdog. |
| HB | - | `wd=<ms>` | heartbeat; feeds the watchdog |
| BYE | - | `bye=1` | disables the watchdog; everything else stays (arm holds, pump unchanged) |

Error codes: `BADCMD` (unknown command), `BADARG` (wrong count/type/range, or a
bad sequence number), `CHECKSUM` (bad or missing `*XX`, or a non-ASCII byte; the
reply uses seq 0 if the seq could not be parsed), `TOOLONG`, `STOPPED`, `LIMIT`,
`UNREACHABLE`, `SERVO`, `BUSY` (PUMP 1 inside the vent window), `INTERNAL`
(unexpected exception; the text names it). An unexpected exception in an
actuator command (MOVE, TORQUE, PUMP) may have left it partly applied: the
firmware sends STOP behaviour and sets the stop latch before answering
`ERR INTERNAL ... (stop latch set)`; the pump is not touched.

Events: `EVT READY fw=saigen-maxarm proto=1 ver=<semver>` (once at boot),
`EVT STOPPED reason=watchdog wd=<ms> servo_ok=<0/1>`,
`EVT FAULT type=<exception> msg=<text>` (the server loop ended; the REPL follows).

### Boot behaviour

The firmware never moves the arm and never changes servo torque at boot. `main.py`
constructs `BusServo()`, `ESPMax(bus)` and the `SuctionNozzle` (none of which
commands a servo; the nozzle is allocated with `SuctionNozzle.__new__` and stored
*before* its `__init__` runs, so PWM outputs a failing constructor already
created at duty 512 are still zeroed by the start-up cleanup), sets the pump and
valve PWM duties to 0 (MicroPython's
`PWM(Pin)` starts at duty 512), prints `EVT READY ...` once and enters the loop
(poll stdin 20 ms, feed bytes, watchdog tick). If any of the four pump/valve
outputs is missing or `duty(0)` raises, start-up fails: no `EVT READY`, the
remaining outputs are zeroed best effort, the servos get STOP and `EVT FAULT` is
printed. The watchdog is disabled at boot (wd = 0) and the stop latch is 0. If
the loop raises — including Ctrl-C from the host — the launcher sends STOP
behaviour (best effort), prints `EVT FAULT ...` and re-raises, so the MicroPython
REPL becomes reachable. Ctrl-C is never masked. After READY the fault path never
touches the pump.

Tick arithmetic follows MicroPython rules: deadlines are `ticks_add(now, ms)` and
compared only with `ticks_diff`; a move deadline is compared only while a move is
active (the loop expires it), so an idle arm never reads as moving after the
ticks counter wraps.

### How the PC client uses it

`MaxArmClient.connect(limits, watchdog_ms)`: open without reset (a port left
open by an earlier connection is closed first), listen 0.3 s for reset banners
(`rst:`, `ets `, `MicroPython v`, `boot:` -> `MaxArmResetDetected`), PING up to 3
times, INFO must report `fw=saigen-maxarm proto=1`, then LIMITS (if given, echo
checked) and WATCHDOG (echo `wd=<ms>` checked), then INFO again, which must
report `limits=1` (if limits were sent) and `wd=<ms>`: a reset in the middle of
the handshake fails the connect. Any `EVT READY` after the pre-PING drain (i.e.
during the handshake, not only after it) is a reset (`MaxArmResetDetected`).
The constructor refuses a `timeout`/`connect_timeout` that is not finite and
> 0 and a `heartbeat_period` that is not None (no heartbeat thread) or finite
and > 0; `move_xyz_and_wait` refuses a `timeout_margin` that is not finite and
>= 0 (`ValueError`, nothing sent). CLEAR is **not** sent
automatically; the safety wrapper decides. A heartbeat thread sends HB whenever
no other command was sent for `heartbeat_period` (0.25 s). `move_xyz_and_wait`
sends MOVE and polls STATUS every 0.1 s until `moving=0`, then checks the
read-back against the target (`tolerance_mm`, finite and > 0). STATUS is parsed
strictly (flags exactly `0`/`1`, finite coordinates; anything else is
`MaxArmProtocolError`). `stop()`/`force_stop()` work from any thread while a move
is being waited for.

**Watchdog timing rule.** `watchdog_ms` must be 0 (disabled) or at least
`1000 * (2 * heartbeat_period + timeout)` — 1000 ms with the defaults
(0.25 s heartbeat, 0.5 s reply timeout); a shorter value raises `ValueError` in
`connect()` before the port is opened. `min_watchdog_ms()` returns the bound.
Because the device watchdog disables itself after firing, `clear()` sends
`CLEAR` and then re-arms `WATCHDOG <ms>` and verifies the echo. The client's
expected watchdog value is updated under the transport lock as soon as a
WATCHDOG echo is verified, so a heartbeat never checks a new device value
against an old expectation.

**Fault latches** (cleared only by a new `connect()`, which redoes LIMITS and
WATCHDOG; STOP / `force_stop()` are always allowed, in every state):

| Attribute | Set when | Effect |
|---|---|---|
| `fault_reason` (with `connected = False`) | reset banner, `EVT READY` during the handshake or after connect, `EVT FAULT`, a MicroPython REPL prompt | every command except STOP raises `MaxArmError`; the device has lost LIMITS and WATCHDOG |
| `link_fault` (with `connected = False`) | two consecutive failed HB transactions; any reply reporting a `wd` other than the armed one (`wd=0` after the watchdog fired); `EVT STOPPED reason=watchdog`; a `WATCHDOG` re-arm after CLEAR that is not confirmed | one best-effort STOP is sent (after `EVT STOPPED` only when it reports `servo_ok` other than `1`, i.e. a servo stop raised on the device or the field is missing; that STOP is sent once, after the transaction that read the event), the heartbeat stops (so the device watchdog fires if it is still armed), a waiting `move_xyz_and_wait` ends, every command except STOP raises `MaxArmLinkFault`. `fault_reason` also reports it. |

What the watchdog detects: the host process dying or freezing for longer than
`watchdog_ms`, and the USB link breaking in either direction (after two lost HB
replies the client stops feeding it). It does not
detect a hung *experiment* thread while the heartbeat thread still runs; the
safety wrapper's own timeouts cover that.

`motion_uncertain` is True from the moment a MOVE is sent until
`move_xyz_and_wait` confirms the arrival or a STOP is acknowledged (a lost MOVE
acknowledgement, a STATUS failure, a timeout or a tolerance error leave it set).
The client never sends STOP on its own after a failed move — the safety wrapper
does — but `close()` does: it refuses new commands, sends STOP first when
`motion_uncertain`, then BYE; if that STOP is not acknowledged, or after a
reset/link fault, BYE is skipped so the device watchdog stays armed.

**Suction policy.** Neither the firmware nor the client ever changes the pump on
its own: STOP, the watchdog, `EVT FAULT`, `close()` and BYE leave suction as it
is, so a held object is not dropped. What happens to the pump after an
emergency stop or an abnormal end is decided by the safety wrapper
(`src.devices.safety.MaxArmRobot`: by default it keeps suction after an
emergency stop and logs that the object must be recovered by hand; see
`docs/setup.md`).

## Talking to it with a terminal

Use a terminal that keeps DTR/RTS low from the moment the port opens, e.g.
pyserial's miniterm (it applies `--rts`/`--dtr` before opening the port; its CR LF
line ending is fine):

```bash
python -m serial.tools.miniterm --rts 0 --dtr 0 COM11 115200
```

Many GUI terminals assert DTR/RTS on open, which resets the board (and, with the
factory program, moves the arm). Compute a checksum with
`python -c "from src.devices.maxarm.protocol import encode_line; print(encode_line('1 PING'))"`.

```
> 1 PING*01
< 1 OK pong=1*2F
> 2 STATUS*06
< 2 OK x=0 y=-163 z=212 p1=500 p2=500 p3=500 moving=0 stopped=0 pump=0 vin=11800*1C
> 3 PING*00
< 3 ERR CHECKSUM mismatch*5F
```

A wrong `*XX` is always answered with `ERR CHECKSUM` and never executed. Typing
Ctrl-C stops the server (`EVT FAULT type=KeyboardInterrupt`) and leaves the board
at the `>>>` REPL with the servos stopped where they are.

## Deploy, backup and restore (human-run)

`deploy_firmware.py` is listed in CLAUDE.md rule 1: an agent never runs it.
Before you start: the arm's reach is clear, the 12 V supply can be cut by hand.

1. List and back up the device filesystem (nothing is written to the device):

   ```bash
   python -m src.devices.maxarm.deploy_firmware --port COM11 --list
   python -m src.devices.maxarm.deploy_firmware --port COM11 --backup backups/maxarm_factory
   ```

   The tool opens the port with RTS/DTR low, interrupts the running program with
   Ctrl-C and enters the raw REPL (`\r\x03\x03`, drain, `\r\x01`, wait for
   `raw REPL; CTRL-B to exit`, up to 5 attempts). `backups/maxarm_factory/` gets
   every file (`boot.py`, `main.py`, ...) and `device_info.json` (os.uname,
   sys.version, file list with sizes and sha256). Keep this directory; it is the
   way back to the factory program. Do not commit it.

2. Check the build (optional on its own; `--install` runs it anyway):

   ```bash
   python -m src.devices.maxarm.deploy_firmware --port COM11 --check
   ```

   `check_device` confirms on the board what the firmware relies on: `ticks_ms` /
   `ticks_diff` / `ticks_add`, `select.poll()` on `sys.stdin` plus a real stdin
   round trip (the tool sends one byte while a snippet polls and reads it),
   `sys.stdin.buffer` (informational: `main.py` falls back to `sys.stdin`),
   `sys.stdout.write`, the vendor modules (`BusServo`, `espmax`, `SuctionNozzle`),
   allocating a `SuctionNozzle` with `SuctionNozzle.__new__` or `object.__new__`
   (whichever the build supports; `main.py` tries both) without running its
   `__init__`, `bytes(memoryview(...)).decode()`, `%`-formatting, free heap, and
   finally that `saigen_maxarm.py` imports from a temporary file (removed
   afterwards), that the command server answers 16 request lines correctly on
   the device interpreter against dummy arm/bus/nozzle objects (every reply
   path: OK, each ERR code, the watchdog event — this is what found the
   `Exception.__init__` incompatibility of MicroPython 1.12 on the bench), and
   that `main.py` compiles on that build. Nothing moves, no torque or PWM duty
   changes. A failed required check makes `--install` refuse (`--force`
   overrides) — better than a launcher that faults at the first boot.

3. Install the SAIGEN firmware:

   ```bash
   python -m src.devices.maxarm.deploy_firmware --port COM11 --install --backup backups/maxarm_factory
   ```

   Refused unless `backups/maxarm_factory/main.py` exists and matches the current
   `main.py` (or the device already runs SAIGEN); `--force` overrides. The factory
   `main.py` is additionally kept on the device as `/main_factory.py`. Each file is
   written to `<name>.tmp`, read back and compared, then committed as
   `<name> -> <name>.old`, `<name>.tmp -> <name>`, remove `<name>.old`, so there
   is always one complete copy on the device. If that step fails or is
   interrupted, the tool prints the recovery steps: at the REPL,
   `os.rename('/main.py.old', '/main.py')` when `/main.py` is missing; a later
   install refuses to proceed while only the `.old` copy exists.

   `--backup DIR` refuses any DIR that is not empty — a complete backup
   (`device_info.json`), an interrupted one (files without `device_info.json`)
   or unrelated files — unless `--overwrite-backup` is given, so the factory
   backup cannot be replaced by a backup of the SAIGEN launcher, or mixed with
   one, by accident. Use a new directory for every backup. `--backup` and
   `--restore` cannot be combined.

4. Start it. Without `--reset` the board is left at the REPL (the arm holds its
   pose); power-cycle the arm to start the new `main.py`, which does not move the
   arm. `--reset` (with a typed confirmation, or `--yes`) soft-resets instead.
   Check with a terminal or `MaxArmClient` that `EVT READY` / `INFO` reports
   `fw=saigen-maxarm proto=1`.

Restore the factory program:

```bash
python -m src.devices.maxarm.deploy_firmware --port COM11 --restore backups/maxarm_factory
```

Every backed-up file whose content differs is written back (after checking the
backup's sha256; missing directories are created on the device; `/boot.py` and
`/main.py` are written last, so a failure part-way leaves the installed launcher
in charge), and `/saigen_maxarm.py` is removed. **The restored factory
`main.py` moves the arm to home at the next power-on or reset.**

Full factory re-flash: Hiwonder ships a recovery image (`MaxArm-V1.0.bin`,
appendix "5" of the Hiwonder MaxArm documentation) that is written with
esptool / Hiwonder's flashing tool. It erases the filesystem; afterwards the
factory `main.py` runs (and moves the arm at boot).

### Raw REPL not reachable

Some MaxArm units ship with a factory `main.py` that reconfigures the USB UART as
a 9600-baud serial slave (`UART(1, 9600, tx=1, rx=3)`) and runs a blocking loop.
Then the REPL is silent at 115200 and the tool reports that it could not enter the
raw REPL (this is what the lab's unit did on 2026-09-24).

That factory `main.py` starts with `time.sleep_ms(10000)` and only *then*
re-configures the UART and homes the arm. `--catch-boot` uses that window: the
tool opens the port, asks you to power-cycle the arm, and sends Ctrl-C ten times
a second until the `>>> ` prompt appears. The interrupt lands in the 10 s sleep,
so **the arm does not move** and the normal `--list` / `--backup` / `--install`
continues from there:

```
python -m src.devices.maxarm.deploy_firmware --port COM11 --catch-boot --list
python -m src.devices.maxarm.deploy_firmware --port COM11 --catch-boot --backup backups/maxarm_factory
```

If the prompt does not appear within 9 s of the boot output, the tool stops with
an error: the slave loop has taken the UART and `go_home` has run; power-cycle
and try again (watch the boot: LED1 flashes, buzzer "DiDi"). `--rts-reset` pulses
RTS instead of a manual power cycle; it only works if RTS is wired to EN on your
board, and a reset that is *not* caught also homes the arm, so clear the reach
first either way. Alternatives: interrupt the boot with Thonny, or re-flash
`MaxArm-V1.0.bin` and then run `--backup` / `--install`.

## Field notes

What commissioning a real arm taught (safe order of work, measured timings,
the reachable envelope, things not to do, recovery) is collected in
[`docs/maxarm-commissioning.md`](../../../docs/maxarm-commissioning.md).

## Teaching positions (`teach.py`, human-run)

`python -m src.devices.maxarm.teach` jogs the arm in small steps and records
named poses: `x+ x- y+ y- z+ z-` (by the `step`, or `z-2.5` by an amount),
`goto X Y Z`, `turn DEG` (base rotation at the same radius and height), `top`
(highest reachable z at the current x, y), `home`,
`pump on|off`, `save NAME`, `list`, `del NAME`, `quit`; Ctrl-C is the emergency
stop. Every move goes through `MaxArmRobot` with the robot's validator, so
targets outside the box or the IK domain are refused before anything is sent.
Positions are written after each `save` to `maxarm_positions.json` (gitignored)
with the date, port and workspace; paste them into `move_xyz` steps or
`home_position`. `--mock` practises the commands without hardware. The tool
never unloads the servos on purpose: a bus servo that is loaded again jumps to
its last commanded target, not to where it was pushed by hand.

## Bench checks (to confirm on the Hiwonder build)

These points are covered by unit tests against stubs only; confirm them once on
the real MaxArm (at the MicroPython REPL, arm powered but nothing commanded).
`deploy_firmware.py --check` does the first group automatically (see step 2 of
"Deploying"); the remaining ones need the firmware running:

- `--check`: `ticks_add`, `select.poll()` on stdin and the stdin round trip,
  `sys.stdin.buffer`, the vendor imports, allocating a `SuctionNozzle` with
  `SuctionNozzle.__new__` or `object.__new__` (older builds lack the former;
  `main.py` tries both), and that `saigen_maxarm.py` imports on the build. If
  neither allocation form works, `main.py` fails safe (no READY, `EVT FAULT`);
  the fallback is then to force the four outputs low directly —
  `Pin(p, Pin.OUT, value=0)` for every pin in the vendor module's `pump_io` +
  `valve_io` (M1 21/19, M2 18/5) — in the start-up cleanup.
- `n.nozzle_st` exists and follows the vendor source: True after `on()`, still
  True for about 1 s after `off()` (vent thread), then False. `PUMP 1` relies on
  it to answer `ERR BUSY` while the vent thread is late.

## Limitations

- **Open loop.** MOVE is acknowledged when the servos have been commanded;
  `moving` is a timer (now < move_until), not a measurement. `move_xyz_and_wait`
  compares the read-back position with the target afterwards.
- **Reachable envelope is narrower than the box.** The vendor inverse kinematics
  (`__espmax.inverse`) raises `ValueError("Invalid angle ...")` when the wrist
  joint would be at or below 0°, which happens near the top of the workspace:
  on the bench (2026-09-24) `(5, -185, 211)` and `(5, -185, 221)` were rejected
  while `(5, -185, 200)` was accepted, and the pose the arm rests in after the
  factory `go_home` (read back as z≈222) is itself outside the IK domain. The
  firmware answers `ERR UNREACHABLE` and nothing moves, the client raises
  `MaxArmMoveError`, and the safety wrapper treats it as a failed move (STOP,
  pose unknown, run ends). To catch this **before** anything is sent,
  `kinematics.py` reproduces the arm's solver on the PC (link lengths verified
  against the compiled `__espmax` to four decimals on the bench; the joint gate
  is 0..240° on every joint, x is mirrored, `hypot(x, y) < 50` and `z > 225`
  are refused). `MaxArmWorkspaceValidator` (built by `validator_for_robot` for
  every `type: maxarm` robot) runs it after the box/radius check, so
  `--validate-only`, `--mock`, the preflight and the wrapper's own pre-move
  check all reject an unreachable target with the reason and the highest
  reachable z at that (x, y). `go_home` lifts only as high as the arm reaches at
  the current (x, y) before moving to the home pose, and the pose the arm rests
  in after the factory homing is accepted as a *start* (box check only) even
  though it cannot be commanded — set `home_position` in `config.yaml` to a
  reachable pose (the example `[0, -160, 180]` is). The PC model only predicts
  acceptance; the arm's own solver still decides.
- **Read-back noise.** `read_position()` is forward kinematics of the servo pulses
  and is noisy by a few mm (up to ~8 mm reported). `espmax` also clamps servo 3
  (≥ 470) and servo 2 (≤ 700) silently; such a target ends away from the command
  and is caught by the tolerance check, not by the firmware.
- **Path shape.** The servos interpolate in joint space: the tool path between two
  points is an arc, not a straight line. Validate the start and end *and* keep
  the space between them clear.
- **STOP is not a safety function.** STOP sends SERVO_MOVE_STOP to each servo; the
  stop latency and overshoot must be measured on the bench. The watchdog only
  helps if the ESP32 is still running.
- **No hardware emergency stop.** Fit a physical switch (latching mushroom button
  or relay) in the 12 V servo supply. Cutting it makes the arm sag under gravity,
  and an object held by suction may drop when the pump stops — plan for that.
- Device limits (`LIMITS`) are not persisted; the client sends them at every connect.
- The nozzle rotation servo is not exposed by proto 1.
