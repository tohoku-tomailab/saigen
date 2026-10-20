# Dobot MG400 (TCP/IP): observation tool

The MG400 is **not** a SAIGEN robot type: no flow, GUI block or `config.yaml` entry can
use it. This package holds a human-run tool that observes the arm without changing its
state, to collect what a later integration plan needs. That plan (enable / disable, a
first small move, stopping) has been reviewed but not accepted; its open questions are
answered only by measurements on the arm.

Protocol reference: Dobot "TCP_IP Remote Control Interface Guide (4axis)", 2024-04-19
([Dobot-Arm/TCP-IP-Protocol-4AXis](https://github.com/Dobot-Arm/TCP-IP-Protocol-4AXis)).

## What the tool sends

Only these commands, on the Dashboard port 29999, each built from typed numbers (there is
no way to send free text, and the final text is checked before it is written):

| Command | Purpose |
|---|---|
| `RobotMode()` | state (4 = disabled) |
| `GetErrorID()` | alarms (controller and each servo) |
| `GetPose()`, `GetPose(User=0,Tool=0)` | flange pose in the current global frames and in frame 0 |
| `GetAngle()` | joint angles |
| `PositiveSolution(j1,j2,j3,j4,0,0)` | the controller's forward kinematics (compute only) |
| `InverseSolution(x,y,z,r,0,0,1,{j1,j2,j3,j4})` | its inverse kinematics, nearest to the given joints (compute only) |

Port 29999 also accepts commands that change the arm's state (EnableRobot, DisableRobot,
ClearError, EmergencyStop ...). The allowlist in `protocol.py`, not the port, keeps the
tool read-only. The feedback port 30004 is only read. The motion port 30003 is never opened.

The observation ends, sending nothing more, as soon as any RobotMode / GetErrorID reply
(on either connection) or any feedback packet shows a mode other than 4 (disabled), an
alarm or an out-of-range value; when no valid feedback packet arrives within 2 s or the
last one is older than 0.5 s; or on a protocol error (non-zero ErrorID, wrong echo, wrong
number of values, timeout). The state is checked before every compute command. That is
the arm's state when read; the tool does not guarantee it afterwards.

## Running it

```bash
python -m src.devices.mg400.observe --dry-run        # the command sequence; connects to nothing
python -m src.devices.mg400.observe --mock           # fake controller on 127.0.0.1
python -m src.devices.mg400.observe <arm address> --controller-version <as shown in DobotStudio Pro>
```

Before a real run: the PC's network interface in the arm's subnet, DobotStudio Pro
disconnected, nobody else controlling the arm. **Ctrl+C ends the observation only**: it
sends nothing to the arm and is not an emergency stop.

## Records

`logs/mg400_observe/<UTC time>/` (gitignored):

- `report.json`: every command, recorded before it is sent, with its send and reply times,
  the reply text (or the error and any partial reply), the echo notes; the query results,
  the two-connection test, the final feedback statistics over the whole recording, the
  kinematics pairs, why the observation ended, the tool's source hash, and `complete`
  (false for an interrupted or ended run; the file is rewritten after each command).
- `feedback.bin`: the raw 1440-byte packets back to back, `feedback_times.csv`: their
  sequence number, byte offset, receive times and reject reason, `feedback.csv`: every
  parsed field with non-finite and out-of-range flags and the packet TimeStamp minus the PC
  receive time.

Feedback fields are read at the offsets of the guide's table only (section 4). The
comparison of the 6-element feedback arrays with the 4-element replies is reported as
"mapping unverified"; the difference between the packet TimeStamp and the PC clock is
recorded but is not a latency (the clocks are not synchronised). The kinematics pairs are
the controller's internal model, not a measurement of the arm's accuracy.
