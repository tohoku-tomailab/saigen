# Commissioning a Dobot MG400 over TCP/IP: what the bench taught us

Field notes from the first observation runs of a Dobot MG400 in the SAIGEN lab
(October 2026). They complement the reference material in
[`src/devices/mg400/README.md`](../src/devices/mg400/README.md) (what the tool
sends, when it stops, what it records). The MG400 is **not** a SAIGEN robot
type: nothing here lets a flow move it.

Configuration checked: one MG400 at the same network address on both runs,
read from a Windows 11 PC on 2026-10-21 (two runs). The controller version was not typed
into either run (`controller_version_typed: null`). Both runs ended before the
kinematics pairs, so §4 lists what is still open. Another unit or firmware may
differ, so treat what follows as expectations to re-check, not as
specifications.

## 1. Scope chosen: observation only

The Dashboard port 29999 accepts commands that change the arm's state
(EnableRobot, DisableRobot, ClearError, EmergencyStop ...), so the port gives
no protection by itself. The tool builds only six read-only commands from typed
numbers and checks the final text against an exact-form allowlist before it is
written; it reads the feedback port 30004 and never opens the motion port
30003. Enabling, a first small move and stopping are a later stage with its
own reviewed plan; they need the complete observation record first.

## 2. Order of work

Every step that talks to the arm is the human's (CLAUDE.md rule 1): the human
asks for the run in the session, with the arm disabled and DobotStudio Pro
disconnected.

1. `python -m src.devices.mg400.observe --dry-run` (the command sequence;
   connects to nothing).
2. `python -m src.devices.mg400.observe --mock` (fake controller on 127.0.0.1).
3. A ping of the arm's address, from a PC whose interface is in the arm's
   subnet. The address belongs in your own notes, never in tracked files.
4. The real run, with `--out` pointing **outside** any synced folder
   (see §3.3), and `--controller-version` as DobotStudio Pro shows it.

## 3. What the bench showed

### 3.1 Replies that matched the guide

On both runs, with the arm disabled:

- `RobotMode()` answered `0,{4},RobotMode();` (4 = disabled).
- `GetErrorID()` answered six empty inner lists: no alarm reported. (Which list
  belongs to what is not settled: the guide's reply notation and its text
  about four servos do not agree.)
- `GetPose()` answered four values (x, y, z, r).
- The recorded `reply_s` of the Dashboard replies were 0 to 47 ms; the
  monotonic times they come from moved in steps of about 16 ms.
- The feedback port sent 1440-byte packets (8 on the first run, 18 on the
  second) whose TimeStamp advanced by 8 ms per packet; the test value matched
  the guide's `0x0123456789ABCDEF`; no packet was rejected. The PC receive
  times do not follow that 8 ms step (median inter-arrival 0 and 15 ms), so
  the PC receive time is not the packet time.
- The packet TimeStamp was about 916 s behind the PC clock. The clocks are not
  synchronised; the difference is recorded, never read as a latency.

On the second run only (the first ended before it):

- `GetAngle()` answered **six** values, the last two `0.000000`; the tool
  reads the first four as the joints.

### 3.2 `GetPose(User=0,Tool=0)` is refused

The guide documents `GetPose(User=0,Tool=0)`. The controller answered
`-30001,{},GetPose(User=0,Tool=0);` ("the type of the first parameter is
incorrect"), which ended the first run. The form was removed from the tool and
from its allowlist; the comparison with the feedback frame indices now uses
`GetPose()` and the `user_index` / `tool_index` of the packet.

### 3.3 The record could not be saved under OneDrive

On the first run, in a folder synced by OneDrive, the latest record stayed in
`report.json.tmp` and `report.json` was left one save behind. The terminal
output of that run (the record itself could not hold it) showed the final
rename refused with WinError 5, and that error printed in place of the
original one. The tool now retries a refused rename for about 1.5 s, reports a
save failure next to the original error, never in its place, names the file
that holds the latest record, and suggests `--out` when the folder is under
OneDrive. The second run wrote outside the synced tree.

### 3.4 The first Dashboard connection's end was detected after a second was opened

The second run opened a second Dashboard connection to see whether the
controller serves two clients. After the second connection was opened, the
next `RobotMode()` on the first, sent 14 ms after its last reply, met a clean
end of stream with no reply bytes. When the first connection actually ended is
not recorded, only when its end was detected. The arm stayed disabled and the
feedback feed was unaffected.

One run does not prove that the controller serves a single Dashboard client.
The tool now records this step in parts (an idle wait and a control
`RobotMode()` on the first connection alone, then the second connection, then
the probe), so the next run separates "the first connection ends on its own"
from "it ends when a second one opens". Only exactly that ending (end of stream
or reset on the probe, no reply bytes, on the first connection), when no save of
the run has failed and the record of the ending is saved, lets the observation
continue on the second connection, after the state is checked there. If it is confirmed, a later control stage cannot keep a separate
monitoring connection on the Dashboard port beside the controlling one.

## 4. Open points

- A complete observation record: the remaining queries on the second
  connection and the kinematics pairs (`PositiveSolution` /
  `InverseSolution`) have not yet been read from this controller.
- Whether the first-connection ending repeats, and whether it happens without
  a second connection (the control probe answers this).
- The controller version, to be typed into the next run.
- The mapping of the 6-element feedback arrays to the 4-element replies. The
  tool always reports its comparison as "mapping unverified" (neither run got
  that far); checking it, for example at a second pose, is a human step still
  to be planned.

## 5. Interrupting a run

Ctrl+C ends the observation only: the tool sends nothing to the arm and it is
not an emergency stop. Use the arm's own emergency stop for that. A run that
ends early leaves `complete: false` in the record, with the reason in `notes`.
