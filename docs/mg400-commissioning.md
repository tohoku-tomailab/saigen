# Commissioning a Dobot MG400 over TCP/IP: what the bench taught us

Field notes from the first observation runs of a Dobot MG400 in the SAIGEN lab
(October 2026). They complement the reference material in
[`src/devices/mg400/README.md`](../src/devices/mg400/README.md) (what the tool
sends, when it stops, what it records). The MG400 is **not** a SAIGEN robot
type: nothing here lets a flow move it.

Configuration checked: one MG400, read from a Windows 11 PC on 2026-10-21 and
2026-10-22 (the address typed on each run is in the operator's notes, not in
the records). The controller version was not typed into any run
(`controller_version_typed: null`). The first three runs ended early; the
fourth (§3.6) is the first complete record.
§4 lists what is still open. Another unit or firmware may differ, so treat
what follows as expectations to re-check, not as specifications.

## 1. Scope chosen: observation only

The Dashboard port 29999 accepts commands that change the arm's state
(EnableRobot, DisableRobot, ClearError, EmergencyStop ...), so the port gives
no protection by itself. The tool builds only six read-only commands from typed
numbers and checks the final text against an exact-form allowlist before it is
written; it reads the feedback port 30004 and never opens the motion port
30003. After the observation record was complete, a second tool was
implemented to its own reviewed plan: `jog.py` (`src/devices/mg400/README.md`,
"Jog tool"), which enables the arm, makes one small joint move and back, and
disables it. It has not been run on the arm yet (§4). The observation tool
itself stayed read-only; the control commands it shares a module with are
refused on its connections.

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

The jog tool comes after a complete observation record, and only on the
human's request in the session:

5. `python -m src.devices.mg400.jog --dry-run`, then `--mock`.
6. In DobotStudio Pro: the arm disabled, no alarm, nothing on the flange; note
   the playback joint speed / acceleration and the global speed ratio (the tool
   bounds only the ratio in its command); then close DobotStudio Pro.
7. Check that the arc of the rotation (default J1 +5 deg and back; at most
   10 deg) is clear for the whole arm and its cables, and that the emergency-
   stop button is within reach. Stay at the arm for the run.
8. The real run: `python -m src.devices.mg400.jog <arm IP> --controller-version
   <V> --note "<speed settings>" --out <folder outside any synced folder>`. The
   tool prints the plan and waits for Enter; the human presses it. Ctrl+C after
   that starts the tool's orderly stop (ResetRobot, DisableRobot, then
   EmergencyStop unless the arm showed itself disabled, or in alarm with its
   joints still, and the Enable's outcome is known); it is not the emergency
   stop, the button is.

## 3. What the bench showed

### 3.1 Replies that matched the guide

On the first two runs, with the arm disabled (the third, §3.5, gave the same
reply forms; its `reply_s` reached 62 ms):

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

From the second run on (the first ended before it):

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
continue on the second connection, after the state is checked there. If it is
confirmed, a later control stage cannot keep a separate monitoring connection
on the Dashboard port beside the controlling one.

### 3.5 Third run: the step in parts, and `InverseSolution` refused

On the third run the control `RobotMode()` on the first connection alone,
after the 1 s idle wait, was answered. After the second connection was opened
and 0.2 s had passed, the probe on the first (0.234 s after its last reply)
met an end of stream with no reply bytes. That is what this run showed; it
does not show that the first connection never ends on its own, nor when it
actually ended, nor that the second connection caused it. The end was the
tolerated one, so the tool checked the state on the second connection and
continued there: `GetPose()`, `GetAngle()`, the 10 s feedback wait, and
`PositiveSolution(j1,j2,j3,j4,0,0)`, which answered four values.

Over the whole recording (11.4 s, including the 10 s wait) 1401 feedback
packets arrived, none rejected and no fault; the TimeStamp step was 7 to 12 ms,
median 8 ms. The comparison of the packet with the replies after the wait
(still reported as "mapping unverified") differed by at most 0.0002 in the
joints and 0 in the pose, with user and tool index 0.

The guide's form with the nearest-joints option,
`InverseSolution(x,y,z,r,0,0,1,{j1,j2,j3,j4})`, was answered `-20000,{},...`,
"parameter number error" in the guide's error table, and the run ended there
(`complete: false`). The form was removed from the tool and its allowlist,
like `GetPose(User=0,Tool=0)`; the tool now sends the six-parameter form
`InverseSolution(x,y,z,r,0,0)`, whose solution, in the guide, is the one
nearest the arm's current joint angles. The fourth run (§3.6) showed that this
controller answers that form. That the controller is older than the guide is
a guess, not a finding.

### 3.6 Fourth run: the first complete record

On the fourth run (2026-10-22, arm disabled, no alarm on any reply) the tool
ran to the end (`complete: true`). What it added:

- The six-parameter `InverseSolution(x,y,z,r,0,0)` was answered with four
  values on all 13 pairs. For every pair (the current joints and the 12 grid
  points with j1 in {-30, 0, 30}, j2 and j3 in {0, 30}, j4 = 0) the inverse
  solution of the forward solution differed from the pair's joints by at most
  0.000015 degrees, so no pair came back as another branch. These are the
  controller's own forward and inverse model agreeing with itself, not a
  measurement of the arm.
- `PositiveSolution` of the joints read by the first `GetAngle()` of the run
  differed from the `GetPose()` sent 12 ms before that `GetAngle()` by at most
  0.0006 in x, y, z and 0.0028 in r. Both readings jitter (below), so this is
  a consistency check, not a calibration.
- The first Dashboard connection's end was detected a third time, in the same
  place: the control `RobotMode()` after the 1 s idle wait was answered; the
  probe, 0.203 s after the first connection's last reply and after the second
  connection had been opened, met an end of stream with no reply bytes. The
  tool continued on the second connection. As before, when the connection
  actually ended and whether the second connection caused it are not shown.
- Feedback over the whole recording (12.1 s): 1484 packets, none rejected, no
  fault, TimeStamp step 7 to 12 ms with median 8 ms, `robot_mode` 4 and
  `error_status` 0 in every packet. The comparison with the replies after the
  wait (still "mapping unverified") differed by at most 0.0002 in the joints
  and 0 in the pose (after rounding to six decimals), with user and tool
  index 0.
- With the arm disabled (mode 4 and `enable_status` 0 in every packet), the
  joint values in the packets (`q_actual`, read at the guide's offsets) varied
  by up to 0.014 degrees (j4) over the recording, and `GetAngle()` replies on
  the same run differed by up to 0.0055 degrees in j4. The fields read as
  `qd_actual` and `tcp_speed_actual` were not zero in every packet: `qd_actual`
  took a few discrete values with median 0 and a maximum absolute value of
  about 66.2 in j4 (in the packet's unit), and `tcp_speed_actual` a maximum
  absolute value of about 65.4 in z. Non-zero values of both fields were also in the second and third
  runs' packets. What those fields hold for a disabled arm is not explained by
  the guide; a later stage should not read them as motion.
- The Dashboard `reply_s` were 0 to 47 ms, as before.

## 4. Open points

- Whether the first-connection ending repeats beyond these three runs (each
  time after a second connection was opened), and whether it happens with
  the arm enabled: a later control stage cannot assume a second monitoring
  connection on the Dashboard port.
- The controller version, to be typed into the next run.
- The mapping of the 6-element feedback arrays to the 4-element replies. The
  tool always reports its comparison as "mapping unverified"; checking it, for
  example at a second pose, is a human step still to be planned.
- The meaning of the non-zero `qd_actual` / `tcp_speed_actual` fields on a
  disabled arm (§3.6).
- What the jog tool will show on the bench, none of it yet observed: whether
  `ResetRobot()` on a disabled arm answers 0 and leaves it disabled; how long
  `EnableRobot()` takes to show mode 5 on the Dashboard and in the feedback;
  whether `JointMovJ` at a 10 % ratio is accepted, how its echo looks (the
  guide's Return line shows the joints only), how long a 5 deg J1 move takes
  and whether `RobotMode()` shows 7 during it; whether J2 / J3 stay; whether
  the arm returns to the start joints within the 0.2 deg tolerance; how long
  `DisableRobot()` takes; whether the Dashboard and motion connections survive.
  The jog tool's limits (J1 / J4 only, 1..10 deg, ratio at most 30 %) are
  design choices, not measurements, and its speed is bounded only relative to
  the controller's own settings.

## 5. Interrupting a run

Ctrl+C ends the observation only: the tool sends nothing to the arm and it is
not an emergency stop. Use the arm's own emergency stop for that. A run that
ends early leaves `complete: false` in the record, with the reason in `notes`.

In the jog tool, Ctrl+C before the Enter ends the run with no control command
sent (only the read-only checks before the prompt had gone out). After
the Enter, and until the arm has reported disabled again (the RobotMode 4 after
the DisableRobot of step 8), it starts the tool's stop sequence (ResetRobot,
DisableRobot, polls, then EmergencyStop unless the arm showed itself disabled,
or in alarm with its reported joints within the stop threshold, and the Enable's
outcome is known), bounded by 15 s, during which a second Ctrl+C is ignored.
After that RobotMode 4 a fault or Ctrl+C adds no stop sequence (only the final read-only
queries follow). After the Enter a Ctrl+C
is taken before the tool's next send (within 0.1 s normally, about 6 s at most: the send
and reply timeouts of the command in progress), never in the middle of a send.
In both cases the record is then written (normally a few seconds); one more Ctrl+C
during that is deferred to its end, a second one ends it at once, with the record
possibly incomplete. A Ctrl+C after that gives exit 130 with the advice to check the
arm and the record; only while that advice is printed and while the process itself exits
after it is a Ctrl+C Python's own (a traceback, the record written). It is still not an
emergency stop: that is the button on the controller. The record's `stop`
section says what was sent, what the arm answered, and which `stop_state` was
reached; the printed guidance line says what the human should do next.
