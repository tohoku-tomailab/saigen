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
fourth (§3.6) is the first complete record. The jog tool's first run on the
arm (2026-10-30, controller version 2.8.5 typed, §3.7) ended at its first
check: the arm was in alarm, and nothing was sent to it. Its second run the
same day, after the alarms were cleared, is the first complete jog record
(§3.8): J1 +5 deg and back, 9.2 s from start to end. A repeat (§3.9) showed
the same things again; a J4 +5 deg run (§3.10) was reported by the
controller as a move that the operator did not see, five invocations ended
on ErrorID -1 because the controller's remote-control mode was no longer
TCP/IP (§3.11; three of them reused record folders and overwrote the
records of §3.8, §3.10 and an earlier -1 run), and a J4 +10 deg run with a
mark on the flange was seen to turn (§3.12; it overwrote a fourth record,
the -1 run before it).
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
disables it. Its first run on the arm ended before any command was sent
(§3.7); its second ran to the end (§3.8). The observation tool
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
6. In DobotStudio Pro: the arm disabled, no alarm, nothing on the flange, the
   remote-control mode TCP/IP (otherwise the first `RobotMode()` is answered
   ErrorID -1 and the tool ends, §3.11); note the playback joint speed /
   acceleration and the global speed ratio (the tool bounds only the ratio in
   its command); then close DobotStudio Pro.
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

### 3.7 Jog tool, first attempt: the arm was in alarm, nothing was sent

On 2026-10-30 the jog tool was started for the first time on the arm
(controller version `2.8.5` and the controller's playback speed, acceleration
and global ratio, all 50 %, typed with `--controller-version` and `--note`;
the record outside the synced tree). It ended 12 ms after opening the
feedback port, before the Dashboard port was opened: `commands: []`,
`armed: false`, `complete: false`, note `jog ended: feedback packet 0: mode 9,
error_status 1`. Nothing was sent to the arm, so none of §4's jog questions
was answered. What the two feedback packets (8 ms apart) showed:

- `robot_mode` 9 (alarm) and `error_status` 1 in both, `enable_status` 0,
  `velocity_ratio`, `xyz_velocity_ratio` and `xyz_acceleration_ratio` 50
  (the setting the operator typed), `speed_scaling` 97.0.
- `q_actual` about [1.58, -28.70, 17.54, 48.17] against [0.06, 10.28, 16.85,
  18.56] in all three records of 2026-10-21 and 22: the arm had been moved
  since then. How, and whether that is related to the alarm, is not in any
  record.
- `qd_actual` and `tcp_speed_actual` again not zero (§3.6).
- The alarm's ID is not in the record: the tool's first check is the feedback
  packet, and it ends the run on an alarm there before `GetErrorID()` could
  be sent. The observation tool waits for the first feedback packet the same
  way before opening the Dashboard port, so it would end there too. The alarm
  has to be read and cleared in DobotStudio Pro.

DobotStudio Pro's alarm panel, opened after the run, listed four controller
errors: code 67, level 5, "Joint2 exceeds negative limit", and code 85, level
0, three times, "Hardware emergency stop button press or software emergency
stop trigger" (its suggested fix: release the hardware button, or click the
software emergency stop again). The J2 reading of about -28.7 deg in the
packets is consistent with code 67; how the arm got there, and when the
emergency stop was pressed, are not recorded anywhere.

The tool's exit code for a run ended before the Enter is 2.

### 3.8 Jog tool, second attempt: the first complete jog record

Seventeen minutes later, after the alarms had been cleared in DobotStudio Pro
and J2 moved back inside its limit, the same command (new record folder) ran
to the end. **This record no longer exists on disk**: at 05:12 UTC a later
invocation reused its `--out` folder and the tool overwrote it (§3.11). The
numbers in this section are from a read of the record made before that,
kept verbatim in a text file next to the records
(`overwritten_records_read_before_overwrite.txt`); they can no longer be
checked against the raw packets. What that read showed:
`complete: true`, `armed: true`, `disabled_confirmed: true`, no
notes, no stop sequence, 65 commands, all three connections (feedback,
dashboard-1, motion) closed by the tool at the end. Start joints
[1.60, -14.68, 20.25, 48.17], target J1 +5 deg, SpeedJ = AccJ = 10. What
the record shows, by §4's questions:

- `ResetRobot()` on the disabled arm was answered `0,{}` after 166 ms;
  `RobotMode()` right after it was 4 and `GetErrorID()` empty; the joints
  after it differed from the start by at most 0.006 deg.
- `EnableRobot()` was answered `0,{}` after 1.24 s, and the next
  `RobotMode()` (1 ms later) was already 5; in the feedback, mode 5 and
  `enable_status` 1 began 2.9 s after the first packet. While enabling, the
  feedback's J3 rose by 0.10 deg for about 0.1 s (12 packets, from 20.25 to
  20.35 and back); the `GetAngle()` 0.5 s after the enable matched the start
  within 0.003 deg, so the tool's "enabling moved the arm" check passed.
- Both `JointMovJ(...,SpeedJ=10,AccJ=10)` were accepted on the motion port and
  answered `0,{},JointMovJ(<the full command as sent>);` after 156 and
  147 ms: the echo carries the whole command, SpeedJ and AccJ included (the
  guide's Return line shows the joints only).
- Each leg: `RobotMode()` 7 on the four polls that followed the move
  command, then 5; the arrival criterion (mode 5, then joints within 0.2 deg
  of the target) was met after 0.93 s on both legs, with 10 polls each. In
  the feedback, mode 7 lasted 94 packets (about 0.77 s) on each leg, and
  `q_target` led `q_actual` by at most 0.55 deg during the motion. The
  feedback's `velocity_ratio` field read 10 in exactly one packet at the
  start of each move (here the last mode-5 packet before each mode-7 run; in
  §3.9 the last mode-5 packet on the first leg and the first mode-7 packet
  on the second; in §3.12 the first mode-7 packet on both) and 50 otherwise;
  the two xyz ratios stayed 50.
- J2 and J3 stayed within 0.08 and 0.11 deg of their start over the whole
  recording (the J3 excursion above is at the enable, not in the moves);
  J4 within 0.011 deg. J1's extremes were 1.598 and 6.604 deg.
- `DisableRobot()` was answered `0,{}` after 1.05 s and the next
  `RobotMode()` was 4; the feedback showed mode 4 and `enable_status` 0 for
  the last 122 packets. After the disable, J1 settled from 1.603 to 1.639 deg
  (in the feedback and in the final `GetAngle()`): the final joints minus the
  start joints were [+0.037, -0.004, +0.008, -0.003] deg.
- Timeline: armed 2.0 s after the start; enabling 0.2 s after that; the two
  moves and their dwells 1.9 s to 6.1 s after the armed point; disabled
  7.1 s after it; record written and connections closed at 9.2 s.
- Feedback over the 9.2 s: 1122 packets, none rejected, no fault, step 7 to
  12 ms (median 8 ms), `pause_cmd_flag` 0 and `brake_status` 0 throughout,
  `run_queued_cmd` 0 and 1. `speed_scaling` read 97.0 in every packet, as in
  §3.7; what it holds is not explained by the guide.
- The Dashboard connection stayed up for the whole run (the tool opens only
  one, so the §3.4 ending was not exercised). `reply_s` were 0.5 ms to 1.24 s;
  the three above 0.15 s were ResetRobot, EnableRobot and DisableRobot.
- `qd_actual` again took values up to about 66 (in j4) with the arm still
  (§3.6).

The operator at the arm saw it move and return, with no unexpected motion
and no unusual sound, and DobotStudio Pro showed no alarm afterwards.

### 3.9 Jog tool, third run: the repeat

Seven minutes after §3.8, the same command again (record `jog_03`, start
joints [1.67, -14.69, 20.27, 48.17]) ran to the end with the same shape:
65 commands, ResetRobot 166 ms, EnableRobot 1.21 s then mode 5 at once,
both JointMovJ echoed in full (148 and 171 ms), mode 7 on four polls and 94
feedback packets per leg, arrival after 0.93 and 0.94 s, DisableRobot
1.03 s, 985 feedback packets, none rejected, no fault. The once-seen items
of §3.8 were seen again: J3 rose by 0.12 deg for 11 packets while enabling;
`velocity_ratio` read 10 in one packet at the start of each move (the last
mode-5 packet on the first leg, the first mode-7 packet on the second); after the disable
J1 settled by 0.023 deg (final minus start [+0.023, -0.002, +0.008, 0.000]);
`speed_scaling` 97.0 throughout.

### 3.10 Jog tool, fourth run: a J4 move the controller reported and the operator did not see

Forty seconds later, `--j1 0 --j4 5` (record `jog_04`). **This record was
overwritten too**, at 05:05 UTC by an invocation that reused the folder
(§3.11); the numbers here are from the read kept in the same text file as
§3.8's. That read showed a record of the
same shape as §3.8 and §3.9 (`complete: true`, 65 commands, no fault), and
by the controller's word J4 moved: `GetAngle()` went from 49.81 to 54.81 deg
over the four mode-7 polls of the first leg and back to 49.81 on the second;
the feedback's `q_actual` J4 followed `q_target` with a lag of up to
0.85 deg, mode 7 lasted 78 and 79 packets (about 0.64 s per leg, against 94
for the J1 legs), and `tool_vector_actual` r changed by 5 deg with it. J1,
J2 and J3 stayed within 0.03, 0.09 and 0.12 deg. The operator, standing at
the arm, saw nothing move. Two more things in the J4 readings:

- This run's start J4 was 49.81 deg, while §3.9's final `GetAngle()` 40 s
  earlier had read 48.17: a change of 1.65 deg with the arm disabled between
  the runs.
- About 0.3 s after the feedback showed mode 4 at the end of the run, the J4
  reading jumped from 49.82 to 51.13 deg within 70 ms (between the sampled
  packets at 7.97 s and 8.04 s of the read, every eighth packet), and the
  final `GetAngle()` read 51.13: final minus start J4 = +1.31 deg, far above
  the 0.00 to 0.04 deg of the other joints and of J4 in the earlier runs.

The tool's arrival criterion is the controller's reported joints, so it
cannot tell a reported move from a physical one. The operator then checked
the arm: the J4 motor is fitted. With nothing on the flange, a 5 deg turn
moves its edge by about 2 mm in 0.6 s (an estimate for an assumed flange
diameter of 40 mm; the flange was not measured), which may simply not have
been visible; a run with a mark on the flange settled it (§3.12). The two
J4 reading changes with the arm disabled are not explained by that.

### 3.11 ErrorID -1 to the first `RobotMode()`: five invocations, three records overwritten

Between 05:05 and 05:14 UTC the tool was started five times and every time
ended on the first Dashboard command: `RobotMode()` was answered
`-1,{},RobotMode();` and the tool stopped there as designed (`stopped by
MG400CommandError: RobotMode() -> ErrorID -1`, `armed: false`, `complete:
false`, nothing else sent, exit 2). The feedback was normal each time (mode
4, `error_status` 0, `enable_status` 0, `velocity_ratio` 50). It is the
first non-zero ErrorID on a read-only command in any record (§3.2's -20000
and -30001 were parameter refusals). The arguments of a run that ends
before its plan is shown are not in its record; the `--note` typed is.

| started (UTC) | folder written | reply after | feedback packets | note typed |
|---|---|---|---|---|
| 05:05:26 | `jog_04` (over §3.10's record) | 27 ms | 8 | speed settings only |
| 05:08:43 | `jog_05` | 72 ms | 15 | speed settings; tape mark on the flange |
| 05:09:14 | `jog_05` (over the row above) | 50.5 ms | 11 | the same |
| 05:12:46 | `jog_02` (over §3.8's record) | 15.6 ms | 5 | the same |
| 05:14:00 | `jog_06` | 28 ms | 8 | the same; DobotStudio Pro restarted then closed |

The rows of 05:08:43 and 05:14:00 are from reads made before those folders
were overwritten (the first by the next row, the second by §3.12's run);
the other three rows are what the folders hold now. Four bench records
were lost this way: the complete runs of §3.8 and §3.10 and the -1 run of
05:08:43 to rows of this table, and the -1 run of 05:14:00 to §3.12's
run. The joints in the feedback were [-0.31, -19.77, 20.30, -28.90] in
every row (J1 -0.29 in the first) against [1.69, -14.70, 20.29, 51.13] at
the end of §3.10, so J1, J2 and J4 had been moved between §3.10 and the
first row (J4 by 80 deg), by what is not in any record. The operator
restarted DobotStudio Pro, disconnected it and closed it before the last
row, so the -1 did not depend on DobotStudio Pro being open and persisted
across its restart. The operator then found the cause in DobotStudio Pro:
its remote-control mode was no longer TCP/IP. With it set back to TCP/IP,
the next run went through (§3.12). So ErrorID -1 to a read-only query is
what the controller answers when TCP/IP remote control is off; the feedback
port keeps streaming regardless, so the tool's feedback check cannot see
it.

The tool wrote into the reused `--out` folders without a warning. Since
this day it refuses a `--out` folder that already holds files (exit 2,
before anything is opened); a new folder name per run was the rule before
and is enforced now.

### 3.12 Jog tool, the run of 05:17:47 UTC: J4 +10 deg, the mark on the flange seen to turn

Four minutes after the last -1 of §3.11, with the remote-control mode back on
TCP/IP, `--j1 0 --j4 10` ran to the end (record in the `jog_06` folder,
started 05:17:47 UTC; `complete: true`, 65 commands, no fault, 1589
feedback packets over 13.0 s, of which 5.9 s before the Enter). The
operator saw the tape mark on the flange turn (the return is the
controller's word, below, not reported as seen). By the
controller: J4 from -27.94 to -17.94 deg and back, mode 7 on four polls and
97 feedback packets per leg (about 0.8 s, against 78 and 79 for 5 deg),
arrival after 0.93 and 0.94 s, `q_actual` J4 lagging `q_target` by up to
1.16 deg during the motion; J1, J2, J3 within 0.02, 0.11 and 0.12 deg (the
J3 excursion while enabling again, 0.12 deg over 9 packets). ResetRobot
170 ms, EnableRobot 1.21 s, DisableRobot 1.03 s, the same as every run.

J4's reading again changed with the arm disabled: it was about -28.90 to
-28.91 at the start of every -1 invocation of §3.11 (-28.91 in the 05:14:00
read) and -27.94 at the start of this one (0.97 deg over
four minutes), and after the disable the final `GetAngle()` read -26.83,
1.11 deg above the start (1.31 in §3.10). Final minus start:
[+0.003, -0.003, +0.009, +1.112].

## 4. Open points

- Whether the first-connection ending repeats beyond these three runs (each
  time after a second connection was opened), and whether it happens with
  the arm enabled: a later control stage cannot assume a second monitoring
  connection on the Dashboard port.
- The controller version, typed into the jog run of 2026-10-30 (`2.8.5`,
  §3.7); not yet into a complete observation record.
- The alarm of 2026-10-30 (§3.7): the `GetErrorID()` reply of that run was
  never taken (DobotStudio Pro's panel showed codes 67 and 85 afterwards);
  its cause, when it arose, and whether the arm's changed joints since
  2026-10-22 are related, are open. When the first feedback packet
  shows an alarm, neither tool records a `GetErrorID()`; whether they should
  send that read-only query before ending is a change to the tools and needs
  a reviewed plan like any other.
- The mapping of the 6-element feedback arrays to the 4-element replies. The
  tool always reports its comparison as "mapping unverified"; checking it, for
  example at a second pose, is a human step still to be planned.
- The meaning of the non-zero `qd_actual` / `tcp_speed_actual` fields on a
  disabled arm (§3.6).
- From the complete jog records (§3.8, §3.9, §3.10, §3.12: J1 +5 deg twice,
  J4 +5 deg once, J4 +10 deg once, all at a 10 % ratio with the controller
  at 50 %; two of the four records survive on disk): the 0.10 to 0.12 deg
  J3 excursion while enabling; the one-packet `velocity_ratio` 10 at the
  start of each move; the J1 reading settling after the disable (0.02 to
  0.04 deg in the J1 runs; in the J4 runs final minus start J1 was 0.000 and
  0.003 deg); the constant
  `speed_scaling` 97.0. Each was seen in every run, none is explained by
  the guide. J1 was moved only by 5 deg, nothing at another ratio. The jog
  tool's limits (J1 / J4 only, 1..10 deg, ratio at most 30 %) are design
  choices, not measurements, and its speed is bounded only relative to the
  controller's own settings. The stop sequence has not run on the arm.
- J4 (§3.10, §3.12): the 5 deg move was not seen, the 10 deg move with a
  mark on the flange was. The J4 reading changes with the arm disabled,
  by 1.65 and 0.97 deg between runs and by 1.31 and 1.11 deg right after a
  disable (the other joints by at most 0.04 deg), are unexplained; a J4 run
  therefore ends with a J4 reading about 1 deg from its start, which the
  tool records and does not judge. The tool's reported arrival is the
  controller's word only.
- Four records of 2026-10-30 were overwritten by reused `--out` folders
  (§3.11: three during the -1 retries, one by the run of §3.12) and survive
  only as reads made before that; the tool now refuses
  a folder that holds files. The observation tool still does not.

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
