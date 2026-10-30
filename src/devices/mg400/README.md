# Dobot MG400 (TCP/IP): observation tool and jog tool

The MG400 is **not** a SAIGEN robot type: no flow, GUI block or `config.yaml` entry can
use it. This package holds two human-run tools. `observe.py` observes the arm without
changing its state. `jog.py` (section "Jog tool" below) enables the arm, rotates J1 and /
or J4 by a small angle and back, and disables it; it is the first tool here that changes
the arm's state, built to a reviewed plan, and its real run is the human's. The sections
up to "Records" describe the observation tool.

What the first bench runs showed, and what is still open, is in
[`docs/mg400-commissioning.md`](../../../docs/mg400-commissioning.md).

Protocol reference: Dobot "TCP_IP Remote Control Interface Guide (4axis)", 2024-04-19
([Dobot-Arm/TCP-IP-Protocol-4AXis](https://github.com/Dobot-Arm/TCP-IP-Protocol-4AXis)).

## What the tool sends

Only these commands, on the Dashboard port 29999, each built from typed numbers (there is
no way to send free text, and the final text is checked before it is written):

| Command | Purpose |
|---|---|
| `RobotMode()` | state (4 = disabled) |
| `GetErrorID()` | alarms (controller and each servo) |
| `GetPose()` | flange pose in the current global user / tool frames |
| `GetAngle()` | joint angles |
| `PositiveSolution(j1,j2,j3,j4,0,0)` | the controller's forward kinematics (compute only) |
| `InverseSolution(x,y,z,r,0,0)` | its inverse kinematics, the solution nearest the arm's current joints (compute only) |

Port 29999 also accepts commands that change the arm's state (EnableRobot, DisableRobot,
ClearError, EmergencyStop ...). The allowlist in `protocol.py`, not the port, keeps the
tool read-only: the control commands of the jog tool exist in `protocol.py` too, but
`check_command` refuses them unless the connection was opened with `control=True`, which
`observe.py` never does (a test reads its source to make sure). The feedback port 30004 is
only read. The observation tool never opens the motion port 30003.

The guide also documents `GetPose(User=0,Tool=0)`. The bench controller refused it
(`-30001,{},GetPose(User=0,Tool=0);`, "the type of the first parameter is incorrect"), so
the tool no longer sends it, and its allowlist no longer admits it.

The same holds for the guide's `InverseSolution(x,y,z,r,0,0,1,{j1,j2,j3,j4})` (solution
nearest the given joints): the bench controller answered `-20000` ("parameter number
error"). The tool sends the six-parameter form instead, whose solution is the one nearest
the arm's current joints, so a pair's inverse solution may be another branch than its
joints; the record says so (`kinematics_note`). The bench controller answered the
six-parameter form with four values on the fourth run (`docs/mg400-commissioning.md`
§3.6).

The observation ends, sending nothing more, as soon as any RobotMode / GetErrorID reply
(on either connection) or any feedback packet shows a mode other than 4 (disabled), an
alarm or an out-of-range value; when no valid feedback packet arrives within 2 s or the
last one is older than 0.5 s, or when two valid packets were ever more than 0.5 s apart
(kept even if the feed resumes); on a protocol error (non-zero ErrorID, wrong echo, wrong
number of values); on a connection error (refused, no reply within 3 s, reset, closed by
the arm); or when the record cannot be saved (the command is then not sent). The state is
checked before every compute command. That is the arm's state when read; the tool does
not guarantee it afterwards.

The one exception is the two-connection step. After a 1 s idle wait and a control
`RobotMode()` on the first Dashboard connection, the tool opens a second one, waits 0.2 s
and sends `RobotMode()` on the first (the probe). On the second bench run the first
connection's end was detected right after the second was opened (end of stream, no reply).
If the probe meets exactly that (an end of stream with no reply bytes, or a reset by the
peer; not a timeout, not a local abort, not after part of a reply), and no save of the run
has failed, and the record of it is saved, the tool closes the first connection, checks the
state on the second and continues there alone. That is recorded as an observation of the
run, not as proof that the controller serves only one client. Any other outcome ends the
observation as above.

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
  every connection with the time it was opened and the time its end was detected (not the
  time the arm closed it), the two-connection step saved after each of its parts (control,
  probe with the time since the connection's last reply, outcome, result), the final
  feedback statistics over the whole recording, the
  kinematics pairs, why the observation ended, the tool's source hash, and `complete`
  (false for an interrupted or ended run; the file is rewritten after each command).
  Each save writes `report.json.tmp` and renames it; a refused rename (a sync client or a
  scanner holding the file) is retried for about 1.5 s. If saving still fails, the run
  ends with "record error" (exit 1) and the message says which file holds the latest
  record. A save failure while the run is already ending for another reason is added to
  that error's line ("record also failed"), never in its place. When the folder is under
  OneDrive, the tool suggests `--out` with a folder outside it.
- `feedback.bin`: the raw 1440-byte packets back to back, `feedback_times.csv`: their
  sequence number, byte offset, receive times and reject reason, `feedback.csv`: every
  parsed field with non-finite and out-of-range flags and the packet TimeStamp minus the PC
  receive time.

Feedback fields are read at the offsets of the guide's table only (section 4). The
comparison of the 6-element feedback arrays with the 4-element replies is reported as
"mapping unverified"; the difference between the packet TimeStamp and the PC clock is
recorded but is not a latency (the clocks are not synchronised). The kinematics pairs are
the controller's internal model, not a measurement of the arm's accuracy.

## Jog tool

`python -m src.devices.mg400.jog` enables the arm, moves it by a small rotation of J1
and / or J4 with `JointMovJ`, moves it back to the start joints and disables it. **The arm
moves.** The whole arm and its cables swing through the arc (10 deg at a 300 mm radius is
about 52 mm at the flange). It is not a robot type and no flow can use it.

```bash
python -m src.devices.mg400.jog --dry-run                 # the command sequence; connects to nothing
python -m src.devices.mg400.jog --mock [--j1 5 --j4 0 --speed 10]
python -m src.devices.mg400.jog <arm IP> --controller-version <V> --note "<playback speed, global ratio>" [--j1 5 --j4 0 --speed 10]
```

What it sends, each built from typed numbers and checked against an exact form before it
is written (no free text, no parameters that are not shown here):

| Port | Command | When |
|---|---|---|
| 29999 | five of the observation tool's read-only commands: `RobotMode()`, `GetErrorID()`, `GetAngle()`, `GetPose()`, `PositiveSolution(...)` (`InverseSolution` is not used) | preconditions, polls, final state |
| 29999 | `ResetRobot()` | after the Enter, before enabling (clears a queue another client may have left); in the stop sequence |
| 29999 | `EnableRobot()` (no load, no offset) | once |
| 29999 | `DisableRobot()` | once at the end; in the stop sequence |
| 29999 | `EmergencyStop()` | only in the stop sequence: unless the arm showed itself disabled (mode 4 held), or in alarm with its reported joints within the stop threshold, **and** the Enable's outcome is known (never sent, refused, or RobotMode 5 was read after it) |
| 30003 | `JointMovJ(j1,j2,j3,j4,SpeedJ=s,AccJ=s)` | exactly twice: to the target, back to the start |

Limits: `--j1` and `--j4` are each 0 or 1..10 deg in size and not both 0, and `--speed`
(SpeedJ = AccJ) is 1..30 % (default 10), both checked before any connection is opened; J2
and J3 never change; the start and the target J1 must be within ±150 deg and J4 within
±170 deg, checked once the start joints have been read, before the prompt.
The tool bounds the ratio in the command, **not the speed**: the actual speed is that
ratio times the controller's playback joint speed times its global speed ratio (guide
section 3.1), which the tool neither reads nor sets. Note both in DobotStudio Pro before
closing it and pass them with `--note`; they are recorded as typed.

Sequence: feedback and Dashboard open; `RobotMode()` must be 4 and `GetErrorID()` empty;
`GetAngle()` gives the start, and the controller must accept `PositiveSolution(target)`
(a refusal ends the run before anything is enabled); the plan is printed
and the human presses **Enter on a terminal** (an empty line; anything else, a pipe or no
terminal ends the run with nothing changed; `--mock` skips the prompt because the fake is
in-process). Then `ResetRobot()`, and the state, alarms and start joints are checked again;
`EnableRobot()` and `RobotMode()` polls until 5 (10 s); 0.5 s settle, the joints must not
have moved; `JointMovJ` to the target and polls alternating `RobotMode()` / `GetAngle()`
every 0.1 s until a RobotMode 5 is followed by a GetAngle within 0.2 deg of the target (at
least 0.3 s after the JointMovJ's reply, 20 s after it at most; J2 / J3 further than 1 deg from the start is a
fault); 1 s dwell; the same back to the start; `DisableRobot()` and polls until 4 (5 s);
`RobotMode() GetErrorID() GetAngle() GetPose()`. Feedback packets are judged against the
set of modes the current phase allows (with a 0.3 s grace for packets of the previous
state); a packet or reply outside it, an alarm, a stale feed or a failed save is a fault.

**Stop sequence.** Any fault, protocol or connection error, record error or Ctrl+C after
the Enter (and before the final disable was confirmed) runs it on the main thread, bounded
by 15 s, with a second Ctrl+C ignored meanwhile: `ResetRobot()`, `DisableRobot()`, then
`RobotMode()` / `GetAngle()` polls for up to 3 s. A RobotMode 4 must then **hold**: another
4 read at least 1 s later with no other mode read between (shown by replies, not by elapsed
time: if the replies stop coming, the hold is unconfirmed); if another mode follows,
`DisableRobot()` is sent once more and the check repeats. Whether the arm's Enable is
accounted for also matters. Its outcome is **known** when it was never sent, when the
controller refused it (ErrorID other than 0), or when RobotMode 5 was read after it. When
it was acknowledged, or its reply was lost, and 5 was never read, the Enable may still take
effect after the tool is gone, so a mode 4 that holds now proves nothing and the sequence
ends in `EmergencyStop()` regardless. Outcomes, each with a line of guidance printed and
recorded:

| `stop_state` | Meaning |
|---|---|
| `reset_disable` | mode 4 held and the Enable's outcome is known: the arm reported disabled. Check it. |
| `alarm_still` | mode 9, the Enable's outcome known, and at least three GetAngle replies over 0.5 s whose range stayed within 0.05 deg: in alarm, and the reported joints stayed within the stop threshold; no EmergencyStop. Check the arm, then clear the alarm in DobotStudio Pro. |
| `emergency_stop` | anything else (mode 7 or 5 persisting, 9 with the joints changing, the hold not shown, no usable reply, the Enable's outcome unknown): `EmergencyStop()` was sent and acknowledged. The arm should be powered off and in alarm; if it still moves, press the emergency-stop button. |
| `unconfirmed` | the EmergencyStop could not be sent or was not acknowledged, or the 15 s ran out first. Press the emergency-stop button unless the arm is clearly at rest. |

The Dashboard connection is reused for the stop only if it is clean (no request without
its reply, no partial reply); otherwise, and after any failure, a new one is opened (at
most two). Once the stop sequence owns a connection, that connection may carry every stop
command including `EmergencyStop()`, which is never sent during normal operation. A refused
stop command is recorded and the sequence goes on; every send and reply is bounded by the
15 s total.

**Ctrl+C is not an emergency stop.** Before the Enter it ends the run with no control
command sent (only the read-only checks before the prompt had gone out).
After it, the tool takes the Ctrl+C before its next send (within 0.1 s normally; at most
the send and reply timeouts of the command in progress, about 6 s; never in the middle of
a send, and no second command goes out in between) and starts the stop sequence
above, until the arm has reported disabled again (the RobotMode 4 after the DisableRobot
of step 8); from then on a fault or Ctrl+C adds no stop sequence (only the final read-only
queries follow), and the message says so.
A further Ctrl+C during the stop sequence is ignored. After the stop sequence, or after
that RobotMode 4, the tool writes its record (normally a few seconds: the I/O thread's
drain of 5 s at most, the feedback thread's stop of 2 s at most, the final save); one
Ctrl+C during that is deferred to its end (exit 130), a second one ends it at once, and
the record may then be incomplete (the message says so). A Ctrl+C after the record is
written gives exit 130 too (one only counted during the clean-up of a run that had a
fault leaves the fault's exit 1); the record is complete either way. The last moments,
while the tool prints that advice and while the process itself exits after it, are
Python's own: a Ctrl+C there shows a traceback, with the record written. Two Ctrl+C are therefore the way
out of a clean-up stuck on a disk or a terminal. The emergency stop is the button on the
controller: the human must have it within reach, standing at the arm, for the whole run.

Exit codes: 0 complete (final mode 4, no alarm, every save confirmed); 2 ended before the
Enter (precondition, limits, prompt, record error, or a `--out` folder that already holds
files when the tool starts: each run gets a new folder; two runs started into the same
empty folder at once are not guarded against); 1 otherwise (a fault after the Enter:
the stop sequence ran and its state is printed, unless the arm had already reported
disabled at step 8, in which case no stop sequence was added and the message says to check the
arm anyway; or the arm's sequence completed but the background I/O reported errors, or
its thread did not finish within 5 s); 130 Ctrl+C (including one seen after the last
command, during or after the clean-up, and reported at the end); 2 also when the process's SIGINT handler was not set from Python (the tool
could not restore it, so it does not run). `complete: true` in the record says
the arm's sequence ran to the end, not that the run succeeded: read the exit code and, in
`report.json`, `io_thread_errors`, `io_thread_drained`, `notice_errors`, `cleanup_errors`
(a failure or interrupt in the clean-up), `ctrl_c` (how many, in which phase) and
`cleanup_cut_short`. `io_thread_errors` lists what the thread had reported when the
clean-up checked it: when `io_thread_drained` is false a later error may be missing, and
the unfinished drain is itself the fault. The record `logs/mg400_jog/<UTC time>/` has the
observation tool's files plus `jog` (start, target, speed ratio, phases with times, the
two legs, enable and disable times, final minus start joints), `stop` (the sequence step by
step, every mode and angle read, `enable_result`, `stop_state`, the guidance) and
`note_typed`. After the Enter, saves and notices run on a background thread so a stuck
disk or a blocked terminal cannot delay the stop commands; a save failure is still a fault.

The address of a real run must be a numeric IP literal; every connection, including the
stop sequence's, connects to it directly with a timeout and no name lookup. `--mock`
ignores the address and connects to the fake on 127.0.0.1 only (a test asserts it).

Preconditions of a real run (the human's): `--mock` passed; the controller's remote-control
mode TCP/IP (set in DobotStudio Pro; otherwise the first `RobotMode()` is answered ErrorID
-1 and the tool ends before enabling); DobotStudio Pro closed and no
other PC or program sending to the arm; nothing on the flange; the arc clear for the whole
arm and its cables; the speed settings noted; the controller version typed; the e-stop
button within reach. The agent never runs it (CLAUDE.md rule 1).
