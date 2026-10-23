"""Dobot MG400 TCP/IP text protocol: the read-only command subset and the reply parser.

Source: Dobot "TCP_IP Remote Control Interface Guide (4axis)", 2024-04-19
(github.com/Dobot-Arm/TCP-IP-Protocol-4AXis). Commands are ASCII ``Name(p1,p2,...)``;
every reply is ``ErrorID,{values},Name(params);`` where ErrorID 0 means the command
was received, and ``{values}`` may span several lines and nest (``GetErrorID``).

This module builds two groups of commands. ``ALLOWED`` are the six read-only commands of
the observation tool: four read state, two compute forward / inverse kinematics (guide
section 2.3); none of them enables, disables, moves or configures the arm. ``CONTROL`` are
the commands of the jog tool (``jog.py``, plan reviewed 2026-10-22): EnableRobot,
DisableRobot, ResetRobot, EmergencyStop on the Dashboard port and JointMovJ on the motion
port. A control command is built here like any other, but :func:`check_command` refuses it
unless the caller passes ``control=True``: the observation tool never does, so its
connections cannot send one even by mistake. There is no way to send free text: each
command is built from typed numbers, and :func:`check_command` re-checks the final text
before it is written.
"""
import json
import math
import re
from typing import List, NamedTuple, Optional, Sequence

ALLOWED = ("RobotMode", "GetErrorID", "GetPose", "GetAngle",
           "PositiveSolution", "InverseSolution")
#: commands that change the arm's state; refused unless check_command(..., control=True)
CONTROL = ("EnableRobot", "DisableRobot", "ResetRobot", "EmergencyStop", "JointMovJ")
#: the control commands that stop or de-energise the arm (never a motion)
STOP = ("ResetRobot", "DisableRobot", "EmergencyStop")
#: the only motion command, on port 30003; its speed / acceleration ratio is capped here
MOTION = ("JointMovJ",)
MAX_SPEED_RATIO = 30                # % of the controller's playback setting x global ratio

#: RobotMode values (guide section 2.3)
ROBOT_MODES = {1: "INIT", 2: "BRAKE_OPEN", 3: "POWER_STATUS", 4: "DISABLED", 5: "ENABLE",
               6: "BACKDRIVE", 7: "RUNNING", 8: "RECORDING", 9: "ERROR", 10: "PAUSE",
               11: "JOG"}
MODE_DISABLED = 4

_NUM = r"-?\d+\.\d{6}"
MAX_ABS = 1.0e6                 # no joint angle or coordinate of the MG400 comes near this
_J4 = rf"{_NUM},{_NUM},{_NUM},{_NUM}"
#: the exact text each allowed command may have (what the builders below produce)
_EXACT = {
    "RobotMode": re.compile(r"RobotMode\(\)"),
    "GetErrorID": re.compile(r"GetErrorID\(\)"),
    "GetAngle": re.compile(r"GetAngle\(\)"),
    # GetPose(User=0,Tool=0) is in the guide but the bench controller refused it
    # (ErrorID -30001, 2026-10-21), so only the bare form is allowed
    "GetPose": re.compile(r"GetPose\(\)"),
    "PositiveSolution": re.compile(rf"PositiveSolution\({_J4},0,0\)"),
    # InverseSolution(...,0,0,1,{j1,j2,j3,j4}) (isJointNear, JointNear) is in the guide but the
    # bench controller refused it (ErrorID -20000, parameter number, 2026-10-21): only the
    # six-parameter form, whose solution is the one nearest the arm's current joint angles
    "InverseSolution": re.compile(rf"InverseSolution\({_J4},0,0\)"),
    # control commands (jog tool only): no parameters, so no load or offset can be typed in
    "EnableRobot": re.compile(r"EnableRobot\(\)"),
    "DisableRobot": re.compile(r"DisableRobot\(\)"),
    "ResetRobot": re.compile(r"ResetRobot\(\)"),
    "EmergencyStop": re.compile(r"EmergencyStop\(\)"),
    # absolute joint target with the speed and acceleration ratio always given, equal, 1..30 %
    "JointMovJ": re.compile(rf"JointMovJ\({_J4},SpeedJ=(?P<s>[1-9]|[12]\d|30),AccJ=(?P=s)\)"),
}
_NAME_RE = re.compile(r"([A-Za-z]+)\(")


class MG400Error(RuntimeError):
    """Any MG400 protocol or communication problem"""


class MG400ProtocolError(MG400Error):
    """A reply that cannot be parsed or does not belong to the command sent"""


class MG400ConnectionError(MG400Error):
    """A Dashboard or feedback connection that could not be opened, or failed: refused,
    timed out, reset, or closed by the arm. ``op`` is "connect", "send" or "recv"; ``kind``
    what was seen: "eof", "refused", "reset", "aborted", "broken pipe", "timeout" or
    "other"; ``partial_reply`` whether bytes of an unfinished reply had arrived"""

    def __init__(self, text: str, label: Optional[str] = None, op: Optional[str] = None,
                 kind: Optional[str] = None, partial_reply: bool = False):
        super().__init__(text)
        self.label, self.op, self.kind, self.partial_reply = label, op, kind, partial_reply


class MG400CommandError(MG400Error):
    """A command refused by the controller (non-zero ErrorID)"""


class Command(NamedTuple):
    name: str
    text: str

    @property
    def control(self) -> bool:
        return self.name in CONTROL


def _number(value, what: str) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{what} must be a number: {value!r}")
    v = float(value)
    if not math.isfinite(v):
        raise ValueError(f"{what} must be finite: {value!r}")
    if abs(v) > MAX_ABS:
        raise ValueError(f"{what} out of range: {value!r}")
    return f"{v:.6f}"


def _joints(values: Sequence, what: str) -> List[str]:
    values = list(values)
    if len(values) != 4:
        raise ValueError(f"{what} needs 4 joint values, got {len(values)}")
    return [_number(v, f"{what} J{i + 1}") for i, v in enumerate(values)]


def check_command(text: str, name: Optional[str] = None, control: bool = False) -> str:
    """Exactly one allowed command in exactly the form its builder produces (finite
    numbers with six decimals, the fixed frame arguments, nothing before or after).
    ``name``, if given, must be the command's name. A ``CONTROL`` command passes only with
    ``control=True`` (the jog tool's connections); the observation tool never passes it.
    Returns the name; raises ValueError"""
    if not isinstance(text, str):
        raise ValueError(f"command must be text: {text!r}")
    m = _NAME_RE.match(text)
    if not m or m.group(1) not in ALLOWED + CONTROL:
        raise ValueError(f"command {text[:40]!r} is not allowed")
    found = m.group(1)
    if found in CONTROL and control is not True:
        raise ValueError(f"{found} changes the arm's state: refused on a read-only connection")
    if not _EXACT[found].fullmatch(text):
        raise ValueError(f"not the exact form of {found}: {text!r}")
    if name is not None and name != found:
        raise ValueError(f"command object named {name!r} carries {found}")
    for number in re.findall(_NUM, text):
        v = float(number)
        if not math.isfinite(v) or abs(v) > MAX_ABS:
            raise ValueError(f"number out of range in {text[:60]!r}: {number[:20]}")
    return found


def _build(name: str, params: str) -> Command:
    text = f"{name}({params})"
    check_command(text, name, control=name in CONTROL)
    return Command(name, text)


# -- control commands (jog tool only; a connection must be opened with control=True) ------
def enable_robot() -> Command:
    """``EnableRobot()`` without load or offset (the controller keeps its own setting)"""
    return _build("EnableRobot", "")


def disable_robot() -> Command:
    return _build("DisableRobot", "")


def reset_robot() -> Command:
    """``ResetRobot()``: stop the arm and clear the planned command queue (guide 2.1)"""
    return _build("ResetRobot", "")


def emergency_stop() -> Command:
    """``EmergencyStop()``: the arm powers off and alarms; the alarm must be cleared by the
    human (DobotStudio Pro) before the arm can be enabled again"""
    return _build("EmergencyStop", "")


def joint_mov_j(joints: Sequence, speed_ratio: int) -> Command:
    """``JointMovJ(j1,j2,j3,j4,SpeedJ=s,AccJ=s)``: absolute joint target (deg) with the
    speed and acceleration ratio ``s`` in 1..MAX_SPEED_RATIO (guide 3: the actual speed is
    this ratio x the playback setting x the global ratio)"""
    if isinstance(speed_ratio, bool) or not isinstance(speed_ratio, int):
        raise ValueError(f"speed ratio must be an integer: {speed_ratio!r}")
    if not 1 <= speed_ratio <= MAX_SPEED_RATIO:
        raise ValueError(f"speed ratio must be 1..{MAX_SPEED_RATIO}: {speed_ratio}")
    return _build("JointMovJ", ",".join(_joints(joints, "JointMovJ")
                                        + [f"SpeedJ={speed_ratio}", f"AccJ={speed_ratio}"]))


def robot_mode() -> Command:
    return _build("RobotMode", "")


def get_error_id() -> Command:
    return _build("GetErrorID", "")


def get_pose() -> Command:
    """``GetPose()``: the pose in the current global user / tool frames"""
    return _build("GetPose", "")


def get_angle() -> Command:
    return _build("GetAngle", "")


def positive_solution(joints: Sequence) -> Command:
    """Forward kinematics in user 0 / tool 0 (compute only)"""
    return _build("PositiveSolution", ",".join(_joints(joints, "PositiveSolution") + ["0", "0"]))


def inverse_solution(pose: Sequence) -> Command:
    """Inverse kinematics in user 0 / tool 0, the solution nearest the arm's current joint
    angles (compute only)"""
    pose = list(pose)
    if len(pose) != 4:
        raise ValueError(f"InverseSolution needs x, y, z, r, got {len(pose)} values")
    xyzr = [_number(v, f"InverseSolution {k}") for k, v in zip("xyzr", pose)]
    return _build("InverseSolution", ",".join(xyzr + ["0", "0"]))


class Reply(NamedTuple):
    error_id: int
    values: str            # the text between the outer braces, as received
    echo: str              # Name(params) as echoed
    raw: str


def _echo_parts(echo: str):
    m = re.match(r"^\s*([A-Za-z]+)\((.*)\)\s*$", echo, re.S)
    if not m:
        raise MG400ProtocolError(f"reply echo is not a command: {echo!r}")
    return m.group(1), m.group(2)


def check_echo(reply: Reply, command: Command) -> Optional[str]:
    """The echoed name must be the command's (case-insensitive). Parameters must match,
    except that an echo without parameters is accepted and reported (the guide's
    per-command Return lines omit them, e.g. GetPose). Returns a note or None"""
    name, params = _echo_parts(reply.echo)
    if name.lower() != command.name.lower():
        raise MG400ProtocolError(f"reply to {command.text!r} echoes {reply.echo!r}")
    _, sent = _echo_parts(command.text)
    if params.replace(" ", "") == sent:
        return None
    if params.strip() == "" and command.name not in MOTION:
        return "echo without parameters"
    # the guide's Return line for JointMovJ shows the joints only: the optional SpeedJ / AccJ
    # may be left out of the echo, but the joints must be there (a motion reply must name
    # its target)
    if command.name in MOTION and sent.startswith(params.replace(" ", "") + ",SpeedJ="):
        return "echo without optional parameters"
    raise MG400ProtocolError(f"reply to {command.text!r} echoes other parameters: {reply.echo!r}")


class ReplyReader:
    """Collects bytes from the Dashboard socket and returns complete replies.

    TCP may split one reply over several reads or join several; a reply ends at the
    first ``;`` outside braces and brackets after its echo's closing parenthesis.
    """

    def __init__(self, max_bytes: int = 65536):
        self._buf = ""
        self._max = max_bytes

    def feed(self, data: bytes) -> List[Reply]:
        self._buf += data.decode("ascii", "replace")
        if len(self._buf) > self._max:
            raise MG400ProtocolError("reply too long without a terminator")
        out = []
        while True:
            end = self._find_end(self._buf)
            if end < 0:
                return out
            raw, self._buf = self._buf[:end + 1], self._buf[end + 1:]
            out.append(parse_reply(raw))

    @staticmethod
    def _find_end(s: str) -> int:
        depth = 0
        for i, ch in enumerate(s):
            if ch in "{[(":
                depth += 1
            elif ch in "}])":
                depth -= 1
                if depth < 0:
                    raise MG400ProtocolError(f"unbalanced reply: {s!r}")
            elif ch == ";" and depth == 0:
                return i
        return -1

    @property
    def pending(self) -> str:
        return self._buf


def parse_reply(raw: str) -> Reply:
    """``ErrorID,{values},Name(params);`` -> Reply. Raises MG400ProtocolError"""
    text = raw.strip()
    if not text.endswith(";"):
        raise MG400ProtocolError(f"reply without terminator: {raw!r}")
    body = text[:-1]
    m = re.match(r"^\s*(-?\d+)\s*,\s*\{", body)
    if not m:
        raise MG400ProtocolError(f"reply does not start with ErrorID,{{: {raw!r}")
    start = m.end() - 1
    depth = 0
    close = -1
    for i in range(start, len(body)):
        if body[i] in "{[(":
            depth += 1
        elif body[i] in "}])":
            depth -= 1
            if depth == 0:
                close = i
                break
    if close < 0 or body[close] != "}":
        raise MG400ProtocolError(f"reply values are not closed: {raw!r}")
    rest = body[close + 1:].lstrip()
    if not rest.startswith(","):
        raise MG400ProtocolError(f"reply has no echo: {raw!r}")
    return Reply(int(m.group(1)), body[start + 1:close], rest[1:].strip(), raw)


def parse_numbers(values: str, count=None) -> List[float]:
    """``308.14,0.38,...`` -> floats (finite). ``count`` (an int or a tuple of allowed
    lengths) checks the length"""
    parts = [p.strip() for p in values.split(",")] if values.strip() else []
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        raise MG400ProtocolError(f"values are not numbers: {values!r}")
    if any(not math.isfinite(v) for v in nums):
        raise MG400ProtocolError(f"non-finite value: {values!r}")
    allowed = (count,) if isinstance(count, int) else count
    if allowed is not None and len(nums) not in allowed:
        raise MG400ProtocolError(f"expected {count} values, got {len(nums)}: {values!r}")
    return nums


def parse_robot_mode(values: str) -> int:
    nums = parse_numbers(values, 1)
    mode = int(nums[0])
    if mode != nums[0] or mode not in ROBOT_MODES:
        raise MG400ProtocolError(f"unknown RobotMode: {values!r}")
    return mode


#: GetErrorID: the controller's list and one per servo (the guide: four servos; the
#: bench reply had six lists in all), so at least five inner lists
MIN_ERROR_LISTS = 5


def parse_error_ids(values: str) -> list:
    """``[[...],[...],...]`` (controller, then one list per servo) -> nested lists of ints"""
    try:
        lists = json.loads(values)
    except ValueError:
        raise MG400ProtocolError(f"GetErrorID values are not a list: {values!r}")
    if not isinstance(lists, list) or not all(isinstance(x, list) for x in lists):
        raise MG400ProtocolError(f"GetErrorID values are not a list of lists: {values!r}")
    if len(lists) < MIN_ERROR_LISTS:
        raise MG400ProtocolError(f"GetErrorID has {len(lists)} lists, expected at least "
                                 f"{MIN_ERROR_LISTS}: {values!r}")
    for inner in lists:
        if not all(isinstance(v, int) and not isinstance(v, bool) for v in inner):
            raise MG400ProtocolError(f"GetErrorID ids are not integers: {values!r}")
    return lists


def no_alarm(error_lists: list) -> bool:
    """True only if every inner list (controller and each servo) is empty"""
    return all(len(inner) == 0 for inner in error_lists)
