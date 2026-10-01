"""Jog the MaxArm in small steps and record named positions (human-run tool).

**Moves the arm** (CLAUDE.md rule 1): run it yourself, in a terminal, with the
arm's reach clear. Every move goes through the safety layer (``MaxArmRobot`` with
the robot's workspace validator: box, radius and the arm's inverse kinematics),
so a target outside the workspace or one the arm cannot reach is refused before
anything is sent. Ctrl-C is the emergency stop (STOP to all servos).

Why jogging instead of moving the arm by hand with the servos unloaded: when a
bus servo is loaded again it jumps to its last commanded target, not to where it
was pushed. Jogging keeps every pose commanded.

Usage (from the repository root)::

    python -m src.devices.maxarm.teach                 # port / workspace from config.yaml (robot 4)
    python -m src.devices.maxarm.teach --mock          # practise without hardware
    python -m src.devices.maxarm.teach --out my_cell_positions.json --step 5

Commands at the ``maxarm>`` prompt (mm; ``step`` is the default jog size)::

    x+ x- y+ y- z+ z-     jog one axis by the step        x+5  z-2.5   jog by a given amount
    goto X Y Z            move to absolute coordinates    step N       set the jog size
    turn DEG              rotate the base by DEG (+ towards +x), same radius and height
    pos                   re-read the pose from the arm   top          highest reachable z here
    home                  go_home (config home_position)  pump on|off  suction
    save NAME             record the current pose         del NAME     forget a position
    list                  show recorded positions         help / quit

Recorded positions are written after every ``save`` to the ``--out`` file
(JSON: name -> [x, y, z], with the date, port and workspace used). Paste the
coordinates into a flow's ``move_xyz`` steps or into ``home_position``.
"""

import argparse
import asyncio
import json
import logging
import math
import os
import re
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from src.devices.maxarm import kinematics

logger = logging.getLogger(__name__)

DEFAULT_OUT = "maxarm_positions.json"     # gitignored: lab-specific
DEFAULT_STEP_MM = 10.0
MAX_STEP_MM = 50.0
MAX_TURN_DEG = 90.0
AXES = ("x", "y", "z")
_JOG_RE = re.compile(r"^([xyz])([+-])\s*([0-9]*\.?[0-9]*)$")
_NAME_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]{0,39}$")

HELP = """commands (mm):
  x+ x- y+ y- z+ z-   jog one axis by the step (x+5, z-2.5: by that amount)
  goto X Y Z          absolute move          step N        jog size (0.1..%g)
  turn DEG            rotate the base by DEG (+ towards +x; same radius and height)
  pos                 re-read the arm pose   top           highest reachable z at this (x, y)
  home                go_home                pump on|off   suction
  save NAME           record current pose    del NAME      forget it
  list                recorded positions     help / quit
Ctrl-C = emergency stop""" % MAX_STEP_MM


class CommandError(ValueError):
    """The typed line is not a valid command (nothing was sent)."""


def parse_command(text: str) -> Tuple[str, List[Any]]:
    """One prompt line -> (kind, args). Raises CommandError on anything unknown.

    kinds: jog(axis, delta) goto(x, y, z) step(mm) pos top home pump(bool)
    save(name) del(name) list help quit
    """
    t = text.strip()
    if not t:
        raise CommandError("empty line")
    low = t.lower()
    m = _JOG_RE.match(low.replace(" ", "")) if low[0] in AXES and len(low) > 1 else None
    if m:
        axis, sign, amount = m.group(1), m.group(2), m.group(3)
        delta = float(amount) if amount not in ("", ".") else None
        if delta is not None and not (0 < delta <= MAX_STEP_MM):
            raise CommandError("jog amount must be 0..%g mm" % MAX_STEP_MM)
        return "jog", [axis, (1.0 if sign == "+" else -1.0), delta]
    parts = t.split()
    cmd = parts[0].lower()
    args = parts[1:]
    if cmd == "goto":
        if len(args) != 3:
            raise CommandError("goto needs X Y Z")
        try:
            return "goto", [float(a) for a in args]
        except ValueError:
            raise CommandError("goto needs three numbers")
    if cmd == "turn":
        if len(args) != 1:
            raise CommandError("turn needs an angle in degrees")
        try:
            deg = float(args[0])
        except ValueError:
            raise CommandError("turn needs a number")
        if not (0 < abs(deg) <= MAX_TURN_DEG):
            raise CommandError("turn angle must be within +-%g deg and not 0" % MAX_TURN_DEG)
        return "turn", [deg]
    if cmd == "step":
        if len(args) != 1:
            raise CommandError("step needs one number")
        try:
            v = float(args[0])
        except ValueError:
            raise CommandError("step needs a number")
        if not (0.1 <= v <= MAX_STEP_MM):
            raise CommandError("step must be 0.1..%g mm" % MAX_STEP_MM)
        return "step", [v]
    if cmd == "pump":
        if len(args) != 1 or args[0].lower() not in ("on", "off", "1", "0"):
            raise CommandError("pump on|off")
        return "pump", [args[0].lower() in ("on", "1")]
    if cmd in ("save", "del"):
        if len(args) != 1 or not _NAME_RE.match(args[0]):
            raise CommandError("%s needs a name (letters, digits, _ . -; max 40)" % cmd)
        return cmd, [args[0]]
    if cmd in ("pos", "top", "home", "list", "help", "quit", "exit", "q") and not args:
        return {"exit": "quit", "q": "quit"}.get(cmd, cmd), []
    raise CommandError("unknown command %r (type help)" % t)


def turn_target(pose: List[float], deg: float) -> List[float]:
    """Target of a base rotation by ``deg`` (positive towards +x) at the same radius and z."""
    x, y, z = (float(v) for v in pose[:3])
    r = math.hypot(x, y)
    theta = math.degrees(math.atan2(y, x)) + deg
    return [r * math.cos(math.radians(theta)), r * math.sin(math.radians(theta)), z]


def jog_target(pose: List[float], axis: str, sign: float, delta: float) -> List[float]:
    """Target of a jog from ``pose`` (the last commanded pose)."""
    i = AXES.index(axis)
    target = [float(v) for v in pose]
    target[i] += sign * delta
    return target


class PositionStore:
    """Named positions, written to a JSON file after every change."""

    def __init__(self, path: str, meta: Optional[Dict[str, Any]] = None):
        self.path = path
        self.meta = dict(meta or {})
        self.positions: Dict[str, List[float]] = {}
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            for name, xyz in (data.get("positions") or {}).items():
                self.positions[str(name)] = [float(v) for v in xyz[:3]]

    def save(self, name: str, pose: List[float]) -> None:
        self.positions[name] = [round(float(v), 1) for v in pose[:3]]
        self.write()

    def delete(self, name: str) -> bool:
        if name not in self.positions:
            return False
        del self.positions[name]
        self.write()
        return True

    def write(self) -> None:
        data = {"format": "saigen-maxarm-positions/1",
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                **self.meta,
                "positions": self.positions}
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        os.replace(tmp, self.path)

    def table(self) -> str:
        if not self.positions:
            return "(no positions recorded)"
        width = max(len(n) for n in self.positions)
        return "\n".join("  %-*s  x=%7.1f  y=%7.1f  z=%7.1f" % (width, n, p[0], p[1], p[2])
                         for n, p in self.positions.items())


def robot_from_config(robot_id: int, mock: bool, port_override: Optional[str] = None):
    """Build the MaxArm safety wrapper (real or mock) from config.yaml robot ``robot_id``."""
    from src import config as lab_config
    from src.flow.experiment_session import robot_type, validator_for_robot
    ports = lab_config.get_robot_ports()
    entry = ports.get(robot_id) or ports.get(str(robot_id)) or {}
    if robot_type(entry) != "maxarm":
        raise SystemExit("robot %d is not a MaxArm (type: maxarm) in config.yaml" % robot_id)
    validator = validator_for_robot(robot_id, robot_ports=ports)
    cfg = {k: v for k, v in entry.items() if k not in ("type", "maxarm_port", "workspace")}
    if mock:
        from src.devices.safety.mock_maxarm import MockMaxArmRobot
        return MockMaxArmRobot(validator, **cfg), "MOCK", validator
    from src.devices.safety.maxarm_robot import MaxArmRobot
    port = port_override or entry.get("maxarm_port") or ""
    if not port:
        raise SystemExit("robots.%d.maxarm_port is empty in config.yaml (or pass --port)" % robot_id)
    return MaxArmRobot(port, validator, **cfg), port, validator


class TeachSession:
    """Runs prompt commands against a MaxArm safety wrapper (real or mock)."""

    def __init__(self, robot, store: PositionStore, step_mm: float = DEFAULT_STEP_MM,
                 out: Callable[[str], None] = print):
        self.robot = robot
        self.store = store
        self.step = float(step_mm)
        self.out = out

    def pose(self) -> List[float]:
        return [float(v) for v in self.robot.get_current_position()]

    def show_pose(self, prefix: str = "pose") -> None:
        x, y, z = self.pose()
        self.out("%s: x=%.1f y=%.1f z=%.1f" % (prefix, x, y, z))

    async def _move(self, target: List[float]) -> bool:
        from src.devices.safety.validators import WorkspaceViolationError
        try:
            await self.robot.move_xyz(*target)
        except WorkspaceViolationError as e:
            self.out("REFUSED (nothing sent): %s" % e)
            return False
        self.show_pose("now")
        return True

    async def run_command(self, line: str) -> bool:
        """Execute one line; returns False when the session should end."""
        try:
            kind, args = parse_command(line)
        except CommandError as e:
            self.out("? %s" % e)
            return True
        if kind == "quit":
            return False
        if kind == "help":
            self.out(HELP)
        elif kind == "step":
            self.step = args[0]
            self.out("step = %.1f mm" % self.step)
        elif kind == "jog":
            axis, sign, delta = args
            target = jog_target(self.pose(), axis, sign, delta if delta is not None else self.step)
            await self._move(target)
        elif kind == "goto":
            await self._move(list(args))
        elif kind == "turn":
            target = turn_target(self.pose(), args[0])
            self.out("base %+.1f deg -> target (%.1f, %.1f, %.1f)" % (args[0], *target))
            await self._move(target)
        elif kind == "pos":
            # the wrapper re-reads before the next move; ask the client directly when real
            client = getattr(self.robot, "client", None)
            if client is not None:
                st = client.status()
                self.out("arm reads: x=%.1f y=%.1f z=%.1f pulses=%s pump=%s stopped=%s"
                         % (st.x, st.y, st.z, st.pulses, st.pump, st.stopped))
            self.show_pose("commanded")
        elif kind == "top":
            x, y, z = self.pose()
            v = self.robot.workspace_validator
            top = kinematics.highest_reachable_z(x, y, v.z_min, min(v.z_max, kinematics.Z_MAX))
            self.out("highest reachable z at (%.1f, %.1f): %s" % (x, y, "none" if top is None
                                                                 else "%.0f mm" % top))
        elif kind == "home":
            from src.devices.safety.validators import WorkspaceViolationError
            try:
                await self.robot.go_home()
            except (WorkspaceViolationError, RuntimeError) as e:
                self.out("REFUSED: %s" % e)
            else:
                self.show_pose("now")
        elif kind == "pump":
            await self.robot.suction(args[0])
            self.out("pump %s" % ("on" if args[0] else "off"))
        elif kind == "save":
            self.store.save(args[0], self.pose())
            self.out("saved %s -> %s" % (args[0], self.store.positions[args[0]]))
        elif kind == "del":
            self.out("deleted %s" % args[0] if self.store.delete(args[0]) else "no such position")
        elif kind == "list":
            self.out(self.store.table())
        return True


async def interactive(session: TeachSession, prompt: str = "maxarm> ",
                      readline: Optional[Callable[[str], str]] = None) -> None:
    readline = readline or (lambda p: input(p))
    while True:
        try:
            line = await asyncio.to_thread(readline, prompt)
        except EOFError:
            return
        if not await session.run_command(line):
            return


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Jog the MaxArm and record positions (human-run; "
                                             "moves the arm).")
    ap.add_argument("--robot", type=int, default=4, help="robot id in config.yaml (default 4)")
    ap.add_argument("--port", default=None, help="override robots.<id>.maxarm_port")
    ap.add_argument("--mock", action="store_true", help="no hardware: practise the commands")
    ap.add_argument("--out", default=DEFAULT_OUT, help="positions file (default %s)" % DEFAULT_OUT)
    ap.add_argument("--step", type=float, default=DEFAULT_STEP_MM, help="initial jog size (mm)")
    return ap


async def _main(args) -> int:
    robot, port, validator = robot_from_config(args.robot, args.mock, args.port)
    store = PositionStore(args.out, meta={"robot_id": args.robot, "port": port,
                                          "workspace": validator.get_limits()})
    session = TeachSession(robot, store, step_mm=args.step)
    print("workspace: %s" % validator)
    print("positions file: %s (%d recorded)" % (os.path.abspath(args.out), len(store.positions)))
    try:
        await robot.initialize()
        session.show_pose("start")
        print(HELP)
        await interactive(session)
    except KeyboardInterrupt:
        print("\nCtrl-C: emergency stop")
        robot.emergency_stop()
    finally:
        await robot.cleanup()
    return 0


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not (0.1 <= args.step <= MAX_STEP_MM):
        build_parser().error("--step must be 0.1..%g" % MAX_STEP_MM)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    return asyncio.run(_main(args))


if __name__ == "__main__":
    sys.exit(main())
