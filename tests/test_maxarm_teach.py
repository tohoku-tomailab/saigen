"""src/devices/maxarm/teach.py: command parsing, jog targets, the position file and a
session driven against MockMaxArmRobot through the IK-aware validator. No hardware,
no interactive input."""
import asyncio
import json

import pytest

from src.devices.maxarm import teach as T
from src.devices.safety.mock_maxarm import MockMaxArmRobot
from src.devices.safety.validators.maxarm import MaxArmWorkspaceValidator

BOX = {"x_min": -250, "x_max": 250, "y_min": -280, "y_max": -60, "z_min": 40, "z_max": 225,
       "r_min": 60, "r_max": 260}


# ----------------------------------------------------------------------
# parsing
# ----------------------------------------------------------------------
@pytest.mark.parametrize("line, expected", [
    ("x+", ("jog", ["x", 1.0, None])),
    ("z-", ("jog", ["z", -1.0, None])),
    ("Y+5", ("jog", ["y", 1.0, 5.0])),
    ("z- 2.5", ("jog", ["z", -1.0, 2.5])),
    ("goto 10 -200 150", ("goto", [10.0, -200.0, 150.0])),
    ("step 2", ("step", [2.0])),
    ("pump on", ("pump", [True])),
    ("pump 0", ("pump", [False])),
    ("save vial_A", ("save", ["vial_A"])),
    ("del vial_A", ("del", ["vial_A"])),
    ("  list ", ("list", [])),
    ("q", ("quit", [])),
    ("exit", ("quit", [])),
    ("HELP", ("help", [])),
    ("top", ("top", [])),
])
def test_parse_command_accepts_the_documented_forms(line, expected):
    assert T.parse_command(line) == expected


@pytest.mark.parametrize("line", ["", "x", "x+100", "x+0", "goto 1 2", "goto a b c", "step 0",
                                  "step 60", "step", "pump maybe", "save", "save bad name",
                                  "save /etc", "fly", "pos now", "w+"])
def test_parse_command_rejects_everything_else(line):
    with pytest.raises(T.CommandError):
        T.parse_command(line)


def test_jog_target_changes_one_axis_only():
    assert T.jog_target([1, 2, 3], "z", -1.0, 5.0) == [1.0, 2.0, -2.0]
    assert T.jog_target([1, 2, 3], "x", 1.0, 0.5) == [1.5, 2.0, 3.0]


# ----------------------------------------------------------------------
# position file
# ----------------------------------------------------------------------
def test_position_store_round_trips_and_writes_after_each_change(tmp_path):
    path = tmp_path / "p.json"
    s = T.PositionStore(str(path), meta={"port": "COMX"})
    s.save("a", [1.04, -2.06, 3.0])
    assert json.load(open(path, encoding="utf-8"))["positions"] == {"a": [1.0, -2.1, 3.0]}
    s.save("b", [0, 0, 0])
    assert s.delete("a") and not s.delete("a")
    again = T.PositionStore(str(path))
    assert again.positions == {"b": [0.0, 0.0, 0.0]}
    data = json.load(open(path, encoding="utf-8"))
    assert data["format"].startswith("saigen-maxarm-positions") and data["port"] == "COMX"
    assert "b" in again.table() and "no positions" in T.PositionStore(str(tmp_path / "e.json")).table()


# ----------------------------------------------------------------------
# session against the mock, through the IK-aware validator
# ----------------------------------------------------------------------
def _session(tmp_path, home=(0.0, -163.0, 200.0)):
    v = MaxArmWorkspaceValidator(**BOX)
    robot = MockMaxArmRobot(v, home_position=list(home))
    lines = []
    store = T.PositionStore(str(tmp_path / "pos.json"))
    return TeachSessionHolder(robot, T.TeachSession(robot, store, step_mm=10.0, out=lines.append),
                              lines)


class TeachSessionHolder:
    """Drives the session from synchronous tests; the mock is initialised once."""

    def __init__(self, robot, session, lines):
        self.robot, self.session, self.lines = robot, session, lines
        self.started = False

    def run(self, *commands):
        async def go():
            if not self.started:
                await self.robot.initialize()
                self.started = True
            return [await self.session.run_command(c) for c in commands]
        return asyncio.run(go())

    def pose(self):
        return self.robot.get_current_position()


def test_session_jogs_from_the_commanded_pose_and_records_positions(tmp_path):
    h = _session(tmp_path)
    h.run("help")
    start = h.pose()
    results = h.run("z-", "step 5", "x+", "y-2.5", "save vial", "list", "quit")
    assert results == [True, True, True, True, True, True, False]
    pose = h.session.store.positions["vial"]
    assert pose == [round(start[0] + 5, 1), round(start[1] - 2.5, 1), round(start[2] - 10, 1)]
    assert any("vial" in ln for ln in h.lines)


def test_session_refuses_unreachable_and_out_of_box_targets_without_moving(tmp_path):
    h = _session(tmp_path)
    h.run("goto 5 -185 200")                      # reachable
    before = h.pose()
    # z+ from 200 -> 210 is the highest reachable there; the second z+ (220), a target
    # outside the box and one inside R_MIN are refused, nothing moves
    h.run("z+", "z+", "goto 300 -180 100", "goto 30 -30 100")
    assert h.pose() == [5.0, -185.0, 210.0]
    refused = [ln for ln in h.lines if ln.startswith("REFUSED")]
    assert len(refused) == 3
    assert "逆運動学" in refused[0] and "210" in refused[0]
    assert before[2] == 200.0


def test_session_top_home_pump_and_bad_input(tmp_path):
    h = _session(tmp_path)
    h.run("goto 5 -185 195", "top", "pump on", "pump off", "home", "fly", "pos", "help")
    joined = "\n".join(h.lines)
    assert "highest reachable z at (5.0, -185.0): 210 mm" in joined
    assert "pump on" in joined and "pump off" in joined
    assert h.pose() == [0.0, -163.0, 200.0]      # home
    assert "? unknown command" in joined and "commands (mm)" in joined
    assert "commanded: x=0.0 y=-163.0 z=200.0" in joined


def test_interactive_loop_ends_on_quit_or_eof(tmp_path):
    h = _session(tmp_path)
    fed = iter(["z-", "quit", "z-"])

    async def go():
        await h.robot.initialize()
        z0 = h.pose()[2]
        await T.interactive(h.session, readline=lambda p: next(fed))
        z_after_quit = h.pose()[2]
        await T.interactive(h.session, readline=lambda p: (_ for _ in ()).throw(EOFError()))
        await h.robot.cleanup()
        return z0 - z_after_quit
    assert asyncio.run(go()) == 10.0           # one jog, then quit stopped the loop


def test_cli_parser_defaults_and_step_bounds():
    ap = T.build_parser()
    a = ap.parse_args(["--mock"])
    assert (a.robot, a.mock, a.out, a.step) == (4, True, T.DEFAULT_OUT, T.DEFAULT_STEP_MM)
    with pytest.raises(SystemExit):
        T.main(["--mock", "--step", "0"])


def test_turn_target_keeps_radius_and_height():
    import math
    x, y, z = T.turn_target([5.0, -185.0, 199.0], 20.0)
    assert math.hypot(x, y) == pytest.approx(math.hypot(5.0, -185.0))
    assert z == 199.0 and x > 5.0 and y > -185.0            # towards +x
    x2, y2, _ = T.turn_target([x, y, z], -20.0)
    assert (x2, y2) == pytest.approx((5.0, -185.0))
    assert T.parse_command("turn -70") == ("turn", [-70.0])
    for bad in ("turn", "turn 0", "turn 91", "turn abc"):
        with pytest.raises(T.CommandError):
            T.parse_command(bad)


def test_session_turn_is_bounded_by_the_box(tmp_path):
    """The example box (y <= -60) allows about +-70 deg of base rotation at r=185:
    turn 20 succeeds, a further turn 70 (past the box) is refused, nothing moves."""
    h = _session(tmp_path)
    h.run("goto 5 -185 199", "turn 20")
    x, y, z = h.pose()
    assert x > 60 and -175 < y < -170 and z == 199.0
    h.run("turn 70")
    assert h.pose() == [x, y, z]
    assert any(ln.startswith("REFUSED") for ln in h.lines)
