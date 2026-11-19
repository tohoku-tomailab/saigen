"""Counterexamples to the workspace and rejection guarantees (roadmap item 0-1).

Each case is a move that the current code lets through, or a rejection that
still has side effects. The cases come from
``docs/superpowers/plans/2026-10-30-improvement-roadmap.md`` (0-1 table):
the real wrapper is checked by the commands a fake pydobot device records
(pose reads are not commands), the mock by its state staying unchanged.

A pair (case, implementation, guarantee) that does not hold today is marked
``xfail(strict=True)``: it must turn into a pass when item 1a / C1 is
implemented, and the strict flag makes the suite fail until the mark is
removed. Pairs that already hold are ordinary tests. No hardware is opened.

Box used unless a case says otherwise: X -250..250, Y -50..50, Z 0..100,
J1 -135..135 (deg).
"""
import asyncio

import pytest

from src.devices.safety import mock_robot as mock_robot_module
from src.devices.safety.lab_robot import LabRobot
from src.devices.safety.mock_robot import MockLabRobot
from src.devices.safety.validators import ValidationError, WorkspaceValidator

BOX = dict(x_min=-250, x_max=250, y_min=-50, y_max=50, z_min=0, z_max=100)

# case -> start pose (x, y, z, r, j1), home, limits, operation (method, args)
CASES = {
    "a_rotate_end_outside": dict(
        pose=(200.0, 0.0, 50.0, 0.0, 0.0), op=("rotate", (90.0,))),
    "b_arc_extremum": dict(
        limits=dict(BOX, x_max=199.9),
        pose=(199.809644, -8.723877, 50.0, -2.5, -2.5), op=("rotate_relative", (5.0,))),
    "c_go_home_intermediate": dict(
        limits=dict(BOX, x_min=100),
        pose=(250.0, 0.0, 50.0, 0.0, 0.0), home=[200.0, 50.0, 50.0, 14.036243],
        op=("go_home", ())),
    "d_radius_inverted": dict(
        pose=(200.0, 0.0, 50.0, 0.0, 0.0), op=("move_radial", (-400.0,))),
    "d2_radius_zero": dict(
        pose=(200.0, 0.0, 50.0, 0.0, 0.0), op=("move_radial", (-200.0,))),
    "h_rejection_triggers_recovery": dict(
        pose=(0.0, 0.0, 50.0, 0.0, 0.0), home=[200.0, 0.0, 50.0, 0.0],
        op=("move_radial", (10.0,))),
    "i_side_effect_before_check": dict(
        pose=(200.0, 0.0, 50.0, 0.0, 0.0), op=("rotate_relative", (180.0, "low"))),
    "j_slider_sends_unchecked_pose": dict(
        pose=(200.0, 0.0, 130.0, 0.0, 0.0), op=("move_slider", (10.0,))),
}


def _xfail(reason):
    return pytest.mark.xfail(strict=True, reason=reason)


def _case(name, mark=None):
    return pytest.param(name, id=name, marks=[mark] if mark else [])


# ----------------------------------------------------------------------
# Fakes
# ----------------------------------------------------------------------
class _Response:
    def __init__(self, params=b""):
        self.params = params


class FakeDevice:
    """The part of ``pydobot.Dobot`` that PyDobotController touches.

    ``pose()`` returns the scripted poses in order and then repeats the last
    one. Every call that changes the arm's state is recorded in ``sent``;
    pose reads are not.
    """

    def __init__(self, *poses):
        self._poses = [tuple(p) for p in poses]
        self.sent = []

    def pose(self):
        if len(self._poses) > 1:
            return self._poses.pop(0)
        return self._poses[0]

    def move_to(self, x, y, z, r, wait=False):
        self.sent.append(("move_to", x, y, z, r))

    def _set_ptp_cmd(self, j1, j2, j3, j4, mode=None, wait=False):
        self.sent.append(("joint_move", j1, j2, j3, j4))

    def speed(self, velocity, acceleration):
        self.sent.append(("speed", velocity, acceleration))

    def _set_ptp_joint_params(self, *params):
        self.sent.append(("joint_params", *params))

    def _send_command(self, msg, wait=False):
        self.sent.append(("command", msg.id, bytes(msg.params)))
        return _Response()


def _full_pose(x, y, z, r, j1):
    return (x, y, z, r, j1, 0.0, 0.0, 0.0)


def _real_robot(case, *extra_poses):
    """LabRobot over the real PyDobotController, which talks to FakeDevice."""
    pytest.importorskip("pydobot")
    from src.devices.dobot.dobot_config import DobotConfig
    from src.devices.dobot.pydobot_controller import PyDobotController

    spec = CASES[case]
    device = FakeDevice(_full_pose(*spec["pose"]), *extra_poses)
    ctrl = object.__new__(PyDobotController)
    ctrl.port_name = "TEST"
    ctrl.device = device
    ctrl._connected = True
    ctrl.verbose = False
    ctrl.move_timeout_s = None
    ctrl.speed_preset = DobotConfig.DEFAULT_SPEED
    ctrl.current_speed_config = DobotConfig.get_speed_preset(ctrl.speed_preset)

    robot = LabRobot(use_dobot=True,
                     workspace_validator=WorkspaceValidator(**spec.get("limits", BOX)))
    robot.dobot = ctrl
    for attr in ("wait_after_angle_move", "wait_after_z_move",
                 "wait_after_xy_move", "wait_after_slider_move"):
        setattr(robot, attr, 0)
    robot.home_position = list(spec.get("home", [200.0, 0.0, 50.0, 0.0]))

    recoveries = []
    real_go_home = robot.go_home

    async def counting_go_home():
        recoveries.append(True)
        await real_go_home()

    if spec["op"][0] != "go_home":
        robot.go_home = counting_go_home  # every go_home here is an error recovery
    return robot, device, recoveries


def _mock_robot(case):
    spec = CASES[case]
    x, y, z, r, _ = spec["pose"]
    robot = MockLabRobot(use_dobot=True,
                         workspace_validator=WorkspaceValidator(**spec.get("limits", BOX)),
                         start_pose=(x, y, z, r))
    if "home" in spec:
        robot.home_position = list(spec["home"])
    return robot


def _mock_state(robot):
    return list(robot._pos), robot._slider_pos, list(robot.home_position)


def _run(robot, case):
    method, args = CASES[case]["op"]
    return asyncio.run(getattr(robot, method)(*args))


@pytest.fixture(autouse=True)
def _no_mock_delays(monkeypatch):
    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(mock_robot_module.asyncio, "sleep", no_sleep)


# ----------------------------------------------------------------------
# Real wrapper: rejected, nothing sent, no recovery
# ----------------------------------------------------------------------
_NOT_YET_1A = "workspace check on the arc / home legs / radius / slider pose is item 1a"
_NOT_YET_C1 = "rejection before side effects and without recovery is contract C1"


@pytest.mark.parametrize("case", [
    _case("a_rotate_end_outside", _xfail(_NOT_YET_1A)),
    _case("b_arc_extremum", _xfail(_NOT_YET_1A)),
    _case("c_go_home_intermediate", _xfail(_NOT_YET_1A)),
    _case("d_radius_inverted", _xfail(_NOT_YET_1A)),
    _case("d2_radius_zero", _xfail(_NOT_YET_1A)),
    _case("h_rejection_triggers_recovery", _xfail(_NOT_YET_C1)),
    _case("i_side_effect_before_check", _xfail(_NOT_YET_C1)),
    _case("j_slider_sends_unchecked_pose", _xfail(_NOT_YET_1A)),
])
def test_real_wrapper_rejects_without_sending(case):
    robot, device, recoveries = _real_robot(case)
    with pytest.raises(Exception):
        _run(robot, case)
    assert device.sent == []
    assert recoveries == []


@pytest.mark.parametrize("case", [
    _case("a_rotate_end_outside", _xfail(_NOT_YET_1A)),
    _case("b_arc_extremum", _xfail(_NOT_YET_1A)),
    _case("c_go_home_intermediate", _xfail(_NOT_YET_1A)),
    _case("d_radius_inverted", _xfail(_NOT_YET_1A)),
    _case("d2_radius_zero", _xfail(_NOT_YET_1A)),
    _case("h_rejection_triggers_recovery", _xfail(_NOT_YET_C1)),
    _case("i_side_effect_before_check"),
    _case("j_slider_sends_unchecked_pose", _xfail(_NOT_YET_C1)),
])
def test_real_wrapper_rejection_is_a_validation_error(case):
    robot, _device, _recoveries = _real_robot(case)
    with pytest.raises(ValidationError):
        _run(robot, case)


@pytest.mark.xfail(strict=True, reason="the driver re-reads the pose in move_Z (item 1a (9))")
def test_real_wrapper_sends_the_z_it_validated():
    # (g): the pose read for the check says Z=50 (target 90 is inside the box);
    # the driver reads again, gets Z=90 and adds the offset once more.
    robot, device, _recoveries = _real_robot(
        "a_rotate_end_outside", _full_pose(200.0, 0.0, 90.0, 0.0, 0.0))
    asyncio.run(robot.move_z(40.0))
    assert device.sent == [("move_to", 200.0, 0.0, 90.0, 0.0)]


def test_real_wrapper_already_rejects_rotate_beyond_joint1_without_sending():
    robot, device, recoveries = _real_robot("a_rotate_end_outside")
    with pytest.raises(ValidationError):
        asyncio.run(robot.rotate(180.0))
    assert device.sent == []
    assert recoveries == []


# ----------------------------------------------------------------------
# Mock: rejected, state unchanged
# ----------------------------------------------------------------------
@pytest.mark.parametrize("case", [
    _case("a_rotate_end_outside", _xfail(_NOT_YET_1A)),
    _case("b_arc_extremum", _xfail(_NOT_YET_1A)),
    _case("c_go_home_intermediate", _xfail(_NOT_YET_1A)),
    _case("d_radius_inverted", _xfail(_NOT_YET_1A)),
    _case("d2_radius_zero", _xfail(_NOT_YET_1A)),
    _case("h_rejection_triggers_recovery"),
    _case("i_side_effect_before_check"),
    _case("j_slider_sends_unchecked_pose", _xfail(_NOT_YET_1A)),
])
def test_mock_rejects_without_changing_state(case):
    robot = _mock_robot(case)
    before = _mock_state(robot)
    with pytest.raises(Exception):
        _run(robot, case)
    assert _mock_state(robot) == before


@pytest.mark.parametrize("case", [
    _case("a_rotate_end_outside", _xfail(_NOT_YET_1A)),
    _case("b_arc_extremum", _xfail(_NOT_YET_1A)),
    _case("c_go_home_intermediate", _xfail(_NOT_YET_1A)),
    _case("d_radius_inverted", _xfail(_NOT_YET_1A)),
    _case("d2_radius_zero", _xfail(_NOT_YET_1A)),
    _case("h_rejection_triggers_recovery", _xfail(_NOT_YET_C1)),
    _case("i_side_effect_before_check"),
    _case("j_slider_sends_unchecked_pose", _xfail(_NOT_YET_C1)),
])
def test_mock_rejection_is_a_validation_error(case):
    robot = _mock_robot(case)
    with pytest.raises(ValidationError):
        _run(robot, case)


def test_mock_moves_z_by_the_validated_offset():
    robot = MockLabRobot(use_dobot=True, workspace_validator=WorkspaceValidator(**BOX),
                         start_pose=(200.0, 0.0, 50.0, 0.0))
    asyncio.run(robot.move_z(40.0))
    assert robot._pos[2] == 90.0


def test_mock_already_rejects_rotate_beyond_joint1_without_changing_state():
    robot = _mock_robot("a_rotate_end_outside")
    before = _mock_state(robot)
    with pytest.raises(ValidationError):
        asyncio.run(robot.rotate(180.0))
    assert _mock_state(robot) == before
