"""MockMaxArmRobot parity with MaxArmRobot, so a clean --mock run predicts the real run.

Same validator, same call sequence -> same exception types, same final pose and
suction state, on MaxArmRobot (driven by FakeMaxArmClient) and MockMaxArmRobot.
"""
import asyncio

import pytest

from src.devices.safety.maxarm_robot import MaxArmRobot
from src.devices.safety.mock_maxarm import (
    DEFAULT_MAXARM_START_POSE,
    MockMaxArmRobot,
    parse_maxarm_start_pose,
)
from src.devices.safety.validators import WorkspaceViolationError
from src.devices.safety.validators.workspace import NonFiniteTargetError
from tests.test_maxarm_robot import FakeMaxArmClient, maxarm_validator
import threading

_real_sleep = asyncio.sleep


async def _instant_sleep(delay, result=None):
    await _real_sleep(0)
    return result


def _real(**cfg):
    client = FakeMaxArmClient()
    robot = MaxArmRobot("FAKE", workspace_validator=maxarm_validator(),
                        client_factory=lambda port: client,
                        pose_settle_interval=0, wait_after_move=0, **cfg)
    return robot


def _mock(**cfg):
    return MockMaxArmRobot(workspace_validator=maxarm_validator(), **cfg)


class _SetHome:
    def __init__(self, home):
        self.home = home


SCENARIOS = [
    # (name, steps, expected exception type of the last step or None)
    ("pick and place", [("move_xyz", 100.0, -180.0, 150.0), ("move_z", -50.0),
                        ("suction", True), ("move_z", 50.0),
                        ("move_xyz", -100.0, -180.0, 150.0), ("move_z", -50.0),
                        ("suction", False), ("move_z", 50.0), ("go_home",)], None),
    ("x out of box", [("move_xyz", 300.0, -180.0, 150.0)], WorkspaceViolationError),
    ("radius too large", [("move_xyz", 240.0, -250.0, 150.0)], WorkspaceViolationError),
    ("y above y_max", [("move_xyz", 0.0, -50.0, 150.0)], WorkspaceViolationError),
    ("nan target", [("move_xyz", float("nan"), -180.0, 150.0)], NonFiniteTargetError),
    ("move_z above z_max", [("move_z", 20.0)], WorkspaceViolationError),
    ("move_z below z_min", [("move_xyz", 0.0, -180.0, 60.0), ("move_z", -30.0)],
     WorkspaceViolationError),
    ("move_z inf", [("move_z", float("inf"))], NonFiniteTargetError),
    ("go_home from below", [("move_xyz", 50.0, -200.0, 80.0), ("go_home",)], None),
    ("go_home without home", [(_SetHome(None),), ("go_home",)], RuntimeError),
    ("go_home to a home out of limits", [(_SetHome([0.0, -162.8, 240.0]),), ("go_home",)],
     WorkspaceViolationError),
    ("estop then move", [("suction", True), ("emergency_stop",),
                         ("move_xyz", 50.0, -180.0, 150.0)], None),
    ("estop then move_z", [("emergency_stop",), ("move_z", -20.0)], None),
    ("estop then go_home", [("move_xyz", 50.0, -180.0, 100.0), ("emergency_stop",),
                            ("go_home",)], None),
]


async def _play(robot, steps):
    await robot.initialize()
    robot.set_current_position_as_home()
    for step in steps:
        name, *args = step
        if isinstance(name, _SetHome):
            robot.home_position = name.home
            continue
        result = getattr(robot, name)(*args)
        if asyncio.iscoroutine(result):
            await result


def _outcome(robot, steps):
    try:
        asyncio.run(_play(robot, steps))
        exc = None
    except Exception as e:  # noqa: BLE001
        exc = type(e)
    pose = [round(v, 3) for v in robot.get_current_position()]
    return exc, pose, robot.suction_state, robot._pose_unknown


@pytest.mark.parametrize("name, steps, exc", SCENARIOS, ids=[s[0] for s in SCENARIOS])
def test_mock_matches_maxarm_robot(name, steps, exc, monkeypatch):
    monkeypatch.setattr(asyncio, "sleep", _instant_sleep)
    real, mock = _outcome(_real(), steps), _outcome(_mock(), steps)
    assert real == mock, (real, mock)
    assert real[0] is exc


def test_estop_parity_pose_and_suction_unknown():
    real, mock = _real(), _mock()
    for robot in (real, mock):
        asyncio.run(robot.initialize())
        asyncio.run(robot.suction(True))
        robot.emergency_stop()
        assert robot._pose_unknown is True
        assert robot.suction_state is None
        assert robot._estop_generation == 1


def test_same_public_api_and_capabilities():
    for attr in ("initialize", "move_xyz", "move_z", "go_home", "suction", "suction_state",
                 "emergency_stop", "cleanup", "get_current_position",
                 "set_current_position_as_home", "home_is_within_limits",
                 "__aenter__", "__aexit__", "firmware_info"):
        assert hasattr(_mock(), attr), attr
        assert hasattr(_real(), attr), attr
    assert MockMaxArmRobot.SUPPORTED_ACTIONS == MaxArmRobot.SUPPORTED_ACTIONS
    assert MockMaxArmRobot.ROBOT_TYPE == MaxArmRobot.ROBOT_TYPE == "maxarm"
    assert _mock().firmware_info == {"fw": "mock"}


def test_invalid_move_ms_is_rejected_by_both():
    with pytest.raises(ValueError):
        _real(move_ms=50)
    with pytest.raises(ValueError):
        _mock(move_ms=50)


def _initialized(robot):
    asyncio.run(robot.initialize())
    return robot


def test_mock_default_start_pose_is_the_maxarm_origin():
    assert DEFAULT_MAXARM_START_POSE == (0.0, -162.8, 212.0)
    assert _initialized(_mock()).get_current_position() == [0.0, -162.8, 212.0]


def test_mock_start_pose_accepts_three_or_four_values():
    assert parse_maxarm_start_pose("10,-150,100") == (10.0, -150.0, 100.0)
    assert parse_maxarm_start_pose([10, -150, 100, 45]) == (10.0, -150.0, 100.0)
    assert parse_maxarm_start_pose("") is None
    assert parse_maxarm_start_pose(None) is None
    for bad in ("1,2", "1,2,3,4,5", "a,b,c", [1, 2, float("nan")]):
        with pytest.raises(ValueError):
            parse_maxarm_start_pose(bad)
    mock = _initialized(MockMaxArmRobot(workspace_validator=maxarm_validator(),
                                        start_pose="10,-150,100,0"))
    assert mock.get_current_position() == [10.0, -150.0, 100.0]


def test_mock_cleanup_pump_off():
    mock = _initialized(_mock())
    asyncio.run(mock.suction(True))
    asyncio.run(mock.cleanup())
    assert mock.suction_state is False


# ----------------------------------------------------------------------
# Review S9 / f9: e-stop during motion and lifecycle guards
# ----------------------------------------------------------------------
def test_mock_estop_during_a_move_raises_and_keeps_the_pose_unknown():
    """Review f9: the mock used to finish the move and clear the unknown state."""
    mock = _mock()

    async def scenario():
        await mock.initialize()
        task = asyncio.ensure_future(mock.move_xyz(50.0, -180.0, 150.0))
        await asyncio.sleep(0.05)
        mock.emergency_stop()
        with pytest.raises(RuntimeError, match="緊急停止"):
            await task
        assert mock._pose_unknown
        assert mock.get_current_position() == [0.0, -162.8, 212.0], "move not committed"
    asyncio.run(scenario())


def _estop_mid_move(robot, client=None):
    async def scenario():
        await robot.initialize()
        robot.set_current_position_as_home()
        await robot.suction(True)
        if client is not None:
            client.block = True
        task = asyncio.ensure_future(robot.move_z(-10.0))
        if client is not None:
            while not client.started.is_set():
                await asyncio.sleep(0.01)
            threading.Thread(target=robot.emergency_stop).start()
        else:
            await asyncio.sleep(0.05)
            robot.emergency_stop()
        try:
            await task
            return None
        except Exception as e:  # noqa: BLE001
            return type(e)
    exc = asyncio.run(scenario())
    return exc, [round(v, 3) for v in robot.get_current_position()], \
        robot.suction_state, robot._pose_unknown


def test_estop_during_motion_parity():
    client = FakeMaxArmClient()
    real = MaxArmRobot("FAKE", workspace_validator=maxarm_validator(),
                       client_factory=lambda port: client,
                       pose_settle_interval=0, wait_after_move=0)
    assert _estop_mid_move(real, client) == _estop_mid_move(_mock())
    assert _estop_mid_move(_mock())[0] is RuntimeError


@pytest.mark.parametrize("op, args", [
    ("move_xyz", (50.0, -180.0, 150.0)), ("move_z", (-10.0,)), ("go_home", ()),
    ("suction", (True,)), ("get_current_position", ()),
    ("set_current_position_as_home", ()),
])
def test_lifecycle_guards_before_initialize_and_after_cleanup(op, args):
    """Review f9b: the mock allowed motion / suction / position before init and after cleanup."""
    for robot in (_real(home_position=[0.0, -162.8, 212.0]),
                  _mock(home_position=[0.0, -162.8, 212.0])):
        for phase in ("before", "after"):
            if phase == "after":
                asyncio.run(robot.initialize())
                asyncio.run(robot.cleanup())
            with pytest.raises(RuntimeError, match="初期化されていません"):
                result = getattr(robot, op)(*args)
                if asyncio.iscoroutine(result):
                    asyncio.run(result)
            assert robot._initialized is False


def test_estop_before_initialize_is_a_no_op_on_both():
    for robot in (_real(), _mock()):
        robot.emergency_stop()
        assert robot._estop_generation == 0 and robot._pose_unknown is False


def test_both_refuse_to_initialize_without_a_validator():
    client = FakeMaxArmClient()
    real = MaxArmRobot("FAKE", client_factory=lambda port: client)
    for robot in (real, MockMaxArmRobot()):
        with pytest.raises(RuntimeError, match="可動域バリデータ"):
            asyncio.run(robot.initialize())
        assert robot._initialized is False
    assert client.calls == []


@pytest.mark.parametrize("bad", [float("nan"), 0, -2.0])
def test_invalid_move_tolerance_is_rejected_by_both(bad):
    with pytest.raises(ValueError):
        _real(move_tolerance_mm=bad)
    with pytest.raises(ValueError):
        _mock(move_tolerance_mm=bad)


@pytest.mark.parametrize("cfg, estop, expected", [
    ({}, False, False),                           # clean run: pump off
    ({}, True, None),                             # after an e-stop: suction kept (unknown)
    ({"pump_off_after_estop": True}, True, False),
    ({"pump_off_on_cleanup": False}, False, True),
])
def test_cleanup_suction_policy_parity(cfg, estop, expected):
    for robot in (_real(**cfg), _mock(**cfg)):
        asyncio.run(robot.initialize())
        asyncio.run(robot.suction(True))
        if estop:
            robot.emergency_stop()
        assert robot.ended_abnormally is estop
        asyncio.run(robot.cleanup())
        assert robot.suction_state is expected, (type(robot).__name__, cfg, estop)
        assert robot._initialized is False


def test_configured_home_parity():
    for robot in (_real(home_position=[0.0, -200.0, 150.0]),
                  _mock(home_position=[0.0, -200.0, 150.0])):
        assert robot.home_configured is True
        assert robot.home_is_within_limits() is True
    for robot in (_real(home_position=[0.0, -200.0, 240.0]),
                  _mock(home_position=[0.0, -200.0, 240.0])):
        assert robot.home_is_within_limits() is False


def test_mock_refuses_a_second_initialize_like_the_real_robot():
    """R2-5 parity: initialize() twice without cleanup() is an error on both robots."""
    import asyncio
    from src.devices.safety.mock_maxarm import MockMaxArmRobot
    from src.devices.safety.validators import WorkspaceValidator
    mock = MockMaxArmRobot(workspace_validator=WorkspaceValidator(
        x_min=-250, x_max=250, y_min=-280, y_max=-60, z_min=40, z_max=225,
        joint1_min=-135, joint1_max=135))

    async def scenario():
        assert await mock.initialize()
        with pytest.raises(RuntimeError):
            await mock.initialize()
        await mock.cleanup()
        assert await mock.initialize()
        await mock.cleanup()

    asyncio.run(scenario())
