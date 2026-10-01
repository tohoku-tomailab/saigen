"""MaxArmRobot: the safety wrapper for the Hiwonder MaxArm.

Same safety semantics as LabRobot (tests/test_safety2_blocking_moves.py):
workspace validation before any command, moves in a worker thread that a
cancellation force-stops at once, pose unknown after a cancelled / failed /
e-stopped move and re-read (settled) before the next one, no automatic go_home
after an error, go_home validates every leg first, cleanup never moves the arm.
All fakes; no serial port is ever opened.
"""
import asyncio
import math
import sys
import threading
import time
import types
from dataclasses import dataclass
from typing import Optional, Tuple

import pytest

from src.devices.safety.maxarm_robot import (
    SUCTION_KEPT_WARNING,
    MaxArmRobot,
    device_limits_from_validator,
    validate_move_ms,
    validate_move_tolerance,
)
from src.devices.safety.validators import WorkspaceValidator, WorkspaceViolationError
from src.devices.safety.validators.workspace import NonFiniteTargetError

ORIGIN = (0.0, -162.8, 212.0)


@dataclass
class FakeStatus:
    x: float
    y: float
    z: float
    pulses: Tuple[int, int, int]
    moving: bool
    stopped: bool
    pump: bool
    vin_mv: Optional[int]


class FakeCommandError(RuntimeError):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


class FakeMaxArmClient:
    """Implements exactly the MaxArmClient API the wrapper codes against.

    ``move_xyz_and_wait`` blocks (like the real poll-until-idle loop) until
    ``released`` is set; ``force_stop`` sets the firmware's stopped latch and,
    by default, releases the blocked move.
    """

    def __init__(self, port="FAKE", pose=ORIGIN, stopped=False, pump=False,
                 moving=False, block=False, release_on_stop=True,
                 stop_raises=False, readback_offset=(0.0, 0.0, 0.0)):
        self.port = port
        self.pose = list(pose)
        self.stopped = stopped
        self.pump_on = pump
        self.moving = moving
        self.block = block
        self.release_on_stop = release_on_stop
        self.stop_raises = stop_raises          # a stopped move raises (tolerance error)
        self.readback_offset = readback_offset  # read-back noise added to the target
        self.connected = False
        self.info = {}
        self.calls = []
        self.moves = []
        self.force_stops = 0
        self.status_reads = 0
        self.status_hook = None
        self.fail = {}                          # method name -> exception to raise
        self.started = threading.Event()
        self.released = threading.Event()
        self.finished = threading.Event()

    def _maybe_fail(self, name):
        if name in self.fail:
            raise self.fail[name]

    def connect(self, limits=None, watchdog_ms=1000):
        self.calls.append(("connect", limits, watchdog_ms))
        self._maybe_fail("connect")
        self.connected = True
        self.info = {"fw": "saigen-maxarm", "proto": "1", "ver": "0.1.0",
                     "stopped": "1" if self.stopped else "0"}
        return dict(self.info)

    def status(self):
        self.status_reads += 1
        self.calls.append(("status",))
        self._maybe_fail("status")
        if self.status_hook is not None:
            self.status_hook(self)
        return FakeStatus(self.pose[0], self.pose[1], self.pose[2], (500, 500, 500),
                          self.moving, self.stopped, self.pump_on, 12000)

    def move_xyz_and_wait(self, x, y, z, ms, tolerance_mm=10.0, timeout_margin=2.0):
        self.calls.append(("move", x, y, z, ms, tolerance_mm))
        self.moves.append((x, y, z))
        self._maybe_fail("move")
        if self.stopped:
            raise FakeCommandError("STOPPED")
        self.started.set()
        try:
            if self.block:
                self.released.wait(10)
        finally:
            self.finished.set()
        if self.stopped:            # STOP arrived mid-move: the arm halted short
            if self.stop_raises:
                raise FakeCommandError("tolerance")
        else:
            self.pose = [x, y, z]
        dx, dy, dz = self.readback_offset
        return FakeStatus(self.pose[0] + dx, self.pose[1] + dy, self.pose[2] + dz,
                          (500, 500, 500), False, self.stopped, self.pump_on, 12000)

    def stop(self):
        self.calls.append(("stop",))
        self.stopped = True

    def force_stop(self):
        self.calls.append(("force_stop",))
        self.force_stops += 1
        self.stopped = True
        if self.release_on_stop:
            self.released.set()
        return True

    def clear(self):
        self.calls.append(("clear",))
        self._maybe_fail("clear")
        self.stopped = False

    def pump(self, on):
        self.calls.append(("pump", bool(on)))
        self._maybe_fail("pump")
        self.pump_on = bool(on)

    def close(self):
        self.calls.append(("close",))
        self._maybe_fail("close")
        self.connected = False


def maxarm_validator():
    """The conservative starting box of config.example.yaml's robot 4."""
    return WorkspaceValidator(x_min=-250, x_max=250, y_min=-280, y_max=-60,
                              z_min=40, z_max=225, r_min=60, r_max=260)


def make_robot(client=None, validator="default", **cfg):
    client = client or FakeMaxArmClient()
    if validator == "default":
        validator = maxarm_validator()
    cfg.setdefault("pose_settle_interval", 0)
    cfg.setdefault("wait_after_move", 0)
    robot = MaxArmRobot("FAKE", workspace_validator=validator,
                        client_factory=lambda port: client, **cfg)
    return robot, client


def ready_robot(client=None, **cfg):
    robot, client = make_robot(client, **cfg)
    asyncio.run(robot.initialize())
    robot.set_current_position_as_home()
    client.calls.clear()
    client.status_reads = 0
    return robot, client


def commands(client):
    """Calls that command the arm or the latch (not status reads)."""
    return [c for c in client.calls if c[0] in ("move", "clear", "stop", "force_stop", "pump")]


async def cancel_when_started(client, coro, ticks):
    task = asyncio.ensure_future(coro)
    t0 = time.monotonic()
    while not client.started.is_set():
        assert time.monotonic() - t0 < 5, "the move never started"
        await asyncio.sleep(0.01)
    for _ in range(3):              # the event loop is not blocked by the move
        await asyncio.sleep(0.01)
        ticks.append(True)
    task.cancel()
    t1 = time.monotonic()
    with pytest.raises(asyncio.CancelledError):
        await task
    return time.monotonic() - t1


# ----------------------------------------------------------------------
# initialize
# ----------------------------------------------------------------------
def test_initialize_connects_with_device_limits_clears_latch_and_stores_info(caplog):
    client = FakeMaxArmClient(stopped=True, pump=True)
    robot, _ = make_robot(client, watchdog_ms=800)
    with caplog.at_level("INFO"):
        assert asyncio.run(robot.initialize()) is True
    kind, limits, wd = client.calls[0]
    assert kind == "connect" and wd == 800
    assert limits == {"xmin": -250.0, "xmax": 250.0, "ymin": -280.0, "ymax": -60.0,
                      "zmin": 40.0, "zmax": 225.0, "rmin": 60.0, "rmax": 260.0}
    assert ("clear",) in client.calls
    assert client.stopped is False
    assert robot.firmware_info["fw"] == "saigen-maxarm"
    assert "saigen-maxarm" in caplog.text
    assert robot.get_current_position() == list(ORIGIN)
    assert robot._pose_unknown is False
    assert robot.suction_state is True
    assert not any(c[0] == "move" for c in client.calls), "initialize never moves"


def test_initialize_without_latch_does_not_clear():
    robot, client = make_robot()
    asyncio.run(robot.initialize())
    assert ("clear",) not in client.calls


def test_initialize_refuses_without_a_validator():
    """P5 / review f4: no validator -> no connection at all (no unvalidated motion,
    no firmware left without LIMITS)."""
    made = []
    client = FakeMaxArmClient()
    robot = MaxArmRobot("FAKE", client_factory=lambda p: made.append(p) or client,
                        pose_settle_interval=0, wait_after_move=0)
    with pytest.raises(RuntimeError, match="可動域バリデータ"):
        asyncio.run(robot.initialize())
    assert made == [] and client.calls == []
    assert robot.client is None


def test_initialize_with_arm_still_moving_marks_pose_unknown():
    robot, client = make_robot(FakeMaxArmClient(moving=True))
    asyncio.run(robot.initialize())
    assert robot._pose_unknown is True


def test_initialize_failure_closes_and_raises():
    client = FakeMaxArmClient()
    client.fail["status"] = IOError("no reply")
    robot, _ = make_robot(client)
    with pytest.raises(IOError):
        asyncio.run(robot.initialize())
    assert ("close",) in client.calls
    assert robot.client is None


def test_client_is_imported_lazily_from_the_driver_package(monkeypatch):
    created = []

    class StubClient(FakeMaxArmClient):
        def __init__(self, port, **kw):
            super().__init__(port)
            created.append(port)

    pkg = types.ModuleType("src.devices.maxarm")
    pkg.__path__ = []
    mod = types.ModuleType("src.devices.maxarm.client")
    mod.MaxArmClient = StubClient
    monkeypatch.setitem(sys.modules, "src.devices.maxarm", pkg)
    monkeypatch.setitem(sys.modules, "src.devices.maxarm.client", mod)
    robot = MaxArmRobot("COMX", workspace_validator=maxarm_validator())
    asyncio.run(robot.initialize())
    assert created == ["COMX"]


def test_move_ms_is_range_checked():
    for bad in (99, 30001, "fast", 1500.5, True, None):
        with pytest.raises(ValueError):
            MaxArmRobot("FAKE", move_ms=bad)
    assert validate_move_ms(100) == 100 and validate_move_ms(30000.0) == 30000


def test_robot_type_and_supported_actions():
    assert MaxArmRobot.ROBOT_TYPE == "maxarm"
    assert MaxArmRobot.SUPPORTED_ACTIONS == frozenset(
        {"move_xyz", "move_z", "go_home", "suction", "wait"})


def test_device_limits_fill_the_missing_radius_bound():
    v = WorkspaceValidator(x_min=-100, x_max=100, y_min=-200, y_max=-50, r_min=60)
    lim = device_limits_from_validator(v)
    assert lim["rmin"] == 60.0
    assert lim["rmax"] == pytest.approx(math.hypot(100, 200))
    assert "rmin" not in device_limits_from_validator(WorkspaceValidator())


def test_not_initialized_raises_before_anything():
    robot = MaxArmRobot("FAKE")
    for coro in (robot.move_xyz(0, -150, 100), robot.move_z(-5), robot.go_home(),
                 robot.suction(True)):
        with pytest.raises(RuntimeError, match="初期化されていません"):
            asyncio.run(coro)
    robot.emergency_stop()           # no client: must not raise
    asyncio.run(robot.cleanup())     # no client: nothing to do


# ----------------------------------------------------------------------
# validation before any command
# ----------------------------------------------------------------------
@pytest.mark.parametrize("target, exc", [
    ((300.0, -150.0, 100.0), WorkspaceViolationError),   # x out of box
    ((0.0, -150.0, 10.0), WorkspaceViolationError),      # z below z_min
    ((240.0, -250.0, 100.0), WorkspaceViolationError),   # inside the box, r > r_max
    ((10.0, -50.0, 100.0), WorkspaceViolationError),     # y out of box
    ((float("nan"), -150.0, 100.0), NonFiniteTargetError),
])
def test_move_xyz_is_validated_before_any_command(target, exc):
    robot, client = ready_robot()
    with pytest.raises(exc):
        asyncio.run(robot.move_xyz(*target))
    assert commands(client) == []
    assert robot._pose_unknown is False


def test_move_z_is_validated_before_any_command():
    robot, client = ready_robot()
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.move_z(20.0))                   # 212 + 20 > z_max 225
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.move_z(-180.0))                 # below z_min
    with pytest.raises(NonFiniteTargetError):
        asyncio.run(robot.move_z(float("inf")))
    assert commands(client) == []


# ----------------------------------------------------------------------
# normal moves
# ----------------------------------------------------------------------
def test_successful_move_uses_config_and_stores_the_read_back_pose():
    client = FakeMaxArmClient(readback_offset=(1.0, -1.0, 2.0))
    robot, _ = ready_robot(client, move_ms=2000, move_tolerance_mm=6.0)
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert commands(client) == [("move", 50.0, -180.0, 150.0, 2000, 6.0)]
    assert client.force_stops == 0
    assert robot._pose_unknown is False
    assert robot.get_current_position() == [51.0, -181.0, 152.0]   # read-back


def test_move_z_is_relative_to_the_commanded_pose_not_the_noisy_read_back():
    client = FakeMaxArmClient(readback_offset=(1.5, -2.0, 2.5))
    robot, _ = ready_robot(client)
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    asyncio.run(robot.move_z(-30.0))
    assert client.moves[-1] == (50.0, -180.0, 120.0)


def test_get_current_position_does_no_io():
    robot, client = ready_robot()
    robot.get_current_position()
    assert client.status_reads == 0


def test_set_home_refuses_while_the_arm_is_moving():
    robot, client = ready_robot()
    client.moving = True
    with pytest.raises(RuntimeError, match="動作中"):
        robot.set_current_position_as_home()


def test_home_outside_limits_is_reported(caplog):
    client = FakeMaxArmClient(pose=(0.0, -40.0, 212.0))   # y > y_max
    robot, _ = make_robot(client)
    asyncio.run(robot.initialize())
    robot.set_current_position_as_home()
    assert robot.home_is_within_limits() is False


# ----------------------------------------------------------------------
# cancellation during a move
# ----------------------------------------------------------------------
@pytest.mark.parametrize("method, args", [
    ("move_xyz", (50.0, -180.0, 150.0)),
    ("move_z", (-30.0,)),
    ("go_home", ()),
])
def test_cancel_during_a_move_force_stops_once(method, args):
    client = FakeMaxArmClient()
    robot, _ = ready_robot(client)
    if method == "go_home":
        asyncio.run(robot.move_xyz(50.0, -180.0, 100.0))   # below home: two legs
        client.calls.clear()
        client.moves.clear()
    client.block = True
    ticks = []
    elapsed = asyncio.run(cancel_when_started(client, getattr(robot, method)(*args), ticks))
    assert ticks == [True, True, True], "the event loop was blocked by the move"
    assert client.force_stops == 1
    assert client.finished.is_set(), "the worker must have returned before re-raising"
    assert elapsed < 5
    assert len(client.moves) == 1, "no further leg / recovery move after the cancel"
    assert robot._pose_unknown


def test_worker_that_never_returns_is_abandoned_after_stop_wait_timeout():
    client = FakeMaxArmClient(block=True, release_on_stop=False)
    robot, _ = ready_robot(client, stop_wait_timeout=0.2)
    try:
        elapsed = asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
        assert client.force_stops == 1
        assert elapsed < 3
    finally:
        client.released.set()


def test_next_move_after_a_cancel_rereads_a_settled_pose_then_clears_the_latch():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    assert robot._pose_unknown and client.stopped
    client.block = False
    reads_before = client.status_reads
    client.calls.clear()
    asyncio.run(robot.move_z(-5.0))
    assert client.status_reads - reads_before >= 2, "settle check: at least two reads"
    kinds = [c[0] for c in client.calls]
    assert kinds.index("clear") < kinds.index("move"), "latch cleared only after the re-read"
    assert kinds.index("status") < kinds.index("clear")
    assert robot._pose_unknown is False
    # the relative move is based on the re-read pose (where the arm stopped)
    assert client.moves[-1] == (ORIGIN[0], ORIGIN[1], ORIGIN[2] - 5.0)


def test_unstable_pose_after_a_cancel_refuses_the_next_move():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client, pose_settle_reads=3)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    client.block = False

    def drifting(c):
        c.pose[0] += 5.0                 # still moving (more than the 3 mm tolerance)

    client.status_hook = drifting
    n = len(client.moves)
    with pytest.raises(RuntimeError, match="安定しません"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert len(client.moves) == n, "no move may be sent while the pose is unstable"
    assert ("clear",) not in client.calls
    assert robot._pose_unknown


def test_firmware_moving_flag_also_counts_as_not_settled():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client, pose_settle_reads=3)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    client.block = False
    client.moving = True                 # pose constant, but the firmware says moving
    with pytest.raises(RuntimeError, match="安定しません"):
        asyncio.run(robot.move_z(-5.0))
    assert len(client.moves) == 1


def test_small_read_back_noise_within_tolerance_is_settled():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    client.block = False
    toggle = [1.0]

    def noise(c):
        toggle[0] = -toggle[0]
        c.pose[1] += 2.0 * toggle[0]     # +-2 mm jitter < 3 mm tolerance

    client.status_hook = noise
    asyncio.run(robot.move_z(-5.0))
    assert robot._pose_unknown is False


# ----------------------------------------------------------------------
# failures and emergency stop
# ----------------------------------------------------------------------
def test_failed_move_marks_pose_unknown_without_automatic_go_home():
    client = FakeMaxArmClient()
    robot, _ = ready_robot(client)
    client.fail["move"] = FakeCommandError("UNREACHABLE")
    with pytest.raises(FakeCommandError):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert robot._pose_unknown
    assert len(client.moves) == 1, "no recovery motion after an error"


def test_firmware_watchdog_latch_is_not_cleared_silently():
    """A STOP the wrapper did not send (e.g. the firmware watchdog) is not cleared."""
    client = FakeMaxArmClient()
    robot, _ = ready_robot(client)
    client.stopped = True
    with pytest.raises(FakeCommandError, match="STOPPED"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert ("clear",) not in client.calls


@pytest.mark.parametrize("stop_raises", [False, True])
def test_estop_from_another_thread_mid_move_stops_the_flow(stop_raises):
    client = FakeMaxArmClient(block=True, stop_raises=stop_raises)
    robot, _ = ready_robot(client)
    asyncio.run(robot.suction(True))

    async def scenario():
        task = asyncio.ensure_future(robot.move_z(-10.0))
        while not client.started.is_set():
            await asyncio.sleep(0.01)
        threading.Thread(target=robot.emergency_stop).start()
        with pytest.raises(RuntimeError, match="緊急停止"):
            await task

    asyncio.run(scenario())
    assert client.force_stops == 1
    assert len(client.moves) == 1, "no automatic go_home after an emergency stop"
    assert robot._pose_unknown
    assert robot.suction_state is None
    assert ("pump", False) not in client.calls, "e-stop leaves the pump as it is"


def test_estop_between_moves_then_move_rereads_and_clears():
    robot, client = ready_robot()
    robot.emergency_stop()
    assert client.force_stops == 1 and robot._pose_unknown
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    kinds = [c[0] for c in client.calls]
    assert kinds.count("status") >= 2
    assert kinds.index("clear") < kinds.index("move")


def test_estop_survives_a_failing_client():
    robot, client = ready_robot()

    def boom():
        raise IOError("port gone")

    client.force_stop = boom
    robot.emergency_stop()               # must not raise
    assert robot._pose_unknown


# ----------------------------------------------------------------------
# go_home
# ----------------------------------------------------------------------
def test_go_home_raises_up_first_when_below_home():
    robot, client = ready_robot()
    asyncio.run(robot.move_xyz(50.0, -180.0, 100.0))
    client.moves.clear()
    asyncio.run(robot.go_home())
    # the lift stops at the highest z the arm's IK reaches at (50, -180): 210, not 212
    assert client.moves == [(50.0, -180.0, 210.0), ORIGIN]


def test_go_home_from_above_home_is_one_leg():
    robot, client = ready_robot()
    robot.home_position = [0.0, -162.8, 150.0]
    asyncio.run(robot.go_home())
    assert client.moves == [(0.0, -162.8, 150.0)]


def test_go_home_refuses_without_home():
    robot, client = ready_robot()
    robot.home_position = None
    with pytest.raises(RuntimeError, match="ホームポジションが設定されていません"):
        asyncio.run(robot.go_home())
    assert commands(client) == []


def test_go_home_validates_every_leg_before_the_first_command():
    # current pose outside the box (e.g. moved by hand); home itself is valid
    client = FakeMaxArmClient(pose=(0.0, -40.0, 100.0))
    robot, _ = make_robot(client, home_position=[0.0, -162.8, 212.0])
    asyncio.run(robot.initialize())
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.go_home())     # the current pose (0, -40, 100) is out of the box
    assert client.moves == []


def test_go_home_to_a_home_outside_limits_is_refused():
    robot, client = ready_robot()
    robot.home_position = [0.0, -162.8, 240.0]
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.go_home())
    assert client.moves == []


def test_go_home_after_a_cancel_rereads_first():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    client.block = False
    client.calls.clear()
    asyncio.run(robot.go_home())
    kinds = [c[0] for c in client.calls]
    assert kinds[:2] == ["status", "status"]
    assert kinds.index("clear") < kinds.index("move")


# ----------------------------------------------------------------------
# suction
# ----------------------------------------------------------------------
def test_suction_on_off_and_failure_makes_state_unknown():
    robot, client = ready_robot()
    asyncio.run(robot.suction(True))
    assert robot.suction_state is True and client.pump_on
    asyncio.run(robot.suction(False))
    assert robot.suction_state is False and not client.pump_on
    client.fail["pump"] = IOError("no reply")
    with pytest.raises(IOError):
        asyncio.run(robot.suction(True))
    assert robot.suction_state is None


# ----------------------------------------------------------------------
# cleanup
# ----------------------------------------------------------------------
def test_cleanup_turns_the_pump_off_then_closes_without_moving():
    robot, client = ready_robot()
    asyncio.run(robot.suction(True))
    client.calls.clear()
    asyncio.run(robot.cleanup())
    assert client.calls == [("pump", False), ("close",)]
    assert robot.client is None


def test_cleanup_still_closes_when_the_pump_fails():
    robot, client = ready_robot()
    client.fail["pump"] = IOError("gone")
    asyncio.run(robot.cleanup())
    assert client.calls[-1] == ("close",)
    assert robot.suction_state is None


def test_cleanup_reraises_an_interrupt_after_closing():
    robot, client = ready_robot()
    client.fail["pump"] = KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt):
        asyncio.run(robot.cleanup())
    assert client.calls[-1] == ("close",)


def test_cleanup_pump_off_is_configurable_and_skipped_when_disconnected():
    robot, client = ready_robot(pump_off_on_cleanup=False)
    asyncio.run(robot.cleanup())
    assert client.calls == [("close",)]

    robot, client = ready_robot()
    client.connected = False
    asyncio.run(robot.cleanup())
    assert client.calls == [("close",)]


def test_cleanup_survives_a_failing_close():
    robot, client = ready_robot()
    client.fail["close"] = IOError("gone")
    asyncio.run(robot.cleanup())         # logged, not raised
    assert robot.client is None


def test_context_manager_initializes_and_cleans_up():
    robot, client = make_robot()

    async def scenario():
        async with robot as r:
            assert r is robot
            assert robot._initialized

    asyncio.run(scenario())
    assert client.calls[-1] == ("close",)


# ======================================================================
# Regression tests for the MaxArm review findings (S#, D#, P# of the fix plan)
# ======================================================================

# ----------------------------------------------------------------------
# D12: move_tolerance_mm finite and > 0
# ----------------------------------------------------------------------
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 0, -1.0, "wide", None, True])
def test_move_tolerance_must_be_finite_and_positive(bad):
    with pytest.raises(ValueError, match="move_tolerance_mm"):
        MaxArmRobot("FAKE", workspace_validator=maxarm_validator(), move_tolerance_mm=bad)
    assert validate_move_tolerance(0.5) == 0.5 and validate_move_tolerance("3") == 3.0


def test_non_finite_read_back_after_a_move_is_a_failure_and_stops():
    client = FakeMaxArmClient(readback_offset=(float("nan"), 0.0, 0.0))
    robot, _ = ready_robot(client)
    with pytest.raises(RuntimeError, match="有限"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert robot._pose_unknown
    assert client.force_stops == 1


# ----------------------------------------------------------------------
# P2: stop latch at connect
# ----------------------------------------------------------------------
def test_initialize_clear_of_the_latch_is_a_warning_with_the_status_fields(caplog):
    client = FakeMaxArmClient(stopped=True)
    robot, _ = make_robot(client)
    with caplog.at_level("WARNING"):
        asyncio.run(robot.initialize())
    assert ("clear",) in client.calls
    warn = [r for r in caplog.records if r.levelname == "WARNING" and "CLEAR" in r.getMessage()]
    assert warn, caplog.text
    msg = warn[0].getMessage()
    assert "stopped=True" in msg and "moving=False" in msg and "x=0.0" in msg
    assert "fw=saigen-maxarm" in msg


def test_initialize_never_clears_the_latch_while_the_arm_is_moving():
    client = FakeMaxArmClient(stopped=True, moving=True)
    robot, _ = make_robot(client)
    asyncio.run(robot.initialize())
    assert ("clear",) not in client.calls
    assert robot._pose_unknown is True
    # once the arm is idle, the next move re-reads a settled pose, then CLEARs, then moves
    client.moving = False
    client.calls.clear()
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    kinds = [c[0] for c in client.calls]
    assert kinds.count("status") >= 2
    assert kinds.index("status") < kinds.index("clear") < kinds.index("move")


# ----------------------------------------------------------------------
# D2: STOP once on any non-validation move failure
# ----------------------------------------------------------------------
@pytest.mark.parametrize("error", [
    FakeCommandError("tolerance"),          # MaxArmMoveError: arrived out of tolerance
    TimeoutError("MOVE ack lost"),          # MaxArmTimeout: the MOVE may be executing
    IOError("serial write failed"),
])
@pytest.mark.parametrize("method, args", [
    ("move_xyz", (50.0, -180.0, 150.0)),
    ("move_z", (-30.0,)),
    ("go_home", ()),
])
def test_a_failed_move_is_force_stopped_exactly_once(method, args, error):
    robot, client = ready_robot()
    if method == "go_home":
        robot.home_position = [0.0, -162.8, 150.0]
    client.fail["move"] = error
    with pytest.raises(type(error)):
        asyncio.run(getattr(robot, method)(*args))
    assert client.force_stops == 1
    assert robot._pose_unknown
    assert len(client.moves) == 1, "no recovery motion after an error"
    kinds = [c[0] for c in client.calls]
    assert kinds.index("move") < kinds.index("force_stop")


def test_a_move_refused_by_a_foreign_latch_sends_no_stop_and_never_clears_it():
    """ERR STOPPED (e.g. the firmware watchdog fired): nothing moved, so no STOP; and
    the wrapper must not adopt the latch as its own and CLEAR it on the next move."""
    robot, client = ready_robot()
    client.stopped = True
    for _ in range(2):
        with pytest.raises(FakeCommandError, match="STOPPED"):
            asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert client.force_stops == 0
    assert ("clear",) not in client.calls


def test_an_unstable_pose_is_force_stopped():
    client = FakeMaxArmClient()
    robot, _ = ready_robot(client, pose_settle_reads=2)
    robot._pose_unknown = True
    client.status_hook = lambda c: c.pose.__setitem__(0, c.pose[0] + 5.0)
    with pytest.raises(RuntimeError, match="安定しません"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert client.force_stops == 1 and client.moves == []


# ----------------------------------------------------------------------
# S2: an e-stop anywhere inside a public motion op aborts it
# ----------------------------------------------------------------------
def test_estop_during_the_pose_reread_aborts_without_clear_or_move():
    """Review f2: GUI Stop lands during the settle read."""
    robot, client = ready_robot()
    robot._pose_unknown = True
    robot._stop_latched = True              # a CLEAR would be due after the re-read
    reads = {"n": 0}

    def hook(c):
        reads["n"] += 1
        if reads["n"] == 2:
            robot.emergency_stop()

    client.status_hook = hook
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert client.moves == [], "a MOVE was sent after the emergency stop"
    assert ("clear",) not in client.calls, "the e-stop latch was cleared"
    assert robot._pose_unknown and client.stopped
    assert client.force_stops == 1, "only the e-stop's own STOP"


@pytest.mark.parametrize("latched", [False, True])
def test_estop_just_before_prepare_move_is_not_cleared(latched):
    """Review f2b: another thread wins the race right before _prepare_move."""
    robot, client = ready_robot()
    robot._stop_latched = latched
    real_prepare = robot._prepare_move

    def interleaved(gen=None):
        robot.emergency_stop()
        real_prepare(gen)

    robot._prepare_move = interleaved
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert client.moves == []
    assert ("clear",) not in client.calls
    assert robot._pose_unknown and client.stopped


def test_estop_during_the_clear_relatches_and_does_not_move():
    robot, client = ready_robot()
    robot._stop_latched = True
    real_clear = client.clear

    def clear_then_estop():
        robot.emergency_stop()               # STOP reached the firmware first ...
        real_clear()                         # ... and our CLEAR removed it

    client.clear = clear_then_estop
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    assert client.moves == []
    assert client.stopped, "the latch must be re-established after the raced CLEAR"
    assert client.force_stops == 2 and robot._stop_latched


def test_estop_between_go_home_legs_sends_no_second_leg():
    robot, client = ready_robot(wait_after_move=0.3)
    asyncio.run(robot.move_xyz(50.0, -180.0, 100.0))       # below home: two legs
    client.moves.clear()
    client.finished.clear()

    async def scenario():
        task = asyncio.ensure_future(robot.go_home())
        t0 = time.monotonic()
        while not client.finished.is_set():
            assert time.monotonic() - t0 < 5
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.05)            # leg 1 done, now in wait_after_move
        robot.emergency_stop()
        with pytest.raises(RuntimeError, match="緊急停止"):
            await task

    asyncio.run(scenario())
    assert client.moves == [(50.0, -180.0, 210.0)], "leg 2 was sent after the e-stop"
    assert robot._pose_unknown


def test_estop_before_a_move_still_rereads_clears_and_moves():
    """The e-stop happened before the op started: gen is taken at the op's entry."""
    robot, client = ready_robot()
    robot.emergency_stop()
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    kinds = [c[0] for c in client.calls]
    assert kinds.index("clear") < kinds.index("move")
    assert robot._pose_unknown is False


# ----------------------------------------------------------------------
# S3: an abandoned worker blocks further motion until it returns
# ----------------------------------------------------------------------
def test_abandoned_worker_refuses_further_motion_until_it_returns():
    client = FakeMaxArmClient(block=True, release_on_stop=False)
    robot, _ = ready_robot(client, stop_wait_timeout=0.2)
    try:
        asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
        client.block = False
        n, calls = len(client.moves), len(client.calls)
        with pytest.raises(RuntimeError, match="まだ戻っていません"):
            asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
        assert len(client.moves) == n
        assert ("clear",) not in client.calls[calls:]
    finally:
        client.released.set()
    assert client.finished.wait(5)
    asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))        # worker gone: allowed again
    assert client.moves[-1] == (50.0, -180.0, 150.0)


# ----------------------------------------------------------------------
# D3 + P1: cleanup after an abnormal end
# ----------------------------------------------------------------------
def test_cleanup_after_estop_stops_first_and_keeps_suction(caplog):
    """Review f6: the e-stop kept suction; cleanup must not release the payload."""
    robot, client = ready_robot()
    asyncio.run(robot.suction(True))
    robot.emergency_stop()
    client.calls.clear()
    with caplog.at_level("WARNING"):
        asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("close",)]
    assert ("pump", False) not in client.calls, "payload released by cleanup after estop"
    assert client.pump_on
    assert SUCTION_KEPT_WARNING in caplog.text


def test_cleanup_after_a_failed_move_stops_first_and_keeps_suction(caplog):
    robot, client = ready_robot()
    asyncio.run(robot.suction(True))
    client.fail["move"] = IOError("ack lost")
    with pytest.raises(IOError):
        asyncio.run(robot.move_xyz(50.0, -180.0, 150.0))
    client.calls.clear()
    with caplog.at_level("WARNING"):
        asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("close",)]
    assert SUCTION_KEPT_WARNING in caplog.text


def test_cleanup_after_a_cancelled_move_stops_first():
    client = FakeMaxArmClient(block=True)
    robot, _ = ready_robot(client)
    asyncio.run(cancel_when_started(client, robot.move_z(-10.0), []))
    client.calls.clear()
    asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("close",)]


def test_pump_off_after_estop_option_releases_after_the_stop():
    robot, client = ready_robot(pump_off_after_estop=True)
    asyncio.run(robot.suction(True))
    robot.emergency_stop()
    client.calls.clear()
    asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("pump", False), ("close",)]
    assert robot.suction_state is False


def test_cleanup_stop_is_sent_even_when_the_client_reports_disconnected():
    robot, client = ready_robot()
    robot.emergency_stop()
    client.connected = False
    client.calls.clear()
    asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("close",)]


def test_a_clean_reconnect_after_an_estop_session_turns_the_pump_off_again():
    robot, client = ready_robot()
    robot.emergency_stop()
    asyncio.run(robot.cleanup())
    asyncio.run(robot.initialize())          # a new, clean connection
    client.calls.clear()
    asyncio.run(robot.cleanup())
    assert client.calls == [("pump", False), ("close",)]


# ----------------------------------------------------------------------
# P3: relative moves are based on the last commanded target (documented policy)
# ----------------------------------------------------------------------
def test_move_z_base_is_the_commanded_target_within_tolerance():
    """Review f5 / plan P3: the read-back (5, -160, 92) is within move_tolerance_mm of
    the target (0, -160, 100); move_z(-5) goes to (0, -160, 95) by design."""
    client = FakeMaxArmClient(readback_offset=(5.0, 0.0, -8.0))
    robot, client = ready_robot(client)
    asyncio.run(robot.move_xyz(0.0, -160.0, 100.0))
    client.moves.clear()
    asyncio.run(robot.move_z(-5))
    assert client.moves == [(0.0, -160.0, 95.0)]


# ----------------------------------------------------------------------
# S10 / R2-2: a cancel in the post-move delay leaves the operation unfinished
# (superseded the round-1 "known idle pose" behaviour: the pose is now unknown,
# so the next move re-reads it and cleanup treats the end as abnormal)
# ----------------------------------------------------------------------
def test_cancel_in_the_post_move_delay_marks_the_pose_unknown():
    robot, client = ready_robot(wait_after_move=1.0)

    async def scenario():
        task = asyncio.ensure_future(robot.move_xyz(50.0, -180.0, 150.0))
        await asyncio.sleep(0.3)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert client.moves == [(50.0, -180.0, 150.0)] and robot._pose_unknown
    assert client.force_stops == 0, "the move had finished: no STOP from the cancel itself"


# ----------------------------------------------------------------------
# S12 / P6: a configured home_position is honoured and validated
# ----------------------------------------------------------------------
def test_configured_home_is_honoured_and_validated():
    robot, client = make_robot(home_position=[0.0, -200.0, 150.0])
    asyncio.run(robot.initialize())
    assert robot.home_configured is True
    assert robot.home_is_within_limits() is True
    asyncio.run(robot.go_home())
    assert client.moves == [(0.0, -200.0, 150.0)]
    assert make_robot()[0].home_configured is False


def test_configured_home_outside_limits_is_reported_and_refused():
    robot, client = make_robot(home_position=[0.0, -200.0, 240.0])   # z > z_max
    asyncio.run(robot.initialize())
    assert robot.home_is_within_limits() is False
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.go_home())
    assert client.moves == []


def test_home_cannot_be_confirmed_without_a_validator():
    robot = MaxArmRobot("FAKE", home_position=[0.0, -200.0, 150.0])
    assert robot.home_is_within_limits() is False


# ======================================================================
# Round 2 (maxarm_fix_plan2.md, Agent S2)
# ======================================================================
def _wait_finished(client):
    assert client.finished.wait(5), "the move never finished"


# ----------------------------------------------------------------------
# R2-4: a failure after connect is an abnormal end (STOP first, suction kept)
# ----------------------------------------------------------------------
def test_r2_4_status_failure_at_init_stops_and_keeps_the_payload(caplog):
    client = FakeMaxArmClient(pump=True)
    client.fail["status"] = RuntimeError("status timeout")
    robot, _ = make_robot(client)
    with caplog.at_level("WARNING"):
        with pytest.raises(RuntimeError, match="status timeout"):
            asyncio.run(robot.initialize())
    assert ("pump", False) not in client.calls, "payload released after a failed init"
    assert client.force_stops == 1 and client.pump_on
    kinds = [c[0] for c in client.calls]
    assert kinds.index("force_stop") < kinds.index("close")
    assert SUCTION_KEPT_WARNING in caplog.text
    assert robot.client is None


def test_r2_4_clear_failure_at_init_stops_and_keeps_the_payload(caplog):
    client = FakeMaxArmClient(pump=True, stopped=True)
    client.fail["clear"] = RuntimeError("clear failed")
    robot, _ = make_robot(client)
    with caplog.at_level("WARNING"):
        with pytest.raises(RuntimeError, match="clear failed"):
            asyncio.run(robot.initialize())
    assert ("pump", False) not in client.calls
    assert client.force_stops == 1 and ("close",) in client.calls
    assert SUCTION_KEPT_WARNING in caplog.text


def test_r2_4_non_finite_status_at_init_is_an_abnormal_end():
    client = FakeMaxArmClient(pose=(float("nan"), -150.0, 150.0), pump=True)
    robot, _ = make_robot(client)
    with pytest.raises(RuntimeError, match="有限"):
        asyncio.run(robot.initialize())
    assert ("pump", False) not in client.calls and client.force_stops == 1


def test_r2_4_connect_failure_never_turns_the_pump_off():
    client = FakeMaxArmClient(pump=True)
    client.fail["connect"] = RuntimeError("no port")
    robot, _ = make_robot(client)
    with pytest.raises(RuntimeError, match="no port"):
        asyncio.run(robot.initialize())
    assert ("pump", False) not in client.calls
    assert ("close",) in client.calls and robot.client is None


def test_r2_4_successful_init_still_ends_normally():
    robot, client = ready_robot()
    assert robot._pose_unknown is False and robot.suction_state is False
    assert not robot.ended_abnormally
    asyncio.run(robot.cleanup())
    assert client.calls == [("pump", False), ("close",)]


def test_r2_4_estop_during_init_fails_the_init_and_keeps_suction():
    client = FakeMaxArmClient(pump=True)
    robot, _ = make_robot(client)
    client.status_hook = lambda c: robot.emergency_stop()     # "GUI Stop" mid-init
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(robot.initialize())
    assert ("pump", False) not in client.calls
    assert client.force_stops >= 1 and client.stopped
    assert robot.client is None


# ----------------------------------------------------------------------
# R2-6: an e-stop right before the latch bookkeeping keeps the latch owned
# ----------------------------------------------------------------------
def test_r2_6_estop_just_before_latch_reset_keeps_latch_ownership():
    import inspect
    from src.devices.safety import maxarm_robot as mr
    robot, client = ready_robot()
    robot._stop_latched = True
    client.stopped = True
    src, first = inspect.getsourcelines(mr.MaxArmRobot._prepare_move)
    line = first + next(i for i, l in enumerate(src) if "self._stop_latched = False" in l)
    code = mr.MaxArmRobot._prepare_move.__code__
    fired = []

    def tracer(frame, event, arg):
        if frame.f_code is code:
            def local(frame, event, arg):
                if event == "line" and frame.f_lineno == line and not fired:
                    fired.append(1)
                    robot.emergency_stop()             # "GUI thread" in the window
                return local
            return local
        return None

    gen = robot._estop_generation
    sys.settrace(tracer)
    try:
        with pytest.raises(RuntimeError, match="緊急停止"):
            robot._prepare_move(gen)
    finally:
        sys.settrace(None)
    assert fired and robot._estop_generation == gen + 1
    assert client.stopped is True and robot._stop_latched is True   # ownership kept
    assert robot._pose_unknown
    # the documented recovery works: the next operation re-reads, CLEARs and moves
    client.calls.clear()
    asyncio.run(robot.move_xyz(0.0, -150.0, 150.0))
    assert [c[0] for c in commands(client)] == ["clear", "move"]
    assert client.pose == [0.0, -150.0, 150.0] and not robot._pose_unknown


# ----------------------------------------------------------------------
# R2-2: e-stop / cancel during the post-move delay
# ----------------------------------------------------------------------
def _estop_after_move_finished(robot, client):
    _wait_finished(client)
    time.sleep(0.05)            # inside wait_after_move
    robot.emergency_stop()


def test_r2_2_estop_during_post_move_delay_fails_the_operation():
    robot, client = ready_robot(wait_after_move=0.5)
    t = threading.Thread(target=_estop_after_move_finished, args=(robot, client))
    t.start()
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(robot.move_xyz(0.0, -150.0, 150.0))
    t.join()
    assert robot._estop_generation == 1 and robot._pose_unknown is True
    assert client.force_stops == 1, "only the e-stop's own STOP"


def test_r2_2_estop_during_the_delay_of_move_z_and_go_home():
    for op in ("move_z", "go_home"):
        robot, client = ready_robot(wait_after_move=0.5)
        coro = robot.move_z(-10.0) if op == "move_z" else robot.go_home()
        t = threading.Thread(target=_estop_after_move_finished, args=(robot, client))
        t.start()
        with pytest.raises(RuntimeError, match="緊急停止"):
            asyncio.run(coro)
        t.join()
        assert robot._pose_unknown, op


def test_r2_2_gui_path_cancel_lands_in_the_delay():
    """GUI request_stop = emergency_stop_all() then call_soon_threadsafe(task.cancel)."""
    robot, client = ready_robot(wait_after_move=0.5)

    async def main():
        loop = asyncio.get_running_loop()
        task = asyncio.ensure_future(robot.move_xyz(0.0, -150.0, 150.0))

        def stop():
            _wait_finished(client)
            time.sleep(0.05)
            robot.emergency_stop()
            loop.call_soon_threadsafe(task.cancel)
        th = threading.Thread(target=stop)
        th.start()
        with pytest.raises((asyncio.CancelledError, RuntimeError)):
            await task
        th.join()
    asyncio.run(main())
    assert robot._pose_unknown is True and robot.ended_abnormally


@pytest.mark.parametrize("op", ["move_xyz", "move_z", "go_home"])
def test_r2_2_plain_cancel_in_delay_is_an_abnormal_end_that_keeps_suction(op, caplog):
    robot, client = ready_robot(wait_after_move=0.5)
    client.pump_on = True
    robot._suction = True

    async def main():
        coro = {"move_xyz": lambda: robot.move_xyz(0.0, -150.0, 150.0),
                "move_z": lambda: robot.move_z(-10.0),
                "go_home": lambda: robot.go_home()}[op]()
        task = asyncio.ensure_future(coro)
        t0 = time.monotonic()
        while not client.finished.is_set():
            assert time.monotonic() - t0 < 5
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.05)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(main())
    assert robot._pose_unknown is True and robot.ended_abnormally
    client.calls.clear()
    with caplog.at_level("WARNING"):
        asyncio.run(robot.cleanup())
    assert client.calls == [("force_stop",), ("close",)]
    assert SUCTION_KEPT_WARNING in caplog.text


def test_r2_2_next_move_after_a_cancel_in_the_delay_rereads_first():
    robot, client = ready_robot(wait_after_move=0.5)

    async def main():
        task = asyncio.ensure_future(robot.move_xyz(0.0, -150.0, 150.0))
        while not client.finished.is_set():
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.05)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(main())
    robot.wait_after_move = 0
    client.calls.clear()
    client.status_reads = 0
    asyncio.run(robot.move_xyz(0.0, -150.0, 160.0))
    assert client.status_reads >= 2, "pose was not re-read before the next move"
    assert [c[0] for c in commands(client)] == ["move"]


# ----------------------------------------------------------------------
# R2-3: cleanup's release decision and the pump command vs. a concurrent e-stop
# ----------------------------------------------------------------------
def test_r2_3_estop_after_classification_does_not_release_the_payload():
    robot, client = ready_robot()
    client.pump_on = True
    robot._suction = True
    orig = robot._worker_alive

    def hook():                                        # runs after `abnormal` was sampled
        robot.emergency_stop()
        return orig()
    robot._worker_alive = hook
    asyncio.run(robot.cleanup())
    assert ("force_stop",) in client.calls
    assert ("pump", False) not in client.calls, "payload released after a concurrent e-stop"
    assert client.pump_on and ("close",) in client.calls


def test_r2_3_estop_during_the_pump_off_waits_then_stops():
    """The pump-off is serialised with the e-stop's state change; STOP follows it."""
    robot, client = ready_robot()
    real_pump = client.pump
    th = {}

    def slow_pump(on):
        th["t"] = threading.Thread(target=robot.emergency_stop)
        th["t"].start()
        time.sleep(0.2)
        th["alive_during_pump"] = th["t"].is_alive()     # blocked on the state lock
        real_pump(on)

    client.pump = slow_pump
    asyncio.run(robot.cleanup())
    th["t"].join(5)
    assert th["alive_during_pump"] is True
    assert not th["t"].is_alive()
    assert ("pump", False) in client.calls


def test_r2_3_emergency_stop_sends_stop_outside_the_state_lock():
    robot, client = ready_robot()
    held = []

    def force_stop():
        # another thread must be able to take the lock while STOP is being sent
        got = []

        def probe():
            ok = robot._state_lock.acquire(timeout=1)
            if ok:
                robot._state_lock.release()
            got.append(ok)
        t = threading.Thread(target=probe)
        t.start()
        t.join(2)
        held.append(bool(got and got[0]))
        client.calls.append(("force_stop",))
        client.stopped = True
        return True

    client.force_stop = force_stop
    robot.emergency_stop()
    assert held == [True]


# ----------------------------------------------------------------------
# R2-5: initialize() refuses an already-initialised robot
# ----------------------------------------------------------------------
def test_r2_5_initialize_twice_is_refused_without_replacing_the_client():
    clients = []

    def factory(port):
        clients.append(FakeMaxArmClient())
        return clients[-1]
    robot = MaxArmRobot("FAKE", workspace_validator=maxarm_validator(),
                        client_factory=factory, pose_settle_interval=0, wait_after_move=0)
    asyncio.run(robot.initialize())
    with pytest.raises(RuntimeError, match="既に初期化"):
        asyncio.run(robot.initialize())
    assert len(clients) == 1 and robot.client is clients[0]
    assert clients[0].connected and ("close",) not in clients[0].calls
    asyncio.run(robot.cleanup())
    asyncio.run(robot.initialize())                    # after cleanup: allowed, fresh client
    assert len(clients) == 2 and robot.client is clients[1]
