"""InterruptibleDriverMixin alone, with a tiny concrete class (no hardware)."""
import asyncio
import threading
import time

import pytest

from src.devices.safety.interruptible import InterruptibleDriverMixin


class TinyArm(InterruptibleDriverMixin):
    def __init__(self, reads=3, stop_wait_timeout=5.0, release_on_stop=True):
        self._pose_unknown = False
        self._estop_generation = 0
        self.pose_settle_reads = reads
        self.pose_settle_interval = 0
        self.pose_settle_tolerance = 0.5
        self.stop_wait_timeout = stop_wait_timeout
        self._arm_name = "Tiny"
        self.pose = [1.0, 2.0, 3.0, 40.0]
        self.reads = 0
        self.stops = 0
        self.idle = True
        self.reread = []
        self.drift = 0.0
        self.released = threading.Event()
        self.started = threading.Event()
        self.release_on_stop = release_on_stop

    def _force_stop_arm(self):
        self.stops += 1
        if self.release_on_stop:
            self.released.set()
        return True

    def _driver_pose(self):
        self.reads += 1
        self.pose[0] += self.drift
        return list(self.pose)

    def _driver_reports_idle(self):
        return self.idle

    def _on_pose_reread(self, pose):
        self.reread.append(pose)

    def block(self, value="done"):
        self.started.set()
        self.released.wait(10)
        return value


async def _cancel_once_started(arm, coro):
    task = asyncio.ensure_future(coro)
    while not arm.started.is_set():
        await asyncio.sleep(0.01)
    task.cancel()
    t0 = time.monotonic()
    with pytest.raises(asyncio.CancelledError):
        await task
    return time.monotonic() - t0


def test_successful_call_returns_the_result_and_keeps_pose_known():
    arm = TinyArm()
    arm.released.set()
    assert asyncio.run(arm._run_blocking(arm.block, 42)) == 42
    assert arm._pose_unknown is False and arm.stops == 0


def test_cancel_force_stops_once_and_marks_pose_unknown():
    arm = TinyArm()
    asyncio.run(_cancel_once_started(arm, arm._run_blocking(arm.block)))
    assert arm.stops == 1
    assert arm._pose_unknown is True


def test_cancel_waits_at_most_stop_wait_timeout():
    arm = TinyArm(release_on_stop=False, stop_wait_timeout=0.2)
    try:
        assert asyncio.run(_cancel_once_started(arm, arm._run_blocking(arm.block))) < 3
        assert arm.stops == 1
    finally:
        arm.released.set()


def test_non_moving_call_keeps_pose_known_even_when_cancelled():
    arm = TinyArm()
    asyncio.run(_cancel_once_started(arm, arm._run_blocking(arm.block, moves_arm=False)))
    assert arm._pose_unknown is False


def test_failure_marks_pose_unknown_and_propagates():
    arm = TinyArm()

    def boom():
        raise IOError("serial")

    with pytest.raises(IOError):
        asyncio.run(arm._run_blocking(boom))
    assert arm._pose_unknown is True


def test_estop_during_a_normally_returning_call_raises():
    arm = TinyArm()

    def estopped():
        arm._estop_generation += 1       # emergency_stop() from another thread
        return "ok"

    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(arm._run_blocking(estopped))
    assert arm._pose_unknown is True


def test_estop_during_a_failing_call_is_reported_as_estop():
    arm = TinyArm()

    def estopped_then_fail():
        arm._estop_generation += 1
        raise IOError("tolerance")

    with pytest.raises(RuntimeError, match="緊急停止") as ei:
        asyncio.run(arm._run_blocking(estopped_then_fail))
    assert isinstance(ei.value.__cause__, IOError)


def test_read_pose_known_reads_once_without_settle():
    arm = TinyArm()
    assert asyncio.run(arm._read_pose()) == [1.0, 2.0, 3.0, 40.0]
    assert arm.reads == 1 and arm.reread == []


def test_read_pose_unknown_requires_two_matching_reads():
    arm = TinyArm()
    arm._pose_unknown = True
    pose = asyncio.run(arm._read_pose())
    assert arm.reads == 2 and arm._pose_unknown is False
    assert arm.reread == [pose]


def test_read_pose_unknown_refuses_a_drifting_arm():
    arm = TinyArm(reads=3)
    arm._pose_unknown = True
    arm.drift = 1.0
    with pytest.raises(RuntimeError, match="安定しません"):
        asyncio.run(arm._read_pose())
    assert arm._pose_unknown is True and arm.reads == 4


def test_read_pose_unknown_refuses_while_driver_reports_moving():
    arm = TinyArm(reads=2)
    arm._pose_unknown = True
    arm.idle = False
    with pytest.raises(RuntimeError, match="安定しません"):
        asyncio.run(arm._read_pose())


def test_pose_indices_hook_selects_the_compared_axes():
    class WithAngle(TinyArm):
        def _pose_indices(self, cur, prev):
            return [0, 1, 2, 3]

    arm = WithAngle(reads=2)
    arm._pose_unknown = True
    reads = iter([[1.0, 2.0, 3.0, 40.0], [1.0, 2.0, 3.0, 45.0], [1.0, 2.0, 3.0, 50.0]])
    arm._driver_pose = lambda: next(reads)
    with pytest.raises(RuntimeError):
        asyncio.run(arm._read_pose())    # the angle drifts: not settled

    plain = TinyArm(reads=2)             # default [0, 1, 2]: the angle is ignored
    plain._pose_unknown = True
    reads2 = iter([[1.0, 2.0, 3.0, 40.0], [1.0, 2.0, 3.0, 45.0]])
    plain._driver_pose = lambda: next(reads2)
    asyncio.run(plain._read_pose())
    assert plain._pose_unknown is False


# ----------------------------------------------------------------------
# e-stop generation across a whole public operation (review S2)
# ----------------------------------------------------------------------
class RacyArm(TinyArm):
    """An e-stop from another thread lands right after ``_pose_unknown = False``."""

    def __init__(self, **kw):
        self._racy_unknown = False
        self.race_armed = False
        super().__init__(**kw)

    @property
    def _pose_unknown(self):
        return self._racy_unknown

    @_pose_unknown.setter
    def _pose_unknown(self, value):
        self._racy_unknown = value
        if value is False and self.race_armed:
            self.race_armed = False
            self._estop_generation += 1      # emergency_stop(): generation first ...
            self._racy_unknown = True        # ... then pose unknown


def test_estop_right_after_run_blocking_marks_the_pose_known_is_not_lost():
    arm = RacyArm()
    arm.released.set()
    arm.race_armed = True
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(arm._run_blocking(arm.block))
    assert arm._pose_unknown is True


def test_estop_right_after_read_pose_marks_the_pose_known_is_not_lost():
    arm = RacyArm()
    arm._pose_unknown = True
    arm.race_armed = True
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(arm._read_pose(arm._estop_generation))
    assert arm._pose_unknown is True
    assert arm.reread == [], "the re-read pose must not be adopted"


def test_run_blocking_with_a_stale_generation_starts_no_worker():
    arm = TinyArm()
    called = []
    gen = arm._estop_generation
    arm._estop_generation += 1               # e-stopped earlier in the same operation
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(arm._run_blocking(lambda: called.append(1), estop_before=gen))
    assert called == []
    assert arm._pose_unknown is True


def test_read_pose_aborts_when_estopped_during_the_settle_reads():
    arm = TinyArm(reads=3)
    arm._pose_unknown = True
    gen = arm._estop_generation
    real = arm._driver_pose

    def pose_then_estop():
        pose = real()
        if arm.reads == 1:
            arm._estop_generation += 1
        return pose

    arm._driver_pose = pose_then_estop
    with pytest.raises(RuntimeError, match="緊急停止"):
        asyncio.run(arm._read_pose(gen))
    assert arm._pose_unknown is True and arm.reads == 1 and arm.reread == []


def test_read_pose_without_a_generation_keeps_the_old_behaviour():
    arm = TinyArm()
    arm._pose_unknown = True
    arm._estop_generation = 7
    asyncio.run(arm._read_pose())
    assert arm._pose_unknown is False


# ----------------------------------------------------------------------
# abandoned worker (review S3)
# ----------------------------------------------------------------------
def test_an_abandoned_worker_refuses_new_moves_until_it_returns():
    arm = TinyArm(release_on_stop=False, stop_wait_timeout=0.1)
    asyncio.run(_cancel_once_started(arm, arm._run_blocking(arm.block)))
    assert arm._worker_alive()
    with pytest.raises(RuntimeError, match="まだ戻っていません"):
        asyncio.run(arm._run_blocking(lambda: "second"))
    # a non-moving call (e.g. a status read) is still allowed
    assert asyncio.run(arm._run_blocking(lambda: "status", moves_arm=False)) == "status"
    arm.released.set()
    t0 = time.monotonic()
    while arm._worker_alive():
        assert time.monotonic() - t0 < 5
        time.sleep(0.01)
    assert asyncio.run(arm._run_blocking(lambda: "second")) == "second"


def test_a_worker_that_returned_is_never_reported_alive():
    arm = TinyArm()
    arm.released.set()
    for _ in range(20):                      # the worker's exit races the waiter's resume
        asyncio.run(arm._run_blocking(arm.block))
        assert not arm._worker_alive()


def test_repeated_cancellation_keeps_one_stop_and_keeps_waiting():
    arm = TinyArm(release_on_stop=False, stop_wait_timeout=5.0)

    async def scenario():
        task = asyncio.ensure_future(arm._run_blocking(arm.block))
        while not arm.started.is_set():
            await asyncio.sleep(0.01)
        task.cancel()
        await asyncio.sleep(0.05)            # now in _wait_worker
        task.cancel()                        # a second Ctrl+C
        await asyncio.sleep(0.05)
        assert not task.done(), "the second cancel must not abandon the worker early"
        arm.released.set()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert arm.stops == 1
    assert not arm._worker_alive()
