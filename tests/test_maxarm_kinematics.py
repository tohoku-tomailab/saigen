"""PC-side MaxArm kinematics (src/devices/maxarm/kinematics.py) and the IK-aware
MaxArmWorkspaceValidator. The reference values were read from the arm's own
compiled solver (__espmax.inverse / deg_to_pulse / forward) at the REPL on
2026-09-24; the pre-check must agree with it, because its only purpose is to
predict what the firmware will refuse. No hardware."""
import math

import pytest

from src.devices.maxarm import kinematics as K
from src.devices.safety.validators import (
    WorkspaceValidator,
    WorkspaceViolationError,
    maxarm_workspace_validator_from_config,
    workspace_validator_from_config,
)
from src.devices.safety.validators.maxarm import MaxArmWorkspaceValidator
from src.flow import run_flow
from src.flow.experiment_session import validator_for_robot

# (target in the solver frame, i.e. x already mirrored) -> (angles deg, pulses) from the device
DEVICE_IK = {
    (-5.0, -185.0, 200.0): ((118.4518, 100.1154, 4.490344), (493.5492, 457.8524, 518.7098)),
    (0.0, -162.8, 212.0): ((120.0, 89.93856, 0.3321248), (500.0, 500.256, 501.3838)),
}
# refused by the device with ValueError("Invalid angle N"): joint 3 below 0 deg
DEVICE_REFUSED = {(-5.0, -185.0, 211.0): 0, (-5.0, -185.0, 221.0): -4, (-6.0, -185.0, 221.0): -4}
# forward kinematics of the resting pulses as the device computed it (solver frame)
DEVICE_FK = ((492, 454, 479), (-6.265042, -186.8893, 222.5473))

BOX = {"x_min": -250, "x_max": 250, "y_min": -280, "y_max": -60, "z_min": 40, "z_max": 225,
       "r_min": 60, "r_max": 260}


# ----------------------------------------------------------------------
# kinematics agree with the arm's own solver
# ----------------------------------------------------------------------
@pytest.mark.parametrize("target", sorted(DEVICE_IK))
def test_inverse_matches_the_device_solver_to_four_decimals(target):
    xs, y, z = target
    angles, pulses = DEVICE_IK[target]
    got = K.inverse(-xs, y, z)                # user frame: the arm mirrors x itself
    assert got == pytest.approx(angles, abs=1e-3)
    assert K.joints_to_pulses(got) == pytest.approx(pulses, abs=1e-3)


@pytest.mark.parametrize("target", sorted(DEVICE_REFUSED))
def test_targets_the_device_refused_are_refused_here_with_the_same_joint(target):
    xs, y, z = target
    err = K.reachability(-xs, y, z)
    assert err is not None and err.axis == "joint3"
    assert int(err.value) == DEVICE_REFUSED[target]         # "Invalid angle N" uses int()
    with pytest.raises(K.Unreachable):
        K.inverse(-xs, y, z)


def test_forward_matches_the_device_and_round_trips():
    pulses, solver_xyz = DEVICE_FK
    x, y, z = K.forward(pulses)
    assert (-x, y, z) == pytest.approx(solver_xyz, abs=1e-3)   # user frame mirrors x
    # inverse(forward(p)) == p wherever the pose is inside the joint range
    for p in ((500, 500, 520), (600, 450, 600), (400, 650, 550)):
        back = K.joints_to_pulses(K.inverse(*K.forward(p)))
        assert back == pytest.approx(p, abs=1e-6)


def test_pulse_angle_maps_are_inverses_and_bounded():
    for p in ((500, 500, 520), (600, 450, 600), (300, 800, 900)):
        assert K.joints_to_pulses(K.pulses_to_joints(p)) == pytest.approx(p, abs=1e-9)
    # the vendor gate is on angles: servo 3 at pulse 0 is -120 deg, refused although
    # the pulse itself is valid (that is how the resting pose ends up unreachable)
    assert K.pulses_to_joints((0, 0, 0)) == pytest.approx((0.0, 210.0, -120.0))
    with pytest.raises(K.Unreachable):
        K.joints_to_pulses(K.pulses_to_joints((0, 0, 0)))
    with pytest.raises(K.Unreachable):
        K.joints_to_pulses((-0.1, 100, 100))
    with pytest.raises(K.Unreachable):
        K.joints_to_pulses((100, 240.1, 100))
    with pytest.raises(ValueError):
        K.pulses_to_joints((1001, 0, 0))


def test_vendor_gates_radius_zmax_reach_and_non_finite():
    assert K.reachability(30, -30, 100).axis == "R"           # hypot < 50
    assert K.reachability(0, -200, 230).axis == "Z"           # z > 225 (firmware refuses)
    assert K.reachability(0, -300, 60).axis == "reach"        # beyond L2 + L3
    assert K.reachability(float("nan"), -200, 100) is not None
    assert K.reachability(math.inf, -200, 100) is not None
    assert K.is_reachable(120, -180, 150) and K.is_reachable(-120, -180, 70)
    assert K.is_reachable(0, -162.8, 212)                     # the vendor home pose


def test_highest_reachable_z_scans_downwards():
    assert K.highest_reachable_z(5, -185, 40, 225) == 210.0
    assert K.highest_reachable_z(120, -180, 40, 225) == 201.0
    assert K.highest_reachable_z(30, -30, 40, 225) is None     # inside R_MIN: never
    assert K.highest_reachable_z(5, -185, 40, float("nan")) is None


def test_the_resting_pose_after_factory_go_home_is_outside_the_ik_domain():
    """The factory program parks the arm where its own solver cannot go back to."""
    x, y, z = K.forward((492, 455, 481))
    assert 220 < z < 225 and K.reachability(x, y, z).axis == "joint3"


# ----------------------------------------------------------------------
# MaxArmWorkspaceValidator: box + radius first, then the IK gate (tighten only)
# ----------------------------------------------------------------------
def test_validator_accepts_reachable_and_rejects_unreachable_inside_the_box():
    v = MaxArmWorkspaceValidator(**BOX)
    v.validate_xyz(5, -185, 200)
    v.validate_xyz(120, -180, 150)
    with pytest.raises(WorkspaceViolationError) as e:
        v.validate_xyz(5, -185, 211)                            # inside the box, IK refuses
    msg = str(e.value)
    assert "逆運動学" in msg and "joint 3" in msg and "210" in msg   # hint: highest z there
    assert e.value.violations[0].axis == "joint3"
    assert e.value.target_position["highest_reachable_z"] == 210.0


def test_validator_keeps_the_base_box_and_radius_checks():
    v = MaxArmWorkspaceValidator(**BOX)
    plain = WorkspaceValidator(**BOX)
    for bad in ((300, -180, 150), (0, -50, 100), (0, -270, 30)):
        with pytest.raises(WorkspaceViolationError) as e1:
            plain.validate_xyz(*bad)
        with pytest.raises(WorkspaceViolationError) as e2:
            v.validate_xyz(*bad)
        assert [d.axis for d in e1.value.violations] == [d.axis for d in e2.value.violations]
    with pytest.raises(WorkspaceViolationError):
        v.validate_z_relative(200.0, 30.0)                     # z box still applies
    assert v.get_limits() == plain.get_limits()
    assert repr(v).startswith("MaxArm") and "[+IK]" in repr(v)


def test_factories_build_the_ik_aware_validator_for_maxarm_only():
    v = maxarm_workspace_validator_from_config(BOX)
    assert isinstance(v, MaxArmWorkspaceValidator)
    assert type(workspace_validator_from_config(BOX)) is WorkspaceValidator
    with pytest.raises(ValueError):
        maxarm_workspace_validator_from_config(dict(BOX, z_max=None))
    ports = {4: {"type": "maxarm", "maxarm_port": "COMX", "workspace": dict(BOX)},
             1: {"dobot_port": "COMY"}}
    assert isinstance(validator_for_robot(4, robot_ports=ports), MaxArmWorkspaceValidator)
    assert not isinstance(validator_for_robot(1, robot_ports=ports), MaxArmWorkspaceValidator)


# ----------------------------------------------------------------------
# go_home leg planning (real and mock drivers share plan_home_legs)
# ----------------------------------------------------------------------
def test_plan_home_legs_lifts_to_the_home_height_when_reachable():
    from src.devices.safety.maxarm_robot import plan_home_legs
    seen = []
    legs = plan_home_legs((120, -180, 70), (0, -162.8, 200), lambda *p: seen.append(p))
    assert legs == [(120.0, -180.0, 200.0), (0.0, -162.8, 200.0)]
    assert seen == legs, "every leg validated before any is returned"


def test_plan_home_legs_lifts_only_as_high_as_the_arm_reaches_there():
    """Home height 212 is reachable near the base only; at (-120, -180) the arm tops
    out at 201 mm, so the lift stops there (the example flow's last go_home)."""
    from src.devices.safety.maxarm_robot import plan_home_legs
    legs = plan_home_legs((-120, -180, 150), (0, -162.8, 212), lambda *p: None)
    assert legs == [(-120.0, -180.0, 201.0), (0.0, -162.8, 212.0)]


def test_plan_home_legs_skips_the_lift_from_an_unreachable_resting_pose():
    """Right after the factory go_home the arm rests where its solver cannot go (z~222):
    no lift leg, straight to home; the home itself is still validated."""
    from src.devices.safety.maxarm_robot import plan_home_legs
    legs = plan_home_legs((5, -185, 221), (0, -163, 225), lambda *p: None)
    assert legs == [(0.0, -163.0, 225.0)]
    v = MaxArmWorkspaceValidator(**BOX)
    with pytest.raises(WorkspaceViolationError):
        plan_home_legs((5, -185, 221), (5, -185, 221), v.validate_xyz)   # unreachable home
    legs = plan_home_legs((5, -185, 221), (0, -163, 200), v.validate_xyz)
    assert legs == [(0.0, -163.0, 200.0)]


def test_plan_home_legs_no_lift_when_already_at_or_above_home_height():
    from src.devices.safety.maxarm_robot import plan_home_legs
    assert plan_home_legs((50, -200, 200), (0, -163, 200), lambda *p: None) == [(0.0, -163.0, 200.0)]
    assert plan_home_legs((50, -200, 210), (0, -163, 200), lambda *p: None) == [(0.0, -163.0, 200.0)]


def test_mock_and_real_go_home_use_the_same_plan(monkeypatch):
    import asyncio
    from src.devices.safety.mock_maxarm import MockMaxArmRobot
    v = MaxArmWorkspaceValidator(**BOX)
    robot = MockMaxArmRobot(v, home_position=[0.0, -162.8, 212.0])

    async def run():
        await robot.initialize()
        await robot.move_xyz(-120, -180, 150)
        moves = []
        orig = robot._move_to

        async def spy(x, y, z, gen):
            moves.append((x, y, z))
            await orig(x, y, z, gen)
        monkeypatch.setattr(robot, "_move_to", spy)
        await robot.go_home()
        await robot.cleanup()
        return moves
    assert asyncio.run(run()) == [(-120.0, -180.0, 201.0), (0.0, -162.8, 212.0)]


def test_preflight_rejects_an_unreachable_move_xyz_on_the_maxarm_before_any_device():
    ports = {4: {"type": "maxarm", "maxarm_port": "COMX", "workspace": dict(BOX)}}
    steps = [{"action": "move_xyz", "robot_id": 4, "x": 5.0, "y": -185.0, "z": 211.0}]
    report = run_flow.run_preflight(steps, WorkspaceValidator(), robot_ports=ports)
    assert report.ok is False
    steps_ok = [{"action": "move_xyz", "robot_id": 4, "x": 5.0, "y": -185.0, "z": 200.0}]
    assert run_flow.run_preflight(steps_ok, WorkspaceValidator(), robot_ports=ports).ok is True
