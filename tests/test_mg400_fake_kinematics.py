"""The MG400 fake's two planar models (fake-kinematics plan): the stand-in (default, unchanged)
and ``fitted_planar_v1``, a fitted planar approximation to one controller's PositiveSolution
answers, chosen explicitly by ``shared_devices.mock_kinematics``. No hardware; loopback only.
"""
import asyncio
import json
import math
import os
import time

import numpy as np
import pytest

from src import config as lab_config
from src.devices.mg400 import control
from src.devices.mg400.fake import KINEMATICS, FakeMG400, planar_fk, planar_ik
from src.devices.safety.mg400_robot import MockMG400Robot
from src.devices.safety.validators import WorkspaceValidator, WorkspaceViolationError
from src.flow import experiment_session as es
from src.flow import run_flow

from tests.test_mg400_wiring import EXAMPLE_FLOW, mock_main, restore_config  # noqa: F401 - fixture

FITTED = KINEMATICS["fitted_planar_v1"]
STANDIN = KINEMATICS["standin"]

#: The controller's answers to PositiveSolution(J1, J2, J3, J4, 0, 0), copied with their six
#: decimals from the read-only observe record ``observe_c2_rest`` (2026-11-04, controller 2.8.5).
#: Answers of one arm's controller, not measured positions; never generated from the fit.
RECORDED = [
    ((9.320353, -21.855961, 20.355614, -58.900452), (206.028458, 33.813602, 47.788395, -49.580101)),
    ((-30.000000, 0.000000, 0.000000, 0.000000), (246.445435, -142.285339, 121.207596, -30.000000)),
    ((-30.000000, 0.000000, 30.000000, 0.000000), (226.132797, -130.557831, 33.672245, -30.000000)),
    ((-30.000000, 30.000000, 0.000000, 0.000000), (321.879547, -185.837250, 97.868210, -30.000000)),
    ((-30.000000, 30.000000, 30.000000, 0.000000), (301.566895, -174.109726, 10.332857, -30.000000)),
    ((0.000000, 0.000000, 0.000000, 0.000000), (284.570679, 0.000000, 121.207596, 0.000000)),
    ((0.000000, 0.000000, 30.000000, 0.000000), (261.115662, 0.000000, 33.672245, 0.000000)),
    ((0.000000, 30.000000, 0.000000, 0.000000), (371.674500, 0.000000, 97.868210, 0.000000)),
    ((0.000000, 30.000000, 30.000000, 0.000000), (348.219452, 0.000000, 10.332857, 0.000000)),
    ((30.000000, 0.000000, 0.000000, 0.000000), (246.445435, 142.285339, 121.207596, 30.000000)),
    ((30.000000, 0.000000, 30.000000, 0.000000), (226.132797, 130.557831, 33.672245, 30.000000)),
    ((30.000000, 30.000000, 0.000000, 0.000000), (321.879547, 185.837250, 97.868210, 30.000000)),
    ((30.000000, 30.000000, 30.000000, 0.000000), (301.566895, 174.109726, 10.332857, 30.000000)),
]
REST_POSE = [206.03, 33.81, 47.79, -49.58]                 # the first answer, rounded as a config holds it


def _close(a, b, tol):
    return len(a) == len(b) and all(abs(u - v) <= tol for u, v in zip(a, b))


# ----------------------------------------------------------------------
# the fitted model against the controller's answers
# ----------------------------------------------------------------------
def test_a_refit_of_the_recorded_answers_reproduces_each_and_matches_the_constants():
    A, b = [], []
    for j, p in RECORDED:
        s2, c2 = math.sin(math.radians(j[1])), math.cos(math.radians(j[1]))
        s3, c3 = math.sin(math.radians(j[2])), math.cos(math.radians(j[2]))
        A += [[1, 0, s2, c3], [0, 1, c2, -s3]]
        b += [math.hypot(p[0], p[1]), p[2]]
    fit = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
    assert _close(fit, FITTED, 0.0005)
    for j, p in RECORDED:
        assert _close(planar_fk(tuple(fit), j), p, 0.001), (j, p)


def test_the_built_in_constants_reproduce_every_recorded_answer():
    for j, p in RECORDED:
        q = planar_fk(FITTED, j)
        assert _close(q[:3], p[:3], 0.001) and abs(q[3] - p[3]) <= 0.001, (j, p, q)


# ----------------------------------------------------------------------
# inverse: the chosen branch round-trips; refusals are documented, never clipped
# ----------------------------------------------------------------------
def _domain(model):
    """Non-singular joint sets of the chosen branch: 1 deg inside both annulus ends
    (0 < J3 + 90 - J2 < 180), J2 / J3 inside the range, a positive radius"""
    for j1 in (-150.0, -30.0, 0.0, 45.0, 150.0):
        for j2 in range(-20, 81, 10):
            for j3 in range(-20, 101, 10):
                bend = j3 + 90 - j2
                if not 1.0 <= bend <= 179.0:
                    continue
                j = (j1, float(j2), float(j3), 12.5)
                l0, _, l1, l2 = model
                if l0 + l1 * math.sin(math.radians(j2)) + l2 * math.cos(math.radians(j3)) <= 1.0:
                    continue
                yield j


@pytest.mark.parametrize("name", ["standin", "fitted_planar_v1"])
def test_ik_inverts_fk_on_the_chosen_branch(name):
    model = KINEMATICS[name]
    n = 0
    for j in _domain(model):
        p = planar_fk(model, j)
        got = planar_ik(model, *p[:3])
        assert got is not None and _close(got, j[:3], 1e-6), (j, got)
        n += 1
    assert n > 200


@pytest.mark.parametrize("name", ["standin", "fitted_planar_v1"])
def test_the_singular_ends_answer_the_same_pose_or_refuse(name):
    model = KINEMATICS[name]
    l0, h, l1, l2 = model
    for j in [(0.0, 80.0, -10.0, 0.0), (0.0, 60.0, -30.0 + 0.0, 0.0), (0.0, 0.0, 90.0, 0.0)]:
        p = planar_fk(model, j)                                # straight (bend 0) or folded (bend 180)
        got = planar_ik(model, *p[:3])
        if got is not None:
            assert _close(planar_fk(model, got + [0.0]), p[:3] + [got[0]], 1e-4)   # acos is flat here
    # just beyond the outer annulus end, along the arm: refused
    for j2 in (40.0, 60.0):
        t = math.radians(j2)
        a, b = (l1 + l2 + 0.01) * math.sin(t), (l1 + l2 + 0.01) * math.cos(t)
        assert planar_ik(model, l0 + a, 0.0, h + b) is None


@pytest.mark.parametrize("name", ["standin", "fitted_planar_v1"])
def test_refusals(name):
    model = KINEMATICS[name]
    l0, h, l1, l2 = model
    assert planar_ik(model, l0 + l1 + l2 + 5.0, 0.0, h) is None     # beyond reach
    assert planar_ik(model, l0, 0.0, h) is None                     # d == 0
    assert planar_ik(model, *planar_fk(model, (0.0, -30.0, 10.0, 0.0))[:3]) is None   # J2 below -25
    assert planar_ik(model, *planar_fk(model, (0.0, 20.0, -30.0, 0.0))[:3]) is None   # J3 below -25
    assert planar_ik(model, *planar_fk(model, (0.0, 90.0, 60.0, 0.0))[:3]) is None    # J2 above 85
    assert planar_ik(model, *planar_fk(model, (0.0, 30.0, 108.0, 0.0))[:3]) is None   # J3 above 105


def test_a_joint_set_with_a_negative_radius_is_not_returned():
    """r = L0 + L1 sin J2 + L2 cos J3 < 0 puts the flange behind the axis: ik never answers
    those joints (J1 comes from atan2 of a positive radius); what it answers has the same pose"""
    j = (10.0, -25.0, 105.0, 0.0)
    l0, _, l1, l2 = FITTED
    assert l0 + l1 * math.sin(math.radians(j[1])) + l2 * math.cos(math.radians(j[2])) < 0
    p = planar_fk(FITTED, j)
    got = planar_ik(FITTED, *p[:3])
    if got is not None:
        assert abs(got[0] - j[0]) > 90.0
        assert _close(planar_fk(FITTED, got + [0.0])[:3], p[:3], 1e-6)


def test_the_fitted_inner_end_is_refused_below_it():
    l0, h, l1, l2 = FITTED
    inner = abs(l1 - l2)
    assert planar_ik(FITTED, l0 + inner * 0.5, 0.0, h) is None


def test_the_stand_in_numbers_are_unchanged():
    """Values computed with the code before this change (fake.py at 9d79193)"""
    assert _close(FakeMG400.fk([0.071847, 10.278645, 16.846972, 18.555908]),
                  [398.715286977, 0.499975958, 221.473640094, 18.627755], 1e-8)
    assert _close(FakeMG400.fk([30.0, 40.0, 50.0, -10.0]), [368.039720486, 212.487831695, 100.0, 20.0], 1e-8)
    assert _close(FakeMG400.fk([-120.0, -20.0, 90.0, 5.0]),
                  [-70.073237459, -121.37040753, 89.446208638, -115.0], 1e-8)
    with FakeMG400() as fake:
        fake.angles[3] = 7.5
        assert _close(fake.ik(398.715287, 0.499976, 221.47364, 18.627755),
                      [0.071847006, 10.278645016, 16.846972029, 18.555907994], 1e-8)
        assert _close(fake.ik(420.0, 10.0, 200.0), [1.363927532, 19.258761557, 21.87667444, 7.5], 1e-8)
        assert _close(fake.ik(250.0, -150.0, 120.0, 3.0),
                      [-30.963756532, 3.205916554, 62.147012571, 33.963756532], 1e-8)
        assert fake.ik(700.0, 0.0, 100.0) is None
        assert fake.kinematics == "standin"


def test_an_unknown_model_is_refused():
    with pytest.raises(ValueError, match="unknown fake kinematics"):
        FakeMG400(kinematics="dobot")


# ----------------------------------------------------------------------
# the fake answers in its model: PositiveSolution, InverseSolution, MovL
# ----------------------------------------------------------------------
def test_every_answer_of_the_fake_follows_its_model(tmp_path):
    j = list(RECORDED[0][0])
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**_box()), record_folder=str(tmp_path / "m"),
                       start_pose=REST_POSE, kinematics="fitted_planar_v1", **_opts())
    try:
        fake = r.fake
        assert fake.kinematics == "fitted_planar_v1"
        assert _close(fake.angles[:4], j, 0.01)                         # the record's joints
        assert _close(fake.pose, REST_POSE, 1e-6)
        reply = fake.reply(f"PositiveSolution({j[0]},{j[1]},{j[2]},{j[3]},0,0)")
        nums = [float(v) for v in reply.split("{", 1)[1].split("}", 1)[0].split(",")]
        assert _close(nums, planar_fk(FITTED, j), 1e-5)
        reply = fake.reply(f"InverseSolution({REST_POSE[0]},{REST_POSE[1]},{REST_POSE[2]},{REST_POSE[3]},0,0)")
        nums = [float(v) for v in reply.split("{", 1)[1].split("}", 1)[0].split(",")]
        assert _close(nums, fake.ik(*REST_POSE), 1e-5)
        target = [REST_POSE[0], REST_POSE[1], REST_POSE[2] + 5.0]
        orig, seen = fake.ik, []

        def watched(x, y, z, rr=None):
            j = orig(x, y, z, rr)
            seen.append(([x, y, z], j))
            return j
        fake.ik = watched

        async def body():
            await r.initialize()
            await r.move_z(5.0)
            await r.cleanup()
        asyncio.run(body())
        assert len(fake.movl_received) == 1
        # MovL interpolation: every intermediate point on the straight segment, solved in the fitted model
        # (every ik call of the run is judged, whatever its coordinates: none may leave the segment)
        assert len([p for p, _ in seen if REST_POSE[2] + 1e-3 < p[2] < target[2] - 1e-3]) >= 10
        for p, j in seen:
            assert abs(p[0] - REST_POSE[0]) < 1e-3 and abs(p[1] - REST_POSE[1]) < 1e-3
            assert REST_POSE[2] - 1e-3 <= p[2] <= target[2] + 1e-3
            assert j is not None and _close(planar_fk(FITTED, j)[:3], p, 1e-6)
        # the final joints follow the fitted model
        assert _close(planar_fk(FITTED, fake.angles[:4])[:3], target, 0.01)
        assert abs(fake.angles[3] + fake.angles[0] - REST_POSE[3]) < 0.01
    finally:
        r.fake.close()


def test_without_a_selection_the_mock_is_the_stand_in_and_ik_model_off_is_unchanged(tmp_path):
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**_box()), record_folder=str(tmp_path / "m"),
                       **_opts())
    try:
        assert r.fake.kinematics == "standin" and r.kinematics == "standin"
    finally:
        r.fake.close()
    with FakeMG400(ik_model=False, kinematics="fitted_planar_v1", mode=5) as fake:
        def no_ik(*a, **k):
            raise AssertionError("ik called with ik_model off")
        fake.ik = no_ik
        reply = fake.reply("InverseSolution(206.03,33.81,47.79,-49.58,0,0)")
        nums = [float(v) for v in reply.split("{", 1)[1].split("}", 1)[0].split(",")]
        assert reply.startswith("0,") and _close(nums, fake.angles[:4], 1e-6)   # the current joints
        assert fake._movl_reply("MovL(206.03,33.81,47.79,-49.58,User=0,Tool=0,SpeedL=10,AccL=10)").startswith("-1,")


def test_the_fitted_model_without_a_start_pose_or_with_an_unsolvable_one_is_refused(tmp_path):
    with pytest.raises(ValueError, match="mock_start_pose が必要"):
        MockMG400Robot(workspace_validator=WorkspaceValidator(**_box()), record_folder=str(tmp_path / "a"),
                       kinematics="fitted_planar_v1", **_opts())
    with pytest.raises(ValueError, match="fitted_planar_v1") as info:
        MockMG400Robot(workspace_validator=WorkspaceValidator(**_box()), record_folder=str(tmp_path / "b"),
                       start_pose=[900.0, 0.0, 50.0, 0.0], kinematics="fitted_planar_v1", **_opts())
    assert "900.0" in str(info.value)
    with pytest.raises(ValueError, match="未知"):
        MockMG400Robot(workspace_validator=WorkspaceValidator(**_box()), record_folder=str(tmp_path / "c"),
                       start_pose=REST_POSE, kinematics="dobot", **_opts())


# ----------------------------------------------------------------------
# the selection: shared_devices.mock_kinematics, mock path only
# ----------------------------------------------------------------------
def test_the_selection_is_read_per_robot_and_fails_closed():
    assert es.mg400_mock_kinematics(5, shared_config={}) == "standin"
    assert es.mg400_mock_kinematics(5, shared_config={"mock_kinematics": {5: "fitted_planar_v1"}}) == "fitted_planar_v1"
    assert es.mg400_mock_kinematics(5, shared_config={"mock_kinematics": {"5": "fitted_planar_v1"}}) == "fitted_planar_v1"
    assert es.mg400_mock_kinematics(4, shared_config={"mock_kinematics": {5: "fitted_planar_v1"}}) == "standin"
    with pytest.raises(lab_config.ConfigError):
        es.mg400_mock_kinematics(5, shared_config={"mock_kinematics": "fitted_planar_v1"})
    for bad in ("fitted_planar_v1", None, [], {5: "dobot"}, {5: None}, {"5": None}, {5: []}, {5: {}}, {5: 1}):
        with pytest.raises(lab_config.ConfigError):
            es.mg400_mock_kinematics(5, shared_config={"mock_kinematics": bad})


def test_a_mock_start_pose_alone_keeps_the_stand_in(monkeypatch, tmp_path):
    cfg = {"mock_start_pose": {5: "420,10,200"}}
    monkeypatch.setattr(lab_config, "get_shared_devices", lambda: cfg)
    from tests.test_mg400_wiring import ports_with_mg400, validators_for
    ports = ports_with_mg400()
    robot = es.mock_robot_factory(5, use_picus2=False, ports=ports[5],
                                  workspace_validator=validators_for(ports)[5], record_folder=str(tmp_path / "r"))
    try:
        assert robot.fake.kinematics == "standin"
    finally:
        robot.fake.close()


def test_the_real_path_never_reads_the_selection(monkeypatch):
    from src.devices.safety import mg400_robot
    from tests.test_mg400_wiring import DOC_ADDRESS, ports_with_mg400, validators_for
    built = []

    class Stub:
        def __init__(self, **kw):
            built.append(kw)
    monkeypatch.setattr(mg400_robot, "MG400Robot", Stub)
    monkeypatch.setattr(es, "mg400_mock_kinematics", lambda *a, **k: pytest.fail("the real path read mock_kinematics"))
    monkeypatch.setattr(lab_config, "get_shared_devices", lambda: {"mock_kinematics": {5: "fitted_planar_v1"}})
    ports = ports_with_mg400(DOC_ADDRESS)
    es.default_robot_factory(5, use_picus2=False, ports=ports[5], workspace_validator=validators_for(ports)[5],
                             record_folder="unused", confirmed=False)
    assert len(built) == 1 and not any("kinematics" in k for k in built[0])


# ----------------------------------------------------------------------
# end to end: run_flow --mock of the example, in a config shaped like a measured cell
# ----------------------------------------------------------------------
def _box():
    x, y, z = REST_POSE[:3]
    return dict(x_min=x - 50.0, x_max=x + 50.0, y_min=y - 50.0, y_max=y + 50.0, z_min=z - 1.0, z_max=z + 50.0)


def _opts(**kw):
    o = dict(home_position=list(REST_POSE[:3]),
             joint_limits=[[-160, 160], [-25, 85], [-25, 105], [-180, 180]], speed_ratio=10, max_move_mm=50.0)
    o.update(kw)
    return o


def _measured_yaml(start):
    b = _box()
    return f"""robots:
  5:
    type: mg400
    mg400_address: ""
    workspace: {{x_min: {b['x_min']}, x_max: {b['x_max']}, y_min: {b['y_min']}, y_max: {b['y_max']}, z_min: {b['z_min']}, z_max: {b['z_max']}}}
    home_position: {list(REST_POSE[:3])}
    joint_limits: [[-160.0, 160.0], [-25.0, 85.0], [-25.0, 105.0], [-180.0, 180.0]]
    speed_ratio: 10
    max_move_mm: 50.0
shared_devices:
  mock_start_pose:
    5: {start}
  mock_kinematics:
    5: fitted_planar_v1
"""


def _the_report(logs):
    runs = [os.path.join(dp, d) for dp, dn, _ in os.walk(logs) for d in dn if d.startswith("mg400_5")]
    assert len(runs) == 1
    return json.load(open(os.path.join(runs[0], "report.json"), encoding="utf-8"))


@pytest.mark.parametrize("start, movl", [(list(REST_POSE), 2),
                                         ([REST_POSE[0] + 10.0, REST_POSE[1], REST_POSE[2], REST_POSE[3]], 3)])
def test_the_example_mock_runs_at_a_measured_pose(monkeypatch, restore_config, tmp_path, start, movl):  # noqa: F811
    cfg = tmp_path / "config.yaml"
    cfg.write_text(_measured_yaml(start), encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    monkeypatch.setattr(control, "confirm_tty", lambda *a, **k: pytest.fail("the mock run asked for Enter"))
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 0)
    report = _the_report(tmp_path / "logs")
    assert report["complete"] and report["disabled_confirmed"]
    assert len(report["control"]["moves"]) == movl
    assert report["flow"]["mock_kinematics"] == "fitted_planar_v1"
    track = report["control"]["tracking"]
    assert track and all(t["outcome"] == "ok" for t in track)
    poses = [p["pose"] for p in report["control"]["poses"]] if "poses" in report["control"] else []
    assert poses and all(abs(p[3] - REST_POSE[3]) < 0.01 for p in poses)


# ----------------------------------------------------------------------
# the wrapper's refusals with the fitted model (nothing relaxed)
# ----------------------------------------------------------------------
def _fitted(tmp_path, start=None, box=None, **kw):
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**(box or _box())), record_folder=str(tmp_path),
                       start_pose=start or REST_POSE, kinematics="fitted_planar_v1", **_opts(**kw))
    return r


def _refused(r, coro_fn, text=None):
    async def body():
        await r.initialize()
        try:
            with pytest.raises(Exception) as info:
                await coro_fn(r)
        finally:
            await r.cleanup()
        return info
    try:
        info = asyncio.run(body())
    finally:
        r.fake.close()
    if text is not None:
        assert text in str(info.value), str(info.value)
    assert r.fake.movl_received == [], r.fake.movl_received
    return info


def test_tightened_joint_limits_refuse_the_hop_before_its_movl(tmp_path):
    j = RECORDED[0][0]
    r = _fitted(tmp_path, joint_limits=[[-160, 160], [j[1] - 0.05, j[1] + 0.05], [j[2] - 0.05, j[2] + 0.05], [-180, 180]])
    info = _refused(r, lambda r: r.move_z(5.0))
    assert isinstance(info.value, WorkspaceViolationError)


def test_a_target_past_the_reach_is_refused_by_inverse_solution(tmp_path):
    joints = (0.0, 70.0, -15.0, 0.0)                           # nearly straight, 0.4 mm inside the reach
    start = planar_fk(FITTED, joints)
    assert planar_ik(FITTED, *start[:3]) is not None
    target = [start[0] + 5.0, start[1], start[2]]
    assert planar_ik(FITTED, *target) is None
    box = dict(x_min=start[0] - 20, x_max=start[0] + 20, y_min=-20.0, y_max=20.0, z_min=start[2] - 20, z_max=start[2] + 20)
    r = MockMG400Robot(workspace_validator=WorkspaceValidator(**box), record_folder=str(tmp_path),
                       start_pose=start, kinematics="fitted_planar_v1",
                       **_opts(home_position=list(start[:3])))
    info = _refused(r, lambda r: r.move_xyz(*target))
    assert isinstance(info.value, WorkspaceViolationError) and "InverseSolution" in str(info.value)


def test_a_joint_jump_between_samples_is_refused_with_the_fitted_model(tmp_path):
    r = _fitted(tmp_path)
    orig = r.fake.ik

    def jumpy(x, y, z, rr=None):
        j = orig(x, y, z, rr)
        if j is not None and z > REST_POSE[2] + 10:
            j = [j[0] + 6.0, j[1], j[2], j[3]]
        return j
    r.fake.ik = jumpy
    info = _refused(r, lambda r: r.move_z(20.0), "関節角")
    assert isinstance(info.value, WorkspaceViolationError)


def test_a_sag_at_the_gate_is_refused_before_the_movl(tmp_path):
    r = _fitted(tmp_path)

    async def body():
        await r.initialize()
        ctl = r._ctl

        def before_track(c):
            r.fake.sag = {2: -2.0}
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                with c.fb.lock:
                    p = c.fb.latest
                if p and p.get("q_actual") and abs(p["q_actual"][1] - p["q_target"][1] + 2.0) < 0.01:
                    return
                time.sleep(0.01)
        ctl.hooks["before_track"] = before_track
        try:
            with pytest.raises(Exception) as info:
                await r.move_z(5.0)
        finally:
            await r.cleanup()
        return info
    try:
        info = asyncio.run(body())
    finally:
        r.fake.close()
    assert "before MovL" in str(info.value), str(info.value)
    assert r.fake.movl_received == []
