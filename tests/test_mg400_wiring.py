"""Wiring of the MG400 robot type (``type: mg400``, flow plan stage two): config keys, the
schema's robot id range, the preflight, the CLI runner (confirmation before anything is
opened), the session (pre-registration, device reports), the GUI runner and the examples.
No hardware; the mock robot is the real wrapper on an in-process fake.
"""
import asyncio
import json
import os
import re
import threading
import time

import pytest

from src import config as lab_config
from src.devices.mg400 import control, protocol
from src.devices.mg400.fake import LOOPBACK, FakeMG400
from src.devices.safety.mg400_robot import MG400Robot, MockMG400Robot
from src.devices.safety.validators import WorkspaceValidator
from src.flow import run_flow
from src.flow import experiment_session as es
from src.flow.experiment_session import ExperimentSession, RobotStopError
from src.flow.schema import MAX_ROBOT_ID, ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_DIR = os.path.join(REPO_ROOT, "examples", "mg400")
EXAMPLE_FLOW = os.path.join(EXAMPLE_DIR, "z_hop.json")
EXAMPLE_CONFIG = os.path.join(EXAMPLE_DIR, "config.mg400.yaml")

BOX = {"x_min": 350.0, "x_max": 450.0, "y_min": -50.0, "y_max": 50.0,
       "z_min": 170.0, "z_max": 270.0, "r_min": 350.0, "r_max": 450.0}
MG400_OPTS = {"home_position": [400.0, 0.0, 220.0],
              "joint_limits": [[-160.0, 160.0], [-25.0, 85.0], [-25.0, 105.0], [-180.0, 180.0]],
              "speed_ratio": 10, "max_move_mm": 50.0}
#: an address of the documentation range (RFC 5737), never a lab machine; no socket is opened with it
DOC_ADDRESS = "192.0.2.5"
Z_HOP_STEPS = [{"action": "go_home", "robot_id": 5},
               {"action": "move_z", "robot_id": 5, "distance": 5.0},
               {"action": "wait", "robot_id": 5, "seconds": 0.0},
               {"action": "move_z", "robot_id": 5, "distance": -5.0},
               {"action": "go_home", "robot_id": 5}]


def ports_with_mg400(address=""):
    p5 = {"type": "mg400", "mg400_address": address, "workspace": dict(BOX)}
    p5.update(json.loads(json.dumps(MG400_OPTS)))
    return {1: {"type": "dobot", "dobot_port": "COM3", "picus2_address": "COM4"},
            3: {"dobot_port": "COM7"},
            5: p5}


def validators_for(ports):
    return {5: es.validator_for_robot(5, WorkspaceValidator(), ports)}


def mock_main(args, logs_dir, expected):
    """run_flow.main for an MG400 mock run; when the exit code is not the expected one, the
    assertion carries what the MG400 record says (phases, stop, feedback, notes), so a failure on
    a CI runner leaves evidence (the runner's temporary folder is gone afterwards)"""
    rc = run_flow.main(args)
    if rc != expected:
        found = []
        for dp, _, fs in os.walk(logs_dir):
            if "report.json" in fs and os.path.basename(dp).startswith("mg400_"):
                try:
                    rep = json.load(open(os.path.join(dp, "report.json"), encoding="utf-8"))
                except Exception as e:  # noqa: BLE001
                    found.append(f"{dp}: unreadable report.json: {e!r}")
                    continue
                found.append(json.dumps({k: rep.get(k) for k in ("phases", "stop", "stop_reason", "feedback", "notes")},
                                        ensure_ascii=False, default=str)[:6000])
        raise AssertionError(f"run_flow.main returned {rc}, expected {expected}; MG400 record: " + " | ".join(found))
    return rc


@pytest.fixture
def restore_config():
    original = (lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH)
    yield
    lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH = original
    lab_config.load_config(force_reload=True)


@pytest.fixture
def example_config(restore_config):
    """config.yaml = examples/mg400/config.mg400.yaml (what the CI copies)"""
    lab_config.CONFIG_PATH = EXAMPLE_CONFIG
    lab_config.load_config(force_reload=True)
    yield lab_config.get_robot_ports()


def write_config(tmp_path, text):
    cfg = tmp_path / "config.yaml"
    cfg.write_text(text, encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)


MG400_YAML = """robots:
  {rid}:
    type: mg400
    mg400_address: ""
    workspace: {{x_min: 350.0, x_max: 450.0, y_min: -50.0, y_max: 50.0, z_min: 170.0, z_max: 270.0}}
    home_position: [400.0, 0.0, 220.0]
    joint_limits: [[-160.0, 160.0], [-25.0, 85.0], [-25.0, 105.0], [-180.0, 180.0]]
    speed_ratio: 10
    max_move_mm: 50.0
"""


# ----------------------------------------------------------------------
# config
# ----------------------------------------------------------------------
def test_defaults_hold_no_mg400_and_the_type_is_known():
    assert "mg400" in lab_config.ROBOT_TYPES
    assert all(e.get("type") != "mg400" for e in lab_config.DEFAULTS["robots"].values())


def test_example_config_loads_robot5_as_mg400(example_config):
    p5 = example_config[5]
    assert p5["type"] == "mg400" and p5["mg400_address"] == ""
    assert lab_config.get_robot_type(5) == "mg400"
    ws = lab_config.get_robot_workspace(5)
    assert (ws["x_min"], ws["x_max"], ws["z_min"], ws["z_max"]) == (350.0, 450.0, 170.0, 270.0)
    assert ws["r_min"] == 350.0 and ws["r_max"] == 450.0
    # robots 1..3 and the Dobot workspace still come from the file/defaults as before
    assert lab_config.get_robot_type(1) == "dobot" and lab_config.get_robot_type(4) == "maxarm"


@pytest.mark.parametrize("missing", lab_config.MG400_REQUIRED_KEYS)
def test_a_missing_required_key_is_a_config_error_naming_it(tmp_path, restore_config, missing):
    lines = [ln for ln in MG400_YAML.format(rid=5).splitlines() if not ln.strip().startswith(missing + ":")]
    write_config(tmp_path, "\n".join(lines) + "\n")
    with pytest.raises(lab_config.ConfigError, match=missing):
        lab_config.get_robot_workspace(5)


def test_a_null_required_key_is_a_config_error_but_a_null_address_is_empty(tmp_path, restore_config):
    write_config(tmp_path, MG400_YAML.format(rid=5).replace('mg400_address: ""', "mg400_address:"))
    assert lab_config.get_robot_workspace(5)["x_min"] == 350.0
    assert lab_config.mg400_config(5, lab_config.get_robot_ports()[5])["mg400_address"] == ""
    write_config(tmp_path, MG400_YAML.format(rid=5).replace("speed_ratio: 10", "speed_ratio:"))
    with pytest.raises(lab_config.ConfigError, match=r"speed_ratio has no value \(null\)"):
        lab_config.get_robot_workspace(5)


def test_a_non_string_address_is_a_config_error(tmp_path, restore_config):
    write_config(tmp_path, MG400_YAML.format(rid=5).replace('mg400_address: ""', "mg400_address: 5"))
    with pytest.raises(lab_config.ConfigError, match="mg400_address"):
        lab_config.get_robot_workspace(5)


def test_an_mg400_written_at_the_maxarm_id_inherits_nothing(tmp_path, restore_config):
    """robots.4 retyped to mg400 must not carry the MaxArm's default workspace or move_ms"""
    write_config(tmp_path, MG400_YAML.format(rid=4).replace("    workspace: {x_min: 350.0, x_max: 450.0, y_min: -50.0, y_max: 50.0, z_min: 170.0, z_max: 270.0}\n", ""))
    p4 = lab_config.get_robot_ports()[4]
    assert p4["type"] == "mg400" and "move_ms" not in p4 and "maxarm_port" not in p4
    with pytest.raises(lab_config.ConfigError, match="workspace"):
        lab_config.get_robot_workspace(4)


def test_an_mg400_workspace_is_not_merged_with_the_dobot_box(tmp_path, restore_config):
    write_config(tmp_path, MG400_YAML.format(rid=5).replace("x_min: 350.0, ", ""))
    with pytest.raises(lab_config.ConfigError, match="x_min"):
        lab_config.get_robot_workspace(5)


def test_the_env_example_names_the_address_override():
    text = open(os.path.join(REPO_ROOT, ".env.example"), encoding="utf-8").read()
    assert "ROBOT5_MG400_ADDRESS=" in text


# ----------------------------------------------------------------------
# schema
# ----------------------------------------------------------------------
def test_robot_id_5_is_accepted_and_6_is_not():
    assert MAX_ROBOT_ID == 5
    ExperimentWorkflow(name="t", steps=[{"action": "move_z", "robot_id": 5, "distance": 1.0}])
    with pytest.raises(Exception):
        ExperimentWorkflow(name="t", steps=[{"action": "move_z", "robot_id": 6, "distance": 1.0}])


# ----------------------------------------------------------------------
# preflight
# ----------------------------------------------------------------------
def test_preflight_rejects_actions_the_mg400_cannot_do():
    ports = ports_with_mg400()
    for action, extra in (("aspirate", {"volume": 1.0, "speed": 5}), ("suction", {"on": True}),
                          ("rotate", {"angle": 10.0}), ("move_radial", {"distance": 1.0})):
        report = run_flow.run_preflight([{"action": action, "robot_id": 5, **extra}],
                                        WorkspaceValidator(), robot_ports=ports)
        assert not report.ok and any("mg400" in e for e in report.capability_errors), action
    report = run_flow.run_preflight(Z_HOP_STEPS, WorkspaceValidator(), robot_ports=ports)
    assert report.ok and report.robot_types == {5: "mg400"}
    assert report.validators_by_robot[5].get_limits()["x"]["min"] == 350.0


def test_preflight_checks_absolute_targets_against_the_mg400_box_only():
    ports = ports_with_mg400()
    inside = [{"action": "move_xyz", "robot_id": 5, "x": 420.0, "y": 10.0, "z": 200.0}]
    outside = [{"action": "move_xyz", "robot_id": 5, "x": 100.0, "y": 0.0, "z": 50.0}]   # a Dobot pose
    assert run_flow.run_preflight(inside, WorkspaceValidator(), robot_ports=ports).ok
    report = run_flow.run_preflight(outside, WorkspaceValidator(), robot_ports=ports)
    assert not report.ok and report.violations


def test_preflight_rejects_a_home_outside_the_box_or_malformed():
    ports = ports_with_mg400()
    ports[5]["home_position"] = [400.0, 0.0, 100.0]
    report = run_flow.run_preflight(Z_HOP_STEPS, WorkspaceValidator(), robot_ports=ports)
    assert not report.ok and any("home_position" in e for e in report.capability_errors)
    ports[5]["home_position"] = [400.0, 0.0]          # malformed: a configuration error before anything else
    with pytest.raises(lab_config.ConfigError, match="home_position"):
        run_flow.run_preflight(Z_HOP_STEPS, WorkspaceValidator(), robot_ports=ports)


@pytest.mark.parametrize("key, value", [("speed_ratio", 11), ("speed_ratio", 0), ("max_move_mm", 51.0),
                                        ("joint_limits", [[-170.0, 170.0], [-25.0, 85.0], [-25.0, 105.0], [-180.0, 180.0]]),
                                        ("joint_limits", [[-160.0, 160.0], [-25.0, 85.0], [-25.0, 105.0]]),
                                        ("home_position", [400.0, 0.0])])
def test_invalid_numeric_options_are_configuration_errors_before_any_device_opens(tmp_path, restore_config, key, value):
    """speed_ratio 1..10, max_move_mm 1..50, joint_limits inside the published envelope and
    home_position [x, y, z] are refused by the configuration (so by --validate-only and the
    preflight), not only by the robot's constructor"""
    ports = ports_with_mg400()
    ports[5][key] = value
    with pytest.raises(lab_config.ConfigError, match=key):
        run_flow.run_preflight(Z_HOP_STEPS, WorkspaceValidator(), robot_ports=ports)
    text = MG400_YAML.format(rid=5)
    line = next(ln for ln in text.splitlines() if ln.strip().startswith(key + ":"))
    write_config(tmp_path, text.replace(line, f"    {key}: {json.dumps(value)}"))
    with pytest.raises(lab_config.ConfigError, match=key):
        lab_config.get_robot_workspace(5)


def test_preflight_refuses_an_mg400_without_its_required_keys_before_anything_opens():
    ports = ports_with_mg400()
    del ports[5]["joint_limits"]
    with pytest.raises(lab_config.ConfigError, match="joint_limits"):
        run_flow.run_preflight(Z_HOP_STEPS, WorkspaceValidator(), robot_ports=ports)


def test_validate_only_exit_code_3_for_an_unsupported_action_on_the_mg400(tmp_path, example_config):
    flow = tmp_path / "bad.json"
    flow.write_text(json.dumps({"name": "t", "steps": [
        {"action": "aspirate", "robot_id": 5, "volume": 1.0, "speed": 5}]}), encoding="utf-8")
    assert run_flow.main([str(flow), "--validate-only"]) == run_flow.PREFLIGHT_EXIT_CODE


def test_validate_only_lists_the_relative_moves_as_unverified(example_config):
    report = run_flow.run_preflight(run_flow.load_and_validate(EXAMPLE_FLOW)[1], WorkspaceValidator(),
                                    robot_ports=lab_config.get_robot_ports())
    assert report.ok and len(report.unverified) == 2            # the two move_z


# ----------------------------------------------------------------------
# resolve_ports / CLI
# ----------------------------------------------------------------------
def test_resolve_ports_mg400_cli_and_env(monkeypatch, example_config):
    monkeypatch.delenv("ROBOT5_MG400_ADDRESS", raising=False)
    parser = run_flow.build_parser()
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert ports[5]["type"] == "mg400" and ports[5]["mg400_address"] == ""
    monkeypatch.setenv("ROBOT5_MG400_ADDRESS", DOC_ADDRESS)
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert ports[5]["mg400_address"] == DOC_ADDRESS
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json", "--robot5-mg400", "192.0.2.6"]))
    assert ports[5]["mg400_address"] == "192.0.2.6"
    # an address for a robot of another type does not change its type (the type decides; as for the MaxArm port)
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json", "--robot1-mg400", DOC_ADDRESS]))
    assert ports[1]["type"] == "dobot" and es.robot_type(ports[1]) == "dobot"


def test_confirmation_text_names_what_the_human_must_check():
    ports = ports_with_mg400(DOC_ADDRESS)
    text = run_flow.mg400_confirmation_text([5], ports, Z_HOP_STEPS)
    assert "WILL move" in text and DOC_ADDRESS in text
    assert "x 350.0..450.0" in text and "z 170.0..270.0" in text and "r 350.0..450.0" in text
    assert "SpeedL = AccL = 10" in text and "home [400.0, 0.0, 220.0]" in text and "max move 50.0 mm" in text
    assert "first MG400 step: go_home" in text
    assert "emergency stop is the button on the controller" in text and "NOT an emergency stop" in text
    assert "(not set)" in run_flow.mg400_confirmation_text([5], ports_with_mg400(""), Z_HOP_STEPS)


def test_a_declined_confirmation_opens_nothing_and_exits_2(monkeypatch, example_config, tmp_path):
    asked = []
    monkeypatch.setattr(control, "confirm_tty", lambda text, *a, **k: asked.append(text) or False)
    monkeypatch.setattr(run_flow, "ExperimentSession", lambda *a, **kw: pytest.fail("session created"))
    monkeypatch.setattr(run_flow, "ExperimentLogger", lambda *a, **kw: pytest.fail("run folder created"))
    monkeypatch.setattr(run_flow, "ensure_dashboard_running", lambda: pytest.fail("recording started"))
    assert run_flow.main([EXAMPLE_FLOW, "--robot5-mg400", DOC_ADDRESS, "--no-record"]) == 2
    assert len(asked) == 1 and "WILL move" in asked[0]


def test_an_accepted_confirmation_is_handed_to_the_session_with_the_run_folder(monkeypatch, example_config, tmp_path):
    monkeypatch.setattr(control, "confirm_tty", lambda text, *a, **k: True)
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    made = {}

    class FakeSession:
        def __init__(self, **kw):
            made["kwargs"], made["session"] = kw, self
            self.confirmed = set()
            self.status, self.error, self.robot_reports, self.robots = "completed", None, {}, {}

        def confirm_robot(self, rid):
            self.confirmed.add(rid)

        async def run(self, body):
            made["body"] = body

        def output_restore_report(self):
            return None

    monkeypatch.setattr(run_flow, "ExperimentSession", FakeSession)
    assert run_flow.main([EXAMPLE_FLOW, "--robot5-mg400", DOC_ADDRESS, "--no-record"]) == 0
    kw = made["kwargs"]
    assert kw["mock"] is False and kw["robot_ports"][5]["mg400_address"] == DOC_ADDRESS
    assert os.path.isdir(kw["record_root"]) and kw["record_root"].startswith(str(tmp_path / "logs"))
    assert "body" in made                       # the flow body was handed to the session
    assert made["session"].confirmed == {5}     # the Enter, handed to the session before anything was opened


def test_a_mock_run_asks_nothing_and_writes_the_record_under_the_run_folder(monkeypatch, example_config, tmp_path):
    monkeypatch.setattr(control, "confirm_tty", lambda *a, **k: pytest.fail("the mock run asked for Enter"))
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 0)
    runs = [os.path.join(dp, d) for dp, dn, _ in os.walk(tmp_path / "logs") for d in dn if d.startswith("mg400_5")]
    assert len(runs) == 1
    report = json.load(open(os.path.join(runs[0], "report.json"), encoding="utf-8"))
    assert report["complete"] and report["disabled_confirmed"] and report["tool"] == "src.devices.safety.mg400_robot"
    # go_home from the fake's start pose, +5, -5; the last go_home finds the arm within 1 mm and sends nothing
    assert 3 <= len(report["control"]["moves"]) <= 4
    run_dir = os.path.dirname(runs[0])
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "completed" and meta["robot_types"] == {"5": "mg400"}
    assert meta["robot_reports"]["5"]["complete"] is True and meta["robot_reports"]["5"]["failed_reason"] is None


def test_the_example_config_and_flow_hold_no_address():
    ip = re.compile(r"\b\d{1,3}(\.\d{1,3}){3}\b")
    for name in os.listdir(EXAMPLE_DIR):
        text = open(os.path.join(EXAMPLE_DIR, name), encoding="utf-8").read()
        assert not ip.search(text), name
    _, steps = run_flow.load_and_validate(EXAMPLE_FLOW)
    assert {s.get("robot_id") for s in steps} == {5}
    assert {s["action"] for s in steps} <= run_flow.ROBOT_ACTIONS_BY_TYPE["mg400"]


def test_the_ci_runs_the_example_against_the_example_config():
    text = open(os.path.join(REPO_ROOT, ".github", "workflows", "ci.yml"), encoding="utf-8").read()
    assert "examples/mg400/config.mg400.yaml" in text and "examples/mg400/z_hop.json" in text
    assert "--validate-only" in text and "--mock" in text


# ----------------------------------------------------------------------
# session
# ----------------------------------------------------------------------
def test_mock_session_builds_the_mock_mg400_with_its_record_under_the_run_folder(tmp_path):
    ports = ports_with_mg400()
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                robot_validators=validators_for(ports), record_root=str(tmp_path))

    async def body():
        robot = await session.add_robot(5, use_picus2=True)        # ignored with a warning
        assert isinstance(robot, MockMG400Robot)
        assert robot.workspace_validator is session.robot_validators[5]
        assert robot.record_folder == os.path.join(str(tmp_path), "mg400_5")
        assert robot.speed_ratio == 10 and robot.max_move_mm == 50.0
        assert list(robot.home_position) == [400.0, 0.0, 220.0]      # the configured home is kept
        z0 = robot.get_current_position()[2]
        await robot.move_z(5.0)
        assert abs(robot.get_current_position()[2] - (z0 + 5.0)) <= control.ARRIVE_TOL_MM
        return robot

    asyncio.run(session.run(body))
    assert session.status == "completed" and session.robot_types == {5: "mg400"}
    assert session.robot_reports[5]["complete"] and session.robot_reports[5]["failed_reason"] is None
    assert os.path.exists(os.path.join(str(tmp_path), "mg400_5", "report.json"))


def test_mock_session_without_a_record_root_uses_the_wrapper_default(monkeypatch, tmp_path):
    ports = ports_with_mg400()
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                robot_validators=validators_for(ports))
    built = []
    real = es.mock_robot_factory

    def factory(rid, **kw):
        robot = real(rid, **kw)
        built.append(robot)
        return robot
    session.robot_factory = factory
    monkeypatch.chdir(tmp_path)

    async def body():
        await session.add_robot(5)
    asyncio.run(session.run(body))
    assert session.status == "completed"
    folder = os.path.normpath(built[0].record_folder)
    assert folder.startswith(os.path.normpath(control.LOG_ROOT_FLOW)) and not os.path.isabs(folder)
    assert os.path.exists(os.path.join(str(tmp_path), folder, "report.json"))


def test_mock_start_pose_comes_from_the_per_robot_config_and_seeds_the_fake(monkeypatch, tmp_path):
    """As for the MaxArm: the Dobot's single MOCK_START_POSE is not used; config.yaml's
    shared_devices.mock_start_pose keyed by robot id is"""
    monkeypatch.setenv("MOCK_START_POSE", "200,0,50,0")
    assert es.mg400_mock_start_pose(5, shared_config={}) is None
    cfg = {"mock_start_pose": {5: "420,10,200", 1: "200,0,50,0"}}
    assert es.mg400_mock_start_pose(5, shared_config=cfg) == [420.0, 10.0, 200.0]
    assert es.mg400_mock_start_pose(5, shared_config={"mock_start_pose": "1,2,3,4"}) is None
    with pytest.raises(ValueError):
        es.mg400_mock_start_pose(5, shared_config={"mock_start_pose": {5: "1,2"}})
    monkeypatch.setattr(lab_config, "get_shared_devices", lambda: cfg)
    robot = es.mock_robot_factory(5, use_picus2=False, ports=ports_with_mg400()[5],
                                  workspace_validator=WorkspaceValidator(**BOX), record_folder=str(tmp_path / "r"))
    assert isinstance(robot, MockMG400Robot)
    x, y, z, _ = robot.fake.pose
    assert abs(x - 420.0) < 0.01 and abs(y - 10.0) < 0.01 and abs(z - 200.0) < 0.01
    robot.fake.close()


def test_real_session_refuses_an_mg400_without_an_address_before_building_anything():
    built = []
    session = ExperimentSession(mock=False, robot_ports=ports_with_mg400(""), workspace_validator=WorkspaceValidator(),
                                robot_factory=lambda *a, **kw: built.append(a))
    with pytest.raises(RuntimeError, match=r"--robot5-mg400.*ROBOT5_MG400_ADDRESS"):
        asyncio.run(session.add_robot(5))
    assert built == []


def test_default_factory_builds_the_mg400_with_record_and_confirmation(monkeypatch):
    import src.devices.safety.mg400_robot as mg400_robot

    class Recorder:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    monkeypatch.setattr(mg400_robot, "MG400Robot", Recorder)
    v = WorkspaceValidator(**BOX)
    robot = es.default_robot_factory(5, use_picus2=False, ports=ports_with_mg400(DOC_ADDRESS)[5],
                                     workspace_validator=v, record_folder="rec", confirmed=True)
    assert isinstance(robot, Recorder)
    assert robot.kwargs == {"mg400_address": DOC_ADDRESS, "workspace_validator": v, "record_folder": "rec",
                            "confirmed": True, **MG400_OPTS}
    # without the extras (another caller): not confirmed, default record folder
    robot = es.default_robot_factory(5, use_picus2=False, ports=ports_with_mg400(DOC_ADDRESS)[5], workspace_validator=v)
    assert robot.kwargs["confirmed"] is False and robot.kwargs["record_folder"] is None


def test_dobot_registration_is_unchanged_by_the_extras():
    """A replacement factory that knows no **extra still works for the Dobots (and the MaxArm)"""
    from src.devices.safety.mock_robot import MockLabRobot
    calls = []

    def factory(rid, *, use_picus2, ports, workspace_validator):
        calls.append(rid)
        return MockLabRobot(use_picus2=use_picus2, workspace_validator=workspace_validator)
    session = ExperimentSession(mock=True, robot_ports=ports_with_mg400(), workspace_validator=WorkspaceValidator(),
                                robot_factory=factory)

    async def body():
        await session.add_robot(1, use_picus2=True)
    asyncio.run(session.run(body))
    assert calls == [1] and session.status == "completed" and session.robot_reports == {}


def _real_mg400_session(fake, tmp_path, ports=None):
    """mock=False with the real class connected to ``fake``: the confirmation gate as a real run sees it"""
    port_map = {control.DASHBOARD_PORT: fake.dashboard_port, control.FEEDBACK_PORT: fake.feedback_port,
                control.MOTION_PORT: fake.motion_port}

    def connect(host, port, timeout):
        return control._connect_fixed(LOOPBACK, port_map[port], timeout)

    def factory(rid, *, use_picus2, ports, workspace_validator, **extra):
        return MG400Robot(ports["mg400_address"], workspace_validator=workspace_validator,
                          controller_factory=lambda address, folder: control.Controller(LOOPBACK, folder, connect=connect),
                          record_folder=extra.get("record_folder"), confirmed=extra.get("confirmed", False),
                          **es.robot_options(ports))
    ports = ports or ports_with_mg400(DOC_ADDRESS)
    return ExperimentSession(mock=False, robot_factory=factory, robot_ports=ports, shared_config={},
                             workspace_validator=WorkspaceValidator(), robot_validators=validators_for(ports),
                             record_root=str(tmp_path))


def test_a_real_run_without_the_confirmation_sends_no_control_command(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        session = _real_mg400_session(fake, tmp_path)
        with pytest.raises(RuntimeError, match="確認"):
            asyncio.run(session.add_robot(5))
        assert not (set(c.split("(", 1)[0] for c in fake.received) & set(protocol.CONTROL))
        assert fake.motion_received == [] and 5 not in session.robots


def test_a_confirmed_real_run_completes_and_reports(tmp_path):
    with FakeMG400(ik_model=True) as fake:
        session = _real_mg400_session(fake, tmp_path)
        session.confirm_robot(5)

        async def body():
            robot = await session.add_robot(5)
            await robot.go_home()
            await robot.move_z(3.0)
        asyncio.run(session.run(body))
        names = [c.split("(", 1)[0] for c in fake.received]
        assert names.count("EnableRobot") == 1 and names.count("DisableRobot") == 1 and "EmergencyStop" not in names
        assert [c.split("(", 1)[0] for c in fake.motion_received] == ["MovL", "MovL"]
        assert session.status == "completed"
        assert session.robot_reports[5]["complete"] and session.robot_reports[5]["record_folder"] == os.path.join(str(tmp_path), "mg400_5")


def test_a_stop_before_initialize_returns_is_a_latch_and_later_robots_are_refused(tmp_path):
    """The MG400 is on the stop list before initialize() (flow plan F6): a Stop while another
    robot's initialize is still pending reaches it before any connection, and add_robot
    refuses every robot after the stop"""
    ports = ports_with_mg400()
    gate = threading.Event()
    stopped_at = {}

    class SlowMG400(MockMG400Robot):
        async def initialize(self):
            await asyncio.get_running_loop().run_in_executor(None, gate.wait, 10)
            return await super().initialize()

    def factory(rid, *, use_picus2, ports, workspace_validator, **extra):
        return SlowMG400(workspace_validator=workspace_validator, record_folder=extra.get("record_folder"),
                         **es.robot_options(ports))
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                robot_validators=validators_for(ports), robot_factory=factory,
                                record_root=str(tmp_path))

    def stopper():
        gate.wait(0.0)
        session.emergency_stop_all()                      # from the GUI's thread, during initialize()
        stopped_at["initializing"] = dict(session._initializing)
        gate.set()
    threading.Timer(0.3, stopper).start()

    async def body():
        await session.add_robot(5)
    with pytest.raises(RuntimeError, match="接続前に停止要求"):
        asyncio.run(session.run(body))
    assert 5 in stopped_at["initializing"]
    robot = stopped_at["initializing"][5]
    assert robot.fake.received == [] and robot.ended_abnormally
    with pytest.raises(RuntimeError, match="停止要求の後"):
        asyncio.run(session.add_robot(1))
    assert session.status == "failed"


def test_go_home_all_skips_the_mg400():
    class R:
        ROBOT_TYPE = "mg400"
        homed = 0

        async def go_home(self):
            self.homed += 1
    session = ExperimentSession(mock=True, robot_ports=ports_with_mg400(), workspace_validator=WorkspaceValidator())
    session.robots[5] = R()
    session._robot_types[5] = "mg400"
    asyncio.run(session.go_home_all())
    assert session.robots[5].homed == 0


class ReportingRobot:
    """A registered robot whose run_report() the session must apply after cleanup"""
    ROBOT_TYPE = "mg400"

    def __init__(self, report):
        self.report, self.cleaned, self.stops = report, 0, 0
        self.home_position = [400.0, 0.0, 220.0]

    async def initialize(self):
        return True

    def home_is_within_limits(self):
        return True

    async def go_home(self):
        pass

    def emergency_stop(self):
        self.stops += 1

    async def cleanup(self):
        self.cleaned += 1

    def run_report(self):
        return self.report


def _reporting_session(report):
    ports = ports_with_mg400(DOC_ADDRESS)
    return ExperimentSession(mock=False, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                             robot_validators=validators_for(ports),
                             robot_factory=lambda rid, **kw: ReportingRobot(report))


def test_a_failed_device_report_after_a_clean_body_fails_the_run_with_robot_stop_error():
    session = _reporting_session({"failed_reason": "stop sequence ended in unconfirmed: check the arm",
                                  "stop_state": "unconfirmed"})

    async def body():
        await session.add_robot(5)
    with pytest.raises(RobotStopError, match="unconfirmed"):
        asyncio.run(session.run(body))
    assert session.status == "failed" and "Robot 5" in session.error
    assert session.robot_reports[5]["stop_state"] == "unconfirmed"


def test_a_failed_device_report_after_a_failed_body_keeps_the_body_error_and_adds_the_note():
    session = _reporting_session({"failed_reason": "the arm's controller stopped the run on its own: alarm"})

    async def body():
        await session.add_robot(5)
        raise RuntimeError("step 3 failed")
    with pytest.raises(RuntimeError, match="step 3 failed"):
        asyncio.run(session.run(body))
    assert session.status == "failed" and "step 3 failed" in session.error and "alarm" in session.error


def test_a_clean_device_report_is_kept_and_raises_nothing():
    session = _reporting_session({"complete": True, "failed_reason": None, "stop_state": None})

    async def body():
        await session.add_robot(5)
    asyncio.run(session.run(body))
    assert session.status == "completed" and session.robot_reports == {5: {"complete": True, "failed_reason": None,
                                                                          "stop_state": None}}


def test_an_interrupted_run_keeps_aborted_with_the_note():
    session = _reporting_session({"failed_reason": "stop sequence ended in emergency_stop: x"})

    async def body():
        await session.add_robot(5)
        raise asyncio.CancelledError()
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(session.run(body))
    assert session.status == "aborted" and "emergency_stop" in session.error


# ----------------------------------------------------------------------
# logger
# ----------------------------------------------------------------------
def test_logger_writes_the_robot_reports_into_the_metadata(tmp_path):
    from src.flow.experiment_logger import ExperimentLogger
    src_json = tmp_path / "f.json"
    src_json.write_text("{}", encoding="utf-8")
    lg = ExperimentLogger("t", "", str(src_json), base_dir=str(tmp_path / "logs"), mode="mock")
    lg.record_resources([5], set(), robot_types={5: "mg400"})
    lg.record_robot_reports({5: {"complete": True, "stop_state": None, "record_folder": "x"}})
    lg.finalize("completed")
    meta = json.load(open(lg.metadata_path, encoding="utf-8"))
    assert meta["robot_types"] == {"5": "mg400"}
    assert meta["robot_reports"] == {"5": {"complete": True, "stop_state": None, "record_folder": "x"}}


# ----------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------
def test_gui_settings_carry_the_mg400_address_and_its_own_workspace(example_config):
    from src.gui import runner as gui_runner
    settings = gui_runner.default_device_settings()
    r5 = settings["robots"][5]
    assert r5["type"] == "mg400" and not r5["use_picus2"] and r5["mg400_address"] == ""
    r5["mg400_address"] = DOC_ADDRESS
    robot_ports, _ = gui_runner.settings_to_session_args(settings)
    assert robot_ports[5]["type"] == "mg400" and robot_ports[5]["mg400_address"] == DOC_ADDRESS
    assert robot_ports[5]["workspace"] == example_config[5]["workspace"]
    assert robot_ports[5]["home_position"] == [400.0, 0.0, 220.0] and robot_ports[5]["speed_ratio"] == 10
    assert "dobot_port" not in robot_ports[5] and "maxarm_port" not in robot_ports[5]


def test_gui_runner_rejects_an_unsupported_action_on_the_mg400_before_anything_exists():
    from src.gui import runner as gui_runner
    with pytest.raises(run_flow.PreflightError):
        gui_runner.FlowRunner({"name": "t", "steps": [{"action": "suction", "robot_id": 5}]},
                              mock=True, robot_ports=ports_with_mg400())


def test_gui_runner_runs_the_example_in_mock_mode_with_the_record_in_the_run_folder(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE_FLOW, encoding="utf-8"))
    flow["steps"] = [s if s["action"] != "wait" else {**s, "seconds": 0.0} for s in flow["steps"]]
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path / "logs"), source_path=EXAMPLE_FLOW,
                                   robot_ports=ports_with_mg400())
    runner.start()
    assert runner.join(120), "the mock run did not finish in time"
    assert runner.status == "completed", runner.error
    assert os.path.exists(os.path.join(runner.log_dir, "mg400_5", "report.json"))
    meta = json.load(open(os.path.join(runner.log_dir, "metadata.json"), encoding="utf-8"))
    assert meta["robot_reports"]["5"]["complete"] is True


def test_gui_app_knows_robot_5_as_the_mg400():
    import ast
    src = open(os.path.join(REPO_ROOT, "src", "gui", "app.py"), encoding="utf-8").read()
    tree = ast.parse(src)
    values = {t.id: n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    colors = ast.literal_eval(values["ROBOT_COLORS"])
    assert "MG400" in colors[5]["name"]
    assert "MG400 Address" in src and "mg400_address" in src
    assert "start_run(mock=False, confirmed_robots=mg400_used)" in src    # the dialog hands its approval on (F7)


# ----------------------------------------------------------------------
# LLM prompt
# ----------------------------------------------------------------------
def test_llm_prompt_describes_robot5_and_its_four_actions():
    from src.agent import llm_service
    assert "Robot 5 (robot_id: 5)" in llm_service.SYSTEM_PROMPT and "MG400" in llm_service.SYSTEM_PROMPT
    assert "move_xyz / move_z / go_home / wait" in llm_service.SYSTEM_PROMPT


# ----------------------------------------------------------------------
# F6 through the session, the CLI and the GUI (review 1)
# ----------------------------------------------------------------------
def test_a_stop_during_home_setup_reaches_the_mg400_and_refuses_the_registration(tmp_path, monkeypatch):
    """A Stop between initialize() returning and the registration: the robot is still on the
    stop list, the latch reaches the enabled arm (stop sequence), the registration is refused,
    and the device report is kept although the robot was never registered"""
    ports = ports_with_mg400()
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                robot_validators=validators_for(ports), record_root=str(tmp_path))
    built = []
    real = es.mock_robot_factory

    calls = []

    def factory(rid, **kw):
        r = real(rid, **kw)
        built.append(r)
        estop, cleanup = r.emergency_stop, r.cleanup

        def spy_estop():
            calls.append("emergency_stop")
            estop()

        async def spy_cleanup():
            calls.append("cleanup")
            await cleanup()
        r.emergency_stop, r.cleanup = spy_estop, spy_cleanup
        return r
    session.robot_factory = factory
    original = session._set_home

    def stopping_set_home(robot_id, robot, rtype, ports_):
        session.emergency_stop_all()                      # the GUI's Stop, at this very moment
        original(robot_id, robot, rtype, ports_)
    monkeypatch.setattr(session, "_set_home", stopping_set_home)

    async def body():
        await session.add_robot(5)
    with pytest.raises(RuntimeError, match="停止要求"):
        asyncio.run(session.run(body))
    assert 5 not in session.robots and session.status == "failed"
    rep = session.robot_reports[5]
    assert rep["stopped"] and rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True
    assert built[0].fake.mode == 4 and built[0].fake.motion_received == []
    # the refusal delivers the stop (again, idempotent) before it disconnects
    assert calls[:3] == ["emergency_stop", "emergency_stop", "cleanup"]


class _StubRobot:
    """A Dobot / MaxArm stand-in that records what the session does with it"""
    def __init__(self, calls):
        self.calls = calls
        self.home_position = None

    async def initialize(self):
        return True

    def set_current_position_as_home(self):
        self.calls.append("set_home")

    def emergency_stop(self):
        self.calls.append("emergency_stop")

    async def cleanup(self):
        self.calls.append("cleanup")


@pytest.mark.parametrize("rtype", ["dobot", "maxarm"])
def test_a_stop_during_home_setup_of_a_dobot_or_maxarm_keeps_the_old_registration(tmp_path, monkeypatch, rtype):
    """Only the pre-registered type (MG400) is refused after a stop during its home setup. A
    Dobot or MaxArm is registered as before stage two, so the run's interruption handler stops
    it through emergency_stop_all(); it is not disconnected as a normal end (a MaxArm would
    release its suction then)"""
    ports = {1: {"type": rtype, "dobot_port": "COM3", "maxarm_port": "COM9",
                 "workspace": dict(x_min=100.0, x_max=200.0, y_min=-50.0, y_max=50.0, z_min=0.0, z_max=100.0)}}
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                record_root=str(tmp_path))
    calls = []
    session.robot_factory = lambda rid, **kw: _StubRobot(calls)
    original = session._set_home

    def stopping_set_home(robot_id, robot, rtype_, ports_):
        session.emergency_stop_all()
        original(robot_id, robot, rtype_, ports_)
    monkeypatch.setattr(session, "_set_home", stopping_set_home)
    robot = asyncio.run(session.add_robot(1))
    assert session.robots[1] is robot
    assert calls == ["set_home"]                          # registered, not disconnected
    session.emergency_stop_all(skip_already_stopped=True)  # what run()'s interruption handler does
    assert calls == ["set_home", "emergency_stop"]


def test_the_report_of_an_unregistered_robot_is_read_when_the_reports_are_collected(tmp_path):
    """The report of a robot whose registration failed is read at collection time (after the
    cleanups), not when it was set aside: a shutdown that went on meanwhile is reported as ended"""
    session = ExperimentSession(mock=True, robot_ports={}, workspace_validator=WorkspaceValidator(),
                                record_root=str(tmp_path))

    class Late:
        report = {"stop_state": None, "worker_alive": True, "failed_reason": None}

        def run_report(self):
            return dict(self.report)
    late = Late()
    session._keep_failed_report(5, late)
    late.report = {"stop_state": "reset_disable", "worker_alive": False, "failed_reason": None}
    session._apply_robot_reports(False, None)
    assert session.robot_reports[5]["stop_state"] == "reset_disable"
    assert session.robot_reports[5]["worker_alive"] is False


def test_a_failed_home_setup_keeps_the_mg400_report(tmp_path):
    ports = ports_with_mg400()
    ports[5]["home_position"] = [400.0, 0.0, 100.0]               # inside the published envelope, outside the box
    session = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=WorkspaceValidator(),
                                robot_validators=validators_for(ports), record_root=str(tmp_path))

    async def body():
        await session.add_robot(5)
    with pytest.raises(RuntimeError, match="home_position"):
        asyncio.run(session.run(body))
    assert 5 not in session.robots and session.status == "failed"
    rep = session.robot_reports[5]
    assert rep["disabled_confirmed"] and rep["failed_reason"] is None and rep["record_folder"].endswith("mg400_5")


def test_an_interruption_during_a_move_stops_the_arm_and_the_run_records_it(monkeypatch, example_config, tmp_path):
    """Ctrl+C / a cancel while a MovL is under way, through run_flow: exit 130, the stop
    sequence from the session's emergency stop, the stop in metadata.json"""
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    real = run_flow.execute_step
    started = []

    async def interrupting(step, robots, shared, log):
        if step.get("action") == "move_z" and not started:
            started.append(asyncio.ensure_future(real(step, robots, shared, log)))
            await asyncio.sleep(0.3)                           # the MovL was sent; the fake "moves"
            raise asyncio.CancelledError()
        return await real(step, robots, shared, log)
    monkeypatch.setattr(run_flow, "execute_step", interrupting)
    mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 130)
    run_dir = next(os.path.join(dp, d) for dp, dn, _ in os.walk(tmp_path / "logs") for d in dn if d.startswith("mg400_5"))
    meta = json.load(open(os.path.join(os.path.dirname(run_dir), "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "aborted"
    rep = meta["robot_reports"]["5"]
    assert rep["stopped"] and rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True
    report = json.load(open(os.path.join(run_dir, "report.json"), encoding="utf-8"))
    assert report["stop"]["stop_state"] == "reset_disable" and len(report["control"]["moves"]) == 1


def test_the_interrupt_guard_stops_once_and_defers_every_later_ctrl_c():
    """The handler only queues a constant (no count, no lock, no log: a nested SIGINT cannot
    lose the first); the relay thread counts, sends the stop requests first and then asks the
    loop to cancel the run; later Ctrl+C are deferred and never raise KeyboardInterrupt"""
    calls = []
    done = threading.Event()

    class Loop:
        def call_soon_threadsafe(self, fn):
            calls.append(("scheduled", threading.current_thread().name))
            fn()
            done.set()

    class Task:
        def cancel(self):
            calls.append("cancel")

    class Session:
        def latch_stops_first(self):
            calls.append(("latch", threading.current_thread().name))

        def emergency_stop_all(self, skip_already_stopped=False):
            calls.append(("stop", skip_already_stopped, threading.current_thread().name))
    g = run_flow.MG400InterruptGuard(Loop(), Task(), Session())
    g.handler(2, None)
    g.handler(2, None)                                        # "nested": both queued before the relay runs
    assert g.count == 0 and g._q.qsize() == 2                 # the handler counts nothing
    g._relay.start()
    assert done.wait(5)
    assert calls == [("latch", "mg400-sigint-relay"), ("scheduled", "mg400-sigint-relay"), "cancel",
                     ("stop", True, "mg400-sigint-relay")]
    for _ in range(3):
        g.handler(2, None)                                    # deferred: no exception
    g._q.put(None)
    g._relay.join(5)
    assert not g._relay.is_alive() and len(calls) == 4       # nothing new after the first
    assert g.count == 5


def test_a_sigint_reaches_the_mg400_while_the_event_loop_is_blocked(monkeypatch, example_config, tmp_path):
    """A real SIGINT while the loop is stuck in synchronous code (a slow capture, say): the stop
    latch of the MG400 worker is set by the relay thread during the block, not when the loop
    yields"""
    import signal as _signal
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    real = run_flow.execute_step
    real_factory = es.mock_robot_factory

    def slow_factory(rid, **kw):
        r = real_factory(rid, **kw)
        if hasattr(r, "fake"):
            r.fake.move_s = 3.0
        return r
    monkeypatch.setattr(es, "mock_robot_factory", slow_factory)
    seen = {}

    async def blocking(step, robots, shared, log):
        if step.get("action") == "move_z" and not seen:
            move = asyncio.ensure_future(real(step, robots, shared, log))
            await asyncio.sleep(0.4)
            ctl = robots[5]._ctl
            _signal.raise_signal(_signal.SIGINT)
            end = time.monotonic() + 2.0                       # the loop is blocked from here on
            while time.monotonic() < end and not ctl.latch.is_set():
                time.sleep(0.01)
            seen["latch_during_block"] = ctl.latch.is_set()
            return await move
        return await real(step, robots, shared, log)
    monkeypatch.setattr(run_flow, "execute_step", blocking)
    mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 130)
    assert seen == {"latch_during_block": True}
    run_dir = next(os.path.join(dp, d) for dp, dn, _ in os.walk(tmp_path / "logs") for d in dn if d.startswith("mg400_5"))
    meta = json.load(open(os.path.join(os.path.dirname(run_dir), "metadata.json"), encoding="utf-8"))
    rep = meta["robot_reports"]["5"]
    assert rep["stop_state"] == "reset_disable" and rep["worker_alive"] is False and "cleanup_incomplete" not in rep


def test_a_sigint_reaches_the_mg400_while_the_logging_handlers_are_held(monkeypatch, example_config, tmp_path):
    """The main thread holds every logging handler's lock (it was interrupted inside a log
    call) when the SIGINT comes: the relay and the wrapper set the latch before they log, so the
    worker's latch is set during the hold"""
    import logging as _logging
    import signal as _signal
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    real = run_flow.execute_step
    real_factory = es.mock_robot_factory

    def slow_factory(rid, **kw):
        r = real_factory(rid, **kw)
        if hasattr(r, "fake"):
            r.fake.move_s = 3.0
        return r
    monkeypatch.setattr(es, "mock_robot_factory", slow_factory)
    seen = {}

    async def holding(step, robots, shared, log):
        if step.get("action") == "move_z" and not seen:
            move = asyncio.ensure_future(real(step, robots, shared, log))
            await asyncio.sleep(0.4)
            ctl = robots[5]._ctl
            handlers = [h for lg in (_logging.getLogger(), _logging.getLogger("src")) for h in lg.handlers]
            for h in handlers:
                h.acquire()
            try:
                _signal.raise_signal(_signal.SIGINT)
                end = time.monotonic() + 2.0
                while time.monotonic() < end and not ctl.latch.is_set():
                    time.sleep(0.01)
                seen["latch_while_logging_is_held"] = ctl.latch.is_set()
            finally:
                for h in reversed(handlers):
                    h.release()
            return await move
        return await real(step, robots, shared, log)
    monkeypatch.setattr(run_flow, "execute_step", holding)
    mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 130)
    assert seen == {"latch_while_logging_is_held": True}


def test_with_logging_held_every_mg400_is_latched_and_the_cancel_is_scheduled(tmp_path):
    """Two MG400s and a Dobot, every logging handler held by the main thread: the relay latches
    both MG400s through latch_stop() (no logging) and schedules the cancel before it reaches any
    stop that logs or blocks"""
    import logging as _logging
    session = ExperimentSession(mock=True, robot_ports={}, workspace_validator=WorkspaceValidator(),
                                record_root=str(tmp_path))
    log = _logging.getLogger("src.test_mg400_relay")
    latched, cancelled = [], threading.Event()

    class Mg:
        STOP_NEVER_BLOCKS = True

        def __init__(self, name):
            self.name = name

        def latch_stop(self):
            latched.append(self.name)
            return True

        def emergency_stop(self):
            log.warning(f"{self.name} stop")                 # blocks while the handlers are held

    class Dobot:
        def emergency_stop(self):
            log.warning("dobot stop")
    session.robots = {1: Dobot(), 5: Mg("a"), 6: Mg("b")}

    class Loop:
        def call_soon_threadsafe(self, fn):
            fn()

    class Task:
        def cancel(self):
            cancelled.set()
    g = run_flow.MG400InterruptGuard(Loop(), Task(), session)
    handlers = [h for lg in (_logging.getLogger(), _logging.getLogger("src")) for h in lg.handlers]
    held = []
    for h in handlers:
        h.acquire()
        held.append(h)
    try:
        if not held:                                         # make sure a log call would block
            blocker = _logging.Handler()
            blocker.acquire()
            _logging.getLogger().addHandler(blocker)
            held.append(blocker)
        g._relay.start()
        g.handler(2, None)
        assert cancelled.wait(5)
        assert sorted(latched) == ["a", "b"]
    finally:
        for h in reversed(held):
            h.release()
        g._q.put(None)
        g._relay.join(5)
        for h in held:
            if h not in handlers:
                _logging.getLogger().removeHandler(h)
    assert session._stop_requested


def test_emergency_stop_all_sends_the_latch_only_stops_first(tmp_path):
    """A robot whose stop only sets a latch (the MG400) is stopped before robots whose stop may
    block (a Dobot's serial STOP), whatever their ids"""
    session = ExperimentSession(mock=True, robot_ports={}, workspace_validator=WorkspaceValidator(),
                                record_root=str(tmp_path))
    order = []

    class Blocking:
        def emergency_stop(self):
            order.append("dobot")

    class Latch:
        STOP_NEVER_BLOCKS = True

        def emergency_stop(self):
            order.append("mg400")
    session.robots = {1: Blocking(), 2: Blocking(), 5: Latch()}
    session.emergency_stop_all()
    assert order == ["mg400", "dobot", "dobot"]
    assert MG400Robot.STOP_NEVER_BLOCKS is True


def test_three_real_sigints_during_a_move_still_end_in_a_confirmed_stop_and_a_final_record(monkeypatch, example_config, tmp_path, caplog):
    """Three SIGINTs (not a substituted CancelledError) while a MovL is under way, through
    run_flow.main: the first starts the stop, the later ones are deferred, the record carries
    the final stop (reset_disable, worker ended, the end of the shutdown confirmed); SIGINT is
    restored afterwards"""
    import signal as _signal
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    real = run_flow.execute_step
    real_factory = es.mock_robot_factory

    def slow_factory(rid, **kw):
        r = real_factory(rid, **kw)
        if hasattr(r, "fake"):
            r.fake.move_s = 3.0
        return r
    monkeypatch.setattr(es, "mock_robot_factory", slow_factory)
    sent = []

    async def interrupting(step, robots, shared, log):
        if step.get("action") == "move_z" and not sent:
            move = asyncio.ensure_future(real(step, robots, shared, log))
            await asyncio.sleep(0.4)                           # the MovL was sent; the fake "moves"
            sent.append(1)
            for _ in range(3):
                _signal.raise_signal(_signal.SIGINT)
            return await move
        return await real(step, robots, shared, log)
    monkeypatch.setattr(run_flow, "execute_step", interrupting)
    before = _signal.getsignal(_signal.SIGINT)
    with caplog.at_level("WARNING"):
        mock_main([EXAMPLE_FLOW, "--mock"], str(tmp_path / "logs"), 130)
    assert _signal.getsignal(_signal.SIGINT) == before
    assert any("記録の確定を待っています" in r.getMessage() for r in caplog.records)   # the second was deferred
    run_dir = next(os.path.join(dp, d) for dp, dn, _ in os.walk(tmp_path / "logs") for d in dn if d.startswith("mg400_5"))
    meta = json.load(open(os.path.join(os.path.dirname(run_dir), "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "aborted"
    rep = meta["robot_reports"]["5"]
    assert rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True
    assert rep["worker_alive"] is False and "cleanup_incomplete" not in rep and rep["failed_reason"] is None
    report = json.load(open(os.path.join(run_dir, "report.json"), encoding="utf-8"))
    assert report["stop"]["stop_state"] == "reset_disable" and len(report["control"]["moves"]) == 1


def test_gui_runner_stop_during_the_normal_cleanup_records_the_final_stop(tmp_path):
    """One GUI Stop while the normal end (DisableRobot and close) is being waited for: the wait
    goes on, metadata.json carries the final report (not a snapshot with stop_state None)"""
    from src.gui import runner as gui_runner
    flow = {"name": "one hop", "steps": [{"action": "move_z", "robot_id": 5, "distance": 5.0}]}
    real = es.mock_robot_factory
    built = []

    def factory(rid, **kw):
        r = real(rid, **kw)
        built.append(r)
        return r
    events = []
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path / "logs"),
                                   robot_ports=ports_with_mg400(), robot_factory=factory, on_event=events.append)
    runner.start()
    deadline = time.time() + 60
    while time.time() < deadline and not any(r._shutdown_thread is not None for r in built):
        time.sleep(0.01)
    assert built and built[0]._shutdown_thread is not None, "the cleanup did not start"
    runner.request_stop()
    deadline = time.time() + 60
    while time.time() < deadline and runner.is_running():
        time.sleep(0.05)
    assert not runner.is_running()
    meta_path = next(os.path.join(dp, f) for dp, _, fs in os.walk(tmp_path / "logs") for f in fs if f == "metadata.json")
    meta = json.load(open(meta_path, encoding="utf-8"))
    rep = meta["robot_reports"]["5"]
    assert any(e.kind == "log" and "切断待ちのキャンセル" in e.message for e in list(events))  # the cancel hit the wait
    assert rep["worker_alive"] is False and "cleanup_incomplete" not in rep and rep["failed_reason"] is None
    assert rep["disabled_confirmed"] or (rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True)


def test_gui_runner_stop_with_an_mg400_runs_the_stop_sequence_and_reports_it(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE_FLOW, encoding="utf-8"))
    real = es.mock_robot_factory

    def factory(rid, **kw):
        r = real(rid, **kw)
        r.fake.move_s = 5.0                                   # time for the Stop during the first move
        return r
    events = []
    runner = gui_runner.FlowRunner(flow, mock=True, logs_dir=str(tmp_path / "logs"), source_path=EXAMPLE_FLOW,
                                   robot_ports=ports_with_mg400(), robot_factory=factory, on_event=events.append)
    runner.start()
    deadline = time.time() + 60
    while time.time() < deadline:
        if any(e.kind == "log" and "MovL" in e.message for e in list(events)):
            break
        time.sleep(0.05)
    else:
        runner.request_stop()
        pytest.fail("no MovL was started")
    time.sleep(0.3)
    runner.request_stop()
    assert runner.join(60), "the run did not stop"
    assert runner.status == "aborted"
    meta = json.load(open(os.path.join(runner.log_dir, "metadata.json"), encoding="utf-8"))
    rep = meta["robot_reports"]["5"]
    assert rep["stopped"] and rep["stop_state"] == "reset_disable" and rep["stop"]["disabled"] is True


def test_gui_runner_refuses_a_real_mg400_run_without_the_dialogs_confirmation(tmp_path):
    from src.gui import runner as gui_runner
    flow = json.load(open(EXAMPLE_FLOW, encoding="utf-8"))
    with pytest.raises(run_flow.PreflightError, match="confirmed_robots"):
        gui_runner.FlowRunner(flow, mock=False, robot_ports=ports_with_mg400(DOC_ADDRESS), logs_dir=str(tmp_path))
    runner = gui_runner.FlowRunner(flow, mock=False, robot_ports=ports_with_mg400(DOC_ADDRESS), logs_dir=str(tmp_path),
                                   confirmed_robots=[5])          # built with the dialog's evidence, never started
    assert runner.confirmed_robots == {5}
    assert not os.path.exists(tmp_path / "logs") and os.listdir(tmp_path) == []
    mock = gui_runner.FlowRunner(flow, mock=True, robot_ports=ports_with_mg400(), logs_dir=str(tmp_path))
    assert mock.confirmed_robots == frozenset()
