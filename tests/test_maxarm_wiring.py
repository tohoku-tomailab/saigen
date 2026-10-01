"""MaxArm (robot 4) wiring: config, schema, executor, preflight, session factories, GUI settings.

No hardware and no serial port: the real MaxArmRobot class is replaced by a
recorder wherever a "real" factory is exercised.
"""
import asyncio
import json
import os

import pytest
from pydantic import ValidationError

from src import config as lab_config
from src.devices.safety.validators import WorkspaceValidator, WorkspaceViolationError
from src.flow import executor, run_flow
from src.flow import experiment_session as es
from src.flow.experiment_session import ExperimentSession
from src.flow.schema import MAX_ROBOT_ID, ExperimentWorkflow

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(REPO_ROOT, "examples", "maxarm", "pick_and_place.json")

MAXARM_WS = {"x_min": -250.0, "x_max": 250.0, "y_min": -280.0, "y_max": -60.0,
             "z_min": 40.0, "z_max": 225.0, "r_min": 60.0, "r_max": 260.0}


def ports_with_maxarm(port=""):
    return {
        1: {"type": "dobot", "dobot_port": "COM3", "picus2_address": "COM4"},
        2: {"type": "dobot", "dobot_port": "COM5"},
        3: {"dobot_port": "COM7"},                       # no type -> dobot
        4: {"type": "maxarm", "maxarm_port": port, "workspace": dict(MAXARM_WS),
            "move_ms": 1500},
    }


@pytest.fixture
def restore_config():
    original = (lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH)
    yield
    lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH = original
    lab_config.load_config(force_reload=True)


# ----------------------------------------------------------------------
# config
# ----------------------------------------------------------------------
def test_defaults_have_a_maxarm_robot4():
    r4 = lab_config.DEFAULTS["robots"][4]
    assert r4["type"] == "maxarm" and r4["maxarm_port"] == ""
    assert r4["workspace"] == MAXARM_WS
    # Dobot robots keep no explicit type in DEFAULTS
    assert all("type" not in lab_config.DEFAULTS["robots"][rid] for rid in (1, 2, 3))


def test_get_robot_ports_fills_dobot_type_and_keeps_old_configs_working(tmp_path, restore_config):
    old = tmp_path / "config.yaml"   # a pre-MaxArm config.yaml: robots 1-3 only, no type
    old.write_text("robots:\n  1:\n    dobot_port: COM42\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(old)
    lab_config.load_config(force_reload=True)
    ports = lab_config.get_robot_ports()
    assert ports[1] == {"dobot_port": "COM42", "picus2_address": "COM4", "type": "dobot"}
    assert ports[4]["type"] == "maxarm"          # deep-merged from DEFAULTS
    assert lab_config.get_robot_type(1) == "dobot"
    assert lab_config.get_robot_type(4) == "maxarm"
    assert lab_config.get_robot_type(9) == "dobot"


def test_get_robot_workspace_merges_global_and_robot(tmp_path, restore_config):
    cfg = tmp_path / "config.yaml"
    cfg.write_text("workspace:\n  z_max: 110.0\nrobots:\n  4:\n    workspace:\n      z_max: 200.0\n",
                   encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    assert lab_config.get_robot_workspace(1) == lab_config.get_workspace()
    assert lab_config.get_robot_workspace(1)["z_max"] == 110.0
    ws4 = lab_config.get_robot_workspace(4)
    assert ws4["z_max"] == 200.0                 # robot's own value wins
    assert ws4["y_max"] == -60.0                 # the rest of the robot-4 block from DEFAULTS
    # a MaxArm's workspace is its own block only: nothing from the global Dobot box
    assert "joint1_min" not in ws4
    assert set(ws4) == set(MAXARM_WS)


def test_get_robot_ports_does_not_leak_mutations():
    ports = lab_config.get_robot_ports()
    ports[4]["workspace"]["z_max"] = 9999
    assert lab_config.get_robot_ports()[4]["workspace"]["z_max"] != 9999


def test_unknown_robot_type_is_a_config_error(tmp_path, restore_config):
    cfg = tmp_path / "config.yaml"
    cfg.write_text("robots:\n  4:\n    type: maxarn\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    with pytest.raises(lab_config.ConfigError):
        lab_config.get_robot_type(4)


# ----------------------------------------------------------------------
# schema
# ----------------------------------------------------------------------
def test_schema_suction_and_robot_id_range():
    assert MAX_ROBOT_ID == 4
    wf = ExperimentWorkflow(name="t", steps=[
        {"action": "suction", "robot_id": 4},
        {"action": "suction", "robot_id": 4, "on": False},
        {"action": "move_xyz", "robot_id": 4, "x": 0, "y": -200, "z": 150},
    ])
    assert wf.steps[0].on is True and wf.steps[1].on is False
    with pytest.raises(ValidationError):
        ExperimentWorkflow(name="t", steps=[{"action": "move_z", "robot_id": 5, "distance": 1}])
    with pytest.raises(ValidationError):
        ExperimentWorkflow(name="t", steps=[{"action": "suction", "robot_id": 4, "power": 1}])


def test_every_robot_id_field_accepts_4():
    schema = ExperimentWorkflow.model_json_schema()
    for name, model in schema["$defs"].items():
        rid = model.get("properties", {}).get("robot_id")
        if rid is not None:
            assert rid["maximum"] == MAX_ROBOT_ID, name


# ----------------------------------------------------------------------
# executor
# ----------------------------------------------------------------------
class FakeMaxArm:
    ROBOT_TYPE = "maxarm"
    SUPPORTED_ACTIONS = frozenset({"move_xyz", "move_z", "go_home", "suction", "wait"})

    def __init__(self):
        self.calls = []

    async def suction(self, on):
        self.calls.append(("suction", on))

    async def move_xyz(self, x, y, z):
        self.calls.append(("move_xyz", x, y, z))

    async def aspirate(self, volume, speed):   # must never be reached
        self.calls.append(("aspirate", volume))


def test_executor_routes_suction():
    robot = FakeMaxArm()
    asyncio.run(executor.execute_step({"action": "suction", "robot_id": 4, "on": False}, {4: robot}))
    asyncio.run(executor.execute_step({"action": "suction", "robot_id": 4}, {4: robot}))
    assert robot.calls == [("suction", False), ("suction", True)]


def test_executor_guard_rejects_unsupported_action():
    robot = FakeMaxArm()
    with pytest.raises(RuntimeError, match=r"Robot 4.*maxarm.*aspirate"):
        asyncio.run(executor.execute_step(
            {"action": "aspirate", "robot_id": 4, "volume": 1.0, "speed": 5}, {4: robot}))
    assert robot.calls == []


def test_executor_guard_ignores_robots_without_supported_actions():
    """LabRobot / MockLabRobot have no SUPPORTED_ACTIONS: no change for Dobots."""
    class Dobot:
        async def move_xyz(self, x, y, z):
            self.hit = True
    d = Dobot()
    asyncio.run(executor.execute_step({"action": "move_xyz", "x": 1, "y": 2, "z": 3}, {1: d}))
    assert d.hit


# ----------------------------------------------------------------------
# plan_resources / preflight
# ----------------------------------------------------------------------
def test_plan_resources_signature_unchanged():
    result = run_flow.plan_resources([
        {"action": "suction", "robot_id": 4}, {"action": "aspirate", "robot_id": 1},
        {"action": "measure_weight"},
    ])
    assert len(result) == 6
    robot_ids, picus2, scale, camera, micro, micro_serial = result
    assert robot_ids == [1, 4] and picus2 == {1} and scale and not camera


def test_preflight_capabilities():
    types = {1: "dobot", 4: "maxarm"}
    ok = [{"action": "suction", "robot_id": 4}, {"action": "move_z", "robot_id": 4, "distance": 1},
          {"action": "aspirate", "robot_id": 1}, {"action": "measure_weight"},
          {"action": "loop_start"}]
    assert run_flow.preflight_capabilities(ok, types) == []

    errors = run_flow.preflight_capabilities([
        {"action": "aspirate", "robot_id": 4},
        {"action": "aspirate", "robot_id": 4},
        {"action": "rotate_relative", "robot_id": 4, "angle": 10},
        {"action": "suction", "robot_id": 1},
        {"action": "suction", "robot_id": 2},      # not in robot_types -> dobot
    ], types)
    assert len(errors) == 4                        # the two aspirates are one message
    assert "Robot 4（maxarm）" in errors[0] and "'aspirate'" in errors[0] and "ピペット" in errors[0]
    assert "rotate_relative" in errors[1]
    assert "Robot 1（dobot）" in errors[2] and "'suction'" in errors[2]

    unknown = run_flow.preflight_capabilities([{"action": "move_z", "robot_id": 4}], {4: "ufo"})
    assert len(unknown) == 1 and "未知" in unknown[0]


def test_run_preflight_reports_capability_errors_like_violations():
    steps = [{"action": "aspirate", "robot_id": 4, "volume": 1.0, "speed": 5}]
    report = run_flow.run_preflight(steps, WorkspaceValidator(), robot_ports=ports_with_maxarm())
    assert not report.ok and report.capability_errors and not report.violations
    with pytest.raises(run_flow.PreflightError, match="対応していない"):
        run_flow.require_preflight(steps, WorkspaceValidator(), robot_ports=ports_with_maxarm())


def test_preflight_uses_robot4_workspace_and_global_for_dobots():
    shared = WorkspaceValidator()       # global Dobot box: z_max 150, y -300..300
    ports = ports_with_maxarm()
    # z 200 is inside the MaxArm box but above the global z_max
    steps = [{"action": "move_xyz", "robot_id": 4, "x": 0.0, "y": -200.0, "z": 200.0}]
    report = run_flow.run_preflight(steps, shared, robot_ports=ports)
    assert report.ok, report.violations
    assert report.robot_types == {4: "maxarm"}
    assert report.validators_by_robot[4] is not shared

    # the same target on a Dobot is checked against the global box
    report = run_flow.run_preflight([{**steps[0], "robot_id": 1}], shared, robot_ports=ports)
    assert not report.ok and report.validators_by_robot[1] is shared

    # y = 0 is fine for a Dobot but outside the MaxArm box (y_max -60)
    far = [{"action": "move_xyz", "robot_id": 4, "x": 100.0, "y": 0.0, "z": 100.0}]
    report = run_flow.run_preflight(far, shared, robot_ports=ports)
    assert not report.ok and len(report.violations) == 1
    assert isinstance(report.violations[0][2], WorkspaceViolationError)
    assert run_flow.run_preflight([{**far[0], "robot_id": 1}], shared, robot_ports=ports).ok


def test_preflight_workspace_accepts_validators_by_robot():
    tight = WorkspaceValidator(z_max=10)
    steps = [{"action": "move_xyz", "robot_id": 2, "x": 0, "y": 100, "z": 50},
             {"action": "move_xyz", "robot_id": 1, "x": 0, "y": 100, "z": 50}]
    report = run_flow.preflight_workspace(steps, WorkspaceValidator(), validators_by_robot={2: tight})
    assert [i for i, _, _ in report.violations] == [1]


def test_validator_for_robot():
    shared = WorkspaceValidator(z_max=100)
    ports = ports_with_maxarm()
    assert es.validator_for_robot(1, shared, ports) is shared
    v4 = es.validator_for_robot(4, shared, ports)
    limits = v4.get_limits()
    assert limits["z"] == {"min": 40.0, "max": 225.0}
    assert limits["y"] == {"min": -280.0, "max": -60.0}
    v4.validate_xyz(0.0, -200.0, 200.0)           # inside the MaxArm box (z above the shared 100)
    with pytest.raises(WorkspaceViolationError):
        v4.validate_xyz(0.0, -50.0, 100.0)        # y above y_max -60
    with pytest.raises(WorkspaceViolationError):
        v4.validate_xyz(250.0, -250.0, 100.0)     # radius 354 > r_max 260


def test_validate_only_exit_code_3_for_unsupported_action(tmp_path):
    flow = tmp_path / "bad.json"
    flow.write_text(json.dumps({"name": "t", "steps": [
        {"action": "aspirate", "robot_id": 4, "volume": 1.0, "speed": 5}]}), encoding="utf-8")
    assert run_flow.main([str(flow), "--validate-only"]) == run_flow.PREFLIGHT_EXIT_CODE


def test_validate_only_exit_code_3_for_suction_on_a_dobot(tmp_path):
    flow = tmp_path / "bad.json"
    flow.write_text(json.dumps({"name": "t", "steps": [
        {"action": "suction", "robot_id": 1}]}), encoding="utf-8")
    assert run_flow.main([str(flow), "--mock"]) == run_flow.PREFLIGHT_EXIT_CODE


def test_example_passes_validate_only():
    assert run_flow.main([EXAMPLE, "--validate-only"]) == 0
    _, steps = run_flow.load_and_validate(EXAMPLE)
    assert {s.get("robot_id") for s in steps} == {4}


# ----------------------------------------------------------------------
# resolve_ports / CLI
# ----------------------------------------------------------------------
def test_resolve_ports_maxarm_cli_and_env(monkeypatch):
    monkeypatch.delenv("ROBOT4_MAXARM_PORT", raising=False)
    parser = run_flow.build_parser()
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert ports[4]["type"] == "maxarm" and "workspace" in ports[4]
    assert ports[1]["type"] == "dobot"
    monkeypatch.setenv("ROBOT4_MAXARM_PORT", "COM_ENV_MAXARM")
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json"]))
    assert ports[4]["maxarm_port"] == "COM_ENV_MAXARM"
    ports, _ = run_flow.resolve_ports(parser.parse_args(["x.json", "--robot4-maxarm", "COM_CLI"]))
    assert ports[4]["maxarm_port"] == "COM_CLI"


# ----------------------------------------------------------------------
# session factories
# ----------------------------------------------------------------------
def test_mock_session_builds_mock_maxarm():
    from src.devices.safety.mock_maxarm import MockMaxArmRobot
    session = ExperimentSession(mock=True, robot_ports=ports_with_maxarm(),
                                workspace_validator=WorkspaceValidator(),
                                robot_validators={4: es.validator_for_robot(
                                    4, WorkspaceValidator(), ports_with_maxarm())})

    async def body():
        robot = await session.add_robot(4, use_picus2=True)   # ignored with a warning
        assert isinstance(robot, MockMaxArmRobot)
        assert robot.workspace_validator is session.robot_validators[4]
        assert robot.move_ms == 1500                              # option passed through
        await robot.suction(True)
        return robot

    asyncio.run(session.run(body))
    assert session.status == "completed"
    assert session.robot_types == {4: "maxarm"}


def test_real_session_refuses_maxarm_without_port(monkeypatch):
    built = []
    session = ExperimentSession(mock=False, robot_ports=ports_with_maxarm(port=""),
                                workspace_validator=WorkspaceValidator(),
                                robot_factory=lambda *a, **kw: built.append(a))
    with pytest.raises(RuntimeError, match=r"--robot4-maxarm.*ROBOT4_MAXARM_PORT"):
        asyncio.run(session.add_robot(4))
    assert built == []


def test_default_factory_dispatches_on_type(monkeypatch):
    """The real factory builds MaxArmRobot for type maxarm (class replaced: no port opened)."""
    import src.devices.safety.maxarm_robot as maxarm_robot

    class Recorder:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    monkeypatch.setattr(maxarm_robot, "MaxArmRobot", Recorder)
    v = WorkspaceValidator()
    robot = es.default_robot_factory(4, use_picus2=False, ports=ports_with_maxarm("COM_X")[4],
                                     workspace_validator=v)
    assert isinstance(robot, Recorder)
    assert robot.kwargs == {"maxarm_port": "COM_X", "workspace_validator": v, "move_ms": 1500}


def test_go_home_all_skips_maxarm():
    calls = []

    class Arm:
        def __init__(self, name, rtype=None):
            self.name = name
            if rtype:
                self.ROBOT_TYPE = rtype

        async def go_home(self):
            calls.append(self.name)

    session = ExperimentSession(mock=True, workspace_validator=WorkspaceValidator())
    session.robots = {1: Arm("dobot"), 4: Arm("maxarm", "maxarm")}
    asyncio.run(session.go_home_all())
    assert calls == ["dobot"]


def test_error_does_not_send_maxarm_home():
    session = ExperimentSession(mock=True, robot_ports=ports_with_maxarm(),
                                workspace_validator=WorkspaceValidator())

    async def body():
        robot = await session.add_robot(4)
        robot.go_home_calls = 0
        original = robot.go_home

        async def spy():
            robot.go_home_calls += 1
            await original()
        robot.go_home = spy
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError):
        asyncio.run(session.run(body))
    assert session.robots[4].go_home_calls == 0


def test_maxarm_mock_start_pose_ignores_dobot_env(monkeypatch):
    monkeypatch.setenv("MOCK_START_POSE", "200,0,50,0")
    assert es.maxarm_mock_start_pose(4, shared_config={}) is None
    cfg = {"mock_start_pose": {4: "10,-150,180", 1: "200,0,50,0"}}
    assert es.maxarm_mock_start_pose(4, shared_config=cfg) == (10.0, -150.0, 180.0)
    assert es.maxarm_mock_start_pose(4, shared_config={"mock_start_pose": "1,2,3,4"}) is None


# ----------------------------------------------------------------------
# logger / GUI settings
# ----------------------------------------------------------------------
def test_logger_records_robot_types(tmp_path):
    from src.flow.experiment_logger import ExperimentLogger
    src_json = tmp_path / "f.json"
    src_json.write_text("{}", encoding="utf-8")
    lg = ExperimentLogger("t", "", str(src_json), base_dir=str(tmp_path / "logs"), mode="mock")
    lg.record_resources([1, 4], set(), robot_types={4: "maxarm"})
    lg.finalize("completed")
    meta = json.load(open(lg.metadata_path, encoding="utf-8"))
    assert meta["robot_types"] == {"1": "dobot", "4": "maxarm"}
    assert "Robot 4 = maxarm" in open(lg.summary_path, encoding="utf-8").read()


def test_gui_settings_carry_type_and_maxarm_port():
    from src.gui import runner as gui_runner
    settings = gui_runner.default_device_settings()
    assert 4 in settings["robots"]
    r4 = settings["robots"][4]
    assert r4["type"] == "maxarm" and not r4["use_picus2"]
    r4["maxarm_port"] = "COM_GUI"
    robot_ports, _ = gui_runner.settings_to_session_args(settings)
    assert robot_ports[4]["type"] == "maxarm"
    assert robot_ports[4]["maxarm_port"] == "COM_GUI"
    assert robot_ports[4]["workspace"] == lab_config.get_robot_ports()[4]["workspace"]
    assert "dobot_port" not in robot_ports[4]
    assert robot_ports[1]["type"] == "dobot" and "dobot_port" in robot_ports[1]


def test_gui_runner_rejects_suction_on_dobot_before_anything_exists():
    from src.gui import runner as gui_runner
    with pytest.raises(run_flow.PreflightError):
        gui_runner.FlowRunner({"name": "t", "steps": [{"action": "suction", "robot_id": 1}]},
                              mock=True)


def test_gui_palette_has_suction_block():
    import ast
    src = open(os.path.join(REPO_ROOT, "src", "gui", "app.py"), encoding="utf-8").read()
    tree = ast.parse(src)
    values = {t.id: n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    config = ast.literal_eval(values["ACTION_CONFIG"])
    assert config["suction"]["defaults"] == {"on": True}
    categories = ast.literal_eval(values["CATEGORIES"])
    assert any("suction" in actions for _, actions in categories.values())


def test_llm_prompt_describes_robot4():
    from src.agent import llm_service
    assert "Robot 4 (robot_id: 4)" in llm_service.SYSTEM_PROMPT
    assert '"action": "suction"' in llm_service.SYSTEM_PROMPT


# ----------------------------------------------------------------------
# review regressions: S1 (per-robot workspace), S11 (cleanup on home failure),
# S12/P6 (configured home), S13 (GUI type from config)
# ----------------------------------------------------------------------
def _ports4(**extra4):
    """Dobot 1 plus a MaxArm at 4 (tests may override robot 4's keys)."""
    p4 = {"type": "maxarm", "maxarm_port": "", "workspace": dict(MAXARM_WS), "move_ms": 1500}
    p4.update(extra4)
    return {1: {"type": "dobot", "dobot_port": "COM3"}, 4: p4}


def test_s1_session_without_robot_validators_uses_maxarm_workspace():
    """validator_for falls back to the MaxArm's own box, never the shared Dobot one."""
    session = ExperimentSession(mock=True, robot_ports=_ports4(), shared_config={},
                                workspace_validator=WorkspaceValidator())

    async def body():
        robot = await session.add_robot(4)
        assert robot.workspace_validator is not session.workspace_validator
        assert robot.workspace_validator.get_limits()["z"] == {"min": 40.0, "max": 225.0}
        with pytest.raises(WorkspaceViolationError):
            await robot.move_xyz(0.0, -150.0, 0.0)        # z 0 < MaxArm z_min 40
        return robot

    asyncio.run(session.run(body))


def test_s1_maxarm_validator_is_not_merged_over_dobot_box():
    # the shared box is much tighter in z; the MaxArm box must not inherit it
    v4 = es.validator_for_robot(4, WorkspaceValidator(z_min=-10, z_max=20), _ports4())
    limits = v4.get_limits()
    assert limits["z"] == {"min": 40.0, "max": 225.0}
    assert limits["x"] == {"min": -250.0, "max": 250.0}
    # radius keys are optional
    ws = {k: v for k, v in MAXARM_WS.items() if not k.startswith("r_")}
    es.validator_for_robot(4, WorkspaceValidator(), _ports4(workspace=ws))


@pytest.mark.parametrize("robot_id", [1, 2, 3, 4])
def test_s1_maxarm_at_any_id_without_workspace_is_a_config_error(robot_id):
    ports = {robot_id: {"type": "maxarm", "maxarm_port": "COMX"}}
    with pytest.raises(lab_config.ConfigError, match=rf"robots\.{robot_id}\.workspace"):
        es.validator_for_robot(robot_id, WorkspaceValidator(), ports)


@pytest.mark.parametrize("missing", ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"])
def test_s1_incomplete_maxarm_workspace_is_a_config_error(missing):
    ws = {k: v for k, v in MAXARM_WS.items() if k != missing}
    with pytest.raises(lab_config.ConfigError, match=missing):
        es.validator_for_robot(4, WorkspaceValidator(), _ports4(workspace=ws))


@pytest.mark.parametrize("key", ["z_min", "x_max", "r_min", "r_max"])
def test_s1_null_value_in_maxarm_workspace_is_a_config_error(key):
    ws = dict(MAXARM_WS, **{key: None})        # YAML "z_min:" with no value
    with pytest.raises(lab_config.ConfigError, match=key):
        es.validator_for_robot(4, WorkspaceValidator(), _ports4(workspace=ws))


def test_s1_invalid_dobot_override_is_a_config_error_naming_the_robot():
    ports = {2: {"type": "dobot", "dobot_port": "COM5", "workspace": {"z_min": 50, "z_max": 10}}}
    with pytest.raises(lab_config.ConfigError, match=r"robots\.2\.workspace"):
        es.validator_for_robot(2, WorkspaceValidator(), ports)


def test_s1_session_refuses_maxarm_without_workspace_before_opening_anything():
    built = []
    session = ExperimentSession(mock=False, workspace_validator=WorkspaceValidator(),
                                robot_ports={4: {"type": "maxarm", "maxarm_port": "COMX"}},
                                robot_factory=lambda *a, **kw: built.append(a))
    with pytest.raises(lab_config.ConfigError, match=r"robots\.4\.workspace"):
        asyncio.run(session.add_robot(4))
    assert built == []


MAXARM_NO_WS = {1: {"type": "maxarm", "maxarm_port": "COMX"}}
MAXARM_STEPS = [{"action": "move_xyz", "robot_id": 1, "x": 0.0, "y": -200.0, "z": 150.0}]


def test_s1_preflight_refuses_maxarm_without_workspace():
    """The runners' entry points (CLI, CSV, GUI) raise before any device opens."""
    with pytest.raises(lab_config.ConfigError, match=r"robots\.1\.workspace"):
        run_flow.run_preflight(MAXARM_STEPS, WorkspaceValidator(), robot_ports=MAXARM_NO_WS)
    with pytest.raises(lab_config.ConfigError, match=r"robots\.1\.workspace"):
        run_flow.require_preflight(MAXARM_STEPS, WorkspaceValidator(), robot_ports=MAXARM_NO_WS)


def test_s1_gui_runner_refuses_maxarm_without_workspace():
    from src.gui import runner as gui_runner
    with pytest.raises(lab_config.ConfigError):
        gui_runner.FlowRunner({"name": "t", "steps": MAXARM_STEPS}, mock=True,
                              robot_ports=MAXARM_NO_WS,
                              robot_factory=lambda *a, **kw: pytest.fail("device built"))


@pytest.mark.parametrize("mode", ["--validate-only", "--mock"])
def test_s1_cli_refuses_maxarm_without_workspace(tmp_path, restore_config, monkeypatch, mode):
    cfg = tmp_path / "config.yaml"
    cfg.write_text('robots:\n  1:\n    type: maxarm\n    maxarm_port: "COMX"\n', encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    assert "workspace" not in lab_config.get_robot_ports()[1]
    with pytest.raises(lab_config.ConfigError):
        lab_config.get_robot_workspace(1)
    monkeypatch.setattr(run_flow, "ExperimentSession",
                        lambda *a, **kw: pytest.fail("session created"))
    flow = tmp_path / "f.json"
    flow.write_text(json.dumps({"name": "t", "steps": MAXARM_STEPS}), encoding="utf-8")
    assert run_flow.main([str(flow), mode]) == 2


def _real_maxarm_session(client, **extra4):
    from src.devices.safety.maxarm_robot import MaxArmRobot

    def factory(rid, *, use_picus2, ports, workspace_validator):
        return MaxArmRobot("FAKE", workspace_validator=workspace_validator,
                           client_factory=lambda p: client, pose_settle_interval=0,
                           wait_after_move=0, **es.robot_options(ports))
    return ExperimentSession(mock=False, robot_factory=factory,
                             robot_ports=_ports4(maxarm_port="COMX", **extra4),
                             shared_config={}, workspace_validator=WorkspaceValidator())


def test_s11_home_setup_failure_disconnects_the_robot():
    from tests.test_maxarm_robot import FakeMaxArmClient
    client = FakeMaxArmClient(moving=True)      # arm still moving: home cannot be set
    session = _real_maxarm_session(client)

    async def body():
        await session.add_robot(4)
    with pytest.raises(RuntimeError):
        asyncio.run(session.run(body))
    assert ("close",) in client.calls, "connection (and heartbeat thread) leaked"
    assert 4 not in session.robots


class _HomeFakeRobot:
    """Minimal robot for add_robot: records cleanup and the home calls."""
    ROBOT_TYPE = "maxarm"

    def __init__(self, home_exc=None, home_position=None, within=True):
        self.home_exc = home_exc
        self.home_position = home_position
        self.within = within
        self.calls = []

    async def initialize(self):
        return True

    def set_current_position_as_home(self):
        self.calls.append("set_home")
        if self.home_exc is not None:
            raise self.home_exc
        self.home_position = [0.0, -150.0, 150.0]

    def home_is_within_limits(self):
        self.calls.append("check_home")
        return self.within

    async def cleanup(self):
        self.calls.append("cleanup")
        raise OSError("close failed")           # logged, never masks the original error


@pytest.mark.parametrize("exc", [RuntimeError("moving"), asyncio.CancelledError(),
                                 KeyboardInterrupt()])
def test_s11_cleanup_on_home_failure_including_cancellation(exc):
    robot = _HomeFakeRobot(home_exc=exc)
    session = ExperimentSession(mock=True, robot_ports=_ports4(), shared_config={},
                                workspace_validator=WorkspaceValidator(),
                                robot_factory=lambda *a, **kw: robot)
    with pytest.raises(type(exc)):
        asyncio.run(session.add_robot(4))
    assert robot.calls == ["set_home", "cleanup"]
    assert session.robots == {}


def test_s12_configured_home_is_preserved():
    session = ExperimentSession(mock=True, robot_ports=_ports4(home_position=[0, -200, 150]),
                                shared_config={}, workspace_validator=WorkspaceValidator())

    async def body():
        robot = await session.add_robot(4)
        return list(robot.home_position)
    home = asyncio.run(session.run(body))
    assert home == [0, -200, 150], f"configured home overwritten by start pose: {home}"


def test_s12_configured_home_outside_workspace_refuses_and_disconnects():
    robot = _HomeFakeRobot(home_position=[0.0, -200.0, 500.0], within=False)
    session = ExperimentSession(mock=True, robot_ports=_ports4(home_position=[0, -200, 500]),
                                shared_config={}, workspace_validator=WorkspaceValidator(),
                                robot_factory=lambda *a, **kw: robot)
    with pytest.raises(RuntimeError, match="home_position"):
        asyncio.run(session.add_robot(4))
    assert robot.calls == ["check_home", "cleanup"]      # never replaced by the start pose
    assert session.robots == {}


def test_s12_configured_home_outside_workspace_mock_maxarm():
    session = ExperimentSession(mock=True, robot_ports=_ports4(home_position=[0, -200, 500]),
                                shared_config={}, workspace_validator=WorkspaceValidator())
    with pytest.raises(RuntimeError, match="home_position"):
        asyncio.run(session.add_robot(4))
    assert session.robots == {}


def test_s12_real_maxarm_keeps_configured_home():
    from tests.test_maxarm_robot import FakeMaxArmClient
    client = FakeMaxArmClient()
    session = _real_maxarm_session(client, home_position=[0.0, -200.0, 150.0])

    async def body():
        robot = await session.add_robot(4)
        return list(robot.home_position)
    assert asyncio.run(session.run(body)) == [0.0, -200.0, 150.0]


def test_s12_dobot_home_behaviour_unchanged():
    """A Dobot still takes its start pose as home, even with home_position in config."""
    class Dobot:
        def __init__(self):
            self.calls = []

        async def initialize(self):
            return True

        def set_current_position_as_home(self):
            self.calls.append("set_home")

        async def cleanup(self):
            self.calls.append("cleanup")
    robot = Dobot()
    ports = {1: {"type": "dobot", "dobot_port": "COM3", "home_position": [1, 2, 3, 4]}}
    session = ExperimentSession(mock=True, robot_ports=ports, shared_config={},
                                workspace_validator=WorkspaceValidator(),
                                robot_factory=lambda *a, **kw: robot)
    asyncio.run(session.add_robot(1))
    assert robot.calls == ["set_home"] and session.robots[1] is robot
    assert session.validator_for(1) is session.workspace_validator


def test_s13_gui_type_comes_from_config(monkeypatch):
    from src.gui import runner as gui_runner
    monkeypatch.setattr(gui_runner.lab_config, "get_robot_ports",
                        lambda: {4: {"type": "dobot", "dobot_port": "COM9"}})
    settings = {"robots": {4: {"type": "maxarm", "maxarm_port": "COM11"}}, "shared": {}}
    robot_ports, _ = gui_runner.settings_to_session_args(settings)
    assert robot_ports[4]["type"] == "dobot", robot_ports[4]
    assert "maxarm_port" not in robot_ports[4]
    assert robot_ports[4]["dobot_port"] == "COM9"       # stale sidebar lacks it: config value

    # and the other way round: a stale "dobot" sidebar entry for a configured MaxArm
    monkeypatch.setattr(gui_runner.lab_config, "get_robot_ports",
                        lambda: {4: {"type": "maxarm", "maxarm_port": "COM12",
                                     "workspace": dict(MAXARM_WS)}})
    settings = {"robots": {4: {"type": "dobot", "dobot_port": "COM3"}}, "shared": {}}
    robot_ports, _ = gui_runner.settings_to_session_args(settings)
    assert robot_ports[4]["type"] == "maxarm"
    assert robot_ports[4]["maxarm_port"] == "COM12"
    assert robot_ports[4]["workspace"] == MAXARM_WS



# ======================================================================
# Round 2 (maxarm_fix_plan2.md, Agent S2)
# ======================================================================
_R2_STEPS = [{"action": "move_xyz", "robot_id": 1, "x": 200, "y": 0, "z": 50},
             # inside the Dobot box but outside the MaxArm box (y_max = -60)
             {"action": "move_xyz", "robot_id": 4, "x": 100, "y": 50, "z": 100}]


def test_r2_1_incomplete_validator_map_is_completed_from_robot_ports():
    shared = WorkspaceValidator()
    ports = ports_with_maxarm("COMX")
    assert not run_flow.run_preflight(_R2_STEPS, shared, robot_ports=ports).ok
    rep = run_flow.run_preflight(_R2_STEPS, shared, robot_ports=ports,
                                 validators_by_robot={1: shared})
    assert not rep.ok, "robot 4 was checked against the shared Dobot box"
    assert rep.validators_by_robot[1] is shared
    v4 = rep.validators_by_robot[4]
    assert v4 is not shared and v4.get_limits()["y"]["max"] == -60.0
    assert len(rep.violations) == 1                     # only the robot-4 step
    sess = ExperimentSession(mock=True, robot_ports=ports, workspace_validator=shared,
                             robot_validators=rep.validators_by_robot)
    assert sess.validator_for(4) is v4 and sess.validator_for(4) is not shared


def test_r2_1_none_entry_is_filled_and_a_valid_maxarm_step_passes():
    shared = WorkspaceValidator()
    ports = ports_with_maxarm("COMX")
    steps = [{"action": "move_xyz", "robot_id": 4, "x": 0, "y": -150, "z": 150}]
    rep = run_flow.run_preflight(steps, shared, robot_ports=ports,
                                 validators_by_robot={4: None})
    assert rep.ok and rep.validators_by_robot[4] is not shared


def test_r2_1_supplied_shared_validator_for_a_maxarm_is_a_config_error():
    shared = WorkspaceValidator()
    ports = ports_with_maxarm("COMX")
    with pytest.raises(lab_config.ConfigError, match="Robot 4"):
        run_flow.run_preflight(_R2_STEPS, shared, robot_ports=ports,
                               validators_by_robot={1: shared, 4: shared})


def test_r2_1_missing_maxarm_entry_loads_robot_ports_when_only_types_are_given(monkeypatch):
    shared = WorkspaceValidator()
    monkeypatch.setattr(lab_config, "get_robot_ports", lambda: ports_with_maxarm("COMX"))
    rep = run_flow.run_preflight(_R2_STEPS, shared, validators_by_robot={1: shared},
                                 robot_types={1: "dobot", 4: "maxarm"})
    assert not rep.ok and rep.validators_by_robot[4] is not shared


def test_r2_1_maxarm_without_workspace_in_an_incomplete_map_is_a_config_error():
    shared = WorkspaceValidator()
    ports = ports_with_maxarm("COMX")
    del ports[4]["workspace"]
    with pytest.raises(lab_config.ConfigError):
        run_flow.run_preflight(_R2_STEPS, shared, robot_ports=ports,
                               validators_by_robot={1: shared})


def test_r2_8_stale_dobot_entry_does_not_override_configured_maxarm_port(monkeypatch):
    from src.gui import runner as gui_runner
    monkeypatch.setattr(lab_config, "get_robot_ports", lambda: {
        4: {"type": "maxarm", "maxarm_port": "COM11", "workspace": dict(MAXARM_WS)}})
    stale = {"robots": {4: {"type": "dobot", "dobot_port": "COM9", "maxarm_port": "",
                            "use_picus2": False, "picus2_port": ""}}}
    ports, _ = gui_runner.settings_to_session_args(stale)
    assert ports[4]["type"] == "maxarm"
    assert ports[4]["maxarm_port"] == "COM11"            # config wins over the stale ""
    assert "dobot_port" not in ports[4]


def test_r2_8_stale_maxarm_entry_does_not_override_configured_dobot_ports(monkeypatch):
    from src.gui import runner as gui_runner
    monkeypatch.setattr(lab_config, "get_robot_ports", lambda: {
        2: {"type": "dobot", "dobot_port": "COM5", "picus2_address": "COM6"}})
    stale = {"robots": {2: {"type": "maxarm", "maxarm_port": "COM11", "dobot_port": "COM99",
                            "use_picus2": False, "picus2_port": "COM98"}}}
    ports, _ = gui_runner.settings_to_session_args(stale)
    assert ports[2] == {"type": "dobot", "dobot_port": "COM5", "picus2_address": "COM6"}


def test_r2_8_matching_type_keeps_the_sidebar_override(monkeypatch):
    from src.gui import runner as gui_runner
    monkeypatch.setattr(lab_config, "get_robot_ports", lambda: {
        4: {"type": "maxarm", "maxarm_port": "COM11", "workspace": dict(MAXARM_WS)}})
    ports, _ = gui_runner.settings_to_session_args(
        {"robots": {4: {"type": "maxarm", "maxarm_port": "COM_EDIT"}}})
    assert ports[4]["maxarm_port"] == "COM_EDIT"
