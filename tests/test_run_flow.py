"""run_flow: ポート優先順位・Mock 実行の成果物・可動域違反での中止"""
import asyncio
import json
import os

import pytest

from src import config as lab_config
from src.devices.safety.mock_robot import MockLabRobot
from src.devices.safety.validators import (
    WorkspaceViolationError,
    default_workspace_validator,
)
from src.flow import run_flow


# ----------------------------------------------------------------------
# 接続ポートの優先順位: 引数 > 環境変数 > config.yaml
# ----------------------------------------------------------------------
def parse(argv):
    return run_flow.build_parser().parse_args(argv)


def test_ports_fall_back_to_config(monkeypatch):
    for name in ("ROBOT1_DOBOT_PORT", "ROBOT1_PICUS2_PORT", "SCALE_PORT", "CAMERA_INDEX"):
        monkeypatch.delenv(name, raising=False)
    ports, shared = run_flow.resolve_ports(parse(["flow.json"]))
    assert ports[1]["dobot_port"] == lab_config.get_robot_ports()[1]["dobot_port"]
    assert shared["scale_port"] == lab_config.get_shared_devices()["scale_port"]


def test_env_overrides_config(monkeypatch):
    monkeypatch.setenv("ROBOT1_DOBOT_PORT", "COM_ENV")
    monkeypatch.setenv("SCALE_PORT", "COM_ENV_SCALE")
    ports, shared = run_flow.resolve_ports(parse(["flow.json"]))
    assert ports[1]["dobot_port"] == "COM_ENV"
    assert shared["scale_port"] == "COM_ENV_SCALE"


def test_cli_overrides_env(monkeypatch):
    monkeypatch.setenv("ROBOT1_DOBOT_PORT", "COM_ENV")
    monkeypatch.setenv("SCALE_PORT", "COM_ENV_SCALE")
    monkeypatch.setenv("CAMERA_INDEX", "7")
    args = parse(["flow.json", "--robot1-dobot", "COM_CLI",
                  "--scale-port", "COM_CLI_SCALE", "--camera-index", "3"])
    ports, shared = run_flow.resolve_ports(args)
    assert ports[1]["dobot_port"] == "COM_CLI"
    assert shared["scale_port"] == "COM_CLI_SCALE"
    assert shared["camera_index"] == 3


def test_record_defaults_on_for_real_and_off_for_mock():
    assert parse(["flow.json"]).record is None            # 未指定 → 実機では ON
    assert parse(["flow.json", "--no-record"]).record is False
    assert parse(["flow.json", "--record"]).record is True


# ----------------------------------------------------------------------
# Mock 実行: 成果物一式が作られる
# ----------------------------------------------------------------------
EXAMPLE_FLOW = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "examples", "zif8", "zif8_two_solution_mixing_speed5.json",
)


def _latest_run_dir(logs_dir):
    runs = [os.path.join(r, d) for r, ds, _ in os.walk(logs_dir) for d in ds
            if os.path.exists(os.path.join(r, d, "metadata.json"))]
    assert runs, f"実行ログフォルダが作られていません: {logs_dir}"
    return max(runs, key=os.path.getmtime)


def run_mock(flow_path, tmp_path, monkeypatch, extra=()):
    """Mock モードで run_flow.main を実行し、(exit_code, ログフォルダ) を返す"""
    monkeypatch.setattr(run_flow, "LOGS_DIR", str(tmp_path / "logs"))
    code = run_flow.main([str(flow_path), "--mock", *extra])
    return code, _latest_run_dir(str(tmp_path / "logs"))


def test_validate_only_accepts_every_published_example():
    example_dir = os.path.dirname(EXAMPLE_FLOW)
    flows = sorted(f for f in os.listdir(example_dir) if f.endswith(".json"))
    assert flows, "examples/zif8 に JSON フローがありません"
    for name in flows:
        assert run_flow.main([os.path.join(example_dir, name), "--validate-only"]) == 0


def test_mock_run_writes_all_artifacts(tmp_path, monkeypatch):
    flow = tmp_path / "short_flow.json"
    flow.write_text(json.dumps({
        "name": "mock artifacts",
        "description": "aspirate, tare, dispense, weigh and photograph",
        "steps": [
            # the mock tracks the held volume like the real wrapper: aspirate first
            {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 5},
            {"action": "tare_scale", "delay": 0.1},
            {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 5},
            {"action": "measure_weight", "stabilization_count": 3},
            {"action": "capture_and_save", "file_path": ""},
        ],
    }), encoding="utf-8")

    code, run_dir = run_mock(flow, tmp_path, monkeypatch)
    assert code == 0
    for name in ("run.log", "measurements.csv", "summary.md", "metadata.json"):
        assert os.path.exists(os.path.join(run_dir, name)), name
    assert os.path.exists(os.path.join(run_dir, os.path.basename(flow)))
    assert os.path.isdir(os.path.join(run_dir, "images"))

    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "completed"
    assert meta["weight_measurements"] >= 1

    csv_text = open(os.path.join(run_dir, "measurements.csv"), encoding="utf-8-sig").read()
    # Mock の共有デバイスは float を返すので重量列が埋まる
    assert "4.980" in csv_text
    # capture_and_save の保存先は実験フォルダ内の images/ に束ねられる
    assert os.path.join(run_dir, "images") in csv_text


def test_mock_run_of_maxarm_example_records_robot_types(tmp_path, monkeypatch):
    """examples/maxarm/pick_and_place.json が Mock で完走し、ロボット種別が残る"""
    monkeypatch.delenv("MOCK_START_POSE", raising=False)
    flow = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "examples", "maxarm", "pick_and_place.json")
    code, run_dir = run_mock(flow, tmp_path, monkeypatch)
    assert code == 0
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "completed"
    assert meta["robots_used"] == [4]
    assert meta["robot_types"] == {"4": "maxarm"}
    assert meta["picus2_robots"] == []


def test_mock_run_of_dobot_flow_records_dobot_type(tmp_path, monkeypatch):
    flow = tmp_path / "dobot.json"
    flow.write_text(json.dumps({"name": "d", "steps": [
        {"action": "wait", "robot_id": 1, "seconds": 0}]}), encoding="utf-8")
    code, run_dir = run_mock(flow, tmp_path, monkeypatch)
    assert code == 0
    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["robot_types"] == {"1": "dobot"}


# ----------------------------------------------------------------------
# 可動域違反: Mock でも実機と同じ WorkspaceValidator で弾かれる
# ----------------------------------------------------------------------
def test_mock_robot_validates_through_workspace_validator():
    robot = MockLabRobot(use_dobot=True, workspace_validator=default_workspace_validator())
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.move_z(-500))


def test_mock_robot_validates_joint1_rotation():
    robot = MockLabRobot(use_dobot=True, workspace_validator=default_workspace_validator())
    with pytest.raises(WorkspaceViolationError):
        asyncio.run(robot.rotate_relative(200.0))


def test_workspace_violation_aborts_before_later_steps(tmp_path, monkeypatch):
    """違反ステップで実行を止め、metadata.json に status=failed を残す"""
    flow = tmp_path / "bad_flow.json"
    flow.write_text(json.dumps({
        "name": "workspace violation",
        "description": "move_z beyond the configured Z limit",
        "steps": [
            {"action": "move_z", "robot_id": 1, "distance": -500.0},
            {"action": "go_home", "robot_id": 1},
        ],
    }), encoding="utf-8")

    code, run_dir = run_mock(flow, tmp_path, monkeypatch)
    assert code != 0

    meta = json.load(open(os.path.join(run_dir, "metadata.json"), encoding="utf-8"))
    assert meta["status"] == "failed"
    assert "可動域" in (meta["error"] or "")
    # 違反したステップだけが記録され、後続の go_home は実行されていない
    assert meta["total_steps"] == 1
    assert meta["error_steps"] == 1
