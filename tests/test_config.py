"""src.config: config.yaml / config.example.yaml の読み込みとフォールバック"""
import os

import pytest

from src import config as lab_config


@pytest.fixture
def restore_config_path():
    """テスト内で CONFIG_PATH を差し替えても元に戻す"""
    original = (lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH)
    yield
    lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH = original
    lab_config.load_config(force_reload=True)


def test_robot_ports_have_int_keys():
    ports = lab_config.get_robot_ports()
    assert all(isinstance(k, int) for k in ports)
    # 1-3: Dobot Magician, 4: optional Hiwonder MaxArm (placeholder port)
    assert set(ports.keys()) == {1, 2, 3, 4}
    assert all(ports[rid]["type"] == "dobot" for rid in (1, 2, 3))
    assert ports[4]["type"] == "maxarm"


def test_example_yaml_robot4_is_a_placeholder_maxarm(tmp_path, restore_config_path):
    """config.example.yaml の Robot 4 はポート未設定の MaxArm（実機ランは起動を拒否する）"""
    lab_config.CONFIG_PATH = str(tmp_path / "absent-config.yaml")
    r4 = lab_config.load_config(force_reload=True)["robots"][4]
    assert r4["type"] == "maxarm"
    assert r4["maxarm_port"] == ""
    ws = r4["workspace"]
    assert ws["z_min"] > 0 and ws["y_max"] < 0 and ws["r_min"] < ws["r_max"]


def test_robot3_has_no_pipette():
    ports = lab_config.get_robot_ports()
    assert "picus2_address" not in ports[3]


def test_workspace_limits_are_published_values():
    """公開しているワークスペース既定値（examples/ のフローが収まる範囲）"""
    ws = lab_config.get_workspace()
    assert ws["z_min"] == -130.0
    assert ws["z_max"] == 150.0
    assert ws["joint1_min"] == -135.0
    assert ws["joint1_max"] == 135.0


def test_example_yaml_and_defaults_are_identical(tmp_path, restore_config_path):
    """config.example.yaml と DEFAULTS のドリフト検出

    ラボ固有の config.yaml が置かれた PC でも壊れないよう、CONFIG_PATH を
    存在しないパスに向けて config.example.yaml だけが読まれるようにする。
    """
    lab_config.CONFIG_PATH = str(tmp_path / "absent-config.yaml")
    assert lab_config.load_config(force_reload=True) == lab_config.DEFAULTS


def test_partial_yaml_merges_over_defaults(tmp_path, restore_config_path):
    partial = tmp_path / "config.yaml"
    partial.write_text(
        "workspace:\n  z_min: -99.0\nrobots:\n  1:\n    dobot_port: COM99\n",
        encoding="utf-8",
    )
    lab_config.CONFIG_PATH = str(partial)
    cfg = lab_config.load_config(force_reload=True)
    # 上書きが効く
    assert cfg["workspace"]["z_min"] == -99.0
    assert cfg["robots"][1]["dobot_port"] == "COM99"
    # 欠けたキーは DEFAULTS で補完
    assert cfg["workspace"]["z_max"] == 150.0
    assert cfg["robots"][1]["picus2_address"] == "COM4"


def test_config_yaml_wins_over_example(tmp_path, restore_config_path):
    """config.yaml があれば config.example.yaml は読まれない"""
    own = tmp_path / "config.yaml"
    own.write_text("shared_devices:\n  scale_port: COM42\n", encoding="utf-8")
    example = tmp_path / "config.example.yaml"
    example.write_text("shared_devices:\n  scale_port: COM99\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(own)
    lab_config.EXAMPLE_CONFIG_PATH = str(example)
    assert lab_config.load_config(force_reload=True)["shared_devices"]["scale_port"] == "COM42"


def test_example_used_when_config_absent(tmp_path, restore_config_path):
    example = tmp_path / "config.example.yaml"
    example.write_text("shared_devices:\n  scale_port: COM99\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(tmp_path / "config.yaml")  # 存在しない
    lab_config.EXAMPLE_CONFIG_PATH = str(example)
    assert lab_config.load_config(force_reload=True)["shared_devices"]["scale_port"] == "COM99"


def test_missing_files_fall_back_to_defaults(restore_config_path):
    lab_config.CONFIG_PATH = "/nonexistent/config.yaml"
    lab_config.EXAMPLE_CONFIG_PATH = "/nonexistent/config.example.yaml"
    assert lab_config.load_config(force_reload=True) == lab_config.DEFAULTS


def test_broken_yaml_raises_instead_of_falling_back(tmp_path, restore_config_path):
    """壊れた config.yaml は DEFAULTS（±300 mm の可動域）に黙って戻さず例外にする"""
    broken = tmp_path / "config.yaml"
    broken.write_text("- broken\n- list\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(broken)
    lab_config.EXAMPLE_CONFIG_PATH = "/nonexistent/config.example.yaml"
    with pytest.raises(lab_config.ConfigError, match="mapping"):
        lab_config.load_config(force_reload=True)


def test_dashboard_ports_come_from_monitoring_config():
    """ダッシュボードのポートは src/monitoring 側の設定が単一ソース"""
    ports = lab_config.get_dashboard_ports()
    assert set(ports) == {"web_port", "tcp_port"}
    assert all(isinstance(v, int) for v in ports.values())
    assert os.path.exists(
        os.path.join(lab_config.MONITORING_DIR, "config.example.yaml")
    )


def test_video_camera_index_is_int():
    assert isinstance(lab_config.get_video_camera_index(), int)
