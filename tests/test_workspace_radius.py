"""WorkspaceValidator r_min / r_max (horizontal radius hypot(x, y)) and
workspace_validator_from_config. Tighten-only: without r_min / r_max nothing changes."""
import logging
import math

import pytest

from src.devices.safety.validators import (
    WorkspaceValidator,
    WorkspaceViolationError,
    workspace_validator_from_config,
)
from src.devices.safety.validators.workspace import NonFiniteTargetError


def _v(**kw):
    return WorkspaceValidator(x_min=-250, x_max=250, y_min=-280, y_max=280,
                              z_min=40, z_max=225, **kw)


def test_defaults_have_no_radius_limits_and_unchanged_limits():
    v = WorkspaceValidator()
    assert v.r_min is None and v.r_max is None
    assert set(v.get_limits()) == {"x", "y", "z", "joint1"}
    v.validate_xyz(0.0, 0.0, 0.0)           # the base axis is fine without r_min
    assert "R=" not in repr(v)


def test_r_min_rejects_points_near_the_base_axis():
    v = _v(r_min=60)
    v.validate_xyz(0.0, -60.0, 100.0)       # exactly on the bound
    with pytest.raises(WorkspaceViolationError) as ei:
        v.validate_xyz(30.0, -40.0, 100.0)  # r = 50
    err = ei.value
    assert err.target_position["r"] == pytest.approx(50.0)
    assert [d.axis for d in err.violations] == ["R"]
    assert err.violations[0].min_value == 60
    assert "R" in str(err)


def test_r_max_rejects_points_beyond_reach():
    v = _v(r_max=260)
    v.validate_xyz(0.0, -260.0, 100.0)
    with pytest.raises(WorkspaceViolationError) as ei:
        v.validate_xyz(200.0, -200.0, 100.0)    # inside the box, r = 282.8
    assert ei.value.violations[0].axis == "R"
    assert ei.value.violations[0].max_value == 260


def test_radius_and_box_violations_are_reported_together():
    v = _v(r_max=100)
    with pytest.raises(WorkspaceViolationError) as ei:
        v.validate_xyz(0.0, -200.0, 10.0)
    assert {d.axis for d in ei.value.violations} == {"Z", "R"}


def test_only_r_min_set_reports_infinite_max():
    v = _v(r_min=60)
    with pytest.raises(WorkspaceViolationError) as ei:
        v.validate_xyz(0.0, 0.0, 100.0)
    assert math.isinf(ei.value.violations[0].max_value)
    str(ei.value)                            # formats without error


def test_nan_and_inf_are_rejected_before_the_radius_check():
    v = _v(r_min=60, r_max=260)
    for x, y in ((float("nan"), -100.0), (0.0, float("inf"))):
        with pytest.raises(NonFiniteTargetError):
            v.validate_xyz(x, y, 100.0)


def test_get_limits_includes_only_the_set_bounds():
    assert _v(r_min=60, r_max=260).get_limits()["r"] == {"min": 60, "max": 260}
    assert _v(r_min=60).get_limits()["r"] == {"min": 60}
    assert _v(r_max=260).get_limits()["r"] == {"max": 260}
    assert "R=[60, 260]" in repr(_v(r_min=60, r_max=260))


@pytest.mark.parametrize("kw", [
    {"r_min": float("nan")}, {"r_max": float("inf")}, {"r_min": -1},
    {"r_min": 100, "r_max": 50}, {"r_max": "far"},
])
def test_invalid_radius_limits_are_rejected(kw):
    with pytest.raises(ValueError):
        _v(**kw)


def test_other_validators_ignore_radius():
    v = _v(r_min=60, r_max=260)
    v.validate_z_relative(100.0, 20.0)       # no x/y: radius not involved
    v.validate_joint1(10.0)


# ----------------------------------------------------------------------
# workspace_validator_from_config
# ----------------------------------------------------------------------
def test_from_config_merges_override_over_base():
    base = {"x_min": -300, "x_max": 300, "y_min": -300, "y_max": 300,
            "z_min": -130, "z_max": 150, "joint1_min": -135, "joint1_max": 135}
    override = {"y_min": -280, "y_max": -60, "z_min": 40, "z_max": 225,
                "r_min": 60, "r_max": 260}
    v = workspace_validator_from_config(base, override)
    assert (v.x_min, v.x_max) == (-300, 300)
    assert (v.y_min, v.y_max, v.z_min, v.z_max) == (-280, -60, 40, 225)
    assert (v.r_min, v.r_max) == (60, 260)
    assert base["y_min"] == -300, "inputs are not mutated"


def test_from_config_without_override():
    v = workspace_validator_from_config({"z_min": -100}, None)
    assert v.z_min == -100 and v.r_min is None
    v = workspace_validator_from_config(None)
    assert repr(v) == repr(WorkspaceValidator())


MAXARM_WS = {"x_min": -250, "x_max": 250, "y_min": -280, "y_max": -60,
             "z_min": 40, "z_max": 225, "r_min": 60, "r_max": 260}


@pytest.mark.parametrize("key", sorted(MAXARM_WS))
def test_from_config_rejects_a_null_override_value(key):
    """S8: YAML ``z_min:`` (null) must not fall back to the wider class default
    (-130 for z_min) nor silently drop a radius bound."""
    with pytest.raises(ValueError, match=f"workspace.{key} is null"):
        workspace_validator_from_config({}, dict(MAXARM_WS, **{key: None}))


def test_from_config_null_override_does_not_erase_a_measured_base_bound():
    base = dict(MAXARM_WS)
    with pytest.raises(ValueError, match="z_min is null"):
        workspace_validator_from_config(base, {"z_min": None})


@pytest.mark.parametrize("key", ["z_min", "r_min", "joint1_max"])
def test_from_config_rejects_a_null_base_value(key):
    with pytest.raises(ValueError, match=f"workspace.{key} is null"):
        workspace_validator_from_config({key: None}, None)


def test_from_config_null_radius_is_rejected_not_removed():
    """Regression for review finding f8b: r_min: null used to remove the constraint."""
    with pytest.raises(ValueError):
        workspace_validator_from_config({}, dict(MAXARM_WS, r_min=None))
    v = workspace_validator_from_config({}, {k: v for k, v in MAXARM_WS.items() if k != "r_min"})
    assert v.r_min is None and v.r_max == 260    # omitting the key is the way to not use it


def test_from_config_ignores_unknown_keys_with_a_warning(caplog):
    with caplog.at_level(logging.WARNING):
        v = workspace_validator_from_config({"z_min": -100, "speed": 5}, {"colour": "red"})
    assert v.z_min == -100
    assert "speed" in caplog.text and "colour" in caplog.text


def test_from_config_propagates_invalid_limits():
    with pytest.raises(ValueError):
        workspace_validator_from_config({"z_min": 10}, {"z_max": 0})



# ----------------------------------------------------------------------
# R2-7: a null in the global workspace is a ConfigError, never "unlimited"
# ----------------------------------------------------------------------
@pytest.fixture
def _restore_config():
    from src import config as lab_config
    original = (lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH)
    yield lab_config
    lab_config.CONFIG_PATH, lab_config.EXAMPLE_CONFIG_PATH = original
    lab_config.load_config(force_reload=True)


@pytest.mark.parametrize("body", ["workspace:\n  r_min: null\n",
                                  "workspace:\n  r_max:\n",
                                  "workspace:\n  z_min: null\n"])
def test_r2_7_null_in_global_workspace_is_a_config_error(tmp_path, _restore_config, body):
    from src.devices.safety.validators import default_workspace_validator
    lab_config = _restore_config
    cfg = tmp_path / "config.yaml"
    cfg.write_text(body, encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    with pytest.raises(lab_config.ConfigError, match="null"):
        lab_config.get_workspace()
    with pytest.raises(lab_config.ConfigError):
        default_workspace_validator()


def test_r2_7_null_in_a_dobot_robot_workspace_is_a_config_error(tmp_path, _restore_config):
    lab_config = _restore_config
    cfg = tmp_path / "config.yaml"
    cfg.write_text("robots:\n  1:\n    workspace:\n      r_max: null\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    with pytest.raises(lab_config.ConfigError, match="robots.1.workspace"):
        lab_config.get_robot_workspace(1)


def test_r2_7_valid_global_workspace_still_loads(tmp_path, _restore_config):
    from src.devices.safety.validators import default_workspace_validator
    lab_config = _restore_config
    cfg = tmp_path / "config.yaml"
    cfg.write_text("workspace:\n  r_max: 290.0\n", encoding="utf-8")
    lab_config.CONFIG_PATH = str(cfg)
    lab_config.load_config(force_reload=True)
    v = default_workspace_validator()
    assert v.r_max == 290.0 and v.r_min is None
