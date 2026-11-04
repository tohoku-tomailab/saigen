"""GUI safety: the sensor gate, SafetyAbort, per-step read-only validation and
the structure of app.py (Mock default, arming, panel order, disabled imports).

Streamlit is not needed: runner.py is Streamlit-free, and app.py is checked
statically (its source / AST), so these tests run in CI without a browser.
"""
import ast
import functools
import os

import pytest

from src.flow.experiment_session import SafetyAbort
from src.gui import runner as gui_runner

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP_PATH = os.path.join(REPO_ROOT, "src", "gui", "app.py")


class _Resp:
    def __init__(self, status=200, body=None, text="", bad_json=False):
        self.status_code = status
        self._body = body
        self.text = text
        self._bad_json = bad_json

    def json(self):
        if self._bad_json:
            raise ValueError("not json")
        return self._body


def _gate(resp=None, exc=None):
    def get(url, timeout):
        assert url.endswith("/api/is_safe")
        if exc is not None:
            raise exc
        return resp
    return functools.partial(gui_runner.check_is_safe, base_url="http://x", http_get=get)


# ----------------------------------------------------------------------
# check_is_safe: only the JSON boolean true is safe
# ----------------------------------------------------------------------
def test_only_safe_true_passes():
    assert _gate(_Resp(200, {"safe": True}))() is True


@pytest.mark.parametrize("body", [
    {"safe": "false"},   # bool("false") is True - the old gate let this through
    {"safe": "true"},
    {"safe": 1},
    {"safe": False},
    {"safe": None},
    {},
    ["safe"],
    None,
])
def test_anything_but_true_is_not_safe(body):
    warnings = []
    assert _gate(_Resp(200, body))(log_fn=warnings.append) is False
    assert warnings


def test_http_error_and_bad_json_are_not_safe():
    assert _gate(_Resp(500, text="boom"))(log_fn=lambda m: None) is False
    assert _gate(_Resp(200, bad_json=True))(log_fn=lambda m: None) is False


def test_unreachable_dashboard_is_optional():
    assert _gate(exc=ConnectionError("refused"))() is True


def test_a_bug_in_the_gate_is_not_swallowed():
    with pytest.raises(ZeroDivisionError):
        _gate(exc=ZeroDivisionError())()


# ----------------------------------------------------------------------
# _safety_gate raises SafetyAbort
# ----------------------------------------------------------------------
def test_safety_gate_raises_safety_abort():
    with pytest.raises(SafetyAbort):
        gui_runner._safety_gate({"action": "go_home"}, check=lambda **kw: False)


def test_safety_gate_rejects_truthy_non_bool():
    with pytest.raises(SafetyAbort):
        gui_runner._safety_gate({"action": "go_home"}, check=lambda **kw: "yes")


def test_safety_gate_passes_when_safe():
    gui_runner._safety_gate({"action": "go_home"}, check=lambda **kw: True)


def test_safety_abort_stops_the_run_without_executing_steps(tmp_path):
    gate = functools.partial(gui_runner._safety_gate, check=lambda **kw: False)
    runner = gui_runner.FlowRunner(
        {"name": "gate", "steps": [{"action": "go_home", "robot_id": 1}]},
        mock=True, logs_dir=str(tmp_path), pre_step=gate)
    status = runner.run_blocking()
    assert status == "aborted", runner.error
    assert "安全確認" in (runner.error or "")
    step_events = [e for e in runner.drain_events() if e.kind == "step"]
    assert step_events == [], "no step may run after the gate refused"


# ----------------------------------------------------------------------
# step_errors: the canvas renders these steps read-only
# ----------------------------------------------------------------------
def test_step_errors_flags_only_the_broken_steps():
    errors = gui_runner.step_errors([
        {"action": "go_home", "robot_id": 1},
        {"action": "aspirate", "robot_id": 1, "volume": "lots", "speed": 5},
        "not a dict",
        {"action": "move_z", "robot_id": 1},
    ])
    assert set(errors) == {1, 2, 3}
    assert "step 2" in errors[1] and "volume" in errors[1]
    assert "step 4" in errors[3] and "distance" in errors[3]


def test_step_errors_does_not_modify_the_steps():
    step = {"action": "aspirate", "robot_id": 1, "volume": "lots"}
    before = dict(step)
    gui_runner.step_errors([step])
    assert step == before


# ----------------------------------------------------------------------
# app.py structure (static)
# ----------------------------------------------------------------------
@pytest.fixture(scope="module")
def app_src():
    with open(APP_PATH, encoding="utf-8") as f:
        return f.read()


@pytest.fixture(scope="module")
def app_tree(app_src):
    return ast.parse(app_src)


def _func(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in app.py")


def test_mock_is_the_default_and_real_needs_arming(app_src):
    assert "st.session_state.use_mock = False" not in app_src
    assert "st.session_state.arm_real_run = False" in app_src
    assert '"実機で実行する"' in app_src
    assert "confirm_real_run_dialog" in app_src
    # start_run defaults to mock, and only the confirmation passes mock=False (with the
    # MG400 ids the dialog showed, which the runner requires for a real MG400 run)
    assert app_src.count("start_run(mock=False") == 1
    assert app_src.count("start_run(mock=False, confirmed_robots=mg400_used)") == 1


def test_start_run_defaults_to_mock(app_tree):
    fn = _func(app_tree, "start_run")
    assert fn.args.args[0].arg == "mock"
    assert isinstance(fn.args.defaults[0], ast.Constant) and fn.args.defaults[0].value is True


def test_real_run_confirmation_disarms(app_tree):
    src = ast.unparse(_func(app_tree, "confirm_real_run_dialog"))
    assert src.index("_disarm_real_run") < src.index("start_run(mock=False, confirmed_robots=mg400_used)")


def test_execution_panel_is_drawn_before_the_canvas(app_tree):
    top_calls = []
    for node in app_tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue  # only the module-level script order matters
        for sub in ast.walk(node):
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name):
                top_calls.append((sub.lineno, sub.func.id))
    lines = {name: ln for ln, name in sorted(top_calls, reverse=True)}
    assert lines["render_execution_panel"] < lines["render_workflow_canvas"]


def test_canvas_rendering_is_guarded(app_tree):
    for node in app_tree.body:
        if isinstance(node, ast.Try):
            if "render_workflow_canvas" in ast.unparse(node.body[0]):
                return
    raise AssertionError("render_workflow_canvas() must be wrapped in try/except")


def test_imports_and_ai_are_disabled_while_running(app_src):
    assert 'st.file_uploader("📂 JSON読込", type="json", label_visibility="collapsed",\n' \
           '                                     disabled=running' in app_src
    assert 'st.button("✨ AIで新規作成", use_container_width=True, disabled=is_running())' in app_src
    assert 'key="empty_state_generate", disabled=running' in app_src


def test_set_workflow_replaces_only_after_validation(app_tree):
    src = ast.unparse(_func(app_tree, "set_workflow"))
    assert src.index("validate_workflow") < src.index("st.session_state.workflow_data = data")
    assert "is_running()" in src


def test_ai_failure_does_not_load_the_template(app_src):
    assert "テンプレートから生成しました" not in app_src


def test_render_step_params_never_assigns_into_the_step(app_tree):
    """Drawing the canvas must not write defaults / coerced values into the flow."""
    fn = _func(app_tree, "render_step_params")
    for node in ast.walk(fn):
        if isinstance(node, (ast.Assign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for t in targets:
                assert not (isinstance(t, ast.Subscript) and isinstance(t.value, ast.Name)
                            and t.value.id == "step"), ast.unparse(node)


def test_gui_volume_minimum_matches_schema(app_src):
    assert "min_value=PIPETTE_MIN_VOLUME_ML" in app_src
    for line in app_src.splitlines():
        if '"容量 (mL)"' in line:
            assert "PIPETTE_MIN_VOLUME_ML" in line, line
