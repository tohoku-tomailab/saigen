"""src/gui/app.py rendered headlessly with Streamlit's AppTest.

The parameter editor of every palette block must render without an error, and
widgets that depend on each other must follow the flow (the DMM's fixed range
is dropped when its function changes). Skipped where Streamlit is not
installed (the CI installs a minimal set without it).
"""
import os
import re

import pytest

pytest.importorskip("streamlit")
from streamlit.testing.v1 import AppTest  # noqa: E402

APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "gui", "app.py")


#: one schema-valid step per palette block (a step that fails validation is shown
#: read-only, which would hide an editor that cannot render)
VALID_STEPS = [
    {"action": "move_xyz", "robot_id": 1, "x": 200.0, "y": 0.0, "z": 50.0},           # 0
    {"action": "move_z", "robot_id": 1, "distance": 10.0},                             # 1
    {"action": "move_radial", "robot_id": 1, "distance": 20.0},                        # 2
    {"action": "rotate_relative", "robot_id": 1, "angle": 30.0},                       # 3
    {"action": "go_home", "robot_id": 1},                                              # 4
    {"action": "move_slider", "robot_id": 3, "position": 500.0},                       # 5
    {"action": "move_conveyer", "robot_id": 3, "speed": 10.0, "duration": 5.0},       # 6
    {"action": "wait", "robot_id": 1, "seconds": 2.0},                                 # 7
    {"action": "loop_start", "loop_id": "a", "count": 2},                              # 8
    {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 5},                  # 9
    {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 5},                  # 10
    {"action": "loop_end", "loop_id": "a"},                                            # 11
    {"action": "blow_out", "robot_id": 1},                                             # 12
    {"action": "suction", "robot_id": 4, "on": True},                                  # 13
    {"action": "capture_and_save", "file_path": ""},                                   # 14
    {"action": "capture_microscope", "file_path": ""},                                 # 15
    {"action": "microscope_led", "on": True, "level": 12},                             # 16
    {"action": "microscope_focus", "mode": "step", "direction": "in", "steps": 2,
     "timeout": 60.0},                                                                 # 17
    {"action": "microscope_focus", "mode": "position", "position": 1568},              # 18
    {"action": "measure_weight", "stabilization_count": 3},                            # 19
    {"action": "go_home", "robot_id": 2},                                              # 20
    {"action": "tare_scale", "delay": 1.0},                                            # 21
    {"action": "measure_dmm", "function": "dc_voltage", "samples": 3, "nplc": 1.0,
     "range": 20.0},                                                                   # 22
    {"action": "rotate", "robot_id": 1, "angle": 90.0},                                # 23
]


def _app_with(steps):
    at = AppTest.from_file(APP, default_timeout=120)
    at.session_state["workflow_data"] = {"name": "t", "description": "", "steps": steps}
    at.run()
    return at


def _errors(at):
    return [e.value for e in at.error] + [e.message for e in at.exception]


def test_every_palette_block_renders_its_parameters():
    """Regression: ``_num()`` took the flow step as a parameter named ``step``, so
    every caller passing ``step=0.1`` (the widget's increment) failed with
    "got multiple values for argument 'step'" and showed no editor (aspirate,
    dispense, tare_scale, microscope_focus, ...)."""
    # ACTION_CONFIG is read from the source: importing app.py would run the app
    with open(APP, encoding="utf-8") as f:
        palette = set(re.findall(r'^    "(\w+)": \{\n        "icon"', f.read(), flags=re.M))
    assert {s["action"] for s in VALID_STEPS} == palette, "a palette block has no sample step"

    at = _app_with(VALID_STEPS)
    assert _errors(at) == []
    # the editors were drawn, not skipped (invalid steps are shown read-only)
    keys = {w.key for w in at.number_input}
    for param, idx in (("delay", 21), ("volume", 9), ("timeout", 17), ("nplc", 22)):
        assert any(k and k.startswith(f"{param}_") and k.endswith(f"_{idx}") for k in keys), param


def _dmm_widgets(at):
    pick = lambda items, prefix: [w for w in items if (w.key or "").startswith(prefix)]  # noqa: E731
    return (pick(at.selectbox, "function_"), pick(at.checkbox, "fix_range_"),
            pick(at.number_input, "range_"))


def test_dmm_function_change_resets_the_range_widgets():
    at = _app_with([{"action": "measure_dmm", "function": "dc_voltage", "range": 20.0}])
    fn, fixed, rng = _dmm_widgets(at)
    assert fixed[0].value is True and rng[0].value == 20.0

    fn[0].select("dc_current").run()
    step = at.session_state["workflow_data"]["steps"][0]
    fn, fixed, rng = _dmm_widgets(at)
    assert step == {"action": "measure_dmm", "function": "dc_current"}
    assert fixed[0].value is False and rng == []          # the GUI shows autorange too

    fixed[0].check().run()
    step = at.session_state["workflow_data"]["steps"][0]
    fn, fixed, rng = _dmm_widgets(at)
    assert step["range"] == 2.1 and rng[0].value == 2.1    # the new function's maximum

    rng[0].set_value(0.2).run()
    assert at.session_state["workflow_data"]["steps"][0]["range"] == 0.2
    fixed[0].uncheck().run()
    assert "range" not in at.session_state["workflow_data"]["steps"][0]
    assert _errors(at) == []
