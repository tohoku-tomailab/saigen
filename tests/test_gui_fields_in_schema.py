"""Every field the GUI step editor writes is a field of that action's schema.

The schema rejects unknown keys (``extra="forbid"``), so an editor widget that
writes a field the schema does not define turns a valid flow into one that is
rejected (roadmap item 0-10: ``rotate`` got a speed selector meant for
``rotate_relative``).

The check reads ``render_step_params`` in ``src/gui/app.py`` with ``ast`` (no
Streamlit needed) and follows the ``action ==`` / ``action in`` branches,
including nested ones, to the four widget helpers ``_num``, ``_select``,
``_check`` and ``_text``, whose fifth argument is the field name. Widgets with
their own callbacks (the DMM's fixed range, the scope's item list) write
through other code and are not covered here.
"""
import ast
import os
import typing

from src.flow.schema import LabRobotAction

APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "gui", "app.py")
HELPERS = {"_num", "_select", "_check", "_text"}


def _schema_fields():
    fields = {}
    for model in typing.get_args(LabRobotAction):
        action = typing.get_args(model.model_fields["action"].annotation)[0]
        fields[action] = set(model.model_fields)
    return fields


def _action_set(test, all_actions):
    """Actions for which an ``if`` test on ``action`` is true; None if it is not one."""
    if not (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
            and test.left.id == "action" and len(test.ops) == 1):
        return None
    op, right = test.ops[0], test.comparators[0]
    if isinstance(right, ast.Constant):
        values = {right.value}
    elif isinstance(right, (ast.List, ast.Tuple, ast.Set)):
        values = {e.value for e in right.elts}
    else:
        return None
    if isinstance(op, (ast.Eq, ast.In)):
        return values
    if isinstance(op, (ast.NotEq, ast.NotIn)):
        return all_actions - values
    return None


def _editor_writes():
    """{action: {field, ...}} written through the helpers in render_step_params."""
    with open(APP, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    func = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == "render_step_params")
    all_actions = set(_schema_fields())
    writes = {}

    def visit(node, actions):
        if isinstance(node, ast.If):
            chosen = _action_set(node.test, all_actions)
            if chosen is not None:
                for stmt in node.body:
                    visit(stmt, actions & chosen)
                for stmt in node.orelse:
                    visit(stmt, actions - chosen)
                return
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id in HELPERS):
            field = node.args[4]
            assert isinstance(field, ast.Constant), ast.dump(node)
            for action in actions:
                writes.setdefault(action, set()).add(field.value)
        for child in ast.iter_child_nodes(node):
            visit(child, actions)

    for stmt in func.body:
        visit(stmt, all_actions)
    return writes


def test_the_editor_reaches_most_actions():
    # guards the walker itself: if it stopped finding the helpers, the main
    # test would pass vacuously
    writes = _editor_writes()
    assert {"move_xyz", "rotate_relative", "aspirate", "measure_dmm"} <= set(writes)
    assert writes["rotate_relative"] >= {"angle", "speed"}


def test_every_field_the_editor_writes_is_in_the_schema():
    schema = _schema_fields()
    unknown = {action: sorted(fields - schema[action])
               for action, fields in _editor_writes().items() if fields - schema[action]}
    assert unknown == {}
