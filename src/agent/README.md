# `src/agent` — AI agent for experimental-flow generation

Turns a natural-language description of an experiment into a JSON experimental flow
(format: [`docs/experimental-flow.md`](../../docs/experimental-flow.md)) that
`src/flow` validates and executes.

## Files

| File | Role |
|---|---|
| `llm_service.py` | `generate_workflow_from_prompt(text, template=None)` — builds the prompt, calls the LLM, extracts and parses the JSON (one automatic retry if the reply is not valid JSON) |
| `presets/*.json` | Validated flows offered in the GUI and usable as templates for the agent |

## How it works

`SYSTEM_PROMPT` in `llm_service.py` describes the robots (`robot_id` 1–3 Dobot, 4 the optional MaxArm), every
available action with its parameters and units, three worked examples, and the safety
conventions (return home at the end of each loop and of the flow). The user text is
appended and the model is asked for a single JSON object at temperature 0.

With `template=` (a preset's `template` dict) the model is instructed to change only
the parameters of the given flow and keep the step order — the safest way to derive a
new condition from a procedure already checked on hardware. The reply is then checked
against the template (`check_template_structure`): the same number of steps, the same
`action`, `robot_id` and `loop_id` in the same order. Any structural change raises
`TemplateStructureError` with the offending steps, and the GUI shows the error and
keeps the prompt; a failed generation never loads the unchanged template in its place.

Rotation directions in the prompt are stated from a named viewpoint: positive
`rotate_relative` is counter-clockwise seen from above (+Y), which for an operator
standing in front of the robot and facing it ("operator facing the robot") moves the
arm to the operator's right.

The returned dict is **not** trusted: `src/flow/schema.py` validates it before
anything moves, and the GUI shows it for review first.

## Backends

Selected with `LLM_MODE` in `.env` (see `.env.example`):

| `LLM_MODE` | Variables | Notes |
|---|---|---|
| `gemini` (default) | `GEMINI_API_KEY`, `GEMINI_MODEL` (default `gemini-3.5-flash`) | Uses `google-generativeai`, JSON response mode |
| `openai_compatible` | `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` | Any server speaking the OpenAI chat-completions API, e.g. a local inference server |

Keys are read only from the environment / `.env`; nothing is hard-coded.

## Usage

```python
from src.agent.llm_service import generate_workflow_from_prompt

flow = generate_workflow_from_prompt("右に90°回転して70mm下降し、5 mL 排出してホームに戻る")
```

The prompt and the built-in examples are in Japanese; the model handles English
instructions as well.

## Adding an action

1. Add the Pydantic model and register it in `LabRobotAction` (`src/flow/schema.py`).
2. Add the dispatch branch in `src/flow/executor.py`.
3. Describe the action, with one example, in `SYSTEM_PROMPT`.
