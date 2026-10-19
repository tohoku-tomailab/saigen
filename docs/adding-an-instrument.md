# Adding a new instrument

This guide walks through adding an instrument that the flow executor can drive,
using a digital multimeter (DMM) as the running example. The same steps apply to
any instrument that does not belong to a particular robot arm: a pH meter, a
spectrometer, a second balance. Instruments mounted on an arm (a second pipette,
a gripper) go through `LabRobot` instead of `SharedDevices`; the layering is the
same, only the class differs.

The DMM in the steps below is a generic, hypothetical one (`measure_voltage`,
`dmm_port`) so that the steps stay short. A real DMM, the Keithley 2002 over
GPIB, was later added by following them: its action is `measure_dmm`, its
setting `dmm_resource`, and its code (`src/devices/keithley2002/`,
`SharedDevices.measure_dmm`, `tests/test_keithley2002.py`) is the worked
example for an instrument whose I/O blocks; what its commissioning taught is in
[`keithley2002-commissioning.md`](keithley2002-commissioning.md). The NF LI5640
lock-in amplifier (`src/devices/li5640/`, `measure_lockin`,
`tests/test_li5640.py`) followed next, as a read-only instrument whose values
mean what its front panel says; its notes are in
[`li5640-commissioning.md`](li5640-commissioning.md). The Agilent 1000 series
oscilloscope (`src/devices/dso1000/`, `measure_scope`, `tests/test_dso1000.py`)
is the third such instrument and the first over USB; it shows how to record
several quantities per step (JSON columns keyed by item) and how to refuse
instrument states whose meaning the code cannot check (X-Y time base, non-edge
trigger) instead of recording ambiguous values; its notes are in
[`dso1000-commissioning.md`](dso1000-commissioning.md).

The electronic balance is the closest existing example, and every step below
names the balance code to copy from. Read `src/devices/README.md` first for the
two-layer design (thin drivers, one safety wrapper).

## 0. Decide where it belongs

| Question | Answer for a DMM | Consequence |
|---|---|---|
| Is it attached to one robot arm? | No, it sits on the bench | Shared device: actions carry no `robot_id`, implemented in `SharedDevices` |
| Does a step produce a value worth recording? | Yes, a voltage | The step returns a result dict and the run log gets a new column |
| Does it move anything or dispense anything? | No | No workspace validation needed; nothing in `src/devices/safety/validators/` changes |

If the probe has to be positioned by the arm, keep the *measurement* a shared-device
action and express the positioning as ordinary `move_xyz` / `move_z` steps in the
flow. Do not add motion to a shared device.

## 1. Driver: `src/devices/<instrument>/`

Create `src/devices/dmm/` with `__init__.py`, a driver module and a short README
(command set, wiring, one usage example).

- Copy the shape of `SerialBalance` in `src/devices/scale/BCE8221.py`: a class that
  opens the connection in `__init__`, exposes a few instrument-level methods
  (`read_voltage()`, `read_current()`, ...) and a `close()`.
- The driver knows nothing about experiments, flows or robots.
- Every address (COM port, VISA resource string, Bluetooth address) is a
  constructor argument with a placeholder default. Never hard-code a real one.
- Use the protocol the instrument speaks. Most bench DMMs (Keysight, Keithley,
  Rigol) accept SCPI, e.g. `MEAS:VOLT:DC?`, over USB-TMC (`pyvisa`) or RS-232C /
  USB-CDC (`pyserial`). Add the library to `requirements.txt`, and if it is
  heavy or platform-specific, import it inside the driver module, not at package
  level, so the mock path never needs it (the balance does this with `serial`).
- Parse the reply into a `float` and return `None` when the reply is not a
  number, as `SerialBalance.get_weight()` does; the wrapper decides what to do
  with `None`.

## 2. Safety wrapper: `src/devices/safety/shared_devices.py`

Add the instrument to `SharedDevices` following the balance:

1. A `use_dmm: bool = False` constructor flag and `self.dmm = None`.
2. A settle time read from `device_configs` (`wait_after_dmm`, like `wait_after_scale`).
3. `_initialize_dmm()`: import the driver lazily, construct it from
   `device_configs['dmm_port']`, take one reading to prove the link, return
   `False` (with a logged error) on failure. Call it from `initialize()` when
   the flag is set.
4. The public async method the executor will call, e.g.
   `async def measure_voltage(self, samples: int = 3) -> float`. Take several
   readings and return the median, as `measure_weight()` does, then sleep the
   settle time. Raise, do not return `None`, when the instrument gives nothing
   usable; the executor records the exception per step.
5. Close the driver in `cleanup()`.

Do not touch `src/devices/safety/validators/` or the `workspace` section of
`config.yaml`; a measuring instrument has no business there.

### Instruments whose I/O blocks

A reading that can take seconds (high integration time, a slow bus, a timeout)
must not freeze the event loop or delay an emergency stop. The Keithley 2002 in
`SharedDevices` (`_initialize_dmm`, `measure_dmm`, `_close_dmm`) is the
reference; the rules, each of which was a real defect caught in review:

- Run the I/O on a worker thread (`asyncio.to_thread`) under a
  `threading.Lock`, so a measurement and the clean-up never use the bus at the
  same time.
- On cancellation, set a stop flag and re-raise at once; do not wait for the
  worker. The robots are emergency-stopped first and the device is closed
  afterwards. The thread itself cannot be killed; it stops at the next check.
- Check the stop flag before every ordinary command you send (the 2002
  driver does it in `write` / `query` through `abort_check`), not only before
  each reading. The clean-up itself (device clear, GTL, close) is not gated.
- Close the handle only while holding the lock. If the clean-up gives up
  waiting, leave the closing to the worker when its I/O ends (give each I/O a
  timeout; a query is a write plus a read, so that is not a bound on the whole
  operation); never close under an I/O in flight.
- A failed close is logged and the handle kept for the next `cleanup()`;
  nothing in a worker may raise into the void.
- Do not swallow I/O errors on read-back queries: a late reply desynchronises
  the next query.
- With more than one such instrument, `SharedDevices.cleanup()` closes each in
  its own `try`, remembers a cancellation, tries the remaining ones and only
  then re-raises it: a second Ctrl+C while one instrument is being closed must
  not leave the next one open.
- If the instrument has a setting you change only to read it (the LI5640's
  `OTYP`), read the old value first and restore it on a path that is exempt
  from the stop flag, after a device clear (a timed-out write may have left
  half a command in the input buffer), and still send GTL and close if the
  restore fails. Restore nothing you never read.
- `ExperimentSession.add_shared()` closes a device whose `initialize()` failed,
  and hands an interrupted one to `run()`'s clean-up after the emergency stop;
  `_initialize_<name>()` therefore only has to leave no handle of its own open.

## 3. Mock: `src/devices/safety/mock_robot.py`

`MockSharedDevices` must accept the same constructor flag and expose the same
method with the same signature. Return a fixed, plausible value
(`MOCK_VOLTAGE = 1.234`, next to `MOCK_WEIGHT`), never `None`: the mock run is
what tests, CI and `--mock` use to check that the value reaches the log.

## 4. Flow schema and executor

`src/flow/schema.py`

- Copy `ActionMeasureWeight` to `ActionMeasureVoltage` with
  `action: Literal["measure_voltage"]` and bounded parameters
  (`samples: int = Field(3, ge=1, le=10)`, range or function if the instrument
  needs them). Use `Field` bounds; they are the only validation a flow gets
  before it runs.
- Add the class to the `LabRobotAction` union. Only listed classes are valid steps.

`src/flow/executor.py`

- Add `"measure_voltage"` to `SHARED_DEVICE_ACTIONS` so the step is routed to
  the shared devices and is accepted without a `robot_id`.
- Add a branch in `execute_shared_device_step()` that reads the parameters from
  either a dict or the model (both forms reach it), calls the wrapper and
  returns `{"voltage": value}`.
- In `plan_resources()` (`src/flow/run_flow.py`), derive `needs_dmm` from the
  actions present in the flow, next to `needs_scale` and `needs_camera`, return
  it with the other flags, and pass `use_dmm=needs_dmm` through
  `ExperimentSession.add_shared()` (`src/flow/experiment_session.py`, whose
  shared-device factories forward it to `SharedDevices` / `MockSharedDevices`).
  `plan_resources()` returns a `ResourcePlan` (a `NamedTuple`) read by name:
  add the field, then extend `needs_shared`, `shared_kwargs()` and
  `shared_labels()`; the three runners (`run_flow`, the CSV runner, the GUI
  runner) pick it up from there. This is what lets a flow that never measures
  voltage run on a cell without a DMM.

`src/flow/experiment_logger.py`

- `record_step()` maps result keys to `measurements.csv` columns (`weight` to
  `weight_g`, `image_path`, `focus_position`, and the `dmm_*` keys of the
  Keithley 2002). Add the new key the same way (`voltage_v`), and add the event
  kind to the time-line report if the value is worth listing there. An
  instrument that measures several quantities should report value, unit and
  function (as `dmm_value` / `dmm_unit` / `dmm_function` do) rather than one
  column per quantity, and keep full precision (`repr`, not a fixed format).
- A step that fails for a reason worth recording (the LI5640's panel changed
  during the readings, every reading was invalid) raises an exception with a
  `diagnostics` dict. All three runners pass `getattr(e, "diagnostics", None)`
  to `record_step()`, which writes it to the `diagnostics` column, the
  time-line and `metadata.json` (`step_diagnostics`). An exception message
  alone loses the structured data.

## 5. Configuration

- `src/config.py`: add `dmm_port` to the `shared_devices` defaults.
- `config.example.yaml`: add the same key with a comment. `config.yaml` is
  gitignored; users copy the example.
- Optional: a `--dmm-port` argument and `DMM_PORT` environment variable in
  `src/flow/run_flow.py`, mirroring `--scale-port` / `SCALE_PORT`, so a port can
  be overridden per run without editing the file.

## 6. Tests

All tests run without hardware.

- Driver parsing: copy `tests/test_wp2_balance_sign.py`. Stub the serial or
  VISA module, build the driver with `object.__new__`, and check that the
  instrument's real reply strings (copy them from the manual or a terminal
  session) parse to the right number and that garbage gives `None`.
- End to end: add a flow containing `measure_voltage` to the mock-run test in
  `tests/test_run_flow.py` and assert that `measurements.csv` has the value in the new
  column. This catches a missing union entry, a missing action in
  `SHARED_DEVICE_ACTIONS`, and a mock that returns `None`.
- Blocking I/O (see §2): drive the wrapper with a fake resource whose write or
  query blocks on a `threading.Event`, then cancel during initialisation,
  during configuration, during the last reading and during clean-up, and
  assert the order of events (emergency stop before the device is released,
  no ordinary command sent after an abort, no I/O after close). Then revert
  each fix once and check that its test fails: a test that releases the fake
  too early passes against the broken code as well.
- Test what the code asks for, not wall-clock time: a check of `elapsed >= 0.3`
  after `asyncio.sleep(0.3)` failed now and then on Windows (the timer
  granularity there was about 15 ms); record the requested pause instead.
- Run the full check before reporting done:

```bash
python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q
python -m src.flow.run_flow <your flow>.json --validate-only
python -m src.flow.run_flow <your flow>.json --mock
```

## 7. GUI (optional): `src/gui/app.py`

The steps above make the action usable from JSON flows, the CSV runner and the
flow-generating agent. The Streamlit GUI has its own block palette and needs
three more edits, all in `src/gui/app.py`, only if users should be able to add
the step from the GUI:

- `ACTION_CONFIG`: an entry keyed by the action name with an icon, a Japanese
  label and the default parameters, e.g.
  `"measure_voltage": {"icon": "🔋", "label": "電圧測定", "category": "sensor", "defaults": {"samples": 3}}`.
- `CATEGORIES`: add the action name to the list of the group it should appear
  under in the toolbox (`"sensor"` for the balance and camera).
- `get_param_summary()` and `render_step_params()`: one `elif action == ...`
  branch in each, the first returning a one-line summary of the parameters,
  the second drawing the input widgets (copy the `measure_weight` branches).

The GUI imports `SHARED_DEVICE_ACTIONS` from the executor, so a step whose
action is in that set is created without a `robot_id` automatically. The
shared-device settings of a GUI run start from `config.yaml` and only the
sidebar fields are overlaid (`settings_to_session_args()` in
`src/gui/runner.py`), so a new configuration key reaches the session's
settings without a sidebar field. It still has to be passed on to the device:
`default_shared_factory()` in `src/flow/experiment_session.py` lists the keys
it gives `SharedDevices`; add yours there (§4), or the setting is silently
dropped. A keyed Streamlit widget keeps its state when its default
changes: if one parameter invalidates another (the 2002's range is in the unit
of its function), reset the dependent widget's state in the `on_change`
callback, as `_change_dmm_function()` does, or the GUI shows a value the step
no longer has. Validation
and the Mock run in the GUI go through the same schema and executor as the CLI,
so nothing else is needed. Verify with `streamlit run src/gui/app.py`: add the
block, check the parameters, run *Mock*. Also add a schema-valid sample of the
new step to `VALID_STEPS` in `tests/test_gui_app_params.py` (the test fails
until every palette block has one); it renders every block's editor headlessly
with Streamlit's `AppTest` and is skipped where Streamlit is not installed,
as in the CI.

## 8. Documentation

- `docs/experimental-flow.md`: one row in the *Shared devices* table.
- `src/devices/README.md`: one row in the driver table and the method in the
  `SharedDevices` row.
- `docs/bom.md`: the instrument, with model and interface.
- `docs/setup.md`: how to connect it and find its port.
- `src/agent/llm_service.py`: the action list given to the flow-generating
  LLM. Add the action there if the agent should be able to use it; leave it out
  if it should not.

## Checklist

```
[ ] src/devices/<name>/            driver + README, addresses as arguments
[ ] shared_devices.py              flag, _initialize_*, async method, cleanup
[ ] mock_robot.py                  same method on MockSharedDevices, fixed value
[ ] schema.py                      Action* model, added to LabRobotAction
[ ] executor.py                    SHARED_DEVICE_ACTIONS, branch, needs_* flag
[ ] experiment_logger.py           result key -> CSV column
[ ] config.py + config.example.yaml   port key
[ ] tests/                         parser test + mock-run test
[ ] src/gui/app.py (optional)      ACTION_CONFIG, CATEGORIES, two elif branches
[ ] docs/experimental-flow.md, src/devices/README.md, docs/bom.md, docs/setup.md
[ ] pytest, --validate-only, --mock all green
```

## What not to do

- Do not make the executor call a driver directly. Every instrument goes
  through `SharedDevices` or `LabRobot`, so that the mock, the logging and the
  emergency stop cover it.
- Do not put a lab's port names, IP addresses or Bluetooth addresses in
  defaults, examples or tests.
- Do not add an instrument whose driver depends on an AGPL package under
  `src/`; `detection/` is kept separate for that reason.
