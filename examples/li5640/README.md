# NF LI5640 example

`read_lockin.json` reads the NF LI5640 lock-in amplifier over GPIB twice: five
readings back to back, then five readings one second apart. It uses only the
lock-in amplifier, so no robot is opened.

The flow only reads. Sensitivity, time constant, reference, input, filters and
the DATA1 / DATA2 display are set on the front panel. The only setting SAIGEN
writes is the read-out selection `OTYP`, which it restores when it closes the
instrument (see [`src/devices/li5640/README.md`](../../src/devices/li5640/README.md)).

Set the VISA resource in `config.yaml` (see
[`docs/setup.md`](../../docs/setup.md), section 11):

```yaml
shared_devices:
  lockin_resource: "GPIB0::<address>::INSTR"
```

or pass `--lockin-resource GPIB0::<address>::INSTR`. The address must differ
from every other instrument on the same GPIB bus; check it on each front panel.
Then:

```bash
python -m src.flow.run_flow examples/li5640/read_lockin.json --validate-only
python -m src.flow.run_flow examples/li5640/read_lockin.json --mock
python -m src.flow.run_flow examples/li5640/read_lockin.json --no-record
```

`--no-record` keeps the first real run to the instrument: it switches off the
sensor CSV and video recording only (the run folder is still written).

Each step writes the median of DATA1 and DATA2 with their kind (`X`, `R`,
`THETA`, `NOISE`, `AUX1`, ...; `X[dB]`, `Y[ratio]` when normalize or ratio is on)
and unit, the measured reference frequency, sensitivity, time constant, the
number of valid readings, the readings discarded and why, and the panel settings
(JSON) to the `lockin_*` columns of `measurements.csv`; `summary.md` lists them
under *ロックイン測定値*. In `--mock` DATA1 is X = 0.001 V, DATA2 is Y = 2e-05 V
and the frequency is 1000 Hz.

Notes:

- **Settling is not waited for.** After any change on the panel, wait before
  the step (Table 4-3 of the manual: 99 % of the final value after about 10
  time constants at 24 dB/oct; noise density about 170 time constants times the
  smoothing factor at 24 dB/oct). `interval` only spaces the readings.
- **Do not use the generic `wait` action in a lock-in-only flow**: its
  `robot_id` defaults to 1, so a real run would open robot 1. Use
  `wait_after_lockin` in `config.yaml` for a pause after each step.
- A step fails if a panel setting differs before and after its readings, if no
  reading is valid (over-level, UNLOCK), or if the phase readings spread over
  180° or more. The settings before and after are then in the `diagnostics`
  column and in `metadata.json`.
