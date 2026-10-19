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

## Oscillator sweep read on the oscilloscope

`oscillator/osc_sweep_scope.json` **sets** the internal oscillator of the LI5640
(`set_lockin_oscillator`) and reads OSC OUT on CH1 of an Agilent 1000 series
oscilloscope: 0.05, 0.1, 0.2 and 0.3 Vrms at 100 Hz, then 50 and 200 Hz at
0.3 Vrms. After each setting has been read back it waits 1 s, reads the lock-in
amplifier (its measured reference frequency) and the oscilloscope (frequency,
period, Vpp, Vrms, mean). It is kept in a subfolder because it is refused with the
default configuration: writing the oscillator is off unless `config.yaml` allows it.

Before running it:

- Only the oscilloscope on OSC OUT (BNC to CH1, 1X). The limits below are for that
  load; choose your own for anything else.
- `config.yaml`:

  ```yaml
  shared_devices:
    lockin_resource: "GPIB0::<address>::INSTR"
    scope_resource: "USB0::0x0957::<product ID>::<serial>::INSTR"
    lockin_osc_max_vrms: 0.3
    lockin_osc_min_hz: 10.0
    lockin_osc_max_hz: 1000.0
  ```

- On the LI5640: the amplitude range 500 mV, and an oscillator setting inside the
  limits (the run writes it back at the end, so it must be allowed too).
- On the oscilloscope: CH1 shown, 0.2 V/div, DC, 5 ms/div, edge trigger on CH1, Run.

```bash
python -m src.flow.run_flow examples/li5640/oscillator/osc_sweep_scope.json --validate-only
python -m src.flow.run_flow examples/li5640/oscillator/osc_sweep_scope.json --mock
python -m src.flow.run_flow examples/li5640/oscillator/osc_sweep_scope.json --no-record
```

At the end, `summary.md` and `metadata.json` (`lockin_oscillator_restore`) say whether
the oscillator was set back to its starting values. If they say `failed` or
`unverified`, check OSC OUT on the front panel. In `--mock` the oscillator starts at
the first frequency of the flow and 0 V.
