# Agilent 1000 series oscilloscope example

`read_scope.json` reads automatic measurements of channel 1 from an Agilent
1000 series oscilloscope (DSO1014A and the other five models of the series)
over USB, twice: Vpp, mean and frequency, then positive duty cycle and rise
time. It uses only the oscilloscope, so no robot is opened.

The flow only reads. Vertical scale, time base, trigger and acquisition are
set on the front panel; SAIGEN sends queries only (plus a device clear and
`*CLS` when it opens the instrument). See
[`src/devices/dso1000/README.md`](../../src/devices/dso1000/README.md).

Set the VISA resource in `config.yaml` (see
[`docs/setup.md`](../../docs/setup.md), section 12). It contains the
instrument's serial number, so it belongs there and not in tracked files:

```yaml
shared_devices:
  scope_resource: "USB0::0x0957::<product id>::<serial>::INSTR"
```

or pass `--scope-resource ...`. Then:

```bash
python -m src.flow.run_flow examples/dso1000/read_scope.json --validate-only
python -m src.flow.run_flow examples/dso1000/read_scope.json --mock
python -m src.flow.run_flow examples/dso1000/read_scope.json --no-record
```

Each step writes, per item, the median of the valid readings (`scope_values`,
JSON), its unit (`scope_units`: V / A / W from the channel's units for the
voltage items, Hz, s, %) and the number of valid readings (`scope_valid`), the
readings discarded and why (`scope_invalid`), and the panel settings
(`scope_settings`) to `measurements.csv`; the raw replies go to
`metadata.json`. In `--mock` channel 1 shows a 2 Vpp, 1 kHz square wave.

Notes:

- `samples` is the number of reads, not of acquisitions: the oscilloscope does
  not acquire while it answers (programmer's guide p.27), and nothing
  guarantees a new acquisition within `interval`.
- A step fails if the time base is X-Y or DELAYED, the trigger is not an edge
  trigger, the channel is not displayed, a setting differs before and after
  the reads, any requested item has no valid reading, or the instrument
  reports an error.
- Do not use the generic `wait` action in an oscilloscope-only flow (it opens
  robot 1); use `wait_after_scope` in `config.yaml` for a pause after each step.
