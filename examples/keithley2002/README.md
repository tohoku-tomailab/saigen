# Keithley 2002 example

`measure_dc_voltage.json` measures a DC voltage three times with the Keithley
2002 digital multimeter over GPIB: on autorange, on the fixed 20 V range at
NPLC 10, and on autorange again. It uses only the DMM, so no robot is opened.

Set the VISA resource of the instrument in `config.yaml` (see
[`docs/setup.md`](../../docs/setup.md), section 10):

```yaml
shared_devices:
  dmm_resource: "GPIB0::16::INSTR"
```

or pass `--dmm-resource GPIB0::16::INSTR`. Then:

```bash
python -m src.flow.run_flow examples/keithley2002/measure_dc_voltage.json --validate-only
python -m src.flow.run_flow examples/keithley2002/measure_dc_voltage.json --mock
python -m src.flow.run_flow examples/keithley2002/measure_dc_voltage.json
```

Each step writes the median value, unit, function, the range and NPLC read
back from the instrument and the number of valid samples to the `dmm_*`
columns of `measurements.csv`; `summary.md` lists them under *DMM 測定値*. In
`--mock` every reading is 1.234 in the unit of the function.

To measure something else, change `function` (`ac_voltage`, `dc_current`,
`ac_current`, `resistance_2w`, `resistance_4w`); `range` is then in A or Ohm.
To measure the product of an arm's work, put `measure_dmm` steps between the
robot steps of any flow.
