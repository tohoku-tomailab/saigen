# National Instruments DAQ (USB-6351)

Basic analog and digital I/O through NI-DAQmx, used by the `daq_*` flow steps.
The code addresses channels generically, so other DAQmx multifunction devices
should work, but the USB-6351 is the only target it was written for.

> Status: implemented and unit-tested against a fake `nidaqmx` module. **Not yet
> exercised on the real device.** Do the bench check below before relying on it.

## What it does

| Flow step | Driver method | Meaning |
|---|---|---|
| `daq_read_analog` | `NIDaq.read_analog(channel, samples, rate_hz, v_range, terminal)` | Mean voltage of `samples` readings of one analog input (`ai0`..). Hardware-clocked at `rate_hz` when `samples > 1`. |
| `daq_write_analog` | `NIDaq.write_analog(channel, voltage)` | Set one analog output (`ao0`, `ao1`) to a DC voltage and hold it. |
| `daq_read_digital` | `NIDaq.read_digital(line)` | Read one digital line (`port0/line0`..) as an input. |
| `daq_write_digital` | `NIDaq.write_digital(line, state)` | Drive one digital line high or low and hold it. |

Not covered (by design, this is the basic set): waveform capture to a file,
waveform generation, counters/encoders, triggering, several channels in one
synchronised acquisition.

## Requirements

1. The **NI-DAQmx driver** from ni.com (administrator rights, reboot). After it,
   the device appears in *NI MAX* under *Devices and Interfaces* with a name such
   as `Dev1`; run *Self-Test* there.
2. `pip install nidaqmx` (listed in `requirements.txt`). The package is imported
   only when a DAQ step actually runs, so mock runs and the tests need neither.

## Configuration (`config.yaml`, section `shared_devices`)

```yaml
daq_device: Dev1        # name shown in NI MAX; "" = no DAQ
daq_ao_channels: []     # analog outputs a flow may drive, e.g. [ao0]
daq_do_lines: []        # digital lines a flow may drive, e.g. [port0/line0]
daq_ao_min_v: 0.0       # allowed analog output span
daq_ao_max_v: 5.0
```

A flow names channels without the device (`ai0`, `ao0`, `port0/line0`), so the
same flow runs in a lab whose device is called something else.

## Safety rules built in

- **Inputs are always allowed. Outputs are refused unless listed** in
  `daq_ao_channels` / `daq_do_lines`, and an analog output must lie inside
  `daq_ao_min_v` .. `daq_ao_max_v`. An output can switch whatever is wired to the
  terminal block; list a channel only after checking the wiring. The mock applies
  the same rule, so `--mock` shows a refusal before the bench does.
- **Every output a run drove returns to 0 V / low when the run ends**, including
  on Ctrl+C and on an error. If a channel cannot be reset, the log says which one;
  switch off what is wired to it by hand.
- The safe state is 0 V / low. Wire loads so that low means off (an active-low
  relay board would switch ON at the end of a run).
- A line being driven as an output cannot be read as an input in the same run
  (that would release the output).
- A power cut or a crash of the PC leaves the outputs to the device's power-up
  state, not to this software. Anything hazardous needs its own hardware
  interlock.

## Electrical limits to respect (check the NI specification before wiring)

- Analog inputs: within the selected range, at most ±10 V. Use the smallest
  `v_range` that contains the signal for the best resolution.
- Outputs are signal-level (a few mA). Do not drive a pump, valve or relay coil
  directly; use a driver stage (relay module, MOSFET, SSR).
- `terminal`: `rse` for a signal referenced to the device ground, `diff` for a
  floating or remote-grounded source wired to an AI+/AI- pair. An unconnected
  input reads an arbitrary, drifting voltage.

## Bench check (reads only, drives nothing)

```bash
python -m src.devices.nidaq.nidaq             # list the devices NI-DAQmx knows
python -m src.devices.nidaq.nidaq Dev1 ai0    # read one analog input
python -m src.flow.run_flow examples/daq/read_voltage.json --mock
python -m src.flow.run_flow examples/daq/read_voltage.json     # real, inputs only
```

A known source makes the reading checkable: a 1.5 V cell across AI0 and AI GND
(`terminal: rse`), or AO0 looped back to AI0 once outputs are enabled.
