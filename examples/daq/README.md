# NI DAQ example flows

| File | Drives outputs? | Needs | What it shows |
|---|---|---|---|
| `read_voltage.json` | no | `daq_device` | Averaged voltage readings and one digital input |
| `acquire_and_count.json` | no | `daq_device` | Waveform capture to CSV, continuous recording in the background, the counters measuring the internal 100 kHz timebase (no wiring) |
| `stimulus_response_template.json` | **yes** | signals defined, wired and enabled | A flow written with signal names only: DC level, waveform excitation, triggered-by-condition waits, valve pulse, pulse train |

The first two are safe with nothing wired: `--validate-only`, `--mock`, then a
real run. An unconnected analog input reads an arbitrary, drifting voltage.

## The template: wiring lives in `config.yaml`, the experiment in the flow

`stimulus_response_template.json` names no terminal. It passes `--validate-only`
as shipped, and stops at the first step under `--mock` until `config.yaml` says
what the names mean and which outputs may be driven. An entry of this shape
(section `shared_devices`; the channels below are placeholders, use your wiring):

```yaml
daq_device: Dev1
daq_signals:
  lamp:          {channel: ao0}
  excitation:    {channel: ao1}
  photodiode:    {channel: ai0, terminal: diff, v_range: 1, scale: 2.0, offset: 0.0, unit: mW}
  valve:         {channel: port0/line0}
  ready:         {channel: port0/line1}
  stirrer_clock: {channel: ctr0}        # output on PFI12
daq_ao_channels: [ao0, ao1]             # the human enables outputs, after checking the wiring
daq_do_lines: [port0/line0]
daq_co_counters: [ctr0]
daq_ao_min_v: 0.0
daq_ao_max_v: 5.0
daq_co_max_hz: 1000.0
```

Another lab with the photodiode on `ai5` changes one line of its `config.yaml`
and runs the same flow. `measurements.csv` records both the name and the channel
(`photodiode:ai0`), the voltage and the scaled value with its unit.

Copy the template, adapt the steps, then `--validate-only` and `--mock`. Run it
for real only after the mock run succeeded and the loads are known to be safe at
the levels the flow sets; every output returns to 0 V / low / stopped when the run
ends.
