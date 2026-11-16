# NI DAQ example flows

| File | Drives outputs? | Needs | What it shows |
|---|---|---|---|
| `read_voltage.json` | no | `daq_device` | Averaged voltage readings and one digital input |
| `acquire_and_count.json` | no | `daq_device` | Waveform capture to CSV, continuous recording in the background, the counters measuring the internal 100 kHz timebase (no wiring) |
| `timing_and_triggers.json` | no | `daq_device` | Period, pulse width and edge-to-edge time of the internal timebase; an acquisition on an external clock and one with pre-trigger samples; a digital waveform of port 0; an encoder (no wiring) |
| `output_loopback_check.json` | **yes**, into the board's own inputs | four jumpers, outputs enabled for the check | Bench check of every output function by loop-back (below) |
| `stimulus_response_template.json` | **yes** | signals defined, wired and enabled | A flow written with signal names only: DC level, waveform excitation, triggered-by-condition waits, valve pulse, pulse train |

The first three are safe with nothing wired: `--validate-only`, `--mock`, then a
real run. An unconnected analog input reads an arbitrary, drifting voltage.
They contain `daq_*` steps only, so a real run opens the DAQ and nothing else.
A `wait` step belongs to a robot (robot 1 unless `robot_id` says otherwise): add
one to a DAQ flow and a real run also connects to that arm, without moving it.

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

## Bench check of the outputs: `output_loopback_check.json`

Each output is wired straight back to an input of the same board, so the board
checks itself and nothing external can be switched. **Disconnect everything else
from the terminal block first.** Four jumpers:

| From | To | Checks |
|---|---|---|
| AO 0 | AI 0 | DC output, waveform output |
| AO 1 | AI 1 | second analog output; DC and waveform at the same time |
| P0.0 | P0.1 | digital output and pulse |
| PFI 12 (ctr0 out) | PFI 0 | pulse train |

The human then enables the four outputs for the check in `config.yaml`
(section `shared_devices`) and removes them again afterwards unless the
experiment needs them:

```yaml
daq_ao_channels: [ao0, ao1]
daq_do_lines: [port0/line0]
daq_co_counters: [ctr0]
daq_ao_min_v: 0.0
daq_ao_max_v: 5.0
daq_co_max_hz: 1000.0
```

`--validate-only`, `--mock` (it passes only once the outputs are enabled; mock
readings are fixed values and prove nothing about the board), then the real run.
What `measurements.csv` and the console should show, step by step:

| Steps | Expected |
|---|---|
| 1–6 | ai0 reads 2.5 V, then 1.0 V, then 0 V (within a few mV) |
| 7–10 | the waveform CSV in `data/` holds 5 cycles of a sine between 1 V and 3 V (mean 2 V); afterwards ai0 reads 0 V |
| 11–16 | port0/line1 reads High, then Low, then Low again after the pulse |
| 17–20 | 100 Hz measured on pfi0; about 200 edges in 2 s |
| 21 | 0 edges: the train has stopped |
| 22 | the step returns after about 0.5 s (50 pulses at 100 Hz) |
| 23 | the CSV holds 5 cycles of a triangle between 1 V and 3 V in `ai0_V`, starting at 2 V at time 0 and following the `ao0_set_V` column |
| 24 | in the CSV the recorded `port0/line1` column repeats the played `port0/line0_set` column 1 0 1 1 0 0 1 0 (possibly one row later, the last state then missing); afterwards the line is low |
| 25–27 | ai1 holds 1.5 V while ai0 shows a square wave between 1 V and 3 V |
| 28–30 | both back at 0 V |

Steps 23 and 24 use functions that had only been run against a simulated driver
when this was written (output and acquisition started together; a clocked digital
pattern with its response). Steps 25–27 drive a DC level and a waveform at the
same time, which had not been tried on a device either. If DAQmx refuses one of
them the flow stops there with its error, the outputs are reset, and everything
before it has still been checked; report the error text.
