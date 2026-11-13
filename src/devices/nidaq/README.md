# National Instruments DAQ (USB-6351)

Analog, digital and counter I/O through NI-DAQmx, used by the `daq_*` flow steps.
The code addresses channels generically, so other X Series devices should work,
but the USB-6351 is the only target it was written for and its limits (sample
rates, 4 counters) are the ones built in.

> Status (2026-11-13, NI-DAQmx 26.8, one USB-6351): **every input function was
> exercised on the real device** — averaged reading, waveform capture of one and
> two channels (up to 1 MS/s), continuous recording, digital input, edge counting
> and frequency measurement on the internal 100 kHz timebase, the timeout of a
> start trigger that never comes. **No output has been driven on the real device
> yet** (DC and waveform analog output, digital output and pulse, pulse train):
> these are unit-tested against a fake `nidaqmx` module only. Check each output on
> the bench, with nothing hazardous wired, before relying on it.

## Three layers

| Layer | Where | Says |
|---|---|---|
| Code | this package | how to acquire, generate, count; nothing about the wiring |
| Wiring | `config.yaml` (`daq_signals`, enabled outputs) | "photodiode" is ai0, differential, 2 mW/V; ao0 may be driven, 0–5 V |
| Experiment | the flow JSON | what happens, by name: "record photodiode for 2 s" |

A flow written with signal names runs unchanged in a lab that wired the same
signals to other terminals. Physical names (`ai0`, `ao1`, `port0/line3`, `ctr0`,
`pfi0`; never with the device name) are accepted too.

## Flow steps

| Step | Kind | What it does |
|---|---|---|
| `daq_read_analog` | AI | Mean of `samples` readings of one input |
| `daq_acquire` | AI | Hardware-timed waveform of 1–16 inputs to a CSV; optional digital start trigger on a PFI terminal |
| `daq_start_logging` / `daq_stop_logging` | AI | Continuous recording to a CSV while the following steps run |
| `daq_wait_analog` | AI | Wait until an input is above / below a threshold (timeout = the step fails) |
| `daq_write_analog` | AO | DC voltage, held |
| `daq_generate_waveform` | AO | Sine / square / triangle / sawtooth, hardware-timed, for a duration or until stopped |
| `daq_read_digital` | DI | One line |
| `daq_write_digital` | DO | One line high / low, held |
| `daq_pulse_digital` | DO | One line active for a duration (software-timed, a few ms of jitter) |
| `daq_wait_digital` | DI | Wait until a line is high / low (timeout = the step fails) |
| `daq_count_edges` | counter in | Edges on a terminal during a duration |
| `daq_measure_frequency` | counter in | Frequency of the signal on a terminal |
| `daq_generate_pulses` | counter out | Pulse train / PWM: frequency, duty cycle, a pulse count, a duration, or until stopped |
| `daq_stop_output` | any output | One output back to 0 V / low / stopped |

Parameters and limits are in [`docs/experimental-flow.md`](../../docs/experimental-flow.md).

Not covered: hardware-timed digital patterns (port 0), change detection,
quadrature encoders and position, pulse-width / two-edge measurements, analog and
reference (pre-)triggers, external sample clocks, thermocouple / bridge channel
types, synchronised AI + AO in one task. `NIDaq` is the place to add them.

## Files

| File | Role |
|---|---|
| `policy.py` | Channel names, `DaqSignals` (named signals), `DaqOutputPolicy` (what may be driven). No hardware. |
| `nidaq.py` | `NIDaq`: the DAQmx tasks. Knows nothing about flows or permissions. |
| `mock.py` | `MockNIDaq`: same methods, no hardware, records what was driven. |
| `service.py` | `DaqService`: one `daq_*` step = resolve the name, check the policy, call the device, return the log record. Used by `SharedDevices` and `MockSharedDevices` alike. |

## Requirements

1. The **NI-DAQmx driver** from ni.com (administrator rights, reboot):
   `python -m nidaqmx installdriver` downloads NI's installer, or install it from
   ni.com by hand. Afterwards the device appears in *NI MAX* under *Devices and
   Interfaces* with a name such as `Dev1`; run *Self-Test* there.
2. `pip install nidaqmx` (listed in `requirements.txt`). The package is imported
   only when a DAQ step actually runs, so mock runs and the tests need neither.

## Configuration (`config.yaml`, section `shared_devices`)

```yaml
daq_device: Dev1        # name shown in NI MAX; "" = no DAQ
daq_signals:            # name -> wiring; flows use the names
  photodiode: {channel: ai0, terminal: diff, v_range: 1, scale: 2.0, offset: 0.0, unit: mW}
  valve: {channel: port0/line0}
  flow_meter: {channel: pfi0}
  stepper_clock: {channel: ctr0}
daq_ao_channels: []     # analog outputs a flow may drive, e.g. [ao0]
daq_do_lines: []        # digital lines a flow may drive, e.g. [port0/line0]
daq_co_counters: []     # counters that may emit pulses, e.g. [ctr0]
daq_ao_min_v: 0.0       # allowed analog output span
daq_ao_max_v: 5.0
daq_co_max_hz: 1000.0   # highest pulse-train frequency
```

A signal is `{channel: <physical name>}`. An analog input may add `terminal`
(`rse` / `nrse` / `diff` / `default`), `v_range` (0.1 … 10) and a linear conversion
`value = volts × scale + offset` with a `unit`. A step that gives `v_range` or
`terminal` itself overrides the signal's. The enabled-output lists always hold
physical names: naming a signal does not enable it.

## Safety rules built in

- **Inputs are always allowed. Outputs are refused unless listed** in
  `daq_ao_channels` / `daq_do_lines` / `daq_co_counters`. An analog output (DC, or
  both ends of a waveform: offset ± amplitude) must lie inside `daq_ao_min_v` ..
  `daq_ao_max_v`; a pulse train must not exceed `daq_co_max_hz`. An output can
  switch whatever is wired to the terminal block; list a channel only after
  checking the wiring. The mock applies the same rules through the same code, so
  `--mock` shows a refusal before the bench does. A refused step sends nothing to
  the device.
- **Every output a run drove returns to its safe state when the run ends**,
  including on Ctrl+C and on an error: analog outputs to 0 V (a running waveform
  is stopped first), digital lines low, pulse trains stopped. If a channel cannot
  be reset, the log says which one; switch off what is wired to it by hand.
- The safe state is 0 V / low. Wire loads so that low means off (an active-low
  relay board would switch ON at the end of a run).
- A step with a duration (`daq_generate_waveform`, `daq_generate_pulses`,
  `daq_pulse_digital`) returns its output to the safe state when the time is up,
  also when it is interrupted. One exception by design: a low pulse
  (`daq_pulse_digital` with `state: false`) that completes leaves the line high,
  as it was meant to idle; interrupted, it ends low like everything else.
- Once the run's cleanup has started, no output call is accepted any more, and the
  cleanup waits for one still in progress: a step cancelled half-way cannot switch
  something on behind the reset.
- A line being driven as an output cannot be read as an input in the same run
  (that would release the output).
- `daq_wait_*` steps fail when their timeout passes; the flow stops there and the
  outputs go to the safe state. In `--mock` a wait is taken as met.
- A power cut or a crash of the PC leaves the outputs to the device's power-up
  state, not to this software. Anything hazardous needs its own hardware
  interlock.

## Things to know

- **One analog-input engine.** While a `daq_start_logging` recording runs, other
  analog-input steps are refused; put every channel you need into the recording.
- **Several channels are scanned, not sampled together**: one ADC, multiplexed.
  `rate_hz` is per channel; the sum over the channels is limited to 1 MS/s
  (1.25 MS/s for a single channel). A high-impedance source next to a very
  different voltage shows ghosting; slow down or buffer the source.
- `daq_start_logging` writes every sample to the CSV (limit 10 kHz per channel).
  The time column is the sample index divided by the rate, from the start of the
  recording.
- `daq_count_edges` gates by software (`duration_s` is accurate to a few ms);
  `daq_measure_frequency` is hardware-timed.
- Counter outputs appear on the counter's default terminal: ctr0 → PFI12,
  ctr1 → PFI13, ctr2 → PFI14, ctr3 → PFI15. A measurement takes the highest free
  counter unless `counter` is given.
- `100khztimebase` and `20mhztimebase` are accepted as input terminals: the
  board's internal clocks, a wiring-free check that the counters work.
- A waveform is generated from up to 1000 points per cycle (fewer above 2 kHz).
- **One waveform at a time**: the device has a single analog-output sample clock,
  so a second `daq_generate_waveform` on the other channel is refused while one
  runs. A DC level on one channel together with a waveform on the other (as in
  the template flow) is accepted by the software but **has not been tried on the
  device**; if DAQmx refuses it, the step fails and the outputs are reset.
- The device rounds a requested sample rate to what its timebase can divide
  (3000 Hz becomes 3000.03 Hz). The CSV time axis and the reported waveform
  frequency use the rate the device reports; the log says so when they differ.
- A counter's output pin is also a digital line (PFI12–15 = port2/line4–7). While
  a pulse train runs, that line is refused as a digital input or output, and a
  line driven as a digital output in a run is refused as a counter output.
- `daq_count_edges` reads the 32-bit counter every 10 s, so long counts of a fast
  signal do not wrap unnoticed.
- `daq_stop_output` on an output this run never drove sends nothing and records
  no level.

## Electrical limits to respect (check the NI specification before wiring)

- Analog inputs: within the selected range, at most ±10 V. Use the smallest
  `v_range` that contains the signal for the best resolution.
- Outputs are signal-level (a few mA). Do not drive a pump, valve or relay coil
  directly; use a driver stage (relay module, MOSFET, SSR).
- Digital lines and PFI terminals are 5 V TTL.
- `terminal`: `rse` for a signal referenced to the device ground, `diff` for a
  floating or remote-grounded source wired to an AI+/AI- pair. An unconnected
  input reads an arbitrary, drifting voltage.

## Bench check

Reads only, drives nothing:

```bash
python -m src.devices.nidaq.nidaq             # list the devices NI-DAQmx knows
python -m src.devices.nidaq.nidaq Dev1 ai0    # read one analog input
python -m src.flow.run_flow examples/daq/acquire_and_count.json --mock
python -m src.flow.run_flow examples/daq/acquire_and_count.json   # real, inputs only
```

`acquire_and_count.json` should report about 100000 edges per second and
100000 Hz for the internal timebase. A known source makes a voltage checkable: a
1.5 V cell across AI0 and AI GND (`terminal: rse`).

Outputs, once a channel is enabled and nothing hazardous is wired: loop AO0 back
to AI0 and compare `daq_write_analog` with `daq_read_analog`; loop PFI12 (ctr0
out) to PFI0 and compare `daq_generate_pulses` with `daq_measure_frequency`.
[`examples/daq/README.md`](../../examples/daq/README.md) has a template flow.
