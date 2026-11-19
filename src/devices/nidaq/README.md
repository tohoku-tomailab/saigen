# National Instruments DAQ (USB-6351, USB-6002)

Analog, digital and counter I/O through NI-DAQmx, used by the `daq_*` flow steps.
The driver, the steps and the flows are the same for every model; what a model
has (channels, rates, which measurements exist) is one table entry in
`profiles.py`. Two models have an entry: the USB-6351 (X Series, the one the code
was written on) and the USB-6002 (low-cost, added afterwards as a profile). A
device of another model is refused until it has a profile: see
[Adding another model](#adding-another-model).

> Status (2026-11-14, NI-DAQmx 26.8, one USB-6351): **every input function was
> exercised on the real device** with nothing wired — averaged reading, waveform
> capture (up to 1 MS/s), continuous recording, thermocouple and RTD channels,
> an external sample clock and a pre-trigger acquisition (the internal 100 kHz
> timebase standing in for the signal), digital input and clocked digital
> acquisition, edge counting, frequency, period, pulse width and edge separation
> on that timebase, the encoder (reads 0°), and the timeouts of a digital trigger,
> an analog trigger and a change detection that never come. What that leaves
> untested: an analog trigger and a change detection that actually fire, an
> encoder that turns, a temperature against a reference. **No output has been
> driven on the real device yet** (DC and waveform analog output, stimulus with
> acquisition, digital output, pulse and pattern, pulse train): these are
> unit-tested against a fake `nidaqmx` module only. Run
> `examples/daq/output_loopback_check.json` on the bench, with nothing else
> wired, before relying on them.
>
> USB-6002 (2026-11-16, one device): the inputs it has were exercised on the real
> device with nothing wired — averaged reading (RSE and differential), waveform
> capture at its limits (50 kS/s on one channel, 25 kS/s on each of two),
> continuous recording, digital input on all three ports, edge counting (0 edges),
> the timeout of a digital trigger that never comes — and what it lacks was asked
> of the device and refused by it (see the table below). **None of its outputs has
> been driven**, and its counter and trigger have not seen a signal:
> `examples/daq/usb6002_loopback_check.json` is the bench check for that.

## Three layers

| Layer | Where | Says |
|---|---|---|
| Code | this package | how to acquire, generate, count; nothing about the wiring |
| Wiring | `config.yaml` (`daq_signals`, enabled outputs) | "photodiode" is ai0, differential, 2 mW/V; ao0 may be driven, 0–5 V |
| Experiment | the flow JSON | what happens, by name: "record photodiode for 2 s" |

A flow written with signal names runs unchanged in a lab that wired the same
signals to other terminals. Physical names (`ai0`, `ao1`, `port0/line3`, `ctr0`,
`pfi0`; never with the device name) are accepted too.

## Supported models

| | USB-6351 | USB-6002 |
|---|---|---|
| Analog inputs | ai0–15 (ai0–7 differential), ±0.1 … ±10 V, 1.25 MS/s (1 MS/s shared by several channels) | ai0–7 (ai0–3 differential), ±10 V only, 50 kS/s shared by the channels |
| Input wiring | `rse`, `nrse`, `diff` | `rse`, `diff` |
| Analog outputs | ao0–1, ±10 V, waveforms up to 2 MS/s | ao0–1, ±10 V, waveforms up to 5 kS/s (a sine of at most 250 Hz) |
| Digital lines | port0–2, 8 lines each; port 0 hardware-timed | port0 (8), port1 (4), port2 (1); software-timed only |
| Counters | ctr0–3: edges, frequency, period, pulse width, edge separation, encoder; pulse trains | ctr0: edge counting only (the device accepts PFI0 and PFI1 as its source; no edge has been counted yet); no output |
| Terminals | pfi0–15 (= port1/line0–7, port2/line0–7), `100khztimebase`, `20mhztimebase` | pfi0 (= port2/line0), pfi1 (= port1/line1) |
| Triggers for `daq_acquire` | digital, analog, pre-trigger; external sample clock | digital start trigger |
| Thermocouple / RTD signals | yes | no: one count of its 16 bits over ±10 V is 305 µV, about 7 K of a type K thermocouple. Read an amplified sensor as a voltage with `scale` |

Steps of the table below that a USB-6002 **cannot** run: `daq_measure_frequency`,
`daq_measure_period`, `daq_measure_pulse_width`, `daq_measure_edge_separation`,
the three encoder steps, `daq_generate_pulses`, `daq_acquire_digital`,
`daq_write_digital_pattern`, `daq_wait_digital_change`, and the options
`trigger_level`, `pretrigger_s` and `clock` of `daq_acquire`. The other fourteen
steps are the same.

The real driver reads the model from the device. `--mock` imitates the model
named in `config.yaml` (`daq_model`; empty means USB-6351) with the same table.
A step or option the model does not have at all stops the flow at `--mock` before
its first step, with the model's name and one line per step; a channel, range,
wiring mode or rate the model does not have is refused when its step is reached,
in `--mock` as on the bench. A real run does not start unless `daq_model` is the
model that is connected (so a lab with a USB-6002 must write `daq_model:
USB-6002`; the error says so). A flow written for the USB-6002 runs unchanged on
the USB-6351.

## Flow steps

| Step | Kind | What it does |
|---|---|---|
| `daq_read_analog` | AI | Mean of `samples` readings of one input |
| `daq_acquire` | AI | Hardware-timed waveform of 1–16 inputs to a CSV; optional digital or analog trigger, pre-trigger samples, external sample clock |
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
| `daq_stimulate_acquire` | AO + AI | A waveform and an acquisition started by one trigger; response and nominal stimulus in one CSV |
| `daq_acquire_digital` | DI | Clocked acquisition of lines of port 0 to a CSV |
| `daq_write_digital_pattern` | DO (+ DI) | Hardware-timed sequence of levels on lines of port 0; optionally records other lines on the same clock |
| `daq_wait_digital_change` | DI | Wait for a change on lines of port 0 (hardware change detection) |
| `daq_measure_pulse_width` / `daq_measure_period` / `daq_measure_edge_separation` | counter in | Times, in seconds: one pulse, one cycle, from an edge on one terminal to an edge on another |
| `daq_start_encoder` / `daq_read_encoder` / `daq_stop_encoder` | counter in | Angle of a quadrature encoder, in degrees |

Parameters and limits are in [`docs/experimental-flow.md`](../../docs/experimental-flow.md).

Not covered: bridge / strain channel types (**the USB-6351 does not support them**:
DAQmx refuses the channel; read a bridge amplifier's output as a voltage with a
scale), linear encoders and the Z index, continuous or repeated digital patterns,
arbitrary (user-supplied) analog waveforms, an external clock for outputs, and
buffered counter measurements (many periods in one step). `NIDaq` is the place to
add them.

## Files

| File | Role |
|---|---|
| `profiles.py` | `DeviceProfile`: what each model has. The only place a model's numbers are written. |
| `probe.py` | `python -m src.devices.nidaq.probe`: what the connected devices report about themselves (property reads only), compared with their profile. |
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
  furnace: {channel: ai2, sensor: thermocouple, tc_type: K, cjc_c: 25.0, terminal: diff, max_c: 400}
  bath: {channel: ai3, sensor: rtd, excitation_a: 0.001, terminal: diff}
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

**Temperature sensors.** `sensor: thermocouple` needs `tc_type` (B E J K N R S T) and
`cjc_c`, the temperature of the terminal block in °C: the USB-6351 has no
cold-junction sensor, so an error in that number is an error of the reading, one
to one. `sensor: rtd` needs `excitation_a`, the current your external source drives
through the RTD (the device supplies none), and takes `rtd_type` (default
`pt3851`), `r0_ohm` (100) and `wires` (2 / 3 / 4, default 4). `min_c` / `max_c`
(default 0 / 200) is the expected range; it selects the input gain. Every
analog-input step then returns °C in `daq_value` (no voltage is recorded), and
`scale` / `offset` / `unit` still apply on top. A thermocouple delivers tens of
microvolts per kelvin into a ±0.1 V range: wire it differentially, expect a few
kelvin of noise without filtering, and check it against a reference before
trusting the numbers.

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
- All of this is software. If the program or the PC dies without running its
  cleanup (a crash, a kill, a USB cable pulled), the device keeps its last output
  values and a running waveform or pulse train may go on; only a power cycle of
  the device brings its power-up state back. Anything hazardous needs its own
  hardware interlock.

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
- **Hardware-timed digital I/O exists on port 0 only** (`daq_acquire_digital`,
  `daq_write_digital_pattern`, `daq_wait_digital_change`); ports 1 and 2 (the PFI
  pins) are static.
- `daq_stimulate_acquire` starts output and acquisition on one trigger, but each
  runs on its own sample clock: the `_set_V` column is the commanded waveform at
  the nominal sample times, and the real alignment (expected within about one
  sample) is still to be measured by loop-back. The same holds for the recorded
  lines of `daq_write_digital_pattern`: they are sampled on the pattern's clock
  edge, so a looped-back line may show each state one row late and miss the last.
- An analog trigger watches the first channel of the acquisition, in volts.
  With `pretrigger_s` the CSV's time 0 is the trigger.
- With an external `clock`, the time column assumes `rate_hz`; it is exact only
  if the clock really has that frequency.
- The encoder holds one counter while it is followed; the time measurements and
  `daq_count_edges` pick another one.
- **One waveform at a time**: the device has a single analog-output sample clock,
  so a second `daq_generate_waveform` on the other channel is refused while one
  runs. A DC level on one channel together with a waveform on the other (as in
  the template flow) is accepted by the software but **has not been tried on the
  device**; if DAQmx refuses it, the step fails and the outputs are reset.
- The device rounds a requested sample rate to what its timebase can divide
  (3000 Hz becomes 3000.03 Hz). The CSV time axis and the reported waveform
  frequency use the rate the device reports; the log says so when they differ.
- A PFI terminal is the same pin as a digital line (table above). While a run
  drives that line as a digital output, the PFI is refused as a counter source,
  trigger or clock, and a line the encoder listens on is refused as an output.
  To count or trigger on a signal the board itself produces, use another pin and
  a jumper.
- A counter's output pin is also a digital line (PFI12–15 = port2/line4–7). While
  a pulse train runs, that line is refused as a digital input or output, and a
  line driven as a digital output in a run is refused as a counter output.
- `daq_count_edges` reads the 32-bit counter every 10 s, so long counts of a fast
  signal do not wrap unnoticed.
- `daq_stop_output` on an output this run never drove sends nothing and records
  no level.

## Adding another model

The USB-6002 was added this way, and it is the pattern for a third model **that
fits what the driver assumes** (listed at the end of this section). No step, no
schema entry and no GUI block was added for it; the one-time work was moving the
USB-6351's numbers out of the code into a profile, and three checks the smaller
board made necessary (differential pairs, the rate of an averaged reading, a PFI
whose pin is being driven as a digital output).

1. **Ask the device** (nothing is driven, no task is created):
   `python -m src.devices.nidaq.probe Dev2` prints channel counts, ranges, rates,
   ports and counters as DAQmx reports them.
2. **Write the profile**: one `DeviceProfile(...)` in `profiles.py` with those
   figures and the specification's. Start with `features=frozenset()`.
3. **Find out what it really supports.** DAQmx's lists are not reliable (the
   USB-6351 lists bridge channels and refuses to create one; the USB-6002 lists
   thermocouples that its resolution makes useless). For each feature in
   `profiles.py`, create the input task on the device and ask DAQmx to verify it
   (`task.control(TaskMode.TASK_VERIFY)`), or run it with nothing wired. Add a
   feature to the profile only when the device accepted it. Input tasks only;
   outputs are step 6.
4. **Tests**: the parametrised tests at the end of `tests/test_nidaq.py` take the
   model's names and limits; add the rows for the new model, and one example flow
   run under `daq_model: <model>`.
5. **Input-only run on the device**: a flow of `daq_*` input steps
   (`examples/daq/usb6002_inputs.json` is the 6002's).
6. **Loop-back check of the outputs**, run by the human after enabling them for
   the check (`examples/daq/usb6002_loopback_check.json`). Until then the README
   says the outputs are untested on that model.

What the driver still assumes of every model, outside the profile: channels,
lines, counters and PFI numbered from 0 without gaps; the differential inputs
are the first channels; inputs of at most ±10 V; 32-bit counters; one
analog-output sample clock (one waveform at a time); 0 V and low as the safe
state of the outputs; hardware-timed digital I/O and change detection on the
same single port; the terminal names of the internal timebases and of the
analog-output start trigger. The choice lists of the flow schema (`terminal`,
`v_range`) are the union of the models'. A model that differs in one of these
needs a new profile field or driver code, not only a table entry.

What was learnt from the USB-6002 on the bench:

- The PC receives the first samples of a continuous acquisition about 0.2 s
  after the start, then in blocks of about 0.2 s. Whether sampling itself starts
  late was not measured. A recording is reported as an error when fewer than
  90 % of the samples expected for its duration arrived.
- An unsupported range is silently coerced by DAQmx (±5 V becomes ±10 V), an
  unsupported rate or wiring mode is refused. The profile refuses both before
  the device is asked.
- Without `terminal`, DAQmx reads ai0–3 differentially and ai4–7
  single-ended (RSE) on this model. Give `terminal: rse` for a signal referenced
  to the device ground.
- A flow can name no internal signal on this model (no timebase terminal), so
  its counter and its PFI trigger can only be checked with a wire (the loop-back
  flow uses its own analog output as a 0–5 V square wave).
- DAQmx refuses a padded channel number (`ai00`, `port0/line00`); so does the
  profile.

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
python -m src.devices.nidaq.probe             # what each device has, against its profile
python -m src.devices.nidaq.nidaq             # list the devices NI-DAQmx knows
python -m src.devices.nidaq.nidaq Dev1 ai0    # read one analog input
python -m src.flow.run_flow examples/daq/acquire_and_count.json --mock
python -m src.flow.run_flow examples/daq/acquire_and_count.json   # real, inputs only
python -m src.flow.run_flow examples/daq/timing_and_triggers.json # real, inputs only
```

On a USB-6002 the input-only flow is `examples/daq/usb6002_inputs.json` (the two
flows above use the internal timebase and the counter measurements of the
USB-6351), and the output check is `examples/daq/usb6002_loopback_check.json`.

`acquire_and_count.json` should report about 100000 edges per second and
100000 Hz for the internal timebase. A known source makes a voltage checkable: a
1.5 V cell across AI0 and AI GND (`terminal: rse`).

Outputs: `examples/daq/output_loopback_check.json` wires each output straight
back to an input of the board (AO0–AI0, AO1–AI1, P0.0–P0.1, PFI12–PFI0) and walks
through every output function. The jumpers, the lines to enable for the check and
the reading each step should give are in
[`examples/daq/README.md`](../../examples/daq/README.md).
