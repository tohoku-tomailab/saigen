# NF LI5640 lock-in amplifier (GPIB, read-only; internal oscillator optional)

A multifunction digital lock-in amplifier by NF Corporation. SAIGEN reads it over GPIB
(IEEE 488.1, partially 488.2) through PyVISA. RS-232 is not supported (the LI5640 cannot
use GPIB and RS-232 at the same time). Setup is in `docs/setup.md` §11. The commands were
checked against chapter 5 of the instruction manual (English edition DB509993-003); the
"5-n" references below are manual pages.

## Policy: measurement settings on the front panel, SAIGEN only reads

Sensitivity, time constant, slope, reference signal, input, filters, dynamic reserve, the
DATA1 / DATA2 display, offset, expand, normalise and ratio are all set on the front panel.
SAIGEN never sends any of them. The only setting it writes is the read-out selection `OTYP`
(remote only, 5-22, 5-32): the value is remembered on connect, restored on disconnect and
read back. The one exception is the internal oscillator (below), and only when the lab
config enables it.

Other side effects:

- Device clear (SDC addressed to this instrument; clears the I/O buffers, settings are
  unchanged, 5-77, 5-78)
- `*CLS` (clears the event registers and the error queue; not restored, 5-36)
- REMOTE state until GTL (the front panel cannot be operated, 5-6). GTL is addressed to
  this instrument; bus-wide REN, IFC and DCL are not used
- One open handle per instrument in a process: the driver takes a lock per instrument
  (the resource name as resolved by PyVISA, so `GPIB::7::INSTR` and `GPIB0::7::INSTR` are
  the same) and releases it only after a successful close. A run whose release is still
  going on (restoring the oscillator) blocks the next run's open, which then fails with a
  message instead of talking to the instrument at the same time. Separate processes are
  not covered

## Internal oscillator (`set_lockin_oscillator`, off by default)

A flow step can set the frequency (`FREQ`) and the OSC OUT amplitude (`AMPL`, Vrms) of the
internal oscillator (5-23). It is **disabled** unless the lab config allows it:

```yaml
shared_devices:
  lockin_osc_max_vrms: 0.3     # 0 = disabled (default); at most 5.0
  lockin_osc_min_hz: 10.0      # default 0.0005
  lockin_osc_max_hz: 1000.0    # default 105000
```

Choose the limits for what is wired to OSC OUT. They bound the **set values** only, not the
physical output (amplitude accuracy 2 % of set + 0.5 % of full scale up to 1 kHz, 7.4), not
how long it stays applied, and not the load: OSC OUT has a 50 Ω output impedance and
±10 mA at 5 Vrms into ≥ 660 Ω (7.4). Keep a sample on OSC OUT only if it tolerates the
configured maximum for an unlimited time: after a failure the run may home robots or close
other instruments before it releases the LI5640 and restores the oscillator.

Rules:

- **Refused before anything opens** (`--validate-only`, `--mock`, real run, GUI): when the
  limit is 0 (also for `amplitude: 0`), when a value is outside the limits, when a frequency
  cannot be represented at 4½ digits (at most 19999 counts, or 0.1 mHz; e.g. 100.01 Hz is
  fine, 200.05 Hz is not), and when no single amplitude range holds every amplitude of the
  flow. Invalid limit values (null, booleans, strings, NaN, negative, above 5) are errors.
- **The range is never switched.** Amplitudes must be representable in the range found on
  connect (0: 0–50 mV in 0.1 mV, 1: 0–500 mV in 1 mV, 2: 0–5 V in 10 mV). Choose the range on
  the front panel before the run. The behaviour of OSC OUT during a range switch is not
  documented.
- **On connect** (only when the flow contains the action) `FREQ?` and `AMPL?` are read and
  remembered. The run stops without writing if that state is outside the limits (the
  restore would write it back), if it cannot be written back exactly (a frequency off the
  4½-digit grid or an amplitude off the range's step: rounding it could exceed a limit), or
  if its range cannot hold the flow's amplitudes. A target that would need rounding is never
  sent, during a step or during the restore.
- **One command at a time.** A lower amplitude is written before the frequency, otherwise
  the frequency first (the set values never exceed the larger of before and after; this
  says nothing about transients). After each command: read back (exact match required),
  then one `EROR?` (must be 0). Any failure stops the step; no dependent command follows,
  the state is treated as unknown and the run writes no more oscillator commands. Equal
  values are not sent. Errors already queued before a step stop it before any write.
- **A step also stops** if the oscillator differs from what SAIGEN last confirmed (changed on
  the panel).
- **`settle`** is a wait after the last confirmed read-back. It does not prove that the
  lock-in output settled or that a scope acquired anew. A step interrupted during the wait
  keeps its values in the `diagnostics` column (`settled_s` is the time waited until the
  interrupt); the `lockin_osc_*` columns stay empty because the step did not finish.
- **Restore at the end** (normal end, failed step, Ctrl+C), only if a write was attempted:
  device clear → read the current state → write back the remembered frequency and
  amplitude, one command at a time with read-back → OTYP restore → GTL → close. If the
  current state cannot be read, nothing is written (`unverified`); if its range differs,
  nothing is written (`failed`); if it already equals the remembered state, nothing is
  written (`verified`). Errors already in the queue are recorded separately
  (`pre_existing_errors`); a queue that does not empty in 21 reads stops the restore.
- **Ctrl+C does not switch OSC OUT off**; the restore may *raise* the amplitude (back to
  what it was). Cleanup waits for the restore without a time limit (each VISA operation is
  bounded by `lockin_timeout_ms`). A second Ctrl+C ends the wait but not the restore, which
  goes on in its own thread. If the restore had already finished, its result is kept;
  otherwise the run records `unverified` with the LI5640 close `pending`. A restore that
  finishes after the run's logs were closed appears only on the console: `run.log`,
  `metadata.json` and `summary.md` are not updated. Check OSC OUT on the front panel.
- **The result decides the run's status.** `metadata.json` has `lockin_oscillator_restore`
  (`not_needed`, `verified`, `failed`, `unverified`, and whether the LI5640 was closed).
  `failed`, `unverified` or a failed close (also in a run that wrote nothing) turn a
  completed run into `failed` (CLI exit code 1) with a note to check OSC OUT on the front
  panel. A run that was already aborted (Ctrl+C, exit code 130) or failed keeps its status
  and exit code and gets the note. After any such note, check the oscillator on the panel.
- **A close that fails keeps the instrument lock.** The handle may still be open, so the same
  process does not open the LI5640 again (the GUI says so); restart SAIGEN.

## Wiring and settings

- Set `intFc` under SHIFT+SPECIAL on the front panel to `GPib` (5-3)
- Set the address (0–30) under SHIFT+ADDRESS on the front panel (5-4). It must not clash
  with other instruments on the same bus (check on both panels). Any transmit terminator
  works (the driver ends reception on END and strips CR / LF)
- Keep continuous data output (`OSTR 1`) off. While it is running, other queries do not get
  through (-440, 5-68), so if `OSTR?` returns 1 on connect, initialisation fails (the stop
  command is not sent)
- `config.yaml`:

  ```yaml
  shared_devices:
    lockin_resource: "GPIB0::<address>::INSTR"
    lockin_timeout_ms: 5000
    wait_after_lockin: 0.0     # wait after each measure_lockin (not a settling wait before the measurement)
  ```

## Command sequence

```
connect:    device clear, *CLS, *IDN? (must contain LI5640), EROR? (until 0, at most 21 times),
            OSTR? (must be 0), OTYP? (remembered)
each step:  OTYP 1,2,3,5 → EROR?        DOUT? then returns DATA1, DATA2, FREQ, OVERLEVEL
            settings snapshot (queries only)
            DOUT?, WRCR? samples times (waiting interval seconds between completed pairs)
            settings snapshot (again)
disconnect: device clear → [oscillator restore, only after a write was attempted]
            → OTYP <remembered value> → OTYP? (read back) → GTL → close
            (continues to the next step if any of them fails; ignores interrupt requests)
```

Settings read in the snapshot: `DDEF? 1/2`, `ISRC?`, `NORM?`, `VSTD?` / `ISTD?`,
`RAT?`, `KFAC?`, `OFSO? 1/2`, `OFFS? 1/2`, `OEXP? 1/2`, `PHAS?`, `ICPL?`, `IGND?`,
`VSEN?` / `ISEN?`, `TCON?`, `SLOP?`, `SYNC?`, `NOIS?`, `ILIN?`, `IFRQ?`, `ITHR?`,
`DRSV?`, `RSRC?`, `REDG?`, `HARM?`, `OPCR?`, `OSTR?`, and the oscillator setting
`FREQ?` / `AMPL?` (recorded as `osc_freq`, `osc_ampl`, `osc_range`). An unreadable or out-of-range reply
fails the step. A difference between the two snapshots fails the step (a setting changed and
changed back in between cannot be detected). `OPCR` 256 / 512 (auto-setting or
auto-sensitivity in progress) also fails.

`FREQ?` is the set frequency of the internal oscillator, not the measured frequency (5-23);
it is recorded as a setting. The measured frequency is read from the FREQ field of `DOUT?`
(the fundamental in harmonic measurement). `DOUT?` and `WRCR?` are not taken at the same instant.

## Meaning of the values (`describe()`)

DATA1 / DATA2 are the **values shown on the panel** (5-70).

| Condition | DATA1 | DATA2 |
|---|---|---|
| Normal | X / R: V (input A, A−B) or A (I input). AUX IN1: V | Y: V or A. θ: deg. AUX: V |
| DATA1 = NOISE | V/√Hz or A/√Hz (4-35) | AUX IN1 regardless of DDEF, V (4-34) |
| Normalization (dB / %) | X / R in dB / % (4-32) | the effect on Y cannot be determined from the manual, so the kind is `Y[normalize?]` with no unit |
| Ratio display | X / R as a ratio (no unit, takes precedence over normalization, 4-33) | Y as a ratio. θ and AUX unchanged |

Offset and expand do not change the unit, but the value is no longer the raw X / Y. The
settings are kept in `lockin_settings`.

## Rejected samples (conservative policy)

If any of the following occurs, the whole sample (including the AUX channels) is rejected
and counted per reason: OVERLEVEL 1 (input) or 2 (intermediate stage), 4 / 8 for the
displayed channel (16 / 32 in ratio display), `WRCR` 1 (UNLOCK) or 2 (cannot synchronise to
the SIGNAL reference), a malformed or non-finite `DOUT?` reply, `WRCR?` not an integer in
0–3. The manual does not say what the numeric value is during an overlevel, so no
threshold on the value (like Keithley's 9.9E37) is used.

The median is taken over the valid samples. For θ the median is taken on the arc only when
all points fall within an arc narrower than 180° (independent of order even across ±180°).
Otherwise the step fails.

## Settling

SAIGEN does not wait for settling. After changing the front-panel settings, allow the
instrument to settle before starting the step (manual Table 4-3: at 24 dB/oct, about 10τ to 99 % and about 13τ to 99.9 %. NOISE at
24 dB/oct is about 170τ × smoothing factor, 4-34). `interval` is the spacing between
readings, not a settling wait.

## Usage and connection check

```python
from src.devices.li5640 import LI5640, READ_OUTPUT_TYPE, describe

lia = LI5640("GPIB0::<address>::INSTR")
saved = None
try:
    lia.clear_status()
    print(lia.identify())
    saved = lia.output_type()
    lia.set_output_type(READ_OUTPUT_TYPE)
    snap = lia.snapshot()
    print(describe(snap), lia.read_sample())
finally:
    lia.release(saved)        # clear, restore OTYP, GTL, disconnect
```

What was confirmed on the instrument and what was not (the replies of firmware 1.60, the
defects found by review, the open points) are in `docs/li5640-commissioning.md`.

Connection check (run by a human; read-only, and OTYP is restored):

```bash
python -m src.devices.li5640.li5640 GPIB0::<address>::INSTR
```
