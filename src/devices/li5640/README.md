# NF LI5640 lock-in amplifier (GPIB, read-only)

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
read back.

Other side effects:

- Device clear (SDC addressed to this instrument; clears the I/O buffers, settings are
  unchanged, 5-77, 5-78)
- `*CLS` (clears the event registers and the error queue; not restored, 5-36)
- REMOTE state until GTL (the front panel cannot be operated, 5-6). GTL is addressed to
  this instrument; bus-wide REN, IFC and DCL are not used

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
disconnect: device clear → OTYP <remembered value> → OTYP? (read back) → GTL → close
            (continues to the next step if any of them fails; ignores interrupt requests)
```

Settings read in the snapshot: `DDEF? 1/2`, `ISRC?`, `NORM?`, `VSTD?` / `ISTD?`,
`RAT?`, `KFAC?`, `OFSO? 1/2`, `OFFS? 1/2`, `OEXP? 1/2`, `PHAS?`, `ICPL?`, `IGND?`,
`VSEN?` / `ISEN?`, `TCON?`, `SLOP?`, `SYNC?`, `NOIS?`, `ILIN?`, `IFRQ?`, `ITHR?`,
`DRSV?`, `RSRC?`, `REDG?`, `HARM?`, `OPCR?`, `OSTR?`. An unreadable or out-of-range reply
fails the step. A difference between the two snapshots fails the step (a setting changed and
changed back in between cannot be detected). `OPCR` 256 / 512 (auto-setting or
auto-sensitivity in progress) also fails.

`FREQ?` is the set frequency of the internal oscillator, not the measured frequency (5-23).
The measured frequency is read from the FREQ field of `DOUT?` (the fundamental in harmonic
measurement). `DOUT?` and `WRCR?` are not taken at the same instant.

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
