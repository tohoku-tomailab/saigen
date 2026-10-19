# Commissioning an NF LI5640 over GPIB: what the bench taught us

Field notes from bringing an NF LI5640 lock-in amplifier into the SAIGEN cell
(October 2026). They complement the reference material in
[`src/devices/li5640/README.md`](../src/devices/li5640/README.md) (commands,
meaning of the values, what is discarded) and the procedure in
[`setup.md` §11](setup.md#11-nf-li5640-lock-in-amplifier-optional-read-only).
The general rules that came out of this work are in
[`adding-an-instrument.md`](adding-an-instrument.md) (§2 and §4).

Configuration checked: one LI5640 (firmware 1.60), one NI GPIB-USB-HS+
adapter, Windows 11, NI-488.2 with NI-VISA, PyVISA 1.16, on 2026-10-16. The
LI5640 was the only instrument on the bus. Only the **R / θ display with the
internal oscillator as reference** was exercised on the instrument; X / Y,
noise, AUX, normalize, ratio, over-level and UNLOCK are covered by the tests
against a simulated instrument only. Another unit or firmware may differ, so
treat what follows as expectations to re-check, not as specifications.

## 1. Scope chosen: read-only

The LI5640 has many settings (sensitivity, time constant, reference, input,
filters, display, offset, expand, normalize, ratio). Writing them from a flow
would make SAIGEN responsible for the measurement setup. It was decided
instead that the human sets the instrument up on the front panel and SAIGEN
only reads. The only setting written is the read-out selection `OTYP`
(remote-only, manual 5-22 and 5-32), and it is restored on close (the one
later exception, the internal oscillator, off by default, is §8). Because
DATA1 and DATA2 are *display* values (manual 5-70), every step also reads the
panel settings that give them their meaning.

## 2. Order of work that worked

The steps that talk to the instrument (2 to 4) are the human's: run them
yourself, or have an agent run them only after you ask for it in the session
(CLAUDE.md rule 1).

1. **Read the manual first.** NF's English instruction manual is a scanned
   PDF; its text cannot be extracted, so the command tables were read as page
   images. Chapter 5 holds the commands (PDF page = manual page 5-n + 104),
   chapter 4 the meaning of the display (4-n + 62).
2. **List GPIB resources only**
   (`pyvisa.ResourceManager().list_resources("GPIB?*::INSTR")`), as for the
   Keithley 2002. Read the address from the list and from the panel
   (*SHIFT + ADDRESS*); do not assume it. The manual's figures and sample
   programs show address 2, which is not evidence of a factory default; on a
   shared bus, make sure no two instruments share an address. The address
   belongs in `config.yaml` (`lockin_resource`), never in tracked files.
3. **The driver's self-check**
   (`python -m src.devices.li5640.li5640 <resource>`): `*IDN?`, the error
   queue, `OSTR?`, `OTYP?`, all panel settings, one `DOUT?`, then `OTYP`
   restored and GTL.
4. **`--validate-only`, `--mock`, then a lock-in-only real run** of
   `examples/li5640/read_lockin.json` with `--no-record`. The flow opened no
   robot (`"robots_used": []` in `metadata.json`).

What was verified: communication, every query the driver sends, the `OTYP`
write and its restore with read-back, the flow and logging path (CSV columns,
`metadata.json` settings per step). What was **not** verified: the meaning of
the readings. The signal at the input was not a known reference, so the values
read are communication checks; they are not listed here.

## 3. What firmware 1.60 answered

- `*IDN?` returned `NF-ELECTRONIC-INSTRUMENTS,LI5640,<serial>,1.60`: the maker
  field is the one in the manual (5-42), not "NF CORPORATION". The driver
  matches the model name anywhere in the reply, so both forms are accepted.
- `OTYP?` returned `1,2` on a unit that had been powered on normally (the
  power-on value, manual 5-22).
- `OSTR?` returned `0`; the error queue was empty at every check.
- Every snapshot query (`DDEF? 1/2`, `ISRC?`, `NORM?`, `VSTD?`, `RAT?`,
  `KFAC?`, `OFSO? 1/2`, `OFFS? 1/2`, `OEXP? 1/2`, `PHAS?`, `ICPL?`, `IGND?`,
  `VSEN?`, `TCON?`, `SLOP?`, `SYNC?`, `NOIS?`, `ILIN?`, `IFRQ?`, `ITHR?`,
  `DRSV?`, `RSRC?`, `REDG?`, `HARM?`, `OPCR?`, `OSTR?`) answered with a bare
  number in the form the manual shows (no header, no unit).
- `DOUT?` with `OTYP 1,2,3,5` answered four comma-separated fields: DATA1 in
  mantissa-exponent form (`d.ddE-03`), θ as a plain decimal, the frequency in
  the form `ddd.ddE+00`, and the over-level integer. All four parsed with
  Python's `float` / `int`.
- `WRCR?` answered `0`.

Step durations in that run (`duration_s` in `measurements.csv`, which covers
`OTYP`, the error check, two snapshots of about thirty queries each and the
readings; `wait_after_lockin` was 0):

| Step | Samples | Interval | Duration |
|---|---|---|---|
| 1 | 5 | 0 s | 2.25 s |
| 2 | 5 | 1 s | 6.24 s |

## 4. Defects found by review before the bench

Each was found by review (Astra, through Codex) of the plan or of the code,
and each now has a regression test in `tests/test_li5640.py` that fails when
the fix is reverted.

- **The display decides the meaning, and the manual has rules beyond
  `DDEF`.** With DATA1 on NOISE, DATA2 shows AUX IN1 whatever `DDEF? 2` says
  (4-34); normalize applies to X / R (4-32); ratio applies to X / Y / R and
  wins over normalize (4-33); noise is in V/√Hz or A/√Hz (4-35). A first plan
  that read `DDEF? 2` alone could have taken a voltage for a phase.
- **The phase median.** A median of unwrapped angles depended on the order of
  the readings. The fix takes the median on the narrowest arc and fails the
  step if no arc narrower than 180° holds all readings. Its first version then
  rejected a **steady** phase (all readings equal: the wrap-around gap came
  out as 0° instead of 360°). The bench run read a steady θ; it would have
  failed without that second fix.
- **Restoring `OTYP` after an interruption.** The restore had to be exempt
  from the abort check (otherwise a Ctrl+C prevents it), preceded by a device
  clear (a timed-out write may have left half a command in the input buffer),
  read back even if its write failed, and followed by GTL and close whatever
  happened before. Nothing is restored that was never read.
- **Two GPIB instruments in one clean-up.** A second Ctrl+C while the Keithley
  was being closed skipped the LI5640. `cleanup()` now sets every closing flag
  first, closes each instrument in its own `try`, and re-raises the
  cancellation after all of them.
- **Diagnostics of a failed step.** The runners passed only the exception text
  to the logger, so the settings before and after a change were lost. They
  now pass the exception's `diagnostics` dict, which lands in the
  `diagnostics` column, the time-line and `metadata.json`.
- **`FREQ?` is not the measured frequency**; it is the internal oscillator's
  setting (5-23). The measured reference frequency comes from `DOUT?`.

## 5. Open points

- **Y under normalize.** The manual says normalize shows "the ratio of X or R"
  (4-32) but its list of restrictions does not mention Y. The driver labels Y
  as `Y[normalize?]` with no unit rather than guess. Turning normalize on with
  an X / Y display and reading Y on the panel would settle it.
- **Not yet seen on the instrument:** X / Y and noise displays, ratio, an
  over-level or UNLOCK reading (and the `DOUT?` text the instrument sends
  then), a panel change during a step, and an instrument at another address on
  a shared bus with the Keithley 2002.

## 6. Things not to do

- Do not send `*RST` or `INIT`: both return the panel set-up the human made to
  the initial state, and `*RST` also clears the data memory (5-31, 5-42).
- Do not stop continuous output (`OSTR 0`) from SAIGEN; if it is running, the
  run refuses to start and the human stops it.
- Do not use `FREQ?` for the measured frequency.
- Do not judge a reading without the panel settings next to it: the same
  DATA1 can be X, R, noise or an AUX voltage, in V, A, dB, % or a ratio.
- Do not use the generic `wait` action in a lock-in-only flow (it opens
  robot 1); use `wait_after_lockin`, which is a pause after the step, not a
  settling time.
- Do not use readings taken on an unknown input to judge the input or the
  instrument; feed a known signal first (for example the LI5640's own OSC OUT
  into SIGNAL A with the internal oscillator as reference).

## 7. Small things

- `python -m src.devices.li5640.li5640` prints a runpy `RuntimeWarning` (the
  package `__init__` imports the module first). It is harmless.
- The panel is locked (REMOTE) from the first command until GTL at the end of
  the run.

## 8. Internal oscillator (`set_lockin_oscillator`), 2026-10-19

The one exception to the read-only scope: a flow step can set the frequency
and the OSC OUT amplitude of the internal oscillator. It is off unless
`config.yaml` sets `lockin_osc_max_vrms` above 0; the rules are in the driver
README. It was checked with the same LI5640 and an Agilent DSO1014A reading
OSC OUT on CH1 (BNC, 1X, DC), nothing else on OSC OUT, reference INT OSC
(`RSRC 1`), SIGNAL input open. Limits for that load: 0.3 Vrms, 10–1000 Hz.
Amplitude range on the panel: 500 mV (range 1). Flow:
[`examples/li5640/oscillator/osc_sweep_scope.json`](../examples/li5640/oscillator/osc_sweep_scope.json),
run with `--no-record`.

**First run: stopped at the first point, by the oscilloscope.** CH1 was at
0.5 V/div with −0.1 V offset. At 0.05 Vrms (about 0.14 Vpp, a third of a
division) the DSO1014A gave Vpp and mean but no frequency or period
(`9.9E+37`, no edges found), so the `measure_scope` step failed and the run
ended `failed`. The oscillator write before it was fine, and the restore at the
end sent `AMPL 0.300,1`, read back `0.300,1` (`verified`, `closed: true`).
That failure was the first real exercise of the restore after a failed step.

**Second run: all 18 steps completed** after CH1 was set to 0.2 V/div, offset
0, trigger level 0 V (5 ms/div, edge on CH1, auto sweep):

| Set | LI5640 measured frequency | DSO frequency | DSO period | DSO Vrms | DSO Vpp |
|---|---|---|---|---|---|
| 100 Hz, 0.05 Vrms | 100.0 Hz | 100.0 Hz | 9.999 ms | 0.0525 V | 0.152 V |
| 100 Hz, 0.1 Vrms | 100.0 Hz | 100.0 Hz | 9.999 ms | 0.1008 V | 0.296 V |
| 100 Hz, 0.2 Vrms | 100.0 Hz | 100.0 Hz | 9.999 ms | 0.2019 V | 0.576 V |
| 100 Hz, 0.3 Vrms | 100.0 Hz | 100.0 Hz | 9.999 ms | 0.3014 V | 0.856 V |
| 50 Hz, 0.3 Vrms | 50.0 Hz | 50.0 Hz | 19.99 ms | 0.3005 V | 0.856 V |
| 200 Hz, 0.3 Vrms | 200.0 Hz | 200.0 Hz | 4.999 ms | 0.3001 V | 0.856 V |

Values are medians of the samples of each step (DSO: 3 readings 0.5 s apart).
Every setting matched its read-back exactly (the step would have failed
otherwise; the restore's read-back of `FREQ?` was `100.00E+00`), the error queue
was 0 after each command, and the range stayed 1. The DSO Vrms is within the manual's amplitude accuracy (2 % of setting +
0.5 % of the 500 mV full scale, up to 1 kHz, 7.4) at every point; the
0.05 Vrms point is the furthest off (+5 %), mostly the full-scale term. The
restore at the end found only the frequency different (200 Hz) and sent
`FREQ 1.0000E+02` alone. `metadata.json`: `lockin_oscillator_restore` =
`verified`, `closed: true`; `OTYP` back to `1,2`.

A repeat a few minutes later gave the same result (completed, restore
`verified` with `FREQ 1.0000E+02` alone) and Vrms within 0.001 V of the table,
except at 0.05 Vrms: 0.0531 V, and the DSO frequency 99.0 Hz (period
10.09 ms) while the LI5640 measured 100.0 Hz. At 0.76 division the
oscilloscope's frequency reading is not reliable; use the LI5640's measured
frequency, or a smaller V/div, for small amplitudes.

**Defects found by review, not by the bench.** The plan went through six
reviews and the implementation through one before the bench; the
implementation review found, among others, that a saved state off the
amplitude step or the 4½-digit frequency grid (e.g. `0.3006,1`) would have been
*rounded* on restore and sent above the limit (now refused at connect, and no
rounded value is ever sent), and that a failed close in a run that wrote
nothing was reported as success (now the run fails).

**Open points**

- The DSO mean was 13–15 mV at every point (also in the earlier manual test).
  Whether that is an offset of OSC OUT or of the oscilloscope has not been
  separated (read OSC OUT with the BNC disconnected, or with a DMM).
- Only range 1 (500 mV), 50–200 Hz and the set values above were exercised.
  Range 0 and 2, frequencies near the ends, the output transient during a write
  and Ctrl+C on the instrument (tested only against a simulated instrument and
  with a real SIGINT in a separate process) have not been seen.
- The smallest amplitude that the DSO1014A still measures as a frequency
  depends on V/div; choose V/div for the smallest point of the sweep, not the
  largest.

**Things not to do**

- Do not raise `lockin_osc_max_vrms` or widen the frequency limits to make a
  flow pass; they protect what is wired to OSC OUT.
- Do not switch the amplitude range on the panel during a run: the next step
  sees a change and stops, and the restore refuses to write across ranges.
- Do not take `settle` as a settling guarantee: it is a wait after the
  read-back.
